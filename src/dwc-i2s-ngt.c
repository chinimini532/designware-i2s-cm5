/*
 * ALSA SoC Synopsys I2S Audio Layer - NGT MASTER MODE
 *
 * This version configures Pi as I2S MASTER for loopback testing.
 * Pi generates BCLK, FS, and sends data on DOUT.
 * FPGA loops DOUT back to DIN.
 *
 * Based on sound/soc/dwc/designware_i2s.c
 * Copyright (C) 2010 ST Microelectronics
 * Modified by NGT for FPGA loopback testing
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <sound/designware_i2s.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/dmaengine_pcm.h>
#include "local.h"
#include <linux/hrtimer.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/bitops.h>

/* Forward declarations */
int dw_pcm_register(struct platform_device *pdev);

/* =====================================================
 * MODULE PARAMETERS
 * ===================================================== */

static bool ngt_use_poll = true;
module_param(ngt_use_poll, bool, 0644);
MODULE_PARM_DESC(ngt_use_poll, "NGT: enable hrtimer polling path");

static unsigned int ngt_poll_us = 125;
module_param(ngt_poll_us, uint, 0644);
MODULE_PARM_DESC(ngt_poll_us, "NGT: poll period in microseconds (125us = 8kHz)");

static bool ngt_autostart = true;
module_param(ngt_autostart, bool, 0644);
MODULE_PARM_DESC(ngt_autostart, "NGT: autostart at probe time");

static unsigned int ngt_log_first = 50;
module_param(ngt_log_first, uint, 0644);
MODULE_PARM_DESC(ngt_log_first, "NGT: count of initial samples to print");

static unsigned int ngt_fifo_burst = 8;
module_param(ngt_fifo_burst, uint, 0644);
MODULE_PARM_DESC(ngt_fifo_burst, "NGT: max words per poll");

static unsigned int ngt_resolution = 0x02;
module_param(ngt_resolution, uint, 0644);
MODULE_PARM_DESC(ngt_resolution, "NGT: resolution (0x01=16b, 0x02=20b, 0x03=24b, 0x04=32b)");

/* Test pattern to send */
static unsigned int ngt_tx_pattern = 0xD5;
module_param(ngt_tx_pattern, uint, 0644);
MODULE_PARM_DESC(ngt_tx_pattern, "NGT: TX test pattern byte (default 0xD5)");

/* CCR value for master mode clock configuration
 * Bits [2:0] = SCLKG (clock gating cycles)
 * Bits [4:3] = WSS (word select size: 0=16, 1=24, 2=32 SCLK cycles)
 *
 * For 8kHz sample rate with various BCLK rates:
 * - 16-bit stereo: BCLK = 8000 * 16 * 2 = 256 kHz
 * - 32-bit stereo: BCLK = 8000 * 32 * 2 = 512 kHz
 */
static int ngt_ccr_value = 0x00;
module_param(ngt_ccr_value, int, 0644);
MODULE_PARM_DESC(ngt_ccr_value, "NGT: CCR register value for clock config");

/* WSS (Word Select Size) */
static unsigned int ngt_wss = 0;
module_param(ngt_wss, uint, 0644);
MODULE_PARM_DESC(ngt_wss, "NGT: Word Select Size (0=16cyc, 1=24cyc, 2=32cyc)");

/* =====================================================
 * Polling state
 * ===================================================== */
struct ngt_poll_data {
    struct hrtimer timer;
    ktime_t period;
    struct dw_i2s_dev *dev;
    u64 polls;
    u64 tx_cnt;
    u64 rx_cnt;
    u64 match_cnt;
    u64 mismatch_cnt;
    bool running;
};

static struct ngt_poll_data *ngt_pd;
static u64 ngt_logged_rx;
static u64 ngt_logged_tx;
static u32 ngt_last_tx_left;
static u32 ngt_last_tx_right;

/* =====================================================
 * Register access helpers
 * ===================================================== */
static inline void i2s_write_reg(void __iomem *io_base, int reg, u32 val)
{
    writel(val, io_base + reg);
}

static inline u32 i2s_read_reg(void __iomem *io_base, int reg)
{
    return readl(io_base + reg);
}

/* =====================================================
 * Channel enable/disable
 * ===================================================== */
static inline void i2s_disable_channels(struct dw_i2s_dev *dev, u32 stream)
{
    u32 i;
    if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
        for (i = 0; i < 4; i++)
            i2s_write_reg(dev->i2s_base, TER(i), 0);
    } else {
        for (i = 0; i < 4; i++)
            i2s_write_reg(dev->i2s_base, RER(i), 0);
    }
}

static inline void i2s_clear_irqs(struct dw_i2s_dev *dev, u32 stream)
{
    u32 i;
    if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
        for (i = 0; i < 4; i++)
            (void)i2s_read_reg(dev->i2s_base, TOR(i));
    } else {
        for (i = 0; i < 4; i++)
            (void)i2s_read_reg(dev->i2s_base, ROR(i));
    }
}

/* =====================================================
 * TX: Push test pattern to FIFO
 * ===================================================== */
static void ngt_fifo_push_tx(struct dw_i2s_dev *dev)
{
    if (unlikely(!ngt_pd))
        return;

    u32 isr0 = i2s_read_reg(dev->i2s_base, ISR(0));

    /* Check if TX FIFO can accept data (TXFE = TX FIFO Empty or has space) */
    if (!(isr0 & ISR_TXFE))
        return;

    unsigned int burst = ngt_fifo_burst;
    while (burst--) {
        /* Create test pattern:
         * Left channel: pattern + counter low byte
         * Right channel: ~pattern + counter high byte
         * This makes it easy to verify loopback
         */
        u32 tx_left = ((ngt_tx_pattern & 0xFF) << 8) | (ngt_pd->tx_cnt & 0xFF);
        u32 tx_right = ((~ngt_tx_pattern & 0xFF) << 8) | ((ngt_pd->tx_cnt >> 8) & 0xFF);

        /* Write to TX FIFO */
        i2s_write_reg(dev->i2s_base, LRBR_LTHR(0), tx_left);
        i2s_write_reg(dev->i2s_base, RRBR_RTHR(0), tx_right);

        ngt_last_tx_left = tx_left;
        ngt_last_tx_right = tx_right;

        /* Log first few TX samples */
        if (ngt_logged_tx < ngt_log_first) {
            dev_info(dev->dev, "NGT_TX[%llu] L=0x%08x R=0x%08x\n",
                     (unsigned long long)ngt_pd->tx_cnt, tx_left, tx_right);
            ngt_logged_tx++;
        }

        ngt_pd->tx_cnt++;

        /* Clear TX overrun if any */
        i2s_read_reg(dev->i2s_base, TOR(0));

        /* Check if we can write more */
        isr0 = i2s_read_reg(dev->i2s_base, ISR(0));
        if (!(isr0 & ISR_TXFE))
            break;
    }
}

/* =====================================================
 * RX: Pop data from FIFO and verify loopback
 * ===================================================== */
static void ngt_fifo_pop_rx(struct dw_i2s_dev *dev)
{
    static u64 no_data_count = 0;

    if (unlikely(!ngt_pd))
        return;

    u32 isr0 = i2s_read_reg(dev->i2s_base, ISR(0));

    /* Check if RX data is available */
    if (!(isr0 & (ISR_RXDA | ISR_RXFO))) {
        no_data_count++;
        if (no_data_count <= 10 || (no_data_count % 10000) == 0) {
            dev_info(dev->dev, "NGT: NO RX DATA! ISR0=0x%08x polls=%llu no_data=%llu tx=%llu\n",
                     isr0, ngt_pd->polls, no_data_count, ngt_pd->tx_cnt);
        }
        return;
    }

    /* Reset no_data counter on first data received */
    if (no_data_count > 0 && ngt_pd->rx_cnt == 0) {
        dev_info(dev->dev, "NGT: *** FIRST RX DATA! *** after %llu no-data polls\n", no_data_count);
    }

    unsigned int burst = ngt_fifo_burst;
    while (burst--) {
        /* Read from RX FIFO */
        u32 rx_left = i2s_read_reg(dev->i2s_base, LRBR_LTHR(0));
        u32 rx_right = i2s_read_reg(dev->i2s_base, RRBR_RTHR(0));

        ngt_pd->rx_cnt++;

        /* Log first few RX samples */
        if (ngt_logged_rx < ngt_log_first) {
            dev_info(dev->dev, "NGT_RX[%llu] L=0x%08x R=0x%08x | bytes: L[1:0]=%02x,%02x R[1:0]=%02x,%02x\n",
                     (unsigned long long)(ngt_pd->rx_cnt - 1),
                     rx_left, rx_right,
                     (rx_left >> 8) & 0xFF, rx_left & 0xFF,
                     (rx_right >> 8) & 0xFF, rx_right & 0xFF);
            ngt_logged_rx++;
        }

        /* Verify loopback: Check if pattern byte matches */
        u8 rx_pattern_l = (rx_left >> 8) & 0xFF;
        u8 rx_pattern_r = (rx_right >> 8) & 0xFF;
        u8 expected_l = ngt_tx_pattern & 0xFF;
        u8 expected_r = ~ngt_tx_pattern & 0xFF;

        if (rx_pattern_l == expected_l && rx_pattern_r == expected_r) {
            ngt_pd->match_cnt++;
        } else {
            ngt_pd->mismatch_cnt++;
            if (ngt_pd->mismatch_cnt <= 10) {
                dev_warn(dev->dev, "NGT: MISMATCH! RX L=0x%02x (exp 0x%02x) R=0x%02x (exp 0x%02x)\n",
                         rx_pattern_l, expected_l, rx_pattern_r, expected_r);
            }
        }

        /* Clear RX overrun if any */
        i2s_read_reg(dev->i2s_base, ROR(0));

        /* Check if more data available */
        isr0 = i2s_read_reg(dev->i2s_base, ISR(0));
        if (!(isr0 & (ISR_RXDA | ISR_RXFO)))
            break;
    }
}

/* =====================================================
 * HR-timer poll callback - handles both TX and RX
 * ===================================================== */
static enum hrtimer_restart ngt_poll_cb(struct hrtimer *t)
{
    struct ngt_poll_data *pd = container_of(t, struct ngt_poll_data, timer);

    if (unlikely(!pd || !pd->dev))
        goto out;

    pd->polls++;

    if (pd->dev->use_pio) {
        /* First push TX data, then read RX data */
        ngt_fifo_push_tx(pd->dev);
        ngt_fifo_pop_rx(pd->dev);
    }

    /* Periodic stats logging */
    if ((pd->polls % 80000) == 0) {  /* Every ~10 seconds at 8kHz */
        dev_info(pd->dev->dev, "NGT: STATS: polls=%llu tx=%llu rx=%llu match=%llu mismatch=%llu\n",
                 pd->polls, pd->tx_cnt, pd->rx_cnt, pd->match_cnt, pd->mismatch_cnt);
    }

out:
    hrtimer_forward_now(&pd->timer, pd->period);
    return HRTIMER_RESTART;
}

/* =====================================================
 * I2S configuration for MASTER mode
 * ===================================================== */
static void dw_i2s_config_master(struct dw_i2s_dev *dev)
{
    u32 comp1, fifo_depth;
    u32 ccr_val;

    dev->config.sample_rate = 8000;
    dev->config.chan_nr = 2;

    switch (ngt_resolution) {
    case 0x00: dev->config.data_width = 12; break;
    case 0x01: dev->config.data_width = 16; break;
    case 0x02: dev->config.data_width = 20; break;
    case 0x03: dev->config.data_width = 24; break;
    case 0x04: dev->config.data_width = 32; break;
    default:   dev->config.data_width = 16; break;
    }

    dev->xfer_resolution = ngt_resolution;

    comp1 = i2s_read_reg(dev->i2s_base, dev->i2s_reg_comp1);
    fifo_depth = 1 << (1 + COMP1_FIFO_DEPTH_GLOBAL(comp1));

    if (!dev->fifo_th)
        dev->fifo_th = fifo_depth / 2;

    /* Disable all channels first */
    i2s_disable_channels(dev, SNDRV_PCM_STREAM_PLAYBACK);
    i2s_disable_channels(dev, SNDRV_PCM_STREAM_CAPTURE);

    /* Configure TX channel 0 */
    i2s_write_reg(dev->i2s_base, TCR(0), ngt_resolution);
    i2s_write_reg(dev->i2s_base, TFCR(0), dev->fifo_th - 1);
    i2s_write_reg(dev->i2s_base, TER(0), TER_TXCHEN);

    /* Configure RX channel 0 */
    i2s_write_reg(dev->i2s_base, RCR(0), ngt_resolution);
    i2s_write_reg(dev->i2s_base, RFCR(0), dev->fifo_th - 1);
    i2s_write_reg(dev->i2s_base, RER(0), RER_RXCHEN);

    /* Configure CCR for MASTER mode
     *
     * CCR Register bits:
     * [2:0] SCLKG - Serial clock gating (0-7)
     * [4:3] WSS   - Word Select Size
     *               0 = 16 SCLK cycles
     *               1 = 24 SCLK cycles
     *               2 = 32 SCLK cycles
     *               3 = Reserved
     *
     * For master mode, we need proper clock gating
     * SCLKG = 0 means no gating (continuous clock)
     */
    ccr_val = (ngt_wss & 0x3) << 3;  /* WSS in bits [4:3] */
    ccr_val |= (ngt_ccr_value & 0x7); /* SCLKG in bits [2:0] */

    i2s_write_reg(dev->i2s_base, CCR, ccr_val);

    dev_info(dev->dev, "NGT: MASTER mode configured: CCR=0x%08x (WSS=%d, SCLKG=%d)\n",
             ccr_val, ngt_wss, ngt_ccr_value & 0x7);
}

/* =====================================================
 * DAI operations (minimal)
 * ===================================================== */
static int dw_i2s_startup(struct snd_pcm_substream *substream, struct snd_soc_dai *cpu_dai)
{
    return 0;
}

static void dw_i2s_shutdown(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
    struct dw_i2s_dev *dev = snd_soc_dai_get_drvdata(dai);
    i2s_disable_channels(dev, substream->stream);
}

static int dw_i2s_hw_params(struct snd_pcm_substream *substream,
        struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
    return 0;
}

static int dw_i2s_prepare(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
    return 0;
}

static int dw_i2s_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *dai)
{
    return 0;
}

static int dw_i2s_set_fmt(struct snd_soc_dai *cpu_dai, unsigned int fmt)
{
    return 0;
}

static int dw_i2s_dai_probe(struct snd_soc_dai *dai)
{
    struct dw_i2s_dev *dev = snd_soc_dai_get_drvdata(dai);
    snd_soc_dai_init_dma_data(dai, &dev->play_dma_data, &dev->capture_dma_data);
    return 0;
}

static const struct snd_soc_dai_ops dw_i2s_dai_ops = {
    .probe      = dw_i2s_dai_probe,
    .startup    = dw_i2s_startup,
    .shutdown   = dw_i2s_shutdown,
    .hw_params  = dw_i2s_hw_params,
    .prepare    = dw_i2s_prepare,
    .trigger    = dw_i2s_trigger,
    .set_fmt    = dw_i2s_set_fmt,
};

static const struct snd_soc_component_driver dw_i2s_component = {
    .name = "dw-i2s-ngt-master",
    .legacy_dai_naming = 1,
};

/* =====================================================
 * Probe - device initialization for MASTER mode
 * ===================================================== */
static int dw_i2s_probe(struct platform_device *pdev)
{
    struct dw_i2s_dev *dev;
    struct resource *res;
    struct snd_soc_dai_driver *dw_i2s_dai;
    int ret;
    u32 comp1, fifo_depth;

    dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
    if (!dev)
        return -ENOMEM;

    dw_i2s_dai = devm_kzalloc(&pdev->dev, sizeof(*dw_i2s_dai), GFP_KERNEL);
    if (!dw_i2s_dai)
        return -ENOMEM;

    dw_i2s_dai->ops = &dw_i2s_dai_ops;

    dev->i2s_base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
    if (IS_ERR(dev->i2s_base))
        return PTR_ERR(dev->i2s_base);

    dev->dev = &pdev->dev;
    dev->i2s_reg_comp1 = I2S_COMP_PARAM_1;
    dev->i2s_reg_comp2 = I2S_COMP_PARAM_2;

    dev->reset = devm_reset_control_array_get_optional_shared(&pdev->dev);
    if (!IS_ERR(dev->reset))
        reset_control_deassert(dev->reset);

    /* Get the I2S clock if available */
    dev->clk = devm_clk_get_optional(&pdev->dev, "i2sclk");
    if (IS_ERR(dev->clk)) {
        dev_warn(&pdev->dev, "Could not get i2sclk: %ld\n", PTR_ERR(dev->clk));
        dev->clk = NULL;
    } else if (dev->clk) {
        ret = clk_prepare_enable(dev->clk);
        if (ret) {
            dev_err(&pdev->dev, "Failed to enable i2sclk: %d\n", ret);
        } else {
            dev_info(&pdev->dev, "NGT: i2sclk enabled, rate=%lu Hz\n",
                     clk_get_rate(dev->clk));
        }
    }

    comp1 = i2s_read_reg(dev->i2s_base, I2S_COMP_PARAM_1);
    fifo_depth = 1 << (1 + COMP1_FIFO_DEPTH_GLOBAL(comp1));

    dev->capability = DW_I2S_MASTER;  /* MASTER mode */
    dev->fifo_th = fifo_depth / 2;

    dw_i2s_dai->playback.channels_min = 2;
    dw_i2s_dai->playback.channels_max = 2;
    dw_i2s_dai->playback.rates = SNDRV_PCM_RATE_8000;
    dw_i2s_dai->playback.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S32_LE;

    dw_i2s_dai->capture.channels_min = 2;
    dw_i2s_dai->capture.channels_max = 2;
    dw_i2s_dai->capture.rates = SNDRV_PCM_RATE_8000;
    dw_i2s_dai->capture.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S32_LE;

    dev_set_drvdata(&pdev->dev, dev);

    ret = devm_snd_soc_register_component(&pdev->dev, &dw_i2s_component, dw_i2s_dai, 1);
    if (ret) {
        dev_err(&pdev->dev, "failed to register component: %d\n", ret);
        return ret;
    }

    ret = dw_pcm_register(pdev);
    dev->use_pio = true;
    dev->l_reg = LRBR_LTHR(0);
    dev->r_reg = RRBR_RTHR(0);

    if (ret) {
        dev_err(&pdev->dev, "could not register pcm: %d\n", ret);
        return ret;
    }

    pm_runtime_enable(&pdev->dev);

    /* Create poller state */
    if (!ngt_pd) {
        ngt_pd = devm_kzalloc(dev->dev, sizeof(*ngt_pd), GFP_KERNEL);
        if (!ngt_pd)
            return -ENOMEM;
        hrtimer_init(&ngt_pd->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL_PINNED);
        ngt_pd->timer.function = ngt_poll_cb;
    }
    ngt_pd->dev = dev;
    ngt_pd->period = ktime_set(0, ngt_poll_us * 1000);

    /* =====================================================
     * AUTOSTART in MASTER mode
     * ===================================================== */
    if (ngt_use_poll && ngt_autostart) {
        dev_info(dev->dev, "NGT: ============================================\n");
        dev_info(dev->dev, "NGT: I2S MASTER MODE - Loopback Test Driver\n");
        dev_info(dev->dev, "NGT: ============================================\n");
        dev_info(dev->dev, "NGT: Resolution      = 0x%02x (%d-bit)\n",
                 ngt_resolution,
                 ngt_resolution == 0x00 ? 12 :
                 ngt_resolution == 0x01 ? 16 :
                 ngt_resolution == 0x02 ? 20 :
                 ngt_resolution == 0x03 ? 24 : 32);
        dev_info(dev->dev, "NGT: TX Pattern      = 0x%02x\n", ngt_tx_pattern & 0xFF);
        dev_info(dev->dev, "NGT: Poll interval   = %d us\n", ngt_poll_us);
        dev_info(dev->dev, "NGT: WSS             = %d (%d SCLK cycles)\n",
                 ngt_wss, ngt_wss == 0 ? 16 : ngt_wss == 1 ? 24 : 32);
        dev_info(dev->dev, "NGT: ============================================\n");

        /* Clean stop first */
        i2s_write_reg(dev->i2s_base, CER, 0);
        i2s_write_reg(dev->i2s_base, IER, 0);
        i2s_write_reg(dev->i2s_base, IRER, 0);
        i2s_write_reg(dev->i2s_base, ITER, 0);

        /* Reset FIFOs */
        i2s_write_reg(dev->i2s_base, RXFFR, 1);
        i2s_write_reg(dev->i2s_base, TXFFR, 1);

        /* Configure for MASTER mode */
        dw_i2s_config_master(dev);

        /* Enable I2S block */
        i2s_write_reg(dev->i2s_base, IER, IER_IEN);

        /* Enable both TX and RX */
        i2s_write_reg(dev->i2s_base, ITER, 1);  /* TX enable */
        i2s_write_reg(dev->i2s_base, IRER, 1);  /* RX enable */

        /* Enable clocks - this starts BCLK and FS generation in master mode */
        i2s_write_reg(dev->i2s_base, CER, 1);

        /* Reset counters */
        ngt_logged_rx = 0;
        ngt_logged_tx = 0;
        ngt_pd->polls = 0;
        ngt_pd->tx_cnt = 0;
        ngt_pd->rx_cnt = 0;
        ngt_pd->match_cnt = 0;
        ngt_pd->mismatch_cnt = 0;

        /* Initial register dump */
        dev_info(dev->dev, "NGT: MASTER MODE REG DUMP:\n");
        dev_info(dev->dev, "  IER=0x%08x CER=0x%08x\n",
            i2s_read_reg(dev->i2s_base, IER),
            i2s_read_reg(dev->i2s_base, CER));
        dev_info(dev->dev, "  ITER=0x%08x IRER=0x%08x\n",
            i2s_read_reg(dev->i2s_base, ITER),
            i2s_read_reg(dev->i2s_base, IRER));
        dev_info(dev->dev, "  TCR0=0x%08x RCR0=0x%08x CCR=0x%08x\n",
            i2s_read_reg(dev->i2s_base, TCR(0)),
            i2s_read_reg(dev->i2s_base, RCR(0)),
            i2s_read_reg(dev->i2s_base, CCR));
        dev_info(dev->dev, "  TER0=0x%08x RER0=0x%08x\n",
            i2s_read_reg(dev->i2s_base, TER(0)),
            i2s_read_reg(dev->i2s_base, RER(0)));
        dev_info(dev->dev, "  TFCR0=0x%08x RFCR0=0x%08x\n",
            i2s_read_reg(dev->i2s_base, TFCR(0)),
            i2s_read_reg(dev->i2s_base, RFCR(0)));
        dev_info(dev->dev, "  ISR0=0x%08x\n",
            i2s_read_reg(dev->i2s_base, ISR(0)));

        /* Start poller */
        ngt_pd->running = true;
        hrtimer_start(&ngt_pd->timer, ngt_pd->period, HRTIMER_MODE_REL_PINNED);

        dev_info(dev->dev, "NGT: MASTER mode polling started!\n");
        dev_info(dev->dev, "NGT: Sending pattern 0x%02x, expecting loopback...\n",
                 ngt_tx_pattern & 0xFF);
    }

    return 0;
}

static void dw_i2s_remove(struct platform_device *pdev)
{
    struct dw_i2s_dev *dev = dev_get_drvdata(&pdev->dev);

    dev_info(&pdev->dev, "NGT: Removing driver.\n");
    dev_info(&pdev->dev, "NGT: FINAL STATS: polls=%llu tx=%llu rx=%llu match=%llu mismatch=%llu\n",
             ngt_pd ? ngt_pd->polls : 0,
             ngt_pd ? ngt_pd->tx_cnt : 0,
             ngt_pd ? ngt_pd->rx_cnt : 0,
             ngt_pd ? ngt_pd->match_cnt : 0,
             ngt_pd ? ngt_pd->mismatch_cnt : 0);

    if (ngt_pd) {
        hrtimer_cancel(&ngt_pd->timer);
        ngt_pd = NULL;
    }

    if (dev->clk)
        clk_disable_unprepare(dev->clk);

    if (!IS_ERR(dev->reset))
        reset_control_assert(dev->reset);
    pm_runtime_disable(&pdev->dev);
}

static const struct of_device_id dw_i2s_of_match[] = {
    { .compatible = "netgenetech,dw-i2s-ngt" },
    { },
};
MODULE_DEVICE_TABLE(of, dw_i2s_of_match);

static struct platform_driver dw_i2s_driver = {
    .probe  = dw_i2s_probe,
    .remove = dw_i2s_remove,
    .driver = {
        .name = "designware-i2s-NGT",
        .of_match_table = of_match_ptr(dw_i2s_of_match),
    },
};

module_platform_driver(dw_i2s_driver);

MODULE_AUTHOR("NGT");
MODULE_DESCRIPTION("DesignWare I2S MASTER Mode - Loopback Test Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:designware_i2s-NGT");
