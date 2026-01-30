/*
 * ALSA SoC Synopsys I2S Audio Layer - NGT Test Version
 *
 * This version includes configurable parameters for testing different
 * I2S/TDM modes without recompiling.
 *
 * Based on sound/soc/dwc/designware_i2s.c
 * Copyright (C) 2010 ST Microelectronics
 * Modified by NGT for FPGA A-law audio testing
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

/* Upstream-style width table reused for DMA FIFO sizing */
static const u32 ngt_fifo_width[COMP_MAX_WORDSIZE] = { 12, 16, 20, 24, 32, 0, 0, 0 };

/* Forward declarations */
int dw_pcm_register(struct platform_device *pdev);
void dw_pcm_push_tx(struct dw_i2s_dev *dev);
void dw_pcm_pop_rx(struct dw_i2s_dev *dev);

/* =====================================================
 * MODULE PARAMETERS - Basic polling control
 * ===================================================== */

static bool ngt_use_poll = true;
module_param(ngt_use_poll, bool, 0644);
MODULE_PARM_DESC(ngt_use_poll, "NGT: enable hrtimer polling path");

static unsigned int ngt_poll_us = 125;
module_param(ngt_poll_us, uint, 0644);
MODULE_PARM_DESC(ngt_poll_us, "NGT: poll period in microseconds");

static bool ngt_autostart = true;
module_param(ngt_autostart, bool, 0644);
MODULE_PARM_DESC(ngt_autostart, "NGT: autostart poller at probe time");

static unsigned int ngt_log_every = 100;
module_param(ngt_log_every, uint, 0644);
MODULE_PARM_DESC(ngt_log_every, "NGT: log ISR snapshot every N polls");

static unsigned int ngt_log_first = 50;
module_param(ngt_log_first, uint, 0644);
MODULE_PARM_DESC(ngt_log_first, "NGT: count of initial samples to print");

static unsigned int ngt_fifo_burst = 8;
module_param(ngt_fifo_burst, uint, 0644);
MODULE_PARM_DESC(ngt_fifo_burst, "NGT: max words per poll per direction");

static bool ngt_loop_rx_to_tx = false;
module_param(ngt_loop_rx_to_tx, bool, 0644);
MODULE_PARM_DESC(ngt_loop_rx_to_tx, "NGT: copy the last RX word into TX FIFO");

static bool ngt_swap_channels = false;
module_param(ngt_swap_channels, bool, 0644);
MODULE_PARM_DESC(ngt_swap_channels, "NGT: swap left and right channels");

static unsigned int ngt_alaw_shift = 8;
module_param(ngt_alaw_shift, uint, 0644);
MODULE_PARM_DESC(ngt_alaw_shift, "NGT: bit shift to extract A-law byte (0/8/16/24)");

/* =====================================================
 * MODULE PARAMETERS - I2S FORMAT TESTING
 * These allow testing different modes without recompiling!
 * ===================================================== */

/* Resolution register value:
 * 0x00 = 12-bit (ignore clock)
 * 0x01 = 16-bit
 * 0x02 = 20-bit
 * 0x03 = 24-bit
 * 0x04 = 32-bit
 */
static unsigned int ngt_resolution = 0x02;
module_param(ngt_resolution, uint, 0644);
MODULE_PARM_DESC(ngt_resolution, "NGT: resolution (0x01=16b, 0x02=20b, 0x03=24b, 0x04=32b)");

/* Frame offset: 0=left-justified, 1=I2S standard (1-bit delay) */
static unsigned int ngt_frame_offset = 1;
module_param(ngt_frame_offset, uint, 0644);
MODULE_PARM_DESC(ngt_frame_offset, "NGT: frame offset (0=left-justified, 1=I2S standard)");

/* TDM mode enable */
static bool ngt_tdm_enable = false;
module_param(ngt_tdm_enable, bool, 0644);
MODULE_PARM_DESC(ngt_tdm_enable, "NGT: enable TDM mode");

/* TDM slots: 2, 4, 8, or 16 */
static unsigned int ngt_tdm_slots = 2;
module_param(ngt_tdm_slots, uint, 0644);
MODULE_PARM_DESC(ngt_tdm_slots, "NGT: TDM slot count (2/4/8/16)");

/* Show full 32-bit raw values regardless of resolution */
static bool ngt_show_raw32 = true;
module_param(ngt_show_raw32, bool, 0644);
MODULE_PARM_DESC(ngt_show_raw32, "NGT: show full 32-bit raw register values");

/* CCR (Clock Configuration Register) override - for testing clock modes */
static int ngt_ccr_override = -1;  /* -1 = don't override */
module_param(ngt_ccr_override, int, 0644);
MODULE_PARM_DESC(ngt_ccr_override, "NGT: CCR register override (-1=auto, 0-7=manual)");

/* =====================================================
 * Polling state
 * ===================================================== */
struct ngt_poll_data {
    struct hrtimer timer;
    ktime_t period;
    struct dw_i2s_dev *dev;
    u64 polls, rx_cnt, tx_cnt;
    bool running;
};

static struct ngt_poll_data *ngt_pd;
static u64 ngt_logged_tx;
static u64 ngt_logged_rx;
static u32 ngt_last_tx_data;
static u32 ngt_last_rx_data;

/* =====================================================
 * G.711 A-law -> signed PCM16 conversion
 * ===================================================== */
static inline s16 ngt_alaw_to_pcm16(u8 a)
{
    a ^= 0x55;
    int sign = a & 0x80;
    int exponent = (a >> 4) & 0x07;
    int mantissa = a & 0x0F;
    int sample = mantissa << 4;
    if (exponent)
        sample = (sample + 0x100) << (exponent - 1);
    return sign ? sample : -sample;
}

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
 * Pattern detection - look for D5 and other patterns
 * ===================================================== */
static int ngt_d5_count = 0;
static int ngt_ff_count = 0;

static void ngt_check_patterns(struct dw_i2s_dev *dev, u32 rawL, u32 rawR, u8 aL, u8 aR)
{
    /* Check all bytes for D5 pattern */
    u8 bytes[8] = {
        rawL & 0xFF, (rawL >> 8) & 0xFF, (rawL >> 16) & 0xFF, (rawL >> 24) & 0xFF,
        rawR & 0xFF, (rawR >> 8) & 0xFF, (rawR >> 16) & 0xFF, (rawR >> 24) & 0xFF
    };

    for (int i = 0; i < 8; i++) {
        if (bytes[i] == 0xD5) {
            ngt_d5_count++;
            if (ngt_d5_count <= 5) {
                dev_info(dev->dev, "NGT: *** FOUND D5 at byte[%d]! *** rawL=0x%08x rawR=0x%08x\n",
                        i, rawL, rawR);
            }
        }
    }

    /* Also check extracted A-law values */
    if (aL == 0xD5 || aR == 0xD5) {
        ngt_d5_count++;
        if (ngt_d5_count <= 5) {
            dev_info(dev->dev, "NGT: *** D5 in A-law! *** L=0x%02x R=0x%02x\n", aL, aR);
        }
    }
}

/* =====================================================
 * FIFO operations
 * ===================================================== */
static void ngt_fifo_push_tx(struct dw_i2s_dev *dev)
{
    if (unlikely(!ngt_pd))
        return;

    unsigned int burst = ngt_fifo_burst;
    while (burst--) {
        u32 width = dev->config.data_width ? dev->config.data_width : 16;
        if (width > 32) width = 32;
        u32 mask = (width == 32) ? 0xFFFFFFFFu : (BIT(width) - 1);
        u32 sample;

        if (ngt_loop_rx_to_tx) {
            sample = ngt_last_rx_data & mask;
        } else {
            sample = (u32)(ngt_pd->tx_cnt & mask);
        }

        i2s_write_reg(dev->i2s_base, LRBR_LTHR(0), sample);
        i2s_write_reg(dev->i2s_base, RRBR_RTHR(0), sample);

        ngt_last_tx_data = sample;
        ngt_pd->tx_cnt++;
    }
}

static void ngt_fifo_pop_rx(struct dw_i2s_dev *dev)
{
    if (unlikely(!ngt_pd))
        return;

    unsigned int burst = ngt_fifo_burst;
    u32 isr0 = i2s_read_reg(dev->i2s_base, ISR(0));

    if (!(isr0 & (ISR_RXDA | ISR_RXFO)))
        return;

    while (burst--) {
        u32 rawL, rawR;

        /* Read raw 32-bit values from FIFOs */
        rawL = i2s_read_reg(dev->i2s_base, LRBR_LTHR(0));
        rawR = i2s_read_reg(dev->i2s_base, RRBR_RTHR(0));

        /* Optional channel swap */
        u32 L, R;
        if (ngt_swap_channels) {
            L = rawR;
            R = rawL;
        } else {
            L = rawL;
            R = rawR;
        }

        /* Extract A-law byte based on shift parameter */
        u8 aL = (L >> ngt_alaw_shift) & 0xFF;
        u8 aR = (R >> ngt_alaw_shift) & 0xFF;

        /* Check for D5 pattern in ALL byte positions */
        ngt_check_patterns(dev, rawL, rawR, aL, aR);

        /* Convert A-law to PCM */
        s16 pcmL = ngt_alaw_to_pcm16(aL);
        s16 pcmR = ngt_alaw_to_pcm16(aR);

        ngt_last_rx_data = R;
        ngt_pd->rx_cnt++;

        /* Logging */
        if (ngt_logged_rx < ngt_log_first) {
            if (ngt_show_raw32) {
                dev_info(dev->dev,
                    "NGT_RX[%llu] RAW: L=0x%08x R=0x%08x | "
                    "bytes L[3:0]=%02x,%02x,%02x,%02x R[3:0]=%02x,%02x,%02x,%02x | "
                    "A-law@%u: L=0x%02x R=0x%02x | PCM: L=%6d R=%6d\n",
                    (unsigned long long)(ngt_pd->rx_cnt - 1),
                    rawL, rawR,
                    (rawL >> 24) & 0xFF, (rawL >> 16) & 0xFF, (rawL >> 8) & 0xFF, rawL & 0xFF,
                    (rawR >> 24) & 0xFF, (rawR >> 16) & 0xFF, (rawR >> 8) & 0xFF, rawR & 0xFF,
                    ngt_alaw_shift, aL, aR,
                    pcmL, pcmR);
            } else {
                dev_info(dev->dev,
                    "NGT_RX[%llu] A-law: L=0x%02x->PCM=%6d, R=0x%02x->PCM=%6d\n",
                    (unsigned long long)(ngt_pd->rx_cnt - 1),
                    aL, pcmL, aR, pcmR);
            }
            ngt_logged_rx++;
        }

        i2s_read_reg(dev->i2s_base, ROR(0)); /* clear RX overrun */
        isr0 = i2s_read_reg(dev->i2s_base, ISR(0));
        if (!(isr0 & (ISR_RXDA | ISR_RXFO)))
            break;
    }
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
 * HR-timer poll callback
 * ===================================================== */
static enum hrtimer_restart ngt_poll_cb(struct hrtimer *t)
{
    struct ngt_poll_data *pd = container_of(t, struct ngt_poll_data, timer);

    if (unlikely(!pd || !pd->dev))
        goto out;

    if (pd->dev->use_pio) {
        u32 isr0 = i2s_read_reg(pd->dev->i2s_base, ISR(0));
        u32 iter = i2s_read_reg(pd->dev->i2s_base, ITER);
        u32 irer = i2s_read_reg(pd->dev->i2s_base, IRER);

        if ((irer & 1) && (isr0 & (ISR_RXDA | ISR_RXFO)))
            ngt_fifo_pop_rx(pd->dev);

        if ((iter & 1) && (isr0 & ISR_TXFE))
            ngt_fifo_push_tx(pd->dev);
    }

    pd->polls++;

out:
    hrtimer_forward_now(&pd->timer, pd->period);
    return HRTIMER_RESTART;
}

/* =====================================================
 * I2S start/stop/config
 * ===================================================== */
static void i2s_start(struct dw_i2s_dev *dev, struct snd_pcm_substream *substream)
{
    u32 reg = IER_IEN;

    /* TDM mode configuration */
    if (ngt_tdm_enable) {
        reg |= (ngt_tdm_slots - 1) << IER_TDM_SLOTS_SHIFT;
        reg |= IER_INTF_TYPE;
        reg |= ngt_frame_offset << IER_FRAME_OFF_SHIFT;
    }

    i2s_write_reg(dev->i2s_base, IER, reg);

    if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
        i2s_write_reg(dev->i2s_base, ITER, 1);
    else
        i2s_write_reg(dev->i2s_base, IRER, 1);

    i2s_write_reg(dev->i2s_base, CER, 1);
}

static void i2s_stop(struct dw_i2s_dev *dev, struct snd_pcm_substream *substream)
{
    i2s_clear_irqs(dev, substream->stream);
}

static void dw_i2s_config(struct dw_i2s_dev *dev, int stream)
{
    u32 ch;
    u32 dmacr;
    u32 comp1, fifo_depth;
    struct i2s_clk_config_data *config = &dev->config;

    /* Use module parameters for configuration */
    config->sample_rate = 8000;
    config->chan_nr = 2;

    /* Data width based on resolution parameter */
    switch (ngt_resolution) {
    case 0x00: config->data_width = 12; break;
    case 0x01: config->data_width = 16; break;
    case 0x02: config->data_width = 20; break;
    case 0x03: config->data_width = 24; break;
    case 0x04: config->data_width = 32; break;
    default:   config->data_width = 16; break;
    }

    dev->xfer_resolution = ngt_resolution;
    dev->frame_offset = ngt_frame_offset;

    /* FIFO depth from HW */
    comp1 = i2s_read_reg(dev->i2s_base, dev->i2s_reg_comp1);
    fifo_depth = 1 << (1 + COMP1_FIFO_DEPTH_GLOBAL(comp1));

    if (!dev->fifo_th)
        dev->fifo_th = fifo_depth / 2;

    i2s_disable_channels(dev, stream);

    dmacr = i2s_read_reg(dev->i2s_base, I2S_DMACR);
    if (stream == SNDRV_PCM_STREAM_PLAYBACK)
        dmacr &= ~(DMACR_DMAEN_TXCH0 * 0xf);
    else
        dmacr &= ~(DMACR_DMAEN_RXCH0 * 0xf);

    /* Program channel 0 */
    for (ch = 0; ch < 1; ch++) {
        if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
            i2s_write_reg(dev->i2s_base, TCR(ch), dev->xfer_resolution);
            i2s_write_reg(dev->i2s_base, TFCR(ch), fifo_depth - dev->fifo_th - 1);
            i2s_write_reg(dev->i2s_base, TER(ch), TER_TXCHEN);
            dmacr |= (DMACR_DMAEN_TXCH0 << ch);
        } else {
            i2s_write_reg(dev->i2s_base, RCR(ch), dev->xfer_resolution);
            i2s_write_reg(dev->i2s_base, RFCR(ch), dev->fifo_th - 1);
            i2s_write_reg(dev->i2s_base, RER(ch), RER_RXCHEN);
            dmacr |= (DMACR_DMAEN_RXCH0 << ch);
        }
    }

    i2s_write_reg(dev->i2s_base, I2S_DMACR, dmacr);

    /* CCR configuration - slave mode by default, but allow override */
    if (ngt_ccr_override >= 0) {
        i2s_write_reg(dev->i2s_base, CCR, ngt_ccr_override);
    }
    /* else CCR stays at 0 for slave mode */
}

/* =====================================================
 * DAI operations (minimal for test driver)
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
    struct dw_i2s_dev *dev = snd_soc_dai_get_drvdata(dai);
    dw_i2s_config(dev, substream->stream);
    return 0;
}

static int dw_i2s_prepare(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
    struct dw_i2s_dev *dev = snd_soc_dai_get_drvdata(dai);
    if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
        i2s_write_reg(dev->i2s_base, TXFFR, 1);
    else
        i2s_write_reg(dev->i2s_base, RXFFR, 1);
    return 0;
}

static int dw_i2s_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *dai)
{
    struct dw_i2s_dev *dev = snd_soc_dai_get_drvdata(dai);
    int ret = 0;

    switch (cmd) {
    case SNDRV_PCM_TRIGGER_START:
    case SNDRV_PCM_TRIGGER_RESUME:
    case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
        dev->active++;
        i2s_start(dev, substream);
        if (ngt_use_poll && dev->active == 1 && ngt_pd && !ngt_pd->running) {
            ngt_logged_rx = 0;
            ngt_logged_tx = 0;
            ngt_pd->running = true;
            hrtimer_start(&ngt_pd->timer, ngt_pd->period, HRTIMER_MODE_REL_PINNED);
        }
        break;
    case SNDRV_PCM_TRIGGER_STOP:
    case SNDRV_PCM_TRIGGER_SUSPEND:
    case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
        dev->active--;
        i2s_stop(dev, substream);
        if (ngt_use_poll && dev->active <= 0 && ngt_pd) {
            hrtimer_cancel(&ngt_pd->timer);
            ngt_pd->running = false;
        }
        break;
    default:
        ret = -EINVAL;
        break;
    }
    return ret;
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
    .name = "dw-i2s-ngt-test",
    .legacy_dai_naming = 1,
};

/* =====================================================
 * Probe - device initialization
 * ===================================================== */
static int dw_i2s_probe(struct platform_device *pdev)
{
    struct dw_i2s_dev *dev;
    struct resource *res;
    struct snd_soc_dai_driver *dw_i2s_dai;
    int ret;
    u32 comp1, comp2, fifo_depth;

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

    /* Reset if available */
    dev->reset = devm_reset_control_array_get_optional_shared(&pdev->dev);
    if (!IS_ERR(dev->reset))
        reset_control_deassert(dev->reset);

    /* Read HW configuration */
    comp1 = i2s_read_reg(dev->i2s_base, I2S_COMP_PARAM_1);
    comp2 = i2s_read_reg(dev->i2s_base, I2S_COMP_PARAM_2);
    fifo_depth = 1 << (1 + COMP1_FIFO_DEPTH_GLOBAL(comp1));

    dev->capability = DW_I2S_SLAVE;  /* Force slave mode */
    dev->fifo_th = fifo_depth / 2;

    /* Configure DAI */
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
     * AUTOSTART with configurable parameters
     * ===================================================== */
    if (ngt_use_poll && ngt_autostart) {
        dev_info(dev->dev, "NGT: ============================================\n");
        dev_info(dev->dev, "NGT: I2S TEST DRIVER - Configurable Parameters\n");
        dev_info(dev->dev, "NGT: ============================================\n");
        dev_info(dev->dev, "NGT: Resolution      = 0x%02x (%d-bit)\n",
                 ngt_resolution,
                 ngt_resolution == 0x00 ? 12 :
                 ngt_resolution == 0x01 ? 16 :
                 ngt_resolution == 0x02 ? 20 :
                 ngt_resolution == 0x03 ? 24 : 32);
        dev_info(dev->dev, "NGT: Frame offset    = %d (%s)\n",
                 ngt_frame_offset,
                 ngt_frame_offset ? "I2S Standard" : "Left-Justified");
        dev_info(dev->dev, "NGT: TDM mode        = %s (slots=%d)\n",
                 ngt_tdm_enable ? "ENABLED" : "disabled", ngt_tdm_slots);
        dev_info(dev->dev, "NGT: A-law shift     = %d bits\n", ngt_alaw_shift);
        dev_info(dev->dev, "NGT: Channel swap    = %s\n", ngt_swap_channels ? "YES" : "NO");
        dev_info(dev->dev, "NGT: Poll interval   = %d us\n", ngt_poll_us);
        dev_info(dev->dev, "NGT: CCR override    = %d\n", ngt_ccr_override);
        dev_info(dev->dev, "NGT: ============================================\n");

        /* Apply configuration */
        dev->config.sample_rate = 8000;
        dev->config.chan_nr = 2;
        dev->xfer_resolution = ngt_resolution;

        /* Clean stop */
        i2s_write_reg(dev->i2s_base, CER, 0);
        i2s_write_reg(dev->i2s_base, IER, 0);
        i2s_write_reg(dev->i2s_base, IRER, 0);
        i2s_write_reg(dev->i2s_base, ITER, 0);

        /* Reset FIFOs */
        i2s_write_reg(dev->i2s_base, RXFFR, 1);
        i2s_write_reg(dev->i2s_base, TXFFR, 1);

        /* Configure both directions */
        dw_i2s_config(dev, SNDRV_PCM_STREAM_PLAYBACK);
        dw_i2s_config(dev, SNDRV_PCM_STREAM_CAPTURE);

        /* Apply resolution to registers */
        i2s_write_reg(dev->i2s_base, TCR(0), ngt_resolution);
        i2s_write_reg(dev->i2s_base, RCR(0), ngt_resolution);
        i2s_write_reg(dev->i2s_base, RFCR(0), 0x07);

        /* Build IER register with TDM settings if enabled */
        u32 ier_val = IER_IEN;
        if (ngt_tdm_enable) {
            ier_val |= IER_INTF_TYPE;  /* Enable TDM */
            ier_val |= ((ngt_tdm_slots - 1) << IER_TDM_SLOTS_SHIFT);
            ier_val |= (ngt_frame_offset << IER_FRAME_OFF_SHIFT);
            dev_info(dev->dev, "NGT: TDM IER = 0x%08x\n", ier_val);
        }

        /* Enable I2S */
        i2s_write_reg(dev->i2s_base, IER, ier_val);
        i2s_write_reg(dev->i2s_base, IRER, 1);
        i2s_write_reg(dev->i2s_base, ITER, 1);
        i2s_write_reg(dev->i2s_base, CER, 1);

        /* Reset counters */
        ngt_logged_rx = 0;
        ngt_logged_tx = 0;
        ngt_d5_count = 0;
        ngt_ff_count = 0;

        /* Start poller */
        ngt_pd->running = true;
        hrtimer_start(&ngt_pd->timer, ngt_pd->period, HRTIMER_MODE_REL_PINNED);

        /* Register dump */
        dev_info(dev->dev, "NGT: REG DUMP:\n");
        dev_info(dev->dev, "  IER=0x%08x CER=0x%08x ITER=0x%08x IRER=0x%08x\n",
            i2s_read_reg(dev->i2s_base, IER),
            i2s_read_reg(dev->i2s_base, CER),
            i2s_read_reg(dev->i2s_base, ITER),
            i2s_read_reg(dev->i2s_base, IRER));
        dev_info(dev->dev, "  TCR0=0x%08x RCR0=0x%08x CCR=0x%08x\n",
            i2s_read_reg(dev->i2s_base, TCR(0)),
            i2s_read_reg(dev->i2s_base, RCR(0)),
            i2s_read_reg(dev->i2s_base, CCR));
        dev_info(dev->dev, "  TFCR0=0x%08x RFCR0=0x%08x\n",
            i2s_read_reg(dev->i2s_base, TFCR(0)),
            i2s_read_reg(dev->i2s_base, RFCR(0)));

        dev_info(dev->dev, "NGT: Polling started!\n");
    }

    return 0;
}

static void dw_i2s_remove(struct platform_device *pdev)
{
    struct dw_i2s_dev *dev = dev_get_drvdata(&pdev->dev);
    if (ngt_pd) {
        hrtimer_cancel(&ngt_pd->timer);
        ngt_pd = NULL;
    }
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
MODULE_DESCRIPTION("DesignWare I2S Test Driver - Configurable I2S/TDM modes");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:designware_i2s-NGT");
