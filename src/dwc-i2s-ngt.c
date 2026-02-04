/*
 * ALSA SoC Synopsys I2S Audio Layer
 *
 * sound/soc/dwc/designware_i2s.c
 *
 * Copyright (C) 2010 ST Microelectronics
 * Rajeev Kumar <rajeevkumar.linux@gmail.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/mfd/syscon.h>
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
#include <linux/delay.h>


/* Upstream-style width table reused for DMA FIFO sizing */
static const u32 ngt_fifo_width[COMP_MAX_WORDSIZE] = { 12, 16, 20, 24, 32, 0, 0, 0 };

/*NGT Mod*/
int dw_pcm_register(struct platform_device *pdev);
void dw_pcm_push_tx(struct dw_i2s_dev *dev);
void dw_pcm_pop_rx(struct dw_i2s_dev *dev);

static bool ngt_use_poll = true;       /* enable hrtimer poller */
module_param(ngt_use_poll, bool, 0644);
MODULE_PARM_DESC(ngt_use_poll, "NGT: enable hrtimer polling path");

static unsigned int ngt_poll_us = 100; /* poll period (µs) */
module_param(ngt_poll_us, uint, 0644);
MODULE_PARM_DESC(ngt_poll_us, "NGT: poll period in microseconds");

static bool ngt_autostart = true;     /* enable lanes + start poller in probe */
module_param(ngt_autostart, bool, 0644);
MODULE_PARM_DESC(ngt_autostart, "NGT: autostart poller at probe time");

static unsigned int ngt_log_every = 100; /* status line every N polls */
module_param(ngt_log_every, uint, 0644);
MODULE_PARM_DESC(ngt_log_every, "NGT: log ISR snapshot every N polls");

static unsigned int ngt_log_first = 8;   /* print first N TX and RX samples */
module_param(ngt_log_first, uint, 0644);
MODULE_PARM_DESC(ngt_log_first, "NGT: count of initial samples to print");

static unsigned int ngt_fifo_burst = 8;  /* words per poll to push/pop */
module_param(ngt_fifo_burst, uint, 0644);
MODULE_PARM_DESC(ngt_fifo_burst, "NGT: max words per poll per direction");

static bool ngt_loop_rx_to_tx=false;
module_param(ngt_loop_rx_to_tx, bool, 0644);
MODULE_PARM_DESC(ngt_loop_rx_to_tx, "NGT: copy the last RX word into TX FIFO");

/* CCR configuration for bit alignment tuning */
static unsigned int ngt_ccr = 0x10;  /* Default: WSS=2 (32-bit) gives both channels */
module_param(ngt_ccr, uint, 0644);
MODULE_PARM_DESC(ngt_ccr, "NGT: CCR register value (bits[4:3]=WSS, bits[2:0]=SCLKG)");

/* Frame offset: 0=Left-Justified (no delay), 1=Standard I2S (1-bit delay) */
static unsigned int ngt_frame_offset = 0;  /* Default: Left-Justified for FPGA loopback */
module_param(ngt_frame_offset, uint, 0644);
MODULE_PARM_DESC(ngt_frame_offset, "NGT: Frame offset (0=Left-Justified, 1=Standard I2S)");

/* Sample rate - controls BCLK speed. Lower = slower BCLK */
static unsigned int ngt_sample_rate = 8000;  /* Default: 8000 Hz */
module_param(ngt_sample_rate, uint, 0644);
MODULE_PARM_DESC(ngt_sample_rate, "NGT: Sample rate in Hz (lower = slower BCLK, try 1000 or 100)");

/* Direct BCLK divider - if clk_set_rate doesn't work, try this
 * BCLK = source_clock / ngt_bclk_div
 * For 50 MHz source: div=100 gives 500 kHz, div=1000 gives 50 kHz, div=10000 gives 5 kHz
 */
static unsigned int ngt_bclk_div = 0;  /* 0 = use clk_set_rate, >0 = direct divider */
module_param(ngt_bclk_div, uint, 0644);
MODULE_PARM_DESC(ngt_bclk_div, "NGT: Direct BCLK divider (0=auto, 100=500kHz, 1000=50kHz, 10000=5kHz from 50MHz)");

//Polar state
struct ngt_poll_data {
	struct hrtimer timer;
	ktime_t period;
	struct dw_i2s_dev *dev;
	u64 polls, rx_cnt, tx_cnt;
	bool running;
};

static struct ngt_poll_data *ngt_pd; /* single instance */
static u64 ngt_logged_tx;            /* how many samples have been logged */
static u64 ngt_logged_rx;
static u32 ngt_last_tx_data;         /* remember most recent data words for periodic logging */
static u32 ngt_last_rx_data;

/*___________________________________________*/

/* --- NGT: A-law decode support (for convincing dmesg) --- */
/*
 * Your RX words look like 0xFFFF0000 / 0x003F0000 etc.
 * That usually means the "interesting" byte is in bits [23:16].
 * Default shift=16 extracts that byte. If needed try 24, 8, or 0.
 */
static unsigned int ngt_alaw_shift = 8;
module_param(ngt_alaw_shift, uint, 0644);
MODULE_PARM_DESC(ngt_alaw_shift, "NGT: which byte inside 32-bit RX word holds A-law (0/8/16/24)");

/* G.711 A-law -> signed PCM16 */
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

static inline u8 ngt_word_to_alaw(u32 w)
{
	unsigned int s = ngt_alaw_shift & 31;
	return (u8)((w >> s) & 0xFF);
}


static inline void i2s_write_reg(void __iomem *io_base, int reg, u32 val)
{
	writel(val, io_base + reg);
}

static inline u32 i2s_read_reg(void __iomem *io_base, int reg)
{
	return readl(io_base + reg);
}

static void __maybe_unused ngt_dump_regs(struct dw_i2s_dev *dev)
{
    dev_info(dev->dev, "NGT REG DUMP:\n"
        " CER=0x%08x IER=0x%08x ITER=0x%08x IRER=0x%08x\n"
        " TER0=0x%08x RER0=0x%08x ISR0=0x%08x CCR=0x%08x\n"
        " TCR0=0x%08x RCR0=0x%08x\n"
        " TFCR0=0x%08x RFCR0=0x%08x\n"
        " RXFLR=0x%08x TXFLR=0x%08x\n"
        " LRBR_LTHR(0)=0x%08x RRBR_RTHR(0)=0x%08x\n",
        i2s_read_reg(dev->i2s_base, CER),
        i2s_read_reg(dev->i2s_base, IER),
        i2s_read_reg(dev->i2s_base, ITER),
        i2s_read_reg(dev->i2s_base, IRER),
        i2s_read_reg(dev->i2s_base, TER(0)),
        i2s_read_reg(dev->i2s_base, RER(0)),
        i2s_read_reg(dev->i2s_base, ISR(0)),
        i2s_read_reg(dev->i2s_base, CCR),
        i2s_read_reg(dev->i2s_base, TCR(0)),
        i2s_read_reg(dev->i2s_base, RCR(0)),
        i2s_read_reg(dev->i2s_base, TFCR(0)),
        i2s_read_reg(dev->i2s_base, RFCR(0)),
        i2s_read_reg(dev->i2s_base, RXFFR),
        i2s_read_reg(dev->i2s_base, TXFFR),
        i2s_read_reg(dev->i2s_base, LRBR_LTHR(0)),
        i2s_read_reg(dev->i2s_base, RRBR_RTHR(0)));
}

/*NGT mod*/
#ifndef I2S_TXDMA
#define I2S_TXDMA	LRBR_LTHR(0)
#endif
#ifndef I2S_RXDMA
#define I2S_RXDMA	LRBR_LTHR(0)
#endif

static void ngt_fifo_push_tx(struct dw_i2s_dev *dev)
{
	if (unlikely(!ngt_pd))
    return;

	unsigned int burst = ngt_fifo_burst;
	while (burst--) {
		/* Pattern: incrementing counter, masked to active data width */
		u32 width = dev->config.data_width ? dev->config.data_width : 16;
		if (width > 32) width =32;
		u32 mask  = (width == 32) ? 0xFFFFFFFFu : (BIT(width)-1);
		u32 sample;

		if (ngt_loop_rx_to_tx){
			sample = ngt_last_rx_data & mask;                           //transmit the most recent RX word
		} else{
			sample = (u32)(ngt_pd->tx_cnt & mask);
		}

		i2s_write_reg(dev->i2s_base, LRBR_LTHR(0), sample);
		i2s_write_reg(dev->i2s_base, RRBR_RTHR(0), sample);

		ngt_last_tx_data = sample;
		ngt_pd->tx_cnt++;
		if (ngt_logged_tx < ngt_log_first) {
			dev_dbg(dev->dev, "NGT: TX[%llu]=0x%08x\n",
				 (unsigned long long)ngt_pd->tx_cnt - 1, sample);
			ngt_logged_tx++;
		}
	}
}

static void ngt_fifo_pop_rx(struct dw_i2s_dev *dev)
{

	if (unlikely(!ngt_pd))
    return;

    unsigned int burst = ngt_fifo_burst;
    u32 isr0 = i2s_read_reg(dev->i2s_base, ISR(0));

    /* Only read if RX data available */
    if (!(isr0 & (ISR_RXDA | ISR_RXFO)))
        return;

    while (burst--) {
        u32 L = i2s_read_reg(dev->i2s_base, LRBR_LTHR(0));
        u32 R = i2s_read_reg(dev->i2s_base, RRBR_RTHR(0));

		u8 aL = ngt_word_to_alaw(L);
        u8 aR = ngt_word_to_alaw(R);
        s16 pL = ngt_alaw_to_pcm16(aL);
        s16 pR = ngt_alaw_to_pcm16(aR);

        ngt_last_rx_data = R;
        ngt_pd->rx_cnt++;
		if (ngt_logged_rx < ngt_log_first) {
			u8 Lb3 = (L >> 24) & 0xff, Lb2 = (L >> 16) & 0xff, Lb1 = (L >> 8) & 0xff, Lb0 = L & 0xff;
			u8 Rb3 = (R >> 24) & 0xff, Rb2 = (R >> 16) & 0xff, Rb1 = (R >> 8) & 0xff, Rb0 = R & 0xff;

			u8 alL = (L >> ngt_alaw_shift) & 0xff;
			u8 alR = (R >> ngt_alaw_shift) & 0xff;

			s16 pcmL = ngt_alaw_to_pcm16(alL);
			s16 pcmR = ngt_alaw_to_pcm16(alR);

			dev_info(dev->dev,
				"NGT_RX[%llu] raw L=0x%08x R=0x%08x | bytes L=%02x %02x %02x %02x R=%02x %02x %02x %02x | ALAW(shift=%u) L=0x%02x R=0x%02x | PCM L=%6d R=%6d\n",
				(unsigned long long)(ngt_pd->rx_cnt - 1),
				L, R,
				Lb3, Lb2, Lb1, Lb0,
				Rb3, Rb2, Rb1, Rb0,
				ngt_alaw_shift, alL, alR, pcmL, pcmR);

			ngt_logged_rx++;
		}
		i2s_read_reg(dev->i2s_base, ROR(0)); /* clear RX overrun */
        isr0 = i2s_read_reg(dev->i2s_base, ISR(0));
        if (!(isr0 & (ISR_RXDA | ISR_RXFO)))
            break;
    }
}



/*_______________________________________________*/


#ifndef NGT_DISABLE_CHANNELS
#define NGT_DISABLE_CHANNELS 1
#endif

#if NGT_DISABLE_CHANNELS
static inline void i2s_disable_channels(struct dw_i2s_dev *dev, u32 stream)
{
	u32 i = 0;

	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		for (i = 0; i < 4; i++)
			i2s_write_reg(dev->i2s_base, TER(i), 0);
	} else {
		for (i = 0; i < 4; i++)
			i2s_write_reg(dev->i2s_base, RER(i), 0);
	}
}
#else
static inline void i2s_disable_channels(struct dw_i2s_dev *dev, u32 stream){
    /* no-op in poll-only builds */
}
#endif /* NGT_DISABLE_CHANNELS */

static inline void i2s_clear_irqs(struct dw_i2s_dev *dev, u32 stream)
{
	u32 i = 0;

	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		for (i = 0; i < 4; i++)
			(void)i2s_read_reg(dev->i2s_base, TOR(i)); //
	} else {
		for (i = 0; i < 4; i++)
			(void)i2s_read_reg(dev->i2s_base, ROR(i)); //
	}
}

#ifndef NGT_ENABLE_IRQS
#define NGT_ENABLE_IRQS 0
#endif

#if NGT_ENABLE_IRQS
static inline void i2s_disable_irqs(struct dw_i2s_dev *dev, u32 stream,
				    int chan_nr)
{
	u32 i, irq;

	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		for (i = 0; i < (chan_nr / 2); i++) {
			irq = i2s_read_reg(dev->i2s_base, IMR(i));
			i2s_write_reg(dev->i2s_base, IMR(i), irq | 0x30);
		}
	} else {
		for (i = 0; i < (chan_nr / 2); i++) {
			irq = i2s_read_reg(dev->i2s_base, IMR(i));
			i2s_write_reg(dev->i2s_base, IMR(i), irq | 0x03);
		}
	}
}

static inline void i2s_enable_irqs(struct dw_i2s_dev *dev, u32 stream,
				   int chan_nr)
{
	u32 i, irq;

	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		for (i = 0; i < (chan_nr / 2); i++) {
			irq = i2s_read_reg(dev->i2s_base, IMR(i));
			i2s_write_reg(dev->i2s_base, IMR(i), irq & ~0x30);
		}
	} else {
		for (i = 0; i < (chan_nr / 2); i++) {
			irq = i2s_read_reg(dev->i2s_base, IMR(i));
			i2s_write_reg(dev->i2s_base, IMR(i), irq & ~0x03);
		}
	}
}
#else
static inline void i2s_enable_irqs(struct dw_i2s_dev *dev, u32 stream,
				    int chan_nr) {/* no-op in poll-only builds */}
static inline void i2s_disable_irqs(struct dw_i2s_dev *dev,u32 stream,
				   int chan_nr) {/* no-op in poll-only builds */}
#endif


static void i2s_enable_dma(struct dw_i2s_dev *dev, u32 stream)
{
	u32 dma_reg = i2s_read_reg(dev->i2s_base, I2S_DMACR);

	/* Enable DMA handshake for stream */
	if (stream == SNDRV_PCM_STREAM_PLAYBACK)
		dma_reg |= I2S_DMAEN_TXBLOCK;
	else
		dma_reg |= I2S_DMAEN_RXBLOCK;

	i2s_write_reg(dev->i2s_base, I2S_DMACR, dma_reg);
}

static void i2s_disable_dma(struct dw_i2s_dev *dev, u32 stream)
{
	u32 dma_reg = i2s_read_reg(dev->i2s_base, I2S_DMACR);

	/* Disable DMA handshake for stream */
	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		dma_reg &= ~I2S_DMAEN_TXBLOCK;
		i2s_write_reg(dev->i2s_base, I2S_RTXDMA, 1);
	} else {
		dma_reg &= ~I2S_DMAEN_RXBLOCK;
		i2s_write_reg(dev->i2s_base, I2S_RRXDMA, 1);
	}
	i2s_write_reg(dev->i2s_base, I2S_DMACR, dma_reg);
}

static void i2s_start(struct dw_i2s_dev *dev,
		      struct snd_pcm_substream *substream)
{
	struct i2s_clk_config_data *config = &dev->config;

	u32 reg = IER_IEN;

	/* NGT: Force 16-bit configuration */
	dev_dbg(dev->dev, "NGT: i2s_start BEFORE: data_width=%d, xfer_resolution=0x%08x\n",
	         config->data_width, dev->xfer_resolution);

	dev_dbg(dev->dev, "NGT: i2s_start AFTER: data_width=%d, xfer_resolution=0x%08x\n",
	         config->data_width, dev->xfer_resolution);


	if (dev->tdm_slots) {
		reg |= (dev->tdm_slots - 1) << IER_TDM_SLOTS_SHIFT;
		reg |= IER_INTF_TYPE;
		reg |= dev->frame_offset << IER_FRAME_OFF_SHIFT;
	}

	i2s_write_reg(dev->i2s_base, IER, reg);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		i2s_write_reg(dev->i2s_base, ITER, 1);
	else
		i2s_write_reg(dev->i2s_base, IRER, 1);

	if (!(dev->use_pio || dev->is_jh7110))
		i2s_enable_dma(dev, substream->stream);

	i2s_enable_irqs(dev, substream->stream, config->chan_nr);

	i2s_write_reg(dev->i2s_base, CER, 1);
}

static void i2s_stop(struct dw_i2s_dev *dev,
		struct snd_pcm_substream *substream)
{
	/*if (dev->is_jh7110) {
		struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
		struct snd_soc_dai_link *dai_link = rtd->dai_link;

		dai_link->trigger_stop = SND_SOC_TRIGGER_ORDER_LDC;
	}*/
	i2s_clear_irqs(dev, substream->stream);

	i2s_disable_irqs(dev, substream->stream, 8);
}

static void i2s_pause(struct dw_i2s_dev *dev,
		struct snd_pcm_substream *substream)
{
	i2s_clear_irqs(dev, substream->stream);

	if (!(dev->use_pio || dev->is_jh7110))
		i2s_disable_dma(dev, substream->stream);

	i2s_disable_irqs(dev, substream->stream, 8);


	if (!dev->active) {
		i2s_write_reg(dev->i2s_base, CER, 0);
		/* Keep the device enabled until the shutdown - do not clear IER */
	}
}

static void dw_i2s_config(struct dw_i2s_dev *dev, int stream)
{
	u32 ch;
	u32 dmacr;
	u32 comp1, fifo_depth;
	struct i2s_clk_config_data *config = &dev->config;

	/* =====================================================
	 * NGT HARD LOCK — FPGA EXPECTATION
	 * ===================================================== */
	config->sample_rate   = ngt_sample_rate;  /* Use module parameter */
	config->chan_nr       = 2;
	config->data_width    = 32;
	dev->xfer_resolution  = 0x05;   /* 16-bit */

	dev_dbg(dev->dev,
		"NGT: dw_i2s_config stream=%s rate=%d width=%d ch=%d\n",
		stream == SNDRV_PCM_STREAM_PLAYBACK ? "PLAYBACK" : "CAPTURE",
		config->sample_rate, config->data_width, config->chan_nr);

	/* =====================================================
	 * FIFO depth from HW
	 * ===================================================== */
	comp1 = i2s_read_reg(dev->i2s_base, dev->i2s_reg_comp1);
	fifo_depth = 1 << (1 + COMP1_FIFO_DEPTH_GLOBAL(comp1));

	if (!dev->fifo_th)
		dev->fifo_th = fifo_depth / 2;

	/* =====================================================
	 * Disable channels before reprogramming
	 * ===================================================== */
	i2s_disable_channels(dev, stream);

	/* =====================================================
	 * Clear DMA enable bits (per channel)
	 * ===================================================== */
	dmacr = i2s_read_reg(dev->i2s_base, I2S_DMACR);
	if (stream == SNDRV_PCM_STREAM_PLAYBACK)
		dmacr &= ~(DMACR_DMAEN_TXCH0 * 0xf);
	else
		dmacr &= ~(DMACR_DMAEN_RXCH0 * 0xf);

	/* =====================================================
	 * Program channel 0 (stereo = 1 slot pair)
	 * ===================================================== */
	for (ch = 0; ch < 1; ch++) {

		if (stream == SNDRV_PCM_STREAM_PLAYBACK) {

			/* --- TX --- */
			i2s_write_reg(dev->i2s_base, TCR(ch), dev->xfer_resolution);
			i2s_write_reg(dev->i2s_base, TFCR(ch),
				      fifo_depth - dev->fifo_th - 1);
			i2s_write_reg(dev->i2s_base, TER(ch),
				      TER_TXCHEN |
				      (dev->tdm_mask << TER_TXSLOT_SHIFT));

			dmacr |= (DMACR_DMAEN_TXCH0 << ch);

			dev_dbg(dev->dev,
				"NGT: TX ch%d TCR=0x%02x TFCR=%u\n",
				ch,
				i2s_read_reg(dev->i2s_base, TCR(ch)),
				i2s_read_reg(dev->i2s_base, TFCR(ch)));

		} else {

			/* --- RX --- */
			i2s_write_reg(dev->i2s_base, RCR(ch), dev->xfer_resolution);
			i2s_write_reg(dev->i2s_base, RFCR(ch),
				      dev->fifo_th - 1);
			i2s_write_reg(dev->i2s_base, RER(ch),
				      RER_RXCHEN |
				      (dev->tdm_mask << RER_RXSLOT_SHIFT));

			dmacr |= (DMACR_DMAEN_RXCH0 << ch);

			dev_dbg(dev->dev,
				"NGT: RX ch%d RCR=0x%02x RFCR=%u\n",
				ch,
				i2s_read_reg(dev->i2s_base, RCR(ch)),
				i2s_read_reg(dev->i2s_base, RFCR(ch)));
		}
	}

	/* =====================================================
	 * Commit DMA control register
	 * ===================================================== */
	i2s_write_reg(dev->i2s_base, I2S_DMACR, dmacr);

	dev_dbg(dev->dev,
		"NGT: I2S_DMACR=0x%08x\n",
		i2s_read_reg(dev->i2s_base, I2S_DMACR));
}


static void dw_i2s_shutdown(struct snd_pcm_substream *substream,
			    struct snd_soc_dai *dai)
{
	struct dw_i2s_dev *dev = snd_soc_dai_get_drvdata(dai);

	//dev_dbg(dev->dev, "%s(%s)\n", __func__, substream->name);
	i2s_disable_channels(dev, substream->stream);
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		i2s_write_reg(dev->i2s_base, ITER, 0);
	else
		i2s_write_reg(dev->i2s_base, IRER, 0);

	i2s_disable_irqs(dev, substream->stream, 8);

	if (!dev->active) {
		i2s_write_reg(dev->i2s_base, CER, 0);
		i2s_write_reg(dev->i2s_base, IER, 0);
	}
}


static int dw_i2s_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct dw_i2s_dev *dev = snd_soc_dai_get_drvdata(dai);
	struct i2s_clk_config_data *config = &dev->config;
	union dw_i2s_snd_dma_data *dma_data = NULL;
	int ret;

	dev_dbg(dev->dev, "NGT: Original params: rate=%d, width=%d, channels=%d\n",
             params_rate(params),
             params_width(params),
             params_channels(params));

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		dma_data = &dev->play_dma_data;
	else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		dma_data = &dev->capture_dma_data;
	else
		return -1;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		config->data_width = 24;
		dma_data->dt.addr_width = 4;
		dev->xfer_resolution = 0x04;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		config->data_width = 32;
		dma_data->dt.addr_width = 4;
		dev->xfer_resolution = 0x05;
		break;
	default:
		dev_err(dev->dev, "designware-i2s: unsupported PCM fmt");
		return -EINVAL;
	}

	if (dev->tdm_slots)
		config->data_width = 32;

	config->chan_nr = params_channels(params);

	switch (config->chan_nr) {
	case EIGHT_CHANNEL_SUPPORT:
	case SIX_CHANNEL_SUPPORT:
	case FOUR_CHANNEL_SUPPORT:
	case TWO_CHANNEL_SUPPORT:
		break;
	default:
		dev_err(dev->dev, "channel count %d not supported\n", config->chan_nr);
		return -EINVAL;
	}

	dw_i2s_config(dev, substream->stream);

	config->sample_rate = params_rate(params);

	if (dev->capability & DW_I2S_MASTER) {
		u32 frame_length = config->data_width * 2;

		if (dev->bclk_ratio)
			frame_length = dev->bclk_ratio;

		switch (frame_length) {
		case 32:
			dev->ccr = 0x00;
			break;
		case 48:
			dev->ccr = 0x08;
			break;
		case 64:
			dev->ccr = 0x10;
			break;
		default:
			return -EINVAL;
		}

		if (dev->i2s_clk_cfg) {
			ret = dev->i2s_clk_cfg(config);
			if (ret < 0) {
				dev_err(dev->dev, "runtime audio clk config fail\n");
				return ret;
			}
		} else {
			u32 bitclk = config->sample_rate * frame_length;

			ret = clk_set_rate(dev->clk, bitclk);
			if (ret) {
				dev_err(dev->dev, "Can't set I2S clock rate: %d\n",
					ret);
				return ret;
			}
		}

		i2s_write_reg(dev->i2s_base, CCR, dev->ccr);
	}

	/* NGT: Force configured sample rate for FPGA - MUST BE AT END! */
	/* NGT: accept only configured rate, 16-bit, stereo */
	if (params_rate(params) != ngt_sample_rate ||
		params_channels(params) != 2 ||
		params_format(params) != SNDRV_PCM_FORMAT_S16_LE) {
		dev_err(dev->dev,
			"NGT: only %uHz / S16_LE / 2ch supported (got %uHz, %uch, fmt=%d)\n",
			ngt_sample_rate, params_rate(params), params_channels(params), params_format(params));
		return -EINVAL;
	}

	/* Lock config to FPGA expectation */
	config->sample_rate = ngt_sample_rate;
	config->chan_nr     = 2;
	config->data_width  = 32;
	dev->xfer_resolution = 0x05;

	dma_data->dt.addr_width = 4;

	dev_dbg(dev->dev, "NGT: FINAL config: rate=%d, width=%d, channels=%d\n",
	         config->sample_rate, config->data_width, config->chan_nr);

	return 0;
}

static int dw_i2s_prepare(struct snd_pcm_substream *substream,
			  struct snd_soc_dai *dai)
{
	struct dw_i2s_dev *dev = snd_soc_dai_get_drvdata(dai);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		i2s_write_reg(dev->i2s_base, TXFFR, 1);
	else
		i2s_write_reg(dev->i2s_base, RXFFR, 1);

	return 0;
}

/*NGT mod*/
/* File-scope hrtimer callback */
static enum hrtimer_restart ngt_poll_cb(struct hrtimer *t)
{
    struct ngt_poll_data *pd = container_of(t, struct ngt_poll_data, timer);
    struct dw_i2s_dev *dev;
    u32 tx_data, rx_data, rx_corrected;
    u32 isr;
    int i;
    bool exact_match, direct_match, shifted_match;
    static u64 total_rx = 0;
    static u64 match_count = 0;
    static u64 nonzero_count = 0;

    if (unlikely(!pd || !pd->dev))
        goto out;

    dev = pd->dev;

    if (dev->use_pio) {
        /* =======================================================
         * LOOPBACK TEST WITH SHIFT CORRECTION
         *
         * Due to I2S protocol timing (1-bit delay after WS edge),
         * and FPGA direct loopback, RX data is shifted right by 1 bit.
         *
         * At 50 MHz BCLK: RX = TX >> 1 (1-bit shift observed)
         * At 512 kHz BCLK: RX = TX (NO shift! timing works correctly)
         *
         * TX Pattern: 0xD5A5D5A5
         * Binary: 1101 0101 1010 0101 1101 0101 1010 0101
         * ======================================================= */

        /* TX pattern for testing */
        tx_data = 0xD5A5D5A5;  /* Original test pattern */

        i2s_write_reg(dev->i2s_base, LRBR_LTHR(0), tx_data);
        i2s_write_reg(dev->i2s_base, RRBR_RTHR(0), tx_data);
        pd->tx_cnt++;

        /* Read from RX FIFO */
        for (i = 0; i < 16; i++) {
            isr = i2s_read_reg(dev->i2s_base, ISR(0));

            if (!(isr & (ISR_RXDA | ISR_RXFO)))
                break;

            rx_data = i2s_read_reg(dev->i2s_base, LRBR_LTHR(0));
            (void)i2s_read_reg(dev->i2s_base, RRBR_RTHR(0));

            total_rx++;

            if (isr & ISR_RXFO)
                (void)i2s_read_reg(dev->i2s_base, ROR(0));

            if (rx_data == 0)
                continue;

            nonzero_count++;

            /* Check for matches:
             * 1. Direct match (RX == TX) - happens at slow BCLK (512 kHz)
             * 2. Shifted match (RX << 1 == TX) - happens at fast BCLK (50 MHz)
             */
            rx_corrected = rx_data << 1;

            direct_match = (rx_data == tx_data);
            shifted_match = (rx_corrected == tx_data);
            exact_match = direct_match || shifted_match;

            if (exact_match)
                match_count++;

            /* Log results */
            if (ngt_logged_rx < ngt_log_first) {
                const char *match_type;
                if (direct_match)
                    match_type = "DIRECT MATCH (no shift)";
                else if (shifted_match)
                    match_type = "SHIFTED MATCH (1-bit delay)";
                else
                    match_type = "MISMATCH";

                dev_info(dev->dev,
                    "NGT[%llu] TX=0x%08x | RX=0x%08x | %s | exact=%llu/%llu (%.0llu%%)\n",
                    total_rx,
                    tx_data,
                    rx_data,
                    match_type,
                    match_count, nonzero_count,
                    nonzero_count > 0 ? (match_count * 100) / nonzero_count : 0);
                ngt_logged_rx++;
            }
        }

        ngt_last_tx_data = tx_data;
        ngt_last_rx_data = rx_data;
        pd->rx_cnt = total_rx;
    }

    pd->polls++;

out:
    hrtimer_forward_now(&pd->timer, pd->period);
    return HRTIMER_RESTART;
}


/*____________________________________________________ */

static int dw_i2s_trigger(struct snd_pcm_substream *substream,
		int cmd, struct snd_soc_dai *dai)
{
	struct dw_i2s_dev *dev = snd_soc_dai_get_drvdata(dai);
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:

	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		dev->active++;
		i2s_start(dev, substream);
        /*NGT mod*/
		if (ngt_use_poll && dev->active == 1 && ngt_pd && !ngt_pd->running) {
			ngt_logged_rx = 0;
			ngt_logged_tx = 0;

			ngt_pd->running = true;
			hrtimer_start(&ngt_pd->timer, ngt_pd->period, HRTIMER_MODE_REL_PINNED);
		}


        /*______________________________*/
		break;

	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		dev->active--;
		i2s_pause(dev, substream);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		dev->active--;
		i2s_stop(dev, substream);
        /*NGT mod*/
		if (ngt_use_poll && dev->active <= 0 && ngt_pd) {
			hrtimer_cancel(&ngt_pd->timer);
			ngt_pd->running = false;
		}

        /*_______________________________*/
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int dw_i2s_set_fmt(struct snd_soc_dai *cpu_dai, unsigned int fmt)
{
	struct dw_i2s_dev *dev = snd_soc_dai_get_drvdata(cpu_dai);

	/* 1. Clock Provider Check */
	switch (fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) {
	case SND_SOC_DAIFMT_BC_FC:
		if (!(dev->capability & DW_I2S_SLAVE)) return -EINVAL;
		break;
	case SND_SOC_DAIFMT_BP_FP:
		if (!(dev->capability & DW_I2S_MASTER)) return -EINVAL;
		break;
	default:
		return -EINVAL;
	}

	/* 2. Data Format (I2S vs Left-Justified)
	 * I2S Standard has a 1-bit delay (frame_offset = 1)
	 * Left-Justified has 0 delay (frame_offset = 0)
	 */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		dev_info(dev->dev, "NGT: Format = I2S Standard (1-bit delay)\n");
		dev->frame_offset = 1;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		dev_info(dev->dev, "NGT: Format = Left-Justified (0-bit delay)\n");
		dev->frame_offset = 0;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		dev->frame_offset = 1;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		dev->frame_offset = 0;
		break;
	default:
		return -EINVAL;
	}

	/* 3. Polarity (WS Inversion)
	 * If your channels are swapped or you only see one value,
	 * switching between NB_NF and NB_IF usually fixes it.
	 */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		dev_info(dev->dev, "NGT: Polarity = Normal\n");
		break;
	case SND_SOC_DAIFMT_NB_IF:
		dev_info(dev->dev, "NGT: Polarity = Inverted\n");
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int dw_i2s_set_tdm_slot(struct snd_soc_dai *cpu_dai,	unsigned int tx_mask,
			   unsigned int rx_mask, int slots, int slot_width)
{
	struct dw_i2s_dev *dev = snd_soc_dai_get_drvdata(cpu_dai);

	if (slot_width != 32)
		return -EINVAL;

	if (slots < 0 || slots > 16)
		return -EINVAL;

	if (rx_mask != tx_mask)
		return -EINVAL;

	if (!rx_mask)
		return -EINVAL;

	dev->tdm_slots = slots;
	dev->tdm_mask = rx_mask;

	dev->l_reg = RSLOT_TSLOT(ffs(rx_mask) - 1);
	dev->r_reg = RSLOT_TSLOT(fls(rx_mask) - 1);

	return 0;
}

static int dw_i2s_set_bclk_ratio(struct snd_soc_dai *cpu_dai,
				 unsigned int ratio)
{
	struct dw_i2s_dev *dev = snd_soc_dai_get_drvdata(cpu_dai);

	dev_dbg(dev->dev, "%s(%d)\n", __func__, ratio);

	dev->bclk_ratio = ratio;

	return 0;
}

static int dw_i2s_dai_probe(struct snd_soc_dai *dai)
{
	struct dw_i2s_dev *dev = snd_soc_dai_get_drvdata(dai);

	snd_soc_dai_init_dma_data(dai, &dev->play_dma_data, &dev->capture_dma_data);
	return 0;
}

static int dw_i2s_startup(struct snd_pcm_substream *substream,
                          struct snd_soc_dai *cpu_dai){
							  return 0;
						}

static const struct snd_soc_dai_ops dw_i2s_dai_ops = {
	.probe		= dw_i2s_dai_probe,
	.startup	= dw_i2s_startup,
	.shutdown	= dw_i2s_shutdown,
	.hw_params	= dw_i2s_hw_params,
	.prepare	= dw_i2s_prepare,
	.trigger	= dw_i2s_trigger,
	.set_fmt	= dw_i2s_set_fmt,
	.set_tdm_slot	= dw_i2s_set_tdm_slot,
	.set_bclk_ratio	= dw_i2s_set_bclk_ratio,
};

#ifdef CONFIG_PM
static int dw_i2s_runtime_suspend(struct device *dev)
{
	struct dw_i2s_dev *dw_dev = dev_get_drvdata(dev);

	if (dw_dev->capability & DW_I2S_MASTER)
		clk_disable(dw_dev->clk);
	return 0;
}

static int dw_i2s_runtime_resume(struct device *dev)
{
	struct dw_i2s_dev *dw_dev = dev_get_drvdata(dev);
	int ret;

	if (dw_dev->capability & DW_I2S_MASTER) {
		ret = clk_enable(dw_dev->clk);
		if (ret)
			return ret;
	}
	return 0;
}

static int dw_i2s_suspend(struct snd_soc_component *component)
{
	struct dw_i2s_dev *dev = snd_soc_component_get_drvdata(component);

	if (dev->capability & DW_I2S_MASTER)
		clk_disable(dev->clk);
	return 0;
}

static int dw_i2s_resume(struct snd_soc_component *component)
{
	struct dw_i2s_dev *dev = snd_soc_component_get_drvdata(component);
	struct snd_soc_dai *dai;
	int stream, ret;

	if (dev->capability & DW_I2S_MASTER) {
		ret = clk_enable(dev->clk);
		if (ret)
			return ret;
	}

	for_each_component_dais(component, dai) {
		for_each_pcm_streams(stream)
			if (snd_soc_dai_stream_active(dai, stream))
				dw_i2s_config(dev, stream);
	}

	return 0;
}

#else
#define dw_i2s_suspend	NULL
#define dw_i2s_resume	NULL
#endif

static const struct snd_soc_component_driver dw_i2s_component = {
	.name			= "dw-i2s",
#ifdef CONFIG_PM
	.suspend		= NULL,             //NGT mod
	.resume			= NULL,
#endif
	.legacy_dai_naming	= 1,
};

/*
 * The following tables allow a direct lookup of various parameters
 * defined in the I2S block's configuration in terms of sound system
 * parameters.  Each table is sized to the number of entries possible
 * according to the number of configuration bits describing an I2S
 * block parameter.
 */

# if 1

// Width of (DMA) bus
static const u32 bus_widths[COMP_MAX_DATA_WIDTH] = {
	DMA_SLAVE_BUSWIDTH_1_BYTE,
	DMA_SLAVE_BUSWIDTH_2_BYTES,
	DMA_SLAVE_BUSWIDTH_4_BYTES,
	DMA_SLAVE_BUSWIDTH_UNDEFINED
};

/* PCM format to support channel resolution */
static const u32 formats[COMP_MAX_WORDSIZE] = {
	SNDRV_PCM_FMTBIT_S16_LE,
	SNDRV_PCM_FMTBIT_S16_LE,
	SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
	SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
	SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE,
	0,
	0,
	0
};

static int dw_configure_dai(struct dw_i2s_dev *dev,
				   struct snd_soc_dai_driver *dw_i2s_dai,
				   unsigned int rates)
{

	 /* Read component parameter registers to extract
	 *the I2S block's configuration.
	 */

	u32 comp1 = i2s_read_reg(dev->i2s_base, dev->i2s_reg_comp1);
	u32 comp2 = i2s_read_reg(dev->i2s_base, dev->i2s_reg_comp2);
	u32 fifo_depth = 1 << (1 + COMP1_FIFO_DEPTH_GLOBAL(comp1));
	u32 idx;

	if (dev->capability & DWC_I2S_RECORD &&
			dev->quirks & DW_I2S_QUIRK_COMP_PARAM1)
		comp1 = comp1 & ~BIT(5);

	if (dev->capability & DWC_I2S_PLAY &&
			dev->quirks & DW_I2S_QUIRK_COMP_PARAM1)
		comp1 = comp1 & ~BIT(6);

	if (COMP1_TX_ENABLED(comp1)) {
		dev_dbg(dev->dev, " designware: play supported\n");
		idx = COMP1_TX_WORDSIZE_0(comp1);
		if (WARN_ON(idx >= ARRAY_SIZE(formats)))
			return -EINVAL;
		if (dev->quirks & DW_I2S_QUIRK_16BIT_IDX_OVERRIDE)
			idx = 1;
		dw_i2s_dai->playback.channels_min = MIN_CHANNEL_NUM;
		dw_i2s_dai->playback.channels_max =
				2 * (COMP1_TX_CHANNELS(comp1) + 1);
		dw_i2s_dai->playback.formats = formats[idx];
		dw_i2s_dai->playback.rates = rates;
	}

	if (COMP1_RX_ENABLED(comp1)) {
		dev_dbg(dev->dev, "designware: record supported\n");
		idx = COMP2_RX_WORDSIZE_0(comp2);
		if (WARN_ON(idx >= ARRAY_SIZE(formats)))
			return -EINVAL;
		if (dev->quirks & DW_I2S_QUIRK_16BIT_IDX_OVERRIDE)
			idx = 1;
		dw_i2s_dai->capture.channels_min = MIN_CHANNEL_NUM;
		dw_i2s_dai->capture.channels_max =
				2 * (COMP1_RX_CHANNELS(comp1) + 1);
		dw_i2s_dai->capture.formats = formats[idx];
		dw_i2s_dai->capture.rates = rates;
	}

	if (COMP1_MODE_EN(comp1)) {
		dev_dbg(dev->dev, "designware: i2s master mode supported\n");
		dev->capability |= DW_I2S_MASTER;
	} else {
		dev_dbg(dev->dev, "designware: i2s slave mode supported\n");
		dev->capability |= DW_I2S_SLAVE;
	}

	dev->fifo_th = fifo_depth / 2;
	return 0;
}

static int dw_configure_dai_by_pd(struct dw_i2s_dev *dev,
				   struct snd_soc_dai_driver *dw_i2s_dai,
				   struct resource *res,
				   const struct i2s_platform_data *pdata)
{
	u32 comp1 = i2s_read_reg(dev->i2s_base, dev->i2s_reg_comp1);
	u32 idx = COMP1_APB_DATA_WIDTH(comp1);
	int ret;

	if (WARN_ON(idx >= ARRAY_SIZE(bus_widths)))
		return -EINVAL;

	ret = dw_configure_dai(dev, dw_i2s_dai, pdata->snd_rates);
	if (ret < 0)
		return ret;

	if (dev->quirks & DW_I2S_QUIRK_16BIT_IDX_OVERRIDE)
		idx = 1;

	if (dev->is_jh7110) {
		/* Use platform data and snd_dmaengine_dai_dma_data struct at the same time */
		u32 comp2 = i2s_read_reg(dev->i2s_base, I2S_COMP_PARAM_2);
		u32 idx2;

		if (COMP1_TX_ENABLED(comp1)) {
			idx2 = COMP1_TX_WORDSIZE_0(comp1);
			dev->play_dma_data.dt.addr = res->start + I2S_TXDMA;
			dev->play_dma_data.dt.fifo_size =
				(dev->fifo_th * 2 * ngt_fifo_width[idx2]) >> 3;
			dev->play_dma_data.dt.maxburst = 16;
		}
		if (COMP1_RX_ENABLED(comp1)) {
			idx2 = COMP2_RX_WORDSIZE_0(comp2);
			dev->capture_dma_data.dt.addr = res->start + I2S_RXDMA;
			dev->capture_dma_data.dt.fifo_size =
				(dev->fifo_th * 2 * ngt_fifo_width[idx2]) >> 3;
			dev->capture_dma_data.dt.maxburst = 16;
		}
	} else {
		/* Set DMA slaves info */
		dev->play_dma_data.pd.data = pdata->play_dma_data;
		dev->capture_dma_data.pd.data = pdata->capture_dma_data;
		dev->play_dma_data.pd.addr = res->start + I2S_TXDMA;
		dev->capture_dma_data.pd.addr = res->start + I2S_RXDMA;
		dev->play_dma_data.pd.max_burst = dev->fifo_th;
		dev->capture_dma_data.pd.max_burst = dev->fifo_th;
		dev->play_dma_data.pd.addr_width = bus_widths[idx];
		dev->capture_dma_data.pd.addr_width = bus_widths[idx];
		dev->play_dma_data.pd.filter = pdata->filter;
		dev->capture_dma_data.pd.filter = pdata->filter;
	}

	return 0;
}

static int dw_configure_dai_by_dt(struct dw_i2s_dev *dev,
				   struct snd_soc_dai_driver *dw_i2s_dai,
				   struct resource *res)
{
	u32 comp1 = i2s_read_reg(dev->i2s_base, I2S_COMP_PARAM_1);
	u32 comp2 = i2s_read_reg(dev->i2s_base, I2S_COMP_PARAM_2);
	u32 fifo_depth = 1 << (1 + COMP1_FIFO_DEPTH_GLOBAL(comp1));
	u32 idx2;
	int ret;

	ret = dw_configure_dai(dev, dw_i2s_dai, SNDRV_PCM_RATE_8000_384000);
	if (ret < 0)
		return ret;

	if (COMP1_TX_ENABLED(comp1)) {
		idx2 = COMP1_TX_WORDSIZE_0(comp1);

		dev->capability |= DWC_I2S_PLAY;
		dev->play_dma_data.dt.addr = res->start + I2S_TXDMA;
		dev->play_dma_data.dt.fifo_size =
			(fifo_depth * ngt_fifo_width[idx2]) >> 3;
		if (dev->max_dma_burst)
			dev->play_dma_data.dt.maxburst = dev->max_dma_burst;
		else
			dev->play_dma_data.dt.maxburst = fifo_depth / 2;
	}
	if (COMP1_RX_ENABLED(comp1)) {
		idx2 = COMP2_RX_WORDSIZE_0(comp2);

		dev->capability |= DWC_I2S_RECORD;
		dev->capture_dma_data.dt.addr = res->start + I2S_RXDMA;
		dev->capture_dma_data.dt.fifo_size =
			(fifo_depth * ngt_fifo_width[idx2]) >> 3;
		if (dev->max_dma_burst)
			dev->capture_dma_data.dt.maxburst = dev->max_dma_burst;
		else
			dev->capture_dma_data.dt.maxburst = fifo_depth / 2;
	}

	if (dev->max_dma_burst)
		dev->fifo_th = min(dev->max_dma_burst, dev->fifo_th);
	return 0;

}
#endif

#ifdef CONFIG_OF
/* clocks initialization with master mode on JH7110 SoC */
static int jh7110_i2s_crg_master_init(struct dw_i2s_dev *dev)
{
	static struct clk_bulk_data clks[] = {
		{ .id = "mclk" },
		{ .id = "mclk_ext" },
		{ .id = "mclk_inner" },
		{ .id = "apb" },
		{ .id = "i2sclk" },
	};
	struct reset_control *resets = devm_reset_control_array_get_exclusive(dev->dev);
	int ret;
	struct clk *pclk;
	struct clk *bclk_mst;
	struct clk *mclk;
	struct clk *mclk_ext;
	struct clk *mclk_inner;

	if (IS_ERR(resets))
		return dev_err_probe(dev->dev, PTR_ERR(resets), "failed to get i2s resets\n");

	ret = clk_bulk_get(dev->dev, ARRAY_SIZE(clks), clks);
	if (ret)
		return dev_err_probe(dev->dev, ret, "failed to get i2s clocks\n");

	mclk = clks[0].clk;
	mclk_ext = clks[1].clk;
	mclk_inner = clks[2].clk;
	pclk = clks[3].clk;
	bclk_mst = clks[4].clk;

	ret = clk_prepare_enable(pclk);
	if (ret)
		goto exit;

	/* Use inner mclk first and avoid uninitialized gpio for external mclk */
	ret = clk_set_parent(mclk, mclk_inner);
	if (ret)
		goto err_dis_pclk;

	ret = clk_prepare_enable(bclk_mst);
	if (ret)
		goto err_dis_pclk;

	/* deassert resets before set clock parent */
	ret = reset_control_deassert(resets);
	if (ret)
		goto err_dis_all;

	/* external clock (12.288MHz) for Audio */
	ret = clk_set_parent(mclk, mclk_ext);
	if (ret)
		goto err_dis_all;

	/* i2sclk will be got and enabled repeatedly later and should be disabled now. */
	clk_disable_unprepare(bclk_mst);
	clk_bulk_put(ARRAY_SIZE(clks), clks);
	dev->is_jh7110 = true;

	return 0;

err_dis_all:
	clk_disable_unprepare(bclk_mst);
err_dis_pclk:
	clk_disable_unprepare(pclk);
exit:
	clk_bulk_put(ARRAY_SIZE(clks), clks);
	return ret;
}

/* clocks initialization with slave mode on JH7110 SoC */
static int jh7110_i2s_crg_slave_init(struct dw_i2s_dev *dev)
{
	static struct clk_bulk_data clks[] = {
		{ .id = "mclk" },
		{ .id = "mclk_ext" },
		{ .id = "apb" },
		{ .id = "bclk_ext" },
		{ .id = "lrck_ext" },
		{ .id = "bclk" },
		{ .id = "lrck" },
		{ .id = "mclk_inner" },
		{ .id = "i2sclk" },
	};
	struct reset_control *resets = devm_reset_control_array_get_exclusive(dev->dev);
	int ret;
	struct clk *pclk;
	struct clk *bclk_mst;
	struct clk *bclk_ext;
	struct clk *lrck_ext;
	struct clk *bclk;
	struct clk *lrck;
	struct clk *mclk;
	struct clk *mclk_ext;
	struct clk *mclk_inner;

	if (IS_ERR(resets))
		return dev_err_probe(dev->dev, PTR_ERR(resets), "failed to get i2s resets\n");

	ret = clk_bulk_get(dev->dev, ARRAY_SIZE(clks), clks);
	if (ret)
		return dev_err_probe(dev->dev, ret, "failed to get i2s clocks\n");

	mclk = clks[0].clk;
	mclk_ext = clks[1].clk;
	pclk = clks[2].clk;
	bclk_ext = clks[3].clk;
	lrck_ext = clks[4].clk;
	bclk = clks[5].clk;
	lrck = clks[6].clk;
	mclk_inner = clks[7].clk;
	bclk_mst = clks[8].clk;

	ret = clk_prepare_enable(pclk);
	if (ret)
		goto exit;

	ret = clk_set_parent(mclk, mclk_inner);
	if (ret)
		goto err_dis_pclk;

	ret = clk_prepare_enable(bclk_mst);
	if (ret)
		goto err_dis_pclk;

	ret = reset_control_deassert(resets);
	if (ret)
		goto err_dis_all;

	/* The sources of BCLK and LRCK are the external codec. */
	ret = clk_set_parent(bclk, bclk_ext);
	if (ret)
		goto err_dis_all;

	ret = clk_set_parent(lrck, lrck_ext);
	if (ret)
		goto err_dis_all;

	ret = clk_set_parent(mclk, mclk_ext);
	if (ret)
		goto err_dis_all;

	/* The i2sclk will be got and enabled repeatedly later and should be disabled now. */
	clk_disable_unprepare(bclk_mst);
	clk_bulk_put(ARRAY_SIZE(clks), clks);
	dev->is_jh7110 = true;

	return 0;

err_dis_all:
	clk_disable_unprepare(bclk_mst);
err_dis_pclk:
	clk_disable_unprepare(pclk);
exit:
	clk_bulk_put(ARRAY_SIZE(clks), clks);
	return ret;
}

/* Special syscon initialization about RX channel with slave mode on JH7110 SoC */
static int jh7110_i2srx_crg_init(struct dw_i2s_dev *dev)
{
	struct regmap *regmap;
	unsigned int args[2];

	regmap = syscon_regmap_lookup_by_phandle_args(dev->dev->of_node,
						      "starfive,syscon",
						      2, args);
	if (IS_ERR(regmap))
		return dev_err_probe(dev->dev, PTR_ERR(regmap), "getting the regmap failed\n");

	/* Enable I2Srx with syscon register, args[0]: offset, args[1]: mask */
	regmap_update_bits(regmap, args[0], args[1], args[1]);

	return jh7110_i2s_crg_slave_init(dev);
}

static int jh7110_i2stx0_clk_cfg(struct i2s_clk_config_data *config)
{
	struct dw_i2s_dev *dev = container_of(config, struct dw_i2s_dev, config);
	u32 bclk_rate = config->sample_rate * 64;

	return clk_set_rate(dev->clk, bclk_rate);
}
#endif /* CONFIG_OF */

static int dw_i2s_probe(struct platform_device *pdev)
{
	const struct i2s_platform_data *pdata = pdev->dev.platform_data;
	struct dw_i2s_dev *dev;
	struct resource *res;
	struct snd_soc_dai_driver *dw_i2s_dai;
	int ret, irq = -ENXIO;
	bool clk_enabled = false;

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
	dev->is_jh7110 = false;

	/* --- Get IRQ EARLY (fix: irq was used uninitialized) --- */
	irq = platform_get_irq_optional(pdev, 0);

	/* Optional platform init (JH7110 etc.) */
	if (pdata && pdata->i2s_pd_init) {
		ret = pdata->i2s_pd_init(dev);
		if (ret)
			return ret;
	}

	/* Reset (non-jh7110 path) */
	if (!dev->is_jh7110) {
		dev->reset = devm_reset_control_array_get_optional_shared(&pdev->dev);
		if (IS_ERR(dev->reset))
			return PTR_ERR(dev->reset);

		ret = reset_control_deassert(dev->reset);
		if (ret)
			return ret;
	}

	/* Poll-only bring-up: do NOT request IRQ (handler removed) */
	if (irq >= 0)
		dev_info(&pdev->dev, "NGT: IRQ %d present but unused (poll-only)\n", irq);
	irq = -ENXIO;


	/* Basic init */
	of_property_read_u32(pdev->dev.of_node, "dma-maxburst", &dev->max_dma_burst);
	dev->bclk_ratio = 0;
	dev->i2s_reg_comp1 = I2S_COMP_PARAM_1;
	dev->i2s_reg_comp2 = I2S_COMP_PARAM_2;

	/* ---- Configure capability/DAI/DMA sizing from component params ---- */
	{
		u32 comp1 = i2s_read_reg(dev->i2s_base, I2S_COMP_PARAM_1);
		u32 comp2 = i2s_read_reg(dev->i2s_base, I2S_COMP_PARAM_2);
		u32 fifo_depth = 1 << (1 + COMP1_FIFO_DEPTH_GLOBAL(comp1));
		u32 idx2;
		unsigned int rates = SNDRV_PCM_RATE_8000_384000;

		dev->capability = 0;

		if (COMP1_TX_ENABLED(comp1)) {
			idx2 = COMP1_TX_WORDSIZE_0(comp1);
			dev->capability |= DWC_I2S_PLAY;

			dev->play_dma_data.dt.addr      = res->start + I2S_TXDMA;
			dev->play_dma_data.dt.fifo_size = (fifo_depth * ngt_fifo_width[idx2]) >> 3;
			dev->play_dma_data.dt.maxburst  = dev->max_dma_burst ? dev->max_dma_burst : fifo_depth / 2;

			dw_i2s_dai->playback.channels_min = MIN_CHANNEL_NUM;
			dw_i2s_dai->playback.channels_max = 2 * (COMP1_TX_CHANNELS(comp1) + 1);
			dw_i2s_dai->playback.formats =
				SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE;
			dw_i2s_dai->playback.rates = rates;
		}

		if (COMP1_RX_ENABLED(comp1)) {
			idx2 = COMP2_RX_WORDSIZE_0(comp2);
			dev->capability |= DWC_I2S_RECORD;

			dev->capture_dma_data.dt.addr      = res->start + I2S_RXDMA;
			dev->capture_dma_data.dt.fifo_size = (fifo_depth * ngt_fifo_width[idx2]) >> 3;
			dev->capture_dma_data.dt.maxburst  = dev->max_dma_burst ? dev->max_dma_burst : fifo_depth / 2;

			dw_i2s_dai->capture.channels_min = MIN_CHANNEL_NUM;
			dw_i2s_dai->capture.channels_max = 2 * (COMP1_RX_CHANNELS(comp1) + 1);
			dw_i2s_dai->capture.formats =
				SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE;
			dw_i2s_dai->capture.rates = rates;
		}

		if (COMP1_MODE_EN(comp1))
			dev->capability |= DW_I2S_MASTER;
		else
			dev->capability |= DW_I2S_SLAVE;

		dev->fifo_th = fifo_depth / 2;
	}

	/* Clock if master */
	if (dev->capability & DW_I2S_MASTER) {
		dev->clk = devm_clk_get(&pdev->dev, "i2sclk");
		if (IS_ERR(dev->clk)) {
			ret = PTR_ERR(dev->clk);
			goto err_assert_reset;
		}
		ret = clk_prepare_enable(dev->clk);
		if (ret)
			goto err_assert_reset;
		clk_enabled = true;
	}

	dev_set_drvdata(&pdev->dev, dev);

	ret = devm_snd_soc_register_component(&pdev->dev, &dw_i2s_component,
					     dw_i2s_dai, 1);
	if (ret) {
		dev_err(&pdev->dev, "failed to register component: %d\n", ret);
		goto err_disable_clk;
	}

	/* PCM registration policy:
	 * - If you want pure poll/PIO bring-up, keep use_pio=true when IRQ is present.
	 * - If no IRQ, use DMAengine.
	 */
	/* Poll-only bring-up: always use your PIO PCM */
	ret = dw_pcm_register(pdev);
	dev->use_pio = true;
	dev->l_reg = LRBR_LTHR(0);
	dev->r_reg = RRBR_RTHR(0);

	if (ret) {
		dev_err(&pdev->dev, "could not register pcm: %d\n", ret);
		goto err_disable_clk;
	}

	pm_runtime_enable(&pdev->dev);

	/* ---- Create poller state ONCE (no double alloc) ---- */
	if (!ngt_pd) {
		ngt_pd = devm_kzalloc(dev->dev, sizeof(*ngt_pd), GFP_KERNEL);
		if (!ngt_pd) {
			ret = -ENOMEM;
			goto err_pm_disable;
		}
		hrtimer_init(&ngt_pd->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL_PINNED);
		ngt_pd->timer.function = ngt_poll_cb;
	}
	ngt_pd->dev = dev;
	ngt_pd->period = ktime_set(0, ngt_poll_us * 1000);

	/* Optional autostart for bring-up without ALSA */

	if (ngt_use_poll && ngt_autostart) {
		/* Hard lock expected stream settings */
		dev->config.sample_rate = ngt_sample_rate;  /* Use module parameter */
		dev->config.data_width  = 32;
		dev->config.chan_nr     = 2;
		dev->xfer_resolution    = 0x05;

		/* ---- clean stop before touching FIFOs/config ---- */
		i2s_write_reg(dev->i2s_base, CER, 0);
		i2s_write_reg(dev->i2s_base, IER, 0);
		i2s_write_reg(dev->i2s_base, IRER, 0);
		i2s_write_reg(dev->i2s_base, ITER, 0);

		/* Reset FIFOs while disabled */
		i2s_write_reg(dev->i2s_base, RXFFR, 1);
		i2s_write_reg(dev->i2s_base, TXFFR, 1);

		/* Program FIFOs + enable channel 0 for both directions */
		dw_i2s_config(dev, SNDRV_PCM_STREAM_PLAYBACK);
		dw_i2s_config(dev, SNDRV_PCM_STREAM_CAPTURE);

		/* Force 32-bit width at the controller (redundant but explicit) */
		i2s_write_reg(dev->i2s_base, TCR(0), 0x05);
		i2s_write_reg(dev->i2s_base, RCR(0), 0x05);
		i2s_write_reg(dev->i2s_base, RFCR(0), 0x07); // Trigger FIFO on both channels

		/* ============================================================
		 * CRITICAL: Set the actual BCLK clock rate!
		 * BCLK = sample_rate * bits_per_channel * channels
		 * BCLK = sample_rate * 32 * 2 = sample_rate * 64
		 *
		 * For slow BCLK demonstration:
		 *   ngt_sample_rate=100  -> BCLK = 6.4 kHz (easy to see on scope)
		 *   ngt_sample_rate=1000 -> BCLK = 64 kHz
		 *   ngt_sample_rate=8000 -> BCLK = 512 kHz (default)
		 * ============================================================ */
		{
			u32 target_bclk = ngt_sample_rate * 64;  /* 32-bit stereo */
			int clk_ret;
			unsigned long rate_before, rate_after, rounded_rate;

			dev_info(dev->dev, "NGT: ========== CLOCK CONFIGURATION ==========\n");
			dev_info(dev->dev, "NGT: Target sample_rate = %u Hz\n", ngt_sample_rate);
			dev_info(dev->dev, "NGT: Target BCLK = %u Hz (sample_rate * 64)\n", target_bclk);

			if (dev->clk) {
				/* Get current rate before any changes */
				rate_before = clk_get_rate(dev->clk);
				dev_info(dev->dev, "NGT: Clock rate BEFORE: %lu Hz (%lu MHz)\n",
					rate_before, rate_before / 1000000);

				/* Try to find out what rate is actually possible */
				rounded_rate = clk_round_rate(dev->clk, target_bclk);
				dev_info(dev->dev, "NGT: clk_round_rate(%u) returned: %lu Hz\n",
					target_bclk, rounded_rate);

				/* Try to set the clock rate */
				clk_ret = clk_set_rate(dev->clk, target_bclk);
				if (clk_ret) {
					dev_err(dev->dev, "NGT: clk_set_rate(%u) FAILED: %d\n",
						target_bclk, clk_ret);
				} else {
					dev_info(dev->dev, "NGT: clk_set_rate(%u) returned SUCCESS (ret=0)\n", target_bclk);
				}

				rate_after = clk_get_rate(dev->clk);
				dev_info(dev->dev, "NGT: Clock rate AFTER: %lu Hz (%lu MHz)\n",
					rate_after, rate_after / 1000000);

				if (rate_before == rate_after) {
					dev_warn(dev->dev, "NGT: *** WARNING - Clock rate did NOT change! ***\n");
					if (rate_after > 10000000) {  /* Still in MHz range */
						dev_warn(dev->dev, "NGT: The 50MHz clock is passing through un-divided!\n");
						dev_warn(dev->dev, "NGT: This is likely because:\n");
						dev_warn(dev->dev, "NGT:   1. RP1 I2S clock doesn't support clk_set_rate(), OR\n");
						dev_warn(dev->dev, "NGT:   2. The clock divider is not in the I2S IP itself\n");
						dev_warn(dev->dev, "NGT: The I2S clock rate may need device tree configuration.\n");
					}
				} else {
					dev_info(dev->dev, "NGT: Clock rate changed! Old=%lu, New=%lu\n",
						rate_before, rate_after);
				}

				/* Also log the clock parent if available */
				{
					struct clk *parent_clk = clk_get_parent(dev->clk);
					if (parent_clk) {
						unsigned long parent_rate = clk_get_rate(parent_clk);
						dev_info(dev->dev, "NGT: Parent clock rate: %lu Hz (%lu MHz)\n",
							parent_rate, parent_rate / 1000000);
					}
				}
			} else {
				dev_err(dev->dev, "NGT: dev->clk is NULL! Cannot control clock rate.\n");
			}

			/* If ngt_bclk_div is set, try to manually calculate what divider would be needed */
			if (ngt_bclk_div > 0) {
				dev_info(dev->dev, "NGT: ngt_bclk_div=%u specified\n", ngt_bclk_div);
				dev_info(dev->dev, "NGT: For 50MHz source: 50000000 / %u = %u Hz BCLK\n",
					ngt_bclk_div, 50000000 / ngt_bclk_div);
				dev_info(dev->dev, "NGT: NOTE: Direct divider control is not yet implemented.\n");
				dev_info(dev->dev, "NGT: The RP1 clock may need device tree overlay configuration.\n");
			}

			dev_info(dev->dev, "NGT: ==========================================\n");
		}

		/* Set CCR register for clock/frame configuration
		 * Bits [4:3] = WSS (Word Select Size): 0=16cyc, 1=24cyc, 2=32cyc
		 * Bits [2:0] = SCLKG (clock gating)
		 * Try different values to fix bit alignment: 0x00, 0x08, 0x10, 0x18
		 */
		i2s_write_reg(dev->i2s_base, CCR, ngt_ccr);
		dev_info(dev->dev, "NGT: CCR set to 0x%02x (WSS=%d, SCLKG=%d)\n",
			ngt_ccr, (ngt_ccr >> 3) & 0x3, ngt_ccr & 0x7);

		/* Enable I2S core with frame offset configuration
		 * IER register bits:
		 *   Bit 0: IEN (global enable)
		 *   Bit 8: INTF_TYPE (0=I2S, 1=TDM) - set to enable frame_offset
		 *   Bits [11:9]: FRAME_OFF (0=Left-Justified, 1=Standard I2S)
		 *
		 * For FPGA direct loopback: use frame_offset=0 (Left-Justified)
		 * This eliminates the 1-bit delay and gives EXACT data match!
		 */
		{
			u32 ier_val = IER_IEN;  /* Start with global enable */

			/* If frame_offset control is needed, set INTF_TYPE bit */
			if (ngt_frame_offset == 0) {
				/* Left-Justified mode: set INTF_TYPE, frame_offset=0 */
				ier_val |= (1 << 8);  /* INTF_TYPE = 1 */
				ier_val |= (0 << 9);  /* FRAME_OFF = 0 (no delay) */
				dev_info(dev->dev, "NGT: IER=0x%08x (Left-Justified, frame_offset=0)\n", ier_val);
			} else {
				/* Standard I2S mode: set INTF_TYPE, frame_offset=1 */
				ier_val |= (1 << 8);  /* INTF_TYPE = 1 */
				ier_val |= (1 << 9);  /* FRAME_OFF = 1 (1-bit delay) */
				dev_info(dev->dev, "NGT: IER=0x%08x (Standard I2S, frame_offset=1)\n", ier_val);
			}

			i2s_write_reg(dev->i2s_base, IER, ier_val);
		}
		i2s_write_reg(dev->i2s_base, IRER, 1);      /* RX block enable */
		i2s_write_reg(dev->i2s_base, ITER, 1);      /* TX block enable */
		i2s_write_reg(dev->i2s_base, CER, 1);       /* core enable */

		dev->use_pio = true;

		/* Reset logging counters */
		ngt_logged_rx = 0;
		ngt_logged_tx = 0;

		/* Start poller */
		ngt_pd->running = true;
		hrtimer_start(&ngt_pd->timer, ngt_pd->period, HRTIMER_MODE_REL_PINNED);
		dev_info(dev->dev, "NGT: autostart done, polling started (%u us)\n", ngt_poll_us);

		/* Print detailed configuration for debugging */
		{
			u32 frame_bits = 32 * 2;  /* 32 bits x 2 channels = 64 bits per frame */
			u32 bclk_hz = ngt_sample_rate * frame_bits;
			dev_info(dev->dev, "NGT: ============================================\n");
			dev_info(dev->dev, "NGT: I2S Configuration:\n");
			dev_info(dev->dev, "NGT:   Sample Rate: %u Hz\n", ngt_sample_rate);
			dev_info(dev->dev, "NGT:   Data Width:  32 bits per channel\n");
			dev_info(dev->dev, "NGT:   Channels:    2 (stereo)\n");
			dev_info(dev->dev, "NGT:   Frame Size:  64 bits (32L + 32R)\n");
			dev_info(dev->dev, "NGT:   Frame Size:  8 bytes (4 LEFT + 4 RIGHT)\n");
			dev_info(dev->dev, "NGT:   BCLK:        %u Hz (%u kHz)\n", bclk_hz, bclk_hz/1000);
			dev_info(dev->dev, "NGT:   CCR:         0x%02x (WSS=%d)\n", ngt_ccr, (ngt_ccr >> 3) & 0x3);
			dev_info(dev->dev, "NGT:   Frame Offset: %u (%s)\n", ngt_frame_offset,
				ngt_frame_offset == 0 ? "Left-Justified" : "Standard I2S");
			dev_info(dev->dev, "NGT: ============================================\n");
		}
	}

	return 0;

err_pm_disable:
	pm_runtime_disable(&pdev->dev);
err_disable_clk:
	if (clk_enabled)
		clk_disable_unprepare(dev->clk);
err_assert_reset:
	if (!IS_ERR_OR_NULL(dev->reset))
		reset_control_assert(dev->reset);
	return ret;
}


static void dw_i2s_remove(struct platform_device *pdev)
{
	struct dw_i2s_dev *dev = dev_get_drvdata(&pdev->dev);
	if (ngt_pd) {
		hrtimer_cancel(&ngt_pd->timer);
		ngt_pd = NULL;
	}
	if (!IS_ERR(dev->reset)) reset_control_assert(dev->reset);
	pm_runtime_disable(&pdev->dev);
}

/*
 * OF match table
 *
 * TODO (optional, for auto-bind with your out-of-tree driver):
 *   - Add a private compatible FIRST, e.g., "mycorp,rp1-designware-i2s"
 *   - Change your board's I2S node compatible in DT/overlay to match it
 */
static const struct of_device_id dw_i2s_of_match[] = {
	/* { .compatible = "mycorp,rp1-designware-i2s" }, */ /* <- enable if you add overlay */
	{ .compatible = "netgenetech,dw-i2s-ngt" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, dw_i2s_of_match);

static const struct dev_pm_ops dwc_pm_ops = {
	SET_RUNTIME_PM_OPS(dw_i2s_runtime_suspend, dw_i2s_runtime_resume, NULL)
};

static struct platform_driver dw_i2s_driver = {
	.probe  = dw_i2s_probe,
	.remove = dw_i2s_remove,
	.driver = {
		.name = "designware-i2s-NGT",
		.of_match_table = of_match_ptr(dw_i2s_of_match),
		.pm = &dwc_pm_ops,
	},
};

module_platform_driver(dw_i2s_driver);

MODULE_AUTHOR("Cinmoy Purkaystha <ciye@netgenetech.com>");
MODULE_DESCRIPTION("DesignWare I2S (NGT poller + FIFO logging)");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:designware_i2s-NGT");
