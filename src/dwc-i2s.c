// SPDX-License-Identifier: GPL-2.0
/*
 * DesignWare I2S – NGT Debug / Bring-up Driver
 *
 * RX-only / poll-based / no ALSA card required
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/hrtimer.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>

#include <sound/designware_i2s.h>
#include "local.h"

/* ============================================================
 * PARAMETERS
 * ============================================================ */

static unsigned int poll_us = 100;
module_param(poll_us, uint, 0644);

static unsigned int alaw_shift = 16;
module_param(alaw_shift, uint, 0644);

/* ============================================================
 * SIMPLE HELPERS
 * ============================================================ */

static inline u32 rd(struct dw_i2s_dev *d, u32 r)
{
	return readl(d->i2s_base + r);
}

static inline void wr(struct dw_i2s_dev *d, u32 r, u32 v)
{
	writel(v, d->i2s_base + r);
}

/* ============================================================
 * A-LAW DECODE
 * ============================================================ */

static inline s16 alaw_to_pcm(u8 a)
{
	a ^= 0x55;
	int sign = a & 0x80;
	int exp  = (a >> 4) & 7;
	int man  = a & 0x0f;

	int v = man << 4;
	if (exp)
		v = (v + 0x100) << (exp - 1);

	return sign ? v : -v;
}

/* ============================================================
 * POLLER
 * ============================================================ */

struct ngt_poll {
	struct hrtimer t;
	struct dw_i2s_dev *dev;
	u64 count;
};

static struct ngt_poll *poller;

static enum hrtimer_restart poll_fn(struct hrtimer *t)
{
	struct ngt_poll *p = container_of(t, struct ngt_poll, t);
	struct dw_i2s_dev *d = p->dev;

	u32 isr = rd(d, ISR(0));
	if (isr & ISR_RXDA) {
		u32 L = rd(d, LRBR_LTHR(0));
		u32 R = rd(d, RRBR_RTHR(0));

		u8 al = (L >> alaw_shift) & 0xff;
		u8 ar = (R >> alaw_shift) & 0xff;

		s16 pl = alaw_to_pcm(al);
		s16 pr = alaw_to_pcm(ar);

		if (p->count < 32)
			dev_info(d->dev,
				"RX[%llu] L=0x%08x R=0x%08x | ALAW %02x %02x | PCM %6d %6d\n",
				p->count, L, R, al, ar, pl, pr);

		p->count++;
	}

	hrtimer_forward_now(&p->t, ns_to_ktime(poll_us * 1000));
	return HRTIMER_RESTART;
}

/* ============================================================
 * PROBE
 * ============================================================ */

static int dw_i2s_ngt_probe(struct platform_device *pdev)
{
	struct dw_i2s_dev *dev;
	struct resource *res;
	int ret;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dev->i2s_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(dev->i2s_base))
		return PTR_ERR(dev->i2s_base);

	dev->reset = devm_reset_control_get_optional_shared(&pdev->dev, NULL);
	if (!IS_ERR(dev->reset))
		reset_control_deassert(dev->reset);

	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);

	/* ================= FORCE HW CONFIG ================= */

	/* 16-bit slots */
	wr(dev, TCR(0), 0x02);
	wr(dev, RCR(0), 0x02);

	/* FIFO threshold minimal */
	wr(dev, RFCR(0), 0);
	wr(dev, TFCR(0), 0);

	/* Enable RX channel 0 */
	wr(dev, RER(0), RER_RXCHEN);

	/* Enable RX block */
	wr(dev, IRER, 1);

	/* Enable global */
	wr(dev, IER, IER_IEN);
	wr(dev, CER, 1);

	dev_info(&pdev->dev, "NGT: RX forced ON, poller starting\n");

	/* ================= START POLLER ================= */

	poller = devm_kzalloc(&pdev->dev, sizeof(*poller), GFP_KERNEL);
	if (!poller)
		return -ENOMEM;

	poller->dev = dev;
	hrtimer_init(&poller->t, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	poller->t.function = poll_fn;
	hrtimer_start(&poller->t,
		      ns_to_ktime(poll_us * 1000),
		      HRTIMER_MODE_REL);

	platform_set_drvdata(pdev, dev);
	return 0;
}

static void dw_i2s_ngt_remove(struct platform_device *pdev)
{
    struct dw_i2s_dev *dev = dev_get_drvdata(&pdev->dev);

    if (ngt_pd) {
        hrtimer_cancel(&ngt_pd->timer);
        ngt_pd = NULL;
    }

    if (dev && !IS_ERR(dev->reset))
        reset_control_assert(dev->reset);

    pm_runtime_disable(&pdev->dev);
}


/* ============================================================
 * OF MATCH
 * ============================================================ */

static const struct of_device_id ngt_of_match[] = {
	{ .compatible = "netgenetech,dw-i2s-ngt" },
	{ }
};
MODULE_DEVICE_TABLE(of, ngt_of_match);

static struct platform_driver ngt_driver = {
	.probe  = dw_i2s_ngt_probe,
	.remove = dw_i2s_ngt_remove,
	.driver = {
		.name = "dw-i2s-ngt",
		.of_match_table = ngt_of_match,
	},
};

module_platform_driver(ngt_driver);

MODULE_AUTHOR("NGT");
MODULE_DESCRIPTION("DesignWare I2S NGT Debug RX Driver");
MODULE_LICENSE("GPL");
