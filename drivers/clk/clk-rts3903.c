/*
 * Copyright 2013-2014 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/delay.h>
#include <linux/of_platform.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <dt-bindings/clock/rlx-clock.h>

#include "bspchip.h"

enum {
	TYPE_RLE0745 = 1,
	TYPE_RTS3901 = 2,
	TYPE_RTS3903 = 3,
	TYPE_RTS3913 = 4,

	TYPE_FPGA = (1 << 16),
};

#define CLK_CHANGE_R		(0)
#define UART_CLK_LP_EN_R			(0x04)
#define MCU_CLK_CFG_R		(0x08)
#define DRAM_CLK_CFG_R		(0x0c)
#define CPU_CLK_CFG_R		(0x10)
#define XB2_CLK_CFG_R		(0x14)
#define BUS_CLK_CFG_R		(0x18)
#define I2S_CLK_CFG_R		(0x1c)
#define CIPHER_CLK_CFG_R	(0x20)
#define ETHERNET_CLK_CFG_R	(0x24)
#define UART_CLK_CFG_R		(0x28)
#define I2C_CLK_CFG_R		(0x2c)
#define H264_CLK_CFG_R		(0x30)
#define RC_CLK_CFG_R				(0x34)
#define RC_OSC_POW_CFG_R			(0x38)
#define RTC32K_DIV_CFG0_R			(0x3c)
#define RTC32K_DIV_CFG1_R			(0x40)
#define RTC32K_DIV_CFG2_R			(0x44)
#define RTC_CLK_CFG_R				(0x48)
#define USBPHY_CLK_CFG_R				(0x4c)
#define JPEG_CLK_CFG_R			(0x50)
#define PLL_BYPASS_CFG_R	(0x54)
#define ISP_SCAN_CLK_CFG_R	(0x58)
#define MIPI_SCAN_CLK_CFG_R	(0x5c)
#define SPDIF_CLK_CFG_R		(0x60)
#define PLATFORM_CONFIGURATION_R		(0x64)
#define CODEC_CLK_CFG_R		(0x68)
#define VIDEO_CLK_SEL_R		(0x68)
#define H265_ACLK_CFG_REG		(0x80)
#define H265_BCLK_CFG_REG		(0x84)
#define H265_CCLK_CFG_REG		(0x88)


#define BSP_CLK_PLL0_BASE_R		(0x000)
#define BSP_CLK_PLL1_BASE_R		(0x100)
#define BSP_CLK_PLL2_BASE_R		(0x200)
#define BSP_CLK_PLL3_BASE_R		(0x300)

#define SYS_PLL0_CTRL 0x00
#define SYS_PLL0_CFG 0x04
#define SYS_PLL0_SCCG_CFG0 0x08
#define SYS_PLL0_SCCG_CFG1 0x0C
#define SYS_PLL0_WDOG	0x10
#define SYS_PLL0_STATUS 0x14

#define SYSPLL_LDO_EN_PLL0 (1<<1)
#define SYSPLL_EN_PLL0  (1<<2)
#define SYSPLL_CK_RDY_PLL0 (1<<0)
#define REG_EN_SSC_PLL0 (1<<31)

#define BIG_SHORT_NUM	2
#define BIG_LONG_NUM	4

static struct clk *clks[RLX_CLK_H265_CK + 1];

static struct clk_onecell_data clk_data;

static DEFINE_SPINLOCK(clk_spinlock);

static u8 div_array[] = { 1, 2, 4, 6, 8, 10, 12, 14 };

static u32 clk_reg_v[32];

static unsigned int clk_platform_type;

struct clk_rlx {
	struct clk_hw hw;
	const char		*name;
	const struct clk_ops	*ops;
	const char * const *parent_names;
	u8			num_parents;
	void __iomem *clkreg;
	u32 clk_change;
	u32 rate;
	u32 *reg_v;
	u32 reg_i;
};

#define DEFINE_CLK_RLX(_name,	\
	_parent_names, _ops, _clk_reg, _clk_change)	\
	static struct clk_rlx _name = {	\
		.name = #_name,					\
		.parent_names = _parent_names,		\
		.num_parents = ARRAY_SIZE(_parent_names), \
		.ops = &_ops, \
		.clkreg	= (void __iomem	*)_clk_reg,		\
		.clk_change	= _clk_change,			\
		.reg_v		= clk_reg_v,	\
		.reg_i		= (((u32)_clk_reg & 0xff) >> 2),	\
	}

static void __iomem *clk_mapped_addr;
static void __iomem *pll_mapped_addr;

static const char * const rlx_root_parent_names[] = {
	"sys_osc",
};

static const char * const rlx_names_cpu_div[] = {
	"usb_pll_3", "sys_pll0_3", "sys_pll0_2", "sys_pll1_2",
};

static const char * const rlx_names_cpu_dec[] = {
	"cpu_ck_div"
};

static const char * const rlx_names_h264_div[] = {
	"usb_pll_2", "sys_pll0_3", "sys_pll1_3",
};

static const char * const rlx_names_h264_dec[] = {
	"h264_ck_div"
};

static const char * const rlx_names_jpeg_div[] = {
	"usb_pll_2", "sys_pll0_5", "sys_pll1_5",
};

static const char * const rlx_names_jpeg_dec[] = {
	"jpeg_ck_div"
};

static const char * const rlx_names_bus_div[] = {
	"usb_pll_2", "sys_pll0_5", "sys_pll1_2",
};

static const char * const rlx_names_bus_dec[] = {
	"bus_ck_div"
};

static const char * const rlx_names_dram_div[] = {
	"usb_pll_2", "sys_pll0_2", "sys_pll1_3", "sys_pll1_2"
};

static const char * const rlx_names_dram_dec[] = {
	"dram_ck_div"
};

static const char * const rlx_names_isp_div[] = {
	"usb_pll_2", "sys_pll0_5", "sys_pll1_5"
};

static const char * const rlx_names_isp_dec[] = {
	"isp_ck_div"
};

static const char * const rlx_names_mipi_div[] = {
	"usb_pll_2", "sys_pll0_5", "sys_pll1_2"
};

static const char * const rlx_names_mipi_dec[] = {
	"mipi_ck_div"
};

static const char * const rlx_names_i2c_div[] = {
	"usb_pll_3", "sys_pll0_2", "sys_pll1_3"
};

static const char * const rlx_names_xb2_div[] = {
	"usb_pll_2", "sys_pll0_5", "sys_pll1_2"
};

static const char * const rlx_names_uart_div[] = {
	"usb_pll_5", "sys_pll0_5", "sys_pll1_5"
};

static const char * const rlx_names_spdif_div[] = {
	"usb_pll_5", "sys_pll0_2", "sys_pll1_2", "sys_pll2_2"
};

static const char * const rlx_names_i2s_div[] = {
	"usb_pll_5", "sys_pll0_2", "sys_pll1_2", "sys_pll2_2",
};

static const char * const rlx_names_codec_div[] = {
	"usb_pll_5", "sys_pll0_2", "sys_pll1_2", "sys_pll2_2"
};

static const char *rlx_names_h265_aclk[] = {
	"dram_ck"
};

static const char *rlx_names_h265_bclk_div[] = {
	"usb_pll_3", "sys_pll0_2", "sys_pll0_3", "sys_pll1_5"
};

static const char *rlx_names_h265_bclk_dec[] = {
	"h265_bclk_ck_div"
};

static const char *rlx_names_h265_cclk_div[] = {
	"usb_pll_3", "sys_pll0_2", "sys_pll0_3", "sys_pll1_5"
};

static const char *rlx_names_h265_cclk_dec[] = {
	"h265_cclk_ck_div"
};

static const char * const rlx_names_v[] = {
	"dummy"
};

#define to_clk_rlx(_hw) container_of(_hw, struct clk_rlx, hw)

static inline u32 rts_clk_readl(void __iomem *offset)
{
	return readl(clk_mapped_addr + (u32)offset);
}

static inline void rts_clk_writel(unsigned int val, void __iomem *offset)
{
	writel(val, clk_mapped_addr + (u32)offset);
}

static inline u32 rts_pll_readl(void __iomem *offset)
{
	return readl(pll_mapped_addr + (u32)offset);
}

static inline void rts_pll_writel(unsigned int val, void __iomem *offset)
{
	writel(val, pll_mapped_addr + (u32)offset);
}


static short bignumcmp(unsigned short *a, unsigned short *b)
{
	short i;

	for (i = BIG_SHORT_NUM - 1; i >= 0; i--) {
		if (a[i] > b[i])
			return 1;
		else if (a[i] < b[i])
			return -1;
	}

	return 0;
}

static short bignumsub(unsigned short *a, unsigned short *b)
{
	short i, sub = 0;

	for (i = 0; i < BIG_SHORT_NUM; i++) {
		if (a[i] < b[i]) {
			a[i] -= b[i] + sub;
			sub = 1;
		} else {
			a[i] -= b[i];
			if (!a[i] && sub) {
				a[i] = 0xffff;
				sub = 1;
			} else {
			  a[i] -= sub;
			  sub = 0;
			}
		}
	}

	return sub;
}

static short bignumsubs(unsigned short *a, unsigned short *b)
{
	short i, sub = 0;

	for (i = 0; i < BIG_SHORT_NUM + 1; i++) {
		if (a[i] < b[i]) {
			a[i] -= b[i];
			a[i] -= sub;
			sub = 1;
		} else {
			a[i] -= b[i];
			if (!a[i] && sub) {
				a[i] = 0xffff;
				sub = 1;
			} else {
				a[i] -= sub;
				sub = 0;
			}
		}
	}

	return sub;
}

static void bignummuls(unsigned short *c,
	unsigned short *a, unsigned short b)
{
	short i, k;
	unsigned short add;
	unsigned long m;

	for (i = 0; i < BIG_SHORT_NUM + 1; i++)
		c[i] = 0;

	for (i = 0; i < BIG_SHORT_NUM; i++) {
		m = (unsigned long)a[i] * (unsigned long)b;
		c[i] += m & 0xffff;

		if (c[i] < (m & 0xffff))
			add = (m >> 16) + 1;
		else
			add = m >> 16;
		k = i + 1;

		for (; c[k] += add, c[k] < add; add = 1, k++)
			;
	}
}

static void bignummul(unsigned short *c,
	unsigned short *a, unsigned short *b)
{
	short i, j, k;
	unsigned short add;
	unsigned long m;

	memset((void *)c, 0, BIG_LONG_NUM * 2);

	for (i = 0; i < BIG_SHORT_NUM; i++) {
		for (j = 0; j < BIG_SHORT_NUM; j++) {

			m = (unsigned long)a[i] * (unsigned long)b[j];

			c[i + j] += m & 0xffff;
			if (c[i + j] < (m & 0xffff))
				add = (m >> 16) + 1;
			else
				add = m >> 16;
			k = i + j + 1;

			for (; c[k] += add, c[k] < add; add = 1, k++)
				;
		}
	}
}

static void bignumdiv(unsigned short *a,
	unsigned short *c, unsigned short *b)
{
	short i, h;
	unsigned long m, n;
	unsigned short *d, e[BIG_SHORT_NUM + 1];

	for (i = 0; i < BIG_SHORT_NUM; i++)
		a[i] = 0;

	d = (unsigned short *)&c[BIG_SHORT_NUM];

	for (i = BIG_SHORT_NUM - 1; i >= 0; i--) {
		for (; h = bignumcmp(d, b), h >= 0;
		     bignumsub(d, b), a[i + 1]++)
			;

		d = (unsigned short *)&c[i];

		do {
			m = ((unsigned long)c[i + BIG_SHORT_NUM] << 16) +
			    (unsigned long)c[i + BIG_SHORT_NUM - 1];
			n = m / ((unsigned long)b[BIG_SHORT_NUM - 1] + 1);
			if (n)
				a[i] += n;
			else {
				if (m > b[BIG_SHORT_NUM - 1]) {
					d[BIG_SHORT_NUM - 1] =
					    1 - bignumsub(d, b);
					a[i]++;
				}
				break;
			}

			memset((void *)e, 0, (BIG_SHORT_NUM + 1) << 1);

			bignummuls(e, b, (unsigned short)n);
			bignumsubs(d, e);

		} while (1);
	}

	for (; h = bignumcmp(d, b), h >= 0; bignumsub(d, b), a[0]++)
		;
}

static int rlx_set_parent(struct clk_hw *hw, u8 field_val)
{
	struct clk_rlx *clk = to_clk_rlx(hw);
	u32 reg;

	reg = clk->reg_v[clk->reg_i] & ~0x3;
	clk->reg_v[clk->reg_i] = (reg | field_val);

	return 0;
}

static u8 rlx_get_parent(struct clk_hw *hw)
{
	struct clk_rlx *clk = to_clk_rlx(hw);
	u32 reg;

	reg = clk->reg_v[clk->reg_i];
	reg &= 3;

	return reg;
}

static long rlx_pll_round_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long *prate)
{
	u32 parent_rate = *prate;
	int n = rate / parent_rate;
	unsigned long round_rate;
	u64 t = rate;

	if (n < 2) {
		round_rate = parent_rate << 1;
		return round_rate;
	}

	t <<= 12;
	t += (parent_rate >> 1);
	bignumdiv((unsigned short *)&n, (unsigned short *)&t,
		(unsigned short *)&parent_rate);

	t = 0;
	bignummul((unsigned short *)&t, (unsigned short *)&parent_rate,
		(unsigned short *)&n);

	round_rate = t >> 12;

	pr_debug("%s round:%lu\n", clk_hw_get_name(hw), round_rate);

	return round_rate;
}

static int rlx_pll_set_rate(struct clk_hw *hw, unsigned long rate,
			   unsigned long parent_rate)
{
	u64 t = rate;
	u32 n, f, reg;

	struct clk_rlx *clk = to_clk_rlx(hw);

	t <<= 12;
	t += (parent_rate >> 1);
	bignumdiv((unsigned short *)&n, (unsigned short *)&t,
		(unsigned short *)&parent_rate);

	f = n & 0xfff;
	n >>= 12;

	reg = (f<<9) + (n - 2);

	rts_pll_writel(reg, (void __iomem *)(clk->clkreg + SYS_PLL0_SCCG_CFG1));

	xb2flush();
	return 0;
}

static int rlx_pll_enable_clk(struct clk_hw *hw)
{
	u32 reg;
	u32 time = 5000;
	struct clk_rlx *clk = to_clk_rlx(hw);

	reg = rts_pll_readl((void __iomem *)(clk->clkreg + SYS_PLL0_CTRL));
	reg |= SYSPLL_LDO_EN_PLL0;
	rts_pll_writel(reg, (void __iomem *)(clk->clkreg + SYS_PLL0_CTRL));
	udelay(5);
	reg |= SYSPLL_EN_PLL0;
	rts_pll_writel(reg, (void __iomem *)clk->clkreg + SYS_PLL0_CTRL);

	while (--time) {
		reg = rts_pll_readl(clk->clkreg + SYS_PLL0_STATUS);
		if (reg & SYSPLL_CK_RDY_PLL0)
			break;
		udelay(1);
	}

	if (time == 0) {
		pr_err("%s enable failed\n", clk_hw_get_name(hw));
		return -ETIMEDOUT;
	}

	xb2flush();
	return 0;
}

static void rlx_pll_disable_clk(struct clk_hw *hw)
{
	u32 reg;
	struct clk_rlx *clk = to_clk_rlx(hw);

	reg = rts_pll_readl((void __iomem *)(clk->clkreg + SYS_PLL0_CTRL));
	reg &= ~SYSPLL_EN_PLL0;
	rts_pll_writel(reg, (void __iomem *)(clk->clkreg + SYS_PLL0_CTRL));
	reg &= ~SYSPLL_LDO_EN_PLL0;
	rts_pll_writel(reg, (void __iomem *)clk->clkreg + SYS_PLL0_CTRL);

	reg = rts_pll_readl((void __iomem *)(clk->clkreg + SYS_PLL0_SCCG_CFG0));
	reg &= ~REG_EN_SSC_PLL0;
	rts_pll_writel(reg, (void __iomem *)clk->clkreg + SYS_PLL0_SCCG_CFG0);

	xb2flush();
}

static unsigned long rlx_pll_recalc(struct clk_hw *hw,
	unsigned long parent_rate)
{
	u32 reg, n, f, rate;
	u64 t;
	struct clk_rlx *clk = to_clk_rlx(hw);

	reg = rts_pll_readl(clk->clkreg + SYS_PLL0_SCCG_CFG1);
	reg &= 0x1fffff;
	n = (reg & 0x1ff) + 2;
	f = reg >> 9;

	n <<= 12;
	n += f;

	bignummul((unsigned short *)&t, (unsigned short *)&parent_rate,
		(unsigned short *)&n);

	rate = (t >> 12);

	pr_debug("%s prate: %u\n", clk_hw_get_name(hw), rate);

	return rate;
}

static int rlx_pll_is_enabled(struct clk_hw *hw)
{
	struct clk_rlx *clk = to_clk_rlx(hw);
	u32 val;

	val = rts_pll_readl((void __iomem *)(clk->clkreg + SYS_PLL0_CTRL));

	return (val & SYSPLL_EN_PLL0);
}


static const struct clk_ops rlx_pll_ops = {
	.is_enabled = rlx_pll_is_enabled,
	.enable = rlx_pll_enable_clk,
	.disable = rlx_pll_disable_clk,
	.round_rate = rlx_pll_round_rate,
	.set_rate = rlx_pll_set_rate,
	.recalc_rate = rlx_pll_recalc,
};

int rts_pll_ssc_config(struct clk *pll, u32 ppm, u32 freq)
{
	u32 tbase, rate, step, reg;
	u64 tl;
	u32 n1, f1, n2, t1, t2;

	struct clk_rlx *clk = to_clk_rlx(__clk_get_hw(pll));

	tbase = 25000000 / freq;
	tbase &= ~1;

	rate = clk_get_rate(pll);

	n1 = rts_pll_readl(clk->clkreg + SYS_PLL0_SCCG_CFG1);
	f1 = n1 >> 9;
	n1 &= 0x1ff;
	n1 += 2;

	n1 <<= 12;
	n1 += f1;

	t1 = 1000000 - ppm;
	bignummul((unsigned short *)&tl, (unsigned short *)&rate,
		(unsigned short *)&t1);

	t1 = 1000000;
	bignumdiv((unsigned short *)&t2, (unsigned short *)&tl,
		(unsigned short *)&t1);

	t1 = 25000000;
	tl = t2;
	tl <<= 12;
	tl += (t1 >> 1);
	bignumdiv((unsigned short *)&n2, (unsigned short *)&tl,
		(unsigned short *)&t1);

	n1 -= n2;

	step = (n1 << 4) / tbase;

	reg = rts_pll_readl((void __iomem *)(clk->clkreg + SYS_PLL0_SCCG_CFG0));
	reg &= 0xfff000;
	reg |= (step << 12);
	reg |= REG_EN_SSC_PLL0;

	rts_pll_writel(reg, (void __iomem *)(clk->clkreg + SYS_PLL0_SCCG_CFG0));

	return 0;
}
EXPORT_SYMBOL_GPL(rts_pll_ssc_config);

static long rlx_decround_rate(struct clk_hw *hw, unsigned long rate,
			   unsigned long *prate)
{
	unsigned long divisor;
	unsigned long f = 0, n = 1, r = 0;
	int i;
	u32 p = *prate;
	u32 t;
	u64 tm;

	tm = p;
	tm <<= 6;
	tm += (rate>>1);

	bignumdiv((unsigned short *)&divisor, (unsigned short *)&tm,
		  (unsigned short *)&rate);

	for (i = 7; i >= 0; i--) {
		t = ((i + 1)  << 6);
			if (t <= divisor)
				break;
	}

	if (i < 0) {
		n = 0;
		f = 0;
	} else {
		n = i;
		f = 64 - (((n + 1) << 12) + (divisor >> 1)) / divisor;
	}

	pr_debug("round:n %ld f %ld\n", n, f);

	tm = 0;
	t = 64 - f;
	bignummul((unsigned short *)&tm, (unsigned short *)&p,
		(unsigned short *)&t);

	tm += ((n + 1) << 5);
	t = (n + 1) << 6;
	r = 0;
	bignumdiv((unsigned short *)&r, (unsigned short *)&tm,
		(unsigned short *)&t);


	pr_debug("%s round:%ld\n", clk_hw_get_name(hw), r);

	return r;
}


void setchgbit(int nr)
{
	u32 val;

	val = rts_clk_readl((void __iomem *)CLK_CHANGE_R);
	val |=	(1<<nr);
	rts_clk_writel(val, (void __iomem *)CLK_CHANGE_R);
}

void clrchgbit(int nr)
{
	u32 val;

	val = rts_clk_readl((void __iomem *)CLK_CHANGE_R);
	val &=	~(1<<nr);
	rts_clk_writel(val, (void __iomem *)CLK_CHANGE_R);
}

static int rlx_decset_rate(struct clk_hw *hw, unsigned long rate,
			unsigned long parent_rate)
{
	u64 p = parent_rate;
	unsigned long divisor;
	unsigned long f, n;
	u32 divreg, reg, t;
	int i;
	unsigned long  flags;

	struct clk_rlx *clk = to_clk_rlx(hw);

	pr_debug("setrate:%s p:%ld r:%ld\n", clk_hw_get_name(hw),
		parent_rate, rate);

	p <<= 6;
	p += (rate>>1);

	bignumdiv((unsigned short *)&divisor, (unsigned short *)&p,
		  (unsigned short *)&rate);

	for (i = 7; i >= 0; i--) {
		t = ((i + 1)  << 6);
		if (t <= divisor)
			break;
	}

	if (i < 0) {
		n = 0;
		f = 0;
	} else {
		n = i;
		f = 64 - (((n + 1) << 12) + (divisor >> 1)) / divisor;
	}

	divreg = (f << 16) | (i << 8);

	clk->rate = rate;

	clk->reg_v[clk->reg_i] &= ~0x3fff00;
	clk->reg_v[clk->reg_i] |= divreg;

	spin_lock_irqsave(&clk_spinlock, flags);

	reg = rts_clk_readl(clk->clkreg) & ~0x3fffff;

	setchgbit(clk->clk_change);

	reg |= clk->reg_v[clk->reg_i];
	rts_clk_writel(reg, (void __iomem *)clk->clkreg);

	clrchgbit(clk->clk_change);
	xb2flush();

	spin_unlock_irqrestore(&clk_spinlock, flags);

	pr_debug("setrate: %s reg:%x\n", clk_hw_get_name(hw), reg);

	return 0;
}

static unsigned long rlx_decrecalc(struct clk_hw *hw,
	unsigned long parent_rate)
{
	u32 reg, n, f, rate;
	struct clk_rlx *clk = to_clk_rlx(hw);
	u64 tm;
	u32 t;

	reg = rts_clk_readl(clk->clkreg) & 0xffff00;

	n = (reg & 0x700) >> 8;
	f = (reg & 0x3f0000) >> 16;

	pr_debug("recalc:%s %x %x %x %ld\n",
		clk_hw_get_name(hw), reg, n, f, parent_rate);

	tm = 0;
	t = 64 - f;
	bignummul((unsigned short *)&tm, (unsigned short *)&parent_rate,
		(unsigned short *)&t);

	tm += ((n + 1) << 5);
	t = (n + 1) << 6;
	rate = 0;
	bignumdiv((unsigned short *)&rate, (unsigned short *)&tm,
		(unsigned short *)&t);

	pr_debug("recalc:%s r:%u, %x\n", clk_hw_get_name(hw), rate, n);

	return rate + 1;
}

static int rlx_enable_clk(struct clk_hw *hw)
{
	u32 time = 5000;
	struct clk_rlx *clk = to_clk_rlx(hw);
	u32 reg = rts_clk_readl(clk->clkreg);

	reg |= CLK_ENABLE;
	rts_clk_writel(reg, clk->clkreg);

	while (--time) {
		if (rts_clk_readl(clk->clkreg) & CLK_ENABLE)
			break;
		udelay(1);
	}

	if (time == 0) {
		pr_err("%s enable failed\n", clk_hw_get_name(hw));
		return -ETIMEDOUT;
	}

	return 0;
}

static void rlx_disable_clk(struct clk_hw *hw)
{
	struct clk_rlx *clk = to_clk_rlx(hw);
	u32 reg;

	reg = rts_clk_readl(clk->clkreg);
	reg &= ~CLK_ENABLE;
	rts_clk_writel(reg, clk->clkreg);
	xb2flush();
}

static int rlx_clk_is_enabled(struct clk_hw *hw)
{
	struct clk_rlx *clk = to_clk_rlx(hw);
	u32 val;

	val = rts_clk_readl((void __iomem *)(clk->clkreg));

	return (val & CLK_ENABLE);
}


static const struct clk_ops rlx_decdivider_ops = {
	.is_enabled = rlx_clk_is_enabled,
	.round_rate = rlx_decround_rate,
	.set_rate = rlx_decset_rate,
	.recalc_rate = rlx_decrecalc,
	.enable = rlx_enable_clk,
	.disable = rlx_disable_clk,
};

static long rlx_round_rate(struct clk_hw *hw, unsigned long rate,
			   unsigned long *prate)
{
	unsigned long parent_rate = *prate;
	unsigned long divisor = (parent_rate + rate / 2) / rate;
	int i;

	for (i = 0; i < ARRAY_SIZE(div_array); i++)
		if (divisor <= div_array[i])
			break;

	if (i == ARRAY_SIZE(div_array))
		i--;

	divisor =  div_array[i];

	pr_debug("round: %s %lu\n", clk_hw_get_name(hw), divisor);

	return parent_rate / divisor;
}

static int rlx_set_rate(struct clk_hw *hw, unsigned long rate,
			unsigned long parent_rate)
{
	u32 divreg;
	struct clk_rlx *clk = to_clk_rlx(hw);
	u32 div = parent_rate / rate;
	u32 i;

	pr_debug("setrate:%s div: %u p:%lu r:%lu\n", clk_hw_get_name(hw),
		 div, parent_rate, rate);

	clk->rate = rate;

	for (i = 0; i < ARRAY_SIZE(div_array); i++)
		if (div <= div_array[i])
			break;

	if (i == ARRAY_SIZE(div_array))
		i--;

	divreg = i << 2;

	pr_debug("setrate:%s div:%u reg:%x\n", clk_hw_get_name(hw),
		div_array[i], divreg);

	clk->reg_v[clk->reg_i] &= ~0xfc;
	clk->reg_v[clk->reg_i] |= divreg;

	return 0;
}

static unsigned long rlx_recalc(struct clk_hw *hw, unsigned long parent_rate)
{
	u32 div;
	struct clk_rlx *clk = to_clk_rlx(hw);

	div = clk->reg_v[clk->reg_i] & 0x1c;
	div >>= 2;
	div = div_array[div];

	pr_debug("recalc:%s p: %lu, div: %u\n",
		clk_hw_get_name(hw), parent_rate, div);

	return parent_rate / div;
}

static int rlx_set_rate_s(struct clk_hw *hw, unsigned long rate,
			unsigned long parent_rate)
{
	u32 divreg;
	struct clk_rlx *clk = to_clk_rlx(hw);
	u32 div = parent_rate / rate;
	u32 i;
	u32 reg;
	unsigned long flags;

	pr_debug("setrate:%s div: %u p:%lu r:%lu\n", clk_hw_get_name(hw),
		 div, parent_rate, rate);

	clk->rate = rate;

	for (i = 0; i < ARRAY_SIZE(div_array); i++)
		if (div <= div_array[i])
			break;

	if (i == ARRAY_SIZE(div_array))
		i--;

	divreg = i << 2;

	pr_debug("setrate:%s div:%u reg:%x\n", clk_hw_get_name(hw),
		div_array[i], divreg);

	clk->reg_v[clk->reg_i] &= ~0x1c;
	clk->reg_v[clk->reg_i] |= divreg;

	reg = rts_clk_readl(clk->clkreg) & ~0x1f;

	spin_lock_irqsave(&clk_spinlock, flags);
	setchgbit(clk->clk_change);

	reg |= clk->reg_v[clk->reg_i];
	rts_clk_writel(reg, (void __iomem *)clk->clkreg);
	clrchgbit(clk->clk_change);
	spin_unlock_irqrestore(&clk_spinlock, flags);

	return 0;
}

static unsigned long rlx_recalc_s(struct clk_hw *hw,
		unsigned long parent_rate)
{
	u32 div;
	struct clk_rlx *clk = to_clk_rlx(hw);

	div = rts_clk_readl(clk->clkreg) & 0x1c;
	div >>= 2;
	div = div_array[div];

	pr_debug("recalc:%s p: %lu, div: %u\n",
		clk_hw_get_name(hw), parent_rate, div);

	return parent_rate / div + 1;
}

static const struct clk_ops rlx_divider_ops = {
	.round_rate = rlx_round_rate,
	.set_rate = rlx_set_rate,
	.recalc_rate = rlx_recalc,
	.set_parent = rlx_set_parent,
	.get_parent = rlx_get_parent,
};

static const struct clk_ops rlx_divider_ops_s = {
	.round_rate = rlx_round_rate,
	.set_rate = rlx_set_rate_s,
	.recalc_rate = rlx_recalc_s,
	.set_parent = rlx_set_parent,
	.get_parent = rlx_get_parent,
};

static long rlx_round_rate_c(struct clk_hw *hw, unsigned long rate,
		unsigned long *prate)
{
	unsigned long parent_rate = *prate;
	unsigned long divisor = (parent_rate + rate / 2) / rate;

	if (divisor > 254)
		divisor = 254;
	else if (divisor == 0)
		divisor = 1;

	if (divisor != 1) {
		divisor >>= 1;
		divisor <<= 1;
	}

	pr_debug("round: %s %lu\n", clk_hw_get_name(hw), divisor);

	return parent_rate / divisor;
}

static int rlx_set_rate_c(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	u32 reg;
	struct clk_rlx *clk = to_clk_rlx(hw);
	u32 div = (parent_rate + (rate >> 1)) / rate;
	unsigned long flags;


	pr_debug("setrate:%s div: %u p:%lu r:%lu\n", clk_hw_get_name(hw),
		div, parent_rate, rate);

	if (div > 254)
		div = 254;
	else if (div == 0)
		div = 1;

	clk->rate = rate;

	if (div != 1)
		div >>= 1;
	else
		div = 0;

	clk->reg_v[clk->reg_i] &= ~0x1fc;
	clk->reg_v[clk->reg_i] |= (div << 2);

	spin_lock_irqsave(&clk_spinlock, flags);
	reg = rts_clk_readl(clk->clkreg) & ~0x1ff;

	setchgbit(clk->clk_change);

	reg |= clk->reg_v[clk->reg_i];
	rts_clk_writel(reg, (void __iomem *)clk->clkreg);
	clrchgbit(clk->clk_change);

	xb2flush();
	spin_unlock_irqrestore(&clk_spinlock, flags);

	return 0;
}

static unsigned long rlx_recalc_c(struct clk_hw *hw,
	unsigned long parent_rate)
{
	u32 div;
	struct clk_rlx *clk = to_clk_rlx(hw);

	div = rts_clk_readl(clk->clkreg) & 0x1fc;
	div >>= 2;
	div <<= 1;

	if (div == 0)
		div++;

	pr_debug("recalc:%s p: %lu, div: %u\n",
		clk_hw_get_name(hw), parent_rate, div);

	return (parent_rate + (div >> 1)) / div + 1;
}

static const struct clk_ops rlx_divider_ops_c = {
	.is_enabled = rlx_clk_is_enabled,
	.round_rate = rlx_round_rate_c,
	.set_rate = rlx_set_rate_c,
	.set_parent = rlx_set_parent,
	.get_parent = rlx_get_parent,
	.recalc_rate = rlx_recalc_c,
	.enable = rlx_enable_clk,
	.disable = rlx_disable_clk,
};

static int rlx_set_rate_cpu(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	struct clk *clk0, *clk1, *clk2;
	int ret = 0;

	clk1 = clk_get(NULL, "cpu_ck_div");
	if (IS_ERR(clk1))
		return PTR_ERR(clk1);

	clk2 = clk_get(NULL, "cpu_ck_dec");
	if (IS_ERR(clk2))
		return PTR_ERR(clk2);

	clk_set_parent(clk2, clk1);

	switch (rate) {
	case 500000000:
		clk0 = clk_get(NULL, "sys_pll1_2");
		if (IS_ERR(clk0))
			return PTR_ERR(clk0);

		clk_set_parent(clk1, clk0);

		clk_set_rate(clk1, 500000000);
		clk_set_rate(clk2, 500000000);
		break;
	case 400000000:
		clk0 = clk_get(NULL, "sys_pll1_2");
		if (IS_ERR(clk0))
			return PTR_ERR(clk0);

		clk_set_parent(clk1, clk0);

		clk_set_rate(clk1, 500000000);
		clk_set_rate(clk2, 398437500);
		break;
	case 300000000:
		clk0 = clk_get(NULL, "sys_pll1_2");
		if (IS_ERR(clk0))
			return PTR_ERR(clk0);

		clk_set_parent(clk1, clk0);

		clk_set_rate(clk1, 500000000);
		clk_set_rate(clk2, 296875000);
		break;
	case 200000000:
		clk0 = clk_get(NULL, "sys_pll1_2");
		if (IS_ERR(clk0))
			return PTR_ERR(clk0);

		clk_set_parent(clk1, clk0);

		clk_set_rate(clk1, 500000000);
		clk_set_rate(clk2, 199218750);
		break;
	case 50000000:
		clk0 = clk_get(NULL, "sys_pll1_2");
		if (IS_ERR(clk0))
			return PTR_ERR(clk0);

		clk_set_parent(clk1, clk0);

		clk_set_rate(clk1, 250000000);
		clk_set_rate(clk2, 50000000);
		break;
	default:
		pr_debug("%s %ld not supported yet\n",
			clk_hw_get_name(hw), rate);
		ret = -EINVAL;
	}

	clk_put(clk0);
	clk_put(clk1);
	clk_put(clk2);

	return ret;
}

static unsigned long rlx_recalc_cpu(struct clk_hw *hw,
	unsigned long parent_rate)
{
	struct clk *clk0;
	u32 rate = 0;

	if (clk_platform_type & TYPE_FPGA)
		return 50000000;

	clk0 = clk_get(NULL, "cpu_ck_dec");
	if (IS_ERR(clk0))
		return PTR_ERR(clk0);
	rate = clk_get_rate(clk0) - 1;
	clk_put(clk0);

	return rate;
}

static long rlx_round_rate_v(struct clk_hw *hw, unsigned long rate,
			   unsigned long *prate)
{
	return rate;
}

static const struct clk_ops rlx_clk_cpu_ops = {
	.is_enabled = rlx_clk_is_enabled,
	.round_rate = rlx_round_rate_v,
	.set_rate = rlx_set_rate_cpu,
	.recalc_rate = rlx_recalc_cpu,
	.enable = rlx_enable_clk,
	.disable = rlx_disable_clk,
};

static int rlx_set_rate_h264(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	struct clk *clk0, *clk1, *clk2;
	int ret = 0;

	clk1 = clk_get(NULL, "h264_ck_div");
	if (IS_ERR(clk1))
		return PTR_ERR(clk1);

	clk2 = clk_get(NULL, "h264_ck_dec");
	if (IS_ERR(clk2))
		return PTR_ERR(clk2);

	clk_set_parent(clk2, clk1);

	switch (rate) {
	case 400000000:
		clk0 = clk_get(NULL, "sys_pll0_3");
		if (IS_ERR(clk0))
			return PTR_ERR(clk0);

		clk_set_parent(clk1, clk0);

		clk_set_rate(clk1, 400000000);
		clk_set_rate(clk2, 400000000);
		break;
	case 333333333:
		clk0 = clk_get(NULL, "sys_pll1_3");
		if (IS_ERR(clk0))
			return PTR_ERR(clk0);

		clk_set_parent(clk1, clk0);

		clk_set_rate(clk1, 333333333);
		clk_set_rate(clk2, 333333333);
		break;
	case 240000000:
		clk0 = clk_get(NULL, "usb_pll_2");
		if (IS_ERR(clk0))
			return PTR_ERR(clk0);
		clk_set_parent(clk1, clk0);

		clk_set_rate(clk1, 240000000);
		clk_set_rate(clk2, 240000000);
		break;
	default:
		clk0 = clk_get(NULL, "sys_pll0_3");
		if (IS_ERR(clk0))
			return PTR_ERR(clk0);

		clk_set_parent(clk1, clk0);

		clk_set_rate(clk1, 400000000);
		clk_set_rate(clk2, rate);
		break;
	}

	clk_put(clk0);
	clk_put(clk1);
	clk_put(clk2);

	return ret;
}

static unsigned long rlx_recalc_h264(struct clk_hw *hw,
	unsigned long parent_rate)
{
	struct clk *clk0;
	u32 rate;

	clk0 = clk_get(NULL, "h264_ck_dec");
	if (IS_ERR(clk0))
		return PTR_ERR(clk0);

	rate = clk_get_rate(clk0) - 1;

	clk_put(clk0);

	return rate;
}

static const struct clk_ops rlx_clk_h264_ops = {
	.is_enabled = rlx_clk_is_enabled,
	.round_rate = rlx_round_rate_v,
	.set_rate = rlx_set_rate_h264,
	.recalc_rate = rlx_recalc_h264,
	.enable = rlx_enable_clk,
	.disable = rlx_disable_clk,
};

static int rlx_set_rate_jpeg(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	struct clk *clk0, *clk1, *clk2;
	int ret = 0;

	clk1 = clk_get(NULL, "jpeg_ck_div");
	if (IS_ERR(clk1))
		return PTR_ERR(clk1);

	clk2 = clk_get(NULL, "jpeg_ck_dec");
	if (IS_ERR(clk2))
		return PTR_ERR(clk2);

	clk_set_parent(clk2, clk1);

	switch (rate) {
	case 120000000:
		clk0 = clk_get(NULL, "usb_2");
		if (IS_ERR(clk0))
			return PTR_ERR(clk0);

		clk_set_parent(clk1, clk0);

		clk_set_rate(clk1, 120000000);
		clk_set_rate(clk0, 120000000);
		break;
	case 100000000:
		clk0 = clk_get(NULL, "sys_pll1_5");
		if (IS_ERR(clk0))
			return PTR_ERR(clk0);

		clk_set_parent(clk1, clk0);

		clk_set_rate(clk1, 100000000);
		clk_set_rate(clk2, 100000000);
		break;
	default:
		pr_debug("%s %ld not supported yet\n",
			clk_hw_get_name(hw), rate);
		ret = -EINVAL;
	}

	clk_put(clk0);
	clk_put(clk1);
	clk_put(clk2);

	return ret;
}

static unsigned long rlx_recalc_jpeg(struct clk_hw *hw,
	unsigned long parent_rate)
{
	struct clk *clk0;
	u32 rate;

	if (clk_platform_type & TYPE_FPGA)
		return 30000000;

	clk0 = clk_get(NULL, "jpeg_ck_dec");
	if (IS_ERR(clk0))
		return -EINVAL;

	rate = clk_get_rate(clk0) - 1;

	clk_put(clk0);

	return rate;
}

static const struct clk_ops rlx_clk_jpeg_ops = {
	.is_enabled = rlx_clk_is_enabled,
	.round_rate = rlx_round_rate_v,
	.set_rate = rlx_set_rate_jpeg,
	.recalc_rate = rlx_recalc_jpeg,
	.enable = rlx_enable_clk,
	.disable = rlx_disable_clk,
};

static int rlx_set_rate_bus(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	struct clk *clk0, *clk1, *clk2;
	int ret = 0;

	clk1 = clk_get(NULL, "bus_ck_div");
	if (IS_ERR(clk1))
		return PTR_ERR(clk1);

	clk2 = clk_get(NULL, "bus_ck_dec");
	if (IS_ERR(clk2))
		return PTR_ERR(clk2);

	clk_set_parent(clk2, clk1);

	switch (rate) {
	case 60000000:
		clk0 = clk_get(NULL, "usb_pll_2");
		if (IS_ERR(clk0))
			return PTR_ERR(clk0);

		clk_set_parent(clk1, clk0);

		clk_set_rate(clk1, 60000000);
		clk_set_rate(clk2, 60000000);
		break;
	case 120000000:
		clk0 = clk_get(NULL, "sys_pll0_5");
		if (IS_ERR(clk0))
			return PTR_ERR(clk0);

		clk_set_parent(clk1, clk0);

		clk_set_rate(clk1, 120000000);
		clk_set_rate(clk2, 120000000);
		break;
	case 125000000:
		clk0 = clk_get(NULL, "sys_pll1_2");
		if (IS_ERR(clk0))
			return PTR_ERR(clk0);

		clk_set_parent(clk1, clk0);

		clk_set_rate(clk1, 125000000);
		clk_set_rate(clk2, 125000000);
		break;

	default:
		pr_debug("%s %ld not supported yet\n",
			clk_hw_get_name(hw), rate);
		ret = -EINVAL;
	}

	clk_put(clk0);
	clk_put(clk1);
	clk_put(clk2);

	return ret;
}

static unsigned long rlx_recalc_bus(struct clk_hw *hw,
	unsigned long parent_rate)
{
	struct clk *clk0;
	u32 rate;

	if (clk_platform_type & TYPE_FPGA)
		return 30000000;

	clk0 = clk_get(NULL, "bus_ck_dec");
	if (IS_ERR(clk0))
		return PTR_ERR(clk0);

	rate = clk_get_rate(clk0) - 1;

	clk_put(clk0);

	return rate;
}

static const struct clk_ops rlx_clk_bus_ops = {
	.is_enabled = rlx_clk_is_enabled,
	.round_rate = rlx_round_rate_v,
	.set_rate = rlx_set_rate_bus,
	.recalc_rate = rlx_recalc_bus,
	.enable = rlx_enable_clk,
	.disable = rlx_disable_clk,
};

static int rlx_set_rate_dram(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	struct clk *clk0, *clk1, *clk2;
	int ret = 0;

	clk1 = clk_get(NULL, "dram_ck_div");
	if (IS_ERR(clk1))
		return PTR_ERR(clk1);

	clk2 = clk_get(NULL, "dram_ck_dec");
	if (IS_ERR(clk2))
		return PTR_ERR(clk2);

	clk_set_parent(clk2, clk1);

	switch (rate) {
	case 240000000:
		clk0 = clk_get(NULL, "usb_pll_2");
		if (IS_ERR(clk0))
			return PTR_ERR(clk0);

		clk_set_parent(clk1, clk0);

		clk_set_rate(clk1, 240000000);
		clk_set_rate(clk2, 240000000);
		break;
	case 300000000:
		clk0 = clk_get(NULL, "sys_pll0_2");
		if (IS_ERR(clk0))
			return PTR_ERR(clk0);

		clk_set_parent(clk1, clk0);

		clk_set_rate(clk1, 300000000);
		clk_set_rate(clk2, 300000000);
		break;
	case 333333333:
		clk0 = clk_get(NULL, "sys_pll1_3");
		if (IS_ERR(clk0))
			return PTR_ERR(clk0);

		clk_set_parent(clk1, clk0);

		clk_set_rate(clk1, 333333333);
		clk_set_rate(clk2, 333333333);
		break;
	case 250000000:
		clk0 = clk_get(NULL, "sys_pll1_2");
		if (IS_ERR(clk0))
			return PTR_ERR(clk0);

		clk_set_parent(clk1, clk0);

		clk_set_rate(clk1, 250000000);
		clk_set_rate(clk2, 250000000);
		break;
	default:
		pr_debug("%s %ld not supported yet\n",
			clk_hw_get_name(hw), rate);
		ret = -EINVAL;
	}

	clk_put(clk0);
	clk_put(clk1);
	clk_put(clk2);

	return ret;
}

static unsigned long rlx_recalc_dram(struct clk_hw *hw,
	unsigned long parent_rate)
{
	struct clk *clk0;
	u32 rate;

	if (clk_platform_type & TYPE_FPGA)
		return 60000000;

	clk0 = clk_get(NULL, "dram_ck_dec");
	if (IS_ERR(clk0))
		return PTR_ERR(clk0);

	rate = clk_get_rate(clk0) - 1;

	clk_put(clk0);

	return rate;
}

static const struct clk_ops rlx_clk_dram_ops = {
	.is_enabled = rlx_clk_is_enabled,
	.round_rate = rlx_round_rate_v,
	.set_rate = rlx_set_rate_dram,
	.recalc_rate = rlx_recalc_dram,
	.enable = rlx_enable_clk,
	.disable = rlx_disable_clk,
};

static int rlx_set_rate_isp(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	struct clk *clk0, *clk1, *clk2;
	int ret = 0;

	clk1 = clk_get(NULL, "isp_ck_div");
	if (IS_ERR(clk1))
		return PTR_ERR(clk1);

	clk2 = clk_get(NULL, "isp_ck_dec");
	if (IS_ERR(clk2))
		return PTR_ERR(clk2);

	clk_set_parent(clk2, clk1);

	switch (rate) {
	case 120000000:
		clk0 = clk_get(NULL, "usb_pll_2");
		if (IS_ERR(clk0))
			return PTR_ERR(clk0);

		clk_set_parent(clk1, clk0);

		clk_set_rate(clk1, 120000000);
		clk_set_rate(clk2, 120000000);
		break;
	case 100000000:
		clk0 = clk_get(NULL, "sys_pll0_5");
		if (IS_ERR(clk0))
			return PTR_ERR(clk0);

		clk_set_parent(clk1, clk0);

		clk_set_rate(clk1, 100000000);
		clk_set_rate(clk2, 100000000);
		break;
	default:
		pr_debug("%s %ld not supported yet\n",
			clk_hw_get_name(hw), rate);
		ret = -EINVAL;
	}

	clk_put(clk0);
	clk_put(clk1);
	clk_put(clk2);

	return ret;
}

static unsigned long rlx_recalc_isp(struct clk_hw *hw,
	unsigned long parent_rate)
{
	struct clk *clk0;
	u32 rate;

	if (clk_platform_type & TYPE_FPGA)
		return 60000000;

	clk0 = clk_get(NULL, "isp_ck_dec");
	if (IS_ERR(clk0))
		return PTR_ERR(clk0);

	rate = clk_get_rate(clk0) - 1;

	clk_put(clk0);

	return rate;
}

static const struct clk_ops rlx_clk_isp_ops = {
	.is_enabled = rlx_clk_is_enabled,
	.round_rate = rlx_round_rate_v,
	.set_rate = rlx_set_rate_isp,
	.recalc_rate = rlx_recalc_isp,
	.enable = rlx_enable_clk,
	.disable = rlx_disable_clk,
};

static int rlx_set_rate_mipi(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	struct clk *clk0, *clk1, *clk2;
	int ret = 0;

	clk1 = clk_get(NULL, "mipi_ck_div");
	if (IS_ERR(clk1))
		return PTR_ERR(clk1);

	clk2 = clk_get(NULL, "mipi_ck_dec");
	if (IS_ERR(clk2))
		return PTR_ERR(clk2);

	clk_set_parent(clk2, clk1);

	switch (rate) {
	case 240000000:
		clk0 = clk_get(NULL, "usb_pll_2");
		if (IS_ERR(clk0))
			return PTR_ERR(clk0);

		clk_set_parent(clk1, clk0);

		clk_set_rate(clk1, 240000000);
		clk_set_rate(clk2, 240000000);
		break;
	case 250000000:
		clk0 = clk_get(NULL, "sys_pll1_2");
		if (IS_ERR(clk0))
			return PTR_ERR(clk0);

		clk_set_parent(clk1, clk0);

		clk_set_rate(clk1, 250000000);
		clk_set_rate(clk2, 250000000);
		break;
	default:
		pr_debug("%s %ld not supported yet\n", clk_hw_get_name(hw),
			rate);
		ret = -EINVAL;
	}

	clk_put(clk0);
	clk_put(clk1);
	clk_put(clk2);

	return ret;
}

static unsigned long rlx_recalc_mipi(struct clk_hw *hw,
	unsigned long parent_rate)
{
	struct clk *clk0;
	u32 rate;

	if (clk_platform_type & TYPE_FPGA)
		return 60000000;

	clk0 = clk_get(NULL, "mipi_ck_dec");
	if (IS_ERR(clk0))
		return PTR_ERR(clk0);

	rate = clk_get_rate(clk0) - 1;

	clk_put(clk0);

	return rate;
}

static const struct clk_ops rlx_clk_mipi_ops = {
	.is_enabled = rlx_clk_is_enabled,
	.round_rate = rlx_round_rate_v,
	.set_rate = rlx_set_rate_mipi,
	.recalc_rate = rlx_recalc_mipi,
	.enable = rlx_enable_clk,
	.disable = rlx_disable_clk,
};

static int rlx_set_rate_i2c(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	struct clk *clk0, *clk1;
	int ret = 0;

	clk1 = clk_get(NULL, "i2c_ck_div");
	if (IS_ERR(clk1))
		return PTR_ERR(clk1);

	switch (rate) {
	case 40000000:
		clk0 = clk_get(NULL, "usb_pll_3");
		if (IS_ERR(clk0))
			return PTR_ERR(clk0);

		clk_set_parent(clk1, clk0);

		clk_set_rate(clk1, 40000000);
		break;
	case 75000000:
		clk0 = clk_get(NULL, "sys_pll0_2");
		if (IS_ERR(clk0))
			return PTR_ERR(clk0);

		clk_set_parent(clk1, clk0);

		clk_set_rate(clk1, 75000000);
		break;
	case 83333333:
		clk0 = clk_get(NULL, "sys_pll1_3");
		if (IS_ERR(clk0))
			return PTR_ERR(clk0);

		clk_set_parent(clk1, clk0);

		clk_set_rate(clk1, 83333333);
		break;
	default:
		pr_debug("%s %ld not supported yet\n",
			clk_hw_get_name(hw), rate);
		ret = -EINVAL;
	}

	clk_put(clk0);
	clk_put(clk1);

	return ret;
}

static unsigned long rlx_recalc_i2c(struct clk_hw *hw,
	unsigned long parent_rate)
{
	struct clk *clk0;
	u32 rate;

	if (clk_platform_type & TYPE_FPGA)
		return 50000000;

	clk0 = clk_get(NULL, "i2c_ck_div");
	if (IS_ERR(clk0))
		return PTR_ERR(clk0);

	rate = clk_get_rate(clk0) - 1;

	clk_put(clk0);

	return rate;
}

static const struct clk_ops rlx_clk_i2c_ops = {
	.is_enabled = rlx_clk_is_enabled,
	.round_rate = rlx_round_rate_v,
	.set_rate = rlx_set_rate_i2c,
	.recalc_rate = rlx_recalc_i2c,
	.enable = rlx_enable_clk,
	.disable = rlx_disable_clk,
};

static int rlx_set_rate_xb2(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	struct clk *clk0, *clk1;
	int ret = 0;

	clk1 = clk_get(NULL, "i2c_ck_div");
	if (IS_ERR(clk1))
		return PTR_ERR(clk1);

	switch (rate) {
	case 60000000:
		clk0 = clk_get(NULL, "usb_pll_2");
		if (IS_ERR(clk0))
			return PTR_ERR(clk0);

		clk_set_parent(clk1, clk0);

		clk_set_rate(clk1, 60000000);
		break;
	case 30000000:
		clk0 = clk_get(NULL, "usb_pll_2");
		if (IS_ERR(clk0))
			return PTR_ERR(clk0);

		clk_set_parent(clk1, clk0);

		clk_set_rate(clk1, 30000000);
		break;
	case 62500000:
		clk0 = clk_get(NULL, "sys_pll1_2");
		if (IS_ERR(clk0))
			return PTR_ERR(clk0);

		clk_set_parent(clk1, clk0);

		clk_set_rate(clk1, 62500000);
		break;
	default:
		pr_debug("%s %ld not supported yet\n",
			clk_hw_get_name(hw), rate);
		ret = -EINVAL;
	}

	clk_put(clk0);
	clk_put(clk1);

	return ret;
}

static unsigned long rlx_recalc_xb2(struct clk_hw *hw,
	unsigned long parent_rate)
{
	struct clk *clk0;
	u32 rate;

	if (clk_platform_type & TYPE_FPGA)
		return 30000000;

	clk0 = clk_get(NULL, "xb2_ck_div");
	if (IS_ERR(clk0))
		return PTR_ERR(clk0);

	rate = clk_get_rate(clk0) - 1;

	clk_put(clk0);

	return rate;
}

static const struct clk_ops rlx_clk_xb2_ops = {
	.is_enabled = rlx_clk_is_enabled,
	.round_rate = rlx_round_rate_v,
	.set_rate = rlx_set_rate_xb2,
	.recalc_rate = rlx_recalc_xb2,
	.enable = rlx_enable_clk,
	.disable = rlx_disable_clk,
};

static int rlx_set_rate_uart(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	struct clk *clk0, *clk1;
	int ret = 0;

	clk1 = clk_get(NULL, "uart_ck_div");
	if (IS_ERR(clk1))
		return PTR_ERR(clk1);

	switch (rate) {
	case 24000000:
		clk0 = clk_get(NULL, "usb_pll_5");
		if (IS_ERR(clk0))
			return PTR_ERR(clk0);

		clk_set_parent(clk1, clk0);
		clk_set_rate(clk1, 24000000);
		break;
	case 25000000:
		clk0 = clk_get(NULL, "sys_pll1_5");
		if (IS_ERR(clk0))
			return PTR_ERR(clk0);

		clk_set_parent(clk1, clk0);
		clk_set_rate(clk1, 25000000);
		break;
	default:
		pr_debug("%s %ld not supported yet\n",
			clk_hw_get_name(hw), rate);
		ret = -EINVAL;
	}

	clk_put(clk0);
	clk_put(clk1);

	return ret;
}

static unsigned long rlx_recalc_uart(struct clk_hw *hw,
	unsigned long parent_rate)
{
	struct clk *clk0;
	u32 rate;

	if (clk_platform_type & TYPE_FPGA)
		return 24000000;

	clk0 = clk_get(NULL, "uart_ck_div");
	if (IS_ERR(clk0))
		return PTR_ERR(clk0);

	rate = clk_get_rate(clk0) - 1;

	clk_put(clk0);

	return rate;
}

static const struct clk_ops rlx_clk_uart_ops = {
	.is_enabled = rlx_clk_is_enabled,
	.round_rate = rlx_round_rate_v,
	.set_rate = rlx_set_rate_uart,
	.recalc_rate = rlx_recalc_uart,
	.enable = rlx_enable_clk,
	.disable = rlx_disable_clk,
};

static int rlx_set_rate_i2s(struct clk_hw *hw, unsigned long rate,
			unsigned long parent_rate)
{
	struct clk *clk0, *clk1, *clk2;
	int ret = 0;
	u32 n, t;

	n = 900000000 / rate;
	n = n + (4 - n % 4);
	t = rate * n;

	clk0 = clk_get(NULL, "sys_pll2");
	if (IS_ERR(clk0))
		return PTR_ERR(clk0);
	clk_set_rate(clk0, t);
	clk_prepare_enable(clk0);
	clk_put(clk0);

	clk1 = clk_get(NULL, "sys_pll2_2");
	if (IS_ERR(clk1))
		return PTR_ERR(clk1);

	clk2 = clk_get(NULL, "i2s_ck_div");
	if (IS_ERR(clk2))
		return PTR_ERR(clk2);

	clk_set_parent(clk2, clk1);
	clk_set_rate(clk2, rate);
	clk_put(clk1);
	clk_put(clk2);

	return ret;
}

static unsigned long rlx_recalc_i2s(struct clk_hw *hw,
	unsigned long parent_rate)
{
	struct clk *clk0;
	u32 rate;

	clk0 = clk_get(NULL, "i2s_ck_div");
	if (IS_ERR(clk0))
		return PTR_ERR(clk0);

	rate = clk_get_rate(clk0) - 1;

	clk_put(clk0);

	return rate;
}

static const struct clk_ops rlx_clk_i2s_ops = {
	.is_enabled = rlx_clk_is_enabled,
	.round_rate = rlx_round_rate_v,
	.set_rate = rlx_set_rate_i2s,
	.recalc_rate = rlx_recalc_i2s,
	.enable = rlx_enable_clk,
	.disable = rlx_disable_clk,
};

static int rlx_set_rate_spdif(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	struct clk *clk0, *clk1, *clk2;
	int ret = 0;
	u32 n, t;

	n = 900000000 / rate;
	n = n + (4 - n % 4);
	t = rate * n;

	clk0 = clk_get(NULL, "sys_pll2");
	if (IS_ERR(clk0))
		return PTR_ERR(clk0);
	clk_set_rate(clk0, t);
	clk_prepare_enable(clk0);
	clk_put(clk0);

	clk1 = clk_get(NULL, "sys_pll2_2");
	if (IS_ERR(clk1))
		return PTR_ERR(clk1);

	clk2 = clk_get(NULL, "spdif_ck_div");
	if (IS_ERR(clk2))
		return PTR_ERR(clk2);

	clk_set_parent(clk2, clk1);
	clk_set_rate(clk2, rate);
	clk_put(clk1);
	clk_put(clk2);

	return ret;
}

static unsigned long rlx_recalc_spdif(struct clk_hw *hw,
	unsigned long parent_rate)
{
	struct clk *clk0;
	u32 rate;

	clk0 = clk_get(NULL, "spdif_ck_div");
	if (IS_ERR(clk0))
		return PTR_ERR(clk0);

	rate = clk_get_rate(clk0) - 1;

	clk_put(clk0);

	return rate;
}

static const struct clk_ops rlx_clk_spdif_ops = {
	.is_enabled = rlx_clk_is_enabled,
	.round_rate = rlx_round_rate_v,
	.set_rate = rlx_set_rate_spdif,
	.recalc_rate = rlx_recalc_spdif,
	.enable = rlx_enable_clk,
	.disable = rlx_disable_clk,
};

static int rlx_set_rate_codec(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	struct clk *clk0, *clk1;
	int ret = 0;

	clk1 = clk_get(NULL, "codec_ck_div");
	if (IS_ERR(clk1))
		return PTR_ERR(clk1);

	switch (rate) {
	case 96000000:
		clk0 = clk_get(NULL, "usb_pll_5");
		if (IS_ERR(clk0))
			return PTR_ERR(clk0);

		clk_set_parent(clk1, clk0);
		clk_set_rate(clk1, 96000000);
		break;
	default:
		pr_debug("%s %ld not supported yet\n",
			clk_hw_get_name(hw), rate);
		ret = -EINVAL;
	}

	clk_put(clk0);
	clk_put(clk1);

	return ret;
}

static unsigned long rlx_recalc_codec(struct clk_hw *hw,
	unsigned long parent_rate)
{
	struct clk *clk0;
	u32 rate;

	clk0 = clk_get(NULL, "codec_ck_div");
	if (IS_ERR(clk0))
		return PTR_ERR(clk0);

	rate = clk_get_rate(clk0) - 1;

	clk_put(clk0);

	return rate;
}

static const struct clk_ops rlx_clk_codec_ops = {
	.is_enabled = rlx_clk_is_enabled,
	.round_rate = rlx_round_rate_v,
	.set_rate = rlx_set_rate_codec,
	.recalc_rate = rlx_recalc_codec,
	.enable = rlx_enable_clk,
	.disable = rlx_disable_clk,
};

static int rlx_eth_prepare_clk(struct clk_hw *hw)
{
	struct clk_rlx *clk = to_clk_rlx(hw);
	u32 reg;

	reg = rts_clk_readl(clk->clkreg);
	reg &= ~ETH_EPHY_RST_N;
	rts_clk_writel(reg, clk->clkreg);
	xb2flush();

	return 0;
}

static void rlx_eth_unprepare_clk(struct clk_hw *hw)
{
}

static int rlx_eth_enable_clk(struct clk_hw *hw)
{
	struct clk_rlx *clk = to_clk_rlx(hw);
	u32 reg;

	reg = rts_clk_readl(clk->clkreg);
	reg |= AFE_POW_STATE | CLK_EN_ETN_250M | ETH_EPHY_RST_N | ETH_EPHY_ADDR;
	rts_clk_writel(reg, clk->clkreg);
	xb2flush();

	return 0;
}

static void rlx_eth_disable_clk(struct clk_hw *hw)
{
}

static const struct clk_ops rlx_eth_clk_ops = {
	.prepare = rlx_eth_prepare_clk,
	.unprepare = rlx_eth_unprepare_clk,
	.enable = rlx_eth_enable_clk,
	.disable = rlx_eth_disable_clk,
};

static int rlx_dma_enable_clk(struct clk_hw *hw)
{
	struct clk_rlx *clk = to_clk_rlx(hw);
	u32 reg;

	reg = rts_clk_readl(clk->clkreg) & ~0x3;
	rts_clk_writel(reg, clk->clkreg);
	xb2flush();

	return 0;
}

static void rlx_dma_disable_clk(struct clk_hw *hw)
{
	struct clk_rlx *clk = to_clk_rlx(hw);
	u32 reg;

	reg = rts_clk_readl(clk->clkreg) & ~0x3;
	reg |= 0x2;
	rts_clk_writel(reg, clk->clkreg);
	xb2flush();
}

static const struct clk_ops rlx_dma_clk_ops = {
	.enable = rlx_dma_enable_clk,
	.disable = rlx_dma_disable_clk,
};

static int usbphy_enable_clk(struct clk_hw *hw)
{
	struct clk_rlx *clk = to_clk_rlx(hw);
	u32 reg;

	reg = rts_clk_readl(clk->clkreg);
	if (!strcmp(clk_hw_get_name(hw), "usbphy_host_ck"))
		reg |= USBPHY_HOST_CLK_EN;
	else if (!strcmp(clk_hw_get_name(hw), "usbphy_dev_ck"))
		reg |= USBPHY_DEV_CLK_EN;
	rts_clk_writel(reg, clk->clkreg);
	xb2flush();

	return 0;
}

static void usbphy_disable_clk(struct clk_hw *hw)
{
	struct clk_rlx *clk = to_clk_rlx(hw);
	u32 reg;

	reg = rts_clk_readl(clk->clkreg);
	if (!strcmp(clk_hw_get_name(hw), "usbphy_host_ck"))
		reg &= ~USBPHY_HOST_CLK_EN;
	else if (!strcmp(clk_hw_get_name(hw), "usbphy_dev_ck"))
		reg &= ~USBPHY_DEV_CLK_EN;
	rts_clk_writel(reg, clk->clkreg);
	xb2flush();
}

static const struct clk_ops usbphy_divider_ops = {
	.enable = usbphy_enable_clk,
	.disable = usbphy_disable_clk,
};

struct clk *rlx_register_clk(struct clk_rlx *rlxclk, int flags)
{
	struct clk *clk;
	struct clk_init_data init;

	init.name = rlxclk->name;
	init.ops = rlxclk->ops;
	init.flags = flags;
	init.parent_names = rlxclk->parent_names;
	init.num_parents = rlxclk->num_parents;

	rlxclk->hw.init = &init;

	clk = clk_register(NULL, &rlxclk->hw);

	clk_register_clkdev(clk, rlxclk->name, NULL);

	return clk;
}

struct clk *rlx_register_fixed_rate(struct device *dev, const char *name,
		const char *parent_name, unsigned long flags,
		unsigned long fixed_rate)
{
	struct clk *clk;

	clk = clk_register_fixed_rate(dev, name,
		parent_name, flags, fixed_rate);

	clk_register_clkdev(clk, name, NULL);

	return clk;
}

struct clk *rlx_register_fixed_factor(struct device *dev, const char *name,
		const char *parent_name, unsigned long flags,
		unsigned int mult, unsigned int div)
{
	struct clk *clk;

	clk = clk_register_fixed_factor(dev, name,
		parent_name, flags, mult, div);

	clk_register_clkdev(clk, name, NULL);

	return clk;
}

static unsigned long rlx_recalc_h265_aclk(struct clk_hw *hw,
	unsigned long parent_rate)
{
	struct clk *clk0;
	u32 rate;

	clk0 = clk_get(NULL, "dram_ck");
	if (IS_ERR(clk0))
		return PTR_ERR(clk0);

	rate = clk_get_rate(clk0) / 2;

	clk_put(clk0);

	return rate;
}

static const struct clk_ops rlx_clk_h265_aclk_ops = {
	.recalc_rate = rlx_recalc_h265_aclk,
	.enable = rlx_enable_clk,
	.disable = rlx_disable_clk,
};

static int rlx_set_rate_h265_bclk(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	struct clk *clk0, *clk1, *clk2;
	int ret = 0;

	clk1 = clk_get(NULL, "h265_bclk_ck_div");
	if (IS_ERR(clk1))
		return PTR_ERR(clk1);

	clk2 = clk_get(NULL, "h265_bclk_ck_dec");
	if (IS_ERR(clk2))
		return PTR_ERR(clk2);

	clk_set_parent(clk2, clk1);

	switch (rate) {
	case 160000000:
		clk0 = clk_get(NULL, "usb_pll_3");
		if (IS_ERR(clk0))
			return PTR_ERR(clk0);

		clk_set_parent(clk1, clk0);

		clk_set_rate(clk1, 160000000);
		clk_set_rate(clk2, 160000000);
		break;
	default:
		pr_debug("%s %ld not supported yet\n",
			clk_hw_get_name(hw), rate);
		ret = -EINVAL;
	}

	clk_put(clk0);
	clk_put(clk1);
	clk_put(clk2);

	return ret;
}

static unsigned long rlx_recalc_h265_bclk(struct clk_hw *hw,
	unsigned long parent_rate)
{
	struct clk *clk0;
	u32 rate;

	if (clk_platform_type & TYPE_FPGA)
		return 50000000;

	clk0 = clk_get(NULL, "h265_bclk_ck_dec");
	if (IS_ERR(clk0))
		return PTR_ERR(clk0);

	rate = clk_get_rate(clk0) - 1;

	clk_put(clk0);

	return rate;
}

static const struct clk_ops rlx_clk_h265_bclk_ops = {
	.round_rate = rlx_round_rate_v,
	.set_rate = rlx_set_rate_h265_bclk,
	.recalc_rate = rlx_recalc_h265_bclk,
	.enable = rlx_enable_clk,
	.disable = rlx_disable_clk,
};


static int rlx_set_rate_h265_cclk(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	struct clk *clk0, *clk1, *clk2;
	int ret = 0;

	clk1 = clk_get(NULL, "h265_cclk_ck_div");
	if (IS_ERR(clk1))
		return PTR_ERR(clk1);

	clk2 = clk_get(NULL, "h265_cclk_ck_dec");
	if (IS_ERR(clk2))
		return PTR_ERR(clk2);

	clk_set_parent(clk2, clk1);

	switch (rate) {
	case 160000000:
		clk0 = clk_get(NULL, "usb_pll_3");
		if (IS_ERR(clk0))
			return PTR_ERR(clk0);

		clk_set_parent(clk1, clk0);

		clk_set_rate(clk1, 160000000);
		clk_set_rate(clk2, 160000000);
		break;
	default:
		pr_debug("%s %ld not supported yet\n",
			clk_hw_get_name(hw), rate);
		ret = -EINVAL;
	}

	clk_put(clk0);
	clk_put(clk1);
	clk_put(clk2);

	return ret;
}

static unsigned long rlx_recalc_h265_cclk(struct clk_hw *hw,
	unsigned long parent_rate)
{
	struct clk *clk0;
	u32 rate;

	if (clk_platform_type & TYPE_FPGA)
		return 50000000;

	clk0 = clk_get(NULL, "h265_cclk_ck_dec");
	if (IS_ERR(clk0))
		return PTR_ERR(clk0);

	rate = clk_get_rate(clk0) - 1;

	clk_put(clk0);

	return rate;
}

static const struct clk_ops rlx_clk_h265_cclk_ops = {
	.round_rate = rlx_round_rate_v,
	.set_rate = rlx_set_rate_h265_cclk,
	.recalc_rate = rlx_recalc_h265_cclk,
	.enable = rlx_enable_clk,
	.disable = rlx_disable_clk,
};

static int rlx_set_rate_h265_clk(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	int ret = 0;

	if (rate != 160000000) {
		pr_debug("%s %ld not supported yet\n",
			clk_hw_get_name(hw), rate);
		ret = -EINVAL;
	}

	return ret;
}

static unsigned long rlx_recalc_h265_clk(struct clk_hw *hw,
	unsigned long parent_rate)
{
	return 160000000;
}

static int rlx_enable_h265_clk(struct clk_hw *hw)
{
	u32 reg;
	void __iomem *addr;

	addr = (void __iomem *)H265_ACLK_CFG_REG;
	reg = rts_clk_readl(addr);
	reg |= CLK_ENABLE;
	rts_clk_writel(reg, addr);
	udelay(1);

	addr = (void __iomem *)H265_BCLK_CFG_REG;
	reg = rts_clk_readl(addr);
	reg |= CLK_ENABLE;
	rts_clk_writel(reg, addr);
	udelay(1);

	addr = (void __iomem *)H265_CCLK_CFG_REG;
	reg = rts_clk_readl(addr);
	reg |= CLK_ENABLE;
	rts_clk_writel(reg, addr);
	udelay(1);

	xb2flush();
	return 0;
}

static void rlx_disable_h265_clk(struct clk_hw *hw)
{
	void __iomem *addr;
	u32 reg;

	addr = (void __iomem *)H265_ACLK_CFG_REG;
	reg = rts_clk_readl(addr);
	reg &= ~CLK_ENABLE;
	rts_clk_writel(reg, addr);

	addr = (void __iomem *)H265_BCLK_CFG_REG;
	reg = rts_clk_readl(addr);
	reg &= ~CLK_ENABLE;
	rts_clk_writel(reg, addr);

	addr = (void __iomem *)H265_CCLK_CFG_REG;
	reg = rts_clk_readl(addr);
	reg &= ~CLK_ENABLE;
	rts_clk_writel(reg, addr);
	xb2flush();
}

static const struct clk_ops rlx_clk_h265_clk_ops = {
	.round_rate = rlx_round_rate_v,
	.set_rate = rlx_set_rate_h265_clk,
	.recalc_rate = rlx_recalc_h265_clk,
	.enable = rlx_enable_h265_clk,
	.disable = rlx_disable_h265_clk,
};


DEFINE_CLK_RLX(sys_pll0, rlx_root_parent_names,
	rlx_pll_ops, BSP_CLK_PLL0_BASE_R, 0);
DEFINE_CLK_RLX(sys_pll1, rlx_root_parent_names,
	rlx_pll_ops, BSP_CLK_PLL1_BASE_R, 0);
DEFINE_CLK_RLX(sys_pll2, rlx_root_parent_names,
	rlx_pll_ops, BSP_CLK_PLL2_BASE_R, 0);
DEFINE_CLK_RLX(sys_pll3, rlx_root_parent_names,
	rlx_pll_ops, BSP_CLK_PLL3_BASE_R, 0);
DEFINE_CLK_RLX(dma_ck, rlx_root_parent_names,
	rlx_dma_clk_ops, PLATFORM_CONFIGURATION_R, CLK_CHANGE_NULL);
DEFINE_CLK_RLX(usbphy_host_ck, rlx_root_parent_names,
	usbphy_divider_ops, USBPHY_CLK_CFG_R, CLK_CHANGE_NULL);
DEFINE_CLK_RLX(usbphy_dev_ck, rlx_root_parent_names,
	usbphy_divider_ops, USBPHY_CLK_CFG_R, CLK_CHANGE_NULL);
DEFINE_CLK_RLX(ethernet_ck, rlx_root_parent_names,
	rlx_eth_clk_ops, ETHERNET_CLK_CFG_R, CLK_CHANGE_NULL);

DEFINE_CLK_RLX(cpu_ck_div, rlx_names_cpu_div,
	rlx_divider_ops, CPU_CLK_CFG_R, CPU_CLK_CHANGE);
DEFINE_CLK_RLX(cpu_ck_dec, rlx_names_cpu_dec,
	rlx_decdivider_ops, CPU_CLK_CFG_R, CPU_CLK_CHANGE);
DEFINE_CLK_RLX(cpu_ck, rlx_names_v,
	rlx_clk_cpu_ops, CPU_CLK_CFG_R, CPU_CLK_CHANGE);

DEFINE_CLK_RLX(h264_ck_div, rlx_names_h264_div,
	rlx_divider_ops, H264_CLK_CFG_R, CLK_CHANGE_NULL);
DEFINE_CLK_RLX(h264_ck_dec, rlx_names_h264_dec,
	rlx_decdivider_ops, H264_CLK_CFG_R, CLK_CHANGE_NULL);
DEFINE_CLK_RLX(h264_ck, rlx_names_v,
	rlx_clk_h264_ops, H264_CLK_CFG_R, CLK_CHANGE_NULL);

DEFINE_CLK_RLX(jpeg_ck_div, rlx_names_jpeg_div,
	rlx_divider_ops, JPEG_CLK_CFG_R, CLK_CHANGE_NULL);
DEFINE_CLK_RLX(jpeg_ck_dec, rlx_names_jpeg_dec,
	rlx_decdivider_ops, JPEG_CLK_CFG_R, CLK_CHANGE_NULL);
DEFINE_CLK_RLX(jpeg_ck, rlx_names_v,
	rlx_clk_jpeg_ops, JPEG_CLK_CFG_R, CLK_CHANGE_NULL);

DEFINE_CLK_RLX(bus_ck_div,	rlx_names_bus_div,
	rlx_divider_ops, BUS_CLK_CFG_R, BUS_CLK_CHANGE);
DEFINE_CLK_RLX(bus_ck_dec, rlx_names_bus_dec,
	rlx_decdivider_ops, BUS_CLK_CFG_R, BUS_CLK_CHANGE);
DEFINE_CLK_RLX(bus_ck, rlx_names_v,
	rlx_clk_bus_ops, BUS_CLK_CFG_R, BUS_CLK_CHANGE);

DEFINE_CLK_RLX(dram_ck_div, rlx_names_dram_div,
	rlx_divider_ops, DRAM_CLK_CFG_R, DRAM_CLK_CHANGE);
DEFINE_CLK_RLX(dram_ck_dec, rlx_names_dram_dec,
	rlx_decdivider_ops, DRAM_CLK_CFG_R, DRAM_CLK_CHANGE);
DEFINE_CLK_RLX(dram_ck, rlx_names_v,
	rlx_clk_dram_ops, DRAM_CLK_CFG_R, DRAM_CLK_CHANGE);

DEFINE_CLK_RLX(isp_ck_div, rlx_names_isp_div,
	rlx_divider_ops, ISP_SCAN_CLK_CFG_R, CLK_CHANGE_NULL);
DEFINE_CLK_RLX(isp_ck_dec, rlx_names_isp_dec,
	rlx_decdivider_ops, ISP_SCAN_CLK_CFG_R, CLK_CHANGE_NULL);
DEFINE_CLK_RLX(isp_ck, rlx_names_v,
	rlx_clk_isp_ops, ISP_SCAN_CLK_CFG_R, CLK_CHANGE_NULL);

DEFINE_CLK_RLX(mipi_ck_div, rlx_names_mipi_div,
	rlx_divider_ops, MIPI_SCAN_CLK_CFG_R, CLK_CHANGE_NULL);
DEFINE_CLK_RLX(mipi_ck_dec, rlx_names_mipi_dec,
	rlx_decdivider_ops, MIPI_SCAN_CLK_CFG_R, CLK_CHANGE_NULL);
DEFINE_CLK_RLX(mipi_ck, rlx_names_v,
	rlx_clk_mipi_ops, MIPI_SCAN_CLK_CFG_R, CLK_CHANGE_NULL);

DEFINE_CLK_RLX(i2c_ck_div, rlx_names_i2c_div,
	rlx_divider_ops_s, I2C_CLK_CFG_R, CLK_CHANGE_NULL);
DEFINE_CLK_RLX(i2c_ck, rlx_names_v,
	rlx_clk_i2c_ops, I2C_CLK_CFG_R, CLK_CHANGE_NULL);

DEFINE_CLK_RLX(xb2_ck_div, rlx_names_xb2_div,
	rlx_divider_ops_s, XB2_CLK_CFG_R, XB2_CLK_CHANGE);
DEFINE_CLK_RLX(xb2_ck, rlx_names_v,
	rlx_clk_xb2_ops, XB2_CLK_CFG_R, XB2_CLK_CHANGE);

DEFINE_CLK_RLX(uart_ck_div, rlx_names_uart_div,
	rlx_divider_ops_s, UART_CLK_CFG_R, CLK_CHANGE_NULL);
DEFINE_CLK_RLX(uart_ck, rlx_names_v,
	rlx_clk_uart_ops, UART_CLK_CFG_R, CLK_CHANGE_NULL);

DEFINE_CLK_RLX(i2s_ck_div, rlx_names_i2s_div,
	rlx_divider_ops_c, I2S_CLK_CFG_R, CLK_CHANGE_NULL);
DEFINE_CLK_RLX(i2s_ck, rlx_names_v,
	rlx_clk_i2s_ops, I2S_CLK_CFG_R, CLK_CHANGE_NULL);

DEFINE_CLK_RLX(spdif_ck_div, rlx_names_spdif_div,
	rlx_divider_ops_c, SPDIF_CLK_CFG_R, CLK_CHANGE_NULL);
DEFINE_CLK_RLX(spdif_ck, rlx_names_v,
	rlx_clk_spdif_ops, SPDIF_CLK_CFG_R, CLK_CHANGE_NULL);

DEFINE_CLK_RLX(codec_ck_div, rlx_names_codec_div,
	rlx_divider_ops_c, CODEC_CLK_CFG_R, CLK_CHANGE_NULL);
DEFINE_CLK_RLX(codec_ck, rlx_names_v,
	rlx_clk_codec_ops, CODEC_CLK_CFG_R, CLK_CHANGE_NULL);

DEFINE_CLK_RLX(h265_aclk_ck, rlx_names_h265_aclk,
	rlx_clk_h265_aclk_ops, H265_ACLK_CFG_REG, CLK_CHANGE_NULL);

DEFINE_CLK_RLX(h265_bclk_ck_div, rlx_names_h265_bclk_div,
	rlx_divider_ops, H265_BCLK_CFG_REG, CLK_CHANGE_NULL);
DEFINE_CLK_RLX(h265_bclk_ck_dec, rlx_names_h265_bclk_dec,
	rlx_decdivider_ops, H265_BCLK_CFG_REG, CLK_CHANGE_NULL);
DEFINE_CLK_RLX(h265_bclk_ck, rlx_names_v,
	rlx_clk_h265_bclk_ops, H265_BCLK_CFG_REG, CLK_CHANGE_NULL);

DEFINE_CLK_RLX(h265_cclk_ck_div, rlx_names_h265_cclk_div,
	rlx_divider_ops, H265_CCLK_CFG_REG, CLK_CHANGE_NULL);
DEFINE_CLK_RLX(h265_cclk_ck_dec, rlx_names_h265_cclk_dec,
	rlx_decdivider_ops, H265_CCLK_CFG_REG, CLK_CHANGE_NULL);
DEFINE_CLK_RLX(h265_cclk_ck, rlx_names_v,
	rlx_clk_h265_cclk_ops, H265_CCLK_CFG_REG, CLK_CHANGE_NULL);

DEFINE_CLK_RLX(h265_ck, rlx_names_v,
	rlx_clk_h265_clk_ops, H265_ACLK_CFG_REG, CLK_CHANGE_NULL);

void rlx_check_clocks(struct clk *clks[], unsigned int count)
{
	unsigned int i;

	for (i = 0; i < count; i++)
		if (IS_ERR(clks[i]))
			pr_err("rlx clk %u: register failed with %ld\n",
			       i, PTR_ERR(clks[i]));
}

static struct clk * __init rlx_obtain_fixed_clock_from_dt(const char *name)
{
	struct of_phandle_args phandle;
	struct clk *clk = ERR_PTR(-ENODEV);
	char *path;

	path = kasprintf(GFP_KERNEL, "/clocks/%s", name);
	if (!path)
		return ERR_PTR(-ENOMEM);

	phandle.np = of_find_node_by_path(path);
	kfree(path);

	if (phandle.np) {
		clk = of_clk_get_from_provider(&phandle);
		of_node_put(phandle.np);
	}
	return clk;
}

struct clk *rlx_obtain_fixed_clock(
			const char *name, unsigned long rate)
{
	struct clk *clk;

	clk = rlx_obtain_fixed_clock_from_dt(name);

	if (IS_ERR(clk))
		clk = clk_register_fixed_rate(NULL, name, NULL, 0, rate);

	return clk;
}

static void rlx_clock_hw_init(void)
{
	u32 reg;

	/* Disable usbphy */
	reg = rts_clk_readl((void __iomem *)USBPHY_CLK_CFG_R);
	reg &= ~(USBPHY_HOST_CLK_EN | USBPHY_DEV_CLK_EN);
	rts_clk_writel(reg, (void __iomem *)USBPHY_CLK_CFG_R);

	/* Disable ephy */
	reg = rts_clk_readl((void __iomem *)ETHERNET_CLK_CFG_R);
	reg &= ~(AFE_POW_STATE | CLK_EN_ETN_250M);
	rts_clk_writel(reg, (void __iomem *)ETHERNET_CLK_CFG_R);
}

static void rlx_clocks_init(struct device_node *node)
{
	int i;
#ifdef CONFIG_SOC_ENABLE_PLL0
	struct clk *pll0_clk;
#endif
	int clksize;

	rlx_clock_hw_init();

	for (i = (u32)UART_CLK_LP_EN_R; i <= (u32)VIDEO_CLK_SEL_R; i += 4)
		clk_reg_v[(i&0xff) >> 2] = rts_clk_readl((void __iomem *)i);

	clks[RLX_CLK_DUMMY] = rlx_register_fixed_rate(NULL,
		"dummy", NULL, 0, 100000000);
	clks[RLX_CLK_SYS_OSC] = rlx_obtain_fixed_clock("oscillator", 25000000);
	clks[RLX_CLK_USB_PLL] = rlx_obtain_fixed_clock("usb_pll", 480000000);
	clks[RLX_CLK_USB_PLL_2] = rlx_register_fixed_factor(NULL,
		"usb_pll_2", "usb_pll", CLK_SET_RATE_PARENT, 1, 2);
	clks[RLX_CLK_USB_PLL_3] = rlx_register_fixed_factor(NULL,
		"usb_pll_3", "usb_pll", CLK_SET_RATE_PARENT, 1, 3);
	clks[RLX_CLK_USB_PLL_5] = rlx_register_fixed_factor(NULL,
		"usb_pll_5", "usb_pll", CLK_SET_RATE_PARENT, 1, 5);
	clks[RLX_CLK_USB_PLL_7] = rlx_register_fixed_factor(NULL,
		"usb_pll_7", "usb_pll", CLK_SET_RATE_PARENT, 1, 7);

	clks[RLX_CLK_SYS_PLL0] = rlx_register_clk(
		&sys_pll0, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED);
	clks[RLX_CLK_SYS_PLL1] = rlx_register_clk(
		&sys_pll1, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED);
	clks[RLX_CLK_SYS_PLL2] = rlx_register_clk(
		&sys_pll2, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED);
	clks[RLX_CLK_SYS_PLL3] = rlx_register_clk(
		&sys_pll3, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED);

	clks[RLX_CLK_SYS_PLL0_2] = rlx_register_fixed_factor(
		NULL, "sys_pll0_2", "sys_pll0", CLK_SET_RATE_PARENT, 1, 2);
	clks[RLX_CLK_SYS_PLL0_3] = rlx_register_fixed_factor(
		NULL, "sys_pll0_3", "sys_pll0", CLK_SET_RATE_PARENT, 1, 3);
	clks[RLX_CLK_SYS_PLL0_5] = rlx_register_fixed_factor(
		NULL, "sys_pll0_5", "sys_pll0", CLK_SET_RATE_PARENT, 1, 5);
	clks[RLX_CLK_SYS_PLL0_7] = rlx_register_fixed_factor(
		NULL, "sys_pll0_7", "sys_pll0", CLK_SET_RATE_PARENT, 1, 7);

	clks[RLX_CLK_SYS_PLL1_2] = rlx_register_fixed_factor(
		NULL, "sys_pll1_2", "sys_pll1", CLK_SET_RATE_PARENT, 1, 2);
	clks[RLX_CLK_SYS_PLL1_3] = rlx_register_fixed_factor(
		NULL, "sys_pll1_3", "sys_pll1", CLK_SET_RATE_PARENT, 1, 3);
	clks[RLX_CLK_SYS_PLL1_5] = rlx_register_fixed_factor(
		NULL, "sys_pll1_5", "sys_pll1", CLK_SET_RATE_PARENT, 1, 5);
	clks[RLX_CLK_SYS_PLL1_7] = rlx_register_fixed_factor(
		NULL, "sys_pll1_7", "sys_pll1", CLK_SET_RATE_PARENT, 1, 7);

	clks[RLX_CLK_SYS_PLL2_2] = rlx_register_fixed_factor(NULL,
		"sys_pll2_2", "sys_pll2", CLK_SET_RATE_PARENT, 1, 2);
	clks[RLX_CLK_SYS_PLL2_3] = rlx_register_fixed_factor(NULL,
		"sys_pll2_3", "sys_pll2", CLK_SET_RATE_PARENT, 1, 3);
	clks[RLX_CLK_SYS_PLL2_5] = rlx_register_fixed_factor(NULL,
		"sys_pll2_5", "sys_pll2", CLK_SET_RATE_PARENT, 1, 5);
	clks[RLX_CLK_SYS_PLL2_7] = rlx_register_fixed_factor(NULL,
		"sys_pll2_7", "sys_pll2", CLK_SET_RATE_PARENT, 1, 7);

	clks[RLX_CLK_SYS_PLL3_2] = rlx_register_fixed_factor(NULL,
		"sys_pll3_2", "sys_pll3", CLK_SET_RATE_PARENT, 1, 2);
	clks[RLX_CLK_SYS_PLL3_3] = rlx_register_fixed_factor(NULL,
		"sys_pll3_3", "sys_pll3", CLK_SET_RATE_PARENT, 1, 3);
	clks[RLX_CLK_SYS_PLL3_5] = rlx_register_fixed_factor(NULL,
		"sys_pll3_5", "sys_pll3", CLK_SET_RATE_PARENT, 1, 5);
	clks[RLX_CLK_SYS_PLL3_7] = rlx_register_fixed_factor(NULL,
		"sys_pll3_7", "sys_pll3", CLK_SET_RATE_PARENT, 1, 7);

	clks[RLX_CLK_DMA_CK] = rlx_register_clk(&dma_ck,
		CLK_IGNORE_UNUSED);
	clks[RLX_CLK_USBPHY_HOST_CK] = rlx_register_clk(&usbphy_host_ck,
		CLK_IGNORE_UNUSED);
	clks[RLX_CLK_USBPHY_DEV_CK] = rlx_register_clk(&usbphy_dev_ck,
		CLK_IGNORE_UNUSED);
	clks[RLX_CLK_ETHERNET_CK] = rlx_register_clk(&ethernet_ck,
		CLK_IGNORE_UNUSED);
	clks[RLX_CLK_CPU_CK_DIV] = rlx_register_clk(&cpu_ck_div,
		CLK_IGNORE_UNUSED);
	clks[RLX_CLK_CPU_CK_DEC] = rlx_register_clk(&cpu_ck_dec,
		CLK_IGNORE_UNUSED);
	clks[RLX_CLK_CPU_CK] = rlx_register_clk(&cpu_ck,
		CLK_IGNORE_UNUSED);
	clks[RLX_CLK_H264_CK_DIV] = rlx_register_clk(&h264_ck_div,
		CLK_IGNORE_UNUSED);
	clks[RLX_CLK_H264_CK_DEC] = rlx_register_clk(&h264_ck_dec,
		CLK_IGNORE_UNUSED);
	clks[RLX_CLK_H264_CK] = rlx_register_clk(&h264_ck,
		CLK_IGNORE_UNUSED);
	clks[RLX_CLK_JPEG_CK_DIV] = rlx_register_clk(&jpeg_ck_div,
		CLK_IGNORE_UNUSED);
	clks[RLX_CLK_JPEG_CK_DEC] = rlx_register_clk(&jpeg_ck_dec,
		CLK_IGNORE_UNUSED);
	clks[RLX_CLK_JPEG_CK] = rlx_register_clk(&jpeg_ck,
		CLK_IGNORE_UNUSED);
	clks[RLX_CLK_BUS_CK_DIV] = rlx_register_clk(&bus_ck_div,
		CLK_IGNORE_UNUSED);
	clks[RLX_CLK_BUS_CK_DEC] = rlx_register_clk(&bus_ck_dec,
		CLK_IGNORE_UNUSED);
	clks[RLX_CLK_BUS_CK] = rlx_register_clk(&bus_ck,
		CLK_IGNORE_UNUSED);
	clks[RLX_CLK_DRAM_CK_DIV] = rlx_register_clk(&dram_ck_div,
		CLK_IGNORE_UNUSED);
	clks[RLX_CLK_DRAM_CK_DEC] = rlx_register_clk(&dram_ck_dec,
		CLK_IGNORE_UNUSED);
	clks[RLX_CLK_DRAM_CK] = rlx_register_clk(&dram_ck,
		CLK_IGNORE_UNUSED);
	clks[RLX_CLK_ISP_CK_DIV] = rlx_register_clk(&isp_ck_div,
		CLK_IGNORE_UNUSED);
	clks[RLX_CLK_ISP_CK_DEC] = rlx_register_clk(&isp_ck_dec,
		CLK_IGNORE_UNUSED);
	clks[RLX_CLK_ISP_CK] = rlx_register_clk(&isp_ck,
		CLK_IGNORE_UNUSED);
	clks[RLX_CLK_MIPI_CK_DIV] = rlx_register_clk(&mipi_ck_div,
		CLK_IGNORE_UNUSED);
	clks[RLX_CLK_MIPI_CK_DEC] = rlx_register_clk(&mipi_ck_dec,
		CLK_IGNORE_UNUSED);
	clks[RLX_CLK_MIPI_CK] = rlx_register_clk(&mipi_ck,
		CLK_IGNORE_UNUSED);
	clks[RLX_CLK_I2C_CK_DIV] = rlx_register_clk(&i2c_ck_div,
		CLK_IGNORE_UNUSED);
	clks[RLX_CLK_I2C_CK] = rlx_register_clk(&i2c_ck,
		CLK_IGNORE_UNUSED);
	clks[RLX_CLK_XB2_CK_DIV] = rlx_register_clk(&xb2_ck_div,
		CLK_IGNORE_UNUSED);
	clks[RLX_CLK_XB2_CK] = rlx_register_clk(&xb2_ck,
		CLK_IGNORE_UNUSED);
	clks[RLX_CLK_UART_CK_DIV] = rlx_register_clk(&uart_ck_div,
		CLK_IGNORE_UNUSED);
	clks[RLX_CLK_UART_CK] = rlx_register_clk(&uart_ck,
		CLK_IGNORE_UNUSED);
	clks[RLX_CLK_I2S_CK_DIV]	= rlx_register_clk(&i2s_ck_div,
		CLK_IGNORE_UNUSED);
	clks[RLX_CLK_I2S_CK] = rlx_register_clk(&i2s_ck,
		CLK_IGNORE_UNUSED);
	clks[RLX_CLK_SPDIF_CK_DIV] = rlx_register_clk(&spdif_ck_div,
		CLK_IGNORE_UNUSED);
	clks[RLX_CLK_SPDIF_CK] = rlx_register_clk(&spdif_ck,
		CLK_IGNORE_UNUSED);
	clks[RLX_CLK_CODEC_CK_DIV] = rlx_register_clk(&codec_ck_div,
		CLK_IGNORE_UNUSED);
	clks[RLX_CLK_CODEC_CK] = rlx_register_clk(&codec_ck,
		CLK_IGNORE_UNUSED);
	clks[RLX_CLK_MCU_CK] = clk_register_gate(NULL,
		"mcu_ck", "usb_pll", 0, MCU_CLK_CFG_REG, 24, 0, NULL);
	clks[RLX_CLK_CIPHER_CK] = clk_register_gate(NULL,
		"cipher_ck", "usb_pll", 0, CIPHER_CLK_CFG_REG, 24, 0, NULL);

	if (clk_platform_type & TYPE_RTS3913) {
		clks[RLX_CLK_H265_ACLK_CK] =
			rlx_register_clk(&h265_aclk_ck, CLK_IGNORE_UNUSED);

		clks[RLX_CLK_H265_BCLK_CK_DIV] =
			rlx_register_clk(&h265_bclk_ck_div, CLK_IGNORE_UNUSED);
		clks[RLX_CLK_H265_BCLK_CK_DEC] =
			rlx_register_clk(&h265_bclk_ck_dec, CLK_IGNORE_UNUSED);
		clks[RLX_CLK_H265_BCLK_CK] =
			rlx_register_clk(&h265_bclk_ck, CLK_IGNORE_UNUSED);

		clks[RLX_CLK_H265_CCLK_CK_DIV] =
			rlx_register_clk(&h265_cclk_ck_div, CLK_IGNORE_UNUSED);
		clks[RLX_CLK_H265_CCLK_CK_DEC] =
			rlx_register_clk(&h265_cclk_ck_dec, CLK_IGNORE_UNUSED);
		clks[RLX_CLK_H265_CCLK_CK] =
			rlx_register_clk(&h265_cclk_ck, CLK_IGNORE_UNUSED);

		clks[RLX_CLK_H265_CK] = rlx_register_clk(&h265_ck,
			CLK_IGNORE_UNUSED);

		clksize = RLX_CLK_H265_CK + 1;
	} else {
		clksize = RLX_CLK_CIPHER_CK + 1;
	}

	rlx_check_clocks(clks, clksize);

	clk_data.clks = clks;
	clk_data.clk_num = clksize;
	of_clk_add_provider(node, of_clk_src_onecell_get, &clk_data);

#ifdef CONFIG_SOC_ENABLE_PLL0
	pll0_clk = clk_get(NULL, "sys_pll0");
	clk_set_rate(pll0_clk, 1200000000);
	clk_prepare_enable(pll0_clk);
	clk_put(pll0_clk);
#endif

}

static const struct of_device_id rlx_clk_match[] = {
	{
		.compatible = "realtek,rts3903-clocks",
		.data = (void *)(TYPE_RTS3903),
	},
	{}
};
MODULE_DEVICE_TABLE(of, rlx_clk_match);

static void __init rlx_clk_init(struct device_node *np)
{
	const struct of_device_id *of_id;

	of_id = of_match_node(rlx_clk_match, np);
	if (!of_id)
		return;
	clk_platform_type = (int)(of_id->data);

	/* fpga board */
	if (of_machine_is_compatible("realtek,rts_fpga"))
		clk_platform_type |= TYPE_FPGA;

	clk_mapped_addr = of_iomap(np, 0);
	if (!clk_mapped_addr) {
		pr_err("Can't map clk registers\n");
		return;
	}

	pll_mapped_addr = of_iomap(np, 1);
	if (!pll_mapped_addr) {
		pr_err("Can't map pll registers\n");
		return;
	}

	rlx_clocks_init(np);
}

CLK_OF_DECLARE(rts3903_clocks, "realtek,rts3903-clocks", rlx_clk_init);
