/*
 * Realtek Semiconductor Corp.
 *
 * bsp/timer.c:
 *     bsp timer initialization code
 *
 * Copyright (C) 2006-2012 Tony Wu (tonywu@realtek.com)
 */
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clocksource.h>
#include <linux/of.h>
#include "bspchip.h"

#ifdef CONFIG_CEVT_R4K
#include <asm/time.h>

void __init plat_time_init(void)
{
	struct device_node *np;
	struct clk *clk;
	int ret;

	of_clk_init(NULL);

	np = of_get_cpu_node(0, NULL);
	if (!np) {
		pr_err("Failed to get CPU node\n");
		return;
	}

	clk = of_clk_get(np, 0);
	if (IS_ERR(clk)) {
		pr_err("Failed to get CPU clock: %ld\n", PTR_ERR(clk));
		return;
	}

	mips_hpt_frequency = clk_get_rate(clk);
	clk_put(clk);

	ret = of_property_read_u32(np, "interrupts", &cp0_compare_irq);
	if (ret < 0) {
		pr_err("Failed to get CPU interrupt: %d\n", ret);
		return;
	}
	cp0_compare_irq += MIPS_VEC_IRQ_BASE;

	write_c0_count(0);
	clear_c0_cause(CAUSEF_DC);

	clocksource_probe();
}
#endif
