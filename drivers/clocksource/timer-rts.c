#include <linux/init.h>
#include <linux/err.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/sched_clock.h>
#include <linux/module.h>
#include <linux/slab.h>

#define RTS_TIMER0_EN		0x00
#define RTS_TIMER0_COMPARE	0x04
#define RTS_TIMER0_CURRENT	0x08
#define RTS_TIMER0_MODE		0x0C

#define RTS_TIMER1_EN		0x20
#define RTS_TIMER1_COMPARE	0x24
#define RTS_TIMER1_CURRENT	0x28
#define RTS_TIMER1_MODE		0x2C

#define RTS_TIMER_INT_EN	0x40
#define RTS_TIMER_INT_STS	0x44

#define RTS_TIMER_EN		RTS_TIMER0_EN
#define RTS_TIMER_COMPARE	RTS_TIMER0_COMPARE
#define RTS_TIMER_CURRENT	RTS_TIMER0_CURRENT
#define RTS_TIMER_MODE		RTS_TIMER0_MODE
#define RTS_TIMER_INT_N		0

#define RTS_TIMER_FREQUENCY	25000000

struct rts_timer_data {
	void __iomem *addr;
	int irq;
};

static struct rts_timer_data *rts_tdata;

static unsigned int rts_timer_read(struct rts_timer_data *tdata,
				    unsigned int reg)
{
	return readl(tdata->addr + reg);
}

static int rts_timer_write(struct rts_timer_data *tdata, unsigned int reg,
			    unsigned int value)
{
	writel(value, tdata->addr + reg);

	return 0;
}

static cycle_t rts_timer_read_count(struct clocksource *cs)
{
	return rts_timer_read(rts_tdata, RTS_TIMER_CURRENT);
}

static struct clocksource clocksource_rts = {
	.name = "RTS CLOCKSOURCE",
	.read = rts_timer_read_count,
	.mask = CLOCKSOURCE_MASK(32),
	.flags = CLOCK_SOURCE_IS_CONTINUOUS,
	.rating = 390,
};

static void rts_setup_clocksource(void)
{
	/* enable timer */
	rts_timer_write(rts_tdata, RTS_TIMER_EN, 0x1);

	clocksource_register_hz(&clocksource_rts, RTS_TIMER_FREQUENCY);
}

static int rts_timer_next_event(unsigned long delta,
		struct clock_event_device *evt)
{
	unsigned int cnt;
	int res;

	cnt = rts_timer_read(rts_tdata, RTS_TIMER_CURRENT);
	cnt += delta;
	rts_timer_write(rts_tdata, RTS_TIMER_COMPARE, cnt);

	res = ((int)(rts_timer_read(rts_tdata, RTS_TIMER_CURRENT) -
				cnt) >= 0) ? -ETIME : 0;

	return 0;
}

static struct clock_event_device clockevent_rts = {
	.name = "RTS CLOCKEVENT",
	.rating = 390,
	.features = CLOCK_EVT_FEAT_ONESHOT,
	.set_next_event = rts_timer_next_event,
};

void rts_timer_event_handler(struct clock_event_device *dev)
{
}

static unsigned int calculate_min_delta(void)
{
	unsigned int cnt, i, j, k, l;
	unsigned int buf1[4], buf2[3];
	unsigned int min_delta;

	/*
	 * Calculate the median of 5 75th percentiles of 5 samples of how long
	 * it takes to set Compare = Count + delta.
	 */
	for (i = 0; i < 5; ++i) {
		for (j = 0; j < 5; ++j) {
			cnt = rts_timer_read(rts_tdata, RTS_TIMER_CURRENT);
			rts_timer_write(rts_tdata, RTS_TIMER_COMPARE, cnt);
			cnt = rts_timer_read(rts_tdata, RTS_TIMER_CURRENT) -
						cnt;

			/* Sorted insert into buf1 */
			for (k = 0; k < j; ++k) {
				if (cnt < buf1[k]) {
					l = min_t(unsigned int,
						  j, ARRAY_SIZE(buf1) - 1);
					for (; l > k; --l)
						buf1[l] = buf1[l - 1];
					break;
				}
			}

			if (k < ARRAY_SIZE(buf1))
				buf1[k] = cnt;
		}

		/* Sorted insert of 75th percentile into buf2 */
		for (k = 0; k < i; ++k) {
			if (buf1[ARRAY_SIZE(buf1) - 1] < buf2[k]) {
				l = min_t(unsigned int,
					  i, ARRAY_SIZE(buf2) - 1);
				for (; l > k; --l)
					buf2[l] = buf2[l - 1];
				break;
			}
		}

		if (k < ARRAY_SIZE(buf2))
			buf2[k] = buf1[ARRAY_SIZE(buf1) - 1];
	}

	min_delta = buf2[ARRAY_SIZE(buf2) - 1] * 2;

	if (min_delta < 0x300)
		min_delta = 0x300;

	return min_delta;
}

static irqreturn_t rts_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = (struct clock_event_device *)dev_id;
	u32 int_val;

	int_val = rts_timer_read(rts_tdata, RTS_TIMER_INT_STS);
	rts_timer_write(rts_tdata, RTS_TIMER_INT_STS, int_val);
	if (int_val & (0x1 << RTS_TIMER_INT_N))
		evt->event_handler(evt);

	return IRQ_HANDLED;
}

static struct irqaction rts_timer_irq = {
	.name = "rts_timer",
	.flags = IRQF_TIMER,
	.handler = rts_timer_interrupt,
	.dev_id = &clockevent_rts,
};

static void rts_setup_clockevent(void)
{
	unsigned int min_delta;

	min_delta = calculate_min_delta();

	clockevent_rts.cpumask = cpumask_of(0);
	clockevent_rts.event_handler = rts_timer_event_handler;

	setup_irq(rts_tdata->irq, &rts_timer_irq);

	/* enable timer interrupt */
	rts_timer_write(rts_tdata, RTS_TIMER_INT_EN, (0x1 << RTS_TIMER_INT_N));

	clockevents_config_and_register(&clockevent_rts, RTS_TIMER_FREQUENCY,
			min_delta, 0x7fffffff);
}

static int __init rts_timer_init(struct device_node *node)
{
	struct rts_timer_data *data;
	void __iomem *timer_base;
	int irq;

	timer_base = of_io_request_and_map(node, 0, of_node_full_name(node));
	if (IS_ERR(timer_base)) {
		pr_err("Can't map registers");
		return PTR_ERR(timer_base);
	}

	irq = irq_of_parse_and_map(node, 0);
	if (irq <= 0) {
		pr_err("Can't parse IRQ");
		return -EINVAL;
	}

	data = kzalloc(sizeof(struct rts_timer_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->addr = timer_base;
	data->irq = irq;

	rts_tdata = data;

	rts_setup_clocksource();
	rts_setup_clockevent();

	return 0;
}
CLOCKSOURCE_OF_DECLARE(rts_timer, "realtek,rts3913-timer",
			rts_timer_init);
