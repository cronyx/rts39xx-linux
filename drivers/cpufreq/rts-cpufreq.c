#include <linux/cpufreq.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <asm/clock.h>
#include <asm/idle.h>
#include <asm/time.h>
#include <linux/cpu.h>
#include <linux/pm_opp.h>

static struct cpufreq_frequency_table *rts_freq_table;

static struct clk *rts_cpu_clk;
static DEFINE_MUTEX(rts_cpu_lock);

static int rts_max_freq;

static int rts_cpu_freq_notifier(struct notifier_block *nb,
					unsigned long val, void *data)
{
	if (val == CPUFREQ_POSTCHANGE) {
		mips_hpt_frequency = clk_get_rate(rts_cpu_clk);
		rlx_clocksource_update();
		rlx_clockevent_update();
	}

	return 0;
}

static struct notifier_block rts_cpufreq_notifier_block = {
	.notifier_call = rts_cpu_freq_notifier,
};

static int rts_cpufreq_verify(struct cpufreq_policy *policy)
{
	return cpufreq_frequency_table_verify(policy, rts_freq_table);
}

static unsigned int rts_cpufreq_round(unsigned long real_freq)
{
	int i;
	unsigned int freq_down = 0, freq_up = ~0, freq_mid;

	for (i = 0; (rts_freq_table[i].frequency != CPUFREQ_TABLE_END); i++) {
		unsigned int freq = rts_freq_table[i].frequency;

		if (freq == CPUFREQ_ENTRY_INVALID)
			continue;

		if (real_freq >= freq * 1000) {
			if (freq_down <= freq)
				freq_down = freq;
		} else {
			if (freq_up >= freq)
				freq_up = freq;
		}
	}

	if (freq_up == ~0)
		return freq_down;
	else if (freq_down == 0)
		return freq_up;

	freq_mid = freq_down + (freq_up - freq_down) / 2;

	if (real_freq > freq_mid * 1000)
		return freq_up;

	return freq_down;
}

static unsigned int rts_cpufreq_get(unsigned int cpu)
{
	return rts_cpufreq_round(clk_get_rate(rts_cpu_clk));
}

static int rts_cpufreq_target_index(struct cpufreq_policy *policy,
			unsigned int index)
{
	unsigned long rate;

	mutex_lock(&rts_cpu_lock);

	rate = rts_freq_table[index].frequency;

	clk_set_rate(rts_cpu_clk, rate * 1000);

	mutex_unlock(&rts_cpu_lock);

	return 0;
}

static int rts_cpufreq_init(struct cpufreq_policy *policy)
{
	unsigned int i;
	struct device *cpu_dev;
	int ret;

	cpu_dev = get_cpu_device(policy->cpu);
	if (!cpu_dev) {
		pr_err("failed to get cpu%d device\n", policy->cpu);
		return -ENODEV;
	}

	rts_cpu_clk = clk_get(cpu_dev, NULL);
	if (IS_ERR(rts_cpu_clk)) {
		ret = PTR_ERR(rts_cpu_clk);
		dev_err(cpu_dev, "%s: failed to get clk: %d\n", __func__, ret);
		return ret;
	}

	ret = dev_pm_opp_of_add_table(cpu_dev);
	if (ret) {
		dev_err(cpu_dev, "failed to init OPP table: %d\n", ret);
		goto err;
	}

	ret = dev_pm_opp_init_cpufreq_table(cpu_dev,
				&rts_freq_table);
	if (ret) {
		dev_err(cpu_dev,
			"failed to init cpufreq table: %d\n", ret);
		goto err;
	}

	if (!rts_max_freq)
		rts_max_freq = rts_cpufreq_round(mips_hpt_frequency);

	/* table init */
	for (i = 0; (rts_freq_table[i].frequency != CPUFREQ_TABLE_END); i++) {
		if (rts_freq_table[i].frequency > rts_max_freq)
			rts_freq_table[i].frequency = CPUFREQ_ENTRY_INVALID;
	}

	cpufreq_generic_init(policy, rts_freq_table, 10 * 1000);
	policy->cur = rts_cpufreq_get(0);

	return 0;
err:
	clk_put(rts_cpu_clk);
	rts_cpu_clk = NULL;

	return ret;
}

static int rts_cpufreq_exit(struct cpufreq_policy *policy)
{
	cpufreq_frequency_table_cpuinfo(policy, rts_freq_table);
	clk_put(rts_cpu_clk);
	rts_cpu_clk = NULL;

	return 0;
}

static struct freq_attr *rts_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static struct cpufreq_driver rts_cpufreq_driver = {
	.name = "rts",
	.verify = rts_cpufreq_verify,
	.target_index = rts_cpufreq_target_index,
	.get = rts_cpufreq_get,
	.init = rts_cpufreq_init,
	.exit = rts_cpufreq_exit,
	.attr = rts_cpufreq_attr,
};

static int __init rts_cpufreq_module_init(void)
{
	cpufreq_register_notifier(&rts_cpufreq_notifier_block,
			CPUFREQ_TRANSITION_NOTIFIER);

	return cpufreq_register_driver(&rts_cpufreq_driver);
}
module_init(rts_cpufreq_module_init);

static void __exit rts_cpufreq_module_exit(void)
{
	cpufreq_unregister_driver(&rts_cpufreq_driver);
	cpufreq_unregister_notifier(&rts_cpufreq_notifier_block,
			CPUFREQ_TRANSITION_NOTIFIER);
}
module_exit(rts_cpufreq_module_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Wind_Han <wind_han@realsil.com.cn>");
MODULE_DESCRIPTION("Realtek RTS cpufreq driver");
