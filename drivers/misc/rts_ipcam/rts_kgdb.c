#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kgdb.h>

#define KGDB_GPIO_NUM	CONFIG_KGDB_GPIO_NUM

static irqreturn_t rtsx_kgdb_isr(int irq, void *dev_id)
{
	disable_irq_nosync(irq);
	pr_info("Enter kgdb.\n");
	kgdb_breakpoint();
	enable_irq(irq);
	return IRQ_HANDLED;
}

static int rtsx_kgdb_acquire_irq(int irq)
{
	int err = 0;

	err = request_irq(irq, rtsx_kgdb_isr, IRQF_TRIGGER_RISING,
				"kgdb-gpio", NULL);
	if (err)
		pr_err("request IRQ %d failed.\n", irq);

	return err;
}

static int __init kgdb_gpio_init(void)
{
	int ret = 0;
	int irq = 0;

	pr_info("kgdb gpio init.\n");
	ret = gpio_request(KGDB_GPIO_NUM, "kgdb");
	if (ret)
		pr_err("request gpio error %d.\n", ret);
	ret = gpio_direction_input(KGDB_GPIO_NUM);
	if (ret) {
		pr_err("set gpio direction input error %d.\n", ret);
		goto fail;
	}
	irq = gpio_to_irq(KGDB_GPIO_NUM);
	if (irq < 0) {
		pr_err("gpio to irq error %d.\n", irq);
		goto fail;
	}
	ret = rtsx_kgdb_acquire_irq(irq);
	if (ret)
		goto fail;
	return 0;

fail:
	gpio_free(KGDB_GPIO_NUM);
	return -1;
}

static void __exit kgdb_gpio_exit(void)
{
	int irq = 0;

	irq = gpio_to_irq(KGDB_GPIO_NUM);
	if (irq < 0)
		pr_err("gpio to irq error %d.\n", irq);
	free_irq(irq, NULL);
	gpio_free(KGDB_GPIO_NUM);
	pr_info("kgdb gpio exit.\n");
}

module_init(kgdb_gpio_init);
module_exit(kgdb_gpio_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Rui Feng <rui_feng@realsil.com.cn>");
MODULE_DESCRIPTION("Realtek ipcam enter kgdb by gpio module");
