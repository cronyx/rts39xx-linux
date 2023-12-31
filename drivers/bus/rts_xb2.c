#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of.h>
#include <linux/of_device.h>

#define XB2_PERIP_INT_EN 0x10
#define XB2_PERIP_INT_FLAG 0x14
#define XB2_PERIP_INT_GLOBAL_EN 0x18
#define XB2_UART0_PULL_CTRL 0x30
#define XB2_UART0_DRV_SEL 0x34
#define XB2_UART0_SR_CTRL 0x38
#define XB2_UART1_PULL_CTRL 0x3C
#define XB2_UART1_DRV_SEL 0x40
#define XB2_UART1_SR_CTRL 0x44
#define XB2_UART2_PULL_CTRL 0x48
#define XB2_UART2_DRV_SEL 0x4C
#define XB2_UART2_SR_CTRL 0x50

#define MDIO_RD_DONE_INT_BIT 27
#define MDIO_WR_DONE_INT_BIT 26
#define SARADC_DONE_INT_BIT 24
#define RTC_ALARM3_INT_BIT 7
#define RTC_ALARM2_INT_BIT 6
#define RTC_ALARM1_INT_BIT 5
#define RTC_ALARM0_INT_BIT 4
#define PWM3_DONE_INT_BIT 3
#define PWM2_DONE_INT_BIT 2
#define PWM1_DONE_INT_BIT 1
#define PWM0_DONE_INT_BIT 0

#define MDIO_RD_DONE_INT_IRQ 10
#define MDIO_WR_DONE_INT_IRQ 9
#define SARADC_DONE_INT_IRQ 8
#define RTC_ALARM3_INT_IRQ 7
#define RTC_ALARM2_INT_IRQ 6
#define RTC_ALARM1_INT_IRQ 5
#define RTC_ALARM0_INT_IRQ 4
#define PWM3_DONE_INT_IRQ 3
#define PWM2_DONE_INT_IRQ 2
#define PWM1_DONE_INT_IRQ 1
#define PWM0_DONE_INT_IRQ 0

#define PWM_IRQ_OFFSET (PWM0_DONE_INT_BIT - PWM0_DONE_INT_IRQ)
#define RTC_IRQ_OFFSET	(RTC_ALARM0_INT_BIT - RTC_ALARM0_INT_IRQ)
#define SARADC_IRQ_OFFSET (SARADC_DONE_INT_BIT - SARADC_DONE_INT_IRQ)
#define MDIO_IRQ_OFFSET (MDIO_WR_DONE_INT_BIT - MDIO_WR_DONE_INT_IRQ)

#define RTS_NUM_IRQS 11

struct rts_xb2 {
	struct irq_domain *irq_domain;
	spinlock_t irq_lock;
	void __iomem *addr;
	int irq;
	int devtype;
	struct resource *mem;
};

struct rts_xb2 xb2;

static struct lock_class_key xb2_lock_class;

int rts_xb2_uart_cw(unsigned int offset, unsigned int value)
{
	if (offset > (XB2_UART2_SR_CTRL - XB2_UART0_PULL_CTRL))
		return -EINVAL;

	writel(value, xb2.addr + XB2_UART0_PULL_CTRL + offset);

	return 0;
}
EXPORT_SYMBOL_GPL(rts_xb2_uart_cw);

int rts_xb2_uart_cr(unsigned int offset, unsigned int *value)
{
	if (offset > (XB2_UART2_SR_CTRL - XB2_UART0_PULL_CTRL))
		return -EINVAL;

	*value = readl(xb2.addr + XB2_UART0_PULL_CTRL + offset);

	return 0;
}
EXPORT_SYMBOL_GPL(rts_xb2_uart_cr);

int rts_xb2_to_irq(unsigned int offset)
{
	return irq_linear_revmap(xb2.irq_domain, offset);
}
EXPORT_SYMBOL_GPL(rts_xb2_to_irq);

static void rts_xb2_irq_enable(struct irq_data *data)
{
	struct rts_xb2 *rtsxb2 = irq_data_get_irq_chip_data(data);
	unsigned int hwirq = irqd_to_hwirq(data);
	unsigned long flags;

	spin_lock_irqsave(&rtsxb2->irq_lock, flags);

	if (hwirq <= PWM3_DONE_INT_IRQ && hwirq >= PWM0_DONE_INT_IRQ)
		set_bit(hwirq + PWM_IRQ_OFFSET,
			rtsxb2->addr + XB2_PERIP_INT_EN);
	else if (hwirq <= RTC_ALARM3_INT_IRQ && hwirq >= RTC_ALARM0_INT_IRQ) {
		set_bit(hwirq + RTC_IRQ_OFFSET,
			rtsxb2->addr + XB2_PERIP_INT_EN);
	} else if (hwirq <= MDIO_WR_DONE_INT_IRQ
		   && hwirq >= MDIO_RD_DONE_INT_IRQ)
		set_bit(hwirq + MDIO_IRQ_OFFSET,
			rtsxb2->addr + XB2_PERIP_INT_EN);
	else if (hwirq == SARADC_DONE_INT_IRQ)
		set_bit(hwirq + SARADC_IRQ_OFFSET,
			rtsxb2->addr + XB2_PERIP_INT_EN);

	spin_unlock_irqrestore(&rtsxb2->irq_lock, flags);
}

static void rts_xb2_irq_disable(struct irq_data *data)
{
	struct rts_xb2 *rtsxb2 = irq_data_get_irq_chip_data(data);
	unsigned int hwirq = irqd_to_hwirq(data);
	unsigned long flags;

	spin_lock_irqsave(&rtsxb2->irq_lock, flags);

	if (hwirq <= PWM3_DONE_INT_IRQ && hwirq >= PWM0_DONE_INT_IRQ)
		clear_bit(hwirq + PWM_IRQ_OFFSET,
			  rtsxb2->addr + XB2_PERIP_INT_EN);
	else if (hwirq <= RTC_ALARM3_INT_IRQ && hwirq >= RTC_ALARM0_INT_IRQ)
		clear_bit(hwirq + RTC_IRQ_OFFSET,
			  rtsxb2->addr + XB2_PERIP_INT_EN);
	else if (hwirq <= MDIO_WR_DONE_INT_IRQ && hwirq >= MDIO_RD_DONE_INT_IRQ)
		clear_bit(hwirq + MDIO_IRQ_OFFSET,
			  rtsxb2->addr + XB2_PERIP_INT_EN);
	else if (hwirq == SARADC_DONE_INT_IRQ)
		clear_bit(hwirq + SARADC_IRQ_OFFSET,
			  rtsxb2->addr + XB2_PERIP_INT_EN);

	spin_unlock_irqrestore(&rtsxb2->irq_lock, flags);
}

static struct irq_chip rts_xb2_irq_chip = {
	.name = "XB2",
	.irq_enable = rts_xb2_irq_enable,
	.irq_disable = rts_xb2_irq_disable,
};

static irqreturn_t rts_irq_handler(int irq, void *pc)
{
	unsigned long offset;
	struct rts_xb2 *rtsxb2 = &xb2;
	unsigned long val = readl(rtsxb2->addr + XB2_PERIP_INT_FLAG);

	int irqno;

	val &= 0xd0000ff;

	writel(val, rtsxb2->addr + XB2_PERIP_INT_FLAG);

	if (val == 0)
		return IRQ_NONE;

	for_each_set_bit(offset, (const unsigned long *)&val, 32) {
		irqno = offset;

		if (irqno <= PWM3_DONE_INT_BIT && irqno >= PWM0_DONE_INT_BIT)
			irqno -= PWM_IRQ_OFFSET;
		else if (irqno <= RTC_ALARM3_INT_BIT
			 && irqno >= RTC_ALARM0_INT_BIT)
			irqno -= RTC_IRQ_OFFSET;
		else if (irqno <= MDIO_WR_DONE_INT_BIT
			 && irqno >= MDIO_RD_DONE_INT_BIT)
			irqno -= MDIO_IRQ_OFFSET;
		else if (irqno == SARADC_DONE_INT_BIT)
			irqno -= SARADC_IRQ_OFFSET;

		set_bit(irqno, rtsxb2->addr + XB2_PERIP_INT_FLAG);

		irqno = irq_linear_revmap(rtsxb2->irq_domain, irqno);
		if (irqno)
			generic_handle_irq(irqno);
	}

	return IRQ_HANDLED;
}

static const struct of_device_id rts_xb2_match[] = {
	{ .compatible = "realtek,rts3903-xb2" },
	{ },
};
MODULE_DEVICE_TABLE(of, rts_xb2_match);

static int rts_xb2_probe(struct platform_device *pdev)
{
	int ret, i;
	struct rts_xb2 *rtsxb2 = &xb2;
	const struct of_device_id *of_id;

	spin_lock_init(&rtsxb2->irq_lock);

	of_id = of_match_device(rts_xb2_match, &pdev->dev);

	rtsxb2->devtype = (int)of_id->data;

	rtsxb2->irq = platform_get_irq(pdev, 0);
	if (rtsxb2->irq < 0) {
		ret = -ENOENT;
		dev_err(&pdev->dev, "Failed to get platform irq\n");
		goto err_failed;
	}
	rtsxb2->mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!rtsxb2->mem) {
		ret = -ENOENT;
		dev_err(&pdev->dev, "Failed to get platform mmio memory\n");
		goto err_failed;
	}
	rtsxb2->mem = request_mem_region(rtsxb2->mem->start,
		resource_size(rtsxb2->mem), pdev->name);
	if (!rtsxb2->mem) {
		ret = -EBUSY;
		dev_err(&pdev->dev, "Failed to request mmio memory region\n");
		goto err_failed;
	}
	rtsxb2->addr = ioremap_nocache(rtsxb2->mem->start,
		resource_size(rtsxb2->mem));
	if (!rtsxb2->addr) {
		ret = -EBUSY;
		dev_err(&pdev->dev, "Failed to ioremap mmio memory\n");
		goto err_release_mem_region;
	}

	rtsxb2->irq_domain = irq_domain_add_linear(NULL, RTS_NUM_IRQS,
						   &irq_domain_simple_ops,
						   NULL);
	if (!rtsxb2->irq_domain) {
		dev_err(&pdev->dev, "could not create IRQ domain\n");
		return -ENOMEM;
	}

	for (i = 0; i < RTS_NUM_IRQS; i++) {
		int irq = irq_create_mapping(rtsxb2->irq_domain, i);

		irq_set_lockdep_class(irq, &xb2_lock_class);
		irq_set_chip_and_handler(irq, &rts_xb2_irq_chip,
					 handle_simple_irq);
		irq_set_chip_data(irq, rtsxb2);
	}

	ret = request_irq(rtsxb2->irq, rts_irq_handler,
			  IRQF_SHARED, "xb2", (void *)rtsxb2);
	if (ret) {
		dev_dbg(&pdev->dev, "failure requesting irq %i\n", rtsxb2->irq);
		goto err_failed;
	}

	set_bit(0, rtsxb2->addr + XB2_PERIP_INT_GLOBAL_EN);

	dev_dbg(&pdev->dev, "rtsxb2 registered with IRQs\n");

	return 0;

err_release_mem_region:
	release_mem_region(rtsxb2->mem->start, resource_size(rtsxb2->mem));
err_failed:
	return ret;
}

static int rts_xb2_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver rts_xb2_driver = {
	.driver = {
		.name = "rts-xb2",
		.of_match_table = of_match_ptr(rts_xb2_match),
	},
	.probe = rts_xb2_probe,
	.remove = rts_xb2_remove,
};

static int __init rts_xb2_init(void)
{
	return platform_driver_register(&rts_xb2_driver);
}
arch_initcall(rts_xb2_init);

