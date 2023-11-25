#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <asm/irq.h>
#include <linux/io.h>
#include <asm/ioctl.h>
#include <linux/signal.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/nvmem-provider.h>

struct rts_efuse_regs {
	u32 efuse_cfg;
#define SYS_EFUSE_CFG(rts) (&(rts)->regs->efuse_cfg)

	u32 efuse_ctl;
#define SYS_EFUSE_CTL(rts) (&(rts)->regs->efuse_ctl)

	u32 efuse_cnt;
#define SYS_EFUSE_CNT(rts) (&(rts)->regs->efuse_cnt)

	u32 efuse_read_dat;
#define SYS_EFUSE_RDATA(rts) (&(rts)->regs->efuse_read_dat)

	u32 efuse_dat[6];
#define SYS_EFUSE_DATA0(rts) ((rts)->regs->efuse_dat)
};

struct rts_efuse {
	struct device				*dev;
	struct rts_efuse_regs __iomem		*regs;
	struct mutex				mutex;
	struct cdev				cdev;
};

static struct rts_efuse *g_rts;

/* cfg bits */
#define EFUSE_DEFAUTL_V	0x2e

#define LOAD_AES_KEY 17
#define EFUSE_POW33 16
#define PROGRAM_POW 14
#define EFUSE_PD 13
#define EFUSE_PS 12
#define EFUSE_CSB 11
#define EFUSE_PGENB 10
#define EFUSE_LOAD 9
#define EFUSE_STROBE 8

#define BYTE_NUMS 28
#define PGM_CNT 120
#define EFUSE_READ_DONE	0x01

static int efuse_check(unsigned int offset, size_t bytes)
{
	if (offset > BYTE_NUMS ||
		(offset + bytes) > BYTE_NUMS)
		return -EINVAL;

	return 0;
}

static void efuse_init(uint32_t pgm_cnt, struct rts_efuse *rts_pt)
{
	u32 tmp;

	tmp = readl(SYS_EFUSE_CFG(rts_pt));
	set_bit(EFUSE_POW33, (unsigned long *)&tmp);
	writel(tmp, SYS_EFUSE_CFG(rts_pt));

	tmp = pgm_cnt & 0xff;
	writel(tmp, SYS_EFUSE_CNT(rts_pt));
}

static uint8_t efuse_read_byte(uint8_t byte_index, struct rts_efuse *rts_pt)
{
	u32 tmp;
	u32  waddr, baddr, val;
	uint8_t valb;

	waddr = byte_index / 4;
	baddr = byte_index % 4;

	tmp = readl(SYS_EFUSE_CFG(rts_pt));

	clear_bit(EFUSE_PD, (unsigned long *)&tmp);
	writel(tmp, SYS_EFUSE_CFG(rts_pt));

	clear_bit(EFUSE_PS, (unsigned long *)&tmp);
	writel(tmp, SYS_EFUSE_CFG(rts_pt));

	set_bit(EFUSE_LOAD, (unsigned long *)&tmp);
	set_bit(EFUSE_PGENB, (unsigned long *)&tmp);
	writel(tmp, SYS_EFUSE_CFG(rts_pt));

	clear_bit(EFUSE_CSB, (unsigned long *)&tmp);
	writel(tmp, SYS_EFUSE_CFG(rts_pt));

	tmp &= 0xffffff00;
	tmp |= (waddr & 0xff);
	writel(tmp, SYS_EFUSE_CFG(rts_pt));

	set_bit(EFUSE_STROBE, (unsigned long *)&tmp);
	writel(tmp, SYS_EFUSE_CFG(rts_pt));

	writel((readl(SYS_EFUSE_RDATA(rts_pt))),
			(SYS_EFUSE_DATA0(rts_pt) + waddr));

	clear_bit(EFUSE_STROBE, (unsigned long *)&tmp);
	writel(tmp, SYS_EFUSE_CFG(rts_pt));

	val = readl(SYS_EFUSE_DATA0(rts_pt) + waddr);

	writel(EFUSE_READ_DONE, SYS_EFUSE_CTL(rts_pt));

	udelay(10);
	writel(EFUSE_DEFAUTL_V, SYS_EFUSE_CFG(rts_pt));

	valb = ((u8 *)&val)[baddr];

	return valb;
}

static void efuse_write_byte(uint8_t byte_val,
	uint8_t byte_index, struct rts_efuse *rts_pt)
{
	u32 tmp;
	u32 addr;
	u32 j, k;
	u32 waddr, baddr;

	waddr = byte_index / 4;
	baddr = byte_index % 4;

	tmp = readl(SYS_EFUSE_CFG(rts_pt));

	set_bit(PROGRAM_POW, (unsigned long *)&tmp);
	writel(tmp, SYS_EFUSE_CFG(rts_pt));

	clear_bit(EFUSE_PD, (unsigned long *)&tmp);
	writel(tmp, SYS_EFUSE_CFG(rts_pt));

	set_bit(EFUSE_PS, (unsigned long *)&tmp);
	writel(tmp, SYS_EFUSE_CFG(rts_pt));

	clear_bit(EFUSE_LOAD, (unsigned long *)&tmp);
	clear_bit(EFUSE_PGENB, (unsigned long *)&tmp);
	clear_bit(EFUSE_CSB, (unsigned long *)&tmp);
	writel(tmp, SYS_EFUSE_CFG(rts_pt));

	k = 1;
	for (j = 0; j < 8; j++) {
		if (byte_val & k) {
			addr = ((j + baddr * 8) << 3) | waddr;
			tmp = readl(SYS_EFUSE_CFG(rts_pt));
			tmp &= 0xffffff00;
			tmp |= (addr & 0xff);
			writel(tmp, SYS_EFUSE_CFG(rts_pt));

			set_bit(EFUSE_STROBE, (unsigned long *)&tmp);
			writel(tmp, SYS_EFUSE_CFG(rts_pt));
			udelay(4);
		}
		k = k << 1;
	}

	tmp = readl(SYS_EFUSE_CFG(rts_pt));
	set_bit(EFUSE_LOAD, (unsigned long *)&tmp);
	set_bit(EFUSE_PGENB, (unsigned long *)&tmp);
	set_bit(EFUSE_CSB, (unsigned long *)&tmp);
	writel(tmp, SYS_EFUSE_CFG(rts_pt));

	clear_bit(EFUSE_PS, (unsigned long *)&tmp);
	writel(tmp, SYS_EFUSE_CFG(rts_pt));

	set_bit(EFUSE_PD, (unsigned long *)&tmp);
	writel(tmp, SYS_EFUSE_CFG(rts_pt));

	clear_bit(PROGRAM_POW, (unsigned long *)&tmp);
	writel(tmp, SYS_EFUSE_CFG(rts_pt));

	udelay(10);
	writel(EFUSE_DEFAUTL_V, SYS_EFUSE_CFG(rts_pt));
}

void rts_efuse_load_aes(void)
{
	u32 tmp;
	u32 i;

	if (!g_rts)
		return;

	efuse_init(PGM_CNT, g_rts);

	tmp = readl(SYS_EFUSE_CFG(g_rts));

	clear_bit(EFUSE_PD, (unsigned long *)&tmp);
	writel(tmp, SYS_EFUSE_CFG(g_rts));

	clear_bit(EFUSE_CSB, (unsigned long *)&tmp);
	writel(tmp, SYS_EFUSE_CFG(g_rts));

	clear_bit(EFUSE_PS, (unsigned long *)&tmp);
	writel(tmp, SYS_EFUSE_CFG(g_rts));

	set_bit(EFUSE_LOAD, (unsigned long *)&tmp);
	set_bit(EFUSE_PGENB, (unsigned long *)&tmp);
	writel(tmp, SYS_EFUSE_CFG(g_rts));

	for (i = 0; i < 4; i++) {
		tmp &= 0xffffff00;
		tmp |= (i & 0xff);
		writel(tmp, SYS_EFUSE_CFG(g_rts));

		set_bit(EFUSE_STROBE, (unsigned long *)&tmp);
		writel(tmp, SYS_EFUSE_CFG(g_rts));

		writel((readl(SYS_EFUSE_RDATA(g_rts))),
					(SYS_EFUSE_DATA0(g_rts) + i));

		clear_bit(EFUSE_STROBE, (unsigned long *)&tmp);
		writel(tmp, SYS_EFUSE_CFG(g_rts));

		set_bit(LOAD_AES_KEY, (unsigned long *)&tmp);
		writel(tmp, SYS_EFUSE_CFG(g_rts));

		clear_bit(LOAD_AES_KEY, (unsigned long *)&tmp);
		writel(tmp, SYS_EFUSE_CFG(g_rts));
	}

	tmp = readl(SYS_EFUSE_CFG(g_rts));

	set_bit(EFUSE_PD, (unsigned long *)&tmp);
	writel(tmp, SYS_EFUSE_CFG(g_rts));

	set_bit(EFUSE_CSB, (unsigned long *)&tmp);
	writel(tmp, SYS_EFUSE_CFG(g_rts));

	tmp = 1;
	writel(tmp, SYS_EFUSE_CFG(g_rts));

	udelay(10);
	writel(EFUSE_DEFAUTL_V, SYS_EFUSE_CFG(g_rts));
}
EXPORT_SYMBOL_GPL(rts_efuse_load_aes);

static int rts_reg_read(void *context, unsigned int offset,
		void *val, size_t bytes)
{
	struct rts_efuse *rts = context;
	u8 *buf = val;
	int ret;

	dev_dbg(NULL, "%s: offset = %d, bytes = %d\n", __func__, offset, bytes);

	ret = efuse_check(offset, bytes);
	if (ret)
		return ret;

	while (bytes--) {
		efuse_init(PGM_CNT, rts);
		*buf++ = efuse_read_byte(offset++, rts);
	}

	return 0;
}

static int rts_reg_write(void *context, unsigned int offset,
		void *val, size_t bytes)
{
	struct rts_efuse *rts = context;
	u8 *buf = val;
	int ret;

	dev_dbg(NULL, "%s: offset = %d, bytes = %d\n", __func__, offset, bytes);

	ret = efuse_check(offset, bytes);
	if (ret)
		return ret;

	while (bytes--) {
		efuse_init(PGM_CNT, rts);
		efuse_write_byte(*buf++, offset++, rts);
	}

	return 0;
}

static struct nvmem_config econfig = {
	.name = "rts-efuse",
	.owner = THIS_MODULE,
	.stride = 1,
	.word_size = 1,
	.size = BYTE_NUMS,
};

static int rts_efuse_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nvmem_device *nvmem;
	struct resource *res;
	struct rts_efuse *rts;

	rts = devm_kzalloc(&pdev->dev, sizeof(*rts), GFP_KERNEL);
	if (!rts)
		return -ENOMEM;

	g_rts = rts;
	rts->dev = dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "no memory resource provided");
		return -ENXIO;
	}

	rts->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(rts->regs))
		return PTR_ERR(rts->regs);

	econfig.dev = dev;
	econfig.reg_read = rts_reg_read;
	econfig.reg_write = rts_reg_write;
	econfig.priv = rts;

	nvmem = nvmem_register(&econfig);
	if (IS_ERR(nvmem))
		return PTR_ERR(nvmem);

	platform_set_drvdata(pdev, nvmem);

	return 0;
}

static int rts_efuse_remove(struct platform_device *pdev)
{
	struct nvmem_device *nvmem = platform_get_drvdata(pdev);

	return nvmem_unregister(nvmem);
}

static const struct of_device_id rts_efuse_dt_ids[] = {
	{ .compatible = "realtek,rts3903-efuse", },
	{ /* sentinel */ },
};

static struct platform_driver rts_efuse_driver = {
	.probe		= rts_efuse_probe,
	.remove		= rts_efuse_remove,
	.driver		= {
		.name = "rts-efuse",
		.of_match_table = of_match_ptr(rts_efuse_dt_ids),
	},
};

module_platform_driver(rts_efuse_driver);
MODULE_AUTHOR("Wind Han <wind_han@realsil.com.cn>");
MODULE_DESCRIPTION("Realtek EFUSE driver");
MODULE_LICENSE("GPL");
