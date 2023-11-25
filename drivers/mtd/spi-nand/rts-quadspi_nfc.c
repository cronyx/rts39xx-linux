/*
 * Driver for Realtek IPCam RTS39XX SPI Nand Flash Controller
 *
 * Copyright (C) 2016 Jim Cao, Realtek <jim_cao@realsil.com.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License verqspiion 2 as
 * published by the Free Software Foundation.
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/clk.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/platform_device.h>
#include <linux/sizes.h>
#include <linux/of_device.h>
#include <linux/reset.h>


/* SPI NAND commands */
#define SPI_NAND_WRITE_ENABLE		0x06
#define SPI_NAND_WRITE_DISABLE		0x04
#define SPI_NAND_GET_FEATURE		0x0f
#define SPI_NAND_SET_FEATURE		0x1f
#define SPI_NAND_PAGE_READ		0x13
#define SPI_NAND_READ_CACHE		0x03
#define SPI_NAND_FAST_READ_CACHE	0x0b
#define SPI_NAND_READ_CACHE_X2		0x3b
#define SPI_NAND_READ_CACHE_X4		0x6b
#define SPI_NAND_READ_CACHE_DUAL_IO	0xbb
#define SPI_NAND_READ_CACHE_QUAD_IO	0xeb
#define SPI_NAND_READ_ID		0x9f
#define SPI_NAND_PROGRAM_LOAD		0x02
#define SPI_NAND_PROGRAM_LOAD4		0x32
#define SPI_NAND_PROGRAM_EXEC		0x10
#define SPI_NAND_PROGRAM_LOAD_RANDOM	0x84
#define SPI_NAND_PROGRAM_LOAD_RANDOM4	0xc4
#define SPI_NAND_BLOCK_ERASE		0xd8
#define SPI_NAND_RESET			0xff
#define SPI_NAND_SW_DIE_SELECT		0xc2	/* only for WINBOND 2Gb Nand */

#define SPI_NAND_READID_LEN		8

/* Registers common to all devices */
#define SPI_NAND_LOCK_REG		0xa0
#define SPI_NAND_PROT_UNLOCK_ALL	0x0

#define SPI_NAND_FEATURE_REG		0xb0
#define SPI_NAND_QUAD_EN		BIT(0)
#define SPI_NAND_ECC_EN			BIT(4)

#define SPI_NAND_WINBOND_BUF		BIT(3)

#define SPI_NAND_STATUS_REG		0xc0
#define SPI_NAND_STATUS_REG_ECC_MASK	0x3
#define SPI_NAND_STATUS_REG_ECC_SHIFT	4
#define SPI_NAND_STATUS_REG_PROG_FAIL	BIT(3)
#define SPI_NAND_STATUS_REG_ERASE_FAIL	BIT(2)
#define SPI_NAND_STATUS_REG_WREN	BIT(1)
#define SPI_NAND_STATUS_REG_BUSY	BIT(0)

#define SPI_NAND_DUAL_MODE		0x00100000
#define SPI_NAND_QUAD_MODE		0x00200000

#define PAGE_PLANE_SELECT		6
#define COLUMN_PLANE_SELECT		12

enum op_mode {
	SPI_OP_1_1_1 = 0,
	SPI_OP_1_1_2,
	SPI_OP_1_1_4,
};

struct spi_nand_device_cmd {
	u8 cmd;
	u32 addr;
	u8 addr_pins;
	u8 addr_width;
	u8 mode;
	u8 mode_pins;
	u32 mode_length;
	u32 dummy_cycles;
};

struct spi_nand {
	struct mtd_info			*mtd;
	struct nand_chip		nand_chip;
	struct spi_nand_device_cmd	cmd;
	struct device			*dev;
	const char			*name;

	union {
		u8	id_data[8];
		struct { u8 maf_id; u8 dev_id0; u8 dev_id1; };
	};

	u8				read_cache_opcode;
	u8				prog_load_opcode;
	u8				read_cache_dummy;
	enum op_mode			read_mode;
	enum op_mode			prog_mode;

	size_t				buf_size;
	size_t				buf_start;
	unsigned int			page_addr;

	int (*reset)(struct spi_nand *snand);
	int (*read_id)(struct spi_nand *snand);

	int (*write_disable)(struct spi_nand *snand);
	int (*write_enable)(struct spi_nand *snand);

	int (*read_reg)(struct spi_nand *snand, u8 opcode, u8 *buf);
	int (*write_reg)(struct spi_nand *snand, u8 opcode, u8 *buf);

	int (*store_cache)(struct spi_nand *snand, unsigned int page_offset,
			size_t length);
	int (*write_page)(struct spi_nand *snand, unsigned int page_addr);
	int (*load_page)(struct spi_nand *snand, unsigned int page_addr);
	int (*read_cache)(struct spi_nand *snand, unsigned int page_offset,
			size_t length);
	int (*block_erase)(struct spi_nand *snand, unsigned int page_addr);

	int (*die_select)(struct spi_nand *snand, int chipnr);

	void *priv;
};

/* QuadSPI NFC register offsets */
#define CTRLR0				0x0000
#define SSIENR				0x0004
#define BAUDR				0x0008
#define URESET				0x000C
#define UCMD				0x0010
#define UADDR				0x0014
#define ADDR_LEN			0x0018
#define DATA_LEN			0x001c
#define PAGE_READ_ADDR_LEN		0x0020
#define READ_DUMMY_LEN			0x0024
#define PROGRAM_DUMMY_LEN		0x0028
#define GET_FEATURE_DUMMY_LEN		0x002c
#define PAGE_READ_CMD			0x0040
#define RANDOM_READ_CMD			0x0044
#define GET_FEATURE_CMD			0x0048
#define GET_FEATURE_ADDR		0x004c
#define GET_FEATURE_STOP_VALUE		0x0050
#define GET_FEATURE_STOP_MASK		0x0054
#define GET_FEATURE_VALUE		0x0058
#define AUTO_GET_FEATURE_CTRL		0x005c
#define AUTO_READ_TYPE			0x0060
#define PLANE_CTRL			0x0064
#define SR				0x0068
#define ISR				0x006c
#define IMR				0x0070
#define ISR_CR				0x0074
#define LA_STATUS			0x0080
#define PA0				0x0084
#define PA1				0x0088
#define PAX				0x008c
#define LAX				0x0090
#define CURRENT_PAGE			0x0094
#define MAIN_STATE			0x00a0
#define LA_FIND_STATE			0x00a4
#define AUTO_READ_STATE			0x00a8
#define PROTOCOL_STATE			0x00ac
#define GP_STATE			0x00b0
#define ECC_WRITE_CTRL			0x00c0
#define ECC_READ_CTRL			0x00c8
#define ECC_READ_STATUS			0x00cc
#define ECC_OOB_MASK0			0x00d0
#define ECC_OOB_MASK1			0x00d4
#define DEBUG0_ADDR			0x00f0
#define DEBUG1_ADDR			0x00f4
#define DA				0x0100
#define DV				0x0104
#define DATA_FIFO			0x1000 /* 0x1000 ~ 0x1840 */

/* Bit fields in CTRLR0 */
#define SCPH				0
#define SCPOL				1
#define TMOD_OFFSET			2
#define TMOD_MASK			3
#define TRANSMIT_MODE			0
#define RECEIVE_MODE			3
#define ADDR_CH_OFFSET			4
#define ADDR_CH_MASK			3
#define DATA_CH_OFFSET			6
#define DATA_CH_MASK			3

/* Bit fields in BAUDR */
#define SCKDV_WIDTH			16
#define SCKDV_MASK			((1 << 12) - 1)

/* Bit fields in SR */
#define READY				0

/* Bit fields in READ_DUMMY_LEN */
#define READ_DUMMY_LEN_MASK		((1 << 11) - 1)

/* Bit fileds in ECC_READ_CTRL */
#define ECC_RD_EN			0
#define ECC_RD_EN_MASK			(0x1)
#define ECC_STATUS_CLR			1
#define ECC_ERR_THRES			2

/* Bit fields in ECC_READ_STATUS */
#define ECC_STATUS			0
#define ECC_ERROR_CNT			1
#define ECC_ERROR_CNT_MASK		(0xf << 1)

#define BCH12_MAX_ERROR			12	/* upto 12 bit correctable */

/* rts qspi nfc */
struct rts_qspi_nfc {
	struct spi_nand		spi_nand;
	struct platform_device	*pdev;
	int			irq;
	void __iomem		*regs;
	phys_addr_t		phybase;

	struct clk		*clk;
	u32			spiclk_hz;
	u32			max_speed_hz;
	u32			min_speed_hz;

	struct spi_board_info	*bi;
};

static int ecc_write_disable;

#define has_die_select(s)				\
		((s->maf_id == NAND_MFR_WINBOND) &&	\
		(s->dev_id0 == 0xab) &&			\
		(s->dev_id1 == 0x21))			\

static struct nand_flash_dev spi_nand_flash_ids[] = {
	{
		.name = "MICRON SPI NAND 128MiB 3,3V",
		.id = { NAND_MFR_MICRON, 0x12 },
		.chipsize = 128,
		.pagesize = SZ_2K,
		.erasesize = SZ_128K,
		.id_len = 2,
		.oobsize = 64,
	},
	{
		.name = "MICRON SPI NAND 256MiB 3,3V",
		.id = { NAND_MFR_MICRON, 0x22 },
		.chipsize = 256,
		.pagesize = SZ_2K,
		.erasesize = SZ_128K,
		.id_len = 2,
		.oobsize = 64,
	},
	{
		.name = "MICRON SPI NAND 512MiB 3,3V",
		.id = { NAND_MFR_MICRON, 0x32 },
		.chipsize = 512,
		.pagesize = SZ_2K,
		.erasesize = SZ_128K,
		.id_len = 2,
		.oobsize = 64,
	},
	{
		.name = "MACRONIX SPI NAND 128MiB 3,3V",
		.id = { NAND_MFR_MACRONIX, 0x12 },
		.chipsize = 128,
		.pagesize = SZ_2K,
		.erasesize = SZ_128K,
		.id_len = 2,
		.oobsize = 64,
		.options = SPI_NAND_QUAD_MODE,
	},
	{
		.name = "MACRONIX SPI NAND 256MiB 3,3V",
		.id = { NAND_MFR_MACRONIX, 0x22 },
		.chipsize = 256,
		.pagesize = SZ_2K,
		.erasesize = SZ_128K,
		.id_len = 2,
		.oobsize = 64,
		.options = SPI_NAND_QUAD_MODE,
	},
	{
		.name = "WINBOND SPI NAND 128MiB 3,3V",
		.id = { NAND_MFR_WINBOND, 0xaa, 0x21 },
		.chipsize = 128,
		.pagesize = SZ_2K,
		.erasesize = SZ_128K,
		.id_len = 3,
		.oobsize = 64,
		.options = SPI_NAND_QUAD_MODE,
	},
	{
		.name = "WINBOND SPI NAND 128MiB*2 3,3V",
		.id = { NAND_MFR_WINBOND, 0xab, 0x21 },
		/* We pretend there are 2 chips, each one has 128MiB */
		.chipsize = 128,
		.pagesize = SZ_2K,
		.erasesize = SZ_128K,
		.id_len = 3,
		.oobsize = 64,
		.options = SPI_NAND_QUAD_MODE,
	},
	{
		.name = "GIGADEVICE SPI NAND 128MiB 3,3V",
		.id = { NAND_MFR_GIGADEVICE, 0xd1 },
		.chipsize = 128,
		.pagesize = SZ_2K,
		.erasesize = SZ_128K,
		.id_len = 2,
		.oobsize = 64,
		.options = SPI_NAND_QUAD_MODE,
	},
	{
		.name = "GIGADEVICE SPI NAND 256MiB 3,3V",
		.id = { NAND_MFR_GIGADEVICE, 0xd2 },
		.chipsize = 256,
		.pagesize = SZ_2K,
		.erasesize = SZ_128K,
		.id_len = 2,
		.oobsize = 64,
		.options = SPI_NAND_QUAD_MODE,
	},
	{NULL},
};

static int rts_nfc_ooblayout_ecc(struct mtd_info *mtd, int section,
				 struct mtd_oob_region *oobregion)
{
	if (section)
		return -ERANGE;

	oobregion->offset = 41;
	oobregion->length = 23;

	return 0;
}

static int rts_nfc_ooblayout_free(struct mtd_info *mtd, int section,
				  struct mtd_oob_region *oobregion)
{
	if (section)
		return -ERANGE;

	oobregion->offset = 2;
	oobregion->length = 39;

	return 0;
}

static const struct mtd_ooblayout_ops rts_nfc_ooblayout_ops = {
	.ecc = rts_nfc_ooblayout_ecc,
	.free = rts_nfc_ooblayout_free,
};

static inline u32 rts_readl(struct rts_qspi_nfc *rqspi_nfc, u32 reg)
{
	return readl(rqspi_nfc->regs + reg);
}

static inline u16 rts_readw(struct rts_qspi_nfc *rqspi_nfc, u32 reg)
{
	return readw(rqspi_nfc->regs + reg);
}

static inline u8 rts_readb(struct rts_qspi_nfc *rqspi_nfc, u32 reg)
{
	return readb(rqspi_nfc->regs + reg);
}

static inline void rts_writel(struct rts_qspi_nfc *rqspi_nfc, u32 reg, u32 val)
{
	writel(val, rqspi_nfc->regs + reg);
}

static inline void rts_writew(struct rts_qspi_nfc *rqspi_nfc, u32 reg, u16 val)
{
	writew(val, rqspi_nfc->regs + reg);
}

static inline void rts_writeb(struct rts_qspi_nfc *rqspi_nfc, u32 reg, u8 val)
{
	writeb(val, rqspi_nfc->regs + reg);
}

static int rts_qspi_nfc_controller_ready(struct rts_qspi_nfc *rqspi_nfc)
{

	u32 cnt;
	u32 reg;

	for (cnt = 0; cnt < 100000; cnt++) {
		reg = rts_readl(rqspi_nfc, SR);
		if (reg & BIT(READY))
			return 0;
		udelay(1);
	}
	return -EBUSY;
}

static inline int rts_qspi_nfc_set_dummy(struct rts_qspi_nfc *rqspi_nfc,
					u32 cycle)
{
#define INTERNAL_DUMMY 1
	u32 baud;
	u32 dummy;

	if (cycle == 0) {
		dummy = INTERNAL_DUMMY;
	} else {
		baud = rts_readl(rqspi_nfc, BAUDR);
		dummy = baud * cycle * 2 + INTERNAL_DUMMY;
	}

	if (dummy > READ_DUMMY_LEN_MASK)
		return -EINVAL;

	rts_writel(rqspi_nfc, READ_DUMMY_LEN, dummy);

	return 0;
}

static int spi_nand_send_command(struct spi_nand *snand,
					struct spi_nand_device_cmd *cmd)
{
	struct rts_qspi_nfc *rqspi_nfc = snand->priv;
	int ret;
	u32 reg;

	ret = rts_qspi_nfc_set_dummy(rqspi_nfc, cmd->dummy_cycles);
	if (ret)
		goto FAIL;

	reg = rts_readl(rqspi_nfc, CTRLR0);
	reg &= ~((TMOD_MASK << TMOD_OFFSET) |
		(ADDR_CH_MASK << ADDR_CH_OFFSET) |
		(DATA_CH_MASK << DATA_CH_OFFSET));
	reg |= (((u32)(cmd->mode) << TMOD_OFFSET) |
		((u32)(cmd->addr_pins >> 1) << ADDR_CH_OFFSET) |
		((u32)(cmd->mode_pins >> 1) << DATA_CH_OFFSET));
	rts_writel(rqspi_nfc, CTRLR0, reg);

	rts_writel(rqspi_nfc, UCMD, cmd->cmd);
	rts_writel(rqspi_nfc, UADDR, cmd->addr);
	rts_writel(rqspi_nfc, ADDR_LEN, cmd->addr_width);
	rts_writel(rqspi_nfc, DATA_LEN, cmd->mode_length);

	rts_writel(rqspi_nfc, SSIENR, 1);

	ret = rts_qspi_nfc_controller_ready(rqspi_nfc);
	if (ret) {
		dev_err(snand->dev, "controller busy\n");
		goto FAIL;
	}

	rts_writel(rqspi_nfc, SSIENR, 0);

	return 0;
FAIL:
	dev_err(snand->dev, "%s() failed, ret = %d\n", __func__, ret);
	return ret;
}

static int rts_qspi_nfc_reset(struct spi_nand *snand)
{
	struct spi_nand_device_cmd *cmd = &snand->cmd;

	cmd->cmd = SPI_NAND_RESET;
	cmd->addr = 0;
	cmd->addr_width = 0;
	cmd->addr_pins = 1;
	cmd->mode = TRANSMIT_MODE;
	cmd->mode_pins = 1;
	cmd->mode_length = 0;
	cmd->dummy_cycles = 0;

	return spi_nand_send_command(snand, cmd);
}

static int rts_qspi_nfc_read_id(struct spi_nand *snand)
{
	struct spi_nand_device_cmd *cmd = &snand->cmd;

	cmd->cmd = SPI_NAND_READ_ID;
	cmd->addr = 0;
	cmd->addr_width = 1;
	cmd->addr_pins = 1;
	cmd->mode = RECEIVE_MODE;
	cmd->mode_pins = 1;
	cmd->mode_length = SPI_NAND_READID_LEN;
	cmd->dummy_cycles = 0;

	return spi_nand_send_command(snand, cmd);
}

static int rts_qspi_nfc_load_page(struct spi_nand *snand,
		unsigned int page_addr)
{
	struct spi_nand_device_cmd *cmd = &snand->cmd;

	cmd->cmd = SPI_NAND_PAGE_READ;
	cmd->addr = page_addr;
	cmd->addr_width = 3;
	cmd->addr_pins = 1;
	cmd->mode = TRANSMIT_MODE;
	cmd->mode_pins = 1;
	cmd->mode_length = 0;
	cmd->dummy_cycles = 0;

	return spi_nand_send_command(snand, cmd);
}

static int rts_qspi_nfc_program_page(struct spi_nand *snand,
		unsigned int page_addr)
{
	struct spi_nand_device_cmd *cmd = &snand->cmd;

	cmd->cmd = SPI_NAND_PROGRAM_EXEC;
	cmd->addr = page_addr;
	cmd->addr_width = 3;
	cmd->addr_pins = 1;
	cmd->mode = TRANSMIT_MODE;
	cmd->mode_pins = 1;
	cmd->mode_length = 0;
	cmd->dummy_cycles = 0;

	return spi_nand_send_command(snand, cmd);
}

static int rts_qspi_nfc_block_erase(struct spi_nand *snand,
		unsigned int page_addr)
{
	struct spi_nand_device_cmd *cmd = &snand->cmd;

	cmd->cmd = SPI_NAND_BLOCK_ERASE;
	cmd->addr = page_addr;
	cmd->addr_width = 3;
	cmd->addr_pins = 1;
	cmd->mode = TRANSMIT_MODE;
	cmd->mode_pins = 1;
	cmd->mode_length = 0;
	cmd->dummy_cycles = 0;

	return spi_nand_send_command(snand, cmd);
}

static int rts_qspi_nfc_write_enable(struct spi_nand *snand)
{
	struct spi_nand_device_cmd *cmd = &snand->cmd;

	cmd->cmd = SPI_NAND_WRITE_ENABLE;
	cmd->addr = 0;
	cmd->addr_width = 0;
	cmd->addr_pins = 1;
	cmd->mode = TRANSMIT_MODE;
	cmd->mode_pins = 1;
	cmd->mode_length = 0;
	cmd->dummy_cycles = 0;

	return spi_nand_send_command(snand, cmd);
}

static int rts_qspi_nfc_write_disable(struct spi_nand *snand)
{
	struct spi_nand_device_cmd *cmd = &snand->cmd;

	cmd->cmd = SPI_NAND_WRITE_DISABLE;
	cmd->addr = 0;
	cmd->addr_width = 0;
	cmd->addr_pins = 1;
	cmd->mode = TRANSMIT_MODE;
	cmd->mode_pins = 1;
	cmd->mode_length = 0;
	cmd->dummy_cycles = 0;

	return spi_nand_send_command(snand, cmd);
}

static int rts_qspi_nfc_read_reg(struct spi_nand *snand, u8 opcode, u8 *buf)
{
	struct rts_qspi_nfc *rqspi_nfc = snand->priv;
	struct spi_nand_device_cmd *cmd = &snand->cmd;
	int ret;

	cmd->cmd = SPI_NAND_GET_FEATURE;
	cmd->addr = opcode;
	cmd->addr_width = 1;
	cmd->addr_pins = 1;
	cmd->mode = RECEIVE_MODE;
	cmd->mode_pins = 1;
	cmd->mode_length = 1;
	cmd->dummy_cycles = 0;

	ret = spi_nand_send_command(snand, cmd);
	if (ret)
		return ret;

	*buf = rts_readb(rqspi_nfc, DATA_FIFO);

	return ret;
}

static int rts_qspi_nfc_write_reg(struct spi_nand *snand,
		u8 opcode, u8 *buf)
{
	struct rts_qspi_nfc *rqspi_nfc = snand->priv;
	struct spi_nand_device_cmd *cmd = &snand->cmd;

	cmd->cmd = SPI_NAND_SET_FEATURE;
	cmd->addr = opcode;
	cmd->addr_width = 1;
	cmd->addr_pins = 1;
	cmd->mode = TRANSMIT_MODE;
	cmd->mode_pins = 1;
	cmd->mode_length = 1;
	cmd->dummy_cycles = 0;

	rts_writeb(rqspi_nfc, DATA_FIFO, *buf);

	return spi_nand_send_command(snand, cmd);
}

/* read from flash chip cache to nfc internal buffer */
static int rts_qspi_nfc_read_cache(struct spi_nand *snand,
		unsigned int page_offset, size_t length)
{
	struct spi_nand_device_cmd *cmd = &snand->cmd;
	int addr_pins, mode_pins;

	switch (snand->read_mode) {
	case SPI_OP_1_1_1:
		addr_pins = 1; mode_pins = 1;
		break;
	case SPI_OP_1_1_2:
		addr_pins = 1; mode_pins = 2;
		break;
	case SPI_OP_1_1_4:
		addr_pins = 1; mode_pins = 4;
		break;
	default:
		dev_err(snand->dev, "unsupported read op mode!\n");
		return -EINVAL;
	}

	cmd->cmd = snand->read_cache_opcode;
	cmd->addr = page_offset;
	cmd->addr_width = 2;
	cmd->addr_pins = addr_pins;
	cmd->mode = RECEIVE_MODE;
	cmd->mode_pins = mode_pins;
	cmd->mode_length = length;
	cmd->dummy_cycles = snand->read_cache_dummy * 8;

	return spi_nand_send_command(snand, cmd);
}

static int rts_qspi_nfc_store_cache(struct spi_nand *snand,
		unsigned int page_offset, size_t length)
{
	struct rts_qspi_nfc *rqspi_nfc = snand->priv;
	struct spi_nand_device_cmd *cmd = &snand->cmd;
	int addr_pins, mode_pins;
	int ret;

	switch (snand->prog_mode) {
	case SPI_OP_1_1_1:
		addr_pins = 1; mode_pins = 1;
		break;
	case SPI_OP_1_1_4:
		addr_pins = 1; mode_pins = 4;
		break;
	default:
		dev_err(snand->dev, "unsupported programe op mode!\n");
		return -EINVAL;
	}

	cmd->cmd = snand->prog_load_opcode;
	cmd->addr = page_offset;
	cmd->addr_width = 2;
	cmd->addr_pins = addr_pins;
	cmd->mode = TRANSMIT_MODE;
	cmd->mode_pins = mode_pins;
	cmd->mode_length = length;
	cmd->dummy_cycles = 0;

	/* Turn off ECC write for write_page_raw */
	if (ecc_write_disable)
		rts_writel(rqspi_nfc, ECC_WRITE_CTRL, 0);

	ret = spi_nand_send_command(snand, cmd);

	/* Re-enable ecc write for write_page_raw */
	if (ecc_write_disable) {
		ecc_write_disable = 0;
		rts_writel(rqspi_nfc, ECC_WRITE_CTRL, 1);
	}

	return ret;
}

static int winbond_die_select(struct spi_nand *snand, int chipnr)
{
	struct spi_nand_device_cmd *cmd = &snand->cmd;

	cmd->cmd = SPI_NAND_SW_DIE_SELECT;
	cmd->addr = chipnr;
	cmd->addr_width = 1;
	cmd->addr_pins = 1;
	cmd->mode = TRANSMIT_MODE;
	cmd->mode_pins = 1;
	cmd->mode_length = 0;
	cmd->dummy_cycles = 0;

	return spi_nand_send_command(snand, cmd);
}

static uint8_t rts_qspi_nfc_read_byte(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct spi_nand *snand = chip->priv;
	uint8_t data;

	data = readb(chip->IO_ADDR_R + snand->buf_start);
	snand->buf_start++;

	return data;
}

static u16 rts_qspi_nfc_read_word(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct spi_nand *snand = chip->priv;
	u16 data;

	data = readb(chip->IO_ADDR_R + snand->buf_start);
	snand->buf_start += 2;

	return data;
}

static void rts_qspi_nfc_read_buf(struct mtd_info *mtd, u8 *buf, int len)
{
	struct nand_chip *chip = mtd->priv;
	struct spi_nand *snand = chip->priv;

	/* TODO:if buffer too large, try DMA copy */
	memcpy_fromio(buf, chip->IO_ADDR_R + snand->buf_start, len);

	dev_dbg(snand->dev, "Copy 0x%x bytes from position 0x%x in read buffer.\n",
		  len, snand->buf_start);
	snand->buf_start += len;
}

static void rts_qspi_nfc_write_buf(struct mtd_info *mtd,
	const uint8_t *buf, int len)
{
	struct nand_chip *chip = mtd->priv;
	struct spi_nand *snand = chip->priv;

	/* TODO:if buffer too large, try DMA copy */

	memcpy_toio(chip->IO_ADDR_W + snand->buf_start, buf, len);
	snand->buf_start += len;

	dev_dbg(snand->dev, "Copy 0x%x bytes to write buffer. datalen 0x%x\n",
		len, snand->buf_start);
}

static int rts_qspi_nfc_read_page_ecc(struct mtd_info *mtd,
	struct nand_chip *chip, uint8_t *buf, int oob_required, int page)
{
	struct spi_nand *snand = chip->priv;
	struct rts_qspi_nfc *rqspi_nfc = snand->priv;
	u32 ecc_read_ctrl_reg;
	u32 reg;

	reg = rts_readl(rqspi_nfc, ECC_READ_STATUS);

	chip->read_buf(mtd, buf, mtd->writesize);
	chip->read_buf(mtd, chip->oob_poi, mtd->oobsize);

	/* update statistics */
	if (reg & BIT(ECC_STATUS))
		mtd->ecc_stats.failed++;
	else
		mtd->ecc_stats.corrected +=
			(reg & ECC_ERROR_CNT_MASK) >> ECC_ERROR_CNT;

	ecc_read_ctrl_reg = rts_readl(rqspi_nfc, ECC_READ_CTRL);
	ecc_read_ctrl_reg |= BIT(ECC_STATUS_CLR);
	rts_writel(rqspi_nfc, ECC_READ_CTRL, ecc_read_ctrl_reg);

	return (reg & ECC_ERROR_CNT_MASK) >> ECC_ERROR_CNT;
}

static int rts_qspi_nfc_read_page_raw(struct mtd_info *mtd,
	struct nand_chip *chip, uint8_t *buf, int oob_required, int page)
{
	struct spi_nand *snand = chip->priv;
	struct rts_qspi_nfc *rqspi_nfc = snand->priv;
	u32 reg;

	/* save ecc engine flag*/
	reg = rts_readl(rqspi_nfc, ECC_READ_CTRL);

	/* disable ecc read engine */
	rts_writel(rqspi_nfc, ECC_READ_CTRL, reg & ~ECC_RD_EN_MASK);

	/* read once again with ecc disabled */
	chip->cmdfunc(mtd, NAND_CMD_READ0, 0x00, page);
	chip->read_buf(mtd, buf, mtd->writesize);
	if (oob_required)
		chip->read_buf(mtd, chip->oob_poi, mtd->oobsize);

	/* restore ecc engine status*/
	rts_writel(rqspi_nfc, ECC_READ_CTRL, reg);

	return 0;
}

static int rts_qspi_nfc_write_page_ecc(struct mtd_info *mtd,
		struct nand_chip *chip, const uint8_t *buf, int oob_required,
		int page)
{
	/* ecc bit is filled by hardware while transmitting
	 * so we can just fill the FIFO interface
	 */
	chip->write_buf(mtd, buf, mtd->writesize);
	chip->write_buf(mtd, chip->oob_poi, mtd->oobsize);

	return 0;
}

static int rts_qspi_nfc_write_page_raw(struct mtd_info *mtd,
		struct nand_chip *chip, const uint8_t *buf, int oob_required,
		int page)
{
	/* disable ecc write */
	ecc_write_disable = 1;

	chip->write_buf(mtd, buf, mtd->writesize);
	chip->write_buf(mtd, chip->oob_poi, mtd->oobsize);

	return 0;
}

/*
 * Read the OOB data from the device without ECC
 */
static int rts_qspi_nfc_read_oob_raw(struct mtd_info *mtd,
		struct nand_chip *chip, int page)
{
	struct spi_nand *snand = chip->priv;
	struct rts_qspi_nfc *rqspi_nfc = snand->priv;
	u32 reg;

	/* save ecc engine flag*/
	reg = rts_readl(rqspi_nfc, ECC_READ_CTRL);

	/* disable ecc read engine */
	rts_writel(rqspi_nfc, ECC_READ_CTRL, reg & ~ECC_RD_EN_MASK);

	/* read with ecc disabled */
	chip->cmdfunc(mtd, NAND_CMD_READOOB, 0, page);
	chip->read_buf(mtd, chip->oob_poi, mtd->oobsize);

	/* restore ecc engine status*/
	rts_writel(rqspi_nfc, ECC_READ_CTRL, reg);

	return 0;
}

static int rts_qspi_nfc_write_oob_ecc(struct mtd_info *mtd,
		struct nand_chip *chip, int page)
{
	int status = 0;
	const uint8_t *buf = chip->oob_poi;
	int length = mtd->oobsize;

	ecc_write_disable = 1;

	chip->cmdfunc(mtd, NAND_CMD_SEQIN, 0, page);
	chip->write_buf(mtd, buf, length);

	/* Send command to program the OOB data */
	chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, mtd->writesize, -1);

	status = chip->waitfunc(mtd, chip);

	return status & NAND_STATUS_FAIL ? -EIO : 0;
}

/*
 * Wait until the status register busy bit is cleared.
 * Returns a negatie errno on error or time out, and a non-negative status
 * value if the device is ready.
 */
static int spi_nand_wait_till_ready(struct spi_nand *snand)
{
	unsigned long deadline = jiffies + msecs_to_jiffies(100);
	bool timeout = false;
	int ret;

	/*
	 * Perhaps we should set a different timeout for each
	 * operation (reset, read, write, erase).
	 */
	while (!timeout) {
		u8 reg;

		if (time_after_eq(jiffies, deadline))
			timeout = true;

		ret = snand->read_reg(snand, SPI_NAND_STATUS_REG, &reg);
		if (ret < 0) {
			dev_err(snand->dev, "error reading status register\n");
			return ret;
		}

		if (!(reg & SPI_NAND_STATUS_REG_BUSY))
			return reg;

		cond_resched();
	}

	dev_err(snand->dev, "operation timed out\n");

	return -ETIMEDOUT;
}

static int spi_nand_write(struct spi_nand *snand, int page_offset, int length)
{
	int ret, maf_id;

	ret = snand->write_enable(snand);
	if (ret < 0) {
		dev_err(snand->dev, "write enable command failed\n");
		return ret;
	}

	/* For MACRONIX 2 Gb nand, there is a plane-select bit
	 * in the 12th bit of address byte. For other nand,
	 * just ignore this bit.
	 */
	maf_id = snand->id_data[0];
	if (maf_id == NAND_MFR_MACRONIX)
		page_offset |= (((snand->page_addr >> PAGE_PLANE_SELECT)
			& 0x01) << COLUMN_PLANE_SELECT);

	/* Force nfc to write whole page to flash chip cache to do ecc ops */
	ret = snand->store_cache(snand, page_offset, length);
	if (ret < 0) {
		dev_err(snand->dev, "error %d storing page 0x%x to cache\n",
			ret, snand->page_addr);
		return ret;
	}

	/* Get page from the device cache into our internal buffer */
	ret = snand->write_page(snand, snand->page_addr);
	if (ret < 0) {
		dev_err(snand->dev, "error %d reading page 0x%x from cache\n",
			ret, snand->page_addr);
		return ret;
	}

	return 0;
}

static int spi_nand_read_page(struct spi_nand *snand, unsigned int page_addr,
			      unsigned int page_offset, size_t length)
{
	int ret, maf_id;

	/* Load a page into the cache register */
	ret = snand->load_page(snand, page_addr);
	if (ret < 0) {
		dev_err(snand->dev, "error %d loading page 0x%x to cache\n",
			ret, page_addr);
		return ret;
	}

	ret = spi_nand_wait_till_ready(snand);
	if (ret < 0)
		return ret;

	/* For MACRONIX 2 Gb nand, there is a plane-select bit
	 * in the 12th bit of address byte. For other nand,
	 * just ignore this bit.
	 */
	maf_id = snand->id_data[0];
	if (maf_id == NAND_MFR_MACRONIX)
		page_offset |= (((page_addr >> PAGE_PLANE_SELECT) & 0x01) <<
			COLUMN_PLANE_SELECT);

	/* Get page from the device cache into nand flash controller buffer */
	ret = snand->read_cache(snand, page_offset, length);
	if (ret < 0) {
		dev_err(snand->dev, "error %d reading page 0x%x from cache\n",
			ret, page_addr);
		return ret;
	}
	return 0;
}

static int spi_nand_read_id(struct spi_nand *snand)
{
	int ret;

	ret = snand->read_id(snand);
	if (ret < 0) {
		dev_err(snand->dev, "error %d reading ID\n", ret);
		return ret;
	}
	return 0;
}

static int spi_nand_reset(struct spi_nand *snand)
{
	int ret;

	ret = snand->reset(snand);
	if (ret < 0) {
		dev_err(snand->dev, "reset command failed\n");
		return ret;
	}

	/*
	 * The NAND core won't wait after a device reset, so we need
	 * to do that here.
	 */
	ret = spi_nand_wait_till_ready(snand);
	if (ret < 0)
		return ret;
	return 0;
}

static int spi_nand_status(struct spi_nand *snand)
{
	struct nand_chip *chip = &snand->nand_chip;
	struct mtd_info *mtd = snand->mtd;
	u8 data;
	u8 status;
	int ret;

	ret = snand->read_reg(snand, SPI_NAND_STATUS_REG, &status);
	if (ret < 0) {
		dev_err(snand->dev, "error reading status register\n");
		return ret;
	}

	/* Convert this into standard NAND_STATUS values */
	if (status & SPI_NAND_STATUS_REG_BUSY)
		data = 0;
	else
		data = NAND_STATUS_READY;

	if (status & SPI_NAND_STATUS_REG_PROG_FAIL ||
	    status & SPI_NAND_STATUS_REG_ERASE_FAIL)
		data |= NAND_STATUS_FAIL;

	/*
	 * Since we unlock the entire device at initialization, unconditionally
	 * set the WP bit to indicate it's not protected.
	 */
	data |= NAND_STATUS_WP;
	chip->write_buf(mtd, &data, 1);

	/* work around for nand_core
	 * nand_check_wp(): chip->read_byte pointer increment
	 */
	snand->buf_start = 0;

	return 0;
}

static int spi_nand_erase(struct spi_nand *snand, int page_addr)
{
	int ret;

	ret = snand->write_enable(snand);
	if (ret < 0) {
		dev_err(snand->dev, "write enable command failed\n");
		return ret;
	}

	ret = snand->block_erase(snand, page_addr);
	if (ret < 0) {
		dev_err(snand->dev, "block erase command failed\n");
		return ret;
	}

	return 0;
}

static void spi_nand_select_chip(struct mtd_info *mtd, int chip)
{
	/* We need this to override the default */
}

static void sw_die_select_chip(struct mtd_info *mtd, int chipnr)
{
	struct nand_chip *chip = mtd->priv;
	struct spi_nand *snand = chip->priv;
	int ret;

	ret = snand->die_select(snand, chipnr);
	if (ret < 0) {
		dev_err(snand->dev, "sw die select command failed\n");
		return;
	}
}

static void rts_qspi_nfc_cmdfunc(struct mtd_info *mtd, unsigned int command,
			     int column, int page_addr)
{
	struct nand_chip *chip = mtd->priv;
	struct spi_nand *snand = chip->priv;

	snand->buf_start = 0;

	switch (command) {
	case NAND_CMD_READ0:
		/* force nfc to read whole page to do ecc ops*/
		spi_nand_read_page(snand, page_addr, 0x0,
				mtd->writesize + mtd->oobsize);
		break;
	case NAND_CMD_READOOB:
		spi_nand_read_page(snand, page_addr, mtd->writesize,
				mtd->oobsize);
		break;
	case NAND_CMD_READID:
		spi_nand_read_id(snand);
		break;
	case NAND_CMD_ERASE1:
		spi_nand_erase(snand, page_addr);
		break;
	case NAND_CMD_ERASE2:
		/* There's nothing to do here, as the erase is one-step */
		break;
	case NAND_CMD_SEQIN:
		snand->buf_start = column;
		snand->page_addr = page_addr;
		break;
	case NAND_CMD_PAGEPROG:
		if (column == -1) {
			/* force nfc to write whole page to do ecc ops*/
			spi_nand_write(snand, 0, mtd->writesize + mtd->oobsize);
		} else {
			/* only write oob */
			spi_nand_write(snand, column, mtd->oobsize);
		}
		break;
	case NAND_CMD_STATUS:
		spi_nand_status(snand);
		break;
	case NAND_CMD_RESET:
		spi_nand_reset(snand);
		break;
	default:
		dev_err(&mtd->dev, "unknown command 0x%x\n", command);
	}
}

static int rts_qspi_nfc_waitfunc(struct mtd_info *mtd, struct nand_chip *chip)
{
	struct spi_nand *snand = chip->priv;
	int ret;

	ret = spi_nand_wait_till_ready(snand);

	if (ret < 0) {
		return NAND_STATUS_FAIL;
	} else if (ret & SPI_NAND_STATUS_REG_PROG_FAIL) {
		dev_err(snand->dev, "page program failed\n");
		return NAND_STATUS_FAIL;
	} else if (ret & SPI_NAND_STATUS_REG_ERASE_FAIL) {
		dev_err(snand->dev, "block erase failed\n");
		return NAND_STATUS_FAIL;
	}

	return NAND_STATUS_READY;
}
static int rts_qspi_nfc_setup(struct rts_qspi_nfc *rqspi_nfc)
{
	struct spi_board_info *bi = rqspi_nfc->bi;
	struct platform_device *pdev = rqspi_nfc->pdev;
	u32 reg, baudr;
	u32 speed_hz;

	/* spi mode */
	reg = 0;
	if (bi->mode & SPI_CPOL)
		reg |= BIT(SCPOL);
	if (bi->mode & SPI_CPHA)
		reg |= BIT(SCPH);
	rts_writel(rqspi_nfc, CTRLR0, reg);

	/* Set clock ratio
	 * F(spi_sclk) = F(bus) / (2 * baudr)
	 */
	speed_hz = bi->max_speed_hz;
	if ((speed_hz == 0) || (speed_hz > rqspi_nfc->max_speed_hz)) {
		speed_hz = rqspi_nfc->max_speed_hz;
		 dev_warn(&pdev->dev, "request %d Hz, force to set %d Hz\n",
			bi->max_speed_hz, rqspi_nfc->max_speed_hz);
	}

	if (speed_hz < rqspi_nfc->min_speed_hz) {
		dev_err(&pdev->dev, "requested speed too low %d Hz\n",
			bi->max_speed_hz);
		return -EINVAL;
	}

	baudr = DIV_ROUND_UP(rqspi_nfc->spiclk_hz, speed_hz) / 2;
	if (baudr > SCKDV_MASK) {
		dev_err(&pdev->dev, "invalid baud reg: %08X\n", baudr);
		return -EINVAL;
	}
	rts_writel(rqspi_nfc, BAUDR, baudr);

	rts_writel(rqspi_nfc, SSIENR, 0);	/* Disable controller */
	rts_writel(rqspi_nfc, IMR, 0);		/* Disable all interrupt */

	/* enable ecc read & write */
	reg = (1 << ECC_RD_EN) | BIT(ECC_STATUS_CLR) |
		(BCH12_MAX_ERROR << ECC_ERR_THRES);
	rts_writel(rqspi_nfc, ECC_READ_CTRL, reg);
	rts_writel(rqspi_nfc, ECC_WRITE_CTRL, 1);

	/* mask all the oob bytes expcept ones occupied by ecc */
	rts_writel(rqspi_nfc, ECC_OOB_MASK0, 0xffffffff);
	rts_writel(rqspi_nfc, ECC_OOB_MASK1, 0xffffffff);

	return 0;
}

static int spi_nand_check(struct spi_nand *snand)
{
	if (!snand->dev)
		return -ENODEV;
	if (!snand->read_cache)
		return -ENODEV;
	if (!snand->load_page)
		return -ENODEV;
	if (!snand->store_cache)
		return -ENODEV;
	if (!snand->write_page)
		return -ENODEV;
	if (!snand->write_reg)
		return -ENODEV;
	if (!snand->read_reg)
		return -ENODEV;
	if (!snand->block_erase)
		return -ENODEV;
	if (!snand->reset)
		return -ENODEV;
	if (!snand->write_enable)
		return -ENODEV;
	if (!snand->write_disable)
		return -ENODEV;
	return 0;
}

static int macronix_quad_enable(struct spi_nand *snand)
{
	int ret;
	u8 feature;

	ret = snand->read_reg(snand, SPI_NAND_FEATURE_REG, &feature);
	if (ret < 0) {
		dev_err(snand->dev, "error reading status register\n");
		return ret;
	}

	feature |= SPI_NAND_QUAD_EN;
	ret = snand->write_reg(snand, SPI_NAND_FEATURE_REG, &feature);
	if (ret < 0) {
		dev_err(snand->dev, "error reading status register\n");
		return ret;
	}

	ret = spi_nand_wait_till_ready(snand);
	if (ret < 0)
		return -ETIMEDOUT;

	ret = snand->read_reg(snand, SPI_NAND_FEATURE_REG, &feature);
	if (!(ret >= 0 && (feature & SPI_NAND_QUAD_EN))) {
		dev_err(snand->dev, "Macronix Quad bit not set\n");
		return -EINVAL;
	}

	return 0;
}

static void simple_get_flash_id(struct spi_nand *snand)
{
	struct nand_chip *chip = &snand->nand_chip;
	struct mtd_info *mtd = snand->mtd;
	int i;

	/* Select the device */
	chip->select_chip(mtd, 0);

	/*
	 * Reset the chip, required by some chips (e.g. Micron MT29FxGxxxxx)
	 * after power-up.
	 */
	chip->cmdfunc(mtd, NAND_CMD_RESET, -1, -1);

	/* Send the command for reading device ID */
	chip->cmdfunc(mtd, NAND_CMD_READID, 0x00, -1);

	/* Read entire ID string */
	for (i = 0; i < 8; i++)
		snand->id_data[i] = chip->read_byte(mtd);

}
static int set_quad_mode(struct spi_nand *snand)
{
	int status, maf_id;

	maf_id = snand->id_data[0];

	switch (maf_id) {
	case NAND_MFR_MACRONIX:
	case NAND_MFR_GIGADEVICE:
		status = macronix_quad_enable(snand);
		if (status) {
			dev_err(snand->dev, "Macronix quad-read not enabled\n");
			status = -EIO;
		}
		break;
	case NAND_MFR_WINBOND:
		/* WINBOND spi nand enable quad by default */
		status = 0;
		break;
	default:
		status = -EINVAL;
		dev_err(snand->dev, "UNSUPPORTED SPI NAND ID %X\n", maf_id);
		break;
	}

	return status;
}

static int spi_nand_init_macronix(struct spi_nand *snand)
{
	int ret;
	u8 reg;

	/* disable block protection */
	ret = snand->read_reg(snand, SPI_NAND_LOCK_REG, &reg);
	if (ret < 0) {
		dev_err(snand->dev, "error reading lock register\n");
		return ret;
	}

	reg &= ~(BIT(3) | BIT(4) | BIT(5));
	ret = snand->write_reg(snand, SPI_NAND_LOCK_REG, &reg);
	if (ret < 0) {
		dev_err(snand->dev, "error writing lock register\n");
		return ret;
	}

	ret = spi_nand_wait_till_ready(snand);
	if (ret < 0)
		return -ETIMEDOUT;

	ret = snand->read_reg(snand, SPI_NAND_LOCK_REG, &reg);
	if (ret < 0 || (reg & (BIT(3) | BIT(4) | BIT(5)))) {
		dev_err(snand->dev, "Macronix lock set failed\n");
		return -EINVAL;
	}

	/* disable on-die ecc */
	ret = snand->read_reg(snand, SPI_NAND_FEATURE_REG, &reg);
	if (ret < 0) {
		dev_err(snand->dev, "error reading feature register\n");
		return ret;
	}

	reg &= ~SPI_NAND_ECC_EN;
	ret = snand->write_reg(snand, SPI_NAND_FEATURE_REG, &reg);
	if (ret < 0) {
		dev_err(snand->dev, "error writing feature register\n");
		return ret;
	}

	ret = spi_nand_wait_till_ready(snand);
	if (ret < 0)
		return -ETIMEDOUT;

	ret = snand->read_reg(snand, SPI_NAND_FEATURE_REG, &reg);
	if (ret < 0 || (reg & SPI_NAND_ECC_EN)) {
		dev_err(snand->dev, "Macronix feature set failed\n");
		return -EINVAL;
	}

	return 0;
}

static int spi_nand_init_winbond(struct spi_nand *snand, int maxchips)
{
	struct mtd_info *mtd = snand->mtd;
	struct nand_chip *chip = &snand->nand_chip;
	int i;

	for (i = 0; i < maxchips; i++) {
		int ret;
		u8 reg;

		chip->select_chip(mtd, i);
		/* disable protection */
		reg = 0;
		ret = snand->write_reg(snand, SPI_NAND_LOCK_REG, &reg);
		if (ret < 0) {
			dev_err(snand->dev, "error writing lock register\n");
			return ret;
		}

		ret = spi_nand_wait_till_ready(snand);
		if (ret < 0)
			return -ETIMEDOUT;

		ret = snand->read_reg(snand, SPI_NAND_LOCK_REG, &reg);
		if (ret < 0 || (reg != 0)) {
			dev_err(snand->dev, "Windbond lock set failed\n");
			return -EINVAL;
		}

		/* disable on-die ecc, using buffer mode */
		ret = snand->read_reg(snand, SPI_NAND_FEATURE_REG, &reg);
		if (ret < 0) {
			dev_err(snand->dev, "error reading feature register\n");
			return ret;
		}

		reg &= ~SPI_NAND_ECC_EN;
		reg |= SPI_NAND_WINBOND_BUF;
		ret = snand->write_reg(snand, SPI_NAND_FEATURE_REG, &reg);
		if (ret < 0) {
			dev_err(snand->dev, "error writing feature register\n");
			return ret;
		}

		ret = spi_nand_wait_till_ready(snand);
		if (ret < 0)
			return -ETIMEDOUT;

		ret = snand->read_reg(snand, SPI_NAND_FEATURE_REG, &reg);
		if (ret < 0 || (reg & SPI_NAND_ECC_EN) ||
			!(reg & SPI_NAND_WINBOND_BUF)) {
			dev_err(snand->dev, "Winbond feature set failed\n");
			return -EINVAL;
		}

		chip->select_chip(mtd, -1);
	}

	return 0;
}

static int spi_nand_init_chip(struct spi_nand *snand, int maxchips)
{
	int status, maf_id;

	maf_id = snand->id_data[0];

	switch (maf_id) {
	case NAND_MFR_MACRONIX:
	case NAND_MFR_GIGADEVICE:
		status = spi_nand_init_macronix(snand);
		if (status) {
			dev_err(snand->dev, "Macronix status init failed\n");
			status = -EIO;
		}
		break;
	case NAND_MFR_WINBOND:
		status = spi_nand_init_winbond(snand, maxchips);
		if (status) {
			dev_err(snand->dev, "winbond status init failed\n");
			status = -EIO;
		}
		break;
	default:
		status = -EINVAL;
		dev_err(snand->dev, "UNSUPPORTED SPI NAND ID %X\n", maf_id);
		break;
	}

	return status;
}

static int spi_nand_enable_quad(struct spi_nand *snand, int maxchips)
{
	struct nand_chip *chip = &snand->nand_chip;

	if (chip->options & SPI_NAND_QUAD_MODE) {
		int ret;

		ret = set_quad_mode(snand);
		if (ret) {
			dev_err(snand->dev, "quad mode not supported\n");
			return ret;
		}
		snand->read_mode = SPI_OP_1_1_4;
		snand->prog_mode = SPI_OP_1_1_4;
		snand->read_cache_opcode = SPI_NAND_READ_CACHE_X4;
		snand->prog_load_opcode = SPI_NAND_PROGRAM_LOAD4;
		snand->read_cache_dummy = 1;
	} else if (chip->options & SPI_NAND_DUAL_MODE) {
		snand->read_mode = SPI_OP_1_1_2;
		snand->prog_mode = SPI_OP_1_1_1;
		snand->read_cache_opcode = SPI_NAND_READ_CACHE_X2;
		snand->prog_load_opcode = SPI_NAND_PROGRAM_LOAD;
		snand->read_cache_dummy = 1;
	} else {
		snand->read_mode = SPI_OP_1_1_1;
		snand->prog_mode = SPI_OP_1_1_1;
		snand->read_cache_opcode = SPI_NAND_FAST_READ_CACHE;
		snand->prog_load_opcode = SPI_NAND_PROGRAM_LOAD;
		snand->read_cache_dummy = 1;
	}

	return 0;
}

static struct platform_nand_data qspi_nfc_nand_data = {
	.chip = {
		.nr_chips	= 1,
	},
};

static struct spi_board_info qspi_nfc_board_info[] = {
	{
		.modalias		= "rts-quadspi-nfc",
		.platform_data		= &qspi_nfc_nand_data,
		.mode			= SPI_MODE_0,
		.max_speed_hz		= 60000000,
		.bus_num		= 0,
		.chip_select		= 0,
	},
};

static int rts_qspi_nfc_probe(struct platform_device *pdev)
{
	const struct platform_nand_data	*pdata;
	struct spi_board_info		*board_info;
	struct rts_qspi_nfc		*rqspi_nfc;
	struct resource			*res;
	struct spi_nand			*snand;
	struct nand_chip		*chip;
	struct clk			*clk;
	void __iomem			*regs;
	int				ret;
	int				irq;
	int				spiclk_hz;
	int				maxchips;
	struct reset_control *rst;

	rqspi_nfc = devm_kzalloc(&pdev->dev, sizeof(*rqspi_nfc), GFP_KERNEL);
	if (!rqspi_nfc)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "get resource failed!\n");
		return -EINVAL;
	}

	res = devm_request_mem_region(&pdev->dev,
		res->start, resource_size(res), pdev->name);
	if (!res) {
		dev_err(&pdev->dev, "request memory region failed!\n");
		return -ENOMEM;
	}

	regs = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!regs) {
		dev_err(&pdev->dev, "ioremap fail, phyaddr:%x\n", res->start);
		return -ENOMEM;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "get irq failed!\n");
		return -ENXIO;
	}

	clk = devm_clk_get(&pdev->dev, "spi_ck");
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "get clock failed!\n");
		return PTR_ERR(clk);
	}

	spiclk_hz = clk_get_rate(clk);
	if (!spiclk_hz) {
		dev_err(&pdev->dev,
			"get invalid clock rate: %d\n", spiclk_hz);
		return -EINVAL;
	}

	board_info = qspi_nfc_board_info;
	if (!(board_info && board_info->platform_data)) {
		dev_err(&pdev->dev, "platform_nand_data is missing\n");
		return -EINVAL;
	}

	pdata = board_info->platform_data;
	if (pdata->chip.nr_chips < 1) {
		dev_err(&pdev->dev, "invalid number of chips specified\n");
		return -EINVAL;
	}
	rqspi_nfc->bi = board_info;
	platform_set_drvdata(pdev, rqspi_nfc);
	snand = &rqspi_nfc->spi_nand;
	chip = &snand->nand_chip;
	snand->mtd = nand_to_mtd(chip);

	rqspi_nfc->irq = irq;
	rqspi_nfc->regs = regs;
	rqspi_nfc->phybase = res->start;
	rqspi_nfc->pdev = pdev;
	rqspi_nfc->spiclk_hz = spiclk_hz;
	rqspi_nfc->max_speed_hz = DIV_ROUND_UP(spiclk_hz, 1) / 2;
	rqspi_nfc->min_speed_hz =
		DIV_ROUND_UP(spiclk_hz, ((1 << SCKDV_WIDTH) - 2)) / 2;

	rqspi_nfc->pdev = pdev;
	snand->dev = &pdev->dev;
	snand->priv = rqspi_nfc;
	chip->priv = snand;
	chip->ecc.mode = NAND_ECC_HW;
	chip->ecc.read_page = rts_qspi_nfc_read_page_ecc;
	chip->ecc.read_page_raw = rts_qspi_nfc_read_page_raw;
	chip->ecc.write_page = rts_qspi_nfc_write_page_ecc;
	chip->ecc.write_page_raw = rts_qspi_nfc_write_page_raw;
	chip->ecc.read_oob = rts_qspi_nfc_read_oob_raw;
	chip->ecc.write_oob = rts_qspi_nfc_write_oob_ecc;
	mtd_set_ooblayout(snand->mtd, &rts_nfc_ooblayout_ops);
	chip->ecc.strength = BCH12_MAX_ERROR;
	chip->IO_ADDR_R = regs + DATA_FIFO;
	chip->IO_ADDR_W = regs + DATA_FIFO;
	chip->read_byte = rts_qspi_nfc_read_byte;
	chip->read_word = rts_qspi_nfc_read_word;
	chip->read_buf = rts_qspi_nfc_read_buf;
	chip->write_buf = rts_qspi_nfc_write_buf;
	chip->cmdfunc = rts_qspi_nfc_cmdfunc;
	chip->waitfunc = rts_qspi_nfc_waitfunc;
	chip->select_chip = spi_nand_select_chip;
	chip->options |= NAND_NO_SUBPAGE_WRITE;

	/* Disable memory shutdown */
	rst = devm_reset_control_get(&pdev->dev, "spi-nand-sysmem-up");
	if (IS_ERR(rst)) {
		if (PTR_ERR(rst) != -EPROBE_DEFER)
			dev_err(&pdev->dev, "No top level reset found\n");

		goto FAIL;
	}
	reset_control_assert(rst);

	mdelay(5);

	/* Initialize the hardware */
	ret = rts_qspi_nfc_setup(rqspi_nfc);
	if (ret)
		goto FAIL;

	snand->mtd->name = dev_name(&pdev->dev);
	snand->mtd->owner = THIS_MODULE;
	snand->mtd->priv = chip;

	snand->reset = rts_qspi_nfc_reset;
	snand->read_id = rts_qspi_nfc_read_id;
	snand->read_reg = rts_qspi_nfc_read_reg;
	snand->write_reg = rts_qspi_nfc_write_reg;
	snand->load_page = rts_qspi_nfc_load_page;
	snand->write_page = rts_qspi_nfc_program_page;
	snand->read_cache = rts_qspi_nfc_read_cache;
	snand->store_cache = rts_qspi_nfc_store_cache;
	snand->block_erase = rts_qspi_nfc_block_erase;
	snand->write_enable = rts_qspi_nfc_write_enable;
	snand->write_disable = rts_qspi_nfc_write_disable;

	ret = spi_nand_check(snand);
	if (ret)
		goto FAIL;

	simple_get_flash_id(snand);

	/* Work around for WINBOND 2Gb Nand:
	 * Winbond 2Gb Nand has 2 * 1Gb die inside the chip,
	 * so we pretend there are two chips on external bus.
	 */
	if (has_die_select(snand)) {
		snand->die_select = winbond_die_select;
		chip->select_chip = sw_die_select_chip;
		maxchips = 2;
	} else {
		maxchips = 1;
	}

	ret = nand_scan_ident(snand->mtd, maxchips, spi_nand_flash_ids);
	if (ret)
		goto FAIL;

	chip->ecc.size = snand->mtd->writesize;

	ret = spi_nand_init_chip(snand, maxchips);
	if (ret)
		goto FAIL;

	ret = spi_nand_enable_quad(snand, maxchips);
	if (ret)
		goto FAIL;

	ret = nand_scan_tail(snand->mtd);
	if (ret)
		goto FAIL;

	ret = mtd_device_register(snand->mtd, NULL, 0);
	if (ret)
		goto FAIL;

	dev_info(&pdev->dev, "Realtek QSPI Nand Flash Controller at 0x%08lx (irq %d)\n",
			(unsigned long)res->start, irq);

	return 0;

FAIL:
	dev_err(&pdev->dev, "Realtek QuadSPI NFC probe failed!\n");
	return ret;
}

static int rts_qspi_nfc_remove(struct platform_device *pdev)
{
	struct rts_qspi_nfc *rqspi_nfc = platform_get_drvdata(pdev);
	struct spi_nand *snand = &rqspi_nfc->spi_nand;
	struct mtd_info *mtd = snand->mtd;

	nand_release(mtd);

	return 0;
}

static const struct of_device_id rts_qspi_nfc_dt_ids[] = {
	{ .compatible = "realtek,rts3903-quadspi-nfc",},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rts_qspi_dt_ids);

static struct platform_driver rts_qspi_nfc_driver = {
	.driver = {
		.name	= "rts-quadspi-nfc",
		.owner	= THIS_MODULE,
		.of_match_table = rts_qspi_nfc_dt_ids,
	},
	.probe		= rts_qspi_nfc_probe,
	.remove		= rts_qspi_nfc_remove,
};

module_platform_driver(rts_qspi_nfc_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("QuadSPI flash nand flash controller driver for realtek rts39xx ipcam soc");
MODULE_AUTHOR("Jim Cao <jim_cao@realsil.com.cn>");
