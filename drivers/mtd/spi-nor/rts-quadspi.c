/*
 * Driver for Realtek IPCam RTS39XX SPI Controller
 *
 * Copyright (C) 2015 Darcy Lu, Realtek <darcy_lu@realsil.com.cn>
 * Copyright (C) 2016 Jim Cao, Realtek <jim_cao@realsil.com.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License verqspiion 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/ctype.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/spi-nor.h>
#include <linux/of_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>

/*
 * The definition of the FIFO look up table shows below:
 *
 *  ---------------------------------------------
 *  | INSTR(1B) | ADDR(1~4B) | DATA(rest FIFO) |
 *  ---------------------------------------------
 */

/* SPI register offsets */
#define CTRLR0					0x0000
#define CTRLR1					0x0004
#define SSIENR					0x0008
#define MWCR					0x000c
#define SER					0x0010
#define BAUDR					0x0014
#define TXFTLR					0x0018
#define RXFTLR					0x001c
#define TXFLR					0x0020
#define RXFLR					0x0024
#define SR					0x0028
#define IMR					0x002c
#define ISR					0x0030
#define RISR					0x0034
#define TXOICR					0x0038
#define RXOICR					0x003c
#define RXUICR					0x0040
#define MSTICR					0x0044
#define ICR					0x0048
#define DMACR					0x004c
#define DMATDLR					0x0050
#define DMARDLR					0x0054
#define IDR					0x0058
#define SPIC_VERSION				0x005c
#define DR					0x0060
#define READ_FAST_SINGLE			0x00e0
#define READ_DUAL_DATA				0x00e4
#define READ_DUAL_ADDR_DATA			0x00e8
#define READ_QUAD_DATA				0x00ec
#define READ_QUAD_ADDR_DATA			0x00f0
#define WRITE_SINGLE				0x00f4
#define WRITE_DUAL_DATA				0x00f8
#define WRITE_DUAL_ADDR_DATA			0x00fc
#define WRITE_QUAD_DATA				0x0100
#define WRITE_QUAD_ADDR_DATA			0x0104
#define WRITE_ENABLE				0x0108
#define READ_STATUS				0x010c
#define CTRLR2					0x0110
#define FBAUDR					0x0114
#define ADDR_LENGTH				0x0118
#define AUTO_LENGTH				0x011c
#define VALID_CMD				0x0120
#define FLASH_SIZE				0x0124
#define FLUSH_FIFO				0x0128
#define PGM_RST_FIFO				0x0140

/* Bit fields in CTRLR0 */
#define SCPH					6
#define SCPOL					7
#define TMOD_OFFSET				8
#define TMOD_MASK				3
#define TRANSMIT_MODE				0
#define RECEIVE_MODE				3
#define ADDR_CH_OFFSET				16
#define ADDR_CH_MASK				3
#define DATA_CH_OFFSET				18
#define DATA_CH_MASK				3
#define CMD_CH_OFFSET				20
#define CMD_CH_MASK				3

/* Bit fields in SR */
#define BUSY					0
/* Bit fields in SSIENR */
#define SPIC_EN					0
#define ATCK_CMD				1
#define PGM_RST_TEST_EN				4

/* Bit fields in BAUDR */
#define SCKDV					0
#define SCKDV_WIDTH				12
#define SCKDV_MASK				((1 << 12) - 1)

/* Bit fields in AUTO_LENGTH */
#define RD_DUMMY_LENGTH				0
#define RD_DUMMY_LENGTH_MASK			((1 << 12) - 1)
#define AUTO_ADDR_LENGTH			16
#define AUTO_ADDR_LENGTH_MASK			(0x3 << 16)
#define AUTO_DUM_LEN				18
#define CS_H_RD_DUM_LEN				26
#define CS_H_WR_DUM_LEN				28

/* Bit fields in VALID_CMD */
#define FRD_SINGLE				0
#define RD_DUAL_I				1
#define RD_DUAL_IO				2
#define RD_QUAD_O				3
#define RD_QUAD_IO				4
#define WR_DUAL_I				5
#define WR_DUAL_II				6
#define WR_QUAD_I				7
#define WR_QUAD_II				8
#define WR_BLOCKING				9

/* Bit fields in CTRLR2 */
#define SO_DNUM					0
#define WPN_SET					1
#define WPN_DNUM				2
#define SEQ_EN					3
#define FIFO_ENTRY				4
#define FIFO_ENTRY_MASK				(0xf << 4)
#define RX_FIFO_ENTRY				8

/* Bit fileds in PGM_RST ctrl reg */
#define PGMRST_CMD_VAL				0
#define PGMRST_CMD_CH				8
#define PGMRST_STATE				10
#define PGMRST_COUNT				12

#define SPI_CH				0
#define QSPI_CH				1
#define QPI_CH				2
#define OPI_CH				3

#define STATE_END			0
#define STATE_NEXT			1
#define STATE_KEEP			2
#define STATE_NOP			3

#define COUNT_2EXP4			2
#define COUNT_2EXP16			4
#define COUNT_2EXP23			8

#define RTSX_QSPI_DRV_NAME		"rts-spi-nor"

static char *channels;
module_param(channels, charp, 0444);
MODULE_PARM_DESC(channels,
	"spi channel mode, fast/dual/quad/qpi");

/**
 * struct spi_nor_xfer_cfg - Structure for defining a Serial Flash transfer
 * @wren:               command for "Write Enable", or 0x00 for not required
 * @cmd:                command for operation
 * @cmd_pins:           number of pins to send @cmd (1, 2, 4)
 * @addr:               address for operation
 * @addr_pins:          number of pins to send @addr (1, 2, 4)
 * @addr_width:         number of address bytes
 *                      (3,4, or 0 for address not required)
 * @mode:               mode data
 * @mode_pins:          number of pins to send @mode (1, 2, 4)
 * @mode_cycles:        number of mode cycles (0 for mode not required)
 * @dummy_cycles:       number of dummy cycles (0 for dummy not required)
 */
struct spi_nor_xfer_cfg {
	u8			wren;
	u8			cmd;
	u8			cmd_pins;
	u32			addr;
	u8			addr_pins;
	u8			addr_width;
	u8			mode;
	u8			mode_pins;
	u8			mode_cycles;
	u8			dummy_cycles;
};

enum rts_qspi_devtype {
	TYPE_FPGA = (1 << 0),
	RTS_QUADSPI_RTS3903 = (1 << 1),
};

/* rts qspi */
struct rts_qspi {
	struct mtd_info		mtd;
	struct spi_nor		nor;
	struct platform_device	*pdev;
	int			irq;
	void __iomem		*regs;
	phys_addr_t		phybase;

	struct clk		*clk;
	u32			spiclk_hz;
	u32			max_speed_hz;
	u32			min_speed_hz;

	struct spi_board_info	*bi;

	bool			use_dma;
	int			fifo_size;
	int			fifo_entry;

#ifdef CONFIG_SPI_RTS_QUADSPI_IRQ
	struct completion		*done;
	int				timeout_ms;
	spinlock_t			__lock;
	unsigned long			__lock_flags;
#endif
	enum rts_qspi_devtype devtype;
};

struct rts_qspi_devtype_data {
	enum rts_qspi_devtype devtype;
	int fifo_entry;
	int fifo_size;
};

static struct rts_qspi_devtype_data rts3903_data = {
	.devtype = RTS_QUADSPI_RTS3903,
	.fifo_entry = 8,
	.fifo_size = 256,
};

static inline u32 rts_readl(struct rts_qspi *rqspi, u32 reg)
{
	return readl(rqspi->regs + reg);
}

static inline u16 rts_readw(struct rts_qspi *rqspi, u32 reg)
{
	return readw(rqspi->regs + reg);
}

static inline u8 rts_readb(struct rts_qspi *rqspi, u32 reg)
{
	return readb(rqspi->regs + reg);
}

static inline void rts_writel(struct rts_qspi *rqspi, u32 reg, u32 val)
{
	writel(val, rqspi->regs + reg);
}

static inline void rts_writew(struct rts_qspi *rqspi, u32 reg, u16 val)
{
	writew(val, rqspi->regs + reg);
}

static inline void rts_writeb(struct rts_qspi *rqspi, u32 reg, u8 val)
{
	writeb(val, rqspi->regs + reg);
}

static void addr2cmd(int addr_width, u32 addr, u8 *cmd)
{
	cmd[1] = addr >> (addr_width * 8 - 8);
	cmd[2] = addr >> (addr_width * 8 - 16);
	cmd[3] = addr >> (addr_width * 8 - 24);
	cmd[4] = addr >> (addr_width * 8 - 32);
}

static int rts_qspi_controller_ready(struct rts_qspi *rqspi)
{

	u32 cnt;
	u32 reg;

	for (cnt = 0; cnt < 1000; cnt++) {
		reg = rts_readl(rqspi, SR);
		if (!(reg & BIT(BUSY)))
			return 0;
		udelay(1);
	}
	return -EBUSY;
}

static inline int rts_qspi_set_dummy(struct rts_qspi *rqspi, u32 cycle)
{
#define INTERNAL_DUMMY 1
	u32 baud;
	u32 dummy;
	u32 reg;

	if (cycle == 0) {
		dummy = 0;
	} else {
		baud = rts_readl(rqspi, BAUDR);
		dummy = baud * cycle * 2 + INTERNAL_DUMMY;
	}

	if (dummy > RD_DUMMY_LENGTH_MASK)
		return -EINVAL;

	reg = rts_readl(rqspi, AUTO_LENGTH);
	reg = (reg & ~RD_DUMMY_LENGTH_MASK) | dummy;
	rts_writel(rqspi, AUTO_LENGTH, reg);

	return 0;
}

static int rts_qspi_read_xfer(struct spi_nor *nor,
		struct spi_nor_xfer_cfg *cfg, u8 *buf, size_t len)
{
	struct rts_qspi *rqspi = nor->priv;
	int ret, cnt;
	u32 reg;

	ret = rts_qspi_set_dummy(rqspi, cfg->dummy_cycles);
	if (ret)
		goto FAIL;

	reg = rts_readl(rqspi, CTRLR0);
	reg &= ~((TMOD_MASK << TMOD_OFFSET) |
		(ADDR_CH_MASK << ADDR_CH_OFFSET) |
		(DATA_CH_MASK << DATA_CH_OFFSET) |
		(CMD_CH_MASK << CMD_CH_OFFSET));
	reg |= (((u32)(cfg->mode) << TMOD_OFFSET) |
		((u32)(cfg->addr_pins >> 1) << ADDR_CH_OFFSET) |
		((u32)(cfg->mode_pins >> 1) << DATA_CH_OFFSET) |
		((u32)(cfg->cmd_pins >> 1) << CMD_CH_OFFSET));

	rts_writel(rqspi, CTRLR0, reg);
	if (cfg->addr_width) {
		u8 cmd[5], cmd_len;
		int cnt;

		cmd[0] = cfg->cmd;
		addr2cmd(cfg->addr_width, cfg->addr, cmd);
		cmd_len = cfg->addr_width + 1;

		for (cnt = 0; cnt < cmd_len; cnt++)
			rts_writeb(rqspi, DR, cmd[cnt]);

		/* Map 4 byte length to zero value */
		rts_writel(rqspi, ADDR_LENGTH, cfg->addr_width & 0x3);
	} else {
		rts_writeb(rqspi, DR, cfg->cmd);
	}
	rts_writel(rqspi, CTRLR1, len);

	rts_writel(rqspi, SSIENR, 1);

	ret = rts_qspi_controller_ready(rqspi);
	if (ret) {
		dev_err(nor->dev, "controller busy\n");
		goto FAIL;
	}

	rts_writel(rqspi, SSIENR, 0);

	/* Optimize for 4 Byte FIFO read */
	for (cnt = 0; cnt < len / 4; cnt++) {
		u32 *buf32 = (u32 *)buf;

		buf32[cnt] = rts_readl(rqspi, DR);
	}

	for (cnt = len - len % 4; cnt < len; cnt++)
		buf[cnt] = rts_readb(rqspi, DR);

	return 0;
FAIL:
	dev_err(nor->dev, "%s() failed, ret = %d\n", __func__, ret);
	return ret;
}

static int rts_qspi_write_xfer(struct spi_nor *nor,
		struct spi_nor_xfer_cfg *cfg, u8 *buf, size_t len)
{
	struct rts_qspi *rqspi = nor->priv;
	int ret, cnt;
	u32 reg;

	ret = rts_qspi_set_dummy(rqspi, cfg->dummy_cycles);
	if (ret)
		goto FAIL;

	reg = rts_readl(rqspi, CTRLR0);
	reg &= ~((TMOD_MASK << TMOD_OFFSET) |
		(ADDR_CH_MASK << ADDR_CH_OFFSET) |
		(DATA_CH_MASK << DATA_CH_OFFSET) |
		(CMD_CH_MASK << CMD_CH_OFFSET));
	reg |= (((u32)(cfg->mode) << TMOD_OFFSET) |
		((u32)(cfg->addr_pins >> 1) << ADDR_CH_OFFSET) |
		((u32)(cfg->mode_pins >> 1) << DATA_CH_OFFSET) |
		((u32)(cfg->cmd_pins >> 1) << CMD_CH_OFFSET));
	rts_writel(rqspi, CTRLR0, reg);

	/* when transmited bytes are not greater than 4, use ADDR_LENGTH
	 * to indicate non-cmd bytes. When len equals zero, we don't
	 * push data into FIFO, just ignore it.
	 */
	if (cfg->addr_width == 0) {
		/* nor->write_reg */
		rts_writeb(rqspi, DR, cfg->cmd);

		if (len >= 4)
			rts_writel(rqspi, ADDR_LENGTH, 0);
		else
			rts_writel(rqspi, ADDR_LENGTH, len);

		for (cnt = 0; cnt < len; cnt++)
			rts_writeb(rqspi, DR, buf[cnt]);
	} else {
		/* nor->write */
		u8 cmd[5];
		u8 cmd_len;

		cmd[0] = cfg->cmd;
		addr2cmd(cfg->addr_width, cfg->addr, cmd);
		cmd_len = cfg->addr_width + 1;

		for (cnt = 0; cnt < cmd_len; cnt++)
			rts_writeb(rqspi, DR, cmd[cnt]);

		/* Map 4 byte length to zero value */
		rts_writel(rqspi, ADDR_LENGTH, cfg->addr_width & 0x3);

		/* Optimize for 4 Byte FIFO write */
		for (cnt = 0; cnt < len / 4; cnt++) {
			u32 *buf32 = (u32 *)buf;

			rts_writel(rqspi, DR, buf32[cnt]);
		}

		for (cnt = len - len % 4; cnt < len; cnt++)
			rts_writeb(rqspi, DR, buf[cnt]);
	}

#ifdef CONFIG_SPI_RTS_QUADSPI_IRQ
	if (cfg->addr_width == 0)
		rts_writel(rqspi, SSIENR, 1);
	else
		rts_writel(rqspi, SSIENR, 3);
	if (cfg->addr_width == 0) {
		ret = rts_qspi_controller_ready(rqspi);
		if (ret) {
			dev_err(nor->dev, "controller busy\n");
			goto FAIL;
		}
		rts_writel(rqspi, SSIENR, 0);
	}
#endif

#ifdef CONFIG_SPI_RTS_QUADSPI_POLLING
	rts_writel(rqspi, SSIENR, 1);

	ret = rts_qspi_controller_ready(rqspi);
	if (ret) {
		dev_err(nor->dev, "controller busy\n");
		goto FAIL;
	}

	rts_writel(rqspi, SSIENR, 0);
#endif

	return 0;

FAIL:
	dev_err(nor->dev, "%s() failed, errno = %d\n", __func__, ret);
	return ret;
}

static int rts_qspi_read_reg(struct spi_nor *nor, u8 opcode, u8 *buf, int len)
{
	struct spi_nor_xfer_cfg cfg;

	cfg.wren = 0;
	cfg.cmd = opcode;
	cfg.cmd_pins = 1;
	cfg.addr = 0;
	cfg.addr_width = 0;
	cfg.addr_pins = 1;
	cfg.mode = RECEIVE_MODE;
	cfg.mode_pins = 1;
	cfg.mode_cycles = 0;
	cfg.dummy_cycles = 0;

	if (nor->flash_read == SPI_NOR_QPI) {
		cfg.cmd_pins = 4;
		cfg.addr_pins = 4;
		cfg.mode_pins = 4;
	}

	return rts_qspi_read_xfer(nor, &cfg, buf, len);
}

static int rts_qspi_write_reg(struct spi_nor *nor, u8 opcode, u8 *buf, int len)
{
	struct spi_nor_xfer_cfg cfg;

	cfg.wren = 0;
	cfg.cmd = opcode;
	cfg.cmd_pins = 1;
	cfg.addr = 0;
	cfg.addr_width = 0;
	cfg.addr_pins = 1;
	cfg.mode = TRANSMIT_MODE;
	cfg.mode_pins = 1;
	cfg.mode_cycles = 0;
	cfg.dummy_cycles = 0;

	if (nor->flash_pp == SPI_NOR_PP_QPI) {
		cfg.cmd_pins = 4;
		cfg.addr_pins = 4;
		cfg.mode_pins = 4;
	}

	return rts_qspi_write_xfer(nor, &cfg, buf, len);
}

static int _rts_qspi_read(struct spi_nor *nor, loff_t from,
		size_t len, size_t *retlen, u_char *buf)
{
	struct spi_nor_xfer_cfg cfg;
	int cmd_pins, addr_pins, mode_pins;
	int ret;

	/* set channel */
	switch (nor->flash_read) {
	case SPI_NOR_QPI:
		cmd_pins = 4; addr_pins = 4; mode_pins = 4;
		break;
	case SPI_NOR_QUAD:
		cmd_pins = 1; addr_pins = 1; mode_pins = 4;
		break;
	case SPI_NOR_DUAL:
		cmd_pins = 1; addr_pins = 1; mode_pins = 2;
		break;
	case SPI_NOR_FAST:
	case SPI_NOR_NORMAL:
		cmd_pins = 1; addr_pins = 1; mode_pins = 1;
		break;
	default:
		dev_err(nor->dev, "Invalid read opcode!\n");
		return -EINVAL;
	}

	cfg.wren = 0;
	cfg.cmd = nor->read_opcode;
	cfg.cmd_pins = cmd_pins;
	cfg.addr = from;
	cfg.addr_width = nor->addr_width;
	cfg.addr_pins = addr_pins;
	cfg.mode = RECEIVE_MODE;
	cfg.mode_pins = mode_pins;
	cfg.mode_cycles = 0;
	cfg.dummy_cycles = nor->read_dummy;

	ret = rts_qspi_read_xfer(nor, &cfg, buf, len);
	if (ret)
		return ret;

	*retlen += len;

	return ret;
}

static ssize_t rts_qspi_read(struct spi_nor *nor, loff_t from,
		size_t len, u_char *buf)
{
	struct rts_qspi *rqspi = nor->priv;
	int fifo = rqspi->fifo_size;
	int err;
	ssize_t retlen = 0;

	while (len) {
		size_t unit = fifo, rlen = 0;

		if (len < unit)
			unit = len;

		err = _rts_qspi_read(nor, from, unit, &rlen, buf);
		if (err)
			return err;

		retlen += rlen;
		len -= unit;
		from += unit;
		buf += unit;
	}

	return retlen;
}

static int _rts_qspi_write(struct spi_nor *nor, loff_t to,
		size_t len, size_t *retlen, const u_char *buf)
{
	struct spi_nor_xfer_cfg cfg;
	int ret;
	int cmd_pins, addr_pins, mode_pins;

	/* set channel */
	switch (nor->flash_pp) {
	case SPI_NOR_PP_NORMAL:
		cmd_pins = 1; addr_pins = 1; mode_pins = 1;
		break;
	case SPI_NOR_PP_QUAD:
		cmd_pins = 1; addr_pins = 1; mode_pins = 4;
		break;
	case SPI_NOR_PP_4IO:
		cmd_pins = 1; addr_pins = 4; mode_pins = 4;
		break;
	case SPI_NOR_PP_QPI:
		cmd_pins = 4; addr_pins = 4; mode_pins = 4;
		break;
	default:
		dev_err(nor->dev, "Invalid Page Program opcode\n");
		return -EINVAL;
	}

	cfg.wren = 0;
	cfg.cmd = nor->program_opcode;
	cfg.cmd_pins = cmd_pins;
	cfg.addr = to;
	cfg.addr_width = nor->addr_width;
	cfg.addr_pins = addr_pins;
	cfg.mode = TRANSMIT_MODE;
	cfg.mode_pins = mode_pins;
	cfg.mode_cycles = 0;
	cfg.dummy_cycles = 0;

	ret = rts_qspi_write_xfer(nor, &cfg, (u_char *)buf, len);
	if (ret)
		return ret;

	*retlen += len;

	return ret;
}

/*
 * Set write enable latch with Write Enable command.
 * Returns negative if error occurred.
 */
static inline int write_enable(struct spi_nor *nor)
{
	return nor->write_reg(nor, SPINOR_OP_WREN, NULL, 0);
}

static ssize_t rts_qspi_write(struct spi_nor *nor, loff_t to,
		size_t len, const u_char *buf)
{
	struct rts_qspi *rqspi = nor->priv;
	int fifo = rqspi->fifo_size;
	int ret;
	ssize_t retlen = 0;

#ifdef CONFIG_SPI_RTS_QUADSPI_IRQ
	struct completion done_data;
	long timeleft;
	int timeout = rqspi->timeout_ms;
#endif

	while (len) {
		size_t unit = fifo, rlen = 0;

		if (len < unit)
			unit = len;

#ifdef CONFIG_SPI_RTS_QUADSPI_IRQ
		spin_lock_irqsave(&rqspi->__lock, rqspi->__lock_flags);
		rqspi->done = &done_data;
		init_completion(&done_data);
#endif
		write_enable(nor);
		ret = _rts_qspi_write(nor, to, unit, &rlen, buf);
		if (ret)
			goto FAIL;
#ifdef CONFIG_SPI_RTS_QUADSPI_POLLING
		ret = nor->wait_till_ready(nor);
		if (ret)
			goto FAIL;
#endif

#ifdef CONFIG_SPI_RTS_QUADSPI_IRQ
		spin_unlock_irqrestore(&rqspi->__lock, rqspi->__lock_flags);

		timeleft = wait_for_completion_interruptible_timeout(
			rqspi->done, msecs_to_jiffies(timeout));
		rts_writel(rqspi, SSIENR, 0);
		if (timeleft <= 0) {
			ret = -ETIMEDOUT;
			goto FAIL;
		}
#endif

		retlen += rlen;
		len -= unit;
		to += unit;
		buf += unit;
	}

	return retlen;
FAIL:
	dev_err(nor->dev, "%s() failed, ret = %d\n", __func__, ret);
	return ret;
}

static int rts_qspi_erase(struct spi_nor *nor, loff_t offs)
{
	int ret;

	ret = nor->write_reg(nor, SPINOR_OP_WREN, NULL, 0);
	if (ret)
		return ret;

	nor->cmd_buf[0] = offs >> (nor->addr_width * 8 - 8);
	nor->cmd_buf[1] = offs >> (nor->addr_width * 8 - 16);
	nor->cmd_buf[2] = offs >> (nor->addr_width * 8 - 24);
	nor->cmd_buf[3] = offs >> (nor->addr_width * 8 - 32);

	ret = nor->write_reg(nor, nor->erase_opcode,
			nor->cmd_buf, nor->addr_width);
	if (ret)
		return ret;

	return 0;
}

static int rts_qspi_setup(struct rts_qspi *rqspi)
{
	struct spi_board_info	*bi = rqspi->bi;
	struct platform_device	*pdev = rqspi->pdev;
	u32			reg, baudr;
	u32			speed_hz;

	/* User can't program some control register if SSIENR
	 * is enabled. So disable it before init registers
	 */
	rts_writel(rqspi, SSIENR, 0);	/* Disable SPIC */

	reg = 0;
	/* spi mode */
	if (bi->mode & SPI_CPOL)
		reg |= BIT(SCPOL);
	if (bi->mode && SPI_CPHA)
		reg |= BIT(SCPH);
#ifdef CONFIG_SPI_RTS_QUADSPI_IRQ
	reg |= (0x1f << 23);
#endif

	rts_writel(rqspi, CTRLR0, reg);

	/* Set clock ratio
	 * F(spi_sclk) = F(bus) / (2 * baudr)
	 */
	speed_hz = bi->max_speed_hz;
	if ((speed_hz == 0) || (speed_hz > rqspi->max_speed_hz)) {
		speed_hz = rqspi->max_speed_hz;
		dev_warn(&pdev->dev, "request %d Hz, force to set %d Hz\n",
			bi->max_speed_hz, rqspi->max_speed_hz);
	}

	if (speed_hz < rqspi->min_speed_hz) {
		dev_err(&pdev->dev, "requested speed too low %d Hz\n",
			bi->max_speed_hz);
		return -EINVAL;
	}

	if (rqspi->devtype & TYPE_FPGA)
		baudr = 8;
	else
		baudr = DIV_ROUND_UP(rqspi->spiclk_hz, speed_hz) / 2;
	if (baudr > SCKDV_MASK) {
		dev_err(&pdev->dev, "invalid baud reg: %08X\n", baudr);
		return -EINVAL;
	}
	rts_writel(rqspi, BAUDR, baudr);

	/* pin route & FIFO depth 2^fifo_entry Byte */
	reg = rts_readl(rqspi, CTRLR2);
	reg &= ~(BIT(SO_DNUM) | BIT(WPN_DNUM) | FIFO_ENTRY_MASK);
	reg |= BIT(SO_DNUM) | ((rqspi->fifo_entry) << FIFO_ENTRY);

	WARN_ON(rqspi->fifo_entry == 0);

	rts_writel(rqspi, CTRLR2, reg);
#ifdef CONFIG_SPI_RTS_QUADSPI_IRQ
	rts_writel(rqspi, IMR, 0x800);	/* enable ACSIM interrupt */
	rqspi->timeout_ms = 5000;
#endif
#ifdef CONFIG_SPI_RTS_QUADSPI_POLLING
	rts_writel(rqspi, IMR, 0);	/* Disable all interrupt */
#endif
	rts_writel(rqspi, SER, 1);	/* cs actived */

	rts_writel(rqspi, WRITE_SINGLE, 0xBB);

	return 0;
}

#define JEDEC_MFR(_jedec_id)	((_jedec_id) >> 16)

static void rts_qspi_init_reset_rom(struct rts_qspi *rqspi)
{
	struct spi_nor *nor = &rqspi->nor;
	u16 ctrl_reg[3];
	u16 cmd_ch;

	rts_writel(rqspi, FLUSH_FIFO, 0xffffffff);
	switch (JEDEC_MFR(nor->jedec_id)) {
	case SNOR_MFR_MACRONIX:
	case SNOR_MFR_GIGADEVICE:
	case SNOR_MFR_WINBOND:
	case SNOR_MFR_MICRON:
	case SNOR_MFR_EON:
	case SNOR_MFR_BOYAMICRO:
	case SNOR_MFR_XTX:
		if (nor->flash_read == SPI_NOR_QPI)
			cmd_ch = QPI_CH;
		else
			cmd_ch = SPI_CH;

		ctrl_reg[0] = (COUNT_2EXP16 << PGMRST_COUNT) |
				(STATE_NEXT << PGMRST_STATE) |
				(cmd_ch << PGMRST_CMD_CH) |
				(SPINOR_OP_RSTEN);
		ctrl_reg[1] = (COUNT_2EXP16 << PGMRST_COUNT) |
				(STATE_NEXT << PGMRST_STATE) |
				(cmd_ch << PGMRST_CMD_CH) |
				(SPINOR_OP_RST);
		ctrl_reg[2] = (COUNT_2EXP16 << PGMRST_COUNT) |
				(STATE_END << PGMRST_STATE) |
				(SPI_CH << PGMRST_CMD_CH) |
				(0x00);

		rts_writew(rqspi, PGM_RST_FIFO, ctrl_reg[0]);
		rts_writew(rqspi, PGM_RST_FIFO, ctrl_reg[1]);
		rts_writew(rqspi, PGM_RST_FIFO, ctrl_reg[2]);
		break;
	default:
		break;
	}

}

static void rts_qspi_reset(struct rts_qspi *rqspi)
{
	struct spi_nor *nor = &rqspi->nor;

	nor->write_reg(nor, SPINOR_OP_RSTEN, NULL, 0);
	nor->write_reg(nor, SPINOR_OP_RST, NULL, 0);
}

static int rts_channel_map(char *str, const char **map)
{
	char lower[10] = {0};
	char *p;
	int index;

	strncpy(lower, str, sizeof(lower) - 1);

	for (p = lower; *p; p++)
		*p = tolower(*p);

	index = 0;
	for (; *map; map++) {
		if (!strcmp(lower, *map))
			return index;
		index++;
	}

	return -EINVAL;
}

#ifdef CONFIG_SPI_RTS_QUADSPI_IRQ
static irqreturn_t rtsx_qspi_isr(int irq, void *dev_id)
{
	struct rts_qspi *rqspi = dev_id;
	u32 int_reg;

	if (!rqspi)
		return IRQ_NONE;

	spin_lock(&(rqspi->__lock));

	int_reg = rts_readl(rqspi, ISR);
	rts_writel(rqspi, ICR, 0); /* clear ICR */

	dev_dbg(&(rqspi->pdev->dev), "----- IRQ: 0x%08x -----\n", int_reg);
	if (int_reg & 0x800) {
		if (rqspi->done)
			complete(rqspi->done);
	}

	spin_unlock(&(rqspi->__lock));

	return IRQ_HANDLED;
}

static int rtsx_qspi_acquire_irq(struct rts_qspi *rqspi)
{
	int err = 0;

	spin_lock_init(&rqspi->__lock);
	err = request_irq(rqspi->irq, rtsx_qspi_isr, IRQF_SHARED,
			RTSX_QSPI_DRV_NAME, rqspi);
	if (err)
		dev_err(&(rqspi->pdev->dev), "request IRQ %d failed\n",
				rqspi->irq);

	return err;
}
#endif

static const struct of_device_id rts_qspi_dt_ids[] = {
	{ .compatible = "realtek,rts3903-quadspi",
		.data = (void *)&rts3903_data,},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rts_qspi_dt_ids);

static struct flash_platform_data rts_spiflash_data = {
	.name		= "m25p80",
	.type		= "mx25l12835f",
};

static struct spi_board_info rts_spi_board_info[] = {
	{
		.modalias		= "m25p80",
		.platform_data		= &rts_spiflash_data,
		.mode			= SPI_MODE_0,
		.max_speed_hz		= 60000000,
		.bus_num		= 0,
		.chip_select		= 0,
		.controller_data	= (void *)SPI_NOR_QUAD,
	},
};

static int rts_qspi_probe(struct platform_device *pdev)
{
	struct resource				*res;
	struct rts_qspi				*rqspi;
	struct clk				*clk;
	struct spi_nor				*nor;
	struct spi_board_info			*board_info;
	struct mtd_info				*mtd;
	struct rts_qspi_devtype_data		*qspi_devtype;
	const struct flash_platform_data	*data;
	int					irq, ret;
	u32					spiclk_hz;
	void __iomem				*regs;
	enum read_mode				mode;
	const struct of_device_id *of_id;

	of_id = of_match_device(rts_qspi_dt_ids, &pdev->dev);

	qspi_devtype = (void *)of_id->data;

	/* fpga board */
	if (of_machine_is_compatible("realtek,rts_fpga"))
		qspi_devtype->devtype |= TYPE_FPGA;

	rqspi = devm_kzalloc(&pdev->dev, sizeof(*rqspi), GFP_KERNEL);
	if (!rqspi)
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
		dev_err(&pdev->dev, "ioremap failed, phy address:%x\n",
				res->start);
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
	if (!spiclk_hz)  {
		dev_err(&pdev->dev,
			"get invalid clock rate: %d\n", spiclk_hz);
		return -EINVAL;
	}

	board_info = rts_spi_board_info;
	data = board_info->platform_data;
	if (!(data && data->type)) {
		dev_err(&pdev->dev, "get invalid flash data info\n");
		return -EINVAL;
	}
	rqspi->bi = board_info;
	mode = (int) board_info->controller_data;
	if (mode <= 0 || mode > SPI_NOR_QPI) {
		dev_err(&pdev->dev, "get invalid spi channel info\n");
		return -EINVAL;
	}

	if (channels) {
		int channel_mode;
		static const char * const channel_str[] = {
			[SPI_NOR_NORMAL] = "normal",
			[SPI_NOR_FAST] = "fast",
			[SPI_NOR_DUAL] = "dual",
			[SPI_NOR_QUAD] = "quad",
			[SPI_NOR_QPI]  = "qpi",
			NULL,
		};

		channel_mode = rts_channel_map(channels, channel_str);
		if (channel_mode <= 0) {
			dev_info(&pdev->dev,
				"invalid channel parameter: %s\n", channels);
			return -EINVAL;
		} else if (channel_mode != mode) {
			dev_info(&pdev->dev,
				"force to set channels from %s mode to %s mode\n",
				channel_str[mode], channel_str[channel_mode]);
			mode = channel_mode;
		}
	}

	/* work around for spi_nor_scan check */
	pdev->dev.platform_data = (struct flash_platform_data *)data;

	platform_set_drvdata(pdev, rqspi);
	rqspi->irq = irq;
	rqspi->clk = clk;
	rqspi->regs = regs;
	rqspi->phybase = res->start;
	rqspi->use_dma = false;
	rqspi->fifo_size = qspi_devtype->fifo_size;
	rqspi->fifo_entry = qspi_devtype->fifo_entry;
	rqspi->spiclk_hz = spiclk_hz;
	rqspi->max_speed_hz = DIV_ROUND_UP(spiclk_hz, 1) / 2;
	rqspi->min_speed_hz =
		DIV_ROUND_UP(spiclk_hz, ((1 << SCKDV_WIDTH) - 2)) / 2;
	rqspi->pdev = pdev;
	rqspi->devtype = qspi_devtype->devtype;
	nor = &rqspi->nor;
	nor->mtd = rqspi->mtd;
	nor->dev = &pdev->dev;
	nor->priv = rqspi;
	mtd = &nor->mtd;
	mtd->priv = nor;

#ifdef CONFIG_SPI_RTS_QUADSPI_IRQ
	ret = rtsx_qspi_acquire_irq(rqspi);
	if (ret < 0)
		return ret;
	synchronize_irq(rqspi->irq);
#endif

	/* Initialize the hardware */
	ret = rts_qspi_setup(rqspi);
	if (ret)
		return ret;

	/* fill the hooks */
	nor->read_reg = rts_qspi_read_reg;
	nor->write_reg = rts_qspi_write_reg;
	nor->read = rts_qspi_read;
	nor->write = rts_qspi_write;
	nor->erase = rts_qspi_erase;

	ret = spi_nor_scan(nor, data->type, mode);
	if (ret)
		return ret;

	ret = mtd_device_register(mtd, NULL, 0);
	if (ret)
		return ret;

	/* restore platform_data */
	pdev->dev.platform_data = board_info;

	rts_qspi_init_reset_rom(rqspi);

	dev_info(&pdev->dev, "Realtek QSPI Controller at 0x%08lx (irq %d)\n",
			(unsigned long)res->start, irq);

	return 0;
}

static int rts_qspi_remove(struct platform_device *pdev)
{
	struct rts_qspi *rqspi = platform_get_drvdata(pdev);

	rts_qspi_reset(rqspi);

	mtd_device_unregister(&rqspi->nor.mtd);

	return 0;
}

static const struct platform_device_id rts_qspi_id_table[] = {
	{ .name = "rts3903-qspi",
		.driver_data = (kernel_ulong_t)&rts3903_data, },
	{},
};
MODULE_DEVICE_TABLE(platform, rts_qspi_id_table);

static struct platform_driver rts_qspi_driver = {
	.driver = {
		.name	= "rts-quadspi",
		.owner	= THIS_MODULE,
		.of_match_table = rts_qspi_dt_ids,
	},
	.id_table	= rts_qspi_id_table,
	.probe		= rts_qspi_probe,
	.remove		= rts_qspi_remove,
};
module_platform_driver(rts_qspi_driver);

MODULE_ALIAS("platform:rts-quadspi");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("QuadSPI flash controller driver for realtek rts39xx ipcam soc");
MODULE_AUTHOR("Jim Cao <jim_cao@realsil.com.cn>, Darcy Lu <darcy_lu@realsil.com.cn>");
