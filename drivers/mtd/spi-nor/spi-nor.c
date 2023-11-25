/*
 * Based on m25p80.c, by Mike Lavender (mike@steroidmicros.com), with
 * influence from lart.c (Abraham Van Der Merwe) and mtd_dataflash.c
 *
 * Copyright (C) 2005, Intec Automation Inc.
 * Copyright (C) 2014, Freescale Semiconductor, Inc.
 *
 * This code is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/math64.h>
#include <linux/sizes.h>
#include <linux/kthread.h>

#include <linux/mtd/mtd.h>
#include <linux/of_platform.h>
#include <linux/of.h>
#include <linux/spi/flash.h>
#include <linux/mtd/spi-nor.h>

/* Define max times to check status register before we give up. */

/*
 * For everything but full-chip erase; probably could be much smaller, but kept
 * around for safety for now
 */
#define DEFAULT_READY_WAIT_JIFFIES		(40UL * HZ)

/*
 * For full-chip erase, calibrated to a 2MB flash (M25P16); should be scaled up
 * for larger flash
 */
#define CHIP_ERASE_2MB_READY_WAIT_JIFFIES	(40UL * HZ)

#define SPI_NOR_MAX_ID_LEN	6
#define SPI_NOR_MAX_ADDR_WIDTH	4

struct flash_info {
	char		*name;

	/*
	 * This array stores the ID bytes.
	 * The first three bytes are the JEDIC ID.
	 * JEDEC ID zero means "no ID" (mostly older chips).
	 */
	u8		id[SPI_NOR_MAX_ID_LEN];
	u8		id_len;

	/* The size listed here is what works with SPINOR_OP_SE, which isn't
	 * necessarily called a "sector" by the vendor.
	 */
	unsigned	sector_size;
	u16		n_sectors;

	u16		page_size;
	u16		addr_width;

	u16		flags;
#define SECT_4K			BIT(0)	/* SPINOR_OP_BE_4K works uniformly */
#define SPI_NOR_NO_ERASE	BIT(1)	/* No erase command needed */
#define SST_WRITE		BIT(2)	/* use SST byte programming */
#define SPI_NOR_NO_FR		BIT(3)	/* Can't do fastread */
#define SECT_4K_PMC		BIT(4)	/* SPINOR_OP_BE_4K_PMC works uniformly */
#define SPI_NOR_DUAL_READ	BIT(5)	/* Flash supports Dual Read */
#define SPI_NOR_QUAD_READ	BIT(6)	/* Flash supports Quad Read */
#define USE_FSR			BIT(7)	/* use flag status register */
#define SPI_NOR_HAS_LOCK	BIT(8)	/* Flash supports lock/unlock via SR */
#define SPI_NOR_HAS_TB		BIT(9)	/*
					 * Flash SR has Top/Bottom (TB) protect
					 * bit. Must be used with
					 * SPI_NOR_HAS_LOCK.
					 */
#define SPI_NOR_QUAD_PP		BIT(10)	/* Flash supports Quad Page Program */
#define SPI_NOR_4IO_PP		BIT(11)	/* Flash supports 4IO Page Program */
#define SPI_NOR_QPI_RW		BIT(12)	/*
					 * Flash supports QPI Read &
					 * Page Program
					 */
};

#ifdef CONFIG_SPI_RTS_QUADSPI_SWP
/*
 * struct spi_swp - Structure for software write protection
 * @time:		last erase or write operation jiffies
 * @swp_flag:		flag for software write protection status
 * @channel_mode:	channel mode
 * @enable_swp_task:	enable software write protection thread
 */
struct spi_swp {
	unsigned long time;
	bool swp_flag;
	enum read_mode channel_mode;
	struct task_struct *enable_swp_task;
} spi_nor_swp;
#endif

#define JEDEC_MFR(info)	((info)->id[0])
#define JEDEC_MFR2(_jedec_id)	((_jedec_id) >> 16)

static const struct flash_info *spi_nor_match_id(const char *name);

/*
 * Read the status register, returning its value in the location
 * Return the status register value.
 * Returns negative if error occurred.
 */
static int read_sr(struct spi_nor *nor)
{
	int ret;
	u8 val;

	ret = nor->read_reg(nor, SPINOR_OP_RDSR, &val, 1);
	if (ret < 0) {
		pr_err("error %d reading SR\n", (int) ret);
		return ret;
	}

	return val;
}

/*
 * Read the flag status register, returning its value in the location
 * Return the status register value.
 * Returns negative if error occurred.
 */
static int read_fsr(struct spi_nor *nor)
{
	int ret;
	u8 val;

	ret = nor->read_reg(nor, SPINOR_OP_RDFSR, &val, 1);
	if (ret < 0) {
		pr_err("error %d reading FSR\n", ret);
		return ret;
	}

	return val;
}

/*
 * Read configuration register, returning its value in the
 * location. Return the configuration register value.
 * Returns negative if error occured.
 */
static int read_cr(struct spi_nor *nor)
{
	int ret;
	u8 val;

	ret = nor->read_reg(nor, SPINOR_OP_RDCR, &val, 1);
	if (ret < 0) {
		dev_err(nor->dev, "error %d reading CR\n", ret);
		return ret;
	}

	return val;
}

/*
 * Dummy Cycle calculation for different type of read.
 * It can be used to support more commands with
 * different dummy cycle requirements.
 */
static inline int spi_nor_read_dummy_cycles(struct spi_nor *nor)
{
	int ret;

	switch (nor->flash_read) {
	case SPI_NOR_QPI:
		switch (JEDEC_MFR2(nor->jedec_id)) {
		case SNOR_MFR_MACRONIX:
			return 6;
		case SNOR_MFR_MICRON:
		case SNOR_MFR_EON:
			nor->cmd_buf[0] = 0x20; /* set 8 dummy clocks */
			ret = nor->write_reg(nor, SPINOR_OP_SET_RDPARA,
					nor->cmd_buf, 1);
			if (ret)
				dev_err(nor->dev, "%s(): set dummy failed!\n",
					__func__);
			return 8;
		case SNOR_MFR_WINBOND:
		case SNOR_MFR_GIGADEVICE:
			nor->cmd_buf[0] = 0xff; /* set 8 dummy clocks */
			ret = nor->write_reg(nor, SPINOR_OP_SET_RDPARA,
					nor->cmd_buf, 1);
			if (ret)
				dev_err(nor->dev, "%s(): set dummy failed!\n",
					__func__);
			return 8;
		}
	case SPI_NOR_FAST:
	case SPI_NOR_DUAL:
	case SPI_NOR_QUAD:
		return 8;
	case SPI_NOR_NORMAL:
		return 0;
	}
	return 0;
}

/*
 * Write status register 1 byte
 * Returns negative if error occurred.
 */
static inline int write_sr(struct spi_nor *nor, u8 val)
{
	nor->cmd_buf[0] = val;
	return nor->write_reg(nor, SPINOR_OP_WRSR, nor->cmd_buf, 1);
}

/*
 * Set write enable latch with Write Enable command.
 * Returns negative if error occurred.
 */
static inline int write_enable(struct spi_nor *nor)
{
	return nor->write_reg(nor, SPINOR_OP_WREN, NULL, 0);
}

/*
 * Send write disble instruction to the chip.
 */
static inline int write_disable(struct spi_nor *nor)
{
	return nor->write_reg(nor, SPINOR_OP_WRDI, NULL, 0);
}

static inline struct spi_nor *mtd_to_spi_nor(struct mtd_info *mtd)
{
	return mtd->priv;
}

static int winbond_quad_enable(struct spi_nor *nor);

/* Enable/disable qpi mode. */
static inline int set_qpi(struct spi_nor *nor,
			const struct flash_info *info, int enable)
{
	int status;
	u8 cmd;

	switch (JEDEC_MFR(info)) {
	case SNOR_MFR_MACRONIX:
		cmd = enable ? SPINOR_OP_MXIC_ENQPI : SPINOR_OP_MXIC_EXQPI;
		return nor->write_reg(nor, cmd, NULL, 0);
	case SNOR_MFR_GIGADEVICE:
	case SNOR_MFR_WINBOND:

		/* QE bit is required to be set to a 1 before
		 * issuing an "Enter QPI (38h)" to switch the
		 * device to QPI
		 */
		status = winbond_quad_enable(nor);
		if (status) {
			dev_err(nor->dev, "%s(): Winbond quad-read not enabled\n",
					__func__);
			return -EINVAL;
		}

	case SNOR_MFR_MICRON:
	case SNOR_MFR_EON:
		cmd = enable ? SPINOR_OP_WINBOND_ENQPI :
				SPINOR_OP_WINBOND_EXQPI;
		return nor->write_reg(nor, cmd, NULL, 0);
	default:
		return -EINVAL;
	}
}

/* Enable/disable 4-byte addressing mode. */
static inline int set_4byte(struct spi_nor *nor, const struct flash_info *info,
			    int enable)
{
	int status;
	bool need_wren = false;
	u8 cmd;

	switch (JEDEC_MFR(info)) {
	case SNOR_MFR_MICRON:
		/* Some Micron need WREN command; all will accept it */
		need_wren = true;
	case SNOR_MFR_MACRONIX:
	case SNOR_MFR_GIGADEVICE:
	case SNOR_MFR_WINBOND:
		if (need_wren)
			write_enable(nor);

		cmd = enable ? SPINOR_OP_EN4B : SPINOR_OP_EX4B;
		status = nor->write_reg(nor, cmd, NULL, 0);
		if (need_wren)
			write_disable(nor);

		return status;
	default:
		/* Spansion style */
		nor->cmd_buf[0] = enable << 7;
		return nor->write_reg(nor, SPINOR_OP_BRWR, nor->cmd_buf, 1);
	}
}
static inline int spi_nor_sr_ready(struct spi_nor *nor)
{
	int sr = read_sr(nor);
	if (sr < 0)
		return sr;
	else
		return !(sr & SR_WIP);
}

static inline int spi_nor_fsr_ready(struct spi_nor *nor)
{
	int fsr = read_fsr(nor);
	if (fsr < 0)
		return fsr;
	else
		return fsr & FSR_READY;
}

static int spi_nor_ready(struct spi_nor *nor)
{
	int sr, fsr;
	sr = spi_nor_sr_ready(nor);
	if (sr < 0)
		return sr;
	fsr = nor->flags & SNOR_F_USE_FSR ? spi_nor_fsr_ready(nor) : 1;
	if (fsr < 0)
		return fsr;
	return sr && fsr;
}

/*
 * Service routine to read status register until ready, or timeout occurs.
 * Returns non-zero if error.
 */
static int spi_nor_wait_till_ready_with_timeout(struct spi_nor *nor,
						unsigned long timeout_jiffies)
{
	unsigned long deadline;
	int timeout = 0, ret;

	deadline = jiffies + timeout_jiffies;

	while (!timeout) {
		if (time_after_eq(jiffies, deadline))
			timeout = 1;

		ret = spi_nor_ready(nor);
		if (ret < 0)
			return ret;
		if (ret)
			return 0;

		cond_resched();
	}

	dev_err(nor->dev, "flash operation timed out\n");

	return -ETIMEDOUT;
}

static int spi_nor_wait_till_ready(struct spi_nor *nor)
{
	return spi_nor_wait_till_ready_with_timeout(nor,
						    DEFAULT_READY_WAIT_JIFFIES);
}

/*
 * Erase the whole flash memory
 *
 * Returns 0 if successful, non-zero otherwise.
 */
static int erase_chip(struct spi_nor *nor)
{
	dev_dbg(nor->dev, " %lldKiB\n", (long long)(nor->mtd.size >> 10));

	return nor->write_reg(nor, SPINOR_OP_CHIP_ERASE, NULL, 0);
}

static int spi_nor_lock_and_prep(struct spi_nor *nor, enum spi_nor_ops ops)
{
	int ret = 0;

	mutex_lock(&nor->lock);

	if (nor->prepare) {
		ret = nor->prepare(nor, ops);
		if (ret) {
			dev_err(nor->dev, "failed in the preparation.\n");
			mutex_unlock(&nor->lock);
			return ret;
		}
	}
	return ret;
}

static void spi_nor_unlock_and_unprep(struct spi_nor *nor, enum spi_nor_ops ops)
{
	if (nor->unprepare)
		nor->unprepare(nor, ops);
	mutex_unlock(&nor->lock);
}

#ifdef CONFIG_SPI_RTS_QUADSPI_SWP
/*
 * Common enable software write protection
 * Now Macronix, Giga device, Winbond, XMC are the same.
 * Returns negative if error occurred.
 */
static int common_swp_enable(struct spi_nor *nor)
{
	int ret;
	u8 val;

	val = read_sr(nor);
	if (val < 0) {
		pr_err("error %d reading SR\n", (int) val);
		return val;
	}

	ret = spi_nor_wait_till_ready(nor);
	if (ret)
		return ret;
	write_enable(nor);
	val = val | SR_BP0 | SR_BP1 | SR_BP2 | SR_BP3 | SR_SRWD;
	ret = write_sr(nor, val);
	if (ret < 0) {
		pr_err("error %d writing SR\n", (int) ret);
		return ret;
	}

	return 0;
}

static int common_swp_disable(struct spi_nor *nor)
{
	int ret;
	u8 val;

	val = read_sr(nor);
	if (val < 0) {
		pr_err("error %d reading SR\n", (int) val);
		return val;
	}

	ret = spi_nor_wait_till_ready(nor);
	if (ret)
		return ret;
	write_enable(nor);
	val = (val & ~(SR_BP0 | SR_BP1 | SR_BP2 | SR_BP3)) | SR_SRWD;
	ret = write_sr(nor, val);
	if (ret < 0) {
		pr_err("error %d writing SR\n", (int) ret);
		return ret;
	}

	return 0;
}

static int spi_swp_enable(struct spi_nor *nor)
{
	int status = 0;

	switch (JEDEC_MFR2(nor->jedec_id)) {
	case SNOR_MFR_MACRONIX:
	case SNOR_MFR_GIGADEVICE:
	case SNOR_MFR_WINBOND:
	case SNOR_MFR_MICRON:
	case SNOR_MFR_EON:
	case SNOR_MFR_BOYAMICRO:
	case SNOR_MFR_XTX:
		status = common_swp_enable(nor);
		if (status) {
			dev_err(nor->dev, "wp not enabled.\n");
			status = -EINVAL;
		}
		break;
	default:
		dev_err(nor->dev, "chip type %x not supported.\n",
			JEDEC_MFR2(nor->jedec_id));
		status = -EINVAL;
		break;
	}
	return status;
}

static int spi_swp_disable(struct spi_nor *nor)
{
	int status = 0;

	switch (JEDEC_MFR2(nor->jedec_id)) {
	case SNOR_MFR_MACRONIX:
	case SNOR_MFR_GIGADEVICE:
	case SNOR_MFR_WINBOND:
	case SNOR_MFR_MICRON:
	case SNOR_MFR_EON:
	case SNOR_MFR_BOYAMICRO:
	case SNOR_MFR_XTX:
		status = common_swp_disable(nor);
		if (status) {
			dev_err(nor->dev, "wp not disabled.\n");
			status = -EINVAL;
		}
		break;
	default:
		dev_err(nor->dev, "chip type %x not supported.\n",
			JEDEC_MFR2(nor->jedec_id));
		status = -EINVAL;
		break;
	}
	return status;
}

int enable_swp_thread(void *data)
{
	struct spi_nor *nor = (struct spi_nor *)data;

	while (1) {
		mutex_lock(&nor->lock);
		if (!spi_nor_swp.swp_flag &&
			time_after_eq(jiffies, spi_nor_swp.time + HZ)) {
			if (!spi_swp_enable(nor))
				spi_nor_swp.swp_flag = true;
			else
				dev_err(nor->dev, "enable swp failed\n");
		}
		mutex_unlock(&nor->lock);
		msleep(1000);
	}
}

static void spi_nor_init_swp(struct spi_nor *nor, enum read_mode mode)
{
	spi_nor_swp.channel_mode = mode;
	spi_nor_swp.time = jiffies;
	spi_nor_swp.swp_flag = false;

	spi_nor_swp.enable_swp_task = kthread_create(enable_swp_thread,
				(void *)nor, "enable_swp_task");
	if (IS_ERR(spi_nor_swp.enable_swp_task)) {
		dev_err(nor->dev, "Unable to start kernel thread.\n");
		spi_nor_swp.enable_swp_task = NULL;
	} else {
		wake_up_process(spi_nor_swp.enable_swp_task);
	}
}
#endif

/*
 * Initiate the erasure of a single sector
 */
static int spi_nor_erase_sector(struct spi_nor *nor, u32 addr)
{
	u8 buf[SPI_NOR_MAX_ADDR_WIDTH];
	int i;

	if (nor->erase)
		return nor->erase(nor, addr);

	/*
	 * Default implementation, if driver doesn't have a specialized HW
	 * control
	 */
	for (i = nor->addr_width - 1; i >= 0; i--) {
		buf[i] = addr & 0xff;
		addr >>= 8;
	}

	return nor->write_reg(nor, nor->erase_opcode, buf, nor->addr_width);
}

/*
 * Erase an address range on the nor chip.  The address range may extend
 * one or more erase sectors.  Return an error is there is a problem erasing.
 */
static int spi_nor_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	struct spi_nor *nor = mtd_to_spi_nor(mtd);
	u32 addr, len;
	uint32_t rem;
	int ret;

	dev_dbg(nor->dev, "at 0x%llx, len %lld\n", (long long)instr->addr,
			(long long)instr->len);

	div_u64_rem(instr->len, mtd->erasesize, &rem);
	if (rem)
		return -EINVAL;

	addr = instr->addr;
	len = instr->len;

	ret = spi_nor_lock_and_prep(nor, SPI_NOR_OPS_ERASE);
	if (ret)
		return ret;

#ifdef CONFIG_SPI_RTS_QUADSPI_SWP
	if (spi_nor_swp.channel_mode < SPI_NOR_QUAD) {
		if (spi_nor_swp.swp_flag) {
			ret = spi_swp_disable(nor);
			if (!ret) {
				spi_nor_swp.swp_flag = false;
			} else {
				dev_err(nor->dev, "disable swp failed\n");
				goto erase_err;
			}
		}
		spi_nor_swp.time = jiffies;
	}
#endif

	/* whole-chip erase? */
	if (len == mtd->size) {
		unsigned long timeout;

		write_enable(nor);

		if (erase_chip(nor)) {
			ret = -EIO;
			goto erase_err;
		}

		/*
		 * Scale the timeout linearly with the size of the flash, with
		 * a minimum calibrated to an old 2MB flash. We could try to
		 * pull these from CFI/SFDP, but these values should be good
		 * enough for now.
		 */
		timeout = max(CHIP_ERASE_2MB_READY_WAIT_JIFFIES,
			      CHIP_ERASE_2MB_READY_WAIT_JIFFIES *
			      (unsigned long)(mtd->size / SZ_2M));
		ret = spi_nor_wait_till_ready_with_timeout(nor, timeout);
		if (ret)
			goto erase_err;

	/* REVISIT in some cases we could speed up erasing large regions
	 * by using SPINOR_OP_SE instead of SPINOR_OP_BE_4K.  We may have set up
	 * to use "small sector erase", but that's not always optimal.
	 */

	/* "sector"-at-a-time erase */
	} else {
		while (len) {
			write_enable(nor);

			ret = spi_nor_erase_sector(nor, addr);
			if (ret)
				goto erase_err;

			addr += mtd->erasesize;
			len -= mtd->erasesize;

			ret = spi_nor_wait_till_ready(nor);
			if (ret)
				goto erase_err;
		}
	}

	write_disable(nor);

erase_err:
	spi_nor_unlock_and_unprep(nor, SPI_NOR_OPS_ERASE);

	instr->state = ret ? MTD_ERASE_FAILED : MTD_ERASE_DONE;
	mtd_erase_callback(instr);

	return ret;
}

static void stm_get_locked_range(struct spi_nor *nor, u8 sr, loff_t *ofs,
				 uint64_t *len)
{
	struct mtd_info *mtd = &nor->mtd;
	u8 mask = SR_BP2 | SR_BP1 | SR_BP0;
	int shift = ffs(mask) - 1;
	int pow;

	if (!(sr & mask)) {
		/* No protection */
		*ofs = 0;
		*len = 0;
	} else {
		pow = ((sr & mask) ^ mask) >> shift;
		*len = mtd->size >> pow;
		if (nor->flags & SNOR_F_HAS_SR_TB && sr & SR_TB)
			*ofs = 0;
		else
			*ofs = mtd->size - *len;
	}
}

/*
 * Return 1 if the entire region is locked (if @locked is true) or unlocked (if
 * @locked is false); 0 otherwise
 */
static int stm_check_lock_status_sr(struct spi_nor *nor, loff_t ofs, uint64_t len,
				    u8 sr, bool locked)
{
	loff_t lock_offs;
	uint64_t lock_len;

	if (!len)
		return 1;

	stm_get_locked_range(nor, sr, &lock_offs, &lock_len);

	if (locked)
		/* Requested range is a sub-range of locked range */
		return (ofs + len <= lock_offs + lock_len) && (ofs >= lock_offs);
	else
		/* Requested range does not overlap with locked range */
		return (ofs >= lock_offs + lock_len) || (ofs + len <= lock_offs);
}

static int stm_is_locked_sr(struct spi_nor *nor, loff_t ofs, uint64_t len,
			    u8 sr)
{
	return stm_check_lock_status_sr(nor, ofs, len, sr, true);
}

static int stm_is_unlocked_sr(struct spi_nor *nor, loff_t ofs, uint64_t len,
			      u8 sr)
{
	return stm_check_lock_status_sr(nor, ofs, len, sr, false);
}

/*
 * Lock a region of the flash. Compatible with ST Micro and similar flash.
 * Supports the block protection bits BP{0,1,2} in the status register
 * (SR). Does not support these features found in newer SR bitfields:
 *   - SEC: sector/block protect - only handle SEC=0 (block protect)
 *   - CMP: complement protect - only support CMP=0 (range is not complemented)
 *
 * Support for the following is provided conditionally for some flash:
 *   - TB: top/bottom protect
 *
 * Sample table portion for 8MB flash (Winbond w25q64fw):
 *
 *   SEC  |  TB   |  BP2  |  BP1  |  BP0  |  Prot Length  | Protected Portion
 *  --------------------------------------------------------------------------
 *    X   |   X   |   0   |   0   |   0   |  NONE         | NONE
 *    0   |   0   |   0   |   0   |   1   |  128 KB       | Upper 1/64
 *    0   |   0   |   0   |   1   |   0   |  256 KB       | Upper 1/32
 *    0   |   0   |   0   |   1   |   1   |  512 KB       | Upper 1/16
 *    0   |   0   |   1   |   0   |   0   |  1 MB         | Upper 1/8
 *    0   |   0   |   1   |   0   |   1   |  2 MB         | Upper 1/4
 *    0   |   0   |   1   |   1   |   0   |  4 MB         | Upper 1/2
 *    X   |   X   |   1   |   1   |   1   |  8 MB         | ALL
 *  ------|-------|-------|-------|-------|---------------|-------------------
 *    0   |   1   |   0   |   0   |   1   |  128 KB       | Lower 1/64
 *    0   |   1   |   0   |   1   |   0   |  256 KB       | Lower 1/32
 *    0   |   1   |   0   |   1   |   1   |  512 KB       | Lower 1/16
 *    0   |   1   |   1   |   0   |   0   |  1 MB         | Lower 1/8
 *    0   |   1   |   1   |   0   |   1   |  2 MB         | Lower 1/4
 *    0   |   1   |   1   |   1   |   0   |  4 MB         | Lower 1/2
 *
 * Returns negative on errors, 0 on success.
 */
static int stm_lock(struct spi_nor *nor, loff_t ofs, uint64_t len)
{
	struct mtd_info *mtd = &nor->mtd;
	int status_old, status_new;
	u8 mask = SR_BP2 | SR_BP1 | SR_BP0;
	u8 shift = ffs(mask) - 1, pow, val;
	loff_t lock_len;
	bool can_be_top = true, can_be_bottom = nor->flags & SNOR_F_HAS_SR_TB;
	bool use_top;
	int ret;

	status_old = read_sr(nor);
	if (status_old < 0)
		return status_old;

	/* If nothing in our range is unlocked, we don't need to do anything */
	if (stm_is_locked_sr(nor, ofs, len, status_old))
		return 0;

	/* If anything below us is unlocked, we can't use 'bottom' protection */
	if (!stm_is_locked_sr(nor, 0, ofs, status_old))
		can_be_bottom = false;

	/* If anything above us is unlocked, we can't use 'top' protection */
	if (!stm_is_locked_sr(nor, ofs + len, mtd->size - (ofs + len),
				status_old))
		can_be_top = false;

	if (!can_be_bottom && !can_be_top)
		return -EINVAL;

	/* Prefer top, if both are valid */
	use_top = can_be_top;

	/* lock_len: length of region that should end up locked */
	if (use_top)
		lock_len = mtd->size - ofs;
	else
		lock_len = ofs + len;

	/*
	 * Need smallest pow such that:
	 *
	 *   1 / (2^pow) <= (len / size)
	 *
	 * so (assuming power-of-2 size) we do:
	 *
	 *   pow = ceil(log2(size / len)) = log2(size) - floor(log2(len))
	 */
	pow = ilog2(mtd->size) - ilog2(lock_len);
	val = mask - (pow << shift);
	if (val & ~mask)
		return -EINVAL;
	/* Don't "lock" with no region! */
	if (!(val & mask))
		return -EINVAL;

	status_new = (status_old & ~mask & ~SR_TB) | val;

	/* Disallow further writes if WP pin is asserted */
	status_new |= SR_SRWD;

	if (!use_top)
		status_new |= SR_TB;

	/* Don't bother if they're the same */
	if (status_new == status_old)
		return 0;

	/* Only modify protection if it will not unlock other areas */
	if ((status_new & mask) < (status_old & mask))
		return -EINVAL;

	write_enable(nor);
	ret = write_sr(nor, status_new);
	if (ret)
		return ret;
	return spi_nor_wait_till_ready(nor);
}

/*
 * Unlock a region of the flash. See stm_lock() for more info
 *
 * Returns negative on errors, 0 on success.
 */
static int stm_unlock(struct spi_nor *nor, loff_t ofs, uint64_t len)
{
	struct mtd_info *mtd = &nor->mtd;
	int status_old, status_new;
	u8 mask = SR_BP2 | SR_BP1 | SR_BP0;
	u8 shift = ffs(mask) - 1, pow, val;
	loff_t lock_len;
	bool can_be_top = true, can_be_bottom = nor->flags & SNOR_F_HAS_SR_TB;
	bool use_top;
	int ret;

	status_old = read_sr(nor);
	if (status_old < 0)
		return status_old;

	/* If nothing in our range is locked, we don't need to do anything */
	if (stm_is_unlocked_sr(nor, ofs, len, status_old))
		return 0;

	/* If anything below us is locked, we can't use 'top' protection */
	if (!stm_is_unlocked_sr(nor, 0, ofs, status_old))
		can_be_top = false;

	/* If anything above us is locked, we can't use 'bottom' protection */
	if (!stm_is_unlocked_sr(nor, ofs + len, mtd->size - (ofs + len),
				status_old))
		can_be_bottom = false;

	if (!can_be_bottom && !can_be_top)
		return -EINVAL;

	/* Prefer top, if both are valid */
	use_top = can_be_top;

	/* lock_len: length of region that should remain locked */
	if (use_top)
		lock_len = mtd->size - (ofs + len);
	else
		lock_len = ofs;

	/*
	 * Need largest pow such that:
	 *
	 *   1 / (2^pow) >= (len / size)
	 *
	 * so (assuming power-of-2 size) we do:
	 *
	 *   pow = floor(log2(size / len)) = log2(size) - ceil(log2(len))
	 */
	pow = ilog2(mtd->size) - order_base_2(lock_len);
	if (lock_len == 0) {
		val = 0; /* fully unlocked */
	} else {
		val = mask - (pow << shift);
		/* Some power-of-two sizes are not supported */
		if (val & ~mask)
			return -EINVAL;
	}

	status_new = (status_old & ~mask & ~SR_TB) | val;

	/* Don't protect status register if we're fully unlocked */
	if (lock_len == 0)
		status_new &= ~SR_SRWD;

	if (!use_top)
		status_new |= SR_TB;

	/* Don't bother if they're the same */
	if (status_new == status_old)
		return 0;

	/* Only modify protection if it will not lock other areas */
	if ((status_new & mask) > (status_old & mask))
		return -EINVAL;

	write_enable(nor);
	ret = write_sr(nor, status_new);
	if (ret)
		return ret;
	return spi_nor_wait_till_ready(nor);
}

/*
 * Check if a region of the flash is (completely) locked. See stm_lock() for
 * more info.
 *
 * Returns 1 if entire region is locked, 0 if any portion is unlocked, and
 * negative on errors.
 */
static int stm_is_locked(struct spi_nor *nor, loff_t ofs, uint64_t len)
{
	int status;

	status = read_sr(nor);
	if (status < 0)
		return status;

	return stm_is_locked_sr(nor, ofs, len, status);
}

static int spi_nor_lock(struct mtd_info *mtd, loff_t ofs, uint64_t len)
{
	struct spi_nor *nor = mtd_to_spi_nor(mtd);
	int ret;

	ret = spi_nor_lock_and_prep(nor, SPI_NOR_OPS_LOCK);
	if (ret)
		return ret;

	ret = nor->flash_lock(nor, ofs, len);

	spi_nor_unlock_and_unprep(nor, SPI_NOR_OPS_UNLOCK);
	return ret;
}

static int spi_nor_unlock(struct mtd_info *mtd, loff_t ofs, uint64_t len)
{
	struct spi_nor *nor = mtd_to_spi_nor(mtd);
	int ret;

	ret = spi_nor_lock_and_prep(nor, SPI_NOR_OPS_UNLOCK);
	if (ret)
		return ret;

	ret = nor->flash_unlock(nor, ofs, len);

	spi_nor_unlock_and_unprep(nor, SPI_NOR_OPS_LOCK);
	return ret;
}

static int spi_nor_is_locked(struct mtd_info *mtd, loff_t ofs, uint64_t len)
{
	struct spi_nor *nor = mtd_to_spi_nor(mtd);
	int ret;

	ret = spi_nor_lock_and_prep(nor, SPI_NOR_OPS_UNLOCK);
	if (ret)
		return ret;

	ret = nor->flash_is_locked(nor, ofs, len);

	spi_nor_unlock_and_unprep(nor, SPI_NOR_OPS_LOCK);
	return ret;
}

/* Used when the "_ext_id" is two bytes at most */
#define INFO(_jedec_id, _ext_id, _sector_size, _n_sectors, _flags)	\
		.id = {							\
			((_jedec_id) >> 16) & 0xff,			\
			((_jedec_id) >> 8) & 0xff,			\
			(_jedec_id) & 0xff,				\
			((_ext_id) >> 8) & 0xff,			\
			(_ext_id) & 0xff,				\
			},						\
		.id_len = (!(_jedec_id) ? 0 : (3 + ((_ext_id) ? 2 : 0))),	\
		.sector_size = (_sector_size),				\
		.n_sectors = (_n_sectors),				\
		.page_size = 256,					\
		.flags = (_flags),

#define INFO6(_jedec_id, _ext_id, _sector_size, _n_sectors, _flags)	\
		.id = {							\
			((_jedec_id) >> 16) & 0xff,			\
			((_jedec_id) >> 8) & 0xff,			\
			(_jedec_id) & 0xff,				\
			((_ext_id) >> 16) & 0xff,			\
			((_ext_id) >> 8) & 0xff,			\
			(_ext_id) & 0xff,				\
			},						\
		.id_len = 6,						\
		.sector_size = (_sector_size),				\
		.n_sectors = (_n_sectors),				\
		.page_size = 256,					\
		.flags = (_flags),

#define CAT25_INFO(_sector_size, _n_sectors, _page_size, _addr_width, _flags)	\
		.sector_size = (_sector_size),				\
		.n_sectors = (_n_sectors),				\
		.page_size = (_page_size),				\
		.addr_width = (_addr_width),				\
		.flags = (_flags),

/* NOTE: double check command sets and memory organization when you add
 * more nor chips.  This current list focusses on newer chips, which
 * have been converging on command sets which including JEDEC ID.
 *
 * All newly added entries should describe *hardware* and should use SECT_4K
 * (or SECT_4K_PMC) if hardware supports erasing 4 KiB sectors. For usage
 * scenarios excluding small sectors there is config option that can be
 * disabled: CONFIG_MTD_SPI_NOR_USE_4K_SECTORS.
 * For historical (and compatibility) reasons (before we got above config) some
 * old entries may be missing 4K flag.
 */
static const struct flash_info spi_nor_ids[] = {
	/* Macronix */
/*{ "kh25l6406e",  INFO(0xc22017, 0, 64 * 1024, 128, SPI_NOR_DUAL_READ) },*/
	{ "mx25l6433f",  INFO(0xc22017, 0, 64 * 1024, 128, SPI_NOR_DUAL_READ
		| SPI_NOR_QUAD_READ | SPI_NOR_4IO_PP) },
	{ "mx25l12835f", INFO(0xc22018, 0, 64 * 1024, 256, SPI_NOR_DUAL_READ
		| SPI_NOR_QUAD_READ | SPI_NOR_4IO_PP | SPI_NOR_QPI_RW) },
	{ "mx25l25635f", INFO(0xc22019, 0, 64 * 1024, 512, SPI_NOR_DUAL_READ
		| SPI_NOR_QUAD_READ | SPI_NOR_4IO_PP | SPI_NOR_QPI_RW) },
	{ "mx25l51245g", INFO(0xc2201a, 0, 64 * 1024, 1024, SPI_NOR_DUAL_READ
		| SPI_NOR_QUAD_READ | SPI_NOR_4IO_PP | SPI_NOR_QPI_RW) },
	{ "mx66l1g45g",  INFO(0xc2201b, 0, 64 * 1024, 2048, SPI_NOR_QUAD_READ
		| SPI_NOR_QUAD_READ | SPI_NOR_4IO_PP | SPI_NOR_QPI_RW) },

	/* Winbond -- w25x "blocks" are 64K, "sectors" are 4KiB */
	{ "w25q64fv", INFO(0xef4017, 0, 64 * 1024, 128, SPI_NOR_DUAL_READ
		| SPI_NOR_QUAD_READ | SPI_NOR_QUAD_PP | SPI_NOR_QPI_RW) },
	{ "w25q128fv", INFO(0xef4018, 0, 64 * 1024, 256, SPI_NOR_DUAL_READ
		| SPI_NOR_QUAD_READ | SPI_NOR_QUAD_PP | SPI_NOR_QPI_RW) },
	{ "w25q256fv", INFO(0xef4019, 0, 64 * 1024, 512, SPI_NOR_DUAL_READ
		| SPI_NOR_QUAD_READ | SPI_NOR_QUAD_PP | SPI_NOR_QPI_RW) },

	/* Giga Device */
	{ "gd25q64c", INFO(0xc84017, 0, 64 * 1024, 128, SPI_NOR_DUAL_READ
		| SPI_NOR_QUAD_READ | SPI_NOR_QUAD_PP) },
	{ "gd25q128c", INFO(0xc84018, 0, 64 * 1024, 256, SPI_NOR_DUAL_READ
		| SPI_NOR_QUAD_READ | SPI_NOR_QUAD_PP | SPI_NOR_QPI_RW) },
	{ "gd25q256c", INFO(0xc84019, 0, 64 * 1024, 512, SPI_NOR_DUAL_READ
		| SPI_NOR_QUAD_READ | SPI_NOR_QUAD_PP) },

	/* BYT is the same with HUAHONG*/
	{ "by25q64as/bh25q64", INFO(0x684017, 0, 64 * 1024, 128,
		SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ | SPI_NOR_QUAD_PP) },
	{ "by25q128as/bh25q128", INFO(0x684018, 0, 64 * 1024, 256,
		SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ | SPI_NOR_QUAD_PP) },

	/* XMC */
	{ "XM25QH64A", INFO(0x207017, 0, 64 * 1024, 128, SPI_NOR_DUAL_READ
		| SPI_NOR_QUAD_READ | SPI_NOR_QUAD_PP | SPI_NOR_QPI_RW) },
	{ "XM25QH128A", INFO(0x207018, 0, 64 * 1024, 256, SPI_NOR_DUAL_READ
		| SPI_NOR_QUAD_READ | SPI_NOR_QUAD_PP | SPI_NOR_QPI_RW) },
	{ "XM25QH256B", INFO(0x206019, 0, 64 * 1024, 512, SPI_NOR_DUAL_READ
		| SPI_NOR_QUAD_READ | SPI_NOR_QUAD_PP) },

	/*EON*/
	{ "EN25QH64A", INFO(0x1c7017, 0, 64 * 1024, 128, SPI_NOR_DUAL_READ
		| SPI_NOR_QUAD_READ | SPI_NOR_QUAD_PP | SPI_NOR_QPI_RW) },
	{ "EN25QH128A", INFO(0x1c7018, 0, 64 * 1024, 256, SPI_NOR_DUAL_READ
		| SPI_NOR_QUAD_READ | SPI_NOR_QUAD_PP | SPI_NOR_QPI_RW) },

	/*XTX*/
	{ "XT25F64B", INFO(0x0b4017, 0, 64 * 1024, 128, SPI_NOR_DUAL_READ
		| SPI_NOR_QUAD_READ | SPI_NOR_QUAD_PP) },
	{ },
};

static const struct flash_info *spi_nor_read_id(struct spi_nor *nor)
{
	int			tmp;
	u8			id[SPI_NOR_MAX_ID_LEN];
	const struct flash_info	*info;

	tmp = nor->read_reg(nor, SPINOR_OP_RDID, id, SPI_NOR_MAX_ID_LEN);
	if (tmp < 0) {
		dev_dbg(nor->dev, "error %d reading JEDEC ID\n", tmp);
		return ERR_PTR(tmp);
	}

	for (tmp = 0; tmp < ARRAY_SIZE(spi_nor_ids) - 1; tmp++) {
		info = &spi_nor_ids[tmp];
		if (info->id_len) {
			if (!memcmp(info->id, id, info->id_len))
				return &spi_nor_ids[tmp];
		}
	}
	dev_err(nor->dev, "unrecognized JEDEC id bytes: %02x, %02x, %02x\n",
		id[0], id[1], id[2]);
	return ERR_PTR(-ENODEV);
}

static int spi_nor_read(struct mtd_info *mtd, loff_t from, size_t len,
			size_t *retlen, u_char *buf)
{
	struct spi_nor *nor = mtd_to_spi_nor(mtd);
	int ret;

	dev_dbg(nor->dev, "from 0x%08x, len %zd\n", (u32)from, len);

	ret = spi_nor_lock_and_prep(nor, SPI_NOR_OPS_READ);
	if (ret)
		return ret;

	while (len) {
		ret = nor->read(nor, from, len, buf);
		if (ret == 0) {
			/* We shouldn't see 0-length reads */
			ret = -EIO;
			goto read_err;
		}
		if (ret < 0)
			goto read_err;

		WARN_ON(ret > len);
		*retlen += ret;
		buf += ret;
		from += ret;
		len -= ret;
	}
	ret = 0;

read_err:
	spi_nor_unlock_and_unprep(nor, SPI_NOR_OPS_READ);
	return ret;
}

static int sst_write(struct mtd_info *mtd, loff_t to, size_t len,
		size_t *retlen, const u_char *buf)
{
	struct spi_nor *nor = mtd_to_spi_nor(mtd);
	size_t actual;
	int ret;

	dev_dbg(nor->dev, "to 0x%08x, len %zd\n", (u32)to, len);

	ret = spi_nor_lock_and_prep(nor, SPI_NOR_OPS_WRITE);
	if (ret)
		return ret;

	write_enable(nor);

	nor->sst_write_second = false;

	actual = to % 2;
	/* Start write from odd address. */
	if (actual) {
		nor->program_opcode = SPINOR_OP_BP;

		/* write one byte. */
		ret = nor->write(nor, to, 1, buf);
		if (ret < 0)
			goto sst_write_err;
		WARN(ret != 1, "While writing 1 byte written %i bytes\n",
		     (int)ret);
		ret = spi_nor_wait_till_ready(nor);
		if (ret)
			goto sst_write_err;
	}
	to += actual;

	/* Write out most of the data here. */
	for (; actual < len - 1; actual += 2) {
		nor->program_opcode = SPINOR_OP_AAI_WP;

		/* write two bytes. */
		ret = nor->write(nor, to, 2, buf + actual);
		if (ret < 0)
			goto sst_write_err;
		WARN(ret != 2, "While writing 2 bytes written %i bytes\n",
		     (int)ret);
		ret = spi_nor_wait_till_ready(nor);
		if (ret)
			goto sst_write_err;
		to += 2;
		nor->sst_write_second = true;
	}
	nor->sst_write_second = false;

	write_disable(nor);
	ret = spi_nor_wait_till_ready(nor);
	if (ret)
		goto sst_write_err;

	/* Write out trailing byte if it exists. */
	if (actual != len) {
		write_enable(nor);

		nor->program_opcode = SPINOR_OP_BP;
		ret = nor->write(nor, to, 1, buf + actual);
		if (ret < 0)
			goto sst_write_err;
		WARN(ret != 1, "While writing 1 byte written %i bytes\n",
		     (int)ret);
		ret = spi_nor_wait_till_ready(nor);
		if (ret)
			goto sst_write_err;
		write_disable(nor);
		actual += 1;
	}
sst_write_err:
	*retlen += actual;
	spi_nor_unlock_and_unprep(nor, SPI_NOR_OPS_WRITE);
	return ret;
}

/*
 * Write an address range to the nor chip.  Data must be written in
 * FLASH_PAGESIZE chunks.  The address range may be any size provided
 * it is within the physical boundaries.
 */
static int spi_nor_write(struct mtd_info *mtd, loff_t to, size_t len,
	size_t *retlen, const u_char *buf)
{
	struct spi_nor *nor = mtd_to_spi_nor(mtd);
	size_t page_offset, page_remain, i;
	ssize_t ret;

	dev_dbg(nor->dev, "to 0x%08x, len %zd\n", (u32)to, len);

	ret = spi_nor_lock_and_prep(nor, SPI_NOR_OPS_WRITE);
	if (ret)
		return ret;

#ifdef CONFIG_SPI_RTS_QUADSPI_SWP
	if (spi_nor_swp.channel_mode < SPI_NOR_QUAD) {
		if (spi_nor_swp.swp_flag) {
			ret = spi_swp_disable(nor);
			if (!ret) {
				spi_nor_swp.swp_flag = false;
			} else {
				dev_err(nor->dev, "disable swp failed\n");
				goto write_err;
			}
		}
		spi_nor_swp.time = jiffies;
	}
#endif

	for (i = 0; i < len; ) {
		ssize_t written;

		page_offset = (to + i) & (nor->page_size - 1);
		/*WARN_ONCE(page_offset,
			  "Writing at offset %zu into a NOR page. Writing partial pages may decrease reliability and increase wear of NOR flash.",
			  page_offset);
		 */
		/* the size of data remaining on the first page */
		page_remain = min_t(size_t,
				    nor->page_size - page_offset, len - i);

		ret = nor->write(nor, to + i, page_remain, buf + i);
		if (ret < 0)
			goto write_err;
		written = ret;

		*retlen += written;
		i += written;
		if (written != page_remain) {
			dev_err(nor->dev,
				"While writing %zu bytes written %zd bytes\n",
				page_remain, written);
			ret = -EIO;
			goto write_err;
		}
	}

	ret = spi_nor_wait_till_ready(nor);
	if (ret)
		goto write_err;


write_err:
	spi_nor_unlock_and_unprep(nor, SPI_NOR_OPS_WRITE);
	return ret;
}

static int macronix_quad_enable(struct spi_nor *nor)
{
	int ret, val;

	val = read_sr(nor);
	if (val < 0)
		return val;
	write_enable(nor);

	write_sr(nor, val | SR_QUAD_EN_MX);

	if (spi_nor_wait_till_ready(nor))
		return 1;

	ret = read_sr(nor);
	if (!(ret > 0 && (ret & SR_QUAD_EN_MX))) {
		dev_err(nor->dev, "Macronix Quad bit not set\n");
		return -EINVAL;
	}

	return 0;
}

static int winbond_quad_enable(struct spi_nor *nor)
{
	int ret;
	u8 val;

	ret = nor->read_reg(nor, SPINOR_OP_RDSR_2, &val, 1);
	if (ret < 0) {
		pr_err("error %d reading SR\n", (int) ret);
		return ret;
	}
	write_enable(nor);

	nor->cmd_buf[0] = val | SR_QUAD_EN_WB;
	nor->write_reg(nor, SPINOR_OP_WRSR_2, nor->cmd_buf, 1);

	if (spi_nor_wait_till_ready(nor))
		return 1;

	ret = nor->read_reg(nor, SPINOR_OP_RDSR_2, &val, 1);
	if (!(ret == 0 && (val & SR_QUAD_EN_WB))) {
		dev_err(nor->dev, "Winbond Quad bit not set\n");
		return -EINVAL;
	}

	return 0;
}

/*
 * Write status Register and configuration register with 2 bytes
 * The first byte will be written to the status register, while the
 * second byte will be written to the configuration register.
 * Return negative if error occured.
 */
static int write_sr_cr(struct spi_nor *nor, u16 val)
{
	nor->cmd_buf[0] = val & 0xff;
	nor->cmd_buf[1] = (val >> 8);

	return nor->write_reg(nor, SPINOR_OP_WRSR, nor->cmd_buf, 2);
}

static int spansion_quad_enable(struct spi_nor *nor)
{
	int ret;
	int quad_en = CR_QUAD_EN_SPAN << 8;

	write_enable(nor);

	ret = write_sr_cr(nor, quad_en);
	if (ret < 0) {
		dev_err(nor->dev,
			"error while writing configuration register\n");
		return -EINVAL;
	}

	ret = spi_nor_wait_till_ready(nor);
	if (ret) {
		dev_err(nor->dev,
			"timeout while writing configuration register\n");
		return ret;
	}

	/* read back and check it */
	ret = read_cr(nor);
	if (!(ret > 0 && (ret & CR_QUAD_EN_SPAN))) {
		dev_err(nor->dev, "Spansion Quad bit not set\n");
		return -EINVAL;
	}

	return 0;
}

static int set_quad_mode(struct spi_nor *nor, const struct flash_info *info)
{
	int status;

	switch (JEDEC_MFR(info)) {
	case SNOR_MFR_MACRONIX:
		status = macronix_quad_enable(nor);
		if (status) {
			dev_err(nor->dev, "Macronix quad-read not enabled\n");
			return -EINVAL;
		}
		return status;
	case SNOR_MFR_MICRON:
	case SNOR_MFR_EON:
		return 0;
	case SNOR_MFR_GIGADEVICE:
	case SNOR_MFR_WINBOND:
	case SNOR_MFR_BOYAMICRO:
	case SNOR_MFR_XTX:
		status = winbond_quad_enable(nor);
		if (status) {
			dev_err(nor->dev, "Winbond quad-read not enabled\n");
			return -EINVAL;
		}
		return status;
	default:
		status = spansion_quad_enable(nor);
		if (status) {
			dev_err(nor->dev, "Spansion quad-read not enabled\n");
			return -EINVAL;
		}
		return status;
	}
}

static int spi_nor_check(struct spi_nor *nor)
{
	if (!nor->dev || !nor->read || !nor->write ||
		!nor->read_reg || !nor->write_reg) {
		pr_err("spi-nor: please fill all the necessary fields!\n");
		return -EINVAL;
	}

	if (!nor->wait_till_ready)
		nor->wait_till_ready = spi_nor_wait_till_ready;

	return 0;
}

int spi_nor_scan(struct spi_nor *nor, const char *name, enum read_mode mode)
{
	const struct flash_info *info = NULL;
	struct device *dev = nor->dev;
	struct mtd_info *mtd = &nor->mtd;
	struct device_node *np = spi_nor_get_flash_node(nor);
	int ret;
	int i;

	ret = spi_nor_check(nor);
	if (ret)
		return ret;

	if (name)
		info = spi_nor_match_id(name);
	/* Try to auto-detect if chip name wasn't specified or not found */
	if (!info)
		info = spi_nor_read_id(nor);
	if (IS_ERR_OR_NULL(info))
		return -ENOENT;

	/*
	 * If caller has specified name of flash model that can normally be
	 * detected using JEDEC, let's verify it.
	 */
	if (name && info->id_len) {
		const struct flash_info *jinfo;

		jinfo = spi_nor_read_id(nor);
		if (IS_ERR(jinfo)) {
			return PTR_ERR(jinfo);
		} else if (jinfo != info) {
			/*
			 * JEDEC knows better, so overwrite platform ID. We
			 * can't trust partitions any longer, but we'll let
			 * mtd apply them anyway, since some partitions may be
			 * marked read-only, and we don't want to lose that
			 * information, even if it's not 100% accurate.
			 */
			dev_warn(dev, "found %s, expected %s\n",
				 jinfo->name, info->name);
			info = jinfo;
		}
	}

	nor->jedec_id = (info->id[0] << 16) | (info->id[1] << 8)
				| (info->id[2]);

	mutex_init(&nor->lock);

	/*
	 * Atmel, SST, Intel/Numonyx, and others serial NOR tend to power up
	 * with the software protection bits set
	 */

	if (JEDEC_MFR(info) == SNOR_MFR_ATMEL ||
	    JEDEC_MFR(info) == SNOR_MFR_INTEL ||
	    JEDEC_MFR(info) == SNOR_MFR_SST ||
	    info->flags & SPI_NOR_HAS_LOCK) {
		write_enable(nor);
		write_sr(nor, 0);
		spi_nor_wait_till_ready(nor);
	}

	if (!mtd->name)
		mtd->name = dev_name(dev);
	mtd->priv = nor;
	mtd->type = MTD_NORFLASH;
	mtd->writesize = 1;
	mtd->flags = MTD_CAP_NORFLASH;
	mtd->size = info->sector_size * info->n_sectors;
	mtd->_erase = spi_nor_erase;
	mtd->_read = spi_nor_read;

	/* NOR protection support for STmicro/Micron chips and similar */
	if (JEDEC_MFR(info) == SNOR_MFR_MICRON ||
			info->flags & SPI_NOR_HAS_LOCK) {
		nor->flash_lock = stm_lock;
		nor->flash_unlock = stm_unlock;
		nor->flash_is_locked = stm_is_locked;
	}

	if (nor->flash_lock && nor->flash_unlock && nor->flash_is_locked) {
		mtd->_lock = spi_nor_lock;
		mtd->_unlock = spi_nor_unlock;
		mtd->_is_locked = spi_nor_is_locked;
	}

	/* sst nor chips use AAI word program */
	if (info->flags & SST_WRITE)
		mtd->_write = sst_write;
	else
		mtd->_write = spi_nor_write;

	if (info->flags & USE_FSR)
		nor->flags |= SNOR_F_USE_FSR;
	if (info->flags & SPI_NOR_HAS_TB)
		nor->flags |= SNOR_F_HAS_SR_TB;

#ifdef CONFIG_MTD_SPI_NOR_USE_4K_SECTORS
	/* prefer "small sector" erase if possible */
	if (info->flags & SECT_4K) {
		nor->erase_opcode = SPINOR_OP_BE_4K;
		mtd->erasesize = 4096;
	} else if (info->flags & SECT_4K_PMC) {
		nor->erase_opcode = SPINOR_OP_BE_4K_PMC;
		mtd->erasesize = 4096;
	} else
#endif
	{
		nor->erase_opcode = SPINOR_OP_SE;
		mtd->erasesize = info->sector_size;
	}

	if (info->flags & SPI_NOR_NO_ERASE)
		mtd->flags |= MTD_NO_ERASE;

	mtd->dev.parent = dev;
	nor->page_size = info->page_size;
	mtd->writebufsize = nor->page_size;

	if (np) {
		/* If we were instantiated by DT, use it */
		if (of_property_read_bool(np, "m25p,fast-read"))
			nor->flash_read = SPI_NOR_FAST;
		else
			nor->flash_read = SPI_NOR_NORMAL;
	} else {
		/* If we weren't instantiated by DT, default to fast-read */
		nor->flash_read = SPI_NOR_FAST;
	}

	/* Some devices cannot do fast-read, no matter what DT tells us */
	if (info->flags & SPI_NOR_NO_FR)
		nor->flash_read = SPI_NOR_NORMAL;

	/* Quad/Dual-read mode takes precedence over fast/normal */
	if (mode == SPI_NOR_QPI && info->flags & SPI_NOR_QPI_RW) {
		ret = set_qpi(nor, info, 1);
		if (ret) {
			dev_err(dev, "qpi mode not supported\n");
			return ret;
		}
		nor->flash_read = SPI_NOR_QPI;
	} else if (mode == SPI_NOR_QUAD && info->flags & SPI_NOR_QUAD_READ) {
		ret = set_quad_mode(nor, info);
		if (ret) {
			dev_err(dev, "quad mode not supported\n");
			return ret;
		}
		nor->flash_read = SPI_NOR_QUAD;
		if (info->flags & SPI_NOR_QUAD_PP)
			nor->flash_pp = SPI_NOR_PP_QUAD;
		else if (info->flags & SPI_NOR_4IO_PP)
			nor->flash_pp = SPI_NOR_PP_4IO;
		else
			nor->flash_pp = SPI_NOR_PP_NORMAL;
	} else if (mode == SPI_NOR_DUAL && info->flags & SPI_NOR_DUAL_READ) {
		nor->flash_read = SPI_NOR_DUAL;
	}

	/* Default commands */
	switch (nor->flash_read) {
	case SPI_NOR_QPI:
		if (JEDEC_MFR(info) == 0xEF /* Winbond */ ||
			JEDEC_MFR(info) == 0xC8 /* Giga Device */)
			nor->read_opcode = SPINOR_OP_READ_FAST;
		else
			nor->read_opcode = SPINOR_OP_READ_1_4_4;
		break;
	case SPI_NOR_QUAD:
		nor->read_opcode = SPINOR_OP_READ_1_1_4;
		break;
	case SPI_NOR_DUAL:
		nor->read_opcode = SPINOR_OP_READ_1_1_2;
		break;
	case SPI_NOR_FAST:
		nor->read_opcode = SPINOR_OP_READ_FAST;
		break;
	case SPI_NOR_NORMAL:
		nor->read_opcode = SPINOR_OP_READ;
		break;
	default:
		dev_err(dev, "No Read opcode defined\n");
		return -EINVAL;
	}

	nor->program_opcode = SPINOR_OP_PP;

	/* Quad/4IO-PP mode takes precedence over normal */
	if (mode == SPI_NOR_QPI && info->flags & SPI_NOR_QPI_RW)
		nor->flash_pp = SPI_NOR_PP_QPI;
	else if (mode == SPI_NOR_QUAD && info->flags & SPI_NOR_QUAD_PP)
		nor->flash_pp = SPI_NOR_PP_QUAD;
	else if (mode == SPI_NOR_QUAD && info->flags & SPI_NOR_4IO_PP)
		nor->flash_pp = SPI_NOR_PP_4IO;
	else
		nor->flash_pp = SPI_NOR_PP_NORMAL;

	/* Default commands for Page Program */
	switch (nor->flash_pp) {
	case SPI_NOR_PP_NORMAL:
		nor->program_opcode = SPINOR_OP_PP;
		break;
	case SPI_NOR_PP_QUAD:
		nor->program_opcode = SPINOR_OP_PP_1_1_4;
		break;
	case SPI_NOR_PP_4IO:
		nor->program_opcode = SPINOR_OP_PP_1_4_4;
		break;
	case SPI_NOR_PP_QPI:
		nor->program_opcode = SPINOR_OP_PP;
		break;
	default:
		dev_err(dev, "No Page Program opcode defined\n");
		return -EINVAL;
	}

	if (info->addr_width)
		nor->addr_width = info->addr_width;
	else if (mtd->size > 0x1000000) {
		/* enable 4-byte addressing if the device exceeds 16MiB */
		nor->addr_width = 4;
		if (JEDEC_MFR(info) == SNOR_MFR_MACRONIX ||
			JEDEC_MFR(info) == SNOR_MFR_MICRON) {
			/* Dedicated 4-byte command set */
			switch (nor->flash_read) {
			case SPI_NOR_QPI:
				nor->read_opcode = SPINOR_OP_READ4_1_4_4;
				break;
			case SPI_NOR_QUAD:
				nor->read_opcode = SPINOR_OP_READ4_1_1_4;
				break;
			case SPI_NOR_DUAL:
				nor->read_opcode = SPINOR_OP_READ4_1_1_2;
				break;
			case SPI_NOR_FAST:
				nor->read_opcode = SPINOR_OP_READ4_FAST;
				break;
			case SPI_NOR_NORMAL:
				nor->read_opcode = SPINOR_OP_READ4;
				break;
			}

			/* Default commands for Page Program */
			switch (nor->flash_pp) {
			case SPI_NOR_PP_NORMAL:
				nor->program_opcode = SPINOR_OP_PP_4B;
				break;
			case SPI_NOR_PP_QUAD:
				nor->program_opcode = SPINOR_OP_QUAD_PP_4B;
				break;
			case SPI_NOR_PP_4IO:
				nor->program_opcode = SPINOR_OP_4IO_PP_4B;
				break;
			case SPI_NOR_PP_QPI:
				nor->program_opcode = SPINOR_OP_PP_4B;
				break;
			default:
				dev_err(dev, "No Page Program opcode defined\n");
				return -EINVAL;
			}
			/* No small sector erase for 4-byte command set */
			nor->erase_opcode = SPINOR_OP_SE_4B;
			mtd->erasesize = info->sector_size;
			set_4byte(nor, info, 1);
		} else if (JEDEC_MFR(info) == 0xEF /* Winbond */ ||
				JEDEC_MFR(info) == 0xC8 /* Giga Device */) {
			/* Winbond flash has same command for 3byte/4byte */
			mtd->erasesize = info->sector_size;

			set_4byte(nor, info, 1);
		} else {
			dev_err(dev, "Unsupported 4 byte mode, maf_id:%x\n",
				JEDEC_MFR(info));
			return -EINVAL;
		}
	} else {
		nor->addr_width = 3;
	}

	if (nor->addr_width > SPI_NOR_MAX_ADDR_WIDTH) {
		dev_err(dev, "address width is too large: %u\n",
			nor->addr_width);
		return -EINVAL;
	}

	nor->read_dummy = spi_nor_read_dummy_cycles(nor);

	dev_info(dev, "%s (%lld Kbytes)\n", info->name,
			(long long)mtd->size >> 10);

	dev_dbg(dev,
		"mtd .name = %s, .size = 0x%llx (%lldMiB), "
		".erasesize = 0x%.8x (%uKiB) .numeraseregions = %d\n",
		mtd->name, (long long)mtd->size, (long long)(mtd->size >> 20),
		mtd->erasesize, mtd->erasesize / 1024, mtd->numeraseregions);

	if (mtd->numeraseregions)
		for (i = 0; i < mtd->numeraseregions; i++)
			dev_dbg(dev,
				"mtd.eraseregions[%d] = { .offset = 0x%llx, "
				".erasesize = 0x%.8x (%uKiB), "
				".numblocks = %d }\n",
				i, (long long)mtd->eraseregions[i].offset,
				mtd->eraseregions[i].erasesize,
				mtd->eraseregions[i].erasesize / 1024,
				mtd->eraseregions[i].numblocks);

#ifdef CONFIG_SPI_RTS_QUADSPI_SWP
	if (mode < SPI_NOR_QUAD)
		spi_nor_init_swp(nor, mode);
#endif
	return 0;
}
EXPORT_SYMBOL_GPL(spi_nor_scan);

static const struct flash_info *spi_nor_match_id(const char *name)
{
	const struct flash_info *id = spi_nor_ids;

	while (id->name) {
		if (!strcmp(name, id->name))
			return id;
		id++;
	}
	return NULL;
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Huang Shijie <shijie8@gmail.com>");
MODULE_AUTHOR("Mike Lavender");
MODULE_DESCRIPTION("framework for SPI NOR");
