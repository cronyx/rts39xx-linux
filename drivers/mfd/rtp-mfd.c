/*
 * rtp-mfd.c  --  Realsil RTP
 *
 * Copyright 2013 Realsil Semiconductor Corp.
 *
 * Author: Wind Han <wind_han@realsil.com.cn>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/mfd/core.h>
#include <linux/mfd/rtp-mfd.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/regulator/machine.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/regulator/of_regulator.h>

/* #define pr_debug pr_info */

int rtp_ic_type;

static struct rtp_mfd_chip *g_chip;
static uint8_t rtp_reg_addr;

static struct mfd_cell rtps[] = {
	{
		.name = "rtp-pmic",
	},
	{
		.name = "rtp-rtc",
	},
	{
		.name = "rtp-power",
	},
};

int rtp_reg_read(struct rtp_mfd_chip *chip, uint8_t reg, uint8_t *val)
{
	unsigned int regval = 0;
	int ret;

	ret = regmap_read(chip->regmap, reg, &regval);
	*val = regval & 0xff;

	return ret;
}
EXPORT_SYMBOL_GPL(rtp_reg_read);

int rtp_reg_write(struct rtp_mfd_chip *chip, uint8_t reg, uint8_t *val)
{
	unsigned int regval = 0;

	regval = regval | (*val);

	return regmap_write(chip->regmap, reg, regval);
}
EXPORT_SYMBOL_GPL(rtp_reg_write);

int rtp_bulk_read(struct rtp_mfd_chip *chip, uint8_t reg, int count,
		      uint8_t *val)
{
	return regmap_bulk_read(chip->regmap, reg, val, count);
}
EXPORT_SYMBOL_GPL(rtp_bulk_read);

int rtp_bulk_write(struct rtp_mfd_chip *chip, uint8_t reg, int count,
		       uint8_t *val)
{
	return regmap_bulk_write(chip->regmap, reg, val, count);
}
EXPORT_SYMBOL_GPL(rtp_bulk_write);

int rtp_set_bits(struct rtp_mfd_chip *chip, int reg, uint8_t bit_mask)
{
	return regmap_update_bits(chip->regmap, reg, bit_mask, bit_mask);
}
EXPORT_SYMBOL_GPL(rtp_set_bits);

int rtp_clr_bits(struct rtp_mfd_chip *chip, int reg, uint8_t bit_mask)
{
	return regmap_update_bits(chip->regmap, reg, bit_mask, 0);
}
EXPORT_SYMBOL_GPL(rtp_clr_bits);

int rtp_update_bits(struct rtp_mfd_chip *chip, int reg,
			uint8_t reg_val, uint8_t mask)
{
	return regmap_update_bits(chip->regmap, reg, mask, reg_val);
}
EXPORT_SYMBOL_GPL(rtp_update_bits);

int rtp_register_notifier(struct rtp_mfd_chip *chip,
			      struct notifier_block *nb, uint64_t irqs)
{
	if (nb != NULL) {
		chip->irq_enable |= irqs;
		chip->update_irqs_en(chip);
		return blocking_notifier_chain_register(&chip->notifier_list,
							nb);
	}

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(rtp_register_notifier);

int rtp_unregister_notifier(struct rtp_mfd_chip *chip,
				struct notifier_block *nb, uint64_t irqs)
{
	if (nb != NULL) {
		chip->irq_enable &= ~irqs;
		chip->update_irqs_en(chip);
		return blocking_notifier_chain_unregister(&chip->notifier_list,
							  nb);
	}

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(rtp_unregister_notifier);

static int rtp_init_irqs(struct rtp_mfd_chip *chip)
{
	uint8_t v1[7] = {0x00, 0xff, 0x00, 0x00, 0xc0, 0x0f, 0x37};
	uint8_t v2[7] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
	int err;

	pr_debug("DEBUG: Init rtp irqs\n");
	err =  rtp_bulk_write(chip, RTP_REG_ALDOIN_IRQ_EN, 7, v1);
	if (err) {
		pr_info("INFO: [RTP-MFD] try to init irq_en failed!\n");
		return err;
	}

	err =  rtp_bulk_write(chip, RTP_REG_AVCC_IRQ_STS2, 7, v2);
	if (err) {
		pr_info("INFO: [RTP-MFD] try to init irq_sts failed!\n");
		return err;
	}

	chip->irq_enable = 0x00000000 | (uint64_t) 0x00000000 << 32;
	chip->update_irqs_en(chip);

	return 0;
}

static int rtp_update_irqs_enable(struct rtp_mfd_chip *chip)
{
	int ret = 0;
	uint64_t irqs;
	uint8_t v[7] = {0, 0, 0, 0, 0, 0, 0};

	pr_debug("DEBUG: Update rtp irqs enable\n");
	ret =  rtp_bulk_read(chip, RTP_REG_ALDOIN_IRQ_EN, 7, v);
	if (ret < 0)
		return ret;

	irqs = (((uint64_t) v[6]) << 48) | (((uint64_t) v[5]) << 40) |
	       (((uint64_t) v[4]) << 32) | (((uint64_t) v[3]) << 24) |
	       (((uint64_t) v[2]) << 16) | (((uint64_t) v[1]) << 8) |
	       ((uint64_t) v[0]);

	if (chip->irq_enable != irqs) {
		v[0] = ((chip->irq_enable) & 0xff);
		v[1] = ((chip->irq_enable) >> 8) & 0xff;
		v[2] = ((chip->irq_enable) >> 16) & 0xff;
		v[3] = ((chip->irq_enable) >> 24) & 0xff;
		v[4] = ((chip->irq_enable) >> 32) & 0xff;
		v[5] = ((chip->irq_enable) >> 40) & 0xff;
		v[6] = ((chip->irq_enable) >> 48) & 0xff;
		ret =  rtp_bulk_write(chip, RTP_REG_ALDOIN_IRQ_EN,
					  7, v);
	}

	return ret;
}

static int rtp_read_irqs(struct rtp_mfd_chip *chip, uint64_t *irqs)
{
	uint8_t v[7] = {0, 0, 0, 0, 0, 0, 0};
	int ret;

	pr_debug("DBUG: Read rtp irqs status\n");
	ret =  rtp_bulk_read(chip, RTP_REG_AVCC_IRQ_STS2, 7, v);
	if (ret < 0)
		return ret;

	*irqs = (((uint64_t) v[0]) << 48) | (((uint64_t) v[6]) << 40) |
		(((uint64_t) v[2]) << 32) | (((uint64_t) v[4]) << 24) |
		(((uint64_t) v[3]) << 16) | (((uint64_t) v[5]) << 8) |
		((uint64_t) v[1]);

	return 0;
}

static int rtp_write_irqs(struct rtp_mfd_chip *chip, uint64_t irqs)
{
	uint8_t v[7];
	int ret;

	pr_debug("DBUG: Write rtp irqs status\n");
	v[0] = (irqs >> 48) & 0xff;
	v[1] = (irqs >> 0) & 0xff;
	v[2] = (irqs >> 32) & 0xff;
	v[3] = (irqs >> 16) & 0xff;
	v[4] = (irqs >> 24) & 0xff;
	v[5] = (irqs >> 8) & 0xff;
	v[6] = (irqs >> 40) & 0xff;

	ret = rtp_bulk_write(chip, RTP_REG_AVCC_IRQ_STS2, 7, v);
	if (ret < 0)
		return ret;

	return 0;
}

static ssize_t rtp_reg_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	uint8_t val;
	struct rtp_mfd_chip *chip = i2c_get_clientdata(to_i2c_client(dev));

	rtp_reg_read(chip, rtp_reg_addr, &val);

	return sprintf(buf, "REG[%x]=%x\n", rtp_reg_addr, val);
}

static ssize_t rtp_reg_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	int tmp;
	uint8_t val;
	struct rtp_mfd_chip *chip = i2c_get_clientdata(to_i2c_client(dev));
	int ret;

	ret = kstrtoul(buf, 16, (unsigned long *)&tmp);
	if (ret)
		return -EINVAL;
	if (tmp < 256)
		rtp_reg_addr = tmp;
	else {
		val = tmp & 0x00FF;
		rtp_reg_addr = (tmp >> 8) & 0x00FF;
		rtp_reg_write(chip, rtp_reg_addr, &val);
	}

	return count;
}

/* init continuous r/w */
int register_rw_init(struct rtp_mfd_chip *chip)
{
	int ret;

	ret = rtp_update_bits(chip, RTP_REG_SYS_PAD_CTL, 0x80, 0xC0);
	if (ret < 0) {
		pr_err("ERROR: Unable to config RTP_REG_SYS_PAD_CTL reg\n");
		return ret;
	}
	return 0;
}

/* initial all interrupts */
int interrupts_init(struct rtp_mfd_chip *chip)
{
	int ret;

	if (chip->init_irqs) {
		ret = chip->init_irqs(chip);
		if (ret)
			return ret;
	}
	return 0;
}

/* config UV threshold and UV_PD_EN */
int system_uv_init(struct rtp_mfd_chip *chip)
{
	int ret;
	uint8_t val;

	if (rtp_ic_type == RTP_IC_TYPE_G) {
		val = 0x0F;
		ret = rtp_reg_write(chip, RTP_REG_DC_UVOV_PD_PMU, &val);
		if (ret < 0) {
			pr_err("ERROR: Failed to write RTP_REG_DC_UVOV_PD_PMU reg\n");
			return ret;
		}
	}

	ret = rtp_set_bits(chip, RTP_REG_SYS_UVOV_EN, 0x10);
	if (ret < 0) {
		pr_err("ERROR: Failed to set RTP_REG_SYS_UVOV_EN\n");
		return ret;
	}

	ret = rtp_clr_bits(chip, RTP_REG_SYS_UVOV_SET, 0x30);
	if (ret < 0) {
		pr_err("ERROR: Failed to clr RTP_REG_SYS_UVOV_SET\n");
		return ret;
	}

	ret = rtp_update_bits(chip, RTP_REG_PWR_OFF_CFG, 0x0C, 0x0C);
	if (ret < 0) {
		pr_err("ERROR: Unable to config RTP_REG_PWR_OFF_CFG reg\n");
		return ret;
	}
	return 0;
}

int ldo_init(struct rtp_mfd_chip *chip)
{
	int ret;
	uint8_t val;

	if (rtp_ic_type == RTP_IC_TYPE_D) {
		val = 0x42;
		ret = rtp_reg_write(chip, RTP_REG_LDO1_CTL, &val);
		if (ret < 0) {
			pr_err("ERROR: Unable to write RTP_REG_LDO1_CTL reg\n");
			return ret;
		}
	} else {
		val = 0x42;
		ret = rtp_reg_write(chip, RTP_REG_LDO1_CTL, &val);
		if (ret < 0) {
			pr_err("ERROR: Failed to write RTP_REG_LDO1_CTL reg\n");
			return ret;
		}

		ret = rtp_clr_bits(chip, RTP_REG_POWER_EN, 0x50);
		if (ret < 0) {
			pr_err("ERROR: Failed to clr RTP_REG_POWER_EN reg\n");
			return ret;
		}
	}
	return 0;
}

/* BUCK init */
int buck_init(struct rtp_mfd_chip *chip)
{
	int ret;
	uint8_t val;

	if (rtp_ic_type == RTP_IC_TYPE_D) {
		val = 0xCF;
		ret = rtp_reg_write(chip, RTP_REG_DUMMY_FF, &val);
		if (ret < 0) {
			pr_err("ERROR: Unable to write RTP_REG_DUMMY_FF reg\n");
			return ret;
		}

		val = 0x5A;
		ret = rtp_reg_write(chip, RTP_REG_DUMMY_5A, &val);
		if (ret < 0) {
			pr_err("ERROR: Unable to write RTP_REG_DUMMY_5A reg\n");
			return ret;
		}

		val = 0xB5;
		ret = rtp_reg_write(chip, RTP_REG_DUMMY_A5, &val);
		if (ret < 0) {
			pr_err("ERROR: Unable to write RTP_REG_DUMMY_A5 reg\n");
			return ret;
		}

		/* ic version */
		ret = rtp_reg_read(chip, RTP_REG_CHIP_VERSION, &val);
		if (ret < 0) {
			pr_err("ERROR: Unable to read RTP_REG_CHIP_VERSION reg\n");
			return ret;
		}
		if (val == 0x11) {
			val = 0xB8;
			ret = rtp_reg_write(chip, RTP_REG_DC1_CTL_03, &val);
			if (ret < 0) {
				pr_err("ERROR: Unable to write RTP_REG_DC1_CTL_03 reg\n");
				return ret;
			}

			val = 0xB8;
			ret = rtp_reg_write(chip, RTP_REG_DC2_CTL_03, &val);
			if (ret < 0) {
				pr_err("ERROR: Unable to write RTP_REG_DC2_CTL_03 reg\n");
				return ret;
			}

			val = 0xA8;
			ret = rtp_reg_write(chip, RTP_REG_DC3_CTL_03, &val);
			if (ret < 0) {
				pr_err("ERROR: Unable to write RTP_REG_DC3_CTL_03 reg\n");
				return ret;
			}

			val = 0xBA;
			ret = rtp_reg_write(chip, RTP_REG_DC4_CTL_03, &val);
			if (ret < 0) {
				pr_err("ERROR: Unable to write RTP_REG_DC4_CTL_03 reg\n");
				return ret;
			}

			val = 0x78;
			ret = rtp_reg_write(chip, RTP_REG_DC4_CTL_05, &val);
			if (ret < 0) {
				pr_err("ERROR: Unable to write RTP_REG_DC4_CTL_05 reg\n");
				return ret;
			}

			/* BUCK 1, 2 PM config */
			val = 0xAE;
			ret = rtp_reg_write(chip, RTP_REG_DC1_CTL_01, &val);
			if (ret < 0) {
				pr_err("ERROR: Unable to write RTP_REG_DC1_CTL_01 reg\n");
				return ret;
			}

			val = 0x13;
			ret = rtp_reg_write(chip, RTP_REG_DC1_CTL_02, &val);
			if (ret < 0) {
				pr_err("ERROR: Unable to write RTP_REG_DC1_CTL_02 reg\n");
				return ret;
			}

			val = 0xAE;
			ret = rtp_reg_write(chip, RTP_REG_DC2_CTL_01, &val);
			if (ret < 0) {
				pr_err("ERROR: Unable to write RTP_REG_DC2_CTL_01 reg\n");
				return ret;
			}

			val = 0x13;
			ret = rtp_reg_write(chip, RTP_REG_DC2_CTL_02, &val);
			if (ret < 0) {
				pr_err("ERROR: Unable to write RTP_REG_DC2_CTL_02 reg\n");
				return ret;
			}
		}

		val = 0x8F;
		ret = rtp_reg_write(chip, RTP_REG_DC_UVOV_PD_PMU, &val);
		if (ret < 0) {
			pr_err("ERROR: Unable to write RTP_REG_DC_UVOV_PD_PMU reg\n");
			return ret;
		}

		val = 0x02;
		ret = rtp_reg_write(chip, RTP_REG_PFM_PWM_CTRL, &val);
		if (ret < 0) {
			pr_err("ERROR: Unable to write RTP_REG_PFM_PWM_CTRL reg\n");
			return ret;
		}
	} else {
		val = 0x02;
		ret = rtp_reg_write(chip, RTP_REG_PFM_PWM_CTRL, &val);
		if (ret < 0) {
			pr_err("ERROR: Unable to write RTP_REG_PFM_PWM_CTRL reg\n");
			return ret;
		}

		val = 0xDF;
		ret = rtp_reg_write(chip, RTP_REG_DUMMY_FF, &val);
		if (ret < 0) {
			pr_err("ERROR: Unable to write RTP_REG_DUMMY_FF reg\n");
			return ret;
		}

		val = 0x25;
		ret = rtp_reg_write(chip, RTP_REG_DUMMY_5A, &val);
		if (ret < 0) {
			pr_err("ERROR: Unable to write RTP_REG_DUMMY_5A reg\n");
			return ret;
		}
	}
	return 0;
}

int power_stb_init(struct rtp_mfd_chip *chip)
{
	int ret;
	uint8_t val;

	if (rtp_ic_type == RTP_IC_TYPE_D) {
		ret = rtp_clr_bits(chip, RTP_REG_EXTEN_EN, 0x80);
		if (ret < 0) {
			pr_err("ERROR: Unable to config RTP_REG_EXTEN_EN reg\n");
			return ret;
		}

		val = 0x8A;
		ret = rtp_reg_write(chip, RTP_REG_GPIO0_CTRL, &val);
		if (ret < 0) {
			pr_err("ERROR: Unable to write RTP_REG_GPIO0_CTRL reg\n");
			return ret;
		}
	} else {
		val = 0x94;
		ret = rtp_reg_write(chip, RTP_REG_EXTEN_EN, &val);
		if (ret < 0) {
			pr_err("ERROR: Unable to config RTP_REG_EXTEN_EN reg\n");
			return ret;
		}
	}
	return 0;
}

int system_global_init(struct rtp_mfd_chip *chip)
{
	int ret;
	uint8_t val;

	if (rtp_ic_type == RTP_IC_TYPE_G) {
		val = 0x00;
		ret = rtp_reg_write(chip, RTP_REG_CAL_ALARM, &val);
		if (ret < 0) {
			pr_err("ERROR: Unable to write RTP_REG_CAL_ALARM reg\n");
			return ret;
		}
	}
	ret = rtp_clr_bits(chip, RTP_REG_SYS_CLK_EN, 0x08);
	if (ret < 0) {
		pr_err("ERROR: Failed to disable rtc output\n");
		return ret;
	}

	ret = rtp_update_bits(chip, RTP_REG_PWRHOLD, 0xB0, 0xF0);
	if (ret < 0) {
		pr_err("ERROR: Failed to set RTP_REG_PWRHOLD\n");
		return ret;
	}

	val = 0x58;
	ret = rtp_reg_write(chip, RTP_REG_GLOBAL_CFG1, &val);
	if (ret < 0) {
		pr_err("ERROR: Unable to write RTP_REG_GLOBAL_CFG1 reg\n");
		return ret;
	}

	val = 0x00;
	ret = rtp_reg_write(chip, RTP_REG_GLOBAL_CFG0, &val);
	if (ret < 0) {
		pr_err("ERROR: Unable to write RTP_REG_GLOBAL_CFG0 reg\n");
		return ret;
	}

	ret = rtp_clr_bits(chip, RTP_REG_GLOBAL_CFG2, 0x02);
	if (ret < 0) {
		pr_err("ERROR: Failed to clear RTP_REG_GLOBAL_CFG2\n");
		return ret;
	}

	val = 0x00;
	ret = rtp_reg_write(chip, RTP_REG_DEBUGO_EXT_MUX1, &val);
	if (ret < 0) {
		pr_err("ERROR: Unable to write RTP_REG_DEBUGO_EXT_MUX1 reg\n");
		return ret;
	}
	return 0;
}

/** Called after subdevices are set up */
int rtp_post_init(struct rtp_mfd_chip *chip)
{
	struct regulator *regu;
	int i, ret;
	uint8_t val;

	struct rtp_pmu_info *rtp_pmu_info = chip->pmu_info;

	for (i = 0; i < ARRAY_SIZE(chip->pmu_info); i++) {
		if (strcmp(rtp_pmu_info[i].regulator_name,
			"None") == 0)
			continue;

		if (rtp_pmu_info[i].regulator_id == RTP_ID_LDO3 ||
			rtp_pmu_info[i].regulator_id == RTP_ID_LDO5)
			continue;

		if (strcmp(rtp_pmu_info[i].regulator_name,
			"SWR_OUT_RSV") == 0)
			continue;

		regu = regulator_get(NULL, rtp_pmu_info[i].regulator_name);
		if (IS_ERR(regu)) {
			dev_err(chip->dev, "regulator %s get failed\n",
					rtp_pmu_info[i].regulator_name);
			continue;
		}

		regulator_set_voltage(regu, rtp_pmu_info[i].regulator_vol,
			rtp_pmu_info[i].regulator_vol);

		ret = regulator_enable(regu);
		if (ret)
			pr_warn("WARN: enable regulator %s failed/n",
					rtp_pmu_info[i].regulator_name);

		pr_debug("DBUG: %s, %s = %d, mV end\n", __func__,
			rtp_pmu_info[i].regulator_name,
			regulator_get_voltage(regu));

		regulator_put(regu);
		udelay(100);
	}

	if (rtp_ic_type != RTP_IC_TYPE_D) {
		val = 0x4A;
		ret = rtp_reg_write(chip, RTP_REG_GPIO0_CTRL, &val);
		if (ret < 0) {
			pr_err("ERROR: Unable to write RTP_REG_GPIO0_CTRL reg\n");
			return ret;
		}

		val = 0x00;
		ret = rtp_reg_write(chip, RTP_REG_PWREN_CTRL, &val);
		if (ret < 0) {
			pr_err("ERROR: Unable to write RTP_REG_PWREN_CTRL reg\n");
			return ret;
		}

		val = 0x4A;
		ret = rtp_reg_write(chip, RTP_REG_GPIO1_CTRL, &val);
		if (ret < 0) {
			pr_err("ERROR: Unable to write RTP_REG_GPIO1_CTRL reg\n");
			return ret;
		}
	}

	return 0;
}

int rtpg_pre_init(struct rtp_mfd_chip *chip)
{
	int ret;

	pr_debug("DBUG: %s,line=%d\n", __func__, __LINE__);

	ret = register_rw_init(chip);
	if (ret < 0)
		return ret;

	ret = buck_init(chip);
	if (ret < 0)
		return ret;

	ret = ldo_init(chip);
	if (ret < 0)
		return ret;

	ret = system_uv_init(chip);
	if (ret < 0)
		return ret;

	ret = interrupts_init(chip);
	if (ret < 0)
		return ret;

	ret = system_global_init(chip);
	if (ret < 0)
		return ret;

	ret = power_stb_init(chip);
	if (ret < 0)
		return ret;

	return 0;
}

int rtpd_pre_init(struct rtp_mfd_chip *chip)
{
	int ret;

	pr_debug("DBUG: %s,line=%d\n", __func__, __LINE__);

	ret = register_rw_init(chip);
	if (ret < 0)
		return ret;

	ret = interrupts_init(chip);
	if (ret < 0)
		return ret;

	ret = system_uv_init(chip);
	if (ret < 0)
		return ret;

	ret = ldo_init(chip);
	if (ret < 0)
		return ret;

	ret = buck_init(chip);
	if (ret < 0)
		return ret;

	ret = power_stb_init(chip);
	if (ret < 0)
		return ret;

	ret = system_global_init(chip);
	if (ret < 0)
		return ret;

	return 0;
}

/* init pmu info  */
static void pmu_info_init(struct rtp_mfd_chip *chip, struct rtp_board *board)
{
	int i;
	struct rtp_pmu_info *info = chip->pmu_info;
	struct regulator_init_data **data = board->rtp_pmic_init_data;

	for (i = 0; i < ARRAY_SIZE(chip->pmu_info); i++) {
		if (!data[i] || !data[i]->constraints.name) {
			info[i].regulator_name = "None";
			continue;
		}

		info[i].regulator_name = data[i]->constraints.name;
		info[i].regulator_id = i;
		info[i].regulator_vol = board->regulator_vol[i];
		info[i].regulator_stb_vol =
			data[i]->constraints.state_standby.uV;
		info[i].regulator_stb_en =
			data[i]->constraints.state_standby.enabled ? 1 : 0;

		pr_debug("DBUG:init regulator name: %s, id: %d, vol: %d, stb_vol: %d, stb_en: %d\n",
			info[i].regulator_name,
			info[i].regulator_id,
			info[i].regulator_vol,
			info[i].regulator_stb_vol,
			info[i].regulator_stb_en);
	}
}

/* power tree optimization */
static bool is_power_tree_optimization(struct rtp_mfd_chip *chip)
{
	uint8_t val, bits, shift, mask;
	int ret;

	/* get dc4 trim offset */
	ret = rtp_reg_read(chip, RTP_REG_DC4_TRIM_OFFSET, &val);
	if (ret < 0) {
		pr_err("ERROR: Unable to read RTP_REG_DC4_TRIM_OFFSET reg\n");
		return false;
	}

	bits = RTP_TRIM_DC4_OFFSET_BITS;
	shift = RTP_TRIM_DC4_OFFSET_SHIFT;
	mask = ((1 << bits) - 1) << shift;
	val = (val & mask) >> shift;

	/* 00: not software trim */
	pr_debug("DBUG: RTP_REG_DC4_TRIM_OFFSET value = %d\n", val);
	if (val == 0)
		return false;

	/* get dc2 voltage */
	ret = rtp_reg_read(chip, RTP_REG_DC2VOUT_SET, &val);
	if (ret < 0) {
		pr_err("ERROR: Unable to read RTP_REG_DC2VOUT_SET reg\n");
		return false;
	}

	/* voltage >= 2.000V */
	pr_debug("DBUG: RTP_REG_DC2VOUT_SET value = 0x%x\n", val);
	if (val < 0x34)
		return false;

	return true;
}

static int power_tree_optimization(struct rtp_board *board)
{
	struct regulator_init_data **data = board->rtp_pmic_init_data;
	struct device_node **node = board->rtp_pmic_of_node;

	pr_info("INFO: power tree optimization.\n");
	if (!data[RTP_ID_BUCK2] || !data[RTP_ID_BUCK4]) {
		pr_err("ERROR: %s: failed, BUCK2 or BUCK4 is NULL\n",
			__func__);
		return -EINVAL;
	}

	swap(data[RTP_ID_BUCK2]->constraints.name,
		data[RTP_ID_BUCK4]->constraints.name);
	swap(data[RTP_ID_BUCK2]->num_consumer_supplies,
		data[RTP_ID_BUCK4]->num_consumer_supplies);
	swap(data[RTP_ID_BUCK2]->consumer_supplies,
		data[RTP_ID_BUCK4]->consumer_supplies);

	swap(node[RTP_ID_BUCK2], node[RTP_ID_BUCK4]);
	/*
	 * 2.000 == 3.0V, 2.050 == 3.1V,
	 * 2.125 == 3.2V, 2.2 == 3.3V
	 */
	board->regulator_vol[RTP_ID_BUCK2] = 2200000;

	return 0;
}

static struct rtp_board rtp_data;
/** Called before subdevices are set up */
int rtp_pre_init(struct rtp_mfd_chip *chip)
{
	uint8_t val;
	int ret;

	/* power tree optimization */
	if (is_power_tree_optimization(chip)) {
		ret = power_tree_optimization(&rtp_data);
		if (ret != 0) {
			pr_err(
				"ERROR: power_tree_optimization() failed: %d\n",
				ret);
			return ret;
		}
	}

	/* pmu info init  */
	pmu_info_init(chip, &rtp_data);

	/* get chip version */
	ret = rtp_reg_read(chip, RTP_REG_CHIP_VERSION, &val);
	if (ret < 0) {
		pr_err("ERROR: Unable to read RTP_REG_CHIP_VERSION reg\n");
		return ret;
	}

	switch (val) {
	case 0x11:
		rtp_ic_type = RTP_IC_TYPE_D;
		ret = rtpd_pre_init(chip);
		if (ret < 0)
			return ret;
	case 0x44:
		rtp_ic_type = RTP_IC_TYPE_G;
		ret = rtpg_pre_init(chip);
		if (ret < 0)
			return ret;
	default:
		ret = rtpg_pre_init(chip);
		if (ret < 0)
			return ret;
	}

	return 0;
}

/** Called before ic is power down */
int rtp_late_exit(struct rtp_mfd_chip *chip)
{
	uint8_t val;

	pr_debug("DBUG: %s,line=%d\n", __func__, __LINE__);

	/* disable extern en */
	rtp_clr_bits(chip, RTP_REG_EXTEN_EN, 0x04);

	/* go to active state */
	rtp_reg_read(chip, RTP_REG_FSM_DEBUG, &val);
	if ((val & 0x07) != 0x06)
		rtp_set_bits(chip, RTP_REG_PMU_STATE_CTL, 0x02);

	pr_debug("DBUG: %s,line=%d END\n", __func__, __LINE__);

	return 0;
}

static uint8_t rtp_regs_addr;
static int rtp_regs_len;
static int rtp_regs_rw;
static uint8_t rtp_regs_datas[256];
static ssize_t rtp_regs_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int count = 0;
	int i;
	struct rtp_mfd_chip *chip = i2c_get_clientdata(to_i2c_client(dev));

	if (rtp_regs_rw == 0) {
		rtp_bulk_read(chip, rtp_regs_addr, rtp_regs_len,
				  rtp_regs_datas);
		for (i = 0; i < rtp_regs_len; i++) {
			count += sprintf(buf + count, "REG[%x]=%x\n",
					 rtp_regs_addr + i,
					 rtp_regs_datas[i]);
		}
	} else if (rtp_regs_rw == 1) {
		rtp_bulk_write(chip, rtp_regs_addr, rtp_regs_len,
				   rtp_regs_datas);
		sprintf(buf, "bulk write ok\n");
	}

	return count;
}

static ssize_t rtp_regs_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	int tmp, ret;
	char buftemp[3];

	if (count < 6)
		return count;

	/* rw flag */
	buftemp[0] = buf[0];
	buftemp[1] = '\0';
	ret = kstrtoul(buftemp, 16, (unsigned long *)&rtp_regs_rw);
	if (ret)
		return -EINVAL;
	pr_info("<0> rtp_regs_store, rtp_regs_rw = %d\n", rtp_regs_rw);
	/* addr */
	buftemp[0] = buf[1];
	buftemp[1] = buf[2];
	buftemp[2] = '\0';
	ret = kstrtoul(buftemp, 16, (unsigned long *)&rtp_regs_addr);
	if (ret)
		return -EINVAL;
	pr_info("<0> rtp_regs_store, rtp_regs_addr = 0x%x\n", rtp_regs_addr);
	/* addr */
	buftemp[0] = buf[3];
	buftemp[1] = buf[4];
	buftemp[2] = '\0';
	ret = kstrtoul(buftemp, 16, (unsigned long *)&rtp_regs_len);
	if (ret)
		return -EINVAL;
	pr_info("<0> rtp_regs_store, rtp_regs_len = %d\n", rtp_regs_len);
	if (rtp_regs_rw == 1) {
		if (count != 5 + rtp_regs_len * 2 + 1)
			pr_info(
				"<0> rtp_regs_store error, count = %d\n",
				count);

		for (tmp = 0; tmp < rtp_regs_len; tmp++) {
			buftemp[0] = buf[tmp * 2 + 5];
			buftemp[1] = buf[tmp * 2 + 6];
			buftemp[2] = '\0';
			ret = kstrtoul(buftemp, 16,
				(unsigned long *)&rtp_regs_datas[tmp]);
			if (ret)
				return -EINVAL;
			pr_info("<0> rtp_regs_store, val[%x] = 0x%x\n",
				tmp + rtp_regs_addr, rtp_regs_datas[tmp]);
		}
	}

	return count;
}

static struct device_attribute rtp_mfd_attrs[] = {
	RTP_MFD_ATTR(rtp_reg),
	RTP_MFD_ATTR(rtp_regs),
};

static int rtp_mfd_create_attrs(struct rtp_mfd_chip *chip)
{
	int j, ret;

	for (j = 0; j < ARRAY_SIZE(rtp_mfd_attrs); j++) {
		ret = device_create_file(chip->dev, &rtp_mfd_attrs[j]);
		if (ret)
			goto sysfs_failed;
	}
	goto succeed;

sysfs_failed:
	while (j--)
		device_remove_file(chip->dev, &rtp_mfd_attrs[j]);
succeed:
	return ret;
}

static const struct regmap_config rtp_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = RTP_REG_MAX_REGISTER,
};

#ifdef CONFIG_PM
static int rtp_suspend(struct device *dev)
{
	return 0;
}

static int rtp_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops rtp_pm_ops = {
	.suspend = rtp_suspend,
	.resume =  rtp_resume,
};
#endif

#ifdef CONFIG_OF
static struct of_regulator_match rtp_reg_matches[] = {
	{ .name = "RTP_LDO1"},
	{ .name = "RTP_LDO2"},
	{ .name = "RTP_LDO3"},
	{ .name = "RTP_LDO4"},
	{ .name = "RTP_LDO5"},
	{ .name = "RTP_BUCK1"},
	{ .name = "RTP_BUCK2"},
	{ .name = "RTP_BUCK3"},
	{ .name = "RTP_BUCK4"},
};

static int rtp_parse_dt(struct i2c_client *i2c, struct rtp_board *board)
{
	struct device_node *pmic, *regulators, *child;
	int i, count;
	u32 pval;

	pmic = of_node_get(i2c->dev.of_node);
	if (!pmic) {
		dev_err(NULL, "could not find pmic sub-node");
		return -EINVAL;
	}

	regulators = of_find_node_by_name(pmic, "regulators");
	if (!regulators)
		return -EINVAL;

	count = of_regulator_match(&i2c->dev, regulators,
				rtp_reg_matches, RTP_ID_REGULATORS_NUM);
	of_node_put(regulators);
	if ((count < 0) || (count > RTP_ID_REGULATORS_NUM))
		return -EINVAL;

	for (i = 0; i < count; i++) {
		if (!rtp_reg_matches[i].init_data ||
			!rtp_reg_matches[i].of_node)
			continue;

		board->rtp_pmic_init_data[i] = rtp_reg_matches[i].init_data;
		board->rtp_pmic_of_node[i] = rtp_reg_matches[i].of_node;
	}

	/* voltage */
	i = 0;
	for_each_child_of_node(regulators, child) {
		if (!of_property_read_u32(child, "regulator-microvolt", &pval))
			board->regulator_vol[i] = pval;
		i++;
	}

	/* pm off */
	board->pm_off = of_property_read_bool(pmic,
				"rtp,system-power-controller");

	return 0;
}

static const struct of_device_id rtp_of_match[] = {
	{ .compatible = "realtek,rtp5903"},
	{},
};
MODULE_DEVICE_TABLE(of, rtp_of_match);
#endif

static int rtp_mfd_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
	struct rtp_mfd_chip *chip;
	struct rtp_board *pmic_plat_data;
	int ret = 0;

	dev_info(&i2c->dev, "rtp_mfd_probe\n");
#ifdef CONFIG_OF
	pmic_plat_data = &rtp_data;
	ret = rtp_parse_dt(i2c, pmic_plat_data);
	if (ret < 0)
		return ret;

	pmic_plat_data->irq = RTP_IRQ;
	pmic_plat_data->gpio_base = RTP_GPIO_BASE;
#else
	pmic_plat_data = dev_get_platdata(&i2c->dev);
#endif
	if (!pmic_plat_data)
		return -EINVAL;

	chip = devm_kzalloc(&i2c->dev, sizeof(struct rtp_mfd_chip), GFP_KERNEL);
	if (chip == NULL)
		return -ENOMEM;

	i2c_set_clientdata(i2c, chip);
	chip->dev = &i2c->dev;
	chip->i2c_client = i2c;
	if (id)
		chip->id = id->driver_data;
	chip->rtp_data = pmic_plat_data;
	chip->init_irqs = rtp_init_irqs;
	chip->update_irqs_en = rtp_update_irqs_enable;
	chip->read_irqs = rtp_read_irqs;
	chip->write_irqs = rtp_write_irqs;
	chip->regmap = devm_regmap_init_i2c(i2c, &rtp_regmap_config);
	if (IS_ERR(chip->regmap)) {
		ret = PTR_ERR(chip->regmap);
		dev_err(chip->dev, "regmap init failed: %d\n", ret);
		return ret;
	}

	/* verify the ic */
/*	ret = rtp_reg_read(chip, 0x22);
 *	if ((ret < 0) || (ret == 0xff)){
 *		pr_err("ERROR: The device is not rtp\n");
 *		return ret;
 *	}
 */
	g_chip = chip;

	ret = rtp_pre_init(chip);
	if (ret != 0) {
		dev_err(chip->dev, "pre_init() failed: %d\n", ret);
		return ret;
	}

	ret = rtp_irq_init(chip, pmic_plat_data->irq);
	if (ret < 0)
		return ret;

	ret = mfd_add_devices(chip->dev, -1,
			      rtps, ARRAY_SIZE(rtps),
			      NULL, 0, NULL);
	if (ret < 0)
		goto mfd_err;

	ret = rtp_post_init(chip);
	if (ret != 0) {
		dev_err(chip->dev, "post_init() failed: %d\n", ret);
		goto mfd_err;
	}

	/* power off */
	if (pmic_plat_data->pm_off && !pm_power_off)
		pm_power_off = rtp_power_off;

	ret = rtp_mfd_create_attrs(chip);
	if (ret)
		goto mfd_err;

	return ret;

mfd_err:
	mfd_remove_devices(chip->dev);
	rtp_irq_exit(chip);
	return ret;
}

void rtp_power_off(void)
{
	int ret;
	struct rtp_mfd_chip *chip = g_chip;

	pr_debug("DBUG: %s\n", __func__);

	ret = rtp_late_exit(chip);
	if (ret != 0)
		dev_err(chip->dev, "late_exit() failed: %d\n", ret);

	mdelay(20);
	rtp_set_bits(chip, RTP_REG_PMU_STATE_CTL, 0x80);
	mdelay(20);
	pr_warn("WARN: warning!!! rtp can't power-off, maybe some error happened!\n");
}
EXPORT_SYMBOL_GPL(rtp_power_off);

static int rtp_mfd_remove(struct i2c_client *client)
{
	struct rtp_mfd_chip *chip = i2c_get_clientdata(client);
	int j;

	pr_info("<0> rtp_mfd_remove\n");
	mfd_remove_devices(chip->dev);
	rtp_irq_exit(chip);
	i2c_set_clientdata(client, NULL);
	for (j = 0; j < ARRAY_SIZE(rtp_mfd_attrs); j++)
		device_remove_file(chip->dev, &rtp_mfd_attrs[j]);
	g_chip = NULL;

	return 0;
}

static const struct i2c_device_id rtp_mfd_id_table[] = {
	{ "rtp_mfd", RTP_ID },
	{},
};
MODULE_DEVICE_TABLE(i2c, rtp_mfd_id_table);

static struct i2c_driver rtp_mfd_driver = {
	.driver = {
		.name = "rtp_mfd",
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &rtp_pm_ops,
#endif
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(rtp_of_match),
#endif
	},
	.probe = rtp_mfd_probe,
	.remove = rtp_mfd_remove,
	.id_table = rtp_mfd_id_table,
};

static int __init rtp_mfd_init(void)
{
	return i2c_add_driver(&rtp_mfd_driver);
}
subsys_initcall_sync(rtp_mfd_init);

static void __exit rtp_mfd_exit(void)
{
	i2c_del_driver(&rtp_mfd_driver);
}
module_exit(rtp_mfd_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Wind_Han <wind_han@realsil.com.cn>");
MODULE_DESCRIPTION("Realtek Power Manager Driver");
