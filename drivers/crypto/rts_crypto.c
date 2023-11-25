#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/completion.h>
#include <linux/crypto.h>
#include <crypto/algapi.h>
#include <crypto/aes.h>
#include <crypto/des.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/reset.h>
#include <linux/cdev.h>

#include "rts_crypto.h"

#if 0
#define DBG(args...)	pr_emerg("%s: %s", __func__, args)
#else
#define DBG(args...)
#endif

#define FLAGS_ENCRYPT		BIT(0)
#define FLAGS_DECRYPT		BIT(1)
#define FLAGS_AES		BIT(2)
#define FLAGS_DES		BIT(3)
#define FLAGS_DES3		BIT(4)
#define FLAGS_ECB		BIT(5)
#define FLAGS_CBC		BIT(6)
#define FLAGS_CBCCS1		BIT(7)
#define FLAGS_CBCCS2		BIT(8)
#define FLAGS_CBCCS3		BIT(9)
#define FLAGS_CTR		BIT(10)

#define RLX_DMA_TABLE_SIZE	256
#define RLX_KEY_LENGTH		32

#define RTS_ASOC_HW_ID(type)	((type) & 0xff)

#define RTS_CIPHER_IOC_MAGIC	0x81
#define RTS_CIPHER_IOC_SET_NORMALKEY	_IO(RTS_CIPHER_IOC_MAGIC, 0xA0)
#define RTS_CIPHER_IOC_SET_EFUSEKEY	_IO(RTS_CIPHER_IOC_MAGIC, 0xA1)

#define set_mode_cap(cap, mode)	((cap) |= (1u << (mode)))
#define get_mode_cap(cap, mode)	((cap) & (1u << (mode)))

enum {
	TYPE_RLE0745 = 1,
	TYPE_RTS3901 = 2,
	TYPE_RTS3903 = 3,

	TYPE_FPGA = (1 << 16),
};

enum {
	ALG_AES = 0,
	ALG_DES = 1,
	ALG_DES3 = 2,
	ALG_COUNT,
};

enum {
	MODE_ECB = 0,
	MODE_CBC = 1,
	MODE_CBCCS1 = 2,
	MODE_CBCCS2 = 3,
	MODE_CBCCS3 = 4,
	MODE_CTR = 5,
	MODE_COUNT,
};

struct rts_crypto_data {
	struct platform_device *pdev;
	struct cdev cdev;
	void __iomem *addr;
	unsigned long size;
	u32 base;
	int irq;
	u32 *table_in;
	dma_addr_t dma_table_in;
	u32 *table_out;
	dma_addr_t dma_table_out;
	struct reset_control *rst;
	struct reset_control *sd;
	struct mutex dma_mutex;
	struct completion crypto_complete;
	struct clk *cipher_clk;
	u32 devtype;
	unsigned int efusekey;
	unsigned int mode_cap[ALG_COUNT];
};

struct rts_crypto_ctx {
	u8 key[RLX_KEY_LENGTH];
	unsigned int keylen;
	u8 *iv;
	int ivflag;
	unsigned int ivlen;
	unsigned int mask;
};

static struct rts_crypto_data *rts_cdata;
static dev_t devno = MKDEV(124, 0);

extern void rts_efuse_load_aes(void);

static unsigned int rts_crypto_read(struct rts_crypto_data *cdata,
				    unsigned int reg)
{
	DBG("rts crypto read\n");

	return readl(cdata->addr + reg);
}

static int rts_crypto_write(struct rts_crypto_data *cdata, unsigned int reg,
			    unsigned int value)
{
	DBG("rts crypto write\n");

	writel(value, cdata->addr + reg);

	return 0;
}

static int rts_sg_nents(struct scatterlist *sg_list, int nbytes)
{
	struct scatterlist *sg = sg_list;
	int sg_nents = 0;

	while (nbytes > 0) {
		++sg_nents;
		nbytes -= sg->length;
		sg = sg_next(sg);
	}

	return sg_nents;
}

static void rts_sg_copy(struct scatterlist *dst,
		struct scatterlist *src, int nbytes) {
	int in_nents, out_nents;
	void *buf = NULL;

	in_nents = rts_sg_nents(src, nbytes);
	out_nents = rts_sg_nents(dst, nbytes);
	buf = kmalloc(nbytes, GFP_KERNEL);
	sg_copy_to_buffer(src, in_nents, buf, nbytes);
	sg_copy_from_buffer(dst, out_nents, buf, nbytes);
	kfree(buf);
	buf = NULL;
}

static int rts_setkey(struct crypto_tfm *tfm, const u8 *key, unsigned int len)
{
	struct rts_crypto_ctx *ctx = crypto_tfm_ctx(tfm);

	DBG("rts setkey\n");

	ctx->keylen = len;
	memcpy(ctx->key, key, len);

	return 0;
}

static int rts_crypto_do(struct rts_crypto_data *cdata,
			 struct rts_crypto_ctx *ctx)
{
	int ret = 0;
	u32 val;
	u32 *reg_val;

	DBG("rts crypto do\n");

	if (ctx->ivflag == 1) {
		/* initial iv */
		reg_val = (u32 *)ctx->iv;
		rts_crypto_write(rts_cdata, RLX_REG_IV_IN_DATA0,
				 cpu_to_be32(reg_val[0]));
		rts_crypto_write(rts_cdata, RLX_REG_IV_IN_DATA1,
				 cpu_to_be32(reg_val[1]));
		if (ctx->ivlen > 8) {
			rts_crypto_write(rts_cdata, RLX_REG_IV_IN_DATA2,
					 cpu_to_be32(reg_val[2]));
			rts_crypto_write(rts_cdata, RLX_REG_IV_IN_DATA3,
					 cpu_to_be32(reg_val[3]));
		}
	}

	/* start crypto */
	val = rts_crypto_read(cdata, RLX_REG_CIPHER_CTL);
	rts_crypto_write(cdata, RLX_REG_CIPHER_CTL, val | 0x1);

	ret = wait_for_completion_timeout(&cdata->crypto_complete,
					  msecs_to_jiffies(10000));
	if (ret == 0) {
		pr_err("timed out\n");
		ret = -ETIMEDOUT;
		goto do_out;
	}

	/* crypto status */
	val = rts_crypto_read(cdata, RLX_REG_CIPHER_STS);
	if (!(val & 0x1)) {
		pr_err("crypto err\n");
		ret = -EIO;
		goto do_out;
	} else {
		DBG("crypto ok\n");
		if (ctx->ivflag == 1) {
			reg_val = (u32 *)ctx->iv;
			reg_val[0] = be32_to_cpu(rts_crypto_read(cdata,
							RLX_REG_IV_OUT_DATA0));
			reg_val[1] = be32_to_cpu(rts_crypto_read(cdata,
							RLX_REG_IV_OUT_DATA1));
			if (ctx->ivlen > 8) {
				reg_val[2] = be32_to_cpu(rts_crypto_read(cdata,
							RLX_REG_IV_OUT_DATA2));
				reg_val[3] = be32_to_cpu(rts_crypto_read(cdata,
							RLX_REG_IV_OUT_DATA3));
			}
		}
	}
	ret = 0;
do_out:
	/* stop crypto */
	val = rts_crypto_read(cdata, RLX_REG_CIPHER_CTL);
	rts_crypto_write(cdata, RLX_REG_CIPHER_CTL, val | 0x2);

	return ret;
}

static int rts_crypto(struct blkcipher_desc *desc,
		      struct scatterlist *dst,
		      struct scatterlist *src,
		      unsigned int nbytes,
		      unsigned int mask)
{
	struct rts_crypto_ctx *ctx = crypto_blkcipher_ctx(desc->tfm);
	u32 *reg_val;
	u32 val;
	dma_addr_t addr_in, addr_out;
	int ret = 0, i, in_nents, out_nents;
	unsigned int align_mask;
	unsigned int *table_in_ptr = (unsigned int *)rts_cdata->table_in;
	unsigned int *table_out_ptr = (unsigned int *)rts_cdata->table_out;
	struct scatterlist *in_sg, *out_sg;

	DBG("rts crypto\n");

	mutex_lock(&rts_cdata->dma_mutex);

	if (((mask & FLAGS_AES) && (nbytes < AES_BLOCK_SIZE)) ||
		((mask & (FLAGS_DES | FLAGS_DES3)) &&
		(nbytes < DES_BLOCK_SIZE))) {
		if (mask & (FLAGS_CBCCS1 | FLAGS_CBCCS2 | FLAGS_CBCCS3)) {
			rts_sg_copy(dst, src, nbytes);
			mutex_unlock(&rts_cdata->dma_mutex);
			return 0;
		}
	}

	clk_prepare_enable(rts_cdata->cipher_clk);
	rts_crypto_write(rts_cdata, RLX_REG_CIPHER_INT_FLAG, 0x5);
	rts_crypto_write(rts_cdata, RLX_REG_CIPHER_INT_EN, 0x5);

	/* initialize key, iv, data, mode */
	/* mode */
	ctx->mask = mask;
	val = rts_crypto_read(rts_cdata, RLX_REG_CIPHER_CTL);

	if (RTS_ASOC_HW_ID(rts_cdata->devtype) == TYPE_RLE0745)
		val = val & 0xFFFFFC07;
	else
		val = val & 0xFFFFF007;

	align_mask = 0x7;
	if (mask & FLAGS_AES) {
		val = val | ((u32)0x0 << RLX_ALGORITHM_SEL);
		if (RTS_ASOC_HW_ID(rts_cdata->devtype) != TYPE_RLE0745) {
			if (ctx->keylen == 16)
				val = val | ((u32)0x0 << RLX_AES_MODE_SEL);
			else if (ctx->keylen == 24)
				val = val | ((u32)0x1 << RLX_AES_MODE_SEL);
			else if (ctx->keylen == 32)
				val = val | ((u32)0x2 << RLX_AES_MODE_SEL);
		}
		align_mask = 0xf;
	} else if (mask & FLAGS_DES) {
		val = val | ((u32)0x1 << RLX_ALGORITHM_SEL);
	} else if (mask & FLAGS_DES3) {
		val = val | ((u32)0x2 << RLX_ALGORITHM_SEL);
	}

	if (mask & (FLAGS_ECB | FLAGS_CBC)) {
		if (nbytes & align_mask) {
			ret = -EINVAL;
			goto out;
		}
	}

	if (mask & FLAGS_ECB)
		val = val | ((u32)0x0 << RLX_OPERATION_MODE);
	else if (mask & FLAGS_CBC)
		val = val | ((u32)0x1 << RLX_OPERATION_MODE);
	else if (mask & FLAGS_CBCCS1)
		val = val | ((u32)0x2 << RLX_OPERATION_MODE);
	else if (mask & FLAGS_CBCCS2)
		val = val | ((u32)0x3 << RLX_OPERATION_MODE);
	else if (mask & FLAGS_CBCCS3)
		val = val | ((u32)0x4 << RLX_OPERATION_MODE);
	else if (mask & FLAGS_CTR)
		val = val | ((u32)0x5 << RLX_OPERATION_MODE);

	if (mask & FLAGS_ECB)
		ctx->ivflag = 0;
	else
		ctx->ivflag = 1;

	if (mask & FLAGS_AES)
		ctx->ivlen = 16;
	else
		ctx->ivlen = 8;

	val = val | ((u32)0x0 << RLX_PADDING_SEL);

	if (mask & FLAGS_ENCRYPT)
		val = val | ((u32)0x0 << RLX_ENCTYPT_DECTYPT_SEL);
	else if (mask & FLAGS_DECRYPT)
		val = val | ((u32)0x1 << RLX_ENCTYPT_DECTYPT_SEL);
	rts_crypto_write(rts_cdata, RLX_REG_CIPHER_CTL, val);

	/* key */
	if (!rts_cdata->efusekey) {
		dev_dbg(&rts_cdata->pdev->dev, "rts normal key\n");
		rts_crypto_write(rts_cdata, RLX_REG_CIPHER_KEY, 0);
		reg_val = (u32 *)ctx->key;
		rts_crypto_write(rts_cdata, RLX_REG_KEY_DATA0,
					cpu_to_be32(reg_val[0]));
		rts_crypto_write(rts_cdata, RLX_REG_KEY_DATA1,
					cpu_to_be32(reg_val[1]));
		if (ctx->keylen > 8) {
			rts_crypto_write(rts_cdata, RLX_REG_KEY_DATA2,
				 cpu_to_be32(reg_val[2]));
			rts_crypto_write(rts_cdata, RLX_REG_KEY_DATA3,
				 cpu_to_be32(reg_val[3]));
		}
		if (ctx->keylen > 16) {
			rts_crypto_write(rts_cdata, RLX_REG_KEY_DATA4,
				 cpu_to_be32(reg_val[4]));
			rts_crypto_write(rts_cdata, RLX_REG_KEY_DATA5,
				 cpu_to_be32(reg_val[5]));
		}

		if (RTS_ASOC_HW_ID(rts_cdata->devtype) != TYPE_RLE0745) {
			if (ctx->keylen > 24) {
				rts_crypto_write(rts_cdata, RLX_REG_KEY_DATA6,
					 cpu_to_be32(reg_val[6]));
				rts_crypto_write(rts_cdata, RLX_REG_KEY_DATA7,
					 cpu_to_be32(reg_val[7]));
			}
		}
	} else { /* KEYMODE_EFUSE */
		dev_dbg(&rts_cdata->pdev->dev, "rts_efuse_load_aes\n");
		rts_crypto_write(rts_cdata, RLX_REG_CIPHER_KEY, 1);
		rts_efuse_load_aes();
	}

	/* iv */
	if (ctx->ivflag == 1)
		ctx->iv = (u8 *)desc->info;

	/* data in & out */
	rts_crypto_write(rts_cdata, RLX_REG_IN_TABLE_ADDR,
			 rts_cdata->dma_table_in);
	rts_crypto_write(rts_cdata, RLX_REG_OUT_TABLE_ADDR,
			 rts_cdata->dma_table_out);

	in_nents = rts_sg_nents(src, nbytes);
	ret = dma_map_sg(&rts_cdata->pdev->dev, src,
			in_nents, DMA_TO_DEVICE);
	if (!ret) {
		pr_err("dma map in sg err\n");
		goto out;
	}

	for_each_sg(src, in_sg, ret, i) {
		addr_in = sg_dma_address(in_sg);
		*(table_in_ptr++) = (u32)addr_in;
		*(table_in_ptr++) = sg_dma_len(in_sg);
	}
	rts_crypto_write(rts_cdata, RLX_REG_IN_BUF_NUM, ret);

	out_nents = rts_sg_nents(dst, nbytes);
	ret = dma_map_sg(&rts_cdata->pdev->dev, dst,
			out_nents, DMA_FROM_DEVICE);
	if (!ret) {
		pr_err("dma map out sg err\n");
		dma_unmap_sg(&rts_cdata->pdev->dev, src,
				in_nents, DMA_TO_DEVICE);
		goto out;
	}

	for_each_sg(dst, out_sg, ret, i) {
		addr_out = sg_dma_address(out_sg);
		*(table_out_ptr++) = (u32)addr_out;
		*(table_out_ptr++) = sg_dma_len(out_sg);
	}
	rts_crypto_write(rts_cdata, RLX_REG_OUT_BUF_NUM, ret);

	rts_crypto_write(rts_cdata, RLX_REG_DATA_IN_LENGTH, nbytes);

	ret = rts_crypto_do(rts_cdata, ctx);

	/* unmap in & out */
	dma_unmap_sg(&rts_cdata->pdev->dev, src, in_nents, DMA_TO_DEVICE);
	dma_unmap_sg(&rts_cdata->pdev->dev, dst, out_nents, DMA_FROM_DEVICE);

out:
	rts_crypto_write(rts_cdata, RLX_REG_CIPHER_INT_EN, 0x0);
	clk_disable(rts_cdata->cipher_clk);
	mutex_unlock(&rts_cdata->dma_mutex);

	return ret;
}

/* aes */
static int rts_aes_ecb_encrypt(struct blkcipher_desc *desc,
			       struct scatterlist *dst,
			       struct scatterlist *src,
			       unsigned int nbytes)
{
	return rts_crypto(desc, dst, src, nbytes,
			  FLAGS_ENCRYPT | FLAGS_AES | FLAGS_ECB);
}

static int rts_aes_ecb_decrypt(struct blkcipher_desc *desc,
			       struct scatterlist *dst,
			       struct scatterlist *src,
			       unsigned int nbytes)
{
	return rts_crypto(desc, dst, src, nbytes,
			  FLAGS_DECRYPT | FLAGS_AES | FLAGS_ECB);
}

static int rts_aes_cbc_encrypt(struct blkcipher_desc *desc,
			       struct scatterlist *dst,
			       struct scatterlist *src,
			       unsigned int nbytes)
{
	return rts_crypto(desc, dst, src, nbytes,
			  FLAGS_ENCRYPT | FLAGS_AES | FLAGS_CBC);
}

static int rts_aes_cbc_decrypt(struct blkcipher_desc *desc,
			       struct scatterlist *dst,
			       struct scatterlist *src,
			       unsigned int nbytes)
{
	return rts_crypto(desc, dst, src, nbytes,
			  FLAGS_DECRYPT | FLAGS_AES | FLAGS_CBC);
}

static int rts_aes_cbccs1_encrypt(struct blkcipher_desc *desc,
			       struct scatterlist *dst,
			       struct scatterlist *src,
			       unsigned int nbytes)
{
	return rts_crypto(desc, dst, src, nbytes,
			  FLAGS_ENCRYPT | FLAGS_AES | FLAGS_CBCCS1);
}

static int rts_aes_cbccs1_decrypt(struct blkcipher_desc *desc,
			       struct scatterlist *dst,
			       struct scatterlist *src,
			       unsigned int nbytes)
{
	return rts_crypto(desc, dst, src, nbytes,
			  FLAGS_DECRYPT | FLAGS_AES | FLAGS_CBCCS1);
}

static int rts_aes_cbccs2_encrypt(struct blkcipher_desc *desc,
			       struct scatterlist *dst,
			       struct scatterlist *src,
			       unsigned int nbytes)
{
	return rts_crypto(desc, dst, src, nbytes,
			  FLAGS_ENCRYPT | FLAGS_AES | FLAGS_CBCCS2);
}

static int rts_aes_cbccs2_decrypt(struct blkcipher_desc *desc,
			       struct scatterlist *dst,
			       struct scatterlist *src,
			       unsigned int nbytes)
{
	return rts_crypto(desc, dst, src, nbytes,
			  FLAGS_DECRYPT | FLAGS_AES | FLAGS_CBCCS2);
}

static int rts_aes_cbccs3_encrypt(struct blkcipher_desc *desc,
			       struct scatterlist *dst,
			       struct scatterlist *src,
			       unsigned int nbytes)
{
	return rts_crypto(desc, dst, src, nbytes,
			  FLAGS_ENCRYPT | FLAGS_AES | FLAGS_CBCCS3);
}

static int rts_aes_cbccs3_decrypt(struct blkcipher_desc *desc,
			       struct scatterlist *dst,
			       struct scatterlist *src,
			       unsigned int nbytes)
{
	return rts_crypto(desc, dst, src, nbytes,
			  FLAGS_DECRYPT | FLAGS_AES | FLAGS_CBCCS3);
}

static int rts_aes_ctr_encrypt(struct blkcipher_desc *desc,
			       struct scatterlist *dst,
			       struct scatterlist *src,
			       unsigned int nbytes)
{
	return rts_crypto(desc, dst, src, nbytes,
			  FLAGS_ENCRYPT | FLAGS_AES | FLAGS_CTR);
}

static int rts_aes_ctr_decrypt(struct blkcipher_desc *desc,
			       struct scatterlist *dst,
			       struct scatterlist *src,
			       unsigned int nbytes)
{
	return rts_crypto(desc, dst, src, nbytes,
			  FLAGS_DECRYPT | FLAGS_AES | FLAGS_CTR);
}

/* des */
static int rts_des_ecb_encrypt(struct blkcipher_desc *desc,
			       struct scatterlist *dst,
			       struct scatterlist *src,
			       unsigned int nbytes)
{
	return rts_crypto(desc, dst, src, nbytes,
			  FLAGS_ENCRYPT | FLAGS_DES | FLAGS_ECB);
}

static int rts_des_ecb_decrypt(struct blkcipher_desc *desc,
			       struct scatterlist *dst,
			       struct scatterlist *src,
			       unsigned int nbytes)
{
	return rts_crypto(desc, dst, src, nbytes,
			  FLAGS_DECRYPT | FLAGS_DES | FLAGS_ECB);
}

static int rts_des_cbc_encrypt(struct blkcipher_desc *desc,
			       struct scatterlist *dst,
			       struct scatterlist *src,
			       unsigned int nbytes)
{
	return rts_crypto(desc, dst, src, nbytes,
			  FLAGS_ENCRYPT | FLAGS_DES | FLAGS_CBC);
}

static int rts_des_cbc_decrypt(struct blkcipher_desc *desc,
			       struct scatterlist *dst,
			       struct scatterlist *src,
			       unsigned int nbytes)
{
	return rts_crypto(desc, dst, src, nbytes,
			  FLAGS_DECRYPT | FLAGS_DES | FLAGS_CBC);
}

static int rts_des_cbccs1_encrypt(struct blkcipher_desc *desc,
			       struct scatterlist *dst,
			       struct scatterlist *src,
			       unsigned int nbytes)
{
	return rts_crypto(desc, dst, src, nbytes,
			  FLAGS_ENCRYPT | FLAGS_DES | FLAGS_CBCCS1);
}

static int rts_des_cbccs1_decrypt(struct blkcipher_desc *desc,
			       struct scatterlist *dst,
			       struct scatterlist *src,
			       unsigned int nbytes)
{
	return rts_crypto(desc, dst, src, nbytes,
			  FLAGS_DECRYPT | FLAGS_DES | FLAGS_CBCCS1);
}

static int rts_des_cbccs2_encrypt(struct blkcipher_desc *desc,
			       struct scatterlist *dst,
			       struct scatterlist *src,
			       unsigned int nbytes)
{
	return rts_crypto(desc, dst, src, nbytes,
			  FLAGS_ENCRYPT | FLAGS_DES | FLAGS_CBCCS2);
}

static int rts_des_cbccs2_decrypt(struct blkcipher_desc *desc,
			       struct scatterlist *dst,
			       struct scatterlist *src,
			       unsigned int nbytes)
{
	return rts_crypto(desc, dst, src, nbytes,
			  FLAGS_DECRYPT | FLAGS_DES | FLAGS_CBCCS2);
}

static int rts_des_cbccs3_encrypt(struct blkcipher_desc *desc,
			       struct scatterlist *dst,
			       struct scatterlist *src,
			       unsigned int nbytes)
{
	return rts_crypto(desc, dst, src, nbytes,
			  FLAGS_ENCRYPT | FLAGS_DES | FLAGS_CBCCS3);
}

static int rts_des_cbccs3_decrypt(struct blkcipher_desc *desc,
			       struct scatterlist *dst,
			       struct scatterlist *src,
			       unsigned int nbytes)
{
	return rts_crypto(desc, dst, src, nbytes,
			  FLAGS_DECRYPT | FLAGS_DES | FLAGS_CBCCS3);
}

static int rts_des_ctr_encrypt(struct blkcipher_desc *desc,
			       struct scatterlist *dst,
			       struct scatterlist *src,
			       unsigned int nbytes)
{
	return rts_crypto(desc, dst, src, nbytes,
			  FLAGS_ENCRYPT | FLAGS_DES | FLAGS_CTR);
}

static int rts_des_ctr_decrypt(struct blkcipher_desc *desc,
			       struct scatterlist *dst,
			       struct scatterlist *src,
			       unsigned int nbytes)
{
	return rts_crypto(desc, dst, src, nbytes,
			  FLAGS_DECRYPT | FLAGS_DES | FLAGS_CTR);
}

/* 3des */
static int rts_des3_ecb_encrypt(struct blkcipher_desc *desc,
			       struct scatterlist *dst,
			       struct scatterlist *src,
			       unsigned int nbytes)
{
	return rts_crypto(desc, dst, src, nbytes,
			  FLAGS_ENCRYPT | FLAGS_DES3 | FLAGS_ECB);
}

static int rts_des3_ecb_decrypt(struct blkcipher_desc *desc,
			       struct scatterlist *dst,
			       struct scatterlist *src,
			       unsigned int nbytes)
{
	return rts_crypto(desc, dst, src, nbytes,
			  FLAGS_DECRYPT | FLAGS_DES3 | FLAGS_ECB);
}

static int rts_des3_cbc_encrypt(struct blkcipher_desc *desc,
			       struct scatterlist *dst,
			       struct scatterlist *src,
			       unsigned int nbytes)
{
	return rts_crypto(desc, dst, src, nbytes,
			  FLAGS_ENCRYPT | FLAGS_DES3 | FLAGS_CBC);
}

static int rts_des3_cbc_decrypt(struct blkcipher_desc *desc,
			       struct scatterlist *dst,
			       struct scatterlist *src,
			       unsigned int nbytes)
{
	return rts_crypto(desc, dst, src, nbytes,
			  FLAGS_DECRYPT | FLAGS_DES3 | FLAGS_CBC);
}

static int rts_des3_cbccs1_encrypt(struct blkcipher_desc *desc,
			       struct scatterlist *dst,
			       struct scatterlist *src,
			       unsigned int nbytes)
{
	return rts_crypto(desc, dst, src, nbytes,
			  FLAGS_ENCRYPT | FLAGS_DES3 | FLAGS_CBCCS1);
}

static int rts_des3_cbccs1_decrypt(struct blkcipher_desc *desc,
			       struct scatterlist *dst,
			       struct scatterlist *src,
			       unsigned int nbytes)
{
	return rts_crypto(desc, dst, src, nbytes,
			  FLAGS_DECRYPT | FLAGS_DES3 | FLAGS_CBCCS1);
}

static int rts_des3_cbccs2_encrypt(struct blkcipher_desc *desc,
			       struct scatterlist *dst,
			       struct scatterlist *src,
			       unsigned int nbytes)
{
	return rts_crypto(desc, dst, src, nbytes,
			  FLAGS_ENCRYPT | FLAGS_DES3 | FLAGS_CBCCS2);
}

static int rts_des3_cbccs2_decrypt(struct blkcipher_desc *desc,
			       struct scatterlist *dst,
			       struct scatterlist *src,
			       unsigned int nbytes)
{
	return rts_crypto(desc, dst, src, nbytes,
			  FLAGS_DECRYPT | FLAGS_DES3 | FLAGS_CBCCS2);
}

static int rts_des3_cbccs3_encrypt(struct blkcipher_desc *desc,
			       struct scatterlist *dst,
			       struct scatterlist *src,
			       unsigned int nbytes)
{
	return rts_crypto(desc, dst, src, nbytes,
			  FLAGS_ENCRYPT | FLAGS_DES3 | FLAGS_CBCCS3);
}

static int rts_des3_cbccs3_decrypt(struct blkcipher_desc *desc,
			       struct scatterlist *dst,
			       struct scatterlist *src,
			       unsigned int nbytes)
{
	return rts_crypto(desc, dst, src, nbytes,
			  FLAGS_DECRYPT | FLAGS_DES3 | FLAGS_CBCCS3);
}

static int rts_des3_ctr_encrypt(struct blkcipher_desc *desc,
			       struct scatterlist *dst,
			       struct scatterlist *src,
			       unsigned int nbytes)
{
	return rts_crypto(desc, dst, src, nbytes,
			  FLAGS_ENCRYPT | FLAGS_DES3 | FLAGS_CTR);
}

static int rts_des3_ctr_decrypt(struct blkcipher_desc *desc,
			       struct scatterlist *dst,
			       struct scatterlist *src,
			       unsigned int nbytes)
{
	return rts_crypto(desc, dst, src, nbytes,
			  FLAGS_DECRYPT | FLAGS_DES3 | FLAGS_CTR);
}

static struct crypto_alg rts_algs_aes[] = {
	[MODE_ECB] = {
		.cra_name = "ecb(aes)",
		.cra_driver_name = "ecb-aes-rlx",
		.cra_blocksize = AES_BLOCK_SIZE,
		.cra_u.blkcipher = {
			.min_keysize = AES_MIN_KEY_SIZE,
			.max_keysize = AES_MAX_KEY_SIZE,
			.setkey = rts_setkey,
			.encrypt = rts_aes_ecb_encrypt,
			.decrypt = rts_aes_ecb_decrypt,
		},
	},
	[MODE_CBC] = {
		.cra_name = "cbc(aes)",
		.cra_driver_name = "cbc-aes-rlx",
		.cra_blocksize = AES_BLOCK_SIZE,
		.cra_u.blkcipher = {
			.min_keysize = AES_MIN_KEY_SIZE,
			.max_keysize = AES_MAX_KEY_SIZE,
			.setkey = rts_setkey,
			.encrypt = rts_aes_cbc_encrypt,
			.decrypt = rts_aes_cbc_decrypt,
			.ivsize = AES_MIN_KEY_SIZE,
		},
	},
	[MODE_CBCCS1] = {
		.cra_name = "cbc-cs1(aes)",
		.cra_driver_name = "cbccs1-aes-rlx",
		.cra_blocksize = AES_BLOCK_SIZE,
		.cra_u.blkcipher = {
			.min_keysize = AES_MIN_KEY_SIZE,
			.max_keysize = AES_MAX_KEY_SIZE,
			.setkey = rts_setkey,
			.encrypt = rts_aes_cbccs1_encrypt,
			.decrypt = rts_aes_cbccs1_decrypt,
			.ivsize = AES_MIN_KEY_SIZE,
		},
	},
	[MODE_CBCCS2] = {
		.cra_name = "cbc-cs2(aes)",
		.cra_driver_name = "cbccs2-aes-rlx",
		.cra_blocksize = AES_BLOCK_SIZE,
		.cra_u.blkcipher = {
			.min_keysize = AES_MIN_KEY_SIZE,
			.max_keysize = AES_MAX_KEY_SIZE,
			.setkey = rts_setkey,
			.encrypt = rts_aes_cbccs2_encrypt,
			.decrypt = rts_aes_cbccs2_decrypt,
			.ivsize = AES_MIN_KEY_SIZE,
		},
	},
	[MODE_CBCCS3] = {
		.cra_name = "cbc-cs3(aes)",
		.cra_driver_name = "cbccs3-aes-rlx",
		.cra_blocksize = AES_BLOCK_SIZE,
		.cra_u.blkcipher = {
			.min_keysize = AES_MIN_KEY_SIZE,
			.max_keysize = AES_MAX_KEY_SIZE,
			.setkey = rts_setkey,
			.encrypt = rts_aes_cbccs3_encrypt,
			.decrypt = rts_aes_cbccs3_decrypt,
			.ivsize = AES_MIN_KEY_SIZE,
		},
	},
	[MODE_CTR] = {
		.cra_name = "ctr(aes)",
		.cra_driver_name = "ctr-aes-rlx",
		.cra_blocksize = AES_BLOCK_SIZE,
		.cra_u.blkcipher = {
			.min_keysize = AES_MIN_KEY_SIZE,
			.max_keysize = AES_MAX_KEY_SIZE,
			.setkey = rts_setkey,
			.encrypt = rts_aes_ctr_encrypt,
			.decrypt = rts_aes_ctr_decrypt,
			.ivsize = AES_MIN_KEY_SIZE,
		},
	},
};

static struct crypto_alg rts_algs_des[] = {
	[MODE_ECB] = {
		.cra_name = "ecb(des)",
		.cra_driver_name = "ecb-des-rlx",
		.cra_blocksize = DES_BLOCK_SIZE,
		.cra_u.blkcipher = {
			.min_keysize = DES_KEY_SIZE,
			.max_keysize = DES_KEY_SIZE,
			.setkey = rts_setkey,
			.encrypt = rts_des_ecb_encrypt,
			.decrypt = rts_des_ecb_decrypt,
		},
	},
	[MODE_CBC] = {
		.cra_name = "cbc(des)",
		.cra_driver_name = "cbc-des-rlx",
		.cra_blocksize = DES_BLOCK_SIZE,
		.cra_u.blkcipher = {
			.min_keysize = DES_KEY_SIZE,
			.max_keysize = DES_KEY_SIZE,
			.setkey = rts_setkey,
			.encrypt = rts_des_cbc_encrypt,
			.decrypt = rts_des_cbc_decrypt,
			.ivsize = DES_BLOCK_SIZE,
		},
	},
	[MODE_CBCCS1] = {
		.cra_name = "cbc-cs1(des)",
		.cra_driver_name = "cbccs1-des-rlx",
		.cra_blocksize = DES_BLOCK_SIZE,
		.cra_u.blkcipher = {
			.min_keysize = DES_KEY_SIZE,
			.max_keysize = DES_KEY_SIZE,
			.setkey = rts_setkey,
			.encrypt = rts_des_cbccs1_encrypt,
			.decrypt = rts_des_cbccs1_decrypt,
			.ivsize = DES_BLOCK_SIZE,
		},
	},
	[MODE_CBCCS2] = {
		.cra_name = "cbc-cs2(des)",
		.cra_driver_name = "cbccs2-des-rlx",
		.cra_blocksize = DES_BLOCK_SIZE,
		.cra_u.blkcipher = {
			.min_keysize = DES_KEY_SIZE,
			.max_keysize = DES_KEY_SIZE,
			.setkey = rts_setkey,
			.encrypt = rts_des_cbccs2_encrypt,
			.decrypt = rts_des_cbccs2_decrypt,
			.ivsize = DES_BLOCK_SIZE,
		},
	},
	[MODE_CBCCS3] = {
		.cra_name = "cbc-cs3(des)",
		.cra_driver_name = "cbccs3-des-rlx",
		.cra_blocksize = DES_BLOCK_SIZE,
		.cra_u.blkcipher = {
			.min_keysize = DES_KEY_SIZE,
			.max_keysize = DES_KEY_SIZE,
			.setkey = rts_setkey,
			.encrypt = rts_des_cbccs3_encrypt,
			.decrypt = rts_des_cbccs3_decrypt,
			.ivsize = DES_BLOCK_SIZE,
		},
	},
	[MODE_CTR] = {
		.cra_name = "ctr(des)",
		.cra_driver_name = "ctr-des-rlx",
		.cra_blocksize = DES_BLOCK_SIZE,
		.cra_u.blkcipher = {
			.min_keysize = DES_KEY_SIZE,
			.max_keysize = DES_KEY_SIZE,
			.setkey = rts_setkey,
			.encrypt = rts_des_ctr_encrypt,
			.decrypt = rts_des_ctr_decrypt,
			.ivsize = DES_BLOCK_SIZE,
		},
	},
};

static struct crypto_alg rts_algs_des3[] = {
	[MODE_ECB] = {
		.cra_name = "ecb(des3_ede)",
		.cra_driver_name = "ecb-des3_ede-rlx",
		.cra_blocksize = DES3_EDE_BLOCK_SIZE,
		.cra_u.blkcipher = {
			.min_keysize = DES3_EDE_KEY_SIZE,
			.max_keysize = DES3_EDE_KEY_SIZE,
			.setkey = rts_setkey,
			.encrypt = rts_des3_ecb_encrypt,
			.decrypt = rts_des3_ecb_decrypt,
		},
	},
	[MODE_CBC] = {
		.cra_name = "cbc(des3_ede)",
		.cra_driver_name = "cbc-des3_ede-rlx",
		.cra_blocksize = DES3_EDE_BLOCK_SIZE,
		.cra_u.blkcipher = {
			.min_keysize = DES3_EDE_KEY_SIZE,
			.max_keysize = DES3_EDE_KEY_SIZE,
			.setkey = rts_setkey,
			.encrypt = rts_des3_cbc_encrypt,
			.decrypt = rts_des3_cbc_decrypt,
			.ivsize = DES3_EDE_BLOCK_SIZE,
		},
	},
	[MODE_CBCCS1] = {
		.cra_name = "cbc-cs1(des3_ede)",
		.cra_driver_name = "cbccs1-des3_ede-rlx",
		.cra_blocksize = DES3_EDE_BLOCK_SIZE,
		.cra_u.blkcipher = {
			.min_keysize = DES3_EDE_KEY_SIZE,
			.max_keysize = DES3_EDE_KEY_SIZE,
			.setkey = rts_setkey,
			.encrypt = rts_des3_cbccs1_encrypt,
			.decrypt = rts_des3_cbccs1_decrypt,
			.ivsize = DES3_EDE_BLOCK_SIZE,
		},
	},
	[MODE_CBCCS2] = {
		.cra_name = "cbc-cs2(des3_ede)",
		.cra_driver_name = "cbccs2-des3_ede-rlx",
		.cra_blocksize = DES3_EDE_BLOCK_SIZE,
		.cra_u.blkcipher = {
			.min_keysize = DES3_EDE_KEY_SIZE,
			.max_keysize = DES3_EDE_KEY_SIZE,
			.setkey = rts_setkey,
			.encrypt = rts_des3_cbccs2_encrypt,
			.decrypt = rts_des3_cbccs2_decrypt,
			.ivsize = DES3_EDE_BLOCK_SIZE,
		},
	},
	[MODE_CBCCS3] = {
		.cra_name = "cbc-cs3(des3_ede)",
		.cra_driver_name = "cbccs3-des3_ede-rlx",
		.cra_blocksize = DES3_EDE_BLOCK_SIZE,
		.cra_u.blkcipher = {
			.min_keysize = DES3_EDE_KEY_SIZE,
			.max_keysize = DES3_EDE_KEY_SIZE,
			.setkey = rts_setkey,
			.encrypt = rts_des3_cbccs3_encrypt,
			.decrypt = rts_des3_cbccs3_decrypt,
			.ivsize = DES3_EDE_BLOCK_SIZE,
		},
	},
	[MODE_CTR] = {
		.cra_name = "ctr(des3_ede)",
		.cra_driver_name = "ctr-des3_ede-rlx",
		.cra_blocksize = DES3_EDE_BLOCK_SIZE,
		.cra_u.blkcipher = {
			.min_keysize = DES3_EDE_KEY_SIZE,
			.max_keysize = DES3_EDE_KEY_SIZE,
			.setkey = rts_setkey,
			.encrypt = rts_des3_ctr_encrypt,
			.decrypt = rts_des3_ctr_decrypt,
			.ivsize = DES3_EDE_BLOCK_SIZE,
		},
	}
};

static int crypto_open(struct inode *inode, struct file *file)
{
	struct rts_crypto_data *cdata =
		container_of(inode->i_cdev, struct rts_crypto_data, cdev);

	file->private_data = cdata;
	return 0;
}

static int crypto_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long crypto_ioctl(struct file *file,
			unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct rts_crypto_data *cdata = file->private_data;

	if (!cdata)
		return -EINVAL;

	switch (cmd) {
	case RTS_CIPHER_IOC_SET_NORMALKEY:
		cdata->efusekey = 0;
		break;
	case RTS_CIPHER_IOC_SET_EFUSEKEY:
		cdata->efusekey = 1;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct file_operations crypto_fops = {
	.owner = THIS_MODULE,
	.open = crypto_open,
	.release = crypto_release,
	.unlocked_ioctl = crypto_ioctl,
};

static struct crypto_alg *rts_algs[] = {
	[ALG_AES] = rts_algs_aes,
	[ALG_DES] = rts_algs_des,
	[ALG_DES3] = rts_algs_des3,
};

static int rts_parse_dts(struct platform_device *pdev)
{
	struct rts_crypto_data *cdata;
	int ret, i, j;
	static const char * const alg[] = {
		[ALG_AES] = "aes",
		[ALG_DES] = "des",
		[ALG_DES3] = "3des"
	};
	static const char * const mode[] = {
		[MODE_ECB] = "ecb",
		[MODE_CBC] = "cbc",
		[MODE_CBCCS1] = "cbccs1",
		[MODE_CBCCS2] = "cbccs2",
		[MODE_CBCCS3] = "cbccs3",
		[MODE_CTR] = "ctr"
	};

	if (!pdev)
		return -EINVAL;

	cdata = platform_get_drvdata(pdev);
	if (!cdata) {
		dev_err(&pdev->dev, "get platform_drvdata failed.\n");
		return -EINVAL;
	}

	for (i = 0; i < ALG_COUNT && alg[i]; i++) {
		for (j = 0; j < MODE_COUNT && mode[j]; j++) {
			ret = of_property_match_string(pdev->dev.of_node,
						alg[i], mode[j]);
			if (ret >= 0)
				set_mode_cap(cdata->mode_cap[i], j);
		}
	}

	return 0;
}

static const struct of_device_id rts_crypto_dt_ids[] = {
	{ .compatible = "realtek,rts3903-crypto" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rts_crypto_dt_ids);

static irqreturn_t rts_crypto_irq_handler(int irq, void *data)
{
	struct rts_crypto_data *crypto_data;
	u32 val;

	crypto_data = (struct rts_crypto_data *)data;

	val = rts_crypto_read(crypto_data, RLX_REG_CIPHER_INT_FLAG);

	val = (~val) & 0x5;
	rts_crypto_write(crypto_data, RLX_REG_CIPHER_INT_EN, val);
	complete(&crypto_data->crypto_complete);

	return IRQ_HANDLED;
}

static struct platform_device_id rts_crypto_devtypes[] = {
	{
		.name = "rle0745-fpga-crypto",
		.driver_data = TYPE_RLE0745 | TYPE_FPGA,
	}, {
		.name = "rlx0745-crypto",
		.driver_data = TYPE_RLE0745,
	}, {
		.name = "rts3901-fpga-crypto",
		.driver_data = TYPE_RTS3901 | TYPE_FPGA,
	}, {
		.name = "rts3901-crypto",
		.driver_data = TYPE_RTS3901,
	}, {
		.name = "rts3903-fpga-crypto",
		.driver_data = TYPE_RTS3903 | TYPE_FPGA,
	}, {
		.name = "rts3903-crypto",
		.driver_data = TYPE_RTS3903,
	},
};

static int rts_crypto_probe(struct platform_device *pdev)
{
	int ret, i, j, k, g;
	struct resource *res;
	struct rts_crypto_data *cdata;

	cdata = devm_kzalloc(&pdev->dev, sizeof(*cdata), GFP_KERNEL);
	if (cdata == NULL)
		return -ENOMEM;

	cdata->pdev = pdev;

	platform_set_drvdata(pdev, cdata);


	ret = rts_parse_dts(pdev);
	if (ret) {
		dev_err(&pdev->dev, "parse dts failed.\n");
		goto mem_err;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		pr_err("unable to get crypto address\n");
		ret = -ENXIO;
		goto mem_err;
	}

	cdata->base = res->start;
	cdata->size = res->end - res->start + 1;

	cdata->addr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(cdata->addr)) {
		pr_err("unable to ioremap\n");
		ret = -ENXIO;
		goto mem_err;
	}

	cdata->cipher_clk = devm_clk_get(&pdev->dev, "cipher_ck");
	if (IS_ERR(cdata->cipher_clk)) {
		dev_err(&pdev->dev, "clock initialization failed.\n");
		goto mem_err;
	}

	ret = clk_prepare_enable(cdata->cipher_clk);
	if (ret) {
		dev_err(&pdev->dev, "clock prepare failed.\n");
		goto mem_err;
	}

	cdata->rst = devm_reset_control_get(&pdev->dev, "rst");
	if (IS_ERR(cdata->rst)) {
		dev_err(&pdev->dev, "no top level reset found.\n");
		goto mem_err;
	}

	/* reset crypto */
	reset_control_reset(cdata->rst);

	cdata->sd = devm_reset_control_get(&pdev->dev, "sd");
	if (IS_ERR(cdata->rst)) {
		dev_err(&pdev->dev, "no top level reset found.\n");
		goto mem_err;
	}

	reset_control_assert(cdata->sd);

	cdata->irq = platform_get_irq(pdev, 0);
	if (cdata->irq < 0) {
		dev_err(&pdev->dev, "can't get IRQ resource\n");
		goto mem_err;
	}

	ret = devm_request_irq(&pdev->dev, cdata->irq, rts_crypto_irq_handler,
			  0, dev_name(&pdev->dev), cdata);

	DBG("using IRQ channel %d\n", cdata->irq);

	cdata->table_in
	    = dma_alloc_coherent(&pdev->dev, RLX_DMA_TABLE_SIZE,
				 &cdata->dma_table_in, GFP_KERNEL);
	if (!cdata->table_in) {
		pr_err("Unable to allocate dma table in buffer\n");
		ret = -ENOMEM;
		goto dma_err;
	}

	cdata->table_out
	    = dma_alloc_coherent(&pdev->dev, RLX_DMA_TABLE_SIZE,
				 &cdata->dma_table_out, GFP_KERNEL);
	if (!cdata->table_out) {
		pr_err("Unable to allocate dma table out buffer\n");
		ret = -ENOMEM;
		goto dma_err;
	}

	for (i = 0; i < ALG_COUNT; i++) {
		for (j = 0; j < MODE_COUNT; j++) {
			if (!get_mode_cap(cdata->mode_cap[i], j))
				continue;

			rts_algs[i][j].cra_priority = 300;
			rts_algs[i][j].cra_flags =
					CRYPTO_ALG_TYPE_BLKCIPHER,
			rts_algs[i][j].cra_ctxsize =
					sizeof(struct rts_crypto_ctx);
			rts_algs[i][j].cra_alignmask = 3;
			rts_algs[i][j].cra_type = &crypto_blkcipher_type;
			rts_algs[i][j].cra_module = THIS_MODULE;

			ret = crypto_register_alg(&rts_algs[i][j]);
			if (ret)
				goto reg_err;
		}
	}

	/* create dev node */
	ret = register_chrdev_region(devno, 1, "crypto");
	if (ret) {
		dev_err(&pdev->dev, "register_chrdev_region failed.\n");
		goto reg_err;
	}

	cdev_init(&cdata->cdev, &crypto_fops);

	ret = cdev_add(&cdata->cdev, devno, 1);
	if (ret) {
		dev_err(&pdev->dev, "cdev_add failed.\n");
		goto cdev_err;
	}

	rts_cdata = cdata;

	mutex_init(&rts_cdata->dma_mutex);
	init_completion(&rts_cdata->crypto_complete);

	mdelay(5);

	dev_info(&pdev->dev, "Realtek RLX crypto driver initialized\n");

	return 0;

cdev_err:
	unregister_chrdev_region(devno, 1);
reg_err:
	for (k = 0; k < i; k++) {
		for (g = 0; g < j; g++) {
			if (!get_mode_cap(cdata->mode_cap[k], g))
				continue;

			crypto_unregister_alg(&rts_algs[k][g]);
		}
	}
dma_err:
	if (cdata->table_in)
		dma_free_coherent(&pdev->dev, RLX_DMA_TABLE_SIZE,
				  cdata->table_in,
				  cdata->dma_table_in);
	if (cdata->table_out)
		dma_free_coherent(&pdev->dev, RLX_DMA_TABLE_SIZE,
				  cdata->table_out,
				  cdata->dma_table_out);

mem_err:
	devm_kfree(&pdev->dev, cdata);
	cdata = NULL;

	return ret;
}

static int rts_crypto_remove(struct platform_device *pdev)
{
	struct rts_crypto_data *cdata;
	struct resource *res;
	int i, j;

	cdata = platform_get_drvdata(pdev);

	/* remove dev node */
	cdev_del(&cdata->cdev);
	unregister_chrdev_region(devno, 1);

	/* memory sd down */
	reset_control_deassert(cdata->sd);

	for (i = 0; i < ALG_COUNT; i++) {
		for (j = 0; j < MODE_COUNT; j++) {
			if (!get_mode_cap(cdata->mode_cap[i], j))
				continue;

			crypto_unregister_alg(&rts_algs[i][j]);
		}
	}

	if (cdata->table_in)
		dma_free_coherent(&pdev->dev, RLX_DMA_TABLE_SIZE,
				  cdata->table_in,
				  cdata->dma_table_in);
	if (cdata->table_out)
		dma_free_coherent(&pdev->dev, RLX_DMA_TABLE_SIZE,
				  cdata->table_out,
				  cdata->dma_table_out);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res)
		release_mem_region(cdata->base, resource_size(res));

	clk_disable_unprepare(cdata->cipher_clk);

	devm_kfree(&pdev->dev, cdata);
	dev_set_drvdata(&pdev->dev, NULL);
	cdata = NULL;
	rts_cdata = NULL;

	return 0;
}

static struct platform_driver rts_crypto_driver = {
	.probe = rts_crypto_probe,
	.remove = rts_crypto_remove,
	.driver = {
		.name = "rts-crypto",
		.of_match_table = of_match_ptr(rts_crypto_dt_ids),
	},
	.id_table = rts_crypto_devtypes,
};
module_platform_driver(rts_crypto_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Wind_Han <wind_han@realsil.com.cn>");
MODULE_DESCRIPTION("Realtek RLX crypto driver");
