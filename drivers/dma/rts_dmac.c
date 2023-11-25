/*
 * Core driver for the Synopsys DesignWare DMA Controller
 *
 * Copyright (C) 2007-2008 Atmel Corporation
 * Copyright (C) 2010-2011 ST Microelectronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_dma.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/acpi.h>
#include <linux/acpi_dma.h>
#include <linux/sched.h>
#include <linux/freezer.h>
#include <linux/wait.h>
#include <linux/dma-direction.h>
#include <linux/dmaengine.h>
#include <linux/pm_runtime.h>

#include "rts_dmac_regs.h"
#include "dmaengine.h"

/*
 * This supports the Synopsys "DesignWare AHB Central DMA Controller",
 * (DW_ahb_dmac) which is used with various AMBA 2.0 systems (not all
 * of which use ARM any more).  See the "Databook" from Synopsys for
 * information beyond what licensees probably provide.
 *
 * The driver has currently been tested only with the Atmel AT32AP7000,
 * which does not support descriptor writeback.
 */

static inline void dw_set_masters(struct dw_dma *dw)
{
	dw->src_master = 0;
	dw->dst_master = 0;
}

/* Get ctl value for config CTLx register */
#define DWC_DEFAULT_CTLLO(_chan) ({					\
		struct dw_dma_vchan *_dwvc = to_dw_dma_vchan(_chan);	\
		struct dw_dma *_dw = to_dw_dma(_chan->device);		\
		struct dma_slave_config	*_sconfig = &_dwvc->dma_sconfig;\
		bool _is_slave = is_slave_direction(_sconfig->direction);\
		u8 _smsize = _is_slave ? _sconfig->src_maxburst :	\
			DW_DMA_MSIZE_16;				\
		u8 _dmsize = _is_slave ? _sconfig->dst_maxburst :	\
			DW_DMA_MSIZE_16;				\
									\
		(DWC_CTLL_DST_MSIZE(_dmsize)				\
		 | DWC_CTLL_SRC_MSIZE(_smsize)				\
		 | DWC_CTLL_LLP_D_EN					\
		 | DWC_CTLL_LLP_S_EN					\
		 | DWC_CTLL_DMS(_dw->dst_master)			\
		 | DWC_CTLL_SMS(_dw->src_master));			\
	})

/* The set of bus widths supported by the DMA controller */
#define RTS_DMA_BUSWIDTHS				\
		BIT(DMA_SLAVE_BUSWIDTH_UNDEFINED) |	\
		BIT(DMA_SLAVE_BUSWIDTH_1_BYTE) |	\
		BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) |	\
		BIT(DMA_SLAVE_BUSWIDTH_4_BYTES)

/*
 * Number of descriptors to allocate for each channel. This should be
 * made configurable somehow; preferably, the clients (at least the
 * ones using slave transfers) should be able to give us a hint.
 */
#define NR_DESCS_PER_CHANNEL	64

/*----------------------------------------------------------------------*/

static struct device *chan2dev(struct dma_chan *chan)
{
	return &chan->dev->device;
}

static void dwc_initialize(struct dw_dma_chan *dwc, struct dw_dma *dw)
{
	u32 cfghi = DWC_CFGH_FIFO_MODE;
	u32 cfglo = DWC_CFGL_CH_PRIOR(dwc->priority);

	if (dwc->initialized == true)
		return;

	if (dwc->direction == DMA_MEM_TO_DEV)
		cfghi = DWC_CFGH_DST_PER(dwc->request_line);
	else if (dwc->direction == DMA_DEV_TO_MEM)
		cfghi = DWC_CFGH_SRC_PER(dwc->request_line);

	channel_writel(dwc, CFG_LO, cfglo);
	channel_writel(dwc, CFG_HI, cfghi);

	/* Enable interrupts */
	channel_set_bit(dw, MASK.XFER, dwc->mask);
	channel_set_bit(dw, MASK.ERROR, dwc->mask);

	dwc->initialized = true;
}

/*----------------------------------------------------------------------*/

static inline unsigned int dwc_fast_fls(unsigned long long v)
{
	/*
	 * We can be a lot more clever here, but this should take care
	 * of the most common optimization.
	 */
	if (!(v & 7))
		return 3;
	else if (!(v & 3))
		return 2;
	else if (!(v & 1))
		return 1;
	return 0;
}

static inline void dwc_chan_disable(struct dw_dma *dw, struct dw_dma_chan *dwc)
{
	channel_clear_bit(dw, CH_EN, dwc->mask);
	while (dma_readl(dw, CH_EN) & dwc->mask)
		cpu_relax();
}

/*----------------------------------------------------------------------*/

/* Perform single block transfer */
static inline void dwc_do_single_block(struct dw_dma *dw,
				       struct dw_dma_chan *dwc,
				       struct dw_desc *desc)
{
	u32		ctllo;

	/*
	 * Software emulation of LLP mode relies on interrupts to continue
	 * multi block transfer.
	 */
	ctllo = desc->lli.ctllo | DWC_CTLL_INT_EN;

	channel_writel(dwc, SAR, desc->lli.sar);
	channel_writel(dwc, DAR, desc->lli.dar);
	channel_writel(dwc, CTL_LO, ctllo);
	channel_writel(dwc, CTL_HI, desc->lli.ctlhi);
	channel_set_bit(dw, CH_EN, dwc->mask);

	/* Move pointer to next descriptor */
	dwc->tx_node_active = dwc->tx_node_active->next;
}

/* Called with dwc->lock held and bh disabled */
static int dwc_dostart(struct dw_dma_vchan *dwvc,
		struct dw_dma_chan *dwc)
{
	struct dw_dma	*dw = to_dw_dma(dwvc->vc.chan.device);
	struct virt_dma_desc *vdesc = vchan_next_desc(&dwvc->vc);
	struct dw_desc *first;
	unsigned long	was_soft_llp;
	unsigned long	was_cyclic;

	/* ASSERT:  channel is idle */
	if (dma_readl(dw, CH_EN) & dwc->mask) {
		dev_err(chan2dev(&dwvc->vc.chan),
			"BUG: Attempted to start non-idle channel\n");

		/* The tasklet will hopefully advance the queue... */
		return -EBUSY;
	}

	if (!vdesc)
		return -EAGAIN;

	first = vd_to_dw_desc(vdesc);

	list_del(&vdesc->node);
	first->pchan = dwc;
	dwc->vdesc = first;

	if (first->is_cyclic) {
		was_cyclic = test_and_set_bit(DW_DMA_IS_CYCLIC,
						&dwc->flags);
		if (was_cyclic) {
			dev_err(chan2dev(&dwvc->vc.chan),
				"BUG: Attempted to start cyclic transfer\n");
			return -EBUSY;
		}
	} else {
		clear_bit(DW_DMA_IS_CYCLIC, &dwc->flags);
	}

	if ((first->direction == DMA_MEM_TO_DEV) ||
		(first->direction == DMA_DEV_TO_MEM)) {
		dwc->initialized = false;
		dwc->request_line = first->request_line;
	} else if (dwc->direction != first->direction) {
		dwc->initialized = false;
		dwc->request_line = ~0;
	}
	dwc->direction = first->direction;

	if (dw->nollp) {
		was_soft_llp = test_and_set_bit(DW_DMA_IS_SOFT_LLP,
						&dwc->flags);
		if (was_soft_llp) {
			dev_err(chan2dev(&dwvc->vc.chan),
				"BUG: Attempted to start new LLP transfer\n");
			return -EBUSY;
		}

		dwc_initialize(dwc, dw);

		dwc->tx_node_active = &first->tx_list;

		/* Submit first block */
		dwc_do_single_block(dw, dwc, first);

		return 0;
	}

	dwc_initialize(dwc, dw);

	channel_writel(dwc, LLP, first->vd.tx.phys);
	channel_writel(dwc, CTL_LO,
			DWC_CTLL_LLP_D_EN | DWC_CTLL_LLP_S_EN);
	channel_writel(dwc, CTL_HI, 0);

	channel_set_bit(dw, CH_EN, dwc->mask);

	return 0;
}

static struct dw_desc *dwc_desc_get(struct dw_dma *dw)
{
	struct dw_desc *desc;
	dma_addr_t phys;

	desc = dma_pool_alloc(dw->desc_pool, GFP_ATOMIC, &phys);
	if (!desc)
		return NULL;

	memset(desc, 0, sizeof(struct dw_desc));
	INIT_LIST_HEAD(&desc->tx_list);

	desc->vd.tx.phys = phys;
	desc->vd.tx.flags = DMA_CTRL_ACK;
	desc->is_cyclic = false;
	desc->request_line = ~0;

	return desc;
}

static void dwc_desc_put(struct dw_dma *dw, struct dw_desc *desc)
{
	if (desc) {
		struct dw_desc *child, *_child;

		list_for_each_entry_safe(child, _child,
				&desc->tx_list, desc_node) {
			list_del(&child->desc_node);
			dma_pool_free(dw->desc_pool, child, child->vd.tx.phys);
		}
		dma_pool_free(dw->desc_pool, desc, desc->vd.tx.phys);
	}
}

/* schedule for 4 physical dma channels */
static void dw_scan_descriptors(struct dw_dma *dw)
{
	struct dw_dma_chan *dwc;
	struct dw_dma_vchan *dwvc;
	int ret, process;
	unsigned int i;
	unsigned long flags;

	spin_lock_irqsave(&dw->lock, flags);
	if (list_empty(&dw->free_list)) {
		spin_unlock_irqrestore(&dw->lock, flags);
		return;
	}

	if (dw->vchan_index > dw->pdata->nr_vchannels)
		dw->vchan_index = 0;
	i = dw->vchan_index;
	process = 0;

	while (!list_empty(&dw->free_list)) {
		dwc = list_first_entry(&dw->free_list,
			struct dw_dma_chan, node);
		dwvc = &dw->vchan[i];
		if (dwvc->pause == 0) {
			spin_lock(&dwvc->vc.lock);
			ret = dwc_dostart(dwvc, dwc);
			spin_unlock(&dwvc->vc.lock);
			if (!ret) {
				process++;
				list_del_init(&dwc->node);
			}
		}
		i++;
		if (i >= dw->pdata->nr_vchannels)
			i = 0;
		if (i == dw->vchan_index) {
			if (process == 0)
				break;
			process = 0;
		}
	}
	dw->vchan_index = i;
	spin_unlock_irqrestore(&dw->lock, flags);
}

static int dw_scan_descriptor_single(struct dw_dma *dw,
		struct dw_dma_chan *dwc)
{
	struct list_head *head, *active = dwc->tx_node_active;
	struct dw_desc *first;
	struct dw_desc *child;
	unsigned long flags;

	spin_lock_irqsave(&dw->lock, flags);
	first = dwc->vdesc;
	head = &first->tx_list;
	if (active != head) {
		child = to_dw_desc(active);

		dwc_do_single_block(dw, dwc, child);
		spin_unlock_irqrestore(&dw->lock, flags);
		return 1;
	}

	clear_bit(DW_DMA_IS_SOFT_LLP, &dwc->flags);

	spin_unlock_irqrestore(&dw->lock, flags);

	return 0;
}

static void dw_dma_tasklet(unsigned long data)
{
	struct dw_dma *dw = (struct dw_dma *)data;
	struct dw_dma_chan *dwc;
	struct dw_dma_vchan *dwvc;
	u32 status_xfer;
	u32 status_err;
	int i;
	unsigned long flags;

	status_xfer = dma_readl(dw, RAW.XFER);
	status_err = dma_readl(dw, RAW.ERROR);

	dev_vdbg(dw->dma.dev, "%s: status_err=%x\n", __func__, status_err);

	for (i = 0; i < dw->pdata->nr_channels; i++) {
		dwc = &dw->pchan[i];
		if (status_err & dwc->mask) {
			dma_writel(dw, CLEAR.ERROR, dwc->mask);
			dma_writel(dw, CLEAR.XFER, dwc->mask);
			if (dwc->vdesc && dwc->vdesc->vchan) {
				dev_err(dw->dma.dev,
					"BUG: channel %i error [0x%x]\n",
					i, status_err);
				dwvc = dwc->vdesc->vchan;
				spin_lock_irqsave(&dwvc->vc.lock, flags);
				vchan_cookie_complete(&dwc->vdesc->vd);
				spin_unlock_irqrestore(&dwvc->vc.lock, flags);
			}
		} else if (status_xfer & dwc->mask) {
			dma_writel(dw, CLEAR.XFER, dwc->mask);
			if (dwc->vdesc && dwc->vdesc->vchan) {
				if (test_bit(DW_DMA_IS_SOFT_LLP,
					&dwc->flags) &&
					dw_scan_descriptor_single(dw, dwc))
					continue;

				dwvc = dwc->vdesc->vchan;
				if (test_bit(DW_DMA_IS_CYCLIC,
					&dwc->flags)) {
					vchan_cyclic_callback(&dwc->vdesc->vd);
				} else {
					spin_lock_irqsave(&dwvc->vc.lock,
							flags);
					vchan_cookie_complete(&dwc->vdesc->vd);
					spin_unlock_irqrestore(&dwvc->vc.lock,
							flags);
				}
			}
		}
	}

	/*
	 * Re-enable interrupts.
	 */
	channel_set_bit(dw, MASK.XFER, dw->all_chan_mask);
	channel_set_bit(dw, MASK.ERROR, dw->all_chan_mask);
}

static irqreturn_t dw_dma_interrupt(int irq, void *dev_id)
{
	struct dw_dma *dw = dev_id;
	u32 status;

	dev_vdbg(dw->dma.dev, "%s: status=0x%x\n", __func__,
			dma_readl(dw, STATUS_INT));

	/*
	 * Just disable the interrupts. We'll turn them back on in the
	 * softirq handler.
	 */
	channel_clear_bit(dw, MASK.XFER, dw->all_chan_mask);
	channel_clear_bit(dw, MASK.ERROR, dw->all_chan_mask);

	status = dma_readl(dw, STATUS_INT);
	if (status) {
		dev_err(dw->dma.dev,
			"BUG: Unexpected interrupts pending: 0x%x\n",
			status);

		/* Try to recover */
		channel_clear_bit(dw, MASK.XFER, (1 << 8) - 1);
		channel_clear_bit(dw, MASK.SRC_TRAN, (1 << 8) - 1);
		channel_clear_bit(dw, MASK.DST_TRAN, (1 << 8) - 1);
		channel_clear_bit(dw, MASK.ERROR, (1 << 8) - 1);
	}

	tasklet_schedule(&dw->tasklet);

	return IRQ_HANDLED;
}

static void dwc_dma_free_desc(struct virt_dma_desc *vd)
{
	struct dw_desc *desc = vd_to_dw_desc(vd);
	struct dw_dma *dw = to_dw_dma(vd->tx.chan->device);
	struct dw_dma_chan *dwc = desc->pchan;
	unsigned long flags;

	dwc_desc_put(dw, desc);

	spin_lock_irqsave(&dw->lock, flags);
	if (dwc) {
		dwc_chan_disable(dw, dwc);
		dwc->vdesc = NULL;
		if (test_bit(DW_DMA_IS_CYCLIC, &dwc->flags))
			clear_bit(DW_DMA_IS_CYCLIC, &dwc->flags);
		list_add_tail(&dwc->node, &dw->free_list);
	}
	spin_unlock_irqrestore(&dw->lock, flags);

	if (dwc)
		dw_scan_descriptors(dw);
}

/*----------------------------------------------------------------------*/

static struct dma_async_tx_descriptor *
dwc_prep_dma_memcpy(struct dma_chan *chan, dma_addr_t dest, dma_addr_t src,
		size_t len, unsigned long flags)
{
	struct dw_dma_vchan	*dwvc = to_dw_dma_vchan(chan);
	struct dw_dma		*dw = to_dw_dma(chan->device);
	struct dw_desc		*desc;
	struct dw_desc		*first;
	struct dw_desc		*prev;
	size_t			xfer_count;
	size_t			offset;
	unsigned int		src_width;
	unsigned int		dst_width;
	unsigned int		data_width;
	u32			ctllo;

	dev_vdbg(chan2dev(chan),
			"%s: d0x%llx s0x%llx l0x%zx f0x%lx\n", __func__,
			(unsigned long long)dest, (unsigned long long)src,
			len, flags);

	if (unlikely(!len)) {
		dev_dbg(chan2dev(chan), "%s: length is zero!\n", __func__);
		return NULL;
	}

	dwvc->dma_sconfig.direction = DMA_MEM_TO_MEM;

	data_width = min_t(unsigned int, dw->pdata->data_width[dw->src_master],
			   dw->pdata->data_width[dw->dst_master]);

	src_width = dst_width = min_t(unsigned int, data_width,
				      dwc_fast_fls(src | dest | len));

	ctllo = DWC_DEFAULT_CTLLO(chan)
			| DWC_CTLL_DST_WIDTH(dst_width)
			| DWC_CTLL_SRC_WIDTH(src_width)
			| DWC_CTLL_DST_INC
			| DWC_CTLL_SRC_INC
			| DWC_CTLL_FC_M2M;

	if (dw->nollp) {
		ctllo &= ~DWC_CTLL_LLP_D_EN;
		ctllo &= ~DWC_CTLL_LLP_S_EN;
	}

	prev = first = NULL;

	for (offset = 0; offset < len; offset += xfer_count << src_width) {
		xfer_count = min_t(size_t, (len - offset) >> src_width,
					   dw->block_size);

		desc = dwc_desc_get(dw);
		if (!desc)
			goto err_desc_get;

		desc->lli.sar = src + offset;
		desc->lli.dar = dest + offset;
		desc->lli.ctllo = ctllo;
		desc->lli.ctlhi = xfer_count;
		desc->len = xfer_count << src_width;
		desc->pchan = NULL;
		desc->vchan = dwvc;

		if (!first) {
			first = desc;
		} else {
			prev->lli.llp = desc->vd.tx.phys;
			list_add_tail(&desc->desc_node,
					&first->tx_list);
		}
		prev = desc;
	}

	if (flags & DMA_PREP_INTERRUPT) {
		/* Trigger interrupt after last block */
		prev->lli.ctllo |= DWC_CTLL_INT_EN;
		prev->lli.ctllo &= ~DWC_CTLL_LLP_D_EN;
		prev->lli.ctllo &= ~DWC_CTLL_LLP_S_EN;
	}

	prev->lli.llp = 0;
	first->total_len = len;
	first->direction = DMA_MEM_TO_MEM;
	first->request_line = ~0;

	return vchan_tx_prep(&dwvc->vc, &first->vd, flags);

err_desc_get:
	dwc_desc_put(dw, first);

	return NULL;
}

static int dwc_sg_size(struct scatterlist *sg, unsigned int nents)
{
	int size = 0, i;
	struct scatterlist *s;

	if (sg == NULL || nents == 0)
		return -EINVAL;

	for_each_sg(sg, s, nents, i)
		size += sg_dma_len(s);

	return size;
}

static struct dma_async_tx_descriptor *
dwc_prep_dma_sg(struct dma_chan *chan,
		struct scatterlist *dst_sg, unsigned int dst_nents,
		struct scatterlist *src_sg, unsigned int src_nents,
		unsigned long flags)
{
	struct dw_dma_vchan	*dwvc = to_dw_dma_vchan(chan);
	struct dw_dma		*dw = to_dw_dma(chan->device);
	struct dw_desc		*desc;
	struct dw_desc		*first;
	struct dw_desc		*prev;
	size_t			xfer_count;
	unsigned int		src_width;
	unsigned int		dst_width;
	unsigned int		data_width;
	u32			ctllo;
	u32			ctllo_t;
	size_t			dst_avail;
	size_t			src_avail;
	size_t			len;
	size_t			total_len = 0;
	dma_addr_t		dst;
	dma_addr_t		src;

	dev_vdbg(chan2dev(chan),
			"%s: d %d s %d f0x%lx\n", __func__,
			dst_nents, src_nents, flags);

	if (src_sg == NULL || src_nents == 0) {
		dev_dbg(chan2dev(chan), "%s: src length is zero!\n", __func__);
		return NULL;
	}

	if (dst_sg == NULL || dst_nents == 0) {
		dev_dbg(chan2dev(chan), "%s: dst length is zero!\n", __func__);
		return NULL;
	}

	dst_avail = dwc_sg_size(dst_sg, dst_nents);
	src_avail = dwc_sg_size(src_sg, src_nents);
	if (src_avail != dst_avail) {
		dev_dbg(chan2dev(chan),
			"%s: src length != dst length!\n", __func__);
		return NULL;
	}

	dwvc->dma_sconfig.direction = DMA_MEM_TO_MEM;
	data_width = min_t(unsigned int, dw->pdata->data_width[dw->src_master],
			   dw->pdata->data_width[dw->dst_master]);

	ctllo_t = DWC_DEFAULT_CTLLO(chan)
			| DWC_CTLL_DST_INC
			| DWC_CTLL_SRC_INC
			| DWC_CTLL_FC_M2M;

	if (dw->nollp) {
		ctllo_t &= ~DWC_CTLL_LLP_D_EN;
		ctllo_t &= ~DWC_CTLL_LLP_S_EN;
	}

	dst_avail = sg_dma_len(dst_sg);
	src_avail = sg_dma_len(src_sg);

	prev = first = NULL;

	while (true) {
		len = min_t(size_t, src_avail, dst_avail);
		if (len == 0)
			goto fetch;

		dst = sg_dma_address(dst_sg) + sg_dma_len(dst_sg) - dst_avail;
		src = sg_dma_address(src_sg) + sg_dma_len(src_sg) - src_avail;

		src_width = dst_width = min_t(unsigned int, data_width,
				dwc_fast_fls(src | dst | len));

		ctllo = ctllo_t
			| DWC_CTLL_DST_WIDTH(dst_width)
			| DWC_CTLL_SRC_WIDTH(src_width);

		xfer_count = min_t(size_t, len >> src_width, dw->block_size);
		len = xfer_count << src_width;

		desc = dwc_desc_get(dw);
		if (!desc) {
			dev_dbg(chan2dev(chan), "%s: get no desc\n", __func__);
			goto err_desc_get;
		}

		desc->lli.sar = src;
		desc->lli.dar = dst;
		desc->lli.ctllo = ctllo;
		desc->lli.ctlhi = xfer_count;
		desc->len = len;
		desc->pchan = NULL;
		desc->vchan = dwvc;

		if (!first) {
			first = desc;
		} else {
			prev->lli.llp = desc->vd.tx.phys;
			list_add_tail(&desc->desc_node,
					&first->tx_list);
		}
		prev = desc;
		total_len += len;

		dst_avail -= len;
		src_avail -= len;
fetch:
		if (dst_avail == 0) {
			if (dst_nents == 0)
				break;

			dst_sg = sg_next(dst_sg);
			if (dst_sg == NULL)
				break;

			dst_nents--;
			dst_avail = sg_dma_len(dst_sg);
		}

		if (src_avail == 0) {
			if (src_nents == 0)
				break;

			src_sg = sg_next(src_sg);
			if (src_sg == NULL)
				break;

			src_nents--;
			src_avail = sg_dma_len(src_sg);
		}
	}

	if (flags & DMA_PREP_INTERRUPT) {
		/* Trigger interrupt after last block */
		prev->lli.ctllo |= DWC_CTLL_INT_EN;
		prev->lli.ctllo &= ~DWC_CTLL_LLP_D_EN;
		prev->lli.ctllo &= ~DWC_CTLL_LLP_S_EN;
	}

	prev->lli.llp = 0;
	first->total_len = total_len;
	first->direction = DMA_MEM_TO_MEM;
	first->request_line = ~0;

	return vchan_tx_prep(&dwvc->vc, &first->vd, flags);

err_desc_get:
	dwc_desc_put(dw, first);

	return NULL;
}

static struct dma_async_tx_descriptor *
dwc_prep_slave_sg(struct dma_chan *chan, struct scatterlist *sgl,
		unsigned int sg_len, enum dma_transfer_direction direction,
		unsigned long flags, void *context)
{
	struct dw_dma_vchan	*dwvc = to_dw_dma_vchan(chan);
	struct dw_dma		*dw = to_dw_dma(chan->device);
	struct dma_slave_config	*sconfig = &dwvc->dma_sconfig;
	struct dw_desc		*prev;
	struct dw_desc		*first;
	u32			ctllo;
	dma_addr_t		reg;
	unsigned int		reg_width;
	unsigned int		mem_width;
	unsigned int		data_width;
	unsigned int		i;
	struct scatterlist	*sg;
	size_t			total_len = 0;

	dev_vdbg(chan2dev(chan), "%s\n", __func__);

	if (unlikely(!is_slave_direction(direction) || !sg_len))
		return NULL;

	dwvc->dma_sconfig.direction = direction;

	prev = first = NULL;

	switch (direction) {
	case DMA_MEM_TO_DEV:
		reg_width = __fls(sconfig->dst_addr_width);
		reg = sconfig->dst_addr;
		ctllo = (DWC_DEFAULT_CTLLO(chan)
				| DWC_CTLL_DST_WIDTH(reg_width)
				| DWC_CTLL_DST_FIX
				| DWC_CTLL_SRC_INC);

		ctllo |= sconfig->device_fc ? DWC_CTLL_FC(DW_DMA_FC_P_M2P) :
			DWC_CTLL_FC(DW_DMA_FC_D_M2P);

		data_width = dw->pdata->data_width[dw->src_master];

		for_each_sg(sgl, sg, sg_len, i) {
			struct dw_desc	*desc;
			u32		len, dlen, mem;

			mem = sg_dma_address(sg);
			len = sg_dma_len(sg);

			mem_width = min_t(unsigned int,
					  data_width, dwc_fast_fls(mem | len));

slave_sg_todev_fill_desc:
			desc = dwc_desc_get(dw);
			if (!desc) {
				dev_err(chan2dev(chan),
					"%s: get no desc\n", __func__);
				goto err_desc_get;
			}

			desc->lli.sar = mem;
			desc->lli.dar = reg;
			desc->lli.ctllo = ctllo | DWC_CTLL_SRC_WIDTH(mem_width);
			if ((len >> mem_width) > dw->block_size) {
				dlen = dw->block_size << mem_width;
				mem += dlen;
				len -= dlen;
			} else {
				dlen = len;
				len = 0;
			}

			desc->lli.ctlhi = dlen >> mem_width;
			desc->len = dlen;
			desc->pchan = NULL;
			desc->vchan = dwvc;

			if (!first) {
				first = desc;
			} else {
				prev->lli.llp = desc->vd.tx.phys;
				list_add_tail(&desc->desc_node,
						&first->tx_list);
			}
			prev = desc;
			total_len += dlen;

			if (len)
				goto slave_sg_todev_fill_desc;
		}
		break;
	case DMA_DEV_TO_MEM:
		reg_width = __fls(sconfig->src_addr_width);
		reg = sconfig->src_addr;
		ctllo = (DWC_DEFAULT_CTLLO(chan)
				| DWC_CTLL_SRC_WIDTH(reg_width)
				| DWC_CTLL_DST_INC
				| DWC_CTLL_SRC_FIX);

		ctllo |= sconfig->device_fc ? DWC_CTLL_FC(DW_DMA_FC_P_P2M) :
			DWC_CTLL_FC(DW_DMA_FC_D_P2M);

		data_width = dw->pdata->data_width[dw->dst_master];

		for_each_sg(sgl, sg, sg_len, i) {
			struct dw_desc	*desc;
			u32		len, dlen, mem;

			mem = sg_dma_address(sg);
			len = sg_dma_len(sg);

			mem_width = min_t(unsigned int,
					  data_width, dwc_fast_fls(mem | len));

slave_sg_fromdev_fill_desc:
			desc = dwc_desc_get(dw);
			if (!desc) {
				dev_err(chan2dev(chan),
					"%s: get no desc\n", __func__);
				goto err_desc_get;
			}

			desc->lli.sar = reg;
			desc->lli.dar = mem;
			desc->lli.ctllo = ctllo | DWC_CTLL_DST_WIDTH(mem_width);
			if ((len >> reg_width) > dw->block_size) {
				dlen = dw->block_size << reg_width;
				mem += dlen;
				len -= dlen;
			} else {
				dlen = len;
				len = 0;
			}
			desc->lli.ctlhi = dlen >> reg_width;
			desc->len = dlen;
			desc->pchan = NULL;
			desc->vchan = dwvc;

			if (!first) {
				first = desc;
			} else {
				prev->lli.llp = desc->vd.tx.phys;
				list_add_tail(&desc->desc_node,
						&first->tx_list);
			}
			prev = desc;
			total_len += dlen;

			if (len)
				goto slave_sg_fromdev_fill_desc;
		}
		break;
	default:
		return NULL;
	}

	if (flags & DMA_PREP_INTERRUPT)
		/* Trigger interrupt after last block */
		prev->lli.ctllo |= DWC_CTLL_INT_EN;

	prev->lli.llp = 0;
	first->total_len = total_len;
	first->direction = direction;
	first->request_line = sconfig->slave_id;

	return vchan_tx_prep(&dwvc->vc, &first->vd, flags);

err_desc_get:
	dwc_desc_put(dw, first);

	return NULL;
}

static struct dma_async_tx_descriptor
*dwc_prep_dma_cyclic(struct dma_chan *chan, dma_addr_t buf_addr,
		size_t buf_len, size_t period_len,
		enum dma_transfer_direction direction,
		unsigned long flags)
{
	struct dw_dma_vchan		*dwvc = to_dw_dma_vchan(chan);
	struct dw_dma *dw = to_dw_dma(chan->device);
	struct dma_slave_config		*sconfig = &dwvc->dma_sconfig;
	struct dw_desc			*desc;
	unsigned int			reg_width;
	unsigned int			periods;
	unsigned int			i;
	struct dw_desc			*prev;
	struct dw_desc			*first;

	if (dw->nollp) {
		dev_dbg(chan2dev(chan),
				"channel doesn't support LLP transfers\n");
		return NULL;
	}

	if (unlikely(!is_slave_direction(direction)))
		return NULL;

	dwvc->dma_sconfig.direction = direction;

	if (direction == DMA_MEM_TO_DEV)
		reg_width = __ffs(sconfig->dst_addr_width);
	else
		reg_width = __ffs(sconfig->src_addr_width);

	periods = buf_len / period_len;

	/* Check for too big/unaligned periods and unaligned DMA buffer. */
	if (period_len > (dw->block_size << reg_width))
		return NULL;
	if (unlikely(period_len & ((1 << reg_width) - 1)))
		return NULL;
	if (unlikely(buf_addr & ((1 << reg_width) - 1)))
		return NULL;

	prev = first = NULL;

	for (i = 0; i < periods; i++) {
		desc = dwc_desc_get(dw);
		if (!desc)
			goto err_desc_get;

		switch (direction) {
		case DMA_MEM_TO_DEV:
			desc->lli.dar = sconfig->dst_addr;
			desc->lli.sar = buf_addr + (period_len * i);
			desc->lli.ctllo = (DWC_DEFAULT_CTLLO(chan)
					| DWC_CTLL_DST_WIDTH(reg_width)
					| DWC_CTLL_SRC_WIDTH(reg_width)
					| DWC_CTLL_DST_FIX
					| DWC_CTLL_SRC_INC
					| DWC_CTLL_INT_EN);

			desc->lli.ctllo |= sconfig->device_fc ?
				DWC_CTLL_FC(DW_DMA_FC_P_M2P) :
				DWC_CTLL_FC(DW_DMA_FC_D_M2P);

			break;
		case DMA_DEV_TO_MEM:
			desc->lli.dar = buf_addr + (period_len * i);
			desc->lli.sar = sconfig->src_addr;
			desc->lli.ctllo = (DWC_DEFAULT_CTLLO(chan)
					| DWC_CTLL_SRC_WIDTH(reg_width)
					| DWC_CTLL_DST_WIDTH(reg_width)
					| DWC_CTLL_DST_INC
					| DWC_CTLL_SRC_FIX
					| DWC_CTLL_INT_EN);

			desc->lli.ctllo |= sconfig->device_fc ?
				DWC_CTLL_FC(DW_DMA_FC_P_P2M) :
				DWC_CTLL_FC(DW_DMA_FC_D_P2M);

			break;
		default:
			break;
		}

		desc->len = period_len;
		desc->pchan = NULL;
		desc->vchan = dwvc;
		desc->is_cyclic = true;

		if (!first) {
			first = desc;
		} else {
			prev->lli.llp = desc->vd.tx.phys;
			list_add_tail(&desc->desc_node,
					&first->tx_list);
		}
		prev = desc;
	}

	/* Let's make a cyclic list */
	prev->lli.llp = first->vd.tx.phys;
	first->total_len = buf_len;
	first->direction = direction;
	first->request_line = sconfig->slave_id;

	return vchan_tx_prep(&dwvc->vc, &first->vd, flags);
err_desc_get:
	dwc_desc_put(dw, first);

	return NULL;
}

/*
 * Fix sconfig's burst size according to dw_dmac. We need to convert them as:
 * 1 -> 0, 4 -> 1, 8 -> 2, 16 -> 3.
 *
 * NOTE: burst size 2 is not supported by controller.
 *
 * This can be done by finding least significant bit set: n & (n - 1)
 */
static inline void convert_burst(u32 *maxburst)
{
	if (*maxburst > 1)
		*maxburst = fls(*maxburst) - 2;
	else
		*maxburst = 0;
}

static int dwc_config(struct dma_chan *chan, struct dma_slave_config *sconfig)
{
	struct dw_dma_vchan *dwvc = to_dw_dma_vchan(chan);

	/* Check if chan will be configured for slave transfers */
	if (!is_slave_direction(sconfig->direction))
		return -EINVAL;

	memcpy(&dwvc->dma_sconfig, sconfig, sizeof(*sconfig));

	convert_burst(&dwvc->dma_sconfig.src_maxburst);
	convert_burst(&dwvc->dma_sconfig.dst_maxburst);

	return 0;
}

static int dwc_pause(struct dma_chan *chan)
{
	struct dw_dma_vchan *dwvc = to_dw_dma_vchan(chan);
	struct dw_dma *dw = to_dw_dma(chan->device);
	struct dw_dma_chan *dwc;
	unsigned long flags;
	int i;
	u32 cfglo;
	unsigned int count = 20;

	spin_lock_irqsave(&dw->lock, flags);
	dwvc->pause = 1;
	for (i = 0; i < dw->pdata->nr_channels; i++) {
		dwc = &dw->pchan[i];
		if (dwc->vdesc && dwc->vdesc->vchan &&
			dwc->vdesc->vchan->idx == dwvc->idx &&
			test_bit(DW_DMA_IS_CYCLIC, &dwc->flags)) {
			cfglo = channel_readl(dwc, CFG_LO);
			channel_writel(dwc, CFG_LO,
				cfglo | DWC_CFGL_CH_SUSP);
			while (!(channel_readl(dwc, CFG_LO) &
				DWC_CFGL_FIFO_EMPTY) && count--)
				udelay(2);
		}
	}
	spin_unlock_irqrestore(&dw->lock, flags);

	return 0;
}

static int dwc_resume(struct dma_chan *chan)
{
	struct dw_dma_vchan *dwvc = to_dw_dma_vchan(chan);
	struct dw_dma *dw = to_dw_dma(chan->device);
	struct dw_dma_chan *dwc;
	unsigned long flags;
	int i;
	u32 cfglo;

	spin_lock_irqsave(&dw->lock, flags);
	dwvc->pause = 0;
	for (i = 0; i < dw->pdata->nr_channels; i++) {
		dwc = &dw->pchan[i];
		if (dwc->vdesc && dwc->vdesc->vchan &&
			dwc->vdesc->vchan->idx == dwvc->idx &&
			test_bit(DW_DMA_IS_CYCLIC, &dwc->flags)) {
			cfglo = channel_readl(dwc, CFG_LO);
			channel_writel(dwc, CFG_LO,
				cfglo & ~DWC_CFGL_CH_SUSP);
		}
	}
	spin_unlock_irqrestore(&dw->lock, flags);

	return 0;
}

static int dwc_terminate_all(struct dma_chan *chan)
{
	struct dw_dma_vchan *dwvc = to_dw_dma_vchan(chan);
	struct dw_dma *dw = to_dw_dma(chan->device);
	struct dw_dma_chan *dwc;
	int i;
	LIST_HEAD(list);
	unsigned long flags;

	spin_lock_irqsave(&dw->lock, flags);

	for (i = 0; i < dw->pdata->nr_channels; i++) {
		dwc = &dw->pchan[i];
		if (dwc->vdesc && dwc->vdesc->vchan &&
			dwc->vdesc->vchan->idx == dwvc->idx &&
			test_bit(DW_DMA_IS_CYCLIC, &dwc->flags)) {
			spin_lock(&dwvc->vc.lock);
			list_add_tail(&dwc->vdesc->vd.node,
				&dwvc->vc.desc_completed);
			spin_unlock(&dwvc->vc.lock);
		}
	}

	spin_lock(&dwvc->vc.lock);
	vchan_get_all_descriptors(&dwvc->vc, &list);
	spin_unlock(&dwvc->vc.lock);
	vchan_dma_desc_free_list(&dwvc->vc, &list);
	dwvc->pause = 0;

	spin_unlock_irqrestore(&dw->lock, flags);

	return 0;
}

/* Returns how many bytes were already received from source */
static inline u32 dwc_get_sent(struct dw_dma_chan *dwc)
{
	u32 ctlhi = channel_readl(dwc, CTL_HI);
	u32 ctllo = channel_readl(dwc, CTL_LO);

	return (ctlhi & DWC_CTLH_BLOCK_TS_MASK) * (1 << (ctllo >> 4 & 7));
}

static inline u32 dwc_get_residue(struct dw_dma_chan *dwc)
{
	size_t bytes = 0;
	struct dw_desc *vdesc, *child;
	dma_addr_t llp;

	if (!dwc && !dwc->vdesc && !dwc->vdesc->vchan)
		return 0;

	llp = channel_readl(dwc, LLP);

	vdesc = dwc->vdesc;
	bytes = vdesc->total_len;

	if (vdesc->vd.tx.phys == llp)
		goto out_res;

	bytes -= vdesc->len;
	list_for_each_entry(child, &vdesc->tx_list, desc_node) {
		if (child->lli.llp == llp) {
			/* Currently in progress */
			bytes -= dwc_get_sent(dwc);
			goto out_res;
		}
		bytes -= child->len;
	}
out_res:
	return bytes;
}

static enum dma_status
dwc_tx_status(struct dma_chan *chan,
	      dma_cookie_t cookie,
	      struct dma_tx_state *txstate)
{
	enum dma_status	ret;
	unsigned long flags;
	struct dw_dma_vchan *dwvc = to_dw_dma_vchan(chan);
	struct dw_dma *dw = to_dw_dma(chan->device);
	struct dw_dma_chan *dwc;
	struct virt_dma_desc *vd;
	struct dw_desc *desc;
	int i;
	size_t bytes = 0;

	ret = dma_cookie_status(chan, cookie, txstate);
	if (ret != DMA_COMPLETE) {
		spin_lock_irqsave(&dwvc->vc.lock, flags);
		vd = vchan_find_desc(&dwvc->vc, cookie);
		spin_unlock_irqrestore(&dwvc->vc.lock, flags);
		if (vd) {
			desc = vd_to_dw_desc(vd);
			bytes = desc->total_len;
			goto out;
		}

		ret = DMA_COMPLETE;
		spin_lock_irqsave(&dw->lock, flags);
		for (i = 0; i < dw->pdata->nr_channels; i++) {
			dwc = &dw->pchan[i];
			if (dwc->vdesc && dwc->vdesc->vchan
				&& (dwc->vdesc->vchan->idx == dwvc->idx)
				&& (dwc->vdesc->vd.tx.cookie == cookie)) {
				ret = DMA_IN_PROGRESS;
				bytes = dwc_get_residue(dwc);
				break;
			}
		}
		spin_unlock_irqrestore(&dw->lock, flags);
	}
out:
	dma_set_residue(txstate, bytes);

	return ret;
}

static void dwc_issue_pending(struct dma_chan *chan)
{
	struct dw_dma_vchan *dwvc = to_dw_dma_vchan(chan);
	struct dw_dma *dw = to_dw_dma(chan->device);
	unsigned long flags;
	bool is_pending;

	spin_lock_irqsave(&dwvc->vc.lock, flags);
	is_pending = vchan_issue_pending(&dwvc->vc);
	spin_unlock_irqrestore(&dwvc->vc.lock, flags);

	if (is_pending)
		dw_scan_descriptors(dw);
}

static int dwc_alloc_chan_resources(struct dma_chan *chan)
{
	dev_vdbg(chan2dev(chan), "%s\n", __func__);

	dma_cookie_init(chan);

	return 1;
}

static void dwc_free_chan_resources(struct dma_chan *chan)
{
	struct dw_dma_vchan	*dwvc = to_dw_dma_vchan(chan);

	vchan_free_chan_resources(&dwvc->vc);

	dev_vdbg(chan2dev(chan), "%s: done\n", __func__);
}

/*----------------------------------------------------------------------*/

struct dw_dma_of_filter_args {
	struct dw_dma *dw;
	unsigned int req;
	unsigned int src;
	unsigned int dst;
};

static bool dw_dma_of_filter(struct dma_chan *chan, void *param)
{
	struct dw_dma_vchan *dwvc = to_dw_dma_vchan(chan);
	struct dw_dma_of_filter_args *fargs = param;
	struct dw_dma *dw = to_dw_dma(chan->device);

	/* Ensure the device matches our channel */
	if (chan->device != &fargs->dw->dma)
		return false;

	dwvc->dma_sconfig.slave_id = fargs->req;
	dw->src_master	= fargs->src;
	dw->dst_master	= fargs->dst;

	return true;
}

static struct dma_chan *dw_dma_of_xlate(struct of_phandle_args *dma_spec,
					struct of_dma *ofdma)
{
	struct dw_dma *dw = ofdma->of_dma_data;
	struct dw_dma_of_filter_args fargs = {
		.dw = dw,
	};
	dma_cap_mask_t cap;

	if (dma_spec->args_count != 3)
		return NULL;

	fargs.req = dma_spec->args[0];
	fargs.src = dma_spec->args[1];
	fargs.dst = dma_spec->args[2];

	if (WARN_ON(fargs.req >= RTS_DMA_MAX_NR_REQUESTS ||
		    fargs.src >= dw->pdata->nr_masters ||
		    fargs.dst >= dw->pdata->nr_masters))
		return NULL;

	dma_cap_zero(cap);
	dma_cap_set(DMA_SLAVE, cap);

	/* TODO: there should be a simpler way to do this */
	return dma_request_channel(cap, dw_dma_of_filter, &fargs);
}

static void dw_dma_off(struct dw_dma *dw)
{
	int i;

	dma_writel(dw, CFG, 0);

	channel_clear_bit(dw, MASK.XFER, dw->all_chan_mask);
	channel_clear_bit(dw, MASK.SRC_TRAN, dw->all_chan_mask);
	channel_clear_bit(dw, MASK.DST_TRAN, dw->all_chan_mask);
	channel_clear_bit(dw, MASK.ERROR, dw->all_chan_mask);

	while (dma_readl(dw, CFG) & DW_CFG_DMA_EN)
		cpu_relax();

	for (i = 0; i < dw->dma.chancnt; i++)
		dw->pchan[i].initialized = false;
}

#ifdef CONFIG_OF
static int rts_dma_parse_dt(struct platform_device *pdev,
		struct rts_dma_platform_data *pdata)
{
	struct device_node *np = pdev->dev.of_node;
	u32 tmp, arr[4];

	if (!np) {
		dev_err(&pdev->dev, "Missing DT data\n");
		return -EINVAL;
	}

	if (!pdata)
		return -EINVAL;

	if (of_property_read_u32(np, "dma-channels", &pdata->nr_channels))
		return -EINVAL;

	if (of_property_read_u32(np, "dma-vchannels", &pdata->nr_vchannels))
		pdata->nr_vchannels = RTS_DMA_MAX_NR_VCHANNELS;

	if (of_property_read_bool(np, "is_private"))
		pdata->is_private = true;

	if (!of_property_read_u32(np, "chan_priority", &tmp))
		pdata->chan_priority = tmp;

	if (!of_property_read_u32(np, "block_size", &tmp))
		pdata->block_size = tmp;

	if (!of_property_read_u32(np, "dma-masters", &tmp)) {
		if (tmp > 4)
			return -EINVAL;

		pdata->nr_masters = tmp;
	}

	if (!of_property_read_u32_array(np, "data-width", arr,
				pdata->nr_masters))
		for (tmp = 0; tmp < pdata->nr_masters; tmp++)
			pdata->data_width[tmp] = arr[tmp];

	return 0;
}
#else
static inline struct rts_dma_platform_data *
rts_dma_parse_dt(struct platform_device *pdev,
		struct rts_dma_platform_data *pdata)
{
	return NULL;
}
#endif

static int dw_probe(struct platform_device *pdev)
{
	struct rts_dma_platform_data *pdata = NULL;
	struct resource *io;
	struct dw_dma *dw;
	bool autocfg = 0;
	unsigned int rts_params;
	unsigned int max_blk_size = 0;
	int err;
	int i;

	dw = devm_kzalloc(&pdev->dev, sizeof(struct dw_dma), GFP_KERNEL);
	if (!dw)
		return -ENOMEM;

	pm_runtime_get_sync(&pdev->dev);

	io = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!io) {
		err = -EINVAL;
		goto err_pdata;
	}

	dw->irq = platform_get_irq(pdev, 0);
	if (dw->irq < 0) {
		err = dw->irq;
		goto err_pdata;
	}

	dw->regs = devm_ioremap_resource(&pdev->dev, io);
	if (IS_ERR(dw->regs)) {
		err = PTR_ERR(dw->regs);
		goto err_pdata;
	}

	err = dma_coerce_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (err)
		goto err_pdata;

	dw->pdata = devm_kzalloc(&pdev->dev, sizeof(*dw->pdata), GFP_KERNEL);
	if (!dw->pdata) {
		err = -ENOMEM;
		goto err_pdata;
	}

	pdata = dev_get_platdata(&pdev->dev);
	if (pdata) {
		memcpy(dw->pdata, pdata, sizeof(*dw->pdata));
	} else {
		err = rts_dma_parse_dt(pdev, dw->pdata);
		if (err) {
			rts_params = dma_readl(dw, DW_PARAMS);
			dev_dbg(&pdev->dev, "DW_PARAMS: 0x%08x\n", rts_params);

			autocfg = rts_params >> DW_PARAMS_EN & 1;
			if (!autocfg) {
				err = -EINVAL;
				goto err_pdata;
			}

			dw->pdata->nr_channels =
				(rts_params >> DW_PARAMS_NR_CHAN & 0x7) + 1;
			dw->pdata->nr_vchannels = RTS_DMA_MAX_NR_VCHANNELS;
			dw->pdata->nr_masters =
				(rts_params >> DW_PARAMS_NR_MASTER & 3) + 1;
			for (i = 0; i < dw->pdata->nr_masters; i++) {
				dw->pdata->data_width[i] = 4 <<
				(rts_params >> DW_PARAMS_DATA_WIDTH(i) & 3);
			}
			dw->pdata->block_size = dma_readl(dw, MAX_BLK_SIZE);

			/* Fill platform data with the default values */
			dw->pdata->is_private = true;
			dw->pdata->chan_priority = CHAN_PRIORITY_ASCENDING;
		}
	}
	pdata = dw->pdata;

	if (pdata->nr_channels > RTS_DMA_MAX_NR_CHANNELS) {
		err = -EINVAL;
		goto err_pdata;
	}

	if (of_device_is_compatible(pdev->dev.of_node,
				"realtek,rts3903-dmac")) {
		dw->clk = devm_clk_get(&pdev->dev, "dma_ck");
		if (IS_ERR(dw->clk)) {
			err = PTR_ERR(dw->clk);
			goto err_pdata;
		}

		err = clk_prepare_enable(dw->clk);
		if (err) {
			dev_err(&pdev->dev, "failed to start dma clk\n");
			goto err_pdata;
		}
	}

	pm_runtime_enable(&pdev->dev);

	dw->pchan = devm_kzalloc(&pdev->dev,
		pdata->nr_channels * sizeof(struct dw_dma_chan), GFP_KERNEL);
	if (!dw->pchan) {
		err = -ENOMEM;
		goto err_pdata;
	}

	dw->vchan = devm_kzalloc(&pdev->dev,
		pdata->nr_vchannels * sizeof(struct dw_dma_vchan), GFP_KERNEL);
	if (!dw->pchan) {
		err = -ENOMEM;
		goto err_pdata;
	}

	spin_lock_init(&dw->lock);
	INIT_LIST_HEAD(&dw->free_list);
	dw->vchan_index = 0;

	/* Calculate all channel mask before DMA setup */
	dw->all_chan_mask = (1 << pdata->nr_channels) - 1;

	/* Force dma off, just in case */
	dw_dma_off(dw);

	/* Disable BLOCK interrupts as well */
	channel_clear_bit(dw, MASK.BLOCK, dw->all_chan_mask);

	err = devm_request_irq(&pdev->dev, dw->irq, dw_dma_interrupt, 0,
			       "rts_dmac", dw);
	if (err)
		return err;

	/* Create a pool of consistent memory blocks for hardware descriptors */
	dw->desc_pool = dmam_pool_create("dw_dmac_desc_pool", &pdev->dev,
					 sizeof(struct dw_desc), 32, 0);
	if (!dw->desc_pool) {
		dev_err(&pdev->dev, "No memory for descriptors dma pool\n");
		return -ENOMEM;
	}

	tasklet_init(&dw->tasklet, dw_dma_tasklet, (unsigned long)dw);

	INIT_LIST_HEAD(&dw->dma.channels);
	for (i = 0; i < pdata->nr_vchannels; i++) {
		struct dw_dma_vchan *dwvc = &dw->vchan[i];

		dwvc->vc.desc_free = dwc_dma_free_desc;
		vchan_init(&dwvc->vc, &dw->dma);
		dwvc->idx = i;
		dwvc->dma_sconfig.slave_id = ~0;
		dwvc->pause = 0;
	}

	/*
	 * NOTE: some controllers may have additional features that we
	 * need to initialize here, like "scatter-gather" (which
	 * doesn't mean what you think it means), and status writeback.
	 */

	dw_set_masters(dw);
	dw->block_size = pdata->block_size;
	dw->nollp = 1;

	for (i = 0; i < pdata->nr_channels; i++) {
		struct dw_dma_chan *dwc = &dw->pchan[i];

		/* 7 is highest priority & 0 is lowest. */
		if (pdata->chan_priority == CHAN_PRIORITY_ASCENDING)
			dwc->priority = pdata->nr_channels - i - 1;
		else
			dwc->priority = i;

		dwc->idx = i;
		dwc->ch_regs = &__dw_regs(dw)->CHAN[i];
		dwc->mask = 1 << i;
		dwc->initialized = false;
		dwc->request_line = ~0;

		channel_clear_bit(dw, CH_EN, dwc->mask);

		dwc->direction = DMA_TRANS_NONE;

		/* Hardware configuration */
		if (autocfg) {
			unsigned int r = RTS_DMA_MAX_NR_CHANNELS - i - 1;
			void __iomem *addr = &__dw_regs(dw)->DWC_PARAMS[r];
			unsigned int dwc_params = dma_readl_native(addr);

			dev_dbg(&pdev->dev, "DWC_PARAMS[%d]: 0x%08x\n", i,
					    dwc_params);

			/*
			 * Decode maximum block size for given channel. The
			 * stored 4 bit value represents blocks from 0x00 for 3
			 * up to 0x0a for 4095.
			 */
			dwc->block_size =
				(4 << ((max_blk_size >> 4 * i) & 0xf)) - 1;
			dwc->nollp =
				(dwc_params >> DWC_PARAMS_MBLK_EN & 0x1) == 0;
		} else {
			dwc->block_size = pdata->block_size;

			if (of_device_is_compatible(pdev->dev.of_node,
				"realtek,rts3903-dmac")) {
				dwc->nollp = 0;
			} else if (of_device_is_compatible(pdev->dev.of_node,
				"realtek,rts3901-dmac")) {
				dwc->nollp = 1;
			} else {
				dev_err(&pdev->dev, "unknown device type\n");
				return -EINVAL;
			}
		}
		if ((dwc->block_size < dw->block_size) &&
				(dwc->block_size > 0))
			dw->block_size = dwc->block_size;
		if (dwc->nollp == 0)
			dw->nollp = 0;
		dwc->vdesc = NULL;
		list_add_tail(&dwc->node, &dw->free_list);
	}

	/* Clear all interrupts on all channels. */
	dma_writel(dw, CLEAR.XFER, dw->all_chan_mask);
	dma_writel(dw, CLEAR.BLOCK, dw->all_chan_mask);
	dma_writel(dw, CLEAR.SRC_TRAN, dw->all_chan_mask);
	dma_writel(dw, CLEAR.DST_TRAN, dw->all_chan_mask);
	dma_writel(dw, CLEAR.ERROR, dw->all_chan_mask);

	/* Set capabilities */
	dma_cap_set(DMA_MEMCPY, dw->dma.cap_mask);
	dma_cap_set(DMA_SG, dw->dma.cap_mask);
	dma_cap_set(DMA_SLAVE, dw->dma.cap_mask);
	dma_cap_set(DMA_CYCLIC, dw->dma.cap_mask);
	if (pdata->is_private)
		dma_cap_set(DMA_PRIVATE, dw->dma.cap_mask);
	dw->dma.dev = &pdev->dev;
	if (of_device_is_compatible(pdev->dev.of_node,
				"realtek,rts3903-dmac")) {
		dw->dma.copy_align = 0;
	} else if (of_device_is_compatible(pdev->dev.of_node,
				"realtek,rts3901-dmac")) {
		dw->dma.copy_align = 2;
	} else {
		dev_err(&pdev->dev, "unknown device type\n");
		return -EINVAL;
	}
	dw->dma.device_alloc_chan_resources = dwc_alloc_chan_resources;
	dw->dma.device_free_chan_resources = dwc_free_chan_resources;

	dw->dma.device_prep_dma_memcpy = dwc_prep_dma_memcpy;
	dw->dma.device_prep_dma_sg = dwc_prep_dma_sg;

	dw->dma.device_prep_slave_sg = dwc_prep_slave_sg;
	dw->dma.device_prep_dma_cyclic = dwc_prep_dma_cyclic;

	dw->dma.device_config = dwc_config;
	dw->dma.device_pause = dwc_pause;
	dw->dma.device_resume = dwc_resume;
	dw->dma.device_terminate_all = dwc_terminate_all;

	dw->dma.device_tx_status = dwc_tx_status;
	dw->dma.device_issue_pending = dwc_issue_pending;

	/* DMA capabilities */
	dw->dma.src_addr_widths = RTS_DMA_BUSWIDTHS;
	dw->dma.dst_addr_widths = RTS_DMA_BUSWIDTHS;
	dw->dma.directions = BIT(DMA_DEV_TO_MEM) | BIT(DMA_MEM_TO_DEV) |
			     BIT(DMA_MEM_TO_MEM);
	dw->dma.residue_granularity = DMA_RESIDUE_GRANULARITY_BURST;

	dma_writel(dw, CFG, DW_CFG_DMA_EN);

	dev_info(&pdev->dev, "DesignWare DMA Controller, %d channels\n",
		dw->pdata->nr_channels);

	dma_async_device_register(&dw->dma);

	pm_runtime_put_sync_suspend(&pdev->dev);

	platform_set_drvdata(pdev, dw);

	if (pdev->dev.of_node) {
		err = of_dma_controller_register(pdev->dev.of_node,
						 dw_dma_of_xlate, dw);
		if (err)
			dev_err(&pdev->dev,
				"could not register of_dma_controller\n");
	}

	pr_info("INFO: realtek DMA engine inited\n");

	return 0;

err_pdata:

	kfree(dw->pchan);
	dw->pchan = NULL;

	kfree(dw->vchan);
	dw->vchan = NULL;

	pm_runtime_put_sync_suspend(&pdev->dev);

	kfree(dw->pdata);
	dw->pdata = NULL;

	kfree(dw);
	dw = NULL;

	return err;
}

static int dw_remove(struct platform_device *pdev)
{
	struct dw_dma *dw = platform_get_drvdata(pdev);
	struct dw_dma_chan *dwc;
	struct dw_dma_vchan *dwvc;
	int i;

	if (pdev->dev.of_node)
		of_dma_controller_free(pdev->dev.of_node);

	pm_runtime_get_sync(&pdev->dev);

	dw_dma_off(dw);
	dma_async_device_unregister(&dw->dma);

	free_irq(dw->irq, dw);
	tasklet_kill(&dw->tasklet);

	for (i = 0; i < dw->pdata->nr_vchannels; i++) {
		dwvc = &dw->vchan[i];
		list_del(&dwvc->vc.chan.device_node);
	}

	for (i = 0; i < dw->pdata->nr_channels; i++) {
		dwc = &dw->pchan[i];
		channel_clear_bit(dw, CH_EN, dwc->mask);
	}

	pm_runtime_put_sync_suspend(&pdev->dev);

	pm_runtime_disable(&pdev->dev);
	clk_disable_unprepare(dw->clk);

	return 0;
}

static void dw_shutdown(struct platform_device *pdev)
{
	struct dw_dma *dw = platform_get_drvdata(pdev);

	pm_runtime_get_sync(&pdev->dev);
	dw_dma_off(dw);
	pm_runtime_put_sync_suspend(&pdev->dev);

	clk_disable_unprepare(dw->clk);
}

#ifdef CONFIG_OF
static const struct of_device_id rts_dma_of_id_table[] = {
	{ .compatible = "realtek,rts-dmac" },
	{ .compatible = "realtek,rts3901-dmac" },
	{ .compatible = "realtek,rts3903-dmac" },
	{}
};
MODULE_DEVICE_TABLE(of, rts_dma_of_id_table);
#endif

#ifdef CONFIG_PM_SLEEP

static int dw_suspend_late(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dw_dma *dw = platform_get_drvdata(pdev);

	dw_dma_off(dw);
	clk_disable_unprepare(dw->clk);

	return 0;
}

static int dw_resume_early(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dw_dma *dw = platform_get_drvdata(pdev);

	clk_prepare_enable(dw->clk);
	dma_writel(dw, CFG, DW_CFG_DMA_EN);

	return 0;
}

#endif /* CONFIG_PM_SLEEP */

static const struct dev_pm_ops dw_dev_pm_ops = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(dw_suspend_late, dw_resume_early)
};

static struct platform_driver dw_driver = {
	.probe		= dw_probe,
	.remove		= dw_remove,
	.shutdown	= dw_shutdown,
	.driver = {
		.name	= "rts-dmac",
		.pm	= &dw_dev_pm_ops,
		.of_match_table = of_match_ptr(rts_dma_of_id_table),
	},
};

struct rts_dma_done {
	bool done;
	wait_queue_head_t *wait;
};

static void rts_dma_callback(void *arg)
{
	struct rts_dma_done *done = arg;

	done->done = true;
	wake_up_all(done->wait);
}

int rts_dma_copy(dma_addr_t dst, dma_addr_t src, size_t len)
{
	DECLARE_WAIT_QUEUE_HEAD_ONSTACK(done_wait);
	struct rts_dma_done done = {
		.wait = &done_wait,
	};
	struct dma_chan *chan;
	struct dma_device *dev;
	dma_cookie_t cookie;
	dma_cap_mask_t mask;
	struct dma_async_tx_descriptor *tx = NULL;
	enum dma_ctrl_flags flags;
	enum dma_status status;

	dma_cap_zero(mask);
	dma_cap_set(DMA_MEMCPY, mask);
	chan = dma_request_channel(mask, NULL, NULL);
	if (!chan) {
		pr_err("no dma channel\n");
		return -ENODEV;
	}

	dev = chan->device;
	flags = DMA_CTRL_ACK | DMA_PREP_INTERRUPT;

	tx = dev->device_prep_dma_memcpy(chan, dst, src, len,
		flags);
	if (tx == NULL) {
		pr_err("prep memcpy failed, len:%X\n", len);
		goto release;
	}

	tx->callback = rts_dma_callback;
	tx->callback_param = &done;
	cookie = tx->tx_submit(tx);
	if (dma_submit_error(cookie)) {
		pr_err("dma submit failed\n");
		goto release;
	}

	dma_async_issue_pending(chan);
	wait_event_freezable_timeout(done_wait, done.done,
		msecs_to_jiffies(10000));
	status = dma_async_is_tx_complete(chan, cookie, NULL, NULL);

	if (!done.done) {
		pr_err("dma transfer never finish\n");
		goto release;
	}
	if (status != DMA_COMPLETE) {
		pr_err("dma transfer failed\n");
		goto release;
	}

	dma_release_channel(chan);
	return len;

release:
	dma_release_channel(chan);
	return -EIO;
}

int rts_dma_sg_copy(struct scatterlist *dst_sg, unsigned int dst_nents,
		struct scatterlist *src_sg, unsigned int src_nents)
{
	DECLARE_WAIT_QUEUE_HEAD_ONSTACK(done_wait);
	struct rts_dma_done done = {
		.wait = &done_wait,
	};
	struct dma_chan *chan;
	struct dma_device *dev;
	dma_cookie_t cookie;
	dma_cap_mask_t mask;
	struct dma_async_tx_descriptor *tx = NULL;
	enum dma_ctrl_flags flags;
	enum dma_status status;

	dma_cap_zero(mask);
	dma_cap_set(DMA_SG, mask);
	chan = dma_request_channel(mask, NULL, NULL);
	if (!chan) {
		pr_err("no dma channel\n");
		return -ENODEV;
	}

	dev = chan->device;
	flags = DMA_CTRL_ACK | DMA_PREP_INTERRUPT;

	tx = dev->device_prep_dma_sg(chan, dst_sg, dst_nents,
			src_sg, src_nents, flags);
	if (tx == NULL) {
		pr_err("prep dma sg failed\n");
		goto release;
	}

	tx->callback = rts_dma_callback;
	tx->callback_param = &done;
	cookie = tx->tx_submit(tx);
	if (dma_submit_error(cookie)) {
		pr_err("dma submit failed\n");
		goto release;
	}

	dma_async_issue_pending(chan);
	wait_event_freezable_timeout(done_wait, done.done,
		msecs_to_jiffies(10000));
	status = dma_async_is_tx_complete(chan, cookie, NULL, NULL);

	if (!done.done) {
		pr_err("dma transfer never finish\n");
		goto release;
	}
	if (status != DMA_COMPLETE) {
		pr_err("dma transfer failed\n");
		goto release;
	}

	dma_release_channel(chan);
	return 0;

release:
	dma_release_channel(chan);
	return -EIO;
}

static int __init dw_init(void)
{
	return platform_driver_register(&dw_driver);
}
subsys_initcall(dw_init);

static void __exit dw_exit(void)
{
	platform_driver_unregister(&dw_driver);
}
module_exit(dw_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Synopsys DesignWare DMA Controller driver");
MODULE_AUTHOR("Haavard Skinnemoen (Atmel)");
MODULE_AUTHOR("Viresh Kumar <viresh.linux@gmail.com>");
