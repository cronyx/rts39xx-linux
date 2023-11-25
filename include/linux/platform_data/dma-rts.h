#ifndef _PLATFORM_DATA_DMA_RTS_H
#define _PLATFORM_DATA_DMA_RTS_H

#include <linux/device.h>

#define RTS_DMA_MAX_NR_MASTERS	4

/**
 * struct rts_dma_platform_data - Controller configuration parameters
 * @nr_channels: Number of channels supported by hardware (max 8)
 * @is_private: The device channels should be marked as private and not for
 *	by the general purpose DMA channel allocator.
 * @is_memcpy: The device channels do support memory-to-memory transfers.
 * @is_nollp: The device channels does not support multi block transfers.
 * @chan_priority: Set channel priority increasing from 0 to 7 or 7 to 0.
 * @block_size: Maximum block size supported by the controller
 * @nr_masters: Number of AHB masters supported by the controller
 * @data_width: Maximum data width supported by hardware per AHB master
 *		(in bytes, power of 2)
 */
struct rts_dma_platform_data {
	unsigned int	nr_channels;
	unsigned int	nr_vchannels;
	bool		is_private;
	bool		is_nollp;
#define CHAN_PRIORITY_ASCENDING		0	/* chan0 highest */
#define CHAN_PRIORITY_DESCENDING	1	/* chan7 highest */
	unsigned char	chan_priority;
	unsigned int	block_size;
	unsigned char	nr_masters;
	unsigned char	data_width[RTS_DMA_MAX_NR_MASTERS];
};

#endif /* _PLATFORM_DATA_DMA_RTS_H */
