/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej <itej89@github.com>
 *
 * Synopsys DesignWare AXI DMA controller (the JH7110 "dma1p" block) --
 * consumer interface.
 *
 * Only the cyclic slave path is implemented, because that is all the audio
 * blocks need: a ring of equal-sized periods walked forever, with a callback
 * at every period boundary. There is no memory-to-memory path and no
 * single-shot scatter-gather path here.
 *
 * Ordering note: a consumer must NOT call jh7110_axidma_get() from its own
 * attach routine. The DMAC is an ordinary simplebus child and may well attach
 * after its consumers, so acquire the channel lazily on first use instead.
 */

#ifndef _JH7110_AXIDMA_H_
#define	_JH7110_AXIDMA_H_

#define	JH7110_DMA_MEM_TO_DEV	1
#define	JH7110_DMA_DEV_TO_MEM	2

struct jh7110_dma_chan;

struct jh7110_dma_slave_config {
	bus_addr_t	dev_addr;	/* physical address of the FIFO */
	u_int		dev_width;	/* bytes per FIFO access: 1,2,4,8 */
	u_int		maxburst;	/* items per burst, rounded down */
	int		direction;	/* JH7110_DMA_{MEM_TO_DEV,DEV_TO_MEM} */
};

/*
 * Look up the channel named by the consumer node's dmas/dma-names properties
 * and bind a free hardware channel to it. Returns NULL if the DMAC has not
 * attached yet, which a consumer should treat as "try again later".
 */
struct jh7110_dma_chan *jh7110_axidma_get(device_t consumer, const char *name);
void	jh7110_axidma_put(struct jh7110_dma_chan *ch);

/*
 * Start a cyclic transfer over [buf_pa, buf_pa + buf_len), split into
 * buf_len / period_len equal periods. cb() is called once per period from the
 * interrupt thread, with the channel lock NOT held.
 *
 * buf_pa must be 4-byte aligned and buf_len an exact multiple of period_len.
 * The buffer is read by the DMAC directly out of DRAM, so the caller is
 * responsible for making its own CPU writes visible there -- on this SoC that
 * means going through the L2 bypass alias, not the cached mapping.
 */
int	jh7110_axidma_cyclic(struct jh7110_dma_chan *ch,
	    const struct jh7110_dma_slave_config *cfg,
	    bus_addr_t buf_pa, size_t buf_len, size_t period_len,
	    void (*cb)(void *), void *cbarg);

int	jh7110_axidma_stop(struct jh7110_dma_chan *ch);

/*
 * Byte offset into the cyclic buffer the engine has reached, rounded down to
 * a period boundary. Safe to call from any context.
 */
uint32_t jh7110_axidma_position(struct jh7110_dma_chan *ch);

#endif /* _JH7110_AXIDMA_H_ */
