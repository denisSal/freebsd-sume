/*-
 * Copyright (c) 2020 Denis Salopek
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#define DEFAULT_ETHER_ADDRESS		"\02SUME\00"
#define SUME_ETH_DEVICE_NAME		"nf"

#ifndef SUME_PORTS_MAX
#define SUME_PORTS_MAX			4
#endif

#define SUME_IOCTL_CMD_WRITE_REG        (SIOCGPRIVATE_0)
#define SUME_IOCTL_CMD_READ_REG         (SIOCGPRIVATE_1)

#define SUME_LOCK(adapter, flags)	\
	spin_lock_irqsave(&adapter->lock, flags);
#define SUME_UNLOCK(adapter, flags)	\
	spin_unlock_irqrestore(&adapter->lock, flags);

#define SUME_LOCK_RX(adapter, i, flags)
#define SUME_UNLOCK_RX(adapter, i, flags)
#define SUME_LOCK_TX(adapter, i, flags)
#define SUME_UNLOCK_TX(adapter, i, flags)

/* Currently SUME only uses two fixed channels for all port traffic and regs. */
#define SUME_RIFFA_CHANNEL_DATA(sp)	0
#define SUME_RIFFA_CHANNEL_REG(sp)	1       /* See description at top. */
#define SUME_RIFFA_CHANNELS(sp)		2

/* RIFFA constants. */
#define RIFFA_MAX_CHNLS			12
#define RIFFA_MAX_BUS_WIDTH_PARAM	4
#define RIFFA_SG_BUF_SIZE		(4*1024)
#define RIFFA_SG_ELEMS			200

/* RIFFA register offsets. */
#define RIFFA_RX_SG_LEN_REG_OFF		0x0
#define RIFFA_RX_SG_ADDR_LO_REG_OFF	0x1
#define RIFFA_RX_SG_ADDR_HI_REG_OFF	0x2
#define RIFFA_RX_LEN_REG_OFF		0x3
#define RIFFA_RX_OFFLAST_REG_OFF	0x4
#define RIFFA_TX_SG_LEN_REG_OFF		0x5
#define RIFFA_TX_SG_ADDR_LO_REG_OFF	0x6
#define RIFFA_TX_SG_ADDR_HI_REG_OFF	0x7
#define RIFFA_TX_LEN_REG_OFF		0x8
#define RIFFA_TX_OFFLAST_REG_OFF	0x9
#define RIFFA_INFO_REG_OFF		0xA
#define RIFFA_IRQ_REG0_OFF		0xB
#define RIFFA_IRQ_REG1_OFF		0xC
#define RIFFA_RX_TNFR_LEN_REG_OFF	0xD
#define RIFFA_TX_TNFR_LEN_REG_OFF	0xE

#define RIFFA_CHNL_REG(c, o)		((c << 4) + o)

/*
 * RIFFA state machine;
 * rather than using complex circular buffers for 1 transaction.
 */
#define SUME_RIFFA_CHAN_STATE_IDLE	0x01
#define SUME_RIFFA_CHAN_STATE_READY	0x02
#define SUME_RIFFA_CHAN_STATE_READ	0x04
#define SUME_RIFFA_CHAN_STATE_LEN	0x08

#define SUME_CHAN_STATE_RECOVERY_FLAG	0x80000000

/* Various bits and pieces. */
#define SUME_RIFFA_MAGIC		0xcafe

/* Accessor macros. */
#define	SUME_RIFFA_LAST(offlast)	((offlast) & 0x01)
#define	SUME_RIFFA_OFFSET(offlast)	\
    ((unsigned long long)((offlast) >> 1) << 2)
#define	SUME_RIFFA_LEN(len)		((unsigned long long)(len) << 2)

#define	SUME_RIFFA_SG_LO_ADDR(sg)	(sg_dma_address(sg) & 0xffffffff);
#define	SUME_RIFFA_SG_HI_ADDR(sg)	\
    ((sg_dma_address(sg) >> 32) & 0xffffffff);
#define	SUME_RIFFA_SG_LEN(sg)		(sg_dma_len(sg) >> 2)	/* Words. */

/* find where this is defined */
enum dma_data_direction {
	DMA_BIDIRECTIONAL = 0,
	DMA_TO_DEVICE = 1,
	DMA_FROM_DEVICE = 2,
	DMA_NONE = 3,
};

struct irq {
	struct resource *res;
	int rid;
	void *tag;
} __aligned(CACHE_LINE_SIZE);

struct riffa_chnl_dir {
	void				*buf_addr;	/* S/G addresses+len. */
	bus_addr_t			buf_hw_addr;	/* -- " -- mapped. */
	int				num_sg;
//#ifndef SUME_GLOBAL_LOCK
	//spinlock_t			lock;
//#endif
	//wait_queue_head_t		waitq;
	unsigned int			state;
	unsigned int			flags;
	unsigned int			offlast;
	unsigned int			len;		/* words */
	unsigned int			rtag;
	uint32_t			*bouncebuf;
	size_t				bouncebuf_len;

	bus_dma_tag_t			my_tag;
	bus_dmamap_t			my_map;

	struct mtx			send_sleep;
	struct mtx			recv_sleep;
	struct mtx			lock;
	int				event;
};

struct sume_ifreq {
	uint32_t        addr;
	uint32_t        val;
};

struct sume_port {
	struct sume_adapter	*adapter;
	struct ifnet		*netdev;
	unsigned int		port;
	unsigned int		port_up;
	unsigned int		msg_enable;
	unsigned int		riffa_channel;
	struct ifmedia		media;
};

struct sume_adapter {
	device_t		dev;
	int			rid;
	struct resource		*bar0_addr;
	bus_size_t		bar0_len;
        bus_space_tag_t		bt;
        bus_space_handle_t	bh;
	struct irq		irq;
	int			num_chnls;
	int			num_sg;
	int			sg_buf_size;
	volatile int		running;
	struct ifnet		*netdev[4];
	struct sume_port	port[4];
	struct mtx		lock;
	struct mtx		rcv_wait;
	struct mtx		wr_wait;

	struct riffa_chnl_dir	**recv;
	struct riffa_chnl_dir	**send;

	unsigned int		vect0;
	unsigned int		vect1;
};
