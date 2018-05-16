/*
 * Copyright (c) 2017 CETITEC GmbH
 *
 * This software is licensed under the terms of the GNU General Public License
 * (GPL) Version 2.
 *
 * The terms of the license can be found in the main directory of the delivery:
 *
 * GPL2 License: LICENSE-GPL2.txt
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/swab.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/kthread.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <asm/unaligned.h>
#include <linux/version.h>
#include <linux/ratelimit.h>
#include <linux/skbuff.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/if_vlan.h>
#include <linux/vmalloc.h>
#include <linux/version.h>
#include <linux/of_mdio.h>

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#include "sja1105.h"

#define SJA1105_SPI_MAX_CHARDEVS 2

#define SJA1105_SPI_SPEED_HZ 500000

#define SJA1105_SPI_BUFSIZE (PAGE_SIZE / 2)

#if SJA1105_SPI_BUFSIZE > (PAGE_SIZE / 2)
#error Unsupported buffer size.
#endif

struct sja1105_phy {
	struct net_device netdev;
	struct phy_device *phydev;
	struct sja1105_spi *sja;
	unsigned port, last_speed;
};

// Dummy netdev_ops for dummy netdevices for phy_connect...
static const struct net_device_ops sja1105_ndo = {};

static int sja1105_spi_major;
static struct cdev *sja1105_spi_chardev;
static struct class *sja1105_spi_class;

static DEFINE_MUTEX(minor_lock);
static struct sja1105_spi *sja_by_minor[SJA1105_SPI_MAX_CHARDEVS];

#define SPI_MODE_MASK (SPI_CPHA | SPI_CPOL | SPI_CS_HIGH          \
                       | SPI_LSB_FIRST | SPI_3WIRE | SPI_LOOP     \
                       | SPI_NO_CS | SPI_READY | SPI_TX_DUAL      \
                       | SPI_TX_QUAD | SPI_RX_DUAL | SPI_RX_QUAD)

struct sja1105_transfer
{
	uint32_t addr;
	uint8_t count;
	bool read;
	bool write;
	uint32_t *rxdata; /* filled in by init_sja_message */
	uint32_t *txdata; /* filled in by init_sja_message */
};

static int sja1105_spi_probe(struct spi_device *spi);
static void sja1105_spi_shutdown(struct spi_device *spi);
static int sja1105_spi_remove(struct spi_device *spi);

static const struct of_device_id sja1105_dt_ids[] = {
	{ .compatible = "nxp,sja1105" },
	{ },
};
MODULE_DEVICE_TABLE(of, sja1105_dt_ids);

static struct spi_driver sja1105_spi_driver = {
	.driver = {
		.name = "SJA1105_SPI",
		.of_match_table = of_match_ptr(sja1105_dt_ids),
	},
	.probe = sja1105_spi_probe,
	.shutdown = sja1105_spi_shutdown,
	.remove = sja1105_spi_remove,
};

static int write_port_speed(struct sja1105_spi *sja, struct sja1105_phy *phy);
static bool port_is_up(struct sja1105_spi *sja, unsigned portnumber);

static int init_sja_message(struct sja1105_spi *sja,
                            struct sja1105_transfer xfers[],
                            size_t tcount,
                            struct spi_message *m,
                            struct spi_transfer *transfers)
{
	int i;
	size_t offset = 0;

	if (!m || !transfers) {
		return -EINVAL;
	}

	spi_message_init(m);

	if (sja->txdma) {
		m->is_dma_mapped = 1;
	}

	for (i = 0; i < tcount; i++) {
		struct sja1105_transfer *xfer = &xfers[i];
		struct spi_transfer *t = &transfers[i];
		uint32_t *txbuf32;

		if (xfer->count > 64) {
			pr_err("Transfer too large!\n");
			return -EINVAL;
		}

		t->len = (xfer->count + 1) * sizeof(uint32_t);
		t->cs_change = (i < tcount - 1) ? 1 : 0;
		t->tx_nbits = 0;
		t->rx_nbits = 0;
		t->bits_per_word = 32;
		t->delay_usecs = 0;
		t->speed_hz = 500000;

		if (xfer->read) {
			t->rx_buf = (void *)(sja->rxbuf + offset);
			if (sja->rxdma) {
				t->rx_dma = sja->rxdma + offset;
			}
			xfer->rxdata = ((uint32_t *)t->rx_buf) + 1;
		} else {
			t->rx_buf = NULL;
			t->rx_dma = 0;
		}

		t->tx_buf = (const void *)(sja->txbuf + offset);

		if (sja->txdma) {
			t->tx_dma = sja->txdma + offset;
		}

		txbuf32 = (uint32_t *)t->tx_buf;
		txbuf32[0] = SJA1105_SPI_CMD(xfer->write, xfer->count, xfer->addr);
		xfer->txdata = &txbuf32[1];

		spi_message_add_tail(t, m);
		offset += t->len;
	}

	return 0;
}

/* Wait for next clkval being read from polling thread. 
   The cb will then be called with a linked list of all list_entries passed via
   this function until just before the corresponding SPI read. */
void sja1105_spi_next_clkval_async(struct sja1105_spi *sja,
                                   struct list_head *list_entry,
                                   sja1105_spi_ts_completion cb, void *cb_ctx)
{
	unsigned long flags;

	spin_lock_irqsave(&sja->ts_complete_lock, flags);

	sja->ts_complete_cb = cb;
	sja->ts_complete_ctx = cb_ctx;

	if (list_entry) {
		list_add_tail(list_entry, &sja->ts_complete_list);
	} else {
		INIT_LIST_HEAD(&sja->ts_complete_list);
	}

	spin_unlock_irqrestore(&sja->ts_complete_lock, flags);
}
EXPORT_SYMBOL(sja1105_spi_next_clkval_async);


static int do_poll(struct sja1105_spi *sja)
{
	int status;
	struct spi_message m;

	uint32_t *clkbuf;
	uint32_t *tsbuf;
	uint64_t clkval;
	int i;
	unsigned tsreg;
	unsigned long flags;
	sja1105_spi_ts_completion cb;
	void *cb_ctx;
	LIST_HEAD(local_list);
	uint32_t *confstatbuf;
#ifdef READ_SWITCH_STATUS
	uint32_t *statusbuf;
	uint32_t *mac0buf;
	struct sja1105_transfer reads[5];
	struct spi_transfer t[5];
#else
	struct sja1105_transfer reads[3];
	struct spi_transfer t[3];
#endif

	/* Get all egress timestamps */
	reads[0].addr = SJA1105_EGRESS_TIMESTAMPS;
	reads[0].count = 10;
	reads[0].read = true;
	reads[0].write = false;

	/* Get current PTP clock time to recreate complete timestamps */
	reads[1].addr = SJA1105_PTPTSCLK;
	reads[1].count = 2;
	reads[1].read = true;
	reads[1].write = false;

	reads[2].addr = SJA1105_CONFSTATUS;
	reads[2].count = 1;
	reads[2].read = true;
	reads[2].write = false;

#ifdef READ_SWITCH_STATUS
	reads[3].addr = SJA1105_STATUS;
	reads[3].count = 0xd;
	reads[3].read = true;
	reads[3].write = false;

	reads[4].addr = 0x201;
	reads[4].count = 1;
	reads[4].read = true;
	reads[4].write = false;
#endif

	mutex_lock(&sja->buf_lock);
	status = init_sja_message(sja, reads, ARRAY_SIZE(reads), &m, t);

	if (status) {
		pr_err("Couldn't create message\n");
		goto done;
	}

	tsbuf = reads[0].rxdata;
	clkbuf = reads[1].rxdata;
	confstatbuf = reads[2].rxdata;
#ifdef READ_SWITCH_STATUS
	statusbuf = reads[3].rxdata;
	mac0buf = reads[4].rxdata;
#endif

	/* Grab list of packets waiting for clkval *before* doing the
	   SPI read. This ensures that the value is not read too early. */
	spin_lock_irqsave(&sja->ts_complete_lock, flags);
	if (!list_empty(&sja->ts_complete_list)) {
		INIT_LIST_HEAD(&local_list);
		list_splice_init(&sja->ts_complete_list, &local_list);
		cb = sja->ts_complete_cb;
		cb_ctx = sja->ts_complete_ctx;
		// Reset cb
		sja->ts_complete_cb = NULL;
	} else {
		cb = NULL;
	}

	spin_unlock_irqrestore(&sja->ts_complete_lock, flags);

	/* Do spi transfer */
	status = spi_sync(sja->dev, &m);

	if (status < 0) {
		pr_err("Couldn't send message (%d)\n", status);
		goto done;
	}

	clkval = ((uint64_t)clkbuf[0]) | (((uint64_t)clkbuf[1]) << 32);

	{
		static unsigned c = 0;
		if (c++ % 1000 == 0) {
			uint8_t *tmp = (uint8_t*) &clkbuf[0];
			pr_debug("clkval: %llu (%02x %02x %02x %02x)\n",
			         clkval, tmp[0], tmp[1], tmp[2], tmp[3]);
		}
	}

	if (confstatbuf[0] & (1<<31)) {
		/* A valid configuration was uploaded to the switch */
		if (!sja->switch_confd) {
			/* switch was previously unconfigured */
			dev_info(sja->chardev, "Switch becomes configured\n");

			/* Since reconfiguring resets the port configs, reconfigure
			   all dynamic ports with the current PHY link speed */
			for(i = 0;i < sja->num_phys;++i) {
				if (sja->phys[i].last_speed) {
					if (write_port_speed(sja, &sja->phys[i]) != 0) {
						dev_warn(&sja->phys[i].netdev.dev,
								 "Could not set port speed\n");
					}
				}
			}

			sja->switch_confd = true;
		}
	} else {
		/* No valid configuration */
		if (sja->switch_confd) {
			/* switch was previously configured */
			dev_info(sja->chardev, "Switch becomes unconfigured\n");

			sja->switch_confd = false;
		}
	}

#ifdef READ_SWITCH_STATUS
	{
		static unsigned c = 0;

		if (c++ % 1000 == 0) {
			for (i = 0; i < 0xd; i++) {
				uint32_t *s = &statusbuf[i];
				pr_info("!!! SWITCH STATUS addr %x: 0x%08X\n", i, *s);
			}

			if (0 != *mac0buf) {
				pr_info_ratelimited("!!! SWITCH MAC 0 STATUS: 0x%08X\n",
				                    *mac0buf);
			}
		}
	}
#endif

	for (i = 0; i < SWITCH_PORTS; i++) {
		for (tsreg = 0; tsreg < 2; tsreg++) {
			uint32_t *reg = &tsbuf[2 * i + tsreg];

			if (*reg & 0x1) {
				/* Do not lock ts (for now). Complete/wait acts as barriers;
				 * and a new timestamp will only be available after a new
				 * message has been sent.
				 * This means that it is currently illegal to prepare two
				 * timestamped messages on the same switch port without
				 * waiting for the first timestamp in between. */
				sja->egress_ts[i][tsreg].ts =
						sja1105_recreate_ts(clkval, (*reg) >> 8);
				complete(&sja->egress_ts[i][tsreg].received);
			}
		}
	}

	pr_debug_ratelimited("sja1105_spi: poll_complete; clkval=%llu\n", clkval);
	mutex_unlock(&sja->buf_lock);

	if (cb) {
		cb(&local_list, clkval, cb_ctx);
	} else if (!list_empty(&local_list)) {
		pr_warn_ratelimited("sja1105_spi: No callback although timestamps requested!\n");
	}
	return 0;

done:
	mutex_unlock(&sja->buf_lock);
	return status;
}

static int write_mgmt(struct sja1105_spi *sja, unsigned index,
                      uint8_t portnumber, bool timestamp, unsigned tsreg)
{
	struct sja1105_transfer write[1];
	int err;
	struct spi_message m;
	struct spi_transfer t[1];
	uint32_t *reg;
	const uint8_t portmask = 1 << portnumber;

	write[0].addr = SJA1105_L2_ADDR_LOOKUP_RECONF;
	write[0].count = 4;
	write[0].read = false;
	write[0].write = true;

	mutex_lock(&sja->buf_lock);
	err = init_sja_message(sja, write, ARRAY_SIZE(write), &m, t);

	if (err) {
		mutex_unlock(&sja->buf_lock);
		return err;
	}

	reg = write[0].txdata;

	reg[3] = SJA1105_L2_ADDR_LOOKUP_RECONF_VALID
	         | SJA1105_L2_ADDR_LOOKUP_RECONF_RDWRSET
	         | SJA1105_L2_ADDR_LOOKUP_RECONF_VALIDENT
	         | SJA1105_L2_ADDR_LOOKUP_RECONF_MGMTROUTE;

	/* tsreg: Timestamp register per port. Can be 0 or 1. */
	reg[2] = (timestamp ? (1 << 20) | ((tsreg & 0x1) << 21) : 0 )
	          | ((PTP_MAC_VAL >> 28) & 0xFFFFF);
	reg[1] = ((PTP_MAC_VAL & 0x0FFFFFFF) << 4)
	          | ((portmask & 0x1E) >> 1);
	reg[0] = ((portmask & 0x1) << 31)
	          | SJA1105_L2_ADDR_LOOKUP_RECONF_ENFPORT
	          | ((index & 0x3FF) << 20);

	pr_debug("sja1105 mgmt write tx: %08X %08X %08X %08X, index=%u\n",
	         reg[3], reg[2], reg[1], reg[0], index);

	err = spi_sync(sja->dev, &m);
	mutex_unlock(&sja->buf_lock);

	if (err) {
		pr_err("sja1105 mgmt write tx error\n");
		return err;
	}

	return 0;
}

static int wait_mgmt_valid(struct sja1105_spi *sja)
{
	struct sja1105_transfer read[1];
	int err;
	struct spi_message m;
	struct spi_transfer t[1];
	uint32_t *reg;
	int count;
	bool done;

	read[0].addr = SJA1105_L2_ADDR_LOOKUP_RECONF;
	read[0].count = 4;
	read[0].read = true;
	read[0].write = false;

	mutex_lock(&sja->buf_lock);

	for (count = 0, done = false;
	     !done && count < SWITCH_MGMT_READ_RETRIES;
	     count++)
	{
		err = init_sja_message(sja, read, ARRAY_SIZE(read), &m, t);
		if (err) {
			return err;
		}

		reg = read[0].rxdata;
		err = spi_sync(sja->dev, &m);

		if (err) {
			mutex_unlock(&sja->buf_lock);
			pr_err("sja1105 mgmt write rx error\n");
			return err;
		}

		pr_debug("sja1105 mgmt write rx: %08X %08X %08X %08X\n",
		         reg[3], reg[2], reg[1], reg[0]);

		if (0 == (reg[3] & SJA1105_L2_ADDR_LOOKUP_RECONF_VALID)) {
			done = true;
		}
	}

	mutex_unlock(&sja->buf_lock);

	if (!done) {
		pr_warn("sja1105 mgmt write not done!\n");
	}

	return 0;
}

#ifdef READ_MGMT_ROUTE_BEFORE_WRITE
static int read_mgmt(struct spi_device *spi, struct sja1105_mgmt_message *msg,
                     unsigned index)
{
	int err;
	int count;
	bool done;

	mutex_lock(&sja->buf_lock);

	msg->txbuf[3] = SJA1105_L2_ADDR_LOOKUP_RECONF_VALID |
		SJA1105_L2_ADDR_LOOKUP_RECONF_MGMTROUTE;

	msg->txbuf[2] = 0;
	msg->txbuf[1] = 0;
	msg->txbuf[0] = (index & 0x3FF) << 20;

	pr_debug("sja1105 mgmt read tx:	%08X %08X %08X %08X, index=%u\n",
	         reg[3], reg[2], reg[1], reg[0], index);

	err = spi_sync(spi, msg->msg_tx);
	if (err) {
		pr_err("sja1105 mgmt read tx error\n");
		return err;
	}

	for (count = 0, done = false; !done && count < SWITCH_MGMT_READ_RETRIES;
	     count++) {
		err = spi_sync(spi, msg->msg_rx);
		if (err) {
			pr_err("sja1105: mgmt read rx error\n");
			return err;
		}

		pr_debug("sja1105 mgmt read rx:	%08X %08X %08X %08X\n",
		         msg->rxbuf[3], msg->rxbuf[2], msg->rxbuf[1], msg->rxbuf[0]);
		if (0 == (msg->rxbuf[3] & SJA1105_L2_ADDR_LOOKUP_RECONF_VALID)) {
			done = true;
		}
	}

	mutex_unlock(&sja->buf_lock);

	if (!done) {
		pr_warn("sja1105 mgmt read not done!\n");
	}

	return 0;
}
#endif

int sja1105_spi_prepare_egress_ptp(struct sja1105_spi *sja, uint8_t portnumber,
                                   bool timestamp, unsigned tsreg)
{
	struct spi_device *spi;
	int err;

	if (!sja || portnumber >= SWITCH_PORTS) {
		return -EINVAL;
	}

	if (!port_is_up(sja, portnumber)) {
		return -ENETDOWN;
	}

	spin_lock_irq(&sja->spi_lock);
	spi = sja->dev;
	spin_unlock_irq(&sja->spi_lock);

	if (!spi) {
		return -ESHUTDOWN;
	}

#ifdef READ_MGMT_ROUTE_BEFORE_WRITE
	err = read_mgmt(spi, &sja->message_mgmt, sja->mgmt_index);
	if (err) {
		return err;
	}
#endif

	reinit_completion(&sja->egress_ts[portnumber][tsreg].received);
	err = write_mgmt(sja, sja->mgmt_index, portnumber, timestamp, tsreg);

	if (err) {
		return err;
	}

	err = wait_mgmt_valid(sja);

	return err;
}
EXPORT_SYMBOL(sja1105_spi_prepare_egress_ptp);

int sja1105_spi_wait_egress_ts(struct sja1105_spi *sja, uint8_t portnumber,
                               uint64_t *timestamp, int timeout_ms, unsigned tsreg)
{
	int err;
	bool closed;

	if (portnumber > SWITCH_PORTS-1 || !timestamp || !sja) {
		return -EINVAL;
	}

	if (!port_is_up(sja, portnumber)) {
		return -ENETDOWN;
	}

	spin_lock_irq(&sja->spi_lock);
	closed = (sja->dev == NULL);
	spin_unlock_irq(&sja->spi_lock);

	if (closed) {
		return -ESHUTDOWN;
	}

	if (timeout_ms < 0) {
		err = wait_for_completion_killable(
					&sja->egress_ts[portnumber][tsreg].received);
		if (err < 0) {
			return -EINTR;
		}
	} else {
		err = wait_for_completion_killable_timeout(
					&sja->egress_ts[portnumber][tsreg].received,
					msecs_to_jiffies(timeout_ms));
		if (err <= 0) {
			return -EINTR;
		}
	}

	*timestamp = sja->egress_ts[portnumber][tsreg].ts;

	return 0;
}
EXPORT_SYMBOL(sja1105_spi_wait_egress_ts);

static int thread_poll(void *data)
{
	struct sja1105_spi *sja = (struct sja1105_spi *)data;
	struct spi_device *spi;
	int err;

	spin_lock_irq(&sja->spi_lock);
	spi = spi_dev_get(sja->dev);
	spin_unlock_irq(&sja->spi_lock);

	while (!kthread_should_stop()) {
		if (sja->poll_config.poll_enable) {
			err = do_poll(sja);
			if (err) {
				pr_err_ratelimited("sja1105_spi: Could not poll switch.\n");
			}
		} else {
			sja->switch_confd = false;
		}

		schedule_timeout_interruptible(1);
	}

	spi_dev_put(spi);

	return 0;
}

static int spidev_message(struct sja1105_spi *sja,
                          struct spi_ioc_transfer *u_xfers,
                          unsigned n_xfers)
{
	struct spi_device *spi = sja->dev;
	struct spi_message	msg;
	struct spi_transfer	*k_xfers;
	struct spi_transfer	*k_tmp;
	struct spi_ioc_transfer *u_tmp;
	unsigned n, total, tx_total, rx_total;
	u8 *tx_buf, *rx_buf;
	int status = -EFAULT;

	spi_message_init(&msg);
	k_xfers = kcalloc(n_xfers, sizeof(*k_tmp), GFP_KERNEL);

	if (k_xfers == NULL) {
		return -ENOMEM;
	}

	mutex_lock(&sja->buf_lock);

	/* Construct spi_message, copying any tx data to bounce buffer.
	 * We walk the array of user-provided transfers, using each one
	 * to initialize a kernel version of the same transfer.
	 */
	tx_buf = sja->txbuf;
	rx_buf = sja->rxbuf;

	total = 0;
	tx_total = 0;
	rx_total = 0;

	for (n = n_xfers, k_tmp = k_xfers, u_tmp = u_xfers;
	     n;
	     n--, k_tmp++, u_tmp++)
	{
		k_tmp->len = u_tmp->len;

		total += k_tmp->len;
		/* Since the function returns the total length of transfers
		 * on success, restrict the total to positive int values to
		 * avoid the return value looking like an error.  Also check
		 * each transfer length to avoid arithmetic overflow.
		 */
		if (total > INT_MAX || k_tmp->len > INT_MAX) {
			status = -EMSGSIZE;
			goto done;
		}

		if (u_tmp->rx_buf) {
			/* This transfer needs space in RX bounce buffer */
			rx_total += k_tmp->len;
			if (rx_total > SJA1105_SPI_BUFSIZE) {
				status = -EMSGSIZE;
				goto done;
			}
			k_tmp->rx_buf = rx_buf;
			if (!access_ok(VERIFY_WRITE,
			               (u8 __user *) (uintptr_t) u_tmp->rx_buf,
			               u_tmp->len)) {
				goto done;
			}
			rx_buf += k_tmp->len;
		}

		if (u_tmp->tx_buf) {
			/* This transfer needs space in TX bounce buffer */
			tx_total += k_tmp->len;
			if (tx_total > SJA1105_SPI_BUFSIZE) {
				status = -EMSGSIZE;
				goto done;
			}
			k_tmp->tx_buf = tx_buf;
			if (copy_from_user(tx_buf,
			                   (const u8 __user *) (uintptr_t) u_tmp->tx_buf,
			                   u_tmp->len)) {
				goto done;
			}
			tx_buf += k_tmp->len;
		}

		k_tmp->cs_change = !!u_tmp->cs_change;
		k_tmp->tx_nbits = u_tmp->tx_nbits;
		k_tmp->rx_nbits = u_tmp->rx_nbits;
		k_tmp->bits_per_word = u_tmp->bits_per_word;
		k_tmp->delay_usecs = u_tmp->delay_usecs;
		k_tmp->speed_hz = u_tmp->speed_hz;

		if (!k_tmp->speed_hz) {
			k_tmp->speed_hz = SJA1105_SPI_SPEED_HZ;
		}

		spi_message_add_tail(k_tmp, &msg);
	}

	status = spi_sync(spi, &msg);

	if (status < 0) {
		goto done;
	} else {
		status = msg.actual_length;
	}

	/* Copy any rx data out of bounce buffer */
	rx_buf = sja->rxbuf;

	for (n = n_xfers, u_tmp = u_xfers; n; n--, u_tmp++) {
		if (u_tmp->rx_buf) {
			if (__copy_to_user((u8 __user *)
					(uintptr_t) u_tmp->rx_buf, rx_buf,
					u_tmp->len)) {
				status = -EFAULT;
				goto done;
			}
			rx_buf += u_tmp->len;
		}
	}
	status = total;

done:
	mutex_unlock(&sja->buf_lock);
	kfree(k_xfers);
	return status;
}

static struct spi_ioc_transfer *
spidev_get_ioc_message(unsigned int cmd, struct spi_ioc_transfer __user *u_ioc,
                       unsigned *n_ioc)
{
	struct spi_ioc_transfer *ioc;
	u32 tmp;

	/* Check type, command number and direction */
	if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC
		|| _IOC_NR(cmd) != _IOC_NR(SPI_IOC_MESSAGE(0))
		|| _IOC_DIR(cmd) != _IOC_WRITE) {
		return ERR_PTR(-ENOTTY);
	}

	tmp = _IOC_SIZE(cmd);

	if ((tmp % sizeof(struct spi_ioc_transfer)) != 0) {
		return ERR_PTR(-EINVAL);
	}

	*n_ioc = tmp / sizeof(struct spi_ioc_transfer);

	if (*n_ioc == 0) {
		return NULL;
	}

	/* copy into scratch area */
	ioc = kmalloc(tmp, GFP_KERNEL);

	if (!ioc) {
		return ERR_PTR(-ENOMEM);
	}
	if (__copy_from_user(ioc, u_ioc, tmp)) {
		kfree(ioc);
		return ERR_PTR(-EFAULT);
	}

	return ioc;
}

static long sja1105_spi_ioctl(struct file *filp, unsigned int cmd,
                              unsigned long arg)
{
	struct sja1105_spi *sja;
	struct spi_device	*spi;
	long ret = 0;
	struct spi_ioc_transfer	*ioc;
	unsigned n_ioc;
	uint32_t tmp;
	struct sja1105_poll_config tmp_poll;

	sja = (struct sja1105_spi *)filp->private_data;

	/* Check type and command number */
	if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC) {
		return -ENOTTY;
	}

	spin_lock_irq(&sja->spi_lock);
	spi = spi_dev_get(sja->dev);
	spin_unlock_irq(&sja->spi_lock);

	if (spi == NULL) {
		return -ESHUTDOWN;
	}


	switch (cmd) {
	case SPI_IOC_RD_MODE:
		ret = __put_user(spi->mode & SPI_MODE_MASK, (__u8 __user *)arg);
		break;
	case SPI_IOC_RD_MODE32:
		ret = __put_user(spi->mode & SPI_MODE_MASK, (__u32 __user *)arg);
		break;
	case SPI_IOC_RD_LSB_FIRST:
		ret = __put_user((spi->mode & SPI_LSB_FIRST) ? 1 : 0,
		                 (__u8 __user *)arg);
		break;
	case SPI_IOC_RD_BITS_PER_WORD:
		ret = __put_user(spi->bits_per_word, (__u8 __user *)arg);
		break;
	case SPI_IOC_RD_MAX_SPEED_HZ:
		ret = __put_user(spi->max_speed_hz, (__u32 __user *)arg);
		break;
	case SPI_IOC_WR_MODE:
	case SPI_IOC_WR_MODE32:
		if (cmd == SPI_IOC_WR_MODE) {
			ret = __get_user(tmp, (u8 __user *)arg);
		} else {
			ret = __get_user(tmp, (u32 __user *)arg);
		}
		if (ret == 0) {
			if (tmp & ~SPI_MODE_MASK) {
				ret = -EINVAL;
			} else if (tmp != (spi->mode & SPI_MODE_MASK)) {
				pr_info("sja1105_spi: SPI_IOC_WR_MODE: 0x%x\n", tmp);
				ret = -EPERM;
			}
		}
		break;
	case SPI_IOC_WR_LSB_FIRST:
		ret = __get_user(tmp, (u8 __user *)arg);
		if (ret == 0) {
			if ((tmp & SPI_LSB_FIRST) != (spi->mode & SPI_LSB_FIRST)) {
				pr_info("sja1105_spi: SPI_IOC_WR_LSB_FIRST: 0x%x\n", tmp);
				ret = -EPERM;
			}
		}
		break;
	case SPI_IOC_WR_BITS_PER_WORD:
		ret = __get_user(tmp, (u8 __user *)arg);
		if (ret == 0) {
			if (tmp != spi->bits_per_word) {
				pr_info("sja1105_spi: SPI_IOC_WR_BITS_PER_WORD: 0x%x\n", tmp);
				ret = -EPERM;
			}
		}
		break;
	case SPI_IOC_WR_MAX_SPEED_HZ:
		ret = __get_user(tmp, (u32 __user *)arg);
		if (ret == 0) {
			if (tmp != spi->max_speed_hz) {
				pr_info("sja1105_spi: SPI_IOC_WR_MAX_SPEED_HZ 0x%x\n", tmp);
				ret = -EPERM;
			}
		}
		break;
	case SJA1105_IOC_POLL:
		ret = copy_from_user(&tmp_poll, (void*)arg, sizeof(tmp_poll));
		if (ret == 0) {
			sja->poll_config.poll_enable = tmp_poll.poll_enable;
			if (sja->poll_config.poll_enable) {
				memcpy(sja->poll_config.mac_config, tmp_poll.mac_config,
					   sizeof(tmp_poll.mac_config));
				dev_info(sja->chardev, "Poll enabled");
			} else {
				dev_info(sja->chardev, "Poll disabled");
			}
		}
		break;
	default:
		ioc = spidev_get_ioc_message(cmd, (struct spi_ioc_transfer __user *)arg,
		                             &n_ioc);
		if (IS_ERR(ioc)) {
			ret = PTR_ERR(ioc);
			break;
		}
		if (!ioc) {
			break;	/* n_ioc is also 0 */
		}

		pr_debug("sja1105_spi: SPI_IOC_MESSAGE\n");
		/* translate to spi_message, execute */
		ret = spidev_message(sja, ioc, n_ioc);
		kfree(ioc);
		break;
	}

	spi_dev_put(spi);

	return ret;
}

int sja1105_find_by_spi(unsigned bus, unsigned chip_select)
{
	int i;

	mutex_lock(&minor_lock);
	for (i = 0;i < SJA1105_SPI_MAX_CHARDEVS;++i) {
		struct spi_device *spi;
		if (sja_by_minor[i] == NULL) {
			continue;
		}

		spin_lock_irq(&sja_by_minor[i]->spi_lock);
		spi = spi_dev_get(sja_by_minor[i]->dev);
		spin_unlock_irq(&sja_by_minor[i]->spi_lock);

		if (spi->master->bus_num == bus &&
			spi->chip_select == chip_select) {
			spi_dev_put(spi);
			mutex_unlock(&minor_lock);
			return i;
		}
		spi_dev_put(spi);
	}

	mutex_unlock(&minor_lock);
	return -1;
}
EXPORT_SYMBOL(sja1105_find_by_spi);

struct sja1105_spi *sja1105_spi_get(int minor)
{
	struct sja1105_spi *sja;

	if (minor >= SJA1105_SPI_MAX_CHARDEVS) {
		return NULL;
	}

	mutex_lock(&minor_lock);

	sja = sja_by_minor[minor];

	if (sja) {
		sja->users++;
	}

	mutex_unlock(&minor_lock);

	return sja;
}
EXPORT_SYMBOL(sja1105_spi_get);

void sja1105_spi_put(struct sja1105_spi *sja)
{
	mutex_lock(&minor_lock);

	sja->users--;

	if (!sja->users) {
		bool dofree;

		spin_lock_irq(&sja->spi_lock);
		dofree = (sja->dev == NULL);
		spin_unlock_irq(&sja->spi_lock);

		if (dofree) {
			kfree(sja);
		}
	}

	mutex_unlock(&minor_lock);
}
EXPORT_SYMBOL(sja1105_spi_put);

static int sja1105_spi_open(struct inode *inode, struct file *filp)
{
	int minor = iminor(file_inode(filp));
	struct sja1105_spi *sja = sja1105_spi_get(minor);

	if (!sja) {
		return -ENODEV;
	}

	filp->private_data = sja;

	return 0;
}

static int sja1105_spi_release(struct inode *inode, struct file *filp)
{
	struct sja1105_spi *sja = filp->private_data;
	sja1105_spi_put(sja);
	return 0;
}

static struct file_operations sja1105_spi_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = sja1105_spi_ioctl,
	.open = sja1105_spi_open,
	.release = sja1105_spi_release,
};

static int write_port_speed(struct sja1105_spi *sja, struct sja1105_phy *phy)
{
	struct sja1105_transfer write[3];
	int err;
	struct spi_message m;
	struct spi_transfer t[3];
	uint32_t *reg, *rgmii_clk = NULL, *idiv = NULL;
	bool rgmii = phy->phydev->interface==PHY_INTERFACE_MODE_RGMII;

	write[0].addr = SJA1105_MAC_RECONF;
	write[0].count = 2;
	write[0].read = false;
	write[0].write = true;

	if (rgmii) {
		write[1].addr = SJA1105_CGU_MII0_RGMII_TX_CLK + phy->port*7;
		write[1].count = 1;
		write[1].read = false;
		write[1].write = true;

		write[2].addr = SJA1105_CGU_IDIV0 + phy->port;
		write[2].count = 1;
		write[2].read = false;
		write[2].write = true;
	}

	err = init_sja_message(sja, write, rgmii?3:1, &m, t);

	if (err) {
		return err;
	}

	reg = write[0].txdata;

	reg[1] = SJA1105_MAC_RECONF_VALID |
		(sja->poll_config.mac_config[phy->port] & 0x00ffffff) |
		(phy->last_speed << SJA1105_MAC_RECONF_SPEED_OFS) |
		(phy->port << SJA1105_MAC_RECONF_PORT_OFS);
	reg[0] = 0;

	if (rgmii) {
		rgmii_clk = write[1].txdata;
		idiv = write[2].txdata;

		switch(phy->last_speed) {
		case 3:
			/* 10Mbps:
			   clock source IDIVn
			*/
			rgmii_clk[0] = ((SJA1105_CLKSRC_IDIV0+phy->port)<<SJA1105_CGU_CLKSRC_OFS) |
				SJA1105_CGU_AUTOBLOCK;
			/* IDIVn:
			   clock source 25MHz, divide by 10 */
			idiv[0] = (SJA1105_CLKSRC_REF25<<SJA1105_CGU_CLKSRC_OFS) |
				SJA1105_CGU_AUTOBLOCK | (9<<SJA1105_CGU_IDIV_OFS);
			break;
		case 2:
			/* 100Mbps:
			   clock source IDIVn
			*/
			rgmii_clk[0] = ((SJA1105_CLKSRC_IDIV0+phy->port)<<SJA1105_CGU_CLKSRC_OFS) |
				SJA1105_CGU_AUTOBLOCK;
			/* IDIVn:
			   clock source 25MHz, divide by 1 */
			idiv[0] = (SJA1105_CLKSRC_REF25<<SJA1105_CGU_CLKSRC_OFS) |
				SJA1105_CGU_AUTOBLOCK;
			break;
		case 1:
			/* 1000Mbps:
			   clock source PLL0
			*/
			rgmii_clk[0] = (SJA1105_CLKSRC_PLL0<<SJA1105_CGU_CLKSRC_OFS) |
				SJA1105_CGU_AUTOBLOCK;
			/* IDIVn:
			   clock source 25MHz, disabled */
			idiv[0] = (SJA1105_CLKSRC_REF25<<SJA1105_CGU_CLKSRC_OFS) |
				SJA1105_CGU_PD;
			break;
		case 0:
		default:
			/* disabled:
			   clock source PLL0, disabled
			*/
			rgmii_clk[0] = (SJA1105_CLKSRC_PLL0<<SJA1105_CGU_CLKSRC_OFS) |
				SJA1105_CGU_PD;
			/* IDIVn:
			   clock source 25MHz, disabled */
			idiv[0] = (SJA1105_CLKSRC_REF25<<SJA1105_CGU_CLKSRC_OFS) |
				SJA1105_CGU_PD;
			break;
		}
	}

	dev_dbg(sja->chardev, "mac write tx: %08X %08X, port %hhu\n",
			reg[1], reg[0], phy->port);
	if (rgmii) {
		dev_dbg(sja->chardev, "cgu write tx: rgmii %08X idiv %08X, port %hhu\n",
			rgmii_clk[0], idiv[0], phy->port);
	}

	err = spi_sync(sja->dev, &m);

	if (err) {
		dev_err(sja->chardev, "mac write tx error\n");
		return err;
	}

	return 0;
}

static bool port_is_up(struct sja1105_spi *sja, unsigned portnumber)
{
	int i;

	for (i = 0;i < sja->num_phys;++i) {
		if (sja->phys[i].port == portnumber) {
			if (sja->phys[i].last_speed == 0) {
				return false;
			} else {
				return true;
			}
		}
	}

	/* Ports without dynamic speed default to on */
	return true;
}

static void sja1105_phy_handler(struct net_device *netdev)
{
	struct sja1105_phy *phy = container_of(netdev, struct sja1105_phy, netdev);
	struct sja1105_spi *sja = phy->sja;
	unsigned sja_speed = 0;

	if (phy->phydev->link) {
		switch (phy->phydev->speed) {
		case 10:
			sja_speed = 3;
			break;
		case 100:
			sja_speed = 2;
			break;
		case 1000:
			sja_speed = 1;
			break;
		default:
			dev_warn(&netdev->dev, "Unknown speed %d\n", phy->phydev->speed);
			break;
		}
	}

	dev_dbg(&netdev->dev, "speed %d\n", sja_speed);
	phy->last_speed = sja_speed;
	if (sja->switch_confd) {
		mutex_lock(&sja->buf_lock);
		if (write_port_speed(sja, phy) != 0) {
			dev_warn(&netdev->dev, "Could not set port speed\n");
		}
		mutex_unlock(&sja->buf_lock);
	}
}

struct sja_phy_setupdata {
	unsigned phyports[SWITCH_PORTS];
	struct phy_device *phydevs[SWITCH_PORTS];
};

/* Performs early setup of PHYs that can fail with EPROBE_DEFER and should take
   place before any other resources are allocated.
   Returns the number of PHYs configured, 0 if no PHYs are configured for this
   switch, negative error code on error.
*/
static int sja1105_phy_early_setup(struct spi_device *spi, struct sja_phy_setupdata *data)
{
	struct device_node *np = spi->dev.of_node;
	int phymodecount, phyhandlecount;
	struct of_phandle_args args;
	int i, err;

	/* Parse device tree to find PHYs */
	phymodecount = of_property_count_strings(np, "phy-modes");
	phyhandlecount = of_property_count_elems_of_size(np, "phy-handles", 2*sizeof(u32));
	if (phymodecount <= 0 || phymodecount > SWITCH_PORTS ||
		phyhandlecount != phymodecount) {
		// Don't print warning if both properties don't exist
		if (phymodecount != -EINVAL || phyhandlecount != -EINVAL) {
			dev_warn(&spi->dev, "Both phy-modes and phy-handle must be defined and of matching length\n");
		} else {
			dev_info(&spi->dev, "No PHYs defined for switch\n");
		}
		return 0;
	}

	for (i = 0;i < phymodecount;++i) {
		err = of_parse_phandle_with_fixed_args(np, "phy-handles", 1, i, &args);
		if (err != 0) {
			dev_err(&spi->dev, "Unable to get phy handle %d: %d\n", i, err);
			return err;
		}

		/* Try to find PHY device from devicetree handle. This can fail
		   depending on driver initialization order, so return EPROBE_DEFER
		   to retry later if the PHY device can't be found.
		*/
		data->phydevs[i] = of_phy_find_device(args.np);
		of_node_put(args.np);
		if (!data->phydevs[i]) {
			dev_info(&spi->dev, "Could not find PHY %d, deferring probe...\n", i);
			for (--i;i >= 0;--i) {
				put_device(&data->phydevs[i]->mdio.dev);
			}
			return -EPROBE_DEFER;
		}
		data->phyports[i] = args.args[0];
	}

	return phymodecount;
}

/* Perform full setup of PHYs and connect the sja1105_phy_handler to all PHYs.
   Return 0 on success, negative error code on error.
*/
static int sja1105_phy_late_setup(struct sja1105_spi *sja, struct sja_phy_setupdata *data)
{
	int i, j, err;
	struct device_node *np = sja->dev->dev.of_node;

	phy_interface_t phyiface;
	const char **phymodes;

	if (!sja->num_phys) {
		return 0;
	}

	phymodes = kcalloc(sja->num_phys, sizeof(char*), GFP_KERNEL);
	if (ZERO_OR_NULL_PTR(phymodes)) {
		return -ENOMEM;
	}

	sja->phys = kcalloc(sja->num_phys, sizeof(struct sja1105_phy), GFP_KERNEL);
	if (ZERO_OR_NULL_PTR(sja->phys)) {
		kfree(phymodes);
		return -ENOMEM;
	}

	err = of_property_read_string_array(np, "phy-modes", phymodes, sja->num_phys);
	if (err < 0) {
		dev_err(sja->chardev, "Unable to get phy modes: %d\n", err);
		kfree(phymodes);
		kfree(sja->phys);
		return err;
	}
	for (i = 0;i < sja->num_phys;++i) {
		/* Setup enough fields of the net_device so that the PHY code and
		   the netdev code called by it are happy.
		   This is _NOT_ a proper netdev and should not be used for anything
		   else
		*/
		memset(&sja->phys[i].netdev, 0, sizeof(struct net_device));
		sja->phys[i].netdev.reg_state = NETREG_DUMMY;
		device_initialize(&sja->phys[i].netdev.dev);
		sja->phys[i].netdev.dev.parent = sja->chardev;
		dev_set_name(&sja->phys[i].netdev.dev, "%s_port%d",
					 dev_name(sja->chardev), data->phyports[i]);
		sja->phys[i].netdev.netdev_ops = &sja1105_ndo;

		phyiface = PHY_INTERFACE_MODE_NA;
		for (j = 0; j < PHY_INTERFACE_MODE_MAX; j++) {
			if (!strcasecmp(phymodes[i], phy_modes(j))) {
				phyiface = j;
				break;
			}
		}
		if (phyiface != PHY_INTERFACE_MODE_MII &&
			phyiface != PHY_INTERFACE_MODE_RMII &&
			phyiface != PHY_INTERFACE_MODE_RGMII) {
			dev_err(sja->chardev, "Invalid PHY mode '%s'\n", phymodes[i]);
			for (--i;i >= 0;--i) {
				phy_disconnect(sja->phys[i].phydev);
			}
			kfree(phymodes);
		    kfree(sja->phys);
			return -EINVAL;
		}

		sja->phys[i].phydev = data->phydevs[i];

		err = phy_connect_direct(&sja->phys[i].netdev, sja->phys[i].phydev,
								 sja1105_phy_handler, phyiface);
		if (err != 0) {
			dev_err(sja->chardev, "Could not connect PHY: %d\n", err);
			for (--i;i >= 0;--i) {
				phy_disconnect(sja->phys[i].phydev);
			}
			kfree(phymodes);
		    kfree(sja->phys);
			return err;
		}
		sja->phys[i].port = data->phyports[i];
		sja->phys[i].sja = sja;
		phy_start(sja->phys[i].phydev);
	}
	kfree(phymodes);

	return 0;
}

/* Call this function to clean up PHY setup data if probe fails between
   sja1105_phy_early_setup and sja1105_phy_late_setup or if
   sja1105_phy_late_setup fails */
static void sja1105_phy_early_cleanup(unsigned num_phys, struct sja_phy_setupdata *data)
{
	int i;

	for (i = 0;i < num_phys;++i) {
		put_device(&data->phydevs[i]->mdio.dev);
	}
}

static void sja1105_phy_cleanup(struct sja1105_spi *sja)
{
	int i;

	for (i = 0;i < sja->num_phys;++i) {
		phy_disconnect(sja->phys[i].phydev);
		put_device(&sja->phys[i].phydev->mdio.dev);
	}
	kfree(sja->phys);
}

static int sja1105_spi_probe(struct spi_device *spi)
{
	struct sja1105_spi *sja;
	static int sja_count = 0;
	int err, num_phys;
	int i;
	unsigned tsreg;
	dev_t devt;
	struct sja_phy_setupdata phy_data;

	num_phys = sja1105_phy_early_setup(spi, &phy_data);
	if (num_phys < 0) {
		return num_phys;
	}

	sja = kzalloc(sizeof(*sja), GFP_KERNEL);
	if (!sja) {
		err = -ENOMEM;
		goto err_sjaalloc;
	}
	spi_set_drvdata(spi, sja);
	sja->dev = spi;
	sja->poll_config.poll_enable = true;
	for (i = 0;i < 5;++i) {
		sja->poll_config.mac_config[i] = SJA1105_MAC_RECONF_DEFAULT;
	}
	sja->num_phys = num_phys;

	/* If requested, allocate DMA buffers */
	spi->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	if (!spi->dev.dma_mask) {
		spi->dev.dma_mask = &spi->dev.coherent_dma_mask;
	}

	/*
	 * Minimum coherent DMA allocation is PAGE_SIZE, so allocate
	 * that much and share it between Tx and Rx DMA buffers.
	 */
	sja->rxbuf = dmam_alloc_coherent(&spi->dev, PAGE_SIZE, &sja->rxdma, GFP_DMA);

	if (sja->rxbuf) {
		pr_info("sja1105_spi: Using DMA coherent memory\n");
		sja->txbuf = (sja->rxbuf + (PAGE_SIZE / 2));
		sja->txdma = (dma_addr_t)(sja->rxdma + (PAGE_SIZE / 2));
	} else {
		pr_info("sja1105_spi: Using kmalloc memory\n");
		sja->rxbuf = devm_kzalloc(&spi->dev, SJA1105_SPI_BUFSIZE, GFP_KERNEL);
		if (!sja->rxbuf) {
			err = -ENOMEM;
			goto err_dev;
		}
		sja->txbuf = devm_kzalloc(&spi->dev, SJA1105_SPI_BUFSIZE, GFP_KERNEL);
		if (!sja->txbuf) {
			devm_kfree(&spi->dev, sja->rxbuf);
		    err = -ENOMEM;
			goto err_dev;
		}
	}

	spin_lock_init(&sja->spi_lock);

	{
		u32 save_mode = spi->mode;
		u8 save_bpw = spi->bits_per_word;
		u32 save_speed = spi->max_speed_hz;

		spi->mode = SPI_MODE_1; /* CPOL=0; CPHA=1 */
		spi->bits_per_word = 8;
		spi->max_speed_hz = SJA1105_SPI_SPEED_HZ;

		err = spi_setup(spi);
		if (err < 0) {
			spi->mode = save_mode;
			spi->bits_per_word = save_bpw;
			spi->max_speed_hz = save_speed;
			goto err_spi;
		}
	}

	for (i = 0; i < SWITCH_PORTS; i++) {
		for (tsreg = 0; tsreg < 2; tsreg++) {
			init_completion(&sja->egress_ts[i][tsreg].received);
		}
	}

	mutex_lock(&minor_lock);
	devt = MKDEV(sja1105_spi_major, sja_count);
	sja_by_minor[sja_count] = sja;
	sja->ts_complete_cb = NULL;
	spin_lock_init(&sja->ts_complete_lock);
	INIT_LIST_HEAD(&sja->ts_complete_list);
	sja->devnum = sja_count;
	mutex_init(&sja->buf_lock);
	sja->chardev = device_create(sja1105_spi_class, NULL, devt, NULL,
								 "sja1105_spi%d.%d", spi->master->bus_num,
								 spi->chip_select);
	if (IS_ERR(sja->chardev)) {
		err = PTR_ERR(sja->chardev);
		pr_err("sja1105_spi: Unable to create device %d\n", sja_count);
		goto err_dev;
	}

	sja->chardev->driver = &sja1105_spi_driver.driver;

	err = sja1105_phy_late_setup(sja, &phy_data);
	if (err != 0) {
		goto err_latephy;
	}
	num_phys = 0; // To skip sja1105_phy_early_cleanup if an error occurs after this point.

	sja->users = 1; /* poll thread uses it */
	sja->poller = kthread_run(thread_poll, sja, "sja1105_spi%d", sja_count);

	if (IS_ERR(sja->poller)) {
		err = PTR_ERR(sja->poller);
		goto err_poller;
	}

	sja->mgmt_index = SWITCH_MGMTROUTE_INDEX;
	sja_count++;
	mutex_unlock(&minor_lock);

	return 0;

  err_poller:
	mutex_destroy(&sja->buf_lock);
	sja1105_phy_cleanup(sja);
  err_latephy:
	device_destroy(sja1105_spi_class, devt);
  err_dev:
	mutex_unlock(&minor_lock);
  err_spi:
	kfree(sja);
  err_sjaalloc:
	if (num_phys > 0) {
		sja1105_phy_early_cleanup(num_phys, &phy_data);
	}

	return err;
}

static int sja1105_spi_remove(struct spi_device *spi)
{
	struct sja1105_spi *sja;
	dev_t devt;

	sja = spi_get_drvdata(spi);
	if (!sja) {
		return -ENXIO;
	}

	spi_set_drvdata(spi, NULL);
	devt = MKDEV(sja1105_spi_major, sja->devnum);

	/* stop thread and wait for it to exit */
	kthread_stop(sja->poller);

	sja1105_phy_cleanup(sja);

	mutex_destroy(&sja->buf_lock);
	/* make sure ops on existing fds can abort cleanly */
	spin_lock_irq(&sja->spi_lock);
	sja->dev = NULL;
	spin_unlock_irq(&sja->spi_lock);

	mutex_lock(&minor_lock);
	sja->users--; /* for poll thread */

	/* prevent new opens */
	sja_by_minor[sja->devnum] = NULL;
	device_destroy(sja1105_spi_class, devt);

	if (!sja->users) {
		kfree(sja);
	}

	mutex_unlock(&minor_lock);

	return 0;
}


static void sja1105_spi_shutdown(struct spi_device *spi)
{
	sja1105_spi_remove(spi);
}



static void destroy_chardev(void)
{
	class_destroy(sja1105_spi_class);
	unregister_chrdev(sja1105_spi_major, "sja1105_spi");
}

static int init_chardev(void)
{
	int err = 0, i;
	dev_t devt;

	err = alloc_chrdev_region(&devt, 0, SJA1105_SPI_MAX_CHARDEVS,
	                          "sja1105_spi");
	if (err) {
		pr_err("sja1105_spi: Unable to allocate chrdev_region: %d\n", err);
		goto out_err;
	}

	sja1105_spi_major = MAJOR(devt);
	sja1105_spi_chardev = cdev_alloc();

	if (!sja1105_spi_chardev) {
		err = -ENOMEM;
		goto out_unreg;
	}

	sja1105_spi_chardev->ops = &sja1105_spi_fops;
	sja1105_spi_chardev->owner = THIS_MODULE;
	err = cdev_add(sja1105_spi_chardev, devt, SJA1105_SPI_MAX_CHARDEVS);

	if (err) {
		pr_err("sja1105_spi: Unable to add chardev: %d\n", err);
		goto out_devdel;
	}

	sja1105_spi_class = class_create(THIS_MODULE, "sja1105_spi");

	if (IS_ERR(sja1105_spi_class)) {
		err = PTR_ERR(sja1105_spi_class);
		pr_err("sja1105_spi: Unable to create class; errno = %d\n", err);
		goto out_devdel;
	}

	for (i = 0;i < SJA1105_SPI_MAX_CHARDEVS;++i) {
		sja_by_minor[i] = NULL;
	}

	return 0;

out_devdel:
	cdev_del(sja1105_spi_chardev);
	sja1105_spi_chardev = NULL;
out_unreg:
	unregister_chrdev_region(devt, SJA1105_SPI_MAX_CHARDEVS);
out_err:
	return err;
}

static int __init sja1105_spi_init(void)
{
	int err;

	err = init_chardev();

	if (err) {
		return err;
	}

	err = spi_register_driver(&sja1105_spi_driver);

	if (err) {
		destroy_chardev();
		return err;
	}

	return 0;
}

static void __exit sja1105_spi_exit(void)
{
	spi_unregister_driver(&sja1105_spi_driver);
	destroy_chardev();
}

module_init(sja1105_spi_init);
module_exit(sja1105_spi_exit);

MODULE_AUTHOR("Cetitec GmbH");
MODULE_DESCRIPTION("NXP SJA1105 Driver");
MODULE_LICENSE("GPL");

/* Required to auto-load module by udev */
MODULE_ALIAS("spi:sja1105");
