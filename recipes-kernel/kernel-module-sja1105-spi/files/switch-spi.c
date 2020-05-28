/*
 * Copyright (c) 2017-2020 CETITEC GmbH
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

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 11, 0)
#include <uapi/linux/sched/types.h>
#endif

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#include "kthread_settings.h"
#include "sja1105.h"


#define SJA1105_SPI_MAX_CHARDEVS 2

#define SJA1105_SPI_SPEED_HZ 10000000

#define SJA1105_SPI_BUFSIZE (PAGE_SIZE / 2)

#if SJA1105_SPI_BUFSIZE > (PAGE_SIZE / 2)
#error Unsupported buffer size.
#endif

#define USER_BUF_LEN PAGE_SIZE
#define PHY_BUF_LEN ALIGN(7*8, ARCH_DMA_MINALIGN)
#define API_BUF_LEN ALIGN((SWITCH_PORTS*2+1)*8, ARCH_DMA_MINALIGN)
#define POLL_BUF_LEN ALIGN(21*8, ARCH_DMA_MINALIGN)

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

static atomic_t spi_sync_count = ATOMIC_INIT(0);

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

static int write_port_speed(struct sja1105_spi *sja, struct spi_device *spi, struct sja1105_phy *phy);
static bool port_is_up(struct sja1105_spi *sja, unsigned portnumber);

static int init_sja_message(struct sja1105_spi *sja,
                            struct sja1105_transfer xfers[],
                            size_t tcount,
                            struct spi_message *m,
                            struct spi_transfer *transfers,
							char *buffer, size_t bufferlen)
{
	int i;
	size_t offset = 0;

	if (!m || !transfers || !buffer) {
		return -EINVAL;
	}

	spi_message_init(m);

	for (i = 0; i < tcount; i++) {
		struct sja1105_transfer *xfer = &xfers[i];
		struct spi_transfer *t = &transfers[i];
		uint32_t *txbuf32;

		if (xfer->count > 64) {
			pr_err("Transfer too large!\n");
			return -EINVAL;
		}

		t->len = (xfer->count + 1) * sizeof(uint32_t);
		if (offset+2*t->len > bufferlen) {
			pr_err("Transfer size exceeds buffer!\n");
			return -EINVAL;
		}

		t->cs_change = (i < tcount - 1) ? 1 : 0;
		t->tx_nbits = 0;
		t->rx_nbits = 0;
		t->bits_per_word = 32;
		t->delay_usecs = 0;
		t->speed_hz = SJA1105_SPI_SPEED_HZ;

		if (xfer->read) {
			t->rx_buf = (void *)(buffer + offset);
			xfer->rxdata = ((uint32_t *)t->rx_buf) + 1;
		} else {
			t->rx_buf = NULL;
			t->rx_dma = 0;
		}

		offset += t->len;

		t->tx_buf = (const void *)(buffer + offset);

		txbuf32 = (uint32_t *)t->tx_buf;
		txbuf32[0] = SJA1105_SPI_CMD(xfer->write, xfer->count, xfer->addr);
		xfer->txdata = &txbuf32[1];

		spi_message_add_tail(t, m);
		offset += t->len;
	}

	return 0;
}

#define PTPTSCLK_IDX 0
#define CONFSTATUS_IDX 1
#define STATUS_IDX 2
#define MAC0IDX 3

static struct spi_device* sja1105_get_spi(struct sja1105_spi *sja)
{
	struct spi_device *spi;
	unsigned long flags;
	if (!sja)
		return NULL;

	spin_lock_irqsave(&sja->spi_lock, flags);
	spi = spi_dev_get(sja->dev);
	spin_unlock_irqrestore(&sja->spi_lock, flags);
	return spi;
}

static int do_poll(struct sja1105_spi *sja)
{
	int status;
	struct spi_device *spi = sja1105_get_spi(sja);
	struct spi_message m;

	uint32_t *clkbuf;
	uint64_t clkval, old_clkval;
	int i;
	uint32_t *confstatbuf;
#ifdef READ_SWITCH_STATUS
	uint32_t *statusbuf;
	uint32_t *mac0buf;
	struct sja1105_transfer reads[4];
	struct spi_transfer t[4];
#else
	struct sja1105_transfer reads[2];
	struct spi_transfer t[2];
#endif

	if (!spi) {
		return -ESHUTDOWN;
	}

	/* Get current PTP clock time to recreate complete timestamps */
	reads[PTPTSCLK_IDX].addr = SJA1105_PTPTSCLK;
	reads[PTPTSCLK_IDX].count = 2;
	reads[PTPTSCLK_IDX].read = true;
	reads[PTPTSCLK_IDX].write = false;

	reads[CONFSTATUS_IDX].addr = SJA1105_CONFSTATUS;
	reads[CONFSTATUS_IDX].count = 1;
	reads[CONFSTATUS_IDX].read = true;
	reads[CONFSTATUS_IDX].write = false;

#ifdef READ_SWITCH_STATUS
	reads[STATUS_IDX].addr = SJA1105_STATUS;
	reads[STATUS_IDX].count = 0xd;
	reads[STATUS_IDX].read = true;
	reads[STATUS_IDX].write = false;

	reads[MAC0IDX].addr = 0x201;
	reads[MAC0IDX].count = 1;
	reads[MAC0IDX].read = true;
	reads[MAC0IDX].write = false;
#endif

	status = init_sja_message(sja, reads, ARRAY_SIZE(reads), &m, t,
							  sja->poll_spi_buf, POLL_BUF_LEN);

	if (status) {
		pr_err("Couldn't create message\n");
		goto done;
	}

	clkbuf = reads[PTPTSCLK_IDX].rxdata;
	confstatbuf = reads[CONFSTATUS_IDX].rxdata;
#ifdef READ_SWITCH_STATUS
	statusbuf = reads[STATUS_IDX].rxdata;
	mac0buf = reads[MAC0IDX].rxdata;
#endif

	/* Do spi transfer */
	atomic_inc(&spi_sync_count);
	status = spi_sync(spi, &m);

	if (status < 0) {
		pr_err("Couldn't send message (%d)\n", status);
		goto done;
	}

	clkval = ((uint64_t)clkbuf[0]) | (((uint64_t)clkbuf[1]) << 32);
	old_clkval = xchg_relaxed(&sja->clkval, clkval);
	if (sja->switch_confd) {
		if (old_clkval > clkval) {
			pr_info("Switch clock wrapped or reset (old: %llu new: %llu)\n",
					old_clkval, clkval);
		} else if (clkval - old_clkval > (1<<23)-1) {
			pr_warn("Switch clock: Elapsed > half epoch (old: %llu new: %llu)\n",
					old_clkval, clkval);
		} else {
			smp_store_release(&sja->clkval_valid, true);
		}
	}

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
					mutex_lock(&sja->phy_lock);
					if (write_port_speed(sja, spi, &sja->phys[i]) != 0) {
						dev_warn(&sja->phys[i].netdev.dev,
								 "Could not set port speed\n");
					}
					mutex_unlock(&sja->phy_lock);
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

	pr_debug_ratelimited("sja1105_spi: poll_complete; clkval=%llu\n", clkval);

	return 0;

done:
	spi_dev_put(spi);
	return status;
}

static int write_mgmt(struct sja1105_spi *sja, struct spi_device *spi, unsigned index,
                      uint8_t portmask, bool timestamp, unsigned tsreg)
{
	struct sja1105_transfer write[1];
	int err;
	struct spi_message m;
	struct spi_transfer t[1];
	uint32_t *reg;

	write[0].addr = SJA1105_L2_ADDR_LOOKUP_RECONF;
	write[0].count = 4;
	write[0].read = false;
	write[0].write = true;

	err = init_sja_message(sja, write, ARRAY_SIZE(write), &m, t,
						   sja->api_spi_buf, API_BUF_LEN);

	if (err) {
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

	atomic_inc(&spi_sync_count);
	err = spi_sync(spi, &m);

	if (err) {
		pr_err("sja1105 mgmt write tx error\n");
		return err;
	}

	return 0;
}

static int wait_mgmt_valid(struct sja1105_spi *sja, struct spi_device *spi)
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

	for (count = 0, done = false;
	     !done && count < SWITCH_MGMT_READ_RETRIES;
	     count++)
	{
		err = init_sja_message(sja, read, ARRAY_SIZE(read), &m, t,
							   sja->api_spi_buf, API_BUF_LEN);
		if (err) {
			return err;
		}

		reg = read[0].rxdata;
		atomic_inc(&spi_sync_count);
		err = spi_sync(spi, &m);

		if (err) {
			pr_err("sja1105 mgmt write rx error\n");
			return err;
		}

		pr_debug("sja1105 mgmt write rx: %08X %08X %08X %08X\n",
		         reg[3], reg[2], reg[1], reg[0]);

		if (0 == (reg[3] & SJA1105_L2_ADDR_LOOKUP_RECONF_VALID)) {
			done = true;
		}
	}

	if (!done) {
		pr_warn("sja1105 mgmt write not done!\n");
	}

	return 0;
}

int sja1105_spi_set_ptp_state(struct sja1105_spi *sja, bool enable_ptp)
{
	if (!sja) {
		return -EINVAL;
	}

	mutex_lock(&sja->poll_lock);
	mutex_lock(&sja->api_lock);
	sja->enable_ptp = enable_ptp;
	if (!sja->enable_ptp) {
		WRITE_ONCE(sja->clkval_valid, false);
	}
	mutex_unlock(&sja->api_lock);
	mutex_unlock(&sja->poll_lock);

	return 0;
}
EXPORT_SYMBOL(sja1105_spi_set_ptp_state);

bool sja1105_spi_is_clkval_valid(struct sja1105_spi *sja)
{
	if (!sja) {
		return false;
	}

	return !!smp_load_acquire(&sja->clkval_valid);
}
EXPORT_SYMBOL(sja1105_spi_is_clkval_valid);

int sja1105_spi_prepare_egress_ptp(struct sja1105_spi *sja, uint8_t portnumber,
                                   bool timestamp, unsigned tsreg)
{
	struct spi_device *spi = sja1105_get_spi(sja);
	int err;

	if (!spi) {
		if (!sja)
			return -EINVAL;
		else
			return -ESHUTDOWN;
	}

	if (portnumber >= SWITCH_PORTS) {
		err = -EINVAL;
		goto out_spi;
	}

	if (!port_is_up(sja, portnumber)) {
		err = -ENETDOWN;
		goto out_spi;
	}

	mutex_lock(&sja->api_lock);
	if (!sja->enable_ptp) {
		err = -ENETDOWN;
		goto out_unlock;
	}

	err = wait_mgmt_valid(sja, spi);
	if (err) {
		goto out_unlock;
	}

	err = write_mgmt(sja, spi, sja->mgmt_index, 1<<portnumber, timestamp, tsreg);

	if (err) {
		goto out_unlock;
	}

	err = wait_mgmt_valid(sja, spi);

  out_unlock:
	mutex_unlock(&sja->api_lock);
  out_spi:
	spi_dev_put(spi);
	return err;
}
EXPORT_SYMBOL(sja1105_spi_prepare_egress_ptp);

int sja1105_spi_prepare_egress_ptp_multiport(struct sja1105_spi *sja,
											 uint8_t *portmap,
											 bool timestamp, unsigned tsreg)
{
	struct spi_device *spi = sja1105_get_spi(sja);
	int err, i;

	if (!spi) {
		if (!sja)
			return -EINVAL;
		else
			return -ESHUTDOWN;
	}

	if ((*portmap >> SWITCH_PORTS) != 0 || *portmap == 0) {
		err = -EINVAL;
		goto out_spi;
	}

	for (i = 0;i < SWITCH_PORTS;++i) {
		if (!port_is_up(sja, i)) {
			*portmap &= ~(1<<i);
		}
	}

	if (*portmap == 0) {
		err = -ENETDOWN;
		goto out_spi;
	}

	mutex_lock(&sja->api_lock);
	if (!sja->enable_ptp) {
		err = -ENETDOWN;
		goto out_unlock;
	}

	err = wait_mgmt_valid(sja, spi);
	if (err) {
		goto out_unlock;
	}

	err = write_mgmt(sja, spi, sja->mgmt_index, *portmap, timestamp, tsreg);
	if (err) {
		goto out_unlock;
	}

	err = wait_mgmt_valid(sja, spi);

  out_unlock:
	mutex_unlock(&sja->api_lock);
  out_spi:
	spi_dev_put(spi);
	return err;
}
EXPORT_SYMBOL(sja1105_spi_prepare_egress_ptp_multiport);

static int poll_tx_ts(struct sja1105_spi *sja, struct spi_device *spi,
					  uint8_t portmask, uint64_t *timestamp, int timeout_ms,
					  unsigned tsreg)
{
	uint32_t *tsbuf;
	struct sja1105_transfer reads[1];
	struct spi_message m;
	struct spi_transfer t[1];
	int status;
	unsigned long timeout_jiffies = jiffies + msecs_to_jiffies(timeout_ms) + 1;
	uint8_t orig_portmask = portmask;
	int i;

	reads[0].addr = SJA1105_EGRESS_TIMESTAMPS;
	reads[0].count = 2*SWITCH_PORTS;
	reads[0].read = true;
	reads[0].write = false;

	status = init_sja_message(sja, reads, ARRAY_SIZE(reads), &m, t,
							  sja->api_spi_buf, API_BUF_LEN);
	if (status) {
		pr_err("Couldn't create message\n");
		goto done;
	}

	tsbuf = reads[0].rxdata;

	pr_debug("switch-spi: Starting Tx poll at %llu\n", ktime_to_ns(ktime_get()));

	spi_bus_lock(spi->controller);
	do {
		/* Do spi transfer */
		atomic_inc(&spi_sync_count);
		status = spi_sync_locked(spi, &m);

		if (status < 0) {
			spi_bus_unlock(spi->controller);
			pr_err("Couldn't send message (%d)\n", status);
			goto done;
		}

		for (i = 0;i < SWITCH_PORTS;++i) {
			if ((portmask&(1<<i)) && (tsbuf[i*2+tsreg] & 0x1)) {
				timestamp[i] = tsbuf[i*2+tsreg]>>8;
				portmask &= ~(1<<i);
			}
		}
	} while (portmask != 0 && time_is_after_jiffies(timeout_jiffies));
	spi_bus_unlock(spi->controller);

	if (portmask != 0) {
		pr_warn("switch-spi: Could not get Tx timestamp: Timeout\n");
		status = -ETIMEDOUT;
		goto done;
	}

	pr_debug("switch-spi: Finished Tx poll at %llu\n", ktime_to_ns(ktime_get()));

	for (i = 0;i < SWITCH_PORTS;++i) {
		if (orig_portmask&(1<<i))
			 timestamp[i] = sja1105_recreate_ts(sja, timestamp[i]);
	}

	status = 0;
  done:
	return status;
}

int sja1105_spi_wait_egress_ts(struct sja1105_spi *sja, uint8_t portmask,
                               uint64_t *timestamp, int timeout_ms, unsigned tsreg)
{
	int err, i;
	struct spi_device *spi = sja1105_get_spi(sja);
	if (!spi) {
		if (!sja)
			return -EINVAL;
		else
			return -ESHUTDOWN;
	}

	if (portmask == 0 || portmask>>SWITCH_PORTS != 0 || !timestamp || !sja) {
		err = -EINVAL;
		goto out_spi;
	}

	for (i = 0;i < SWITCH_PORTS;++i) {
		if (portmask&(1<<i) && !port_is_up(sja, i)) {
			err = -ENETDOWN;
			goto out_spi;
		}
	}

	if (timeout_ms < 0 || timeout_ms > SWITCH_EPOCH_MS/3) {
		timeout_ms = SWITCH_EPOCH_MS/3;
	}

	mutex_lock(&sja->api_lock);
	if (!sja->enable_ptp || !smp_load_acquire(&sja->clkval_valid)) {
		err = -ENETDOWN;
		goto out_unlock;
	}

	err = poll_tx_ts(sja, spi, portmask, timestamp, timeout_ms, tsreg);

  out_unlock:
	mutex_unlock(&sja->api_lock);
  out_spi:
	spi_dev_put(spi);
	return err;
}
EXPORT_SYMBOL(sja1105_spi_wait_egress_ts);

static int thread_poll(void *data)
{
	struct sja1105_spi *sja = (struct sja1105_spi *)data;
	int err;

	while (!kthread_should_stop()) {
		mutex_lock(&sja->poll_lock);
		if (sja->poll_config.poll_enable && (sja->enable_ptp || sja->num_phys > 0)) {
			err = do_poll(sja);
			if (err) {
				pr_err_ratelimited("sja1105_spi: Could not poll switch.\n");
			}
		} else {
			sja->switch_confd = false;
		}
		mutex_unlock(&sja->poll_lock);

		schedule_timeout_interruptible(msecs_to_jiffies(SWITCH_EPOCH_MS/4));
	}

	return 0;
}

static int spidev_message(struct sja1105_spi *sja,
						  struct spi_device *spi,
                          struct spi_ioc_transfer *u_xfers,
                          unsigned n_xfers)
{
	struct spi_message	msg;
	struct spi_transfer	*k_xfers;
	struct spi_transfer	*k_tmp;
	struct spi_ioc_transfer *u_tmp;
	unsigned n, total, tx_total, rx_total;
	u8 *tx_buf = sja->user_spi_buf,
		*rx_buf = sja->user_spi_buf+SJA1105_SPI_BUFSIZE;
	int status = -EFAULT;
	static uint64_t message_counter = 0;

	spi_message_init(&msg);
	k_xfers = kcalloc(n_xfers, sizeof(*k_tmp), GFP_KERNEL);

	if (k_xfers == NULL) {
		return -ENOMEM;
	}

	/* Construct spi_message, copying any tx data to bounce buffer.
	 * We walk the array of user-provided transfers, using each one
	 * to initialize a kernel version of the same transfer.
	 */
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
			if (0) {
				uint32_t txd = *((uint32_t *)tx_buf);
				bool write;
				unsigned wordcount;
				unsigned address;

				write = (txd >> 31) & 1;
				wordcount = (txd >> 25) & 0x3f;
				address = (txd >> 4) & 0x1fffff;
				if (!write && ((address >= 0x200 && address <= 0x209)
											 || (address >= 0x400 && address <= 0x440)
											 || (address >= 0x600 && address <= 0x640))) {
					pr_info("SJA1105 IOCTL: high-level port status read from 0x%X\n", address);
				} else if (!write && ((address >= 0 && address <= 0xC)
															|| (address >= 0xc0 && address <= 0xc9))) {
					pr_info("SJA1105 IOCTL: general status read from 0x%X\n", address);
				} else if (!write && ((address >= 0x100 && address <= 0x107)
															|| (address >= 0x1000 && address <= 0x1007))) {
					pr_info("SJA1105 IOCTL: L2 memory partition status read from 0x%X\n", address);
				} else if (address >= 0x100016 && address <= 0x100030) {
					// CGU access
				} else {
					pr_info("SJA1105 IOCTL #%llu: %s length %u at 0x%X\n", message_counter, write ? "write" : "read", wordcount, address);
				}
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

	atomic_inc(&spi_sync_count);
	status = spi_sync(spi, &msg);

	if (status < 0) {
		goto done;
	} else {
		status = msg.actual_length;
	}

	/* Copy any rx data out of bounce buffer */
	rx_buf = sja->user_spi_buf+SJA1105_SPI_BUFSIZE;

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
	message_counter++;
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

static long sja1105_file_ioctl(struct file *filp, unsigned int cmd,
                              unsigned long arg)
{
	struct sja1105_spi *sja;
	struct spi_device *spi;
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

	if (!(spi = sja1105_get_spi(sja)))
		return -ESHUTDOWN;

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
#ifdef SPI_FAKE_SPEED
		ret = __put_user(READ_ONCE(sja->fake_speed), (__u32 __user *)arg);
#else
		ret = __put_user(spi->max_speed_hz, (__u32 __user *)arg);
#endif
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
#ifdef SPI_FAKE_SPEED
			WRITE_ONCE(sja->fake_speed, tmp);
#else
			if (tmp != spi->max_speed_hz) {
				pr_info("sja1105_spi: SPI_IOC_WR_MAX_SPEED_HZ 0x%x\n", tmp);
				ret = -EPERM;
			}
#endif
		}
		break;
	case SJA1105_IOC_POLL:
		ret = copy_from_user(&tmp_poll, (const void __user*)arg, sizeof(tmp_poll));
		if (ret == 0) {
			mutex_lock(&sja->poll_lock);
			mutex_lock(&sja->phy_lock);
			mutex_lock(&sja->api_lock);
			sja->poll_config.poll_enable = tmp_poll.poll_enable;
			if (sja->poll_config.poll_enable) {
				memcpy(sja->poll_config.mac_config, tmp_poll.mac_config,
					   sizeof(tmp_poll.mac_config));
				dev_info(sja->chardev, "Poll enabled");
				sja->switch_confd = false;
			} else {
				dev_info(sja->chardev, "Poll disabled");
				WRITE_ONCE(sja->clkval_valid, false);
			}
			mutex_unlock(&sja->api_lock);
			mutex_unlock(&sja->phy_lock);
			mutex_unlock(&sja->poll_lock);
		}
		break;
	case SJA1105_IOC_SWITCHID:
		ret = __get_user(tmp, (u8 __user*)arg);
		if (ret == 0)
			WRITE_ONCE(sja->switch_id, tmp);
		break;
	default:
	{
		ktime_t before, after, duration;
		static uint64_t ioctl_count = 0;

		int c = atomic_xchg(&spi_sync_count, 0);

		before = ktime_get();
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
		mutex_lock(&sja->user_buf_lock);
		ret = spidev_message(sja, spi, ioc, n_ioc);
		mutex_unlock(&sja->user_buf_lock);
		kfree(ioc);
		after = ktime_get();
		duration = ktime_sub(after, before);
		if (ktime_to_ns(duration) > 1000000) {
			pr_info("SJA1105 IOCTL #%llu DURATION: %lld ns, n_ioc=%u, count since last: %d\n", ioctl_count, ktime_to_ns(duration), n_ioc, c);
		}
		ioctl_count++;
		break;
	}
	}

	spi_dev_put(spi);

	return ret;
}

static int match_of_node(struct device *dev, void *data)
{
	return (dev_of_node(dev) == data);
}

struct sja1105_spi *sja1105_get_by_of_node(struct device_node *np)
{
	struct device *dev = driver_find_device(&sja1105_spi_driver.driver, NULL, np,
											&match_of_node);
	struct sja1105_spi *sja;
	if (!dev)
		return NULL;

	mutex_lock(&minor_lock);
	sja = spi_get_drvdata(to_spi_device(dev));
	if (sja)
		kref_get(&sja->ref);
	mutex_unlock(&minor_lock);
	put_device(dev);
	return sja;
}
EXPORT_SYMBOL(sja1105_get_by_of_node);

struct sja1105_spi *sja1105_get_by_spi(unsigned bus, unsigned chip_select)
{
#ifdef MULTISWITCH
	int i;

	mutex_lock(&minor_lock);
	for (i = 0;i < SJA1105_SPI_MAX_CHARDEVS;++i) {
		struct sja1105_spi *sja = sja_by_minor[i];
		struct spi_device *spi = sja1105_get_spi(sja);
		if (!spi) {
			continue;
		}

		if (spi->master->bus_num == bus &&
			spi->chip_select == chip_select) {
			kref_get(&sja->ref);
			spi_dev_put(spi);
			mutex_unlock(&minor_lock);
			return sja;
		}
		spi_dev_put(spi);
	}

	mutex_unlock(&minor_lock);
	return NULL;
#else
	struct sja1105_spi *sja;
	(void)bus;
	(void)chip_select;
	mutex_lock(&minor_lock);
	sja = sja_by_minor[0];
	if (sja)
		kref_get(&sja->ref);
	mutex_unlock(&minor_lock);
	return sja;
#endif
}
EXPORT_SYMBOL(sja1105_get_by_spi);

int sja1105_get_switch_id(struct sja1105_spi *sja)
{
	return READ_ONCE(sja->switch_id);
}
EXPORT_SYMBOL(sja1105_get_switch_id);

static int sja1105_file_open(struct inode *inode, struct file *filp)
{
	int minor = iminor(file_inode(filp));
	struct sja1105_spi *sja;
	mutex_lock(&minor_lock);
	sja = sja_by_minor[minor];
	kref_get(&sja->ref);
	mutex_unlock(&minor_lock);

	if (!sja) {
		return -ENODEV;
	}

	filp->private_data = sja;

	return 0;
}

static int sja1105_file_release(struct inode *inode, struct file *filp)
{
	struct sja1105_spi *sja = filp->private_data;
	sja1105_spi_put(sja);
	return 0;
}

static struct file_operations sja1105_spi_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = sja1105_file_ioctl,
	.open = sja1105_file_open,
	.release = sja1105_file_release,
};

static bool port_is_dynamic(struct sja1105_spi *sja, unsigned portnumber) {
	return (sja->poll_config.mac_config[portnumber] & SJA1105_MAC_RECONF_SPEED_MASK) == 0;
}

static int write_port_speed(struct sja1105_spi *sja, struct spi_device *spi,
							struct sja1105_phy *phy)
{
	struct sja1105_transfer write[3];
	int err;
	struct spi_message m;
	struct spi_transfer t[3];
	uint32_t *reg, *rgmii_clk = NULL, *idiv = NULL;
	bool rgmii = phy->phydev->interface==PHY_INTERFACE_MODE_RGMII;

	if (!port_is_dynamic(sja, phy->port)) {
		dev_dbg(sja->chardev, "Port %d: Ignore PHY speed due to static speed in switch config",
				phy->port);
		return 0;
	}

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

	err = init_sja_message(sja, write, rgmii?3:1, &m, t,
						   sja->phy_spi_buf, PHY_BUF_LEN);

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

	atomic_inc(&spi_sync_count);
	err = spi_sync(spi, &m);

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
		if (sja->phys[i].port == portnumber && port_is_dynamic(sja, portnumber)) {
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
	struct spi_device *spi = sja1105_get_spi(sja);
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

	mutex_lock(&sja->phy_lock);
	if (spi && sja->switch_confd && sja->poll_config.poll_enable) {
		if (write_port_speed(sja, spi, phy) != 0) {
			dev_warn(&netdev->dev, "Could not set port speed\n");
		}
	}
	mutex_unlock(&sja->phy_lock);
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
	struct device_node *np = dev_of_node(&spi->dev);
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
	int err, num_phys, sja_minor;
	int i;
	dev_t devt;
	struct sja_phy_setupdata phy_data;
	u8 *spi_buf;
	struct sched_param param = { .sched_priority = SWITCH_POLLER_SCHED_PRIORITY };
	struct cpumask mask = { .bits = SWITCH_POLLER_CPU_MASK_BITS };

	num_phys = sja1105_phy_early_setup(spi, &phy_data);
	if (num_phys < 0) {
		return num_phys;
	}

	sja = kvzalloc(sizeof(*sja), GFP_KERNEL);
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
	WRITE_ONCE(sja->clkval_valid, false);
	sja->enable_ptp = false;
	sja->num_phys = num_phys;
#ifdef SPI_FAKE_SPEED
	sja->fake_speed = SJA1105_SPI_SPEED_HZ;
#endif

	spi_buf = kzalloc(PHY_BUF_LEN+API_BUF_LEN+POLL_BUF_LEN, GFP_KERNEL);
	if (!spi_buf) {
		err = -ENOMEM;
		goto err_spi;
	}
	sja->phy_spi_buf = spi_buf;
	sja->api_spi_buf = spi_buf+PHY_BUF_LEN;
	sja->poll_spi_buf = spi_buf+PHY_BUF_LEN+API_BUF_LEN;
	sja->user_spi_buf = kzalloc(USER_BUF_LEN, GFP_KERNEL);
	if (!sja->user_spi_buf) {
		err = -ENOMEM;
		goto err_spibuf;
	}

	spin_lock_init(&sja->spi_lock);

	{
		u32 save_mode = spi->mode;
		u8 save_bpw = spi->bits_per_word;
		u32 save_speed = spi->max_speed_hz;

		spi->mode = SPI_MODE_1; /* CPOL=0; CPHA=1 */
		spi->bits_per_word = SPI_BITS_PER_WORD;
		spi->max_speed_hz = SJA1105_SPI_SPEED_HZ;

		err = spi_setup(spi);
		if (err < 0) {
			spi->mode = save_mode;
			spi->bits_per_word = save_bpw;
			spi->max_speed_hz = save_speed;
			goto err_userbuf;
		}
	}

	mutex_lock(&minor_lock);
	for (sja_minor = 0; sja_minor < SJA1105_SPI_MAX_CHARDEVS && sja_by_minor[sja_minor]; ++sja_minor);
	if (sja_minor >= SJA1105_SPI_MAX_CHARDEVS) {
		err = -ENOSPC;
		goto err_minor;
	}
	devt = MKDEV(sja1105_spi_major, sja_minor);
	sja_by_minor[sja_minor] = sja;
	sja->devnum = sja_minor;
	mutex_init(&sja->phy_lock);
	mutex_init(&sja->api_lock);
	mutex_init(&sja->user_buf_lock);
	mutex_init(&sja->poll_lock);
#ifdef MULTISWITCH
	sja->chardev = device_create(sja1105_spi_class, NULL, devt, NULL,
								 "sja1105_spi%d.%d", spi->master->bus_num,
								 spi->chip_select);
#else
	sja->chardev = device_create(sja1105_spi_class, NULL, devt, NULL, "sja1105_spi%d", sja_minor);
#endif

	if (IS_ERR(sja->chardev)) {
		err = PTR_ERR(sja->chardev);
		pr_err("sja1105_spi: Unable to create device %d\n", sja_minor);
		goto err_dev;
	}

	sja->chardev->driver = &sja1105_spi_driver.driver;

	err = sja1105_phy_late_setup(sja, &phy_data);
	if (err != 0) {
		goto err_latephy;
	}
	num_phys = 0; // To skip sja1105_phy_early_cleanup if an error occurs after this point.

	kref_init(&sja->ref);
	sja->poller = kthread_run(thread_poll, sja, "sja1105_spi%d", sja_minor);

	if (IS_ERR(sja->poller)) {
		err = PTR_ERR(sja->poller);
		goto err_poller;
	}
	err = sched_setscheduler_nocheck(sja->poller,
					SWITCH_POLLER_SCHED_POLICY,
					&param);
	if (err) {
		pr_err("Could not set switch poller scheduling attrs.\n");
		goto err_sched;
	}
	err = set_cpus_allowed_ptr(sja->poller, &mask);
	if (err) {
		pr_err("Could not set switch poller allowed CPUs.\n");
		goto err_sched;
	}
	sja->mgmt_index = SWITCH_MGMTROUTE_INDEX;
	mutex_unlock(&minor_lock);

	return 0;
  err_sched:
	kthread_stop(sja->poller);
  err_poller:
	sja1105_phy_cleanup(sja);
  err_latephy:
	device_destroy(sja1105_spi_class, devt);
  err_dev:
	mutex_destroy(&sja->poll_lock);
	mutex_destroy(&sja->user_buf_lock);
	mutex_destroy(&sja->api_lock);
	mutex_destroy(&sja->phy_lock);
  err_minor:
	mutex_unlock(&minor_lock);
  err_userbuf:
	kfree(sja->user_spi_buf);
  err_spibuf:
	kfree(sja->phy_spi_buf);
  err_spi:
	kvfree(sja);
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

	devt = MKDEV(sja1105_spi_major, sja->devnum);

	/* stop thread and wait for it to exit */
	kthread_stop(sja->poller);

	sja1105_phy_cleanup(sja);

	/* make sure ops on existing fds can abort cleanly */
	spin_lock_irq(&sja->spi_lock);
	sja->dev = NULL;
	spin_unlock_irq(&sja->spi_lock);

	/* prevent new opens */
	mutex_lock(&minor_lock);
	spi_set_drvdata(spi, NULL);
	sja_by_minor[sja->devnum] = NULL;
	device_destroy(sja1105_spi_class, devt);
	sja1105_spi_put(sja);
	mutex_unlock(&minor_lock);

	return 0;
}

static void sja1105_spi_release(struct sja1105_spi *sja)
{
	mutex_destroy(&sja->poll_lock);
	mutex_destroy(&sja->user_buf_lock);
	mutex_destroy(&sja->api_lock);
	mutex_destroy(&sja->phy_lock);

	kfree(sja->user_spi_buf);
	kfree(sja->phy_spi_buf);
	kvfree(sja);
}

void sja1105_spi_put(struct sja1105_spi *sja)
{
	kref_put(&sja->ref, (void(*)(struct kref*))sja1105_spi_release);
}
EXPORT_SYMBOL(sja1105_spi_put);

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
