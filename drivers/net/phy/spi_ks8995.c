/*
 * SPI driver for Micrel/Kendin KS8995M and KSZ8864RMN ethernet switches
 *
 * Copyright (C) 2008 Gabor Juhos <juhosg at openwrt.org>
 *
 * Plumbing into DSA for KSZ8794 is
 *     Copyright (c) 2018 Michael Walton <mike@farsouthnet.com>
 *
 * This file was based on: drivers/spi/at25.c
 *     Copyright (C) 2006 David Brownell
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/mii.h>

#include <linux/spi/spi.h>
#include <linux/netdevice.h>
#include <net/dsa.h>

#define DRV_VERSION		"0.1.3"
#define DRV_DESC		"Micrel KS8995/KSZ8794 Ethernet switch SPI driver"

/* ------------------------------------------------------------------------ */

#define KS8995_REG_ID0		0x00    /* Chip ID0 */
#define KS8995_REG_ID1		0x01    /* Chip ID1 */

#define KS8995_REG_GC0		0x02    /* Global Control 0 */
#define KS8995_REG_GC1		0x03    /* Global Control 1 */
#define KS8995_REG_GC2		0x04    /* Global Control 2 */
#define KS8995_REG_GC3		0x05    /* Global Control 3 */
#define KS8995_REG_GC4		0x06    /* Global Control 4 */
#define KS8995_REG_GC5		0x07    /* Global Control 5 */
#define KS8995_REG_GC6		0x08    /* Global Control 6 */
#define KS8995_REG_GC7		0x09    /* Global Control 7 */
#define KS8995_REG_GC8		0x0a    /* Global Control 8 */
#define KS8995_REG_GC9		0x0b    /* Global Control 9 */
#define KS8995_REG_GC10		0x0c    /* Global Control 10 */

#define KS8995_REG_PC(p, r)	((0x10 * p) + r)	 /* Port Control p = 0,1,2,... */
#define KS8995_REG_PS(p, r)	((0x10 * p) + r + 0xe)  /* Port Status */

#define KS8995_REG_TPC0		0x60    /* TOS Priority Control 0 */
#define KS8995_REG_TPC1		0x61    /* TOS Priority Control 1 */
#define KS8995_REG_TPC2		0x62    /* TOS Priority Control 2 */
#define KS8995_REG_TPC3		0x63    /* TOS Priority Control 3 */
#define KS8995_REG_TPC4		0x64    /* TOS Priority Control 4 */
#define KS8995_REG_TPC5		0x65    /* TOS Priority Control 5 */
#define KS8995_REG_TPC6		0x66    /* TOS Priority Control 6 */
#define KS8995_REG_TPC7		0x67    /* TOS Priority Control 7 */

#define KS8995_REG_MAC0		0x68    /* MAC address 0 */
#define KS8995_REG_MAC1		0x69    /* MAC address 1 */
#define KS8995_REG_MAC2		0x6a    /* MAC address 2 */
#define KS8995_REG_MAC3		0x6b    /* MAC address 3 */
#define KS8995_REG_MAC4		0x6c    /* MAC address 4 */
#define KS8995_REG_MAC5		0x6d    /* MAC address 5 */

#define KS8995_REG_IAC0		0x6e    /* Indirect Access Control 0 */
#define KS8995_REG_IAC1		0x6f    /* Indirect Access Control 0 */
#define KS8995_REG_IAD7		0x70    /* Indirect Access Data 7 */
#define KS8995_REG_IAD6		0x71    /* Indirect Access Data 6 */
#define KS8995_REG_IAD5		0x72    /* Indirect Access Data 5 */
#define KS8995_REG_IAD4		0x73    /* Indirect Access Data 4 */
#define KS8995_REG_IAD3		0x74    /* Indirect Access Data 3 */
#define KS8995_REG_IAD2		0x75    /* Indirect Access Data 2 */
#define KS8995_REG_IAD1		0x76    /* Indirect Access Data 1 */
#define KS8995_REG_IAD0		0x77    /* Indirect Access Data 0 */

#define KSZ8864_REG_ID1		0xfe	/* Chip ID in bit 7 */

#define KS8995_REGS_SIZE	0x80
#define KSZ8864_REGS_SIZE	0x100
#define KSZ8795_REGS_SIZE	0x100

#define ID1_CHIPID_M		0xf
#define ID1_CHIPID_S		4
#define ID1_REVISION_M		0x7
#define ID1_REVISION_S		1
#define ID1_START_SW		1	/* start the switch */

#define FAMILY_KS8995		0x95
#define FAMILY_KSZ8795		0x87
#define CHIPID_M		0
#define KS8995_CHIP_ID		0x00
#define KSZ8864_CHIP_ID		0x01
#define KSZ8795_CHIP_ID		0x09
#define KSZ8794_CHIP_ID		0x06

#define KS8995_CMD_WRITE	0x02U
#define KS8995_CMD_READ		0x03U

#define KS8995_RESET_DELAY	10 /* usec */

enum ks8995_chip_variant {
	ks8995,
	ksz8864,
	ksz8795,
	ksz8794,
	max_variant
};

struct ks8995_chip_params {
	char *name;
	int family_id;
	int chip_id;
	int regs_size;
	int addr_width;
	int addr_shift;
};

static const struct ks8995_chip_params ks8995_chip[] = {
	[ks8995] = {
		.name = "KS8995MA",
		.family_id = FAMILY_KS8995,
		.chip_id = KS8995_CHIP_ID,
		.regs_size = KS8995_REGS_SIZE,
		.addr_width = 8,
		.addr_shift = 0,
	},
	[ksz8864] = {
		.name = "KSZ8864RMN",
		.family_id = FAMILY_KS8995,
		.chip_id = KSZ8864_CHIP_ID,
		.regs_size = KSZ8864_REGS_SIZE,
		.addr_width = 8,
		.addr_shift = 0,
	},
	[ksz8795] = {
		.name = "KSZ8795CLX",
		.family_id = FAMILY_KSZ8795,
		.chip_id = KSZ8795_CHIP_ID,
		.regs_size = KSZ8795_REGS_SIZE,
		.addr_width = 12,
		.addr_shift = 1,
	},
	[ksz8794] = {
		.name = "KSZ8794CNX",
		.family_id = FAMILY_KSZ8795,
		.chip_id = KSZ8794_CHIP_ID,
		.regs_size = KSZ8795_REGS_SIZE,
		.addr_width = 12,
		.addr_shift = 1,
	},
};

struct ks8995_pdata {
	int reset_gpio;
	enum of_gpio_flags reset_gpio_flags;
};

struct ks8995_switch {
	struct dsa_switch 	*ds;
	int 				cpu_port;			/* port connected to CPU */
	struct spi_device	*spi;
	struct mutex		lock;
	struct ks8995_pdata	*pdata;
	struct bin_attribute	regs_attr;
	const struct ks8995_chip_params	*chip;
	int			revision_id;
};

static const struct spi_device_id ks8995_id[] = {
	{"ks8995", ks8995},
	{"ksz8864", ksz8864},
	{"ksz8795", ksz8795},
	{"ksz8794", ksz8794},
	{ }
};
MODULE_DEVICE_TABLE(spi, ks8995_id);

static inline u8 get_chip_id(u8 val)
{
	return (val >> ID1_CHIPID_S) & ID1_CHIPID_M;
}

static inline u8 get_chip_rev(u8 val)
{
	return (val >> ID1_REVISION_S) & ID1_REVISION_M;
}

/* create_spi_cmd - create a chip specific SPI command header
 * @ks: pointer to switch instance
 * @cmd: SPI command for switch
 * @address: register address for command
 *
 * Different chip families use different bit pattern to address the switches
 * registers:
 *
 * KS8995: 8bit command + 8bit address
 * KSZ8795: 3bit command + 12bit address + 1bit TR (?)
 */
static inline __be16 create_spi_cmd(struct ks8995_switch *ks, int cmd,
				    unsigned address)
{
	u16 result = cmd;

	/* make room for address (incl. address shift) */
	result <<= ks->chip->addr_width + ks->chip->addr_shift;
	/* add address */
	result |= address << ks->chip->addr_shift;
	/* SPI protocol needs big endian */
	return cpu_to_be16(result);
}
/* ------------------------------------------------------------------------ */
static int ks8995_read(struct ks8995_switch *ks, char *buf,
		 unsigned offset, size_t count)
{
	__be16 cmd;
	struct spi_transfer t[2];
	struct spi_message m;
	int err;

	cmd = create_spi_cmd(ks, KS8995_CMD_READ, offset);
	spi_message_init(&m);

	memset(&t, 0, sizeof(t));

	t[0].tx_buf = &cmd;
	t[0].len = sizeof(cmd);
	spi_message_add_tail(&t[0], &m);

	t[1].rx_buf = buf;
	t[1].len = count;
	spi_message_add_tail(&t[1], &m);

	mutex_lock(&ks->lock);
	err = spi_sync(ks->spi, &m);
	mutex_unlock(&ks->lock);

	return err ? err : count;
}

static int ks8995_write(struct ks8995_switch *ks, char *buf,
		 unsigned offset, size_t count)
{
	__be16 cmd;
	struct spi_transfer t[2];
	struct spi_message m;
	int err;

	cmd = create_spi_cmd(ks, KS8995_CMD_WRITE, offset);
	spi_message_init(&m);

	memset(&t, 0, sizeof(t));

	t[0].tx_buf = &cmd;
	t[0].len = sizeof(cmd);
	spi_message_add_tail(&t[0], &m);

	t[1].tx_buf = buf;
	t[1].len = count;
	spi_message_add_tail(&t[1], &m);

	mutex_lock(&ks->lock);
	err = spi_sync(ks->spi, &m);
	mutex_unlock(&ks->lock);

	return err ? err : count;
}

static inline int ks8995_read_reg(struct ks8995_switch *ks, u8 addr, u8 *buf)
{
	return ks8995_read(ks, buf, addr, 1) != 1;
}

static inline int ks8995_write_reg(struct ks8995_switch *ks, u8 addr, u8 val)
{
	char buf = val;

	return ks8995_write(ks, &buf, addr, 1) != 1;
}

/* ------------------------------------------------------------------------ */

static int ks8995_stop(struct ks8995_switch *ks)
{
	return ks8995_write_reg(ks, KS8995_REG_ID1, 0);
}

static int ks8995_start(struct ks8995_switch *ks)
{
	return ks8995_write_reg(ks, KS8995_REG_ID1, 1);
}

static int ks8995_reset(struct ks8995_switch *ks)
{
	int err;

	err = ks8995_stop(ks);
	if (err)
		return err;

	udelay(KS8995_RESET_DELAY);

	// MW: temp, enable ingress clock delay
	ks8995_write_reg(ks, 0x56, 0xff);

	return ks8995_start(ks);
}

static ssize_t ks8995_registers_read(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count)
{
	struct device *dev;
	struct ks8995_switch *ks8995;

	dev = container_of(kobj, struct device, kobj);
	ks8995 = dev_get_drvdata(dev);

	return ks8995_read(ks8995, buf, off, count);
}

static ssize_t ks8995_registers_write(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count)
{
	struct device *dev;
	struct ks8995_switch *ks8995;

	dev = container_of(kobj, struct device, kobj);
	ks8995 = dev_get_drvdata(dev);

	return ks8995_write(ks8995, buf, off, count);
}

/* ks8995_get_revision - get chip revision
 * @ks: pointer to switch instance
 *
 * Verify chip family and id and get chip revision.
 */
static int ks8995_get_revision(struct ks8995_switch *ks)
{
	int err;
	u8 id0, id1, ksz8864_id;

	/* read family id */
	err = ks8995_read_reg(ks, KS8995_REG_ID0, &id0);
	if (err) {
		err = -EIO;
		goto err_out;
	}

	/* verify family id */
	if (id0 != ks->chip->family_id) {
		dev_err(&ks->spi->dev, "chip family id mismatch: expected 0x%02x but 0x%02x read\n",
			ks->chip->family_id, id0);
		err = -ENODEV;
		goto err_out;
	}

	switch (ks->chip->family_id) {
	case FAMILY_KS8995:
		/* try reading chip id at CHIP ID1 */
		err = ks8995_read_reg(ks, KS8995_REG_ID1, &id1);
		if (err) {
			err = -EIO;
			goto err_out;
		}

		/* verify chip id */
		if ((get_chip_id(id1) == CHIPID_M) &&
		    (get_chip_id(id1) == ks->chip->chip_id)) {
			/* KS8995MA */
			ks->revision_id = get_chip_rev(id1);
		} else if (get_chip_id(id1) != CHIPID_M) {
			/* KSZ8864RMN */
			err = ks8995_read_reg(ks, KS8995_REG_ID1, &ksz8864_id);
			if (err) {
				err = -EIO;
				goto err_out;
			}

			if ((ksz8864_id & 0x80) &&
			    (ks->chip->chip_id == KSZ8864_CHIP_ID)) {
				ks->revision_id = get_chip_rev(id1);
			}

		} else {
			dev_err(&ks->spi->dev, "unsupported chip id for KS8995 family: 0x%02x\n",
				id1);
			err = -ENODEV;
		}
		break;
	case FAMILY_KSZ8795:
		/* try reading chip id at CHIP ID1 */
		err = ks8995_read_reg(ks, KS8995_REG_ID1, &id1);
		if (err) {
			err = -EIO;
			goto err_out;
		}

		if (get_chip_id(id1) == ks->chip->chip_id) {
			ks->revision_id = get_chip_rev(id1);
		} else {
			dev_err(&ks->spi->dev, "unsupported chip id for KSZ8795 family: 0x%02x\n",
				id1);
			err = -ENODEV;
		}
		break;
	default:
		dev_err(&ks->spi->dev, "unsupported family id: 0x%02x\n", id0);
		err = -ENODEV;
		break;
	}
err_out:
	return err;
}

/* ks8995_parse_dt - setup platform data from devicetree
 * @ks: pointer to switch instance
 *
 * Parses supported DT properties and sets up platform data
 * accordingly.
 */
static void ks8995_parse_dt(struct ks8995_switch *ks)
{
	struct device_node *np = ks->spi->dev.of_node;
	struct ks8995_pdata *pdata = ks->pdata;

	if (!np)
		return;

	pdata->reset_gpio = of_get_named_gpio_flags(np, "reset-gpios", 0,
		&pdata->reset_gpio_flags);
}

/*static void ksz_port_cfg(struct ks8995_switch *ks, int port, int offset, u8 bits,
			 bool set)
{
	int addr;
	u8 data;

	addr = KS8995_REG_PC(port, offset);
	ks8995_read_reg(ks, addr, &data);

	if (set)
		data |= bits;
	else
		data &= ~bits;

	ks8995_write_reg(ks, addr, data);
}*/

#define CPU_PORT_OFS 4

static void port_setup(struct ks8995_switch *ks, int port)
{
	/* enable tag tail for host port */
	if (port == ks->cpu_port) {
		ks8995_write_reg(ks, 0x02, 0x4c); // soft power down
		usleep_range(10, 20);
		ks8995_write_reg(ks, 0x02, 0x0c);
	    //ksz8895_write(dev, 0x0e, 0x00);
		ks8995_write_reg(ks, KS8995_REG_GC10, 0x46); // Tail tagging enable
		ks8995_write_reg(ks, KS8995_REG_PC(CPU_PORT_OFS, 0x12), 0x07); // learning disable, tx/rx enable
		ks8995_write_reg(ks, KS8995_REG_PC(CPU_PORT_OFS, 0xb1), 0x02); // 4 egress queues for switch MII
	} else {
		ks8995_write_reg(ks, KS8995_REG_PC(port, 0x12), 0x03); // learning disable, rx enable
	}
	/* set flow control */
	/*ksz_port_cfg(dev, port, REG_PORT_CTRL_0,
		     PORT_FORCE_TX_FLOW_CTRL | PORT_FORCE_RX_FLOW_CTRL, true);*/

	/* disable DiffServ priority */
	//ksz_port_cfg(dev, port, P_PRIO_CTRL, PORT_DIFFSERV_PRIO_ENABLE, false);

	/* replace priority */
	/*ksz_port_cfg(dev, port, REG_PORT_MRI_MAC_CTRL, PORT_USER_PRIO_CEILING,
		     false);
	ksz_port_cfg32(dev, port, REG_PORT_MTI_QUEUE_CTRL_0__4,
		       MTI_PVID_REPLACE, false);*/

	/* enable 802.1p priority */
	//ksz_port_cfg(dev, port, P_PRIO_CTRL, PORT_802_1P_PRIO_ENABLE, true);


	/* clear pending interrupts */
	//ksz_pread16(dev, port, REG_PORT_PHY_INT_ENABLE, &data16);
}

static enum dsa_tag_protocol ks8995_get_tag_protocol(struct dsa_switch *ds)
{
	return DSA_TAG_PROTO_KSZ879X;
}

static int ks8995_setup(struct dsa_switch *ds)
{
	struct ks8995_switch *ks = ds->priv;
	int i;

	ds->num_ports = 4;

	for (i = 0; i < ds->num_ports; i++) {
		if (dsa_is_cpu_port(ds, i)) {
			struct dsa_port *port = &ds->ports[i];
			ks->cpu_port = i;
			/* enable cpu port */
			port_setup(ks, i);

			if (!netif_running(port->netdev)) {
				rtnl_lock();
				dev_open(port->netdev);
				rtnl_unlock();
			}

			// Rename our cpu port's device
			/*dev_get_valid_name(dev_net(port->netdev), port->netdev, "sw%d");
			dev_change_name(port->netdev, "sw%d");*/
			break;
		}
	}

	// TODO: restart switch?
	return 0;
}

static int ks8995_phy_read16(struct dsa_switch *ds, int p, int reg)
{
	struct ks8995_switch *ks = ds->priv;
	u8 s0, s1, s2;
	int r = 0;

	/* phyaddr is 0 - 2 for KSZ8794 */
	/* Simulate MIIM of this device */
	if (reg == MII_PHYSID1) return 0x0022;
	if (reg == MII_PHYSID2) return 0x1550;
	if (reg == MII_BMCR) {
		ks8995_read_reg(ks, KS8995_REG_PC(p, 0x1c), &s0); // CONTROL 9
		ks8995_read_reg(ks, KS8995_REG_PC(p, 0x1d), &s1); // CONTROL 10
		ks8995_read_reg(ks, KS8995_REG_PC(p, 0x1f), &s2); // CONTROL 11
		if (!(s0 & 0x80)) r |= BMCR_ANENABLE;
		if (s0 & 0x40) r |= BMCR_SPEED100;
		if (s0 & 0x20) r |= BMCR_FULLDPLX;
		if (s1 & 0x80) r |= 0x0001; // led off
		if (s1 & 0x40) r |= 0x0002; // disable tx
		if (s1 & 0x20) r |= BMCR_ANRESTART;
		if (s1 & 0x08) r |= BMCR_PDOWN;
		if (s1 & 0x04) r |= 0x0008; // disable MDI/MDIX
		if (s1 & 0x02) r |= 0x0010; // force MDI
		if (s1 & 0x01) r |= BMCR_LOOPBACK;
		if (s2 & 0x20) r |= BMCR_ISOLATE;
		if (s2 & 0x10) r |= BMCR_RESET;
	} else if (reg == MII_BMSR) {
		r = BMSR_100FULL | BMSR_100HALF | BMSR_10FULL | BMSR_10HALF | BMSR_ANEGCAPABLE;
		ks8995_read_reg(ks, KS8995_REG_PC(p, 0x1e), &s0); // STATUS 2
		if (s0 & 0x40) r |= BMSR_ANEGCOMPLETE;
		if (s0 & 0x20) r |= BMSR_LSTATUS;
	} else if (reg == MII_ADVERTISE) {
		/* Just hard code it */
		r = ADVERTISE_PAUSE_CAP | ADVERTISE_100FULL | ADVERTISE_10FULL | ADVERTISE_100HALF | ADVERTISE_10HALF | ADVERTISE_CSMA;
	} else if (reg == MII_LPA) {
		ks8995_read_reg(ks, KS8995_REG_PC(p, 0x18), &s0); // STATUS 0
		if (s0 & 0x01) r |= LPA_10HALF;
		if (s0 & 0x02) r |= LPA_10FULL;
		if (s0 & 0x04) r |= LPA_100HALF;
		if (s0 & 0x08) r |= LPA_100FULL;
		if (s0 & 0x20) r |= LPA_PAUSE_ASYM;
	} else if (reg == 0x1f) { /* PHY special control/status */
		/* TODO? */
	} else {
		return 0xffff;
	}

	return r;
}

static int ks8995_phy_write16(struct dsa_switch *ds, int p, int reg, u16 val)
{
	struct ks8995_switch *ks = ds->priv;
	u8 s0, s1, s2;

	/* phyaddr is 0 - 2 for KSZ8794 */
	/* Simulate MIIM of this device */
	if (reg == MII_BMCR) {
		ks8995_read_reg(ks, KS8995_REG_PC(p, 0x1c), &s0); // CONTROL 9
		ks8995_read_reg(ks, KS8995_REG_PC(p, 0x1f), &s2); // CONTROL 11
		s0 &= ~(0x80|0x40|0x20);
		s1 = 0;
		s2 &= ~(0x20|0x10);
		if (!(val & BMCR_ANENABLE)) s0 |= 0x80; // this bit is opposite in polarity
		if (val & BMCR_SPEED100) s0 |= 0x40;
		if (val & BMCR_FULLDPLX) s0 |= 0x20;
		if (val & 0x0001) s1 |= 0x80;
		if (val & 0x0002) s1 |= 0x40;
		if (val & BMCR_ANRESTART) s1 |= 0x20;
		if (val & BMCR_PDOWN) s1 |= 0x08;
		if (val & 0x0008) s1 |= 0x04;
		if (val & 0x0010) s1 |= 0x02;
		if (val & BMCR_LOOPBACK) s1 |= 0x01;
		if (val & BMCR_ISOLATE) s2 |= 0x20;
		if (val & BMCR_RESET) s2 |= 0x10;

		/* reset (if enabled) first */
		ks8995_write_reg(ks, KS8995_REG_PC(p, 0x1f), s2);
		// Hang on?
		usleep_range(10, 20);
		//udelay(KSZ8794_RESET_DELAY);
		ks8995_write_reg(ks, KS8995_REG_PC(p, 0x1c), s0);
		ks8995_write_reg(ks, KS8995_REG_PC(p, 0x1d), s1); // CONTROL 10
	}
	return 0;
}

static int ks8995_enable_port(struct dsa_switch *ds, int port,
			   struct phy_device *phy)
{
	struct ks8995_switch *ks = ds->priv;

	/* setup slave port */
	port_setup(ks, port);

	return 0;
}

static void ks8995_disable_port(struct dsa_switch *ds, int port,
			     struct phy_device *phy)
{
	struct ks8995_switch *ks = ds->priv;

    ks8995_write_reg(ks, KS8995_REG_PC(port, 0x12), 0x01); // rx disabled
}



static const struct dsa_switch_ops ks8995_switch_ops = {
	.get_tag_protocol	= ks8995_get_tag_protocol,
	.setup			= ks8995_setup,
	.phy_read		= ks8995_phy_read16,
	.phy_write		= ks8995_phy_write16,
	.port_enable		= ks8995_enable_port,
	.port_disable		= ks8995_disable_port,
	//.get_strings		= ksz_get_strings,
	//.get_ethtool_stats	= ksz_get_ethtool_stats,
	//.get_sset_count		= ksz_sset_count,
	//.port_stp_state_set	= ksz_port_stp_state_set,
	//.port_fast_age		= ksz_port_fast_age,
	//.port_vlan_filtering	= ksz_port_vlan_filtering,
	//.port_vlan_prepare	= ksz_port_vlan_prepare,
	//.port_vlan_add		= ksz_port_vlan_add,
	//.port_vlan_del		= ksz_port_vlan_del,
	//.port_fdb_dump		= ksz_port_fdb_dump,
	//.port_fdb_add		= ksz_port_fdb_add,
	//.port_fdb_del		= ksz_port_fdb_del,
	//.port_mdb_prepare       = ksz_port_mdb_prepare,
	//.port_mdb_add           = ksz_port_mdb_add,
	//.port_mdb_del           = ksz_port_mdb_del,
	//.port_mirror_add	= ksz_port_mirror_add,
	//.port_mirror_del	= ksz_port_mirror_del,
};



static const struct bin_attribute ks8995_registers_attr = {
	.attr = {
		.name   = "registers",
		.mode   = S_IRUSR | S_IWUSR,
	},
	.size   = KS8995_REGS_SIZE,
	.read   = ks8995_registers_read,
	.write  = ks8995_registers_write,
};

/* ------------------------------------------------------------------------ */
static int ks8995_probe(struct spi_device *spi)
{
	struct ks8995_switch *ks;
	int err;
	int variant = spi_get_device_id(spi)->driver_data;
	int i;

	if (variant >= max_variant) {
		dev_err(&spi->dev, "bad chip variant %d\n", variant);
		return -ENODEV;
	}

	ks = devm_kzalloc(&spi->dev, sizeof(*ks), GFP_KERNEL);
	if (!ks)
		return -ENOMEM;

	// Allocate a 4-port switch
	ks->ds = dsa_switch_alloc(&spi->dev, 4);
	if (!ks->ds) {
		devm_kfree(&spi->dev, ks);
		return -ENOMEM;
	}
	ks->ds->priv = ks;
	ks->ds->ops = &ks8995_switch_ops;

	mutex_init(&ks->lock);
	ks->spi = spi;
	ks->chip = &ks8995_chip[variant];

	if (ks->spi->dev.of_node) {
		ks->pdata = devm_kzalloc(&spi->dev, sizeof(*ks->pdata),
					 GFP_KERNEL);
		if (!ks->pdata)
			return -ENOMEM;

		ks->pdata->reset_gpio = -1;

		ks8995_parse_dt(ks);
	}

	if (!ks->pdata)
		ks->pdata = spi->dev.platform_data;

	/* de-assert switch reset */
	if (ks->pdata && gpio_is_valid(ks->pdata->reset_gpio)) {
		unsigned long flags;

		flags = (ks->pdata->reset_gpio_flags == OF_GPIO_ACTIVE_LOW ?
			 GPIOF_ACTIVE_LOW : 0);

		err = devm_gpio_request_one(&spi->dev,
					    ks->pdata->reset_gpio,
					    flags, "switch-reset");
		if (err) {
			dev_err(&spi->dev,
				"failed to get reset-gpios: %d\n", err);
			return -EIO;
		}

		gpiod_set_value(gpio_to_desc(ks->pdata->reset_gpio), 0);
	}

	spi_set_drvdata(spi, ks);

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	err = spi_setup(spi);
	if (err) {
		dev_err(&spi->dev, "spi_setup failed, err=%d\n", err);
		return err;
	}

	err = ks8995_get_revision(ks);
	if (err)
		return err;

	memcpy(&ks->regs_attr, &ks8995_registers_attr, sizeof(ks->regs_attr));
	ks->regs_attr.size = ks->chip->regs_size;

	err = ks8995_reset(ks);
	if (err)
		return err;

	sysfs_attr_init(&ks->regs_attr.attr);
	err = sysfs_create_bin_file(&spi->dev.kobj, &ks->regs_attr);
	if (err) {
		dev_err(&spi->dev, "unable to create sysfs file, err=%d\n",
				    err);
		return err;
	}

	dev_info(&spi->dev, "%s device found, Chip ID:%x, Revision:%x\n",
		 ks->chip->name, ks->chip->chip_id, ks->revision_id);

	err = dsa_register_switch(ks->ds);
	if (err != 0) {
		dev_err(&spi->dev, "unable to register dsa switch, err=%d\n",
				    err);
		return err;
	}
	// Now is the time to fix up MAC addresses
	for (i = 0; i < ks->ds->num_ports; i++) {
		struct dsa_port *port = &ks->ds->ports[i];
		if (dsa_is_cpu_port(ks->ds, i))
			continue;
		if (port && port->netdev) {
			bool havemac = false;
			// Get label on port (e.g. eth0) and look for a corresponding alias to get the MAC address (e.g. ethernet0)
			if (strncmp("eth", port->netdev->name, 3) == 0 && strlen(port->netdev->name) == 4) {
				char n = port->netdev->name[3];
				char enb[16];
				const struct device_node *nd;
				sprintf(enb, "ethernet%c", (int) n);
				nd = of_find_node_by_path(enb);
				if (nd) {
					int len;
					const void *addr = of_get_property(nd, "local-mac-address", &len);
					if (addr && len == ETH_ALEN) {
						memcpy(port->netdev->dev_addr, addr, ETH_ALEN);
						havemac = true;
					}
				}
			}
			if (!havemac) {
				dev_warn(&spi->dev, "No MAC address found for %s, munging by adding %d to lsb\n",
					 port->netdev->name, i);
				port->netdev->dev_addr[5] += i;
			}
		}
	}



	return 0;
}

static int ks8995_remove(struct spi_device *spi)
{
	struct ks8995_switch *ks = spi_get_drvdata(spi);

	dsa_unregister_switch(ks->ds);

	sysfs_remove_bin_file(&spi->dev.kobj, &ks->regs_attr);

	/* assert reset */
	if (ks->pdata && gpio_is_valid(ks->pdata->reset_gpio))
		gpiod_set_value(gpio_to_desc(ks->pdata->reset_gpio), 1);

	return 0;
}

/* ------------------------------------------------------------------------ */
static struct spi_driver ks8995_driver = {
	.driver = {
		.name	    = "spi-ks8995",
	},
	.probe	  = ks8995_probe,
	.remove	  = ks8995_remove,
	.id_table = ks8995_id,
};

module_spi_driver(ks8995_driver);

MODULE_DESCRIPTION(DRV_DESC);
MODULE_VERSION(DRV_VERSION);
MODULE_AUTHOR("Gabor Juhos <juhosg at openwrt.org>");
MODULE_AUTHOR("Michael Walton <mike@farsouthnet.com>");
MODULE_LICENSE("GPL v2");
