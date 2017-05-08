/*
 * MCP23S08 SPI/I2C GPIO gpio expander driver
 *
 * The inputs and outputs of the mcp23s08, mcp23s17, mcp23008 and mcp23017 are
 * supported.
 * For the I2C versions of the chips (mcp23008 and mcp23017) generation of
 * interrupts is also supported.
 * The hardware of the SPI versions of the chips (mcp23s08 and mcp23s17) is
 * also capable of generating interrupts, but the linux driver does not
 * support that yet.
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/spi/mcp23s08.h>
#include <linux/slab.h>
#include <asm/byteorder.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>

/**
 * MCP types supported by driver
 */
#define MCP_TYPE_008	2
#define MCP_TYPE_017	3

/* Registers are all 8 bits wide.
 *
 * The mcp23s17 has twice as many bits, and can be configured to work
 * with either 16 bit registers or with two adjacent 8 bit banks.
 */
#define MCP_IODIR	0x00		/* init/reset:  all ones */
#define MCP_IPOL	0x01
#define MCP_GPINTEN	0x02
#define MCP_DEFVAL	0x03
#define MCP_INTCON	0x04
#define MCP_IOCON	0x05
#	define IOCON_MIRROR	(1 << 6)
#	define IOCON_SEQOP	(1 << 5)
#	define IOCON_HAEN	(1 << 3)
#	define IOCON_ODR	(1 << 2)
#	define IOCON_INTPOL	(1 << 1)
#	define IOCON_INTCC	(1)
#define MCP_GPPU	0x06
#define MCP_INTF	0x07
#define MCP_INTCAP	0x08
#define MCP_GPIO	0x09
#define MCP_OLAT	0x0a

struct mcp23s08;

struct mcp23s08_ops {
	int	(*read)(struct mcp23s08 *mcp, unsigned reg);
	int	(*write)(struct mcp23s08 *mcp, unsigned reg, unsigned val);
	int	(*read_regs)(struct mcp23s08 *mcp, unsigned reg,
			     u16 *vals, unsigned n);
};

struct mcp23s08 {
	u8			addr;
	bool			irq_active_high;

	u16			cache[11];
	u16			irq_rise;
	u16			irq_fall;
	int			irq;
	bool			irq_controller;
	/* lock protects the cached values */
	struct mutex		lock;
	struct mutex		irq_lock;

	struct gpio_chip	chip;

	const struct mcp23s08_ops	*ops;
	void			*data; /* ops specific data */
};

/* A given spi_device can represent up to eight mcp23sxx chips
 * sharing the same chipselect but using different addresses
 * (e.g. chips #0 and #3 might be populated, but not #1 or $2).
 * Driver data holds all the per-chip data.
 */
struct mcp23s08_driver_data {
	unsigned		ngpio;
	struct mcp23s08		*mcp[8];
	struct mcp23s08		chip[];
};

/*----------------------------------------------------------------------*/

static int mcp23008_read(struct mcp23s08 *mcp, unsigned reg)
{
	return i2c_smbus_read_byte_data(mcp->data, reg);
}

static int mcp23008_write(struct mcp23s08 *mcp, unsigned reg, unsigned val)
{
	return i2c_smbus_write_byte_data(mcp->data, reg, val);
}

static int
mcp23008_read_regs(struct mcp23s08 *mcp, unsigned reg, u16 *vals, unsigned n)
{
	while (n--) {
		int ret = mcp23008_read(mcp, reg++);
		if (ret < 0)
			return ret;
		*vals++ = ret;
	}

	return 0;
}

static int mcp23017_read(struct mcp23s08 *mcp, unsigned reg)
{
	return i2c_smbus_read_word_data(mcp->data, reg << 1);
}

static int mcp23017_write(struct mcp23s08 *mcp, unsigned reg, unsigned val)
{
	return i2c_smbus_write_word_data(mcp->data, reg << 1, val);
}

static int
mcp23017_read_regs(struct mcp23s08 *mcp, unsigned reg, u16 *vals, unsigned n)
{
	while (n--) {
		int ret = mcp23017_read(mcp, reg++);
		if (ret < 0)
			return ret;
		*vals++ = ret;
	}

	return 0;
}

static const struct mcp23s08_ops mcp23008_ops = {
	.read		= mcp23008_read,
	.write		= mcp23008_write,
	.read_regs	= mcp23008_read_regs,
};

static const struct mcp23s08_ops mcp23017_ops = {
	.read		= mcp23017_read,
	.write		= mcp23017_write,
	.read_regs	= mcp23017_read_regs,
};

/*----------------------------------------------------------------------*/


static int mcp23s08_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct mcp23s08	*mcp = gpiochip_get_data(chip);
	int status;

	mutex_lock(&mcp->lock);
	mcp->cache[MCP_IODIR] |= (1 << offset);
	status = mcp->ops->write(mcp, MCP_IODIR, mcp->cache[MCP_IODIR]);
	mutex_unlock(&mcp->lock);
	return status;
}

static int mcp23s08_get(struct gpio_chip *chip, unsigned offset)
{
	struct mcp23s08	*mcp = gpiochip_get_data(chip);
	int status;

	mutex_lock(&mcp->lock);

	/* REVISIT reading this clears any IRQ ... */
	status = mcp->ops->read(mcp, MCP_GPIO);
	if (status < 0)
		status = 0;
	else {
		mcp->cache[MCP_GPIO] = status;
		status = !!(status & (1 << offset));
	}
	mutex_unlock(&mcp->lock);
	return status;
}

static int __mcp23s08_set(struct mcp23s08 *mcp, unsigned mask, int value)
{
	unsigned olat = mcp->cache[MCP_OLAT];

	if (value)
		olat |= mask;
	else
		olat &= ~mask;
	mcp->cache[MCP_OLAT] = olat;
	return mcp->ops->write(mcp, MCP_OLAT, olat);
}

static void mcp23s08_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct mcp23s08	*mcp = gpiochip_get_data(chip);
	unsigned mask = 1 << offset;

	mutex_lock(&mcp->lock);
	__mcp23s08_set(mcp, mask, value);
	mutex_unlock(&mcp->lock);
}

static int
mcp23s08_direction_output(struct gpio_chip *chip, unsigned offset, int value)
{
	struct mcp23s08	*mcp = gpiochip_get_data(chip);
	unsigned mask = 1 << offset;
	int status;

	mutex_lock(&mcp->lock);
	status = __mcp23s08_set(mcp, mask, value);
	if (status == 0) {
		mcp->cache[MCP_IODIR] &= ~mask;
		status = mcp->ops->write(mcp, MCP_IODIR, mcp->cache[MCP_IODIR]);
	}
	mutex_unlock(&mcp->lock);
	return status;
}

/*----------------------------------------------------------------------*/
static irqreturn_t mcp23s08_irq(int irq, void *data)
{
	struct mcp23s08 *mcp = data;
	int intcap, intf, i;
	unsigned int child_irq;

	mutex_lock(&mcp->lock);
	intf = mcp->ops->read(mcp, MCP_INTF);
	if (intf < 0) {
		mutex_unlock(&mcp->lock);
		return IRQ_HANDLED;
	}

	mcp->cache[MCP_INTF] = intf;

	intcap = mcp->ops->read(mcp, MCP_INTCAP);
	if (intcap < 0) {
		mutex_unlock(&mcp->lock);
		return IRQ_HANDLED;
	}

	mcp->cache[MCP_INTCAP] = intcap;
	mutex_unlock(&mcp->lock);


	for (i = 0; i < mcp->chip.ngpio; i++) {
		if ((BIT(i) & mcp->cache[MCP_INTF]) &&
		    ((BIT(i) & intcap & mcp->irq_rise) ||
		     (mcp->irq_fall & ~intcap & BIT(i)) ||
		     (BIT(i) & mcp->cache[MCP_INTCON]))) {
			child_irq = irq_find_mapping(mcp->chip.irqdomain, i);
			handle_nested_irq(child_irq);
		}
	}

	return IRQ_HANDLED;
}

static void mcp23s08_irq_mask(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct mcp23s08 *mcp = gpiochip_get_data(gc);
	unsigned int pos = data->hwirq;

	mcp->cache[MCP_GPINTEN] &= ~BIT(pos);
}

static void mcp23s08_irq_unmask(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct mcp23s08 *mcp = gpiochip_get_data(gc);
	unsigned int pos = data->hwirq;

	mcp->cache[MCP_GPINTEN] |= BIT(pos);
}

static int mcp23s08_irq_set_type(struct irq_data *data, unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct mcp23s08 *mcp = gpiochip_get_data(gc);
	unsigned int pos = data->hwirq;
	int status = 0;

	if ((type & IRQ_TYPE_EDGE_BOTH) == IRQ_TYPE_EDGE_BOTH) {
		mcp->cache[MCP_INTCON] &= ~BIT(pos);
		mcp->irq_rise |= BIT(pos);
		mcp->irq_fall |= BIT(pos);
	} else if (type & IRQ_TYPE_EDGE_RISING) {
		mcp->cache[MCP_INTCON] &= ~BIT(pos);
		mcp->irq_rise |= BIT(pos);
		mcp->irq_fall &= ~BIT(pos);
	} else if (type & IRQ_TYPE_EDGE_FALLING) {
		mcp->cache[MCP_INTCON] &= ~BIT(pos);
		mcp->irq_rise &= ~BIT(pos);
		mcp->irq_fall |= BIT(pos);
	} else if (type & IRQ_TYPE_LEVEL_HIGH) {
		mcp->cache[MCP_INTCON] |= BIT(pos);
		mcp->cache[MCP_DEFVAL] &= ~BIT(pos);
	} else if (type & IRQ_TYPE_LEVEL_LOW) {
		mcp->cache[MCP_INTCON] |= BIT(pos);
		mcp->cache[MCP_DEFVAL] |= BIT(pos);
	} else
		return -EINVAL;

	return status;
}

static void mcp23s08_irq_bus_lock(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct mcp23s08 *mcp = gpiochip_get_data(gc);

	mutex_lock(&mcp->irq_lock);
}

static void mcp23s08_irq_bus_unlock(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct mcp23s08 *mcp = gpiochip_get_data(gc);

	mutex_lock(&mcp->lock);
	mcp->ops->write(mcp, MCP_GPINTEN, mcp->cache[MCP_GPINTEN]);
	mcp->ops->write(mcp, MCP_DEFVAL, mcp->cache[MCP_DEFVAL]);
	mcp->ops->write(mcp, MCP_INTCON, mcp->cache[MCP_INTCON]);
	mutex_unlock(&mcp->lock);
	mutex_unlock(&mcp->irq_lock);
}

static struct irq_chip mcp23s08_irq_chip = {
	.name = "gpio-mcp23xxx",
	.irq_mask = mcp23s08_irq_mask,
	.irq_unmask = mcp23s08_irq_unmask,
	.irq_set_type = mcp23s08_irq_set_type,
	.irq_bus_lock = mcp23s08_irq_bus_lock,
	.irq_bus_sync_unlock = mcp23s08_irq_bus_unlock,
};

static int mcp23s08_irq_setup(struct mcp23s08 *mcp)
{
	struct gpio_chip *chip = &mcp->chip;
	int err;
	unsigned long irqflags = IRQF_ONESHOT | IRQF_SHARED;

	mutex_init(&mcp->irq_lock);

	if (mcp->irq_active_high)
		irqflags |= IRQF_TRIGGER_HIGH;
	else
		irqflags |= IRQF_TRIGGER_LOW;

	err = devm_request_threaded_irq(chip->parent, mcp->irq, NULL,
					mcp23s08_irq,
					irqflags, dev_name(chip->parent), mcp);
	if (err != 0) {
		dev_err(chip->parent, "unable to request IRQ#%d: %d\n",
			mcp->irq, err);
		return err;
	}

	err =  gpiochip_irqchip_add_nested(chip,
					   &mcp23s08_irq_chip,
					   0,
					   handle_simple_irq,
					   IRQ_TYPE_NONE);
	if (err) {
		dev_err(chip->parent,
			"could not connect irqchip to gpiochip: %d\n", err);
		return err;
	}

	gpiochip_set_nested_irqchip(chip,
				    &mcp23s08_irq_chip,
				    mcp->irq);

	return 0;
}

/*----------------------------------------------------------------------*/

#ifdef CONFIG_DEBUG_FS

#include <linux/seq_file.h>

/*
 * This shows more info than the generic gpio dump code:
 * pullups, deglitching, open drain drive.
 */
static void mcp23s08_dbg_show(struct seq_file *s, struct gpio_chip *chip)
{
	struct mcp23s08	*mcp;
	char		bank;
	int		t;
	unsigned	mask;

	mcp = gpiochip_get_data(chip);

	/* NOTE: we only handle one bank for now ... */
	bank = '0' + ((mcp->addr >> 1) & 0x7);

	mutex_lock(&mcp->lock);
	t = mcp->ops->read_regs(mcp, 0, mcp->cache, ARRAY_SIZE(mcp->cache));
	if (t < 0) {
		seq_printf(s, " I/O ERROR %d\n", t);
		goto done;
	}

	for (t = 0, mask = 1; t < chip->ngpio; t++, mask <<= 1) {
		const char	*label;

		label = gpiochip_is_requested(chip, t);
		if (!label)
			continue;

		seq_printf(s, " gpio-%-3d P%c.%d (%-12s) %s %s %s",
			chip->base + t, bank, t, label,
			(mcp->cache[MCP_IODIR] & mask) ? "in " : "out",
			(mcp->cache[MCP_GPIO] & mask) ? "hi" : "lo",
			(mcp->cache[MCP_GPPU] & mask) ? "up" : "  ");
		/* NOTE:  ignoring the irq-related registers */
		seq_puts(s, "\n");
	}
done:
	mutex_unlock(&mcp->lock);
}

#else
#define mcp23s08_dbg_show	NULL
#endif

/*----------------------------------------------------------------------*/

static int mcp23s08_probe_one(struct mcp23s08 *mcp, struct device *dev,
			      void *data, unsigned addr, unsigned type,
			      struct mcp23s08_platform_data *pdata, int cs)
{
	int status;
	bool mirror = false;

	mutex_init(&mcp->lock);

	mcp->data = data;
	mcp->addr = addr;
	mcp->irq_active_high = false;

	mcp->chip.direction_input = mcp23s08_direction_input;
	mcp->chip.get = mcp23s08_get;
	mcp->chip.direction_output = mcp23s08_direction_output;
	mcp->chip.set = mcp23s08_set;
	mcp->chip.dbg_show = mcp23s08_dbg_show;
#ifdef CONFIG_OF_GPIO
	mcp->chip.of_gpio_n_cells = 2;
	mcp->chip.of_node = dev->of_node;
#endif

	switch (type) {
	case MCP_TYPE_008:
		mcp->ops = &mcp23008_ops;
		mcp->chip.ngpio = 8;
		mcp->chip.label = "mcp23008";
		break;

	case MCP_TYPE_017:
		mcp->ops = &mcp23017_ops;
		mcp->chip.ngpio = 16;
		mcp->chip.label = "mcp23017";
		break;
	default:
		dev_err(dev, "invalid device type (%d)\n", type);
		return -EINVAL;
	}

	mcp->chip.base = pdata->base;
	mcp->chip.can_sleep = true;
	mcp->chip.parent = dev;
	mcp->chip.owner = THIS_MODULE;

	/* verify MCP_IOCON.SEQOP = 0, so sequential reads work,
	 * and MCP_IOCON.HAEN = 1, so we work with all chips.
	 */

	status = mcp->ops->read(mcp, MCP_IOCON);
	if (status < 0)
		goto fail;

	mcp->irq_controller = pdata->irq_controller;
	if (mcp->irq && mcp->irq_controller) {
		mcp->irq_active_high =
			of_property_read_bool(mcp->chip.parent->of_node,
					      "microchip,irq-active-high");

		mirror = pdata->mirror;
	}

	if ((status & IOCON_SEQOP) || !(status & IOCON_HAEN) || mirror ||
	     mcp->irq_active_high) {
		/* mcp23s17 has IOCON twice, make sure they are in sync */
		status &= ~(IOCON_SEQOP | (IOCON_SEQOP << 8));
		status |= IOCON_HAEN | (IOCON_HAEN << 8);
		if (mcp->irq_active_high)
			status |= IOCON_INTPOL | (IOCON_INTPOL << 8);
		else
			status &= ~(IOCON_INTPOL | (IOCON_INTPOL << 8));

		if (mirror)
			status |= IOCON_MIRROR | (IOCON_MIRROR << 8);

		status = mcp->ops->write(mcp, MCP_IOCON, status);
		if (status < 0)
			goto fail;
	}

	/* configure ~100K pullups */
	status = mcp->ops->write(mcp, MCP_GPPU, pdata->chip[cs].pullups);
	if (status < 0)
		goto fail;

	status = mcp->ops->read_regs(mcp, 0, mcp->cache, ARRAY_SIZE(mcp->cache));
	if (status < 0)
		goto fail;

	/* disable inverter on input */
	if (mcp->cache[MCP_IPOL] != 0) {
		mcp->cache[MCP_IPOL] = 0;
		status = mcp->ops->write(mcp, MCP_IPOL, 0);
		if (status < 0)
			goto fail;
	}

	/* disable irqs */
	if (mcp->cache[MCP_GPINTEN] != 0) {
		mcp->cache[MCP_GPINTEN] = 0;
		status = mcp->ops->write(mcp, MCP_GPINTEN, 0);
		if (status < 0)
			goto fail;
	}

	status = gpiochip_add_data(&mcp->chip, mcp);
	if (status < 0)
		goto fail;

	if (mcp->irq && mcp->irq_controller) {
		status = mcp23s08_irq_setup(mcp);
		if (status) {
			goto fail;
		}
	}
fail:
	if (status < 0)
		dev_dbg(dev, "can't setup chip %d, --> %d\n",
			addr, status);
	return status;
}

/*----------------------------------------------------------------------*/

static int mcp230xx_probe(struct i2c_client *client,
				    const struct i2c_device_id *id)
{
	struct mcp23s08_platform_data *pdata, local_pdata;
	struct mcp23s08 *mcp;
	int status;
	const struct of_device_id *match;

	printk(KERN_INFO "Start probing mcp230xx module.\n");

	match = of_match_device(of_match_ptr(mcp23s08_i2c_of_match),
					&client->dev);
	if (match) {
		pdata = &local_pdata;
		pdata->base = -1;
		pdata->chip[0].pullups = 0;
		pdata->irq_controller =	of_property_read_bool(
					client->dev.of_node,
					"interrupt-controller");
		pdata->mirror = of_property_read_bool(client->dev.of_node,
						      "microchip,irq-mirror");
		client->irq = irq_of_parse_and_map(client->dev.of_node, 0);
	} else {
		pdata = dev_get_platdata(&client->dev);
		if (!pdata) {
			pdata = devm_kzalloc(&client->dev,
					sizeof(struct mcp23s08_platform_data),
					GFP_KERNEL);
			if (!pdata)
				return -ENOMEM;
			pdata->base = -1;
		}
	}

	mcp = kzalloc(sizeof(*mcp), GFP_KERNEL);
	if (!mcp)
		return -ENOMEM;

	mcp->irq = client->irq;
	status = mcp23s08_probe_one(mcp, &client->dev, client, client->addr,
				    id->driver_data, pdata, 0);
	if (status)
		goto fail;

	printk(KERN_INFO "Succesfully probed mcp230xx module.\n");

	i2c_set_clientdata(client, mcp);

	return 0;

fail:
	printk(KERN_INFO "Failed to probe mcp230xx module.\n");
	kfree(mcp);

	return status;
}

static int mcp230xx_remove(struct i2c_client *client)
{
	struct mcp23s08 *mcp = i2c_get_clientdata(client);
	printk(KERN_INFO "Removed  mcp230xx module.\n");
	gpiochip_remove(&mcp->chip);
	kfree(mcp);

	return 0;
}

static const struct i2c_device_id mcp230xx_id[] = {
	{ "mcp23008", MCP_TYPE_008 },
	{ "mcp23017", MCP_TYPE_017 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, mcp230xx_id);

static struct i2c_driver mcp230xx_driver = {
	.driver = {
		.name	= "mcp230xx",
		.of_match_table = of_match_ptr(mcp23s08_i2c_of_match),
	},
	.probe		= mcp230xx_probe,
	.remove		= mcp230xx_remove,
	.id_table	= mcp230xx_id,
};

static int __init mcp23s08_i2c_init(void)
{
	return i2c_add_driver(&mcp230xx_driver);
}

static void mcp23s08_i2c_exit(void)
{
	i2c_del_driver(&mcp230xx_driver);
}

/*----------------------------------------------------------------------*/

static int __init mcp23s08_init(void)
{
	int ret;
	printk(KERN_INFO "mcp230xx init.\n");
	ret = mcp23s08_i2c_init();
	if (ret)
		goto i2c_fail;

	return 0;
i2c_fail:
	printk(KERN_INFO "Failed to init mcp230xx module.\n");
	mcp23s08_i2c_exit();
	return ret;
}
/* register after spi/i2c postcore initcall and before
 * subsys initcalls that may rely on these GPIOs
 */
subsys_initcall(mcp23s08_init);

static void __exit mcp23s08_exit(void)
{
	printk(KERN_INFO "Exited mcp230xx module.\n");
	mcp23s08_i2c_exit();
}
module_exit(mcp23s08_exit);

MODULE_LICENSE("GPL");
