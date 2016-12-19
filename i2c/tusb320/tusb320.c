#include <linux/kernel.h>
#include <linux/module.h>

#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/acpi.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>

#define dbg_info(fmt,...)    printk(KERN_CRIT "[TYPEC] %s "fmt"\n", __func__, ##__VA_ARGS__);

/* store reg info read by i2c*/
static unsigned char typec_info[3];
static unsigned char reg_base=0x08;

struct tusb320_i2c {
	struct i2c_client *client;
	spinlock_t lock;
	struct i2c_msg *msg;
	bool cable_dir;
#define P_SIDE    0
#define N_SIDE    1
	char attached_state;
#define NO_ATTACHED   0
#define ATTACH_SNK    1
#define ATTACH_SRC    2
#define ACCESSORY     3
	bool interrupt_status;
	unsigned gpio_cc_int;
	unsigned gpio_switch;
	unsigned int irq;
	struct i2c_adapter adap;
	struct device *dev;
};
static struct tusb320_i2c *i2c_dev;

static char *attached_state[4] = {
	"no_attached",
	"attach_device",
	"attach_host",
	"accessory"
};

static char *cable_dir[2] = {
	"positive",
	"opposite"
};

static unsigned int gpio_value;

/**
 * data_switch - switch data channel
 *
 * Hardware(HD3SS460) should switch data channel once detect device
 * pluging in opposite orientation, simply by write '1' to ISH_GPIO_13
 */
static void data_switch(int on)
{
	gpio_set_value(i2c_dev->gpio_switch, on);
}

static int update_typec_status(void)
{
	int rc;

	struct i2c_msg msgs[] = {
		{i2c_dev->client->addr, 0, 1, &reg_base},
		{i2c_dev->client->addr, I2C_M_RD, 3, typec_info}
	};

	if ((rc = i2c_transfer(i2c_dev->client->adapter, msgs, 2)) != 2) {
		dbg_info("i2c_transfer failed %d", rc);
		return rc;
	}

	//dbg_info(":%02x.%02x.%02x", typec_info[0], typec_info[1], typec_info[2]);

	i2c_dev->attached_state = (typec_info[1]>>6) & 0x3;
	i2c_dev->cable_dir = (typec_info[1]>>5) & 0x1;
	i2c_dev->interrupt_status = (typec_info[1]>>4) & 0x1;

	dbg_info(":%s %s\n", attached_state[i2c_dev->attached_state],
			cable_dir[i2c_dev->cable_dir]);

	return 0;
}

static int tusb320_i2c_send(unsigned char addr, char cmd)
{
	unsigned char txbuf[2] = {addr, cmd};

	struct i2c_msg msg = { .addr = i2c_dev->client->addr,
		.flags = 0,
		.len = 2,
		.buf = txbuf };

	return i2c_transfer(i2c_dev->client->adapter, &msg, 1);
}

static irqreturn_t tusb320_threadhandler(unsigned long num)
{
	int ret;
	unsigned char cmd;

	update_typec_status();

	if (i2c_dev->attached_state == NO_ATTACHED) {
		data_switch(0);
		goto done;
	}

	if (i2c_dev->cable_dir == N_SIDE) {
		data_switch(1);
	}

done:
	/* clear interrupt status by writing 1'b1 */
	cmd = typec_info[1] | 1<<4;
	ret = tusb320_i2c_send(0x09, cmd);

	return IRQ_HANDLED;
}

static irqreturn_t tusb320_irqhandler(int irqno, void *dev_id)
{
	return IRQ_WAKE_THREAD;
}

static ssize_t attached_state_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	update_typec_status();
	return scnprintf(buf, PAGE_SIZE, "%s\n",
			attached_state[i2c_dev->attached_state]);
}

static ssize_t cable_dir_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	update_typec_status();
	return scnprintf(buf, PAGE_SIZE, "%s\n",
			cable_dir[i2c_dev->cable_dir]);
}

static DEVICE_ATTR(attached_state, 0444, attached_state_show, NULL);
static DEVICE_ATTR(cable_dir, 0444, cable_dir_show, NULL);

static struct attribute *dev_attrs[] = {
	&dev_attr_attached_state.attr,
	&dev_attr_cable_dir.attr,
	NULL
};

static struct attribute_group dev_attr_grp = {
	.attrs = dev_attrs,
};

static void tusb320_init(void)
{
	char cmd;
	int ret;

	/* reset tusb320 by software */
	update_typec_status();
	cmd = typec_info[2] | 1<<3;
	msleep(10);
	ret = tusb320_i2c_send(0x0A, cmd);

	/* clear interrupt status by writing 1'b1 */
	msleep(10);
	if (i2c_dev->interrupt_status == 1) {
		cmd = typec_info[1] | 1<<4;
		ret = tusb320_i2c_send(0x09, cmd);
		dbg_info("clear interrupt_status cmd:0x%02x ret:%d", cmd, ret);
	}
}

static void tusb320_i2c_remove(struct i2c_client *client)
{
	return i2c_unregister_device(client);
}

static int tusb320_probe(struct i2c_client *client,
		const struct i2c_device_id *dev_id)
{
	struct tusb320_i2c *tusb320;
	struct gpio_desc *desc;
	int rc;

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_I2C_BLOCK)) {
		dbg_info("i2c bus does not support tusb320");
		rc = -ENODEV;
		goto exit;
	}

	rc = sysfs_create_group(&client->dev.kobj, &dev_attr_grp);
	if (rc < 0) {
		dbg_info("sysfs_create_group failed");
		goto exit;
	}

	tusb320 = kmalloc(sizeof(struct tusb320_i2c), GFP_KERNEL);
	if (!tusb320) {
		dbg_info("no memory for tusb320_i2c");
		return -ENOMEM;
	}

	memset(tusb320, 0, sizeof(struct tusb320_i2c));
	tusb320->client = client;
	tusb320->dev = &client->dev;

	i2c_dev = tusb320;

	desc = devm_gpiod_get_index(&client->dev, "switch", 1);
	if (!IS_ERR(desc)) {
		rc = gpiod_direction_output(desc, 1);
		if (rc)
			return rc;
		i2c_dev->gpio_switch = desc_to_gpio(desc);
		dbg_info("gpio_switch:%d", i2c_dev->gpio_switch);
	}

	desc = devm_gpiod_get_index(&client->dev, "cc_intr", 0);
	if (!IS_ERR(desc)) {
		i2c_dev->gpio_cc_int = desc_to_gpio(desc);
		dbg_info("gpio_cc_int:%d", i2c_dev->gpio_cc_int);
	}

	if (request_threaded_irq(gpio_to_irq(i2c_dev->gpio_cc_int),
				tusb320_irqhandler,
				tusb320_threadhandler,
				IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				"tusb320",
				dev_id)) {
		dbg_info("request_irq failed");
	}

	gpio_value = gpio_get_value(i2c_dev->gpio_switch);
	if (gpio_value) {
		data_switch(0);
	}

	tusb320_init();

	return 0;
exit:
	return rc;
}

static int tusb320_remove(struct i2c_client *client)
{
	i2c_unregister_device(client);
	return 0;
}

static struct i2c_device_id tusb320_id[] = {
	{ "TUSB320", 0 },
	{ }
};

static const struct acpi_device_id tusb320_acpi_match[] = {
	{ "TUSB320", 0 },
	{ }
};
MODULE_DEVICE_TABLE(acpi , tusb320_acpi_match);

static struct i2c_driver tusb320_i2c_driver = {
	.driver = {
		.name = "tusb320",
		.owner = THIS_MODULE,
		.acpi_match_table = ACPI_PTR(tusb320_acpi_match),
	},
	.probe = tusb320_probe,
	.remove = tusb320_remove,
	.id_table = tusb320_id,
};

module_i2c_driver(tusb320_i2c_driver);
