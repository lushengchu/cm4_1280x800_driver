// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2012 Simon Budig, <simon.budig@kernelconcepts.de>
 * Daniel Wagener <daniel.wagener@kernelconcepts.de> (M09 firmware support)
 * Lothar Wa脽mann <LW@KARO-electronics.de> (DT support)
 */

/*
 * This is a driver for the EDT "Polytouch" family of touch controllers
 * based on the FocalTech FT5x06 line of chips.
 *
 * Development of this driver has been sponsored by Glyn:
 *    http://www.glyn.com/Products/Displays
 */

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/ratelimit.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/random.h>

#include <asm/unaligned.h>

#define WORK_REGISTER_THRESHOLD		0x00
#define WORK_REGISTER_REPORT_RATE	0x08
#define WORK_REGISTER_GAIN		0x30
#define WORK_REGISTER_OFFSET		0x31
#define WORK_REGISTER_NUM_X		0x33
#define WORK_REGISTER_NUM_Y		0x34

#define PMOD_REGISTER_ACTIVE		0x00
#define PMOD_REGISTER_HIBERNATE		0x03

#define M09_REGISTER_THRESHOLD		0x80
#define M09_REGISTER_GAIN		0x92
#define M09_REGISTER_OFFSET		0x93
#define M09_REGISTER_NUM_X		0x94
#define M09_REGISTER_NUM_Y		0x95

#define EV_REGISTER_THRESHOLD		0x40
#define EV_REGISTER_GAIN		0x41
#define EV_REGISTER_OFFSET_Y		0x45
#define EV_REGISTER_OFFSET_X		0x46

#define NO_REGISTER			0xff

#define WORK_REGISTER_OPMODE		0x3c
#define FACTORY_REGISTER_OPMODE		0x01
#define PMOD_REGISTER_OPMODE		0xa5

#define TOUCH_EVENT_DOWN		0x00
#define TOUCH_EVENT_UP			0x01
#define TOUCH_EVENT_ON			0x02
#define TOUCH_EVENT_RESERVED		0x03

#define EDT_NAME_LEN			23
#define EDT_SWITCH_MODE_RETRIES		10
#define EDT_SWITCH_MODE_DELAY		5 /* msec */
#define EDT_RAW_DATA_RETRIES		100
#define EDT_RAW_DATA_DELAY		1000 /* usec */

#define POLL_INTERVAL_MS		17	/* 17ms = 60fps */

enum edt_pmode {
	EDT_PMODE_NOT_SUPPORTED,
	EDT_PMODE_HIBERNATE,
	EDT_PMODE_POWEROFF,
};

enum edt_ver {
	EDT_M06,
	EDT_M09,
	EDT_M12,
	EV_FT,
	GENERIC_FT,
};

struct edt_reg_addr {
	int reg_threshold;
	int reg_report_rate;
	int reg_gain;
	int reg_offset;
	int reg_offset_x;
	int reg_offset_y;
	int reg_num_x;
	int reg_num_y;
};

struct edt_ft5x06_ts_data {
	struct i2c_client *client;
	struct input_dev *input;
	struct touchscreen_properties prop;
	u16 num_x;
	u16 num_y;
	struct regulator *vcc;

	struct gpio_desc *reset_gpio;
	struct gpio_desc *wake_gpio;

#if defined(CONFIG_DEBUG_FS)
	struct dentry *debug_dir;
	u8 *raw_buffer;
	size_t raw_bufsize;
#endif

	struct mutex mutex;
	bool factory_mode;
	enum edt_pmode suspend_mode;
	int threshold;
	int gain;
	int offset;
	int offset_x;
	int offset_y;
	int report_rate;
	int max_support_points;
	unsigned int known_ids;

	char name[EDT_NAME_LEN];

	struct edt_reg_addr reg_addr;
	enum edt_ver version;

	struct timer_list timer;
	struct work_struct work_i2c_poll;
};

struct edt_i2c_chip_data {
	int  max_support_points;
};

static int screen_max_x = 1280;
static int screen_max_y = 800;

static int edt_ft5x06_ts_readwrite(struct i2c_client *client,
				   u16 wr_len, u8 *wr_buf,
				   u16 rd_len, u8 *rd_buf)
{
	struct i2c_msg wrmsg[2];
	int i = 0;
	int ret;

    client->addr = 0x38;
	if (wr_len) {
		wrmsg[i].addr  = client->addr;
		wrmsg[i].flags = 0;
		wrmsg[i].len = wr_len;
		wrmsg[i].buf = wr_buf;
		i++;
	}
	if (rd_len) {
		wrmsg[i].addr  = client->addr;
		wrmsg[i].flags = I2C_M_RD;
		wrmsg[i].len = rd_len;
		wrmsg[i].buf = rd_buf;
		i++;
	}

	ret = i2c_transfer(client->adapter, wrmsg, i);
	if (ret < 0)
		return ret;
	if (ret != i)
		return -EIO;

	return 0;
}

static bool edt_ft5x06_ts_check_crc(struct edt_ft5x06_ts_data *tsdata,
				    u8 *buf, int buflen)
{
	int i;
	u8 crc = 0;

	for (i = 0; i < buflen - 1; i++)
		crc ^= buf[i];

	if (crc != buf[buflen-1]) {
		dev_err_ratelimited(&tsdata->client->dev,
				    "crc error: 0x%02x expected, got 0x%02x\n",
				    crc, buf[buflen-1]);
		return false;
	}

	return true;
}

static unsigned int test_num = 100;
static irqreturn_t edt_ft5x06_ts_isr(int irq, void *dev_id)
{
	struct edt_ft5x06_ts_data *tsdata = dev_id;
	struct device *dev = &tsdata->client->dev;
	u8 cmd;
	u8 rdbuf[63];
	int i, type, x, y, id;
	int offset, tplen, datalen, crclen;
	int error;
	unsigned int active_ids = 0, known_ids = tsdata->known_ids;
	long released_ids;
	int b = 0;
	unsigned int num_points;

    tsdata->version = GENERIC_FT;
	switch (tsdata->version) {
	case EDT_M06:
		cmd = 0xf9; /* tell the controller to send touch data */
		offset = 5; /* where the actual touch data starts */
		tplen = 4;  /* data comes in so called frames */
		crclen = 1; /* length of the crc data */
		break;

	case EDT_M09:
	case EDT_M12:
	case EV_FT:
	case GENERIC_FT:
		cmd = 0x0;
		offset = 3;
		tplen = 6;
		crclen = 0;
		break;

	default:
		goto out;
	}

	memset(rdbuf, 0, sizeof(rdbuf));
	datalen = tplen * tsdata->max_support_points + offset + crclen;

	error = edt_ft5x06_ts_readwrite(tsdata->client,
					sizeof(cmd), &cmd,
					datalen, rdbuf);
	if (error) {
	//	dev_err_ratelimited(dev, "Unable to fetch data, error: %d\n",
	//			    error);
		goto out;
	}

	/* M09/M12 does not send header or CRC */
	if (tsdata->version == EDT_M06) {
		if (rdbuf[0] != 0xaa || rdbuf[1] != 0xaa ||
			rdbuf[2] != datalen) {
			dev_err_ratelimited(dev,
					"Unexpected header: %02x%02x%02x!\n",
					rdbuf[0], rdbuf[1], rdbuf[2]);
			goto out;
		}

		if (!edt_ft5x06_ts_check_crc(tsdata, rdbuf, datalen))
			goto out;
		num_points = tsdata->max_support_points;
	} else {
		/* Register 2 is TD_STATUS, containing the number of touch
		 * points.
		 */
		num_points = min(rdbuf[2] & 0xf, tsdata->max_support_points);
	}
    if (num_points != test_num) {
        //printk("num_points=%d, offset=%d tplen=%d\n", num_points, offset, tplen);
    }
    test_num = num_points;
	for (i = 0; i < num_points; i++) {
		u8 *buf = &rdbuf[i * tplen + offset];

		type = buf[0] >> 6;
        //printk("type=0x%02x\n", type);
		/* ignore Reserved events */
		if (type == TOUCH_EVENT_RESERVED)
			continue;

		/* M06 sometimes sends bogus coordinates in TOUCH_DOWN */
		if (tsdata->version == EDT_M06 && type == TOUCH_EVENT_DOWN)
			continue;

		x = get_unaligned_be16(buf) & 0x0fff;
		y = get_unaligned_be16(buf + 2) & 0x0fff;
		/* The FT5x26 send the y coordinate first */
		if (tsdata->version == EV_FT)
			swap(x, y);

		id = (buf[2] >> 4) & 0x0f;

		input_mt_slot(tsdata->input, id);
		if (input_mt_report_slot_state(tsdata->input, MT_TOOL_FINGER,
					       type != TOUCH_EVENT_UP)) {
			touchscreen_report_pos(tsdata->input, &tsdata->prop,
					       x, y, true);
            //printk("buf=0x%02x offset=%d i=%d xy=%d,%d id=%d\n", buf[0], i * tplen + offset, i, x, y, id);
			active_ids |= BIT(id);
		} else {
			known_ids &= ~BIT(id);
		}
	}

	/* One issue with the device is the TOUCH_UP message is not always
	 * returned. Instead track which ids we know about and report when they
	 * are no longer updated
	 */
	released_ids = known_ids & ~active_ids;
	for_each_set_bit_from(b, &released_ids, tsdata->max_support_points) {
		input_mt_slot(tsdata->input, b);
		input_mt_report_slot_inactive(tsdata->input);
	}
	tsdata->known_ids = active_ids;

	input_mt_report_pointer_emulation(tsdata->input, true);
	input_sync(tsdata->input);

out:
	return IRQ_HANDLED;
}

static void edt_ft5x06_ts_irq_poll_timer(struct timer_list *t)
{
	struct edt_ft5x06_ts_data *tsdata = from_timer(tsdata, t, timer);

	schedule_work(&tsdata->work_i2c_poll);
	mod_timer(&tsdata->timer, jiffies + msecs_to_jiffies(POLL_INTERVAL_MS));
}

static void edt_ft5x06_ts_work_i2c_poll(struct work_struct *work)
{
	struct edt_ft5x06_ts_data *tsdata = container_of(work,
			struct edt_ft5x06_ts_data, work_i2c_poll);

	edt_ft5x06_ts_isr(0, tsdata);
}

static int edt_ft5x06_register_write(struct edt_ft5x06_ts_data *tsdata,
				     u8 addr, u8 value)
{
	u8 wrbuf[4];

	switch (tsdata->version) {
	case EDT_M06:
		wrbuf[0] = tsdata->factory_mode ? 0xf3 : 0xfc;
		wrbuf[1] = tsdata->factory_mode ? addr & 0x7f : addr & 0x3f;
		wrbuf[2] = value;
		wrbuf[3] = wrbuf[0] ^ wrbuf[1] ^ wrbuf[2];
		return edt_ft5x06_ts_readwrite(tsdata->client, 4,
					wrbuf, 0, NULL);

	case EDT_M09:
	case EDT_M12:
	case EV_FT:
	case GENERIC_FT:
		wrbuf[0] = addr;
		wrbuf[1] = value;

		return edt_ft5x06_ts_readwrite(tsdata->client, 2,
					wrbuf, 0, NULL);

	default:
		return -EINVAL;
	}
}

static int edt_ft5x06_register_read(struct edt_ft5x06_ts_data *tsdata,
				    u8 addr)
{
	u8 wrbuf[2], rdbuf[2];
	int error;

	switch (tsdata->version) {
	case EDT_M06:
		wrbuf[0] = tsdata->factory_mode ? 0xf3 : 0xfc;
		wrbuf[1] = tsdata->factory_mode ? addr & 0x7f : addr & 0x3f;
		wrbuf[1] |= tsdata->factory_mode ? 0x80 : 0x40;

		error = edt_ft5x06_ts_readwrite(tsdata->client, 2, wrbuf, 2,
						rdbuf);
		if (error)
			return error;

		if ((wrbuf[0] ^ wrbuf[1] ^ rdbuf[0]) != rdbuf[1]) {
			dev_err(&tsdata->client->dev,
				"crc error: 0x%02x expected, got 0x%02x\n",
				wrbuf[0] ^ wrbuf[1] ^ rdbuf[0],
				rdbuf[1]);
			return -EIO;
		}
		break;

	case EDT_M09:
	case EDT_M12:
	case EV_FT:
	case GENERIC_FT:
		wrbuf[0] = addr;
		error = edt_ft5x06_ts_readwrite(tsdata->client, 1,
						wrbuf, 1, rdbuf);
		if (error)
			return error;
		break;

	default:
		return -EINVAL;
	}

	return rdbuf[0];
}

struct edt_ft5x06_attribute {
	struct device_attribute dattr;
	size_t field_offset;
	u8 limit_low;
	u8 limit_high;
	u8 addr_m06;
	u8 addr_m09;
	u8 addr_ev;
};

#define EDT_ATTR(_field, _mode, _addr_m06, _addr_m09, _addr_ev,		\
		_limit_low, _limit_high)				\
	struct edt_ft5x06_attribute edt_ft5x06_attr_##_field = {	\
		.dattr = __ATTR(_field, _mode,				\
				edt_ft5x06_setting_show,		\
				edt_ft5x06_setting_store),		\
		.field_offset = offsetof(struct edt_ft5x06_ts_data, _field), \
		.addr_m06 = _addr_m06,					\
		.addr_m09 = _addr_m09,					\
		.addr_ev  = _addr_ev,					\
		.limit_low = _limit_low,				\
		.limit_high = _limit_high,				\
	}

static ssize_t edt_ft5x06_setting_show(struct device *dev,
				       struct device_attribute *dattr,
				       char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct edt_ft5x06_ts_data *tsdata = i2c_get_clientdata(client);
	struct edt_ft5x06_attribute *attr =
			container_of(dattr, struct edt_ft5x06_attribute, dattr);
	u8 *field = (u8 *)tsdata + attr->field_offset;
	int val;
	size_t count = 0;
	int error = 0;
	u8 addr;

	mutex_lock(&tsdata->mutex);

	if (tsdata->factory_mode) {
		error = -EIO;
		goto out;
	}

	switch (tsdata->version) {
	case EDT_M06:
		addr = attr->addr_m06;
		break;

	case EDT_M09:
	case EDT_M12:
	case GENERIC_FT:
		addr = attr->addr_m09;
		break;

	case EV_FT:
		addr = attr->addr_ev;
		break;

	default:
		error = -ENODEV;
		goto out;
	}

	if (addr != NO_REGISTER) {
		val = edt_ft5x06_register_read(tsdata, addr);
		if (val < 0) {
			error = val;
			dev_err(&tsdata->client->dev,
				"Failed to fetch attribute %s, error %d\n",
				dattr->attr.name, error);
			goto out;
		}
	} else {
		val = *field;
	}

	if (val != *field) {
		dev_warn(&tsdata->client->dev,
			 "%s: read (%d) and stored value (%d) differ\n",
			 dattr->attr.name, val, *field);
		*field = val;
	}

	count = scnprintf(buf, PAGE_SIZE, "%d\n", val);
out:
	mutex_unlock(&tsdata->mutex);
	return error ?: count;
}

static ssize_t edt_ft5x06_setting_store(struct device *dev,
					struct device_attribute *dattr,
					const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct edt_ft5x06_ts_data *tsdata = i2c_get_clientdata(client);
	struct edt_ft5x06_attribute *attr =
			container_of(dattr, struct edt_ft5x06_attribute, dattr);
	u8 *field = (u8 *)tsdata + attr->field_offset;
	unsigned int val;
	int error;
	u8 addr;

	mutex_lock(&tsdata->mutex);

	if (tsdata->factory_mode) {
		error = -EIO;
		goto out;
	}

	error = kstrtouint(buf, 0, &val);
	if (error)
		goto out;

	if (val < attr->limit_low || val > attr->limit_high) {
		error = -ERANGE;
		goto out;
	}

	switch (tsdata->version) {
	case EDT_M06:
		addr = attr->addr_m06;
		break;

	case EDT_M09:
	case EDT_M12:
	case GENERIC_FT:
		addr = attr->addr_m09;
		break;

	case EV_FT:
		addr = attr->addr_ev;
		break;

	default:
		error = -ENODEV;
		goto out;
	}

	if (addr != NO_REGISTER) {
		error = edt_ft5x06_register_write(tsdata, addr, val);
		if (error) {
			dev_err(&tsdata->client->dev,
				"Failed to update attribute %s, error: %d\n",
				dattr->attr.name, error);
			goto out;
		}
	}
	*field = val;

out:
	mutex_unlock(&tsdata->mutex);
	return error ?: count;
}

/* m06, m09: range 0-31, m12: range 0-5 */
static EDT_ATTR(gain, S_IWUSR | S_IRUGO, WORK_REGISTER_GAIN,
		M09_REGISTER_GAIN, EV_REGISTER_GAIN, 0, 31);
/* m06, m09: range 0-31, m12: range 0-16 */
static EDT_ATTR(offset, S_IWUSR | S_IRUGO, WORK_REGISTER_OFFSET,
		M09_REGISTER_OFFSET, NO_REGISTER, 0, 31);
/* m06, m09, m12: no supported, ev_ft: range 0-80 */
static EDT_ATTR(offset_x, S_IWUSR | S_IRUGO, NO_REGISTER, NO_REGISTER,
		EV_REGISTER_OFFSET_X, 0, 80);
/* m06, m09, m12: no supported, ev_ft: range 0-80 */
static EDT_ATTR(offset_y, S_IWUSR | S_IRUGO, NO_REGISTER, NO_REGISTER,
		EV_REGISTER_OFFSET_Y, 0, 80);
/* m06: range 20 to 80, m09: range 0 to 30, m12: range 1 to 255... */
static EDT_ATTR(threshold, S_IWUSR | S_IRUGO, WORK_REGISTER_THRESHOLD,
		M09_REGISTER_THRESHOLD, EV_REGISTER_THRESHOLD, 0, 255);
/* m06: range 3 to 14, m12: (0x64: 100Hz) */
static EDT_ATTR(report_rate, S_IWUSR | S_IRUGO, WORK_REGISTER_REPORT_RATE,
		NO_REGISTER, NO_REGISTER, 0, 255);

static struct attribute *edt_ft5x06_attrs[] = {
	&edt_ft5x06_attr_gain.dattr.attr,
	&edt_ft5x06_attr_offset.dattr.attr,
	&edt_ft5x06_attr_offset_x.dattr.attr,
	&edt_ft5x06_attr_offset_y.dattr.attr,
	&edt_ft5x06_attr_threshold.dattr.attr,
	&edt_ft5x06_attr_report_rate.dattr.attr,
	NULL
};

static const struct attribute_group edt_ft5x06_attr_group = {
	.attrs = edt_ft5x06_attrs,
};

static void edt_ft5x06_restore_reg_parameters(struct edt_ft5x06_ts_data *tsdata)
{
	struct edt_reg_addr *reg_addr = &tsdata->reg_addr;

	edt_ft5x06_register_write(tsdata, reg_addr->reg_threshold,
				  tsdata->threshold);
	edt_ft5x06_register_write(tsdata, reg_addr->reg_gain,
				  tsdata->gain);
	if (reg_addr->reg_offset != NO_REGISTER)
		edt_ft5x06_register_write(tsdata, reg_addr->reg_offset,
					  tsdata->offset);
	if (reg_addr->reg_offset_x != NO_REGISTER)
		edt_ft5x06_register_write(tsdata, reg_addr->reg_offset_x,
					  tsdata->offset_x);
	if (reg_addr->reg_offset_y != NO_REGISTER)
		edt_ft5x06_register_write(tsdata, reg_addr->reg_offset_y,
					  tsdata->offset_y);
	if (reg_addr->reg_report_rate != NO_REGISTER)
		edt_ft5x06_register_write(tsdata, reg_addr->reg_report_rate,
				  tsdata->report_rate);

}

#ifdef CONFIG_DEBUG_FS
static int edt_ft5x06_factory_mode(struct edt_ft5x06_ts_data *tsdata)
{
	struct i2c_client *client = tsdata->client;
	int retries = EDT_SWITCH_MODE_RETRIES;
	int ret;
	int error;

	if (tsdata->version != EDT_M06) {
		dev_err(&client->dev,
			"No factory mode support for non-M06 devices\n");
		return -EINVAL;
	}

	disable_irq(client->irq);

	if (!tsdata->raw_buffer) {
		tsdata->raw_bufsize = tsdata->num_x * tsdata->num_y *
				      sizeof(u16);
		tsdata->raw_buffer = kzalloc(tsdata->raw_bufsize, GFP_KERNEL);
		if (!tsdata->raw_buffer) {
			error = -ENOMEM;
			goto err_out;
		}
	}

	/* mode register is 0x3c when in the work mode */
	error = edt_ft5x06_register_write(tsdata, WORK_REGISTER_OPMODE, 0x03);
	if (error) {
		dev_err(&client->dev,
			"failed to switch to factory mode, error %d\n", error);
		goto err_out;
	}

	tsdata->factory_mode = true;
	do {
		mdelay(EDT_SWITCH_MODE_DELAY);
		/* mode register is 0x01 when in factory mode */
		ret = edt_ft5x06_register_read(tsdata, FACTORY_REGISTER_OPMODE);
		if (ret == 0x03)
			break;
	} while (--retries > 0);

	if (retries == 0) {
		dev_err(&client->dev, "not in factory mode after %dms.\n",
			EDT_SWITCH_MODE_RETRIES * EDT_SWITCH_MODE_DELAY);
		error = -EIO;
		goto err_out;
	}

	return 0;

err_out:
	kfree(tsdata->raw_buffer);
	tsdata->raw_buffer = NULL;
	tsdata->factory_mode = false;
	enable_irq(client->irq);

	return error;
}

static int edt_ft5x06_work_mode(struct edt_ft5x06_ts_data *tsdata)
{
	struct i2c_client *client = tsdata->client;
	int retries = EDT_SWITCH_MODE_RETRIES;
	int ret;
	int error;

	/* mode register is 0x01 when in the factory mode */
	error = edt_ft5x06_register_write(tsdata, FACTORY_REGISTER_OPMODE, 0x1);
	if (error) {
		dev_err(&client->dev,
			"failed to switch to work mode, error: %d\n", error);
		return error;
	}

	tsdata->factory_mode = false;

	do {
		mdelay(EDT_SWITCH_MODE_DELAY);
		/* mode register is 0x01 when in factory mode */
		ret = edt_ft5x06_register_read(tsdata, WORK_REGISTER_OPMODE);
		if (ret == 0x01)
			break;
	} while (--retries > 0);

	if (retries == 0) {
		dev_err(&client->dev, "not in work mode after %dms.\n",
			EDT_SWITCH_MODE_RETRIES * EDT_SWITCH_MODE_DELAY);
		tsdata->factory_mode = true;
		return -EIO;
	}

	kfree(tsdata->raw_buffer);
	tsdata->raw_buffer = NULL;

	edt_ft5x06_restore_reg_parameters(tsdata);
	enable_irq(client->irq);

	return 0;
}

static int edt_ft5x06_debugfs_mode_get(void *data, u64 *mode)
{
	struct edt_ft5x06_ts_data *tsdata = data;

	*mode = tsdata->factory_mode;

	return 0;
};

static int edt_ft5x06_debugfs_mode_set(void *data, u64 mode)
{
	struct edt_ft5x06_ts_data *tsdata = data;
	int retval = 0;

	if (mode > 1)
		return -ERANGE;

	mutex_lock(&tsdata->mutex);

	if (mode != tsdata->factory_mode) {
		retval = mode ? edt_ft5x06_factory_mode(tsdata) :
				edt_ft5x06_work_mode(tsdata);
	}

	mutex_unlock(&tsdata->mutex);

	return retval;
};

DEFINE_SIMPLE_ATTRIBUTE(debugfs_mode_fops, edt_ft5x06_debugfs_mode_get,
			edt_ft5x06_debugfs_mode_set, "%llu\n");

static ssize_t edt_ft5x06_debugfs_raw_data_read(struct file *file,
				char __user *buf, size_t count, loff_t *off)
{
	struct edt_ft5x06_ts_data *tsdata = file->private_data;
	struct i2c_client *client = tsdata->client;
	int retries  = EDT_RAW_DATA_RETRIES;
	int val, i, error;
	size_t read = 0;
	int colbytes;
	char wrbuf[3];
	u8 *rdbuf;

	if (*off < 0 || *off >= tsdata->raw_bufsize)
		return 0;

	mutex_lock(&tsdata->mutex);

	if (!tsdata->factory_mode || !tsdata->raw_buffer) {
		error = -EIO;
		goto out;
	}

	error = edt_ft5x06_register_write(tsdata, 0x08, 0x01);
	if (error) {
		dev_dbg(&client->dev,
			"failed to write 0x08 register, error %d\n", error);
		goto out;
	}

	do {
		usleep_range(EDT_RAW_DATA_DELAY, EDT_RAW_DATA_DELAY + 100);
		val = edt_ft5x06_register_read(tsdata, 0x08);
		if (val < 1)
			break;
	} while (--retries > 0);

	if (val < 0) {
		error = val;
		dev_dbg(&client->dev,
			"failed to read 0x08 register, error %d\n", error);
		goto out;
	}

	if (retries == 0) {
		dev_dbg(&client->dev,
			"timed out waiting for register to settle\n");
		error = -ETIMEDOUT;
		goto out;
	}

	rdbuf = tsdata->raw_buffer;
	colbytes = tsdata->num_y * sizeof(u16);

	wrbuf[0] = 0xf5;
	wrbuf[1] = 0x0e;
	for (i = 0; i < tsdata->num_x; i++) {
		wrbuf[2] = i;  /* column index */
		error = edt_ft5x06_ts_readwrite(tsdata->client,
						sizeof(wrbuf), wrbuf,
						colbytes, rdbuf);
		if (error)
			goto out;

		rdbuf += colbytes;
	}

	read = min_t(size_t, count, tsdata->raw_bufsize - *off);
	if (copy_to_user(buf, tsdata->raw_buffer + *off, read)) {
		error = -EFAULT;
		goto out;
	}

	*off += read;
out:
	mutex_unlock(&tsdata->mutex);
	return error ?: read;
};

static const struct file_operations debugfs_raw_data_fops = {
	.open = simple_open,
	.read = edt_ft5x06_debugfs_raw_data_read,
};

static void edt_ft5x06_ts_prepare_debugfs(struct edt_ft5x06_ts_data *tsdata,
					  const char *debugfs_name)
{
	tsdata->debug_dir = debugfs_create_dir(debugfs_name, NULL);

	debugfs_create_u16("num_x", S_IRUSR, tsdata->debug_dir, &tsdata->num_x);
	debugfs_create_u16("num_y", S_IRUSR, tsdata->debug_dir, &tsdata->num_y);

	debugfs_create_file("mode", S_IRUSR | S_IWUSR,
			    tsdata->debug_dir, tsdata, &debugfs_mode_fops);
	debugfs_create_file("raw_data", S_IRUSR,
			    tsdata->debug_dir, tsdata, &debugfs_raw_data_fops);
}

static void edt_ft5x06_ts_teardown_debugfs(struct edt_ft5x06_ts_data *tsdata)
{
	debugfs_remove_recursive(tsdata->debug_dir);
	kfree(tsdata->raw_buffer);
}

#else

static int edt_ft5x06_factory_mode(struct edt_ft5x06_ts_data *tsdata)
{
	return -ENOSYS;
}

static void edt_ft5x06_ts_prepare_debugfs(struct edt_ft5x06_ts_data *tsdata,
					  const char *debugfs_name)
{
}

static void edt_ft5x06_ts_teardown_debugfs(struct edt_ft5x06_ts_data *tsdata)
{
}

#endif /* CONFIG_DEBUGFS */


static void edt_ft5x06_ts_get_defaults(struct device *dev,
				       struct edt_ft5x06_ts_data *tsdata)
{
	struct edt_reg_addr *reg_addr = &tsdata->reg_addr;
	u32 val;
	int error;

	error = device_property_read_u32(dev, "threshold", &val);
	if (!error) {
		edt_ft5x06_register_write(tsdata, reg_addr->reg_threshold, val);
		tsdata->threshold = val;
	}

	error = device_property_read_u32(dev, "gain", &val);
	if (!error) {
		edt_ft5x06_register_write(tsdata, reg_addr->reg_gain, val);
		tsdata->gain = val;
	}

	error = device_property_read_u32(dev, "offset", &val);
	if (!error) {
		if (reg_addr->reg_offset != NO_REGISTER)
			edt_ft5x06_register_write(tsdata,
						  reg_addr->reg_offset, val);
		tsdata->offset = val;
	}

	error = device_property_read_u32(dev, "offset-x", &val);
	if (!error) {
		if (reg_addr->reg_offset_x != NO_REGISTER)
			edt_ft5x06_register_write(tsdata,
						  reg_addr->reg_offset_x, val);
		tsdata->offset_x = val;
	}

	error = device_property_read_u32(dev, "offset-y", &val);
	if (!error) {
		if (reg_addr->reg_offset_y != NO_REGISTER)
			edt_ft5x06_register_write(tsdata,
						  reg_addr->reg_offset_y, val);
		tsdata->offset_y = val;
	}
}

static int generate_crc(uint8_t *w_buf, uint8_t *r_buf) {
    int i;
    unsigned char dest_buf[8];
    unsigned int w_sum1 = 0, w_sum2 = 0;
    unsigned int r_sum1, r_sum2;

    for (i = 0; i < 4; i++) {//先计算w_buf,和r_buf中计较是不是一致
        uint8_t src = w_buf[i];
        if (src < 12) {
            dest_buf[i*2] = (src*76) >> 2;
            dest_buf[i*2+1] = (src*76) & 0xff;
        } else if (src < 35) {
            dest_buf[i*2] = (src*35) >> 3;
            dest_buf[i*2+1] = (src*35) & 0xff;
        } else if (src < 58) {
            dest_buf[i*2] = (src*8) >> 1;
            dest_buf[i*2+1] = (src*8) & 0xff;
        } else if (src < 85) {
            dest_buf[i*2] = (src*35) >> 3;
            dest_buf[i*2+1] = (src*35) & 0xff;
        } else if (src < 115) {
            dest_buf[i*2] = (src*92) >> 4;
            dest_buf[i*2+1] = (src*92) & 0xff;
        } else if (src < 150) {
            dest_buf[i*2] = (src*12) >> 2;
            dest_buf[i*2+1] = (src*12) & 0xff;
        } else if (src < 201) {
            dest_buf[i*2] = (src*21) >> 6;
            dest_buf[i*2+1] = (src*21) & 0xff;
        } else {
            dest_buf[i*2] = (src*6) >> 1;
            dest_buf[i*2+1] = (src*6) & 0xff;
        }
    }

    w_sum1 += dest_buf[0]*dest_buf[1];
    w_sum1 += dest_buf[0]*dest_buf[2];
    w_sum1 += dest_buf[0]*dest_buf[3];
    w_sum1 += dest_buf[1]*dest_buf[2];
    w_sum1 *= 65521;

    w_sum2 += dest_buf[1]*dest_buf[2];
    w_sum2 += dest_buf[2]*dest_buf[3];
    w_sum2 += dest_buf[3]*dest_buf[3];
    w_sum2 += dest_buf[1]*dest_buf[1];
    w_sum2 *= 32854;
#if 0
    printk("touch crc_data:");
    for (i = 0; i < 4; i++) {
        printk("%02x ", w_buf[i]);
    }
    printk("\n");
    printk("touch dest_buf:");
    for (i = 0; i < 8; i++) {
        printk("%02x ", dest_buf[i]);
    }
    printk("\n");
    printk("touch crc_result:");
    printk("%02x %02x %02x %02x", (w_sum1>>24)&0xff, (w_sum1>>16)&0xff, (w_sum1>>8)&0xff, (w_sum1>>0)&0xff);
    printk("%02x %02x %02x %02x", (w_sum2>>24)&0xff, (w_sum2>>16)&0xff, (w_sum2>>8)&0xff, (w_sum2>>0)&0xff);
    printk("\n");
#endif


    r_sum1 = (r_buf[0] << 24) | (r_buf[1] << 16) | (r_buf[2] << 8) | r_buf[3];
    r_sum2 = (r_buf[4] << 24) | (r_buf[5] << 16) | (r_buf[6] << 8) | r_buf[7];

#if 0
    printk("w_sum=%x %x, r_sum=%x %x\n", w_sum1, w_sum2, r_sum1, r_sum2);
#endif
    if((w_sum1 == r_sum1) && (w_sum2 == r_sum2)) {
        return 0;
    }
   
    return -1;
}

    static void
edt_ft5x06_ts_get_parameters(struct edt_ft5x06_ts_data *tsdata)
{
    uint8_t wrbuf[1], rdbuf[4];

    wrbuf[0] = 0xd1;
    edt_ft5x06_ts_readwrite(tsdata->client, 1, wrbuf, 4, rdbuf);
    //screen_max_x = rdbuf[0] << 8 | rdbuf[1];
    //screen_max_y = rdbuf[2] << 8 | rdbuf[3];
    printk("screen_max_x=%d screen_max_y=%d\n", screen_max_x, screen_max_y);
}

static void
edt_ft5x06_ts_set_regs(struct edt_ft5x06_ts_data *tsdata)
{
	struct edt_reg_addr *reg_addr = &tsdata->reg_addr;

	switch (tsdata->version) {
	case EDT_M06:
		reg_addr->reg_threshold = WORK_REGISTER_THRESHOLD;
		reg_addr->reg_report_rate = WORK_REGISTER_REPORT_RATE;
		reg_addr->reg_gain = WORK_REGISTER_GAIN;
		reg_addr->reg_offset = WORK_REGISTER_OFFSET;
		reg_addr->reg_offset_x = NO_REGISTER;
		reg_addr->reg_offset_y = NO_REGISTER;
		reg_addr->reg_num_x = WORK_REGISTER_NUM_X;
		reg_addr->reg_num_y = WORK_REGISTER_NUM_Y;
		break;

	case EDT_M09:
	case EDT_M12:
		reg_addr->reg_threshold = M09_REGISTER_THRESHOLD;
		reg_addr->reg_report_rate = NO_REGISTER;
		reg_addr->reg_gain = M09_REGISTER_GAIN;
		reg_addr->reg_offset = M09_REGISTER_OFFSET;
		reg_addr->reg_offset_x = NO_REGISTER;
		reg_addr->reg_offset_y = NO_REGISTER;
		reg_addr->reg_num_x = M09_REGISTER_NUM_X;
		reg_addr->reg_num_y = M09_REGISTER_NUM_Y;
		break;

	case EV_FT:
		reg_addr->reg_threshold = EV_REGISTER_THRESHOLD;
		reg_addr->reg_gain = EV_REGISTER_GAIN;
		reg_addr->reg_offset = NO_REGISTER;
		reg_addr->reg_offset_x = EV_REGISTER_OFFSET_X;
		reg_addr->reg_offset_y = EV_REGISTER_OFFSET_Y;
		reg_addr->reg_num_x = NO_REGISTER;
		reg_addr->reg_num_y = NO_REGISTER;
		reg_addr->reg_report_rate = NO_REGISTER;
		break;

	case GENERIC_FT:
		/* this is a guesswork */
		reg_addr->reg_threshold = M09_REGISTER_THRESHOLD;
		reg_addr->reg_gain = M09_REGISTER_GAIN;
		reg_addr->reg_offset = M09_REGISTER_OFFSET;
		reg_addr->reg_offset_x = NO_REGISTER;
		reg_addr->reg_offset_y = NO_REGISTER;
		break;
	}
}

static int edt_ft5x06_ts_probe(struct i2c_client *client,
					 const struct i2c_device_id *id)
{
	struct edt_ft5x06_ts_data *tsdata;
	u8 buf[2] = { 0xfc, 0x00 };
	struct input_dev *input;
	unsigned long irq_flags;
	int error;
	char fw_version[EDT_NAME_LEN];

	dev_dbg(&client->dev, "probing for EDT FT5x06 I2C\n");

	tsdata = devm_kzalloc(&client->dev, sizeof(*tsdata), GFP_KERNEL);
	if (!tsdata) {
		dev_err(&client->dev, "failed to allocate driver data.\n");
		return -ENOMEM;
	}

    tsdata->version = GENERIC_FT;
	tsdata->max_support_points = 5;

	input = devm_input_allocate_device(&client->dev);
	if (!input) {
		dev_err(&client->dev, "failed to allocate input device.\n");
		return -ENOMEM;
	}

	mutex_init(&tsdata->mutex);
	tsdata->client = client;
	tsdata->input = input;
	tsdata->factory_mode = false;

	input->name = "JMO-Touch";
	input->id.bustype = BUS_I2C;
	input->dev.parent = &client->dev;


    edt_ft5x06_ts_get_parameters(tsdata);
    /* Unknown maximum values. Specify via devicetree */
    input_set_abs_params(input, ABS_MT_POSITION_X,
            0, screen_max_x, 0, 0);
    input_set_abs_params(input, ABS_MT_POSITION_Y,
            0, screen_max_y, 0, 0);


	touchscreen_parse_properties(input, true, &tsdata->prop);

	error = input_mt_init_slots(input, tsdata->max_support_points,
				INPUT_MT_DIRECT);
	if (error) {
		dev_err(&client->dev, "Unable to init MT slots.\n");
		return error;
	}

	i2c_set_clientdata(client, tsdata);

	if (0) {
		irq_flags = irq_get_trigger_type(client->irq);
		if (irq_flags == IRQF_TRIGGER_NONE)
			irq_flags = IRQF_TRIGGER_FALLING;
		irq_flags |= IRQF_ONESHOT;

		error = devm_request_threaded_irq(&client->dev, client->irq,
						  NULL, edt_ft5x06_ts_isr,
						  irq_flags, client->name,
						  tsdata);
		if (error) {
			dev_err(&client->dev, "Unable to request touchscreen IRQ.\n");
			return error;
		}
	} else {
		INIT_WORK(&tsdata->work_i2c_poll,
			  edt_ft5x06_ts_work_i2c_poll);
		timer_setup(&tsdata->timer, edt_ft5x06_ts_irq_poll_timer, 0);
		tsdata->timer.expires = jiffies +
					msecs_to_jiffies(POLL_INTERVAL_MS);
		add_timer(&tsdata->timer);
	}

	error = devm_device_add_group(&client->dev, &edt_ft5x06_attr_group);
	if (error)
		return error;

	error = input_register_device(input);
	if (error)
		return error;

	edt_ft5x06_ts_prepare_debugfs(tsdata, dev_driver_string(&client->dev));

	printk(
		"EDT FT5x06 initialized: IRQ %d, WAKE pin %d, Reset pin %d.\n",
		client->irq,
		tsdata->wake_gpio ? desc_to_gpio(tsdata->wake_gpio) : -1,
		tsdata->reset_gpio ? desc_to_gpio(tsdata->reset_gpio) : -1);

	return 0;
}

static int edt_ft5x06_ts_remove(struct i2c_client *client)
{
	struct edt_ft5x06_ts_data *tsdata = i2c_get_clientdata(client);

	if (!client->irq) {
		del_timer(&tsdata->timer);
		cancel_work_sync(&tsdata->work_i2c_poll);
	}
	edt_ft5x06_ts_teardown_debugfs(tsdata);

	return 0;
}

static int __maybe_unused edt_ft5x06_ts_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct edt_ft5x06_ts_data *tsdata = i2c_get_clientdata(client);
	struct gpio_desc *reset_gpio = tsdata->reset_gpio;
	int ret;

	if (device_may_wakeup(dev))
		return 0;

	if (tsdata->suspend_mode == EDT_PMODE_NOT_SUPPORTED)
		return 0;

	/* Enter hibernate mode. */
	ret = edt_ft5x06_register_write(tsdata, PMOD_REGISTER_OPMODE,
					PMOD_REGISTER_HIBERNATE);
	if (ret)
		dev_warn(dev, "Failed to set hibernate mode\n");

	if (tsdata->suspend_mode == EDT_PMODE_HIBERNATE)
		return 0;

	/*
	 * Power-off according the datasheet. Cut the power may leaf the irq
	 * line in an undefined state depending on the host pull resistor
	 * settings. Disable the irq to avoid adjusting each host till the
	 * device is back in a full functional state.
	 */
	disable_irq(tsdata->client->irq);

	gpiod_set_value_cansleep(reset_gpio, 1);
	usleep_range(1000, 2000);

	ret = regulator_disable(tsdata->vcc);
	if (ret)
		dev_warn(dev, "Failed to disable vcc\n");

	return 0;
}

static int __maybe_unused edt_ft5x06_ts_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct edt_ft5x06_ts_data *tsdata = i2c_get_clientdata(client);
	int ret = 0;

	if (device_may_wakeup(dev))
		return 0;

	if (tsdata->suspend_mode == EDT_PMODE_NOT_SUPPORTED)
		return 0;


	return ret;
}

static SIMPLE_DEV_PM_OPS(edt_ft5x06_ts_pm_ops,
			 edt_ft5x06_ts_suspend, edt_ft5x06_ts_resume);

static const struct edt_i2c_chip_data edt_ft5x06_data = {
	.max_support_points = 5,
};

static const struct edt_i2c_chip_data edt_ft5506_data = {
	.max_support_points = 10,
};

static const struct edt_i2c_chip_data edt_ft6236_data = {
	.max_support_points = 2,
};

static const struct i2c_device_id edt_ft5x06_ts_id[] = {
	{ .name = "edt-ft5x06", .driver_data = (long)&edt_ft5x06_data },
	{ .name = "edt-ft5506", .driver_data = (long)&edt_ft5506_data },
	{ .name = "ev-ft5726", .driver_data = (long)&edt_ft5506_data },
	/* Note no edt- prefix for compatibility with the ft6236.c driver */
	{ .name = "ft6236", .driver_data = (long)&edt_ft6236_data },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, edt_ft5x06_ts_id);

static const struct of_device_id edt_ft5x06_of_match[] = {
	{ .compatible = "JMO_DSI1024x600_Touch_compatible", .data = &edt_ft5506_data },
	/* Note focaltech vendor prefix for compatibility with ft6236.c */
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, edt_ft5x06_of_match);

static struct i2c_driver edt_ft5x06_ts_driver = {
	.driver = {
		.name = "JMO_Touch",
		.of_match_table = edt_ft5x06_of_match,
		.pm = &edt_ft5x06_ts_pm_ops,
		//.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.probe_type = PROBE_FORCE_SYNCHRONOUS,
	},
	.id_table = edt_ft5x06_ts_id,
	.probe    = edt_ft5x06_ts_probe,
	.remove   = edt_ft5x06_ts_remove,
};

#if 0
static int __init init_cxl(void)
{
    return i2c_add_driver(&edt_ft5x06_ts_driver);
}
static void exit_cxl(void)
{
    i2c_del_driver(edt_ft5x06_ts_driver);

}
#endif
module_i2c_driver(edt_ft5x06_ts_driver);
//arch_initcall(init_cxl);
//module_init(init_cxl);
//module_exit(exit_cxl);

MODULE_AUTHOR("Simon Budig <simon.budig@kernelconcepts.de>");
MODULE_DESCRIPTION("EDT FT5x06 I2C Touchscreen Driver");
MODULE_LICENSE("GPL v2");
