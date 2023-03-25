// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2016 InforceComputing
 * Author: Vinay Simha BN <simhavcs@gmail.com>
 *
 * Copyright (C) 2016 Linaro Ltd
 * Author: Sumit Semwal <sumit.semwal@linaro.org>
 *
 * From internet archives, the panel for Nexus 7 2nd Gen, 2013 model is a
 * JDI model LT070ME05000, and its data sheet is at:
 * http://panelone.net/en/7-0-inch/JDI_LT070ME05000_7.0_inch-datasheet
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/i2c.h>
#include <linux/random.h>

#include <video/mipi_display.h>

#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

static int Backlight = 255;
static struct i2c_client *client = NULL;

static const char * const regulator_names[] = {
	"vddp",
	"iovcc"
};

struct jdi_panel {
	struct drm_panel base;
	struct mipi_dsi_device *dsi;

	struct regulator_bulk_data supplies[ARRAY_SIZE(regulator_names)];

	struct gpio_desc *enable_gpio;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *dcdc_en_gpio;
	struct backlight_device *backlight;

	bool prepared;
	bool enabled;

	const struct drm_display_mode *mode;
};


static int i2c_readwrite(struct i2c_client *client,
				   u16 wr_len, u8 *wr_buf,
				   u16 rd_len, u8 *rd_buf)
{
	struct i2c_msg wrmsg[2];
	int i = 0;
	int ret;

    if (!client)
        return -1;

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

static void backlight_ctl(uint8_t brightness) {
    uint8_t bl_buf[2] = {0x86, brightness};//86寄存器，背光调节到最亮

	i2c_readwrite(client, 2, bl_buf, 0, NULL);
}

static inline struct jdi_panel *to_jdi_panel(struct drm_panel *panel)
{
	return container_of(panel, struct jdi_panel, base);
}


static int jdi_panel_disable(struct drm_panel *panel)
{
	struct jdi_panel *jdi = to_jdi_panel(panel);

	if (!jdi->enabled)
		return 0;

	backlight_disable(jdi->backlight);

	jdi->enabled = false;

	return 0;
}

static int jdi_panel_unprepare(struct drm_panel *panel)
{
    backlight_ctl(0);
	return 0;
}

static int jdi_panel_prepare(struct drm_panel *panel)
{
    return 0;
}

static int jdi_panel_enable(struct drm_panel *panel)
{
	struct jdi_panel *jdi = to_jdi_panel(panel);

	if (jdi->enabled)
		return 0;

	backlight_enable(jdi->backlight);

    backlight_ctl(jdi->backlight->props.brightness);
	jdi->enabled = true;

	return 0;
}

static struct drm_display_mode default_mode = {
		.clock = 76000000 / 1000,
		.hdisplay = 1280,
		.hsync_start = 1280 + 100,
		.hsync_end = 1280 + 100 + 100,
		.htotal = 1280 + 100 + 100 + 100,
		.vdisplay = 800,
		.vsync_start = 800 + 10,
		.vsync_end = 800 + 10 + 10,
		.vtotal = 800 + 10 + 10 + 10,
        .flags = 0,
};

static int jdi_panel_get_modes(struct drm_panel *panel,
			       struct drm_connector *connector)
{
	struct drm_display_mode *mode;
	struct jdi_panel *jdi = to_jdi_panel(panel);
	struct device *dev = &jdi->dsi->dev;

	mode = drm_mode_duplicate(connector->dev, &default_mode);
	if (!mode) {
		dev_err(dev, "failed to add mode %ux%ux@%u\n",
			default_mode.hdisplay, default_mode.vdisplay,
			drm_mode_vrefresh(&default_mode));
		return -ENOMEM;
	}

	drm_mode_set_name(mode);

	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = 95;
	connector->display_info.height_mm = 151;

	return 1;
}

static int dsi_dcs_bl_get_brightness(struct backlight_device *bl)
{
	u16 brightness = bl->props.brightness;

	//dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	//dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return brightness & 0xff;
}

static int dsi_dcs_bl_update_status(struct backlight_device *bl)
{

	//dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

    backlight_ctl(bl->props.brightness);

	//dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return 0;
}

static const struct backlight_ops dsi_bl_ops = {
	.update_status = dsi_dcs_bl_update_status,
	.get_brightness = dsi_dcs_bl_get_brightness,
};

static struct backlight_device *
drm_panel_create_dsi_backlight(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct backlight_properties props;

	memset(&props, 0, sizeof(props));
	props.type = BACKLIGHT_RAW;
	props.brightness = Backlight;
	props.max_brightness = 255;
   // printk("sclu Backlight=%d\n", Backlight);
	return devm_backlight_device_register(dev, "rpi-backlight", dev, dsi,
					      &dsi_bl_ops, &props);
}

static const struct drm_panel_funcs jdi_panel_funcs = {
	.disable = jdi_panel_disable,
	.unprepare = jdi_panel_unprepare,
	.prepare = jdi_panel_prepare,
	.enable = jdi_panel_enable,
	.get_modes = jdi_panel_get_modes,
};

static const struct of_device_id jdi_of_match[] = {
	{ .compatible = "JMO_DSI1024x600_Screen_compatible" },
	{ }
};
MODULE_DEVICE_TABLE(of, jdi_of_match);

static int jdi_panel_add(struct jdi_panel *jdi)
{
	struct device *dev = &jdi->dsi->dev;
	int ret;

	jdi->mode = &default_mode;
#if 0
	for (i = 0; i < ARRAY_SIZE(jdi->supplies); i++)
		jdi->supplies[i].supply = regulator_names[i];

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(jdi->supplies),
				      jdi->supplies);
	if (ret < 0) {
		dev_err(dev, "failed to init regulator, ret=%d\n", ret);
		return ret;
	}

	jdi->enable_gpio = devm_gpiod_get(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(jdi->enable_gpio)) {
		ret = PTR_ERR(jdi->enable_gpio);
		dev_err(dev, "cannot get enable-gpio %d\n", ret);
		return ret;
	}

	jdi->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(jdi->reset_gpio)) {
		ret = PTR_ERR(jdi->reset_gpio);
		dev_err(dev, "cannot get reset-gpios %d\n", ret);
		return ret;
	}

	jdi->dcdc_en_gpio = devm_gpiod_get(dev, "dcdc-en", GPIOD_OUT_LOW);
	if (IS_ERR(jdi->dcdc_en_gpio)) {
		ret = PTR_ERR(jdi->dcdc_en_gpio);
		dev_err(dev, "cannot get dcdc-en-gpio %d\n", ret);
		return ret;
	}
#endif
    

	jdi->backlight = drm_panel_create_dsi_backlight(jdi->dsi);
	if (IS_ERR(jdi->backlight)) {
		ret = PTR_ERR(jdi->backlight);
		dev_err(dev, "failed to register backlight %d\n", ret);
		return ret;
	}

	drm_panel_init(&jdi->base, &jdi->dsi->dev, &jdi_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);

	drm_panel_add(&jdi->base);

	return 0;
}

static void jdi_panel_del(struct jdi_panel *jdi)
{
    backlight_ctl(0);
	//if (jdi->base.dev)
	drm_panel_remove(&jdi->base);
    devm_kfree(&jdi->dsi->dev, jdi);
}


static int i2c_test(void) {
    uint8_t w_buf[4] = {0};
    uint8_t r_buf[4] = {0};
    int x, y;
    w_buf[0] = 0xd1;
    if (i2c_readwrite(client, 1, w_buf, 4, r_buf)) {
        printk("%s error\n", __func__);
        return -1;
    }
    x = r_buf[0] << 8 | r_buf[1];
    y = r_buf[2] << 8 | r_buf[3];
    if (x > 0 && y > 0)
        return 0;
    return -1;
}




static int get_adapter(int nr) {
    struct i2c_board_info info;
    struct i2c_adapter *adapter;
    
    memset(&info, 0, sizeof(struct i2c_board_info));
    info.addr = 0x45;
    strlcpy(info.type, "lcd_backlight", I2C_NAME_SIZE);
    
    adapter = i2c_get_adapter(nr);
    if (!adapter) {
        return -1;
    }
    client = i2c_new_client_device(adapter, &info);
    if (IS_ERR(client)) {
        client = NULL;
        return -1;
    }

    i2c_put_adapter(adapter);
    if (!client) {
        printk("%s : can't add i2c device at 0x%x\n",__func__,
                (unsigned int)info.addr);
        return -1;
    }

    if ((nr != 0) && i2c_test()) {
        i2c_unregister_device(client);
        return -1;
    }

    printk("get adapter %d success\n", nr);
    return 0;
}


static int register_iic_client(void)
{
    if (get_adapter(1)) {//也就是用i2c1
        printk("get adapter 1 fail\n");
        if (get_adapter(10)) {//也就是用i2c0
            printk("get adapter 10 fail\n");
            if (get_adapter(0)) {//也就是用i2c0
                printk("get adapter 0 fail\n");
                return -1;
            }
        }
    }
    return 0;
}




static int flags = 0;
static int jdi_panel_probe(struct mipi_dsi_device *dsi)
{
	struct jdi_panel *jdi;
	struct device *dev;
    struct device_node *np;
    int backlight;
	int ret;

    //printk("sclu %s %d\n", __func__, __LINE__);
	dsi->lanes = 2;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = (0x11);
	jdi = devm_kzalloc(&dsi->dev, sizeof(*jdi), GFP_KERNEL);
	if (!jdi)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, jdi);

	jdi->dsi = dsi;

	dev = &jdi->dsi->dev;
    np = dev->of_node;
    ret = of_property_read_u32(np, "Backlight", &backlight);
    if (ret) {
        printk("sclu read Backlight err\n");
    } else {
        Backlight = backlight;
        printk("sclu backlight=%d\n", backlight);
    }
	if (!flags) {
		if (register_iic_client()) {
			return -1;
		} else {
			flags = 1;
		}
	}

	ret = jdi_panel_add(jdi);
	if (ret < 0)
		return ret;


	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
	    jdi_panel_del(jdi);
	    return ret;
	}

	return 0;
}

static int jdi_panel_remove(struct mipi_dsi_device *dsi)
{
	struct jdi_panel *jdi = mipi_dsi_get_drvdata(dsi);
	int ret;

    backlight_ctl(0);
	ret = jdi_panel_disable(&jdi->base);
	if (ret < 0)
		dev_err(&dsi->dev, "failed to disable panel: %d\n", ret);

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "failed to detach from DSI host: %d\n",
			ret);

	jdi_panel_del(jdi);

	return 0;
}

static void jdi_panel_shutdown(struct mipi_dsi_device *dsi)
{
	struct jdi_panel *jdi = mipi_dsi_get_drvdata(dsi);

    backlight_ctl(0);
	jdi_panel_disable(&jdi->base);
}

static int __maybe_unused jdi_suspend(struct device *dev)
{
	return 0;
}

static int __maybe_unused jdi_resume(struct device *dev)
{
	return 0;
}


static SIMPLE_DEV_PM_OPS(jdi_pm_ops, jdi_suspend,
			 jdi_resume);

static struct mipi_dsi_driver jdi_panel_driver = {
	.driver = {
		.name = "JMO-Screen",
		.of_match_table = jdi_of_match,
		.pm	= &jdi_pm_ops,
	},
	.probe = jdi_panel_probe,
	.remove = jdi_panel_remove,
	.shutdown = jdi_panel_shutdown,
};
#if 0
static int __init rpi_screen_init(void)
{
    return mipi_dsi_driver_register_full(&jdi_panel_driver, THIS_MODULE);
}

static void __exit rpi_screen_exit(void) {
    mipi_dsi_driver_unregister(&jdi_panel_driver);
}
#endif
module_mipi_dsi_driver(jdi_panel_driver);
//module_init(rpi_screen_init);
//late_initcall_sync(rpi_screen_init);
//module_exit(rpi_screen_exit);


//module_param(Backlight, byte, 0644);

MODULE_AUTHOR("Sumit Semwal <sumit.semwal@linaro.org>");
MODULE_AUTHOR("Vinay Simha BN <simhavcs@gmail.com>");
MODULE_DESCRIPTION("JDI LT070ME05000 WUXGA");
MODULE_LICENSE("GPL v2");
