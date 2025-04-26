/**
 * Copyright (c) 2023 Mr Beam Lasers GmbH.
 * Copyright (c) 2023 Amrith Venkat Kesavamoorthi <amrith@mr-beam.org>
 * Copyright (c) 2023 Martin Kiepfer <mrmarteng@teleschirm.org>
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT galaxycore_gc9d01

#include "display_gc9d01.h"

#include <zephyr/dt-bindings/display/panel.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/mipi_dbi.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(display_gc9d01, CONFIG_DISPLAY_LOG_LEVEL);

/* Maximum number of default init registers  */
#define GC9X01X_NUM_DEFAULT_INIT_REGS 32U

/* Display data struct */
struct gc9x01x_data {
	uint8_t bytes_per_pixel;
	enum display_pixel_format pixel_format;
	enum display_orientation orientation;
};

/* Configuration data struct.*/
struct gc9x01x_config {
	const struct device *mipi_dev;
	struct mipi_dbi_config dbi_config;
	uint8_t pixel_format;
	uint16_t orientation;
	uint16_t x_resolution;
	uint16_t y_resolution;
	bool inversion;
	const void *regs;
};

/* Initialization command data struct  */
struct gc9x01x_default_init_regs {
	uint8_t cmd;
	uint8_t len;
	uint8_t data[GC9X01X_NUM_DEFAULT_INIT_REGS];
};

/*
 * Default initialization commands. There are a lot of undocumented commands
 * within the manufacturer sample code, that are essential for proper operation of
 * the display controller
 */
static const struct gc9x01x_default_init_regs default_init_regs[] = {
	{
		.cmd = 0xEBU,
		.len = 1U,
		.data = {0x14U},
	},
	{
		.cmd = 0x84U,
		.len = 1U,
		.data = {0x40U},
	},
	{
		.cmd = 0x85U,
		.len = 1U,
		.data = {0xFFU},
	},
	{
		.cmd = 0x86U,
		.len = 1U,
		.data = {0xFFU},
	},
	{
		.cmd = 0x87U,
		.len = 1U,
		.data = {0xFFU},
	},
	{
		.cmd = 0x88U,
		.len = 1U,
		.data = {0x0AU},
	},
	{
		.cmd = 0x89U,
		.len = 1U,
		.data = {0x21U},
	},
	{
		.cmd = 0x8AU,
		.len = 1U,
		.data = {0x00U},
	},
	{
		.cmd = 0x8BU,
		.len = 1U,
		.data = {0x80U},
	},
	{
		.cmd = 0x8CU,
		.len = 1U,
		.data = {0x01U},
	},
	{
		.cmd = 0x8DU,
		.len = 1U,
		.data = {0x01U},
	},
	{
		.cmd = 0x8EU,
		.len = 1U,
		.data = {0xFFU},
	},
	{
		.cmd = 0x8FU,
		.len = 1U,
		.data = {0xFFU},
	},
	{
		.cmd = 0xB6U,
		.len = 2U,
		.data = {0x00U, 0x20U},
	},
	{
		.cmd = 0x90U,
		.len = 4U,
		.data = {0x08U, 0x08U, 0x08U, 0x08U},
	},
	{
		.cmd = 0xBDU,
		.len = 1U,
		.data = {0x06U},
	},
	{
		.cmd = 0xBCU,
		.len = 1U,
		.data = {0x00U},
	},
	{
		.cmd = 0xFFU,
		.len = 3U,
		.data = {0x60U, 0x01U, 0x04U},
	},
	{
		.cmd = 0xBEU,
		.len = 1U,
		.data = {0x11U},
	},
	{
		.cmd = 0xE1U,
		.len = 2U,
		.data = {0x10U, 0x0EU},
	},
	{
		.cmd = 0xDFU,
		.len = 3U,
		.data = {0x21U, 0x0CU, 0x02U},
	},
	{
		.cmd = 0xEDU,
		.len = 2U,
		.data = {0x1BU, 0x0BU},
	},
	{
		.cmd = 0xAEU,
		.len = 1U,
		.data = {0x77U},
	},
	{
		.cmd = 0xCDU,
		.len = 1U,
		.data = {0x63U},
	},
	{
		.cmd = 0x70U,
		.len = 9U,
		.data = {0x07U, 0x07U, 0x04U, 0x0EU, 0x0FU, 0x09U, 0x07U, 0x08U, 0x03U},
	},
	{
		.cmd = 0x62U,
		.len = 12U,
		.data = {0x18U, 0x0DU, 0x71U, 0xEDU, 0x70U, 0x70U, 0x18U, 0x0FU, 0x71U, 0xEFU,
			 0x70U, 0x70U},
	},
	{
		.cmd = 0x63U,
		.len = 12U,
		.data = {0x18U, 0x11U, 0x71U, 0xF1U, 0x70U, 0x70U, 0x18U, 0x13U, 0x71U, 0xF3U,
			 0x70U, 0x70U},
	},
	{
		.cmd = 0x64U,
		.len = 7U,
		.data = {0x28U, 0x29U, 0xF1U, 0x01U, 0xF1U, 0x00U, 0x07U},
	},
	{
		.cmd = 0x66U,
		.len = 10U,
		.data = {0x3CU, 0x00U, 0xCDU, 0x67U, 0x45U, 0x45U, 0x10U, 0x00U, 0x00U, 0x00U},
	},
	{
		.cmd = 0x67U,
		.len = 10U,
		.data = {0x00U, 0x3CU, 0x00U, 0x00U, 0x00U, 0x01U, 0x54U, 0x10U, 0x32U, 0x98U},
	},
	{
		.cmd = 0x74U,
		.len = 7U,
		.data = {0x10U, 0x85U, 0x80U, 0x00U, 0x00U, 0x4EU, 0x00U},
	},
	{
		.cmd = 0x98U,
		.len = 2U,
		.data = {0x3EU, 0x07U},
	},
};

static const struct gc9x01x_default_init_regs default_init_regs_gc9d01[] = {
	{.cmd = 0x80,.len = 1,.data={0xFF}},
{.cmd = 0x81,.len = 1,.data={0xFF}},
{.cmd = 0x82,.len = 1,.data={0xFF}},
{.cmd = 0x83,.len = 1,.data={0xFF}},
{.cmd = 0x84,.len = 1,.data={0xFF}},
{.cmd = 0x85,.len = 1,.data={0xFF}},
{.cmd = 0x86,.len = 1,.data={0xFF}},
{.cmd = 0x87,.len = 1,.data={0xFF}},
{.cmd = 0x88,.len = 1,.data={0xFF}},
{.cmd = 0x89,.len = 1,.data={0xFF}},
{.cmd = 0x8A,.len = 1,.data={0xFF}},
{.cmd = 0x8B,.len = 1,.data={0xFF}},
{.cmd = 0x8C,.len = 1,.data={0xFF}},
{.cmd = 0x8D,.len = 1,.data={0xFF}},
{.cmd = 0x8E,.len = 1,.data={0xFF}},
{.cmd = 0x8F,.len = 1,.data={0xFF}},
{.cmd = 0xEC,.len = 1,.data={0x00}},
{.cmd = 0x7E,.len = 1,.data={0x7a}},
{.cmd = 0x74,.len = 7,.data={0x02, 0x0E, 0x00, 0x00, 0x28, 0x00, 0x00}},
{.cmd = 0x98,.len = 1,.data={0x3E}},
{.cmd = 0x99,.len = 1,.data={0x3E}},
{.cmd = 0xB5,.len = 2,.data={0x0E, 0x0E}},
{.cmd = 0x60,.len = 4,.data={0x38, 0x09, 0x6D, 0x67}},
{.cmd = 0x63,.len = 5,.data={0x38, 0xAD, 0x6D, 0x67, 0x05}},
{.cmd = 0x64,.len = 6,.data={0x38, 0x0B, 0x70, 0xAB, 0x6D, 0x67}},
{.cmd = 0x66,.len = 6,.data={0x38, 0x0F, 0x70, 0xAF, 0x6d, 0x67}},
{.cmd = 0x6A,.len = 2,.data={0x00, 0x00}},
{.cmd = 0x68,.len = 7,.data={0x3B, 0x08, 0x04, 0x00, 0x04, 0x64, 0x67}},
{.cmd = 0x6C,.len = 7,.data={0x22, 0x02, 0x22, 0x02, 0x22, 0x22, 0x50}},
{.cmd = 0x6E,.len = 32,.data={0x00, 0x00, 0x00, 0x00, 0x07, 0x01, 0x13, 0x11, 0x0B, 0x09, 0x16, 0x15, 0x1D, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x1D, 0x15, 0x16, 0x0A, 0x0C, 0x12, 0x14, 0x02, 0x08, 0x00, 0x00, 0x00, 0x00}},
{.cmd = 0xA9,.len = 1,.data={0x1B}},
{.cmd = 0xA8,.len = 1,.data={0x6B}},
{.cmd = 0xA8,.len = 1,.data={0x6D}},
{.cmd = 0xA7,.len = 1,.data={0x40}},
{.cmd = 0xAD,.len = 1,.data={0x47}},
{.cmd = 0xAF,.len = 1,.data={0x73}},
{.cmd = 0xAF,.len = 1,.data={0x73}},
{.cmd = 0xAC,.len = 1,.data={0x44}},
{.cmd = 0xA3,.len = 1,.data={0x6C}},
{.cmd = 0xCB,.len = 1,.data={0x00}},
{.cmd = 0xCD,.len = 1,.data={0x22}},
{.cmd = 0xC2,.len = 1,.data={0x10}},
{.cmd = 0xC5,.len = 1,.data={0x00}},
{.cmd = 0xC6,.len = 1,.data={0x0E}},
{.cmd = 0xC7,.len = 1,.data={0x1f}},
{.cmd = 0xC8,.len = 1,.data={0x0E}},
{.cmd = 0xbf,.len = 1,.data={0x00}},
{.cmd = 0xF9,.len = 1,.data={0x20}},
{.cmd = 0x9b,.len = 1,.data={0x3b}},
{.cmd = 0x93,.len = 3,.data={0x33, 0x7f, 0x00}},
{.cmd = 0x70,.len = 6,.data={0x0E, 0x0f, 0x03, 0x0e, 0x0f, 0x03}},
{.cmd = 0x71,.len = 3,.data={0x0e, 0x16, 0x03}},
{.cmd = 0x91,.len = 2,.data={0x0e, 0x09}},
{.cmd = 0xc3,.len = 1,.data={0x2c}},
{.cmd = 0xc4,.len = 1,.data={0x1a}},
{.cmd = 0xf0,.len = 6,.data={0x51, 0x13, 0x0c, 0x06, 0x00, 0x2f}},
{.cmd = 0xf2,.len = 6,.data={0x51, 0x13, 0x0c, 0x06, 0x00, 0x33}},
{.cmd = 0xf1,.len = 6,.data={0x3c, 0x94, 0x4f, 0x33, 0x34, 0xCf}},
{.cmd = 0xf3,.len = 6,.data={0x4d, 0x94, 0x4f, 0x33, 0x34, 0xCf}},

	// 添加 CASET/RASET
{.cmd = 0x2A,.len = 4,.data={0x00, 0x3C, 0x00, 0xB4}},
{.cmd = 0x2B,.len = 4,.data={0x00, 0x00, 0x00, 0x9F}},



};

static int gc9x01x_transmit(const struct device *dev, uint8_t cmd, const void *tx_data,
			    size_t tx_len)
{
	const struct gc9x01x_config *config = dev->config;

	return mipi_dbi_command_write(config->mipi_dev, &config->dbi_config,
				      cmd, tx_data, tx_len);
}

static int gc9x01x_regs_init(const struct device *dev)
{
	const struct gc9x01x_config *config = dev->config;
	const struct gc9x01x_regs *regs = config->regs;
	int ret;

	if (!device_is_ready(config->mipi_dev)) {
		return -ENODEV;
	}

	/* Enable inter-command mode */
	ret = gc9x01x_transmit(dev, GC9X01X_CMD_INREGEN1, NULL, 0);
	if (ret < 0) {
		return ret;
	}
	ret = gc9x01x_transmit(dev, GC9X01X_CMD_INREGEN2, NULL, 0);
	if (ret < 0) {
		return ret;
	}

	/* Apply default init sequence */
	for (int i = 0; (i < ARRAY_SIZE(default_init_regs_gc9d01)) && (ret == 0); i++) {
		ret = gc9x01x_transmit(dev, default_init_regs_gc9d01[i].cmd, default_init_regs_gc9d01[i].data,
				       default_init_regs_gc9d01[i].len);
		if (ret < 0) {
			return ret;
		}
	}

	/* Apply generic configuration */
	//ret = gc9x01x_transmit(dev, GC9X01X_CMD_PWRCTRL2, regs->pwrctrl2, sizeof(regs->pwrctrl2));
	//if (ret < 0) {
	//	return ret;
	//}
	//ret = gc9x01x_transmit(dev, GC9X01X_CMD_PWRCTRL3, regs->pwrctrl3, sizeof(regs->pwrctrl3));
	//if (ret < 0) {
	//	return ret;
	//}
	//ret = gc9x01x_transmit(dev, GC9X01X_CMD_PWRCTRL4, regs->pwrctrl4, sizeof(regs->pwrctrl4));
	//if (ret < 0) {
	//	return ret;
	//}
	//ret = gc9x01x_transmit(dev, GC9X01X_CMD_GAMMA1, regs->gamma1, sizeof(regs->gamma1));
	//if (ret < 0) {
	//	return ret;
	//}
	//ret = gc9x01x_transmit(dev, GC9X01X_CMD_GAMMA2, regs->gamma2, sizeof(regs->gamma2));
	//if (ret < 0) {
	//	return ret;
	//}
	//ret = gc9x01x_transmit(dev, GC9X01X_CMD_GAMMA3, regs->gamma3, sizeof(regs->gamma3));
	//if (ret < 0) {
	//	return ret;
	//}
	//ret = gc9x01x_transmit(dev, GC9X01X_CMD_GAMMA4, regs->gamma4, sizeof(regs->gamma4));
	//if (ret < 0) {
	//	return ret;
	//}
	//ret = gc9x01x_transmit(dev, GC9X01X_CMD_FRAMERATE, regs->framerate,
	//		       sizeof(regs->framerate));
	//if (ret < 0) {
	//	return ret;
	//}

	///* Enable Tearing line */
	//ret = gc9x01x_transmit(dev, GC9X01X_CMD_TEON, NULL, 0);
	//if (ret < 0) {
	//	return ret;
	//}

	return 0;
}

static int gc9x01x_exit_sleep(const struct device *dev)
{
	int ret;

	ret = gc9x01x_transmit(dev, GC9X01X_CMD_SLPOUT, NULL, 0);
	if (ret < 0) {
		return ret;
	}

	/*
	 * Exit sleepmode and enable display. 30ms on top of the sleepout time to account for
	 * any manufacturing defects.
	 * This is to allow time for the supply voltages and clock circuits stabilize
	 */
	k_msleep(GC9X01X_SLEEP_IN_OUT_DURATION_MS + 30);

	return 0;
}

#ifdef CONFIG_PM_DEVICE
static int gc9x01x_enter_sleep(const struct device *dev)
{
	int ret;

	ret = gc9x01x_transmit(dev, GC9X01X_CMD_SLPIN, NULL, 0);
	if (ret < 0) {
		return ret;
	}

	/*
	 * Exit sleepmode and enable display. 30ms on top of the sleepout time to account for
	 * any manufacturing defects.
	 */
	k_msleep(GC9X01X_SLEEP_IN_OUT_DURATION_MS + 30);

	return 0;
}
#endif

static int gc9x01x_hw_reset(const struct device *dev)
{
	const struct gc9x01x_config *config = dev->config;
	int ret;

	ret = mipi_dbi_reset(config->mipi_dev, 100);
	if (ret < 0) {
		return ret;
	}
	k_msleep(10);

	return ret;
}

static int gc9x01x_display_blanking_off(const struct device *dev)
{
	LOG_DBG("Turning display blanking off");
	return gc9x01x_transmit(dev, GC9X01X_CMD_DISPON, NULL, 0);
}

static int gc9x01x_display_blanking_on(const struct device *dev)
{
	LOG_DBG("Turning display blanking on");
	return gc9x01x_transmit(dev, GC9X01X_CMD_DISPOFF, NULL, 0);
}

static int gc9x01x_set_pixel_format(const struct device *dev,
				    const enum display_pixel_format pixel_format)
{
	struct gc9x01x_data *data = dev->data;
	int ret;
	uint8_t tx_data;
	uint8_t bytes_per_pixel;

	if (pixel_format == PIXEL_FORMAT_RGB_565) {
		bytes_per_pixel = 2U;
		tx_data = GC9X01X_PIXFMT_VAL_MCU_16_BIT | GC9X01X_PIXFMT_VAL_RGB_16_BIT;
	} else if (pixel_format == PIXEL_FORMAT_RGB_888) {
		bytes_per_pixel = 3U;
		tx_data = GC9X01X_PIXFMT_VAL_MCU_18_BIT | GC9X01X_PIXFMT_VAL_RGB_18_BIT;
	} else {
		LOG_ERR("Unsupported pixel format");
		return -ENOTSUP;
	}

	ret = gc9x01x_transmit(dev, GC9X01X_CMD_PIXFMT, &tx_data, 1U);
	if (ret < 0) {
		return ret;
	}

	data->pixel_format = pixel_format;
	data->bytes_per_pixel = bytes_per_pixel;

	return 0;
}

static int gc9x01x_set_orientation(const struct device *dev,
				   const enum display_orientation orientation)
{
	struct gc9x01x_data *data = dev->data;
	int ret;
	uint8_t tx_data = 0;

	if (orientation == DISPLAY_ORIENTATION_NORMAL) {
		/* works 0° - default */
	} else if (orientation == DISPLAY_ORIENTATION_ROTATED_90) {
		/* works CW 90° */
		tx_data |= GC9X01X_MADCTL_VAL_MV | GC9X01X_MADCTL_VAL_MY;
	} else if (orientation == DISPLAY_ORIENTATION_ROTATED_180) {
		/* works CW 180° */
		tx_data |= GC9X01X_MADCTL_VAL_MY | GC9X01X_MADCTL_VAL_MX | GC9X01X_MADCTL_VAL_MH;
	} else if (orientation == DISPLAY_ORIENTATION_ROTATED_270) {
		/* works CW 270° */
		tx_data |= GC9X01X_MADCTL_VAL_MV | GC9X01X_MADCTL_VAL_MX;
	}

	ret = gc9x01x_transmit(dev, GC9X01X_CMD_MADCTL, &tx_data, 1U);
	if (ret < 0) {
		return ret;
	}

	data->orientation = orientation;

	return 0;
}

static int gc9x01x_configure(const struct device *dev)
{
	const struct gc9x01x_config *config = dev->config;
	int ret;

	/* Set all the required registers. */
	ret = gc9x01x_regs_init(dev);
	if (ret < 0) {
		return ret;
	}

	/* Pixel format */
	ret = gc9x01x_set_pixel_format(dev, config->pixel_format);
	if (ret < 0) {
		return ret;
	}

	/* Orientation */
	ret = gc9x01x_set_orientation(dev, config->orientation);
	if (ret < 0) {
		return ret;
	}

	/* Display inversion mode. */
	if (config->inversion) {
		ret = gc9x01x_transmit(dev, GC9X01X_CMD_INVON, NULL, 0);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

static int gc9x01x_init(const struct device *dev)
{
	int ret;

	gc9x01x_hw_reset(dev);

	gc9x01x_display_blanking_on(dev);

	ret = gc9x01x_configure(dev);
	if (ret < 0) {
		LOG_ERR("Could not configure display (%d)", ret);
		return ret;
	}

	ret = gc9x01x_exit_sleep(dev);
	if (ret < 0) {
		LOG_ERR("Could not exit sleep mode (%d)", ret);
		return ret;
	}

	return 0;
}

static int gc9x01x_set_mem_area(const struct device *dev, const uint16_t x, const uint16_t y,
				const uint16_t w, const uint16_t h)
{
	int ret;
	uint16_t spi_data[2];

	uint16_t ram_x =x;
	uint16_t ram_y =y;

	spi_data[0] = sys_cpu_to_be16(ram_x);
	spi_data[1] = sys_cpu_to_be16(ram_x + w-1);
	ret = gc9x01x_transmit(dev, GC9X01X_CMD_COLSET, &spi_data[0], 4U);
	if (ret < 0) {
		return ret;
	}

	spi_data[0] = sys_cpu_to_be16(ram_y);
	spi_data[1] = sys_cpu_to_be16(ram_y + h -1);
	ret = gc9x01x_transmit(dev, GC9X01X_CMD_ROWSET, &spi_data[0], 4U);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int gc9x01x_write(const struct device *dev, const uint16_t x, const uint16_t y,
			 const struct display_buffer_descriptor *desc, const void *buf)
{
	const struct gc9x01x_config *config = dev->config;
	struct gc9x01x_data *data = dev->data;
	int ret;
	const uint8_t *write_data_start = (const uint8_t *)buf;
	struct display_buffer_descriptor mipi_desc;
	uint16_t write_cnt;
	uint16_t nbr_of_writes;
	uint16_t write_h;

	__ASSERT(desc->width <= desc->pitch, "Pitch is smaller than width");
	__ASSERT((desc->pitch * data->bytes_per_pixel * desc->height) <= desc->buf_size,
		 "Input buffer too small");

	LOG_DBG("Writing %dx%d (w,h) @ %dx%d (x,y)", desc->width, desc->height, x, y);
	ret = gc9x01x_set_mem_area(dev, x, y, desc->width, desc->height);
	if (ret < 0) {
		return ret;
	}

	if (desc->pitch > desc->width) {
		write_h = 1U;
		nbr_of_writes = desc->height;
		mipi_desc.height = 1;
		mipi_desc.buf_size = desc->width * data->bytes_per_pixel;
	} else {
		write_h = desc->height;
		mipi_desc.height = desc->height;
		mipi_desc.buf_size = desc->width * data->bytes_per_pixel * write_h;
		nbr_of_writes = 1U;
	}

	mipi_desc.width = desc->width;
	/* Per MIPI API, pitch must always match width */
	mipi_desc.pitch = desc->width;
	mipi_desc.frame_incomplete = desc->frame_incomplete;

	ret = gc9x01x_transmit(dev, GC9X01X_CMD_MEMWR, NULL, 0);
	if (ret < 0) {
		return ret;
	}

	for (write_cnt = 0U; write_cnt < nbr_of_writes; ++write_cnt) {
		ret = mipi_dbi_write_display(config->mipi_dev,
					     &config->dbi_config,
					     write_data_start,
					     &mipi_desc,
					     data->pixel_format);
		if (ret < 0) {
			return ret;
		}

		write_data_start += desc->pitch * data->bytes_per_pixel;
	}

	return 0;
}

static void gc9x01x_get_capabilities(const struct device *dev,
				     struct display_capabilities *capabilities)
{
	struct gc9x01x_data *data = dev->data;
	const struct gc9x01x_config *config = dev->config;

	memset(capabilities, 0, sizeof(struct display_capabilities));

	capabilities->supported_pixel_formats = PIXEL_FORMAT_RGB_565 | PIXEL_FORMAT_RGB_888;
	capabilities->current_pixel_format = data->pixel_format;

	if (data->orientation == DISPLAY_ORIENTATION_NORMAL ||
	    data->orientation == DISPLAY_ORIENTATION_ROTATED_180) {
		capabilities->x_resolution = config->x_resolution;
		capabilities->y_resolution = config->y_resolution;
	} else {
		capabilities->x_resolution = config->y_resolution;
		capabilities->y_resolution = config->x_resolution;
	}

	capabilities->current_orientation = data->orientation;
}

#ifdef CONFIG_PM_DEVICE
static int gc9x01x_pm_action(const struct device *dev, enum pm_device_action action)
{
	int ret;

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		ret = gc9x01x_exit_sleep(dev);
		break;
	case PM_DEVICE_ACTION_SUSPEND:
		ret = gc9x01x_enter_sleep(dev);
		break;
	default:
		ret = -ENOTSUP;
		break;
	}

	return ret;
}
#endif /* CONFIG_PM_DEVICE */

/* Device driver API*/
static DEVICE_API(display, gc9x01x_api) = {
	.blanking_on = gc9x01x_display_blanking_on,
	.blanking_off = gc9x01x_display_blanking_off,
	.write = gc9x01x_write,
	.get_capabilities = gc9x01x_get_capabilities,
	.set_pixel_format = gc9x01x_set_pixel_format,
	.set_orientation = gc9x01x_set_orientation,
};

#define GC9D01_INIT(inst)                                                                         \
	GC9D01_REGS_INIT(inst);                                                                   \
	static const struct gc9x01x_config gc9x01x_config_##inst = {                               \
		.mipi_dev = DEVICE_DT_GET(DT_INST_PARENT(inst)),                                   \
		.dbi_config = {                                                                    \
			.mode = MIPI_DBI_MODE_SPI_4WIRE,                                           \
			.config = MIPI_DBI_SPI_CONFIG_DT_INST(inst,                                \
							      SPI_OP_MODE_MASTER |                 \
							      SPI_WORD_SET(8), 0),                 \
		},                                                                                 \
		.pixel_format = DT_INST_PROP(inst, pixel_format),                                  \
		.orientation = DT_INST_ENUM_IDX(inst, orientation),                                \
		.x_resolution = DT_INST_PROP(inst, width),                                         \
		.y_resolution = DT_INST_PROP(inst, height),                                        \
		.inversion = DT_INST_PROP(inst, display_inversion),                                \
		.regs = &gc9x01x_regs_##inst,                                                      \
	};                                                                                         \
	static struct gc9x01x_data gc9x01x_data_##inst;                                            \
	PM_DEVICE_DT_INST_DEFINE(inst, gc9x01x_pm_action);                                         \
	DEVICE_DT_INST_DEFINE(inst, &gc9x01x_init, PM_DEVICE_DT_INST_GET(inst),                    \
			      &gc9x01x_data_##inst, &gc9x01x_config_##inst, POST_KERNEL,           \
			      CONFIG_DISPLAY_INIT_PRIORITY, &gc9x01x_api);

DT_INST_FOREACH_STATUS_OKAY(GC9D01_INIT)
