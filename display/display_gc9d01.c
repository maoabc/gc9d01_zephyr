/**
 * Copyright (c) 2023 Mr Beam Lasers GmbH.
 * Copyright (c) 2023 Amrith Venkat Kesavamoorthi <amrith@mr-beam.org>
 * Copyright (c) 2023 Martin Kiepfer <mrmarteng@teleschirm.org>
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT galaxycore_gc9d01

#include "display_gc9d01.h"

#include <zephyr/drivers/display.h>
#include <zephyr/drivers/mipi_dbi.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(display_gc9d01, CONFIG_DISPLAY_LOG_LEVEL);

/* Maximum number of default init registers  */
#define GC9D01_NUM_DEFAULT_INIT_REGS 32U

/* Display data struct */
struct gc9d01_data {
  uint16_t x_off;
  uint16_t y_off;
  uint8_t bytes_per_pixel;
  enum display_pixel_format pixel_format;
  enum display_orientation orientation;
};

/* Configuration data struct.*/
struct gc9d01_config {
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
struct gc9d01_default_init_regs {
  uint8_t cmd;
  uint8_t len;
  uint8_t data[GC9D01_NUM_DEFAULT_INIT_REGS];
};

static const struct gc9d01_default_init_regs default_init_regs_gc9d01[] = {
    {.cmd = 0x80, .len = 1, .data = {0xff}},
    {.cmd = 0x81, .len = 1, .data = {0xff}},
    {.cmd = 0x82, .len = 1, .data = {0xff}},
    {.cmd = 0x83, .len = 1, .data = {0xff}},
    {.cmd = 0x84, .len = 1, .data = {0xff}},
    {.cmd = 0x85, .len = 1, .data = {0xff}},
    {.cmd = 0x86, .len = 1, .data = {0xff}},
    {.cmd = 0x87, .len = 1, .data = {0xff}},
    {.cmd = 0x88, .len = 1, .data = {0xff}},
    {.cmd = 0x89, .len = 1, .data = {0xff}},
    {.cmd = 0x8a, .len = 1, .data = {0xff}},
    {.cmd = 0x8b, .len = 1, .data = {0xff}},
    {.cmd = 0x8c, .len = 1, .data = {0xff}},
    {.cmd = 0x8d, .len = 1, .data = {0xff}},
    {.cmd = 0x8e, .len = 1, .data = {0xff}},
    {.cmd = 0x8f, .len = 1, .data = {0xff}},
    {.cmd = 0x3a, .len = 1, .data = {0x05}},
    {.cmd = 0xec, .len = 1, .data = {0x10}},
    {.cmd = 0x7e, .len = 1, .data = {0x7a}},
    {.cmd = 0x74, .len = 7, .data = {0x02, 0x0e, 0x00, 0x00, 0x28, 0x00, 0x00}},
    {.cmd = 0x98, .len = 1, .data = {0x3e}},
    {.cmd = 0x99, .len = 1, .data = {0x3e}},
    {.cmd = 0xb5, .len = 3, .data = {0x0e, 0x0e, 0x00}},
    {.cmd = 0x60, .len = 4, .data = {0x38, 0x09, 0x6d, 0x67}},
    {.cmd = 0x63, .len = 5, .data = {0x38, 0xad, 0x6d, 0x67, 0x05}},
    {.cmd = 0x64, .len = 6, .data = {0x38, 0x0b, 0x70, 0xab, 0x6d, 0x67}},
    {.cmd = 0x66, .len = 6, .data = {0x38, 0x0f, 0x70, 0xaf, 0x6d, 0x67}},
    {.cmd = 0x6a, .len = 2, .data = {0x00, 0x00}},
    {.cmd = 0x68, .len = 7, .data = {0x3b, 0x08, 0x04, 0x00, 0x04, 0x64, 0x67}},
    {.cmd = 0x6c, .len = 7, .data = {0x22, 0x02, 0x22, 0x02, 0x22, 0x22, 0x50}},
    {.cmd = 0x6e,
     .len = 32,
     .data = {0x00, 0x00, 0x00, 0x00, 0x07, 0x01, 0x13, 0x11, 0x0b, 0x09, 0x16,
              0x15, 0x1d, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x1e, 0x1d, 0x15, 0x16,
              0x0a, 0x0c, 0x12, 0x14, 0x02, 0x08, 0x00, 0x00, 0x00, 0x00}},
    {.cmd = 0xa9, .len = 1, .data = {0x1b}},
    {.cmd = 0xa8, .len = 1, .data = {0x6b}},
    {.cmd = 0xa8, .len = 1, .data = {0x6d}},
    {.cmd = 0xa7, .len = 1, .data = {0x40}},
    {.cmd = 0xad, .len = 1, .data = {0x47}},
    {.cmd = 0xaf, .len = 1, .data = {0x73}},
    {.cmd = 0xaf, .len = 1, .data = {0x73}},
    {.cmd = 0xac, .len = 1, .data = {0x44}},
    {.cmd = 0xa3, .len = 1, .data = {0x6c}},
    {.cmd = 0xcb, .len = 1, .data = {0x00}},
    {.cmd = 0xcd, .len = 1, .data = {0x22}},
    {.cmd = 0xc2, .len = 1, .data = {0x10}},
    {.cmd = 0xc5, .len = 1, .data = {0x00}},
    {.cmd = 0xc6, .len = 1, .data = {0x0e}},
    {.cmd = 0xc7, .len = 1, .data = {0x1f}},
    {.cmd = 0xc8, .len = 1, .data = {0x0e}},
    {.cmd = 0xbf, .len = 1, .data = {0x00}},
    {.cmd = 0xf9, .len = 1, .data = {0x20}},
    {.cmd = 0x9b, .len = 1, .data = {0x3b}},
    {.cmd = 0x93, .len = 3, .data = {0x33, 0x7f, 0x00}},
    {.cmd = 0x70, .len = 6, .data = {0x0e, 0x0f, 0x03, 0x0e, 0x0f, 0x03}},
    {.cmd = 0x71, .len = 3, .data = {0x0e, 0x16, 0x03}},
    {.cmd = 0x91, .len = 2, .data = {0x0e, 0x09}},
};

static int gc9d01_transmit(const struct device *dev, uint8_t cmd,
                           const void *tx_data, size_t tx_len) {
  const struct gc9d01_config *config = dev->config;

  return mipi_dbi_command_write(config->mipi_dev, &config->dbi_config, cmd,
                                tx_data, tx_len);
}

static int gc9d01_regs_init(const struct device *dev) {
  const struct gc9d01_config *config = dev->config;
  const struct gc9d01_regs *regs = config->regs;
  int ret;

  if (!device_is_ready(config->mipi_dev)) {
    return -ENODEV;
  }

  /* Enable inter-command mode */
  ret = gc9d01_transmit(dev, GC9D01_CMD_INREGEN1, NULL, 0);
  if (ret < 0) {
    return ret;
  }
  ret = gc9d01_transmit(dev, GC9D01_CMD_INREGEN2, NULL, 0);
  if (ret < 0) {
    return ret;
  }

  /* Apply default init sequence */
  for (int i = 0; (i < ARRAY_SIZE(default_init_regs_gc9d01)) && (ret == 0);
       i++) {
    ret = gc9d01_transmit(dev, default_init_regs_gc9d01[i].cmd,
                          default_init_regs_gc9d01[i].data,
                          default_init_regs_gc9d01[i].len);
    if (ret < 0) {
      return ret;
    }
  }

  /* Apply generic configuration */
  ret = gc9d01_transmit(dev, GC9D01_CMD_PWRCTRL2, regs->pwrctrl2,
                        sizeof(regs->pwrctrl2));
  if (ret < 0) {
    return ret;
  }
  ret = gc9d01_transmit(dev, GC9D01_CMD_PWRCTRL3, regs->pwrctrl3,
                        sizeof(regs->pwrctrl3));
  if (ret < 0) {
    return ret;
  }
  ret = gc9d01_transmit(dev, GC9D01_CMD_PWRCTRL4, regs->pwrctrl4,
                        sizeof(regs->pwrctrl4));
  if (ret < 0) {
    return ret;
  }
  ret = gc9d01_transmit(dev, GC9D01_CMD_GAMMA1, regs->gamma1,
                        sizeof(regs->gamma1));
  if (ret < 0) {
    return ret;
  }
  ret = gc9d01_transmit(dev, GC9D01_CMD_GAMMA2, regs->gamma2,
                        sizeof(regs->gamma2));
  if (ret < 0) {
    return ret;
  }
  ret = gc9d01_transmit(dev, GC9D01_CMD_GAMMA3, regs->gamma3,
                        sizeof(regs->gamma3));
  if (ret < 0) {
    return ret;
  }
  ret = gc9d01_transmit(dev, GC9D01_CMD_GAMMA4, regs->gamma4,
                        sizeof(regs->gamma4));
  if (ret < 0) {
    return ret;
  }

  ///* Enable Tearing line */
  // ret = gc9d01_transmit(dev, GC9D01_CMD_TEON, NULL, 0);
  // if (ret < 0) {
  //	return ret;
  // }

  return 0;
}

static int gc9d01_exit_sleep(const struct device *dev) {
  int ret;

  ret = gc9d01_transmit(dev, GC9D01_CMD_SLPOUT, NULL, 0);
  if (ret < 0) {
    return ret;
  }

  /*
   * Exit sleepmode and enable display. 30ms on top of the sleepout time to
   * account for any manufacturing defects. This is to allow time for the supply
   * voltages and clock circuits stabilize
   */
  k_msleep(GC9D01_SLEEP_IN_OUT_DURATION_MS + 30);

  return 0;
}

#ifdef CONFIG_PM_DEVICE
static int gc9d01_enter_sleep(const struct device *dev) {
  int ret;

  ret = gc9d01_transmit(dev, GC9D01_CMD_SLPIN, NULL, 0);
  if (ret < 0) {
    return ret;
  }

  /*
   * Exit sleepmode and enable display. 30ms on top of the sleepout time to
   * account for any manufacturing defects.
   */
  k_msleep(GC9D01_SLEEP_IN_OUT_DURATION_MS + 30);

  return 0;
}
#endif

static int gc9d01_hw_reset(const struct device *dev) {
  const struct gc9d01_config *config = dev->config;
  int ret;

  ret = mipi_dbi_reset(config->mipi_dev, 100);
  if (ret < 0) {
    return ret;
  }
  k_msleep(10);

  return ret;
}

static int gc9d01_display_blanking_off(const struct device *dev) {
  LOG_DBG("Turning display blanking off");
  return gc9d01_transmit(dev, GC9D01_CMD_DISPON, NULL, 0);
}

static int gc9d01_display_blanking_on(const struct device *dev) {
  LOG_DBG("Turning display blanking on");
  return gc9d01_transmit(dev, GC9D01_CMD_DISPOFF, NULL, 0);
}

static int
gc9d01_set_pixel_format(const struct device *dev,
                        const enum display_pixel_format pixel_format) {
  struct gc9d01_data *data = dev->data;
  int ret;
  uint8_t tx_data;
  uint8_t bytes_per_pixel;

  if (pixel_format == PIXEL_FORMAT_RGB_565) {
    bytes_per_pixel = 2U;
    tx_data = GC9D01_PIXFMT_VAL_MCU_16_BIT;
  } else if (pixel_format == PIXEL_FORMAT_RGB_888) {
    bytes_per_pixel = 3U;
    tx_data = GC9D01_PIXFMT_VAL_MCU_18_BIT;
  } else {
    LOG_ERR("Unsupported pixel format");
    return -ENOTSUP;
  }

  ret = gc9d01_transmit(dev, GC9D01_CMD_PIXFMT, &tx_data, 1U);
  if (ret < 0) {
    return ret;
  }

  data->pixel_format = pixel_format;
  data->bytes_per_pixel = bytes_per_pixel;

  return 0;
}

static int gc9d01_set_orientation(const struct device *dev,
                                  const enum display_orientation orientation) {
  struct gc9d01_data *data = dev->data;
  int ret;
  uint8_t tx_data = 0;

  data->x_off = 0;
  data->y_off = 0;
  if (orientation == DISPLAY_ORIENTATION_NORMAL) {
    /* works 0° - default */
  } else if (orientation == DISPLAY_ORIENTATION_ROTATED_90) {
    /* works CW 90° */
    tx_data |= GC9D01_MADCTL_VAL_MV | GC9D01_MADCTL_VAL_MX |
               GC9D01_MADCTL_VAL_MY | GC9D01_MADCTL_VAL_ML;
    data->x_off = -60;
    data->y_off = 60;
  } else if (orientation == DISPLAY_ORIENTATION_ROTATED_180) {
    /* works CW 180° */
    tx_data |=
        GC9D01_MADCTL_VAL_MY | GC9D01_MADCTL_VAL_MX | GC9D01_MADCTL_VAL_MH;
  } else if (orientation == DISPLAY_ORIENTATION_ROTATED_270) {
    /* works CW 270° */
    tx_data |= GC9D01_MADCTL_VAL_MV | GC9D01_MADCTL_VAL_ML;
    data->x_off = -60;
    data->y_off = 60;
  }

  ret = gc9d01_transmit(dev, GC9D01_CMD_MADCTL, &tx_data, 1U);
  if (ret < 0) {
    return ret;
  }

  data->orientation = orientation;

  return 0;
}

static int gc9d01_configure(const struct device *dev) {
  const struct gc9d01_config *config = dev->config;
  int ret;

  /* Set all the required registers. */
  ret = gc9d01_regs_init(dev);
  if (ret < 0) {
    return ret;
  }

  /* Pixel format */
  ret = gc9d01_set_pixel_format(dev, config->pixel_format);
  if (ret < 0) {
    return ret;
  }

  /* Orientation */
  ret = gc9d01_set_orientation(dev, config->orientation);
  if (ret < 0) {
    return ret;
  }

  /* Display inversion mode. */
  if (config->inversion) {
    ret = gc9d01_transmit(dev, GC9D01_CMD_INVON, NULL, 0);
    if (ret < 0) {
      return ret;
    }
  }

  return 0;
}

static int gc9d01_init(const struct device *dev) {
  int ret;

  gc9d01_hw_reset(dev);

  gc9d01_display_blanking_on(dev);

  ret = gc9d01_configure(dev);
  if (ret < 0) {
    LOG_ERR("Could not configure display (%d)", ret);
    return ret;
  }

  ret = gc9d01_exit_sleep(dev);
  if (ret < 0) {
    LOG_ERR("Could not exit sleep mode (%d)", ret);
    return ret;
  }

  return 0;
}

static int gc9d01_set_mem_area(const struct device *dev, const uint16_t x,
                               const uint16_t y, const uint16_t w,
                               const uint16_t h) {
  int ret;
  uint16_t spi_data[2];
  struct gc9d01_data *data = dev->data;

  uint16_t ram_x = x + data->x_off;
  uint16_t ram_y = y + data->y_off;

  spi_data[0] = sys_cpu_to_be16(ram_x);
  spi_data[1] = sys_cpu_to_be16(ram_x + w - 1);
  ret = gc9d01_transmit(dev, GC9D01_CMD_COLSET, &spi_data[0], 4U);
  if (ret < 0) {
    return ret;
  }

  spi_data[0] = sys_cpu_to_be16(ram_y);
  spi_data[1] = sys_cpu_to_be16(ram_y + h - 1);
  ret = gc9d01_transmit(dev, GC9D01_CMD_ROWSET, &spi_data[0], 4U);
  if (ret < 0) {
    return ret;
  }

  return 0;
}

#define MAX_RESOLUTION 160

static int gc9d01_write(const struct device *dev, const uint16_t x,
                        const uint16_t y,
                        const struct display_buffer_descriptor *desc,
                        const void *buf) {
  const struct gc9d01_config *config = dev->config;
  struct gc9d01_data *data = dev->data;
  int ret;
  const uint8_t *write_data_start = (const uint8_t *)buf;
  struct display_buffer_descriptor mipi_desc;
  uint16_t nbr_of_writes;
  uint16_t write_h;

  __ASSERT(desc->width <= desc->pitch, "Pitch is smaller than width");
  __ASSERT((desc->pitch * data->bytes_per_pixel * desc->height) <=
               desc->buf_size,
           "Input buffer too small");

  LOG_DBG("Writing %dx%d (w,h) @ %dx%d (x,y)", desc->width, desc->height, x, y);
  ret = gc9d01_set_mem_area(dev, x, y, desc->width, desc->height);
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

  ret = gc9d01_transmit(dev, GC9D01_CMD_MEMWR, NULL, 0);
  if (ret < 0) {
    return ret;
  }

  for (uint16_t write_cnt = 0U; write_cnt < nbr_of_writes; ++write_cnt) {
    ret = mipi_dbi_write_display(config->mipi_dev, &config->dbi_config,
                                 write_data_start, &mipi_desc,
                                 data->pixel_format);
    if (ret < 0) {
      return ret;
    }

    write_data_start += desc->pitch * data->bytes_per_pixel;
  }

  return 0;
}

static void gc9d01_get_capabilities(const struct device *dev,
                                    struct display_capabilities *capabilities) {
  struct gc9d01_data *data = dev->data;
  const struct gc9d01_config *config = dev->config;

  memset(capabilities, 0, sizeof(struct display_capabilities));

  capabilities->supported_pixel_formats =
      PIXEL_FORMAT_RGB_565 | PIXEL_FORMAT_RGB_888;
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
static int gc9d01_pm_action(const struct device *dev,
                            enum pm_device_action action) {
  int ret;

  switch (action) {
  case PM_DEVICE_ACTION_RESUME:
    ret = gc9d01_exit_sleep(dev);
    break;
  case PM_DEVICE_ACTION_SUSPEND:
    ret = gc9d01_enter_sleep(dev);
    break;
  default:
    ret = -ENOTSUP;
    break;
  }

  return ret;
}
#endif /* CONFIG_PM_DEVICE */

/* Device driver API*/
static DEVICE_API(display, gc9d01_api) = {
    .blanking_on = gc9d01_display_blanking_on,
    .blanking_off = gc9d01_display_blanking_off,
    .write = gc9d01_write,
    .get_capabilities = gc9d01_get_capabilities,
    .set_pixel_format = gc9d01_set_pixel_format,
    .set_orientation = gc9d01_set_orientation,
};

#define GC9D01_INIT(inst)                                                      \
  GC9D01_REGS_INIT(inst);                                                      \
  static const struct gc9d01_config gc9d01_config_##inst = {                   \
      .mipi_dev = DEVICE_DT_GET(DT_INST_PARENT(inst)),                         \
      .dbi_config =                                                            \
          {                                                                    \
              .mode = MIPI_DBI_MODE_SPI_4WIRE,                                 \
              .config = MIPI_DBI_SPI_CONFIG_DT_INST(                           \
                  inst, SPI_OP_MODE_MASTER | SPI_WORD_SET(8), 0),              \
          },                                                                   \
      .pixel_format = DT_INST_PROP(inst, pixel_format),                        \
      .orientation = DT_INST_ENUM_IDX(inst, orientation),                      \
      .x_resolution = DT_INST_PROP(inst, width),                               \
      .y_resolution = DT_INST_PROP(inst, height),                              \
      .inversion = DT_INST_PROP(inst, display_inversion),                      \
      .regs = &gc9d01_regs_##inst,                                             \
  };                                                                           \
  static struct gc9d01_data gc9d01_data_##inst;                                \
  PM_DEVICE_DT_INST_DEFINE(inst, gc9d01_pm_action);                            \
  DEVICE_DT_INST_DEFINE(inst, &gc9d01_init, PM_DEVICE_DT_INST_GET(inst),       \
                        &gc9d01_data_##inst, &gc9d01_config_##inst,            \
                        POST_KERNEL, CONFIG_DISPLAY_INIT_PRIORITY,             \
                        &gc9d01_api);

DT_INST_FOREACH_STATUS_OKAY(GC9D01_INIT)
