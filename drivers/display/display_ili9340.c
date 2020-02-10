/*
 * Copyright (c) 2017 Jan Van Winkel <jan.van_winkel@dxplore.eu>
 * Copyright (c) 2019 Nordic Semiconductor ASA
 * Copyright (c) 2020 Pavlo Hamov <pasha.gamov@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "display_ili9340.h"
#include <drivers/display.h>

#include <drivers/gpio.h>
#include <sys/byteorder.h>
#include <drivers/spi.h>
#include <string.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(display_ili9340, CONFIG_DISPLAY_LOG_LEVEL);

#if (DT_INST_0_ILITEK_ILI9XXX_TYPE_ENUM == 0)
#define ILI9XXX_X_MAX 132U
#define ILI9XXX_Y_MAX 162U
#else
#define ILI9XXX_X_MAX 240U
#define ILI9XXX_Y_MAX 320U
#endif

#define RES_VER DT_INST_0_ILITEK_ILI9XXX_HEIGHT
#define RES_HOR DT_INST_0_ILITEK_ILI9XXX_WIDTH

#define DEV_TYPE DT_INST_0_ILITEK_ILI9XXX_TYPE

#if (RES_VER > ILI9XXX_Y_MAX) && (RES_VER > ILI9XXX_X_MAX)
#error "Please check height of \"ilitek,"DEV_TYPE""\" device"
#endif

#if (RES_HOR > ILI9XXX_X_MAX) && (RES_HOR > ILI9XXX_Y_MAX)
#error "Please check height of \"ilitek,ili9340\" device"
#endif

#if defined(DT_INST_0_ILITEK_ILI9XXX_X_OFFSET) && \
	(DT_INST_0_ILITEK_ILI9XXX_X_OFFSET > (ILI9XXX_X_MAX - RES_HOR))
#error "Please check x-offset of \"ilitek,ili9340\" device"
#endif

#if defined(DT_INST_0_ILITEK_ILI9XXX_Y_OFFSET) && \
	(DT_INST_0_ILITEK_ILI9XXX_Y_OFFSET > (ILI9XXX_Y_MAX - RES_VER))
#error "Please check y-offset of \"ilitek,ili9340\" device"
#endif

struct ili9xxx_data {
#ifdef DT_INST_0_ILITEK_ILI9XXX_RESET_GPIOS_CONTROLLER
	struct device *rst;
#endif
	struct device *cmd;
	struct device *spi_dev;
	struct spi_config spi_config;
#ifdef DT_INST_0_ILITEK_ILI9XXX_CS_GPIOS_CONTROLLER
	struct spi_cs_control cs_ctrl;
#endif
	enum display_pixel_format pixel_format;
	enum display_orientation orientation;
	u16_t height;
	u16_t width;
	u16_t x_offset;
	u16_t y_offset;
};

static int ili9xxx_transmit(struct ili9xxx_data *data, u8_t cmd, void *tx_data,
		      size_t tx_len)
{
	int rv = 0;
	struct spi_buf tx_buf = { .buf = &cmd, .len = 1 };
	struct spi_buf_set tx_bufs = { .buffers = &tx_buf, .count = 1 };

	gpio_pin_set(data->cmd,
			DT_INST_0_ILITEK_ILI9XXX_CMD_DATA_GPIOS_PIN, 0);
	rv = spi_write(data->spi_dev, &data->spi_config, &tx_bufs);
	if (rv) {
		LOG_ERR("Failed to dend cmd 0x%02X", cmd);
		return rv;
	}

	if (tx_data != NULL) {
		tx_buf.buf = tx_data;
		tx_buf.len = tx_len;
		gpio_pin_set(data->cmd,
			       DT_INST_0_ILITEK_ILI9XXX_CMD_DATA_GPIOS_PIN, 1);
		rv = spi_write(data->spi_dev, &data->spi_config, &tx_bufs);
	}
	return rv;
}

static void ili9xxx_reset(struct ili9xxx_data *data)
{
	LOG_DBG("Resetting display");
#ifdef DT_INST_0_ILITEK_ILI9XXX_RESET_GPIOS_CONTROLLER
	gpio_pin_set(data->rst, DT_INST_0_ILITEK_ILI9XXX_RESET_GPIOS_PIN, 1);
	k_busy_wait(5);
	gpio_pin_set(data->rst, DT_INST_0_ILITEK_ILI9XXX_RESET_GPIOS_PIN, 0);
	k_busy_wait(10);
	gpio_pin_set(data->rst, DT_INST_0_ILITEK_ILI9XXX_RESET_GPIOS_PIN, 1);
#else
	ili9xxx_transmit(data, ILI9XXX_CMD_SOFTWARE_RESET, NULL, 0);
#endif
	k_sleep(K_MSEC(120));
}

static void ili9xxx_exit_sleep(struct ili9xxx_data *data)
{
	ili9xxx_transmit(data, ILI9XXX_CMD_EXIT_SLEEP, NULL, 0);
	k_sleep(K_MSEC(120));
}

static void ili9xxx_set_mem_area(struct ili9xxx_data *data, const u16_t x,
				 const u16_t y, const u16_t w, const u16_t h)
{
	u16_t spi_data[2];

	spi_data[0] = sys_cpu_to_be16(x);
	spi_data[1] = sys_cpu_to_be16(x + w - 1);
	ili9xxx_transmit(data, ILI9XXX_CMD_COLUMN_ADDR, &spi_data[0], 4);

	spi_data[0] = sys_cpu_to_be16(y);
	spi_data[1] = sys_cpu_to_be16(y + h - 1);
	ili9xxx_transmit(data, ILI9XXX_CMD_PAGE_ADDR, &spi_data[0], 4);
}

static int ili9xxx_write(const struct device *dev, const u16_t x,
			 const u16_t y,
			 const struct display_buffer_descriptor *desc,
			 const void *buf)
{
	struct ili9xxx_data *data = (struct ili9xxx_data *)dev->driver_data;
	const u8_t *write_data_start = (u8_t *) buf;
	struct spi_buf tx_buf;
	struct spi_buf_set tx_bufs;
	u16_t write_cnt;
	u16_t nbr_of_writes;
	u16_t write_h;
	const u8_t rgb_size = 2 + (data->pixel_format == PIXEL_FORMAT_RGB_888);

	__ASSERT(desc->width <= desc->pitch, "Pitch is smaller then width");
	__ASSERT((desc->pitch * rgb_size * desc->height) <= desc->bu_size,
			"Input buffer to small");

	LOG_DBG("Writing %dx%d (w,h) @ %dx%d (x,y)", desc->width, desc->height,
			x, y);
	ili9xxx_set_mem_area(data, x, y, desc->width, desc->height);

	if (desc->pitch > desc->width) {
		write_h = 1U;
		nbr_of_writes = desc->height;
	} else {
		write_h = desc->height;
		nbr_of_writes = 1U;
	}

	ili9xxx_transmit(data, ILI9XXX_CMD_MEM_WRITE,
			 (void *) write_data_start,
			 desc->width * rgb_size * write_h);

	tx_bufs.buffers = &tx_buf;
	tx_bufs.count = 1;

	write_data_start += (desc->pitch * rgb_size);
	for (write_cnt = 1U; write_cnt < nbr_of_writes; ++write_cnt) {
		tx_buf.buf = (void *)write_data_start;
		tx_buf.len = desc->width * rgb_size * write_h;
		spi_write(data->spi_dev, &data->spi_config, &tx_bufs);
		write_data_start += (desc->pitch * rgb_size);
	}

	return 0;
}

static int ili9xxx_read(const struct device *dev, const u16_t x,
			const u16_t y,
			const struct display_buffer_descriptor *desc,
			void *buf)
{
	LOG_ERR("Reading not supported");
	return -ENOTSUP;
}

static void *ili9xxx_get_framebuffer(const struct device *dev)
{
	LOG_ERR("Direct framebuffer access not supported");
	return NULL;
}

static int ili9xxx_blanking_off(const struct device *dev)
{
	struct ili9xxx_data *data = (struct ili9xxx_data *)dev->driver_data;

	LOG_DBG("Turning display blanking off");
	return ili9xxx_transmit(data, ILI9XXX_CMD_DISPLAY_ON, NULL, 0);
}

static int ili9xxx_blanking_on(const struct device *dev)
{
	struct ili9xxx_data *data = (struct ili9xxx_data *)dev->driver_data;

	LOG_DBG("Turning display blanking on");
	return ili9xxx_transmit(data, ILI9XXX_CMD_DISPLAY_OFF, NULL, 0);
}

static int ili9xxx_set_brightness(const struct device *dev,
				  const u8_t brightness)
{
	LOG_WRN("Set brightness not implemented");
	return -ENOTSUP;
}

static int ili9xxx_set_contrast(const struct device *dev, const u8_t contrast)
{
	LOG_ERR("Set contrast not supported");
	return -ENOTSUP;
}

static int ili9xxx_set_pixel_format(const struct device *dev,
				    const enum display_pixel_format
				    pixel_format)
{
	struct ili9xxx_data *data = (struct ili9xxx_data *)dev->driver_data;
	u8_t arg;
	int rv = 0;

	LOG_DBG("Format change %d -> %d", data->pixel_format, pixel_format);

	if (data->pixel_format == pixel_format) {
		return 0;
	} else if (pixel_format == PIXEL_FORMAT_RGB_565) {
		arg = ILI9XXX_DATA_PIXEL_FORMAT_MCU_16_BIT |
				ILI9XXX_DATA_PIXEL_FORMAT_RGB_16_BIT;
	} else if (pixel_format == PIXEL_FORMAT_RGB_888) {
		arg = ILI9XXX_DATA_PIXEL_FORMAT_MCU_18_BIT |
				ILI9XXX_DATA_PIXEL_FORMAT_RGB_18_BIT;
	} else {
		LOG_ERR("Pixel format 0x%X is not supported", pixel_format);
		return -ENOTSUP;
	}

	rv = ili9xxx_transmit(data, ILI9XXX_CMD_PIXEL_FORMAT_SET, &arg, 1);
	if (rv) {
		LOG_ERR("Failed to change pix fmt %d -> %d",
				data->pixel_format, pixel_format);
		return rv;
	}
	data->pixel_format = pixel_format;
	return 0;
}


static int ili9xxx_set_orientation(const struct device *dev,
				   const enum display_orientation orientation)
{
	u8_t cmd;
	struct ili9xxx_data *data = (struct ili9xxx_data *)dev->driver_data;

	if (orientation == data->orientation) {
		return 0;
	}
	/* todo: check if square */
	switch (orientation) {
	case DISPLAY_ORIENTATION_NORMAL:
		cmd = 0;
		break;
	case DISPLAY_ORIENTATION_ROTATED_90:
		cmd = ILI9XXX_DATA_MEM_ACCESS_CTRL_MV
			| ILI9XXX_DATA_MEM_ACCESS_CTRL_MY;
		break;
	case DISPLAY_ORIENTATION_ROTATED_180:
		cmd = ILI9XXX_DATA_MEM_ACCESS_CTRL_MX
			| ILI9XXX_DATA_MEM_ACCESS_CTRL_MY;
		break;
	case DISPLAY_ORIENTATION_ROTATED_270:
		cmd = ILI9XXX_DATA_MEM_ACCESS_CTRL_MV
			| ILI9XXX_DATA_MEM_ACCESS_CTRL_MX;
		break;
	default:
		return -EINVAL;
	}
#ifdef DT_INST_0_ILITEK_ILI9XXX_MADC
	cmd |= DT_INST_0_ILITEK_ILI9XXX_MADC;
#else
	cmd |= ILI9XXX_DATA_MEM_ACCESS_CTRL_BGR;
#endif
	data->orientation = orientation;
	return ili9xxx_transmit(data, ILI9XXX_CMD_MEM_ACCESS_CTRL, &cmd, sizeof(cmd));
}

static void ili9xxx_get_capabilities(const struct device *dev,
				     struct display_capabilities *capabilities)
{
	const struct ili9xxx_data *data = (struct ili9xxx_data *)dev->driver_data;
	memset(capabilities, 0, sizeof(struct display_capabilities));
	capabilities->x_resolution = RES_HOR;
	capabilities->y_resolution = RES_VER;
	capabilities->supported_pixel_formats =
			PIXEL_FORMAT_RGB_565 | PIXEL_FORMAT_RGB_888;
	capabilities->current_pixel_format = data->pixel_format;
	capabilities->current_orientation = data->orientation;
}

#define EXEC_CMD(cmd) { \
	u8_t _cmd = ILI9XXX_CMD_##cmd; \
	u8_t _arr[] = DT_INST_0_ILITEK_ILI9XXX_##cmd; \
	int _rv = ili9xxx_transmit(data, _cmd, _arr, sizeof(_arr)); \
	if (_rv) { \
		LOG_ERR("CMD "#cmd" failed %d", _rv); \
		return _rv; \
	} \
}

static int ili9xxx_lcd_init(struct device *dev)
{
	struct ili9xxx_data *data = (struct ili9xxx_data *)dev->driver_data;

#ifdef DT_INST_0_ILITEK_ILI9XXX_POWER_CTRL_A
	EXEC_CMD(POWER_CTRL_A);
#endif

#ifdef DT_INST_0_ILITEK_ILI9XXX_POWER_CTRL_B
	EXEC_CMD(POWER_CTRL_B);
#endif

#ifdef DT_INST_0_ILITEK_ILI9XXX_POWER_ON_SEQ_CTRL
	EXEC_CMD(POWER_ON_SEQ_CTRL);
#endif

#ifdef DT_INST_0_ILITEK_ILI9XXX_DRVR_TIMING_CTRL_A_I
	EXEC_CMD(DRVR_TIMING_CTRL_A_I);
#endif

#ifdef DT_INST_0_ILITEK_ILI9XXX_PUMP_RATIO_CTRL
	EXEC_CMD(PUMP_RATIO_CTRL);
#endif

#ifdef DT_INST_0_ILITEK_ILI9XXX_DRVR_TIMING_CTRL_B
	EXEC_CMD(DRVR_TIMING_CTRL_B);
#endif

#ifdef DT_INST_0_ILITEK_ILI9XXX_POWER_CTRL_1
	EXEC_CMD(POWER_CTRL_1);
#endif

#ifdef DT_INST_0_ILITEK_ILI9XXX_POWER_CTRL_2
	EXEC_CMD(POWER_CTRL_2);
#endif

#ifdef DT_INST_0_ILITEK_ILI9XXX_VCOM_CTRL_1
	EXEC_CMD(VCOM_CTRL_1);
#endif

#ifdef DT_INST_0_ILITEK_ILI9XXX_VCOM_CTRL_2
	EXEC_CMD(VCOM_CTRL_2);
#endif

#ifdef DT_INST_0_ILITEK_ILI9XXX_MEM_ACCESS_CTRL
	EXEC_CMD(MEM_ACCESS_CTRL);
#endif

	u8_t arg;
	if (data->pixel_format == PIXEL_FORMAT_RGB_565) {
		arg = ILI9XXX_DATA_PIXEL_FORMAT_MCU_16_BIT |
				ILI9XXX_DATA_PIXEL_FORMAT_RGB_16_BIT;
	} else if (data->pixel_format == PIXEL_FORMAT_RGB_888) {
		arg = ILI9XXX_DATA_PIXEL_FORMAT_MCU_18_BIT |
				ILI9XXX_DATA_PIXEL_FORMAT_RGB_18_BIT;
	} else {
		LOG_ERR("Unsupported pixel format 0x%X", data->pixel_format);
		return -EINVAL;
	}
	ili9xxx_transmit(data, ILI9XXX_CMD_PIXEL_FORMAT_SET, &arg, 1);

#ifdef DT_INST_0_ILITEK_ILI9XXX_FRAME_CTRL_NORMAL_MODE
	EXEC_CMD(FRAME_CTRL_NORMAL_MODE);
#endif

#ifdef DT_INST_0_ILITEK_ILI9XXX_DISPLAY_FUNCTION_CTRL
	EXEC_CMD(DISPLAY_FUNCTION_CTRL);
#endif

#ifdef DT_INST_0_ILITEK_ILI9XXX_ENABLE_3G
	EXEC_CMD(ENABLE_3G);
#endif

#ifdef DT_INST_0_ILITEK_ILI9XXX_GAMMA_SET
	EXEC_CMD(GAMMA_SET);
#endif

#ifdef DT_INST_0_ILITEK_ILI9XXX_POSITIVE_GAMMA_CORRECTION
	EXEC_CMD(POSITIVE_GAMMA_CORRECTION);
#endif

#ifdef DT_INST_0_ILITEK_ILI9XXX_NEGATIVE_GAMMA_CORRECTION
	EXEC_CMD(NEGATIVE_GAMMA_CORRECTION);
#endif

	return 0;
}

static int ili9xxx_init(struct device *dev)
{
	struct ili9xxx_data *data = (struct ili9xxx_data *)dev->driver_data;

	data->pixel_format = !DT_INST_0_ILITEK_ILI9XXX_PIXEL_FORMAT_ENUM ?
			PIXEL_FORMAT_RGB_565 : PIXEL_FORMAT_RGB_888;
	data->orientation = DISPLAY_ORIENTATION_NORMAL;

	LOG_DBG("Initializing display driver");

	data->spi_dev = device_get_binding(DT_INST_0_ILITEK_ILI9XXX_BUS_NAME);
	if (data->spi_dev == NULL) {
		LOG_ERR("No '%s' device", DT_INST_0_ILITEK_ILI9XXX_BUS_NAME);
		return -EPERM;
	}

	data->spi_config.frequency = DT_INST_0_ILITEK_ILI9XXX_SPI_MAX_FREQUENCY;
	data->spi_config.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8);
	data->spi_config.slave = DT_INST_0_ILITEK_ILI9XXX_BASE_ADDRESS;

#ifdef DT_INST_0_ILITEK_ILI9XXX_CS_GPIOS_CONTROLLER
	data->cs_ctrl.gpio_dev = device_get_binding(
			DT_INST_0_ILITEK_ILI9XXX_CS_GPIOS_CONTROLLER);
	data->cs_ctrl.gpio_pin = DT_INST_0_ILITEK_ILI9XXX_CS_GPIOS_PIN;
	data->cs_ctrl.delay = 0U;
	data->spi_config.cs = &(data->cs_ctrl);
#else
	data->spi_config.cs = NULL;
#endif

#ifdef DT_INST_0_ILITEK_ILI9XXX_RESET_GPIOS_CONTROLLER
	data->rst = device_get_binding(
			DT_INST_0_ILITEK_ILI9XXX_RESET_GPIOS_CONTROLLER);
	if (data->rst == NULL) {
		LOG_ERR("No '%s' device",
			DT_INST_0_ILITEK_ILI9XXX_RESET_GPIOS_CONTROLLER);
		return -EPERM;
	}

	gpio_pin_configure(data->rst, DT_INST_0_ILITEK_ILI9XXX_RESET_GPIOS_PIN,
			GPIO_OUTPUT_INACTIVE | DT_INST_0_ILITEK_ILI9XXX_RESET_GPIOS_FLAGS);
#endif

	data->cmd = device_get_binding(
			DT_INST_0_ILITEK_ILI9XXX_CMD_DATA_GPIOS_CONTROLLER);
	if (data->cmd == NULL) {
		LOG_ERR("No '%s' device",
			DT_INST_0_ILITEK_ILI9XXX_CMD_DATA_GPIOS_CONTROLLER);
		return -EPERM;
	}

	gpio_pin_configure(data->cmd,
		DT_INST_0_ILITEK_ILI9XXX_CMD_DATA_GPIOS_PIN,
		GPIO_OUTPUT_INACTIVE | DT_INST_0_ILITEK_ILI9XXX_CMD_DATA_GPIOS_FLAGS);

	LOG_DBG("Initializing LCD");

	ili9xxx_reset(data);

	ili9xxx_blanking_on(dev);

	ili9xxx_lcd_init(dev);

	ili9xxx_exit_sleep(data);

	return 0;
}

static const struct display_driver_api ili9xxx_api = {
	.blanking_on = ili9xxx_blanking_on,
	.blanking_off = ili9xxx_blanking_off,
	.write = ili9xxx_write,
	.read = ili9xxx_read,
	.get_framebuffer = ili9xxx_get_framebuffer,
	.set_brightness = ili9xxx_set_brightness,
	.set_contrast = ili9xxx_set_contrast,
	.get_capabilities = ili9xxx_get_capabilities,
	.set_pixel_format = ili9xxx_set_pixel_format,
	.set_orientation = ili9xxx_set_orientation,
};

static struct ili9xxx_data ili9xxx_data;

DEVICE_AND_API_INIT(ili9340, DT_INST_0_ILITEK_ILI9XXX_LABEL, &ili9xxx_init,
		    &ili9xxx_data, NULL, APPLICATION,
		    CONFIG_APPLICATION_INIT_PRIORITY, &ili9xxx_api);
