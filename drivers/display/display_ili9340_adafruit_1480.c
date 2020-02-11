/*
 * Copyright (c) 2017 Jan Van Winkel <jan.van_winkel@dxplore.eu>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "display_ili9xxx.h"

void ili9xxx_lcd_init(struct ili9xxx_data *data)
{
	u8_t tx_data[15];

	tx_data[0] = 0x23;
	ili9xxx_transmit(data, ILI9XXX_CMD_POWER_CTRL_1, tx_data, 1);

	tx_data[0] = 0x10;
	ili9xxx_transmit(data, ILI9XXX_CMD_POWER_CTRL_2, tx_data, 1);

	tx_data[0] = 0x3e;
	tx_data[1] = 0x28;
	ili9xxx_transmit(data, ILI9XXX_CMD_VCOM_CTRL_1, tx_data, 2);

	tx_data[0] = 0x86;
	ili9xxx_transmit(data, ILI9XXX_CMD_VCOM_CTRL_2, tx_data, 1);

	tx_data[0] =
	    ILI9XXX_DATA_MEM_ACCESS_CTRL_MV | ILI9XXX_DATA_MEM_ACCESS_CTRL_BGR;
	ili9xxx_transmit(data, ILI9XXX_CMD_MEM_ACCESS_CTRL, tx_data, 1);

	tx_data[0] = 0x00;
	tx_data[1] = 0x18;
	ili9xxx_transmit(data, ILI9XXX_CMD_FRAME_CTRL_NORMAL_MODE, tx_data, 2);

	tx_data[0] = 0x08;
	tx_data[1] = 0x82;
	tx_data[2] = 0x27;
	ili9xxx_transmit(data, ILI9XXX_CMD_DISPLAY_FUNCTION_CTRL, tx_data, 3);

	tx_data[0] = 0x01;
	ili9xxx_transmit(data, ILI9XXX_CMD_GAMMA_SET, tx_data, 1);

	tx_data[0] = 0x0F;
	tx_data[1] = 0x31;
	tx_data[2] = 0x2B;
	tx_data[3] = 0x0C;
	tx_data[4] = 0x0E;
	tx_data[5] = 0x08;
	tx_data[6] = 0x4E;
	tx_data[7] = 0xF1;
	tx_data[8] = 0x37;
	tx_data[9] = 0x07;
	tx_data[10] = 0x10;
	tx_data[11] = 0x03;
	tx_data[12] = 0x0E;
	tx_data[13] = 0x09;
	tx_data[14] = 0x00;
	ili9xxx_transmit(data, ILI9XXX_CMD_POSITIVE_GAMMA_CORRECTION, tx_data,
			 15);

	tx_data[0] = 0x00;
	tx_data[1] = 0x0E;
	tx_data[2] = 0x14;
	tx_data[3] = 0x03;
	tx_data[4] = 0x11;
	tx_data[5] = 0x07;
	tx_data[6] = 0x31;
	tx_data[7] = 0xC1;
	tx_data[8] = 0x48;
	tx_data[9] = 0x08;
	tx_data[10] = 0x0F;
	tx_data[11] = 0x0C;
	tx_data[12] = 0x31;
	tx_data[13] = 0x36;
	tx_data[14] = 0x0F;
	ili9xxx_transmit(data, ILI9XXX_CMD_NEGATIVE_GAMMA_CORRECTION, tx_data,
			 15);
}
