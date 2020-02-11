/*
 * Copyright (c) 2019 Synopsys, Inc. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* SoC level DTS fixup file */

/*
 * UART configuration
 */
#define DT_UART_NS16550_PORT_0_BASE_ADDR DT_NS16550_F0005000_BASE_ADDRESS
#define DT_UART_NS16550_PORT_0_IRQ	 DT_NS16550_F0005000_IRQ_0
#define DT_UART_NS16550_PORT_0_CLK_FREQ	 DT_NS16550_F0005000_CLOCK_FREQUENCY
#define DT_UART_NS16550_PORT_0_BAUD_RATE DT_NS16550_F0005000_CURRENT_SPEED
#define DT_UART_NS16550_PORT_0_NAME	 DT_NS16550_F0005000_LABEL
#define DT_UART_NS16550_PORT_0_IRQ_PRI	 DT_NS16550_F0005000_IRQ_0_PRIORITY

/*
 * GPIO configuration
 */
#define DT_GPIO_DW_0_BASE_ADDR	DT_SNPS_DESIGNWARE_GPIO_F0003000_BASE_ADDRESS
#define DT_GPIO_DW_0_BITS		DT_SNPS_DESIGNWARE_GPIO_F0003000_BITS
#define CONFIG_GPIO_DW_0_NAME	DT_SNPS_DESIGNWARE_GPIO_F0003000_LABEL
#define DT_GPIO_DW_0_IRQ		DT_SNPS_DESIGNWARE_GPIO_F0003000_IRQ_0
#define CONFIG_GPIO_DW_0_IRQ_PRI DT_SNPS_DESIGNWARE_GPIO_F0003000_IRQ_0_PRIORITY
#define DT_GPIO_DW_0_IRQ_FLAGS	0

/*
 * SPI configuration
 */

#define DT_SPI_DW_0_BASE_ADDRESS	\
	DT_SNPS_DESIGNWARE_SPI_F0020000_BASE_ADDRESS
#define DT_SPI_DW_0_CLOCK_FREQUENCY	\
	DT_SNPS_DESIGNWARE_SPI_F0020000_CLOCK_FREQUENCY
#define DT_SPI_DW_0_NAME		DT_SNPS_DESIGNWARE_SPI_F0020000_LABEL
#define DT_SPI_DW_0_IRQ			DT_SNPS_DESIGNWARE_SPI_F0020000_IRQ_0
#define DT_SPI_DW_0_IRQ_PRI		\
	DT_SNPS_DESIGNWARE_SPI_F0020000_IRQ_0_PRIORITY
#define DT_SPI_DW_0_IRQ_FLAGS		0


#define DT_SPI_DW_1_BASE_ADDRESS	\
	DT_SNPS_DESIGNWARE_SPI_F0021000_BASE_ADDRESS
#define DT_SPI_DW_1_CLOCK_FREQUENCY	\
	DT_SNPS_DESIGNWARE_SPI_F0021000_CLOCK_FREQUENCY
#define DT_SPI_DW_1_NAME		DT_SNPS_DESIGNWARE_SPI_F0021000_LABEL
#define DT_SPI_DW_1_IRQ			DT_SNPS_DESIGNWARE_SPI_F0021000_IRQ_0
#define DT_SPI_DW_1_IRQ_PRI		\
	DT_SNPS_DESIGNWARE_SPI_F0021000_IRQ_0_PRIORITY
#define DT_SPI_DW_1_IRQ_FLAGS		0


#define DT_SPI_DW_2_BASE_ADDRESS	\
	DT_SNPS_DESIGNWARE_SPI_F0022000_BASE_ADDRESS
#define DT_SPI_DW_2_CLOCK_FREQUENCY	\
	DT_SNPS_DESIGNWARE_SPI_F0022000_CLOCK_FREQUENCY
#define DT_SPI_DW_2_NAME		DT_SNPS_DESIGNWARE_SPI_F0022000_LABEL
#define DT_SPI_DW_2_IRQ			DT_SNPS_DESIGNWARE_SPI_F0022000_IRQ_0
#define DT_SPI_DW_2_IRQ_PRI		\
	DT_SNPS_DESIGNWARE_SPI_F0022000_IRQ_0_PRIORITY
#define DT_SPI_DW_2_IRQ_FLAGS		0

/* For spi_fujistu_fram sample */
#define DT_SPI_1_NAME			DT_SPI_DW_1_NAME

/*
 * seeed TFT TOUCH SHIELD configuration
 */

#define DT_INST_0_ILITEK_ILI9XXX_BUS_NAME		DT_SPI_2_NAME
#define DT_INST_0_ILITEK_ILI9XXX_SPI_MAX_FREQUENCY		12000000
#define DT_INST_0_ILITEK_ILI9XXX_BASE_ADDRESS		1

#define DT_INST_0_ILITEK_ILI9XXX_CS_GPIOS_CONTROLLER	CONFIG_GPIO_DW_0_NAME
#define	DT_INST_0_ILITEK_ILI9XXX_CS_GPIOS_PIN		9

#define DT_INST_0_ILITEK_ILI9XXX_CMD_DATA_GPIOS_CONTROLLER \
	CONFIG_GPIO_DW_0_NAME
#define	DT_INST_0_ILITEK_ILI9XXX_CMD_DATA_GPIOS_PIN		21

#define DT_INST_0_ILITEK_ILI9XXX_LABEL	"DISPLAY"

/* End of SoC Level DTS fixup file */
