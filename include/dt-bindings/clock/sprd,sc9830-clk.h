/* SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause */
#ifndef _DT_BINDINGS_CLK_SC9830_H_
#define _DT_BINDINGS_CLK_SC9830_H_

#define CLK_ADI_EB	0
#define CLK_SPLK_EB	1
#define CLK_SINDRV	2
#define CLK_AON_GATE_NUM	3
#define CLK_TWPLL	3
#define CLK_TWPLL_48M	4
#define CLK_TWPLL_51M2	5
#define CLK_TWPLL_153M6	6
#define CLK_AUX0_MUX	7
#define CLK_AUX0		8
#define CLK_TWPLL_96M	9
#define CLK_TWPLL_256M	10
#define CLK_TWPLL_307M2	11
#define CLK_TWPLL_384M	12
#define CLK_TWPLL_76M8	13
#define CLK_TWPLL_128M	14
#define CLK_AON_TMR_EB	15
#define CLK_AON_INTC_EB	16
#define CLK_DPLL		17
#define CLK_TWPLL_192M	18
#define CLK_TWPLL_512M	19
#define CLK_TWPLL_614M4	20
#define CLK_TWPLL_768M	21
#define CLK_MBOX_EB	22
#define CLK_AP_TMR0_EB	23
#define CLK_AP_TMR1_EB	24
#define CLK_AP_TMR2_EB	25
#define CLK_MPLL		26
#define CLK_MPLL_DIV32	27
#define CLK_LTEPLL	28
#define CLK_GPU_EB	29
#define CLK_AON_CLK_NUM	30

#define CLK_PMU_LTEPLL_GATE	0
#define CLK_PMU_LTEPLL_614M4	1
#define CLK_PMU_CLK_NUM		2

#define CLK_EMC		0
#define CLK_AON_MUX_NUM	1

#define CLK_I2C2		0
#define CLK_SDIO0		1
#define CLK_UART0		2
#define CLK_SDIO1		3
#define CLK_EMMC		4
#define CLK_AP_CLK_NUM	5

#define CLK_I2C2_EB		0
#define CLK_UART0_EB		1
#define CLK_INTC0_EB	2
#define CLK_INTC1_EB	3
#define CLK_INTC2_EB	4
#define CLK_INTC3_EB	5
#define CLK_AP_APB_GATE_NUM	6

#define CLK_SDIO0_EB		0
#define CLK_USB_EB		1
#define CLK_SDIO1_EB		2
#define CLK_EMMC_EB		3
#define CLK_CA7_MCU		4
#define CLK_CA7_AXI		5
#define CLK_AP_AHB_GATE_NUM	6

#endif
