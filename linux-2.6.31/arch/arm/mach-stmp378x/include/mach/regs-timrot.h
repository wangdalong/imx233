/*
 * STMP TIMROT Register Definitions
 *
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 * This file is created by xml file. Don't Edit it.
 */

#ifndef __ARCH_ARM___TIMROT_H
#define __ARCH_ARM___TIMROT_H  1

#define REGS_TIMROT_BASE (STMP3XXX_REGS_BASE + 0x68000)
#define REGS_TIMROT_PHYS (0x80068000)
#define REGS_TIMROT_SIZE 0x00002000

#define HW_TIMROT_ROTCTRL	(0x00000000)
#define HW_TIMROT_ROTCTRL_SET	(0x00000004)
#define HW_TIMROT_ROTCTRL_CLR	(0x00000008)
#define HW_TIMROT_ROTCTRL_TOG	(0x0000000c)
#define HW_TIMROT_ROTCTRL_ADDR  \
		(REGS_TIMROT_BASE + HW_TIMROT_ROTCTRL)
#define HW_TIMROT_ROTCTRL_SET_ADDR \
		(REGS_TIMROT_BASE + HW_TIMROT_ROTCTRL_SET)
#define HW_TIMROT_ROTCTRL_CLR_ADDR \
		(REGS_TIMROT_BASE + HW_TIMROT_ROTCTRL_CLR)
#define HW_TIMROT_ROTCTRL_TOG_ADDR \
		(REGS_TIMROT_BASE + HW_TIMROT_ROTCTRL_TOG)

#define BM_TIMROT_ROTCTRL_SFTRST	0x80000000
#define BM_TIMROT_ROTCTRL_CLKGATE	0x40000000
#define BM_TIMROT_ROTCTRL_ROTARY_PRESENT	0x20000000
#define BM_TIMROT_ROTCTRL_TIM3_PRESENT	0x10000000
#define BM_TIMROT_ROTCTRL_TIM2_PRESENT	0x08000000
#define BM_TIMROT_ROTCTRL_TIM1_PRESENT	0x04000000
#define BM_TIMROT_ROTCTRL_TIM0_PRESENT	0x02000000
#define BP_TIMROT_ROTCTRL_STATE	22
#define BM_TIMROT_ROTCTRL_STATE	0x01C00000
#define BF_TIMROT_ROTCTRL_STATE(v)  \
		(((v) << 22) & BM_TIMROT_ROTCTRL_STATE)
#define BP_TIMROT_ROTCTRL_DIVIDER	16
#define BM_TIMROT_ROTCTRL_DIVIDER	0x003F0000
#define BF_TIMROT_ROTCTRL_DIVIDER(v)  \
		(((v) << 16) & BM_TIMROT_ROTCTRL_DIVIDER)
#define BP_TIMROT_ROTCTRL_RSRVD3	13
#define BM_TIMROT_ROTCTRL_RSRVD3	0x0000E000
#define BF_TIMROT_ROTCTRL_RSRVD3(v)  \
		(((v) << 13) & BM_TIMROT_ROTCTRL_RSRVD3)
#define BM_TIMROT_ROTCTRL_RELATIVE	0x00001000
#define BP_TIMROT_ROTCTRL_OVERSAMPLE	10
#define BM_TIMROT_ROTCTRL_OVERSAMPLE	0x00000C00
#define BF_TIMROT_ROTCTRL_OVERSAMPLE(v)  \
		(((v) << 10) & BM_TIMROT_ROTCTRL_OVERSAMPLE)
#define BV_TIMROT_ROTCTRL_OVERSAMPLE__8X 0x0
#define BV_TIMROT_ROTCTRL_OVERSAMPLE__4X 0x1
#define BV_TIMROT_ROTCTRL_OVERSAMPLE__2X 0x2
#define BV_TIMROT_ROTCTRL_OVERSAMPLE__1X 0x3
#define BM_TIMROT_ROTCTRL_POLARITY_B	0x00000200
#define BM_TIMROT_ROTCTRL_POLARITY_A	0x00000100
#define BM_TIMROT_ROTCTRL_RSRVD2	0x00000080
#define BP_TIMROT_ROTCTRL_SELECT_B	4
#define BM_TIMROT_ROTCTRL_SELECT_B	0x00000070
#define BF_TIMROT_ROTCTRL_SELECT_B(v)  \
		(((v) << 4) & BM_TIMROT_ROTCTRL_SELECT_B)
#define BV_TIMROT_ROTCTRL_SELECT_B__NEVER_TICK 0x0
#define BV_TIMROT_ROTCTRL_SELECT_B__PWM0       0x1
#define BV_TIMROT_ROTCTRL_SELECT_B__PWM1       0x2
#define BV_TIMROT_ROTCTRL_SELECT_B__PWM2       0x3
#define BV_TIMROT_ROTCTRL_SELECT_B__PWM3       0x4
#define BV_TIMROT_ROTCTRL_SELECT_B__PWM4       0x5
#define BV_TIMROT_ROTCTRL_SELECT_B__ROTARYA    0x6
#define BV_TIMROT_ROTCTRL_SELECT_B__ROTARYB    0x7
#define BM_TIMROT_ROTCTRL_RSRVD1	0x00000008
#define BP_TIMROT_ROTCTRL_SELECT_A	0
#define BM_TIMROT_ROTCTRL_SELECT_A	0x00000007
#define BF_TIMROT_ROTCTRL_SELECT_A(v)  \
		(((v) << 0) & BM_TIMROT_ROTCTRL_SELECT_A)
#define BV_TIMROT_ROTCTRL_SELECT_A__NEVER_TICK 0x0
#define BV_TIMROT_ROTCTRL_SELECT_A__PWM0       0x1
#define BV_TIMROT_ROTCTRL_SELECT_A__PWM1       0x2
#define BV_TIMROT_ROTCTRL_SELECT_A__PWM2       0x3
#define BV_TIMROT_ROTCTRL_SELECT_A__PWM3       0x4
#define BV_TIMROT_ROTCTRL_SELECT_A__PWM4       0x5
#define BV_TIMROT_ROTCTRL_SELECT_A__ROTARYA    0x6
#define BV_TIMROT_ROTCTRL_SELECT_A__ROTARYB    0x7

#define HW_TIMROT_ROTCOUNT	(0x00000010)
#define HW_TIMROT_ROTCOUNT_ADDR \
		(REGS_TIMROT_BASE + HW_TIMROT_ROTCOUNT)

#define BP_TIMROT_ROTCOUNT_RSRVD1	16
#define BM_TIMROT_ROTCOUNT_RSRVD1	0xFFFF0000
#define BF_TIMROT_ROTCOUNT_RSRVD1(v) \
		(((v) << 16) & BM_TIMROT_ROTCOUNT_RSRVD1)
#define BP_TIMROT_ROTCOUNT_UPDOWN	0
#define BM_TIMROT_ROTCOUNT_UPDOWN	0x0000FFFF
#define BF_TIMROT_ROTCOUNT_UPDOWN(v)  \
		(((v) << 0) & BM_TIMROT_ROTCOUNT_UPDOWN)

/*
 *  multi-register-define name HW_TIMROT_TIMCTRLn
 *              base 0x00000020
 *              count 3
 *              offset 0x20
 */
#define HW_TIMROT_TIMCTRLn(n)	(0x00000020 + (n) * 0x20)
#define HW_TIMROT_TIMCTRLn_SET(n)	(0x00000024 + (n) * 0x20)
#define HW_TIMROT_TIMCTRLn_CLR(n)	(0x00000028 + (n) * 0x20)
#define HW_TIMROT_TIMCTRLn_TOG(n)	(0x0000002c + (n) * 0x20)
#define HW_TIMROT_TIMCTRLn_ADDR(n) \
		(REGS_TIMROT_BASE + HW_TIMROT_TIMCTRLn(n))
#define HW_TIMROT_TIMCTRLn_SET_ADDR(n) \
		(REGS_TIMROT_BASE + HW_TIMROT_TIMCTRLn_SET(n))
#define HW_TIMROT_TIMCTRLn_CLR_ADDR(n) \
		(REGS_TIMROT_BASE + HW_TIMROT_TIMCTRLn_CLR(n))
#define HW_TIMROT_TIMCTRLn_TOG_ADDR(n) \
		(REGS_TIMROT_BASE + HW_TIMROT_TIMCTRLn_TOG(n))
#define BP_TIMROT_TIMCTRLn_RSRVD2	16
#define BM_TIMROT_TIMCTRLn_RSRVD2	0xFFFF0000
#define BF_TIMROT_TIMCTRLn_RSRVD2(v) \
		(((v) << 16) & BM_TIMROT_TIMCTRLn_RSRVD2)
#define BM_TIMROT_TIMCTRLn_IRQ	0x00008000
#define BM_TIMROT_TIMCTRLn_IRQ_EN	0x00004000
#define BP_TIMROT_TIMCTRLn_RSRVD1	9
#define BM_TIMROT_TIMCTRLn_RSRVD1	0x00003E00
#define BF_TIMROT_TIMCTRLn_RSRVD1(v)  \
		(((v) << 9) & BM_TIMROT_TIMCTRLn_RSRVD1)
#define BM_TIMROT_TIMCTRLn_POLARITY	0x00000100
#define BM_TIMROT_TIMCTRLn_UPDATE	0x00000080
#define BM_TIMROT_TIMCTRLn_RELOAD	0x00000040
#define BP_TIMROT_TIMCTRLn_PRESCALE	4
#define BM_TIMROT_TIMCTRLn_PRESCALE	0x00000030
#define BF_TIMROT_TIMCTRLn_PRESCALE(v)  \
		(((v) << 4) & BM_TIMROT_TIMCTRLn_PRESCALE)
#define BV_TIMROT_TIMCTRLn_PRESCALE__DIV_BY_1 0x0
#define BV_TIMROT_TIMCTRLn_PRESCALE__DIV_BY_2 0x1
#define BV_TIMROT_TIMCTRLn_PRESCALE__DIV_BY_4 0x2
#define BV_TIMROT_TIMCTRLn_PRESCALE__DIV_BY_8 0x3
#define BP_TIMROT_TIMCTRLn_SELECT	0
#define BM_TIMROT_TIMCTRLn_SELECT	0x0000000F
#define BF_TIMROT_TIMCTRLn_SELECT(v)  \
		(((v) << 0) & BM_TIMROT_TIMCTRLn_SELECT)
#define BV_TIMROT_TIMCTRLn_SELECT__NEVER_TICK  0x0
#define BV_TIMROT_TIMCTRLn_SELECT__PWM0        0x1
#define BV_TIMROT_TIMCTRLn_SELECT__PWM1        0x2
#define BV_TIMROT_TIMCTRLn_SELECT__PWM2        0x3
#define BV_TIMROT_TIMCTRLn_SELECT__PWM3        0x4
#define BV_TIMROT_TIMCTRLn_SELECT__PWM4        0x5
#define BV_TIMROT_TIMCTRLn_SELECT__ROTARYA     0x6
#define BV_TIMROT_TIMCTRLn_SELECT__ROTARYB     0x7
#define BV_TIMROT_TIMCTRLn_SELECT__32KHZ_XTAL  0x8
#define BV_TIMROT_TIMCTRLn_SELECT__8KHZ_XTAL   0x9
#define BV_TIMROT_TIMCTRLn_SELECT__4KHZ_XTAL   0xA
#define BV_TIMROT_TIMCTRLn_SELECT__1KHZ_XTAL   0xB
#define BV_TIMROT_TIMCTRLn_SELECT__TICK_ALWAYS 0xC

/*
 *  multi-register-define name HW_TIMROT_TIMCOUNTn
 *              base 0x00000030
 *              count 3
 *              offset 0x20
 */
#define HW_TIMROT_TIMCOUNTn(n)	(0x00000030 + (n) * 0x20)
#define HW_TIMROT_TIMCOUNTn_ADDR(n) \
		(REGS_TIMROT_BASE + HW_TIMROT_TIMCOUNTn(n))
#define BP_TIMROT_TIMCOUNTn_RUNNING_COUNT	16
#define BM_TIMROT_TIMCOUNTn_RUNNING_COUNT	0xFFFF0000
#define BF_TIMROT_TIMCOUNTn_RUNNING_COUNT(v) \
		(((v) << 16) & BM_TIMROT_TIMCOUNTn_RUNNING_COUNT)
#define BP_TIMROT_TIMCOUNTn_FIXED_COUNT	0
#define BM_TIMROT_TIMCOUNTn_FIXED_COUNT	0x0000FFFF
#define BF_TIMROT_TIMCOUNTn_FIXED_COUNT(v)  \
		(((v) << 0) & BM_TIMROT_TIMCOUNTn_FIXED_COUNT)

#define HW_TIMROT_TIMCTRL3	(0x00000080)
#define HW_TIMROT_TIMCTRL3_SET	(0x00000084)
#define HW_TIMROT_TIMCTRL3_CLR	(0x00000088)
#define HW_TIMROT_TIMCTRL3_TOG	(0x0000008c)
#define HW_TIMROT_TIMCTRL3_ADDR  \
		(REGS_TIMROT_BASE + HW_TIMROT_TIMCTRL3)
#define HW_TIMROT_TIMCTRL3_SET_ADDR \
		(REGS_TIMROT_BASE + HW_TIMROT_TIMCTRL3_SET)
#define HW_TIMROT_TIMCTRL3_CLR_ADDR \
		(REGS_TIMROT_BASE + HW_TIMROT_TIMCTRL3_CLR)
#define HW_TIMROT_TIMCTRL3_TOG_ADDR \
		(REGS_TIMROT_BASE + HW_TIMROT_TIMCTRL3_TOG)

#define BP_TIMROT_TIMCTRL3_RSRVD2	20
#define BM_TIMROT_TIMCTRL3_RSRVD2	0xFFF00000
#define BF_TIMROT_TIMCTRL3_RSRVD2(v) \
		(((v) << 20) & BM_TIMROT_TIMCTRL3_RSRVD2)
#define BP_TIMROT_TIMCTRL3_TEST_SIGNAL	16
#define BM_TIMROT_TIMCTRL3_TEST_SIGNAL	0x000F0000
#define BF_TIMROT_TIMCTRL3_TEST_SIGNAL(v)  \
		(((v) << 16) & BM_TIMROT_TIMCTRL3_TEST_SIGNAL)
#define BV_TIMROT_TIMCTRL3_TEST_SIGNAL__NEVER_TICK  0x0
#define BV_TIMROT_TIMCTRL3_TEST_SIGNAL__PWM0        0x1
#define BV_TIMROT_TIMCTRL3_TEST_SIGNAL__PWM1        0x2
#define BV_TIMROT_TIMCTRL3_TEST_SIGNAL__PWM2        0x3
#define BV_TIMROT_TIMCTRL3_TEST_SIGNAL__PWM3        0x4
#define BV_TIMROT_TIMCTRL3_TEST_SIGNAL__PWM4        0x5
#define BV_TIMROT_TIMCTRL3_TEST_SIGNAL__ROTARYA     0x6
#define BV_TIMROT_TIMCTRL3_TEST_SIGNAL__ROTARYB     0x7
#define BV_TIMROT_TIMCTRL3_TEST_SIGNAL__32KHZ_XTAL  0x8
#define BV_TIMROT_TIMCTRL3_TEST_SIGNAL__8KHZ_XTAL   0x9
#define BV_TIMROT_TIMCTRL3_TEST_SIGNAL__4KHZ_XTAL   0xA
#define BV_TIMROT_TIMCTRL3_TEST_SIGNAL__1KHZ_XTAL   0xB
#define BV_TIMROT_TIMCTRL3_TEST_SIGNAL__TICK_ALWAYS 0xC
#define BM_TIMROT_TIMCTRL3_IRQ	0x00008000
#define BM_TIMROT_TIMCTRL3_IRQ_EN	0x00004000
#define BP_TIMROT_TIMCTRL3_RSRVD1	11
#define BM_TIMROT_TIMCTRL3_RSRVD1	0x00003800
#define BF_TIMROT_TIMCTRL3_RSRVD1(v)  \
		(((v) << 11) & BM_TIMROT_TIMCTRL3_RSRVD1)
#define BM_TIMROT_TIMCTRL3_DUTY_VALID	0x00000400
#define BM_TIMROT_TIMCTRL3_DUTY_CYCLE	0x00000200
#define BM_TIMROT_TIMCTRL3_POLARITY	0x00000100
#define BM_TIMROT_TIMCTRL3_UPDATE	0x00000080
#define BM_TIMROT_TIMCTRL3_RELOAD	0x00000040
#define BP_TIMROT_TIMCTRL3_PRESCALE	4
#define BM_TIMROT_TIMCTRL3_PRESCALE	0x00000030
#define BF_TIMROT_TIMCTRL3_PRESCALE(v)  \
		(((v) << 4) & BM_TIMROT_TIMCTRL3_PRESCALE)
#define BV_TIMROT_TIMCTRL3_PRESCALE__DIV_BY_1 0x0
#define BV_TIMROT_TIMCTRL3_PRESCALE__DIV_BY_2 0x1
#define BV_TIMROT_TIMCTRL3_PRESCALE__DIV_BY_4 0x2
#define BV_TIMROT_TIMCTRL3_PRESCALE__DIV_BY_8 0x3
#define BP_TIMROT_TIMCTRL3_SELECT	0
#define BM_TIMROT_TIMCTRL3_SELECT	0x0000000F
#define BF_TIMROT_TIMCTRL3_SELECT(v)  \
		(((v) << 0) & BM_TIMROT_TIMCTRL3_SELECT)
#define BV_TIMROT_TIMCTRL3_SELECT__NEVER_TICK  0x0
#define BV_TIMROT_TIMCTRL3_SELECT__PWM0        0x1
#define BV_TIMROT_TIMCTRL3_SELECT__PWM1        0x2
#define BV_TIMROT_TIMCTRL3_SELECT__PWM2        0x3
#define BV_TIMROT_TIMCTRL3_SELECT__PWM3        0x4
#define BV_TIMROT_TIMCTRL3_SELECT__PWM4        0x5
#define BV_TIMROT_TIMCTRL3_SELECT__ROTARYA     0x6
#define BV_TIMROT_TIMCTRL3_SELECT__ROTARYB     0x7
#define BV_TIMROT_TIMCTRL3_SELECT__32KHZ_XTAL  0x8
#define BV_TIMROT_TIMCTRL3_SELECT__8KHZ_XTAL   0x9
#define BV_TIMROT_TIMCTRL3_SELECT__4KHZ_XTAL   0xA
#define BV_TIMROT_TIMCTRL3_SELECT__1KHZ_XTAL   0xB
#define BV_TIMROT_TIMCTRL3_SELECT__TICK_ALWAYS 0xC

#define HW_TIMROT_TIMCOUNT3	(0x00000090)
#define HW_TIMROT_TIMCOUNT3_ADDR \
		(REGS_TIMROT_BASE + HW_TIMROT_TIMCOUNT3)

#define BP_TIMROT_TIMCOUNT3_LOW_RUNNING_COUNT	16
#define BM_TIMROT_TIMCOUNT3_LOW_RUNNING_COUNT	0xFFFF0000
#define BF_TIMROT_TIMCOUNT3_LOW_RUNNING_COUNT(v) \
		(((v) << 16) & BM_TIMROT_TIMCOUNT3_LOW_RUNNING_COUNT)
#define BP_TIMROT_TIMCOUNT3_HIGH_FIXED_COUNT	0
#define BM_TIMROT_TIMCOUNT3_HIGH_FIXED_COUNT	0x0000FFFF
#define BF_TIMROT_TIMCOUNT3_HIGH_FIXED_COUNT(v)  \
		(((v) << 0) & BM_TIMROT_TIMCOUNT3_HIGH_FIXED_COUNT)

#define HW_TIMROT_VERSION	(0x000000a0)
#define HW_TIMROT_VERSION_ADDR \
		(REGS_TIMROT_BASE + HW_TIMROT_VERSION)

#define BP_TIMROT_VERSION_MAJOR	24
#define BM_TIMROT_VERSION_MAJOR	0xFF000000
#define BF_TIMROT_VERSION_MAJOR(v) \
		(((v) << 24) & BM_TIMROT_VERSION_MAJOR)
#define BP_TIMROT_VERSION_MINOR	16
#define BM_TIMROT_VERSION_MINOR	0x00FF0000
#define BF_TIMROT_VERSION_MINOR(v)  \
		(((v) << 16) & BM_TIMROT_VERSION_MINOR)
#define BP_TIMROT_VERSION_STEP	0
#define BM_TIMROT_VERSION_STEP	0x0000FFFF
#define BF_TIMROT_VERSION_STEP(v)  \
		(((v) << 0) & BM_TIMROT_VERSION_STEP)
#endif /* __ARCH_ARM___TIMROT_H */
