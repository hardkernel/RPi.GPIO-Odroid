/*
*  Odroid specific code borrowed from Hardkernel's wiringPi port
*/

// License and info from Hardkernel's original file:
/*
 * wiringPi:
 *	Arduino compatable (ish) Wiring library for the Raspberry Pi
 *	Copyright (c) 2012 Gordon Henderson
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */

#ifndef ODROID_H_INCLUDED
#define ODROID_H_INCLUDED

/* start wiringPi.h code */

#define	PI_MODEL_UNKNOWN  0
#define	PI_MODEL_A        1
#define	PI_MODEL_B        2
#define	PI_MODEL_BP       3
#define	PI_MODEL_CM       4
#define	PI_MODEL_AP       5
#define	PI_MODEL_ODROIDC  6
#define PI_MODEL_ODROIDXU_34    7
#define	PI_MODEL_ODROIDC2	8
#define	PI_MODEL_ODROIDN2 9
#define	PI_MODEL_ODROIDC4 10
#define	PI_MODEL_ODROIDM1 11
#define	PI_MODEL_ODROIDM1S 12

// Failure modes

#define	WPI_FATAL	(1==1)
#define	WPI_ALMOST	(1==2)

/* end wiringPi.h code */


/* start wiringPi.c code */

#ifndef	TRUE
#define	TRUE	(1==1)
#define	FALSE	(1==2)
#endif

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x)	(sizeof(x) / sizeof((x)[0]))
#endif

//
// For ODROID-C Board
//
#define ODROIDC_GPIO_MASK (0xFFFFFF80)

#define ODROIDC_PERI_BASE 0xC1100000
#define GPIO_REG_OFFSET   0x8000
#define ODROID_GPIO_BASE  (ODROIDC_PERI_BASE + GPIO_REG_OFFSET)

#define GPIO_PIN_BASE           80
#define GPIOY_PIN_START         80
#define GPIOY_PIN_END           96
#define GPIOX_PIN_START         97
#define GPIOX_PIN_END           118

#define GPIOX_FSEL_REG_OFFSET   0x0C
#define GPIOX_OUTP_REG_OFFSET   0x0D
#define GPIOX_INP_REG_OFFSET    0x0E
#define GPIOX_PUPD_REG_OFFSET   0x3E
#define GPIOX_PUEN_REG_OFFSET   0x4C

#define GPIOY_FSEL_REG_OFFSET   0x0F
#define GPIOY_OUTP_REG_OFFSET   0x10
#define GPIOY_INP_REG_OFFSET    0x11
#define GPIOY_PUPD_REG_OFFSET   0x3D
#define GPIOY_PUEN_REG_OFFSET   0x4B

#define piAinNode0   "/sys/class/saradc/saradc_ch0"
#define piAinNode1   "/sys/class/saradc/saradc_ch1"

static int adcFds [2] = {
    -1, -1,
} ;

//
// For ODROID-C2 Board
//
#define ODROIDC2_GPIO_MASK		(0xFFFFFF00)
#define ODROIDC2_GPIO_BASE		0xC8834000

#define C2_GPIO_PIN_BASE           136
#define C2_GPIOY_PIN_START         (C2_GPIO_PIN_BASE + 75)
#define C2_GPIOY_PIN_END           (C2_GPIO_PIN_BASE + 91)
#define C2_GPIOX_PIN_START         (C2_GPIO_PIN_BASE + 92)
#define C2_GPIOX_PIN_END           (C2_GPIO_PIN_BASE + 114)

#define C2_GPIOX_FSEL_REG_OFFSET   0x118
#define C2_GPIOX_OUTP_REG_OFFSET   0x119
#define C2_GPIOX_INP_REG_OFFSET    0x11A
#define C2_GPIOX_PUPD_REG_OFFSET   0x13E
#define C2_GPIOX_PUEN_REG_OFFSET   0x14C

#define C2_GPIOY_FSEL_REG_OFFSET   0x10F
#define C2_GPIOY_OUTP_REG_OFFSET   0x110
#define C2_GPIOY_INP_REG_OFFSET    0x111
#define C2_GPIOY_PUPD_REG_OFFSET   0x13B
#define C2_GPIOY_PUEN_REG_OFFSET   0x149

#define C2_piAinNode0   "/sys/class/saradc/ch0"
#define C2_piAinNode1   "/sys/class/saradc/ch1"

//
// For ODROID-XU3/4 Board
//
#define ODROIDXU_GPIO_MASK  (0xFFFFFF00)

#define ODROIDXU_GPX_BASE   0x13400000  // GPX0,1,2,3
#define ODROIDXU_GPA_BASE   0x14010000  // GPA0,1,2, GPB0,1,2,3,4

#define GPIO_X1_START       16
#define GPIO_X1_CON_OFFSET  0x0C20
#define GPIO_X1_DAT_OFFSET  0x0C24
#define GPIO_X1_PUD_OFFSET  0x0C28
#define GPIO_X1_END         23

#define GPIO_X2_START       24
#define GPIO_X2_CON_OFFSET  0x0C40
#define GPIO_X2_DAT_OFFSET  0x0C44
#define GPIO_X2_PUD_OFFSET  0x0C48
#define GPIO_X2_END         31

#define GPIO_X3_START       32
#define GPIO_X3_CON_OFFSET  0x0C60
#define GPIO_X3_DAT_OFFSET  0x0C64
#define GPIO_X3_PUD_OFFSET  0x0C68
#define GPIO_X3_END         39

#define GPIO_A0_START       171
#define GPIO_A0_CON_OFFSET  0x0000
#define GPIO_A0_DAT_OFFSET  0x0004
#define GPIO_A0_PUD_OFFSET  0x0008
#define GPIO_A0_END         178

#define GPIO_A2_START       185
#define GPIO_A2_CON_OFFSET  0x0040
#define GPIO_A2_DAT_OFFSET  0x0044
#define GPIO_A2_PUD_OFFSET  0x0048
#define GPIO_A2_END         192

#define GPIO_B3_START       207
#define GPIO_B3_CON_OFFSET  0x00C0
#define GPIO_B3_DAT_OFFSET  0x00C4
#define GPIO_B3_PUD_OFFSET  0x00C8
#define GPIO_B3_END         214

//
// For ODROID-N2 Board
//
#define N2_GPIO_MASK			(0xFFFFFF00)
#define N2_GPIO_BASE			0xff634000

#define N2_GPIO_PIN_BASE		410

#define N2_GPIOA_PIN_START		(N2_GPIO_PIN_BASE + 50) // GPIOA_0
#define N2_GPIOA_PIN_END		(N2_GPIO_PIN_BASE + 65) // GPIOA_15
#define N2_GPIOX_PIN_START		(N2_GPIO_PIN_BASE + 66) // GPIOX_0
#define N2_GPIOX_PIN_MID		(N2_GPIO_PIN_BASE + 81) // GPIOX_15
#define N2_GPIOX_PIN_END		(N2_GPIO_PIN_BASE + 85) // GPIOX_19

#define N2_GPIOX_FSEL_REG_OFFSET	0x116
#define N2_GPIOX_OUTP_REG_OFFSET	0x117
#define N2_GPIOX_INP_REG_OFFSET		0x118
#define N2_GPIOX_PUPD_REG_OFFSET	0x13C
#define N2_GPIOX_PUEN_REG_OFFSET	0x14A

#define N2_GPIOA_FSEL_REG_OFFSET	0x120
#define N2_GPIOA_OUTP_REG_OFFSET	0x121
#define N2_GPIOA_INP_REG_OFFSET		0x122
#define N2_GPIOA_PUPD_REG_OFFSET	0x13F
#define N2_GPIOA_PUEN_REG_OFFSET	0x14D

//
// For ODROID-C4 Board
//
#define C4_GPIO_MASK			(0xFFFFFF00)
#define C4_GPIO_BASE			0xFF634000

#define C4_GPIO_PIN_BASE		460

#define C4_GPIOH_PIN_START		17				// GPIOH_0
#define C4_GPIOH_PIN_END		25				// GPIOH_8
#define C4_GPIOA_PIN_START		C4_GPIO_PIN_BASE		// GPIOA_0
#define C4_GPIOA_PIN_END		(C4_GPIO_PIN_BASE + 15)		// GPIOA_15
#define C4_GPIOX_PIN_START		(C4_GPIO_PIN_BASE + 16)		// GPIOX_0
#define C4_GPIOX_PIN_MID		(C4_GPIO_PIN_BASE + 31)		// GPIOX_15
#define C4_GPIOX_PIN_END		(C4_GPIO_PIN_BASE + 35)		// GPIOX_19

#define C4_GPIOH_FSEL_REG_OFFSET	0x119
#define C4_GPIOH_OUTP_REG_OFFSET	0x11A
#define C4_GPIOH_INP_REG_OFFSET		0x11B
#define C4_GPIOH_PUPD_REG_OFFSET	0x13D
#define C4_GPIOH_PUEN_REG_OFFSET	0x14B

#define C4_GPIOA_FSEL_REG_OFFSET	0x120
#define C4_GPIOA_OUTP_REG_OFFSET	0x121
#define C4_GPIOA_INP_REG_OFFSET		0x122
#define C4_GPIOA_PUPD_REG_OFFSET	0x13F
#define C4_GPIOA_PUEN_REG_OFFSET	0x14D

#define C4_GPIOX_FSEL_REG_OFFSET	0x116
#define C4_GPIOX_OUTP_REG_OFFSET	0x117
#define C4_GPIOX_INP_REG_OFFSET		0x118
#define C4_GPIOX_PUPD_REG_OFFSET	0x13C
#define C4_GPIOX_PUEN_REG_OFFSET	0x14A

//
// For ODROID-M1/M1S Board
//
#define M1_GPIO_PIN_BASE    0
//setClkState mode
#define M1_CLK_ENABLE   0
#define M1_CLK_DISABLE  1

#define M1_GRF_BLOCK_SIZE 0xFFFF
#define M1_GPIO_SIZE   32

#define M1_FUNC_GPIO 0
#define M1_FUNC_PWM 1

// GPIO[0]
#define M1_GPIO_0_BASE  0xFDD60000
// to control clock (PMU_CRU)
#define M1_PMU_CRU_BASE 0xFDD00000
#define M1_PMU_CRU_GPIO_CLK_OFFSET  0x0184
#define M1_PMU_CRU_GPIO_PCLK_BIT    9
// to control IOMUX
#define M1_PMU_GRF_BASE 0xFDC20000
#define M1_PMU_GRF_IOMUX_OFFSET 0x0000
#define M1_PMU_GRF_PUPD_OFFSET  0x0020
#define M1_PMU_GRF_DS_OFFSET    0x0070

// GPIO[1:4]
#define M1_GPIO_1_BASE  0xFE740000
#define M1_GPIO_2_BASE  0xFE750000
#define M1_GPIO_3_BASE  0xFE760000
#define M1_GPIO_4_BASE  0xFE770000
// to control clock (SYS_CRU)
#define M1_CRU_BASE 0xFDD20000
#define M1_CRU_GPIO_CLK_OFFSET  0x037C
#define M1_CRU_GPIO_PCLK_BIT    2
// to control IOMUX
#define M1_SYS_GRF_BASE 0xFDC60000
#define M1_SYS_GRF_IOMUX_OFFSET 0x0000
#define M1_SYS_GRF_PUPD_OFFSET  0x0080
#define M1_SYS_GRF_DS_OFFSET    0x0200

// Common offset for GPIO registers from each GPIO bank's base address
#define M1_GPIO_DIR_OFFSET  0x0008
#define M1_GPIO_SET_OFFSET  0x0000
#define M1_GPIO_GET_OFFSET  0x0070

#ifdef DEFINE_ODROID_VARS

//From c_gpio.c and c_gpio.h
#define PAGE_SIZE  (4*1024)
#define BLOCK_SIZE (4*1024)

#define INPUT  1 // is really 0 for control register!
#define OUTPUT 0 // is really 1 for control register!
#define ALT0   4

#define HIGH 1
#define LOW  0

#define PUD_OFF  0
#define PUD_DOWN 1
#define PUD_UP   2
//End from c_gpio.c and c_gpio.h

//From common.h
#define MAXPINCOUNT 40
extern const int (*pin_to_gpio)[MAXPINCOUNT+1];
//End from common.h

int wiringPiReturnCodes = FALSE ;

static volatile uint32_t *gpio, *gpio1;
static volatile uint32_t *cru[2], *gpioarr[5], *grf[2];
static volatile void *mmapped_cru[2], *mmapped_grf[2], *mmapped_gpio[5];

// pinToGpio:
//	Take a Wiring pin (0 through X) and re-map it to the BCM_GPIO pin
//	Cope for 3 different board revisions here.
//static int *pinToGpio ;
//static int pin_array_count;

// physToGpio:
//	Take a physical pin (1 through 26) and re-map it to the BCM_GPIO pin
//	Cope for 2 different board revisions here.
//	Also add in the P5 connector, so the P5 pins are 3,4,5,6, so 53,54,55,56

//static int *physToGpio ;

static char *piAinNode0_xu;
static char *piAinNode1_xu;

static int sysFdData [64] = {
    -1, -1, -1, -1, -1, -1, -1, -1, // 0...7
    -1, -1, -1, -1, -1, -1, -1, -1, // 8...15
    -1, -1, -1, -1, -1, -1, -1, -1, // 16...23
    -1, -1, -1, -1, -1, -1, -1, -1, // 24...31
    -1, -1, -1, -1, -1, -1, -1, -1, // 32...39
    -1, -1, -1, -1, -1, -1, -1, -1, // 40...47
    -1, -1, -1, -1, -1, -1, -1, -1, // 48...55
    -1, -1, -1, -1, -1, -1, -1, -1, // 56...63
};

static int sysFdIrqType [64] = {
    -1, -1, -1, -1, -1, -1, -1, -1, // 0...7
    -1, -1, -1, -1, -1, -1, -1, -1, // 8...15
    -1, -1, -1, -1, -1, -1, -1, -1, // 16...23
    -1, -1, -1, -1, -1, -1, -1, -1, // 24...31
    -1, -1, -1, -1, -1, -1, -1, -1, // 32...39
    -1, -1, -1, -1, -1, -1, -1, -1, // 40...47
    -1, -1, -1, -1, -1, -1, -1, -1, // 48...55
    -1, -1, -1, -1, -1, -1, -1, -1, // 56...63
};


//
// pinToGpio:
//	Take a Wiring pin (0 through X) and re-map it to the ODROID_GPIO pin
//
static int pinToGpioOdroidC [64] = {
    88,  87, 116, 115, 104, 102, 103,  83, // 0..7
    -1,  -1, 117, 118, 107, 106, 105,  -1, // 8..16
    -1,  -1,  -1,  -1,  -1, 101, 100, 108, // 16..23
    97,  -1,  99,  98,  -1,  -1,  -1,  -1, // 24..31
// Padding:
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// ... 47
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// ... 63
};

//
// physToGpio:
//	Take a physical pin (1 through 40) and re-map it to the ODROID_GPIO pin
//
//!!!Odroid - don't know why wiringPi code uses array size of 64 instead of 41.
//Also need to access from other files, so changed static int to const int
/*odroid static int */const int physToGpioOdroidC [64] =
{
  -1,       // 0
  -1,  -1,	// 1, 2
  -1,  -1,
  -1,  -1,
  83,  -1,
  -1,  -1,
  88,  87,
 116,  -1,
 115, 104,
  -1, 102,
 107,  -1,
 106, 103,
 105, 117,
  -1, 118,	// 25, 26

  -1,  -1,
 101,  -1,
 100,  99,
 108,  -1,
  97,  98,
  -1,  -1,
  -1,  -1, // 39, 40

// Not used
  -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1,
} ;

//
// pinToGpio:
//	Take a Wiring pin (0 through X) and re-map it to the ODROIDC2_GPIO pin
//
static int pinToGpioOdroidC2_Rev1_1 [64] = {
   247, 238, 239, 237, 236, 233, 231, 249, // 0..7
    -1,  -1, 229, 225, 235, 232, 230,  -1, // 8..15
    -1,  -1,  -1,  -1,  -1, 228, 219, 234, // 16..23
   214,  -1, 224, 218,  -1,  -1,  -1,  -1, // 24..31
// Padding:
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// ... 47
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// ... 63
};


static int pinToGpioOdroidC2_Rev1_0 [64] = {
   219, 218, 247,  -1, 235, 233, 234, 214, // 0..7
    -1,  -1, 248, 249, 238, 237, 236,  -1, // 8..15
    -1,  -1,  -1,  -1,  -1, 232, 231, 239, // 16..23
   228,  -1, 230, 229,  -1,  -1,  -1,  -1, // 24..31
// Padding:
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// ... 47
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// ... 63
};

//
// physToGpio:
//	Take a physical pin (1 through 40) and re-map it to the ODROIDC2_GPIO pin
//
/*odroid static int*/const int physToGpioOdroidC2_Rev1_1 [64] =
{
  -1,       // 0
  -1,  -1,	// 1, 2
  -1,  -1,
  -1,  -1,
 249,  -1,
  -1,  -1,
 247, 238,
 239,  -1,
 237, 236,
  -1, 233,
 235,  -1,
 232, 231,
 230, 229,
  -1, 225,	// 25, 26

  -1,  -1,
 228,  -1,
 219, 224,
 234,  -1,
 214, 218,
  -1,  -1,
  -1,  -1, // 39, 40

// Not used
  -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1,
} ;


/*odroid static int*/const int physToGpioOdroidC2_Rev1_0 [64] =
{
  -1,       // 0
  -1,  -1,	// 1, 2
  -1,  -1,
  -1,  -1,
 214,  -1,
  -1,  -1,
 219, 218,
 247,  -1,
  -1, 235,
  -1, 233,
 238,  -1,
 237, 234,
 236, 248,
  -1, 249,	// 25, 26

  -1,  -1,
 232,  -1,
 231, 230,
 239,  -1,
 228, 229,
  -1,  -1,
  -1,  -1, // 39, 40

// Not used
  -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1,
} ;

//
// pinToGpio:
//	Take a Wiring pin (0 through X) and re-map it to the ODROIDXU_GPIO pin
//
static int pinToGpioOdroidXU [64] = {
   174, 173,    //  0 |  1 : GPA0.3(UART_0.CTSN), GPA0.2(UART_0.RTSN)
    21,  22,    //  2 |  3 : GPX1.5, GPX1.6
    19,  23,    //  4 |  5 : GPX1.3, GPX1.7
    24,  18,    //  6 |  7 : GPX2.0, GPX1.2

   209, 210,    //  8 |  9 : GPB3.2(I2C_1.SDA), GPB3.3(I2C_1.SCL)
   190,  25,    // 10 | 11 : GPA2.5(SPI_1.CSN), GPX2.1
   192, 191,    // 12 | 13 : GPA2.7(SPI_1.MOSI), GPA2.6(SPI_1.MISO)
   189, 172,    // 14 | 15 : GPA2.4(SPI_1.SCLK), GPA0.1(UART_0.TXD)
   171,  -1,    // 16 | 17 : GPA0.0(UART_0.RXD),
    -1,  -1,    // 18 | 19
    -1,  28,    // 20 | 21 :  , GPX2.4
    30,  31,    // 22 | 23 : GPX2.6, GPX2.7
    -1,  -1,    // 24 | 25   PWR_ON(INPUT), ADC_0.AIN0
    29,  33,    // 26 | 27 : GPX2.5, GPX3.1
    -1,  -1,    // 28 | 29 : REF1.8V OUT, ADC_0.AIN3
   187, 188,    // 30 | 31 : GPA2.2(I2C_5.SDA), GPA2.3(I2C_5.SCL)

    // Padding:
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// 32...47
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// 48...63
};

//
// physToGpio:
//	Take a physical pin (1 through 40) and re-map it to the ODROIDXU_GPIO pin
//
/*odroid static int*/const int physToGpioOdroidXU [64] =
{
    -1,         //  0
    -1,  -1,	//  1 |  2 : 3.3V, 5.0V
   209,  -1,    //  3 |  4 : GPB3.2(I2C_1.SDA), 5.0V
   210,  -1,    //  5 |  6 : GPB3.3(I2C_1.SCL), GND
    18, 172,    //  7 |  8 : GPX1.2, GPA0.1(UART_0.TXD)
    -1, 171,    //  9 | 10 : GND, GPA0.0(UART_0.RXD)
   174, 173,    // 11 | 12 : GPA0.3(UART_0.CTSN), GPA0.2(UART_0.RTSN)
    21,  -1,    // 13 | 14 : GPX1.5, GND
    22,  19,    // 15 | 16 : GPX1.6, GPX1.3
    -1,  23,    // 17 | 18 : 3.3V, GPX1.7
   192,  -1,    // 19 | 20 : GPA2.7(SPI_1.MOSI), GND
   191,  24,    // 21 | 22 : GPA2.6(SPI_1.MISO), GPX2.0
   189, 190,    // 23 | 24 : GPA2.4(SPI_1.SCLK), GPA2.5(SPI_1.CSN)
    -1,  25,    // 25 | 26 : GND, GPX2.1
   187, 188,    // 27 | 28 : GPA2.2(I2C_5.SDA), GPA2.4(I2C_5.SCL)
    28,  -1,    // 29 | 30 : GPX2.4, GND
    30,  29,    // 31 | 32 : GPX2.6, GPX2.5
    31,  -1,    // 33 | 34 : GPX2.7, GND
    -1,  33,    // 35 | 36 : PWR_ON(INPUT), GPX3.1
    -1,  -1,    // 37 | 38 : ADC_0.AIN0, 1.8V REF OUT
    -1,  -1,    // 39 | 40 : GND, AADC_0.AIN3

    // Not used
    -1, -1, -1, -1, -1, -1, -1, -1, // 41...48
    -1, -1, -1, -1, -1, -1, -1, -1, // 49...56
    -1, -1, -1, -1, -1, -1, -1      // 57...63
} ;

static const int pinToGpioOdroidN2[64] = {
	// wiringPi number to native gpio number
	479, 492,	//  0 |  1 : GPIOX.3, GPIOX.16
	480, 483,	//  2 |  3 : GPIOX.4, GPIOX.7
	476, 477,	//  4 |  5 : GPIOX.0, GPIOX.1
	478, 473,	//  6 |  7 : GPIOX.2, GPIOA.13
	493, 494,	//  8 |  9 : GPIOX.17(I2C-2_SDA), GPIOX.18(I2C-2_SCL)
	486, 464,	// 10 | 11 : GPIOX.10, GPIOA.4
	484, 485,	// 12 | 13 : GPIOX.8, GPIOX.9
	487, 488,	// 14 | 15 : GPIOX.11, GPIOX.12
	489,  -1,	// 16 | 17 : GPIOX.13,
	-1,  -1,	// 18 | 19 :
	-1,  490,	// 20 | 21 : , GPIOX.14
	491, 481,	// 22 | 23 : GPIOX.15, GPIOX.5
	482, -1,	// 24 | 25 : GPIOX.6, ADC.AIN3
	472, 495,	// 26 | 27 : GPIOA.12, GPIOX.19
	-1,  -1,	// 28 | 29 : REF1.8V OUT, ADC.AIN2
	474, 475,	// 30 | 31 : GPIOA.14(I2C-3_SDA), GPIOA.15(I2C-3_SCL)
	// Padding:
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// 32...47
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// 48...63
};

static const int physToGpioOdroidN2[64] = {
	// physical header pin number to native gpio number
	 -1,		//  0
	 -1,  -1,	//  1 |  2 : 3.3V, 5.0V
	493,  -1,	//  3 |  4 : GPIOX.17(I2C-2_SDA), 5.0V
	494,  -1,	//  5 |  6 : GPIOX.18(I2C-2_SCL), GND
	473, 488,	//  7 |  8 : GPIOA.13, GPIOX.12(UART_TX_B)
	 -1, 489,	//  9 | 10 : GND, GPIOX.13(UART_RX_B)
	479, 492,	// 11 | 12 : GPIOX.3, GPIOX.16
	480,  -1,	// 13 | 14 : GPIOX.4, GND
	483, 476,	// 15 | 16 : GPIOX.7, GPIOX.0
	 -1, 477,	// 17 | 18 : 3.3V, GPIOX.1
	484,  -1,	// 19 | 20 : GPIOX.8(SPI_MOSI), GND
	485, 478,	// 21 | 22 : GPIOX.9(SPI_MISO), GPIOX.2
	487, 486,	// 23 | 24 : GPIOX.11(SPI_SCLK), GPIOX.10(SPI_CE0)
	 -1, 464,	// 25 | 26 : GND, GPIOA.4(SPI_CE1)
	474, 475,	// 27 | 28 : GPIOA.14(I2C-3_SDA), GPIOA.15(I2C-3_SCL)
	490,  -1,	// 29 | 30 : GPIOX.14, GND
	491, 472,	// 31 | 32 : GPIOX.15, GPIOA.12
	481,  -1,	// 33 | 34 : GPIOX.5, GND
	482, 495,	// 35 | 36 : GPIOX.6, GPIOX.19
	 -1,  -1,	// 37 | 38 : ADC.AIN3, 1.8V REF OUT
	 -1,  -1,	// 39 | 40 : GND, ADC.AIN2
	// Not used
	-1, -1, -1, -1, -1, -1, -1, -1,	// 41...48
	-1, -1, -1, -1, -1, -1, -1, -1,	// 49...56
	-1, -1, -1, -1, -1, -1, -1	// 57...63
};

static const int pinToGpioOdroidC4[64] = {
	// wiringPi number to native gpio number
	479, 492,	//  0 |  1 : GPIOX.3, GPIOX.16
	480, 483,	//  2 |  3 : GPIOX.4, GPIOX.7
	476, 477,	//  4 |  5 : GPIOX.0, GPIOX.1
	478, 481,	//  6 |  7 : GPIOX.2, GPIOX.5
	493, 494,	//  8 |  9 : GPIOX.17(I2C-2_SDA), GPIOX.18(I2C-2_SCL)
	486,  23,	// 10 | 11 : GPIOX.10(SPI_SS), GPIOH.6
	484, 485,	// 12 | 13 : GPIOX.8(SPI_MOSI), GPIOX.9(SPI_MISO)
	487, 488,	// 14 | 15 : GPIOX.11(SPI_CLK), GPIOX.12(UART_TX_B)
	489,  -1,	// 16 | 17 : GPIOX.13(UART_RX_B),
	 -1, - 1,	// 18 | 19 :
	 -1, 490,	// 20 | 21 : , GPIOX.14
	491, 482,	// 22 | 23 : GPIOX.15, GPIOX.6
	495,  -1,	// 24 | 25 : GPIOX.19, ADC.AIN3
	 24,  22,	// 26 | 27 : GPIOH.7, GPIOH.5
	 -1, - 1,	// 28 | 29 : REF1.8V OUT, ADC.AIC4
	474, 475,	// 30 | 31 : GPIOA.14(I2C-3_SDA), GPIOA.15(I2C-3_SCL)
	// Padding:
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// 32...47
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// 48...63
};

static const int physToGpioOdroidC4[64] = {
	// physical header pin number to native gpio number
	 -1,		//  0
	 -1,  -1,	//  1 |  2 : 3.3V, 5.0V
	493,  -1,	//  3 |  4 : GPIOX.17(I2C-2_SDA), 5.0V
	494,  -1,	//  5 |  6 : GPIOX.18(I2C-2_SCL), GND
	481, 488,	//  7 |  8 : GPIOX.5, GPIOX.12(UART_TX_B)
	 -1, 489,	//  9 | 10 : GND, GPIOX.13(UART_RX_B)
	479, 492,	// 11 | 12 : GPIOX.3, GPIOX.16
	480,  -1,	// 13 | 14 : GPIOX.4, GND
	483, 476,	// 15 | 16 : GPIOX.7, GPIOX.0
	 -1, 477,	// 17 | 18 : 3.3V, GPIOX.1
	484,  -1,	// 19 | 20 : GPIOX.8(SPI_MOSI), GND
	485, 478,	// 21 | 22 : GPIOX.9(SPI_MISO), GPIOX.2
	487, 486,	// 23 | 24 : GPIOX.11(SPI_CLK), GPIOX.10(SPI_SS)
	 -1,  23,	// 25 | 26 : GND, GPIOH.6
	474, 475,	// 27 | 28 : GPIOA.14(I2C-3_SDA), GPIOA.15(I2C-3_SCL)
	490,  -1,	// 29 | 30 : GPIOX.14, GND
	491,  24,	// 31 | 32 : GPIOX.15, GPIOH.7
	482,  -1,	// 33 | 34 : GPIOX.6, GND
	495,  22,	// 35 | 36 : GPIOX.19, GPIOH.5
	 -1,  -1,	// 37 | 38 : ADC.AIN3, 1.8V REF OUT
	 -1,  -1,	// 39 | 40 : GND, ADC.AIC4
	// Not used
	-1, -1, -1, -1, -1, -1, -1, -1,	// 41...48
	-1, -1, -1, -1, -1, -1, -1, -1,	// 49...56
	-1, -1, -1, -1, -1, -1, -1	// 57...63
};

static const int pinToGpioOdroidM1[64] = {
	// wiringPi number to native gpio number
	16, 120,    //  0 |  1 : GPIO0_C0, GPIO3_D0
	17, 106,    //  2 |  3 : GPIO0_C1, GPIO3_B2
	118,119,    //  4 |  5 : GPIO3_C6, GPIO3_C7
	121,14,     //  6 |  7 : GPIO3_D1, GPIO0_B6
	110,109,    //  8 |  9 : GPIO3_B6, GPIO3_B5
	90, 122,    // 10 | 11 : GPIO2_D2, GPIO3_D2
	89,  88,    // 12 | 13 : GPIO2_D1, GPIO2_D0
	91, 126,    // 14 | 15 : GPIO2_D3, GPIO3_D6
	127, -1,    // 16 | 17 : GPIO3_D7
	-1,  -1,    // 18 | 19 :
	-1, 145,    // 20 | 21 : , GPIO4_C1
	142, 13,    // 22 | 23 : GPIO4_B6, GPIO0_B5
	125, -1,    // 24 | 25 : GPIO3_D5,
	123,124,    // 26 | 27 : GPIO3_D3, GPIO3_D4
	-1,  -1,    // 28 | 29 :
	 12, 11,    // 30 | 31 : GPIO0_B4, GPIO0_B3

	// Padding:
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // 32...47
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // 48...63
};

static const int physToGpioOdroidM1[64] = {
    // physical header pin number to native gpio number
    -1,     //  0
    -1,  -1,    //  1 |  2 : 3.3V, 5.0V
    110, -1,    //  3 |  4 : GPIO3_B6, 5.0V
    109, -1,    //  5 |  6 : GPIO3_B5, GND
    14, 126,    //  7 |  8 : GPIO0_B6, GPIO3_D6
    -1, 127,    //  9 | 10 : GND, GPIO3_D7
    16 ,120,    // 11 | 12 : GPIO0_C0, GPIO3_D0
    17 , -1,    // 13 | 14 : GPIO0_C1, GND
    106,118,    // 15 | 16 : GPIO3_B2, GPIO3_C6
    -1, 119,    // 17 | 18 : 3.3V, GPIO3_C7
    89,  -1,    // 19 | 20 : GPIO2_D1, GND
    88, 121,    // 21 | 22 : GPIO2_D0, GPIO3_D1
    91,  90,    // 23 | 24 : GPIO2_D3, GPIO2_D2
    -1, 122,    // 25 | 26 : GND, GPIO3_D2
    12,  11,    // 27 | 28 : GPIO0_B4, GPIO0_B3
    145, -1,    // 29 | 30 : GPIO4_C1, GND
    142,123,    // 31 | 32 : GPIO4_B6, GPIO3_D3
    13,  -1,    // 33 | 34 : GPIO0_B5, GND
    125,124,    // 35 | 36 : GPIO3_D5, GPIO3_D4
    -1,  -1,    // 37 | 38 : ADC.AIN1, 1.8V REF
    -1,  -1,    // 39 | 40 : GND, ADC.AIN0
    // Not used
    -1, -1, -1, -1, -1, -1, -1, -1, // 41...48
    -1, -1, -1, -1, -1, -1, -1, -1, // 49...56
    -1, -1, -1, -1, -1, -1, -1  // 57...63
};

static const int pinToGpioOdroidM1S[64] = {
	// wiringPi number to native gpio number
	16,  71,	//  0 |  1 : GPIO0_C0, GPIO2_A7
	17,  18,	//  2 |  3 : GPIO0_C1, GPIO0_C2
	77,  78,	//  4 |  5 : GPIO2_B5, GPIO2_B6
	72,  14,	//  6 |  7 : GPIO2_B0, GPIO0_B6
	110,109,	//  8 |  9 : GPIO3_B6, GPIO3_B5
	97,  73,	// 10 | 11 : GPIO3_A1, GPIO2_B1
	113,114,	// 12 | 13 : GPIO3_C1, GPIO3_C2
	115, 68,	// 14 | 15 : GPIO3_C3, GPIO2_A4
	67,  -1,	// 16 | 17 : GPIO2_A3
	-1,  -1,	// 18 | 19 :
	-1,  80,	// 20 | 21 : , GPIO2_C0
	79,  13,	// 22 | 23 : GPIO2_B7, GPIO0_B5
	69,  -1,	// 24 | 25 : GPIO2_A5,
	74,  70,	// 26 | 27 : GPIO2_B2, GPIO2_A6
	-1,  -1,	// 28 | 29 :
	 12, 11,	// 30 | 31 : GPIO0_B4, GPIO0_B3

	// EXT_PINS:
	116, 117, 107, 108, // 32...35

	// Padding:
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// 36...49
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// 50...63

};

static const int physToGpioOdroidM1S[64] = {
	// physical header pin number to native gpio number
	-1,		//  0
	-1,  -1,	//  1 |  2 : 3.3V, 5.0V
	110, -1,	//  3 |  4 : GPIO3_B6, 5.0V
	109, -1,	//  5 |  6 : GPIO3_B5, GND
	14,  68,	//  7 |  8 : GPIO0_B6, GPIO2_A4
	-1,  67,	//  9 | 10 : GND, GPIO2_A3
	16,  71,	// 11 | 12 : GPIO0_C0, GPIO2_A7
	17,  -1,	// 13 | 14 : GPIO0_C1, GND
	18 , 77,	// 15 | 16 : GPIO0_C2, GPIO2_B5
	-1,  78,	// 17 | 18 : 3.3V, GPIO2_B6
	113, -1,	// 19 | 20 : GPIO3_C1, GND
	114, 72,	// 21 | 22 : GPIO3_C2, GPIO2_B0
	115, 97,	// 23 | 24 : GPIO3_C3, GPIO3_A1
	-1,  73,	// 25 | 26 : GND, GPIO2_B1
	12,  11,	// 27 | 28 : GPIO0_B4, GPIO0_B3
	80,  -1,	// 29 | 30 : GPIO2_C0, GND
	79,  74,	// 31 | 32 : GPIO2_B7, GPIO2_B2
	13,  -1,	// 33 | 34 : GPIO0_B5, GND
	69,  70,	// 35 | 36 : GPIO2_A5, GPIO2_A6
	-1,  -1,	// 37 | 38 : ADC.AIN1, 1.8V REF
	-1,  -1,	// 39 | 40 : GND, ADC.AIN0

	// Not used
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// 41...50

	// EXT_PINS
	116, 117, 107, 108,	// 51...54

	// Not used
	-1, -1, -1, -1, -1, -1, -1, -1, -1	// 55...63
};
/* end wiringPi.c code */


/* Non-static add extern definition below */
int odroid_found;
int  piModel;

const int bcmToOGpioOdroidC[64] = {	// BCM ModE
     -1,  -1,  -1,  -1,  83, 101, 100, 118, // 0..7
    117, 106, 107, 105,  99, 108,  -1,  -1, // 8..15
     98,  88,  87,  97,  -1,  -1, 115, 104, // 16..23
    102, 103,  -1, 116,  -1,  -1,  -1,  -1, // 24..31
// Padding:
     -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1, // 32..39
     -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1, // 40..47
     -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1, // 48..55
     -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1  // 56..63
};

const int bcmToOGpioOdroidC2[64] = {	// BCM ModE
     -1,  -1,  -1,  -1, 249, 228, 219, 225, // 0..7
    229, 232, 235, 230, 224, 234,  -1,  -1, // 8..15
    218, 247, 238, 214,  -1,  -1, 237, 236, // 16..23
    233, 231,  -1, 239,  -1,  -1,  -1,  -1, // 24..31
// Padding:
     -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1, // 32..39
     -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1, // 40..47
     -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1, // 48..55
     -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1  // 56..63
};

const int bcmToOGpioOdroidXU[64] = {	// BCM ModE
     -1,  -1, 209, 210,  18,  28,  30,  25, // 0..7
    190, 191, 192, 189,  29,  31, 172, 171, // 8..15
     33, 174, 173,  -1,  -1,  -1,  22,  19, // 16..23
     23,  24,  -1,  21,  -1,  -1,  -1,  -1, // 24..31
// Padding:
     -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1, // 32..39
     -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1, // 40..47
     -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1, // 48..55
     -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1  // 56..63
};

const int bcmToOGpioOdroidN2[64] = {	// BCM ModE
     -1,  -1, 493, 494, 473, 490, 491, 464, // 0..7
    486, 485, 484, 487, 472, 481, 488, 489, // 8..15
    495, 479, 492, 482,  -1,  -1, 483, 476, // 16..23
    477, 478,  -1, 480,  -1,  -1,  -1,  -1, // 24..31
// Padding:
     -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1, // 32..39
     -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1, // 40..47
     -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1, // 48..55
     -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1  // 56..63
};

const int bcmToOGpioOdroidC4[64] = {	// BCM ModE
    474, 475, 493, 494, 481, 490, 491,  23, // 0..7
    486, 485, 484, 487,  24, 482, 488, 489, // 8..15
     22, 479, 492, 495,  -1,  -1, 483, 476, // 16..23
    477, 478,  -1, 480,  -1,  -1,  -1,  -1, // 24..31
// Padding:
     -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1, // 32..39
     -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1, // 40..47
     -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1, // 48..55
     -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1  // 56..63
};

const int bcmToOGpioOdroidM1[64] = {	// BCM ModE
     12,  11, 110, 109,  14, 145, 142, 122, // 0..7
     90,  88,  89,  91, 123,  13, 126, 127, // 8..15
    124,  16, 120, 125,  -1,  -1, 106, 118, // 16..23
    119, 121,  -1,  17,  -1,  -1,  -1,  -1, // 24..31
// Padding:
     -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1, // 32..39
     -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1, // 40..47
     -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1, // 48..55
     -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1  // 56..63
};

const int bcmToOGpioOdroidM1S[64] = {	// BCM ModE
     12,  11, 110, 109,  14,  80,  79,  73, // 0..7
     97, 114, 113, 115,  74,  13,  68,  67, // 8..15
     70,  16,  71,  69,  -1,  -1,  18,  77, // 16..23
     78,  72,  -1,  17,  -1,  -1,  -1,  -1, // 24..31
    116, 117, 107, 108, // 32..35
// Padding:
     -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1, // 36..45
     -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1, // 46..54
     -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1 // 55..63
};

const int bcmToOGpioRPi[64] = {	// BCM ModE
      0,   1,   2,   3,   4,   5,   6,   7, // 0..7
      8,   9,  10,  11,  12,  13,  14,  15, // 8..15
     16,  17,  16,  19,  20,  21,  22,  23, // 16..23
     24,  25,  26,  27,  28,  29,  30,  31, // 24..31
// Padding:
     32,  33,  34,  35,  36,  37,  38,  39, // 32..39
     40,  41,  42,  43,  44,  45,  46,  47, // 40..47
     48,  49,  50,  51,  52,  53,  54,  55, // 48..55
     56,  57,  58,  59,  60,  61,  62,  63  // 56..63
};

const int (*bcm_to_odroidgpio)[64];


#else /* DEFINE_ODROID_VARS */

extern int odroid_found;
extern int  piModel;
extern const int physToGpioOdroidC[64];
extern const int physToGpioOdroidC2_Rev1_1[64];
extern const int physToGpioOdroidXU[64];
extern const int physToGpioOdroidN2[64];
extern const int physToGpioOdroidC4[64];
extern const int physToGpioOdroidM1[64];
extern const int physToGpioOdroidM1S[64];
extern const int bcmToOGpioOdroidC[64];
extern const int bcmToOGpioOdroidC2[64];
extern const int bcmToOGpioOdroidXU[64];
extern const int bcmToOGpioOdroidN2[64];
extern const int bcmToOGpioOdroidC4[64];
extern const int bcmToOGpioOdroidM1[64];
extern const int bcmToOGpioOdroidM1S[64];
extern const int bcmToOGpioRPi[64];
extern const int (*bcm_to_odroidgpio)[64];

int wiringPiSetupOdroid (void);
void wiringPiCleanupOdroid (void);
void pinModeOdroid (int pin, int mode);
void pullUpDnControlOdroid (int pin, int pud);
int digitalReadOdroid (int pin);
void digitalWriteOdroid (int pin, int value);
int analogReadOdroid (int pin);
void analogWriteOdroid (int pin, int value);
int pinGetModeOdroid (int pin);
void setInfoOdroid(char *hardware, void *vinfo);
void setMappingPtrsOdroid(void);

#endif /* DEFINE_ODROID_VARS */


#endif /* ODROID_H_INCLUDED */
