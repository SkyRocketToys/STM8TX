/*
 * stm8l.h
 *
 * Copyright 2014 Edward V. Emelianoff <eddy@sao.ru>
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
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 */


#pragma once
#ifndef __STM8L_H__
#define __STM8L_H__

#include <stdint.h>
#include <stdbool.h>

typedef unsigned char U8;
typedef unsigned int U16;
typedef unsigned long U32;
#ifndef NULL
#define NULL (void*)0
#endif

/* functions */
#define enableInterrupts()    {__asm__("rim\n");}    // enable interrupts
#define disableInterrupts()   {__asm__("sim\n");}    // disable interrupts
#define iret()                {__asm__("iret\n");}   // Interrupt routine return
#define pop_ccr()             {__asm__("pop cc\n");} // Pop CCR from the stack
#define push_ccr()            {__asm__("push cc\n");}// Push CCR on the stack
#define rim()                 {__asm__("rim\n");}    // enable interrupts
#define sim()                 {__asm__("sim\n");}    // disable interrupts
#define nop()                 {__asm__("nop\n");}    // No Operation
#define trap()                {__asm__("trap\n");}   // Trap (soft IT)
#define wfi()                 {__asm__("wfi\n");}    // Wait For Interrupt
#define halt()                {__asm__("halt\n");}   // Halt

/*
 * Registers map is shown in short datasheet, page 26
 */

/* GPIO */
#define PA_ODR *(volatile U8*)0x5000
#define PA_IDR *(volatile U8*)0x5001
#define PA_DDR *(volatile U8*)0x5002
#define PA_CR1 *(volatile U8*)0x5003
#define PA_CR2 *(volatile U8*)0x5004

#define PB_ODR *(volatile U8*)0x5005
#define PB_IDR *(volatile U8*)0x5006
#define PB_DDR *(volatile U8*)0x5007
#define PB_CR1 *(volatile U8*)0x5008
#define PB_CR2 *(volatile U8*)0x5009

#define PC_ODR *(volatile U8*)0x500A
#define PC_IDR *(volatile U8*)0x500B
#define PC_DDR *(volatile U8*)0x500C
#define PC_CR1 *(volatile U8*)0x500D
#define PC_CR2 *(volatile U8*)0x500E

#define PD_ODR *(volatile U8*)0x500F
#define PD_IDR *(volatile U8*)0x5010
#define PD_DDR *(volatile U8*)0x5011
#define PD_CR1 *(volatile U8*)0x5012
#define PD_CR2 *(volatile U8*)0x5013

#define PE_ODR *(volatile U8*)0x5014
#define PE_IDR *(volatile U8*)0x5015
#define PE_DDR *(volatile U8*)0x5016
#define PE_CR1 *(volatile U8*)0x5017
#define PE_CR2 *(volatile U8*)0x5018

#define PF_ODR *(volatile U8*)0x5019
#define PF_IDR *(volatile U8*)0x501A
#define PF_DDR *(volatile U8*)0x501B
#define PF_CR1 *(volatile U8*)0x501C
#define PF_CR2 *(volatile U8*)0x501D

#ifdef STM8S105
#define PG_ODR *(volatile U8*)0x501E
#define PG_IDR *(volatile U8*)0x501F
#define PG_DDR *(volatile U8*)0x5020
#define PG_CR1 *(volatile U8*)0x5021
#define PG_CR2 *(volatile U8*)0x5022

#define PH_ODR *(volatile U8*)0x5023
#define PH_IDR *(volatile U8*)0x5024
#define PH_DDR *(volatile U8*)0x5025
#define PH_CR1 *(volatile U8*)0x5026
#define PH_CR2 *(volatile U8*)0x5027

#define PI_ODR *(volatile U8*)0x5028
#define PI_IDR *(volatile U8*)0x5029
#define PI_DDR *(volatile U8*)0x502A
#define PI_CR1 *(volatile U8*)0x502B
#define PI_CR2 *(volatile U8*)0x502C
#endif // STM8S105

/* -------------------- FLASH/EEPROM -------------------- */
#define FLASH_CR1	*(volatile U8*)0x505A
#define FLASH_CR2	*(volatile U8*)0x505B
#define FLASH_NCR2	*(volatile U8*)0x505C
#define FLASH_FPR	*(volatile U8*)0x505D
#define FLASH_NFPR	*(volatile U8*)0x505E
#define FLASH_IAPSR	*(volatile U8*)0x505F
#define FLASH_PUKR	*(volatile U8*)0x5062 // progmem unprotection
#define FLASH_DUKR	*(volatile U8*)0x5064 // EEPROM unprotection

#define EEPROM_KEY1		0xAE  // keys to manage EEPROM's write access
#define EEPROM_KEY2		0x56
#define EEPROM_START_ADDR  (volatile U8*)0x4000

/* ------------------- interrupts ------------------- */
#define EXTI_CR1	*(volatile U8*)0x50A0
#define EXTI_CR2	*(volatile U8*)0x50A1
#define INTERRUPT_HANDLER(fn, num)		void fn() __interrupt(num)
#define INTERRUPT_DEFINITION(fn, num)	extern void fn() __interrupt(num)

// Reset status register
#define RST_SR		*(volatile U8*)0x50B3

/* ------------------- CLOCK ------------------- */
#define CLK_ICKR		*(volatile U8*)0x50C0
#define CLK_ECKR		*(volatile U8*)0x50C1
#define CLK_CMSR		*(volatile U8*)0x50C3
#define CLK_SWR			*(volatile U8*)0x50C4
#define CLK_SWCR		*(volatile U8*)0x50C5
#define CLK_CKDIVR		*(volatile U8*)0x50C6
#define CLK_SPCKENR1	*(volatile U8*)0x50C7
#define CLK_CSSR		*(volatile U8*)0x50C8
#define CLK_CCOR		*(volatile U8*)0x50C9
#define CLK_PCKENR2		*(volatile U8*)0x50CA
#define CLK_HSITRIMR	*(volatile U8*)0x50CC
#define CLK_SWIMCCR		*(volatile U8*)0x50CD

/* ------------------- Watchdog ------------------ */
#define WWDG_CR			*(volatile U8*)0x50D1
#define WWDG_WR			*(volatile U8*)0x50D2
#define IWDG_KR			*(volatile U8*)0x50E0
#define IWDG_PR			*(volatile U8*)0x50E1
#define IWDG_RLR		*(volatile U8*)0x50E2

/* ------------------- AWU, BEEP ------------------- */
#define AWU_CSR1		*(volatile U8*)0x50F0
#define AWU_APR			*(volatile U8*)0x50F1
#define AWU_TBR			*(volatile U8*)0x50F2
#define BEEP_CSR		*(volatile U8*)0x50F3

/* ------------------- SPI ------------------- */
#define SPI_CR1			*(volatile U8*)0x5200
#define SPI_CR2			*(volatile U8*)0x5201
#define SPI_ICR			*(volatile U8*)0x5202
#define SPI_SR			*(volatile U8*)0x5203
#define SPI_DR			*(volatile U8*)0x5204
#define SPI_CRCPR		*(volatile U8*)0x5205
#define SPI_RXCRCR		*(volatile U8*)0x5206
#define SPI_TXCRCR		*(volatile U8*)0x5207

#define SPI_CR1_MODE0           0x00
#define SPI_CR1_MODE1           0x01
#define SPI_CR1_MODE2           0x02
#define SPI_CR1_MODE3           0x03

/* ------------------- I2C ------------------- */
#define I2C_CR1			*(volatile U8*)0x5210
#define I2C_CR2			*(volatile U8*)0x5211
#define I2C_FREQR		*(volatile U8*)0x5212
#define I2C_OARL		*(volatile U8*)0x5213
#define I2C_OARH		*(volatile U8*)0x5214
#define I2C_DR			*(volatile U8*)0x5216
#define I2C_SR1			*(volatile U8*)0x5217
#define I2C_SR2			*(volatile U8*)0x5218
#define I2C_SR3			*(volatile U8*)0x5219
#define I2C_ITR			*(volatile U8*)0x521A
#define I2C_CCRL		*(volatile U8*)0x521B
#define I2C_CCRH		*(volatile U8*)0x521C
#define I2C_TRISER		*(volatile U8*)0x521D
#define I2C_PECR		*(volatile U8*)0x521E

/* ------------------- UART ------------------- */
#ifdef STM8S003
#define UART1_SR	*(volatile U8*)0x5230
#define UART1_DR	*(volatile U8*)0x5231
#define UART1_BRR1	*(volatile U8*)0x5232
#define UART1_BRR2	*(volatile U8*)0x5233
#define UART1_CR1	*(volatile U8*)0x5234
#define UART1_CR2	*(volatile U8*)0x5235
#define UART1_CR3	*(volatile U8*)0x5236
#define UART1_CR4	*(volatile U8*)0x5237
#define UART1_CR5	*(volatile U8*)0x5238
#define UART1_GTR	*(volatile U8*)0x5239
#define UART1_PSCR	*(volatile U8*)0x523A
#endif // STM8S003
#ifdef STM8S105
#define UART2_SR	*(volatile U8*)0x5240
#define UART2_DR	*(volatile U8*)0x5241
#define UART2_BRR1	*(volatile U8*)0x5242
#define UART2_BRR2	*(volatile U8*)0x5243
#define UART2_CR1	*(volatile U8*)0x5244
#define UART2_CR2	*(volatile U8*)0x5245
#define UART2_CR3	*(volatile U8*)0x5246
#define UART2_CR4	*(volatile U8*)0x5247
#define UART2_CR5	*(volatile U8*)0x5248
#define UART2_CR6	*(volatile U8*)0x5249
#define UART2_GTR	*(volatile U8*)0x524A
#define UART2_PSCR	*(volatile U8*)0x524B
#endif // STM8S105

/* UART_CR1 bits */
#define UART_CR1_R8 (1 << 7)
#define UART_CR1_T8 (1 << 6)
#define UART_CR1_UARTD (1 << 5)
#define UART_CR1_M (1 << 4)
#define UART_CR1_WAKE (1 << 3)
#define UART_CR1_PCEN (1 << 2)
#define UART_CR1_PS (1 << 1)
#define UART_CR1_PIEN (1 << 0)

/* UART_CR2 bits */
#define UART_CR2_TIEN (1 << 7)
#define UART_CR2_TCIEN (1 << 6)
#define UART_CR2_RIEN (1 << 5)
#define UART_CR2_ILIEN (1 << 4)
#define UART_CR2_TEN (1 << 3)
#define UART_CR2_REN (1 << 2)
#define UART_CR2_RWU (1 << 1)
#define UART_CR2_SBK (1 << 0)

/* USART_CR3 bits */
#define UART_CR3_LINEN (1 << 6)
#define UART_CR3_STOP2 (1 << 5)
#define UART_CR3_STOP1 (1 << 4)
#define UART_CR3_CLKEN (1 << 3)
#define UART_CR3_CPOL (1 << 2)
#define UART_CR3_CPHA (1 << 1)
#define UART_CR3_LBCL (1 << 0)

/* UART_SR bits */
#define UART_SR_TXE (1 << 7)
#define UART_SR_TC (1 << 6)
#define UART_SR_RXNE (1 << 5)
#define UART_SR_IDLE (1 << 4)
#define UART_SR_OR (1 << 3)
#define UART_SR_NF (1 << 2)
#define UART_SR_FE (1 << 1)
#define UART_SR_PE (1 << 0)


/* ------------------- TIMERS ------------------- */
/* TIM1 */
#define TIM1_CR1	*(volatile U8*)0x5250
#define TIM1_CR2	*(volatile U8*)0x5251
#define TIM1_SMCR	*(volatile U8*)0x5252
#define TIM1_ETR	*(volatile U8*)0x5253
#define TIM1_IER	*(volatile U8*)0x5254
#define TIM1_SR1	*(volatile U8*)0x5255
#define TIM1_SR2	*(volatile U8*)0x5256
#define TIM1_EGR	*(volatile U8*)0x5257
#define TIM1_CCMR1	*(volatile U8*)0x5258
#define TIM1_CCMR2	*(volatile U8*)0x5259
#define TIM1_CCMR3	*(volatile U8*)0x525A
#define TIM1_CCMR4	*(volatile U8*)0x525B
#define TIM1_CCER1	*(volatile U8*)0x525C
#define TIM1_CCER2	*(volatile U8*)0x525D
#define TIM1_CNTRH	*(volatile U8*)0x525E
#define TIM1_CNTRL	*(volatile U8*)0x525F
#define TIM1_PSCRH	*(volatile U8*)0x5260
#define TIM1_PSCRL	*(volatile U8*)0x5261
#define TIM1_ARRH	*(volatile U8*)0x5262
#define TIM1_ARRL	*(volatile U8*)0x5263
#define TIM1_RCR	*(volatile U8*)0x5264
#define TIM1_CCR1H	*(volatile U8*)0x5265
#define TIM1_CCR1L	*(volatile U8*)0x5266
#define TIM1_CCR2H	*(volatile U8*)0x5267
#define TIM1_CCR2L	*(volatile U8*)0x5268
#define TIM1_CCR3H	*(volatile U8*)0x5269
#define TIM1_CCR3L	*(volatile U8*)0x526A
#define TIM1_CCR4H	*(volatile U8*)0x526B
#define TIM1_CCR4L	*(volatile U8*)0x526C
#define TIM1_BKR	*(volatile U8*)0x526D
#define TIM1_DTR	*(volatile U8*)0x526E
#define TIM1_OISR	*(volatile U8*)0x526F


/* TIM_IER bits */
#define TIM_IER_BIE (1 << 7)
#define TIM_IER_TIE (1 << 6)
#define TIM_IER_COMIE (1 << 5)
#define TIM_IER_CC4IE (1 << 4)
#define TIM_IER_CC3IE (1 << 3)
#define TIM_IER_CC2IE (1 << 2)
#define TIM_IER_CC1IE (1 << 1)
#define TIM_IER_UIE (1 << 0)

/* TIM_CR1 bits */
#define TIM_CR1_APRE (1 << 7)
#define TIM_CR1_CMSH (1 << 6)
#define TIM_CR1_CMSL (1 << 5)
#define TIM_CR1_DIR (1 << 4)
#define TIM_CR1_OPM (1 << 3)
#define TIM_CR1_URS (1 << 2)
#define TIM_CR1_UDIS (1 << 1)
#define TIM_CR1_CEN (1 << 0)

/* TIM_SR1 bits */
#define TIM_SR1_BIF (1 << 7)
#define TIM_SR1_TIF (1 << 6)
#define TIM_SR1_COMIF (1 << 5)
#define TIM_SR1_CC4IF (1 << 4)
#define TIM_SR1_CC3IF (1 << 3)
#define TIM_SR1_CC2IF (1 << 2)
#define TIM_SR1_CC1IF (1 << 1)
#define TIM_SR1_UIF (1 << 0)

/* TIM2 */
#define TIM2_CR1	*(volatile U8*)0x5300
#if defined STM8S105 || defined STM8S103
#define TIM2_IER	*(volatile U8*)0x5301
#define TIM2_SR1	*(volatile U8*)0x5302
#define TIM2_SR2	*(volatile U8*)0x5303
#define TIM2_EGR	*(volatile U8*)0x5304
#define TIM2_CCMR1	*(volatile U8*)0x5305
#define TIM2_CCMR2	*(volatile U8*)0x5306
#define TIM2_CCMR3	*(volatile U8*)0x5307
#define TIM2_CCER1	*(volatile U8*)0x5308
#define TIM2_CCER2	*(volatile U8*)0x5309
#define TIM2_CNTRH	*(volatile U8*)0x530A
#define TIM2_CNTRL	*(volatile U8*)0x530B
#define TIM2_PSCR	*(volatile U8*)0x530C
#define TIM2_ARRH	*(volatile U8*)0x530D
#define TIM2_ARRL	*(volatile U8*)0x530E
#define TIM2_CCR1H	*(volatile U8*)0x530F
#define TIM2_CCR1L	*(volatile U8*)0x5310
#define TIM2_CCR2H	*(volatile U8*)0x5311
#define TIM2_CCR2L	*(volatile U8*)0x5312
#define TIM2_CCR3H	*(volatile U8*)0x5313
#define TIM2_CCR3L	*(volatile U8*)0x5314
#elif defined STM8S003
#define TIM2_IER	*(volatile U8*)0x5303
#define TIM2_SR1	*(volatile U8*)0x5304
#define TIM2_SR2	*(volatile U8*)0x5305
#define TIM2_EGR	*(volatile U8*)0x5306
#define TIM2_CCMR1	*(volatile U8*)0x5307
#define TIM2_CCMR2	*(volatile U8*)0x5308
#define TIM2_CCMR3	*(volatile U8*)0x5309
#define TIM2_CCER1	*(volatile U8*)0x530A
#define TIM2_CCER2	*(volatile U8*)0x530B
#define TIM2_CNTRH	*(volatile U8*)0x530C
#define TIM2_CNTRL	*(volatile U8*)0x530D
#define TIM2_PSCR	*(volatile U8*)0x530E
#define TIM2_ARRH	*(volatile U8*)0x530F
#define TIM2_ARRL	*(volatile U8*)0x5310
#define TIM2_CCR1H	*(volatile U8*)0x5311
#define TIM2_CCR1L	*(volatile U8*)0x5312
#define TIM2_CCR2H	*(volatile U8*)0x5313
#define TIM2_CCR2L	*(volatile U8*)0x5314
#define TIM2_CCR3H	*(volatile U8*)0x5315
#define TIM2_CCR3L	*(volatile U8*)0x5316
#endif


/* TIM3 */
#if defined STM8S105 || defined STM8S103
#define TIM3_CR1	*(volatile U8*)0x5320
#define TIM3_IER	*(volatile U8*)0x5321
#define TIM3_SR1	*(volatile U8*)0x5322
#define TIM3_SR2	*(volatile U8*)0x5323
#define TIM3_EGR	*(volatile U8*)0x5324
#define TIM3_CCMR1	*(volatile U8*)0x5325
#define TIM3_CCMR2	*(volatile U8*)0x5326
#define TIM3_CCER1	*(volatile U8*)0x5327
#define TIM3_CNTRH	*(volatile U8*)0x5328
#define TIM3_CNTRL	*(volatile U8*)0x5329
#define TIM3_PSCR	*(volatile U8*)0x532A
#define TIM3_ARRH	*(volatile U8*)0x532B
#define TIM3_ARRL	*(volatile U8*)0x532C
#define TIM3_CCR1H	*(volatile U8*)0x532D
#define TIM3_CCR1L	*(volatile U8*)0x532E
#define TIM3_CCR2H	*(volatile U8*)0x532F
#define TIM3_CCR2L	*(volatile U8*)0x5330
#endif

/* TIM4 */
#define TIM4_CR1	*(volatile U8*)0x5340
#if defined STM8S105 || defined STM8S103
#define TIM4_IER	*(volatile U8*)0x5341
#define TIM4_SR		*(volatile U8*)0x5342
#define TIM4_EGR	*(volatile U8*)0x5343
#define TIM4_CNTR	*(volatile U8*)0x5344
#define TIM4_PSCR	*(volatile U8*)0x5345
#define TIM4_ARR	*(volatile U8*)0x5346
#elif defined STM8S003
#define TIM4_IER	*(volatile U8*)0x5343
#define TIM4_SR		*(volatile U8*)0x5344
#define TIM4_EGR	*(volatile U8*)0x5345
#define TIM4_CNTR	*(volatile U8*)0x5346
#define TIM4_PSCR	*(volatile U8*)0x5347
#define TIM4_ARR	*(volatile U8*)0x5348
#endif

/* ------------------- ADC ------------------- */
#define ADC_DB0RH	*(volatile U8*)0x53E0
#define ADC_DB0RL	*(volatile U8*)0x53E1
#define ADC_DB1RH	*(volatile U8*)0x53E2
#define ADC_DB1RL	*(volatile U8*)0x53E3
#define ADC_DB2RH	*(volatile U8*)0x53E4
#define ADC_DB2RL	*(volatile U8*)0x53E5
#define ADC_DB3RH	*(volatile U8*)0x53E6
#define ADC_DB3RL	*(volatile U8*)0x53E7
#define ADC_DB4RH	*(volatile U8*)0x53E8
#define ADC_DB4RL	*(volatile U8*)0x53E9
#define ADC_DB5RH	*(volatile U8*)0x53EA
#define ADC_DB5RL	*(volatile U8*)0x53EB
#define ADC_DB6RH	*(volatile U8*)0x53EC
#define ADC_DB6RL	*(volatile U8*)0x53ED
#define ADC_DB7RH	*(volatile U8*)0x53EE
#define ADC_DB7RL	*(volatile U8*)0x53EF
#define ADC_DB8RH	*(volatile U8*)0x53F0
#define ADC_DB8RL	*(volatile U8*)0x53F1
#define ADC_DB9RH	*(volatile U8*)0x53F2
#define ADC_DB9RL	*(volatile U8*)0x53F3
#define ADC_CSR		*(volatile U8*)0x5400
#define ADC_CR1		*(volatile U8*)0x5401
#define ADC_CR2		*(volatile U8*)0x5402
#define ADC_CR3		*(volatile U8*)0x5403
#define ADC_DRH		*(volatile U8*)0x5404
#define ADC_DRL		*(volatile U8*)0x5405
#define ADC_TDRH	*(volatile U8*)0x5406
#define ADC_TDRL	*(volatile U8*)0x5407
#define ADC_HTRH	*(volatile U8*)0x5408
#define ADC_HTRL	*(volatile U8*)0x5409
#define ADC_LTRH	*(volatile U8*)0x540A
#define ADC_LTRL	*(volatile U8*)0x540B
#define ADC_AWSRH	*(volatile U8*)0x540C
#define ADC_AWSRL	*(volatile U8*)0x540D
#define ADC_AWCRH	*(volatile U8*)0x540E
#define ADC_AWCRL	*(volatile U8*)0x540F

/* ------------------- swim control ------------------- */
#define CFG_GCR			*(volatile U8*)0x7F60
#define SWIM_CSR		*(volatile U8*)0x7F80

/* ------------------- ITC ------------------- */
#define ITC_SPR1		*(volatile U8*)0x7F70
#define ITC_SPR2		*(volatile U8*)0x7F71
#define ITC_SPR3		*(volatile U8*)0x7F72
#define ITC_SPR4		*(volatile U8*)0x7F73
#define ITC_SPR5		*(volatile U8*)0x7F74
#define ITC_SPR6		*(volatile U8*)0x7F75
#define ITC_SPR7		*(volatile U8*)0x7F76
#define ITC_SPR8		*(volatile U8*)0x7F77


/* -------------------- UNIQUE ID -------------------- */
#if defined STM8S105 || defined STM8S103 // maybe some other MCU have this too???
#define U_ID00		(volatile U8*)0x48CD
#define U_ID01		(volatile U8*)0x48CE
#define U_ID02		(volatile U8*)0x48CF
#define U_ID03		(volatile U8*)0x48D0
#define U_ID04		(volatile U8*)0x48D1
#define U_ID05		(volatile U8*)0x48D2
#define U_ID06		(volatile U8*)0x48D3
#define U_ID07		(volatile U8*)0x48D4
#define U_ID08		(volatile U8*)0x48D5
#define U_ID09		(volatile U8*)0x48D6
#define U_ID10		(volatile U8*)0x48D7
#define U_ID11		(volatile U8*)0x48D8
#endif // defined STM8S105 || defined STM8S103

// CCR REGISTER: bits 3&5 should be 1 if you wanna change EXTI_CRx
#define CCR			*(volatile U8*)0x7F0A

#endif // __STM8L_H__

// #define 		*(volatile U8*)0x
