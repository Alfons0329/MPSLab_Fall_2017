
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "stm32l476xx.h"
#define PERIPH_BASE ((uint32_t)0x40000000U) /*!< Peripheral base address */
#define AHB2PERIPH_BASE (PERIPH_BASE + 0x08000000U)
#define GPIOA_BASE  (AHB2PERIPH_BASE + 0x0000U)
#define GPIOB_BASE  (AHB2PERIPH_BASE + 0x0400U)
#define GPIOC_BASE  (AHB2PERIPH_BASE + 0x0800U)
#define GPIOD_BASE  (AHB2PERIPH_BASE + 0x0C00U)
#define GPIOE_BASE  (AHB2PERIPH_BASE + 0x1000U)
#define GPIOF_BASE  (AHB2PERIPH_BASE + 0x1400U)
#define GPIOG_BASE  (AHB2PERIPH_BASE + 0x1800U)
#define GPIOH_BASE  (AHB2PERIPH_BASE + 0x1C00U)
typedef struct
{
   uint32_t MODER;       /*!< GPIO port mode register,               Address offset: 0x00      */
   uint32_t OTYPER;      /*!< GPIO port output type register,        Address offset: 0x04      */
   uint32_t OSPEEDR;     /*!< GPIO port output speed register,       Address offset: 0x08      */
   uint32_t PUPDR;       /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
   uint32_t IDR;         /*!< GPIO port input data register,         Address offset: 0x10      */
   uint32_t ODR;         /*!< GPIO port output data register,        Address offset: 0x14      */
   uint32_t BSRR;        /*!< GPIO port bit set/reset  register,     Address offset: 0x18      */
   uint32_t LCKR;        /*!< GPIO port configuration lock register, Address offset: 0x1C      */
   uint32_t AFR[2];      /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
   uint32_t BRR;         /*!< GPIO Bit Reset register,               Address offset: 0x28      */
   uint32_t ASCR;        /*!< GPIO analog switch control register,   Address offset: 0x2C     */

} GPIO_TypeDef;
typedef struct
{
   uint32_t CR;          /*!< RCC clock control register,                                              Address offset: 0x00 */
   uint32_t ICSCR;       /*!< RCC internal clock sources calibration register,                         Address offset: 0x04 */
   uint32_t CFGR;        /*!< RCC clock configuration register,                                        Address offset: 0x08 */
   uint32_t PLLCFGR;     /*!< RCC system PLL configuration register,                                   Address offset: 0x0C */
   uint32_t PLLSAI1CFGR; /*!< RCC PLL SAI1 configuration register,                                     Address offset: 0x10 */
   uint32_t PLLSAI2CFGR; /*!< RCC PLL SAI2 configuration register,                                     Address offset: 0x14 */
   uint32_t CIER;        /*!< RCC clock interrupt enable register,                                     Address offset: 0x18 */
   uint32_t CIFR;        /*!< RCC clock interrupt flag register,                                       Address offset: 0x1C */
   uint32_t CICR;        /*!< RCC clock interrupt clear register,                                      Address offset: 0x20 */
  uint32_t      RESERVED0;   /*!< Reserved,                                                                Address offset: 0x24 */
   uint32_t AHB1RSTR;    /*!< RCC AHB1 peripheral reset register,                                      Address offset: 0x28 */
   uint32_t AHB2RSTR;    /*!< RCC AHB2 peripheral reset register,                                      Address offset: 0x2C */
   uint32_t AHB3RSTR;    /*!< RCC AHB3 peripheral reset register,                                      Address offset: 0x30 */
  uint32_t      RESERVED1;   /*!< Reserved,                                                                Address offset: 0x34 */
   uint32_t APB1RSTR1;   /*!< RCC APB1 peripheral reset register 1,                                    Address offset: 0x38 */
   uint32_t APB1RSTR2;   /*!< RCC APB1 peripheral reset register 2,                                    Address offset: 0x3C */
   uint32_t APB2RSTR;    /*!< RCC APB2 peripheral reset register,                                      Address offset: 0x40 */
  uint32_t      RESERVED2;   /*!< Reserved,                                                                Address offset: 0x44 */
   uint32_t AHB1ENR;     /*!< RCC AHB1 peripheral clocks enable register,                              Address offset: 0x48 */
   uint32_t AHB2ENR;     /*!< RCC AHB2 peripheral clocks enable register,                              Address offset: 0x4C */
   uint32_t AHB3ENR;     /*!< RCC AHB3 peripheral clocks enable register,                              Address offset: 0x50 */
  uint32_t      RESERVED3;   /*!< Reserved,                                                                Address offset: 0x54 */
   uint32_t APB1ENR1;    /*!< RCC APB1 peripheral clocks enable register 1,                            Address offset: 0x58 */
   uint32_t APB1ENR2;    /*!< RCC APB1 peripheral clocks enable register 2,                            Address offset: 0x5C */
   uint32_t APB2ENR;     /*!< RCC APB2 peripheral clocks enable register,                              Address offset: 0x60 */
  uint32_t      RESERVED4;   /*!< Reserved,                                                                Address offset: 0x64 */
   uint32_t AHB1SMENR;   /*!< RCC AHB1 peripheral clocks enable in sleep and stop modes register,      Address offset: 0x68 */
   uint32_t AHB2SMENR;   /*!< RCC AHB2 peripheral clocks enable in sleep and stop modes register,      Address offset: 0x6C */
   uint32_t AHB3SMENR;   /*!< RCC AHB3 peripheral clocks enable in sleep and stop modes register,      Address offset: 0x70 */
  uint32_t      RESERVED5;   /*!< Reserved,                                                                Address offset: 0x74 */
   uint32_t APB1SMENR1;  /*!< RCC APB1 peripheral clocks enable in sleep mode and stop modes register 1, Address offset: 0x78 */
   uint32_t APB1SMENR2;  /*!< RCC APB1 peripheral clocks enable in sleep mode and stop modes register 2, Address offset: 0x7C */
   uint32_t APB2SMENR;   /*!< RCC APB2 peripheral clocks enable in sleep mode and stop modes register, Address offset: 0x80 */
  uint32_t      RESERVED6;   /*!< Reserved,                                                                Address offset: 0x84 */
   uint32_t CCIPR;       /*!< RCC peripherals independent clock configuration register,                Address offset: 0x88 */
   uint32_t RESERVED7;   /*!< Reserved,                                                                Address offset: 0x8C */
   uint32_t BDCR;        /*!< RCC backup domain control register,                                      Address offset: 0x90 */
   uint32_t CSR;         /*!< RCC clock control & status register,                                     Address offset: 0x94 */
} RCC_TypeDef;
