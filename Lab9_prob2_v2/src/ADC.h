#include "mylib.h"
#ifndef _ADC_H_
#define _ADC_H_
float resistor_value = 0.0f;
/************************************************************************************
ADC Reference to manual p501
ADC Data structure is here

typedef struct
{
  __IO uint32_t ISR;          !< ADC interrupt and status register,             Address offset: 0x00
  __IO uint32_t IER;          !< ADC interrupt enable register,                 Address offset: 0x04
  __IO uint32_t CR;           !< ADC control register,                          Address offset: 0x08
  __IO uint32_t CFGR;         !< ADC configuration register 1,                  Address offset: 0x0C
  __IO uint32_t CFGR2;        !< ADC configuration register 2,                  Address offset: 0x10
  __IO uint32_t SMPR1;        !< ADC sampling time register 1,                  Address offset: 0x14
  __IO uint32_t SMPR2;        !< ADC sampling time register 2,                  Address offset: 0x18
       uint32_t RESERVED1;    !< Reserved,                                                      0x1C
  __IO uint32_t TR1;          !< ADC analog watchdog 1 threshold register,      Address offset: 0x20
  __IO uint32_t TR2;          !< ADC analog watchdog 2 threshold register,      Address offset: 0x24
  __IO uint32_t TR3;          !< ADC analog watchdog 3 threshold register,      Address offset: 0x28
       uint32_t RESERVED2;    !< Reserved,                                                      0x2C
  __IO uint32_t SQR1;         !< ADC group regular sequencer register 1,        Address offset: 0x30
  __IO uint32_t SQR2;         !< ADC group regular sequencer register 2,        Address offset: 0x34
  __IO uint32_t SQR3;         !< ADC group regular sequencer register 3,        Address offset: 0x38
  __IO uint32_t SQR4;         !< ADC group regular sequencer register 4,        Address offset: 0x3C
  __IO uint32_t DR;           !< ADC group regular data register,               Address offset: 0x40
       uint32_t RESERVED3;    !< Reserved,                                                      0x44
       uint32_t RESERVED4;    !< Reserved,                                                      0x48
  __IO uint32_t JSQR;         !< ADC group injected sequencer register,         Address offset: 0x4C
       uint32_t RESERVED5[4]; !< Reserved,                                               0x50 - 0x5C
  __IO uint32_t OFR1;         !< ADC offset register 1,                         Address offset: 0x60
  __IO uint32_t OFR2;         !< ADC offset register 2,                         Address offset: 0x64
  __IO uint32_t OFR3;         !< ADC offset register 3,                         Address offset: 0x68
  __IO uint32_t OFR4;         !< ADC offset register 4,                         Address offset: 0x6C
       uint32_t RESERVED6[4]; !< Reserved,                                               0x70 - 0x7C
  __IO uint32_t JDR1;         !< ADC group injected rank 1 data register,       Address offset: 0x80
  __IO uint32_t JDR2;         !< ADC group injected rank 2 data register,       Address offset: 0x84
  __IO uint32_t JDR3;         !< ADC group injected rank 3 data register,       Address offset: 0x88
  __IO uint32_t JDR4;         !< ADC group injected rank 4 data register,       Address offset: 0x8C
       uint32_t RESERVED7[4]; !< Reserved,                                             0x090 - 0x09C
  __IO uint32_t AWD2CR;       !< ADC analog watchdog 1 configuration register,  Address offset: 0xA0
  __IO uint32_t AWD3CR;       !< ADC analog watchdog 3 Configuration Register,  Address offset: 0xA4
       uint32_t RESERVED8;    !< Reserved,                                                     0x0A8
       uint32_t RESERVED9;    !< Reserved,                                                     0x0AC
  __IO uint32_t DIFSEL;       !< ADC differential mode selection register,      Address offset: 0xB0
  __IO uint32_t CALFACT;      !< ADC calibration factors,                       Address offset: 0xB4

} ADC_TypeDef;

typedef struct
{
  __IO uint32_t CSR;          !< ADC common status register,                    Address offset: ADC1 base address + 0x300
  uint32_t      RESERVED;     !< Reserved,                                      Address offset: ADC1 base address + 0x304
  __IO uint32_t CCR;          !< ADC common configuration register,             Address offset: ADC1 base address + 0x308
  __IO uint32_t CDR;          !< ADC common group regular data register         Address offset: ADC1 base address + 0x30C
} ADC_Common_TypeDef;


***********************************************************************************/
void configureADC()
{
	// TODO
    RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN; //Turn on the adc function
    GPIOB->ASCR |= 1; //turn on the analog controller in PB0
    ADC1->CR &= ~ADC_CR_DEEPPWD; // turn off power
    ADC1->CR |= ADC_CR_ADVREGEN; // enable adc voltage regulator
}
void startADC()
{
	// TODO
}

#endif
