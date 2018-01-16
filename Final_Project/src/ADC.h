#include "mylib.h"
#ifndef _ADC_H_
#define _ADC_H_
float resistor_value = 0.0f;
/************************************************************************************
ADC Reference to manual p501
ADC register reference to manual p580
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

stm32l476xx.h ---> Line 1477 has the configuration of ADC
***********************************************************************************/
void configureADC()
{
	// TODO
    RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN; //Turn on the adc function
    GPIOB->ASCR |= 0b1; //turn on the analog controller in PB0
    /************************ADC clock config starts here********************************/
    /* ############################################################################
     *  Set the ADC clock first by using ADC common register Reset value: 0x0000 0000
     *  The ADC common register can be found at manual p608
     *  We set the clock source as the sysclk , which is default 4MHz
     *  From LSB to MSB
     *  Bits
     *  4:0 Dual adc or not, we dont set dual adc, so keep reset value 00000 for independent mode
     *  11:8 Delay b/w 2 sampling phases, setting for 5 clock cycle will be fine which is 0100
     *  13 No DMA, reset value is fine
     *  15:14 No DMA, reset value is fine
     *  17:16 IMPORTANT!!!!!!!!!! Must config the Clock correctly!!!!!!!!!!! use HCLK hardware system clock/1 will be fine, set to 01
     *  21:18 No division, reset value is fine
     *  22 Not used, reset value is fine
     *  23 Not used, reset value is fine
     *  24 Not used, reset value is fine
     *  Else is the reserved value, should be kept in the reset state.
     * ###########################################################################*/
                          //10987654321098765432109876543210
    ADC123_COMMON->CCR |= 0b00000000000000010000010000000000;
    /************************ADC clock and some other config ends here********************************/
    /************************ADC main settings starts here********************************************/
    ADC1->CR &= ~ADC_CR_DEEPPWD; //Turn off the deep-power mode before the configuration
    ADC1->CR |= ADC_CR_ADVREGEN; //Turn on the voltage regulator
    while(!READ_BIT(ADC1->CR, ADC_CR_ADVREGEN)); //Polling until the ADCVERGEN is pulled up ot 1

    ADC1->CR |= 0x80000000; //Tell the ADC to do the calibration.
    while((ADC1->CR & 0x80000000) >> 31); //Polling until the calibration is done
    ADC1->CFGR &= ~ADC_CFGR_RES; // 12-bit resolution
    ADC1->CFGR &= ~ADC_CFGR_CONT; // Disable continuous conversion
    ADC1->CFGR &= ~ADC_CFGR_ALIGN; // Right align
                   //10987654321098765432109876543210
    ADC1->SQR1 |=  0b00000000000000000000001111000000; //We use only one ADC channel but problem is, which channel--> rank1 for channel15(pb0)
    //the higher sampling cycle can ensure the more detalied data but takes more process time, set12.5 will be fine, all takes 25 cycles
    ADC1->SMPR1 |= 0b00000000000000000000000000000010;
    //ADC conversion time may reference to manual p518
    ADC1->IER |= ADC_IER_EOCIE; //When conversion ends, do the interrupt (the end of conversion interrupt)
    /************************ADC main settings ends here********************************************/

}
//Start the ADC and do the resistor conversion
//ADC Clock is 4MHz,
void startADC()
{
	// TODO
    ADC1->CR |= ADC_CR_ADEN; //Turn on the ADC, write to enable the adc
    while(!READ_BIT(ADC1->ISR, ADC_ISR_ADRDY)); //Wait until ADC is ready for conversion
}

void get_light_resistor()
{
    ADC1->CR |= ADC_CR_ADSTART;
    while(!READ_BIT(ADC1->ISR, ADC_ISR_EOC)); //Polling until the ADC conversion of light resistor is done
    resistor_value = ADC1->DR;
}
#endif
