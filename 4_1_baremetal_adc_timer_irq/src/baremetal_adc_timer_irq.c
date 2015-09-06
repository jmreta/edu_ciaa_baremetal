/* Copyright 2015, Eduardo Filomena
 * All rights reserved.
 *
 * This file is part of CIAA Firmware.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** \brief Bare Metal example source file
 **
 ** This is a mini example of the CIAA Firmware.
 **
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */

/** \addtogroup Examples CIAA Firmware Examples
 ** @{ */
/** \addtogroup Baremetal Bare Metal example source file
 ** @{ */

/*
 * Initials     Name
 * ---------------------------
 *
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * yyyymmdd v0.0.1 initials initial version
 */

/*==================[inclusions]=============================================*/
#include "baremetal_adc_timer_irq.h"       /* <= own header */

#ifndef CPU
#error CPU shall be defined
#endif
#if (lpc4337 == CPU)
#include "chip.h"
#elif (mk60fx512vlq15 == CPU)
#else
#endif


/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/
/** \brief Main function
 *
 * This is the main entry point of the software.
 *
 * \returns 0
 *
 * \remarks This function never returns. Return value is only to avoid compiler
 *          warnings or errors.
 */


void RIT_IRQHandler(void){
    /* Clearn interrupt */
   Chip_RIT_ClearInt(LPC_RITIMER);

   /* Toggle LED */
   Chip_GPIO_SetPortToggle(LPC_GPIO_PORT,5,2);      //Puerto 5  bit 1--> LED VERDE
    }

int main(void)
{
   /* perform the needed initialization here */


	volatile uint64_t i;
	static ADC_CLOCK_SETUP_T ADCSetup;
	static volatile uint8_t Burst_Mode_Flag = 0, Interrupt_Continue_Flag;
	static volatile uint8_t ADC_Interrupt_Done_Flag;
	uint16_t dataADC;

	Chip_SCU_ADC_Channel_Config(0,1);
	Chip_ADC_Init(LPC_ADC0,&ADCSetup);

	ADCSetup.adcRate=1000;
    ADCSetup.bitsAccuracy=ADC_10BITS;
    ADCSetup.burstMode=DISABLE;


	Chip_ADC_EnableChannel(LPC_ADC0,ADC_CH1,ENABLE);
	Chip_ADC_SetSampleRate(LPC_ADC0, &ADCSetup,ADC_MAX_SAMPLE_RATE);


    Chip_RIT_Init(LPC_RITIMER);
    Chip_RIT_SetTimerInterval(LPC_RITIMER,1000);

    Chip_GPIO_Init(LPC_GPIO_PORT);
    Chip_SCU_PinMux(2,0,MD_PUP,FUNC4);  /* GPIO5[0], LED0R */
    Chip_SCU_PinMux(2,1,MD_PUP,FUNC4);  /* GPIO5[1], LED0G */
    Chip_SCU_PinMux(2,2,MD_PUP,FUNC4);  /* GPIO5[2], LED0B */
    Chip_SCU_PinMux(2,10,MD_PUP,FUNC0); /* GPIO0[14], LED1 */
    Chip_SCU_PinMux(2,11,MD_PUP,FUNC0); /* GPIO1[11], LED2 */
    Chip_SCU_PinMux(2,12,MD_PUP,FUNC0); /* GPIO1[12], LED3 */

//    Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT,2,10);      //puerto 2 bit 10
  //  Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT,2,11);      //puerto 2 bit 11
  //  Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,2,10);      //puerto 2 bit 10)
  //  Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,2,10);      //puerto 2 bit 10)

    Chip_GPIO_SetDir(LPC_GPIO_PORT, 5,(1<<0)|(1<<1)|(1<<2),1);
    Chip_GPIO_SetDir(LPC_GPIO_PORT, 0,(1<<14),1);
    Chip_GPIO_SetDir(LPC_GPIO_PORT, 1,(1<<11)|(1<<12),1);

    Chip_GPIO_ClearValue(LPC_GPIO_PORT, 5,(1<<0)|(1<<1)|(1<<2));
    Chip_GPIO_ClearValue(LPC_GPIO_PORT, 0,(1<<14));
    Chip_GPIO_ClearValue(LPC_GPIO_PORT, 1,(1<<11)|(1<<12));


   // NVIC_EnableIRQ(RITIMER_IRQn);

    while(1){
    	/* Start A/D conversion */
    	Chip_ADC_SetStartMode(LPC_ADC0, ADC_START_NOW, ADC_TRIGGERMODE_RISING);
      /* Waiting for A/D conversion complete */
      while (Chip_ADC_ReadStatus(LPC_ADC0,ADC_CH1,ADC_DR_DONE_STAT) != SET) {}
      /* Read ADC value */
      Chip_ADC_ReadValue(LPC_ADC0,ADC_CH1, &dataADC);
     if (dataADC>500){
    	 Chip_GPIO_ClearValue(LPC_GPIO_PORT, 5, 1);
       }
     else{
    	 Chip_GPIO_SetValue(LPC_GPIO_PORT, 5, 1);
       }
     }


       while(1) {
            /* add your code here */
    	     Chip_GPIO_SetPortToggle(LPC_GPIO_PORT,0,1<<14);      //Puerto 5  bit 0 --> LED ROJO
             //Chip_GPIO_ClearValue(LPC_GPIO_PORT, 5, 1);

          //   Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,5,0);      //Puerto 5  bit 0 --> LED ROJO
             for (i=0;i<3000000;i++){
               asm  ("nop");
               }
            // Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, 5, 0);   //Puerto 5  bit 0 --> LED ROJO
             for (i=0;i<3000000;i++){
               asm  ("nop");
               }
         }
         return 0;


}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

