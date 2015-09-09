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
#include "baremetal_dac_timer_irq.h"       /* <= own header */
#include "teclas_y_leds.h"

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




uint32_t DatoDAC;
uint32_t Vmax,T,t;

void RIT_IRQHandler(void){
    /* Clearn interrupt */
   Chip_RIT_ClearInt(LPC_RITIMER);
   t++;
   t%=T;
   DatoDAC=(t*Vmax)/T;
   /* Saco dato por DAC */
   Chip_DAC_UpdateValue(LPC_DAC,DatoDAC);
   if (t==0) {
  	   InvierteLed(1);
       }
    }

int main(void)
{
   /* perform the needed initialization here */
	uint32_t tecla =0;
	uint64_t i;
	Vmax=930;
    T=100;
	DatoDAC=0;


	Chip_SCU_DAC_Analog_Config(); //select DAC function
	Chip_DAC_Init(LPC_DAC); //initialize DAC
	Chip_DAC_SetBias(LPC_DAC, DAC_MAX_UPDATE_RATE_400kHz);
	Chip_DAC_SetDMATimeOut(LPC_DAC, 0xffff);
	Chip_DAC_ConfigDAConverterControl(LPC_DAC, DAC_CNT_ENA | DAC_DMA_ENA);


	
    Chip_RIT_Init(LPC_RITIMER);
    Chip_RIT_SetTimerInterval(LPC_RITIMER,10);

    InicializaPuertosTeclasYLeds();

    NVIC_EnableIRQ(RITIMER_IRQn);

       while(1) {
    	 tecla=LeeTecla();
    	 if (tecla!=0 ) {
    	   switch(tecla){
    	     case 1:{if (Vmax<1011){
    		           Vmax+=10;
    	               }
                     break;
    	             }
    	     case 2:{if (Vmax>0){
		               Vmax-=10;
	                   }
    	             break;
    	       	     }
    	     case 3:{T+=10;
    	             break;
    	       	     }
    	     case 4:{if(T>10){
    		           T-=10;
    	               }
    	             break;
    	       	     }
    	     }
    	   for (i=0;i<3000000;i++){
    	     asm  ("nop");
    	     }
    	   }
         }
         return 0;


}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

