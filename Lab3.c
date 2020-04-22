/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 /*Alex Escobar EE 128 */

#include "fsl_device_registers.h"
#include <stdint.h>


unsigned short decoder[10] = {0x01AE, 0x0A0, 0x018D, 0x01A9, 0x00A3, 0x012B, 0x012F, 0x01A0, 0x01AF, 0x01A3};

uint32_t outp = 0, inp1 = 0, loc = 0, inp2 = 0;
unsigned long ADCcon = 0;
unsigned long volts = 0;


unsigned short ADC_read16b(void)
{
	ADC0_SC1A = 0x00; /*  Changed here to fit specifiations of lab 3*/
	while(ADC0_SC2 & ADC_SC2_ADACT_MASK);
	while(!(ADC0_SC1A & ADC_SC1_COCO_MASK));
	return ADC0_RA;
}


void software_delay(unsigned long delay)
{
	while (delay > 0) delay--;
}


void PORTA_IRQHandler(void)
{
	NVIC_ClearPendingIRQ(PORTA_IRQn);

	/*Read Port B*/
	inp1 = GPIOB_PDIR & 0x08;
	inp2 = GPIOB_PDIR & 0x04;
	ADCcon = ADC_read16b();

	outp = 0;

	if (inp1 == 0) /* ADC Mode 			BLUE*/
	{
		/*Read from ADC and convert to decimal value; (e.g., ADC reads 0xFF, voltage is 1.6)*/
		volts = (ADCcon*3.3)/65535;
		loc = volts%10;
		outp = decoder[loc];
	}
	else /* Count Mode */
	{
		if (inp2 == 0) /* Count Direction 		GREEN*/
		{
			if(loc == 0)
			{
				outp = decoder[loc];
				loc++;
			}
			else if(loc == 1)
			{
				outp = decoder[loc];
				loc++;
			}
			else if (loc == 2)
			{
				outp = decoder[loc];
				loc++;
			}
			else if(loc == 3)
			{
				outp = decoder[loc];
				loc++;
			}
			else if (loc == 4)
			{
				outp = decoder[loc];
				loc++;
			}
			else if(loc == 5)
			{
				outp = decoder[loc];
				loc++;
			}
			else if (loc == 6)
			{
				outp = decoder[loc];
				loc++;
			}
			else if(loc == 7)
			{
				outp = decoder[loc];
				loc++;
			}
			else if (loc == 8)
			{
				outp = decoder[loc];
				loc++;
			}
			else if( loc == 9)
			{
				outp = decoder[loc];
				loc = 0;
			}

		}

		else/*count down to 0 and roll over to 99*/
		{
			if(loc == 0)
			{
				outp = decoder[loc];
				loc = 9;
			}

			else if(loc == 1)
			{
				outp = decoder[loc];
				loc--;
			}
			else if( loc == 2)
			{
				outp = decoder[loc];
				loc--;
			}
			else if(loc == 3)
			{
				outp = decoder[loc];
				loc--;
			}
			else if (loc == 4)
			{
				outp = decoder[loc];
				loc--;
			}
			else if(loc == 5)
			{
				outp = decoder[loc];
				loc--;
			}
			else if (loc == 6)
			{
				outp = decoder[loc];
				loc--;
			}
			else if(loc == 7)
			{
				outp = decoder[loc];
				loc--;
			}
			else if (loc == 8)
			{
				outp = decoder[loc];
				loc--;
			}
			else if( loc == 9)
			{
				outp = decoder[loc];
				loc--;
			}
		}
	}
		/*Display ADC value or Counter value based on MODE_SW; PORT C and D to seven segments */
		GPIOC_PDOR = outp;


		/*Clear PORTA ISFR*/
		 PORTA_ISFR = (1 << 1);
}

void main (void)
{
/*Enable Port A, B, C, D, and ADC0 clock gating.  */
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK; /*Enable Port A Clock Gate Control*/
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK; /*Enable Port B Clock Gate Control*/
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK; /*Enable Port C Clock Gate Control*/
	SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK;  /*Enable ADC0   Clock Gate Control*/

 /*Configure PA1, PB[3:2], PC[8:7, 5:0], PD[7:0] for GPIO.
   Configure PA1 to trigger interrupts on falling edge input.*/
	PORTA_PCR1 = 0xA0100;
	PORTB_GPCLR = 0x000C0100;
	PORTC_GPCLR = 0x01BF0100;

/*Clear PORTA ISFR*/
 PORTA_ISFR = (1 << 1);

 /*Configure ADC for 16 bits, and to use bus clock.
 Disable the ADC module;*/
	 ADC0_CFG1 = 0x0C;
	 ADC0_SC1A = 0x1F;

 /*Set PB[3:2] for input;*/
	 //GPIOA_PDDR |= (0 << 1);
	 GPIOB_PDDR |= 0x00000000;

 /*Set PD[7:0] and PC[8:7,5:0] for output;*/
	 GPIOC_PDDR |= 0x000001BF;

  /*Enable Port A IRQ interrupts;*/
	 NVIC_EnableIRQ(PORTA_IRQn);

	 //unsigned long Delay = 0x200000;

	 for (;;)
	 {
		 PORTA_IRQHandler();

		 software_delay(Delay); /*Wait Delay Value*/
	 }
 }


////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
