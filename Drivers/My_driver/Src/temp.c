/*
 * adc.c
 *
 *  Created on: Nov 2, 2023
 *      Author: ADMIN
 */


#include <stdint.h>
#include "temp.h"

void adc_init()
{
	//__HAL_RCC_ADC_CLK_ENABLE();
	uint32_t* RCC_APB2ENR = (uint32_t*)(0x40023800 + 0x44);
	*RCC_APB2ENR |= (1 << 8);

	uint32_t* ADC_CCR = (uint32_t*)(0x40012000 + 0x300 + 0x04);
	*ADC_CCR |= (1 << 23);

	uint32_t* ADC_JSQR = (uint32_t*)(0x40012000 + 0x38);
	*ADC_JSQR &= ~(0b11 << 20); //1 conversion
	*ADC_JSQR |= (16 << 15);    //select channel 16 (temp sensor) to measure

	uint32_t* ADC_CR2 = (uint32_t*)(0x40012000 + 0x08);
	*ADC_CR2 |= (1 << 0);

	uint32_t* ADC_SMPR1 = (uint32_t*)(0x40012000 + 0x0C);
	*ADC_SMPR1 |= (0b111 << 18); //select sample time for channel 16 is 480 cycles
}

int adc_measure_vin()
{
	uint32_t* ADC_CR2 = (uint32_t*)(0x40012000 + 0x08);
	*ADC_CR2 |= (1 << 22); //start injected conversion

	uint32_t* ADC_SR = (uint32_t*)(0x40012000 + 0x00);
	while(((*ADC_SR >> 2) &1 ) !=1); //wait until injected conversion complete

	uint32_t* ADC_JDR1 = (uint32_t*)(0x40012000 + 0x3C);
	uint16_t Dr = *ADC_JDR1 & 0Xfff;
	float Vin = (Dr * 3000)/4095;
	return Vin;
}





