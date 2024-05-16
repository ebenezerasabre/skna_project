/*
 * adc.c
 *
 *  Created on: Feb 16, 2024
 *      Author: ebenezer
 */


#include "stm32f405xx.h"
#include "adc.h"
#include "stdbool.h"


#define ADC_EOC		(1U<<1)
#define ADC_CONT	(1U<<1)		// continuous conversion
#define ADC_ADON	(1U<<0)		// Enable ADC
#define ADC_SWSTART	(1U<<30)	// start conversion
#define PC10		(1U<<10)	//


#define USER_LED	PC10
#define PB8			(1U<<8)
#define PC13		(1U<<13)
#define SW_BUTTON	PB8		// first input button
#define SW2_BUTTON	PC13	// second input button

bool prsd = false;
int clk = 0;

int i;


void gpio_init(void){


}




// PA1, PA2, PA3 are analog channels
void adc_init(void){
	// enable clock access to GPIOA

	/*Set pin modes*/

	// set SW_BUTTON as input button
	GPIOB->MODER = 0;	// set all bits to zero

	// set SW_BUTTON as input button
	GPIOC->MODER = 0; // set all bits to zero

	/*
	GPIOB->MODER |= (1U<<16);
	GPIOB->MODER |= (1U<<16);

	// set SW2_BUTTON as input button
	GPIOB->MODER |= (1U<<26);
	GPIOB->MODER |= (1U<<27);

	*/


	// set mode of PA0 to analog

	// enable clock access to adc module


	// ADC data resolution

	// select ADC channel


	// enable ADC module
}


void adc_cnvrsn(void){

	// continuous conversion mode
	ADC1->CR2 |= ADC_CONT;

	// conversion sequence start
	ADC1->CR2 |= ADC_SWSTART;
}


int adc_read(void){
	while(!(ADC1->SR & ADC_EOC)){}
	return ADC1->DR;
}




void prgrm_state_set(int * prgrm_state){

}

void prgrm_state_dsply(int * prgrm_state){

}

void adc_smp_sgnl(char * buff, int * prgrm_state){


}


void led_off(void){
	GPIOC->ODR &= ~USER_LED;
}

void led_on(void){
	GPIOC->ODR |= USER_LED;

}

void user_led_toggle(void){
	GPIOC->ODR ^= USER_LED;
	use_delay();
}

void use_delay(void){
	for(i=0; i < 500000; i++)
		;
}

void btn_prsd(void){
	if(GPIOB->IDR & SW_BUTTON){
		//led_on();
		prsd = true;

	}

	if(!(GPIOB->IDR & SW_BUTTON)){
			//led_on();

		}

	if(GPIOC->IDR & SW_BUTTON){
		//led_on();
		//prsd = true;

	}

	if(!(GPIOC->IDR & SW_BUTTON)){
			//led_on();

		}

	if(prsd){
		led_on();
		//user_led_toggle();
	}

}





