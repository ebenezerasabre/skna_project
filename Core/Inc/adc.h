/*
 * adc.h
 *
 *  Created on: Feb 16, 2024
 *      Author: ebenezer
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include <stdint.h>
#include <stdio.h>

int adc_read(void);
void adc_init(void);
void adc_cnvrsn(void);
void gpio_init(void);

void led_off(void);
void led_on(void);
void use_delay(void);

void btn_prsd(void);

void user_led_toggle(void);
void prgrm_state_set(int * prgrm_state);
void prgrm_state_dsply(int * prgrm_state);
void adc_smp_sgnl(char * buff, int * prgrm_state);



#endif /* INC_ADC_H_ */
