/*
 * PWM1B.c
 *
 * Created: 22/4/2025 12:56:42
 *  Author: yelena Cotzojay
 */ 

#include <avr/io.h>
#include "PWM1B.h"

void initPWM1B(){
	
	DDRB |= (1 << DDB2);	//Setear bit 2 del puerto B como salida
	TCCR1A |= (1 << COM1B1);	//no invertido	
}

void  updateDutyCycle1B(uint8_t duty){
	OCR1B = duty;
}