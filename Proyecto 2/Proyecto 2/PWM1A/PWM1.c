/*
 * PWM1.c
 *
 * Created: 22/4/2025 12:18:40
 *  Author: yelena Cotzojay
 */ 

#include <avr/io.h>
#include "PWM1.h"

void initPWM1A(){
	
	DDRB |= (1 << DDB1);	//Setear bit 1 del puerto B como salida
	
	//CONFIGURACIÓN DEL TIMER 1 PARA FAST PWM CON OCR1A COMO TOP
	TCCR1A = 0;
	TCCR1A |= (1 << COM1A1);	//no invertido
	
	//Modo fast PMW y top->OCR1A (MODO 14)
	TCCR1A |= (1 << WGM11);
	TCCR1B |= (1 << WGM13) | (1 << WGM12);
	TCCR1B |= (1<<CS11);	//Prescaler de 8
	ICR1 = 2499;	//TOP
	
}

void  updateDutyCycle1(uint8_t duty){
	OCR1A = duty;
}
