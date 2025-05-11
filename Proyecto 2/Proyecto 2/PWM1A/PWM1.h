/*
 * PWM1.h
 *
 * Created: 22/4/2025 12:19:00
 *  Author: yelen
 */ 


#ifndef PWM1_H_
#define PWM1_H_

#define  invert  1
#define non_invert  0
void initPWM1A();
void updateDutyCycle1(uint8_t duty);		//Asignarle valor a 0CR1A

#endif /* PWM1_H_ */