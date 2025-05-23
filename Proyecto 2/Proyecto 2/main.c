/*
 * Proyecto 2.c
 *
 * Created: 1/5/2025 10:25:48
 * Author : yelena Cotzojay
 * Descripci�n: El proyecto consta de la utilizaci�n de 4 servomotores
 haciendo uso de diferentes modos:
	Modo 1: Modo manual
	Modo 2: Modo EEPROM
	Modo 3: Modo Adafruit
 */

#define	F_CPU	16000000
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "PWM1A/PWM1.h"
#include "PWM1B/PWM1B.h"
uint8_t dutyCycle1= 128;	//Valor inicial en el centro (1.5 ms)
uint8_t dutyCycle2= 128;	//Valor inicial en el centro (1.5 ms)
uint8_t dutyCycle3= 128;	//Valor inicial en el centro (1.5 ms)
uint8_t dutyCycle4= 128;	//Valor inicial en el centro (1.5 ms)
uint8_t POT=0;	//Variable a utilizar para el multiplexeo de la lectura del ADC
uint8_t ADC1=0;
uint8_t ADC2=0;
uint8_t ADC3=0;
uint8_t ADC4=0;
uint8_t MODO=0;	//Selecciona en qu� modo se encuentra por medio de banderas
uint8_t contador=0;	//Contador para ver el modo en el que se encuentra
uint8_t contador1=0;	//contador para ver qu� posici�n se quiere leer
uint8_t indicador_acb=0;	//Bandera para el indicador de acci�n por medio de
uint8_t indicador_no_posicion=0;	//seleccione qu� posicio�n se va a guardar 
uint8_t lectura_servo_M1=0;
uint8_t lectura_servo_M2=0;
uint8_t lectura_servo_M3=0;
uint8_t lectura_servo_M4=0;

//Funcion Prototypes
void setup();
void INIT_TMR1();
void INIT_ADC();
void INIT_TIMER2();
void updateDutyCycle2A(uint8_t duty);
void updateDutyCycle2b(uint8_t duty);
void initUART();
void INIT_PIN_CHANGE();
void writeEEPROM(uint8_t dato, uint16_t direccion);
uint8_t readEEPROM(uint16_t direccion);
void writeChar(char caracter);
void writeString(char* texto);
void savepositions_servo();
void readpositions_servo();
void cargarposicion(uint8_t index);



int main(void)
{
	setup();
	//writeEEPROM('B', 0x00);
	uint8_t dire = 0x00;
	uint8_t temporal = readEEPROM(dire);
	while (temporal != 0xFF){
		writeChar(temporal);
		//writeEEPROM(0xFF,dire+0x00);
		dire++;
		temporal = readEEPROM(dire);
	}
	
	while (1)
	{	
		//Modo 1: Modo manual y guardar posiciones
		if(MODO==0b00000001){
			PORTD &= ~(1 << PORTD7);
			PORTB &= ~((1 << PORTB0)|(1 << PORTB4));
			PORTD |= (1 << PORTD6);
			savepositions_servo();
		}
		
		//Si se est� en modo dos verificar al funci�n de guardar posiciones
		if(MODO==0b00000010){
			PORTD &= ~(1 << PORTD6);
			PORTB &= ~((1 << PORTB0)|(1 << PORTB4));
			PORTD |= (1 << PORTD7);
			readpositions_servo();
		}
		
		//Si la est� en modo 3: Verifica la funci�n de leer posiciones para cada motor
		if (MODO==0b00000100){
			PORTD &= ~((1 << PORTD6)|(1 << PORTD7));
			PORTB &= ~(1 << PORTB4);
			PORTB |= (1 << PORTB0);
		}
		
	}
}

void setup(){
	cli();	//Desabilitar interrupciones
	INIT_TIMER2();
	INIT_ADC();
	initPWM1A();
	initPWM1B();
	
	INIT_PIN_CHANGE();
	initUART();
	//contador==1;
	

	//Configurar los puertos de salida (LEDS)
	DDRB |= (1 << PORTB0)|(1 << PORTB4);
	PORTB &= ~((1<< PORTB0)|(1<< PORTB4));
	DDRD |= (1 << PORTD7)|(1 << PORTD6);
	PORTD &= ~((1 << PORTD7)|(PORTD6));
	//Configurar el bit 3 del puerto C como entrada
	DDRC &= ~((1 << PORTC3)|(1 << PORTC2)|(1 << PORTC1)|(1 << PORTC0));
	PORTC |= ((1 << PORTC3)|(1 << PORTC2)|(1 << PORTC1)|(1 << PORTC0));
	//DDRB &= ~(1 << PORTB4);
	//PORTB |= (1 << PORTB4);//Pull up activado
	
	CLKPR = (1 << CLKPCE); //Habilita cambios de prescaler
	CLKPR = (1 << CLKPS2);	// 1MHz
	sei();		//Habilita cambios de interrupci�n
	
	ADCSRA |= (1 << ADSC); // Iniciar primera conversi�n

}

void INIT_TIMER2(){
	

	DDRB |= (1 << DDB3);	//Setear bit 3 del puerto B como salida
	DDRD |= (1 << DDD3);	//Setear bit 3 del puerto D como salida
		
	//CONFIGURACI�N DEL TIMER 2 PARA FAST PWM CON OCR1A COMO TOP
	TCCR2A = 0;
	TCCR2A |= (1 << COM2A1);	//no invertido para la salida 2A
	TCCR2A |= (1 << COM2B1);	//no invertido para la salida 2B
		
		
	//Modo fast PMW (MODO 3)
	TCCR2A |= (1 << WGM21)|(1 << WGM20);
	TCCR2B = 0;
	TCCR2B |= (1<<CS22)|(1<<CS20);	//Prescaler de 128
	
		
}

void  updateDutyCycle2A(uint8_t duty){
	OCR2A = duty;
}

void  updateDutyCycle2b(uint8_t duty){
	OCR2B = duty;
}

void INIT_PIN_CHANGE()
{//Habilitar interrupciones de pin change
	PCICR |= (1 << PCIE1)|(1 << PCIE0);     // Habilitar interrupci�n de cambio de estado
	// Habilita interrupci�n para bit 3, 2, 1, 0 deL Puerto C
	PCMSK1 |= (1 << PCINT11)|(1 << PCINT10)|(1 << PCINT9)|(1 << PCINT8); 
	
	//Habilitar interrupci�n para  bit 4 del puerto B
	PCMSK0 |= (1 << PCINT4);
}

void initUART(){
	//Configurar pines PD0 (TX) y PD1 (TX)
	DDRD |= (1<<DDD1);
	DDRD &= ~(1<<DDD0);
	
	//Configurar UCSR0A
	UCSR0A=0;
	UCSR0A= (1 << U2X0);	//Double the USART transmission Speed
	//Configuraci�n UCSR0B: Habililitndo la interrupci�n al escribir:
	//-Habilibitar recepci�n
	//-Habilitaci�n de transmisi�n
	UCSR0B |= (1<<RXCIE0)|(1<<RXEN0)| (1<< TXEN0);
	//Configurar UCSR0C
	UCSR0C |= (1<<UCSZ01) | (1<<UCSZ00);
	//Configurar UBRR0: UBRR0 = 12 -> 9600  @ 1MHz
	UBRR0 = 12;
}


void INIT_ADC(){
	ADMUX = 0;	//Apagar todo
	
	//Voltaje de referencia 5V
	ADMUX |=(1<<REFS0);
	
	ADMUX |= (1<<ADLAR); //orientaci�n (izquierda)
	ADCSRA = 0;	//Apagar todo
	ADCSRA |=(1<<ADPS1)	| (1 << ADPS0); //Configuraci�n del presacaler 8
	ADCSRA |= (1 << ADIE);	//Habilitaci�n de interrupciones
	ADCSRA |= (1 << ADEN);	//Habilitar ADS
}

void writeEEPROM(uint8_t dato, uint16_t direccion){
	//Esperar a que termine la escritura anterior
	while (EECR & (1 << EEPE));
	//Asignar direcci�n de escritura
	EEAR = direccion;
	//asignar dato a "escribir"
	EEDR = dato;
	//Setear en 1 el "master write enable"
	EECR |= (1 << EEMPE);
	//Empezar a escribir
	EECR |= (1 << EEPE);
}

uint8_t readEEPROM(uint16_t direccion){
	//Esperar a que termine la escritura anterior
	while (EECR & (1 << EEPE));
	//Asignar direcci�n de escritura
	EEAR = direccion;
	//Empezara a leer
	EECR |= (1 << EERE);
	return EEDR;
}

void writeChar(char caracter){
	while ( (UCSR0A & (1<<UDRE0))==0){
		
	}

	UDR0=caracter;
	
}

void writeString(char* texto)
{
	for(uint8_t i = 0; *(texto+i) !='\0'; i++)
	{
		writeChar(*(texto+i));
	}
	
}

void savepositions_servo(){
	//Verificar si se presion� el bot�n para guardar posici�n
	if (indicador_acb==0b00000001){
		
		writeEEPROM(ADC1, 0x00 + indicador_no_posicion*4);
		writeEEPROM(ADC2, 0x01 + indicador_no_posicion*4);
		writeEEPROM(dutyCycle3, 0x02 + indicador_no_posicion*4);
		writeEEPROM(dutyCycle4, 0x03 + indicador_no_posicion*4);
		
		writeChar('.');
		writeChar(dutyCycle1);
		writeChar('.');
		writeChar(dutyCycle2);
		writeChar('.');
		writeChar(dutyCycle3);
		writeChar('.');
		writeChar(dutyCycle4);
			
		writeString("Guardada posici�n ");
		writeChar('1' + indicador_no_posicion); // Mostrar posici�n 1, 2, 3, 4
		writeString("\r\n");
		
		indicador_no_posicion++;
		if (indicador_no_posicion >=4){
			indicador_no_posicion=0;
		}
	 indicador_acb=0;	//Apagar la bandera 
	}
	
}

void readpositions_servo(){
	if(indicador_acb==0b00000001){
		cargarposicion(0);
		}
		
	if(indicador_acb==0b00000010){
		cargarposicion(1);
	}
	
	if(indicador_acb==0b00000100){
		cargarposicion(2);
	}
	
	if(indicador_acb==0b00001000){
		cargarposicion(3);
	}
	indicador_acb=0; //Apagar bandera
}

void cargarposicion(uint8_t index){
	
	uint8_t adc1 = readEEPROM(0x00 + index*4);
	uint8_t adc2 = readEEPROM(0x01 + index*4);
	dutyCycle3 = readEEPROM(0x02 + index*4);
	dutyCycle4 = readEEPROM(0x03 + index*4);
	
	dutyCycle1 = (adc1 * (188.0 / 255.0)) + 69.0;
	dutyCycle2 = (adc2 * (188.0 / 255.0)) + 69.0;
	
	// Actualizar todos los PWM
	updateDutyCycle1(dutyCycle1);
	updateDutyCycle1B(dutyCycle2);
	updateDutyCycle2A(dutyCycle3);
	updateDutyCycle2b(dutyCycle4);
	 
	writeString("posici�n le�da ");
	writeChar('1' + index); // Mostrar posici�n 1, 2, 3, 4
	writeString("\r\n");
}
/*************VECTOR DE INTERRUPCI�N************/

ISR(ADC_vect)
{
	if (MODO == 0b00000001){
		POT++;
		
		switch (POT){
			case 1:
			ADMUX &= ~((1<<MUX2)|(1<<MUX1)|(1<<MUX0));  // Limpiar bit
			ADMUX |= (1<<MUX2) | (1<<MUX1);//Selecci�n de canal Bit 6 del puerto C
			ADC1=ADCH;
			dutyCycle1 = (ADC1 * (186.0 / 255.0)) + 68.0;
			updateDutyCycle1(dutyCycle1); // Actualizar PWM
			break;
			
			case 2:
			ADMUX &= ~((1<<MUX2)|(1<<MUX1)|(1<<MUX0));  // Limpiar bit
			ADMUX |= (1<<MUX2)|(1<<MUX1)|(1<<MUX0); //Selecci�n de canal Bit 7 del puerto C
			ADC2=ADCH;
			dutyCycle2 = (ADC2 *  (186.0 / 255.0)) + 68.0;
			updateDutyCycle1B(dutyCycle2); // Actualizar PWM
			break;
			
			case 3:
			
			ADMUX &= ~((1<<MUX2)|(1<<MUX1)|(1<<MUX0));  // Limpiar bit MUX0 primero
			ADMUX |= (1<<MUX2) | (1<<MUX0);//Selecci�n de canal Bit 5 del puerto C
			ADC3=ADCH;
			dutyCycle3 = (ADC3 * (24.0 / 255.0)) + 2.0;
			updateDutyCycle2A(dutyCycle3); // Actualizar PWM
			break;
			
			case 4:
			
			ADMUX &= ~((1<<MUX2)|(1<<MUX1)|(1<<MUX0));  // Limpiar bit MUX0 primero
			ADMUX |= (1<<MUX2);//Selecci�n de canal Bit 4 del puerto C
			ADC4=ADCH;
			dutyCycle4 = (ADC4 * (24.0 / 255.0)) + 2.0;
			updateDutyCycle2b(dutyCycle4); // Actualizar PWM
			break;
			
			case 5:
			POT=0;
			break;
			
			default:
			break;
		}
	}
	ADCSRA |= (1 << ADSC);	//Iniciar nueva conversi�n
}
	


ISR(PCINT1_vect) {
	//Leer estado actua de los botones
	uint8_t	estado_actual_C=PINC;

	
	
	//Detectar flanco para el bot�n de modo
	if (!(estado_actual_C & (1<<PINC3))){
		contador++;
	}
	switch (contador){
		case 1:
			MODO=0b00000001;	//Activar la bandra del modo 1: Manual
			break;
			
		case 2:
			MODO=0b00000010;	// Modo 2: Guardar posiciones en la EEPROM
			break;
			
		case 3:
			MODO=0b00000100;	//Modo 3: leer datos de la EEPROM
			break;
			
		case 4:
			contador=1;
			break;
		
		default:
			break;
		
	}
	
	//Bot�n 2 (guardar posiciones) para el modo 1
	//Leer primera posici�n para el modo 2
	if (!(estado_actual_C & (1<<PINC2))){
		indicador_acb=0b00000001;
	}

	
	//Leer segunda posici�n para el modo 2
	if (!(estado_actual_C & (1<<PINC1))){
		indicador_acb=0b00000010;
	}
	
	
	//Leer tercera posici�n para el modo 2
	if (!(estado_actual_C & (1<<PINC0))){
		indicador_acb=0b00000100;
	}
	
	
}

/*ISR(PCINT0_vect) {
	//Leer cuarta posici�n para el modo 2/
	if (!(PINB & (1 << PINB4))){
		indicador_acb=0b00001000;
	}
}*/

ISR(USART_RX_vect){
	char caracter=UDR0;
	switch(caracter){
		case '0':
			MODO=0b00000001;
			break;
			
		case '1':
			MODO=0b00000010;
			break;
			
		case '2':
			MODO=0b00000100;
			break;
			
		default:
			break;
	}
	
	writeChar(caracter);
	}
	
	
