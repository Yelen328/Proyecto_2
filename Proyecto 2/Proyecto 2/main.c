/*
 * Proyecto 2.c
 *
 * Created: 1/5/2025 10:25:48
 * Author : yelena Cotzojay
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
uint8_t MODO=0;
uint8_t contador=0;



//Función Prototypes
void setup();
void INIT_TMR1();
void INIT_ADC();
void INIT_TIMER2();
void updateDutyCycle2A(uint8_t duty);
void initUART();
void INIT_PIN_CHANGE();


int main(void)
{
	setup();
	
	while (1)
	{	
		
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
	

	//Configurar los puertos de salida (LEDS)
	DDRB |= (1 << PORTB0);
	PORTB &= ~(1<< PORTB4);
	DDRD |= (1 << PORTD7)|(1 << PORTD6);
	PORTD &= ~((1 << PORTD7)|(PORTD6));
	//Configurar el bit 3 del puerto C como entrada
	PORTC &= ~(1 << PORTC3);
	PORTC |= (1 << PORTC3);		//Pull up activado
	
	//CLKPR = (1 << CLKPCE); //Habilita cambios de prescaler
	//CLKPR = (1 << CLKPS2);	// 1MHz
	sei();		//Habilita cambios de interrupción
	
	ADCSRA |= (1 << ADSC); // Iniciar primera conversión

}

void INIT_TIMER2(){
	

	DDRB |= (1 << DDB3);	//Setear bit 3 del puerto B como salida
	//DDRD |= (1 << DDD3);	//Setear bit 3 del puerto D como salida
		
	//CONFIGURACIÓN DEL TIMER 2 PARA FAST PWM CON OCR1A COMO TOP
	TCCR2A = 0;
	TCCR2A |= (1 << COM2A1);	//no invertido para la salida 2A
	//TCCR2A |= (1 << COM2B1);	//no invertido para la salida 2B
		
		
	//Modo fast PMW (MODO 3)
	TCCR2A |= (1 << WGM21)|(1 << WGM20);
	TCCR2B = 0;
	TCCR2B |= (1<<CS22)|(1<<CS20);	//Prescaler de 128
	OCR2A =200;  // ~78% duty
		
}

void  updateDutyCycle2A(uint8_t duty){
	OCR2A = duty;
}

void INIT_PIN_CHANGE()
{//Habilitar interrupciones de pin change
	PCICR |= (1 << PCIE1);     // Habilitar interrupción de cambio de estado
	PCMSK1 |= (1 << PCINT11);  // Habilita interrupción para PORTC3 (bit 3 de Puerto C)
}

void initUART(){
	//Configurar pines PD0 (TX) y PD1 (TX)
	DDRD |= (1<<DDD1);
	DDRD &= ~(1<<DDD0);
	
	//Configurar UCSR0A
	UCSR0A=0;
	//Configuración UCSR0B: Habililitndo la interrupción al escribir:
	//-Habilibitar recepción
	//-Habilitación de transmisión
	UCSR0B |= (1<<RXCIE0)|(1<<RXEN0)| (1<< TXEN0);
	//Configurar UCSR0C
	UCSR0C |= (1<<UCSZ01) | (1<<UCSZ00);
	//Configurar UBRR0: UBRR0 = 103 -> 9600  @ 16MHz
	UBRR0 = 103;
}


void INIT_ADC(){
	ADMUX = 0;	//Apagar todo
	
	//Voltaje de referencia 5V
	ADMUX |=(1<<REFS0);
	
	ADMUX |= (1<<ADLAR); //orientación (izquierda)
	ADCSRA = 0;	//Apagar todo
	ADCSRA |=(1<<ADPS1)	| (1 << ADPS0); //Configuración del presacaler 8
	ADCSRA |= (1 << ADIE);	//Habilitación de interrupciones
	ADCSRA |= (1 << ADEN);	//Habilitar ADS
}

/*************VECTOR DE INTERRUPCIÓN************/

ISR(ADC_vect)
{
	POT++;
	
	switch (POT){
		case 1:
		ADMUX &= ~((1<<MUX2)|(1<<MUX1)|(1<<MUX0));  // Limpiar bit
		ADMUX |= (1<<MUX2) | (1<<MUX1);//Selección de canal Bit 6 del puerto C
		ADC1=ADCH;
		dutyCycle1 = (ADC1 * (180.0 / 255.0)) + 70.0;
		updateDutyCycle1(dutyCycle1); // Actualizar PWM
		break;
		
		case 2:
		ADMUX &= ~((1<<MUX2)|(1<<MUX1)|(1<<MUX0));  // Limpiar bit
		ADMUX |= (1<<MUX2)|(1<<MUX1)|(1<<MUX0); //Selección de canal Bit 7 del puerto C
		ADC2=ADCH;
		dutyCycle2 = 1010.00 + ADC2 * (4000.00/255.00);
		updateDutyCycle1B(dutyCycle2); // Actualizar PWM
		break;
		
		case 3:
		
		ADMUX &= ~((1<<MUX2)|(1<<MUX1)|(1<<MUX0));  // Limpiar bit MUX0 primero
		ADMUX |= (1<<MUX2) | (1<<MUX0);//Selección de canal Bit 5 del puerto C
		ADC3=ADCH;
		dutyCycle3=200;
		updateDutyCycle2A(dutyCycle3); // Actualizar PWM
		break;
		
		case 4:
		POT=0;
		break;
		
		default:
		break;
	}
	
	ADCSRA |= (1 << ADSC);	//Iniciar nueva conversión
}
	


/*ISR(PCINT1_vect) {
	//Leer estado actua de los botones
	uint8_t	estado_actual=PINC;
	
	
	//Detectar flanco
	if (!(estado_actual & (1<<PINC3))){
		contador++;
	}
	switch (contador){
		case 1:
			MODO=0b00000001;	//Activar la bandra del modo 1
			break;
			
		case 2:
			MODO=0b00000010;	//Activar al bandera del modo 2
			break;
			
		case 3:
			MODO=0b00000100;	//Activar la bandera del modo 3
			break;
			
		case 4:
			contador=1;
			break;
		
		default:
			break;
		
	}
	
}*/

/*ISR(USART_RX_vect){
	
	if (MODO==0b00000010){
		//APAGAR LOS LEDS QUE NO SE ESTÁN UTILIZANDO
		PORTD &= ~(PORTD6);
		PORTB &= ~(1 << PORTB0);
		//Enceder el led indicador
		PORTD |= (1 << PORTD7);
	}*/
	
/*
	caracter=UDR0;
	
	if (waiting_for_led_char){
		PORTB= caracter;
		writeString("\n\rLEDs actualizados con: ");
		WriteChar(caracter);
		writeString("\n\r");
		waiting_for_led_char = 0;  // Desactivar bandera
	}
	
	else{
		WriteChar(caracter);
		writeString("\n\r");  // Añadir un salto de línea
	}
	
	if (caracter & (1 << 6))
	{
		PORTD |= (1 << PORTD6);
		}else{
		PORTD &= ~(1 << PORTD6);
	}
	if (caracter & (1 << 7))
	{
		PORTD |= (1 << PORTD7);
		}else{
		PORTD &= ~(1 << PORTD7);
	}*/

