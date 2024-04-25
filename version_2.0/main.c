#define __AVR_ATmega48__
#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 1000000UL
#include <util/delay.h>
#include <avr/cpufunc.h>
#include <string.h>
#include <stdio.h>

// color sense ADC channels
#define AL_adc 2
#define AH_adc 3

char selected_adc_channel = AL_adc;

uint16_t ADC_high_buffer[10] = { 0 }, ADC_low_buffer[10] = { 0 }, ADC_high_value = 0, ADC_low_value = 0;
uint8_t adc_read_count = 0, int_cnt = 0;
uint16_t second_counter = 0;
uint8_t sec, min, hour, adc_hold = 0;

void USART_Transmit( unsigned char data );
unsigned char USART_Receive( void );
void USART_Flush( void );
void USART_text(unsigned char* text);


ISR(TIMER0_OVF_vect){
    if ( ADCSRA & ( 1 << ADIF ) ) {         // ADC conversion complete flag
        if ( adc_hold ) {
            if ( adc_hold > 3 ) {       // wait with starting new conversion for ca. 6 ms after changing mux
                ADCSRA |= ( 1 << ADSC ) | ( 1 << ADIF );
                adc_hold = 0;
            } else
                adc_hold++;
        } else {
            if ( selected_adc_channel == AL_adc ){
                selected_adc_channel = AH_adc;
                ADC_low_value -= ADC_low_buffer[adc_read_count];
                ADC_low_buffer[adc_read_count] = ADC;
                ADC_low_value += ADC_low_buffer[adc_read_count];
            } else {
                selected_adc_channel = AL_adc;
                ADC_high_value -= ADC_high_buffer[adc_read_count];
                ADC_high_buffer[adc_read_count] = ADC;
                ADC_high_value += ADC_high_buffer[adc_read_count];
                adc_read_count++;
                if ( adc_read_count > 9){
                    adc_read_count = 0;
                }                
            }
            ADMUX = ( 1 << REFS0) | ( selected_adc_channel & 0x0F );        // switch ADC mux to other channel
            adc_hold = 1;       // hold conversion for stable readings
        }
    }
    if ( int_cnt > 200 ){
        int_cnt = 0;
        unsigned char message[15];

        USART_text("\e[2J");
        USART_text("\e[H");

        sprintf(message, "AL = 0x%x\n\r", ADC_high_value / 10);
        USART_text(message);        

        sprintf(message, "AH = 0x%x\n\r", ADC_low_value / 10);
        USART_text(message);

        unsigned char time[24];

        sprintf(time, "\nRuntime: %d:%d:%d", hour, min, sec);
        USART_text(time);
    }
    if ( second_counter == 488 ){
        second_counter = 0;
        sec++;
        if ( sec > 59 ) {
            sec = 0;
            min++;
            if ( min > 59 ) {
                min = 0;
                hour++;
                if ( hour > 23 )
                    hour = 0;
            }
        }
    }
    int_cnt++;
    second_counter++;
}

ISR(BADISR_vect){}

void USART_Init( unsigned int ubrr){
    UBRR0H = (unsigned char)(ubrr>>8);
    UBRR0L = (unsigned char)ubrr;
    UCSR0A = (1<<U2X0);
    UCSR0B = (1<<RXEN0)|(1<<TXEN0);
    UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}

void USART_Transmit( unsigned char data ){
    while ( !( UCSR0A & (1<<UDRE0)) );
    UDR0 = data;
}

unsigned char USART_Receive( void ){
    while ( !(UCSR0A & (1<<RXC0)) );
    return UDR0;
}

void USART_Flush( void ){
    unsigned char dummy;
    while ( UCSR0A & (1<<RXC0) ) dummy = UDR0;
}

void USART_text(unsigned char* text){
    while(*text){
        USART_Transmit((unsigned char) *text++);
    }
}

void setup(void){
    USART_Init(13);
    DDRB |= ( 1 << PB4 );
    PORTB |= ( 1 << PB4 );
    DDRC = ( 0 << PC2 ) | ( 0 << PC3 ) | ( 0 << PC3 );

    DIDR0 = ( 1 << ADC4D ) | ( 1 << ADC3D ) | ( 1 << ADC2D);

    ADMUX = ( 1 << REFS0) | (selected_adc_channel & 0x0F);
    ADCSRA = ( 1 << ADEN ) | ( 1 << ADSC ) | ( 1 << ADPS1 ) | ( 1 << ADPS0 );

    TCCR0B = ( 1 << CS01 );     // Setup Timer0 to overflow mode, interrupt on every 2.048 ms
    TIMSK0 = ( 1 << TOIE0 );
    sei();
}

int main(void){
    setup();
    // start first ADC conversion

    for(;;){
        _delay_ms(500);
        PORTB ^= ( 1 << PB4 );
    }
}

/*
typedef struct IO_B {
	unsigned int LCD_RS:1;				//PB0
	unsigned int LCD_EN:1;				//PB1
	unsigned int B2:1;		            //PB2
	unsigned int B3:1;			        //PB3
	unsigned int B4:1;		            //PB4
	unsigned int B5:1;			        //PB5
	unsigned int B6:1;			        //PB6
	unsigned int B7:1;			        //PB7
} IO_B;

#define _PORTB	(*( volatile IO_B*)&PORTB)

typedef struct IO_D {
	unsigned int D0:1;				//PD0
	unsigned int D1:1;				//PD1
	unsigned int LCD_D4:1;		    //PD2
	unsigned int LCD_D5:1;			//PD3
	unsigned int LCD_D6:1;		    //PD4
	unsigned int LCD_D7:1;			//PD5
	unsigned int D6:1;			    //PD6
	unsigned int D7:1;			    //PD7
} IO_D;

#define _PORTD	(*( volatile IO_D*)&PORTD)
*/