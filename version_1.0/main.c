/*
 * MegaPower_program.c
 *
 * Created: 24.06.2020 18:11:56
 * Author : Mateusz Ferenc
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

typedef struct  {
	unsigned int b0:1;				//PB0
	unsigned int b1:1;				//PB1
	unsigned int TEST_out:1;		//PB2
	unsigned int AC_on:1;			//PB3
	unsigned int ATARI_LED:1;		//PB4
	unsigned int C64_LED:1;			//PB5
	unsigned int TEST_in:1;			//PB6
	unsigned int MOSFET:1;			//PB7
} IO;

#define _PORTB	(*( volatile IO*)&PORTB)

#define m_TEST_out		PB2
#define m_AC_on			PB3
#define m_ATARI_LED		PB4
#define m_C64_LED		PB5
#define m_TEST_in		PB6
#define m_MOSFET		PB7

#define m_TEST_SW		((PIND >> PD2) & 1)
#define m_START_SW		((PIND >> PD3) & 1)

#define m_COMP_STATE	(ACSR >> ACO) & 1

uint8_t test_bnt_cnt = 0, stasto_btn_cnt = 0, MOSFET_on = 0, ERROR_flag = 0, test_btn_state = 0, stasto_btn_state = 0, 
 cmp_state = 1, error_cnt = 0, cmp_cnt = 0, device = 0;// t2_off_cnt = 0, t1_off_cnt = 0, cmp_cnt = 0, cmp_off_t = 0, device = 0;//,
// cmp_last = 0, test_btn_last = 0, stasto_btn_last = 0;


ISR(TIMER1_COMPA_vect){				// Interrupt every 0.02s = 20ms = 50Hz
	if(ERROR_flag == 1){			// On error blink leds
		error_cnt++;
		if(error_cnt == 50){		// Change leds state every 1s
			error_cnt = 0;
			_PORTB.ATARI_LED ^= 1;
			_PORTB.C64_LED ^= 1;
		}
	} else {
		if(error_cnt != 7)	error_cnt++;
		
		cmp_cnt = (m_COMP_STATE)? cmp_cnt + 1 : (cmp_cnt > 0)? cmp_cnt - 1 : cmp_cnt;
		
		if((cmp_cnt == 0) & (error_cnt == 7)){
			cmp_state = 0;
		}
		
		if(cmp_cnt == 6){
			cmp_cnt = 0;
			cmp_state = 1;
		}
		if((cmp_state) & (MOSFET_on)){			//	When mosfet is active but circuit is opened that means load turned off or plugged out
			MOSFET_on = 0;
			device = 0;
			_PORTB.AC_on = 1;
			_PORTB.MOSFET = 0;
		} else if((!cmp_state) & (!MOSFET_on) & (error_cnt >= 6)){	//	If voltage present on comparator and mosfet turned off that means error!
			ERROR_flag = 1;						//	Fatal error!
			_PORTB.AC_on = 1;
			_PORTB.MOSFET = 0;
		}
		
		test_bnt_cnt = (m_TEST_SW)? test_bnt_cnt + 1 : (test_bnt_cnt > 0)? test_bnt_cnt - 1 : test_bnt_cnt;
		stasto_btn_cnt = (m_START_SW)? stasto_btn_cnt + 1 : (stasto_btn_cnt > 0)? stasto_btn_cnt - 1 : stasto_btn_cnt;
	
	
		if(test_bnt_cnt == 0){
			test_btn_state = 0;
		}
		if(stasto_btn_cnt == 0){
			stasto_btn_state = 0;
		}
	
		if(test_bnt_cnt > 5){
			test_bnt_cnt = 0;
			test_btn_state = 1;
		}
		if(stasto_btn_cnt > 5){
			stasto_btn_cnt = 0;
			stasto_btn_state = 1;
		}
	
		if((test_btn_state) & (!MOSFET_on) & (!m_TEST_SW) & (!stasto_btn_state) & (ERROR_flag == 0)){		
			test_btn_state = 0;
			//DDRB |= 0x04;
			_PORTB.MOSFET = 1;					//_PORTB.TEST_out = 0;
			asm volatile ("NOP"::);				// Delay for signal propagation
			asm volatile ("NOP"::);
			asm volatile ("NOP"::);
			asm volatile ("NOP"::);
			asm volatile ("NOP"::);
			if(((PINB >> m_TEST_in) & 1)){
				device = 0x0F;					//device = Atari
				_PORTB.ATARI_LED = 1;
			} else {
				device = 0xF0;					//device = C64
				_PORTB.C64_LED = 1;
			}
			_PORTB.MOSFET = 0;					//_PORTB.TEST_out = 1;				// Pull up error!!	Test whenever device is hooked up and powered when testing
												//DDRB &= 0xFB;
		}
		
		if((!test_btn_state) & (stasto_btn_state) & (!m_START_SW)  & (ERROR_flag == 0)){
			stasto_btn_state = 0;
			if(MOSFET_on){
				device = 0;					// device = none
				_PORTB.ATARI_LED = 0;
				_PORTB.C64_LED = 0;
			}
			if(device == 0x0F){				// Atari connected
				MOSFET_on = 1;
				_PORTB.AC_on = 1;
			} else if(device == 0xF0){		// Commodore 64 connected
				MOSFET_on = 1;
				_PORTB.AC_on = 0;
			} else {						// None device connected
				MOSFET_on = 0;
				_PORTB.AC_on = 1;
			}
			_PORTB.MOSFET = MOSFET_on;
		}
	}
}

ISR(WDT_OVERFLOW_vect){
	WDTCR |= 0x40;
}

void setup(void){
	cli();
	DDRB = (1 << m_MOSFET)|(0 << m_TEST_in)|(1 << m_C64_LED)|(1 << m_ATARI_LED)|(1 << m_AC_on)|(0 << m_TEST_out);	//0xBC
	PORTB = (0 << m_MOSFET)|(1 << m_TEST_in)|(0 << m_C64_LED)|(0 << m_ATARI_LED)|(1 << m_AC_on)|(0 << m_TEST_out);	//0xC
	
	DDRD = 0;
	PORTD = (1 << PD3)|(1 << PD2);	//0xC
	
	TCCR1A = 0;
	TCCR1B = 0x0B;
	TCNT1 = 0;
	OCR1A = 1249;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
	TIMSK = 0x40;
	
	WDTCR = 0x4D;
	sei();
}

int main(void){
    setup();
    while (1){
		asm volatile ("NOP"::);
    }
}

