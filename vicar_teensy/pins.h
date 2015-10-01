#ifndef _MYPINS_H_
#define _MYPINS_H_

#include "vicar.h"

#define CORE_NUM_TOTAL_PINS	25
#define CORE_NUM_DIGITAL	13
#define CORE_NUM_ANALOG		12
#define CORE_NUM_PWM		7
#define CORE_NUM_INTERRUPT	4
#define PIN_B0		0
#define PIN_B1		1
#define PIN_B2		2
#define PIN_B3		3
#define PIN_B7		4
#define PIN_D0		5
#define PIN_D1		6
#define PIN_D2		7
#define PIN_D3		8
#define PIN_C6		9
#define PIN_C7		10
#define PIN_D6		11
#define PIN_D7		12
#define PIN_B4		13
#define PIN_B5		14
#define PIN_B6		15
#define PIN_F7		16
#define PIN_F6		17
#define PIN_F5		18
#define PIN_F4		19
#define PIN_F1		20
#define PIN_F0		21
#define PIN_D4		22
#define PIN_D5		23
#define PIN_E6		24
#define CORE_PIN0_BIT		0
#define CORE_PIN1_BIT		1
#define CORE_PIN2_BIT		2
#define CORE_PIN3_BIT		3
#define CORE_PIN4_BIT		7
#define CORE_PIN5_BIT		0
#define CORE_PIN6_BIT		1
#define CORE_PIN7_BIT		2
#define CORE_PIN8_BIT		3
#define CORE_PIN9_BIT		6
#define CORE_PIN10_BIT		7
#define CORE_PIN11_BIT		6
#define CORE_PIN12_BIT		7
#define CORE_PIN13_BIT		4
#define CORE_PIN14_BIT		5
#define CORE_PIN15_BIT		6
#define CORE_PIN16_BIT		7
#define CORE_PIN17_BIT		6
#define CORE_PIN18_BIT		5
#define CORE_PIN19_BIT		4
#define CORE_PIN20_BIT		1
#define CORE_PIN21_BIT		0
#define CORE_PIN22_BIT		4
#define CORE_PIN23_BIT		5
#define CORE_PIN24_BIT		6
#define CORE_PIN0_BITMASK	_BV(CORE_PIN0_BIT)
#define CORE_PIN1_BITMASK	_BV(CORE_PIN1_BIT)
#define CORE_PIN2_BITMASK	_BV(CORE_PIN2_BIT)
#define CORE_PIN3_BITMASK	_BV(CORE_PIN3_BIT)
#define CORE_PIN4_BITMASK	_BV(CORE_PIN4_BIT)
#define CORE_PIN5_BITMASK	_BV(CORE_PIN5_BIT)
#define CORE_PIN6_BITMASK	_BV(CORE_PIN6_BIT)
#define CORE_PIN7_BITMASK	_BV(CORE_PIN7_BIT)
#define CORE_PIN8_BITMASK	_BV(CORE_PIN8_BIT)
#define CORE_PIN9_BITMASK	_BV(CORE_PIN9_BIT)
#define CORE_PIN10_BITMASK	_BV(CORE_PIN10_BIT)
#define CORE_PIN11_BITMASK	_BV(CORE_PIN11_BIT)
#define CORE_PIN12_BITMASK	_BV(CORE_PIN12_BIT)
#define CORE_PIN13_BITMASK	_BV(CORE_PIN13_BIT)
#define CORE_PIN14_BITMASK	_BV(CORE_PIN14_BIT)
#define CORE_PIN15_BITMASK	_BV(CORE_PIN15_BIT)
#define CORE_PIN16_BITMASK	_BV(CORE_PIN16_BIT)
#define CORE_PIN17_BITMASK	_BV(CORE_PIN17_BIT)
#define CORE_PIN18_BITMASK	_BV(CORE_PIN18_BIT)
#define CORE_PIN19_BITMASK	_BV(CORE_PIN19_BIT)
#define CORE_PIN20_BITMASK	_BV(CORE_PIN20_BIT)
#define CORE_PIN21_BITMASK	_BV(CORE_PIN21_BIT)
#define CORE_PIN22_BITMASK	_BV(CORE_PIN22_BIT)
#define CORE_PIN23_BITMASK	_BV(CORE_PIN23_BIT)
#define CORE_PIN24_BITMASK	_BV(CORE_PIN24_BIT)
#define CORE_PIN0_PORTREG	PORTB
#define CORE_PIN1_PORTREG	PORTB
#define CORE_PIN2_PORTREG	PORTB
#define CORE_PIN3_PORTREG	PORTB
#define CORE_PIN4_PORTREG	PORTB
#define CORE_PIN5_PORTREG	PORTD
#define CORE_PIN6_PORTREG	PORTD
#define CORE_PIN7_PORTREG	PORTD
#define CORE_PIN8_PORTREG	PORTD
#define CORE_PIN9_PORTREG	PORTC
#define CORE_PIN10_PORTREG	PORTC
#define CORE_PIN11_PORTREG	PORTD
#define CORE_PIN12_PORTREG	PORTD
#define CORE_PIN13_PORTREG	PORTB
#define CORE_PIN14_PORTREG	PORTB
#define CORE_PIN15_PORTREG	PORTB
#define CORE_PIN16_PORTREG	PORTF
#define CORE_PIN17_PORTREG	PORTF
#define CORE_PIN18_PORTREG	PORTF
#define CORE_PIN19_PORTREG	PORTF
#define CORE_PIN20_PORTREG	PORTF
#define CORE_PIN21_PORTREG	PORTF
#define CORE_PIN22_PORTREG	PORTD
#define CORE_PIN23_PORTREG	PORTD
#define CORE_PIN24_PORTREG	PORTE
#define CORE_PIN0_DDRREG	DDRB
#define CORE_PIN1_DDRREG	DDRB
#define CORE_PIN2_DDRREG	DDRB
#define CORE_PIN3_DDRREG	DDRB
#define CORE_PIN4_DDRREG	DDRB
#define CORE_PIN5_DDRREG	DDRD
#define CORE_PIN6_DDRREG	DDRD
#define CORE_PIN7_DDRREG	DDRD
#define CORE_PIN8_DDRREG	DDRD
#define CORE_PIN9_DDRREG	DDRC
#define CORE_PIN10_DDRREG	DDRC
#define CORE_PIN11_DDRREG	DDRD
#define CORE_PIN12_DDRREG	DDRD
#define CORE_PIN13_DDRREG	DDRB
#define CORE_PIN14_DDRREG	DDRB
#define CORE_PIN15_DDRREG	DDRB
#define CORE_PIN16_DDRREG	DDRF
#define CORE_PIN17_DDRREG	DDRF
#define CORE_PIN18_DDRREG	DDRF
#define CORE_PIN19_DDRREG	DDRF
#define CORE_PIN20_DDRREG	DDRF
#define CORE_PIN21_DDRREG	DDRF
#define CORE_PIN22_DDRREG	DDRD
#define CORE_PIN23_DDRREG	DDRD
#define CORE_PIN24_DDRREG	DDRE
#define CORE_PIN0_PINREG	PINB
#define CORE_PIN1_PINREG	PINB
#define CORE_PIN2_PINREG	PINB
#define CORE_PIN3_PINREG	PINB
#define CORE_PIN4_PINREG	PINB
#define CORE_PIN5_PINREG	PIND
#define CORE_PIN6_PINREG	PIND
#define CORE_PIN7_PINREG	PIND
#define CORE_PIN8_PINREG	PIND
#define CORE_PIN9_PINREG	PINC
#define CORE_PIN10_PINREG	PINC
#define CORE_PIN11_PINREG	PIND
#define CORE_PIN12_PINREG	PIND
#define CORE_PIN13_PINREG	PINB
#define CORE_PIN14_PINREG	PINB
#define CORE_PIN15_PINREG	PINB
#define CORE_PIN16_PINREG	PINF
#define CORE_PIN17_PINREG	PINF
#define CORE_PIN18_PINREG	PINF
#define CORE_PIN19_PINREG	PINF
#define CORE_PIN20_PINREG	PINF
#define CORE_PIN21_PINREG	PINF
#define CORE_PIN22_PINREG	PIND
#define CORE_PIN23_PINREG	PIND
#define CORE_PIN24_PINREG	PINE
#define CORE_ADC0_PIN		PIN_F0
#define CORE_ADC1_PIN		PIN_F1
#define CORE_ADC4_PIN		PIN_F4
#define CORE_ADC5_PIN		PIN_F5
#define CORE_ADC6_PIN		PIN_F6
#define CORE_ADC7_PIN		PIN_F7
#define CORE_ADC8_PIN		PIN_D4
#define CORE_ADC9_PIN		PIN_D6
#define CORE_ADC10_PIN		PIN_D7
#define CORE_ADC11_PIN		PIN_B4
#define CORE_ADC12_PIN		PIN_B5
#define CORE_ADC13_PIN		PIN_B6
#define CORE_RXD1_PIN           PIN_D2
#define CORE_TXD1_PIN           PIN_D3
#define CORE_XCK1_PIN           PIN_D5
#define CORE_SDA0_PIN           PIN_D1
#define CORE_SCL0_PIN           PIN_D0
#define CORE_INT0_PIN           PIN_D0
#define CORE_INT1_PIN           PIN_D1
#define CORE_INT2_PIN           PIN_D2
#define CORE_INT3_PIN           PIN_D3
#define CORE_SS0_PIN            PIN_B0
#define CORE_MOSI0_PIN          PIN_B2
#define CORE_MISO0_PIN          PIN_B3
#define CORE_SCLK0_PIN          PIN_B1
#define CORE_T0_PIN             PIN_D7
#define CORE_T1_PIN             PIN_D6
#define CORE_ICP1_PIN           PIN_D4
#define CORE_ICP3_PIN           PIN_C7
#define CORE_OC0A_PIN           PIN_B7
#define CORE_OC0B_PIN           PIN_D0
#define CORE_OC1A_PIN           PIN_B5
#define CORE_OC1B_PIN           PIN_B6
#define CORE_OC1C_PIN           PIN_B7
#define CORE_OC3A_PIN           PIN_C6
#define CORE_OC4A_PIN           PIN_C7
#define CORE_OC4AN_PIN          PIN_C6
#define CORE_OC4B_PIN           PIN_B6
#define CORE_OC4BN_PIN          PIN_B5
#define CORE_OC4D_PIN           PIN_D7
#define CORE_OC4DN_PIN          PIN_D6
#define CORE_PCINT0_PIN		PIN_B0
#define CORE_PCINT1_PIN		PIN_B1
#define CORE_PCINT2_PIN		PIN_B2
#define CORE_PCINT3_PIN		PIN_B3
#define CORE_PCINT4_PIN		PIN_B4
#define CORE_PCINT5_PIN		PIN_B5
#define CORE_PCINT6_PIN		PIN_B6
#define CORE_PCINT7_PIN		PIN_B7
#define CORE_LED0_PIN		PIN_D6
#define CORE_PWM0_PIN		CORE_OC1C_PIN	// B7  4
#define CORE_PWM1_PIN		CORE_OC0B_PIN	// D0  5
#define CORE_PWM2_PIN		CORE_OC3A_PIN	// C6  9
#define CORE_PWM3_PIN		CORE_OC4A_PIN	// C7  10
#define CORE_PWM4_PIN		CORE_OC4D_PIN	// D7  12
#define CORE_PWM5_PIN		CORE_OC1A_PIN	// B5  14
#define CORE_PWM6_PIN		CORE_OC1B_PIN	// B6  15
#define CORE_ANALOG0_PIN	PIN_F0	// 21  ADC0
#define CORE_ANALOG1_PIN	PIN_F1	// 20  ADC1
#define CORE_ANALOG2_PIN	PIN_F4	// 19  ADC4
#define CORE_ANALOG3_PIN	PIN_F5	// 18  ADC5
#define CORE_ANALOG4_PIN	PIN_F6	// 17  ADC6
#define CORE_ANALOG5_PIN	PIN_F7	// 16  ADC7
#define CORE_ANALOG6_PIN	PIN_B6	// 15  ADC13
#define CORE_ANALOG7_PIN	PIN_B5	// 14  ADC12
#define CORE_ANALOG8_PIN	PIN_B4	// 13  ADC11
#define CORE_ANALOG9_PIN	PIN_D7	// 12  ADC10
#define CORE_ANALOG10_PIN	PIN_D6  // 11  ADC9
#define CORE_ANALOG11_PIN	PIN_D4	// 22  ADC8

extern const uint8_t PROGMEM digital_pin_table_PGM[];

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif 
#define HIGH 0x1
#define LOW  0x0

#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2

#define NOT_A_PIN 127

#define NOT_ON_TIMER 0
static inline uint8_t digitalPinToTimer(uint8_t) __attribute__((always_inline, unused));
static inline uint8_t digitalPinToTimer(uint8_t pin)
{
	switch (pin) {
	#ifdef CORE_PWM0_PIN
	case CORE_PWM0_PIN: return 1;
	#endif
	#ifdef CORE_PWM1_PIN
	case CORE_PWM1_PIN: return 2;
	#endif
	#ifdef CORE_PWM2_PIN
	case CORE_PWM2_PIN: return 3;
	#endif
	#ifdef CORE_PWM3_PIN
	case CORE_PWM3_PIN: return 4;
	#endif
	#ifdef CORE_PWM4_PIN
	case CORE_PWM4_PIN: return 5;
	#endif
	#ifdef CORE_PWM5_PIN
	case CORE_PWM5_PIN: return 6;
	#endif
	#ifdef CORE_PWM6_PIN
	case CORE_PWM6_PIN: return 7;
	#endif
	#ifdef CORE_PWM7_PIN
	case CORE_PWM7_PIN: return 8;
	#endif
	#ifdef CORE_PWM8_PIN
	case CORE_PWM8_PIN: return 9;
	#endif
	default: return NOT_ON_TIMER;
	}
}

#define TIMER0A 1
#define TIMER0B 2
#define TIMER1A 3
#define TIMER1B 4
#define TIMER1C 5
#define TIMER2  6
#define TIMER2A 7
#define TIMER2B 8

#define TIMER3A 9
#define TIMER3B 10
#define TIMER3C 11
#define TIMER4A 12
#define TIMER4B 13
#define TIMER4C 14
#define TIMER4D 15
#define TIMER5A 16
#define TIMER5B 17
#define TIMER5C 18

#define digitalPinToPort(P) (P)
#define portInputRegister(P) ((volatile uint8_t *)((int)pgm_read_byte(digital_pin_table_PGM+(P)*2+1)))
#define portModeRegister(P) (portInputRegister(P) + 1)
#define portOutputRegister(P) (portInputRegister(P) + 2)
#define digitalPinToBitMask(P) (pgm_read_byte(digital_pin_table_PGM+(P)*2))

void pinMode(uint8_t pin, uint8_t mode);

// Forcing this inline keeps the callers from having to push their own stuff
// on the stack. It is a good performance win and only takes 1 more byte per
// user than calling. (It will take more bytes on the 168.)
//
// But shouldn't this be moved into pinMode? Seems silly to check and do on
// each digitalread or write.
//
// Mark Sproul:
// - Removed inline. Save 170 bytes on atmega1280
// - changed to a switch statment; added 32 bytes but much easier to read and maintain.
// - Added more #ifdefs, now compiles for atmega645
//
//static inline void turnOffPWM(uint8_t timer) __attribute__ ((always_inline));
//static inline void turnOffPWM(uint8_t timer)
static void turnOffPWM(uint8_t timer)
{
	switch (timer)
	{
		#if defined(TCCR1A) && defined(COM1A1)
		case TIMER1A:   cbi(TCCR1A, COM1A1);    break;
		#endif
		#if defined(TCCR1A) && defined(COM1B1)
		case TIMER1B:   cbi(TCCR1A, COM1B1);    break;
		#endif
		#if defined(TCCR1A) && defined(COM1C1)
		case TIMER1C:   cbi(TCCR1A, COM1C1);    break;
		#endif
		
		#if defined(TCCR2) && defined(COM21)
		case  TIMER2:   cbi(TCCR2, COM21);      break;
		#endif
		
		#if defined(TCCR0A) && defined(COM0A1)
		case  TIMER0A:  cbi(TCCR0A, COM0A1);    break;
		#endif
		
		#if defined(TIMER0B) && defined(COM0B1)
		case  TIMER0B:  cbi(TCCR0A, COM0B1);    break;
		#endif
		#if defined(TCCR2A) && defined(COM2A1)
		case  TIMER2A:  cbi(TCCR2A, COM2A1);    break;
		#endif
		#if defined(TCCR2A) && defined(COM2B1)
		case  TIMER2B:  cbi(TCCR2A, COM2B1);    break;
		#endif
		
		#if defined(TCCR3A) && defined(COM3A1)
		case  TIMER3A:  cbi(TCCR3A, COM3A1);    break;
		#endif
		#if defined(TCCR3A) && defined(COM3B1)
		case  TIMER3B:  cbi(TCCR3A, COM3B1);    break;
		#endif
		#if defined(TCCR3A) && defined(COM3C1)
		case  TIMER3C:  cbi(TCCR3A, COM3C1);    break;
		#endif

		#if defined(TCCR4A) && defined(COM4A1)
		case  TIMER4A:  cbi(TCCR4A, COM4A1);    break;
		#endif					
		#if defined(TCCR4A) && defined(COM4B1)
		case  TIMER4B:  cbi(TCCR4A, COM4B1);    break;
		#endif
		#if defined(TCCR4A) && defined(COM4C1)
		case  TIMER4C:  cbi(TCCR4A, COM4C1);    break;
		#endif			
		#if defined(TCCR4C) && defined(COM4D1)
		case TIMER4D:	cbi(TCCR4C, COM4D1);	break;
		#endif			
			
		#if defined(TCCR5A)
		case  TIMER5A:  cbi(TCCR5A, COM5A1);    break;
		case  TIMER5B:  cbi(TCCR5A, COM5B1);    break;
		case  TIMER5C:  cbi(TCCR5A, COM5C1);    break;
		#endif
	}
}

unsigned long millis();
//static uint32_t millis(void) __attribute__((always_inline, unused));
/*static uint32_t millis(void) __attribute__((unused));
static uint32_t millis(void)
{
	uint32_t out;
	asm volatile(
		"in	__tmp_reg__, __SREG__"		"\n\t"
		"cli"					"\n\t"
		"lds	%A0, timer0_millis_count"	"\n\t"
		"lds	%B0, timer0_millis_count+1"	"\n\t"
		"lds	%C0, timer0_millis_count+2"	"\n\t"
		"lds	%D0, timer0_millis_count+3"	"\n\t"
		"out	__SREG__, __tmp_reg__"
		: "=r" (out) : : "r0"
	);
	return out;
}*/

int digitalRead(uint8_t pin);
void digitalWrite(uint8_t pin, uint8_t val);

#if F_CPU == 16000000L
  #define TIMER0_MILLIS_INC  	1
  #define TIMER0_FRACT_INC	3
  #define TIMER0_MICROS_INC  	4
#elif F_CPU == 8000000L
  #define TIMER0_MILLIS_INC  	2
  #define TIMER0_FRACT_INC	6
  #define TIMER0_MICROS_INC  	8
#elif F_CPU == 4000000L
  #define TIMER0_MILLIS_INC  	4
  #define TIMER0_FRACT_INC	12
  #define TIMER0_MICROS_INC  	16
#elif F_CPU == 2000000L
  #define TIMER0_MILLIS_INC  	8
  #define TIMER0_FRACT_INC	24
  #define TIMER0_MICROS_INC  	32
#elif F_CPU == 1000000L
  #define TIMER0_MILLIS_INC  	16
  #define TIMER0_FRACT_INC	48
  #define TIMER0_MICROS_INC  	64
#endif

#endif //_MYPINS_H_
