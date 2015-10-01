#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "pins.h"

const uint8_t PROGMEM digital_pin_table_PGM[] = {
	CORE_PIN0_BITMASK,	(int)&CORE_PIN0_PINREG,
	CORE_PIN1_BITMASK,	(int)&CORE_PIN1_PINREG,
	CORE_PIN2_BITMASK,	(int)&CORE_PIN2_PINREG,
	CORE_PIN3_BITMASK,	(int)&CORE_PIN3_PINREG,
	CORE_PIN4_BITMASK,	(int)&CORE_PIN4_PINREG,
	CORE_PIN5_BITMASK,	(int)&CORE_PIN5_PINREG,
	CORE_PIN6_BITMASK,	(int)&CORE_PIN6_PINREG,
	CORE_PIN7_BITMASK,	(int)&CORE_PIN7_PINREG,
	CORE_PIN8_BITMASK,	(int)&CORE_PIN8_PINREG,
	CORE_PIN9_BITMASK,	(int)&CORE_PIN9_PINREG,
	CORE_PIN10_BITMASK,	(int)&CORE_PIN10_PINREG,
	CORE_PIN11_BITMASK,	(int)&CORE_PIN11_PINREG,
	CORE_PIN12_BITMASK,	(int)&CORE_PIN12_PINREG,
	CORE_PIN13_BITMASK,	(int)&CORE_PIN13_PINREG,
	CORE_PIN14_BITMASK,	(int)&CORE_PIN14_PINREG,
	CORE_PIN15_BITMASK,	(int)&CORE_PIN15_PINREG,
	CORE_PIN16_BITMASK,	(int)&CORE_PIN16_PINREG,
	CORE_PIN17_BITMASK,	(int)&CORE_PIN17_PINREG,
	CORE_PIN18_BITMASK,	(int)&CORE_PIN18_PINREG,
	CORE_PIN19_BITMASK,	(int)&CORE_PIN19_PINREG,
	CORE_PIN20_BITMASK,	(int)&CORE_PIN20_PINREG,
	#if CORE_NUM_TOTAL_PINS > 21
	CORE_PIN21_BITMASK,	(int)&CORE_PIN21_PINREG,
	CORE_PIN22_BITMASK,	(int)&CORE_PIN22_PINREG,
	CORE_PIN23_BITMASK,	(int)&CORE_PIN23_PINREG,
	CORE_PIN24_BITMASK,	(int)&CORE_PIN24_PINREG,
	#endif
	#if CORE_NUM_TOTAL_PINS > 25
	CORE_PIN25_BITMASK,	(int)&CORE_PIN25_PINREG,
	CORE_PIN26_BITMASK,	(int)&CORE_PIN26_PINREG,
	CORE_PIN27_BITMASK,	(int)&CORE_PIN27_PINREG,
	CORE_PIN28_BITMASK,	(int)&CORE_PIN28_PINREG,
	CORE_PIN29_BITMASK,	(int)&CORE_PIN29_PINREG,
	CORE_PIN30_BITMASK,	(int)&CORE_PIN30_PINREG,
	CORE_PIN31_BITMASK,	(int)&CORE_PIN31_PINREG,
	CORE_PIN32_BITMASK,	(int)&CORE_PIN32_PINREG,
	CORE_PIN33_BITMASK,	(int)&CORE_PIN33_PINREG,
	CORE_PIN34_BITMASK,	(int)&CORE_PIN34_PINREG,
	CORE_PIN35_BITMASK,	(int)&CORE_PIN35_PINREG,
	CORE_PIN36_BITMASK,	(int)&CORE_PIN36_PINREG,
	CORE_PIN37_BITMASK,	(int)&CORE_PIN37_PINREG,
	CORE_PIN38_BITMASK,	(int)&CORE_PIN38_PINREG,
	CORE_PIN39_BITMASK,	(int)&CORE_PIN39_PINREG,
	CORE_PIN40_BITMASK,	(int)&CORE_PIN40_PINREG,
	CORE_PIN41_BITMASK,	(int)&CORE_PIN41_PINREG,
	CORE_PIN42_BITMASK,	(int)&CORE_PIN42_PINREG,
	CORE_PIN43_BITMASK,	(int)&CORE_PIN43_PINREG,
	CORE_PIN44_BITMASK,	(int)&CORE_PIN44_PINREG,
	CORE_PIN45_BITMASK,	(int)&CORE_PIN45_PINREG
        #endif
};

volatile unsigned long timer0_micros_count = 0;
volatile unsigned long timer0_millis_count = 0;
volatile unsigned char timer0_fract_count = 0;

#define LIB8STATIC __attribute__ ((unused)) static

#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )

// the prescaler is set so that timer0 ticks every 64 clock cycles, and the
// the overflow handler is called every 256 ticks.
#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))
typedef union { unsigned long _long; uint8_t raw[4]; } tBytesForLong;
// tBytesForLong FastLED_timer0_overflow_count;
volatile unsigned long FastLED_timer0_overflow_count=0;
volatile unsigned long FastLED_timer0_millis = 0;

LIB8STATIC void  __attribute__((always_inline)) fastinc32 (volatile uint32_t & _long) {
  uint8_t b = ++((tBytesForLong&)_long).raw[0];
  if(!b) {
    b = ++((tBytesForLong&)_long).raw[1];
    if(!b) {
      b = ++((tBytesForLong&)_long).raw[2];
      if(!b) {
        ++((tBytesForLong&)_long).raw[3];
      }
    }
  }
}

#if defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
ISR(TIM0_OVF_vect)
#else
ISR(TIMER0_OVF_vect)
#endif
{
  fastinc32(FastLED_timer0_overflow_count);
  // FastLED_timer0_overflow_count++;
}

// there are 1024 microseconds per overflow counter tick.
unsigned long millis()
{
        unsigned long m;
        uint8_t oldSREG = SREG;

        // disable interrupts while we read FastLED_timer0_millis or we might get an
        // inconsistent value (e.g. in the middle of a write to FastLED_timer0_millis)
        cli();
        m = FastLED_timer0_overflow_count;  //._long;
        SREG = oldSREG;

        return (m*(MICROSECONDS_PER_TIMER0_OVERFLOW/8))/(1000/8);
}

unsigned long micros() {
        unsigned long m;
        uint8_t oldSREG = SREG, t;

        cli();
        m = FastLED_timer0_overflow_count; // ._long;
#if defined(TCNT0)
        t = TCNT0;
#elif defined(TCNT0L)
        t = TCNT0L;
#else
        #error TIMER 0 not defined
#endif


#ifdef TIFR0
        if ((TIFR0 & _BV(TOV0)) && (t < 255))
                m++;
#else
        if ((TIFR & _BV(TOV0)) && (t < 255))
                m++;
#endif

        SREG = oldSREG;

        return ((m << 8) + t) * (64 / clockCyclesPerMicrosecond());
}

void delay(unsigned long ms)
{
        uint16_t start = (uint16_t)micros();

        while (ms > 0) {
                if (((uint16_t)micros() - start) >= 1000) {
                        ms--;
                        start += 1000;
                }
        }
}

/*ISR(TIMER0_COMPA_vect)//ISR(TIMER0_OVF_vect,ISR_NAKED)
{
	timer0_millis_count++;
	asm volatile(
		"push	r24"				"\n\t"
		"in	r24, __SREG__"			"\n\t"
		"push	r24"				"\n\t"

		"lds	r24, timer0_fract_count"	"\n\t"
		"subi	r24, 256 - %0"			"\n\t"
		"cpi	r24, 125"			"\n\t"
		"brsh	L_%=_fract_roll"		"\n\t"

	"L_%=_fract_noroll:"				"\n\t"
		"sts	timer0_fract_count, r24"	"\n\t"
		"lds	r24, timer0_millis_count"	"\n\t"
		"subi	r24, 256 - %1"			"\n\t"
		"sts	timer0_millis_count, r24"	"\n\t"
		"brcs	L_%=_ovcount"			"\n\t"

	"L_%=_millis_inc_sext:"
		"lds	r24, timer0_millis_count+1"	"\n\t"
		"sbci	r24, 255"			"\n\t"
		"sts	timer0_millis_count+1, r24"	"\n\t"
		"brcs	L_%=_ovcount"			"\n\t"
		"lds	r24, timer0_millis_count+2"	"\n\t"
		"sbci	r24, 255"			"\n\t"
		"sts	timer0_millis_count+2, r24"	"\n\t"
		"brcs	L_%=_ovcount"			"\n\t"
		"lds	r24, timer0_millis_count+3"	"\n\t"
		"sbci	r24, 255"			"\n\t"
		"sts	timer0_millis_count+3, r24"	"\n\t"
		"rjmp	L_%=_ovcount"			"\n\t"

	"L_%=_fract_roll:"				"\n\t"
		"subi	r24, 125"			"\n\t"
		"sts	timer0_fract_count, r24"	"\n\t"
		"lds	r24, timer0_millis_count"	"\n\t"
		"subi	r24, 256 - %1 - 1"		"\n\t"
		"sts	timer0_millis_count, r24"	"\n\t"
		"brcc	L_%=_millis_inc_sext"		"\n\t"

	"L_%=_ovcount:"
		"lds	r24, timer0_micros_count"	"\n\t"
		"subi	r24, 256 - %2"			"\n\t"
		"sts	timer0_micros_count, r24"	"\n\t"
		"brcs	L_%=_end"			"\n\t"
		"lds	r24, timer0_micros_count+1"	"\n\t"
		"sbci	r24, 255"			"\n\t"
		"sts	timer0_micros_count+1, r24"	"\n\t"
		"brcs	L_%=_end"			"\n\t"
		"lds	r24, timer0_micros_count+2"	"\n\t"
		"sbci	r24, 255"			"\n\t"
		"sts	timer0_micros_count+2, r24"	"\n\t"

	"L_%=_end:"
		"pop	r24"				"\n\t"
		"out	__SREG__, r24"			"\n\t"
		"pop	r24"				"\n\t"
		"reti"
		: 
		: "M" (TIMER0_FRACT_INC), "M" (TIMER0_MILLIS_INC),
		  "M" (TIMER0_MICROS_INC)
	);
}*/

int digitalRead(uint8_t pin)
{
	uint8_t timer = digitalPinToTimer(pin);
	uint8_t bit = digitalPinToBitMask(pin);
	uint8_t port = digitalPinToPort(pin);

	if (port == NOT_A_PIN) return LOW;

	// If the pin that support PWM output, we need to turn it off
	// before getting a digital reading.
	if (timer != NOT_ON_TIMER) turnOffPWM(timer);

	if (*portInputRegister(port) & bit) return HIGH;
	return LOW;
}

void digitalWrite(uint8_t pin, uint8_t val)
{
	uint8_t timer = digitalPinToTimer(pin);
	uint8_t bit = digitalPinToBitMask(pin);
	uint8_t port = digitalPinToPort(pin);
	volatile uint8_t *out;

	if (port == NOT_A_PIN) return;

	// If the pin that support PWM output, we need to turn it off
	// before doing a digital write.
	if (timer != NOT_ON_TIMER) turnOffPWM(timer);

	out = portOutputRegister(port);

	uint8_t oldSREG = SREG;
	cli();

	if (val == LOW) {
		*out &= ~bit;
	} else {
		*out |= bit;
	}

	SREG = oldSREG;
}

void pinMode(uint8_t pin, uint8_t mode)
{
	uint8_t bit = digitalPinToBitMask(pin);
	uint8_t port = digitalPinToPort(pin);
	volatile uint8_t *reg, *out;

	if (port == NOT_A_PIN) return;

	// JWS: can I let the optimizer do this?
	reg = portModeRegister(port);
	out = portOutputRegister(port);

	if (mode == INPUT) { 
		uint8_t oldSREG = SREG;
                cli();
		*reg &= ~bit;
		*out &= ~bit;
		SREG = oldSREG;
	} else if (mode == INPUT_PULLUP) {
		uint8_t oldSREG = SREG;
                cli();
		*reg &= ~bit;
		*out |= bit;
		SREG = oldSREG;
	} else {
		uint8_t oldSREG = SREG;
                cli();
		*reg |= bit;
		SREG = oldSREG;
	}
}
