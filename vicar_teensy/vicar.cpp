#include <LUFA/Drivers/USB/USB.h>
#include <LUFA/Drivers/Board/LEDs.h>
#include <LUFA/Drivers/Peripheral/Serial.h>
#include <Usb.h>
#include "vicar.h"
#include "descriptors.h"

uint8_t deviceType = DEVICE_TYPE_DEFAULT;
uint8_t deviceConfigurations = CUSTOM_CONFIGS_DEFAULT;

USB Usb; //this is the MAX3421E interface

void _HandleUSBEvent(uint8_t type, void* data);
void SendReceive_Task(void);
void MassStorage_Task(void);

void SetupTimer0(void)
{
  sei();

    // on the ATmega168, timer 0 is also used for fast hardware pwm
    // (using phase-correct PWM would mean that timer 0 overflowed half as often
    // resulting in different millis() behavior on the ATmega8 and ATmega168)
#if defined(TCCR0A) && defined(WGM01)
    sbi(TCCR0A, WGM01);
    sbi(TCCR0A, WGM00);
#endif

    // set timer 0 prescale factor to 64
#if defined(__AVR_ATmega128__)
    // CPU specific: different values for the ATmega128
    sbi(TCCR0, CS02);
#elif defined(TCCR0) && defined(CS01) && defined(CS00)
    // this combination is for the standard atmega8
    sbi(TCCR0, CS01);
    sbi(TCCR0, CS00);
#elif defined(TCCR0B) && defined(CS01) && defined(CS00)
    // this combination is for the standard 168/328/1280/2560
    sbi(TCCR0B, CS01);
    sbi(TCCR0B, CS00);
#elif defined(TCCR0A) && defined(CS01) && defined(CS00)
    // this combination is for the __AVR_ATmega645__ series
    sbi(TCCR0A, CS01);
    sbi(TCCR0A, CS00);
#else
    #error Timer 0 prescale factor 64 not set correctly
#endif

    // enable timer 0 overflow interrupt
#if defined(TIMSK) && defined(TOIE0)
    sbi(TIMSK, TOIE0);
#elif defined(TIMSK0) && defined(TOIE0)
    sbi(TIMSK0, TOIE0);
#else
    #error  Timer 0 overflow interrupt not set correctly
#endif
}

void eventCallback(uint8_t type, void* data)
{
    _HandleUSBEvent(type, data);
}

void SetupHardware(void)
{
    /* Disable watchdog if enabled by bootloader/fuses */
    MCUSR &= ~(1 << WDRF);
    wdt_disable();

    /* Disable clock division */
    clock_prescale_set(clock_div_1);

    SetupTimer0();

    /* Hardware Initialization */
    LEDs_Init();
    LEDs_SetAllLEDs(LEDS_NO_LEDS);

    /* USB Initialization */
    USB_Init();
    Usb.powerOn();
    Usb.setEventCallback(eventCallback);

    //TODO: Not sure I need this...
    _delay_ms(1000);
    sei();
}

int main(void)
{
    SetupHardware();

    while (true)
    {
        USB_USBTask();
        Usb.Task();

        //Process URBs only if we're not in HID mode
        //In HID mode, we only process URBs when the host requests variable data (via control request)
        if (deviceType != DEVICE_TYPE_HID)
        {
            Usb.processUrbs(eventCallback);
            MassStorage_Task();
        }
        
        SendReceive_Task();
    }
}
