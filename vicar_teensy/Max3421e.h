/* Copyright 2009-2011 Oleg Mazurov, Circuits At Home, http://www.circuitsathome.com */
/* Additions/changes by Brandon Wilson */

#ifndef _MAX3421E_H_
#define _MAX3421E_H_

#include "vicar.h"
#include <LUFA/Drivers/Peripheral/SPI.h>
#include "pins.h"
#include "Max3421e_constants.h"

class MAX3421E
{
    public:
        uint8_t setupNewDevices;

        MAX3421E( void );
        uint8_t getVbusState( void );
        static void regWr( uint8_t, uint8_t );
        char * bytesWr( uint8_t, uint8_t, char * );
        static void gpioWr( uint8_t );
        uint8_t regRd( uint8_t );
        char * bytesRd( uint8_t, uint8_t, char * );
        uint8_t gpioRd( void );
        uint8_t reset();
        uint8_t vbusPwr ( uint8_t );
        void busprobe( void );
        void powerOn();
        uint8_t IntHandler();
        uint8_t GpxHandler();
        uint8_t Task();
    private:
		static void spi_init()
		{
			SPI_Init(SPI_MODE_MASTER | SPI_SPEED_FCPU_DIV_2);
		}
};

#endif //_MAX3421E_H_
