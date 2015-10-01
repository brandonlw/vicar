/* Copyright 2009-2011 Oleg Mazurov, Circuits At Home, http://www.circuitsathome.com */
/* Additions/changes by Brandon Wilson */

#include "Max3421e.h"
#include <LUFA/Drivers/Board/LEDs.h>

static uint8_t vbusState;

MAX3421E::MAX3421E()
{
    setupNewDevices = 0;

    spi_init();
    pinMode( MAX_INT, INPUT);
    pinMode( MAX_GPX, INPUT );

    //activate pullups on INT and GPX
    digitalWrite(MAX_INT, HIGH);
    digitalWrite(MAX_GPX, HIGH);

    pinMode(MAX_SS, OUTPUT);
    digitalWrite(MAX_SS,HIGH);   
    
    //release MAX3421E from reset
    pinMode(MAX_RESET, OUTPUT);
    digitalWrite(MAX_RESET, HIGH);
}

uint8_t MAX3421E::getVbusState(void)
{ 
    return vbusState;
}

/* Single host register write   */
void MAX3421E::regWr( uint8_t reg, uint8_t val)
{
      digitalWrite(MAX_SS,LOW);
      //SPDR = ( reg | 0x02 );
      //while(!( SPSR & ( 1 << SPIF )));
      SPI_SendByte(reg | 0x02);
      //SPDR = val;
      //while(!( SPSR & ( 1 << SPIF )));
      SPI_SendByte(val);
      digitalWrite(MAX_SS,HIGH);
      return;
}

/* multiple-byte write */
/* returns a pointer to a memory position after last written */
char* MAX3421E::bytesWr(uint8_t reg, uint8_t nbytes, char* data)
{
    digitalWrite(MAX_SS,LOW);
    SPI_SendByte(reg | 0x02);
    while (nbytes--)
    {
        //send next data byte
        SPI_SendByte(*data);
        data++;
    }

    digitalWrite(MAX_SS,HIGH);
    return data;
}

/* GPIO write. GPIO byte is split between 2 registers, so two writes are needed to write one byte */
/* GPOUT bits are in the low nibble. 0-3 in IOPINS1, 4-7 in IOPINS2 */
/* upper 4 bits of IOPINS1, IOPINS2 are read-only, so no masking is necessary */
void MAX3421E::gpioWr(uint8_t val)
{
    regWr(rIOPINS1, val);
    val = val >> 4;
    regWr(rIOPINS2, val);
}

/* Single host register read        */
uint8_t MAX3421E::regRd(uint8_t reg)    
{
    uint8_t tmp;
    digitalWrite(MAX_SS,LOW);
    SPI_SendByte(reg);
    tmp = SPI_ReceiveByte();
    digitalWrite(MAX_SS,HIGH); 

    return tmp;
}

/* multiple-bytes register read                             */
/* returns a pointer to a memory position after last read   */
char* MAX3421E::bytesRd(uint8_t reg, uint8_t nbytes, char* data)
{
    digitalWrite(MAX_SS,LOW);
    SPI_SendByte(reg);
    while( nbytes )
    {
        *data = SPI_ReceiveByte();
        data++;
        nbytes--;
    }
    
    digitalWrite(MAX_SS,HIGH);
    return data;
}

/* GPIO read. See gpioWr for explanation */
/* GPIN pins are in high nibbles of IOPINS1, IOPINS2    */
uint8_t MAX3421E::gpioRd(void)
{
    uint8_t tmpbyte = 0;
    tmpbyte = regRd( rIOPINS2 );            //pins 4-7
    tmpbyte &= 0xf0;                        //clean lower nibble
    tmpbyte |= ( regRd( rIOPINS1 ) >>4 ) ;  //shift low bits and OR with upper from previous operation. Upper nibble zeroes during shift, at least with this compiler
    
    return tmpbyte;
}

/* reset MAX3421E using chip reset bit. SPI configuration is not affected   */
uint8_t MAX3421E::reset()
{
    int tmp = 0;
    regWr(rUSBCTL, bmCHIPRES);                        //Chip reset. This stops the oscillator
    regWr(rUSBCTL, 0x00);                             //Remove the reset
    while(!(regRd(rUSBIRQ) & bmOSCOKIRQ))
    {
        //wait until the PLL is stable
        tmp++;                                          //timeout after 65536 attempts
        if (tmp == 0)
        {
            return false;
        }
    }
 
    return true;
}

/* probe bus to determine device presense and speed and switch host to this speed */
void MAX3421E::busprobe(void)
{
    uint8_t bus_sample;
    bus_sample = regRd(rHRSL);              //Get J,K status
    bus_sample &= (bmJSTATUS|bmKSTATUS);    //zero the rest of the byte

    //start full-speed or low-speed host 
    vbusState = 0x00;
    switch (bus_sample)
    {
        case bmJSTATUS:
        {
            if (setupNewDevices)
            {
                if ((regRd(rMODE) & bmLOWSPEED) == 0)
                {
                    regWr(rMODE, MODE_FS_HOST);       //start full-speed host
                    vbusState = FSHOST;
                }
                else
                {
                    regWr(rMODE, MODE_LS_HOST);       //start low-speed host
                    vbusState = LSHOST;
                }
            }

            break;
        }

        case bmKSTATUS:
        {
            if (setupNewDevices)
            {
                if ((regRd(rMODE) & bmLOWSPEED) == 0)
                {
                    regWr(rMODE, MODE_LS_HOST);       //start low-speed host
                    vbusState = LSHOST;
                }
                else
                {
                    regWr(rMODE, MODE_FS_HOST);       //start full-speed host
                    vbusState = FSHOST;
                }
            }

            break;
        }

        case bmSE1: //illegal state
        {
            vbusState = SE1;
            break;
        }

        case bmSE0: //disconnected state
        {
            regWr(rMODE, bmDPPULLDN|bmDMPULLDN|bmHOST|bmSEPIRQ);
            vbusState = SE0;
            break;
        }
    }
}

/* MAX3421E initialization after power-on */
void MAX3421E::powerOn()
{
    /* Configure full-duplex SPI, interrupt pulse   */
    regWr( rPINCTL,( bmFDUPSPI + bmINTLEVEL + bmGPXB ));        //Full-duplex SPI, level interrupt, GPX
    
    //stop/start the oscillator
    reset();

    /* configure host operation */
    regWr(rMODE, bmDPPULLDN|bmDMPULLDN|bmHOST|bmSEPIRQ);        // set pull-downs, Host, Separate GPIN IRQ on GPX
    regWr(rHIEN, bmCONDETIE|bmFRAMEIE);                         //connection detection
 
    /* check if device is connected */
    regWr(rHCTL, bmSAMPLEBUS);                                 // sample USB bus
    while(!(regRd( rHCTL ) & bmSAMPLEBUS ));                   //wait for sample operation to finish
    busprobe();                                                //check if anything is connected
    regWr(rHIRQ, bmCONDETIRQ);                                 //clear connection detect interrupt                 
    regWr(rCPUCTL, 0x01);                                      //enable interrupt pin
}

/* MAX3421 state change task and interrupt handler */
uint8_t MAX3421E::Task(void)
{
    uint8_t rcode = 0;
    uint8_t pinvalue;
    pinvalue = digitalRead(MAX_INT);    
    if(pinvalue == LOW)
    {
        rcode = IntHandler();
    }

    pinvalue = digitalRead(MAX_GPX);
    if(pinvalue == LOW)
    {
        GpxHandler();
    }

    return rcode;
}   

uint8_t MAX3421E::IntHandler()
{
    uint8_t HIRQ;
    uint8_t HIRQ_sendback = 0x00;
    HIRQ = regRd( rHIRQ ); //determine interrupt source
    if (HIRQ & bmCONDETIRQ)
    {
        busprobe();
        HIRQ_sendback |= bmCONDETIRQ;
    }

    /* End HIRQ interrupts handling, clear serviced IRQs */
    regWr(rHIRQ, HIRQ_sendback);
    return HIRQ_sendback;
}

uint8_t MAX3421E::GpxHandler()
{
    uint8_t GPINIRQ = regRd(rGPINIRQ); //read GPIN IRQ register
    return( GPINIRQ );
}
