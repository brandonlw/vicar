#include <LUFA/Drivers/USB/USB.h>
#include <LUFA/Drivers/Board/LEDs.h>
#include <LUFA/Drivers/Peripheral/Serial.h>
#include "vicar.h"

extern uint8_t packetBuffer[HID_PACKET_SIZE];

char lastCBWSignature[4];
uint8_t failedMsdCommand = 0;
uint8_t msdRetCode;
uint8_t forwardingEnabled = 0;
uint8_t forwardingAddress;
uint8_t forwardingEndpointIn;
uint8_t forwardingEndpointOut;

uint8_t LUFA_Send(uint8_t ep, uint8_t* data, int length);
uint8_t LUFA_Receive(uint8_t ep, uint8_t* data, int length);

void _SendCBW(void)
{
    memset(packetBuffer+8, 0, 5);
    memcpy(packetBuffer+4, lastCBWSignature, 4);
    packetBuffer[0] = 'U';
    packetBuffer[1] = 'S';
    packetBuffer[2] = 'B';
    packetBuffer[3] = 'S';
    packetBuffer[12] = msdRetCode;
    LUFA_Send(CUSTOM_MSD_INCOMING_ENDPOINT, packetBuffer, 13);
}

void _HandleMassStoragePacket(void)
{
    //Back up the signature, because we're going to need it for the response
    memcpy(lastCBWSignature, packetBuffer+4, 4);
    
    //Handle this mass storage command
    int numToSend = 0;
    uint8_t retCode = 0x00;
    uint8_t doStall = 0;
    msdRetCode = 0;
    switch (packetBuffer[0x0F]) //CBW command
    {
        case 0x12: //inquiry
        {
            //Just indicate that we're present
            numToSend = 36;
            memset(packetBuffer, 0, numToSend);
            packetBuffer[1] = 0x80;
            packetBuffer[3] = 0x00;
            packetBuffer[4] = 0x1F;
            break;
        }
        
        case 0x00:
        {
            //Always freak out on this
            msdRetCode = 0x01;
            break;
        }
        
        case 0x03:
        {
            //Indicate media isn't present
            numToSend = 18;
            memset(packetBuffer, 0, numToSend);
            packetBuffer[0] = 0xF0;
            packetBuffer[2] = 0x02;
            packetBuffer[7] = 0x0A;
            packetBuffer[9] = 0xAA;
            packetBuffer[10] = 0x55;
            packetBuffer[11] = 0x40;
            packetBuffer[12] = 0x3A;
            break;
        }
        
        /*case 0x23:
        {
            numToSend = 12;
            memset(descriptorBuffer, 0, numToSend);
            descriptorBuffer[3] = 0x08; //capaacity list length
            descriptorBuffer[6] = 0x10; //number of blocks (sectors) (2MB)
            descriptorBuffer[8] = 0x01; //reserved/descriptor code (maximum unformatted memory)
            descriptorBuffer[10] = 0x02; //block length (512 bytes/sector)
            break;
        }
        
        case 0x25:
        {
            numToSend = 8;
            memset(descriptorBuffer, 0, numToSend);
            descriptorBuffer[2] = 0x0F; //last logical block address
            descriptorBuffer[3] = 0xFF;
            descriptorBuffer[6] = 0x02; //block length (512 bytes/sector)
            break;
        }*/

        default:
        {
            doStall = 1;
            break;
        }
    }

    if (doStall)
    {
        failedMsdCommand = 1;
        msdRetCode = 0x01;

        Endpoint_SelectEndpoint(CUSTOM_MSD_INCOMING_ENDPOINT);
        Endpoint_StallTransaction();
    }
    else
    {
        //Send back any data
        if (!msdRetCode && numToSend > 0)
        {
            LUFA_Send(CUSTOM_MSD_INCOMING_ENDPOINT, packetBuffer, numToSend);
        }
    }

    _SendCBW();
}

void MassStorage_Task(void)
{
    if (!forwardingEnabled)
    {
        Endpoint_SelectEndpoint(CUSTOM_MSD_INCOMING_ENDPOINT);
        if (!Endpoint_IsStalled())
        {
            if (failedMsdCommand)
            {
                _SendCBW();
            }
            
            failedMsdCommand = 0;
        }
    }
}
