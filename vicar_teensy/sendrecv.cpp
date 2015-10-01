#include <LUFA/Drivers/USB/USB.h>
#include <LUFA/Drivers/Board/LEDs.h>
#include <LUFA/Drivers/Peripheral/Serial.h>
#include "vicar.h"
#include "Usb.h"
#include "pins.h"

extern uint8_t deviceType;
extern uint8_t deviceConfigurations;
extern uint8_t configured;

extern USB Usb; //this is the MAX3421E interface

extern uint8_t forwardingEnabled;
extern uint8_t forwardingAddress;
extern uint8_t forwardingEndpointIn;
extern uint8_t forwardingEndpointOut;

unsigned int sequence = FIRST_SEQUENCE;
uint16_t bytesReceived = 0;
uint8_t packetBuffer[HID_PACKET_SIZE];
uint8_t* packetData = packetBuffer+8;

uint8_t LUFA_SendAndContinue(uint8_t ep, uint8_t* data, int length);
uint8_t LUFA_Send(uint8_t ep, uint8_t* data, int length);
uint8_t LUFA_Receive(uint8_t ep, uint8_t* data, int length);
uint8_t LUFA_DataPipeSend(uint8_t* data, int length);
uint8_t LUFA_CmdPipeSend(uint8_t* data, int length);

void _HandleMassStoragePacket(void);

void _PreparePacketBuffer(uint8_t command, unsigned int length)
{
    packetBuffer[0] = 0x01;
    packetBuffer[1] = command;
    packetBuffer[6] = (length >> 8) & 0xFF;
    packetBuffer[7] = length & 0xFF;
}

void _PrepareEventBuffer(uint8_t command, unsigned int length)
{
    unsigned int seq = sequence;
    if ((sequence + 1) & 0x80000000)
    {
        sequence = 1;
    }
    else
    {
        sequence++;
    }
    
    packetBuffer[2] = ((seq & 0xFF000000) >> 24) & 0xFF;
    packetBuffer[3] = ((seq & 0x00FF0000) >> 16) & 0xFF;
    packetBuffer[4] = (seq >> 8) & 0xFF;
    packetBuffer[5] = seq & 0xFF;

    _PreparePacketBuffer(command, length);
}

void _SendPacket(void)
{
    //Determine size of packet
    uint16_t bytesToSend = 0;
    if (deviceType == DEVICE_TYPE_CUSTOM)
    {
        bytesToSend = (((packetBuffer[6] << 8) & 0xFF00) | packetBuffer[7]) + 7;
    }
    else
    {
        bytesToSend = HID_PACKET_SIZE;
    }

    while (bytesToSend > 0)
    {
        USB_USBTask();
 
        uint8_t errorCode;
        if (deviceType == DEVICE_TYPE_HID)
        {
            errorCode = LUFA_Send(HID_INCOMING_ENDPOINT1, packetBuffer, HID_PACKET_SIZE);
        }
        else
        {
            errorCode = LUFA_CmdPipeSend(packetBuffer+1, bytesToSend);
        }

        if (errorCode == ENDPOINT_RWSTREAM_NoError)
        {
            Endpoint_ClearIN();
            bytesToSend = 0;
        }
    }
}

void _SendUrbCompletionEvent(URB* urb)
{
    _PrepareEventBuffer('B', 9);
    packetData[0] = urb->id & 0xFF;
    packetData[1] = (urb->id >> 8) & 0xFF;
    packetData[2] = (urb->id >> 16) & 0xFF;
    packetData[3] = (urb->id >> 24) & 0xFF;
    packetData[4] = urb->result;
    packetData[5] = urb->actualLength & 0xFF;
    packetData[6] = (urb->actualLength >> 8) & 0xFF;
    packetData[7] = (urb->actualLength >> 16) & 0xFF;
    packetData[8] = (urb->actualLength >> 24) & 0xFF;
    _SendPacket();
}

void _HandleVicarPacket(void)
{
    //Assume we're going to be sending a response
    uint8_t doSend = 1;

    switch (packetBuffer[1])
    {
        case 'N': //enable/disable new device detection
        {
            //HACK: Back up the packet header somewher else, because we're about to trigger events that'll mess with it
            memcpy(packetBuffer+0x40, packetBuffer, 0x40);

            //Only do stuff if we're changing it
            uint8_t setupNew = packetData[0];
            if (setupNew != Usb.setupNewDevices)
            {
                Usb.setupNewDevices = setupNew;
                
                //Reset the chip (and what we believe is our VBUS state)
                Usb.powerOn();
                
                //Reset our state machine
                Usb.setUsbTaskState(USB_DETACHED_SUBSTATE_INITIALIZE);
            }

            //Restore the packet header from backup
            memcpy(packetBuffer, packetBuffer+0x40, 0x40);
 
            //Send back response
            _PreparePacketBuffer('N', 1);
            packetData[0] = USB_SUCCESS;
            break;
        }
        
        case 'D':
        {
            //We're going to handle this ourselves
            uint8_t newConfigs = packetData[0];
            doSend = 0;

            //Send back the status
            _PreparePacketBuffer('D', 1);
            packetData[0] = USB_SUCCESS;
            _SendPacket();

            //Only actually apply the configuration if it's different and appropriate
            if (deviceType == DEVICE_TYPE_CUSTOM && deviceConfigurations != newConfigs)
            {
                //Detach from USB, wait, then re-attach
                USB_Detach();
                int timeout = millis() + 2000;
                while (timeout > millis())
                {
                    USB_USBTask();
                }

                //Set the new device configuration and re-attach
                deviceConfigurations = newConfigs;
                USB_Attach();
            }

            break;
        }

        case 'M': //set operating mode
        {
            //We're going to handle this ourselves
            uint8_t newType = packetData[0];
            doSend = 0;

            //Send back the status
            _PreparePacketBuffer('M', 1);
            packetData[0] = USB_SUCCESS;
            _SendPacket();

            //Detach from USB, wait, then re-attach
            USB_Detach();
            int timeout = millis() + 2000;
            while (timeout > millis())
            {
                USB_USBTask();
            }

            //Set the new device type and re-attach
            deviceType = newType;
            USB_Attach();
            
            break;
        }
        
        case 'G': //get status
        {
            //Send back the status
            _PreparePacketBuffer('G', 3);
            packetData[0] = deviceType;
            packetData[1] = deviceConfigurations;
            packetData[2] = Usb.getUsbTaskState();
            break;
        }
        
        case 'V': //send input data to host
        {
            /*_PreparePacketBuffer('V', 1);
            packetData[0] =*/ LUFA_Send(packetData[0], packetData+3, packetData[1] | ((packetData[2] << 8) & 0xFF00));
            doSend = 0;
            break;
        }
        
        case 'W': //enable mass storage interface packet forwarding
        {
            forwardingEnabled = packetData[0];
            forwardingAddress = packetData[1];
            forwardingEndpointIn = packetData[2];
            forwardingEndpointOut = packetData[3];
            
            _PreparePacketBuffer('W', 1);
            packetData[0] = USB_SUCCESS;
            break;
        }
        
        case 'R': //reset device
        {
            Usb.powerOn();
            Usb.setUsbTaskState(USB_DETACHED_SUBSTATE_INITIALIZE);
            
            //Send back the status (always success)
            _PreparePacketBuffer('R', 1);
            packetData[0] = USB_SUCCESS;
            break;
        }
        
        case 'O': //send endpoint data
        {
            uint8_t res = Usb.outTransfer(packetData[0], packetData[1], packetData[5], (char*)&packetData[6]);

            //Send back the result
            _PreparePacketBuffer('O', 1);
            packetData[0] = res;
            break;
        }

        case 'U': //submit URB
        {
            URB* urb;
            unsigned long length = (packetData[5] | ((packetData[4] << 8) & 0xFF00)) + (packetData[3] |
                ((packetData[2] << 8) & 0xFF00) * 0x10000);

            _PreparePacketBuffer('U', 5);
            packetData[0] = Usb.makeUrb(packetData[0], packetData[1], length, &urb);
            if (packetData[0] == USB_SUCCESS)
            {
                packetData[1] = urb->id & 0xFF;
                packetData[2] = (urb->id >> 8) & 0xFF;
                packetData[3] = (urb->id >> 16) & 0xFF;
                packetData[4] = (urb->id >> 24) & 0xFF;
            }

            break;
        }
        
        case 'L': //cancel URB
        {
            _PreparePacketBuffer('L', 1);
            packetData[0] = Usb.cancelUrb(packetData[0] | ((packetData[1] << 8) & 0xFF00) |
                ((packetData[2] << 16) & 0xFF0000) | ((packetData[3] << 24) & 0xFF000000));
            break;
        }
        
        case 'F': //forget endpoint(s)
        {
            unsigned int length = ((packetBuffer[6] << 8) & 0xFF00) | packetBuffer[5];
            if (length == 0)
            {
                //Forget all of them
                Usb.forgetAllEndpoints();
            }
            else
            {
                //Just forget the one specified
                Usb.forgetEndpoint(packetBuffer[7], packetBuffer[8]);
            }

            _PreparePacketBuffer('F', 1);
            packetData[0] = USB_SUCCESS;
            break;
        }
        
        case 'E': //configure endpoint
        {
            unsigned int wMaxPacketSize = ((packetData[3] << 8) & 0x0000FF00) | (packetData[2] & 0x000000FF);

            _PreparePacketBuffer('E', 1);
            packetData[0] = Usb.configureEndpoint(packetData[0], packetData[1], wMaxPacketSize,
                packetData[4], packetData[5], packetData[6]);
            break;
        }
        
        case 'C':
        {
            //Issue control request
            //TODO: Make this deal with "large" control requests by breaking up setup/data/status stages
            uint8_t deviceToHost = (packetData[1] & 0x80);
            unsigned int length = ((packetData[8] << 8) & 0xFF00) | packetData[7];
            uint8_t ret = Usb.ctrlSetup(packetData[0], packetData[1], packetData[2],
                ((packetData[4] << 8) & 0xFF00) | packetData[3],
                ((packetData[6] << 8) & 0xFF00) | packetData[5], length);
            if (!ret)
            {
                ret = Usb.ctrlData(packetData[0], &length, (char*)(packetData+3), deviceToHost);

                if (!ret)
                {
                    ret = Usb.ctrlStatus(deviceToHost);
                }
            }

            if (ret)
            {
                length = 0;
            }

            //Write the result and actual length transferred
            _PreparePacketBuffer('C', length + 3);
            packetData[0] = ret;
            packetData[1] = length & 0xFF;
            packetData[2] = (length >> 8) & 0xFF;
            break;
        }

        default:
        {
            //Not sending a response after all
            doSend = 0;

            //packetData[0] = packetBuffer[1];
            //_PreparePacketBuffer('X', 1);
            break;
        }
    }

    //Consider this packet handled
    if (doSend)
    {
        _SendPacket();
    }
}

void SendReceive_Task(void)
{
    //Handle anything on the custom command pipe
    Endpoint_SelectEndpoint(CUSTOM_OUTGOING_CMD_ENDPOINT);
    if (Endpoint_IsReadWriteAllowed() && Endpoint_IsOUTReceived())
    {
        //Receive this data into our buffer
        uint8_t errorCode = LUFA_Receive(CUSTOM_OUTGOING_CMD_ENDPOINT, packetBuffer+1, 7);
        if (errorCode == ENDPOINT_RWSTREAM_NoError)
        {
            int numToReceive = ((packetBuffer[6] << 8) & 0xFF00) | packetBuffer[7];
            if (numToReceive > (HID_PACKET_SIZE - 7))
            {
                //If we're receiving a really large packet, only get up to HID_PACKET_SIZE bytes
                //If we can deal with the rest later, then we will
                numToReceive = HID_PACKET_SIZE - 7;
            }

            if (LUFA_Receive(CUSTOM_OUTGOING_CMD_ENDPOINT, packetBuffer+1+7, numToReceive) ==
                ENDPOINT_RWSTREAM_NoError)
            {
                Endpoint_ClearOUT();
                bytesReceived = numToReceive + 7;

                //Handle received packet
                _HandleVicarPacket();
            }
        }
    }

    //Handle anything on the custom mass storage outgoing endpoint
    Endpoint_SelectEndpoint(CUSTOM_MSD_OUTGOING_ENDPOINT);
    if (Endpoint_IsReadWriteAllowed() && Endpoint_IsOUTReceived())
    {
        //Receive this data into our buffer
        uint8_t errorCode;
        uint16_t bytesProcessed = 0;
        Endpoint_SelectEndpoint(CUSTOM_MSD_OUTGOING_ENDPOINT);
        errorCode = Endpoint_Read_Stream_LE((char*)packetBuffer, USB_ENDPOINT_MAX_PACKET_SIZE, &bytesProcessed);
        Endpoint_ClearOUT();
        if (errorCode == ENDPOINT_RWSTREAM_NoError)
        {
            bytesProcessed = USB_ENDPOINT_MAX_PACKET_SIZE;
        }

        //Handle received packet
        if (forwardingEnabled)
        {
            Usb.outTransfer(forwardingAddress, forwardingEndpointOut, bytesProcessed, (char*)packetBuffer);
        }
        else
        {
            _HandleMassStoragePacket();
        }
    }
}

void _HandleUSBEvent(uint8_t type, void* data)
{
    if (configured) //ignore anything before we're able to talk to the host
    {
        switch (type)
        {
            case USB_EVENT_STATE_CHANGED:
            {
                _PrepareEventBuffer('S', 1);
                packetData[0] = Usb.getUsbTaskState();
                _SendPacket();
                break;
            }
            
            case USB_EVENT_INCOMING_DATA:
            {
                INCOMING_INFO* info = (INCOMING_INFO*)data;

                if (forwardingEnabled)
                {
                    if (info->result == hrSTALL)
                    {
                        Endpoint_SelectEndpoint(CUSTOM_MSD_INCOMING_ENDPOINT);
                        Endpoint_StallTransaction();
                    }
                    else if (info->result == USB_SUCCESS)
                    {
                        if (info->length > 0)
                        {
                            Endpoint_SelectEndpoint(CUSTOM_MSD_INCOMING_ENDPOINT);
                            Endpoint_Write_Stream_LE(info->buffer, info->length, NULL);
                            Endpoint_ClearIN();
                        }
                    }
                }
                else
                {
                    packetData[0] = info->addr;
                    packetData[1] = info->ep;
                    packetData[2] = info->result;
                    if (info->result == USB_SUCCESS)
                    {
                        _PrepareEventBuffer('I', 3 + info->length);

                        for (unsigned int j = 0; j < info->length; j++)
                        {
                            packetData[j+3] = info->buffer[j];
                        }
                    }
                    else
                    {
                        _PrepareEventBuffer('I', 3);

                        //Disable the auto-polling, something's wrong
                        Usb.epInfo[Usb.findEndpoint(info->addr, info->ep)].autoPoll = 0;
                    }

                    _SendPacket();
                }

                break;
            }
            
            case USB_EVENT_URB_DATA_START:
            {
                URB_DATA* u = (URB_DATA*)data;

                packetBuffer[0] = u->result;
                packetBuffer[1] = u->id & 0xFF;
                packetBuffer[2] = (u->id >> 8) & 0xFF;
                packetBuffer[3] = (u->id >> 16) & 0xFF;
                packetBuffer[4] = (u->id >> 24) & 0xFF;
                LUFA_SendAndContinue(CUSTOM_INCOMING_DATA_ENDPOINT, packetBuffer, 5);

                break;
            }
            
            case USB_EVENT_URB_DATA:
            {
                URB_DATA* u = (URB_DATA*)data;

                //Send URB data on the data pipe
                LUFA_SendAndContinue(CUSTOM_INCOMING_DATA_ENDPOINT, (uint8_t*)u->buffer, u->length);

                break;
            }
            
            case USB_EVENT_URB_COMPLETE:
            {
                URB* urb = (URB*)data;

                //Send an IN packet with zero bytes to signal that the URB data is over
                packetBuffer[0] = urb->result;
                packetBuffer[1] = urb->actualLength & 0xFF;
                packetBuffer[2] = (urb->actualLength >> 8) & 0xFF;
                packetBuffer[3] = (urb->actualLength >> 16) & 0xFF;
                packetBuffer[4] = (urb->actualLength >> 24) & 0xFF;
                LUFA_Send(CUSTOM_INCOMING_DATA_ENDPOINT, packetBuffer, 5);
                LUFA_Send(CUSTOM_INCOMING_DATA_ENDPOINT, NULL, 0);
                Endpoint_ClearIN();
                
                break;
            }
            
            default:
            {
                break;
            }
        }
    }
}
