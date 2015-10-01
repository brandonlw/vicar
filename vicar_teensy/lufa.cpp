#include <LUFA/Drivers/USB/USB.h>
#include <LUFA/Drivers/Board/LEDs.h>
#include <LUFA/Drivers/Peripheral/Serial.h>
#include "vicar.h"
#include "descriptors.h"
#include "Usb.h"

extern uint8_t deviceType;
extern uint8_t deviceConfigurations;

extern uint16_t bytesReceived;
extern uint8_t packetBuffer[HID_PACKET_SIZE];
extern uint8_t* packetData;

extern uint8_t forwardingEnabled;
extern uint8_t forwardingAddress;
extern uint8_t forwardingEndpointIn;
extern uint8_t forwardingEndpointOut;

extern USB Usb; //this is the MAX3421E interface

uint8_t descriptorBuffer[DESCRIPTOR_BUFFER_SIZE];
uint8_t configured = 0;
uint8_t nextUrb = 0;

void _HandleVicarPacket(void);
void _SendUrbCompletionEvent(URB* urb);
void _PreparePacketBuffer(uint8_t command, unsigned int length);
void _SendPacket(void);

void _CopyDescriptorToRAM(const uint8_t* data, int size)
{
    for (int i = 0; i < size; i++)
    {
        descriptorBuffer[i] = pgm_read_byte(&(data[i]));
    }
}

uint8_t LUFA_Receive(uint8_t ep, uint8_t* data, int length)
{
    uint8_t errorCode;
    uint16_t count = 0;
    while ((errorCode = Endpoint_Read_Stream_LE((char*)data, length,
        &count)) == ENDPOINT_RWSTREAM_IncompleteTransfer);
}

uint8_t LUFA_SendAndContinue(uint8_t ep, uint8_t* data, int length)
{
    uint8_t ret = ENDPOINT_RWSTREAM_NoError;

    //Select our endpoint
    Endpoint_SelectEndpoint(ep);
    
    //TODO: Is this really necessary?
    //I'm thinking so...
    Endpoint_WaitUntilReady();

    //Send this data over USB
    ret = ENDPOINT_RWSTREAM_NoError;
    if (length > 0)
    {
        ret = Endpoint_Write_Stream_LE(data, length, NULL);
    }
    
    return ret;
}

uint8_t LUFA_Send(uint8_t ep, uint8_t* data, int length)
{
    uint8_t ret = LUFA_SendAndContinue(ep, data, length);
    
    if (ret == ENDPOINT_RWSTREAM_NoError)
    {
        Endpoint_ClearIN();
    }
    
    return ret;
}

uint8_t LUFA_CmdPipeSend(uint8_t* data, int length)
{
    return LUFA_Send(CUSTOM_INCOMING_CMD_ENDPOINT, data, length);
}

uint8_t LUFA_DataPipeSend(uint8_t* data, int length)
{
    return LUFA_Send(CUSTOM_INCOMING_DATA_ENDPOINT, data, length);
}

uint8_t LUFA_Control_Finish_Write()
{
    while (!Endpoint_IsINReady());
    if (Endpoint_BytesInEndpoint() < USB_Device_ControlEndpointSize)
    {
        Endpoint_ClearIN();
    }

    while (!(Endpoint_IsOUTReceived()))
    {
        uint8_t USB_DeviceState_LCL = USB_DeviceState;

        if (USB_DeviceState_LCL == DEVICE_STATE_Unattached)
            return ENDPOINT_RWCSTREAM_DeviceDisconnected;
        else if (USB_DeviceState_LCL == DEVICE_STATE_Suspended)
            return ENDPOINT_RWCSTREAM_BusSuspended;
    }
    
    Endpoint_ClearOUT();
    return ENDPOINT_RWCSTREAM_NoError;
}

uint8_t LUFA_Control_Write(const void* const buffer, uint16_t length)
{
    uint8_t* dataStream = (uint8_t*)buffer;

    if (!length)
    {
        Endpoint_ClearIN();
    }

    while (length)
    {
        uint8_t USB_DeviceState_LCL = USB_DeviceState;

        if (USB_DeviceState_LCL == DEVICE_STATE_Unattached)
            return ENDPOINT_RWCSTREAM_DeviceDisconnected;
        else if (USB_DeviceState_LCL == DEVICE_STATE_Suspended)
            return ENDPOINT_RWCSTREAM_BusSuspended;
        else if (Endpoint_IsSETUPReceived())
            return ENDPOINT_RWCSTREAM_HostAborted;
        else if (Endpoint_IsOUTReceived())
            break;

        if (Endpoint_IsINReady())
        {
            uint16_t bytesInEndpoint = Endpoint_BytesInEndpoint();

            while (length && (bytesInEndpoint < USB_Device_ControlEndpointSize))
            {
                Endpoint_Write_8(*dataStream);
                dataStream++;
                length--;
                bytesInEndpoint++;
            }

            if (bytesInEndpoint >= USB_Device_ControlEndpointSize)
            {
                Endpoint_ClearIN();
            }
        }
    }

    return ENDPOINT_RWCSTREAM_NoError;
}

void EVENT_USB_Device_StartOfFrame(void)
{
    //Do nothing
}

void EVENT_USB_Device_Connect(void)
{
    //Do nothing
}

void EVENT_USB_Device_Disconnect(void)
{
    //Do nothing
}

void EVENT_USB_Device_ConfigurationChanged(void)
{
    configured = 1;
    switch (deviceType)
    {
        case DEVICE_TYPE_HID:
        {
            Endpoint_ConfigureEndpoint(0x80 | HID_INCOMING_ENDPOINT1, 0x03, 0x40, 1);
            Endpoint_ConfigureEndpoint(0x80 | HID_INCOMING_ENDPOINT2, 0x03, 0x40, 1);
            break;
        }

        case DEVICE_TYPE_CUSTOM:
        {
            Endpoint_ConfigureEndpoint(0x80 | CUSTOM_INCOMING_CMD_ENDPOINT, 0x02, 0x40, 1);
            Endpoint_ConfigureEndpoint(0x80 | CUSTOM_INCOMING_DATA_ENDPOINT, 0x02, 0x40, 1);
            Endpoint_ConfigureEndpoint(CUSTOM_OUTGOING_CMD_ENDPOINT, 0x02, 0x40, 1);
            Endpoint_ConfigureEndpoint(0x80 | CUSTOM_MSD_INCOMING_ENDPOINT, 0x02, 0x40, 1);
            Endpoint_ConfigureEndpoint(CUSTOM_MSD_OUTGOING_ENDPOINT, 0x02, 0x40, 1);
            Endpoint_ConfigureEndpoint(0x80 | CUSTOM_HID_INCOMING_ENDPOINT, 0x03, 0x40, 1);
            break;
        }
        
        default:
        {
            //Whoops, nevermind...
            configured = 0;
            break;
        }
    }
}

uint16_t CALLBACK_USB_GetDescriptor(const uint16_t wValue,
                                    const uint8_t wIndex,
                                    const void** const descriptorAddress)
{
    const uint8_t descriptorType = (wValue >> 8);
    const uint8_t descriptorNumber = (wIndex & 0xFF);
    uint16_t size = NO_DESCRIPTOR;
    
    switch (descriptorType)
    {
        case DTYPE_Device:
        {
            switch (deviceType)
            {
                case DEVICE_TYPE_HID:
                {
                    size = sizeof(hidDeviceDescriptor);
                    _CopyDescriptorToRAM(hidDeviceDescriptor, size);
                    break;
                }
                
                case DEVICE_TYPE_CUSTOM:
                {
                    size = sizeof(customDeviceDescriptor);
                    _CopyDescriptorToRAM(customDeviceDescriptor, size);
                    break;
                }

                default:
                {
                    //Uh...
                    break;
                }
            }
            break;
        }

        case DTYPE_Configuration:
        {
            switch (deviceType)
            {
                case DEVICE_TYPE_HID:
                {
                    size = sizeof(hidConfigDescriptor);
                    _CopyDescriptorToRAM(hidConfigDescriptor, size);
                    descriptorBuffer[2] = sizeof(hidConfigDescriptor) & 0xFF;
                    descriptorBuffer[3] = (sizeof(hidConfigDescriptor) >> 8) & 0xFF;
                    break;
                }
                
                case DEVICE_TYPE_CUSTOM:
                {
                    int i = 0;
                    int idx = 0;
                    int numInterfaces = 0;
                    uint8_t copy = 1;
                    while (i < sizeof(customConfigDescriptor))
                    {
                        uint8_t length = pgm_read_byte(&(customConfigDescriptor[i]));
                        uint8_t type = pgm_read_byte(&(customConfigDescriptor[i+1]));

                        if (type == 0x04)
                        {
                            uint8_t inum = pgm_read_byte(&(customConfigDescriptor[i+2]));
                            if ((!(deviceConfigurations & DEVICE_CONFIG_MSD) && inum == 0x00) ||
                             (!(deviceConfigurations & DEVICE_CONFIG_HID) && inum == 0x01))
                            {
                                //Start skipping stuff
                                copy = 0;
                            }
                            else
                            {
                                //We can start copying again
                                copy = 1;
                                numInterfaces++;
                            }
                        }

                        if (copy)
                        {
                            for (int j = 0; j < length; j++)
                            {
                                descriptorBuffer[idx++] = pgm_read_byte(&(customConfigDescriptor[i+j]));
                            }
                        }
                        
                        i += length;
                    }

                    size = idx;
                    descriptorBuffer[4] = numInterfaces; //number of interfaces
                    descriptorBuffer[2] = size & 0xFF;
                    descriptorBuffer[3] = (size >> 8) & 0xFF;
                    break;
                }

                default:
                {
                    //Uh...
                    break;
                }
            }
            break;
        }
        
        case DTYPE_String:
        {
            if (wValue == 0x03EE)
            {
                size = sizeof(customOSDescriptor);
                _CopyDescriptorToRAM(customOSDescriptor, size);
            }
            else if (wValue == 0x0301)
            {
                size = sizeof(nameDescriptor);
                _CopyDescriptorToRAM(nameDescriptor, size);
            }
            else
            {
                size = sizeof(stringDescriptor);
                _CopyDescriptorToRAM(stringDescriptor, size);
            }

            break;
        }

        case 0x22:
        {
            if (deviceType == DEVICE_TYPE_HID)
            {
                if (descriptorNumber == 0)
                {
                    size = sizeof(hidReportDescriptor1);
                    _CopyDescriptorToRAM(hidReportDescriptor1, size);
                }
                else
                {
                    size = sizeof(hidReportDescriptor2);
                    _CopyDescriptorToRAM(hidReportDescriptor2, size);
                }
            }
            else
            {
                size = sizeof(hidReportDescriptorKeyboard);
                _CopyDescriptorToRAM(hidReportDescriptorKeyboard, size);
            }

            break;
        }

        default:
        {
            //Uh...
            break;
        }
    }

    *descriptorAddress = descriptorBuffer;
    return size;
}

void urbCallback(uint8_t type, void* data)
{
    if (type == USB_EVENT_URB_DATA)
    {
        URB_DATA* info = (URB_DATA*)data;
        
        LUFA_Control_Write(info->buffer, info->length);
    }
}

void EVENT_USB_Device_ControlRequest(void)
{
    switch (USB_ControlRequest.bmRequestType)
    {
        case 0x02:
        {
            switch (USB_ControlRequest.bRequest)
            {
                case 0x01: //clear endpoint feature
                {
                    if (forwardingEnabled)
                    {
                        Endpoint_ClearSETUP();
                        Endpoint_ClearStatusStage();
                        Endpoint_ResetEndpoint(CUSTOM_MSD_INCOMING_ENDPOINT);
                        Endpoint_SelectEndpoint(CUSTOM_MSD_INCOMING_ENDPOINT);
                        Endpoint_ClearStall();
                        Endpoint_ResetDataToggle();

                        if (USB_ControlRequest.wIndex == (CUSTOM_MSD_INCOMING_ENDPOINT | 0x80))
                        {
                            //Forward this on to the device
                            unsigned int len = 0;
                            uint8_t res = Usb.ctrlReq(forwardingAddress, 0x02, 0x01, 0, forwardingEndpointIn, &len, NULL);
                            Usb.epInfo[Usb.findEndpoint(forwardingAddress, forwardingEndpointIn)].rcvToggle = bmRCVTOG0;
                        }
                    }

                    break;
                }

                break;
            }

            break;
        }

        case 0xC0:
        {
            switch (USB_ControlRequest.bRequest)
            {
                case CUSTOM_VENDOR_CODE:
                {
                    if (USB_ControlRequest.wIndex == 0x0004)
                    {
                        uint16_t size = sizeof(OSFeatureDescriptor) > USB_ControlRequest.wLength ?
                            USB_ControlRequest.wLength : sizeof(OSFeatureDescriptor);
                        Endpoint_ClearSETUP();
                        Endpoint_Write_Control_PStream_LE(OSFeatureDescriptor, size);
                        Endpoint_ClearIN();
                        Endpoint_ClearStatusStage();
                    }
                    else
                    {
                        Endpoint_StallTransaction();
                    }
                    
                    break;
                }
                
                default:
                {
                    break;
                }
            }

            break;
        }
        
        case 0xC1:
        {
            switch (USB_ControlRequest.bRequest)
            {
                case CUSTOM_VENDOR_CODE:
                {
                    if (USB_ControlRequest.wIndex == 0x0005)
                    {
                        uint16_t size = sizeof(OSExtendedDescriptor) > USB_ControlRequest.wLength ?
                            USB_ControlRequest.wLength : sizeof(OSExtendedDescriptor);
                        Endpoint_ClearSETUP();
                        Endpoint_Write_Control_PStream_LE(OSExtendedDescriptor, size);
                        Endpoint_ClearIN();
                    }
                    else
                    {
                        Endpoint_StallTransaction();
                    }
                    
                    break;
                }
                
                default:
                {
                    break;
                }
            }
            
            break;
        }

        case 0xA1:
        {
            switch (USB_ControlRequest.bRequest)
            {
                case 0xFE: //get max LUN
                {
                    Endpoint_ClearSETUP();
                    Endpoint_Write_8(0);
                    Endpoint_ClearIN();
                    Endpoint_ClearStatusStage();
                    break;
                }

                case 0x01:
                {
                    if (deviceType == DEVICE_TYPE_HID && USB_ControlRequest.wIndex == HID_SECOND_INTERFACE)
                    {
                        Endpoint_ClearSETUP();

                        //Find an URB and process it, sending as much data back as possible
                        URB* urb = NULL;
                        int idx = 0;
                        while (idx < Usb.urbFirstFreeIndex)
                        {
                            idx = nextUrb % Usb.urbFirstFreeIndex;
                            if (!Usb.urbs[idx].complete)
                            {
                                urb = &Usb.urbs[idx];
                                nextUrb = idx++;
                                break;
                            }
                        }

                        if (urb != NULL)
                        {
                            int epIdx = Usb.findEndpoint(urb->addr, urb->ep);
                            if (epIdx != -1)
                            {
                                int transferrable = HID_MAX_CONTROL_SIZE - 7;
                                packetBuffer[0] = HID_REPORT_ID;
                                packetBuffer[1] = urb->id & 0xFF;
                                packetBuffer[2] = (urb->id >> 8) & 0xFF;
                                packetBuffer[3] = (urb->id >> 16) & 0xFF;
                                packetBuffer[4] = (urb->id >> 24) & 0xFF;
                                packetBuffer[5] = urb->addr;
                                packetBuffer[6] = urb->ep;
                                LUFA_Control_Write(packetBuffer, 7);

                                uint8_t res = USB_SUCCESS;
                                int maxPacketSize = Usb.epInfo[epIdx].maxPacketSize;
                                while (!urb->complete && transferrable > 0 && urb->requestedLength > 0)
                                {
                                    //Find out how much to retrieve this time (no more than we can send)
                                    unsigned long remaining = urb->requestedLength;
                                    unsigned long length = maxPacketSize > transferrable ? transferrable : maxPacketSize;

                                    res = Usb.urbInTransfer(urb->id, &length, urbCallback);
                                    urb->result = res;
                                    if (res == USB_SUCCESS)
                                    {
                                        urb->actualLength += length;
                                        urb->requestedLength -= length;
                                        transferrable -= length;
                                        
                                        //If we could've received at least the max packet size but didn't...
                                        if (remaining >= maxPacketSize && length < maxPacketSize)
                                        {
                                            //It's over early, we're done
                                            urb->complete = 1;
                                            break;
                                        }

                                        //If we got it all, then we're definitely done
                                        if (!urb->requestedLength)
                                        {
                                            urb->complete = 1;
                                        }
                                    }
                                    else
                                    {
                                        //We failed, send that result back instead
                                        urb->complete = 1;
                                    }
                                }

                                LUFA_Control_Finish_Write();
                                Endpoint_ClearIN();
                                if (urb->complete)
                                {
                                    //Send completion event
                                    _SendUrbCompletionEvent(urb);
                                    Usb.flushCompletedUrbs();
                                }
                            }
                            else
                            {
                                Endpoint_StallTransaction();
                            }
                        }
                        else
                        {
                            Endpoint_StallTransaction();
                        }
                    }
                    else
                    {
                        //Not valid for anything but HID device's second interface
                        Endpoint_StallTransaction();
                    }

                    break;
                }

                default:
                {
                    break;
                }
            }

            break;
        }
        case 0x21:
        {
            if (deviceType == DEVICE_TYPE_HID)
            {
                switch (USB_ControlRequest.bRequest)
                {
                    case 0x09: //set output report
                    {
                        if (USB_ControlRequest.wIndex == HID_FIRST_INTERFACE)
                        {
                            Endpoint_ClearSETUP();
                            Endpoint_Read_Control_Stream_LE(packetBuffer, HID_PACKET_SIZE);
                            Endpoint_ClearIN();
                            Endpoint_ClearStatusStage();
                            bytesReceived = HID_PACKET_SIZE;
                            
                            _HandleVicarPacket();
                        }
                        else
                        {
                            Endpoint_StallTransaction();
                        }
                        
                        break;
                    }

                    case 0x0A: //set idle
                    {
                        Endpoint_ClearSETUP();
                        Endpoint_ClearStatusStage();
                        break;
                    }

                    default:
                    {
                        break;
                    }
                }
            }
            else
            {
                //Not valid for anything but HID device's first interface
                Endpoint_StallTransaction();
            }

            break;
        }

        default:
        {
            break;
        }
    }
}
