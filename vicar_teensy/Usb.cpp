/* Copyright 2009-2011 Oleg Mazurov, Circuits At Home, http://www.circuitsathome.com */
/* Additions/changes by Brandon Wilson */

#include "Usb.h"
#include <LUFA/Drivers/Board/LEDs.h>

uint8_t last_addr = 0;
uint32_t next_urb_id = 1;

void USB::_set_addr(uint8_t addr)
{
    if (last_addr != addr)
    {
        regWr(rPERADDR, addr);
        last_addr = addr;
    }
}

uint32_t _GetNextUrbId()
{
    uint32_t ret = next_urb_id;
    
    if (!next_urb_id)
    {
        next_urb_id = 1;
    }
    else
    {
        next_urb_id++;
    }
    
    return ret;
}

/* constructor */
USB::USB()
{
    eventCallback = NULL;
    init();
    setUsbTaskState(USB_DETACHED_SUBSTATE_INITIALIZE); //set up state machine
}

uint8_t USB::makeUrb(uint8_t addr, uint8_t ep, unsigned long requestedLength, URB** urb)
{
    //Allocate URB for this request
    uint16_t idx = urbFirstFreeIndex;
    if (idx == USB_MAX_URBS)
    {
        return USB_ERROR_URB_OUT_OF_SPACE;
    }
    
    urbs[idx].id = _GetNextUrbId();
    urbs[idx].addr = addr;
    urbs[idx].ep = ep;
    urbs[idx].requestedLength = requestedLength;
    urbs[idx].complete = 0;
    urbFirstFreeIndex++;
    if (urb != NULL)
    {
        *urb = &urbs[idx];
    }
    
    return USB_SUCCESS;
}

void USB::completeUrb(uint16_t idx)
{
    memcpy((void*)&urbs[idx], (void*)&urbs[idx+1], (USB_MAX_URBS - (idx + 1)) * sizeof(URB));
    urbFirstFreeIndex--;
}

URB* USB::findUrb(uint32_t id)
{
    URB* ret = NULL;
    
    for (int i = 0; i < urbFirstFreeIndex; i++)
    {
        if (urbs[i].id == id)
        {
            ret = &urbs[i];
            break;
        }
    }
    
    return ret;
}

void USB::processUrb(URB* urb, event_callback callback)
{
    unsigned long len = urb->requestedLength;
    uint8_t res = urbInTransfer(urb->id, &len, callback, 1);
    urb->actualLength += len;

    if (res != USB_ERROR_TRANSFER_TIMED_OUT && res != hrNAK)
    {
        //The URB is complete
        urb->complete = 1;
        urb->result = res;
        callback(USB_EVENT_URB_COMPLETE, urb);
    }
}

void USB::flushCompletedUrbs(void)
{
    //Clear out all the completed URBs
    int i = 0;
    while (i < urbFirstFreeIndex)
    {
        if (urbs[i].complete)
        {
            completeUrb(i);
        }
        else
        {
            i++;
        }
    }
}

uint8_t USB::cancelUrb(uint32_t id)
{
    uint8_t ret = USB_ERROR_URB_NOT_FOUND;
    
    for (int i = 0; i < urbFirstFreeIndex; i++)
    {
        if (urbs[i].id == id)
        {
            urbs[i].complete = 1;
            ret = USB_SUCCESS;
            break;
        }
    }
    
    flushCompletedUrbs();
    
    return ret;
}

void USB::processUrbs(event_callback callback)
{
    for (int i = 0; i < urbFirstFreeIndex; i++)
    {
        if (!urbs[i].complete && (urbs[i].ep & 0x80) && (urbs[i].ep & 0x7F) > 0)
        {
            processUrb(&urbs[i], callback);
        }
    }
    
    flushCompletedUrbs();
}

void USB::init(void)
{
    urbFirstFreeIndex = 0;
    
    forgetAllEndpoints();
}

uint8_t USB::getUsbTaskState(void)
{
    return usb_task_state;
}

void USB::setUsbTaskState(uint8_t state)
{
    uint8_t old = usb_task_state;
    usb_task_state = state;
    if (state != old && eventCallback != NULL)
    {
        eventCallback(USB_EVENT_STATE_CHANGED, NULL);
    }
}

void USB::setEventCallback(event_callback callback)
{
    eventCallback = callback;
}

int USB::findEndpoint(uint8_t addr, uint8_t ep)
{
    int ret = -1;
    
    for (uint16_t i = 0; i < epFirstFreeIndex; i++)
    {
        if (epInfo[i].addr == addr && epInfo[i].ep == ep)
        {
            ret = i;
            break;
        }
    }
    
    return ret;
}

uint8_t USB::configureEndpoint(uint8_t addr, uint8_t ep, uint8_t autoPoll)
{
    int idx = findEndpoint(addr, ep);
    if (idx == -1) return USB_ERROR_ENDPOINT_NOT_CONFIGURED;
    
    epInfo[idx].autoPoll = autoPoll;
}

uint8_t USB::configureEndpoint(uint8_t addr, uint8_t ep, unsigned int maxPacketSize, uint8_t sndToggle, uint8_t rcvToggle, uint8_t autoPoll)
{
    ENDPOINT_INFO* info;
    uint8_t ret = USB_SUCCESS;

    //See if this endpoint already exists
    int idx = findEndpoint(addr, ep);
    if (idx != -1)
    {
        //It does, so update its information
        info = &epInfo[idx];
        info->maxPacketSize = maxPacketSize;
        info->sndToggle = sndToggle;
        info->rcvToggle = rcvToggle;
        info->autoPoll = autoPoll;
    }
    else
    {
        //It does not, so allocate one and save it
        if (epFirstFreeIndex < USB_MAX_CONFIGURED_ENDPOINTS)
        {
            info = &epInfo[epFirstFreeIndex];
            info->addr = addr;
            info->ep = ep;
            info->maxPacketSize = maxPacketSize;
            info->sndToggle = sndToggle;
            info->rcvToggle = rcvToggle;
            info->autoPoll = autoPoll;
            epFirstFreeIndex++;
        }
        else
        {
            ret = USB_ERROR_CONFIGURE_ENDPOINT_OUT_OF_SPACE;
        }
    }
    
    return ret;
}

void USB::forgetEndpoint(uint8_t addr, uint8_t ep)
{
    //Find this endpoint in the list
    int idx = findEndpoint(addr, ep);
    if (idx != -1)
    {
        //Copy everything past it back on top of it
        int len = sizeof(ENDPOINT_INFO) * ((USB_MAX_CONFIGURED_ENDPOINTS - 1) - idx);
        char* dst = (char*)&epInfo[idx];
        char* src = (char*)&epInfo[idx+1];
        for (int i = 0; i < len; i++)
        {
            dst[i] = src[i];
        }
        
        //Decrement the last index
        epFirstFreeIndex--;
    }
}

void USB::forgetAllEndpoints()
{
    epFirstFreeIndex = 0;
}

uint8_t USB::ctrlReq(uint8_t addr, uint8_t bmReqType, uint8_t bRequest, unsigned int wValue, unsigned int wIndex,
    unsigned int* nbytes, char* dataptr, unsigned int nak_limit)
{
    int idx = findEndpoint(addr, 0);
    if (idx == -1) return USB_ERROR_ENDPOINT_NOT_CONFIGURED; //oops
    ENDPOINT_INFO* info = &epInfo[idx];
    
    return ctrlReq(addr, bmReqType, bRequest, wValue, wIndex, nbytes, dataptr, info->maxPacketSize, &(info->sndToggle), &(info->rcvToggle), nak_limit);
}

uint8_t USB::ctrlReq(uint8_t addr, uint8_t bmReqType, uint8_t bRequest, unsigned int wValue, unsigned int wIndex,
    unsigned int* nbytes, char* dataptr, unsigned int wMaxPacketSize, uint8_t* sndToggle, uint8_t* rcvToggle, unsigned int nak_limit)
{
    //Request direction, IN or OUT
    //This is hard-coded logic for now -- needs reworking if we want to do weird stuff
    uint8_t deviceToHost = (bmReqType & 0x80);
    uint8_t ret;

    //Setup packet
    if ((ret = ctrlSetup(addr, bmReqType, bRequest, wValue, wIndex, *nbytes, nak_limit)))
    {
        return ret;
    }

    //Data stage
    if ((ret = ctrlData(addr, nbytes, dataptr, deviceToHost, wMaxPacketSize, sndToggle, rcvToggle, nak_limit)))
    {
        return ret;
    }

    //Status stage
    return ctrlStatus(deviceToHost);
}

uint8_t USB::ctrlSetup(uint8_t addr, uint8_t bmReqType, uint8_t bRequest,
    unsigned int wValue, unsigned int wIndex, unsigned int nbytes, unsigned int nak_limit)
{
    //set peripheral address
    _set_addr(addr);
    
    /* fill in setup packet */
    char setup[8];
    setup[0] = bmReqType;
    setup[1] = bRequest;
    setup[2] = wValue & 0xFF;
    setup[3] = (wValue >> 8) & 0xFF;
    setup[4] = wIndex & 0xFF;
    setup[5] = (wIndex >> 8) & 0xFF;
    setup[6] = nbytes & 0xFF;
    setup[7] = (nbytes >> 8) & 0xFF;
    bytesWr(rSUDFIFO, 8, setup);  //transfer to setup packet FIFO
    
    //dispatch packet
    return dispatchPkt(tokSETUP, 0, nak_limit);
}

uint8_t USB::ctrlData(uint8_t addr, unsigned int* nbytes, char* dataptr, uint8_t deviceToHost, unsigned int nak_limit)
{
    int idx = findEndpoint(addr, 0);
    if (idx == -1) return USB_ERROR_ENDPOINT_NOT_CONFIGURED; //oops
    ENDPOINT_INFO* info = &epInfo[idx];

    return ctrlData(addr, nbytes, dataptr, deviceToHost, info->maxPacketSize, &(info->sndToggle), &(info->rcvToggle), nak_limit);
}

/* Control transfer with data stage. Stages 2 and 3 of control transfer. Assumes setup packet has been sent */
uint8_t USB::ctrlData(uint8_t addr, unsigned int* nbytes, char* dataptr, uint8_t deviceToHost, unsigned int wMaxPacketSize,
    uint8_t* sndToggle, uint8_t* rcvToggle, unsigned int nak_limit)
{
    if ((dataptr != NULL) && (*nbytes > 0))
    {
        if (deviceToHost)
        {
            //IN transfer
            *rcvToggle = bmRCVTOG1;
            return inTransfer(addr, 0, nbytes, dataptr, wMaxPacketSize, rcvToggle, nak_limit);
        }
        else
        {
            //OUT transfer
            *sndToggle = bmSNDTOG1;
            return outTransfer(addr, 0, *nbytes, dataptr, wMaxPacketSize, sndToggle, nak_limit);
        }
    }
    else
    {
        return USB_SUCCESS;
    }
}

/* Control transfer with status stage and no data stage */
/* Assumes peripheral address is already set */
uint8_t USB::ctrlStatus(uint8_t deviceToHost, unsigned int nak_limit)
{
    uint8_t ret;
    
    if (deviceToHost)
    {
        ret = dispatchPkt(tokOUTHS, 0, nak_limit);
    }
    else
    {
        ret = dispatchPkt(tokINHS, 0, nak_limit);
    }

    return ret;
}

uint8_t USB::inTransfer(uint8_t addr, uint8_t ep, unsigned int* nbytes, char* data, unsigned int nak_limit)
{
    //Find our endpoint information
    int idx = findEndpoint(addr, ep);
    if (idx == -1) return USB_ERROR_ENDPOINT_NOT_CONFIGURED; //oops
    ENDPOINT_INFO* info = &epInfo[idx];

    return inTransfer(info->addr, info->ep, nbytes, data, info->maxPacketSize, &(info->rcvToggle), nak_limit);
}

/* IN transfer to arbitrary endpoint. Handles multiple packets if necessary. Transfers 'nbytes' bytes. */
/* Keeps sending INs and writes data to memory area pointed by 'data'. */
/* Returns 0 if no errors, 0x01-0x0F is relayed from dispatchPkt(). */
uint8_t USB::inTransfer(uint8_t addr, uint8_t ep, unsigned int* nbytes, char* data, unsigned int wMaxPacketSize, uint8_t* rcvToggle, unsigned int nak_limit)
{
    uint8_t ret;
    uint8_t pktsize;
    unsigned int xfrlen = 0;

    _set_addr(addr); //set peripheral address
    regWr(rHCTL, *rcvToggle); //set toggle value
    while (1)
    {
        //IN packet to EP-'endpoint'. Function takes care of NAKs.
        ret = dispatchPkt(tokIN, ep, nak_limit);
        if (ret)
        {
            //should be 0, indicating ACK. Else return error code.
            return ret;
        }

        /* check for RCVDAVIRQ and generate error if not present */ 
        /* the only case when absense of RCVDAVIRQ makes sense is when toggle error occurred. Need to add handling for that */
        if ((regRd(rHIRQ) & bmRCVDAVIRQ) == 0)
        {
            //receive error
            return USB_ERROR_RCVDAV_IRQ;
        }

        pktsize = regRd(rRCVBC); //number of received bytes
        data = bytesRd(rRCVFIFO, pktsize, data);

        // Clear the IRQ & free the buffer
        regWr(rHIRQ, bmRCVDAVIRQ);
        // add this packet's byte count to total transfer length
        xfrlen += pktsize;
        
        /* The transfer is complete under two conditions:           */
        /* 1. The device sent a short packet (L.T. maxPacketSize)   */
        /* 2. 'nbytes' have been transferred.                       */
        // have we transferred 'nbytes' bytes?
        if ((pktsize < wMaxPacketSize) || (xfrlen >= *nbytes))
        {
            //save toggle value
            if (regRd(rHRSL) & bmRCVTOGRD)
            {
                *rcvToggle = bmRCVTOG1;
            }
            else
            {
                *rcvToggle = bmRCVTOG0;
            }

            *nbytes = xfrlen;
            return USB_SUCCESS;
        }
    }
}

/* IN transfer to arbitrary endpoint. Handles multiple packets if necessary. Transfers 'nbytes' bytes. */
/* Keeps sending INs and writes data to memory area pointed by 'data'. */
/* Returns 0 if no errors, 0x01-0x0F is relayed from dispatchPkt(). */
uint8_t USB::urbInTransfer(uint32_t id, unsigned long* nbytes, event_callback callback, unsigned int nak_limit)
{
    URB_DATA udata;
    uint8_t ret;
    uint8_t pktsize;
    unsigned int xfrlen = 0;
    uint8_t started = 0;
    
    URB* urb = findUrb(id);
    if (urb == NULL)
    {
        return USB_ERROR_URB_NOT_FOUND;
    }
    
    int epIdx = findEndpoint(urb->addr, urb->ep);
    if (epIdx == -1)
    {
        return USB_ERROR_ENDPOINT_NOT_CONFIGURED;
    }

    udata.id = id;
    udata.result = USB_SUCCESS;

    _set_addr(epInfo[epIdx].addr); //set peripheral address
    regWr(rHCTL, epInfo[epIdx].rcvToggle); //set toggle value
    while (1)
    {
        //IN packet to EP-'endpoint'. Function takes care of NAKs.
        if (ret = dispatchPkt(tokIN, epInfo[epIdx].ep, nak_limit))
        {
            //should be 0, indicating ACK. Else return error code.
            return ret;
        }

        /* check for RCVDAVIRQ and generate error if not present */ 
        /* the only case when absense of RCVDAVIRQ makes sense is when toggle error occurred. Need to add handling for that */
        if ((regRd(rHIRQ) & bmRCVDAVIRQ) == 0)
        {
            //receive error
            return USB_ERROR_RCVDAV_IRQ;
        }

        if (!started)
        {
            callback(USB_EVENT_URB_DATA_START, (void*)&udata);
            started = 1;
        }

        pktsize = regRd(rRCVBC); //number of received bytes
        udata.length = pktsize;
        bytesRd(rRCVFIFO, pktsize, udata.buffer);

        // Clear the IRQ & free the buffer
        regWr(rHIRQ, bmRCVDAVIRQ);
        // add this packet's byte count to total transfer length
        xfrlen += pktsize;

        //Call the callback routine with this data
        callback(USB_EVENT_URB_DATA, (void*)&udata);
 
        /* The transfer is complete under two conditions:           */
        /* 1. The device sent a short packet (L.T. maxPacketSize)   */
        /* 2. 'nbytes' have been transferred.                       */
        // have we transferred 'nbytes' bytes?
        if ((pktsize < epInfo[epIdx].maxPacketSize) || (xfrlen >= *nbytes))
        {
            //save toggle value
            if (regRd(rHRSL) & bmRCVTOGRD)
            {
                epInfo[epIdx].rcvToggle = bmRCVTOG1;
            }
            else
            {
                epInfo[epIdx].rcvToggle = bmRCVTOG0;
            }

            *nbytes = xfrlen;
            return USB_SUCCESS;
        }
    }
}

uint8_t USB::outTransfer(uint8_t addr, uint8_t ep, unsigned int nbytes, char* data, unsigned int nak_limit)
{
    //Find our endpoint information
    int idx = findEndpoint(addr, ep);
    if (idx == -1) return USB_ERROR_ENDPOINT_NOT_CONFIGURED; //oops
    ENDPOINT_INFO* info = &epInfo[idx];

    return outTransfer(info->addr, info->ep, nbytes, data, info->maxPacketSize, &(info->sndToggle), nak_limit);
}

/* OUT transfer to arbitrary endpoint. Handles multiple packets if necessary. Transfers 'nbytes' bytes. */
/* Handles NAK bug per Maxim Application Note 4000 for single buffer transfer   */
/* Returns 0 if no errors, 0x01-0x0F is relayed from HRSL                       */
/* major part of this function borrowed from code shared by Richard Ibbotson    */
uint8_t USB::outTransfer( uint8_t addr, uint8_t ep, unsigned int nbytes, char* data, unsigned int wMaxPacketSize, uint8_t* sndToggle, unsigned int nak_limit )
{
    uint8_t ret = USB_SUCCESS, retry_count;
    char* data_p = data; //local copy of the data pointer
    unsigned int bytes_tosend, nak_count;
    unsigned int bytes_left = nbytes;
    unsigned long timeout = millis() + USB_XFER_TIMEOUT;

    _set_addr(addr); //set peripheral address
    regWr(rHCTL, *sndToggle); //set toggle value
    while (bytes_left)
    {
        retry_count = 0;
        nak_count = 0;
        bytes_tosend = (bytes_left >= wMaxPacketSize) ? wMaxPacketSize : bytes_left;
        bytesWr(rSNDFIFO, bytes_tosend, data_p);        //filling output FIFO
        regWr(rSNDBC, bytes_tosend);                    //set number of bytes    
        regWr(rHXFR, (tokOUT | (ep & 0x7F)));           //dispatch packet
        while (!(regRd(rHIRQ) & bmHXFRDNIRQ))           //wait for the completion IRQ
        {
            if (millis() > timeout)
            {
                return USB_ERROR_TRANSFER_TIMED_OUT;
            }
        }
        
        regWr(rHIRQ, bmHXFRDNIRQ);                      //clear IRQ
        ret = (regRd(rHRSL) & 0x0F);
        while (ret && (timeout > millis()))
        {
            switch (ret)
            {
                case hrNAK:
                {
                    nak_count++;
                    if (nak_limit && (nak_count == USB_NAK_LIMIT))
                    {
                        return ret; //return NAK
                    }
     
                    break;
                }

                case hrTIMEOUT:
                {
                    retry_count++;
                    if (retry_count == USB_RETRY_LIMIT)
                    {
                        return ret; //return TIMEOUT
                    }

                    break;
                }

                default:
                {
                    return ret;
                }
            }

            /* process NAK according to Host out NAK bug */
            regWr(rSNDBC, 0);
            regWr(rSNDFIFO, *data_p);
            regWr(rSNDBC, bytes_tosend);
            regWr(rHXFR, (tokOUT | ep));                 //dispatch packet
            while(!(regRd(rHIRQ) & bmHXFRDNIRQ));        //wait for the completion IRQ
            regWr(rHIRQ, bmHXFRDNIRQ);                   //clear IRQ
            ret = (regRd(rHRSL) & 0x0F);
        }

        bytes_left -= bytes_tosend;
        data_p += bytes_tosend;
    }
  
    //update toggle
    *sndToggle = (regRd(rHRSL) & bmSNDTOGRD) ? bmSNDTOG1 : bmSNDTOG0;
    return ret;
}

/* dispatch USB packet. Assumes peripheral address is set and relevant buffer is loaded/empty       */
/* If NAK, tries to re-send up to nak_limit times                                                   */
/* If nak_limit == 0, do not count NAKs, exit after timeout                                         */
/* If bus timeout, re-sends up to USB_RETRY_LIMIT times                                             */
/* Return codes 0x01-0x0f are HRSLT                                                                 */
uint8_t USB::dispatchPkt(uint8_t token, uint8_t ep, unsigned int nak_limit)
{
    unsigned long timeout = millis() + USB_XFER_TIMEOUT;
    uint8_t tmpdata;   
    uint8_t ret = USB_ERROR_TRANSFER_TIMED_OUT;
    unsigned int nak_count = 0;
    char retry_count = 0;

    while (timeout > millis())
    {
        regWr(rHXFR, (token|(ep & 0x7F))); //launch the transfer
        ret = USB_ERROR_TRANSFER_TIMED_OUT;
        while (millis() < timeout)
        {
            //wait for transfer completion
            tmpdata = regRd(rHIRQ);
            if (tmpdata & bmHXFRDNIRQ)
            {
                regWr(rHIRQ, bmHXFRDNIRQ); //clear the interrupt
                ret = USB_SUCCESS;
                break;
            }
        }

        if (ret)
        {
            //exit if timeout
            return ret;
        }

        ret = regRd(rHRSL) & 0x0F; //analyze transfer result
        switch (ret)
        {
            case hrNAK:
            {
                nak_count++;
                if (nak_limit && (nak_count == nak_limit))
                {
                    return ret;
                }

                break;
            }

            case hrTIMEOUT:
            {
                retry_count++;
                if (retry_count == USB_RETRY_LIMIT)
                {
                    return ret;
                }

                break;
            }

            default:
            {
                return ret;
            }
        }
    }

    return ret;
}

/* USB main task. Performs enumeration/cleanup */
//USB state machine
void USB::Task(void)
{
    static unsigned long delay = 0;

    MAX3421E::Task();
    switch (getVbusState())
    {
        case SE1: //illegal state
        {
            setUsbTaskState(USB_DETACHED_SUBSTATE_ILLEGAL);
            break;
        }

        case SE0: //disconnected
        {
            if ((usb_task_state & USB_STATE_MASK) != USB_STATE_DETACHED)
            {
                setUsbTaskState(USB_DETACHED_SUBSTATE_INITIALIZE);
            }

            break;
        }

        case FSHOST: //attached
        case LSHOST:
        {
            if ((usb_task_state & USB_STATE_MASK ) == USB_STATE_DETACHED)
            {
                delay = millis() + USB_SETTLE_DELAY;
                setUsbTaskState(USB_ATTACHED_SUBSTATE_SETTLE);
            }
 
            break;
        }

        default:
        {
            break;
        }
    }
    
    //Check all our incoming endpoints for data
    for (uint16_t i = 0; i < epFirstFreeIndex; i++)
    {
        ENDPOINT_INFO* info = &epInfo[i];
        if (info->autoPoll && (info->ep & 0x7F) != 0 && (info->ep & 0x80) > 0) //needs to be non-zero address and incoming endpoint
        {
            //Back up existing peripheral address and set new one
            uint8_t temp = regRd(rPERADDR);
            _set_addr(info->addr);

            //Attempt to read data into our buffer
            INCOMING_INFO dinfo;
            dinfo.addr = info->addr;
            dinfo.ep = info->ep;
            dinfo.length = info->maxPacketSize;
            uint8_t res = inTransfer(info->addr, info->ep, &dinfo.length, dinfo.buffer, info->maxPacketSize, &(info->rcvToggle), 1);
            dinfo.result = res;

            //Restore old peripheral address
            _set_addr(temp);

            if (eventCallback != NULL)
            {
                if (res != USB_ERROR_TRANSFER_TIMED_OUT)
                {
                    if (res == USB_SUCCESS)
                    {
                        eventCallback(USB_EVENT_INCOMING_DATA, &dinfo);
                    }
                    else if (res == hrSTALL)
                    {
                        eventCallback(USB_EVENT_INCOMING_DATA, &dinfo);
                    }
                }
            }
        }
    }

    switch (usb_task_state)
    {
        case USB_DETACHED_SUBSTATE_INITIALIZE:
        {
            setUsbTaskState(USB_DETACHED_SUBSTATE_WAIT_FOR_DEVICE);
            break;
        }

        case USB_DETACHED_SUBSTATE_WAIT_FOR_DEVICE: //just sit here
        {
            break;
        }

        case USB_DETACHED_SUBSTATE_ILLEGAL: //just sit here
        {
            break;
        }

        case USB_ATTACHED_SUBSTATE_SETTLE: //setlle time for just attached device                  
        {
            if (delay < millis())
            {
                setUsbTaskState(USB_ATTACHED_SUBSTATE_RESET_DEVICE);
            }
 
            break;
        }

        case USB_ATTACHED_SUBSTATE_RESET_DEVICE:
        {
            regWr(rHCTL, bmBUSRST); //issue bus reset
            setUsbTaskState(USB_ATTACHED_SUBSTATE_WAIT_RESET_COMPLETE);
            break;
        }

        case USB_ATTACHED_SUBSTATE_WAIT_RESET_COMPLETE:
        {
            if ((regRd(rHCTL) & bmBUSRST) == 0)
            {
                regWr(rMODE, regRd( rMODE ) | bmSOFKAENAB); //start SOF generation
                setUsbTaskState(USB_ATTACHED_SUBSTATE_WAIT_SOF);
                delay = millis() + 20; //20ms wait after reset per USB spec
            }

            break;
        }

        case USB_ATTACHED_SUBSTATE_WAIT_SOF:
        {
            if (regRd(rHIRQ) & bmFRAMEIRQ)
            {
                //when first SOF received we can continue
                if( delay < millis() )
                {
                    //Configure control endpoint with max packet size of 8 until we know better
                    configureEndpoint(0, 0, 8, bmSNDTOG0, bmRCVTOG0);

                    //20ms passed
                    setUsbTaskState(USB_STATE_RUNNING);
                }
            }
 
            break;
        }

        case USB_STATE_RUNNING:
        {
            break;
        }

        case USB_STATE_ERROR:
        {
            break;
        }
        
        default:
        {
            break;
        }
    }
}    
