/* Copyright 2009-2011 Oleg Mazurov, Circuits At Home, http://www.circuitsathome.com */
/* Additions/changes by Brandon Wilson */

/* USB functions */
#ifndef _usb_h_
#define _usb_h_

#include <Max3421e.h>

#define USB_XFER_TIMEOUT                5000    //USB transfer timeout in milliseconds, per section 9.2.6.1 of USB 2.0 spec
#define USB_NAK_LIMIT                   32000   //NAK limit for a transfer
#define USB_RETRY_LIMIT                 30      //retry limit for a transfer
#define USB_SETTLE_DELAY                200     //settle delay in milliseconds
#define USB_MAX_CONFIGURED_ENDPOINTS    8       //maximum number of endpoints that are configurable by this code
#define USB_MAX_URBS                    8       //maximum number of simultaneous USB requests
#define USB_ENDPOINT_MAX_PACKET_SIZE    64      //largest endpoint maximum packet size

/* USB state machine states */
#define USB_STATE_MASK                                      0xf0

#define USB_STATE_DETACHED                                  0x10
#define USB_DETACHED_SUBSTATE_INITIALIZE                    0x11        
#define USB_DETACHED_SUBSTATE_WAIT_FOR_DEVICE               0x12
#define USB_DETACHED_SUBSTATE_ILLEGAL                       0x13
#define USB_ATTACHED_SUBSTATE_SETTLE                        0x20
#define USB_ATTACHED_SUBSTATE_RESET_DEVICE                  0x30    
#define USB_ATTACHED_SUBSTATE_WAIT_RESET_COMPLETE           0x40
#define USB_ATTACHED_SUBSTATE_WAIT_SOF                      0x50
#define USB_STATE_RUNNING                                   0x60
#define USB_STATE_ERROR                                     0xA0

/* USB callback event types */
#define USB_EVENT_STATE_CHANGED         0x01
#define USB_EVENT_INCOMING_DATA         0x02
#define USB_EVENT_URB_DATA_START        0x03
#define USB_EVENT_URB_DATA              0x04
#define USB_EVENT_URB_COMPLETE          0x05

/* Miscellaneous error/return codes */
#define USB_SUCCESS                                 0x00
#define USB_ERROR_TRANSFER_TIMED_OUT                0xFF
#define USB_ERROR_CONFIGURE_ENDPOINT_OUT_OF_SPACE   0xFE
#define USB_ERROR_ENDPOINT_NOT_CONFIGURED           0xFD
#define USB_ERROR_URB_OUT_OF_SPACE                  0xFC
#define USB_ERROR_URB_NOT_FOUND                     0xFB
#define USB_ERROR_RCVDAV_IRQ                        0xF0

struct ENDPOINT_INFO
{
    uint8_t addr;
    uint8_t ep;
    unsigned int maxPacketSize;
    uint8_t sndToggle;
    uint8_t rcvToggle;
    uint8_t autoPoll;
};

struct INCOMING_INFO
{
    uint8_t result;
    uint8_t addr;
    uint8_t ep;
    unsigned int length;
    char buffer[USB_ENDPOINT_MAX_PACKET_SIZE];
};

struct URB_DATA
{
    uint32_t id;
    uint8_t result;
    unsigned int length;
    char buffer[USB_ENDPOINT_MAX_PACKET_SIZE];
};

struct URB
{
    uint32_t id;
    uint8_t result;
    uint8_t addr;
    uint8_t ep;
    unsigned long requestedLength;
    unsigned long actualLength;
    uint8_t complete;
};

typedef void (*event_callback) (uint8_t eventType, void* data);

class USB : public MAX3421E
{
    public:
        USB(void);

        //URB information
        uint16_t urbFirstFreeIndex;
        URB urbs[USB_MAX_URBS];

        //Endpoint information
        uint16_t epFirstFreeIndex;
        ENDPOINT_INFO epInfo[USB_MAX_CONFIGURED_ENDPOINTS];

        /* High-level requests */
        void Task(void);
        uint8_t getUsbTaskState(void);
        void setUsbTaskState(uint8_t state);
        uint8_t configureEndpoint(uint8_t addr, uint8_t ep, uint8_t autoPoll);
        uint8_t configureEndpoint(uint8_t addr, uint8_t ep, unsigned int maxPacketSize, uint8_t sndToggle, uint8_t rcvToggle, uint8_t autoPoll = 1);
        void forgetEndpoint(uint8_t addr, uint8_t ep);
        void forgetAllEndpoints();
        int findEndpoint(uint8_t addr, uint8_t ep);
        void setEventCallback(event_callback callback);
        URB* findUrb(uint32_t id);
        uint8_t makeUrb(uint8_t addr, uint8_t ep, unsigned long requestedLength, URB** urb);
        void completeUrb(uint16_t idx);
        void processUrbs(event_callback callback);
        uint8_t cancelUrb(uint32_t id);
        void flushCompletedUrbs(void);

        /* Control requests */
        uint8_t ctrlReq(uint8_t addr, uint8_t bmReqType, uint8_t bRequest, unsigned int wValue, unsigned int wIndex,
            unsigned int* nbytes, char* dataptr, unsigned int nak_limit = USB_NAK_LIMIT);
        uint8_t ctrlReq(uint8_t addr, uint8_t bmReqType, uint8_t bRequest, unsigned int wValue, unsigned int wIndex,
            unsigned int* nbytes, char* dataptr, unsigned int wMaxPacketSize, uint8_t* sndToggle, uint8_t* rcvToggle, unsigned int nak_limit = USB_NAK_LIMIT);
        uint8_t ctrlSetup(uint8_t addr, uint8_t bmReqType, uint8_t bRequest,
            unsigned int wValue, unsigned int wIndex, unsigned int nbytes, unsigned int nak_limit = USB_NAK_LIMIT);
        uint8_t ctrlData(uint8_t addr, unsigned int* nbytes, char* dataptr, uint8_t deviceToHost, unsigned int nak_limit = USB_NAK_LIMIT);
        uint8_t ctrlData(uint8_t addr, unsigned int* nbytes, char* dataptr, uint8_t deviceToHost, unsigned int wMaxPacketSize,
            uint8_t* sndToggle, uint8_t* rcvToggle, unsigned int nak_limit = USB_NAK_LIMIT);
        uint8_t ctrlStatus(uint8_t deviceToHost, unsigned int nak_limit = USB_NAK_LIMIT);

        /* Non-control requests */
        uint8_t outTransfer(uint8_t addr, uint8_t ep, unsigned int nbytes, char* data, unsigned int nak_limit = USB_NAK_LIMIT);
        uint8_t outTransfer(uint8_t addr, uint8_t ep, unsigned int numbytes, char* data, unsigned int wMaxPacketSize, uint8_t* sndToggle, unsigned int nak_limit = USB_NAK_LIMIT);
        uint8_t inTransfer(uint8_t addr, uint8_t ep, unsigned int* nbytes, char* data, unsigned int nak_limit = USB_NAK_LIMIT);
        uint8_t inTransfer(uint8_t addr, uint8_t ep, unsigned int* nbytes, char* data, unsigned int wMaxPacketSize, uint8_t* rcvToggle, unsigned int nak_limit = USB_NAK_LIMIT);
        uint8_t urbInTransfer(uint32_t id, unsigned long* nbytes, event_callback callback, unsigned int nak_limit = USB_NAK_LIMIT);

        /* Low-level requests */
        uint8_t dispatchPkt(uint8_t token, uint8_t ep, unsigned int nak_limit = USB_NAK_LIMIT);

    private:
        uint8_t usb_task_state;
        event_callback eventCallback;

        void init(void);
        void processUrb(URB* urb, event_callback callback);
        void _set_addr(uint8_t addr);
};

#endif //_usb_h_
