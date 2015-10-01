#ifndef _VICAR_H_
#define _VICAR_H_

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define DEVICE_TYPE_HID         1
#define DEVICE_TYPE_CUSTOM      2
#define DEVICE_CONFIG_HID       0x80
#define DEVICE_CONFIG_MSD       0x40
#define DEVICE_TYPE_DEFAULT     DEVICE_TYPE_HID
#define CUSTOM_CONFIGS_DEFAULT  0

#define HID_VENDOR_ID           0xDEAD
#define HID_PRODUCT_ID          0xBEEF
#define HID_FIRST_INTERFACE     0x00
#define HID_SECOND_INTERFACE    0x01
#define HID_INCOMING_ENDPOINT1  0x01
#define HID_INCOMING_ENDPOINT2  0x02
#define HID_PACKET_SIZE         0x80
#define HID_REPORT_ID           0x01
#define HID_MAX_CONTROL_SIZE    0x1000

#define CUSTOM_VENDOR_ID                0xBAAD
#define CUSTOM_PRODUCT_ID               0xF00D
#define CUSTOM_INCOMING_CMD_ENDPOINT    0x01
#define CUSTOM_INCOMING_DATA_ENDPOINT   0x06
#define CUSTOM_OUTGOING_CMD_ENDPOINT    0x02
#define CUSTOM_MSD_INCOMING_ENDPOINT    0x03
#define CUSTOM_MSD_OUTGOING_ENDPOINT    0x04
#define CUSTOM_HID_INCOMING_ENDPOINT    0x05
#define CUSTOM_VENDOR_CODE              0x01
#define CUSTOM_INTERFACE_MSD            0x00
#define CUSTOM_INTERFACE_HID            0x01
#define CUSTOM_INTERFACE_VENDOR         0x02

#define FIRST_SEQUENCE      0x00000001
#define DESCRIPTOR_BUFFER_SIZE  256

#endif //_VICAR_H_
