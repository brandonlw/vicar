MCU          = atmega32u4
ARCH         = AVR8
BOARD        = TEENSY2
F_CPU        = 16000000

TARGET       = vicar
SRC          = $(TARGET).cpp Max3421e.cpp Usb.cpp pins.cpp msd.cpp lufa.cpp sendrecv.cpp $(LUFA_SRC_USB)

OPTIMIZATION = s
LUFA_PATH    = ../LUFA
CC_FLAGS     = -DUSB_STREAM_TIMEOUT_MS=250 -DUSE_STATIC_OPTIONS="USB_DEVICE_OPT_FULLSPEED|USB_OPT_REG_ENABLED|USB_OPT_AUTO_PLL" -DUSB_DEVICE_ONLY -DFIXED_CONTROL_ENDPOINT_SIZE=64 -DFIXED_NUM_CONFIGURATIONS=1 -DUSE_RAM_DESCRIPTORS
LD_FLAGS     = 
F_USB        = $(F_CPU)

# Default target
all:

# Include LUFA build script makefiles
include $(LUFA_PATH)/Build/lufa_core.mk
include $(LUFA_PATH)/Build/lufa_sources.mk
include $(LUFA_PATH)/Build/lufa_build.mk
include $(LUFA_PATH)/Build/lufa_cppcheck.mk
include $(LUFA_PATH)/Build/lufa_doxygen.mk
include $(LUFA_PATH)/Build/lufa_dfu.mk
include $(LUFA_PATH)/Build/lufa_hid.mk
include $(LUFA_PATH)/Build/lufa_avrdude.mk
include $(LUFA_PATH)/Build/lufa_atprogram.mk
