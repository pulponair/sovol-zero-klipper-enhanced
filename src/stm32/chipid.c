// Support for extracting the hardware chip id on stm32
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "generic/canserial.h" // canserial_set_uuid
#include "generic/usb_cdc.h" // usb_fill_serial
#include "generic/usbstd.h" // usb_string_descriptor
#include "internal.h" // UID_BASE
#include "sched.h" // DECL_INIT

#define CHIP_UID_LEN 12

static struct {
    struct usb_string_descriptor desc;
    uint16_t data[CHIP_UID_LEN * 2];
} cdc_chipid;

struct usb_string_descriptor *
usbserial_get_serialid(void)
{
   return &cdc_chipid.desc;
}

//stm32f750     1234567890123456780ULL     canbus_uuid:946fab27ac7f
//stm32f103     1234567890123456788ULL     canbus_uuid:466a08fc77da

void
chipid_init(void)
{
    uint64_t custumUID = 1234567890123456780ULL;
    uint64_t *uid_base = &custumUID;
    if (CONFIG_USB_SERIAL_NUMBER_CHIPID)
        usb_fill_serial(&cdc_chipid.desc, ARRAY_SIZE(cdc_chipid.data)
                        , (void*)uid_base);
    if (CONFIG_CANBUS)
        canserial_set_uuid((void *)uid_base, CHIP_UID_LEN);  //(void*)UID_BASE
}
DECL_INIT(chipid_init);
