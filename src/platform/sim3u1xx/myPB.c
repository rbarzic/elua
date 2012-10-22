// Copyright (c) 2012

#include "myPB.h"
#include <SI32_PBSTD_A_Type.h>
#include <SI32_PBHD_A_Type.h>
#include <si32_device.h>

void myPB_enter_off_config()
{
   SI32_PBSTD_A_write_pins_high(SI32_PBSTD_2, 0xC00);

   // all ports hi-z (analog)
   SI32_PBSTD_A_set_pins_analog(SI32_PBSTD_0, 0x0000FFFF);
   SI32_PBSTD_A_set_pins_analog(SI32_PBSTD_1, 0x0000FFFF);
   SI32_PBSTD_A_set_pins_analog(SI32_PBSTD_2, 0x0000FFFF);
   SI32_PBSTD_A_set_pins_analog(SI32_PBSTD_3, 0x0000FFFF);

   //PBHD to High impedance
   SI32_PBHD_A_disable_drivers(SI32_PBHD_4);
   SI32_PBHD_A_disable_bias(SI32_PBHD_4);
}
