// Copyright (c) 2012

#include "myPMU0.h"
#include <SI32_PMU_A_Type.h>
#include <SI32_RSTSRC_A_Type.h>

void myPMU_enter_sleep(void)
{
  // Configure for Sleep
  SI32_PMU_A_clear_wakeup_flags(SI32_PMU_0);
  SI32_RSTSRC_A_enable_power_mode_9(SI32_RSTSRC_0);
  SCB->SCR = SCB_SCR_SLEEPDEEP_Msk; // set SLEEPDEEP
  __set_FAULTMASK(1);
  __WFI();
}
