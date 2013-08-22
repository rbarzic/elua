// Generic platform-wide header

#ifndef __PLATFORM_GENERIC_H__
#define __PLATFORM_GENERIC_H__

#define VTMR_CH               2    // Which hardware timer to use for VTMR

#if VTMR_NUM_TIMERS > 0
#undef NUM_TIMER
#define NUM_TIMER             2
#endif

#define PLATFORM_HAS_SYSTIMER

#if AVR32_TC_NUM>1
#define AVR32_TC AVR32_TC0
#define AVR32_TC_IRQ0 AVR32_TC0_IRQ0
#define AVR32_TC_IRQ1 AVR32_TC0_IRQ1
#define AVR32_TC_IRQ2 AVR32_TC0_IRQ2
#endif


#endif // #ifndef __PLATFORM_GENERIC_H__

