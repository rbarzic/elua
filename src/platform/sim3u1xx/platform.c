// Platform-dependent functions

#include "platform.h"
#include "type.h"
#include "devman.h"
#include "genstd.h"
#include "stacks.h"
#include <reent.h>
#include <errno.h>
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include "utils.h"
#include "common.h"
#include "elua_adc.h"
#include "platform_conf.h"
#include "lrotable.h"
#include "buf.h"

// Platform includes
#include "sim3u1xx.h"
#include "sim3u1xx_Types.h"

#define SYSTICKHZ             100

// ****************************************************************************
// Platform initialization

// forward dcls
static void pios_init();
static void clk_init();

void hard_fault_handler_c(unsigned int * hardfault_args)
{
  unsigned int stacked_r0;
  unsigned int stacked_r1;
  unsigned int stacked_r2;
  unsigned int stacked_r3;
  unsigned int stacked_r12;
  unsigned int stacked_lr;
  unsigned int stacked_pc;
  unsigned int stacked_psr;

  stacked_r0 = ((unsigned long) hardfault_args[0]);
  stacked_r1 = ((unsigned long) hardfault_args[1]);
  stacked_r2 = ((unsigned long) hardfault_args[2]);
  stacked_r3 = ((unsigned long) hardfault_args[3]);

  stacked_r12 = ((unsigned long) hardfault_args[4]);
  stacked_lr = ((unsigned long) hardfault_args[5]);
  stacked_pc = ((unsigned long) hardfault_args[6]);
  stacked_psr = ((unsigned long) hardfault_args[7]);

  printf ("[Hard fault handler]\n");
  printf ("R0 = %x\n", stacked_r0);
  printf ("R1 = %x\n", stacked_r1);
  printf ("R2 = %x\n", stacked_r2);
  printf ("R3 = %x\n", stacked_r3);
  printf ("R12 = %x\n", stacked_r12);
  printf ("LR = %x\n", stacked_lr);
  printf ("PC = %x\n", stacked_pc);
  printf ("PSR = %x\n", stacked_psr);
  printf ("BFAR = %x\n", (*((volatile unsigned  *)(0xE000ED38))));
  printf ("CFSR = %x\n", (*((volatile unsigned  *)(0xE000ED28))));
  printf ("HFSR = %x\n", (*((volatile unsigned  *)(0xE000ED2C))));
  printf ("DFSR = %x\n", (*((volatile unsigned  *)(0xE000ED30))));
  printf ("AFSR = %x\n", (*((volatile unsigned  *)(0xE000ED3C))));

  while (1) { ;; }
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  asm(
  "tst lr, #4\n\t"
  "ite eq\n\t"
  "mrseq r0, msp\n\t"
  "mrsne r0, psp\n\t"
  "b hard_fault_handler_c"
  );
}


// SiM3 SystemInit calls this function, disable watchdog timer
void mySystemInit(void)
{
  SI32_WDTIMER_A_stop_counter(SI32_WDTIMER_0);

  // enable APB clock to the Port Bank module
  SI32_CLKCTRL_A_enable_apb_to_modules_0 (SI32_CLKCTRL_0, SI32_CLKCTRL_A_APBCLKG0_PB0CEN_MASK);
  // make the SWO pin (PB1.3) push-pull to enable SWV printf
  SI32_PBSTD_A_set_pins_push_pull_output (SI32_PBSTD_1, (1<<3));
}

int platform_init()
{
  SystemInit();

  // Configure the NVIC Preemption Priority Bits:
  // two (2) bits of preemption priority, six (6) bits of sub-priority.
  // Since the Number of Bits used for Priority Levels is five (5), so the
  // actual bit number of sub-priority is three (3)
  NVIC_SetPriorityGrouping(0x05);

  // Setup peripherals
  // platform_setup_timers();

  // Peripheral Clocking setup
  clk_init();

  // GPIO setup
  pios_init();

  // System timer setup
  cmn_systimer_set_base_freq( cmsis_get_cpu_frequency() );
  cmn_systimer_set_interrupt_freq( SYSTICKHZ );

  // Enable SysTick
  SysTick_Config( cmsis_get_cpu_frequency() / SYSTICKHZ );

  NVIC_SetPriority(SysTick_IRQn, (1 << __NVIC_PRIO_BITS) - 1);

  // Common platform initialization code
  cmn_platform_init();

  return PLATFORM_OK;
} 


void clk_init( void )
{
#if defined( ELUA_BOARD_GSBRD )
  SI32_CLKCTRL_A_enable_apb_to_modules_0(SI32_CLKCTRL_0,
                                         SI32_CLKCTRL_A_APBCLKG0_PB0 |
                                         SI32_CLKCTRL_A_APBCLKG0_USART0 |
                                         SI32_CLKCTRL_A_APBCLKG0_USART1 |
                                         SI32_CLKCTRL_A_APBCLKG0_UART0 |
                                         SI32_CLKCTRL_A_APBCLKG0_UART1 |
                                         SI32_CLKCTRL_A_APBCLKG0_SPI0 |
                                         SI32_CLKCTRL_A_APBCLKG0_I2C0 |
                                         SI32_CLKCTRL_A_APBCLKG0_SARADC0 |
                                         SI32_CLKCTRL_A_APBCLKG0_AES0 |
                                         SI32_CLKCTRL_A_APBCLKG0_CRC0 |
                                         SI32_CLKCTRL_A_APBCLKG0_LPTIMER0 |
                                         SI32_CLKCTRL_A_APBCLKG0_USB0 |
                                         SI32_CLKCTRL_A_APBCLKG0_FLASHCTRL0);
  SI32_CLKCTRL_A_enable_apb_to_modules_1(SI32_CLKCTRL_0,
                                         SI32_CLKCTRL_A_APBCLKG1_MISC1 |
                                         SI32_CLKCTRL_A_APBCLKG1_MISC0);
  //SI32_CLKCTRL_A_enable_ahb_to_emif(SI32_CLKCTRL_0);

#else
  SI32_CLKCTRL_A_enable_apb_to_modules_0(SI32_CLKCTRL_0, 
                                         SI32_CLKCTRL_A_APBCLKG0_PB0 |
                                         SI32_CLKCTRL_A_APBCLKG0_USART0 |
                                         SI32_CLKCTRL_A_APBCLKG0_USART1 |
                                         SI32_CLKCTRL_A_APBCLKG0_UART0 |
                                         SI32_CLKCTRL_A_APBCLKG0_UART1);
#endif
}


extern u32 SystemCoreClock;
u32 cmsis_get_cpu_frequency()
{
  return SystemCoreClock;
}

// SysTick interrupt handler
void SysTick_Handler()
{
  // Handle virtual timers
  cmn_virtual_timer_cb();

  // Handle system timer call
  cmn_systimer_periodic();
}

// ****************************************************************************
// PIO section

static SI32_PBSTD_A_Type* const port_std[] = { SI32_PBSTD_0, SI32_PBSTD_1, SI32_PBSTD_2, SI32_PBSTD_3 };

void pios_init( void )
{
#if defined( ELUA_BOARD_GSBRD )

  SI32_PBCFG_A_unlock_ports(SI32_PBCFG_0);

  // PB0 Setup
  SI32_PBSTD_A_set_pins_analog(SI32_PBSTD_0, 0x0603);
  SI32_PBSTD_A_set_pins_push_pull_output(SI32_PBSTD_0, 0x1114);
  SI32_PBSTD_A_write_pbskipen(SI32_PBSTD_0, 0x0603);

  // PB1 Setup
  SI32_PBSTD_A_set_pins_push_pull_output(SI32_PBSTD_1, 0x00A1);
  SI32_PBSTD_A_write_pbskipen(SI32_PBSTD_1, 0xFC1C);

  // Enable Crossbar0 signals & set properties
  SI32_PBCFG_A_enable_xbar0h_peripherals(SI32_PBCFG_0,
                                         SI32_PBCFG_A_XBAR0H_UART0EN |
                                         SI32_PBCFG_A_XBAR0H_UART1EN);
  SI32_PBCFG_A_enable_xbar0l_peripherals(SI32_PBCFG_0,
                                         SI32_PBCFG_A_XBAR0L_USART0EN |
                                         SI32_PBCFG_A_XBAR0L_SPI0EN |
                                         SI32_PBCFG_A_XBAR0L_SPI0NSSEN |
                                         SI32_PBCFG_A_XBAR0L_USART1EN |
                                         SI32_PBCFG_A_XBAR0L_USART1FCEN |
                                         SI32_PBCFG_A_XBAR0L_I2C0EN);
  SI32_PBCFG_A_enable_crossbar_0(SI32_PBCFG_0);

  // PB2 Setup
  SI32_PBSTD_A_write_pbskipen(SI32_PBSTD_2, 0x7FFF);

  // PB3 Setup
  SI32_PBSTD_A_write_pbskipen(SI32_PBSTD_3, 0x00FF);

  // Enable Crossbar1 signals & set properties
  SI32_PBCFG_A_enable_crossbar_1(SI32_PBCFG_0);

  // PB4 Setup
  SI32_PBHD_A_write_pblock(SI32_PBHD_4, 0x0000);
  SI32_PBHD_A_set_pins_analog(SI32_PBHD_4, 0x000C);

#else
  // Set up prinf pin
  SI32_PBSTD_A_set_pins_push_pull_output(SI32_PBSTD_1, 0x00000008);

  SI32_PBCFG_A_enable_crossbar_1(SI32_PBCFG_0);
  SI32_PBCFG_A_enable_crossbar_0(SI32_PBCFG_0);

  // ENABLE LED DRIVERS (P2.11, P2.10)
  //SI32_PBSTD_A_set_pins_push_pull_output(SI32_PBSTD_2, 0x00000C00);

  // Enable switch sensing (P2.10, P2.11)
  //SI32_PBSTD_A_set_pins_digital_input(SI32_PBSTD_2, 0x00000300);

  // UART PINS TO PROPER CONFIG (TX = PB1.12, RX = PB1.13)
  SI32_PBSTD_A_set_pins_push_pull_output(SI32_PBSTD_1, 0x0001000);    
  SI32_PBSTD_A_set_pins_digital_input(SI32_PBSTD_1, 0x00002000);
  SI32_PBSTD_A_write_pbskipen(SI32_PBSTD_0, 0x0000FFFF);
  SI32_PBSTD_A_write_pbskipen(SI32_PBSTD_1, 0x00000FFF);

  // BRING OUT UART
  SI32_PBCFG_A_enable_xbar0h_peripherals(SI32_PBCFG_0, SI32_PBCFG_A_XBAR0H_UART0EN);
#endif
}


// The platform I/O functions
pio_type platform_pio_op( unsigned port, pio_type pinmask, int op )
{
  pio_type retval = 1;
  
  switch( op )
  {
    case PLATFORM_IO_PORT_SET_VALUE:
      SI32_PBSTD_A_write_pins_masked( port_std[ port ], 0xFFFF, pinmask);
      break;
    
    case PLATFORM_IO_PIN_SET:
      SI32_PBSTD_A_write_pins_high( port_std[ port ], pinmask );
      break;
    
    case PLATFORM_IO_PIN_CLEAR:
      SI32_PBSTD_A_write_pins_low( port_std[ port ], pinmask );
      break;
    
    case PLATFORM_IO_PORT_DIR_OUTPUT:
      SI32_PBSTD_A_set_pins_push_pull_output( port_std[ port ], 0xFFFF );
      break;    

    case PLATFORM_IO_PIN_DIR_OUTPUT:
      SI32_PBSTD_A_set_pins_push_pull_output( port_std[ port ], pinmask );
      break;
    
    case PLATFORM_IO_PORT_DIR_INPUT:
      SI32_PBSTD_A_set_pins_digital_input( port_std[ port ], 0xFFFF );
      break;

    case PLATFORM_IO_PIN_DIR_INPUT:
      SI32_PBSTD_A_set_pins_digital_input( port_std[ port ], pinmask );
      break;    
          
    case PLATFORM_IO_PORT_GET_VALUE:
      retval = SI32_PBSTD_A_read_pins(port_std[ port ]);
      break;
    
    case PLATFORM_IO_PIN_GET:
      retval = ( SI32_PBSTD_A_read_pins(port_std[ port ]) & pinmask ) ? 1 : 0;
      break;
    
    default:
      retval = 0;
      break;
  }
  return retval;
}


// ****************************************************************************
// UART section

static SI32_UART_A_Type* const uart[] = { SI32_UART_0, SI32_UART_1 };
static SI32_USART_A_Type* const usart[] = { SI32_USART_0, SI32_USART_1 };

//For DK board, interface is On:
// PB1.12 RX
// PB1.13 TX
// PB1.14 CTS
// PB1.15 RTS


u32 platform_uart_setup( unsigned id, u32 baud, int databits, int parity, int stopbits )
{
  if( id < 2 )
  {
    SI32_USART_A_enter_full_duplex_mode( uart[ id ] );

    // Set Baud Rate
    // rate = F_APB / ( N * (R/TBAUD + 1 ) )
    SI32_USART_A_set_rx_baudrate( usart[ id ], (SystemCoreClock / (2 * baud)) - 1);
    SI32_USART_A_set_tx_baudrate( usart[ id ], (SystemCoreClock / (2 * baud)) - 1);

    // Use Asynchronous Mode
    SI32_USART_A_select_tx_asynchronous_mode ( usart[ id ] );
    SI32_USART_A_select_rx_asynchronous_mode ( usart[ id ] );

    // Set Data Bits
    SI32_USART_A_select_tx_data_length( usart[ id ], databits );
    SI32_USART_A_select_rx_data_length( usart[ id ], databits );


    SI32_USART_A_enable_tx_start_bit( usart[ id ] );  
    SI32_USART_A_enable_tx_stop_bit( usart[ id ] );
    SI32_USART_A_enable_rx_start_bit( usart[ id ] );  
    SI32_USART_A_enable_rx_stop_bit( usart[ id ] );

    if( stopbits == PLATFORM_UART_STOPBITS_2 )
    {
      SI32_USART_A_select_tx_stop_bits( usart[ id ], SI32_USART_A_STOP_BITS_2_BITS );
      SI32_USART_A_select_rx_stop_bits( usart[ id ], SI32_USART_A_STOP_BITS_2_BITS );
    }
    else
    {
      SI32_USART_A_select_tx_stop_bits( usart[ id ], SI32_USART_A_STOP_BITS_1_BIT );
      SI32_USART_A_select_rx_stop_bits( usart[ id ], SI32_USART_A_STOP_BITS_1_BIT );
    }

    // Set Parity
    switch( parity )
    {
      case PLATFORM_UART_PARITY_NONE:
        SI32_USART_A_disable_tx_parity_bit( usart[ id ] );
        SI32_USART_A_disable_rx_parity_bit( usart[ id ] );
        break;
    
      case PLATFORM_UART_PARITY_ODD:
        SI32_USART_A_enable_tx_parity_bit( usart[ id ] );
        SI32_USART_A_select_tx_parity( usart[ id ], 0 );
        SI32_USART_A_enable_rx_parity_bit( usart[ id ] );
        SI32_USART_A_select_rx_parity( usart[ id ], 0 );
        break;
    
      case PLATFORM_UART_PARITY_EVEN:
        SI32_USART_A_enable_tx_parity_bit( usart[ id ] );
        SI32_USART_A_select_tx_parity( usart[ id ], 1 );
        SI32_USART_A_enable_rx_parity_bit( usart[ id ] );
        SI32_USART_A_select_rx_parity( usart[ id ], 1 );
        break;
    }

    SI32_USART_A_disable_tx_signal_inversion( usart[ id ] );
    SI32_USART_A_disable_rx_signal_inversion( usart[ id ] );

    //SI32_USART_A_select_rx_fifo_threshold_1( usart[ id ] );

    // Enable RX & TX
    SI32_USART_A_enable_tx( usart[ id ] );
    SI32_USART_A_enable_rx( usart[ id ] );
  }
  else
  {
    id = id - 2;
 
    SI32_UART_A_enter_full_duplex_mode( uart[ id ] );

    // Set Baud Rate
    // rate = F_APB / ( N * (R/TBAUD + 1 ) )
    SI32_UART_A_set_rx_baudrate( uart[ id ], (SystemCoreClock / (2 * baud)) - 1);
    SI32_UART_A_set_tx_baudrate( uart[ id ], (SystemCoreClock / (2 * baud)) - 1);

    // Use Asynchronous Mode
    //SI32_UART_A_select_tx_asynchronous_mode ( uart[ id ] );
    //SI32_UART_A_select_rx_asynchronous_mode ( uart[ id ] );

    // Set Data Bits
    SI32_UART_A_select_tx_data_length( uart[ id ], databits );
    SI32_UART_A_select_rx_data_length( uart[ id ], databits );


    SI32_UART_A_enable_tx_start_bit( uart[ id ] );  
    SI32_UART_A_enable_tx_stop_bit( uart[ id ] );
    SI32_UART_A_enable_rx_start_bit( uart[ id ] );  
    SI32_UART_A_enable_rx_stop_bit( uart[ id ] );

    if( stopbits == PLATFORM_UART_STOPBITS_2 )
    {
      SI32_UART_A_select_tx_stop_bits( uart[ id ], SI32_UART_A_STOP_BITS_2_BITS );
      SI32_UART_A_select_rx_stop_bits( uart[ id ], SI32_UART_A_STOP_BITS_2_BITS );
    }
    else
    {
      SI32_UART_A_select_tx_stop_bits( uart[ id ], SI32_UART_A_STOP_BITS_1_BIT );
      SI32_UART_A_select_rx_stop_bits( uart[ id ], SI32_UART_A_STOP_BITS_1_BIT );
    }

    // Set Parity
    switch( parity )
    {
      case PLATFORM_UART_PARITY_NONE:
        SI32_UART_A_disable_tx_parity_bit( uart[ id ] );
        SI32_UART_A_disable_rx_parity_bit( uart[ id ] );
        break;
    
      case PLATFORM_UART_PARITY_ODD:
        SI32_UART_A_enable_tx_parity_bit( uart[ id ] );
        SI32_UART_A_select_tx_parity( uart[ id ], 0 );
        SI32_UART_A_enable_rx_parity_bit( uart[ id ] );
        SI32_UART_A_select_rx_parity( uart[ id ], 0 );
        break;
    
      case PLATFORM_UART_PARITY_EVEN:
        SI32_UART_A_enable_tx_parity_bit( uart[ id ] );
        SI32_UART_A_select_tx_parity( uart[ id ], 1 );
        SI32_UART_A_enable_rx_parity_bit( uart[ id ] );
        SI32_UART_A_select_rx_parity( uart[ id ], 1 );
        break;
    }

    SI32_UART_A_disable_tx_signal_inversion( uart[ id ] );
    SI32_UART_A_disable_rx_signal_inversion( uart[ id ] );

    //SI32_UART_A_select_rx_fifo_threshold_1( uart[ id ] );

    // Enable RX & TX
    SI32_UART_A_enable_tx( uart[ id ] );
    SI32_UART_A_enable_rx( uart[ id ] );
  }
  
  
  return baud; // FIXME: find a way to actually get baud
}

void platform_s_uart_send( unsigned id, u8 data )
{
  if( id < 2 )
  {
    // Block if the output buffer is full
    while (SI32_USART_A_read_tx_fifo_count(usart[ id ]) >= 4);

    // Write character to the output buffer
    SI32_USART_A_write_data_u8(usart[ id ], data);
  }
  else
  {
    id = id - 2;
    // Block if the output buffer is full
    while (SI32_UART_A_read_tx_fifo_count(uart[ id ]) >= 4);

    // Write character to the output buffer
    SI32_UART_A_write_data_u8(uart[ id ], data);
  }
}

int platform_s_uart_recv( unsigned id, timer_data_type timeout )
{
  if( id < 2 )
  {
    if( timeout == 0 )
    {
      if ( SI32_USART_A_read_rx_fifo_count( usart[ id ] ) == 0 )
        return -1;
      else
        return ( int )SI32_USART_A_read_data_u8( usart[ id ] );
    }
  
    // Block if input buffer is empty
    while (SI32_USART_A_read_rx_fifo_count( usart[ id ] ) == 0);

    // Read character from the input buffer
    return ( int )SI32_USART_A_read_data_u8( usart[ id ] );
  }
  else
  {
    id = id - 2;
    if( timeout == 0 )
    {
      if ( SI32_UART_A_read_rx_fifo_count( uart[ id ] ) == 0 )
        return -1;
      else
        return ( int )SI32_UART_A_read_data_u8( uart[ id ] );
    }
  
    // Block if input buffer is empty
    while (SI32_UART_A_read_rx_fifo_count( uart[ id ] ) == 0);

    // Read character from the input buffer
    return ( int )SI32_UART_A_read_data_u8( uart[ id ] );
  }
}

int platform_s_uart_set_flow_control( unsigned id, int type )
{
  if( id < 2 )
  {
    if( type == PLATFORM_UART_FLOW_NONE )
    {
      SI32_USART_A_disable_cts( usart[ id ] );
      SI32_USART_A_disable_rts( usart[ id ] );
    }
    if( type & PLATFORM_UART_FLOW_CTS )
    {
      SI32_USART_A_enable_cts( usart[ id ] );
    }
    if( type & PLATFORM_UART_FLOW_RTS )
    {
      SI32_USART_A_enable_rts( usart[ id ] );
    }
  }
  else
  {
    id = id - 2;
    if( type == PLATFORM_UART_FLOW_NONE )
    {
      SI32_UART_A_disable_cts( uart[ id ] );
      SI32_UART_A_disable_rts( uart[ id ] );
    }
    if( type & PLATFORM_UART_FLOW_CTS )
    {
      SI32_UART_A_enable_cts( uart[ id ] );
    }
    if( type & PLATFORM_UART_FLOW_RTS )
    {
      SI32_UART_A_enable_rts( uart[ id ] );
    }
  }

  return PLATFORM_OK;
}

// ****************************************************************************
// Timer section



// Helper function: get timer clock
static u32 platform_timer_get_clock( unsigned id )
{
}

// Helper function: set timer clock
static u32 platform_timer_set_clock( unsigned id, u32 clock )
{

  
  return clock;
}

// Helper function: setup timers
static void platform_setup_timers()
{

}

void platform_s_timer_delay( unsigned id, timer_data_type delay_us )
{

}
      
timer_data_type platform_s_timer_op( unsigned id, int op, timer_data_type data )
{
  u32 res = 0;

  switch( op )
  {
    case PLATFORM_TIMER_OP_START:
      break;
      
    case PLATFORM_TIMER_OP_READ:
      break;

    case PLATFORM_TIMER_OP_SET_CLOCK:
      break;
      
    case PLATFORM_TIMER_OP_GET_CLOCK:
      break;

    case PLATFORM_TIMER_OP_GET_MAX_CNT:
      break;
  }
  return res;
}

u64 platform_timer_sys_raw_read()
{
  return SysTick->LOAD - SysTick->VAL;
}

void platform_timer_sys_disable_int()
{
  SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
}

void platform_timer_sys_enable_int()
{
  SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
}

timer_data_type platform_timer_read_sys()
{
  return cmn_systimer_get();
}
