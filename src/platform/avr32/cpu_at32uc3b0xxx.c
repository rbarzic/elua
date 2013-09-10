// AT32UC3B0256/ CPU special code
// copied from
// AT32UC3A0128/AT32UC3A0256/AT32UC3A0512 CPU special code


#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
#include "platform.h"
#include "auxmods.h"
#include "lrotable.h"
#include "platform_conf.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>


int pio_mt_index( lua_State* L )
{
  const char *key = luaL_checkstring( L ,2 );
  int port = 0xFFFF, pin = 0xFFFF, isport = 0, sz;
  
  if( !key || *key != 'P' )
    return 0;
  if( isupper( key[ 1 ] ) ) // PA, PB, ...
  {
    if( PIO_PREFIX != 'A' )
      return 0;
    port = key[ 1 ] - 'A';
    if( key[ 2 ] == '\0' )
      isport = 1;
    else if( key[ 2 ] == '_' )      
    {
      if( sscanf( key + 3, "%d%n", &pin, &sz ) != 1 || sz != strlen( key ) - 3 )
        return 0;      

      /* AVR32UC3A0 has a bizarre "port" called "PX" with 40 pins which map to
       * random areas of hardware ports 2 and 3:
       * PX00-PX10 = GPIO100-GPIO90     //Port 3 pins 04-00; port 2 pins 31-26
       * PX11-PX14 = GPIO109-GPIO106    //Port 3 pins 13-10
       * PX15-PX34 = GPIO89-GPIO70      //Port 2 pins 25-06
       * PX35-PX39 = GPIO105-GPIO101    //Port 3 pins 09-05
       * Then port = trunc(GPIO/32) and pin = GPIO % 32
       *
       * This "Port X" exists in EVK1100 and MIZAR32 but not on EVK1101, which
       * only has ports A and B. On EXK1101, the PC and PX syntax will still be
       * accepted but will return nil thanks to the checks against NUM_PIO.
       */

      // Disallow "PC_06-PC_31" as aliases for PX pins
      if (key[1] == 'C' && pin > 5)
        return 0;

      // Disallow "PD_nn" as aliases for PX pins
      if (key[1] == 'D')
        return 0;

      // Map PX pins 00-39 to their ports/pins in the hardware register layout.
      if (key[1] == 'X')
      {
        unsigned gpio;

        // You cannot perform port operations on port X because it
        // doesn't exist in hardware.
        if (pin == 0xFFFF)
          return 0;

        // Map PX pin numbers to GPIO pin numbers
        if( pin < 0 ) return 0;
        if( pin <= 10 ) gpio = 100 - pin;
        else if( pin <= 14 ) gpio = 109 - (pin - 11);
        else if( pin <= 34 ) gpio = 89 - (pin - 15);
        else if( pin <= 39 ) gpio = 105 - (pin - 35);
        else return 0;

        port = gpio >> 5;
        pin = gpio & 0x1F;
      }
    }
  }

  sz = -1;
  if( isport )
  {
    if( platform_pio_has_port( port ) )
      sz = PLATFORM_IO_ENCODE( port, 0, 1 );
  }
  else
  {
    if( platform_pio_has_port( port ) && platform_pio_has_pin( port, pin ) )
      sz = PLATFORM_IO_ENCODE( port, pin, 0 );
  }
  if( sz == -1 )
    return 0;
  else
  {
    lua_pushinteger( L, sz );
    return 1;
  }
}



