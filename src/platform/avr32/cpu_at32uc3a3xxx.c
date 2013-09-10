// AT32UC3A3128/AT32UC3A3256/AT32UC3A3512 CPU special code

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
       /* AVR32UC3A3 port decoding
          See uc3a3256.h in avr32 headers
          PA00->PA31 : GPIO 0..31     port 0
          PB00->PB12 : GPIO 32..44    port 1
          PC00->PC05 : GPIO 45..50    port 2
          PX00->PX59 : GPIO 51..110   ports 3-4
          
       */
      // Disallow "PC_06-PC_31" 
      if (key[1] == 'C' && pin > 5)
        return 0;

      // Disallow "PD_nn" 
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
        if( pin <= 59 ) gpio = 51+pin;       
        else return 0;

        port = gpio >> 5; // div by 32
        pin = gpio & 0x1F; // %32
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




int pio_decode( lua_State *L )
{
  int code = ( int )luaL_checkinteger( L, 1 );
  int port = PLATFORM_IO_GET_PORT( code );
  int pin  = PLATFORM_IO_GET_PIN( code );

    /* AVR32UC3A3 port decoding
          See uc3a3256.h in avr32 headers
          PA00->PA31 : GPIO 0..31     port 0
          PB00->PB12 : GPIO 32..44    port 1
          PC00->PC05 : GPIO 45..50    port 1
          PX00->PX12 : GPIO 51..63    port 1
          PX13->PX44 : GPIO 64..95    ports 2
          PX45->PX59 : GPIO 96..110   ports 2
          
    */

  // Nothing to do - much cleaner than uc3s/uc3b :-)

  lua_pushinteger( L, port );
  lua_pushinteger( L, pin );
  return 2;
}





