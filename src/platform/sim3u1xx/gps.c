// eLua Module for capturing NMEA sentences

#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
#include "platform.h"
#include "lrotable.h"
#include "common.h"
#include "sermux.h"
#include "platform_conf.h"
#include "auxmods.h"
#include "elua_int.h"
#include "buf.h"

static int gps_uart_id = -1;
static elua_int_c_handler prev_handler;

static void gps_read( elua_int_resnum resnum );

//Lua: init( uart_id, baud, databits, parity, stopbits )
static int sim3_gps_init( lua_State *L )
{
  unsigned id, databits, parity, stopbits;
  u32 baud, res;
  
  id = luaL_checkinteger( L, 1 );
  MOD_CHECK_ID( uart, id );
  if( id >= SERMUX_SERVICE_ID_FIRST )
    return luaL_error( L, "cannot use virtual UARTs" );
  gps_uart_id = id;
  baud = luaL_checkinteger( L, 2 );
  databits = luaL_checkinteger( L, 3 );
  parity = luaL_checkinteger( L, 4 );
  stopbits = luaL_checkinteger( L, 5 );
  res = platform_uart_setup( gps_uart_id, baud, databits, parity, stopbits );
  lua_pushinteger( L, res );

  platform_uart_set_buffer( gps_uart_id, BUF_SIZE_128 );
  prev_handler = elua_int_set_c_handler( INT_UART_BUF_MATCH, gps_read );
  return 1;
}


//Lua: gpsread( uart_id )
static void gps_read( elua_int_resnum resnum )
{
  int d;

  if( prev_handler )
    prev_handler( resnum );

  if( resnum != gps_uart_id )
    return;

  do
  {
    d = platform_uart_recv( gps_uart_id, PLATFORM_TIMER_SYS_ID, 0 );

    if ( d >= 0 && d != 0x0d )
      printf("%c", ( char )d);

  } while ( d >= 0 );

  printf("\n");
}

#define MIN_OPT_LEVEL 2
#include "lrodefs.h"  

// Module function map
const LUA_REG_TYPE gps_map[] =
{ 
  { LSTRKEY( "init" ),  LFUNCVAL( sim3_gps_init ) },
  { LNILKEY, LNILVAL }
};

LUALIB_API int luaopen_gps( lua_State *L )
{
  LREGISTER( L, AUXLIB_GPS, gps_map );
}  

