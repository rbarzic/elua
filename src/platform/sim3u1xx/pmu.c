// eLua Module for SIM3 power features

#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
#include "platform.h"
#include "lrotable.h"
#include "platform_conf.h"
#include "auxmods.h"
#include "elua_int.h"
#include "pmu.h"


//Lua: sleep(time)
static int pmu_sleep( lua_State *L )
{
  unsigned seconds;
  seconds = luaL_checkinteger( L, 1 );

  sim3_pmu_sleep( seconds );
  return 0;
}

//Lua: sleep(time)
static int pmu_pm9( lua_State *L )
{
  unsigned seconds;
  seconds = luaL_checkinteger( L, 1 );

  sim3_pmu_pm9( seconds );
  return 0;
}

//Lua: sleep(time)
static int pmu_reboot( lua_State *L )
{
  sim3_pmu_reboot();
  return 0;
}


#define MIN_OPT_LEVEL 2
#include "lrodefs.h"  

// Module function map
const LUA_REG_TYPE pmu_map[] =
{ 
  { LSTRKEY( "sleep" ),  LFUNCVAL( pmu_sleep ) },
  { LSTRKEY( "pm9" ),  LFUNCVAL( pmu_pm9 ) },
  { LSTRKEY( "reboot" ),  LFUNCVAL( pmu_reboot ) },
  { LNILKEY, LNILVAL }
};

LUALIB_API int luaopen_pmu( lua_State *L )
{
  LREGISTER( L, AUXLIB_PMU, pmu_map );
}  

