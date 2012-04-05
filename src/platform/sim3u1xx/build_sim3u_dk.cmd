call c:\Silabs\Precision32_v1.0\IDE\Precision32Path.cmd
@echo off
pushd ..\..\..\
lua build_elua.lua board=SIM3U1XX-B-DK toolchain=codered allocator=multiple prog
echo connect/prepare device to be flashed (hit enter when ready)
pause

C:\Silabs\Precision32_v1.0\IDE\precision32\bin\crt_emu_cm3_ng -g -2 -s1000 -vendor=SiLabs -pSiM3U167 -flash-load-partial="elua_lua_sim3u167.elf"  -wire=sladi
popd
pause