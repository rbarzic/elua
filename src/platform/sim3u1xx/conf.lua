-- Configuration file for the sim3u1xx backend

addi( sf( 'src/platform/%s/si32Hal/SI32_Modules', platform ) )
addi( sf( 'src/platform/%s/si32Hal/sim3u1xx', platform ) )

local fwlib_files = utils.get_files( sf( "src/platform/%s/si32Hal/SI32_Modules", platform ), ".*%.c$" )
fwlib_files = fwlib_files .. " " .. utils.get_files( sf( "src/platform/%s/si32Hal/sim3u1xx", platform ), ".*%.c$" )
specific_files = "platform.c platform_int.c pmu.c"

local ldscript = "sim3u1xx.ld"
 
-- Prepend with path
specific_files = fwlib_files .. " " .. utils.prepend_path( specific_files, sf( "src/platform/%s", platform ) )
specific_files = specific_files .. " src/platform/cortex_utils.s src/platform/arm_cortex_interrupts.c"
ldscript = sf( "src/platform/%s/%s", platform, ldscript )

addm{ "FOR" .. comp.cpu:upper(), 'gcc', 'CORTEX_M3' }

addm{ "__NEWLIB__" }

-- Standard GCC flags
addcf{ '-ffunction-sections', '-fdata-sections', '-fno-strict-aliasing', '-Wall' }
addlf{ '-nostartfiles', '-nostdlib', '-T', ldscript, '-Wl,--gc-sections', '-Wl,--allow-multiple-definition' }
addaf{ '-x', 'assembler-with-cpp', '-Wall' }
addlib{ 'c','gcc','m' }

local target_flags = { '-mcpu=cortex-m3','-mthumb' }

-- Configure general flags for target
addcf{ target_flags, '-mlittle-endian' }
addlf{ target_flags, '-Wl,-static', sf("-Wl,-Map=%s.map",output) }
addaf{ target_flags }
-- Toolset data
tools.sim3u1xx = {}

-- Array of file names that will be checked against the 'prog' target; their absence will force a rebuild
tools.sim3u1xx.prog_flist = { output .. ".hex" }

-- We use 'gcc' as the assembler
toolset.asm = toolset.compile

