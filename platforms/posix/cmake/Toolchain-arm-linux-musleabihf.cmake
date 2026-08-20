# arm-linux-musleabihf-gcc toolchain

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_VERSION 1)

if(NOT CMAKE_SYSTEM_PROCESSOR)
	set(CMAKE_SYSTEM_PROCESSOR arm)
endif()

set(triple arm-linux-musleabihf)
set(TOOLCHAIN_PREFIX ${triple})
set(CMAKE_LIBRARY_ARCHITECTURE ${triple})

set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}-gcc)
set(CMAKE_C_COMPILER_TARGET ${triple})

set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}-g++)
set(CMAKE_CXX_COMPILER_TARGET ${triple})

set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}-gcc)

# Load compiler flags for the architecture selected by the board configuration.
set(platform_file
	"${CMAKE_CURRENT_LIST_DIR}/Platform/${CMAKE_SYSTEM_NAME}-${triple}-${CMAKE_SYSTEM_PROCESSOR}.cmake"
)

if(EXISTS "${platform_file}")
	include("${platform_file}")
endif()

# Compiler tools
find_program(CMAKE_AR ${TOOLCHAIN_PREFIX}-gcc-ar REQUIRED)
find_program(CMAKE_GDB ${TOOLCHAIN_PREFIX}-gdb)
find_program(CMAKE_LD ${TOOLCHAIN_PREFIX}-ld REQUIRED)
find_program(CMAKE_LINKER ${TOOLCHAIN_PREFIX}-ld REQUIRED)
find_program(CMAKE_NM ${TOOLCHAIN_PREFIX}-gcc-nm REQUIRED)
find_program(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}-objcopy REQUIRED)
find_program(CMAKE_OBJDUMP ${TOOLCHAIN_PREFIX}-objdump REQUIRED)
find_program(CMAKE_RANLIB ${TOOLCHAIN_PREFIX}-gcc-ranlib REQUIRED)
find_program(CMAKE_STRIP ${TOOLCHAIN_PREFIX}-strip REQUIRED)

# Use headers and libraries from the musl toolchain, never from the build host.
execute_process(
	COMMAND ${CMAKE_C_COMPILER} -print-sysroot
	OUTPUT_VARIABLE toolchain_sysroot
	OUTPUT_STRIP_TRAILING_WHITESPACE
	RESULT_VARIABLE toolchain_sysroot_result
)

if(NOT toolchain_sysroot_result EQUAL 0 OR NOT toolchain_sysroot OR toolchain_sysroot STREQUAL "/")
	message(FATAL_ERROR "${CMAKE_C_COMPILER} did not report a usable sysroot")
endif()

set(CMAKE_SYSROOT ${toolchain_sysroot})
set(CMAKE_FIND_ROOT_PATH ${toolchain_sysroot})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Host tools
foreach(tool grep make)
	string(TOUPPER ${tool} TOOL)
	find_program(${TOOL} ${tool})
	if(NOT ${TOOL})
		message(FATAL_ERROR "could not find ${tool}")
	endif()
endforeach()
