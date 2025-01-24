set(CMAKE_VERBOSE_MAKEFILE on)
set(CMAKE_COLOR_MAKEFILE on)
set(CMAKE_ERROR_DEPRECATED on)

cmake_minimum_required(VERSION 3.6.0)
project("stm32f030" C ASM)

set(TARGET ${CMAKE_PROJECT_NAME})

set(PROCESSOR_FAMILY ARM_CM3)
set(STM32_FAMILY "F0")
string(TOLOWER ${STM32_FAMILY} STM32_FAMILY_LC)
set(MCU_NAME "STM32F030X6")
string(TOLOWER ${MCU_NAME} MCU_NAME_LC)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/ssd1306/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32${STM32_FAMILY}xx_HAL_Driver/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32${STM32_FAMILY}xx/Include/
    ${CMAKE_CURRENT_SOURCE_DIR}/app/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/board/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/common/inc
    )

set(SOURCES
   Drivers/CMSIS/Device/ST/STM32${STM32_FAMILY}xx/Source/Templates/gcc/startup_${MCU_NAME_LC}.s
   Drivers/CMSIS/Device/ST/STM32${STM32_FAMILY}xx/Source/Templates/system_stm32${STM32_FAMILY_LC}xx.c
   common/src/syscalls.c
   common/src/stm32fxxx_it.c
   Drivers/ssd1306/src/ssd1306.c
   )


include(Drivers/STM32${STM32_FAMILY}xx_HAL_Driver/CMakeLists.txt)
include(app/CMakeLists.txt)
include(board/CMakeLists.txt)

add_executable(${TARGET} ${SOURCES})

# Set a default build type if none was specified
set(default_build_type "Release")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(WARNING "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
      STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release")
endif()

SET(CMAKE_C_FLAGS "-mthumb -fno-builtin -mcpu=cortex-m0 -mfloat-abi=soft -Wall -DNDEBUG -std=gnu99 -ffunction-sections -fdata-sections -fomit-frame-pointer -mabi=aapcs -ffast-math -ftree-vectorize" CACHE INTERNAL "c compiler flags")
SET(CMAKE_ASM_FLAGS "-mthumb -mcpu=cortex-m0 -mfloat-abi=soft -Wa,--no-warn -Wall -DNDEBUG -x assembler-with-cpp" CACHE INTERNAL "asm compiler flags")

SET(CMAKE_EXE_LINKER_FLAGS "-Wl,--gc-sections -mthumb --specs=nano.specs --specs=nosys.specs -Wl,-Map,\"${TARGET}.map\"" CACHE INTERNAL "executable linker flags")

IF(NOT CROSS_COMPILE_PREFIX)
    SET(CROSS_COMPILE_PREFIX "arm-none-eabi-")
    MESSAGE(WARNING "No CROSS_COMPILE_PREFIX specified, using default: " ${CROSS_COMPILE_PREFIX})
ENDIF()

SET(CMAKE_C_COMPILER "${CROSS_COMPILE_PREFIX}gcc")
set(CMAKE_ASM_COMPILER "${CROSS_COMPILE_PREFIX}gcc")
SET(CMAKE_OBJCOPY "${CROSS_COMPILE_PREFIX}objcopy" CACHE INTERNAL "objcopy tool")
SET(CMAKE_OBJDUMP "${CROSS_COMPILE_PREFIX}objdump" CACHE INTERNAL "objdump tool")
SET(CMAKE_SIZE "${CROSS_COMPILE_PREFIX}size" CACHE INTERNAL "size tool")

SET(CMAKE_C_FLAGS_DEBUG "-Og -g" CACHE INTERNAL "c compiler flags debug")
SET(CMAKE_ASM_FLAGS_DEBUG "-g" CACHE INTERNAL "asm compiler flags debug")
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "" CACHE INTERNAL "linker flags debug")

SET(CMAKE_C_FLAGS_RELEASE "-O2 -flto" CACHE INTERNAL "c compiler flags release")
SET(CMAKE_ASM_FLAGS_RELEASE "" CACHE INTERNAL "asm compiler flags release")
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "-flto" CACHE INTERNAL "linker flags release")

CONFIGURE_FILE(Drivers/CMSIS/Device/ST/STM32${STM32_FAMILY}xx/Source/Templates/gcc/linker/${MCU_NAME}_FLASH.ld ${CMAKE_CURRENT_BINARY_DIR}/${TARGET}_flash.ld)

GET_TARGET_PROPERTY(TARGET_LD_FLAGS ${TARGET} LINK_FLAGS)
IF(TARGET_LD_FLAGS)
    SET(TARGET_LD_FLAGS "\"-T${CMAKE_CURRENT_BINARY_DIR}/${TARGET}_flash.ld\" ${TARGET_LD_FLAGS}")
ELSE()
    SET(TARGET_LD_FLAGS "\"-T${CMAKE_CURRENT_BINARY_DIR}/${TARGET}_flash.ld\"")
ENDIF()
SET_TARGET_PROPERTIES(${TARGET} PROPERTIES LINK_FLAGS ${TARGET_LD_FLAGS})

SET(FILENAME "${TARGET}")
ADD_CUSTOM_TARGET(${TARGET}.bin ALL DEPENDS ${TARGET} COMMAND ${CMAKE_OBJCOPY} -Obinary ${FILENAME} ${FILENAME}.bin)
add_custom_command(TARGET ${TARGET} POST_BUILD COMMAND ${CMAKE_SIZE} ${FILENAME})
