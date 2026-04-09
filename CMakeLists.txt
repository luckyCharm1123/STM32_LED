cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME LED)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

option(LORA_DEBUG_VERBOSE_BUILD "Enable verbose LoRa debug logs" OFF)
option(DEBUG_SEMIHOSTING_FALLBACK "Enable semihosting fallback in DEBUG_SendString" OFF)
option(ENABLE_RELEASE_LTO "Enable -flto for Release builds" ON)
set(APP_FLASH_BYTES 57344 CACHE STRING "Usable application FLASH bytes")
set(FLASH_BUDGET_RELEASE_PERCENT 60 CACHE STRING "Release FLASH budget percent")
set(FLASH_BUDGET_DEBUG_PERCENT 75 CACHE STRING "Debug FLASH budget percent")

get_filename_component(TOOLCHAIN_BIN_DIR "${CMAKE_C_COMPILER}" DIRECTORY)
set(SIZE_TOOL_EXEC "${CMAKE_SIZE}")
if(NOT IS_ABSOLUTE "${SIZE_TOOL_EXEC}")
    set(SIZE_TOOL_CANDIDATE "${TOOLCHAIN_BIN_DIR}/${SIZE_TOOL_EXEC}")
    if(EXISTS "${SIZE_TOOL_CANDIDATE}")
        set(SIZE_TOOL_EXEC "${SIZE_TOOL_CANDIDATE}")
    endif()
endif()

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    "Core/Src/ir_learner.c"
    "Core/Src/lora.c"
    "Core/Src/veml7700_soft.c"
    "Core/Src/RED_LED.c"
    "Core/Src/GREEN_LED.c"
    "Core/Src/BLUE_LED.c"
    "Core/Src/mhz19b_pwm.c"
    "Core/Src/sound_accumulator.c"
    "Core/Src/deferred_action.c"
    "Core/Src/state_sender.c"
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    $<$<BOOL:${LORA_DEBUG_VERBOSE_BUILD}>:LORA_DEBUG_VERBOSE=1>
    $<$<BOOL:${DEBUG_SEMIHOSTING_FALLBACK}>:DEBUG_SEMIHOSTING_FALLBACK=1>
)

if(CMAKE_C_COMPILER_ID STREQUAL "GNU" AND ENABLE_RELEASE_LTO)
    target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
        $<$<CONFIG:Release>:-flto>
    )
    target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
        $<$<CONFIG:Release>:-flto>
    )
endif()

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

add_custom_target(size-report
    COMMAND ${SIZE_TOOL_EXEC} -A $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
    DEPENDS ${CMAKE_PROJECT_NAME}
    COMMENT "Section size report for ${CMAKE_PROJECT_NAME}"
)

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND}
        -DELF_FILE=$<TARGET_FILE:${CMAKE_PROJECT_NAME}>
        -DSIZE_TOOL=${SIZE_TOOL_EXEC}
        -DC_COMPILER_PATH=${CMAKE_C_COMPILER}
        -DAPP_FLASH_BYTES=${APP_FLASH_BYTES}
        -DBUILD_CFG=$<CONFIG>
        -DRELEASE_BUDGET_PERCENT=${FLASH_BUDGET_RELEASE_PERCENT}
        -DDEBUG_BUDGET_PERCENT=${FLASH_BUDGET_DEBUG_PERCENT}
        -P ${CMAKE_SOURCE_DIR}/cmake/check_flash_budget.cmake
)
