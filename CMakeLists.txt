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
set(CMAKE_PROJECT_NAME h7_frame)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Shared compiler and generated HAL context. This target exports no user-layer headers.
add_library(h7_platform INTERFACE)
target_link_libraries(h7_platform INTERFACE stm32cubemx)
target_compile_options(h7_platform INTERFACE
    $<$<COMPILE_LANGUAGE:C>:-include>
    $<$<COMPILE_LANGUAGE:C>:${CMAKE_CURRENT_SOURCE_DIR}/cmake/legacy_compat.h>
)
target_compile_definitions(h7_platform INTERFACE ARM_MATH_CM7)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Shared fixed-width and control-value types; no hardware implementation.
add_library(h7_types INTERFACE)
target_include_directories(h7_types INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/model/common)

add_subdirectory(bsp)
add_subdirectory(model)
add_subdirectory(device)
add_subdirectory(app)

# Implementation dependency: app -> model -> device -> bsp -> generated HAL.
# The reverse-looking device -> model_api edge carries declarations only.
target_link_libraries(h7_model PRIVATE h7_device)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    PRIVATE h7_platform h7_types
    # Device implements model ports; rescan resolves both directions.
    "-Wl,--start-group"
    h7_app h7_model h7_device h7_bsp
    "-Wl,--end-group"

    m
)
