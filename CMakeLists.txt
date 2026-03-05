cmake_minimum_required(VERSION 3.16)
project(beepi_gc9a01a_nt CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# Architecture / platform detection
# ---------------------------------------------------------------------------
# CMake target architecture  (set automatically or override with -DCMAKE_SYSTEM_PROCESSOR)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    message(STATUS "Target: 64-bit ARM (aarch64)")
    set(BEEPI_ARCH_64 TRUE)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "armv8|armhf|arm")
    message(STATUS "Target: 32-bit ARM (armhf / ARMv8 running 32-bit OS)")
    set(BEEPI_ARCH_32 TRUE)
else()
    message(WARNING "Unknown processor: ${CMAKE_SYSTEM_PROCESSOR} — building without ARM-specific flags")
endif()

# ---------------------------------------------------------------------------
# Compiler flags
# ---------------------------------------------------------------------------

set(BEEPI_COMMON_FLAGS "-Wall -Wextra -Wno-unused-parameter")

if(BEEPI_ARCH_64)
    # All 64-bit Pi models: Raspberry Pi Zero 2W / 4B / 5
    # -mcpu=native  — auto-detects the actual Cortex-A72/A76/A53 etc.
    # REV16 is available from ARMv6T2 onward, so __builtin_bswap16 is free.
    set(BEEPI_ARCH_FLAGS "-mcpu=native -O2")
elseif(BEEPI_ARCH_32)
    # Raspberry Pi 3B+ running 32-bit Raspberry Pi OS (ARMv8 in 32-bit mode)
    # Cortex-A53 with NEON — safe and compatible across all 32-bit Pi OS
    set(BEEPI_ARCH_FLAGS "-march=armv8-a -mcpu=cortex-a53 -mfpu=neon-fp-armv8 -mfloat-abi=hard -O2")
else()
    set(BEEPI_ARCH_FLAGS "-O2")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${BEEPI_COMMON_FLAGS} ${BEEPI_ARCH_FLAGS}")

# ---------------------------------------------------------------------------
# Find lgpio
# ---------------------------------------------------------------------------

find_library(LGPIO_LIB lgpio
    HINTS /usr/lib /usr/local/lib /usr/lib/aarch64-linux-gnu /usr/lib/arm-linux-gnueabihf
)

if(NOT LGPIO_LIB)
    message(FATAL_ERROR
        "liblgpio not found.\n"
        "Install with:  sudo apt install liblgpio-dev\n"
        "If on Pi 5 Bookworm it should already be present.\n"
    )
endif()
message(STATUS "Found lgpio: ${LGPIO_LIB}")

# ---------------------------------------------------------------------------
# beepi_gc9a01a_nt static library
# ---------------------------------------------------------------------------

add_library(beepi_gc9a01a_nt STATIC
    lib/beepi_gc9a01a_nt.cpp
    lib/hal/beepi_hal_linux.cpp
)

target_include_directories(beepi_gc9a01a_nt PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/lib
)

target_link_libraries(beepi_gc9a01a_nt PUBLIC
    ${LGPIO_LIB}
    m       # libm for sinf/cosf/sqrtf
)

# ---------------------------------------------------------------------------
# Example: testpattern
# ---------------------------------------------------------------------------

add_executable(testpattern
    example/testpattern.cpp
)

target_include_directories(testpattern PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/lib
)

target_link_libraries(testpattern PRIVATE
    beepi_gc9a01a_nt
)

# ---------------------------------------------------------------------------
# Install targets (optional)
# ---------------------------------------------------------------------------

install(TARGETS beepi_gc9a01a_nt ARCHIVE DESTINATION lib)
install(FILES
    lib/beepi_gc9a01a_nt.h
    lib/hal/beepi_hal.h
    DESTINATION include/beepi_gc9a01a_nt
)

# ---------------------------------------------------------------------------
# Print config summary
# ---------------------------------------------------------------------------

message(STATUS "")
message(STATUS "BeeBotix GC9A01A build configuration")
message(STATUS "  Processor  : ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "  Arch flags : ${BEEPI_ARCH_FLAGS}")
message(STATUS "  lgpio      : ${LGPIO_LIB}")
message(STATUS "  Build type : ${CMAKE_BUILD_TYPE}")
message(STATUS "")
message(STATUS "To build:")
message(STATUS "  mkdir build && cd build")
message(STATUS "  cmake ..")
message(STATUS "  make -j$(nproc)")
message(STATUS "")
message(STATUS "To run (GPIO needs root or gpio group):")
message(STATUS "  sudo ./testpattern")
message(STATUS "  -- or --")
message(STATUS "  sudo usermod -aG gpio $USER  (then re-login)")
message(STATUS "  ./testpattern")