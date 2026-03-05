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
    # mcpu=cortex-a53 already implies the correct march — do not set both.
    set(BEEPI_ARCH_FLAGS "-mcpu=cortex-a53 -mfpu=neon-fp-armv8 -mfloat-abi=hard -O2")
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
# Example: testpattern2
# ---------------------------------------------------------------------------

add_executable(testpattern2
    example/testpattern2.cpp
)

target_include_directories(testpattern2 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/lib
)

target_link_libraries(testpattern2 PRIVATE
    beepi_gc9a01a_nt
)

# ---------------------------------------------------------------------------
# Image viewer: imgview
#
# Step 1 — Convert assets/logo.png → assets/logo_rgb565.h at build time.
#           Uses the pure-stdlib Python3 script png_to_rgb565.py.
#           Re-runs automatically whenever logo.png changes.
# Step 2 — Compile imgview.cpp which #includes the generated header.
# ---------------------------------------------------------------------------

find_package(Python3 QUIET COMPONENTS Interpreter)
if(NOT Python3_FOUND)
    message(WARNING
        "Python3 not found — imgview will not be built.\n"
        "Install with: sudo apt install python3")
else()
    set(LOGO_PNG    "${CMAKE_CURRENT_SOURCE_DIR}/assets/logo.png")
    set(LOGO_H      "${CMAKE_CURRENT_SOURCE_DIR}/assets/logo_rgb565.h")
    set(CONVERTER   "${CMAKE_CURRENT_SOURCE_DIR}/assets/png_to_rgb565.py")

    add_custom_command(
        OUTPUT  ${LOGO_H}
        COMMAND ${Python3_EXECUTABLE} ${CONVERTER} ${LOGO_PNG} ${LOGO_H} logo_rgb565
        DEPENDS ${LOGO_PNG} ${CONVERTER}
        COMMENT "Converting logo.png → logo_rgb565.h"
        VERBATIM
    )
    add_custom_target(logo_header DEPENDS ${LOGO_H})

    add_executable(imgview
        example/imgview.cpp
    )
    add_dependencies(imgview logo_header)

    target_include_directories(imgview PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/lib
        ${CMAKE_CURRENT_SOURCE_DIR}/assets
    )

    target_link_libraries(imgview PRIVATE
        beepi_gc9a01a_nt
    )

    message(STATUS "imgview: enabled  (logo: ${LOGO_PNG})")
endif()

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
message(STATUS "  sudo ./testpattern2")
message(STATUS "  -- or --")
message(STATUS "  sudo usermod -aG gpio $USER  (then re-login)")
message(STATUS "  ./testpattern  /  ./testpattern2")