# ESP8266 NON_SDK NON_OS

Unofficial documentation and development framework for ESP8266 ROM functions. This project provides low-level access to the ESP8266's built-in ROM routines, enabling bare-metal development without the official SDK or RTOS overhead.

## Overview

The ESP8266 contains a significant amount of functionality in its mask ROM, including:
- **PHY/RF control functions** - Low-level Wi-Fi hardware configuration
- **Calibration routines** - TX power, DC offset, IQ imbalance compensation
- **Hardware abstraction** - UART, timers, interrupts, memory management
- **System utilities** - Delay functions, exception handlers, bootloader support

This repository documents these ROM functions and provides a minimal framework for direct ROM-based development.
The entire documentation and the function signatures are in the <a href="/lib/esp8266.h">esp8266.h</a> file. Which also means that it is a single header file library, making it easy to use.

## Project Status

🚧 **Work in Progress** 🚧

Currently documenting and reverse-engineering ROM functions. Contributions welcome!

### Completed
- ETS Espressif's internal library (`ets_*` functions)
- Memory management functions (`mem*` functions)
- Exception/interrupt handlers (`_xtos_*` functions)

### In Progress
- Complete ROM function header file
- Working examples demonstrating ROM usage
- Documentation for all discovered functions

## Repository Structure

```
esp8266_NON_SDK_NON_OS/
├── lib/
│   └── esp8266.h               # Documented ROM function declarations and ESP8266 SoC register definitions (from Espressif)
├── ld/
│   ├── addr.ld                 # ROM function addresses
│   └── app.ld                  # Application memory layout
├── user/
│   └── main.c                  # User application code
├── start.c                     # Basic system initialization
├── Makefile                    # Build, flash, and monitor automation
├── build_toolchain.py          # Toolchain setup script
├── tests/                      # Tests folder
└── README.md                   # This file
```

## Prerequisites

- **ESP8266 Xtensa toolchain** (`xtensa-lx106-elf-gcc`)
- **esptool.py** for flashing
- **Python 3.x**
- Serial terminal (optional, for monitoring)

## Documentation Instructions

Every documented functions follows this pattern:
```c
/* Name: <function name>
 * Address: <function address in ROM>
 * Description: <function description>
 * <more detailed function description>
 * */
```
This pattern facilitates quick lookups to a function's documentation (i.e. in vim, "/Name: \<function name\>").
It is also possible to find between the Description and the details a series of the following:
```c
/* ...
 * Note: SA stands for Some Abbreviation. Optional short description of the abbreviation.
 * ...
 * */
```
Meaning that if you do not understand an abbreviation in a function's documentation you can search for that pattern.

## Quick Start

### 1. Install Toolchain

*Toolchain script does not exist yet.*

```bash
python3 build_toolchain.py
```

This script will download and configure the Xtensa toolchain for ESP8266 development.

### 2. Build

```bash
make
```

### 3. Flash

```bash
make flash
```

### 4. Monitor

Use a serial terminal:
```bash
screen /dev/ttyUSB0 115200
# or
minicom -D /dev/ttyUSB0 -b 115200
# or
arduino-cli monitor -p /dev/ttyUSB0 -b 115200
```

## Example Usage

```c
#include "esp8266.h"

void user_init(void) {
    // Initialize memory allocator
    extern uint8_t _heap_start;
    mem_init(&_heap_start);
    
    // Initialize UART for debug output
    uart_div_modify(0, UART_CLK_FREQ / 115200);
    
    // Basic PHY initialization
    rom_chip_v5_rx_init();
    rom_chip_50_set_channel(6, 0, 1, 0, 0, 0, 0);
    
    // Perform calibration
    uint8_t cal_results[2];
    rom_cal_tos_v50(1, 0, 100, 10, cal_results);
    
    ets_printf("Calibration complete: I=%d, Q=%d\n", 
               cal_results[0], cal_results[1]);
}
```

## Key Concepts

### What is ROM?
The ESP8266's mask ROM contains ~64KB of pre-programmed functions that cannot be modified. These functions provide:
- **Space efficiency** - No need to include code in flash
- **Proven stability** - Factory-tested implementations
- **Direct hardware access** - Intimate knowledge of chip internals

### Why Use ROM Functions?
- **Minimal footprint** - Reduce flash usage
- **Maximum performance** - No SDK overhead
- **Educational value** - Understand chip architecture
- **Special applications** - Custom protocols, research, embedded systems

### Limitations
- **No official support** - Reverse-engineered, limited documentation
- **API instability** - May vary between chip revisions
- **Complexity** - Requires deep hardware knowledge
- **Limited abstraction** - More low-level than SDK

## Technical Details

### Xtensa Architecture
The ESP8266 uses a Tensilica Xtensa LX106 processor:
- 32-bit RISC architecture
- Windowed register file (64 registers, 16 visible)
- Custom instruction set extensions
- Little-endian byte order

### Function Calling Convention
- Arguments: `a2-a7` (up to 6 args)
- Return value: `a2`
- Stack grows downward
- Callee-saved: `a12-a15`
- Caller-saved: `a2-a11`

### Memory Map
```
0x40000000 - 0x40010000  : ROM (64KB)
0x3FFE8000 - 0x40000000  : DRAM (96KB)
0x40100000 - 0x40140000  : IRAM (256KB)
0x40200000 - 0x40300000  : Flash mapping (1MB window)
```

## Contributing

Contributions are highly encouraged! Ways to help:

1. **Document unknown functions** - Reverse-engineer and document ROM routines
2. **Add examples** - Demonstrate ROM function usage
3. **Improve tooling** - Enhance build scripts and utilities
4. **Fix bugs** - Correct errors in documentation or code
5. **Test on hardware** - Verify functions on different ESP8266 variants

### Contribution Guidelines
- Follow the <a href="#documentation-instructions">Documentation Instructions</a>
- Test on real hardware when possible
- Use clear, descriptive commit messages

## Resources

- [ESP8266 Technical Reference](https://www.espressif.com/sites/default/files/documentation/esp8266-technical_reference_en.pdf)
- [Xtensa ISA Summary](https://0x04.net/~mwk/doc/xtensa.pdf)
- [ESP8266 Community Forum](https://www.esp8266.com/)
- [Espressif GitHub](https://github.com/espressif)

## Disclaimer

This project is **not affiliated with or endorsed by Espressif Systems**. Use of undocumented ROM functions is at your own risk. The ROM API may change without notice in future chip revisions.

## License

Documentation and code in this repository are provided under the MIT License. See LICENSE file for details.

ROM functions themselves are © Espressif Systems and subject to their licensing terms.

## Acknowledgments

- Espressif Systems - for creating the ESP8266
- The [sheinz/esp-open-bootrom](https://github.com/sheinz/esp-open-bootrom) repo for the reverse engineering done on the assembly.
- The [cnlohr/nosdk8266](https://github.com/cnlohr/nosdk8266) repo for the inspiration of not using any SDK.

---

**Status**: Alpha - Actively documenting ROM functions. API subject to change.

**Last Updated**: 2025
