# US-2000

A project focused on modifying and enhancing the US-2000 Pro Ultrasound device through custom firmware and hardware modifications.

## Description

This project aims to modify and enhance the US-2000 Pro Ultrasound device by leveraging the internal microcontroller chips. The project utilizes the GD32F130C8T6 microcontroller and Raspberry Pi RP2040 with NFC capabilities found within these devices.

## Key Components

### GD32 Microcontroller Documentation
The [`GD32/GD32.md`](GD32/GD32.md) file contains a comprehensive reference guide covering:
- **Complete Hardware Setup** - ST-LINK V2 wiring with critical VAPP connection details
- **Custom OpenOCD Configuration** - The `gd32f1x.cfg` target file developed for this project
- **All Working Commands** - Tested OpenOCD and telnet commands from troubleshooting sessions
- **Raspberry Pi Integration** - GPIO connections, Python examples, and SWD implementation
- **Troubleshooting Guide** - Solutions for all encountered issues
- **Memory Maps** - Complete GD32F130C8T6 memory layout
- **ARM Toolchain Integration** - Complete setup for compilation and analysis
- **Flash Operations** - Including `flash read_bank 0 flash.bin 0 0x10000` for extraction and programming identical chips
- **Device Specifics** - Key differences from STM32F103C8T6 (8KB vs 20KB SRAM, device ID 0x13030410)

### RP2040 NFC Board Documentation
The [`RP2040NFC/RP2040NFC.md`](RP2040NFC/RP2040NFC.md) file provides comprehensive documentation for the Challenger RP2040 NFC board:
- **Hardware Overview** - RP2040 and PN7150 specifications
- **Supported NFC Protocols** - All tag types and communication modes
- **Arduino IDE Setup** - Step-by-step installation instructions
- **Complete Code Examples** - Working code snippets for direct use
- **Advanced Features** - Custom data reading/writing functions
- **Troubleshooting Guide** - Common issues and solutions
- **Performance Specs** - Operating ranges and memory usage
- **Practical Examples** - NFC tag detection, LED control, card information display, power management

## Features

- Custom firmware development for GD32F130C8T6 microcontroller
- NFC integration using RP2040 with PN7150 chipset
- Hardware modification guides and documentation
- Clone-to-clone transfer capabilities
- Comprehensive troubleshooting and setup guides

## Requirements

- ARM GCC toolchain
- OpenOCD for debugging and flashing
- ST-LINK V2 or compatible debugger
- Raspberry Pi (for integration projects)
- Arduino IDE (for RP2040 development)
- GD32F130C8T6 and RP2040 development boards

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

## License

This project is licensed under the MIT License.
