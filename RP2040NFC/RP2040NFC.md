# Challenger RP2040 NFC Board Documentation

## Overview

The Challenger RP2040 NFC board is a microcontroller development board manufactured by Invector Labs that combines the power of the Raspberry Pi RP2040 microcontroller with NXP's PN7150 NFC controller. This board provides a complete NFC solution in an Adafruit Feather form factor, making it ideal for projects requiring Near Field Communication capabilities.

## Hardware Specifications

### Main Components
- **Microcontroller**: Raspberry Pi RP2040 dual-core ARM Cortex-M0+
- **NFC Controller**: NXP PN7150 with integrated firmware
- **Form Factor**: Adafruit Feather compatible
- **NFC Antenna**: High-performance integrated antenna
- **Power Supply**: USB or battery powered (2.3V to 5.5V range)

### RP2040 Features
- Dual-core ARM Cortex-M0+ processors running at 133MHz
- 264KB SRAM
- 2MB-16MB Flash memory (board dependent)
- 30 GPIO pins
- 2 × UART, 2 × SPI, 2 × I2C controllers
- 16 × PWM channels
- USB 1.1 Device and Host support
- 8 × Programmable I/O (PIO) state machines

### PN7150 NFC Controller Features
- Full NFC Forum compliance
- Embedded ARM Cortex-M0 microcontroller with integrated firmware
- NCI 1.0 host communication protocol
- I2C interface for host communication
- Ultra-low power consumption
- Integrated power management unit (PMU)
- Direct battery connection support

## Supported NFC Protocols

The PN7150 controller supports a comprehensive range of NFC protocols:

### Reader/Writer Mode
- **ISO/IEC 14443A**: MIFARE Classic, MIFARE Ultralight, NTAG series
- **ISO/IEC 14443B**: Standard Type B cards
- **ISO/IEC 15693**: Vicinity cards (ICODE series)
- **FeliCa**: Sony FeliCa cards
- **NFC Forum Tags**: Types 1-5 (T1T through T5T)

### Peer-to-Peer Mode
- **NFCIP-1**: ISO/IEC 18092 protocol
- **NFCIP-2**: Enhanced peer-to-peer communication
- **Active/Passive modes**: Both initiator and target roles
- **Data rates**: 106, 212, 424 kbps

### Card Emulation Mode
- **Type 4A/4B emulation**: ISO-DEP compatible
- **MIFARE Classic emulation**: 1K/4K support
- **Custom protocol emulation**: Via NCI extensions

## Pin Configuration

### Standard RP2040 Pins
```
GPIO0-GPIO29: General purpose I/O pins
3V3: 3.3V power output
GND: Ground connections
VBUS: USB bus voltage (5V when connected)
VSYS: System voltage input
EN: Enable pin (reset when pulled low)
```

### NFC-Specific Pins
```
PIN_PN7150_IRQ_B: Interrupt request from PN7150 (active high)
PIN_PN7150_RST_B: Reset pin for PN7150 (active low)
VEN: Voltage enable for PN7150
I2C SDA/SCL: Communication lines (Wire1 interface)
```

### I2C Configuration
- **Default Address**: 0x50-0x57 (configurable via hardware pins)
- **Interface**: Wire1 (secondary I2C bus)
- **Speed**: Up to 3.4MHz (High-speed mode)

## Arduino IDE Setup

### Board Installation
1. Open Arduino IDE
2. Go to File → Preferences
3. Add this URL to Additional Boards Manager URLs:
   ```
   https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
   ```
4. Go to Tools → Board → Boards Manager
5. Search for "pico" and install "Raspberry Pi Pico/RP2040"
6. Select "Invector Labs Challenger RP2040 NFC" from the board list

### Required Libraries
Install these libraries through the Library Manager:
- **Electroniccats_PN7150**: NFC controller driver
- **Adafruit_NeoPixel**: For onboard RGB LED control

## Basic Code Structure

### Library Includes and Setup
```cpp
#include "Electroniccats_PN7150.h"
#include <Adafruit_NeoPixel.h>

// NeoPixel configuration
Adafruit_NeoPixel pixels(1, NEOPIXEL, NEO_GRB + NEO_KHZ800);

// Color definitions
uint8_t colors[3][3] = { 
    { 15, 0, 0 },   // Red
    { 0, 10, 0 },   // Green  
    { 0, 0, 20 }    // Blue
};

#define RGB_RED 0
#define RGB_GREEN 1
#define RGB_BLUE 2
#define BUZZER_PIN D13

// Create NFC interface object
Electroniccats_PN7150 nfc(PIN_PN7150_IRQ_B, PIN_PN7150_RST_B, PN7150_I2C_ADDR, &Wire1);

// Interface for tag data
RfIntf_t RfInterface;
uint8_t mode = 1; // 1=Reader/Writer, 2=Emulation, 3=P2P
```

### Essential Functions

#### LED Control Function
```cpp
void set_rgb_led(int color) {
    pixels.setPixelColor(0, pixels.Color(
        colors[color][0], 
        colors[color][1], 
        colors[color][2]
    ));
    pixels.show();
}
```

#### NFC Reset and Configuration
```cpp
void ResetMode() {
    Serial.println("Re-initializing...");
    nfc.ConfigMode(mode);
    nfc.StartDiscovery(mode);
}
```

#### Data Buffer Printing
```cpp
void PrintBuf(const byte * data, const uint32_t numBytes) {
    for (uint32_t i = 0; i < numBytes; i++) {
        Serial.print(F("0x"));
        if (data[i] <= 0xF) Serial.print(F("0"));
        Serial.print(data[i], HEX);
        if (i != numBytes - 1) Serial.print(F(" "));
    }
    Serial.println();
}
```

## Complete Basic Example

### Setup Function
```cpp
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    pinMode(BUZZER_PIN, OUTPUT);
    
    delay(10);
    Serial.begin(115200);
    
    // Initialize NeoPixel
    pixels.begin();
    pixels.clear();
    set_rgb_led(RGB_RED);
    
    Serial.println("Detect NFC tags with PN7150");
    Serial.println("Initializing...");
    
    // Initialize NFC controller
    if (nfc.connectNCI()) {
        Serial.println("Error while setting up the mode, check connections!");
        while (1);
    }
    
    if (nfc.ConfigureSettings()) {
        Serial.println("The Configure Settings is failed!");
        while (1);
    }
    
    if (nfc.ConfigMode(mode)) {
        Serial.println("The Configure Mode is failed!!");
        while (1);
    }
    
    Serial.println("Initialization succeeded!");
    
    if (nfc.StartDiscovery(mode) == SUCCESS) {
        Serial.println("Successfully started discovery mode!");
    } else {
        Serial.println("Failed to start discovery mode!");
        while(1);
    }
    
    Serial.println("Waiting for a Card...");
    set_rgb_led(RGB_GREEN);
}
```

### Main Loop Function
```cpp
void loop() {
    if (!nfc.WaitForDiscoveryNotification(&RfInterface)) {
        set_rgb_led(RGB_BLUE);
        digitalWrite(LED_BUILTIN, HIGH);
        
        displayCardInfo(RfInterface);
        
        // Process based on detected protocol
        switch(RfInterface.Protocol) {
            case PROT_T1T:
            case PROT_T2T:
            case PROT_T3T:
            case PROT_ISODEP:
                nfc.ProcessReaderMode(RfInterface, READ_NDEF);
                break;
            case PROT_ISO15693:
                // Special handling for ISO15693 if needed
                break;
            case PROT_MIFARE:
                nfc.ProcessReaderMode(RfInterface, READ_NDEF);
                break;
            default:
                break;
        }
        
        // Provide audio feedback
        tone(BUZZER_PIN, 2000, 100);
        
        // Handle multiple tags
        if (RfInterface.MoreTags) {
            nfc.ReaderActivateNext(&RfInterface);
        }
        
        // Wait for card removal
        nfc.ProcessReaderMode(RfInterface, PRESENCE_CHECK);
        Serial.println("CARD REMOVED!");
        
        // Reset to discovery mode
        set_rgb_led(RGB_GREEN);
        digitalWrite(LED_BUILTIN, LOW);
        nfc.StopDiscovery();
        nfc.StartDiscovery(mode);
    }
    
    ResetMode();
    delay(50);
}
```

## Advanced Features

### Card Information Display
```cpp
void displayCardInfo(RfIntf_t RfIntf) {
    char tmp[16];
    
    switch(RfIntf.Protocol) {
        case PROT_T1T:
        case PROT_T2T:
        case PROT_T3T:
        case PROT_ISODEP:
            Serial.print(" - POLL MODE: Remote activated tag type: ");
            Serial.println(RfIntf.Protocol);
            break;
        case PROT_ISO15693:
            Serial.println(" - POLL MODE: Remote ISO15693 card activated");
            break;
        case PROT_MIFARE:
            Serial.println(" - POLL MODE: Remote MIFARE card activated");
            break;
        default:
            Serial.println(" - POLL MODE: Undetermined target");
            return;
    }
    
    // Display technology-specific information
    switch(RfIntf.ModeTech) {
        case (MODE_POLL | TECH_PASSIVE_NFCA):
            Serial.print("\tSENS_RES = ");
            sprintf(tmp, "0x%.2X", RfIntf.Info.NFC_APP.SensRes[0]);
            Serial.print(tmp); Serial.print(" ");
            sprintf(tmp, "0x%.2X", RfIntf.Info.NFC_APP.SensRes[1]);
            Serial.println(tmp);
            
            Serial.print("\tNFCID = ");
            PrintBuf(RfIntf.Info.NFC_APP.NfcId, RfIntf.Info.NFC_APP.NfcIdLen);
            
            if(RfIntf.Info.NFC_APP.SelResLen != 0) {
                Serial.print("\tSEL_RES = ");
                sprintf(tmp, "0x%.2X", RfIntf.Info.NFC_APP.SelRes[0]);
                Serial.println(tmp);
            }
            break;
            
        case (MODE_POLL | TECH_PASSIVE_15693):
            Serial.print("\tID = ");
            PrintBuf(RfIntf.Info.NFC_VPP.ID, sizeof(RfIntf.Info.NFC_VPP.ID));
            Serial.print("\tAFI = ");
            Serial.println(RfIntf.Info.NFC_VPP.AFI);
            Serial.print("\tDSFID = ");
            Serial.println(RfIntf.Info.NFC_VPP.DSFID, HEX);
            break;
            
        default:
            break;
    }
}
```

### Custom Data Reading
```cpp
bool readCustomData(uint8_t* buffer, uint16_t maxSize, uint16_t* actualSize) {
    uint8_t command[16];
    uint8_t response[256];
    uint8_t responseSize;
    
    // Example: Read NDEF message
    if (nfc.readerTagCmd(command, sizeof(command), response, &responseSize)) {
        if (responseSize <= maxSize) {
            memcpy(buffer, response, responseSize);
            *actualSize = responseSize;
            return true;
        }
    }
    return false;
}
```

### Writing Data to Tags
```cpp
bool writeCustomData(uint8_t* data, uint16_t dataSize) {
    uint8_t command[256];
    uint8_t response[256];
    uint8_t responseSize;
    
    // Prepare write command
    command[0] = 0xA2; // Write command for NTAG
    command[1] = 0x04; // Starting page
    memcpy(&command[2], data, min(dataSize, 4)); // Max 4 bytes per write
    
    return nfc.readerTagCmd(command, 6, response, &responseSize);
}
```

## Power Management

### Low Power Configuration
```cpp
void enterLowPowerMode() {
    // Stop discovery to save power
    nfc.StopDiscovery();
    
    // Configure for standby mode
    uint8_t standbyCmd[] = {0x2F, 0x00, 0x01, 0x01};
    nfc.writeData(standbyCmd, sizeof(standbyCmd));
    
    // Put RP2040 into sleep mode
    // (Implementation depends on specific power requirements)
}
```

### Wake-up Handling
```cpp
void handleWakeUp() {
    // Re-initialize NFC controller
    if (nfc.connectNCI() == SUCCESS) {
        nfc.ConfigureSettings();
        nfc.ConfigMode(mode);
        nfc.StartDiscovery(mode);
    }
}
```

## Troubleshooting

### Common Issues and Solutions

1. **"Error while setting up the mode"**
   - Check I2C connections (SDA, SCL)
   - Verify power supply (3.3V stable)
   - Ensure VEN pin is properly controlled

2. **No tag detection**
   - Check antenna positioning
   - Verify supported tag types
   - Try different NFC tags for testing

3. **I2C communication failures**
   - Check pull-up resistors on I2C lines
   - Verify I2C address configuration
   - Test I2C bus with scanner code

4. **Compilation errors**
   - Ensure correct board selection in Arduino IDE
   - Verify all required libraries are installed
   - Check for conflicting library versions

### Debug Configuration
```cpp
// Enable debug output
#define DEBUG
#define DEBUG2

// Add debug prints in critical sections
#ifdef DEBUG
Serial.println("[DEBUG] NFC initialization started");
#endif
```

## Performance Characteristics

### Operating Ranges
- **Communication Distance**: Up to 4cm (depending on tag type)
- **Data Transfer Speed**: 106-424 kbps
- **Power Consumption**: 
  - Active mode: ~15-25mA
  - Standby mode: ~2-5µA
  - Deep sleep: <1µA

### Memory Usage
- **Flash**: ~50-100KB (depending on features used)
- **RAM**: ~10-20KB (including buffers)

## Integration Examples

### PIC Programming Application
```cpp
// Custom function for PIC16F programming via NFC
bool processPICFirmware(uint8_t* nfcData, uint16_t dataLength) {
    // Validate firmware data
    if (!validateFirmwareData(nfcData, dataLength)) {
        return false;
    }
    
    // Extract programming data
    uint8_t* firmwareData = extractFirmware(nfcData);
    uint16_t firmwareSize = getFirmwareSize(nfcData);
    
    // Program PIC16F via ICSP
    return programPIC16F(firmwareData, firmwareSize);
}
```

## Resources and References

### Documentation Links
- [PN7150 User Manual](https://www.nxp.com/docs/en/user-guide/UM10935.pdf)
- [RP2040 Datasheet](https://datasheets.raspberrypi.org/rp2040/rp2040-datasheet.pdf)
- [Arduino-Pico Documentation](https://arduino-pico.readthedocs.io/)
- [Electronic Cats PN7150 Library](https://github.com/ElectronicCats/ElectronicCats-PN7150)

### Community Support
- [Arduino Forum](https://forum.arduino.cc/)
- [Raspberry Pi Pico Community](https://www.raspberrypi.org/forums/)
- [Electronic Cats Discord](https://electroniccats.com/discord/)

### Example Projects
- NFC-enabled IoT sensors
- Contactless payment systems
- Access control applications
- Data logging with NFC tags
- Peer-to-peer data exchange

## Conclusion

The Challenger RP2040 NFC board provides a powerful and flexible platform for NFC-enabled applications. Its combination of the capable RP2040 microcontroller and the feature-rich PN7150 NFC controller makes it suitable for both simple tag reading applications and complex NFC system implementations.

The comprehensive library support and Arduino IDE compatibility ensure rapid prototyping and development, while the extensive protocol support enables integration with virtually any NFC-enabled device or system.