# STEOlabs_Modbus_Swissknife
Modbus Swissknife
===

Modbus Swissknife is a universal decoder and analyzer for Modbus RTU protocols, specifically designed for ESP32 devices. The project was born from the need to reverse engineer "black box" industrial sensors where the register map or data format is unknown.

The main features are:

* Robust bus scanning: Automatically identifies IDs (1-247) and baud rates by testing common speeds (4800, 9600, 19200, 115200).
* 32-bit "Total Diagnostic" decoding: Ability to interpret two adjacent registers in all 4 combinations of IEEE 754 endianness (ABCD, DCBA, CDAB, BADC) simultaneously.
* Low-pressure Watch mode: A monitoring loop that polls the bus at a steady pace, showing raw RX frames to facilitate documentation.
* Raw memory inspection: A dump command to view registers in both hexadecimal and decimal (signed/unsigned) formats.
* Minimalism: The implementation targets maximum code readability, limiting dependencies to the ModbusMaster library for packet transport.

Installation
---

Type "platformio run --target upload" to flash the firmware to your ESP32. Ensure that the RS485 transceiver pins are correctly configured in the `config.h` file.

Normal usage
---

To capture traffic directly from the bus and identify connected devices, run the scan command:

    swissknife> scan 1 10

To analyze a specific register to identify 32-bit values (such as temperature or gas concentrations):

    swissknife> analyze <id> <baud> <reg>

To monitor a register in interactive mode, observing how bytes change in response to external stimuli:

    swissknife> watch <id> <baud> <reg>

In interactive mode, the screen is refreshed every few seconds displaying all possible data interpretations.

32-bit Data Analysis
---

While most of the Modbus specification is obvious, the byte order for 32-bit data is not. Swissknife shows every possible interpretation to avoid decoding errors:

* ABCD: Big Endian (Standard industrial format).
* DCBA: Little Endian.
* CDAB: Mid-Little (Word swap).
* BADC: Mid-Big (Byte swap).

Reliability
---

By default, the program displays raw data without applying filters or scaling. While this approach is reliable in my experience, I suggest always verifying the displayed TX/RX frames to confirm communication correctness.

How this program works?
---


The algorithms used for 32-bit decoding extract raw bytes from Modbus registers and map them in memory using `memcpy`, ensuring that the representation of floating-point numbers follows the IEEE 754 standard.

Credits
---

Modbus Swissknife was written by STEOlab is released under the BSD three-clause license.
