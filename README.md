# Active-RFID-with-CC1200-and-MSP430
In this project, measurements of 5 sensors are put in the payload of an active RFID system connected to Mega2560. In this system, the user has full control of register configurations. The register configuration is facilitated by SmartRF Studio. 


The user can change the carrier frequency, packet length, etc. Multiple scenarios can be implemented with this system. The physical layer is CC1200. A majority of wireless sensor modules need two-sided communication and this system enables the bidirectional communication.

**1- CC1200**

The CC1200 device is a fully integrated single-chip radio transceiver designed for high performance at
very low-power and low-voltage operation in cost-effective wireless systems. All filters are integrated, thus
removing the need for costly external SAW and IF filters. The device is mainly intended for the ISM
(Industrial, Scientific, and Medical) and SRD (Short Range Device) frequency bands at 164–190 MHz,
410–475 MHz, and 820–950 MHz.
The CC1200 device provides extensive hardware support for packet handling, data buffering, burst
transmissions, clear channel assessment, link quality indication, and Wake-On-Radio. The main operating
parameters of the CC1200 device can be controlled through an SPI interface. In a typical system, the
CC1200 device will be used with a microcontroller and only a few external passive components.

The CC1200 and the CC1120 devices are both part of the high-performance transceiver family. The
CC1120 device is more optimized toward narrowband applications, while the CC1200 device is optimized
toward wideband applications but can also effectively cover narrowband down to 12.5-kHz channels.

**2- MSP430**

The Mega 2560 is a microcontroller board based on the ATmega2560. It has 54 digital input/output pins (of which 15 can be used as PWM outputs), 16 analog inputs, 4 UARTs (hardware serial ports), a 16 MHz crystal oscillator, a USB connection, a power jack, an ICSP header, and a reset button. It contains everything needed to support the microcontroller; simply connect it to a computer with a USB cable or power it with a AC-to-DC adapter or battery to get started. The Mega 2560 board is compatible with most shields designed for the Uno and the former boards Duemilanove or Diecimila.
