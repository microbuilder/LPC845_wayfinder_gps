# LPC845 GPS Wayfinder Project

This project uses a standard NMEA-base GPS modules to track the current
position, and determine the current position's distance from a pre-defined
destination position (defined in degrees latitude and longitude).

The design can be used as a simple treasure-hunt demo where you have a 
limited number of tokens to read the distance from the target, with the
goal of finding the hidden object before the tokens are used up.

More generally, it demonstrates how you can parse GPS data, though,
which provides data not just on position, but an accurate timestamp,
speed, altitude, and other useful datapoints.

## Toolchain

This project is based on the free MCUXpresso IDE from NXP. This IDE
includes the required toolchain, and is available for Windows, OS X and
Linux.

## Hardware Setup

The GPS should be connected to the [LPC845-BRK](https://www.nxp.com/products/processors-and-microcontrollers/arm-based-processors-and-mcus/lpc-cortex-m-mcus/lpc800-series-cortex-m0-plus-mcus/lpc845-breakout-board-for-lpc84x-family-mcus:LPC845-BRK)
(etc.) as follows:

	o TXD to P0_26 (USART1 TXD)
	o RXD to P0_27 (USART1 RXD)
	o FIX to P0_28 (GPIO Input)
	o PIEZO to P0_29 (SCT_OUT2 PWM)

## PRINTF Macro (Debug Output)

This project is configured to output PRINTF to USART0, which will be
redirected through the USB CDC available on the debugger. This is more
reliable than using SEMIHOSTING, although it does require an extra
step to open a connection from the console (etc.).

To see debug messages, simply open a terminal emulator at **9600 bps**.

On Linux or OS X, you can use minicom as follows:

	$ minicom -D /dev/tty.usbmodem1201F2 -b 9600
	
## LED Status Indicators

The RGB LED on the LPC845-BRK is used as a status indicator with the
following meaning:

- **GREEN LED**: Waiting for a GPS fix.
- **BLUE LED**: Parsing a GPS packet.
- **RED LED**: Error (invalid GPS packet, etc).

## Debug Mode

By default, the firmware will be built using the `release` mode GPS
parser, which has minimal PRINTF output.

If you wish to do something different with the GPS data, or see more
GPS data over USB Serial, you can change the following macro at the
top of `main.c`:

``` 
/* Set this to the `_debug` or `_release` variant depending on your needs. */
#define NMEA_PARSER				parse_nmea_sentence_release
``` 

Change the target function to `parse_nmea_sentence_debug`.