# LPC845 GPS Wayfinder Project

## Hardware Setup

The GPS should be connected as follows:

	o TXD to P0_26 (USART1 TXD)
	o RXD to P0_27 (USART1 RXD)
	o FIX to P0_28 (GPIO Input)

# PRINTF Macro (Debug Output)

This project is configured to output PRINTF to USART0, which will be
redirected through the USB CDC available on the debugger. This is more
reliable than using SEMIHOSTING, although it does require an extra
step to open a connection from the console (etc.).

To see debug messages, simply open a terminal emulator at 9600 bps.

On Linux or OS X, you can use minicom as follows:

	$ minicom -D /dev/tty.usbmodem1201F2 -b 9600
