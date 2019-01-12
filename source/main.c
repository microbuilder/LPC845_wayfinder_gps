/*
 * The Clear BSD License
 * Copyright (c) 2017, NXP Semiconductors, Inc.
 * All rights reserved.
 *
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "board.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "minmea/minmea.h"
#include "uart.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define UART_CLK_SRC 			(kCLOCK_MainClk)
#define UART_CLK_FREQ 			(CLOCK_GetFreq(UART_CLK_SRC))
#define NMEA_BUFFER_LENGTH 		(MINMEA_MAX_LENGTH*2)

/*******************************************************************************
 * Variables
 ******************************************************************************/
char g_nmeaBuffer[NMEA_BUFFER_LENGTH] = { 0 };

/*******************************************************************************
 * Code
 ******************************************************************************/
int
parse_nmea_sentence(char *line)
{
	switch (minmea_sentence_id(line, false))
	{
		case MINMEA_SENTENCE_RMC: {
			struct minmea_sentence_rmc frame;
			if (minmea_parse_rmc(&frame, line)) {
#if 0
				PRINTF("$xxRMC: raw coordinates and speed: (%d/%d,%d/%d) %d/%d\r\n",
						frame.latitude.value, frame.latitude.scale,
						frame.longitude.value, frame.longitude.scale,
						frame.speed.value, frame.speed.scale);
				PRINTF("$xxRMC fixed-point coordinates and speed scaled to three decimal places: (%d,%d) %d\r\n",
						minmea_rescale(&frame.latitude, 1000),
						minmea_rescale(&frame.longitude, 1000),
						minmea_rescale(&frame.speed, 1000));
#endif
				PRINTF("$xxRMC floating point degree coordinates and speed: (%f,%f) %f\r\n",
						minmea_tocoord(&frame.latitude),
						minmea_tocoord(&frame.longitude),
						minmea_tofloat(&frame.speed));
			}
			else {
				PRINTF("$xxRMC sentence is not parsed\r\n");
			}
		} break;

        case MINMEA_SENTENCE_GGA: {
            struct minmea_sentence_gga frame;
            if (minmea_parse_gga(&frame, line)) {
                PRINTF("$xxGGA: fix quality: %d\r\n", frame.fix_quality);
            }
            else {
                PRINTF("$xxGGA sentence is not parsed\r\n");
            }
        } break;

        case MINMEA_SENTENCE_VTG: {
           struct minmea_sentence_vtg frame;
           if (minmea_parse_vtg(&frame, line)) {
                PRINTF("$xxVTG: true track degrees = %f\r\n",
                       minmea_tofloat(&frame.true_track_degrees));
                PRINTF("        magnetic track degrees = %f\r\n",
                       minmea_tofloat(&frame.magnetic_track_degrees));
                PRINTF("        speed knots = %f\r\n",
                        minmea_tofloat(&frame.speed_knots));
                PRINTF("        speed kph = %f\r\n",
                        minmea_tofloat(&frame.speed_kph));
           }
           else {
                PRINTF("$xxVTG sentence is not parsed\r\n");
           }
        } break;

        case MINMEA_SENTENCE_GSV: {
            struct minmea_sentence_gsv frame;
            if (minmea_parse_gsv(&frame, line)) {
            	PRINTF("$xxGSV: message %d of %d\r\n", frame.msg_nr, frame.total_msgs);
            	PRINTF("$xxGSV: satellites in view: %d\r\n", frame.total_sats);
                for (int i = 0; i < 4; i++)
                	PRINTF("$xxGSV: sat nr %d, elevation: %d, azimuth: %d, snr: %d dbm\r\n",
                        frame.sats[i].nr,
                        frame.sats[i].elevation,
                        frame.sats[i].azimuth,
                        frame.sats[i].snr);
            }
            else {
            	PRINTF("$xxGSV sentence is not parsed\r\n");
            }
        } break;

        case MINMEA_SENTENCE_ZDA: {
            struct minmea_sentence_zda frame;
            if (minmea_parse_zda(&frame, line)) {
            	PRINTF("$xxZDA: %d:%d:%d %02d.%02d.%d UTC%+03d:%02d\r\n",
                       frame.time.hours,
                       frame.time.minutes,
                       frame.time.seconds,
                       frame.date.day,
                       frame.date.month,
                       frame.date.year,
                       frame.hour_offset,
                       frame.minute_offset);
            }
            else {
            	PRINTF("$xxZDA sentence is not parsed\r\n");
            }
        } break;

        case MINMEA_INVALID: {
        	PRINTF("$xxxxx sentence is not valid: %s", line);
        } break;

        default: {
        	PRINTF("$xxxxx sentence is not parsed: %s", line);
        } break;
	}

	return 0;
}

/*!
 * @brief Main function
 */
int main(void)
{
    char gps_rx;

    BOARD_InitPins();
    BOARD_BootClockFRO30M();
    BOARD_InitDebugConsole();

    LED_GREEN_INIT(1);
    LED_BLUE_INIT(1);
    LED_RED_INIT(1);
    BOARD_GPS_FIX_INIT();

    PRINTF("\r\nLPC845 GPS Wayfinder\r\n");

    /* Init GPS USART port */
    CLOCK_Select(kUART1_Clk_From_MainClk);
    //RESET_PeripheralReset(kUART1_RST_N_SHIFT_RSTn);
    uart_init(UART_CLK_FREQ);

    /* Set buffers to a known state. */
    gps_rx = 0;
    memset(g_nmeaBuffer, 0, sizeof g_nmeaBuffer);

    while (1)
    {
    	/* Check if we have a FIX on the GPS */
    	uint32_t fix = GPIO_PinRead(BOARD_GPS_FIX_GPIO, BOARD_GPS_FIX_GPIO_PORT, BOARD_GPS_FIX_GPIO_PIN);
    	if (fix) {
    		LED_GREEN_ON();
    	} else {
    		LED_GREEN_OFF();
    	}

        /* Buffer and process any incoming GPS data. */
    	while(kStatus_Success == uart_read(&gps_rx)) {
			/* Append chars to NMEA buffer until we hit a '\r\n'. */
			size_t pos = strlen(g_nmeaBuffer);
			g_nmeaBuffer[pos] = gps_rx;
    		/* Parse NMEA sentence at newline char. */
    		if (gps_rx == '\n') {
    			LED_BLUE_ON();
    	        /* Echo any incoming data out to the debug console. */
            	// PRINTF("> %s", g_nmeaBuffer);
            	/* Run the NMEA sentence through the parser. */
    			parse_nmea_sentence(g_nmeaBuffer);
    			/* Clear the NMEA sentence buffer. */
    			memset(g_nmeaBuffer, 0, sizeof g_nmeaBuffer);
    			LED_BLUE_OFF();
    		}
    	}
    }
}
