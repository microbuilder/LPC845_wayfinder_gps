/*
 * The Clear BSD License
 * Copyright (c) 2019 Kevin Townsend
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
#include <math.h>
#include <errno.h>
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_sctimer.h"
#include "pin_mux.h"
#include "minmea/minmea.h"
#include "uart.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define UART_CLK_SRC 			(kCLOCK_MainClk)
#define UART_CLK_FREQ 			(CLOCK_GetFreq(UART_CLK_SRC))
#define NMEA_BUFFER_LENGTH 		(MINMEA_MAX_LENGTH*2)
#define SCTIMER_CLK_FREQ 		(CLOCK_GetFreq(kCLOCK_Fro))
#define SCTIMER_OUT 			(kSCTIMER_Out_2)
#define SCTIMER_PIEZO_FREQ		(4000U)
#define SCTIMER_PIEZO_DUTY		(25U)
#define PIEZO_ON()				SCTIMER_StartTimer(SCT0, kSCTIMER_Counter_L);
#define PIEZO_OFF()				SCTIMER_StopTimer(SCT0, kSCTIMER_Counter_L);

/* Set this to the `_debug` or `_release` variant depending on your needs. */
#define NMEA_PARSER				parse_nmea_sentence_release

/* This should be included with Newlib, but adding it here just in case. */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*******************************************************************************
 * Structs
 ******************************************************************************/
/**
 * The last GPS co-ordinates in floating point degrees.
 */
struct gps_coord_fp_deg {
	float latitude;
	float longitude;
	float speed;
	bool is_valid;
};

/*******************************************************************************
 * Variables
 ******************************************************************************/
char g_nmeaBuffer[NMEA_BUFFER_LENGTH] = { 0 };
struct gps_coord_fp_deg g_gps_coord_fp_deg_last = { 0.0F, 0.0F, 0.0F, false };
struct gps_coord_fp_deg g_gps_coord_fp_deg_trgt = { 41.3918189F, 2.1771756F, 0.0F, false };

/*******************************************************************************
 * Code
 ******************************************************************************/
void
error_blink(void)
{
	uint32_t i;

	for (uint32_t loops = 0; loops < 2; loops++) {
		LED_RED_ON();
		for (i = 100000; i > 0; i--) {
			__asm volatile("nop");
		}
		LED_RED_OFF();
		for (i = 100000; i > 0; i--) {
			__asm volatile("nop");
		}
	}
}

float
deg_to_rad(float degrees)
{
	float rad = degrees * M_PI / 180.0F;
	return rad;
}

float
calc_distance(struct gps_coord_fp_deg *a, struct gps_coord_fp_deg *b)
{
	float hav_r_meters = 6371e3;	/* Mean radius of the earth in meters. */
	//float hav_r_miles = 3961;		/* Mean radius of the earth in miles. */

	/* Convert degrees to radians and calculate the deltas. */
	float lat1 = deg_to_rad(a->latitude);
	float lat2 = deg_to_rad(b->latitude);
	float lon1 = deg_to_rad(a->longitude);
	float lon2 = deg_to_rad(b->longitude);
	float delta_lat = lat2 - lat1;
	float delta_long = lon2 - lon1;

	/* Haversine */
	float hav_a = pow(sin(delta_lat/2.0f),2.0f) + cos(lat1) * cos(lat2) * pow(sin(delta_long/2.0f),2.0f);
	float hav_c = 2.0f * atan2(sqrt(hav_a), sqrt(1.0f-hav_a));

	/* Calculate the great circle distance in meters. */
	return hav_c * hav_r_meters;
}

int
parse_nmea_sentence_release(char *line)
{
	switch (minmea_sentence_id(line, false))
	{
		case MINMEA_SENTENCE_RMC: {
			struct minmea_sentence_rmc frame;
			if (minmea_parse_rmc(&frame, line)) {
				g_gps_coord_fp_deg_last.latitude = minmea_tocoord(&frame.latitude);
				g_gps_coord_fp_deg_last.longitude = minmea_tocoord(&frame.longitude);
				g_gps_coord_fp_deg_last.speed = minmea_tocoord(&frame.speed);
				g_gps_coord_fp_deg_last.is_valid = true;
				PRINTF("Current degree coordinates and speed: %f, %f (%f)\r\n",
						g_gps_coord_fp_deg_last.latitude,
						g_gps_coord_fp_deg_last.longitude,
						g_gps_coord_fp_deg_last.speed);
				float dist_m = calc_distance(&g_gps_coord_fp_deg_last, &g_gps_coord_fp_deg_trgt);
				if (dist_m > 1000.0F) {
					/* Show distance in kilometers. */
					PRINTF("Distance to target: %.2f km.\r\n", dist_m / 1000.0F);
				} else {
					/* Show distance in meters */
					PRINTF("Distance to target: %.1f meters.\r\n", dist_m);
				}
			}
			else {
				PRINTF("$xxRMC sentence is not parsed\r\n");
			}
		} break;

        case MINMEA_INVALID: {
        	/* $xxxxx sentence is not valid */
        	error_blink();
        } break;

        default: {
        	/* $xxxxx sentence is valid but wasn't handled by the parsing code above. */
        } break;
	}

	return 0;
}

int
parse_nmea_sentence_debug(char *line)
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
				g_gps_coord_fp_deg_last.latitude = minmea_tocoord(&frame.latitude);
				g_gps_coord_fp_deg_last.longitude = minmea_tocoord(&frame.longitude);
				g_gps_coord_fp_deg_last.speed = minmea_tocoord(&frame.speed);
				g_gps_coord_fp_deg_last.is_valid = true;
				PRINTF("$xxRMC floating point degree coordinates and speed: (%f,%f) %f\r\n",
						g_gps_coord_fp_deg_last.latitude,
						g_gps_coord_fp_deg_last.longitude,
						g_gps_coord_fp_deg_last.speed);
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
        	error_blink();
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

    sctimer_config_t sctimerInfo;
    sctimer_pwm_signal_param_t pwmParam;
    uint32_t event;
    uint32_t sctimerClock;

    BOARD_InitPins();
    BOARD_BootClockFRO30M();
    BOARD_InitDebugConsole();

    LED_GREEN_INIT(1);
    LED_BLUE_INIT(1);
    LED_RED_INIT(1);
    BOARD_GPS_FIX_INIT();

    PRINTF("\r\nLPC845 GPS Wayfinder\r\n");
    PRINTF("\r\nWaiting for a fix on the GPS module.\r\n");
    PRINTF("\r\nThe GREEN LED indicates that we are waiting for a fix.\r\n");
    PRINTF("\r\nThe BLUE LED indicates that we are parsing GPS data.\r\n");
    PRINTF("\r\nThe RED LED indicates an error like an invalid GPS data packet.\r\n");

    /* Initialise the SCTimer for PWM output to the piezo buzzer. */
    sctimerClock = SCTIMER_CLK_FREQ;
    SCTIMER_GetDefaultConfig(&sctimerInfo);
    SCTIMER_Init(SCT0, &sctimerInfo);
    pwmParam.output = SCTIMER_OUT;
    pwmParam.level = kSCTIMER_HighTrue;
    pwmParam.dutyCyclePercent = SCTIMER_PIEZO_DUTY;
    if (SCTIMER_SetupPwm(SCT0, &pwmParam, kSCTIMER_CenterAlignedPwm, SCTIMER_PIEZO_FREQ, sctimerClock, &event) == kStatus_Fail)
    {
        return -1;
    }

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
    			//LED_BLUE_ON();
    			//PIEZO_ON();
    	        /* Echo any incoming data out to the debug console. */
            	// PRINTF("> %s", g_nmeaBuffer);
            	/* Run the NMEA sentence through the parser. */
    			NMEA_PARSER(g_nmeaBuffer);
    			/* Clear the NMEA sentence buffer. */
    			memset(g_nmeaBuffer, 0, sizeof g_nmeaBuffer);
    			//LED_BLUE_OFF();
    			//PIEZO_OFF();
    		}
    	}
    }
}
