/*******************************************************************************
* Copyright (c) 2020, STMicroelectronics - All Rights Reserved
*
* This file is part of the VL53L5CX Ultra Lite Driver and is dual licensed,
* either 'STMicroelectronics Proprietary license'
* or 'BSD 3-clause "New" or "Revised" License' , at your option.
*
********************************************************************************
*
* 'STMicroelectronics Proprietary license'
*
********************************************************************************
*
* License terms: STMicroelectronics Proprietary in accordance with licensing
* terms at www.st.com/sla0081
*
* STMicroelectronics confidential
* Reproduction and Communication of this document is strictly prohibited unless
* specifically authorized in writing by STMicroelectronics.
*
*
********************************************************************************
*
* Alternatively, the VL53L5CX Ultra Lite Driver may be distributed under the
* terms of 'BSD 3-clause "New" or "Revised" License', in which case the
* following provisions apply instead of the ones mentioned above :
*
********************************************************************************
*
* License terms: BSD 3-clause "New" or "Revised" License.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*
*******************************************************************************/

/***********************************/
/*   VL53L5CX ULD basic example    */
/***********************************/
/*
* This example is the most basic. It initializes the VL53L5CX ULD, and starts
* a ranging to capture 10 frames.
*
* By default, ULD is configured to have the following settings :
* - Resolution 4x4
* - Ranging period 1Hz
*
* In this example, we also suppose that the number of target per zone is
* set to 1 , and all output are enabled (see file platform.h).
*/

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/un.h>
#include <signal.h>
#include "vl53l5cx_api.h"

#define SERVER_PATH "/tmp/chobits_server2"
#define SOCK_PATH "/tmp/chobits_2345"

int gogogo = 1;

void abort_handler(int signum) {
    gogogo = 0;
}

int main(int argc, char** argv)
{
    int sock_fd, prx_msg = 1;
    struct sockaddr_un ipc_addr, local_addr;
    struct sigaction abort_act;

	/*********************************/
	/*   VL53L5CX ranging variables  */
	/*********************************/

	uint8_t 				status, isAlive, isReady;
	VL53L5CX_ResultsData 	Results;		/* Results data from VL53L5CX */
	VL53L5CX_Configuration 	Dev;

    sock_fd = socket(AF_UNIX, SOCK_DGRAM, 0);
    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sun_family = AF_UNIX;
    strcpy(local_addr.sun_path, SOCK_PATH);
    unlink(SOCK_PATH);
    bind(sock_fd, (struct sockaddr*)&local_addr, sizeof(local_addr));

    memset(&ipc_addr, 0, sizeof(ipc_addr));
    ipc_addr.sun_family = AF_UNIX;
    strcpy(ipc_addr.sun_path, SERVER_PATH);

    abort_act.sa_handler = abort_handler;
    sigemptyset(&abort_act.sa_mask);
    abort_act.sa_flags = 0;
    sigaction(SIGINT, &abort_act, NULL);

	/*********************************/
	/*   Power on sensor and init    */
	/*********************************/

	/* Initialize channel com */
	status = vl53l5cx_comms_init(&Dev.platform);
	if(status)
	{
		printf("VL53L5CX comms init failed\n");
		return -1;
	}

	/* (Optional) Check if there is a VL53L5CX sensor connected */
	status = vl53l5cx_is_alive(&Dev, &isAlive);
	if(!isAlive || status)
	{
		printf("VL53L5CX not detected at requested address\n");
		return status;
	}

	/* (Mandatory) Init VL53L5CX sensor */
	status = vl53l5cx_init(&Dev);
	if(status)
	{
		printf("VL53L5CX ULD Loading failed\n");
		return status;
	}

	printf("VL53L5CX ULD ready ! (Version : %s)\n",
			VL53L5CX_API_REVISION);

	/* Set resolution in 8x8. WARNING : As others settings depend to this
	 * one, it must be the first to use.
	 */
	status = vl53l5cx_set_resolution(&Dev, VL53L5CX_RESOLUTION_4X4);
	if(status)
	{
		printf("vl53l5cx_set_resolution failed, status %u\n", status);
		return status;
	}

	/* Set ranging frequency to 10Hz.
	 * Using 4x4, min frequency is 1Hz and max is 60Hz
	 * Using 8x8, min frequency is 1Hz and max is 15Hz
	 */
	status = vl53l5cx_set_ranging_frequency_hz(&Dev, 20);
	if(status)
	{
		printf("vl53l5cx_set_ranging_frequency_hz failed, status %u\n", status);
		return status;
	}

	status = vl53l5cx_set_ranging_mode(&Dev, VL53L5CX_RANGING_MODE_CONTINUOUS);
	if(status)
	{
		printf("vl53l5cx_set_ranging_mode failed, status %u\n", status);
		return status;
	}

	/*********************************/
	/*         Ranging loop          */
	/*********************************/

	status = vl53l5cx_start_ranging(&Dev);

	while(gogogo)
	{
		/* Use polling function to know when a new measurement is ready.
		 * Another way can be to wait for HW interrupt raised on PIN A3
		 * (GPIO 1) when a new measurement is ready */

		status = vl53l5cx_check_data_ready(&Dev, &isReady);

		if(isReady)
		{
			vl53l5cx_get_ranging_data(&Dev, &Results);

			/* As the sensor is set in 4x4 mode by default, we have a total 
			 * of 16 zones to print. For this example, only the data of first zone are 
			 * print */
			/*printf("Print data no : %3u\n", Dev.streamcount);
			for(int i = 0; i < 16; i++)
			{
				printf("Zone : %3d, Status : %3u, Distance : %4d mm\n",
					i,
					Results.target_status[VL53L5CX_NB_TARGET_PER_ZONE*i],
					Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*i]);
			}
			printf("\n");*/
            if ((Results.target_status[5] == 5 && Results.distance_mm[5] < 1500) ||
                (Results.target_status[6] == 5 && Results.distance_mm[6] < 1500) ||
                (Results.target_status[9] == 5 && Results.distance_mm[9] < 1500) ||
                (Results.target_status[10] == 5 && Results.distance_mm[10] < 1500)) {
                    sendto(sock_fd, &prx_msg, sizeof(prx_msg), 0, (struct sockaddr*)&ipc_addr, sizeof(ipc_addr));
            }
		}

		/* Wait a few ms to avoid too high polling (function in platform
		 * file, not in API) */
		usleep(20*1000);
	}

	status = vl53l5cx_stop_ranging(&Dev);
	printf("End of ULD demo\n");
	return status;
}
