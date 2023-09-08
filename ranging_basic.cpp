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
#include <signal.h>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PointStamped.h>
#include "vl53l5cx_api.h"

#define TAN_B (sqrt(0.75*0.75+0.75*0.75)/tan(67.5*M_PI/180.0))
#define TAN_A (sqrt(0.75*0.75+0.25*0.25)/tan(67.5*M_PI/180.0))
#define TAN_C (sqrt(0.25*0.25+0.25*0.25)/tan(67.5*M_PI/180.0))

const double TanOfPitch[16] = {
    TAN_B, TAN_A, TAN_A, TAN_B,
    TAN_A, TAN_C, TAN_C, TAN_A,
    TAN_A, TAN_C, TAN_C, TAN_A,
    TAN_B, TAN_A, TAN_A, TAN_B
};

#define SIN_45 (1.0/sqrt(2))
#define SIN_A (0.75/sqrt(0.75*0.75+0.25*0.25))
#define SIN_B (0.25/sqrt(0.75*0.75+0.25*0.25))

const double SinOfYaw[16] = {
    SIN_45, SIN_A, SIN_A, SIN_45,
    SIN_B, SIN_45, SIN_45, SIN_B,
    -SIN_B, -SIN_45, -SIN_45, -SIN_B,
    -SIN_45, -SIN_A, -SIN_A, -SIN_45
};

#define COS_45 (1.0/sqrt(2))
#define COS_A (0.25/sqrt(0.75*0.75+0.25*0.25))
#define COS_B (0.75/sqrt(0.75*0.75+0.25*0.25))

const double CosOfYaw[16] = {
    -COS_45, -COS_A, COS_A, COS_45,
    -COS_B, -COS_45, COS_45, COS_B,
    -COS_B, -COS_45, COS_45, COS_B,
    -COS_45, -COS_A, COS_A, COS_45
};

int gogogo = 1;

//https://community.st.com/t5/imaging-sensors/vl53l5cx-multi-zone-sensor-get-x-y-z-of-points-relative-to/td-p/172929
//https://community.st.com/t5/imaging-sensors/vl53l5cx-distances-don-t-match-theory-what-am-i-missing/td-p/82657

void ConvertDist2XYZCoords(VL53L5CX_ResultsData *ResultsData, sensor_msgs::PointCloud* point_cloud)
{
	uint8_t ZoneNum;
    double dist_m;
	for (ZoneNum = 0; ZoneNum < 16; ZoneNum++)
	{
		if ((ResultsData->nb_target_detected[ZoneNum] > 0) && (ResultsData->distance_mm[ZoneNum] > 0) && ((ResultsData->target_status[ZoneNum] == 5) || (ResultsData->target_status[ZoneNum] == 6) || (ResultsData->target_status[ZoneNum] == 9)) )
		{
            geometry_msgs::Point32 p;
            dist_m = ResultsData->distance_mm[ZoneNum]*0.001;
            p.y = CosOfYaw[ZoneNum]*TanOfPitch[ZoneNum]*dist_m;
			p.z = -SinOfYaw[ZoneNum]*TanOfPitch[ZoneNum]*dist_m;
			p.x = dist_m;
            point_cloud->points.push_back(p);
		}
	}
}

void abort_handler(int signum) {
    gogogo = 0;
}

int main(int argc, char** argv)
{
    struct sigaction abort_act;

	/*********************************/
	/*   VL53L5CX ranging variables  */
	/*********************************/

	uint8_t 				status, isAlive, isReady;
	VL53L5CX_ResultsData 	Results;		/* Results data from VL53L5CX */
	VL53L5CX_Configuration 	Dev;

    abort_act.sa_handler = abort_handler;
    sigemptyset(&abort_act.sa_mask);
    abort_act.sa_flags = 0;
    sigaction(SIGINT, &abort_act, NULL);

    ros::init(argc, argv, "tof", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh("~");
    ROS_INFO("vl53l5cx ros");
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud>("point_cloud", 5);

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
	status = vl53l5cx_set_ranging_frequency_hz(&Dev, 30);
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
#if 0
			printf("Print data no : %3u\n", Dev.streamcount);
			for(int i = 0; i < 16; i++)
			{
				printf("Zone : %3d, Status : %3u, Distance : %4d mm\n",
					i,
					Results.target_status[VL53L5CX_NB_TARGET_PER_ZONE*i],
					Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*i]);
			}
			printf("\n");
#endif
            sensor_msgs::PointCloud point_cloud;
            point_cloud.header.stamp = ros::Time::now();
            point_cloud.header.frame_id = "body";
            ConvertDist2XYZCoords(&Results, &point_cloud);
            pub.publish(point_cloud);
		}

		/* Wait a few ms to avoid too high polling (function in platform
		 * file, not in API) */
		usleep(20*1000);
	}

	status = vl53l5cx_stop_ranging(&Dev);
	printf("End of ULD demo\n");
	return status;
}
