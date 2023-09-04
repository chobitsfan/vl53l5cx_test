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

const double VL53L5_Zone_Pitch8x8[64] = {
	59.00,64.00,67.50,70.00,70.00,67.50,64.00,59.00,
	64.00,70.00,72.90,74.90,74.90,72.90,70.00,64.00,
	67.50,72.90,77.40,80.50,80.50,77.40,72.90,67.50,
	70.00,74.90,80.50,85.75,85.75,80.50,74.90,70.00,
	70.00,74.90,80.50,85.75,85.75,80.50,74.90,70.00,
	67.50,72.90,77.40,80.50,80.50,77.40,72.90,67.50,
	64.00,70.00,72.90,74.90,74.90,72.90,70.00,64.00,
	59.00,64.00,67.50,70.00,70.00,67.50,64.00,59.00
};
const double VL53L5_Zone_Yaw8x8[64] = {
	135.00,125.40,113.20, 98.13, 81.87, 66.80, 54.60, 45.00,
	144.60,135.00,120.96,101.31, 78.69, 59.04, 45.00, 35.40,
	156.80,149.04,135.00,108.45, 71.55, 45.00, 30.96, 23.20,
	171.87,168.69,161.55,135.00, 45.00, 18.45, 11.31,  8.13,
	188.13,191.31,198.45,225.00,315.00,341.55,348.69,351.87,
	203.20,210.96,225.00,251.55,288.45,315.00,329.04,336.80,
	203.20,225.00,239.04,258.69,281.31,300.96,315.00,324.60,
    225.00,234.60,246.80,261.87,278.13,293.20,305.40,315.00
};
double SinOfPitch[64], CosOfPitch[64], SinOfYaw[64], CosOfYaw[64];

int gogogo = 1;

void ComputeSinCosTables(void)
{
	//This function will save the math processing time of the code.  If the user wishes to not
	//perform this function, these tables can be generated and saved as a constant.
	uint8_t ZoneNum;
	for (ZoneNum = 0; ZoneNum < 64; ZoneNum++)
	{
		SinOfPitch[ZoneNum] = sin((VL53L5_Zone_Pitch8x8[ZoneNum])*M_PI/180);
		CosOfPitch[ZoneNum] = cos((VL53L5_Zone_Pitch8x8[ZoneNum])*M_PI/180);
		SinOfYaw[ZoneNum] = sin(VL53L5_Zone_Yaw8x8[ZoneNum]*M_PI/180);
		CosOfYaw[ZoneNum] = cos(VL53L5_Zone_Yaw8x8[ZoneNum]*M_PI/180);
	}
}

void ConvertDist2XYZCoords8x8(VL53L5CX_ResultsData *ResultsData, sensor_msgs::PointCloud* point_cloud)
{
	uint8_t ZoneNum;
	float Hyp;
	for (ZoneNum = 0; ZoneNum < 64; ZoneNum++)
	{
		if ((ResultsData->nb_target_detected[ZoneNum] > 0) && (ResultsData->distance_mm[ZoneNum] > 0) && ((ResultsData->target_status[ZoneNum] == 5) || (ResultsData->target_status[ZoneNum] == 6) || (ResultsData->target_status[ZoneNum] == 9)) )
		{
            geometry_msgs::Point32 p;
			Hyp = ResultsData->distance_mm[ZoneNum]/SinOfPitch[ZoneNum];
			p.y = CosOfYaw[ZoneNum]*CosOfPitch[ZoneNum]*Hyp*0.001;
			p.z = SinOfYaw[ZoneNum]*CosOfPitch[ZoneNum]*Hyp*-0.001;
			p.x = ResultsData->distance_mm[ZoneNum]*0.001;
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

    ComputeSinCosTables();

    ros::init(argc, argv, "tof", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh("~");
    ROS_INFO("vl53l5cx ros");
    /*sensor_msgs::PointCloud point_cloud;
    point_cloud.header.frame_id = "world"
    point_cloud.points.resize(64);
    point_cloud.channels.resize(1);
    point_cloud.channels[0].name = "intensity";
    point_cloud.channels[0].values.resize(64);*/
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
	status = vl53l5cx_set_resolution(&Dev, VL53L5CX_RESOLUTION_8X8);
	if(status)
	{
		printf("vl53l5cx_set_resolution failed, status %u\n", status);
		return status;
	}

	/* Set ranging frequency to 10Hz.
	 * Using 4x4, min frequency is 1Hz and max is 60Hz
	 * Using 8x8, min frequency is 1Hz and max is 15Hz
	 */
	status = vl53l5cx_set_ranging_frequency_hz(&Dev, 15);
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
			for(int i = 0; i < 64; i++)
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
            ConvertDist2XYZCoords8x8(&Results, &point_cloud);
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
