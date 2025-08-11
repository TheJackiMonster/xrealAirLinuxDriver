//
// Created by thejackimonster on 05.04.23.
//
// Copyright (c) 2023-2024 thejackimonster. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//

#include "device_imu.h"

#include <stdio.h>

void test(uint64_t timestamp,
          device_imu_event_type event,
          const device_imu_ahrs_type* ahrs) {
	device_imu_quat_type orientation;
	device_imu_euler_type euler;
	
	switch (event) {
		case DEVICE_IMU_EVENT_INIT:
			printf("Initialized\n");
			break;
		case DEVICE_IMU_EVENT_UPDATE:
			orientation = device_imu_get_orientation(ahrs);
			euler = device_imu_get_euler(orientation);
			printf("Roll: %.2f; Pitch: %.2f; Yaw: %.2f\n", euler.roll, euler.pitch, euler.yaw);
			break;
		default:
			break;
	}
}

int main(int argc, const char** argv) {
	device_imu_type dev;
	if (DEVICE_IMU_ERROR_NO_ERROR != device_imu_open(&dev, test)) {
		return 1;
	}

	if ((argc > 1) && (DEVICE_IMU_ERROR_NO_ERROR != device_imu_export_calibration(&dev, argv[1]))) {
		return 2;
	}
	
	device_imu_clear(&dev);
	while (DEVICE_IMU_ERROR_NO_ERROR == device_imu_read(&dev, -1));
	device_imu_close(&dev);
	return 0;
}
