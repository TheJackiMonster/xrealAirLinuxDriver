//
// Created by thejackimonster on 29.03.23.
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
#include "device_mcu.h"

#include <stdio.h>
#include <sys/wait.h>
#include <unistd.h>

#include <math.h>

void test_imu(uint64_t timestamp,
              device_imu_event_type event,
              const device_imu_ahrs_type* ahrs) {
	static device_imu_quat_type old;
	static float dmax = -1.0f;
	
	if (event != DEVICE_IMU_EVENT_UPDATE) {
		return;
	}
	
	device_imu_quat_type q = device_imu_get_orientation(ahrs);
	
	const float dx = (old.x - q.x) * (old.x - q.x);
	const float dy = (old.y - q.y) * (old.y - q.y);
	const float dz = (old.z - q.z) * (old.z - q.z);
	const float dw = (old.w - q.w) * (old.w - q.w);
	
	const float d = sqrtf(dx*dx + dy*dy + dz*dz + dw*dw);
	
	if (dmax < 0.0f) {
		dmax = 0.0f;
	} else {
		dmax = (d > dmax? d : dmax);
	}
	
	device_imu_euler_type e = device_imu_get_euler(q);
	
	if (d >= 0.00005f) {
		device_imu_euler_type e0 = device_imu_get_euler(old);
		
		printf("Roll: %f; Pitch: %f; Yaw: %f\n", e0.roll, e0.pitch, e0.yaw);
		printf("Roll: %f; Pitch: %f; Yaw: %f\n", e.roll, e.pitch, e.yaw);
		printf("D = %f; ( %f )\n", d, dmax);
		
		printf("X: %f; Y: %f; Z: %f; W: %f;\n", old.x, old.y, old.z, old.w);
		printf("X: %f; Y: %f; Z: %f; W: %f;\n", q.x, q.y, q.z, q.w);
	} else {
		printf("Roll: %.2f; Pitch: %.2f; Yaw: %.2f\n", e.roll, e.pitch, e.yaw);
	}
	
	old = q;
}

void test_mcu(uint64_t timestamp,
              device_mcu_event_type event,
              uint8_t brightness,
              const char* msg) {
	switch (event) {
		case DEVICE_MCU_EVENT_MESSAGE:
			printf("Message: `%s`\n", msg);
			break;
		case DEVICE_MCU_EVENT_BRIGHTNESS_UP:
			printf("Increase Brightness: %u\n", brightness);
			break;
		case DEVICE_MCU_EVENT_BRIGHTNESS_DOWN:
			printf("Decrease Brightness: %u\n", brightness);
			break;
		default:
			break;
	}
}

int main(int argc, const char** argv) {
	pid_t pid = fork();
	
	if (pid == -1) {
		perror("Could not fork!\n");
		return 1;
	}
	
	if (pid == 0) {
		device_imu_type dev_imu;
		if (DEVICE_IMU_ERROR_NO_ERROR != device_imu_open(&dev_imu, test_imu)) {
			return 1;
		}

		device_imu_clear(&dev_imu);
		device_imu_calibrate(&dev_imu, 1000, true, true, false);
		while (DEVICE_IMU_ERROR_NO_ERROR == device_imu_read(&dev_imu, -1));
		device_imu_close(&dev_imu);
		return 0;
	} else {
		int status = 0;

		device_mcu_type dev_mcu;
		if (DEVICE_MCU_ERROR_NO_ERROR != device_mcu_open(&dev_mcu, test_mcu)) {
			status = 1;
			goto exit;
		}

		device_mcu_clear(&dev_mcu);
		while (DEVICE_MCU_ERROR_NO_ERROR == device_mcu_read(&dev_mcu, -1));
		device_mcu_close(&dev_mcu);
		
	exit:
		if (pid != waitpid(pid, &status, 0)) {
			return 1;
		}
		
		return status;
	}
}
