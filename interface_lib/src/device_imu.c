//
// Created by thejackimonster on 30.03.23.
//
// Copyright (c) 2023-2025 thejackimonster. All rights reserved.
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
#include "device.h"

#include <Fusion/FusionAxes.h>
#include <Fusion/FusionMath.h>
#include <float.h>
#include <json-c/json_object.h>
#include <json-c/json_types.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <Fusion/Fusion.h>
#include <json-c/json.h>

#include <hidapi/hidapi.h>

#include "crc32.h"
#include "hid_ids.h"
#include "endian_compat.h"

#define GRAVITY_G (9.806f)

#ifndef NDEBUG
#define device_imu_error(msg) fprintf(stderr, "ERROR: %s\n", msg)
#else
#define device_imu_error(msg) (0)
#endif

struct device_imu_camera_sensor_t {
	FusionMatrix cameraMisalignment;
	FusionVector cameraOffset;

	uint16_t resolution [2];

	float cc [2];
	float fc [2];

	uint32_t num_kc;
	float* kc;
};

struct device_imu_camera_t {
	uint32_t num_sensors;
	device_imu_camera_sensor_type* sensors;
};

struct device_imu_camera_calibration_t {
	uint32_t num_cameras;
	device_imu_camera_type *cameras;
};

struct device_imu_calibration_t {
	FusionMatrix gyroscopeMisalignment;
	FusionVector gyroscopeSensitivity;
	FusionVector gyroscopeOffset;
	
	FusionMatrix accelerometerMisalignment;
	FusionVector accelerometerSensitivity;
	FusionVector accelerometerOffset;
	
	FusionMatrix magnetometerMisalignment;
	FusionVector magnetometerSensitivity;
	FusionVector magnetometerOffset;
	
	FusionMatrix softIronMatrix;
	FusionVector hardIronOffset;
	
	FusionQuaternion noises;

	device_imu_camera_calibration_type cam;
};

static bool send_payload(device_imu_type* device, uint16_t size, const uint8_t* payload) {
	int payload_size = size;
	if (payload_size > device->max_payload_size) {
		payload_size = device->max_payload_size;
	}
	
	int transferred = hid_write(device->handle, payload, payload_size);
	
	if (transferred != payload_size) {
		device_imu_error("Sending payload failed");
		return false;
	}
	
	return (transferred == size);
}

static bool recv_payload(device_imu_type* device, uint16_t size, uint8_t* payload) {
	int payload_size = size;
	if (payload_size > device->max_payload_size) {
		payload_size = device->max_payload_size;
	}
	
	int transferred = hid_read(device->handle, payload, payload_size);
	
	if (transferred >= payload_size) {
		transferred = payload_size;
	}

	if (transferred == 0) {
		return false;
	}
	
	if (transferred != payload_size) {
		device_imu_error("Receiving payload failed");
		return false;
	}
	
	return (transferred == size);
}

struct __attribute__((__packed__)) device_imu_payload_packet_t {
	uint8_t head;
	uint32_t checksum;
	uint16_t length;
	uint8_t msgid;
	uint8_t data [512 - 8];
};

typedef struct device_imu_payload_packet_t device_imu_payload_packet_type;

static bool send_payload_msg(device_imu_type* device, uint8_t msgid, uint16_t len, const uint8_t* data) {
	static device_imu_payload_packet_type packet;
	
	const uint16_t packet_len = 3 + len;
	const uint16_t payload_len = 5 + packet_len;
	
	packet.head = 0xAA;
	packet.length = htole16(packet_len);
	packet.msgid = msgid;
	
	memcpy(packet.data, data, len);
	packet.checksum = htole32(
		crc32_checksum(
			(const uint8_t*) (&packet.length),
			packet.length
		)
	);
	
	return send_payload(device, payload_len, (uint8_t*) (&packet));
}

static bool send_payload_msg_signal(device_imu_type* device, uint8_t msgid, uint8_t signal) {
	return send_payload_msg(device, msgid, 1, &signal);
}

static bool recv_payload_msg(device_imu_type* device, uint8_t msgid, uint16_t len, uint8_t* data) {
	static device_imu_payload_packet_type packet;
	
	packet.head = 0;
	packet.length = 0;
	packet.msgid = 0;
	
	const uint16_t packet_len = 3 + len;
	const uint16_t payload_len = 5 + packet_len;
	
	do {
		if (!recv_payload(device, payload_len, (uint8_t*) (&packet))) {
			return false;
		}
	} while (packet.msgid != msgid);
	
	memcpy(data, packet.data, len);
	return true;
}

static device_imu_error_type load_device_imu_calibration_data(device_imu_type* device, uint32_t* len, char** data) {
	if (!device) {
		return DEVICE_IMU_ERROR_NO_DEVICE;
	}

	if ((!len) || (!data)) {
		return DEVICE_IMU_ERROR_NO_ALLOCATION;
	}

	if (!send_payload_msg(device, DEVICE_IMU_MSG_GET_CAL_DATA_LENGTH, 0, NULL)) {
		device_imu_error("Failed sending payload to get calibration data length");
		return DEVICE_IMU_ERROR_PAYLOAD_FAILED;
	}

	*len = 0;
	if (!recv_payload_msg(device, DEVICE_IMU_MSG_GET_CAL_DATA_LENGTH, 4, (uint8_t*) len)) {
		*data = NULL;
		return DEVICE_IMU_ERROR_LOADING_FAILED;
	}

	const uint16_t max_packet_size = (device->max_payload_size - 8);
	*data = malloc(*len + 1);

	if (!(*data)) {
		return DEVICE_IMU_ERROR_NO_ALLOCATION;
	}
	
	uint32_t position = 0;
	while (position < *len) {
		const uint32_t remaining = (*len - position);

		if (!send_payload_msg(device, DEVICE_IMU_MSG_CAL_DATA_GET_NEXT_SEGMENT, 0, NULL)) {
			break;
		}
		
		const uint16_t next = (remaining > max_packet_size? max_packet_size : remaining);
		
		if (!recv_payload_msg(device, DEVICE_IMU_MSG_CAL_DATA_GET_NEXT_SEGMENT, next, (uint8_t*) (*data) + position)) {
			break;
		}
		
		position += next;
	}

	if (position > *len) {
		(*data)[*len] = '\0';
	} else {
		(*data)[position] = '\0';
	}

	return DEVICE_IMU_ERROR_NO_ERROR;
}

static FusionVector json_object_get_vector(struct json_object* obj) {
	if ((!json_object_is_type(obj, json_type_array)) ||
		(json_object_array_length(obj) != 3)) {
		return FUSION_VECTOR_ZERO;
	}
	
	FusionVector vector;
	vector.axis.x = (float) json_object_get_double(json_object_array_get_idx(obj, 0));
	vector.axis.y = (float) json_object_get_double(json_object_array_get_idx(obj, 1));
	vector.axis.z = (float) json_object_get_double(json_object_array_get_idx(obj, 2));
	return vector;
}

static FusionQuaternion json_object_get_quaternion(struct json_object* obj) {
	if ((!json_object_is_type(obj, json_type_array)) ||
		(json_object_array_length(obj) != 4)) {
		return FUSION_IDENTITY_QUATERNION;
	}
	
	FusionQuaternion quaternion;
	quaternion.element.x = (float) json_object_get_double(json_object_array_get_idx(obj, 0));
	quaternion.element.y = (float) json_object_get_double(json_object_array_get_idx(obj, 1));
	quaternion.element.z = (float) json_object_get_double(json_object_array_get_idx(obj, 2));
	quaternion.element.w = (float) json_object_get_double(json_object_array_get_idx(obj, 3));
	return quaternion;
}

static uint32_t json_object_get_array_f32(struct json_object* obj, float** array, uint32_t n) {
	if ((!json_object_is_type(obj, json_type_array)) ||
		  ((n > 0) && (json_object_array_length(obj) != n))) {
		return 0;
	}

	if (n == 0) {
		n = json_object_array_length(obj);
		*array = malloc(sizeof(float) * n);
	}

	for (uint32_t i = 0; i < n; i++) {
		(*array)[i] = (float) json_object_get_double(json_object_array_get_idx(obj, i));
	}

	return n;
}

static uint32_t json_object_get_array_u16(struct json_object* obj, uint16_t** array, uint32_t n) {
	if ((!json_object_is_type(obj, json_type_array)) ||
		  ((n > 0) && (json_object_array_length(obj) != n))) {
		return 0;
	}

	if (n == 0) {
		n = json_object_array_length(obj);
		*array = malloc(sizeof(uint16_t) * n);
	}

	for (uint32_t i = 0; i < n; i++) {
		(*array)[i] = (uint16_t) json_object_get_int(json_object_array_get_idx(obj, i));
	}

	return n;
}

static void init_device_imu_camera(device_imu_camera_type *camera, json_object *cam) {
	uint32_t num_sensors = json_object_get_int(json_object_object_get(cam, "num_of_cameras"));

	device_imu_camera_sensor_type *sensors = NULL;

	if (num_sensors > 0) {
		sensors = malloc(sizeof(device_imu_camera_sensor_type) * num_sensors);
	}

	if (!sensors) {
		num_sensors = 0;
	}

	for (uint32_t n = 0; n < num_sensors; n++) {
		device_imu_camera_sensor_type *sensor = &(sensors[n]);

		char device_name [64];
		snprintf(device_name, 64, "device_%u", (n + 1));

		struct json_object* dev = json_object_object_get(cam, device_name);

		FusionQuaternion imu_q_cam = json_object_get_quaternion(json_object_object_get(dev, "imu_q_cam"));
		FusionVector cam_offset = json_object_get_vector(json_object_object_get(dev, "imu_p_cam"));

		sensor->cameraMisalignment = FusionQuaternionToMatrix(imu_q_cam);
		sensor->cameraOffset = cam_offset;

		uint16_t* resolution = sensor->resolution;
		float* cc = sensor->cc;
		float* fc = sensor->fc;
		float* kc = sensor->kc;
		
		json_object_get_array_u16(json_object_object_get(dev, "resolution"), &resolution, 2);

		json_object_get_array_f32(json_object_object_get(dev, "cc"), &cc, 2);
		json_object_get_array_f32(json_object_object_get(dev, "fc"), &fc, 2);

		sensor->num_kc = json_object_get_array_f32(json_object_object_get(dev, "kc"), &kc, 0);
		sensor->kc = kc;
	}

	camera->num_sensors = num_sensors;
	camera->sensors = sensors;
}

device_imu_error_type device_imu_open(device_imu_type* device, device_imu_event_callback callback) {
	if (!device) {
		device_imu_error("No device");
		return DEVICE_IMU_ERROR_NO_DEVICE;
	}
	
	memset(device, 0, sizeof(device_imu_type));
	device->vendor_id 	= xreal_vendor_id;
	device->product_id 	= 0;
	device->callback 	= callback;
	
	if (!device_init()) {
		device_imu_error("Not initialized");
		return DEVICE_IMU_ERROR_NOT_INITIALIZED;
	}

	struct hid_device_info* info = hid_enumerate(
		device->vendor_id, 
		device->product_id
	);

	struct hid_device_info* it = info;
	while (it) {
		int interface_id = xreal_imu_interface_id(it->product_id);
		if (interface_id != -1 && it->interface_number == interface_id) {
#ifndef NDEBUG
            printf("Found IMU device with product_id 0x%x on interface %d\n", it->product_id, interface_id);
#endif
			device->product_id = it->product_id;
			device->handle = hid_open_path(it->path);
			device->max_payload_size = xreal_imu_max_payload_size(device->product_id);
			break;
		}

		it = it->next;
	}

	hid_free_enumeration(info);
	
	if (!device->handle) {
		device_imu_error("No handle");
		return DEVICE_IMU_ERROR_NO_HANDLE;
	}

	if ((!send_payload_msg_signal(device, DEVICE_IMU_MSG_START_IMU_DATA, 0x0)) ||
        (!recv_payload_msg(device, DEVICE_IMU_MSG_START_IMU_DATA, 0, NULL))) {
		device_imu_error("Failed sending payload to stop imu data stream");
		return DEVICE_IMU_ERROR_PAYLOAD_FAILED;
	}

	device_imu_clear(device);
	
	if (!send_payload_msg(device, DEVICE_IMU_MSG_GET_STATIC_ID, 0, NULL)) {
		device_imu_error("Failed sending payload to get static id");
		return DEVICE_IMU_ERROR_PAYLOAD_FAILED;
	}
	
	uint32_t static_id = 0;
	if (recv_payload_msg(device, DEVICE_IMU_MSG_GET_STATIC_ID, 4, (uint8_t*) &static_id)) {
		device->static_id = static_id;
	} else {
		device->static_id = 0x20220101;
	}
	
	device->calibration = malloc(sizeof(device_imu_calibration_type));
	memset(device->calibration, 0, sizeof(device_imu_calibration_type));

	device_imu_reset_calibration(device);
	
	uint32_t calibration_len = 0;
	char *calibration_data = NULL;

	if (DEVICE_IMU_ERROR_NO_ERROR == load_device_imu_calibration_data(device, &calibration_len, &calibration_data)) {
		struct json_tokener* tokener = json_tokener_new();
		struct json_object* root = json_tokener_parse_ex(tokener, calibration_data, calibration_len);
		struct json_object* imu = json_object_object_get(root, "IMU");
		struct json_object* dev1 = json_object_object_get(imu, "device_1");
		
		FusionVector accel_bias = json_object_get_vector(json_object_object_get(dev1, "accel_bias"));
		FusionQuaternion accel_q_gyro = json_object_get_quaternion(json_object_object_get(dev1, "accel_q_gyro"));
		FusionVector gyro_bias = json_object_get_vector(json_object_object_get(dev1, "gyro_bias"));
		FusionQuaternion gyro_q_mag = json_object_get_quaternion(json_object_object_get(dev1, "gyro_q_mag"));
		FusionVector mag_bias = json_object_get_vector(json_object_object_get(dev1, "mag_bias"));
		FusionQuaternion imu_noises = json_object_get_quaternion(json_object_object_get(dev1, "imu_noises"));
		FusionVector scale_accel = json_object_get_vector(json_object_object_get(dev1, "scale_accel"));
		FusionVector scale_gyro = json_object_get_vector(json_object_object_get(dev1, "scale_gyro"));
		FusionVector scale_mag = json_object_get_vector(json_object_object_get(dev1, "scale_mag"));

		const FusionQuaternion accel_q_mag = FusionQuaternionMultiply(accel_q_gyro, gyro_q_mag);
		
		device->calibration->gyroscopeMisalignment = FusionQuaternionToMatrix(accel_q_gyro);
		device->calibration->gyroscopeSensitivity = scale_gyro;
		device->calibration->gyroscopeOffset = gyro_bias;
		
		device->calibration->accelerometerMisalignment = FUSION_IDENTITY_MATRIX;
		device->calibration->accelerometerSensitivity = scale_accel;
		device->calibration->accelerometerOffset = accel_bias;
		
		device->calibration->magnetometerMisalignment = FusionQuaternionToMatrix(accel_q_mag);
		device->calibration->magnetometerSensitivity = scale_mag;
		device->calibration->magnetometerOffset = mag_bias;
		
		device->calibration->noises = imu_noises;

		struct json_object* rgb = json_object_object_get(root, "RGB_camera");
		struct json_object* slam = json_object_object_get(root, "SLAM_camera");

		uint32_t num_cameras_rgb = json_object_get_int(json_object_object_get(rgb, "num_of_cameras"));
		uint32_t num_cameras_slam = json_object_get_int(json_object_object_get(slam, "num_of_cameras"));

		const uint32_t num_cameras = (num_cameras_rgb > 0? 1 : 0) + (num_cameras_slam > 0? 1 : 0);

		device_imu_camera_type *cameras = NULL;
		uint32_t camera_index = 0;

		if (num_cameras > 0) {
			cameras = malloc(sizeof(device_imu_camera_type) * num_cameras);
		}

		if ((cameras) && (num_cameras_rgb > 0)) {
			init_device_imu_camera(&(cameras[camera_index++]), rgb);
		}

		if ((cameras) && (num_cameras_slam > 0)) {
			init_device_imu_camera(&(cameras[camera_index++]), slam);
		}

		device->calibration->cam.num_cameras = cameras? num_cameras : 0;
		device->calibration->cam.cameras = cameras;
		
		json_tokener_free(tokener);
		free(calibration_data);
	}

	if ((!send_payload_msg_signal(device, DEVICE_IMU_MSG_START_IMU_DATA, 0x1)) ||
		(!recv_payload_msg(device, DEVICE_IMU_MSG_START_IMU_DATA, 0, NULL))) {
		device_imu_error("Failed sending payload to start imu data stream");
		return DEVICE_IMU_ERROR_PAYLOAD_FAILED;
	}

	const uint32_t SAMPLE_RATE = 1000;
	
	device->offset = malloc(sizeof(FusionOffset));
	device->ahrs = malloc(sizeof(FusionAhrs));
	
	if (device->offset) {
		FusionOffsetInitialise((FusionOffset*) device->offset, SAMPLE_RATE);
	}

	FusionAhrsInitialise((FusionAhrs*) device->ahrs);
	
	const FusionAhrsSettings settings = {
			.convention = FusionConventionNed,
			.gain = 0.5f,
			.accelerationRejection = 10.0f,
			.magneticRejection = 20.0f,
			.recoveryTriggerPeriod = 5 * SAMPLE_RATE, /* 5 seconds */
	};
	
	FusionAhrsSetSettings((FusionAhrs*) device->ahrs, &settings);
	return DEVICE_IMU_ERROR_NO_ERROR;
}

device_imu_error_type device_imu_reset_calibration(device_imu_type* device) {
	if (!device) {
		device_imu_error("No device");
		return DEVICE_IMU_ERROR_NO_DEVICE;
	}
	
	if (!device->calibration) {
		device_imu_error("Not allocated");
		return DEVICE_IMU_ERROR_NO_ALLOCATION;
	}
	
	device->calibration->gyroscopeMisalignment = FUSION_IDENTITY_MATRIX;
	device->calibration->gyroscopeSensitivity = FUSION_VECTOR_ONES;
	device->calibration->gyroscopeOffset = FUSION_VECTOR_ZERO;
	
	device->calibration->accelerometerMisalignment = FUSION_IDENTITY_MATRIX;
	device->calibration->accelerometerSensitivity = FUSION_VECTOR_ONES;
	device->calibration->accelerometerOffset = FUSION_VECTOR_ZERO;
	
	device->calibration->magnetometerMisalignment = FUSION_IDENTITY_MATRIX;
	device->calibration->magnetometerSensitivity = FUSION_VECTOR_ONES;
	device->calibration->magnetometerOffset = FUSION_VECTOR_ZERO;
	
	device->calibration->softIronMatrix = FUSION_IDENTITY_MATRIX;
	device->calibration->hardIronOffset = FUSION_VECTOR_ZERO;
	
	device->calibration->noises = FUSION_IDENTITY_QUATERNION;
	device->calibration->noises.element.w = 0.0f;

	if (device->calibration->cam.cameras) {
		for (uint32_t i = 0; i < device->calibration->cam.num_cameras; i++) {
			if (!device->calibration->cam.cameras[i].sensors) {
				continue;
			}

			for (uint32_t j = 0; j < device->calibration->cam.cameras[i].num_sensors; j++) {
				if (!device->calibration->cam.cameras[i].sensors[j].kc) {
					continue;
				}

				free(device->calibration->cam.cameras[i].sensors[j].kc);

				device->calibration->cam.cameras[i].sensors[j].num_kc = 0;
				device->calibration->cam.cameras[i].sensors[j].kc = NULL;
			}

			free(device->calibration->cam.cameras[i].sensors);

			device->calibration->cam.cameras[i].num_sensors = 0;
			device->calibration->cam.cameras[i].sensors = NULL;
		}

		free(device->calibration->cam.cameras);
	}

	device->calibration->cam.num_cameras = 0;
	device->calibration->cam.cameras = NULL;
}

device_imu_error_type device_imu_load_calibration(device_imu_type* device, const char* path) {
	if (!device) {
		device_imu_error("No device");
		return DEVICE_IMU_ERROR_NO_DEVICE;
	}
	
	if (!device->calibration) {
		device_imu_error("Not allocated");
		return DEVICE_IMU_ERROR_NO_ALLOCATION;
	}
	
	FILE* file = fopen(path, "rb");
	if (!file) {
		device_imu_error("No file opened");
		return DEVICE_IMU_ERROR_FILE_NOT_OPEN;
	}

	device_imu_error_type result = DEVICE_IMU_ERROR_NO_ERROR;
	const size_t calibration_size = (
		sizeof(device_imu_calibration_type) - sizeof(device_imu_camera_calibration_type)
	);
	
	size_t count;
	count = fread(device->calibration, 1, calibration_size, file);
	
	if (calibration_size != count) {
		device_imu_error("Not fully loaded");
		result = DEVICE_IMU_ERROR_LOADING_FAILED;
	}
	
	if (0 != fclose(file)) {
		device_imu_error("No file closed");
		return DEVICE_IMU_ERROR_FILE_NOT_CLOSED;
	}
	
	return result;
}

device_imu_error_type device_imu_save_calibration(device_imu_type* device, const char* path) {
	if (!device) {
		device_imu_error("No device");
		return DEVICE_IMU_ERROR_NO_DEVICE;
	}
	
	if (!device->calibration) {
		device_imu_error("Not allocated");
		return DEVICE_IMU_ERROR_NO_ALLOCATION;
	}
	
	FILE* file = fopen(path, "wb");
	if (!file) {
		device_imu_error("No file opened");
		return DEVICE_IMU_ERROR_FILE_NOT_OPEN;
	}

	device_imu_error_type result = DEVICE_IMU_ERROR_NO_ERROR;
	const size_t calibration_size = (
		sizeof(device_imu_calibration_type) - sizeof(device_imu_camera_calibration_type)
	);
	
	size_t count;
	count = fwrite(device->calibration, 1, calibration_size, file);
	
	if (calibration_size != count) {
		device_imu_error("Not fully saved");
		result = DEVICE_IMU_ERROR_SAVING_FAILED;
	}
	
	if (0 != fclose(file)) {
		device_imu_error("No file closed");
		return DEVICE_IMU_ERROR_FILE_NOT_CLOSED;
	}
	
	return result;
}

device_imu_error_type device_imu_export_calibration(device_imu_type* device, const char *path) {
	if ((!send_payload_msg_signal(device, DEVICE_IMU_MSG_START_IMU_DATA, 0x0)) ||
        (!recv_payload_msg(device, DEVICE_IMU_MSG_START_IMU_DATA, 0, NULL))) {
		device_imu_error("Failed sending payload to stop imu data stream");
		return DEVICE_IMU_ERROR_PAYLOAD_FAILED;
	}

	device_imu_clear(device);

	uint32_t calibration_len = 0;
	char *calibration_data = NULL;

	device_imu_error_type result = load_device_imu_calibration_data(device, &calibration_len, &calibration_data);

	if (DEVICE_IMU_ERROR_NO_ERROR != result) {
		goto free_data;
	}

	FILE* file = fopen(path, "w");
	if (!file) {
		device_imu_error("No file opened");
		result = DEVICE_IMU_ERROR_FILE_NOT_OPEN;
		goto free_data;
	}

	size_t count;
	count = fwrite(calibration_data, 1, calibration_len, file);
	
	if (calibration_len != count) {
		device_imu_error("Not fully saved");
		result = DEVICE_IMU_ERROR_SAVING_FAILED;
	}

	if (0 != fclose(file)) {
		device_imu_error("No file closed");
		result = DEVICE_IMU_ERROR_FILE_NOT_CLOSED;
	}

free_data:
	if (calibration_data) {
		free(calibration_data);
	}

	device_imu_clear(device);

	if ((!send_payload_msg_signal(device, DEVICE_IMU_MSG_START_IMU_DATA, 0x1)) ||
      	(!recv_payload_msg(device, DEVICE_IMU_MSG_START_IMU_DATA, 0, NULL))) {
		device_imu_error("Failed sending payload to stop imu data stream");
		result = DEVICE_IMU_ERROR_PAYLOAD_FAILED;
	}

	return result;
}

static void device_imu_callback(device_imu_type* device,
							 uint64_t timestamp,
							 device_imu_event_type event) {
	if (!device->callback) {
		return;
	}
	
	device->callback(timestamp, event, device->ahrs);
}

static int32_t pack32bit_signed(const uint8_t* data) {
	uint32_t unsigned_value = (data[0]) | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
	return ((int32_t) unsigned_value);
}

static int32_t pack24bit_signed(const uint8_t* data) {
	uint32_t unsigned_value = (data[0]) | (data[1] << 8) | (data[2] << 16);
	if ((data[2] & 0x80) != 0) unsigned_value |= (0xFF << 24);
	return ((int32_t) unsigned_value);
}

static int16_t pack16bit_signed(const uint8_t* data) {
	uint16_t unsigned_value = (data[1] << 8) | (data[0]);
	return (int16_t) unsigned_value;
}

static int32_t pack32bit_signed_swap(const uint8_t* data) {
	uint32_t unsigned_value = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | (data[3]);
	return ((int32_t) unsigned_value);
}

static int16_t pack16bit_signed_swap(const uint8_t* data) {
	uint16_t unsigned_value = (data[0] << 8) | (data[1]);
	return (int16_t) unsigned_value;
}

static int16_t pack16bit_signed_bizarre(const uint8_t* data) {
	uint16_t unsigned_value = (data[0]) | ((data[1] ^ 0x80) << 8);
	return (int16_t) unsigned_value;
}

static void readIMU_from_packet(const device_imu_packet_type* packet,
								FusionVector* gyroscope,
								FusionVector* accelerometer,
								FusionVector* magnetometer) {
	int32_t vel_m = pack16bit_signed(packet->angular_multiplier);
	int32_t vel_d = pack32bit_signed(packet->angular_divisor);
	
	int32_t vel_x = pack24bit_signed(packet->angular_velocity_x);
	int32_t vel_y = pack24bit_signed(packet->angular_velocity_y);
	int32_t vel_z = pack24bit_signed(packet->angular_velocity_z);
	
	gyroscope->axis.x = (float) vel_x * (float) vel_m / (float) vel_d;
	gyroscope->axis.y = (float) vel_y * (float) vel_m / (float) vel_d;
	gyroscope->axis.z = (float) vel_z * (float) vel_m / (float) vel_d;
	
	int32_t accel_m = pack16bit_signed(packet->acceleration_multiplier);
	int32_t accel_d = pack32bit_signed(packet->acceleration_divisor);
	
	int32_t accel_x = pack24bit_signed(packet->acceleration_x);
	int32_t accel_y = pack24bit_signed(packet->acceleration_y);
	int32_t accel_z = pack24bit_signed(packet->acceleration_z);
	
	accelerometer->axis.x = (float) accel_x * (float) accel_m / (float) accel_d;
	accelerometer->axis.y = (float) accel_y * (float) accel_m / (float) accel_d;
	accelerometer->axis.z = (float) accel_z * (float) accel_m / (float) accel_d;
	
	int32_t magnet_m = pack16bit_signed_swap(packet->magnetic_multiplier);
	int32_t magnet_d = pack32bit_signed_swap(packet->magnetic_divisor);
	
	int16_t magnet_x = pack16bit_signed_bizarre(packet->magnetic_x);
	int16_t magnet_y = pack16bit_signed_bizarre(packet->magnetic_y);
	int16_t magnet_z = pack16bit_signed_bizarre(packet->magnetic_z);
	
	magnetometer->axis.x = (float) magnet_x * (float) magnet_m / (float) magnet_d;
	magnetometer->axis.y = (float) magnet_y * (float) magnet_m / (float) magnet_d;
	magnetometer->axis.z = (float) magnet_z * (float) magnet_m / (float) magnet_d;
}

#define min(x, y) ((x) < (y)? (x) : (y))
#define max(x, y) ((x) > (y)? (x) : (y))

static void pre_biased_coordinate_system(FusionVector* v) {
	*v = FusionAxesSwap(*v, FusionAxesAlignmentNXNZNY);
}

static void post_biased_coordinate_system(const FusionVector* v, FusionVector* res) {
	*res = FusionAxesSwap(*v, FusionAxesAlignmentPZPXPY);
}

static void iterate_iron_offset_estimation(const FusionVector* magnetometer, FusionMatrix* softIronMatrix, FusionVector* hardIronOffset) {
	static FusionVector max = { FLT_MIN, FLT_MIN, FLT_MIN };
	static FusionVector min = { FLT_MAX, FLT_MAX, FLT_MAX };

	for (int i = 0; i < 3; i++) {
		max.array[i] = max(max.array[i], magnetometer->array[i]);
		min.array[i] = min(min.array[i], magnetometer->array[i]);
	}
	
	const float mx = (max.axis.x - min.axis.x) / 2.0f;
	const float my = (max.axis.y - min.axis.y) / 2.0f;
	const float mz = (max.axis.z - min.axis.z) / 2.0f;
	
	const float cx = (min.axis.x + max.axis.x) / 2.0f;
	const float cy = (min.axis.y + max.axis.y) / 2.0f;
	const float cz = (min.axis.z + max.axis.z) / 2.0f;

	memset(softIronMatrix, 0, sizeof(*softIronMatrix));
	
	softIronMatrix->element.xx = 1.0f / mx;
	softIronMatrix->element.yy = 1.0f / my;
	softIronMatrix->element.zz = 1.0f / mz;

	hardIronOffset->axis.x = cx;
	hardIronOffset->axis.y = cy;
	hardIronOffset->axis.z = cz;
}

static void apply_calibration(const device_imu_type* device,
							  FusionVector* gyroscope,
							  FusionVector* accelerometer,
							  FusionVector* magnetometer) {
	FusionMatrix gyroscopeMisalignment;
	FusionVector gyroscopeSensitivity;
	FusionVector gyroscopeOffset;
	
	FusionMatrix accelerometerMisalignment;
	FusionVector accelerometerSensitivity;
	FusionVector accelerometerOffset;
	
	FusionMatrix magnetometerMisalignment;
	FusionVector magnetometerSensitivity;
	FusionVector magnetometerOffset;
	
	FusionMatrix softIronMatrix;
	FusionVector hardIronOffset;
	
	if (device->calibration) {
		gyroscopeMisalignment = device->calibration->gyroscopeMisalignment;
		gyroscopeSensitivity = device->calibration->gyroscopeSensitivity;
		gyroscopeOffset = device->calibration->gyroscopeOffset;
		
		accelerometerMisalignment = device->calibration->accelerometerMisalignment;
		accelerometerSensitivity = device->calibration->accelerometerSensitivity;
		accelerometerOffset = device->calibration->accelerometerOffset;
		
		magnetometerMisalignment = device->calibration->magnetometerMisalignment;
		magnetometerSensitivity = device->calibration->magnetometerSensitivity;
		magnetometerOffset = device->calibration->magnetometerOffset;
		
		softIronMatrix = device->calibration->softIronMatrix;
		hardIronOffset = device->calibration->hardIronOffset;
	} else {
		gyroscopeMisalignment = FUSION_IDENTITY_MATRIX;
		gyroscopeSensitivity = FUSION_VECTOR_ONES;
		gyroscopeOffset = FUSION_VECTOR_ZERO;
		
		accelerometerMisalignment = FUSION_IDENTITY_MATRIX;
		accelerometerSensitivity = FUSION_VECTOR_ONES;
		accelerometerOffset = FUSION_VECTOR_ZERO;
		
		magnetometerMisalignment = FUSION_IDENTITY_MATRIX;
		magnetometerSensitivity = FUSION_VECTOR_ONES;
		magnetometerOffset = FUSION_VECTOR_ZERO;
		
		softIronMatrix = FUSION_IDENTITY_MATRIX;
		hardIronOffset = FUSION_VECTOR_ZERO;
	}

	gyroscopeOffset = FusionVectorMultiplyScalar(
		gyroscopeOffset, 
		FusionRadiansToDegrees(1.0f)
	);

	accelerometerOffset = FusionVectorMultiplyScalar(
		accelerometerOffset, 
		1.0f / GRAVITY_G
	);

	FusionVector g = *gyroscope;
	FusionVector a = *accelerometer;
	FusionVector m = *magnetometer;

	pre_biased_coordinate_system(&g);
	pre_biased_coordinate_system(&a);
	pre_biased_coordinate_system(&m);

	g = FusionCalibrationInertial(
			g,
			gyroscopeMisalignment,
			gyroscopeSensitivity,
			gyroscopeOffset
	);
	
	a = FusionCalibrationInertial(
			a,
			accelerometerMisalignment,
			accelerometerSensitivity,
			accelerometerOffset
	);
	
	m = FusionCalibrationInertial(
			m,
			magnetometerMisalignment,
			magnetometerSensitivity,
			magnetometerOffset
	);

	iterate_iron_offset_estimation(
		&m, 
		&softIronMatrix, 
		&hardIronOffset
	);
	
	if (device->calibration) {
		device->calibration->softIronMatrix = softIronMatrix;
		device->calibration->hardIronOffset = hardIronOffset;
	}
	
	m = FusionCalibrationMagnetic(
			m,
			softIronMatrix,
			hardIronOffset
	);

	post_biased_coordinate_system(&g, gyroscope);
	post_biased_coordinate_system(&a, accelerometer);
	post_biased_coordinate_system(&m, magnetometer);
}

device_imu_error_type device_imu_clear(device_imu_type* device) {
	return device_imu_read(device, 10);
}

device_imu_error_type device_imu_calibrate(device_imu_type* device, uint32_t iterations, bool gyro, bool accel, bool magnet) {
	if (!device) {
		device_imu_error("No device");
		return DEVICE_IMU_ERROR_NO_DEVICE;
	}

	if (!device->handle) {
		device_imu_error("No handle");
		return DEVICE_IMU_ERROR_NO_HANDLE;
	}

	if (!device->calibration) {
		device_imu_error("No calibration allocated");
		return DEVICE_IMU_ERROR_NO_ALLOCATION;
	}
	
	if (sizeof(device_imu_packet_type) > device->max_payload_size) {
		device_imu_error("Not proper size");
		return DEVICE_IMU_ERROR_WRONG_SIZE;
	}
	
	device_imu_packet_type packet;
	int transferred;
	
	bool initialized = false;
	
	FusionVector cal_gyroscope;
	FusionVector cal_accelerometer;

	FusionMatrix softIronMatrix;
	FusionVector hardIronOffset;
	
	const float factor = iterations > 0? 1.0f / ((float) iterations) : 0.0f;

	FusionVector prev_accel;
	while (iterations > 0) {
		memset(&packet, 0, sizeof(device_imu_packet_type));
		
		transferred = hid_read(
			device->handle, 
			(uint8_t*) &packet, 
			sizeof(device_imu_packet_type)
		);

		if (transferred == -1) {
			device_imu_error("Device may be unplugged");
			return DEVICE_IMU_ERROR_UNPLUGGED;
		}

		if (transferred == 0) {
			continue;
		}

		if (sizeof(device_imu_packet_type) != transferred) {
			device_imu_error("Unexpected packet size");
			return DEVICE_IMU_ERROR_UNEXPECTED;
		}
		
		if ((packet.signature[0] != 0x01) || (packet.signature[1] != 0x02)) {
			continue;
		}
		
		FusionVector gyroscope;
		FusionVector accelerometer;
		FusionVector magnetometer;
		
		readIMU_from_packet(&packet, &gyroscope, &accelerometer, &magnetometer);
		
		pre_biased_coordinate_system(&gyroscope);
		pre_biased_coordinate_system(&accelerometer);
		pre_biased_coordinate_system(&magnetometer);

		if (initialized) {
			cal_gyroscope = FusionVectorAdd(cal_gyroscope, gyroscope);
			cal_accelerometer = FusionVectorAdd(cal_accelerometer, FusionVectorSubtract(accelerometer, prev_accel));
		} else {
			cal_gyroscope = gyroscope;
			cal_accelerometer = FUSION_VECTOR_ZERO;
		}

		prev_accel = accelerometer;

		iterate_iron_offset_estimation(
			&magnetometer, 
			&softIronMatrix, 
			&hardIronOffset
		);
		
		iterations--;
	}
	
	if (factor > 0.0f) {
		if (gyro) {
			device->calibration->gyroscopeOffset = FusionVectorAdd(
					device->calibration->gyroscopeOffset,
					FusionVectorMultiplyScalar(
							cal_gyroscope,
							FusionDegreesToRadians(factor)
					)
			);
		}
		
		if (accel) {
			device->calibration->accelerometerOffset = FusionVectorAdd(
					device->calibration->accelerometerOffset,
					FusionVectorMultiplyScalar(
							cal_accelerometer,
							factor * GRAVITY_G
					)
			);
		}
		
		if (magnet) {
			device->calibration->softIronMatrix = softIronMatrix;
			device->calibration->hardIronOffset = hardIronOffset;
		}
	}
	
	return DEVICE_IMU_ERROR_NO_ERROR;
}

device_imu_error_type device_imu_read(device_imu_type* device, int timeout) {
	if (!device) {
		device_imu_error("No device");
		return DEVICE_IMU_ERROR_NO_DEVICE;
	}

	if (!device->handle) {
		device_imu_error("No handle");
		return DEVICE_IMU_ERROR_NO_HANDLE;
	}
	
	if (sizeof(device_imu_packet_type) > device->max_payload_size) {
		device_imu_error("Not proper size");
		return DEVICE_IMU_ERROR_WRONG_SIZE;
	}
	
	device_imu_packet_type packet;
	memset(&packet, 0, sizeof(device_imu_packet_type));
	
	int transferred = hid_read_timeout(
		device->handle, 
		(uint8_t*) &packet, 
		sizeof(device_imu_packet_type),
		timeout
	);

	if (transferred == -1) {
		device_imu_error("Device may be unplugged");
		return DEVICE_IMU_ERROR_UNPLUGGED;
	}
	
	if (transferred == 0) {
		return DEVICE_IMU_ERROR_NO_ERROR;
	}
	
	if (sizeof(device_imu_packet_type) != transferred) {
		device_imu_error("Unexpected packet size");
		return DEVICE_IMU_ERROR_UNEXPECTED;
	}
	
	const uint64_t timestamp = le64toh(packet.timestamp);
	
	if ((packet.signature[0] == 0xaa) && (packet.signature[1] == 0x53)) {
		device_imu_callback(device, timestamp, DEVICE_IMU_EVENT_INIT);
		return DEVICE_IMU_ERROR_NO_ERROR;
	}
	
	if ((packet.signature[0] != 0x01) || (packet.signature[1] != 0x02)) {
		device_imu_error("Not matching signature");
		return DEVICE_IMU_ERROR_WRONG_SIGNATURE;
	}
	
	const uint64_t delta = timestamp - device->last_timestamp;
	const float deltaTime = (float) ((double) delta / 1e9);
	
	device->last_timestamp = timestamp;
	
	int16_t temperature = pack16bit_signed(packet.temperature);
	
	// According to the ICM-42688-P datasheet: (offset: 25 °C, sensitivity: 132.48 LSB/°C)
	device->temperature = ((float) temperature) / 132.48f + 25.0f;
	
	FusionVector gyroscope;
	FusionVector accelerometer;
	FusionVector magnetometer;
	
	readIMU_from_packet(&packet, &gyroscope, &accelerometer, &magnetometer);
	apply_calibration(device, &gyroscope, &accelerometer, &magnetometer);
	
	if (device->offset) {
		gyroscope = FusionOffsetUpdate((FusionOffset*) device->offset, gyroscope);
	}
	
#ifndef NDEBUG
	printf("G: %.2f %.2f %.2f\n", gyroscope.axis.x, gyroscope.axis.y, gyroscope.axis.z);
	printf("A: %.2f %.2f %.2f\n", accelerometer.axis.x, accelerometer.axis.y, accelerometer.axis.z);
	printf("M: %.2f %.2f %.2f\n", magnetometer.axis.x, magnetometer.axis.y, magnetometer.axis.z);
#endif

	if (device->ahrs) {
		if (isnan(magnetometer.axis.x) || isnan(magnetometer.axis.x) || isnan(magnetometer.axis.x)) {
			FusionAhrsUpdateNoMagnetometer((FusionAhrs*) device->ahrs, gyroscope, accelerometer, deltaTime);
		} else {
			/* The magnetometer seems to make results of sensor fusion generally worse. So it is not used currently. */
			// FusionAhrsUpdate((FusionAhrs*) device->ahrs, gyroscope, accelerometer, magnetometer, deltaTime);
			FusionAhrsUpdateNoMagnetometer((FusionAhrs*) device->ahrs, gyroscope, accelerometer, deltaTime);
		}

		const device_imu_quat_type orientation = device_imu_get_orientation(device->ahrs);

		// TODO: fix detection of this case; quat.x as a nan value is only a side-effect of some issue with ahrs or
		//       the gyro/accel/magnet readings
		if (isnan(orientation.x) || isnan(orientation.y) || isnan(orientation.z) || isnan(orientation.w)) {
			device_imu_error("Invalid orientation reading");
			return DEVICE_IMU_ERROR_INVALID_VALUE;
		}
	}
	
	device_imu_callback(device, timestamp, DEVICE_IMU_EVENT_UPDATE);
	return DEVICE_IMU_ERROR_NO_ERROR;
}

device_imu_vec3_type device_imu_get_earth_acceleration(const device_imu_ahrs_type* ahrs) {
	FusionVector acceleration = ahrs? FusionAhrsGetEarthAcceleration((const FusionAhrs*) ahrs) : FUSION_VECTOR_ZERO;
	device_imu_vec3_type a;
	a.x = acceleration.axis.x;
	a.y = acceleration.axis.y;
	a.z = acceleration.axis.z;
	return a;
}

device_imu_vec3_type device_imu_get_linear_acceleration(const device_imu_ahrs_type* ahrs) {
	FusionVector acceleration = ahrs? FusionAhrsGetLinearAcceleration((const FusionAhrs*) ahrs) : FUSION_VECTOR_ZERO;
	device_imu_vec3_type a;
	a.x = acceleration.axis.x;
	a.y = acceleration.axis.y;
	a.z = acceleration.axis.z;
	return a;
}

device_imu_quat_type device_imu_get_orientation(const device_imu_ahrs_type* ahrs) {
	FusionQuaternion quaternion = ahrs? FusionAhrsGetQuaternion((const FusionAhrs*) ahrs) : FUSION_IDENTITY_QUATERNION;
	device_imu_quat_type q;
	q.x = quaternion.element.x;
	q.y = quaternion.element.y;
	q.z = quaternion.element.z;
	q.w = quaternion.element.w;
	return q;
}

device_imu_euler_type device_imu_get_euler(device_imu_quat_type quat) {
	FusionQuaternion quaternion;
	quaternion.element.x = quat.x;
	quaternion.element.y = quat.y;
	quaternion.element.z = quat.z;
	quaternion.element.w = quat.w;
	FusionEuler euler = FusionQuaternionToEuler(quaternion);
	device_imu_euler_type e;
	e.roll = euler.angle.roll;
	e.pitch = euler.angle.pitch;
	e.yaw = euler.angle.yaw;
	return e;
}

uint32_t device_imu_get_num_of_cameras(device_imu_type *device) {
	if (!device->calibration) {
		return 0;
	}

	return device->calibration->cam.num_cameras;
}

const device_imu_camera_type* device_imu_get_camera(const device_imu_type *device, uint32_t index) {
	if ((!device->calibration) || (!device->calibration->cam.cameras)) {
		return NULL;
	}

	return &(device->calibration->cam.cameras[index]);
}

uint32_t device_imu_camera_get_num_of_sensors(const device_imu_camera_type *camera) {
	if (!camera) {
		return 0;
	}

	return camera->num_sensors;
}

const device_imu_camera_sensor_type* device_imu_camera_get_sensor(const device_imu_camera_type *camera, uint32_t index) {
	if (!camera->sensors) {
		return NULL;
	}

	return &(camera->sensors[index]);
}

device_imu_mat3x3_type device_imu_sensor_get_rotation(const device_imu_camera_sensor_type *sensor) {
	device_imu_mat3x3_type rotation;
	rotation.m[0] = sensor->cameraMisalignment.array[0][0];
	rotation.m[1] = sensor->cameraMisalignment.array[0][1];
	rotation.m[2] = sensor->cameraMisalignment.array[0][2];
	rotation.m[3] = sensor->cameraMisalignment.array[1][0];
	rotation.m[4] = sensor->cameraMisalignment.array[1][1];
	rotation.m[5] = sensor->cameraMisalignment.array[1][2];
	rotation.m[6] = sensor->cameraMisalignment.array[2][0];
	rotation.m[7] = sensor->cameraMisalignment.array[2][1];
	rotation.m[8] = sensor->cameraMisalignment.array[2][2];
	return rotation;
}

device_imu_vec3_type device_imu_sensor_get_position(const device_imu_camera_sensor_type *sensor) {
	device_imu_vec3_type position;
	position.x = sensor->cameraOffset.axis.x;
	position.y = sensor->cameraOffset.axis.y;
	position.z = sensor->cameraOffset.axis.z;
	return position;
}

device_imu_size_type device_imu_sensor_get_resolution(const device_imu_camera_sensor_type *sensor) {
	device_imu_size_type resolution;
	resolution.width = sensor->resolution[0];
	resolution.height = sensor->resolution[1];
	return resolution;
}

device_imu_vec2_type device_imu_sensor_get_cc(const device_imu_camera_sensor_type *sensor) {
	device_imu_vec2_type cc;
	cc.x = sensor->cc[0];
	cc.y = sensor->cc[1];
	return cc;
}

device_imu_vec2_type device_imu_sensor_get_fc(const device_imu_camera_sensor_type *sensor) {
	device_imu_vec2_type fc;
	fc.x = sensor->fc[0];
	fc.y = sensor->fc[1];
	return fc;
}

device_imu_error_type device_imu_sensor_get_kc(const device_imu_camera_sensor_type *sensor, uint32_t *num_kc, float *kc) {
	if ((!sensor) || (!num_kc)) {
		device_imu_error("Wrong argument");
		return DEVICE_IMU_ERROR_NO_ALLOCATION;
	}

	if (!kc) {
		*num_kc = sensor->num_kc;
	} else {
		uint32_t n = *num_kc;

		if (sensor->num_kc < n) {
			n = sensor->num_kc;
		}

		for (uint32_t i = 0; i < *num_kc; i++) {
			kc[i] = sensor->kc[i];
		}
	}

	return DEVICE_IMU_ERROR_NO_ERROR;
}

device_imu_error_type device_imu_close(device_imu_type* device) {
	if (!device) {
		device_imu_error("No device");
		return DEVICE_IMU_ERROR_NO_DEVICE;
	}
	
	if (device->calibration) {
		if (device->calibration->cam.cameras) {
			for (uint32_t i = 0; i < device->calibration->cam.num_cameras; i++) {
				if (!device->calibration->cam.cameras[i].sensors) {
					continue;
				}

				for (uint32_t j = 0; j < device->calibration->cam.cameras[i].num_sensors; j++) {
					if (!device->calibration->cam.cameras[i].sensors[j].kc) {
						continue;
					}

					free(device->calibration->cam.cameras[i].sensors[j].kc);
				}

				free(device->calibration->cam.cameras[i].sensors);
			}

			free(device->calibration->cam.cameras);
		}

		free(device->calibration);
	}
	
	if (device->ahrs) {
		free(device->ahrs);
	}
	
	if (device->offset) {
		free(device->offset);
	}

	if (device->handle) {
		if ((!send_payload_msg_signal(device, DEVICE_IMU_MSG_START_IMU_DATA, 0x0)) ||
			(!recv_payload_msg(device, DEVICE_IMU_MSG_START_IMU_DATA, 0, NULL))) {
			device_imu_error("Failed sending payload to stop imu data stream");
		}

		hid_close(device->handle);
	}
	
	memset(device, 0, sizeof(device_imu_type));
	device_exit();

	return DEVICE_IMU_ERROR_NO_ERROR;
}
