//
// Created by thejackimonster on 04.06.25.
//
// Copyright (c) 2025 thejackimonster. All rights reserved.
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

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <stdio.h>

#include "device_imu.h"

const uint8_t chunk_map[128] = {
    119, 54,  21,  0,   108, 22,  51,  63,  93, 99,  67, 7,   32,  112, 52,
    43,  14,  35,  75,  116, 64,  71,  44,  89, 18,  88, 26,  61,  70,  56,
    90,  79,  87,  120, 81,  101, 121, 17,  72, 31,  53, 124, 127, 113, 111,
    36,  48,  19,  37,  83,  126, 74,  109, 5,  84,  41, 76,  30,  110, 29,
    12,  115, 28,  102, 105, 62,  103, 20,  3,  68,  49, 77,  117, 125, 106,
    60,  69,  98,  9,   16,  78,  47,  40,  2,  118, 34, 13,  50,  46,  80,
    85,  66,  42,  123, 122, 96,  11,  25,  97, 39,  6,  86,  1,   8,   82,
    92,  59,  104, 24,  15,  73,  65,  38,  58, 10,  23, 33,  55,  57,  107,
    100, 94,  27,  95,  45,  91,  4,   114};

#define CHUNK_SIZE 2400
#define CHUNK_AMOUNT 128
#define CHUNK_SUM_LEN 128

#define CAM_WIDTH 640
#define CAM_HEIGHT 480
#define CAM_HEADER_LEN 2

#define CAM_BUFFER_SIZE ((CAM_HEIGHT + CAM_HEADER_LEN) * CAM_WIDTH)
#define CAM_IMAGE_DATA_SIZE (CAM_HEIGHT * CAM_WIDTH)

struct CameraCalibration {
    struct {
        size_t width;
        size_t height;
    } resolution;

    float fx;
    float fy;
    float cx;
    float cy;

    size_t num_kc;
    union {
        float kc [12];
        struct {
            float k1;
            float k2;
            float k3;
            float k4;
            float k5;
            float k6;

            float p1;
            float p2;

            float s1;
            float s2;
            float s3;
            float s4;
        };
    };
};

typedef cv::Point2f (*apply_variant_func)(const CameraCalibration& cam, float xn, float yn);

cv::Point2f apply_intrinsic_only(const CameraCalibration& cam, float xn, float yn) {
    cv::Point2f p;

    p.x = cam.fx * xn + cam.cx;
    p.y = cam.fy * yn + cam.cy;

    return p;
}

cv::Point2f apply_radial_only(const CameraCalibration& cam, float xn, float yn) {
    cv::Point2f p;
    float r, sp, cp;
    float th, th_dist;

    r = sqrtf(xn * xn + yn * yn);

    if (r == 0) {
        cp = 1.0F;
        sp = 0.0F;
    } else {
        cp = xn / r;
        sp = yn / r;
    }

    th = atanf(r);
    th_dist = th;

    const float th2 = th * th;
    const float th3 = th * th * th;

    th_dist += cam.k1 * th3;
    th_dist += cam.k2 * th2 * th3;
    th_dist += cam.k3 * th2 * th2 * th3;
    th_dist += cam.k4 * th3 * th3 * th3;
    th_dist += cam.k5 * th3 * th3 * th2 * th3;
    th_dist += cam.k6 * th3 * th3 * th2 * th2 * th3;

    xn = th_dist * cp;
    yn = th_dist * sp;

    p.x = cam.fx * xn + cam.cx;
    p.y = cam.fy * yn + cam.cy;

    return p;
}

cv::Point2f apply_radial_tangential(const CameraCalibration& cam, float xn, float yn) {
    cv::Point2f p;
    float r, sp, cp;
    float th, th_dist;
    float dx, dy;

    r = sqrtf(xn * xn + yn * yn);

    if (r == 0) {
        cp = 1.0F;
        sp = 0.0F;
    } else {
        cp = xn / r;
        sp = yn / r;
    }

    th = atanf(r);
    th_dist = th;

    const float th2 = th * th;
    const float th3 = th * th * th;

    th_dist += cam.k1 * th3;
    th_dist += cam.k2 * th2 * th3;
    th_dist += cam.k3 * th2 * th2 * th3;
    th_dist += cam.k4 * th3 * th3 * th3;
    th_dist += cam.k5 * th3 * th3 * th2 * th3;
    th_dist += cam.k6 * th3 * th3 * th2 * th2 * th3;

    xn = th_dist * cp;
    yn = th_dist * sp;

    r = xn * xn + yn * yn;

    dx = (2.0F * xn * xn + r) * cam.p1 + 2.0F * xn * yn * cam.p2;
    dy = (2.0F * yn * yn + r) * cam.p2 + 2.0F * xn * yn * cam.p1;

    xn += dx;
    yn += dy;

    p.x = cam.fx * xn + cam.cx;
    p.y = cam.fy * yn + cam.cy;

    return p;
}

cv::Point2f apply_fisheye624(const CameraCalibration& cam, float xn, float yn) {
    cv::Point2f p;
    float r, sp, cp;
    float th, th_dist;
    float dx, dy;

    r = sqrtf(xn * xn + yn * yn);

    if (r == 0) {
        cp = 1.0F;
        sp = 0.0F;
    } else {
        cp = xn / r;
        sp = yn / r;
    }

    th = atanf(r);
    th_dist = th;

    const float th2 = th * th;
    const float th3 = th * th * th;
    const float th5 = th3 * th2;
    const float th6 = th3 * th3;

    th_dist += cam.k1 * th3;
    th_dist += cam.k2 * th2 * th3;
    th_dist += cam.k3 * th2 * th2 * th3;
    th_dist += cam.k4 * th3 * th3 * th3;
    th_dist += cam.k5 * th3 * th3 * th2 * th3;
    th_dist += cam.k6 * th3 * th3 * th2 * th2 * th3;

    xn = th_dist * cp;
    yn = th_dist * sp;

    r = xn * xn + yn * yn;

    dx = (2.0F * xn * xn + r) * cam.p1 + 2.0F * xn * yn * cam.p2;
    dy = (2.0F * yn * yn + r) * cam.p2 + 2.0F * xn * yn * cam.p1;

    dx += (cam.s1 + cam.s2 * r) * r;
    dy += (cam.s3 + cam.s4 * r) * r;

    xn += dx;
    yn += dy;

    p.x = cam.fx * xn + cam.cx;
    p.y = cam.fy * yn + cam.cy;

    return p;
}

cv::Mat build_maps_variant(apply_variant_func func, const CameraCalibration& cam, cv::Mat& indices) {
    const cv::Size map_size (cam.resolution.width, cam.resolution.height);
    
    cv::Mat map_x (map_size, CV_32FC1);
    cv::Mat map_y (map_size, CV_32FC1);
    
    cv::Point2f p;
    size_t x, y;

    cv::Mat map (map_size, CV_16SC2);
    indices = cv::Mat(map_size, CV_16UC1);

    for (y = 0; y < map.rows; y++) {
        for (x = 0; x < map.cols; x++) {
            const float xn = (x - cam.cx) / cam.fx;
            const float yn = (y - cam.cy) / cam.fy;

            p = func(cam, xn, yn);

            map_x.at<float>(y, x) = p.x;
            map_y.at<float>(y, x) = p.y;
        }
    }

    cv::convertMaps(map_x, map_y, map, indices, CV_16SC2);
    return map;
}

cv::Mat build_maps(const CameraCalibration& cam, cv::Mat& indices) {
    if (cam.num_kc >= 12) {
        return build_maps_variant(apply_fisheye624, cam, indices);
    } else
    if (cam.num_kc >= 8) {
        return build_maps_variant(apply_radial_tangential, cam, indices);
    } else
    if (cam.num_kc >= 6) {
        return build_maps_variant(apply_radial_only, cam, indices);
    } else {
        return build_maps_variant(apply_intrinsic_only, cam, indices);
    }
}

int main(int argc, const char **argv) {
    CameraCalibration cam [2];
    device_imu_type dev;

    const char *video_filename = "/dev/video0";

    if (argc > 1) {
        video_filename = argv[1];
    }
    
	if (DEVICE_IMU_ERROR_NO_ERROR != device_imu_open(&dev, nullptr)) {
		return 1;
	}
	
	device_imu_clear(&dev);

    size_t num_cameras = device_imu_get_num_of_cameras(&dev);
    size_t num_sensors = 0;

    if (num_cameras > 0) {
        const device_imu_camera_type *camera = device_imu_get_camera(&dev, 0);

        num_sensors = device_imu_camera_get_num_of_sensors(camera);

        for (size_t i = 0; (i < num_sensors) && (i < 2); i++) {
            memset(&(cam[i]), 0, sizeof(CameraCalibration));

            const device_imu_camera_sensor_type *sensor = device_imu_camera_get_sensor(camera, i);

            const device_imu_size_type resolution = device_imu_sensor_get_resolution(sensor);

            const device_imu_vec2_type cc = device_imu_sensor_get_cc(sensor);
            const device_imu_vec2_type fc = device_imu_sensor_get_fc(sensor);

            uint32_t num_kc = 0;
            if (DEVICE_IMU_ERROR_NO_ERROR != device_imu_sensor_get_kc(sensor, &num_kc, nullptr)) {
                continue;
            }

            cam[i].resolution.width = resolution.width;
            cam[i].resolution.height = resolution.height;

            cam[i].fx = fc.x;
            cam[i].fy = fc.y;

            cam[i].cx = cc.x;
            cam[i].cy = cc.y;

            cam[i].num_kc = num_kc;

            if (num_kc > 12) {
                num_kc = 12;
            }

            device_imu_sensor_get_kc(sensor, &num_kc, cam[i].kc);
        }
    }

	device_imu_close(&dev);

    if ((cam[0].resolution.width != cam[1].resolution.width) ||
        (cam[0].resolution.height != cam[1].resolution.height)) {
        return 2;
    }

    const cv::Size sensor_res (cam[0].resolution.height, cam[0].resolution.width);
    const cv::Size camera_res (cam[0].resolution.width, cam[0].resolution.height);

    cv::Ptr<cv::StereoBM> stereo = nullptr; //*/ cv::StereoBM::create(16, 15);

    cv::VideoCapture cap;
    cv::Mat frame;

    cap.open(video_filename, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        return 3;
    }

    cap.set(cv::CAP_PROP_FORMAT, -1);
    cap.set(cv::CAP_PROP_CONVERT_RGB, false);

    cv::Mat img = cv::Mat(sensor_res, CV_8UC1);
    cv::Mat left = cv::Mat(camera_res, img.type());
    cv::Mat right = cv::Mat(camera_res, img.type());

    cv::Mat indices0;
    cv::Mat indices1;

    cv::Mat map0_fisheye624 = build_maps_variant(apply_fisheye624, cam[0], indices0);
    cv::Mat map1_fisheye624 = build_maps_variant(apply_fisheye624, cam[1], indices1);

    cv::Mat undist_left = cv::Mat(left.size(), left.type());
    cv::Mat undist_right = cv::Mat(right.size(), right.type());

    std::time_t time_start = std::time(nullptr);
    std::time_t time_current;
    uint32_t frames = 0;

    while (true) {
        cap.read(frame);
        if (frame.empty()) {
            break;
        }

        time_current = std::time(nullptr);
        frames++;

        if (time_current > time_start) {
            std::cout << (frames / 2) << " frames per second" << std::endl;

            time_start = time_current;
            frames = 0;
        }

        if (frame.cols < CAM_BUFFER_SIZE) {
            continue;
        }

        uint8_t *blocks = frame.ptr();
        uint8_t *data = img.ptr();

        const uint8_t *header = blocks + CAM_IMAGE_DATA_SIZE;

        uint64_t t_ns = *((const uint64_t *)(header + 0x00));
        uint64_t t_ms = *((const uint64_t *)(header + 0x3E));

        uint16_t seq = *((const uint16_t *)(header + 0x12));
        uint8_t is_right = *(header + 0x3B);

        size_t offset = 0;
        size_t sum = CHUNK_SUM_LEN * 0xFF + 1;

        for (size_t i = 0; i < CHUNK_AMOUNT; i++) {
            size_t val = 0;

            for (size_t j = 0; j < CHUNK_SUM_LEN; j++) {
                val += blocks[CHUNK_SIZE * i + j];
            }

            if (val < sum) {
                offset = i;
                sum = val;
            }
        }

        for (size_t i = 0; i < CHUNK_AMOUNT; i++) {
            if (chunk_map[i] == offset) {
                offset = i;
                break;
            }
        }

        for (size_t i = 0; i < CHUNK_AMOUNT; i++) {
            size_t src_idx = CHUNK_SIZE * chunk_map[(offset + i) % CHUNK_AMOUNT];
            size_t dst_idx = CHUNK_SIZE * i;

            std::memcpy(data + dst_idx, blocks + src_idx, CHUNK_SIZE);
        }

        if (is_right) {
            cv::rotate(img, right, cv::ROTATE_90_CLOCKWISE);
        } else {
            cv::rotate(img, left, cv::ROTATE_90_COUNTERCLOCKWISE);
        }

        if ((left.empty()) || (right.empty())) {
            continue;
        }

        cv::remap(left, undist_left, map0_fisheye624, indices0, cv::INTER_LINEAR);
        cv::remap(right, undist_right, map1_fisheye624, indices1, cv::INTER_LINEAR);

        if (stereo) {
            cv::Mat disparity;
            cv::Mat disp8;

            stereo->compute(undist_left, undist_right, disparity);

            disparity.convertTo(disp8, CV_8U, 1.0f / stereo->getNumDisparities());

            cv::imshow("Live", disp8);
        } else {
            cv::Mat row;

            cv::hconcat(undist_left, undist_right, row);

            cv::imshow("Live", row);
        }

        if (cv::waitKey(5) >= 0) {
            break;
        }
    }

    cap.release();
    cv::destroyAllWindows();

    return 0;
}
