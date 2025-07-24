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

#include <cstdint>
#include <cstring>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <stdio.h>

#include "device_imu.h"

const uint8_t chunk_map [128] = {
    119, 54,  21,  0,   108, 22,  51,  63,  93, 99,  67, 7,   32,  112, 52,
    43,  14,  35,  75,  116, 64,  71,  44,  89, 18,  88, 26,  61,  70,  56,
    90,  79,  87,  120, 81,  101, 121, 17,  72, 31,  53, 124, 127, 113, 111,
    36,  48,  19,  37,  83,  126, 74,  109, 5,  84,  41, 76,  30,  110, 29,
    12,  115, 28,  102, 105, 62,  103, 20,  3,  68,  49, 77,  117, 125, 106,
    60,  69,  98,  9,   16,  78,  47,  40,  2,  118, 34, 13,  50,  46,  80,
    85,  66,  42,  123, 122, 96,  11,  25,  97, 39,  6,  86,  1,   8,   82,
    92,  59,  104, 24,  15,  73,  65,  38,  58, 10,  23, 33,  55,  57,  107,
    100, 94,  27,  95,  45,  91,  4,   114};

uint8_t reverse_chunk_map [128];

#define CHUNK_SIZE 2400
#define CHUNK_AMOUNT 128
#define CHUNK_SUM_LEN 128

#define CAM_WIDTH 640
#define CAM_HEIGHT 480
#define CAM_HEADER_LEN 2

#define CAM_BUFFER_SIZE ((CAM_HEIGHT + CAM_HEADER_LEN) * CAM_WIDTH)
#define CAM_IMAGE_DATA_SIZE (CAM_HEIGHT * CAM_WIDTH)

struct CameraCalibration {
    float position [3];
    float rotation [9];

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

    float fx_t;
    float fy_t;
    float cx_t;
    float cy_t;
};

void calculate_camera_matrices(const CameraCalibration& cam, cv::Mat& K, cv::Mat& D, cv::Mat& P, cv::Mat& R) {
    K = cv::Mat::eye(3, 3, CV_64FC1);
    D = cv::Mat::zeros(1, 12, CV_64FC1);
    P = cv::Mat::zeros(3, 1, CV_64FC1);
    R = cv::Mat::eye(3, 3, CV_64FC1);

    K.at<double>(0, 0) = cam.fx;
    K.at<double>(1, 1) = cam.fy;
    K.at<double>(0, 2) = cam.cx;
    K.at<double>(1, 2) = cam.cy;

    if (cam.num_kc >= 6) {
        D.at<double>(0, 0) = cam.k1;
        D.at<double>(0, 1) = cam.k2;
        D.at<double>(0, 4) = cam.k3;
        D.at<double>(0, 5) = cam.k4;
        D.at<double>(0, 6) = cam.k5;
        D.at<double>(0, 7) = cam.k6;
    }

    if (cam.num_kc >= 8) {
        D.at<double>(0, 2) = cam.p1;
        D.at<double>(0, 3) = cam.p2;
    }

    if (cam.num_kc >= 12) {
        D.at<double>(0, 8) = cam.s1;
        D.at<double>(0, 9) = cam.s2;
        D.at<double>(0, 10) = cam.s3;
        D.at<double>(0, 11) = cam.s4;
    }

    P.at<double>(0) = cam.position[0];
    P.at<double>(1) = cam.position[1];
    P.at<double>(2) = cam.position[2];

    R.at<double>(0, 0) = cam.rotation[0];
    R.at<double>(0, 1) = cam.rotation[1];
    R.at<double>(0, 2) = cam.rotation[2];
    R.at<double>(1, 0) = cam.rotation[3];
    R.at<double>(1, 1) = cam.rotation[4];
    R.at<double>(1, 2) = cam.rotation[5];
    R.at<double>(2, 0) = cam.rotation[6];
    R.at<double>(2, 1) = cam.rotation[7];
    R.at<double>(2, 2) = cam.rotation[8];
}

typedef cv::Point2f (*apply_variant_func)(const CameraCalibration& cam, float xn, float yn);

cv::Point2f apply_intrinsic_only(const CameraCalibration& cam, float xn, float yn) {
    cv::Point2f p;

    p.x = cam.fx_t * xn + cam.cx_t;
    p.y = cam.fy_t * yn + cam.cy_t;

    return p;
}

cv::Point2f apply_radial_only(const CameraCalibration& cam, float xn, float yn) {
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

    return apply_intrinsic_only(cam, xn, yn);
}

cv::Point2f apply_radial_tangential(const CameraCalibration& cam, float xn, float yn) {
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

    return apply_intrinsic_only(cam, xn, yn);
}

cv::Point2f apply_fisheye624(const CameraCalibration& cam, float xn, float yn) {
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

    return apply_intrinsic_only(cam, xn, yn);
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

    memset(&(cam[0]), 0, sizeof(cam[0]));
    memset(&(cam[1]), 0, sizeof(cam[1]));

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

            const device_imu_vec3_type position = device_imu_sensor_get_position(sensor);
            const device_imu_mat3x3_type rotation = device_imu_sensor_get_rotation(sensor);

            const device_imu_size_type resolution = device_imu_sensor_get_resolution(sensor);

            const device_imu_vec2_type cc = device_imu_sensor_get_cc(sensor);
            const device_imu_vec2_type fc = device_imu_sensor_get_fc(sensor);

            uint32_t num_kc = 0;
            if (DEVICE_IMU_ERROR_NO_ERROR != device_imu_sensor_get_kc(sensor, &num_kc, nullptr)) {
                continue;
            }

            memcpy(cam[i].position, (const float*) &(position), sizeof(float) * 3);
            memcpy(cam[i].rotation, (const float*) &(rotation), sizeof(float) * 9);

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

            cam[i].fx_t = cam[i].fx;
            cam[i].fy_t = cam[i].fy;

            cam[i].cx_t = cam[i].cx;
            cam[i].cy_t = cam[i].cy;
        }
    }

	device_imu_close(&dev);

    if ((cam[0].resolution.width != cam[1].resolution.width) ||
        (cam[0].resolution.height != cam[1].resolution.height)) {
        return 2;
    }

    const cv::Size sensor_res (cam[0].resolution.height, cam[0].resolution.width);
    const cv::Size camera_res (cam[0].resolution.width, cam[0].resolution.height);

    const float sigmaColor = 50.0F;
    const float sigmaSpace = 25.0F;

    cv::Ptr<cv::StereoBM> stereo = nullptr;

    if ((argc > 2) && (0 == strcmp(argv[2], "--depth"))) {
        stereo = cv::StereoBM::create(16, 9);
    }

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

    for (size_t i = 0; i < CHUNK_AMOUNT; i++) {
        reverse_chunk_map[chunk_map[i]] = i;
    }

    cv::Mat K [2];
    cv::Mat D [2];
    cv::Mat P [2];
    cv::Mat R [2];

    calculate_camera_matrices(cam[0], K[0], D[0], P[0], R[0]);
    calculate_camera_matrices(cam[1], K[1], D[1], P[1], R[1]);

    cv::Mat cR = R[0].inv() * R[1];
    cv::Mat cT = P[1] - P[0];

    cv::Mat Q;

    cv::stereoRectify(K[0], D[0], K[1], D[1], camera_res, cR, cT, R[0], R[1], P[0], P[1], Q);

    cv::Mat indices0;
    cv::Mat indices1;

    cam[0].cx_t -= P[0].at<double>(0, 3);
    cam[0].cy_t -= P[0].at<double>(1, 3);

    cam[1].cx_t -= P[1].at<double>(0, 3);
    cam[1].cy_t -= P[1].at<double>(1, 3);

    cv::Mat map0_fisheye624 = build_maps_variant(apply_fisheye624, cam[0], indices0);
    cv::Mat map1_fisheye624 = build_maps_variant(apply_fisheye624, cam[1], indices1);

    cv::Mat undist_left = cv::Mat(left.size(), left.type());
    cv::Mat undist_right = cv::Mat(right.size(), right.type());

    uint64_t t_last = 0;
    uint32_t frames = 0;

    while (true) {
        cap.read(frame);
        if (frame.empty()) {
            break;
        }

        if (frame.cols < CAM_BUFFER_SIZE) {
            continue;
        }

        uint8_t *blocks = frame.ptr();
        uint8_t *data = img.ptr();

        const uint8_t *header = blocks + CAM_IMAGE_DATA_SIZE;
        const size_t header_len = CAM_BUFFER_SIZE - CAM_IMAGE_DATA_SIZE;

        uint64_t t_eye = *((const uint64_t *)(header + 0x00));
        uint64_t t_frame = *((const uint64_t *)(header + 0x3E));

        uint16_t seq = *((const uint16_t *)(header + 0x12));
        uint32_t aperture = *((const uint32_t *)(header + 0x16));
        uint16_t exposure = *((const uint16_t *)(header + 0x1A));
        uint64_t number = *((const uint64_t *)(header + 0x33));
        uint8_t is_right = *(header + 0x3B);

        frames++;

        if ((t_frame - t_last) > 1000000000L) {
            std::cout << (frames / 2) << " frames per second" << std::endl;

            t_last = t_frame;
            frames = 0;
        }

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

        offset = reverse_chunk_map[offset];

        for (size_t i = 0; i < CHUNK_AMOUNT; i++) {
            const size_t src_idx = CHUNK_SIZE * chunk_map[(offset + i) % CHUNK_AMOUNT];
            const size_t dst_idx = CHUNK_SIZE * i;

            std::memcpy(data + dst_idx, blocks + src_idx, CHUNK_SIZE);
        }

        if (is_right) {
            cv::Mat filtered;

            cv::bilateralFilter(img, filtered, 5, sigmaColor, sigmaSpace);
            cv::rotate(filtered, right, cv::ROTATE_90_CLOCKWISE);
            cv::remap(right, undist_right, map1_fisheye624, indices1, cv::INTER_LINEAR);

            if (undist_left.empty()) {
                continue;
            }
        } else {
            cv::Mat filtered;

            cv::bilateralFilter(img, filtered, 5, sigmaColor, sigmaSpace);
            cv::rotate(filtered, left, cv::ROTATE_90_COUNTERCLOCKWISE);
            cv::remap(left, undist_left, map0_fisheye624, indices0, cv::INTER_LINEAR);
            
            if (undist_right.empty()) {
                continue;
            }
        }

        {
            cv::Mat row;

            if (stereo) {
                cv::Mat disparity;
                stereo->compute(undist_left, undist_right, disparity);
                disparity.convertTo(row, CV_8U, 128.0f / (stereo->getNumDisparities() * 16.0f));
            } else {
                cv::hconcat(undist_left, undist_right, row);
                
                for (size_t i = 0; i < 640; i += 40) {
                    cv::line(row, cv::Point(0, i), cv::Point(960, i), cv::Scalar(255, 0, 0));
                }
            }

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
