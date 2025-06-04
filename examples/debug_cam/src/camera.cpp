
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <stdio.h>

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

const float fx = 239.9401992353717F;
const float fy = 240.1440084390103F;
const float cx = 244.88379123391678F;
const float cy = 315.48779284276173F;

const float k1 = -0.00023167964822506118F;
const float k2 = +0.0936140967533583F;
const float k3 = -0.12407217981118918F;
const float k4 = +0.07253328684439207F;
const float k5 = -0.02173257609236286F;
const float k6 = +0.0026589892415407887F;
const float p1 = +0.002702726113704367F;
const float p2 = -0.003645482535483966F;
const float s1 = -0.00747665631898417F;
const float s2 = -0.0003866048077984273F;
const float s3 = +0.007856725692160133F;
const float s4 = +0.0003370697673688772F;

typedef cv::Point2f (*apply_variant_func)(float xn, float yn);

cv::Point2f apply_intrinsic_only(float xn, float yn) {
  cv::Point2f p;

  p.x = fx * xn + cx;
  p.y = fy * yn + cy;

  return p;
}

cv::Point2f apply_radial_only(float xn, float yn) {
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

  th_dist += k1 * th3;
  th_dist += k2 * th2 * th3;
  th_dist += k3 * th2 * th2 * th3;
  th_dist += k4 * th3 * th3 * th3;
  th_dist += k5 * th3 * th3 * th2 * th3;
  th_dist += k6 * th3 * th3 * th2 * th2 * th3;

  xn = th_dist * cp;
  yn = th_dist * sp;

  p.x = fx * xn + cx;
  p.y = fy * yn + cy;

  return p;
}

cv::Point2f apply_radial_tangential(float xn, float yn) {
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

  th_dist += k1 * th3;
  th_dist += k2 * th2 * th3;
  th_dist += k3 * th2 * th2 * th3;
  th_dist += k4 * th3 * th3 * th3;
  th_dist += k5 * th3 * th3 * th2 * th3;
  th_dist += k6 * th3 * th3 * th2 * th2 * th3;

  xn = th_dist * cp;
  yn = th_dist * sp;

  r = xn * xn + yn * yn;

  dx = (2.0F * xn * xn + r) * p1 + 2.0F * xn * yn * p2;
  dy = (2.0F * yn * yn + r) * p2 + 2.0F * xn * yn * p1;

  xn += dx;
  yn += dy;

  p.x = fx * xn + cx;
  p.y = fy * yn + cy;

  return p;
}

cv::Point2f apply_fisheye624(float xn, float yn) {
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

  th_dist += k1 * th3;
  th_dist += k2 * th5;
  th_dist += k3 * th5 * th2;
  th_dist += k4 * th6 * th3;
  th_dist += k5 * th6 * th5;
  th_dist += k6 * th6 * th5 * th2;

  xn = th_dist * cp;
  yn = th_dist * sp;

  r = xn * xn + yn * yn;

  dx = (2.0F * xn * xn + r) * p1 + 2.0F * xn * yn * p2;
  dy = (2.0F * yn * yn + r) * p2 + 2.0F * xn * yn * p1;

  dx += (s1 + s2 * r) * r;
  dy += (s3 + s4 * r) * r;

  xn += dx;
  yn += dy;

  p.x = fx * xn + cx;
  p.y = fy * yn + cy;

  return p;
}

cv::Mat build_maps_variant(apply_variant_func func, uint32_t width,
                           uint32_t height) {
  cv::Mat map = cv::Mat(cv::Size(height, width), CV_16SC2);
  cv::Point2f p;
  cv::Vec2s v;
  size_t x, y;

  for (y = 0; y < map.rows; y++) {
    for (x = 0; x < map.cols; x++) {
      const float xn = (x - cx) / fx;
      const float yn = (y - cy) / fy;

      p = func(xn, yn);

      v[0] = static_cast<short>(p.x);
      v[1] = static_cast<short>(p.y);

      map.at<cv::Vec2s>(y, x) = v;
    }
  }

  return map;
}

void remap(const cv::Mat &input, cv::Mat &output, const cv::Mat &map) {
  cv::Vec2s v;
  size_t x, y;

  for (y = 0; y < input.rows; y++) {
    for (x = 0; x < input.cols; x++) {
      v = map.at<cv::Vec2s>(y, x);

      output.at<uint8_t>(y, x) = input.at<uint8_t>(v[1], v[0]);
    }
  }
}

int main(int argc, const char **argv) {
  cv::VideoCapture cap;
  cv::Mat frame;

  cv::Ptr<cv::StereoBM> stereo = nullptr; // cv::StereoBM::create(16, 15);

  cap.open(0, cv::CAP_V4L2);
  if (!cap.isOpened()) {
    return 2;
  }

  cap.set(cv::CAP_PROP_FORMAT, -1);
  cap.set(cv::CAP_PROP_CONVERT_RGB, false);

  cv::Mat img = cv::Mat(cv::Size(CAM_WIDTH, CAM_HEIGHT), CV_8UC1);
  cv::Mat left = cv::Mat(cv::Size(CAM_HEIGHT, CAM_WIDTH), CV_8UC1);
  cv::Mat right = cv::Mat(cv::Size(CAM_HEIGHT, CAM_WIDTH), CV_8UC1);

  cv::Mat map_fisheye624 =
      build_maps_variant(apply_fisheye624, CAM_WIDTH, CAM_HEIGHT);

  cv::Mat undist_left = cv::Mat(left.size(), left.type());
  cv::Mat undist_right = cv::Mat(right.size(), right.type());

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

    // cv::remap(left, undist, map_intrinsic, cv::noArray(), cv::INTER_LINEAR);
    remap(left, undist_left, map_fisheye624);
    remap(right, undist_right, map_fisheye624);

    if (stereo) {
      cv::Mat disparity;
      cv::Mat disp8;

      stereo->compute(undist_left, undist_right, disparity);

      disparity.convertTo(disp8, CV_8U, 255.0f / (16.0f * 16.0f));

      cv::imshow("Live", disp8);
    } else {
      cv::Mat row;

      cv::hconcat(left, right, row);
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
