#include "corners.hh"
#include <Eigen/Core>
#include <opencv2/imgproc.hpp>
#include "log_image.hh"

using namespace calibrator;

inline float squared(float x) { return x * x; }

// https://stackoverflow.com/a/29397597
const float ONE_OVER_SQRT_2PI = 0.39894228040143267793994605993438f;
float normpdf(float x, float u, float s) {
  return (ONE_OVER_SQRT_2PI / s) * expf(-0.5f * squared((x - u) / s));
}

CorrelationPatch::CorrelationPatch(const float angle1, const float angle2,
                                   const uint8_t radius) {
  const int wh = radius * 2 + 1;
  a1_ = cv::Mat::zeros({wh, wh}, CV_32F);
  a2_ = cv::Mat::zeros({wh, wh}, CV_32F);
  b1_ = cv::Mat::zeros({wh, wh}, CV_32F);
  b2_ = cv::Mat::zeros({wh, wh}, CV_32F);

  const int mid = radius + 1;
  const Eigen::Vector2f n1{-sinf(angle1), cosf(angle1)};
  const Eigen::Vector2f n2{-sinf(angle2), cosf(angle2)};

  float a1sum = 0.0f;
  float a2sum = 0.0f;
  float b1sum = 0.0f;
  float b2sum = 0.0f;

  const float half_radius = radius / 2.0f;

  for (int i = 0; i < wh; ++i) {
    for (int j = 0; j < wh; ++j) {
      Eigen::Vector2f vec{i + 1 - mid, j + 1 - mid};
      const auto dist = vec.norm();
      const auto s1 = vec.dot(n1);
      const auto s2 = vec.dot(n2);

      const auto val = normpdf(dist, 0.0f, half_radius);
      if (s1 <= -0.1 && s2 <= -0.1) {
        a1sum += val;
        a1_.at<float>(i, j) = val;
      } else if (s1 >= 0.1 && s2 >= 0.1) {
        a2sum += val;
        a2_.at<float>(i, j) = val;
      } else if (s1 <= -0.1 && s2 >= 0.1) {
        b1sum += val;
        b1_.at<float>(i, j) = val;
      } else if (s1 >= 0.1 && s2 <= -0.1) {
        b2sum += val;
        b2_.at<float>(i, j) = val;
      }
    }
  }

  a1_ /= a1sum;
  a2_ /= a2sum;
  b1_ /= b1sum;
  b2_ /= b2sum;

  LOG_IMAGE("a1", a1_);
  LOG_IMAGE("a2", a2_);
  LOG_IMAGE("b1", b1_);
  LOG_IMAGE("b2", b2_);
}

CorrelationPatch::CorrelationPatch() {}

CorrelationPatch::CorrelationPatch(const CorrelationPatch& other)
    : a1_(other.a1_), a2_(other.a2_), b1_(other.b1_), b2_(other.b2_) {}

CorrelationPatch::CorrelationPatch(CorrelationPatch&& other) {
  *this = std::move(other);
}
CorrelationPatch& CorrelationPatch::operator=(const CorrelationPatch& other) {
  a1_ = other.a1_;
  a2_ = other.a2_;
  b1_ = other.b1_;
  b2_ = other.b2_;
  return *this;
}

CorrelationPatch& CorrelationPatch::operator=(CorrelationPatch&& other) {
  a1_ = std::move(other.a1_);
  a2_ = std::move(other.a2_);
  b1_ = std::move(other.b1_);
  b2_ = std::move(other.b2_);
  return *this;
}

cv::Mat CorrelationPatch::Detect(const cv::Mat& img_gray) const {
  cv::Mat a1f, a2f, b1f, b2f;
  cv::filter2D(img_gray, a1f, -1, a1_);
  cv::filter2D(img_gray, a2f, -1, a2_);
  cv::filter2D(img_gray, b1f, -1, b1_);
  cv::filter2D(img_gray, b2f, -1, b2_);

  LOG_IMAGE("a1f", a1f);
  LOG_IMAGE("a2f", a2f);
  LOG_IMAGE("b1f", b1f);
  LOG_IMAGE("b2f", b2f);

  const cv::Mat mean = (a1f + a2f + b1f + b2f) / 4.0f;
  cv::Mat a = cv::min(cv::Mat(a1f - mean), cv::Mat(a2f - mean));
  cv::Mat b = cv::min(cv::Mat(mean - b1f), cv::Mat(mean - b2f));
  const cv::Mat ab1 = cv::min(a, b);

  a = cv::min(cv::Mat(mean - a1f), cv::Mat(mean - a2f));
  b = cv::min(cv::Mat(b1f - mean), cv::Mat(b2f - mean));
  const cv::Mat ab2 = cv::min(a, b);

  return cv::max(ab1, ab2);
}

std::tuple<cv::Mat, cv::Mat> ConernerDetector::FilterSobel(
    const cv::Mat& img_gray) {
  cv::Mat gx, gy;
  // OpenCV Sobel uses [-1,0,1;-2,0,2;-1,0,1]. Original implementation
  // uses [-1,0,1;-1,0,1;-1,0,1]
  cv::Sobel(img_gray, gx, -1, 1, 0);
  cv::Sobel(img_gray, gy, -1, 0, 1);
  return {gx, gy};
}

cv::Mat ConernerDetector::AngleFromSobel(const cv::Mat& gx, const cv::Mat& gy) {
  cv::Mat angle = cv::Mat::zeros(gx.rows, gx.cols, gx.type());
  for (int row = 0; row < angle.rows; ++row) {
    auto gx_row = reinterpret_cast<const float*>(gx.ptr<const float>(row));
    auto gy_row = reinterpret_cast<const float*>(gy.ptr<const float>(row));
    auto angle_row = angle.ptr<float>(row);
    for (int col = 0; col < angle.cols; ++col) {
      angle_row[col] = atan2f(gy_row[col], gx_row[col]);
    }
  }
  return angle;
}

cv::Mat ConernerDetector::WeightFromSobel(const cv::Mat& gx,
                                          const cv::Mat& gy) {
  cv::Mat weight = cv::Mat::zeros(gx.rows, gx.cols, gx.type());
  for (int row = 0; row < weight.rows; ++row) {
    auto gx_row = reinterpret_cast<const float*>(gx.ptr<const float>(row));
    auto gy_row = reinterpret_cast<const float*>(gy.ptr<const float>(row));
    auto weight_row = weight.ptr<float>(row);
    for (int col = 0; col < weight.cols; ++col) {
      const auto x = gx_row[col];
      const auto y = gy_row[col];

      // We could probably leave the sqrt out and scale
      // other computations
      weight_row[col] = sqrtf(x * x + y * y);
    }
  }
  return weight;
}

cv::Mat ConernerDetector::ScaleImage(const cv::Mat& img_gray) {
  double min, max;
  cv::minMaxLoc(img_gray, &min, &max);
  return (img_gray - min) / (max - min);
}

ConernerDetector::GxGyAngleWeight ConernerDetector::GradientsAngleAndWeight(
    const cv::Mat& img_gray) {
  GxGyAngleWeight out;
  const auto& [gx, gy] = FilterSobel(img_gray);
  out.gx = gx;
  out.gy = gy;
  out.angle = AngleFromSobel(out.gx, out.gy);
  out.weight = WeightFromSobel(out.gx, out.gy);
  return out;
}

Points2D ConernerDetector::NMS(const cv::Mat& corners, const float n,
                               const float tau, const float margin) const {
  Points2D points;
  // TODO: Implement
  return points;
}

void ConernerDetector::Detect(const cv::Mat& img_in) {
  LOG_IMAGE("corners_input", img_in);
  cv::Mat img;
  if (img_in.channels() == 3) {
    cv::cvtColor(img_in, img, cv::COLOR_BGR2GRAY);
  } else {
    img = img_in;
  }

  if (img.depth() != CV_32F) {
    img.convertTo(img, CV_32F);
  }
  LOG_IMAGE("corners_input_converter", img);
  img = ScaleImage(img);
  LOG_IMAGE("corners_input_scaled", img);
  cv::Mat corners = cv::Mat::zeros(img.rows, img.cols, img.type());
  for (const auto& t : templates_) {
    const auto new_corners = t.Detect(img);
    corners = cv::max(corners, new_corners);
  }
  LOG_IMAGE("corners_final", corners);
  const auto selected_corners = NMS(corners, 3, 0.025f, 5);

  // Refinement
  const auto gxgyaw = GradientsAngleAndWeight(img);
  const auto refined_corners = RefineCorners(selected_corners, gxgyaw, 10);

  const auto scored_corners =
      ScoreCorners(selected_corners, img, gxgyaw.angle, gxgyaw.weight);
}

ConernerDetector::ScoredCorners ConernerDetector::ScoreCorners(
    const Points2D& corners, const cv::Mat& img, const cv::Mat& angle,
    const cv::Mat& weight) {
  ScoredCorners out;
  // TODO: Implement
  return out;
}

Points2D ConernerDetector::RefineCorners(const Points2D& corners,
                                         const GxGyAngleWeight& gxgyaw,
                                         const uint8_t radius) {
  Points2D out;
  // TODO: Implement
  return out;
}

ConernerDetector::ConernerDetector() {
  const auto pi = 3.141592653589793f;
  templates_.emplace_back(0.0f, pi / 2.0f, 4);
  templates_.emplace_back(pi / 4.0f, -pi / 4.0f, 4);
  templates_.emplace_back(0.0f, pi / 2.0f, 8);
  templates_.emplace_back(pi / 4.0f, -pi / 4.0f, 8);
  templates_.emplace_back(0.0f, pi / 2.0f, 12);
  templates_.emplace_back(pi / 4.0f, -pi / 4.0f, 12);
}
