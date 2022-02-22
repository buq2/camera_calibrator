#include "corners.hh"
#include <Eigen/Core>
#include <opencv2/imgproc.hpp>
#include "log_image.hh"
#include <optional>

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
  // Could try Scharr too
  // cv::Sobel(img_gray, gx, -1, 1, 0);
  // cv::Sobel(img_gray, gy, -1, 0, 1);
  const cv::Matx33f kernel_gx(1.0f, 0.0f, -1.0f,
                              1.0f, 0.0f, -1.0f,
                              1.0f, 0.0f, -1.0f);

  cv::filter2D(img_gray, gx, -1, kernel_gx);
  cv::filter2D(img_gray, gy, -1, kernel_gx.t());
  
  return {gx, gy};
}

cv::Mat ConernerDetector::AngleFromSobel(const cv::Mat& gx, const cv::Mat& gy) {
  cv::Mat angle = cv::Mat::zeros(gx.rows, gx.cols, gx.type());
  constexpr auto pi = 3.141592653589793f;
  for (int row = 0; row < angle.rows; ++row) {
    auto gx_row = reinterpret_cast<const float*>(gx.ptr<const float>(row));
    auto gy_row = reinterpret_cast<const float*>(gy.ptr<const float>(row));
    auto angle_row = angle.ptr<float>(row);
    for (int col = 0; col < angle.cols; ++col) {
      auto angle = atan2f(gy_row[col], gx_row[col]);
      if (angle < 0) angle += pi;
      if (angle > pi) angle -= pi; // TODO: Check if we hit this
      angle_row[col] = angle;
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

Points2D ConernerDetector::NMS(const cv::Mat& corners, const uint8_t n,
                               const float tau, const float margin) const {
  assert(corners.depth() == CV_32F && corners.channels() == 1);
  assert(n % 2 == 1);

  Points2D points;

  const auto w = corners.cols;
  const auto h = corners.rows;
  const auto half_n = n / 2;

  // Find local maxima
  for (int row = half_n; row < h - half_n; ++row) {
    for (int col = half_n; col < w - half_n; ++col) {
      const auto val = corners.at<float>(row, col);

      // Ensure that the center pixel has larger value than tau
      if (val < tau) continue;

      // Ensure that center pixel is larger than all neighboring values
      bool fail = false;
      for (int i = -half_n; i <= half_n && !fail; ++i) {
        auto ptr = corners.ptr<float>(row + i);
        for (int j = -half_n; j <= half_n; ++j) {
          const auto& cmp_val = ptr[col + j];
          if (cmp_val > val) {
            fail = true;
            break;
          }
        }
      }

      if (!fail) {
        points.emplace_back(static_cast<float>(col), static_cast<float>(row));
      }
    }
  }

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
  const auto selected_corners = NMS(corners, 7, 0.025f, 5);
  LOG_POINTS_2D("points_nms", selected_corners);

  // Refinement
  const auto gxgyaw = GradientsAngleAndWeight(img);
  LOG_IMAGE("angle", gxgyaw.angle);
  LOG_IMAGE("weight", gxgyaw.weight);
  const auto refined_corners = RefineCorners(selected_corners, gxgyaw, 10);
  LOG_POINTS_2D("points_refined", refined_corners);

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

struct HistMode {
  explicit HistMode(int idx, float value):idx(idx), value(value) {}
  int idx{0};
  float value{0.0f};

  bool operator<(const HistMode& other) const {return value < other.value;}
  bool operator>(const HistMode& other) const {return value > other.value;}
};

std::vector<HistMode> FindModesMeanShift(const std::vector<float>& hist, float sigma) {
  // Convolve histogram with normpdf with wrap around semantics
  const auto n_hist = static_cast<int>(hist.size());
  std::vector<float> hist_smoothed(n_hist, 0.0f);
  const int rsigma2 = static_cast<int>(round(sigma*2));

  auto wrapmod = [](int i,int len) {
    // Wrap negative number such as -1 to len-1 etc. Assumes that i is never
    // too far negative :)
    return (i+len) % len;
  };

  for (int i = 0; i < n_hist; ++i) {
    for (int j = -rsigma2; j <= rsigma2; ++j) {
      // Wrap idx around the histogram
      const int idx = wrapmod(i+j, n_hist);
      hist_smoothed[i] += hist[idx] * normpdf(static_cast<float>(j), 0.0f, sigma);
    }
  }

  {
    // Check that there is at least one mode and hist is not constant
    bool nonzero = false;
    const auto first = hist_smoothed[0];
    for (int i = 1; i < n_hist; ++i) {
      if (std::abs(hist_smoothed[i] - first) > 1e-5) {
        nonzero = true;
        break;
      }
    }

    if (!nonzero) {
      // Histogram is flat, no modes
      return {};
    }
  }

  std::vector<HistMode> modes;
  for (int i = 0; i < n_hist; ++i) {
    const auto h0 = hist_smoothed[i];
    const auto j_prev = wrapmod(i-1, n_hist);
    const auto j_next = (i+1)%n_hist;
    const auto h_prev = hist_smoothed[j_prev];
    const auto h_next = hist_smoothed[j_next];

    if (h0 > h_prev && h0 > h_next) {
      // New mode
      modes.emplace_back(i, h0);
    }
  }

  std::sort(modes.begin(), modes.end(), std::greater<HistMode>());

  return modes;
}

/// Returns edge orientation angles in sorted order
std::optional<std::array<float,2>> EdgeOrientation(const Point2D& p, const int r, const cv::Mat& angle, const cv::Mat& weight) {
  constexpr int num_bins = 32;
  std::vector<float> hist(num_bins);

  const auto w = angle.cols;
  const auto h = angle.rows;

  assert(weight.cols == w);
  assert(weight.rows == h);
  assert(angle.channels() == 1);
  assert(weight.channels() == 1);
  assert(angle.depth() == CV_32F);
  assert(weight.depth() == CV_32F);

  const int x = static_cast<int>(roundf(p(0)));
  const int y = static_cast<int>(roundf(p(1)));

  constexpr auto pi = 3.141592653589793f;
  for (int i_y = std::max(0, y-r); i_y < std::min(h, y+r); ++i_y) {
    auto* ptr_angle = angle.ptr<float>(i_y);
    auto* ptr_weight = weight.ptr<float>(i_y);
    for (int i_x = std::max(0, x-r); i_x < std::min(w, x+r); ++i_x) {
      auto a = ptr_angle[i_x];
      
      // TODO: This is probably not needed as angles are already in range [0 pi]
      a += pi/2.0f;
      if (a >= pi) {
        a -= pi;
      }

      const auto binf = a/(pi/num_bins);
      const auto bin = std::max<int>(std::min<int>(static_cast<int>(binf), num_bins-1), 0);

      hist[bin] += ptr_weight[i_x];
    }
  }

  const auto modes = FindModesMeanShift(hist, 1);
  
  if (modes.size() < 2) {
    // Only one mode or no modes at all, this can not be a corner
    return std::nullopt;
  }

  // Take two largest modes
  const auto m0 = modes[0];
  const auto m1 = modes[1];
  // Compute mode angles
  auto angle0 = m0.idx*pi/num_bins;
  auto angle1 = m1.idx*pi/num_bins;
  // Sort
  if (angle0 > angle1) std::swap(angle0, angle1);
  const auto delta_angle = std::min(angle1-angle0, angle0+pi-angle1);

  if (delta_angle <= 0.3) return std::nullopt;
  return std::array<float,2>{angle0, angle1};
}

Points2D ConernerDetector::RefineCorners(const Points2D& corners,
                                         const GxGyAngleWeight& gxgyaw,
                                         const uint8_t radius) {
  Points2D out;
  
  for (const auto& corner : corners) {
    const auto orientation = EdgeOrientation(corner, radius, gxgyaw.angle, gxgyaw.weight);
    if (!orientation) continue;

    out.push_back(corner);
  }

  return out;
}

ConernerDetector::ConernerDetector() {
  constexpr auto pi = 3.141592653589793f;
  templates_.emplace_back(0.0f, pi / 2.0f, 4);
  templates_.emplace_back(pi / 4.0f, -pi / 4.0f, 4);
  templates_.emplace_back(0.0f, pi / 2.0f, 8);
  templates_.emplace_back(pi / 4.0f, -pi / 4.0f, 8);
  templates_.emplace_back(0.0f, pi / 2.0f, 12);
  templates_.emplace_back(pi / 4.0f, -pi / 4.0f, 12);
}
