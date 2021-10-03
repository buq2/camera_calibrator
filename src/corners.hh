#pragma once

#include <opencv2/core.hpp>
#include "types.hh"

namespace calibrator {

class CorrelationPatch {
 public:
  CorrelationPatch(const float angle1, const float angle2,
                   const uint8_t radius);
  CorrelationPatch();
  CorrelationPatch(const CorrelationPatch& other);
  CorrelationPatch(CorrelationPatch&& other);
  CorrelationPatch& operator=(const CorrelationPatch& other);
  CorrelationPatch& operator=(CorrelationPatch&& other);

  cv::Mat Detect(const cv::Mat& img_gray) const;

 private:
  cv::Mat a1_, a2_, b1_, b2_;
};  // class CorrelationPatch

class ConernerDetector {
 public:
  ConernerDetector();
  void Detect(const cv::Mat& img_in);

  struct ScoredCorners {
    Points2D points;
    std::vector<float> scores;
  };

 private:
  struct GxGyAngleWeight {
    cv::Mat gx;
    cv::Mat gy;
    cv::Mat angle;
    cv::Mat weight;
  };
  std::tuple<cv::Mat, cv::Mat> FilterSobel(const cv::Mat& img_gray);
  GxGyAngleWeight GradientsAngleAndWeight(const cv::Mat& img_gray);
  static cv::Mat AngleFromSobel(const cv::Mat& gx, const cv::Mat& gy);
  static cv::Mat WeightFromSobel(const cv::Mat& gx, const cv::Mat& gy);
  cv::Mat ScaleImage(const cv::Mat& img_gray);
  Points2D NMS(const cv::Mat& corners, const float n, const float tau,
               const float margin) const;
  Points2D RefineCorners(const Points2D& corners, const GxGyAngleWeight& gxgyaw,
                         const uint8_t radius);
  ScoredCorners ScoreCorners(const Points2D& corners, const cv::Mat& img,
    const cv::Mat& angle, const cv::Mat& weight);

 private:
  std::vector<CorrelationPatch> templates_;
  float tau_{0.01f};
  std::vector<uint8_t> radiuses_{{4,8,12}};
};  // class ConernerDetector

}  // namespace calibrator
