#pragma once

#include <mutex>
#include <string>
#include <unordered_map>
#include "gui.hh"
#include "types.hh"

namespace calibrator {

class ImageLog {
 public:
  static ImageLog& Get();
  void Log(const std::string& name, const cv::Mat& img);
  void LogPoints2D(const std::string& name, const Points2D& points);
  void Display();

 private:
  ImageLog() {}
  std::unordered_map<std::string, Image> imgs_;
  std::unordered_map<std::string, Points2D> points2d_;
  std::mutex mutex_;
  std::string selected_img_;
  std::string selected_points2d_;
};  // class ImageLog

}  // namespace calibrator

#define LOG_IMAGE(name, img)                    \
  do {                                          \
    calibrator::ImageLog::Get().Log(name, img); \
  } while (0)

#define LOG_POINTS_2D(name, points)                        \
  do {                                                     \
    calibrator::ImageLog::Get().LogPoints2D(name, points); \
  } while (0)
