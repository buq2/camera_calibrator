#pragma once

#include <mutex>
#include <string>
#include <unordered_map>
#include "gui.hh"

namespace calibrator {

class ImageLog {
 public:
  static ImageLog& Get();
  void Log(const std::string& name, const cv::Mat& img);
  void Display();

 private:
  ImageLog() {}
  std::unordered_map<std::string, Image> imgs_;
  std::mutex mutex_;
  std::string selected_;
};  // class ImageLog

}  // namespace calibrator

#define LOG_IMAGE(name, img)                    \
  do {                                          \
    calibrator::ImageLog::Get().Log(name, img); \
  } while (0)
