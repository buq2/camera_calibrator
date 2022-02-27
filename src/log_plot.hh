#pragma once

#include <string>
#include "circular_buffer.hh"
#include <mutex>
#include <set>
#include <unordered_map>

class PlotLog {
 public:
  static PlotLog& Get();
  void Log(const std::string& name, const double val);
  void Display();
 private:
  PlotLog() {}
  std::tuple<std::vector<double>, std::vector<double>> GetData(const std::string sel_plot);

  std::set<std::string> selected_plots_;
  std::mutex mutex_;
  uint64_t index_{0};
  size_t default_plot_size_{1000};
  struct PlotPoint {
    uint64_t index;
    double value;
  };
  std::unordered_map<std::string, std::string> types_;
  std::unordered_map<std::string, std::string> selected_x_data_;
  std::unordered_map<std::string, CircularBuffer<PlotPoint> > data_;
}; // class PlotLog

#define LOG_POINT(name, val)       \
  do {                             \
    PlotLog::Get().Log(name, val); \
  } while (0)
