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
  std::tuple<std::vector<double>, std::vector<double>, std::string> GetData(const std::string sel_plot);

  std::set<std::string> selected_plots_;
  std::mutex mutex_;
  uint64_t index_{0};
  size_t default_plot_size_{1000};
  struct PlotPoint {
    uint64_t index;
    double value;
  };
  struct PlotData {
    CircularBuffer<PlotPoint> points;

    // Save earliest seen index as this allows us to determine
    // if "val x" was plotted first or "val y".
    // When searching for pairs, if "val x" has smaller earliest_index
    // then there is no reason to use "std::lower_bound"
    // to pair "val x"s to "val y" vector, instead use
    // std::upper_bound or reverse search.
    uint64_t earliest_index{std::numeric_limits<uint64_t>::max()};
  };
  std::unordered_map<std::string, std::string> types_;
  std::unordered_map<std::string, std::string> selected_x_data_;
  std::unordered_map<std::string, PlotData > data_;
}; // class PlotLog

#define LOG_POINT(name, val)       \
  do {                             \
    PlotLog::Get().Log(name, val); \
  } while (0)
