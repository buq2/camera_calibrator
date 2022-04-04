#include "log_plot.hh"
#include <algorithm>
#include <map>
#include "imgui.h"
#include "implot.h"

PlotLog &PlotLog::Get() {
  static PlotLog logger;
  return logger;
}

void PlotLog::Log(const std::string &name, const double val) {
  std::scoped_lock lock(mutex_);
  auto &pd = data_[name];
  auto &cb = pd.points;
  if (cb.capacity() == 0) {
    cb.reserve(default_plot_size_);
    pd.earliest_index = index_;
  }
  cb.push_back({index_, val});
  ++index_;
}

template <typename T>
void DisplayList(const std::string &name, const T &iterable,
                 std::set<std::string> &selected) {
  std::string full_name("## ");
  full_name += name;

  // TODO: Do not hardcode the margin
  // Without the margin two or more lists will take too much
  // space and horizontal scroll bar will be created
  constexpr float margin = 20.0f;
  if (ImGui::BeginListBox(full_name.c_str(),
                          ImVec2(ImGui::GetWindowWidth() / 2 - margin,
                                 5 * ImGui::GetTextLineHeightWithSpacing()))) {
    for (const auto &it : iterable) {
      const bool is_selected = selected.count(it.first);
      if (ImGui::Selectable(it.first.c_str(), is_selected)) {
        if (selected.count(it.first)) {
          selected.erase(it.first);
        } else {
          selected.insert(it.first);
        }
      }
      if (is_selected) {
        ImGui::SetItemDefaultFocus();
      }
    }
    ImGui::EndListBox();
  }
}

std::tuple<std::vector<double>, std::vector<double>, std::string>
PlotLog::GetData(const std::string &sel_plot) {
  const auto &pd = data_[sel_plot];
  const auto &cb = pd.points;
  std::vector<double> x;
  std::vector<double> y;

  const auto &xaxis = selected_x_data_[sel_plot];
  const std::string plot_name = sel_plot + " " + xaxis;
  if (xaxis == "" || xaxis == "<index>") {
    for (const auto &p : cb) {
      x.push_back(static_cast<double>(p.index));
      y.push_back(p.value);
    }
  } else {
    // Use some other plot as x-axis
    const auto &pd_x = data_[xaxis];
    const auto &cb_x = pd_x.points;

    auto fill_data = [](const auto &cb_x, const auto &cb_y, auto &x, auto &y) {
      auto comp = [](const auto &point, const auto &index) {
        return point.index < index;
      };

      auto it_x = cb_x.begin();
      for (size_t i = 0; i < cb_y.size(); ++i) {
        const auto &p = cb_y[i];
        const auto idx = p.index;

        const auto it = std::lower_bound(it_x, cb_x.end(), idx, comp);
        if (it == cb_x.end()) {
          break;
        }
        // No need to search earlier points again
        it_x = it;

        x.push_back(it->value);
        y.push_back(p.value);
      }
    };

    if (pd_x.earliest_index < pd.earliest_index) {
      // x values were plotted first -> for each y, use lower bound to find
      // matching x
      fill_data(cb_x, cb, x, y);
    } else {
      // y values were plotted first -> for each x, use lower bound to find
      // matching y
      fill_data(cb, cb_x, y, x);
    }
  }

  return {x, y, plot_name};
}

void PlotLog::Display() {
  std::scoped_lock lock(mutex_);
  // ImPlot::ShowDemoWindow();

  ImGui::Begin("Plot Log", NULL, ImGuiWindowFlags_HorizontalScrollbar);
  ImGui::Text("Display plots:");
  DisplayList("Plots", data_, selected_plots_);

  // TODO: Not very nice way to create the plot types.
  std::unordered_map<std::string, int> plot_types{{"scatter", 0}, {"line", 0}};
  std::set<std::string> selected_plot_type;
  DisplayList("Plot types", plot_types, selected_plot_type);

  // TODO: Same, use set instead
  std::map<std::string, int> x_data{{"<index>", 0}};
  for (const auto &d : data_) {
    x_data[d.first] = 0;
  }
  std::set<std::string> selected_x_data;
  DisplayList("X data", x_data, selected_x_data);

  ImPlot::BeginPlot("Debug plot");
  for (const auto &sel_plot : selected_plots_) {
    // Change plot type?
    if (selected_plot_type.size()) {
      types_[sel_plot] = *selected_plot_type.begin();
    }

    // Change x-axis data source?
    if (selected_x_data.size()) {
      selected_x_data_[sel_plot] = *selected_x_data.begin();
    }

    const auto &[x, y, name] = GetData(sel_plot);

    const auto &type = types_[sel_plot];
    const auto num = static_cast<int>(x.size());
    if (type == "line") {
      ImPlot::PlotLine(name.c_str(), x.data(), y.data(), num);
    } else if (type == "scatter" || type == "") {
      ImPlot::PlotScatter(name.c_str(), x.data(), y.data(), num);
    }
  }
  ImPlot::EndPlot();

  ImGui::End();
}
