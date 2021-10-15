#include "log_image.hh"
#include "imgui.h"

using namespace calibrator;

ImageLog& ImageLog::Get() {
  static ImageLog logger;
  return logger;
}

void ImageLog::Log(const std::string& name, const cv::Mat& img_in) {
  std::scoped_lock lock(mutex_);
  imgs_[name].SetImage(img_in);
}

void ImageLog::LogPoints2D(const std::string& name, const Points2D& points) {
  std::scoped_lock lock(mutex_);
  points2d_[name] = points;
}

template <typename T>
void DisplayList(const std::string& name, const T& iterable,
                 std::string& selected) {
  std::string full_name("## ");
  full_name += name;
  
  // TODO: Do not hardcode the margin
  // Without the margin two or more lists will take too much
  // space and horizontal scroll bar will be created
  constexpr float margin = 20.0f;
  if (ImGui::BeginListBox(full_name.c_str(),
                          ImVec2(ImGui::GetWindowWidth() / 2 - margin,
                                 5 * ImGui::GetTextLineHeightWithSpacing()))) {
    for (const auto& it : iterable) {
      const bool is_selected = it.first == selected;
      if (ImGui::Selectable(it.first.c_str(), is_selected)) {
        selected = it.first;
      }
      if (is_selected) {
        ImGui::SetItemDefaultFocus();
      }
    }
    ImGui::EndListBox();
  }
}

void ImageLog::Display() {
  ImGui::Begin("Image Log", NULL, ImGuiWindowFlags_HorizontalScrollbar);
  ImGui::Text("Display image and points:");
  DisplayList("Image", imgs_, selected_img_);
  ImGui::SameLine();
  DisplayList("Points", points2d_, selected_points2d_);

  // Do we have image selected?
  auto it_img = imgs_.find(selected_img_);
  if (it_img != imgs_.end()) {
    auto& img = it_img->second;

    // Check if points selected
    auto it_p2 = points2d_.find(selected_points2d_);
    if (it_p2 != points2d_.end()) {
      // Draw points
      const auto& points = it_p2->second;

      it_img->second.SetObjectDrawCallback([&]() {
        auto dl = ImGui::GetWindowDrawList();
        for (const auto& p : points) {
          const auto [x, y] = img.GetImageDrawCoordinate(p(0), p(1));
          dl->AddCircleFilled({x, y}, 2 * img.GetScale(), 0xFF0000FF, 5);
        }
      });
    }

    img.Display();
  }

  ImGui::End();
}
