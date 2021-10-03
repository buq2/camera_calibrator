#include "log_image.hh"
#include "imgui.h"

using namespace calibrator;

ImageLog& ImageLog::Get() {
  static ImageLog logger;
  return logger;
}

void ImageLog::Log(const std::string& name, const cv::Mat& img_in) {
  cv::Mat img = img_in;
  std::scoped_lock lock(mutex_);
  imgs_[name].SetImage(img);
}

void ImageLog::Display() {
  ImGui::Begin("Image Log", NULL, ImGuiWindowFlags_HorizontalScrollbar);
  ImGui::Text("Display image:");
  if (ImGui::BeginListBox(
          "## Display image:",
          ImVec2(-FLT_MIN, 5 * ImGui::GetTextLineHeightWithSpacing()))) {
    for (const auto& it : imgs_) {
      const bool is_selected = it.first == selected_;
      if (ImGui::Selectable(it.first.c_str(), is_selected)) {
        selected_ = it.first;
      }
      if (is_selected) {
        ImGui::SetItemDefaultFocus();
      }
    }
    ImGui::EndListBox();
  }

  auto it = imgs_.find(selected_);
  if (it != imgs_.end()) {
    it->second.Display();
  }

  ImGui::End();
}
