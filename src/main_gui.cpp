#include <GL/glew.h>
#include <future>
#include <opencv2/highgui.hpp>
#include "corners.hh"
#include "gui.hh"
#include "imgui.h"
#include "log_image.hh"
#include "log_plot.hh"
#include "opencv2/imgproc.hpp"
#include "scene.hh"

using namespace calibrator;

template <typename T>
bool IsFutureReady(T &future) {
  // MSVC has _Is_ready(), but it is not included in GCC
  return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
}

int main(int argc, char *argv[]) {
  GuiWindow win;

  auto img = cv::imread("test.png");
  cv::cvtColor(img, img, cv::COLOR_BGR2RGB);

  win.Initialize();
  Image image;
  CalibrationScene scene;
  ConernerDetector detector;

  image.SetObjectDrawCallback([&] {
    auto dl = ImGui::GetWindowDrawList();
    const auto [x, y] = image.GetImageDrawCoordinate(100, 100);
    dl->AddCircleFilled({x, y}, 30 * image.GetScale(), 0xFF445533);
  });
  image.SetImage(img);

  scene.AddPoints({{-1.0f, -1.0f, 0.0f},
                   {1.0f, -1.0f, 0.0f},
                   {0.0f, 1.0f, 0.0f},
                   {0.0f, -1.0f, 0.5f}},
                  {1.0, 0.0, 0.0});

  // We want to have the first future already filled -> buttons are
  // enabled.
  std::future<void> computation = std::promise<void>().get_future();

  while (!win.Draw([&]() {
    ImGui::NewFrame();

    ImGui::Begin("win", NULL, ImGuiWindowFlags_HorizontalScrollbar);
    image.Display();
    ImGui::End();

    ImGui::Begin("controls");

    bool disabled = false;
    if (!IsFutureReady(computation)) {
      // Computation is on going, do not start another one
      ImGui::BeginDisabled();
      disabled = true;
    }

    if (ImGui::Button("Calculate corners")) {
      computation =
          std::async(std::launch::async, [&]() { detector.Detect(img); });
    }
    if (ImGui::Button("Create log float 2")) {
      cv::Mat img = cv::Mat::zeros(1000, 1000, CV_32F);
      for (int row = 0; row < img.rows; ++row) {
        for (int col = 0; col < img.cols; ++col) {
          img.at<float>(row, col) = static_cast<float>(rand() % 1000);
        }
      }
      LOG_IMAGE("img2", img);
    }

    if (disabled) {
      ImGui::EndDisabled();
    }

    ImGui::End();

    scene.Render();

    ImageLog::Get().Display();
    PlotLog::Get().Display();
  })) {
  }

  return 0;
}
