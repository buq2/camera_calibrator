#include <GL/glew.h>
#include <opencv2/highgui.hpp>
#include "corners.hh"
#include "gui.hh"
#include "imgui.h"
#include "log_image.hh"
#include "opencv2/imgproc.hpp"
#include "scene.hh"

using namespace calibrator;

void Gui() {}

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

  while (!win.Draw([&]() {
    ImGui::NewFrame();

    ImGui::Begin("win", NULL, ImGuiWindowFlags_HorizontalScrollbar);
    image.Display();
    ImGui::End();

    ImGui::Begin("controls");
    if (ImGui::Button("Calculate corners")) {
      detector.Detect(img);
    }
    if (ImGui::Button("Create log gray 1")) {
      cv::Mat img = cv::Mat::zeros(100, 100, CV_8U);
      cv::circle(img, {50, 50}, 20, {255, 255, 255}, 2);
      LOG_IMAGE("img1", img);
    }
    if (ImGui::Button("Create log float 1")) {
      cv::Mat img = cv::Mat::zeros(100, 100, CV_32F);
      cv::circle(img, {50, 50}, 20, {255, 255, 255}, 2);
      LOG_IMAGE("img2", img);
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
    ImGui::End();

    scene.Render();

    ImageLog::Get().Display();
  })) {
  }

  return 0;
}
