#include <GL/glew.h>
#include <opencv2/highgui.hpp>
#include "gui.hh"
#include "imgui.h"
#include "opencv2/imgproc.hpp"

void Gui() {}

int main(int argc, char *argv[]) {
  GuiWindow conf;

  auto img = cv::imread("test.png");
  cv::cvtColor(img, img, cv::COLOR_BGR2RGB);

  conf.Initialize();
  Image image;
  image.SetObjectDrawCallback([&] {
    auto dl = ImGui::GetWindowDrawList();
    const auto [x, y] = image.GetImageDrawCoordinate(100, 100);
    dl->AddCircleFilled({x, y}, 30 * image.GetScale(), 0xFF445533);
  });
  image.SetImage(img);
  while (!conf.Draw([&]() {
    ImGui::NewFrame();
    ImGui::Begin("win", NULL, ImGuiWindowFlags_HorizontalScrollbar);

    image.Display();

    ImGui::End();
  })) {
  }

  return 0;
}
