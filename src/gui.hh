#pragma once

#include <functional>
#include <opencv2/core.hpp>
#include <string>

class GuiWindowPrivate;
class TexturePrivate;
class ImagePrivate;

class Texture {
 public:
  Texture();
  ~Texture();

  void Display(const float scale = 1.0f);

  void SetTexture(const int image_width, const int image_height,
                  const unsigned char *image_data, const bool rgb = true);
  void SetTexture(const cv::Mat &in);
  int GetWidth() const { return width_; }
  int GetHeight() const { return height_; }

 private:
  void DestroyTexture();

 private:
  TexturePrivate *p_{nullptr};
  int width_{0};
  int height_{0};
  bool texture_created_{false};
};

class Image {
 public:
  Image();
  ~Image();
  void SetImage(const cv::Mat &in);
  void Display();
  float MousePosOnImageX();
  float MousePosOnImageY();

  /// Set callback which is called when ImGui objects can be drawn
  /// on top of the image.
  /// For example:
  /// \code
  /// img.SetObjectDrawCallback([&](){
  ///   float x = 10;
  ///   float y = 10;
  ///   auto dl = ImGui::GetWindowDrawList();
  ///   dl->AddCircleFilled({x, y}, 30, 0xFFFFFFFF);
  /// });
  /// \endcode
  void SetObjectDrawCallback(const std::function<void()> fun) {
    draw_fun_ = fun;
  }
  void ImageCoordinateToDrawCoordinate(float &x, float &y) const;
  std::tuple<float, float> GetImageDrawCoordinate(float x, float y) const;
  float GetScale() const;
  void SetMinMaxDisplayedValues(const float min, const float max);

 private:
  void CheckMouse();
  void DisplayImage();
  void DisplayInfoWidgets();
  void DisplayIntensityClampWidgets();
  void AdjustZoomToMouse();
  void DrawObjects();
  cv::Mat GetProcessed();

 private:
  ImagePrivate *p_{nullptr};
  Texture texture_;
  std::function<void()> draw_fun_;
  cv::Mat original_data_;
  float min_displayed_{0.0f};
  float max_displayed_{255.0f};
  float data_min_{0.0f};
  float data_max_{0.0f};
};

class GuiWindow {
 public:
  GuiWindow();
  ~GuiWindow();

  bool Initialize();
  bool Draw(std::function<void()> fun);
  bool StartDraw();
  void EndDraw();

 private:
  void Uninit();
  GuiWindow(const GuiWindow &) = delete;
  GuiWindow &operator=(const GuiWindow &) = delete;

 private:
  GuiWindowPrivate *p_{nullptr};
  std::string title_{"Calibrator"};
};
