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
                  const unsigned char *image_data);
  void SetTexture(const cv::Mat &in);
  int GetWidth() const {return width_;}
  int GetHeight() const {return height_;}
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
  void SetImage(const int image_width, const int image_height,
                const unsigned char *image_data);
  void SetImage(const cv::Mat &in);
  void Display();
  float MousePosOnImageX();
  float MousePosOnImageY();
 private:
  void CheckMouse();

 private:
  ImagePrivate *p_{nullptr};
  Texture texture_;
  float scale_{1.0f};
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
