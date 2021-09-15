#pragma once

#include <functional>
#include <string>

class GuiWindowPrivate;
class TexturePrivate;

class Texture {
 public:
  Texture();
  ~Texture();

  void Display();

  void SetTexture(const int image_width, const int image_height,
                  const unsigned char *image_data);

 private:
  void DestroyTexture();

 private:
  TexturePrivate *p_{nullptr};
  int width_{0};
  int height_{0};
  bool texture_created_{false};
};

class GuiWindow {
 public:
  GuiWindow(const std::string &title = "Config");
  ~GuiWindow();

  bool Initialize();
  bool Draw(std::function<void()> fun);
  bool StartDraw();
  void EndDraw();
  void SetAutoResize(const bool auto_resize) { auto_resize_ = auto_resize; };

 private:
  void Uninit();
  GuiWindow(const GuiWindow &) = delete;
  GuiWindow &operator=(const GuiWindow &) = delete;

 private:
  GuiWindowPrivate *p_{nullptr};
  std::string title_;
  bool auto_resize_{true};
};
