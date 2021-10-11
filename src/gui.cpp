#include "gui.hh"

#include <SDL.h>
#include <algorithm>
#include <iostream>
#include "imgui.h"
#include "imgui_impl_opengl3.h"
#include "imgui_impl_sdl.h"
#define IMGUI_IMPL_OPENGL_LOADER_GLEW

// About Desktop OpenGL function loaders:
//  Modern desktop OpenGL doesn't have a standard portable header file to load
//  OpenGL function pointers. Helper libraries are often used for this purpose!
//  Here we are supporting a few common ones (gl3w, glew, glad). You may use
//  another loader/header of your choice (glext, glLoadGen, etc.), or chose to
//  manually implement your own.
#if defined(IMGUI_IMPL_OPENGL_LOADER_GL3W)
#include <GL/gl3w.h>  // Initialize with gl3wInit()
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLEW)
#include <GL/glew.h>  // Initialize with glewInit()
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLAD)
#include <glad/glad.h>  // Initialize with gladLoadGL()
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLBINDING)
#include <glbinding/gl/gl.h>
#include <glbinding/glbinding.h>  // Initialize with glbinding::initialize()
using namespace gl;
#else
#include IMGUI_IMPL_OPENGL_LOADER_CUSTOM
#endif

class TexturePrivate {
 public:
  GLuint image_texture_;
};

class ImagePrivate {
 public:
  ImVec2 mouse_pos_on_image_{0.0f, 0.0f};

  ImVec2 cursor_screen_pos_before_display{0.0f, 0.0f};

  // Where mouse was clicked?
  ImVec2 mouse_pos_on_image_prev_click_{0.0f, 0.0f};  // On image
  ImVec2 mouse_pos_prev_click_{0.0f, 0.0f};           // In gui coordinates
  bool mouse_dragging{false};
  bool mouse_clicked_inside_{false};
  bool mouse_inside_{false};
  ImVec2 scroll_when_drag_started{0.0f, 0.0f};
  float scale{1.0f};
  float widget_space_{0.0f};

  ImVec2 GetMousePosInImageCoordinates() {
    const auto& io = ImGui::GetIO();
    const auto& mpos = io.MousePos;
    return {(mpos.x - cursor_screen_pos_before_display.x) / scale,
            (mpos.y - cursor_screen_pos_before_display.y) / scale};
  }

  ImVec2 GetImagePixelPosInScreenCoordinates(ImVec2 pos) {
    // First remove scale
    pos.x *= scale;
    pos.y *= scale;

    // Remove cursor pos
    pos.x += cursor_screen_pos_before_display.x;
    pos.y += cursor_screen_pos_before_display.y;

    return pos;
  }
};

Texture::Texture() : p_(new TexturePrivate) {}

Texture::~Texture() {
  DestroyTexture();
  delete p_;
  p_ = nullptr;
}

void Texture::Display(const float scale) {
  if (texture_created_) {
    ImGui::Image((void*)(intptr_t)p_->image_texture_,
                 ImVec2(static_cast<float>(width_ * scale),
                        static_cast<float>(height_ * scale)));
  }
}

void Texture::DestroyTexture() {
  if (texture_created_) {
    glDeleteTextures(1, &p_->image_texture_);
    texture_created_ = false;
  }
}

void Texture::SetTexture(const cv::Mat& in) {
  const auto depth = in.depth();
  const auto chans = in.channels();
  assert(depth == CV_8U);
  assert(chans == 3 || chans == 1);

  SetTexture(in.cols, in.rows, in.data, chans == 3);
}

void Texture::SetTexture(const int image_width, const int image_height,
                         const unsigned char* image_data, const bool rgb) {
  if (!texture_created_) {
    glGenTextures(1, &p_->image_texture_);
  }
  texture_created_ = true;
  width_ = image_width;
  height_ = image_height;
  glBindTexture(GL_TEXTURE_2D, p_->image_texture_);

  // Setup filtering parameters for display
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S,
                  GL_CLAMP_TO_EDGE);  // This is required on WebGL for non
                                      // power-of-two textures
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T,
                  GL_CLAMP_TO_EDGE);  // Same

  // Upload pixels into texture
#if defined(GL_UNPACK_ROW_LENGTH) && !defined(__EMSCRIPTEN__)
  glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
#endif

  // Set row alignment requirement to "none". Originally this value is
  // set to 4, which then requires 4 byte aligned rows. We are bit lazy
  // and will not do it right now, as we would have to monitor
  // what kind of alignment the input has.
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

  const auto type = rgb ? GL_RGB : GL_LUMINANCE;
  glTexImage2D(GL_TEXTURE_2D, 0, type, image_width, image_height, 0, type,
               GL_UNSIGNED_BYTE, image_data);
}

Image::Image() : p_(new ImagePrivate) {}

Image::~Image() { delete p_; }

void Image::SetMinMaxDisplayedValues(const float min, const float max) {
  if (min == min_displayed_ && max == max_displayed_) return;
  min_displayed_ = min;
  max_displayed_ = max;
  texture_.SetTexture(GetProcessed());
}

cv::Mat ApplyScalingAndConvertTo8U(const cv::Mat& img, float min_val,
                                   float max_val) {
  cv::Mat out = (img - min_val) / (max_val - min_val) * 255.0f;
  out.convertTo(out, CV_8U);
  return out;
}

cv::Mat Image::GetProcessed() {
  if (original_data_.type() == CV_8U && min_displayed_ == 0.0f &&
      max_displayed_ == 255.0f) {
    return original_data_;
  }

  return ApplyScalingAndConvertTo8U(original_data_, min_displayed_,
                                    max_displayed_);
}

void Image::SetImage(const cv::Mat& in) {
  original_data_ = in.clone();
  if (in.type() != CV_8U) {
    double min, max;
    cv::minMaxLoc(original_data_, &min, &max);
    data_min_ = static_cast<float>(min);
    data_max_ = static_cast<float>(max);
  } else {
    data_min_ = 0;
    data_max_ = 255;
  }

  texture_.SetTexture(GetProcessed());
}

void Image::ImageCoordinateToDrawCoordinate(float& x, float& y) const {
  const auto pos = p_->GetImagePixelPosInScreenCoordinates({x, y});
  x = pos.x;
  y = pos.y;
}

std::tuple<float, float> Image::GetImageDrawCoordinate(float x, float y) const {
  ImageCoordinateToDrawCoordinate(x, y);
  return {x, y};
}

float Image::GetScale() const { return p_->scale; }

void Image::DisplayIntensityClampWidgets() {
  float min = min_displayed_;
  float max = max_displayed_;

  ImGui::PushItemWidth(ImGui::GetWindowWidth() / 3.0f);
  ImGui::SliderFloat("Min", &min, data_min_, data_max_);
  ImGui::SameLine();
  ImGui::SliderFloat("Max", &max, data_min_, data_max_);
  ImGui::PopItemWidth();

  SetMinMaxDisplayedValues(min, max);
}

void Image::DisplayImage() {
  // Screen pos needs to be taken before displaying the image
  p_->cursor_screen_pos_before_display = ImGui::GetCursorScreenPos();

  const auto available_space = ImGui::GetWindowHeight()-ImGui::GetCursorPos().y;
  const auto used_by_widgets = p_->widget_space_ + 5;
  const auto usable_area_image = available_space - used_by_widgets;
  const auto size = ImVec2(0, usable_area_image);
  ImGui::BeginChild("image", size, false, ImGuiWindowFlags_HorizontalScrollbar);
  texture_.Display(p_->scale);
  if (draw_fun_) draw_fun_();
  // CheckMouse must be called after texture_.Display and before EndChild
  CheckMouse(); 
  ImGui::EndChild();
}

void Image::Display() { 
  DisplayImage();

  // Calculate space used by other widgets which is then used as margin during next call
  const auto y_before = ImGui::GetCursorScreenPos().y;
  DisplayInfoWidgets();
  DisplayIntensityClampWidgets();
  const auto y_after = ImGui::GetCursorScreenPos().y;
  p_->widget_space_ = y_after-y_before;
}

float Image::MousePosOnImageX() { return p_->mouse_pos_on_image_.x; }
float Image::MousePosOnImageY() { return p_->mouse_pos_on_image_.y; }

float GetGrayVal(const cv::Mat& img, float x, float y) {
  y = std::max<float>(std::min<float>(y, static_cast<float>(img.rows - 1)),
                      0.0f);
  x = std::max<float>(std::min<float>(x, static_cast<float>(img.cols - 1)),
                      0.0f);
  if (img.depth() == CV_32F) {
    return img.at<float>((int)y, (int)x);
  } else if (img.depth() == CV_64F) {
    return static_cast<float>(img.at<double>((int)y, (int)x));
  } else if (img.depth() == CV_8U) {
    return static_cast<float>(img.at<uint8_t>((int)y, (int)x));
  } else if (img.depth() == CV_8S) {
    return static_cast<float>(img.at<int8_t>((int)y, (int)x));
  } else if (img.depth() == CV_16U) {
    return static_cast<float>(img.at<uint16_t>((int)y, (int)x));
  } else if (img.depth() == CV_16S) {
    return static_cast<float>(img.at<int16_t>((int)y, (int)x));
  }
  assert(false);
  return 0.0f;
}

template <typename T>
std::vector<float> GetRGBValT(const cv::Mat& img, float x, float y) {
  y = std::max<float>(std::min<float>(y, static_cast<float>(img.rows - 1)),
                      0.0f);
  x = std::max<float>(std::min<float>(x, static_cast<float>(img.cols - 1)),
                      0.0f);
  auto v = img.ptr<T>((int)y, (int)x);
  return {static_cast<float>(v[0]), static_cast<float>(v[1]),
          static_cast<float>(v[2])};
}

std::vector<float> GetRGBVal(const cv::Mat& img, float x, float y) {
  if (img.depth() == CV_32F) {
    return GetRGBValT<float>(img, x, y);
  } else if (img.depth() == CV_64F) {
    return GetRGBValT<double>(img, x, y);
  } else if (img.depth() == CV_8U) {
    return GetRGBValT<uint8_t>(img, x, y);
  } else if (img.depth() == CV_8S) {
    return GetRGBValT<int8_t>(img, x, y);
  } else if (img.depth() == CV_16U) {
    return GetRGBValT<uint16_t>(img, x, y);
  } else if (img.depth() == CV_16S) {
    return GetRGBValT<int16_t>(img, x, y);
  }
  assert(false);
  return {};
}

std::vector<float> GetImageVal(const cv::Mat& img, const float x,
                               const float y) {
  if (img.channels() == 1) {
    const auto val = GetGrayVal(img, x, y);
    return {val};
  } else if (img.channels() == 3) {
    return GetRGBVal(img, x, y);
  }
  assert(false);
  return {};
}

void Image::AdjustZoomToMouse() {
  // Get new position of the mouse. It should be the same as before
  auto new_mpos = p_->GetMousePosInImageCoordinates();
  // Calculate error (image coordinates)
  const auto dx = new_mpos.x - p_->mouse_pos_on_image_.x;
  const auto dy = new_mpos.y - p_->mouse_pos_on_image_.y;
  // (screen coordinates)
  const auto dxs = dx * p_->scale;
  const auto dys = dy * p_->scale;
  // Adjust scroll
  ImGui::SetScrollX(ImGui::GetScrollX() - dxs);
  ImGui::SetScrollY(ImGui::GetScrollY() - dys);
}

void Image::CheckMouse() {
  p_->mouse_pos_on_image_ = p_->GetMousePosInImageCoordinates();

  auto& io = ImGui::GetIO();
  // Last created item was the image, easy to check if mouse was inside of it
  p_->mouse_inside_ = ImGui::IsItemHovered();

  if (ImGui::IsMouseClicked(0)) {
    p_->mouse_pos_on_image_prev_click_ = {MousePosOnImageX(),
                                          MousePosOnImageY()};
    p_->mouse_pos_prev_click_ = ImGui::GetMousePos();
    p_->mouse_clicked_inside_ = p_->mouse_inside_;
  }

  if (!p_->mouse_inside_) {
    return;
  }

  // Drag related
  if (p_->mouse_clicked_inside_ && ImGui::IsMouseDown(0)) {
    // Mouse button is down, check if it is being dragged:
    const auto dx = p_->mouse_pos_prev_click_.x - ImGui::GetMousePos().x;
    const auto dy = p_->mouse_pos_prev_click_.y - ImGui::GetMousePos().y;
    const auto delta = sqrt(dx * dx + dy * dy);
    if (delta > io.MouseDragThreshold && !p_->mouse_dragging) {
      p_->mouse_dragging = true;
      p_->scroll_when_drag_started = {ImGui::GetScrollX(), ImGui::GetScrollY()};
    }
  } else {
    p_->mouse_dragging = false;
  }

  if (p_->mouse_dragging) {
    // Set scroll bar based on drag
    const auto dx = p_->mouse_pos_prev_click_.x - ImGui::GetMousePos().x;
    const auto dy = p_->mouse_pos_prev_click_.y - ImGui::GetMousePos().y;

    ImGui::SetScrollX(p_->scroll_when_drag_started.x + dx);
    ImGui::SetScrollY(p_->scroll_when_drag_started.y + dy);
  }

  // Zoom related
  {
    auto wheel_delta = io.MouseWheel;
    if (io.MouseDoubleClicked[0]) {
      p_->scale = 1.0f;
      AdjustZoomToMouse();
    } else if (io.KeyCtrl && wheel_delta != 0.0f) {
      if (wheel_delta > 0) {
        p_->scale *= wheel_delta * 1.1f;
      } else {
        p_->scale /= std::abs(wheel_delta * 1.1f);
      }
      AdjustZoomToMouse();
    }
  }
}

void Image::DisplayInfoWidgets() {
  if (p_->mouse_inside_) {
    const auto vals = GetImageVal(original_data_, p_->mouse_pos_on_image_.x,
                                  p_->mouse_pos_on_image_.y);
    if (vals.size() == 3) {
      ImGui::Text("Val: %f, %f, %f", vals[0], vals[1], vals[2]);
    } else {
      ImGui::Text("Val: %f", vals[0]);
    }
    ImGui::Text("Pos: %f, %f", p_->mouse_pos_on_image_.x, p_->mouse_pos_on_image_.y);
  } else {
    // Reserve space
    const auto h = ImGui::GetFontSize() + 2; // +2 from padding?
    ImGui::Dummy(ImVec2(0, h*2));
  }
}

class GuiWindowPrivate {
 public:
  SDL_Window* window;
  SDL_GLContext gl_context;
};

GuiWindow::GuiWindow() : p_(new GuiWindowPrivate) {}

GuiWindow::~GuiWindow() {
  delete p_;
  p_ = nullptr;
}

bool GuiWindow::Initialize() {
  // Setup SDL
  // (Some versions of SDL before <2.0.10 appears to have performance/stalling
  // issues on a minority of Windows systems, depending on whether
  // SDL_INIT_GAMECONTROLLER is enabled or disabled.. updating to latest version
  // of SDL is recommended!)
  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_GAMECONTROLLER) !=
      0) {
    std::cerr << "Error: " << SDL_GetError() << std::endl;
    return false;
  }

  // Decide GL+GLSL versions
#if __APPLE__
  // GL 3.2 Core + GLSL 150
  const char* glsl_version = "#version 150";
  SDL_GL_SetAttribute(
      SDL_GL_CONTEXT_FLAGS,
      SDL_GL_CONTEXT_FORWARD_COMPATIBLE_FLAG);  // Always required on Mac
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
#else
  // GL 3.0 + GLSL 130
  const char* glsl_version = "#version 130";
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
#endif

  // Create window with graphics context
  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
  SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
  SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
  SDL_WindowFlags window_flags = (SDL_WindowFlags)(
      SDL_WINDOW_OPENGL | SDL_WINDOW_ALLOW_HIGHDPI | SDL_WINDOW_RESIZABLE);
  p_->window =
      SDL_CreateWindow(title_.c_str(), SDL_WINDOWPOS_CENTERED,
                       SDL_WINDOWPOS_CENTERED, 1280, 720, window_flags);
  p_->gl_context = SDL_GL_CreateContext(p_->window);
  SDL_GL_MakeCurrent(p_->window, p_->gl_context);
  SDL_GL_SetSwapInterval(1);  // Enable vsync

  // Initialize OpenGL loader
#if defined(IMGUI_IMPL_OPENGL_LOADER_GL3W)
  bool err = gl3wInit() != 0;
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLEW)
  bool err = glewInit() != GLEW_OK;
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLAD)
  bool err = gladLoadGL() == 0;
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLBINDING)
  bool err = false;
  glbinding::initialize([](const char* name) {
    return (glbinding::ProcAddress)SDL_GL_GetProcAddress(name);
  });
#else
  bool err = false;  // If you use IMGUI_IMPL_OPENGL_LOADER_CUSTOM, your loader
                     // is likely to requires some form of initialization.
#endif
  if (err) {
    std::cerr << "Failed to initialize OpenGL loader!\n";
    return false;
  }

  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();

  // Allow movement of windows only from title bar. This way we can use
  // mouse drag etc for other purposes
  io.ConfigWindowsMoveFromTitleBarOnly = true;

  // Setup Dear ImGui style
  ImGui::StyleColorsDark();
  // ImGui::StyleColorsClassic();

  // Setup Platform/Renderer bindings
  ImGui_ImplSDL2_InitForOpenGL(p_->window, p_->gl_context);
  ImGui_ImplOpenGL3_Init(glsl_version);

  return true;
}

void GuiWindow::Uninit() {
  // Cleanup
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplSDL2_Shutdown();
  ImGui::DestroyContext();

  SDL_GL_DeleteContext(p_->gl_context);
  SDL_DestroyWindow(p_->window);
  SDL_Quit();
}

bool GuiWindow::Draw(std::function<void()> fun) {
  auto done = StartDraw();
  fun();
  EndDraw();
  return done;
}

bool GuiWindow::StartDraw() {
  bool done = false;
  // Poll and handle events (inputs, window resize, etc.)
  // You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell
  // if dear imgui wants to use your inputs.
  // - When io.WantCaptureMouse is true, do not dispatch mouse input data to
  // your main application.
  // - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data
  // to your main application. Generally you may always pass all inputs to dear
  // imgui, and hide them from your application based on those two flags.
  SDL_Event event;
  while (SDL_PollEvent(&event)) {
    ImGui_ImplSDL2_ProcessEvent(&event);
    if (event.type == SDL_QUIT) done = true;
    if (event.type == SDL_WINDOWEVENT &&
        event.window.event == SDL_WINDOWEVENT_CLOSE &&
        event.window.windowID == SDL_GetWindowID(p_->window))
      done = true;
  }

  // Start the Dear ImGui frame
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplSDL2_NewFrame(p_->window);
  return done;
}

void GuiWindow::EndDraw() {
  // Rendering
  ImGui::Render();
  ImGuiIO& io = ImGui::GetIO();
  glViewport(0, 0, (int)io.DisplaySize.x, (int)io.DisplaySize.y);
  // glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
  glClear(GL_COLOR_BUFFER_BIT);
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
  SDL_GL_SwapWindow(p_->window);
}
