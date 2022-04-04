#pragma once

#include <GL/glew.h>
#include <functional>
#include "types.hh"
namespace calibrator {

void InitGlDebugMessages();
class VertexBuffer {
 public:
  ~VertexBuffer();
  VertexBuffer() = default;
  VertexBuffer(const VertexBuffer& other) = delete;
  VertexBuffer& operator=(const VertexBuffer& other) = delete;
  VertexBuffer(VertexBuffer&& other) noexcept;
  VertexBuffer& operator=(VertexBuffer&& other) noexcept;
  void Delete();
  void Create(const Points3D& points);
  void Create(const float* data, size_t num_floats);
  void Bind(const uint32_t location = 0);
  void Unbind();
  uint32_t NumberOfElements() const { return num_elements_; };

 private:
  bool loaded_{false};
  uint32_t id_{0};
  uint32_t num_elements_{0};
  uint32_t component_size_{3};
  bool normalized_{false};
  uint32_t stride_{0};
  uint32_t location_{0};
  bool bound_{false};
};  // class VertexBuffer

class VertexArray {
 public:
  VertexArray();
  ~VertexArray();
  VertexArray(const VertexArray& other) = delete;
  VertexArray& operator=(const VertexArray& other) = delete;
  VertexArray(VertexArray&& other) noexcept;
  VertexArray& operator=(VertexArray&& other) noexcept;
  void Bind();
  void Add(const uint32_t location, VertexBuffer&& buffer);
  uint32_t NumberOfElements() const;

 private:
  std::vector<VertexBuffer> vbos_;
  uint32_t id_{0};
  bool initialized_{false};
};

class Shader {
 public:
  Shader() {}
  ~Shader() { Delete(); }
  bool Load(const std::string& vertex_shader,
            const std::string& fragment_shader);
  void Use();
  void Delete();
  uint32_t GetId() const { return id_; }

  void Set(const std::string& name, bool val);
  void Set(const std::string& name, int val);
  void Set(const std::string& name, int i1, int i2, int i3);
  void Set(const std::string& name, float val);
  void Set(const std::string& name, float f1, float f2, float f3);
  void Set(const std::string& name, Vector3 val);
  void Set(const std::string& name, Vector4 val);
  void Set(const std::string& name, Matrix3 val);
  void Set(const std::string& name, Matrix4 val);

 private:
  void PrintProgramInfoLog(const std::string& msg);

 private:
  uint32_t id_{0};
  bool loaded_{false};
};  // class Shader
class FrameBuffer {
 public:
  FrameBuffer() {}

  void Init(int w, int h) {
    if (w == width_ && h == height_) return;
    DeleteBuffers();
    width_ = w;
    height_ = h;

    glGenFramebuffers(1, &frame_buffer_);
    glBindFramebuffer(GL_FRAMEBUFFER, frame_buffer_);
    glCreateTextures(GL_TEXTURE_2D, 1, &texture_);
    glBindTexture(GL_TEXTURE_2D, texture_);

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width_, height_, 0, GL_RGBA,
                 GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                           texture_, 0);

    glCreateTextures(GL_TEXTURE_2D, 1, &depth_);
    glBindTexture(GL_TEXTURE_2D, depth_);
    glTexStorage2D(GL_TEXTURE_2D, 1, GL_DEPTH24_STENCIL8, width_, height_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT,
                           GL_TEXTURE_2D, depth_, 0);

    GLenum buffers[4] = {GL_COLOR_ATTACHMENT0};
    glDrawBuffers(texture_, buffers);

    Unbind();
  }
  ~FrameBuffer() { DeleteBuffers(); }

  void Bind() {
    glBindFramebuffer(GL_FRAMEBUFFER, frame_buffer_);
    glViewport(0, 0, width_, height_);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  }

  void Unbind() const { glBindFramebuffer(GL_FRAMEBUFFER, 0); }

  void DeleteBuffers() {
    if (!frame_buffer_) return;
    Unbind();
    glDeleteFramebuffers(1, &frame_buffer_);
    glDeleteTextures(1, &texture_);
    glDeleteTextures(1, &depth_);
    frame_buffer_ = 0;
    texture_ = 0;
    depth_ = 0;
  }

  uint32_t GetTexture() const { return texture_; }

 private:
  uint32_t frame_buffer_{0};
  uint32_t texture_{0};
  uint32_t depth_{0};
  int width_{0};
  int height_{0};
};  // class FrameBuffer

class SceneCamera {
 public:
  SceneCamera();
  Matrix4 GetView() const;
  Vector3 GetCameraLocation() const;
  void MouseRotate(float dx, float dy);
  void MousePan(float dx, float dy);
  void SetMousePos(const Point2D& pos, bool rotate = true,
                   bool just_started = false);
  void MouseForward(float d);
  void SetAspect(const float aspect);

 private:
  void UpdateViewMatrix();
  static Matrix4 GetProjection(float fovy, float aspect, float near, float far);

 private:
  float pitch_{0.0f};
  float yaw_{0.0f};
  float distance_{5.0};
  float rotation_speed_{0.005f};
  float pan_speed_{0.005f};
  float forward_speed_{1.05f};
  Vector3 focus_{0.0f, 0.0f, 0.0f};

  float fov_{45.0f};
  float near_{0.01f};
  float far_{20.0f};

  Matrix3 r_{Matrix3::Identity()};
  Matrix4 view_matrix_{Matrix4::Identity()};
  Matrix4 projection_{Matrix4::Identity()};

  const Vector3 up_{Eigen::Vector3f::UnitY()};
  const Vector3 right_{Eigen::Vector3f::UnitX()};
  const Vector3 forward_{-Eigen::Vector3f::UnitZ()};

  Point2D prev_mouse_pos_;
};  // class SceneCamera

class Scene {
 public:
  Scene();
  SceneCamera& GetCamera() { return cam_; }
  void Render();
  void SetDrawFunction(const std::function<void()>& draw_objects) {
    draw_objects_ = draw_objects;
  };

 private:
  void CheckMouse();

 private:
  SceneCamera cam_;
  FrameBuffer fb_;

  bool mouse_clicked_inside_{false};
  std::function<void()> draw_objects_;
};  // class Scene

class CalibrationScene {
 public:
  CalibrationScene();
  void Render();
  void AddPoints(const Points3D& points,
                 const Vector3& color = {1.0, 1.0, 1.0});

 private:
  void Draw();
  void DrawPoints();

 private:
  Scene scene_;
  VertexArray vao_;
  Shader point_shader_;

  struct PointData {
    VertexArray vao;
    Point3D color;
  };
  std::vector<PointData> points_;
};

}  // namespace calibrator
