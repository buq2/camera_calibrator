#pragma once

#include <GL/glew.h>
#include "types.hh"

namespace calibrator {

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
  Matrix4 GetView();
  void MouseRotate(float dx, float dy);
  void MousePan(float dx, float dy);
  void SetMousePos(const Point2D& pos);
  void SetAspect(const float aspect);
 private:
  
  void UpdateViewMatrix();
  Matrix4 GetProjection(float fovy, float aspect, float near, float far);

 private:
  float pitch_{0.0f};
  float yaw_{0.0f};
  float distance_{1.0};
  float rotation_speed_{0.005f};
  Vector3 focus_{0.0f, 0.0f, 0.0f};
  
  float fov_{120.0f};
  float near_{0.01f};
  float far_{20.0f};

  Matrix3 r_{Matrix3::Identity()};
  Matrix4 view_matrix_{Matrix4::Identity()};
  Matrix4 projection_;

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

 private:
  SceneCamera cam_;
  GLuint vertexbuffer;
  FrameBuffer fb_;
};  // class Scene

}  // namespace calibrator