#include "scene.hh"
#include <GL/glew.h>
#include <Eigen/Geometry>
#include "imgui.h"
#include <iostream>

using namespace calibrator;

SceneCamera::SceneCamera()
{
  SetAspect(1.0f);
}
void SceneCamera::SetAspect(const float aspect)
{
  projection_ = GetProjection(fov_, aspect, near_, far_);
}

Matrix4 SceneCamera::GetView() { return projection_*view_matrix_; }

void SceneCamera::UpdateViewMatrix() {
  Eigen::AngleAxisf roll(0.0f, Eigen::Vector3f::UnitZ());
  Eigen::AngleAxisf yaw(yaw_, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf pitch(pitch_, Eigen::Vector3f::UnitX());
  Eigen::Quaternionf q = roll * yaw * pitch;
  r_ = q.matrix();

  const auto pos = focus_ - r_ * forward_ * distance_;
  
  view_matrix_ = Matrix4::Identity();
  view_matrix_.block<3, 3>(0, 0) = r_;
  view_matrix_.block<3, 1>(0, 3) = pos.transpose() * r_;
  view_matrix_ = (view_matrix_.inverse()).eval();
}

void SceneCamera::SetMousePos(const Point2D& pos)
{
  const auto dx = prev_mouse_pos_.x()-pos.x();
  const auto dy = prev_mouse_pos_.y()-pos.y();
  MouseRotate(dx, dy);
  prev_mouse_pos_ = pos;
}

void SceneCamera::MouseRotate(float dx, float dy) {
  const auto s = (r_ * up_).y() >= 0.0f ? 1.0f : -1.0f;
  yaw_ += s * dx * rotation_speed_;
  pitch_ += dy * rotation_speed_;
  UpdateViewMatrix();
}

void SceneCamera::MousePan(float dx, float dy) {
  focus_ -= r_ * right_ * dx * distance_;
  focus_ += r_ * up_ * dy * distance_;
  UpdateViewMatrix();
}

Matrix4 SceneCamera::GetProjection(float fovy, float aspect, float near, float far)
{
  Matrix4 out{Matrix4::Identity()};

  auto tanHalfFovy = tan((fovy/180.0f*3.14159265358979323846f) / 2.0f);
  out(0,0) = 1.0f/ (aspect * tanHalfFovy);
  out(1,1) = 1.0f / (tanHalfFovy);
  out(2,2) = - (far + near) / (far - near);
  out(2,3) = - 1.0f;
  out(3,2) = - (2.0f * far * near) / (far - near);
  return out;
}

Scene::Scene() {
  static const GLfloat g_vertex_buffer_data[] = {
      -1.0f, -1.0f, 0.0f, 1.0f, -1.0f, 0.0f, 0.0f, 1.0f, 0.0f,
  };

  // Generate 1 buffer, put the resulting identifier in vertexbuffer
  glGenBuffers(1, &vertexbuffer);
  // The following commands will talk about our 'vertexbuffer' buffer
  glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
  // Give our vertices to OpenGL.
  glBufferData(GL_ARRAY_BUFFER, sizeof(g_vertex_buffer_data),
               g_vertex_buffer_data, GL_STATIC_DRAW);
}

void Scene::Render() {
  ImGui::Begin("Scene");
  ImVec2 viewport_size = ImGui::GetContentRegionAvail();
  fb_.Init((int)viewport_size.x, (int)viewport_size.y);
  cam_.SetAspect(viewport_size.x/viewport_size.y);
  const auto mpos = ImGui::GetMousePos();
  cam_.SetMousePos({mpos.x, mpos.y});

  fb_.Bind();

  glLoadMatrixf(cam_.GetView().data());

  glEnableVertexAttribArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
  glVertexAttribPointer(0,  // attribute 0. No particular reason for 0, but must
                            // match the layout in the shader.
                        3,  // size
                        GL_FLOAT,  // type
                        GL_FALSE,  // normalized?
                        0,         // stride
                        (void*)0   // array buffer offset
  );
  // Draw the triangle !
  glDrawArrays(GL_TRIANGLES, 0,
               3);  // Starting from vertex 0; 3 vertices total -> 1 triangle
  glDisableVertexAttribArray(0);

  fb_.Unbind();

  ImGui::Image(reinterpret_cast<void*>(fb_.GetTexture()),
               {viewport_size.x, viewport_size.y}, ImVec2{0, 1}, ImVec2{1, 0});
  ImGui::End();
}
