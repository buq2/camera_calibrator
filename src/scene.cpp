#include "scene.hh"
#include <GL/glew.h>
#include <Eigen/Geometry>
#include "imgui.h"

using namespace calibrator;

Matrix4 SceneCamera::GetView() { return view_matrix_; }

void SceneCamera::UpdateViewMatrix() {
  Eigen::AngleAxisf roll(0.0f, Eigen::Vector3f::UnitZ());
  Eigen::AngleAxisf yaw(yaw_, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf pitch(pitch_, Eigen::Vector3f::UnitX());
  Eigen::Quaternionf q = roll * yaw * pitch;
  r_ = q.matrix();

  const auto pos = focus_ - r_ * forward_ * distance_;
  view_matrix_.block<3, 3>(0, 0) = r_;
  view_matrix_.block<3, 1>(0, 3) = pos;
}

void SceneCamera::MouseRotate(float dx, float dy) {
  const auto s = (r_ * up_).y() >= 0.0f ? 1.0f : -1.0f;
  yaw_ = s * dx * rotation_speed_;
  pitch_ = dy * rotation_speed_;
  UpdateViewMatrix();
}

void SceneCamera::MousePan(float dx, float dy) {
  focus_ -= r_ * right_ * dx * distance_;
  focus_ += r_ * up_ * dy * distance_;
  UpdateViewMatrix();
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
  fb_.Bind();

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glFrustum(0.0f, viewport_size.x, viewport_size.y, 0.0f, 0.01f, 10.0f);

  glMatrixMode(GL_MODELVIEW);
  // glLoadIdentity();
  glLoadMatrixf(cam_.GetView().data());
  // gluPerspective(120, 1.0, 0.01, 2.0);

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
