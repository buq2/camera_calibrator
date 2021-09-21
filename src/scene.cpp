#include "scene.hh"
#include <GL/glew.h>
#include <Eigen/Geometry>
#include "imgui.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <optional>
#include <filesystem>

using namespace calibrator;

std::string GetFileContent(const std::string fname) {
  std::ifstream input(fname);
  std::stringstream ss;
  ss << input.rdbuf();
  return ss.str();
}

std::optional<uint32_t> CompileShader(uint32_t shader_type, const std::string source) {
  unsigned int id = glCreateShader(shader_type);

  auto c_str_source = source.c_str();
  glShaderSource(id, 1, &(c_str_source), nullptr);
  glCompileShader(id);

  GLint result;
  glGetShaderiv(id, GL_COMPILE_STATUS, &result);

  if (result == GL_FALSE) {
    int error_len;
    glGetShaderiv(id, GL_INFO_LOG_LENGTH, &error_len);

    std::vector<GLchar> error_str(error_len + 1);
    glGetShaderInfoLog(id, error_len, &error_len, error_str.data());

    std::cout << "Failed to compile shader: " << error_str.data() << "\n";
    glDeleteShader(id);
    return std::nullopt;
  }

  return id;
}

void Shader::Use() {
  if (loaded_) { 
    std::cerr << "Tried to use shader program which is not properly compiled\n";
    return;
  }
  glUseProgram(id_);
}

void Shader::Delete() {
  if (!loaded_) return;
  glDeleteProgram(id_);
  loaded_ = false;
}

std::string LoadOrUseAsSource(const std::string& fname_or_source) {
  try {
    if (std::filesystem::exists(fname_or_source)) {
      return GetFileContent(fname_or_source);
    }
  } catch(std::filesystem::filesystem_error& /*e*/) {
  }
  return fname_or_source;
}

void Shader::PrintProgramInfoLog(const std::string& msg) {
  int error_len = 0;
  glGetProgramiv(id_, GL_INFO_LOG_LENGTH, &error_len);
  if (error_len == 0) return;
  std::vector<GLchar> error_str(error_len + 1);
  glGetProgramInfoLog(id_, error_len, NULL, error_str.data());
  std::cout << msg << " " << error_str.data() << "\n";
}

bool Shader::Load(const std::string& vertex_shader, const std::string& fragment_shader) {
  Delete();
  id_ = glCreateProgram();

  const auto vs = CompileShader(GL_VERTEX_SHADER, LoadOrUseAsSource(vertex_shader));
  if (!vs) {
    return false;
  }

  const auto fs = CompileShader(GL_FRAGMENT_SHADER, LoadOrUseAsSource(fragment_shader));
  if (!fs) {
    return false;
  }

  glAttachShader(id_, *vs);
  glAttachShader(id_, *fs);

  glLinkProgram(id_);

  int success = 0;
  glGetProgramiv(id_, GL_LINK_STATUS, &success);
  if(!success) {
    PrintProgramInfoLog("Failed to link program:");
    glDeleteProgram(id_);
    id_ = 0;
  }

  glValidateProgram(id_);

  glGetProgramiv(id_, GL_VALIDATE_STATUS, &success);
  if(!success) {
    PrintProgramInfoLog("Failed to validate program:");
    glDeleteProgram(id_);
    id_ = 0;
  }

  glDeleteShader(*vs);
  glDeleteShader(*fs);

  loaded_ = true;
  return true;
}

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
  shader_.Load(R""""(
#version 330 core
layout (location = 0) in vec3 aPos;   // the position variable has attribute position 0
layout (location = 1) in vec3 aColor; // the color variable has attribute position 1
  
out vec3 ourColor; // output a color to the fragment shader

void main()
{
    gl_Position = vec4(aPos, 1.0);
    ourColor = aColor; // set ourColor to the input color we got from the vertex data
}  
)""""
,
R""""(
#version 330 core
out vec4 FragColor;  
in vec3 ourColor;
  
void main()
{
    FragColor = vec4(ourColor, 1.0);
}
)""""
  );

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

  ImGui::Image((void*)(intptr_t)fb_.GetTexture(),
               {viewport_size.x, viewport_size.y}, ImVec2{0, 1}, ImVec2{1, 0});
  ImGui::End();
}
