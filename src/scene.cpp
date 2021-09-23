#include "scene.hh"
#include <GL/glew.h>
#include <Eigen/Geometry>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <utility>
#include "imgui.h"

using namespace calibrator;

void GLAPIENTRY GlDebugMessageCallback(GLenum source, GLenum type, GLuint id,
                                       GLenum severity, GLsizei length,
                                       const GLchar* message,
                                       const void* userParam) {
  fprintf(stderr,
          "GL CALLBACK: %s type = 0x%x, severity = 0x%x, message = %s\n",
          (type == GL_DEBUG_TYPE_ERROR ? "** GL ERROR **" : ""), type, severity,
          message);
}

void calibrator::InitGlDebugMessages() {
  static bool inited = false;
  if (inited) return;

  glEnable(GL_DEBUG_OUTPUT);
  glDebugMessageCallback(GlDebugMessageCallback, 0);

  inited = true;
}

std::string GetFileContent(const std::string fname) {
  std::ifstream input(fname);
  std::stringstream ss;
  ss << input.rdbuf();
  return ss.str();
}

std::optional<uint32_t> CompileShader(uint32_t shader_type,
                                      const std::string source) {
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
  if (!loaded_) {
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
  } catch (std::filesystem::filesystem_error& /*e*/) {
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

bool Shader::Load(const std::string& vertex_shader,
                  const std::string& fragment_shader) {
  Delete();
  id_ = glCreateProgram();

  const auto vs =
      CompileShader(GL_VERTEX_SHADER, LoadOrUseAsSource(vertex_shader));
  if (!vs) {
    return false;
  }

  const auto fs =
      CompileShader(GL_FRAGMENT_SHADER, LoadOrUseAsSource(fragment_shader));
  if (!fs) {
    return false;
  }

  glAttachShader(id_, *vs);
  glAttachShader(id_, *fs);

  glLinkProgram(id_);

  int success = 0;
  glGetProgramiv(id_, GL_LINK_STATUS, &success);
  if (!success) {
    PrintProgramInfoLog("Failed to link program:");
    glDeleteProgram(id_);
    id_ = 0;
  }

  glValidateProgram(id_);

  glGetProgramiv(id_, GL_VALIDATE_STATUS, &success);
  if (!success) {
    PrintProgramInfoLog("Failed to validate program:");
    glDeleteProgram(id_);
    id_ = 0;
  }

  glDeleteShader(*vs);
  glDeleteShader(*fs);

  loaded_ = true;
  return true;
}

void Shader::Set(const std::string& name, bool val) { Set(name, (int)val); };
void Shader::Set(const std::string& name, int val) {
  glUniform1i(glGetUniformLocation(id_, name.c_str()), val);
}
void Shader::Set(const std::string& name, int i1, int i2, int i3) {
  glUniform3i(glGetUniformLocation(id_, name.c_str()), i1, i2, i3);
}
void Shader::Set(const std::string& name, float val) {
  glUniform1f(glGetUniformLocation(id_, name.c_str()), val);
}
void Shader::Set(const std::string& name, float f1, float f2, float f3) {
  glUniform3f(glGetUniformLocation(id_, name.c_str()), f1, f2, f3);
}
void Shader::Set(const std::string& name, Vector3 val) {
  glProgramUniform3fv(id_, glGetUniformLocation(id_, name.c_str()), 1,
                      val.data());
}
void Shader::Set(const std::string& name, Vector4 val) {
  glProgramUniform4fv(id_, glGetUniformLocation(id_, name.c_str()), 1,
                      val.data());
}
void Shader::Set(const std::string& name, Matrix3 val) {
  glUniformMatrix3fv(glGetUniformLocation(id_, name.c_str()), 1, GL_FALSE,
                     val.data());
}
void Shader::Set(const std::string& name, Matrix4 val) {
  glUniformMatrix4fv(glGetUniformLocation(id_, name.c_str()), 1, GL_FALSE,
                     val.data());
}

SceneCamera::SceneCamera() { SetAspect(1.0f); }
void SceneCamera::SetAspect(const float aspect) {
  projection_ = GetProjection(fov_, aspect, near_, far_);
}

Matrix4 SceneCamera::GetView() { return projection_ * view_matrix_; }

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

void SceneCamera::SetMousePos(const Point2D& pos) {
  const auto dx = prev_mouse_pos_.x() - pos.x();
  const auto dy = prev_mouse_pos_.y() - pos.y();
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

Matrix4 SceneCamera::GetProjection(float fovy, float aspect, float near,
                                   float far) {
  Matrix4 out{Matrix4::Identity()};

  auto tanHalfFovy = tan((fovy / 180.0f * 3.14159265358979323846f) / 2.0f);
  out(0, 0) = 1.0f / (aspect * tanHalfFovy);
  out(1, 1) = 1.0f / (tanHalfFovy);
  out(2, 2) = -(far + near) / (far - near);
  out(2, 3) = -1.0f;
  out(3, 2) = -(2.0f * far * near) / (far - near);
  return out;
}

VertexBuffer::~VertexBuffer() { Delete(); }

VertexBuffer::VertexBuffer(VertexBuffer&& other) noexcept {
  *this = std::move(other);
}

VertexBuffer& VertexBuffer::operator=(VertexBuffer&& other) noexcept {
  if (this == &other) return *this;

  loaded_ = std::exchange(other.loaded_, false);
  id_ = std::exchange(other.id_, 0);
  size_ = std::exchange(other.size_, 0);
  normalized_ = std::exchange(other.normalized_, false);
  stride_ = std::exchange(other.stride_, 0);
  location_ = std::exchange(other.location_, 0);
  bound_ = std::exchange(other.bound_, false);

  return *this;
}

void VertexBuffer::Delete() {
  if (!loaded_) return;
}

void VertexBuffer::Create(const float* data, size_t num_floats) {
  Delete();
  glGenBuffers(1, &id_);
  loaded_ = true;

  glBindBuffer(GL_ARRAY_BUFFER, id_);

  glBufferData(GL_ARRAY_BUFFER, sizeof(float) * num_floats, data,
               GL_STATIC_DRAW);
}

void VertexBuffer::Create(const Points3D& points) {
  // Convert data to continious
  std::vector<float> buffer_data(points.size() * 3);
  size_t pos = 0;
  for (const auto& p : points) {
    for (const auto& val : p) {
      buffer_data[pos++] = val;
    }
  }

  Create(buffer_data.data(), 3 * points.size());
}

void VertexBuffer::Bind(const uint32_t location) {
  if (!loaded_) {
    std::cerr << "Tried to bind vertex buffer which has not been initialized"
              << std::endl;
    return;
  }
  if (bound_) {
    Unbind();
  }

  location_ = location;
  glBindBuffer(GL_ARRAY_BUFFER, id_);
  glVertexAttribPointer(location, size_, GL_FLOAT, normalized_, stride_,
                        (void*)0);
  glEnableVertexAttribArray(location);
  bound_ = true;
}

void VertexBuffer::Unbind() {
  if (!loaded_) {
    std::cerr << "Tried to unbind vertex buffer which has not been initialized"
              << std::endl;
    return;
  }
  if (!bound_) {
    std::cerr << "Tried to unbind vertex buffer which has not been binded"
              << std::endl;
    return;
  }

  glDisableVertexAttribArray(location_);
  bound_ = false;
}

VertexArray::VertexArray() { glGenVertexArrays(1, &id_); }

void VertexArray::Add(const uint32_t location, VertexBuffer&& buffer) {
  Bind();
  buffer.Bind(location);
  vbos_.push_back(std::move(buffer));
}

VertexArray::~VertexArray() {
  glDeleteVertexArrays(1, &id_);
  id_ = 0;
}

void VertexArray::Bind() { glBindVertexArray(id_); }

Scene::Scene() {
  InitGlDebugMessages();

  shader_.Load(R""""(
#version 330 core
layout (location = 0) in vec3 pos;

uniform mat4 transform;

void main()
{
    gl_Position = transform*vec4(pos, 1.0);
}  
)"""",
               R""""(
#version 330 core
out vec4 FragColor;
  
void main()
{
    FragColor = vec4(1.0,0.5,0.7,1.0);
}
)"""");

  const GLfloat g_vertex_buffer_data[] = {
      -1.0f, -1.0f, 0.0f, 1.0f, -1.0f, 0.0f, 0.0f, 1.0f, 0.0f,
  };

  VertexBuffer vbo;
  vbo.Create(g_vertex_buffer_data, 9);
  vao_.Add(0, std::move(vbo));
}

void Scene::Render() {
  ImGui::Begin("Scene");
  ImVec2 viewport_size = ImGui::GetContentRegionAvail();
  fb_.Init((int)viewport_size.x, (int)viewport_size.y);
  cam_.SetAspect(viewport_size.x / viewport_size.y);
  const auto mpos = ImGui::GetMousePos();
  cam_.SetMousePos({mpos.x, mpos.y});

  shader_.Use();
  fb_.Bind();

  const auto transform_location = glGetUniformLocation(shader_.GetId(), "transform");
  glUniformMatrix4fv(transform_location, 1, GL_FALSE, cam_.GetView().data());

  vao_.Bind();

  // Draw the triangle !
  glDrawArrays(GL_TRIANGLES, 0,
               3);  // Starting from vertex 0; 3 vertices total -> 1 triangle

  fb_.Unbind();

  ImGui::Image((void*)(intptr_t)fb_.GetTexture(),
               {viewport_size.x, viewport_size.y}, ImVec2{0, 1}, ImVec2{1, 0});
  ImGui::End();
}
