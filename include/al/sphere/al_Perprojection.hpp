#ifndef INLCUDE_AL_PERPROJECTION_HPP
#define INLCUDE_AL_PERPROJECTION_HPP

#include "al/core/math/al_Matrix4.hpp"
#include "al/core/graphics/al_Viewpoint.hpp"
#include "al/core/graphics/al_FBO.hpp"
#include "al/core/graphics/al_Shader.hpp"
#include "al/core/graphics/al_Texture.hpp"
#include "al/core/graphics/al_VAOMesh.hpp"
#include "al/core/graphics/al_Graphics.hpp"
#include "al/core/graphics/al_Shapes.hpp"

#include <fstream>
#include <iostream>
#include <vector>

namespace al {

Mat4f get_cube_mat(int face) {
  switch (face) {
    // GL_TEXTURE_CUBE_MAP_POSITIVE_X
    // vertex.xyz = vec3(-vertex.z, -vertex.y, -vertex.x);
    case 0: return Mat4f {
      0, 0,-1, 0,
      0,-1, 0, 0,
      -1, 0, 0, 0,
      0, 0, 0, 1
    };
    // GL_TEXTURE_CUBE_MAP_NEGATIVE_X
    // vertex.xyz = vec3(vertex.z, -vertex.y, vertex.x);
    case 1: return Mat4f {
      0, 0, 1, 0,
      0, -1, 0, 0,
      1, 0, 0, 0,
      0, 0, 0, 1
    };
    // GL_TEXTURE_CUBE_MAP_POSITIVE_Y
    // vertex.xyz = vec3(vertex.x, vertex.z, -vertex.y);
    case 2: return Mat4f {
      1, 0, 0, 0,
      0, 0, 1, 0,
      0, -1, 0, 0,
      0, 0, 0, 1
    };
    // GL_TEXTURE_CUBE_MAP_NEGATIVE_Y
    // vertex.xyz = vec3(vertex.x, -vertex.z, vertex.y);
    case 3: return Mat4f {
      1, 0, 0, 0,
      0, 0, -1, 0,
      0, 1, 0, 0,
      0, 0, 0, 1
    };
    // GL_TEXTURE_CUBE_MAP_POSITIVE_Z
    // vertex.xyz = vec3(vertex.x, -vertex.y, -vertex.z);
    case 4: return Mat4f {
      1, 0, 0, 0,
      0, -1, 0, 0,
      0, 0, -1, 0,
      0, 0, 0, 1
    };
    // GL_TEXTURE_CUBE_MAP_NEGATIVE_Z
    // vertex.xyz = vec3(-vertex.x, -vertex.y, vertex.z);
    case 5: return Mat4f {
      -1, 0, 0, 0,
      0, -1, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1
    };
  }
  return Mat4f::identity();
}

struct bhlw { float b; float h; float l; float w; };

bhlw viewport_for_cubemap_face(int idx) {
  /*
    _________
    | |2|   |
    |1|5|0|4|
    | |3|   |
    '''''''''
  */
  bhlw v;
  v.h = 0.33; v.w = 0.25;
  if      (idx == 0) { v.b = 0.33; v.l = 0.5;  }
  else if (idx == 1) { v.b = 0.33; v.l = 0.0;  }
  else if (idx == 2) { v.b = 0.66; v.l = 0.25; }
  else if (idx == 3) { v.b = 0.0;  v.l = 0.25; }
  else if (idx == 4) { v.b = 0.33; v.l = 0.75; }
  else if (idx == 5) { v.b = 0.33; v.l = 0.25; }
  return v;
}

class PerProjectionRenderConstants {
public:
  static int const sampletex_binding_point = 10;
  static int const textures_bidning_point = 11;
};

inline std::string perprojection_samplevert() { return R"(
#version 330
uniform mat4 MV;
uniform mat4 P;

layout (location = 0) in vec3 position;
layout (location = 2) in vec2 texcoord;

out vec2 texcoord_;

void main() {
  gl_Position = P * MV * vec4(position, 1.0);
  texcoord_ = texcoord;
}
)";}

inline std::string perprojection_samplefrag() { return R"(
#version 330
uniform sampler2D sample_tex;
uniform sampler2D color_tex;
uniform mat4 R;
uniform float tanFovDiv2;
in vec2 texcoord_;
out vec4 frag_color;
void main() {
  vec4 sample = texture(sample_tex, texcoord_);
  vec3 dir = sample.rgb;
  vec3 p_coord = (R * vec4(dir, 0)).xyz;
  p_coord.xy /= -p_coord.z;
  p_coord.xy /= tanFovDiv2;
  vec3 sampled_color = texture(color_tex, p_coord.xy / 2.0 + 0.5).rgb;
  frag_color = vec4(sampled_color * sample.a, 1.0);
}
)";}

class ProjectionViewport {
public:
  std::string id;
  float b, h, l, w;
  int active;
  std::string filepath;
  int width, height;
  std::vector<Vec3f> warp_data;
  std::vector<float> blend_data;
  std::vector<Vec4f> warp_and_blend_data;
};

class WarpBlendData {
public:
  std::vector<ProjectionViewport> viewports;

  void load_allosphere_calibration(const char* path, const char* hostname) {
    std::ifstream config((std::string(path) + "/" + std::string(hostname) + ".txt").c_str());

    std::string id;
    if(config >> id) {
      bool got_next = false;
      do {
        ProjectionViewport vp;
        std::string tag;

        config >> id;
        got_next = false;
        while(config >> tag) {
          vp.id = id;
          if(tag == "id") {
            got_next = true;
            break;
          }
          if(tag == "width") config >> vp.width;
          else if(tag == "height") config >> vp.height;
          else if(tag == "b") config >> vp.b;
          else if(tag == "h") config >> vp.h;
          else if(tag == "l") config >> vp.l;
          else if(tag == "w") config >> vp.w;
          else if(tag == "active") config >> vp.active;
          else if(tag == "filepath") config >> vp.filepath;
          else std::cout << "unrecognized tag: " << tag << std::endl;
        }
        // std::cout << id << std::endl << vp.width << ", " << vp.height << std::endl;
        // std::cout << vp.b << ", " << vp.h << ", " << vp.l << ", " << vp.w << ", " << vp.active << ", " << vp.filepath << std::endl;
        viewports.push_back(vp);
      } while(got_next);
    }

    // std::cout << "loaded " << viewports.size() << " viewports from " << path << "/" << hostname << ".txt" << std::endl;

    // Load warp data
    for(size_t i = 0; i < viewports.size(); i++) {
      ProjectionViewport& vp = viewports[i];

      std::ifstream file(vp.filepath, std::ios::in | std::ios::binary);
      if(!file) {
        std::cout << "could not open file: " << vp.filepath << std::endl;
        continue;
      }

      vp.warp_and_blend_data.resize(vp.width * vp.height);
      {
        auto* data = reinterpret_cast<char*>(vp.warp_and_blend_data.data());
        file.read(data, sizeof(Vec4f) * vp.width * vp.height);
      }
      file.close();
      // std::cout << "loaded warp/blend data from " << vp.filepath << std::endl;
    }
  }

  void load_desktop_mode_calibration() {
    viewports.resize(6);
    for (int i = 0; i < 6; i += 1) {
      auto& vp = viewports[i];
      vp.width = 256;
      vp.height = 256;
      bhlw v = viewport_for_cubemap_face(i);
      vp.b = v.b;
      vp.h = v.h;
      vp.l = v.l;
      vp.w = v.w;
      vp.active = false;
      vp.filepath = "";
    }

    for(size_t i = 0; i < viewports.size(); i++) {
      auto& vp = viewports[i];
      vp.warp_and_blend_data.clear();
    }
  }
};


class PerProjectionRender {
public:
  struct ProjectionInfo {
    std::shared_ptr<Texture> texture[2];
    std::shared_ptr<Texture> warp_texture;
    Mat4f pc_matrix, p_matrix, r_matrix;
    float tanFovDiv2;
  };

  WarpBlendData warpblend_;
  int res_ = 4096;
  Pose pose_;
  Viewpoint view_ {pose_};
  Viewport viewport_;
  std::vector<ProjectionInfo> projection_infos_;
  RBO rbo_;
  FBO fbo_;
  // ShaderProgram pp_shader_;
  ShaderProgram composite_shader_;
  Lens lens_;
  Graphics* g;
  VAOMesh texquad;
  bool calibration_loaded = false;
  bool did_begin = false;
  int current_eye = 0;
  int current_projection = 0;

  // instead of push/pop
  Lens prev_lens_;
  ShaderProgram* prev_shader_;

  Mat4f get_rotation_matrix(Vec3f axis, float angle) {
    Mat4f r;
    float cosT = cos(angle);
    float sinT = sin(angle);
    float X = axis.x, Y = axis.y, Z = axis.z;
    r.set(
      cosT + X * X * (1 - cosT), X * Y * (1 - cosT) - Z * sinT, X * Z * (1 - cosT) + Y * sinT, 0,
      Y * X * (1 - cosT) + Z * sinT, cosT + Y * Y * (1 - cosT), Y * Z * (1 - cosT) - X * sinT, 0,
      Z * X * (1 - cosT) - Y * sinT, Z * Y * (1 - cosT) + X * sinT, cosT + Z * Z * (1 - cosT), 0,
      0, 0, 0, 1
    );
    return r;
  }

  // load_calibration_data needed to be called before this function
  // also projection_infos_.resize(warpblend_.viewports.size());
  void update_resolution(int resolution) {
    res_ = resolution;
    viewport_.set(0, 0, res_, res_);

    if (!calibration_loaded) return;

    // we have calibration data loaded,
    // so update textures(color) & renderbuffer(depth)
    projection_infos_.resize(warpblend_.viewports.size());
    for(size_t index = 0; index < warpblend_.viewports.size(); index++) {
      ProjectionInfo& info = projection_infos_[index];
      info.texture[0].reset(new Texture());
      info.texture[1].reset(new Texture());
      info.texture[0]->create2D(res_, res_, GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE);
      info.texture[0]->filter(Texture::LINEAR);
      info.texture[1]->create2D(res_, res_, GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE);
      info.texture[1]->filter(Texture::LINEAR);
    }
    rbo_.create(res_, res_);
    fbo_.bind();
    fbo_.attachTexture2D(*projection_infos_[0].texture[0]);
    fbo_.attachRBO(rbo_);
    fbo_.unbind();
  }

  void sphereRadius(float r) {
    lens_.focalLength(r);
    if (did_begin) g->lens(lens_);
  }

  void init(float near=0.1, float far=1000, float radius = 1e10)
  {
    lens_.focalLength(radius);
    viewport_.set(0, 0, res_, res_);

    projection_infos_.resize(warpblend_.viewports.size());
    for(size_t index = 0; index < warpblend_.viewports.size(); index++) {
      const ProjectionViewport& vp = warpblend_.viewports[index];
      ProjectionInfo& info = projection_infos_[index];
      info.texture[0].reset(new Texture());
      info.texture[1].reset(new Texture());
      info.texture[0]->create2D(res_, res_, GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE);
      info.texture[1]->create2D(res_, res_, GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE);
      // Determine projection dimensions.
      // First determine the central direction.
      Vec3f direction(0, 0, 0);
      for(int i = 0; i < vp.width * vp.height; i++) {
        direction.x += vp.warp_and_blend_data[i].x;
        direction.y += vp.warp_and_blend_data[i].y;
        direction.z += vp.warp_and_blend_data[i].z;
      }
      direction = direction.normalize();
#if 0
      // Determine the radius.
      float dot_max = 1;
      for(int i = 0; i < vp.width * vp.height; i++) {
        auto d = Vec3f{
          vp.warp_and_blend_data[i].x,
          vp.warp_and_blend_data[i].y,
          vp.warp_and_blend_data[i].z 
        }.normalize();
        float dot = d.dot(direction);
        if(dot_max > dot) dot_max = dot;
      }
      if(dot_max < 0.1) {
        // The angle is too wide, throw an error.
        std::cout << "unable to use per-projection mode, viewport angle too large." << std::endl;
      }
      float fov = std::acos(dot_max) * 2.0f;
      // std::cout << "fov: " <<fov << std::endl;
#endif

      // value 2/3 * PI is the smallest value that is bigger than fov of
      // any projector in AlloSphere
      // setting same sampling fov for all projectors has advantage that
      // pixels density in terms of OpenGL space dimension remains constant
      // throughout projectors
      float fov = 2.0f / 3 * M_PI;
      Vec3f rotation_axis = Vec3f(0, 0, -1).cross(direction);
      rotation_axis = rotation_axis.normalize();
      // (0, 0, -1) because that is the direction camera is looking at
      float rotation_angle = std::acos(Vec3f(0, 0, -1).dot(direction));
      Mat4f rmat = get_rotation_matrix(rotation_axis, -rotation_angle);
      Mat4f proj;
      proj.set(
        1.0f / std::tan(fov / 2.0f), 0, 0, 0,
        0, 1.0f / std::tan(fov / 2.0f), 0, 0,
        0, 0, (near + far) / (near - far), (2.0f * near * far) / (near - far),
        0, 0, -1, 0
      );

      Mat4f::multiply(info.pc_matrix, proj, rmat);
      info.r_matrix = rmat;
      info.p_matrix = proj;
      info.tanFovDiv2 = tan(fov / 2.0);

      info.warp_texture.reset(new Texture());
      info.warp_texture->create2D(vp.width, vp.height, GL_RGBA32F, GL_RGBA, GL_FLOAT);
      info.warp_texture->submit(vp.warp_and_blend_data.data());
    }

    rbo_.create(res_, res_);
    fbo_.bind();
    fbo_.attachTexture2D(*projection_infos_[0].texture[0]);
    fbo_.attachRBO(rbo_);                   // ^ attach left texture by deafult
    fbo_.unbind();

    // pp_shader_.compile(perprojection_vert(), perprojection_frag());
    // pp_shader_.begin();
    // pp_shader_.uniform("omni_radius", radius_);
    // pp_shader_.end();

    composite_shader_.compile(perprojection_samplevert(), perprojection_samplefrag());
    composite_shader_.begin();
    composite_shader_.uniform("sample_tex", PerProjectionRenderConstants::sampletex_binding_point);
    composite_shader_.uniform("color_tex", PerProjectionRenderConstants::textures_bidning_point);
    composite_shader_.end();

    // prepare textured quad to fill viewport with the result
    addTexQuad(texquad);
    texquad.update();
  }

  void load_calibration_data(const char* path, const char* hostname) {
    warpblend_.load_allosphere_calibration(path, hostname);
    calibration_loaded = true;
  }

  void load_and_init_as_desktop_config(float near=0.1, float far=1000, float radius = 1e10) {
    // add six projection infos that will serve as each face of cubemap
    // https://www.khronos.org/opengl/wiki/Cubemap_Texture
    // 0: GL_TEXTURE_CUBE_MAP_POSITIVE_X
    // 1: GL_TEXTURE_CUBE_MAP_NEGATIVE_X
    // 2: GL_TEXTURE_CUBE_MAP_POSITIVE_Y
    // 3: GL_TEXTURE_CUBE_MAP_NEGATIVE_Y
    // 4: GL_TEXTURE_CUBE_MAP_POSITIVE_Z
    // 5: GL_TEXTURE_CUBE_MAP_NEGATIVE_Z

    warpblend_.load_desktop_mode_calibration();
    calibration_loaded = true;

    lens_.focalLength(radius);
    res_ = 256;
    viewport_.set(0, 0, res_, res_);

    projection_infos_.resize(6);
    for(int index = 0; index < 6; index++) {
      ProjectionInfo& info = projection_infos_[index];
      info.texture[0].reset(new Texture());
      info.texture[1].reset(new Texture());
      info.texture[0]->create2D(res_, res_, GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE);
      info.texture[1]->create2D(res_, res_, GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE);

      float fov = 3.1415926535 / 2;
      Mat4f rmat = get_cube_mat(index);
      Mat4f proj;
      proj.set(
        1.0f / std::tan(fov / 2.0f), 0, 0, 0,
        0, 1.0f / std::tan(fov / 2.0f), 0, 0,
        0, 0, (near + far) / (near - far), (2.0f * near * far) / (near - far),
        0, 0, -1, 0
      );
      Mat4f::multiply(info.pc_matrix, proj, rmat);
      info.r_matrix = rmat;
      info.p_matrix = proj;
      info.tanFovDiv2 = tan(fov / 2.0);
      info.warp_texture.reset();
    }

    rbo_.create(res_, res_);
    fbo_.bind();
    fbo_.attachTexture2D(*projection_infos_[0].texture[0]);
    fbo_.attachRBO(rbo_);
    fbo_.unbind();

    addTexQuad(texquad);
    texquad.update();
  }

  void begin(Graphics& graphics) {
    g = &graphics;
    g->pushFramebuffer(fbo_);
    g->pushViewport(viewport_);
    g->pushViewMatrix(view_mat(pose_));
    g->pushProjMatrix();
    // TODO: pushLens and pushShader
    prev_lens_ = g->lens();
    g->lens(lens_);
    prev_shader_ = g->shaderPtr();
    did_begin = true;
    // g->shader(pp_shader_);
  }

  // must only be used between begin and end
  void set_eye(int i) {
    if (i == 0) {
      current_eye = 0;
      g->eye(Graphics::LEFT_EYE);
    }
    else if (i == 1) {
      current_eye = 1;
      g->eye(Graphics::RIGHT_EYE);
    }
    else if (i == -1) {
      current_eye = 0;
      g->eye(Graphics::MONO_EYE);
    }
  }

  int num_projections() {
    return projection_infos_.size();
  }

  // must only use between begin and end
  void set_projection(int index) {
    g->projMatrix(projection_infos_[index].pc_matrix);
    fbo_.attachTexture2D(*projection_infos_[index].texture[current_eye]);
    current_projection = index;
  }

  Mat4f get_r_mat_for_current_projection() {
    return projection_infos_[current_projection].r_matrix;
  }

  Mat4f get_p_mat_for_current_projection() {
    return projection_infos_[current_projection].p_matrix;
  }

  void end() {
    g->popFramebuffer();
    g->popViewport();
    g->popViewMatrix();
    g->popProjMatrix();
    g->lens(prev_lens_);
    if (prev_shader_ != nullptr) {
      g->shader(*prev_shader_);
    }
    did_begin = false;
  }
  
  void pose(Pose const& p) { pose_ = p; if (did_begin) g->viewMatrix(view_mat(pose_)); }
  Pose& pose() { return pose_; }
  Pose const& pose() const { return pose_; }

  void composite(Graphics& g, int eye=0) {
    g.pushCamera(Viewpoint::IDENTITY);
    g.pushViewport();
    GLint dims[4];
    glGetIntegerv(GL_VIEWPORT, dims);
    int width = dims[2];
    int height = dims[3];
    g.shader(composite_shader_);
    for(size_t i = 0; i < warpblend_.viewports.size(); i++) {
      const ProjectionViewport& vp = warpblend_.viewports[i];
      g.viewport(vp.l * width, vp.b * height, vp.w * width, vp.h * height);
      projection_infos_[i].warp_texture->bind(PerProjectionRenderConstants::sampletex_binding_point);
      projection_infos_[i].texture[eye]->bind(PerProjectionRenderConstants::textures_bidning_point);
      g.shader().uniform("R", projection_infos_[i].r_matrix);
      g.shader().uniform("tanFovDiv2", projection_infos_[i].tanFovDiv2);
      g.draw(texquad); // fill viewport
      projection_infos_[i].warp_texture->unbind(PerProjectionRenderConstants::sampletex_binding_point);
      projection_infos_[i].texture[eye]->unbind(PerProjectionRenderConstants::textures_bidning_point);
    }
    g.popViewport();
    g.popCamera();
  }

  void composite_desktop(Graphics& g, int eye=0) {
    g.pushCamera(Viewpoint::IDENTITY);
    g.pushViewport();
    GLint dims[4];
    glGetIntegerv(GL_VIEWPORT, dims);
    int width = dims[2];
    int height = dims[3];
    for(size_t i = 0; i < warpblend_.viewports.size(); i++) {
      const ProjectionViewport& vp = warpblend_.viewports[i];
      g.viewport(vp.l * width, vp.b * height, vp.w * width, vp.h * height);
      g.pushMatrix();
      // flip cubemap faces for them to match the orientation with other faces
      if (i == 2 || i == 3) g.scale(1, 1, 1);
      else g.scale(-1, -1, 1);
      g.quadViewport(*(projection_infos_[i].texture[eye]));
      g.popMatrix();
    }
    g.popViewport();
    g.popCamera();
  }
};
}

#endif
