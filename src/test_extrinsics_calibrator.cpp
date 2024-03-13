#define CATCH_CONFIG_MAIN
#include <catch2/catch_test_macros.hpp>
#include <iostream>
#include <random>
#include "data_generator.hh"
#include "extrinsics_calibrator.hh"

using namespace calibrator;

Eigen::Affine3f DistortTransformation(const Eigen::Affine3f &T,
                                      std::mt19937 &gen,
                                      const float translation_error,
                                      const float rotation_error_deg) {
  std::uniform_real_distribution<float> rand_trans_err(-translation_error,
                                                       translation_error);
  std::uniform_real_distribution<float> rand_rot_angle_err(-rotation_error_deg,
                                                           rotation_error_deg);

  Eigen::Affine3f T_distorted;

  constexpr auto pi = 3.141592653589793f;
  const auto R_err = Eigen::AngleAxisf(rand_rot_angle_err(gen) / 180.0f * pi,
                                       Eigen::Vector3f::UnitZ()) *
                     Eigen::AngleAxisf(rand_rot_angle_err(gen) / 180.0f * pi,
                                       Eigen::Vector3f::UnitY()) *
                     Eigen::AngleAxisf(rand_rot_angle_err(gen) / 180.0f * pi,
                                       Eigen::Vector3f::UnitZ());

  T_distorted.linear() = T.linear() * R_err;

  Eigen::Vector3f t = T.translation();
  t[0] += rand_trans_err(gen);
  t[1] += rand_trans_err(gen);
  t[2] += rand_trans_err(gen);
  T_distorted.translation() = t;

  return T_distorted;
}

TEST_CASE("simple extrinsics", "[extrinsics_calibrator]")
// int main(int argc, char *argv[])
{
  ExtrinsicsCalibrator calib;

  std::vector<Eigen::Affine3f, Eigen::aligned_allocator<Eigen::Affine3f>>
      camera_T_rigs;
  camera_T_rigs.push_back(Eigen::Affine3f::Identity());
  constexpr int num_cams = 2;
  constexpr int num_frames = 1000;
  constexpr int num_points_per_frame = 4;

  // Noise added to measurements
  constexpr float kCamTRigTranslationDistortion = 0.005f;
  constexpr float kCamTRigRotationDistortionDeg = 0.1f;
  constexpr float kRigTWorldTranslationDistortion = 0.02f;
  constexpr float kRigTWorldRotationDistortionDeg = 1.0f;
  constexpr float k2dPointError = 2.0f / 500.0f;
  constexpr float k3dPointError = 0.001f;

  std::mt19937 gen{0};
  std::uniform_real_distribution<float> rand_trans_rig(-0.03f, 0.03f);
  for (int i = 1; i < num_cams; ++i) {
    Eigen::Affine3f cam_T_rig = Eigen::Affine3f::Identity();
    cam_T_rig.translation() =
        Eigen::Vector3f{rand_trans_rig(gen), rand_trans_rig(gen), 0.0f};
    camera_T_rigs.push_back(cam_T_rig);
  }

  std::vector<Eigen::Affine3f, Eigen::aligned_allocator<Eigen::Affine3f>>
      camera_T_rigs_distorted;
  for (size_t i = 0; i < camera_T_rigs.size(); ++i) {
    const auto &cam_T_rig = camera_T_rigs[i];
    if (i == 0) {
      camera_T_rigs_distorted.push_back(cam_T_rig);
      calib.AddCameraTRig(cam_T_rig, true);
    } else {
      auto cam_T_rig_distorted =
          DistortTransformation(cam_T_rig, gen, kCamTRigTranslationDistortion,
                                kCamTRigRotationDistortionDeg);
      camera_T_rigs_distorted.push_back(cam_T_rig_distorted);
      calib.AddCameraTRig(cam_T_rig_distorted);
    }
  }

  std::uniform_real_distribution<float> rand_trans(0.3f, 1.0f);
  std::uniform_real_distribution<float> rand_point_trans(-0.2f, 0.2f);
  std::uniform_real_distribution<float> rand_point_2d_err(-k2dPointError,
                                                          k2dPointError);
  std::uniform_real_distribution<float> rand_point_3d_err(-k3dPointError,
                                                          k3dPointError);
  for (int i_frame = 0; i_frame < num_frames; ++i_frame) {
    Eigen::Affine3f rig_T_world = Eigen::Affine3f::Identity();
    Eigen::Vector3f t;
    t(0) = rand_trans(gen);
    t(1) = rand_trans(gen);
    t(2) = rand_trans(gen);
    rig_T_world.translation() = t;
    const auto forward = t.normalized();
    const auto right =
        Eigen::Vector3f(0.0, 1.0, 0.0).cross(forward).normalized();
    const auto up = forward.cross(right);
    Eigen::Matrix3f R;
    R.block<1, 3>(0, 0) = forward;
    R.block<1, 3>(1, 0) = right;
    R.block<1, 3>(2, 0) = up;
    rig_T_world.linear() = R;

    const auto rig_T_world_distorted =
        DistortTransformation(rig_T_world, gen, kRigTWorldTranslationDistortion,
                              kRigTWorldRotationDistortionDeg);
    calib.AddObservationFrame(rig_T_world_distorted);

    for (int i_point = 0; i_point < num_points_per_frame; ++i_point) {
      const Eigen::Vector4f point3d(rand_point_trans(gen),
                                    rand_point_trans(gen),
                                    rand_point_trans(gen), 1.0f);
      Eigen::Vector3f point3d_distorted = point3d.block<3, 1>(0, 0);
      point3d_distorted(0) += rand_point_3d_err(gen);
      point3d_distorted(1) += rand_point_3d_err(gen);
      point3d_distorted(2) += rand_point_3d_err(gen);
      const auto point_id = calib.AddWorldPoint(i_frame, point3d_distorted);

      for (int i_cam = 0; i_cam < num_cams; ++i_cam) {
        Eigen::Vector4f point3d_cam =
            camera_T_rigs[i_cam] * rig_T_world * point3d;
        Eigen::Vector2f point_2d(point3d_cam(0) / point3d_cam(2),
                                 point3d_cam(1) / point3d_cam(2));
        point_2d(0) += rand_point_2d_err(gen);
        point_2d(1) += rand_point_2d_err(gen);

        calib.AddObservation(i_cam, point_id, point_2d);
      }
    }
  }

  const std::string fname("serialized_extrinsic_calibration.json");
  calib.Serialize(fname);
  calib.Parse(fname);
  calib.Optimize();
  for (int i = 0; i < num_cams; ++i) {
    std::cout << "-----" << std::endl;
    const auto optimized_cam = calib.GetCameraTRig(i);
    std::cout << "Original distorted cam:" << std::endl;
    std::cout << camera_T_rigs_distorted[i].matrix() << std::endl;
    std::cout << "Optimized cam:" << std::endl;
    std::cout << optimized_cam.matrix() << std::endl;
    std::cout << "Ideal cam:" << std::endl;
    std::cout << camera_T_rigs[i].matrix() << std::endl;
  }
}
