#pragma once

#include <set>
#include "types.hh"

namespace calibrator {

class ExtrinsicsCalibrator {
 public:
  /// \return Camera ID
  size_t AddCameraTRig(const Eigen::Affine3f &camera_T_rig,
                       const bool freeze = false);
  Eigen::Affine3f GetCameraTRig(const size_t id);

  /// Add initial rig pose for a new observations
  /// \return Frame ID
  size_t AddObservationFrame(const Eigen::Affine3f &rig_T_world);
  Eigen::Affine3f GetObservationFrame(const size_t id);

  /// Add new world point into observation frame
  /// \return World point ID
  size_t AddWorldPoint(const size_t frame_id, const Point3D &world_point);

  /// Add world point observation in normalized image coordinates
  void AddObservation(const size_t camera_id, const size_t world_point_id,
                      const Point2D &image_point);

  void Optimize();

 private:
  std::vector<Eigen::Affine3f, Eigen::aligned_allocator<Eigen::Affine3f> >
      camera_T_rigs_;

  struct ObservationFrame {
    ObservationFrame(const Eigen::Affine3f &rig_T_world)
        : rig_T_world(rig_T_world) {}
    Eigen::Affine3f rig_T_world;
    Points3D world_points;

    struct Observation {
      size_t camera_id;
      size_t world_point_idx;
      size_t world_point_id;
      Point2D image_point;
    };
    std::vector<Observation, Eigen::aligned_allocator<Observation> >
        observations;
  };

  std::vector<ObservationFrame, Eigen::aligned_allocator<ObservationFrame> >
      observation_frames_;

  struct WorldPointInfo {
    size_t observation_frame_id;
    size_t world_point_idx;
  };
  std::vector<WorldPointInfo> world_point_infos_;
  std::set<size_t> frozen_camera_T_rigs_;
};

}  // namespace calibrator
