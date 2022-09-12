#include "extrinsics_calibrator.hh"
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include <nlohmann/json.hpp>

using namespace calibrator;
using json = nlohmann::json;

size_t ExtrinsicsCalibrator::AddCameraTRig(const Eigen::Affine3f &camera_T_rig,
                                           const bool freeze) {
  camera_T_rigs_.push_back(camera_T_rig);
  const auto id = camera_T_rigs_.size() - 1;
  if (freeze) {
    frozen_camera_T_rigs_.insert(id);
  }
  return id;
}

size_t ExtrinsicsCalibrator::AddObservationFrame(
    const Eigen::Affine3f &rig_T_world) {
  observation_frames_.emplace_back(rig_T_world);
  return observation_frames_.size() - 1;
}

size_t ExtrinsicsCalibrator::AddWorldPoint(const size_t frame_id,
                                           const Point3D &world_point) {
  WorldPointInfo info;
  info.observation_frame_id = frame_id;
  info.world_point_idx = observation_frames_[frame_id].world_points.size();
  world_point_infos_.push_back(info);

  observation_frames_[frame_id].world_points.push_back(world_point);
  return world_point_infos_.size() - 1;
}

void ExtrinsicsCalibrator::AddObservation(const size_t camera_id,
                                          const size_t world_point_id,
                                          const Point2D &image_point) {
  const auto &world_point_info = world_point_infos_[world_point_id];

  ObservationFrame::Observation observation;
  observation.camera_id = camera_id;
  observation.image_point = image_point;
  observation.world_point_idx = world_point_info.world_point_idx;
  observation.world_point_id = world_point_id;
  observation_frames_[world_point_info.observation_frame_id]
      .observations.push_back(observation);
}

struct ReprojectionErrorExtrinsics {
  ReprojectionErrorExtrinsics(const double normalized_image_point_x,
                              const double normalized_image_point_y)
      : normalized_image_point_x(normalized_image_point_x),
        normalized_image_point_y(normalized_image_point_y) {}
  template <typename T>
  bool operator()(const T *const q_rig_T_world, const T *const t_rig_T_world,
                  const T *const q_camera_T_rig, const T *const t_camera_T_rig,
                  const T *const X_world, T *residuals) const {
    // Rotate and add translation to rig coordinates
    T X_rig[3];
    ceres::QuaternionRotatePoint(q_rig_T_world, X_world, X_rig);
    X_rig[0] += t_rig_T_world[0];
    X_rig[1] += t_rig_T_world[1];
    X_rig[2] += t_rig_T_world[2];

    // Rotate and add translation to camera coordinates
    T x[3];
    ceres::QuaternionRotatePoint(q_camera_T_rig, X_rig, x);
    x[0] += t_camera_T_rig[0];
    x[1] += t_camera_T_rig[1];
    x[2] += t_camera_T_rig[2];

    // Normalize to image plane
    T xn = x[0] / x[2];
    T yn = x[1] / x[2];

    residuals[0] = xn - normalized_image_point_x;
    residuals[1] = yn - normalized_image_point_y;
    return true;
  }
  const double normalized_image_point_x;
  const double normalized_image_point_y;
};

void ExtrinsicsCalibrator::Optimize() {
  constexpr size_t num_residuals = 2;
  constexpr size_t num_param_q = 4;
  constexpr size_t num_param_t = 3;
  constexpr size_t num_param_p3d = 3;

  ceres::Problem::Options problem_options;
  problem_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  ceres::Problem problem(problem_options);
  std::vector<ReprojectionErrorExtrinsics> errors;
  std::vector<ceres::AutoDiffCostFunction<
      ReprojectionErrorExtrinsics, num_residuals, num_param_q, num_param_t,
      num_param_q, num_param_t, num_param_p3d>>
      cost_functions;

  const size_t num_cameras = camera_T_rigs_.size();
  const size_t num_observation_frames = observation_frames_.size();
  const size_t num_world_points = world_point_infos_.size();

  std::vector<std::array<double, num_param_p3d>> param_p3d(num_world_points);
  std::vector<std::array<double, num_param_q>> param_q_camera_T_rig(
      num_cameras);
  std::vector<std::array<double, num_param_t>> param_t_camera_T_rig(
      num_cameras);
  std::vector<std::array<double, num_param_q>> param_q_rig_T_world(
      num_observation_frames);
  std::vector<std::array<double, num_param_t>> param_t_rig_T_world(
      num_observation_frames);

  // Fill initial parameters
  for (size_t i = 0; i < num_cameras; ++i) {
    const auto &camera_T_rig = camera_T_rigs_[i];
    Eigen::Quaterniond q(camera_T_rig.rotation().cast<double>());
    param_q_camera_T_rig[i] = {q.w(), q.x(), q.y(), q.z()};
    const auto t = camera_T_rig.translation();
    param_t_camera_T_rig[i] = {t(0), t(1), t(2)};
  }

  for (size_t i = 0; i < num_observation_frames; ++i) {
    const auto &rig_T_world = observation_frames_[i].rig_T_world;
    Eigen::Quaterniond q(rig_T_world.rotation().cast<double>());
    param_q_rig_T_world[i] = {q.w(), q.x(), q.y(), q.z()};
    const auto t = rig_T_world.translation();
    param_t_rig_T_world[i] = {t(0), t(1), t(2)};
  }

  for (size_t i = 0; i < world_point_infos_.size(); ++i) {
    const auto &info = world_point_infos_[i];
    const auto &frame = observation_frames_[info.observation_frame_id];
    const auto &p = frame.world_points[info.world_point_idx];
    param_p3d[i] = {p(0), p(1), p(2)};
  }

  // Need to reserve all params, as otherwise pointers will move around
  // -> ceres will not know where stuff is stored
  size_t num_errors = 0;
  for (const auto &frame : observation_frames_) {
    num_errors += frame.observations.size();
  }

  errors.reserve(num_errors);
  cost_functions.reserve(num_errors);

  std::set<size_t> camera_T_rig_configured;
  std::set<size_t> rig_T_world_configured;
  std::set<size_t> world_point_configured;
  for (size_t observation_frames_id = 0;
       observation_frames_id < observation_frames_.size();
       ++observation_frames_id) {
    const auto &frame = observation_frames_[observation_frames_id];
    for (const auto &observation : frame.observations) {
      const auto &p2d = observation.image_point;
      const auto &p3d = frame.world_points[observation.world_point_idx];

      auto current_q_camera_T_rig =
          param_q_camera_T_rig[observation.camera_id].data();
      auto current_t_camera_T_rig =
          param_t_camera_T_rig[observation.camera_id].data();
      auto current_q_rig_T_world =
          param_q_rig_T_world[observation_frames_id].data();
      auto current_t_rig_T_world =
          param_t_rig_T_world[observation_frames_id].data();
      auto current_p3d = param_p3d[observation.world_point_id].data();

      errors.emplace_back(p2d.x(), p2d.y());
      cost_functions.emplace_back(&errors.back(), ceres::DO_NOT_TAKE_OWNERSHIP);
      auto loss_function = new ceres::LossFunctionWrapper(
          new ceres::HuberLoss(3.0f / 500.0f), ceres::TAKE_OWNERSHIP);
      auto block_id = problem.AddResidualBlock(&cost_functions.back(), loss_function,
                               current_q_rig_T_world, current_t_rig_T_world,
                               current_q_camera_T_rig, current_t_camera_T_rig,
                               current_p3d);

      // First time we encounter the quaternions, we need to set the quaternion
      // parameterization
      if (!camera_T_rig_configured.count(observation.camera_id)) {
        problem.SetManifold(current_q_camera_T_rig, new ceres::QuaternionManifold);
        camera_T_rig_configured.insert(observation.camera_id);

        if (frozen_camera_T_rigs_.count(observation.camera_id)) {
          // We should not change this transformation
          problem.SetParameterBlockConstant(current_q_camera_T_rig);
          problem.SetParameterBlockConstant(current_t_camera_T_rig);
        }
      }
      if (!rig_T_world_configured.count(observation_frames_id)) {
        problem.SetManifold(current_q_rig_T_world, new ceres::QuaternionManifold);
        rig_T_world_configured.insert(observation_frames_id);
      }
      if (!world_point_configured.count(observation.world_point_id)) {
        problem.SetParameterBlockConstant(current_p3d);
        world_point_configured.insert(observation.world_point_id);
      }
    }
  }

  ceres::Solver::Options options;
  options.use_nonmonotonic_steps = true;
  options.preconditioner_type = ceres::SCHUR_JACOBI;
  options.linear_solver_type = ceres::ITERATIVE_SCHUR;
  options.use_inner_iterations = true;
  options.max_num_iterations = 1000;
  options.minimizer_progress_to_stdout = true;
  options.logging_type = ceres::PER_MINIMIZER_ITERATION;  // ceres::SILENT;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // Update parameters
  for (size_t i = 0; i < num_cameras; ++i) {
    auto &camera_T_rig = camera_T_rigs_[i];
    Eigen::Quaterniond q;
    q.w() = static_cast<float>(param_q_camera_T_rig[i][0]);
    q.x() = static_cast<float>(param_q_camera_T_rig[i][1]);
    q.y() = static_cast<float>(param_q_camera_T_rig[i][2]);
    q.z() = static_cast<float>(param_q_camera_T_rig[i][3]);

    camera_T_rig.linear() = q.normalized().toRotationMatrix().cast<float>();
    camera_T_rig.translation() =
        Eigen::Vector3f{static_cast<float>(param_t_camera_T_rig[i][0]),
                        static_cast<float>(param_t_camera_T_rig[i][1]),
                        static_cast<float>(param_t_camera_T_rig[i][2])};
  }

  for (size_t i = 0; i < num_observation_frames; ++i) {
    auto &rig_T_world = observation_frames_[i].rig_T_world;
    Eigen::Quaterniond q;
    q.w() = static_cast<float>(param_q_rig_T_world[i][0]);
    q.x() = static_cast<float>(param_q_rig_T_world[i][1]);
    q.y() = static_cast<float>(param_q_rig_T_world[i][2]);
    q.z() = static_cast<float>(param_q_rig_T_world[i][3]);

    rig_T_world.translation() =
        Eigen::Vector3f{static_cast<float>(param_t_rig_T_world[i][0]),
                        static_cast<float>(param_t_rig_T_world[i][1]),
                        static_cast<float>(param_t_rig_T_world[i][2])};
  }
}

Eigen::Affine3f ExtrinsicsCalibrator::GetCameraTRig(const size_t id) const {
  return camera_T_rigs_[id];
}

Eigen::Affine3f ExtrinsicsCalibrator::GetObservationFrame(
    const size_t id) const {
  return observation_frames_[id].rig_T_world;
}

void ExtrinsicsCalibrator::Serialize(const std::string& fname) const {
  const auto serialize_transformation = [](const Eigen::Affine3f& T) {
    json root;
    for (const auto &val : T.matrix().reshaped()) {
      root.push_back(val);
    }
    return root;
  };

  const auto serialize_vector = [](const auto& v) {
    json root;
    for (const auto &val : v) {
      root.push_back(val);
    }
    return root;
  };

  const auto serialize_camera_t_rigs = [&](){
    json root;
    for (size_t i = 0; i < camera_T_rigs_.size(); ++i) {
      json camera_T_rig;
      camera_T_rig["frozen"] = static_cast<bool>(frozen_camera_T_rigs_.count(i));
      camera_T_rig["camera_T_rig"] = serialize_transformation(camera_T_rigs_[i]);
      root.push_back(camera_T_rig);
    }
    return root;
  };

  const auto serialize_world_points = [&]() {
    json root;
    for (const auto &info : world_point_infos_) {
      json wp;

      wp["frame_id"] = info.observation_frame_id;
      const auto v = observation_frames_[info.observation_frame_id].world_points[info.world_point_idx];
      wp["world_point"] = serialize_vector(v);

      root.push_back(wp);
    }
    return root;
  };

  const auto serialize_observations = [&](const auto &observations) {
    json root;
    for (const auto observation : observations) {
      json obs;

      obs["camera_id"] = observation.camera_id;
      obs["world_point_id"] = observation.world_point_id;
      obs["image_point"] = serialize_vector(observation.image_point); 

      root.push_back(obs);
    }

    return root;
  };

  const auto serialize_observation_frames = [&]() {
    json root;
    for (const auto &observation_frame : observation_frames_) {
      json obs;

      obs["rig_T_world"] = serialize_transformation(observation_frame.rig_T_world);
      obs["observations"] = serialize_observations(observation_frame.observations);

      root.push_back(obs);
    }
    return root;
  };
  
  json root;
  root["camera_T_rigs"] = serialize_camera_t_rigs();
  root["world_points"] = serialize_world_points();
  root["observation_frames"] = serialize_observation_frames();

  std::ofstream o(fname.c_str());
  o << root;
}

void ExtrinsicsCalibrator::Parse(const std::string& fname) {
  frozen_camera_T_rigs_.clear();
  world_point_infos_.clear();
  observation_frames_.clear();

  json root;
  {
    std::ifstream in(fname.c_str());
    in >> root;
  }

  const auto parse_transformation = [](const auto& root) {
    Eigen::Matrix4f T;
    for (size_t i = 0; i < root.size(); ++i) {
      T.matrix()(i) = root[i].template  get<float>();
    }
    return Eigen::Affine3f(T);
  };

  const auto parse_vector = [](const auto& root, auto &v) {
    for (size_t i = 0; i < root.size(); ++i) {
      v(i) = root[i].template get<float>();
    }
  };

  const auto parse_camera_t_rigs = [&](const auto& root) {
    for (const auto &camera_T_rig_node : root) {
      const bool frozen = camera_T_rig_node["frozen"].template get<bool>();
      const auto camera_T_rig = parse_transformation(camera_T_rig_node["camera_T_rig"]);
      AddCameraTRig(camera_T_rig, frozen);
    }
  };

  const auto parse_observation_frames_transformations = [&](const auto& root) {
    for (const auto &frame : root) {
      const auto rig_T_world = parse_transformation(frame["rig_T_world"]);
      AddObservationFrame(rig_T_world);
    }
  };

  const auto parse_world_points = [&](const auto& root) {
    for (const auto &wp : root) {
      Point3D p;
      parse_vector(wp["world_point"], p);
      const auto frame_id = wp["frame_id"].template get<uint64_t>();
      AddWorldPoint(frame_id, p);
    }
  };

  const auto parse_observation_frames_observations = [&](const auto& root) {
    for (const auto& frame : root) {
      for (const auto& obs : frame["observations"]) {
        const auto camera_id = obs["camera_id"].template get<uint64_t>();
        const auto world_point_id = obs["world_point_id"].template get<uint64_t>();
        Point2D p;
        parse_vector(obs["image_point"], p);
        AddObservation(camera_id, world_point_id, p);
      }
    }
  };

  parse_camera_t_rigs(root["camera_T_rigs"]);
  parse_observation_frames_transformations(root["observation_frames"]);
  parse_world_points(root["world_points"]);
  parse_observation_frames_observations(root["observation_frames"]);
}

void ExtrinsicsCalibrator::RemoveObservationFrame(const size_t observation_frame_id)
{
  const auto num_world_points = observation_frames_[observation_frame_id].world_points.size();

  // Remove the observation frame
  observation_frames_.erase(observation_frames_.begin()+observation_frame_id);

  // Decrement all world point ids on later frames
  for (size_t frame_id = observation_frame_id; frame_id < observation_frames_.size(); ++frame_id) {
    auto &of = observation_frames_[frame_id];
    for (auto &o : of.observations) {
      o.world_point_id -= num_world_points;
    }
  }

  // Remove world points belonging to the frame
  world_point_infos_.erase(std::remove_if(world_point_infos_.begin(), world_point_infos_.end(), 
    [=](const auto &wp) {
      return wp.observation_frame_id == observation_frame_id;
    }), world_point_infos_.end());
    
  // Decrement all wps which index later frames
  for (auto &wp : world_point_infos_) {
    if (wp.observation_frame_id >= observation_frame_id) {
      --wp.observation_frame_id;
    }
  }
}

void ExtrinsicsCalibrator::RemoveObservationFrames(const std::vector<size_t> observation_frame_ids)
{
  // Need to remove larger idx observation frames first -> no need to modify input idxs
  std::vector<size_t> ids = observation_frame_ids;
  std::sort(ids.begin(), ids.end(), std::greater());
  for (const auto &id : ids) {
    RemoveObservationFrame(id);
  }
}
