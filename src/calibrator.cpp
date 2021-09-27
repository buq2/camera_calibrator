#include "calibrator.hh"
#include "convert.hh"
#include "geometry.hh"

#include <array>
#include <opencv2/calib3d.hpp>
#include "ceres/ceres.h"
#include "ceres/rotation.h"

using namespace calibrator;

Calibrator::Calibrator(const int img_width, const int img_height)
    : width_(img_width), height_(img_height) {}

void Calibrator::EstimateOpenCv(const std::vector<Points2D>& in_img_points,
                                const std::vector<Points3D>& in_world_points) {
  assert(in_img_points.size() == in_world_points.size());
  const auto n_img = in_img_points.size();
  std::vector<std::vector<cv::Point2f>> image_points(n_img);
  std::vector<std::vector<cv::Point3f>> world_points(n_img);
  for (size_t i = 0; i < n_img; ++i) {
    const auto n_p = in_img_points[i].size();
    assert(n_p == in_world_points[i].size());

    image_points[i].resize(n_p);
    world_points[i].resize(n_p);

    for (size_t j = 0; j < n_p; ++j) {
      image_points[i][j] = ToCvPoint2<float>(in_img_points[i][j]);
      world_points[i][j] = ToCvPoint3<float>(in_world_points[i][j]);
    }
  }

  cv::Mat R, T;
  cv::Matx33f camera_matrix = ToCvMat3x3<float>(K_);
  cv::Mat dist_coeffs = ToCvMat<float>(dist_);
  cv::calibrateCamera(world_points, image_points, cv::Size(width_, height_),
                      camera_matrix, dist_coeffs, R, T);

  K_ = ToEigen<decltype(K_), float>(camera_matrix);

  // dist_coeffs is a double matrix
  dist_ = ToEigen<decltype(dist_), double>(dist_coeffs);
}

void Calibrator::Estimate(const std::vector<Points2D>& in_img_points,
                          const std::vector<Points3D>& in_world_points) {
  assert(in_img_points.size() == in_world_points.size());
  const auto num_imgs = in_img_points.size();

  std::vector<Matrix3> Hs;
  for (size_t i = 0; i < num_imgs; ++i) {
    Hs.push_back(EstimateHomography(in_world_points[i], in_img_points[i]));
  }
  K_ = EstimateKFromHomographies(Hs);

  std::vector<Quaternion> qs;
  std::vector<Point3D> ts;
  const auto K_inv = K_.inverse();
  for (const auto& H : Hs) {
    const auto& [R, t] = RecoverExtrinsics(K_inv, H);
    qs.emplace_back(R);
    ts.push_back(t);
  }

  Optimize(in_img_points, in_world_points, qs, ts);
}

template <typename T>
inline void Distort(const T& fx, const T& fy, const T& px, const T& py,
                    const T& k1, const T& k2, const T& k3, const T& p1,
                    const T& p2, const T& nx, const T& ny, T* image_x,
                    T* image_y) {
  T x = nx;
  T y = ny;
  T r2 = x * x + y * y;
  T r4 = r2 * r2;
  T r6 = r4 * r2;
  T r_mult = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
  T xd = x * r_mult + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
  T yd = y * r_mult + 2.0 * p2 * x * y + p1 * (r2 + 2.0 * y * y);
  *image_x = fx * xd + px;
  *image_y = fy * yd + py;
}

enum {
  OFFSET_FX,
  OFFSET_FY,
  OFFSET_PX,
  OFFSET_PY,
  OFFSET_K1,
  OFFSET_K2,
  OFFSET_K3,
  OFFSET_P1,
  OFFSET_P2,
  NUM_INTRINSICS
};

// Heavily inspired by:
// https://ceres-solver.googlesource.com/ceres-solver/+/master/examples/libmv_bundle_adjuster.cc
struct ReprojectionError {
  ReprojectionError(const double meas_x, const double meas_y)
      : meas_x(meas_x), meas_y(meas_y) {}
  template <typename T>
  bool operator()(const T* const intr, const T* const q, const T* const t,
                  const T* const X, T* residuals) const {
    // Get intrinsics
    const T& fx = intr[OFFSET_FX];
    const T& fy = intr[OFFSET_FY];
    const T& px = intr[OFFSET_PX];
    const T& py = intr[OFFSET_PY];
    const T& k1 = intr[OFFSET_K1];
    const T& k2 = intr[OFFSET_K2];
    const T& k3 = intr[OFFSET_K3];
    const T& p1 = intr[OFFSET_P1];
    const T& p2 = intr[OFFSET_P2];
    // Rotate and add translation
    T x[3];
    ceres::QuaternionRotatePoint(q, X, x);
    x[0] += t[0];
    x[1] += t[1];
    x[2] += t[2];
    // Normalize to image plane
    T xn = x[0] / x[2];
    T yn = x[1] / x[2];

    // Apply distortion
    T pred_x, pred_y;
    Distort(fx, fy, px, py, k1, k2, k3, p1, p2, xn, yn, &pred_x, &pred_y);

    residuals[0] = pred_x - meas_x;
    residuals[1] = pred_y - meas_y;
    return true;
  }
  const double meas_x;
  const double meas_y;
};

void Calibrator::Optimize(const std::vector<Points2D>& in_img_points,
                          const std::vector<Points3D>& in_world_points,
                          std::vector<Quaternion>& qs,
                          std::vector<Point3D>& ts) {
  const auto n_img = in_img_points.size();
  assert(n_img == in_world_points.size());
  assert(n_img == qs.size());
  assert(n_img == ts.size());

  constexpr int num_residuals = 2;
  constexpr int num_param_intrinsics = static_cast<int>(NUM_INTRINSICS);
  constexpr int num_param_q = 4;
  constexpr int num_param_t = 3;
  constexpr int num_param_p3d = 3;

  ceres::Problem::Options problem_options;
  problem_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  ceres::Problem problem(problem_options);
  std::vector<ReprojectionError> errors;
  std::vector<ceres::AutoDiffCostFunction<ReprojectionError, num_residuals,
                                          num_param_intrinsics, num_param_q,
                                          num_param_t, num_param_p3d>>
      cost_functions;

  double cam_intrinsics[num_param_intrinsics];
  cam_intrinsics[OFFSET_FX] = K_(0, 0);
  cam_intrinsics[OFFSET_FY] = K_(1, 1);
  cam_intrinsics[OFFSET_PX] = K_(0, 2);
  cam_intrinsics[OFFSET_PY] = K_(1, 2);
  cam_intrinsics[OFFSET_K1] = dist_(0);
  cam_intrinsics[OFFSET_K2] = dist_(1);
  cam_intrinsics[OFFSET_P1] = dist_(2);
  cam_intrinsics[OFFSET_P2] = dist_(3);
  cam_intrinsics[OFFSET_K3] = dist_(4);

  std::vector<std::array<double, num_param_q>> param_q(n_img);
  std::vector<std::array<double, num_param_t>> param_t(n_img);
  std::vector<std::vector<std::array<double, num_param_p3d>>> param_p3d(n_img);

  // Need to reserve all params, as otherwise pointers will move around
  // -> ceres will not know where stuff is stored
  size_t num_errors = 0;
  for (size_t i_img = 0; i_img < n_img; ++i_img) {
    const auto& p2ds = in_img_points[i_img];
    const auto num_p = p2ds.size();
    param_p3d[i_img].resize(num_p);
    num_errors += num_p;
  }

  errors.reserve(num_errors);
  cost_functions.reserve(num_errors);

  for (size_t i_img = 0; i_img < n_img; ++i_img) {
    // Put qt params
    const auto& q = qs[i_img];
    const auto& t = ts[i_img];
    param_q[i_img] =
        std::array<double, num_param_q>{q.w(), q.x(), q.y(), q.z()};
    param_t[i_img] = std::array<double, num_param_t>{t.x(), t.y(), t.z()};
    auto current_cam_q = param_q[i_img].data();
    auto current_cam_t = param_t[i_img].data();

    const auto& p2ds = in_img_points[i_img];
    const auto& p3ds = in_world_points[i_img];
    const auto num_p = p2ds.size();
    assert(num_p == p3ds.size());

    auto& p_p3d = param_p3d[i_img];
    for (size_t i_p = 0; i_p < num_p; ++i_p) {
      const auto p3d = p3ds[i_p];
      const auto p2d = p2ds[i_p];
      p_p3d[i_p] = std::array<double, num_param_p3d>{p3d.x(), p3d.y(), p3d.z()};

      errors.emplace_back(p2d.x(), p2d.y());
      cost_functions.emplace_back(&errors.back(), ceres::DO_NOT_TAKE_OWNERSHIP);
      problem.AddResidualBlock(&cost_functions.back(), NULL, cam_intrinsics,
                               current_cam_q, current_cam_t, p_p3d[i_p].data());
      problem.SetParameterization(current_cam_q,
                                  new ceres::QuaternionParameterization);

      // If 3D points are not moved, we can set it here
      problem.SetParameterBlockConstant(p_p3d[i_p].data());
    }
  }

  ceres::Solver::Options options;
  options.use_nonmonotonic_steps = true;
  options.preconditioner_type = ceres::SCHUR_JACOBI;
  options.linear_solver_type = ceres::ITERATIVE_SCHUR;
  options.use_inner_iterations = true;
  options.max_num_iterations = 100;
  options.minimizer_progress_to_stdout = false;
  options.logging_type = ceres::SILENT;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  //std::cout << summary.FullReport();

  // Put params back
  K_(0, 0) = static_cast<float>(cam_intrinsics[OFFSET_FX]);
  K_(1, 1) = static_cast<float>(cam_intrinsics[OFFSET_FY]);
  K_(0, 2) = static_cast<float>(cam_intrinsics[OFFSET_PX]);
  K_(1, 2) = static_cast<float>(cam_intrinsics[OFFSET_PY]);
  dist_(0) = static_cast<float>(cam_intrinsics[OFFSET_K1]);
  dist_(1) = static_cast<float>(cam_intrinsics[OFFSET_K2]);
  dist_(2) = static_cast<float>(cam_intrinsics[OFFSET_P1]);
  dist_(3) = static_cast<float>(cam_intrinsics[OFFSET_P2]);
  dist_(4) = static_cast<float>(cam_intrinsics[OFFSET_K3]);
}
