#include "geometry.hh"

namespace calibrator {

Plane EstimatePlaneFinite(const Point3D& p1, const Point3D& p2,
                          const Point3D& p3) {
  Matrix3 A;
  A << p1.x(), p1.y(), p1.z(), p2.x(), p2.y(), p2.z(), p3.x(), p3.y(), p3.z();
  Point3D p;
  p << 1.0f, 1.0f, 1.0f;

  Plane out;
  out.block<3, 1>(0, 0) = A.inverse() * p;
  out(3) = 1.0f;
  return out;
}

Point3D PlaneNormal(const Plane& plane) {
  return plane.block<3, 1>(0, 0).normalized();
}

Matrix3 RotationMatrixFromPlane(const Plane& plane, const Point3D& new_normal) {
  const auto normal = PlaneNormal(plane);
  const auto v1 = normal.cross(new_normal);
  const auto v2 = normal.cross(v1);

  Matrix3 out;
  out.row(0) = v1;
  out.row(1) = v2;
  out.row(2) = normal;

  return out;
}

Point3D ProjectToPlane(const Plane& plane, const Point3D& p,
                       const std::optional<Point3D>& projection_direction) {
  Point3D direction;
  if (!projection_direction) {
    // Plane normal without scaling
    Point3D direction = plane.block<3, 1>(0, 0);
  } else {
    direction = *projection_direction;
  }

  // (p-direction*t).dot(plane.block<3,1>(0,0)) + plane(3) = 0
  const auto pn = plane.block<3, 1>(0, 0);
  const auto t = (p.dot(pn) + plane(3)) / direction.dot(pn);
  return p - direction * t;
}

template<typename Points1, typename Points2>
Matrix3 EstimateHomography(const Points1& p1, const Points2& p2)
{
  // HZ2 Algorithm 4.1 p.91 (note also Algorithm 4.2 p.109)
  // HZ2 (4.1) and (4.3) p.89 for 2D->2D case
  assert(p1.size() == p2.size());
  const auto n = p1.size();
  Eigen::MatrixXf A(2*n, 9);
  for (int i = 0; i < n; ++i) {
    const auto row = i*2;
    A.block<1,3>(row, 0).setZero();
    A(row, 3) = -p1[i].x();
    A(row, 4) = -p1[i].y();
    A(row, 5) = -1.0f;
    A(row, 6) = p1[i].x() * p2[i].y();
    A(row, 7) = p1[i].y() * p2[i].y();
    A(row, 8) = p2[i].y();

    A(row+1, 0) = p1[i].x();
    A(row+1, 1) = p1[i].y();
    A(row+1, 2) = 1.0f;
    A.block<1,3>(row+1, 3).setZero();
    A(row+1, 6) = -p1[i].x() * p2[i].x();
    A(row+1, 7) = -p1[i].y() * p2[i].x();
    A(row+1, 8) = -p2[i].x();
  }

  Eigen::JacobiSVD<Eigen::MatrixXf> svd_computer(A, Eigen::ComputeFullV);
  const auto h = svd_computer.matrixV().rightCols(1);
  Matrix3 H;
  H << h(0), h(1), h(2),
       h(3), h(4), h(5),
       h(6), h(7), h(8);
  
  return H;
}

Matrix3 EstimateHomography(const Points2D& p1, const Points3D& p2)
{
  return EstimateHomography<Points2D, Points3D>(p1, p2);
}

Matrix3 EstimateHomography(const Points3D& p1, const Points2D& p2)
{
  return EstimateHomography<Points3D, Points2D>(p1, p2);
}

Matrix3 EstimateHomography(const Points2D& p1, const Points2D& p2)
{
  return EstimateHomography<Points2D, Points2D>(p1, p2);
}

Matrix3 EstimateKFromHomographies(const std::vector<Matrix3>& Hs)
{
  // Zhang's camera calibration:
  // https://www.microsoft.com/en-us/research/wp-content/uploads/2016/02/tr98-71.pdf
  assert(Hs.size() >= 3);

  auto vij = [&](const Eigen::Matrix3d& H, int i, int j) {
    Eigen::Matrix<double, 1, 6> v;
    v << H.col(i)(0)*H.col(j)(0), 
         H.col(i)(0)*H.col(j)(1) + H.col(i)(1)*H.col(j)(0), 
         H.col(i)(1)*H.col(j)(1), 
         H.col(i)(2)*H.col(j)(0) + H.col(i)(0)*H.col(j)(2), 
         H.col(i)(2)*H.col(j)(1) + H.col(i)(1)*H.col(j)(2), 
         H.col(i)(2)*H.col(j)(2);
    return v;
  };

  const auto n = Hs.size();
  // Using double matrix for stability
  // TODO: Check if we really need double matrix
  Eigen::MatrixXd A(2*n + 1, 6);
  for (size_t i = 0; i < n; ++i) {
    const auto& H = Hs[i].cast<double>();
    const auto row = i*2;
    A.row(row) = vij(H, 0,1);
    A.row(row+1) = vij(H, 0,0) - vij(H, 1,1);
  }

  // Zero skew, does not enforce. Weighted with number of images
  A.row(2*n).setZero();
  A(2*n,1) = static_cast<double>(n); 

  Eigen::JacobiSVD<Eigen::MatrixXd> svd_computer(A, Eigen::ComputeFullV);
  const auto b = svd_computer.matrixV().rightCols(1);

  const auto B11 = b(0);
  const auto B12 = b(1);
  const auto B22 = b(2);
  const auto B13 = b(3);
  const auto B23 = b(4);
  const auto B33 = b(5);

  auto v0 = (B12*B13 - B11*B23)/(B11*B22 - B12*B12);
  const auto l = B33 - (B13*B13 + v0*(B12*B13 - B11*B23))/B11;
  const auto alpha = sqrt(l/B11);
  const auto beta = sqrt(l*B11/(B11*B22 - B12*B12));
  // Force zero skew, otherwise we could use:
  const auto y = 0.0f; 
  //const auto y = -B12 * alpha*alpha*beta/l;
  const auto u0 = y*v0/beta-B13*alpha*alpha/l;

  Matrix3 K;
  K << static_cast<float>(alpha), static_cast<float>(y), static_cast<float>(u0),
       0.0f, static_cast<float>(beta), static_cast<float>(v0),
       0.0f, 0.0f, 1.0f;
  return K;
}

}  // namespace calibrator
