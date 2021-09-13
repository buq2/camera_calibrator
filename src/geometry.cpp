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

}  // namespace calibrator
