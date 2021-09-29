#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include "geometry.hh"

using namespace calibrator;

float rand_float() { return (rand() % 100) / 100.0f; }

class PlaneFixture {
 public:
  PlaneFixture() {
    for (int i = 0; i < 3; ++i) {
      points.push_back({rand_float(), rand_float(), rand_float()});
    }
    plane = EstimatePlaneFinite(points[0], points[1], points[2]);
  }

 protected:
  Plane plane;
  Points3D points;
};

TEST_CASE_METHOD(PlaneFixture, "generating points on plane", "[geometry]") {
  // Must meet condition a*x + b*y + c*y = 1
  for (const auto& p : points) {
    REQUIRE(p(0) * plane(0) + p(1) * plane(1) + p(2) * plane(2) - plane(3) ==
            Approx(0.0f).margin(1e-6));
  }
}

TEST_CASE_METHOD(PlaneFixture,
                 "linear combination of generating points on plane",
                 "[geometry]") {
  // Any linear combination is still on the plane
  for (int i = 0; i < 100; ++i) {
    const auto p0 = points[rand() % points.size()];
    const auto p1 = points[rand() % points.size()] - p0;
    const auto p2 = points[rand() % points.size()] - p0;

    const auto p = p0 + p1 + p2;
    REQUIRE(p(0) * plane(0) + p(1) * plane(1) + p(2) * plane(2) - plane(3) ==
            Approx(0.0f).margin(1e-6));
  }
}

TEST_CASE_METHOD(PlaneFixture, "plane normal", "[geometry]") {
  const auto n = PlaneNormal(plane);
  // For any linear combination of points on the plane (passing trough origin),
  // dot product with normal is 0
  for (int i = 0; i < 100; ++i) {
    const auto p0 = points[rand() % points.size()];
    const auto p1 = points[rand() % points.size()] - p0;
    const auto p2 = points[rand() % points.size()] - p0;

    const auto p = p1 + p2;
    REQUIRE(p(0) * n(0) + p(1) * n(1) + p(2) * n(2) ==
            Approx(0.0f).margin(1e-6));
  }
}

TEST_CASE_METHOD(PlaneFixture, "plane rotation matrix is rotation matrix", "[geometry]") {
  Point3D n{rand_float(), rand_float(), rand_float()};
  n.normalize();
  const auto R = RotationMatrixFromPlane(plane, n);

  // Transpose of R is it's inverse
  const auto eye = R * R.transpose();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      if (i == j) {
        REQUIRE(eye(i, j) == Approx(1).margin(1e-6));
      } else {
        REQUIRE(eye(i, j) == Approx(0).margin(1e-6));
      }
    }
  }

  // Cols and rows are unit length
  for (int i = 0; i < 3; ++i) {
    REQUIRE(R.row(i).norm() == Approx(1));
    REQUIRE(R.col(i).norm() == Approx(1));
  }
}


TEST_CASE_METHOD(PlaneFixture, "plane rotation matrix rotates correctly", "[geometry]") {
  //Point3D n{rand_float(), rand_float(), rand_float()};
  Point3D n{0.0f, 0.0f, 1.0f};
  n.normalize();
  const auto R = RotationMatrixFromPlane(plane, n);

  Points3D rotated_points;
  for (const auto& p : points) {
    rotated_points.push_back(R * p);
  }

  // For any linear combination of points on the plane (passing trough origin),
  // dot product with normal is 0
  for (int i = 0; i < 100; ++i) {
    INFO("Test iteration: " << i);
    const auto p0 = rotated_points[rand() % rotated_points.size()];
    const auto p1 = rotated_points[rand() % rotated_points.size()] - p0;
    const auto p2 = rotated_points[rand() % rotated_points.size()] - p0;

    const auto p = p1 + p2;
    REQUIRE(p(0) * n(0) + p(1) * n(1) + p(2) * n(2) ==
            Approx(0.0f).margin(1e-6));
  }
}
