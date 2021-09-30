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
  // Must meet condition a*x + b*y + c*y + d = 0
  for (const auto& p : points) {
    REQUIRE(p(0) * plane(0) + p(1) * plane(1) + p(2) * plane(2) + plane(3) ==
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
    REQUIRE(p(0) * plane(0) + p(1) * plane(1) + p(2) * plane(2) + plane(3) ==
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

    INFO("Test iteration: " << i);
    REQUIRE(p(0) * n(0) + p(1) * n(1) + p(2) * n(2) ==
            Approx(0.0f).margin(1e-6));
  }
}

void TestIsRotationMatrix(const Matrix3& R) {
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

TEST_CASE_METHOD(PlaneFixture, "plane rotation matrix is rotation matrix",
                 "[geometry]") {
  Point3D n{rand_float(), rand_float(), rand_float()};
  n.normalize();
  const auto R = RotationMatrixFromPlane(plane, n);

  TestIsRotationMatrix(R);
}

TEST_CASE_METHOD(PlaneFixture, "plane rotation matrix rotates correctly",
                 "[geometry]") {
  // RotationMatrixFromPlane currently supports only
  // UnitZ() as new normal for the rotated plane.
  // Need to investigate if other normals should be used.
  // Point3D n{rand_float(), rand_float(), rand_float()};
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

TEST_CASE_METHOD(PlaneFixture, "project to plane", "[geometry]") {
  for (int i = 0; i < 100; ++i) {
    const Point3D p{rand_float(), rand_float(), rand_float()};
    const auto projected = ProjectToPlane(plane, p);
    INFO("Test iteration: " << i);
    REQUIRE(plane(0) * projected(0) + plane(1) * projected(1) +
                plane(2) * projected(2) + plane(3) ==
            Approx(0).margin(1e-5));
  }
}

TEST_CASE_METHOD(PlaneFixture, "project to plane using different normal",
                 "[geometry]") {
  int num_tests_run = 0;
  for (int i = 0; i < 100; ++i) {
    const Point3D p{rand_float(), rand_float(), rand_float()};
    const Point3D n{rand_float(), rand_float(), rand_float()};

    // Check that the proejction normal is not very much parallel to the plane.
    // Otherwise we will have numerical issues and test will fail.
    const auto condition =
        plane.block<3, 1>(0, 0).normalized().dot(n.normalized());
    if (condition < 1e-2) {
      // Too parallel
      continue;
    }

    const auto projected = ProjectToPlane(plane, p, n.normalized());
    INFO("Test iteration: " << i);
    ++num_tests_run;
    REQUIRE(plane(0) * projected(0) + plane(1) * projected(1) +
                plane(2) * projected(2) + plane(3) ==
            Approx(0).margin(1e-5));
  }

  // Make sure we ran at least one test
  REQUIRE(num_tests_run > 0);
}

TEST_CASE("rotation matrix fixing", "[geometry]") {
  for (int i = 0; i < 100; ++i) {
    INFO("Test iteration: " << i);
    Matrix3 R(Matrix3::Random());
    const auto fixed_R = FixRotationMatrix(R);
    TestIsRotationMatrix(fixed_R);
  }
}

TEST_CASE("homography", "[geometry]") {
  for (int i = 0; i < 10; ++i) {
    INFO("Test iteration: " << i);
    Matrix3 H(Matrix3::Random());
    H /= H(2, 2);
    Points3D p1, p2;
    for (int j = 0; j < 10; ++j) {
      p1.emplace_back(rand_float(), rand_float(), 1.0f);
      p2.push_back(H * p1.back());
      p2.back() /= p2.back()(2);
    }

    auto H2 = EstimateHomography(p1, p2);
    H2 /= H2(2, 2);

    const Matrix3 diff = H - H2;
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        REQUIRE(diff(row, col) == Approx(0).margin(1e-4));
      }
    }
  }
}
