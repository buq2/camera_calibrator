#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include "calibrator.hh"
#include "data_generator.hh"

using namespace calibrator;

class DataGeneratorFixture {
 public:
  DataGeneratorFixture()
      : dist(5), generator(1600, 1000), calibrator(1600, 1000) {
    constexpr float f = 1000.0f;
    K << f, 0, generator.GetWidth() / 2.0f, 0, f, generator.GetHeight() / 2.0f,
        0, 0, 1;
    generator.SetK(K);

    dist << -4.0e-2f, 5e-4f, 1.0e-3f, 2.0e-5f, -3e-4f;
    generator.SetDistortion(dist);
    generator.SetNoiseInPixels(0.5);
  }

 protected:
  Matrix3 K;
  DynamicVector dist;
  DataGenerator generator;
  Calibrator calibrator;
};

TEST_CASE_METHOD(DataGeneratorFixture, "data is generated", "[generator]") {
  int num = 100;
  const auto p = generator.GetDistortedPoints(num);
  REQUIRE(p.image.size() == p.world.size());
  REQUIRE(p.image.size() == num);
}

TEST_CASE_METHOD(DataGeneratorFixture, "planar data is generated",
                 "[generator]") {
  int num = 100;
  const auto p = generator.GetDistortedPointsPlanar(num);
  REQUIRE(p.image.size() == p.world.size());
  REQUIRE(p.image.size() == num);
}

TEST_CASE_METHOD(DataGeneratorFixture, "opencv estimation works",
                 "[generator,calibrator]") {
  int num_p = 100;
  int num_cams = 5;

  std::vector<Points2D> img_points;
  std::vector<Points3D> world_points;

  for (int i = 0; i < num_cams; ++i) {
    const auto p = generator.GetDistortedPointsPlanar(num_p);
    img_points.push_back(std::move(p.image));
    REQUIRE(p.image.size() == p.world.size());
    world_points.push_back(std::move(p.world));
  }

  calibrator.Estimate(img_points, world_points);
  const auto new_K = calibrator.GetK();
  Matrix3 change = (new_K - K).array() / K.array();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      if (K(i, j) != 0.0f) {
        // Allow max 1% error
        REQUIRE(change(i, j) < 0.01);
      } else {
        REQUIRE(new_K(i, j) == 0);
      }
    }
  }

  std::cout << calibrator.GetDistortion() << std::endl;
}
