#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include "calibrator.hh"
#include "data_generator.hh"

using namespace calibrator;

class DataGeneratorFixture {
 public:
  DataGeneratorFixture() : generator(1600, 1000), calibrator(1600, 1000) {
    Matrix3 K;
    constexpr float f = 1000.0f;
    K << f, 0, generator.GetWidth() / 2.0f, 0, f, generator.GetHeight() / 2.0f,
        0, 0, 1;
    generator.SetK(K);

    DynamicVector dist(5);
    dist << -4.0e-2f, 5e-4f, 1.0e-3f, 2.0e-5f, -3e-4f;
    generator.SetDistortion(dist);
    generator.SetNoiseInPixels(2);
  }

 protected:
  DataGenerator generator;
  Calibrator calibrator;
};

TEST_CASE_METHOD(DataGeneratorFixture, "data is generated", "[generator]") {
  int num = 100;
  const auto p = generator.GetDistortedPoints(num);
  REQUIRE(p.image.size() == p.world.size());
  REQUIRE(p.image.size() == num);
}

TEST_CASE_METHOD(DataGeneratorFixture, "opencv estimation works",
                 "[generator,calibrator]") {
  int num_p = 100;
  int num_cams = 3;

  std::vector<Points2D> img_points;
  std::vector<Points3D> world_points;

  for (int i = 0; i < num_cams; ++i) {
    const auto p = generator.GetDistortedPoints(num_p);
    img_points.push_back(std::move(p.image));
    REQUIRE(p.image.size() == p.world.size());
    world_points.push_back(std::move(p.world));
  }

  calibrator.Estimate(img_points, world_points);
}
