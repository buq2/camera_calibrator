#define CATCH_CONFIG_MAIN
#include <catch2/catch_test_macros.hpp>
#include "circular_buffer.hh"

TEST_CASE("simple", "[circular_buffer]") {
  constexpr size_t cap = 5;
  CircularBuffer<int> cb(cap);
  REQUIRE(cb.size() == 0);
  REQUIRE(cb.capacity() == cap);
  REQUIRE(cb.empty());

  for (int i = 0; i < 5; ++i) {
    cb.push_back(i);
    REQUIRE(cb.size() == i + 1);
  }

  REQUIRE(cb.full());
  for (int i = 0; i < 5; ++i) {
    REQUIRE(cb[i] == i);
  }

  cb.push_back(5);
  REQUIRE(cb.full());
  REQUIRE(cb.size() == cap);
  for (int i = 0; i < 5; ++i) {
    REQUIRE(cb[i] == i + 1);
  }

  cb.push_back(6);
  REQUIRE(cb.full());
  REQUIRE(cb.size() == cap);
  for (int i = 0; i < 5; ++i) {
    REQUIRE(cb[i] == i + 2);
  }
}
