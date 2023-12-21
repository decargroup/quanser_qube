#include <stdint.h>

#include <catch2/catch_all.hpp>

extern "C" {
#include "prbs.h"
}

TEST_CASE("test_prbs", "[prbs]") {
  bool expected_bits[] = {false, true,  false, false, false,
                          true,  false, false, true,  true};
  double expected[10];
  for (size_t i = 0; i < 10; i++) {
    expected[i] = expected_bits[i] ? 5.0 : 4.0;
  }
  double out[10];
  PrbsStatus status = prbs(4.0, 5.0, 1.0, 1.0, 0xACE1, 10, out);
  for (size_t i = 0; i < 10; i++) {
    REQUIRE(out[i] == expected[i]);
  }
  REQUIRE(status == PRBS_SUCCESS);
}

TEST_CASE("test_prbs_min_period", "[prbs]") {
  bool expected_bits[] = {false, true,  false, false, false,
                          true,  false, false, true,  true};
  double expected[20];
  for (size_t i = 0; i < 10; i++) {
    expected[2 * i] = expected_bits[i] ? 5.0 : 4.0;
    expected[2 * i + 1] = expected_bits[i] ? 5.0 : 4.0;
  }
  double out[20];
  PrbsStatus status = prbs(4.0, 5.0, 2.0, 1.0, 0xACE1, 20, out);
  for (size_t i = 0; i < 20; i++) {
    REQUIRE(out[i] == expected[i]);
  }
  REQUIRE(status == PRBS_SUCCESS);
}

TEST_CASE("test_prbs_bits", "[prbs]") {
  bool expected[] = {false, true,  false, false, false,
                     true,  false, false, true,  true};
  bool out[10];
  PrbsStatus status = prbs_bits(0xACE1, 10, out);
  for (size_t i = 0; i < 10; i++) {
    REQUIRE(out[i] == expected[i]);
  }
  REQUIRE(status == PRBS_SUCCESS);
}

TEST_CASE("test_prbs_repreat", "[prbs]") {
  bool out[150000];
  PrbsStatus status = prbs_bits(0xACE1, 150000, out);
  REQUIRE(status == PRBS_WARNING_REPEAT);
}
