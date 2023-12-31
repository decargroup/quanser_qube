#include <stdint.h>

#include <catch2/catch_all.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

extern "C" {
#include "lag_buffer.h"
#include "lti_sys.h"
}

TEST_CASE("Lag buffer is initialized correctly", "[lag_buffer]") {
  double arr[10];
  LagBuff buff;
  lagbuff_init(&buff, arr, 10);
  // Check that capacity is correct
  REQUIRE(lagbuff_capacity(&buff) == 10);
  // Check internal values, not meant to be accessed by user.
  REQUIRE(buff.capacity == 10);
  REQUIRE(buff.tail == 0);
  for (uint32_t i = 0; i < 10; i++) {
    REQUIRE(buff.buffer[i] == 0);
  }
}

TEST_CASE("Lag buffer is written correctly", "[lag_buffer]") {
  double arr[10];
  LagBuff buff;
  lagbuff_init(&buff, arr, 10);
  // Write N values and check tail
  for (uint32_t i = 0; i < 10; i++) {
    REQUIRE(buff.tail == i);
    lagbuff_append(&buff, (double)(i + 1));
  }
  REQUIRE(buff.tail == 0);
  // Check contents of internal buffer
  double verif_arr_1[] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0};
  for (uint32_t i = 0; i < 10; i++) {
    REQUIRE(verif_arr_1[i] == buff.buffer[i]);
  }
  // Write 5 more values
  for (uint32_t i = 0; i < (10 / 2); i++) {
    REQUIRE(buff.tail == i);
    lagbuff_append(&buff, (double)(i * 2));
  }
  REQUIRE(buff.tail == (10 / 2));
  // Check contents of internal buffer
  double verif_arr_2[] = {0.0, 2.0, 4.0, 6.0, 8.0, 6.0, 7.0, 8.0, 9.0, 10.0};
  for (uint32_t i = 0; i < 10; i++) {
    REQUIRE(verif_arr_2[i] == buff.buffer[i]);
  }
}

TEST_CASE("Lag buffer is read correctly", "[lag_buffer]") {
  double arr[10];
  double test_arr[10];
  LagBuff buff;
  lagbuff_init(&buff, arr, 10);
  // Write N values and check tail
  for (uint32_t i = 0; i < 10; i++) {
    lagbuff_append(&buff, (double)(i + 1));
  }
  // Read all at once and one at a time. Check both.
  lagbuff_read_all(&buff, test_arr);
  double verif_arr_1[] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0};
  for (uint32_t i = 0; i < 10; i++) {
    REQUIRE(lagbuff_read(&buff, i) == verif_arr_1[i]);
    REQUIRE(test_arr[i] == verif_arr_1[i]);
  }
  // Write 5 more values
  for (uint32_t i = 0; i < (10 / 2); i++) {
    lagbuff_append(&buff, (double)(i * 2));
  }
  // Read all at once and one at a time. Check both.
  lagbuff_read_all(&buff, test_arr);
  double verif_arr_2[] = {6.0, 7.0, 8.0, 9.0, 10.0, 0.0, 2.0, 4.0, 6.0, 8.0};
  for (uint32_t i = 0; i < 10; i++) {
    REQUIRE(lagbuff_read(&buff, i) == verif_arr_2[i]);
    REQUIRE(test_arr[i] == verif_arr_2[i]);
  }
}

TEST_CASE("LTI system responds correctly compared to C calculation.",
          "[lti_sys]") {
  LtiSys sys;
#define N_IN (3)
#define N_OUT (2)
  double buff_in[N_IN];
  double buff_out[N_OUT];
  double coeff_in[] = {6.0, 4.0, 2.0};
  double coeff_out[] = {3.0, 1.0};
  ltisys_init(&sys, LTI_SYS_NO_OUTPUT_SAT, coeff_in, coeff_out, buff_in,
              buff_out, N_IN, N_OUT);
  double input[4] = {0.1, 0.2, 0.3, 0.4};
  double output = 0.0;
  double next_output[N_OUT];

  next_output[0] = input[0] * coeff_in[0];
  output = ltisys_output(&sys, input[0]);
  REQUIRE_THAT(output, Catch::Matchers::WithinAbs(next_output[0], 1e-6));

  next_output[1] = -(next_output[0] * coeff_out[0]) + (input[1] * coeff_in[0]) +
                   (input[0] * coeff_in[1]);
  output = ltisys_output(&sys, input[1]);
  REQUIRE_THAT(output, Catch::Matchers::WithinAbs(next_output[1], 1e-6));

  next_output[0] = -(next_output[0] * coeff_out[1]) -
                   (next_output[1] * coeff_out[0]) + (input[2] * coeff_in[0]) +
                   (input[1] * coeff_in[1]) + (input[0] * coeff_in[2]);
  output = ltisys_output(&sys, input[2]);
  REQUIRE_THAT(output, Catch::Matchers::WithinAbs(next_output[0], 1e-6));

  next_output[1] = -(next_output[1] * coeff_out[1]) +
                   -(next_output[0] * coeff_out[0]) + (input[3] * coeff_in[0]) +
                   (input[2] * coeff_in[1]) + (input[1] * coeff_in[2]);
  output = ltisys_output(&sys, input[3]);
  REQUIRE_THAT(output, Catch::Matchers::WithinAbs(next_output[1], 1e-6));
}

TEST_CASE(
    "LTI system responds correctly compared to Python-generated test vectors",
    "[lti_sys]") {
  LtiSys sys;
#define N_IN (3)
#define N_OUT (2)
  double buff_in[N_IN] = {0};
  double buff_out[N_OUT] = {0};
  double coeff_in[] = {6.0, 4.0, 2.0};
  double coeff_out[] = {3.0, 1.0};
  ltisys_init(&sys, LTI_SYS_NO_OUTPUT_SAT, coeff_in, coeff_out, buff_in,
              buff_out, N_IN, N_OUT);

  /**
   * Test vectors used here are generated by `test_lag_buffer.py`
   */
  double input[] = {0.0,
                    0.1,
                    0.2,
                    0.30000000000000004,
                    0.4,
                    0.5,
                    0.6000000000000001,
                    0.7000000000000001,
                    0.8,
                    0.9};
  double output[] = {0.0,
                     0.6000000000000001,
                     -0.19999999999999973,
                     2.799999999999999,
                     -4.199999999999996,
                     14.999999999999984,
                     -34.39999999999995,
                     95.79999999999987,
                     -244.19999999999953,
                     646.7999999999987};

  for (int32_t i = 0; i < 10; i++) {
    double out_i = ltisys_output(&sys, input[i]);
    double exp_i = output[i];
    REQUIRE_THAT(out_i, Catch::Matchers::WithinAbs(exp_i, 1e-6));
  }

  for (int32_t i = 0; i < 10; i++) {
    double out_i = ltisys_output(&sys, input[i]);
    double exp_i = output[i];
    REQUIRE_THAT(out_i, !Catch::Matchers::WithinAbs(exp_i, 1e-6));
  }

  ltisys_reset(&sys);

  for (int32_t i = 0; i < 10; i++) {
    double out_i = ltisys_output(&sys, input[i]);
    double exp_i = output[i];
    REQUIRE_THAT(out_i, Catch::Matchers::WithinAbs(exp_i, 1e-6));
  }
}