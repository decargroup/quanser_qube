#include "prbs.h"

#include <malloc.h>

PrbsStatus prbs(double min_value, double max_value, double min_period,
                double f_samp, uint16_t seed, size_t len, double* out) {
  PrbsStatus status = PRBS_SUCCESS;
  // Calculate how many PRBS bits are needed
  uint32_t prbs_scale = (uint32_t)(min_period * f_samp);
  size_t prbs_bits_len = len / prbs_scale;
  bool* prbs_bits_out = (bool*)calloc(prbs_bits_len, sizeof(bool));
  // Generate the PRBS as an array of booleans
  status = prbs_bits(seed, prbs_bits_len, prbs_bits_out);
  // Fill in the output array with doubles
  for (size_t i = 0; i < prbs_bits_len; i++) {
    for (size_t j = 0; j < prbs_scale; j++) {
      // Fill `prbs_scale` bits with the max or min analog value
      out[prbs_scale * i + j] = prbs_bits_out[i] ? max_value : min_value;
    }
  }
  // Free PRBS bits and return
  free(prbs_bits_out);
  return status;
}

PrbsStatus prbs_bits(uint16_t seed, size_t len, bool* out) {
  PrbsStatus status = PRBS_SUCCESS;
  uint16_t lfsr = seed;
  uint16_t bit;
  for (size_t i = 0; i < len; i++) {
    // Generate a new bit
    bit = ((lfsr >> 0) ^ (lfsr >> 2) ^ (lfsr >> 3) ^ (lfsr >> 5)) & 0x0001;
    // Shift new bit into register
    lfsr = (lfsr >> 1) | (bit << 15);
    // Generate output boolean
    out[i] = (bit == 0x0001);
    // Warn if sequence repeats
    if (lfsr == seed) {
      status = PRBS_WARNING_REPEAT;
    }
  }
  return status;
}
