/** Simple pseudorandom binary sequence implementation using a linear-feedback
 * shift register.
 */

#ifndef PRBS_H_
#define PRBS_H_

/** Error codes returned by functions.
 */
typedef enum {
  PRBS_SUCCESS,         ///< Function ran successfully.
  PRBS_WARNING_REPEAT,  ///< PRBS sequence repeats for given length.
} PrbsStatus;

#include <stdbool.h>
#include <stdint.h>

/** Generate a pseudorandom binary sequence as an array of analog values.
 *
 * @param[in]  min_value Minimum waveform value.
 * @param[in]  max_value Maximum waveform value.
 * @param[in] min_period Minimum high or low period (s)
 * @param[in]     f_samp Sampling frequency (Hz).
 * @param[in]       seed Random seed.
 * @param[in]        len Length of sequence to generate.
 * @param[out]       out Array to store sequence.
 *
 * @retval        PRBS_SUCCESS Successfully generated sequence.
 * @retval PRBS_WARNING_REPEAT Successfully generated sequence, bit it repeats.
 */
PrbsStatus prbs(double min_value, double max_value, double min_period,
                double f_samp, uint16_t seed, size_t len, double* out);

/** Generate a pseudorandom binary sequence as a boolean array.
 *
 * Uses a 16-bit Fibonacci linear-feedback shift register with feedback
 * polynomial x^16 + x^14 + x^13 + x^11 + 1 (taps at 16, 14, 13, and 11). See
 * https://en.wikipedia.org/wiki/Linear-feedback_shift_register#Fibonacci_LFSRs
 *
 * @param[in] seed Initial state of shift register, acts as pseudorandom seed.
 * @param[in]  len Length of sequence to generate.
 * @param[out] out Array to store sequence.
 *
 * @retval        PRBS_SUCCESS Successfully generated sequence.
 * @retval PRBS_WARNING_REPEAT Successfully generated sequence, bit it repeats.
 */
PrbsStatus prbs_bits(uint16_t seed, size_t len, bool* out);

#endif  // PRBS_H_
