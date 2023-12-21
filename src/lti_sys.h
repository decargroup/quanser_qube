/** Discrete-time linear time-invariant system implementation.
 *
 * Coefficients determine the transfer function, which is
 *
 *     y[z]            a_m*z^m + ... + a_2*z^2 + a_1*z + a_0
 *    ------  =  -------------------------------------------------
 *     x[z]       z^n + b_n-1*z^n-1 + ... + b_2*z^2 + b_1*z + b_0
 *
 * where
 *     a_m   = num_coeff[0],
 *     a_0   = num_coeff[end],
 *     b_n-1 = den_coeff[0],
 *     b_0   = den_coeff[end],
 *
 * and n >= m (since the system is causal).
 *
 * Note that z^n has no coefficient in front of it (i.e. there is no b_n).
 *
 * The corresponding difference equation is
 *
 *     y[k] = a_m*x[k-n+m] + ... + a_2*x[k-n+2] + a_1*x[k-n+1] + a_0*x[k-n]
 *            - b_n-1*y[k-1] - ... - b_2*y[k-n+2] - b_1*y[k-n+1] - b_0*y[k-n].
 *
 * Optional output saturation is also implemented. If output_sat is positive,
 * the output y[k] is restricted to the range [-output_sat, output_sat]. If
 * output_sat is negative or zero, the output is unrestricted.
 *
 * To use, set up the buffers and coefficients in the `LtiSys` struct, then
 * call `ltisys_init()` on it. You can then just repeatedly call
 * `ltisys_output()` to get outputs as new inputs become available.
 */

#ifndef LTI_SYS_H_
#define LTI_SYS_H_

#include <stdint.h>

#include "lag_buffer.h"

/** Set output_sat to this value to disable output saturation.
 *
 * Any negative or zero floating point value will disable saturation. This
 * macro will just make it clear that you intend to disable saturation.
 */
#define LTI_SYS_NO_OUTPUT_SAT (-1.0f)

#define LTI_SYS_NUM_SIZE_PI (2)
#define LTI_SYS_DEN_SIZE_PI (1)

/** LTI system struct.
 */
typedef struct {
  double output_sat;     ///< Output saturation value. Ignored if negative or 0.
  double* num_coeff;     ///< Coefficients in transfer function numerator.
  double* den_coeff;     ///< Coefficients in transfer function denominator,
                         ///< excluding the highest order term.
  LagBuff num_lag_buff;  ///< LagBuff containing past inputs.
  LagBuff den_lag_buff;  ///< LagBuff containing past outputs.
} LtiSys;

/** Initialize an LtiSys struct.
 *
 * @param[in, out]    lti_sys Pointer to uninitialized LtiSys struct.
 * @param[in]      output_sat Output saturation value. If positive, output is
 *                            restricted to [-output_sat, output_sat]. If
 *                            negative or zero, output is unrestricted.
 * @param[in]       num_coeff Pointer to array of numerator coefficients,
 *                            highest order to lowest.
 * @param[in]       den_coeff Pointer to array of denominator coefficients,
 *                            highest order to lowest.
 * @param[in, out] num_buffer Pointer to array to store input history (will
 *                            multiply numerator coefficients).
 * @param[in, out] den_buffer Pointer to array to store output history (will
 *                            multiply denominator coefficients).
 * @param[in]     n_num_coeff Size of num_coeff.
 * @param[in]     n_den_coeff Size of den_coeff.
 *
 */
void ltisys_init(LtiSys* lti_sys, double output_sat, double* num_coeff,
                 double* den_coeff, volatile double* num_buffer,
                 volatile double* den_buffer, uint32_t n_num_coeff,
                 uint32_t n_den_coeff);

/** Reset the input and output histories in an LtiSys struct. Sets their values
 * to zero.
 *
 * @param[in, out] lti_sys Pointer to LtiSys struct.
 */
void ltisys_reset(LtiSys* lti_sys);

/** Apply an input to an LtiSys and get the corresponding output.
 *
 * Adds the input to the input history, computes the output, adds that output to
 * the output history and returns it.
 *
 * @param[in, out] lti_sys Pointer to LtiSys struct.
 * @param[in]        input Input to the system.
 *
 * @return Output of the system.
 */
double ltisys_output(LtiSys* lti_sys, double input);

#endif  // LTI_SYS_H_
