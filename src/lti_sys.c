#include "lti_sys.h"

void ltisys_init(LtiSys* lti_sys, double output_sat, double* num_coeff,
                 double* den_coeff, volatile double* num_buffer,
                 volatile double* den_buffer, uint32_t n_num_coeff,
                 uint32_t n_den_coeff) {
  lti_sys->output_sat = output_sat;
  lti_sys->num_coeff = num_coeff;
  lti_sys->den_coeff = den_coeff;
  lagbuff_init(&(lti_sys->num_lag_buff), num_buffer, n_num_coeff);
  lagbuff_init(&(lti_sys->den_lag_buff), den_buffer, n_den_coeff);
}

void ltisys_reset(LtiSys* lti_sys) {
  lagbuff_clear(&(lti_sys->num_lag_buff));
  lagbuff_clear(&(lti_sys->den_lag_buff));
}

double ltisys_output(LtiSys* lti_sys, double in_input) {
  lagbuff_append(&(lti_sys->num_lag_buff), in_input);
  double output = lagbuff_mac(&(lti_sys->num_lag_buff), lti_sys->num_coeff) -
                  lagbuff_mac(&(lti_sys->den_lag_buff), lti_sys->den_coeff);
  // If saturation is enabled (output_sat is positive), saturate the output.
  // Otherwise, don't modify the output
  if (lti_sys->output_sat > 0.0f) {
    if (output > lti_sys->output_sat) {
      output = lti_sys->output_sat;
    } else if (output < -lti_sys->output_sat) {
      output = -lti_sys->output_sat;
    }
  }
  lagbuff_append(&(lti_sys->den_lag_buff), output);
  return output;
}
