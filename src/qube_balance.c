/** Main executable to run the Quanser QUBE-Servo rotary inverted pendulum.
 * 
 * Once started, the system will wait until the pendulum is raised upright
 * by the user. Then the controller will run for a specified amount of time. If
 * any safety limits are hit, the controller will stop. The executable will save
 * a CSV file with the results in the directory from which it was run.
 * 
 * If the executable is called with no arguments, random seeds are generated
 * for the three pseudorandom binary sequences. These seeds are printed to the
 * console and saved in the CSV file name. To reproduce a run, the seeds can
 * be provided as command line arguments to the executable.
 * 
 * The run can be stopped at any time by hitting CTRL-C.
*/

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <malloc.h>
#include <math.h>
#include <string.h>
#include <time.h>

#include "csv.h"
#include "hil.h"
#include "lti_sys.h"
#include "prbs.h"
#include "quanser_messages.h"
#include "quanser_signal.h"

/** Number of samples to collect.
 */
#define N_SAMPLES (10000)

/** Number of samples to zero out in inputs.
 */
#define N_PAD (2000)

/** Encoder counts per revolution.
 */
#define COUNT_PER_REV (2048)

/** Encoder sensitivity for all four encoders, before gearboxes (rad/count).
 */
#define K_ENC (2.0 * M_PI / COUNT_PER_REV)

/** Maximum number of encoder 0 counts before shutting off.
 */
#define ENC0_MAX_RAD (M_PI_2)

/** Maximum number of encoder 1 counts before shutting off.
 */
#define ENC1_MAX_RAD (M_PI_4)

/** Maximum analog output voltage (V).
 */
#define ANALOG_MAX_V (10)

/** Start controller when pendulum is in this range.
 */
#define CONT_START_RAD (2 * M_PI * 5.0 / 360.0)

/** Control frequency (Hz).
 */
#define FREQUENCY (500.0)

/** Proportional gain for motor angle.
 */
#define KP_THETA (6.0)

/** Integral gain for motor angle.
 */
#define KI_THETA (0.0)

/** Derivative gain for motor angle.
 */
#define KD_THETA (1.8)

/** Derivative time constant for motor angle.
 */
#define TAU_THETA (50.0)

/** Proportional gain for pendulum angle.
 */
#define KP_ALPHA (30.0)

/** Integral gain for pendulum angle.
 */
#define KI_ALPHA (0.0)

/** Derivative gain for pendulum angle.
 */
#define KD_ALPHA (2.5)

/** Derivative time constant for pendulum angle.
 */
#define TAU_ALPHA (50.0)

/** If true, stagger inputs.
 */
#define STAGGER (false)

/** Maximum length of a Quanser message.
 */
#define MESSAGE_LEN (512)

/** Qube finite state machine states.
 */
typedef enum {
  QUBE_OPEN_BOARD,
  QUBE_SAVE_OFFSET,
  QUBE_CREATE_TASK,
  QUBE_START_TASK,
  QUBE_IDLE,
  QUBE_RUN,
  QUBE_STOP_TASK,
  QUBE_DELETE_TASK,
  QUBE_CLOSE_BOARD,
  QUBE_SAVE,
  QUBE_STOP,
} QubeState;

/** Becomes true if SIGINT is captured.
 */
static bool sigint = false;

/** Signal handler for SIGINT.
 */
void signal_handler(int signal) { sigint = true; }

/** Print error message.
 *
 * @param[in] error Quanser error code.
 */
void print_error(t_error result);

/** Convert encoder counts to radians and apply offset.
 *
 * Also wraps pendulum angle (i.e., `rad[1]`).
 *
 * @param[in]  count Array of two encoder counts.
 * @param[in] offset Array of two encoder offsets (initial positions).
 * @param[out]   rad Array of two encoder position in radians.
 */
void count_to_rad(t_int32 count[2], t_int32 offset[2], t_double rad[2]);

/** Wrap angle to [-pi, pi).
 *
 * @param[in] angle Angle in radians.
 *
 * @return Angle wrapped in range.
 */
t_double wrap_angle(t_double angle);

/** Clamp voltage within a range.
 *
 * @param[in] voltage Array containing voltage to clamp.
 * @param[in]   limit Positive saturation limit for voltage.
 * @param[out]    out Clamped voltage.
 *
 * @retval 0.0  Not clamping voltage.
 * @retval 1.0  Clamping voltage to maximum value.
 * @retval -1.0 Clamping voltage to minimum value
 */
t_double clamp(t_double voltage[1], t_double limit, t_double out[1]);

/** Generate a smoothed pseudorandom binary sequence with zero-padding.
 *
 * `n_pad` can lead to `min_period` being violated once at the beginning of the
 * sequence.
 *
 * @param[in]     cutoff Cutoff frequency (Hz).
 * @param[in]      n_pad Number of timesteps to replace with zeros.
 * @param[in]  min_value Minimum waveform value.
 * @param[in]  max_value Maximum waveform value.
 * @param[in] min_period Minimum high or low period (s)
 * @param[in]     f_samp Sampling frequency (Hz).
 * @param[in]       seed Random seed.
 * @param[in]        len Length of sequence to generate.
 * @param[out]       out Array to store sequence.
 *
 * @retval        PRBS_SUCCESS Successfully generated sequence.
 * @retval PRBS_WARNING_REPEAT Successfully generated sequence, but it repeats.
 */
PrbsStatus smooth_prbs(double cutoff, size_t n_pad, double min_value,
                       double max_value, double min_period, double f_samp,
                       uint16_t seed, size_t len, double *out);

/** Generate a chirp signal.
 *
 * @param[in]     n_pad Number of timesteps of zeros to prepend.
 * @param[in] amplitude Chirp amplitude.
 * @param[in]  min_freq Starting chirp frequency (Hz).
 * @param[in]  max_freq Ending chirp frequency (Hz).
 * @param[in]    f_samp Sampling frequency (Hz).
 * @param[in]       len Length of sequence to generate.
 * @param[out]      out Array to store sequence.
 */
void chirp(size_t n_pad, double amplitude, double min_freq, double max_freq,
           double f_samp, size_t len, double *out);

/** Compute the running integral of a signal.
 *
 * @param[in]     in Signal to integrate
 * @param[in] f_samp Sampling frequency (Hz).
 * @param[in]    len Length of sequence to generate.
 * @param[out]   out Array to store sequence.
 */
void integrate(double *in, double f_samp, size_t len, double *out);

int main(int argc, char *argv[]) {
  t_int16 seed_t;
  t_int16 seed_a;
  t_int16 seed_f;
  if (argc != 4) {
    t_int16 lower = 1;
    t_int16 upper = 999;
    srand(time(NULL));
    seed_t = lower + rand() % (upper - lower + 1);
    seed_a = lower + rand() % (upper - lower + 1);
    seed_f = lower + rand() % (upper - lower + 1);
  } else {
    seed_t = atoi(argv[1]);
    seed_a = atoi(argv[2]);
    seed_f = atoi(argv[3]);
  }
  printf("theta seed: %d; alpha seed: %d; ff seed: %d.\n", seed_t, seed_a,
         seed_f);
  // Catch Ctrl+C so application may be shut down cleanly
  qsigaction_t action;
  action.sa_handler = signal_handler;
  action.sa_flags = 0;
  qsigemptyset(&action.sa_mask);
  qsigaction(SIGINT, &action, NULL);
  // Board parameters
  const char board_type[] = "qube_servo_usb";
  const char board_identifier[] = "0";
  // Board handle
  t_card board;
  // Task handle
  t_task task;
  // Current sample
  t_int32 k = 0;
  // Timestep
  t_double timestep = 1.0 / FREQUENCY;
  // Exogenous inputs
  t_double target_theta_d[N_SAMPLES] = {0};
  t_double target_theta[N_SAMPLES] = {0};
  t_double target_alpha[N_SAMPLES] = {0};
  t_double feedforward[N_SAMPLES] = {0};
  if (STAGGER) {
    // Target motor angle
    smooth_prbs(200.0, 1000, -0.5, 0.5, 0.5, FREQUENCY, seed_t, 3000,
                target_theta_d);
    integrate(target_theta_d, FREQUENCY, N_SAMPLES, target_theta);
    // Target pendulum angle
    smooth_prbs(200.0, 4000, -0.01, 0.01, 0.1, FREQUENCY, seed_a, 6000,
                target_alpha);
    // Feedforward
    smooth_prbs(200.0, 7000, -1.0, 1.0, 0.1, FREQUENCY, seed_f, 9000,
                feedforward);

  } else {
    // Target motor angle
    smooth_prbs(200.0, N_PAD, -0.25, 0.25, 0.25, FREQUENCY, seed_t,
                (N_SAMPLES - N_PAD), target_theta_d);
    integrate(target_theta_d, FREQUENCY, N_SAMPLES, target_theta);
    // Target pendulum angle
    smooth_prbs(200.0, N_PAD, -0.01, 0.01, 0.1, FREQUENCY, seed_a,
                (N_SAMPLES - N_PAD), target_alpha);
    // Feedforward
    smooth_prbs(200.0, N_PAD, -0.5, 0.5, 0.1, FREQUENCY, seed_f,
                (N_SAMPLES - N_PAD), feedforward);
  }
  // Analog output channels in use
  const t_uint32 analog_channels[] = {0};
  // Encoder channels in use
  const t_uint32 encoder_channels[] = {0, 1};
  // Analog values to output (V)
  t_double analog_outputs[ARRAY_LENGTH(analog_channels)] = {0};
  // Control output without saturation (V)
  t_double analog_outputs_nosat[ARRAY_LENGTH(analog_channels)] = {0};
  // Encoder counts
  t_int32 encoder_counts[ARRAY_LENGTH(encoder_channels)] = {0};
  // Encoder offsets (counts)
  t_int32 encoder_offsets[ARRAY_LENGTH(encoder_channels)] = {0};
  // Encoder counts converted to radians (with offset)
  t_double encoder_radians[ARRAY_LENGTH(encoder_channels)] = {0};
  // Encoder error (rad)
  t_double encoder_error[ARRAY_LENGTH(encoder_channels)] = {0};
  // Filtered encoder error derivatives (rad/s)
  t_double encoder_error_dot[ARRAY_LENGTH(encoder_channels)] = {0};
  // Encoder error integrals (rad.s)
  t_double encoder_error_int[ARRAY_LENGTH(encoder_channels)] = {0};
  // Set up CSV
  CsvData csv_data;
  csv_data.header =
      "t, target_theta, target_alpha, theta, alpha, control_output, "
      "feedforward, plant_input, saturation";
  csv_data.n_col = 9;
  csv_data.n_row = N_SAMPLES;
  csv_init(&csv_data);
  // Integral filter for motor angle
  LtiSys integ_theta;
  t_double num_integ_theta[] = {1.0 / FREQUENCY, 0};
  t_double den_integ_theta[] = {-1.0};
  t_double num_buff_integ_theta[2] = {0};
  t_double den_buff_integ_theta[1] = {0};
  ltisys_init(&integ_theta, LTI_SYS_NO_OUTPUT_SAT, num_integ_theta,
              den_integ_theta, num_buff_integ_theta, den_buff_integ_theta, 2,
              1);
  // Integral filter for pendulum angle
  LtiSys integ_alpha;
  t_double num_integ_alpha[] = {1.0 / FREQUENCY, 0};
  t_double den_integ_alpha[] = {-1.0};
  t_double num_buff_integ_alpha[2] = {0};
  t_double den_buff_integ_alpha[1] = {0};
  ltisys_init(&integ_alpha, LTI_SYS_NO_OUTPUT_SAT, num_integ_alpha,
              den_integ_alpha, num_buff_integ_alpha, den_buff_integ_alpha, 2,
              1);
  // Derivative filter for motor angle
  LtiSys deriv_theta;
  t_double num_deriv_theta[] = {TAU_THETA / (1 + TAU_THETA / FREQUENCY),
                                -TAU_THETA / (1 + TAU_THETA / FREQUENCY)};
  t_double den_deriv_theta[] = {-1.0 / (1 + TAU_THETA / FREQUENCY)};
  t_double num_buff_deriv_theta[2] = {0};
  t_double den_buff_deriv_theta[1] = {0};
  ltisys_init(&deriv_theta, LTI_SYS_NO_OUTPUT_SAT, num_deriv_theta,
              den_deriv_theta, num_buff_deriv_theta, den_buff_deriv_theta, 2,
              1);
  // Derivative filter for pendulum angle
  LtiSys deriv_alpha;
  t_double num_deriv_alpha[] = {TAU_ALPHA / (1 + TAU_ALPHA / FREQUENCY),
                                -TAU_ALPHA / (1 + TAU_ALPHA / FREQUENCY)};
  t_double den_deriv_alpha[] = {-1.0 / (1 + TAU_ALPHA / FREQUENCY)};
  t_double num_buff_deriv_alpha[2] = {0};
  t_double den_buff_deriv_alpha[1] = {0};
  ltisys_init(&deriv_alpha, LTI_SYS_NO_OUTPUT_SAT, num_deriv_alpha,
              den_deriv_alpha, num_buff_deriv_alpha, den_buff_deriv_alpha, 2,
              1);
  // Current Qube FSM state
  QubeState state = QUBE_OPEN_BOARD;
  // Run finite state machine
  while (true) {
    switch (state) {
      case QUBE_OPEN_BOARD: {
        // Open board
        t_error result = hil_open(board_type, board_identifier, &board);
        // Check result
        if (result == 0) {
          printf("Press CTRL-C to stop program.\n");
          state = QUBE_SAVE_OFFSET;
        } else {
          print_error(result);
          state = QUBE_CLOSE_BOARD;
        }
        break;
      }
      case QUBE_SAVE_OFFSET: {
        // Read encoder to get offsets
        t_error result =
            hil_read_encoder(board, encoder_channels,
                             ARRAY_LENGTH(encoder_channels), encoder_offsets);
        // Shift pendulum zero to upright position
        encoder_offsets[1] += (COUNT_PER_REV / 2);
        // Check result
        if (result == 0) {
          state = QUBE_CREATE_TASK;
        } else {
          print_error(result);
          state = QUBE_CLOSE_BOARD;
        }
        break;
      }
      case QUBE_CREATE_TASK: {
        // Create encoder reader task
        t_error result = hil_task_create_encoder_reader(
            board, (t_uint32)FREQUENCY, encoder_channels,
            ARRAY_LENGTH(encoder_channels), &task);
        // Check result
        if (result == 0) {
          state = QUBE_START_TASK;
        } else {
          print_error(result);
          state = QUBE_DELETE_TASK;
        }
        break;
      }
      case QUBE_START_TASK: {
        // Start encoder reader task
        t_error result = hil_task_start(task, HARDWARE_CLOCK_0, FREQUENCY, -1);
        // Check result
        if (result == 0) {
          state = QUBE_IDLE;
        } else {
          print_error(result);
          state = QUBE_STOP_TASK;
        }
        break;
      }
      case QUBE_IDLE: {
        // Check for interrrupt
        if (sigint) {
          state = QUBE_STOP_TASK;
          break;
        }
        // Read encoder
        t_error result = hil_task_read_encoder(task, 1, encoder_counts);
        if (result < 0) {
          print_error(result);
          state = QUBE_STOP_TASK;
          break;
        }
        // Write zero to output
        analog_outputs[0] = 0.0;
        result =
            hil_write_analog(board, analog_channels,
                             ARRAY_LENGTH(analog_channels), analog_outputs);
        if (result != 0) {
          print_error(result);
          state = QUBE_STOP_TASK;
          break;
        }
        // Convert encoder counts
        count_to_rad(encoder_counts, encoder_offsets, encoder_radians);
        // Check starting condition
        if ((0.0 - encoder_radians[1]) < CONT_START_RAD &&
            (0.0 - encoder_radians[1]) > -1 * CONT_START_RAD) {
          state = QUBE_RUN;
        }
        break;
      }
      case QUBE_RUN: {
        // Check for interrrupt
        if (sigint) {
          state = QUBE_STOP_TASK;
          break;
        }
        // Read encoder
        t_error result = hil_task_read_encoder(task, 1, encoder_counts);
        if (result < 0) {
          print_error(result);
          state = QUBE_STOP_TASK;
          break;
        }
        // Convert encoder counts
        count_to_rad(encoder_counts, encoder_offsets, encoder_radians);
        // Apply derivative filters
        encoder_error[0] = target_theta[k] - encoder_radians[0];
        encoder_error[1] = target_alpha[k] - encoder_radians[1];
        encoder_error_dot[0] = ltisys_output(&deriv_theta, encoder_error[0]);
        encoder_error_dot[1] = ltisys_output(&deriv_alpha, encoder_error[1]);
        encoder_error_int[0] = ltisys_output(&integ_theta, encoder_error[0]);
        encoder_error_int[1] = ltisys_output(&integ_alpha, encoder_error[1]);
        // Check safety limits
        if ((encoder_radians[0] > ENC0_MAX_RAD) ||
            (encoder_radians[0] < -1 * ENC0_MAX_RAD) ||
            (encoder_radians[1] > ENC1_MAX_RAD) ||
            (encoder_radians[1] < -1 * ENC1_MAX_RAD)) {
          state = QUBE_STOP_TASK;
          break;
        }
        t_double control_output =
            -KP_THETA * encoder_error[0] - KD_THETA * encoder_error_dot[0] -
            KI_THETA * encoder_error_int[0] - KP_ALPHA * encoder_error[1] -
            KD_ALPHA * encoder_error_dot[1] - KI_ALPHA * encoder_error_int[1];
        // Calculate the output
        analog_outputs_nosat[0] = control_output + feedforward[k];
        t_double saturation =
            clamp(analog_outputs_nosat, ANALOG_MAX_V, analog_outputs);
        // Write output
        result =
            hil_write_analog(board, analog_channels,
                             ARRAY_LENGTH(analog_channels), analog_outputs);
        if (result != 0) {
          print_error(result);
          state = QUBE_STOP_TASK;
          break;
        }
        csv_set((double)(timestep * k), k, 0, &csv_data);
        csv_set((double)target_theta[k], k, 1, &csv_data);
        csv_set((double)target_alpha[k], k, 2, &csv_data);
        csv_set((double)encoder_radians[0], k, 3, &csv_data);
        csv_set((double)encoder_radians[1], k, 4, &csv_data);
        csv_set((double)control_output, k, 5, &csv_data);
        csv_set((double)feedforward[k], k, 6, &csv_data);
        csv_set((double)analog_outputs[0], k, 7, &csv_data);
        csv_set((double)saturation, k, 8, &csv_data);
        if (k >= N_SAMPLES) {
          state = QUBE_STOP_TASK;
        }
        k++;
        break;
      }
      case QUBE_STOP_TASK: {
        // Stop the task
        t_error result = hil_task_stop(task);
        // Check it it was successful
        if (result != 0) {
          print_error(result);
        }
        // Write zero to analog outputs
        analog_outputs[0] = 0.0;
        result =
            hil_write_analog(board, analog_channels,
                             ARRAY_LENGTH(analog_channels), analog_outputs);
        // Check it it was successful
        if (result != 0) {
          print_error(result);
        }
        state = QUBE_DELETE_TASK;
        break;
      }
      case QUBE_DELETE_TASK: {
        // Delete the task
        t_error result = hil_task_delete(task);
        // Check it it was successful
        if (result != 0) {
          print_error(result);
        }
        state = QUBE_CLOSE_BOARD;
        break;
      }
      case QUBE_CLOSE_BOARD: {
        // Close the board
        t_error result = hil_close(board);
        // Check it it was successful
        if (result != 0) {
          print_error(result);
        }
        state = QUBE_SAVE;
        break;
      }
      case QUBE_SAVE: {
        // Get timestamp
        time_t timer;
        char prefix_buffer[MESSAGE_LEN];
        char suffix_buffer[MESSAGE_LEN];
        struct tm *tm_info;
        timer = time(NULL);
        tm_info = localtime(&timer);
        strftime(prefix_buffer, sizeof(prefix_buffer), "qube_%Y%m%dT%H%M%S_",
                 tm_info);
        snprintf(suffix_buffer, sizeof(suffix_buffer), "%03d_%03d_%03d.csv",
                 seed_t, seed_a, seed_f);
        strcat(prefix_buffer, suffix_buffer);
        // Save CSV
        csv_save(prefix_buffer, &csv_data);
        csv_free(&csv_data);
        state = QUBE_STOP;
        break;
      }
      case QUBE_STOP: {
        // Make sure all boards are closed
        t_error result = hil_close_all();
        // Check it it was successful
        if (result != 0) {
          print_error(result);
        }
        return 0;
      }
    }
  }
}

void print_error(t_error result) {
  char message[MESSAGE_LEN];
  msg_get_error_message(NULL, result, message, ARRAY_LENGTH(message));
  printf("ERROR %d: %s\n", -result, message);
}

void count_to_rad(t_int32 count[2], t_int32 offset[2], t_double rad[2]) {
  // Only need to wrap pendulum angle
  rad[0] = (count[0] - offset[0]) * K_ENC;
  rad[1] = wrap_angle((count[1] - offset[1]) * K_ENC);
}

t_double wrap_angle(t_double angle) {
  angle = fmod(angle + M_PI, 2.0 * M_PI);
  if (angle < 0) {
    angle += 2.0 * M_PI;
  }
  return angle - M_PI;
}

t_double clamp(t_double voltage[1], t_double limit, t_double out[1]) {
  // Make sure limit is positive
  if (limit < 0) {
    out[0] = 0.0;
    return 1.0;
  }
  // Saturate
  t_double saturated = 0.0;
  if (voltage[0] > limit) {
    out[0] = limit;
    saturated = 1.0;
  } else if (voltage[0] < -limit) {
    out[0] = -limit;
    saturated = -1.0;
  } else {
    out[0] = voltage[0];
    saturated = 0.0;
  }
  return saturated;
}

PrbsStatus smooth_prbs(double cutoff, size_t n_pad, double min_value,
                       double max_value, double min_period, double f_samp,
                       uint16_t seed, size_t len, double *out) {
  // Generate PRBS
  t_double *raw_prbs = (t_double *)calloc(len, sizeof(t_double));
  PrbsStatus status =
      prbs(min_value, max_value, min_period, f_samp, seed, len, raw_prbs);
  for (int i = 0; i < n_pad; i++) {
    raw_prbs[i] = 0.0;
  }
  if (cutoff <= 0.0) {
    // Copy
    for (int i = 0; i < len; i++) {
      out[i] = raw_prbs[i];
    }
  } else {
    // Create filter
    LtiSys filt;
    t_double num_filt[] = {(cutoff / f_samp) / (1.0 + cutoff / f_samp), 0};
    t_double den_filt[] = {-1.0 / (1.0 + cutoff / f_samp)};
    t_double num_buff_filt[2] = {0};
    t_double den_buff_filt[1] = {0};
    ltisys_init(&filt, LTI_SYS_NO_OUTPUT_SAT, num_filt, den_filt, num_buff_filt,
                den_buff_filt, 2, 1);
    // Run filter
    for (int i = 0; i < len; i++) {
      out[i] = ltisys_output(&filt, raw_prbs[i]);
    }
  }
  // Clean up
  free(raw_prbs);
  return status;
}

void chirp(size_t n_pad, double amplitude, double min_freq, double max_freq,
           double f_samp, size_t len, double *out) {
  // Chirp rate
  double c = (max_freq - min_freq) / ((len - n_pad) / f_samp);
  // Fill in output array
  for (int i = 0; i < len; i++) {
    if (i < n_pad) {
      out[i] = 0.0;
    } else {
      double phi = 2.0 * M_PI * c * ((i - n_pad) / f_samp) + min_freq;
      out[i] = amplitude * sin(phi * ((i - n_pad) / f_samp));
    }
  }
}

void integrate(double *in, double f_samp, size_t len, double *out) {
  double t_step = 1 / f_samp;
  out[0] = 0;
  for (int i = 1; i < len; i++) {
    out[i] = out[i - 1] + t_step * in[i];
  }
}