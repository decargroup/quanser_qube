/** Lightweight circular buffer that stores the last N values of a variable.
 *
 * Linear time-invariant filters and controllers take the form
 *
 *     y[k] = (a1 * y[k - 1] + a2 * y[k - 2] + ... + am * y[k - m])
 *          + (b0 * x[k] + b1 * x[k - 1] + b2 * x[k - 2] + ... + bn * x[k - n])
 *
 * where y[k] is the output and x[k] is the input. To implement them, we need to
 * store a fixed number of lags of y[k] and x[k]. The lag buffer implemented
 * here is used for that purpose. One lag buffer is needed for y[k] and another
 * is needed for x[k].
 *
 * No dynamic memory allocation is used here. The user must initialize their own
 * array of doubles and create a LagBuff struct using a pointer to that buffer.
 * From that point on, the LagBuff struct should only be accessed through the
 * provided function calls. It should never be accessed directly.
 */

#ifndef LAG_BUFFER_H_
#define LAG_BUFFER_H_

#include <stdint.h>

/** Lag buffer struct.
 *
 * Once initialized, it should only be accessed through the function calls
 * provided.
 */
typedef struct {
  uint32_t capacity;        ///< Number of floats the buffer can hold.
  volatile uint32_t tail;   ///< Index of oldest item in the buffer.
  volatile double* buffer;  ///< Pointer to array where floats are stored.
} LagBuff;

/** Initialize a lag buffer.
 *
 * @param[in, out] lag_buff Pointer to LagBuff to initialize.
 * @param[in, out]   buffer Pointer to buffer to use for data storage.
 * @param[in]      capacity Capacity of buffer.
 */
void lagbuff_init(LagBuff* lag_buff, volatile double* buffer,
                  uint32_t capacity);

/** Set lag buffer contents to zero and reset tail index.
 *
 * @param[in, out] lag_buff Pointer to LagBuff.
 */
void lagbuff_clear(LagBuff* lag_buff);

/** Get lag buffer capacity.
 *
 * @param[in] lag_buff Pointer to LagBuff.
 *
 * @return Capacity of lag buffer.
 */
uint32_t lagbuff_capacity(const LagBuff* lag_buff);

/** Append value to lag buffer.
 *
 * @param[in, out] lag_buff Pointer to LagBuff.
 * @param[in]         value Value to append.
 */
void lagbuff_append(LagBuff* lag_buff, double value);

/** Read value from lag buffer.
 *
 * Index 0 corresponds to oldest value. Index at capacity corresponds to latest
 * value.
 *
 * @param[in] lag_buff Pointer to LagBuff.
 * @param[in]    index Index of value to read.
 *
 * @return Value at index.
 */
double lagbuff_read(const LagBuff* lag_buff, uint32_t index);

/** Read all values from lag buffer.
 *
 * Index 0 corresponds to oldest value. Index at capacity corresponds to latest
 * value.
 *
 * @param[in] lag_buff Pointer to LagBuff.
 * @param[out]  values Pointer to array that will hold values.
 */
void lagbuff_read_all(const LagBuff* lag_buff, double* values);

/** Multiply-accumulate lag buffer contents with coefficients.
 *
 * @param[in] lag_buff Pointer to LagBuff.
 * @param[in]    coeff Pointer to coefficient array.
 *
 * @returns Sum of lag buffer contents multiplied elementwise with coefficients.
 */
double lagbuff_mac(const LagBuff* lag_buff, const double* coeff);

#endif  // LAG_BUFFER_H_
