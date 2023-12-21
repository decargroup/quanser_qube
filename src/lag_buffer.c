#include "lag_buffer.h"

void lagbuff_init(LagBuff* lag_buff, volatile double* buffer,
                  uint32_t capacity) {
  lag_buff->buffer = buffer;
  lag_buff->capacity = capacity;
  lagbuff_clear(lag_buff);
}

void lagbuff_clear(LagBuff* lag_buff) {
  lag_buff->tail = 0;
  for (uint32_t i = 0; i < lag_buff->capacity; i++) {
    *(lag_buff->buffer + i) = 0.0;
  }
}

uint32_t lagbuff_capacity(const LagBuff* lag_buff) {
  return lag_buff->capacity;
}

void lagbuff_append(LagBuff* lag_buff, double value) {
  // Write new value
  *(lag_buff->buffer + lag_buff->tail) = value;
  // Advance tail index
  uint32_t newTail = lag_buff->tail + 1;
  if (newTail >= lag_buff->capacity) {
    lag_buff->tail = 0;
  } else {
    lag_buff->tail = newTail;
  }
}

double lagbuff_read(const LagBuff* lag_buff, uint32_t index) {
  // Calculate index to read
  // TODO Get rid of modulo? Probably not worth it here.
  uint32_t readIndex = (lag_buff->tail + index) % lag_buff->capacity;
  // Return value at that index
  return *(lag_buff->buffer + readIndex);
}

void lagbuff_read_all(const LagBuff* lag_buff, double* buffer) {
  uint32_t readIndex = lag_buff->tail;
  for (uint32_t i = 0; i < lag_buff->capacity; i++) {
    // Copy value at that index
    *(buffer + i) = *(lag_buff->buffer + readIndex);
    // Calculate index to read
    readIndex++;
    if (readIndex >= lag_buff->capacity) {
      readIndex = 0;
    }
  }
}

double lagbuff_mac(const LagBuff* lag_buff, const double* coeff) {
  double output = 0.0;
  uint32_t readIndex = lag_buff->tail;
  for (uint32_t i = 0; i < lag_buff->capacity; i++) {
    // Copy value at that index
    output +=
        *(coeff + lag_buff->capacity - i - 1) * *(lag_buff->buffer + readIndex);
    readIndex++;
    if (readIndex >= lag_buff->capacity) {
      readIndex = 0;
    }
  }
  return output;
}
