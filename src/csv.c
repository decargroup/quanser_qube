#include "csv.h"

#include <malloc.h>
#include <stdio.h>

CsvStatus csv_init(CsvData *data) {
  if (data->n_row < 1) {
    return CSV_DIM_ERROR;
  }
  if (data->n_col < 1) {
    return CSV_DIM_ERROR;
  }
  data->buffer = (double *)calloc(data->n_row * data->n_col, sizeof(double));
  if (!data->buffer) {
    return CSV_ALLOC_ERROR;
  }
  return CSV_SUCCESS;
}

CsvStatus csv_free(CsvData *data) {
  free(data->buffer);
  return CSV_SUCCESS;
}

CsvStatus csv_get(const int row, const int col, CsvData *data, double *out) {
  if (data->n_row < 1) {
    return CSV_DIM_ERROR;
  }
  if (data->n_col < 1) {
    return CSV_DIM_ERROR;
  }
  if (!data->buffer) {
    return CSV_ALLOC_ERROR;
  }
  if (row < 0 || row >= data->n_row) {
    return CSV_ARG_ERROR;
  }
  if (col < 0 || col >= data->n_col) {
    return CSV_ARG_ERROR;
  }
  *out = data->buffer[row * data->n_col + col];
  return CSV_SUCCESS;
}

CsvStatus csv_set(double value, const int row, const int col, CsvData *data) {
  if (data->n_row < 1) {
    return CSV_DIM_ERROR;
  }
  if (data->n_col < 1) {
    return CSV_DIM_ERROR;
  }
  if (!data->buffer) {
    return CSV_ALLOC_ERROR;
  }
  if (row < 0 || row >= data->n_row) {
    return CSV_ARG_ERROR;
  }
  if (col < 0 || col >= data->n_col) {
    return CSV_ARG_ERROR;
  }
  data->buffer[row * data->n_col + col] = value;
  return CSV_SUCCESS;
}

int csv_size(CsvData *data) {
  if (data->n_row < 1) {
    return CSV_DIM_ERROR;
  }
  if (data->n_col < 1) {
    return CSV_DIM_ERROR;
  }
  return data->n_row * data->n_col;
}

CsvStatus csv_save(const char *path, CsvData *data) {
  if (data->n_row < 1) {
    return CSV_DIM_ERROR;
  }
  if (data->n_col < 1) {
    return CSV_DIM_ERROR;
  }
  if (!data->buffer) {
    return CSV_ALLOC_ERROR;
  }
  FILE *fpt;
  fpt = fopen(path, "w+");
  if (!fpt) {
    return CSV_IO_ERROR;
  }
  fprintf(fpt, "%s\n", data->header);
  for (int row = 0; row < data->n_row; row++) {
    for (int col = 0; col < data->n_col; col++) {
      fprintf(fpt, "%.17e", data->buffer[row * data->n_col + col]);
      if (col < data->n_col - 1) {
        fprintf(fpt, ", ");
      } else {
        fprintf(fpt, "\n");
      }
    }
  }
  if (fclose(fpt) != 0) {
    return CSV_IO_ERROR;
  }
  return CSV_SUCCESS;
}
