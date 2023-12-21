/** Simple library for writing CSV files containing only floating point numbers
 * and a header.
 *
 * First, set up the header, number of columns, and number of rows in a
 * `CsvData` struct. Call `csv_init()` on it, then add values with `csv_set()`.
 * When you're done, call `csv_save()` to save the file and `csv_free()` to
 * clean up.
 */

#ifndef CSV_H_
#define CSV_H_

/** Error codes returned by functions.
 */
typedef int CsvStatus;

/** Function ran successfully.
 */
#define CSV_SUCCESS (0)

/** Dimension error.
 */
#define CSV_DIM_ERROR (-1)

/** Input-output (file reading or writing) error.
 */
#define CSV_IO_ERROR (-2)

/** Memory allocation error.
 */
#define CSV_ALLOC_ERROR (-3)

/** Invalid argument error.
 */
#define CSV_ARG_ERROR (-4)

/** Array of numerical data to be written to CSV.
 */
typedef struct {
  char *header;    ///< Header string in CSV format. Must match `n_col`.
  int n_row;       ///< Number of rows.
  int n_col;       ///< Number of columns.
  double *buffer;  ///< Storage buffer for numerical data.
} CsvData;

/** Initialize CSV struct.
 *
 * Must be run before any other functions.
 *
 * Must call `csv_free` after the program is done to avoid a memory leak.
 *
 * @param[in] data CSV struct.
 *
 * @retval CSV_SUCCESS Successfully initialized CSV struct.
 * @retval CSV_DIM_ERROR `n_row` or `n_col` is invalid.
 * @retval CSV_ALLOC_ERROR Could not allocate memory for `buffer`.
 */
CsvStatus csv_init(CsvData *data);

/** Clean up CSV struct.
 *
 * Must run at the end of the program to avoid a memory leak.
 *
 * @param[in] data CSV struct.
 *
 * @retval CSV_SUCCESS Successfully freed CSV struct.
 */
CsvStatus csv_free(CsvData *data);

/** Get an entry in the CSV struct.
 *
 * @param[in] row Row to read.
 * @param[in] col Column to read.
 * @param[in] data CSV struct.
 * @param[out] out Result.
 *
 * @retval CSV_SUCCESS Successfully read value.
 * @retval CSV_DIM_ERROR Incorrect dimension in `data.
 * @retval CSV_ALLOC_ERROR Invalid `buffer` in `data`.
 * @retval CSV_ARG_ERROR Invalid `row` or `col`.
 */
CsvStatus csv_get(const int row, const int col, CsvData *data, double *out);

/** Set an entry in the CSV struct.
 *
 * @param[in] value Value to write.
 * @param[in] row Row to write.
 * @param[in] col Column to write.
 * @param[in] data CSV struct.
 *
 * @retval CSV_SUCCESS Successfully wrote value.
 * @retval CSV_DIM_ERROR Incorrect dimension in `data.
 * @retval CSV_ALLOC_ERROR Invalid `buffer` in `data`.
 * @retval CSV_ARG_ERROR Invalid `row` or `col`.
 */
CsvStatus csv_set(double value, const int row, const int col, CsvData *data);

/** Get the number of entries of a CSV struct.
 *
 * @param[in] data CSV struct.
 *
 * @return Number of entries if positive. If negative, see below.
 * @retval CSV_DIM_ERROR Incorrect dimension in `data.
 */
int csv_size(CsvData *data);

/** Save a CSV struct to a file.
 *
 * @param[in] path Path to CSV file, ending with ".csv".
 * @param[in] data CSV struct.
 *
 * @retval CSV_SUCCESS Successfully wrote file.
 * @retval CSV_DIM_ERROR Incorrect dimension in `data.
 * @retval CSV_ALLOC_ERROR Invalid `buffer` in `data`.
 * @retval CSV_IO_ERROR Error opening or closing file.
 */
CsvStatus csv_save(const char *path, CsvData *data);

#endif  // CSV_H_
