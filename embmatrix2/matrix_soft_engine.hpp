#ifndef MATRIX_SOFT_ENGINE_HPP
#define MATRIX_SOFT_ENGINE_HPP

#include <cstdint>
#define _USE_MATH_DEFINES
#include <cmath>

namespace matrix {

/*
 ******************************************************************************
 * LOW LEVEL
 * Hi speed functions.
 * WARNING!!! no size checks
 ******************************************************************************
 */

/**
 * @brief   deep transpose matrix A(m x n) to B(n x m)
 */
template <typename T>
void matrix_soft_transpose(size_t m, size_t n, const T *A, T *B) {
  size_t i, j;

  for(i=0; i<m; i++) {
    for(j=0; j<n; j++) {
      B[i + j*m] = *A++;
    }
  }
}

/**
 * @brief   scalar product B = scale x A
 */
template <typename T>
void matrix_soft_scale(size_t len, const T *A, T *B, T scale) {
  while (len--) {
    *B++ = *A++ * scale;
  }
}

/**
 * @brief   multiply matrix A(m x p) by  B(p x n), put result in C(m x n)
 */
template <typename T>
void matrix_soft_dot(size_t m, size_t p, size_t n,
                     const T *A, const T *B, T *C) {
  size_t i, j, k;
  T tmp;

  for(i=0; i<m; i++) {     //each row in A
    for(j=0; j<n; j++) {   //each column in B
      tmp = 0;
      for(k=0; k<p; k++)  //each element in row A & column B
        tmp += A[i*p + k] * B[k*n + j];
      //C[i*n + j] = tmp;
      *C++ = tmp;
    }
  }
}

/**
 * @brief     C[m x n] = A[m x n] + B[m x n];
 */
template <typename T>
void matrix_soft_add(size_t len, const T *A, const T *B, T *C) {
  for (size_t i=0; i<len; i++) {
    C[i] = A[i] + B[i];
  }
}

/**
 * @brief     C[m x n] = A[m x n] - B[m x n];
 */
template <typename T>
void matrix_soft_sub(size_t len, const T *A, const T *B, T *C) {
  for (size_t i=0; i<len; i++) {
    C[i] = A[i] - B[i];
  }
}

// Matrix Inversion Routine from http://www.arduino.cc/playground/Code/MatrixMath
// * This function inverts a matrix based on the Gauss Jordan method.
// * Specifically, it uses partial pivoting to improve numeric stability.
// * The algorithm is drawn from those presented in
//   NUMERICAL RECIPES: The Art of Scientific Computing.
// * The function returns 1 on success, 0 on failure.
// * NOTE: The argument is ALSO the result matrix, meaning the input matrix is REPLACED

// A = input matrix AND result matrix
// n = number of rows = number of columns in A (n x n)
template <typename T>
int matrix_soft_inverse(int n, T *A){

  int pivrow = 0; // keeps track of current pivot row
  int k,i,j;      // k: overall index along diagonal; i: row index; j: col index
  int pivrows[n]; // keeps track of rows swaps to undo at end
  T tmp;    // used for finding max value and making column swaps

  for (k=0; k<n; k++){
    // find pivot row, the row with biggest entry in current column
    tmp = 0;
    for (i=k; i<n; i++){
      if (fabs(A[i*n + k]) >= tmp){ // 'Avoid using other functions inside abs()?'
        tmp = fabs(A[i*n + k]);
        pivrow = i;
      }
    }

    // check for singular matrix
    if (A[pivrow*n + k] == 0)
      return 0; //Inversion failed due to singular matrix

    // Execute pivot (row swap) if needed
    if (pivrow != k) {
      for (j=0; j<n; j++) {// swap row k with pivrow
        tmp = A[k*n + j];
        A[k*n + j] = A[pivrow*n + j];
        A[pivrow*n + j] = tmp;
      }
    }
    pivrows[k] = pivrow;  // record row swap (even if no swap happened)

    tmp = 1 / A[k*n + k];  // invert pivot element
    A[k*n + k] = 1;    // This element of input matrix becomes result matrix

    // Perform row reduction (divide every element by pivot)
    for (j=0; j<n; j++)
      A[k*n + j] = A[k*n + j] * tmp;

    // Now eliminate all other entries in this column
    for (i=0; i<n; i++){
      if (i != k){
        tmp = A[i*n + k];
        A[i*n + k] = 0;  // The other place where in matrix becomes result mat
        for (j=0; j<n; j++)
          A[i*n + j] = A[i*n + j] - A[k*n + j] * tmp;
      }
    }
  }

  // Done, now need to undo pivot row swaps by doing column swaps in reverse order
  for (k=n-1; k >= 0; k--){
    if (pivrows[k] != k){
      for (i = 0; i < n; i++){
        tmp = A[i*n + k];
        A[i*n + k] = A[i*n + pivrows[k]];
        A[i*n + pivrows[k]] = tmp;
      }
    }
  }
  return 1;
}


} //namespace matrix

#endif /* MATRIX_SOFT_ENGINE_HPP */
