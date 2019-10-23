#ifndef INVERSE_H 
#define INVERSE_H

void sgemm(float *m1, float *m2, float *m3);
void strmm(float *m1, float *m2, CBLAS_SIDE Side, CBLAS_UPLO Uplo, CBLAS_TRANSPOSE TransA, CBLAS_DIAG Diag);
void ssymm(float *m1, float *m2, float *m3);
int matrix_inverse(float* A, int N);

#endif