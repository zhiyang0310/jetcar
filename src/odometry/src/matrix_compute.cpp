#include <cblas.h>
#include <f2c.h>

//#include <cstdio>

extern "C" {
    // LU decomoposition of a general matrix
     void sgetrf_(int* M, int *N, float* A, int* lda, int* IPIV, int* INFO);
 
    // generate inverse of a matrix given its LU decomposition
     void sgetri_(int* N, float* A, int* lda, int* IPIV, float* WORK, int* lwork, int* INFO);
}

int matrix_inverse(float* A, int N)
{
    int *IPIV = new int[N];
    int LWORK = N*N;
    float *WORK = new float[LWORK];
    int INFO = 1;
 
    sgetrf_(&N, &N, A, &N, IPIV, &INFO);

    if(INFO != 0){return 1;}
    else
    {
        INFO = 1;
    }
    
    sgetri_(&N, A, &N, IPIV, WORK, &LWORK, &INFO);
 
    delete [] IPIV;
    delete [] WORK;

    return INFO;
}
//test//
// int main(){
//     int N = 3;
//     float M[N*N]={1.0,2.0,3.0,
//                   2.0,4.0,5.0,
//                   3.0,5.0,6.0};
    
//     printf("%d\n", matrix_inverse(M, N));
 
//     printf("%f %f %f\n", M[0], M[1], M[2]);
//     printf("%f %f %f\n", M[3], M[4], M[5]);
//     printf("%f %f %f\n", M[6], M[7], M[8]);
 
//     return 0;
// }

////////////////////////function prototype in OPENBLAS/////////////////////////
// void cblas_ssymm(OPENBLAS_CONST enum CBLAS_ORDER Order, 
// 				 OPENBLAS_CONST enum CBLAS_SIDE Side, 
// 				 OPENBLAS_CONST enum CBLAS_UPLO Uplo, 
// 				 OPENBLAS_CONST blasint M, 
// 				 OPENBLAS_CONST blasint N,
//                  OPENBLAS_CONST float alpha, 
//                  OPENBLAS_CONST float *A, 
//                  OPENBLAS_CONST blasint lda, 
//                  OPENBLAS_CONST float *B,
//                  OPENBLAS_CONST blasint ldb, 
//                  OPENBLAS_CONST float beta, 
//                  float *C, 
//                  OPENBLAS_CONST blasint ldc);

// void cblas_sgemm(OPENBLAS_CONST enum CBLAS_ORDER Order, 
// 				 OPENBLAS_CONST enum CBLAS_TRANSPOSE TransA, 
// 				 OPENBLAS_CONST enum CBLAS_TRANSPOSE TransB, 
// 				 OPENBLAS_CONST blasint M, 
// 				 OPENBLAS_CONST blasint N, 
// 				 OPENBLAS_CONST blasint K,
// 				 OPENBLAS_CONST float alpha, 
// 				 OPENBLAS_CONST float *A, 
// 				 OPENBLAS_CONST blasint lda, 
// 				 OPENBLAS_CONST float *B, 
// 				 OPENBLAS_CONST blasint ldb, 
// 				 OPENBLAS_CONST float beta, 
// 				 float *C, 
// 				 OPENBLAS_CONST blasint ldc);

// void cblas_strmm(OPENBLAS_CONST enum CBLAS_ORDER Order, 
// 				 OPENBLAS_CONST enum CBLAS_SIDE Side, 
// 				 OPENBLAS_CONST enum CBLAS_UPLO Uplo,
// 				 OPENBLAS_CONST enum CBLAS_TRANSPOSE TransA,
//                  OPENBLAS_CONST enum CBLAS_DIAG Diag, 
//                  OPENBLAS_CONST blasint M, 
//                  OPENBLAS_CONST blasint N, 
//                  OPENBLAS_CONST float alpha, 
//                  OPENBLAS_CONST float *A, 
//                  OPENBLAS_CONST blasint lda, 
//                  float *B, 
//                  OPENBLAS_CONST blasint ldb);

//3-by-3 matrix m1*m2 -> m3
void sgemm(float *m1, float *m2, float *m3){
    cblas_sgemm(CblasColMajor,
                CblasNoTrans,
                CblasNoTrans,
                3, 3, 3, 1.0, m1, 3, m2, 3, 0.0, m3, 3);
}
//3-by-3 matrix m1*m2 -> m2 (m1 or m2 is triangle matrix)
void strmm(float *m1, float *m2, CBLAS_SIDE Side, CBLAS_UPLO Uplo, 
           CBLAS_TRANSPOSE TransA, CBLAS_DIAG Diag){
               cblas_strmm(CblasColMajor,
                           Side, Uplo, TransA, Diag, 
                           3, 3, 1.0, m1, 3, m2, 3);
}
//3-by-3 matrix m1*m2 -> m3 (m1 is symmetry matrix)
void ssymm(float *m1, float *m2, float *m3){
    cblas_ssymm(CblasColMajor,
                CblasLeft,
                CblasUpper,
                3, 3, 1.0, m1, 3, m2, 3, 0.0, m3, 3);
}

//test
// int main(){
//     float m1[9] = {1,0,0,2,1,0,3,4,1};
//     float m2[9] = {1,2,3,2,4,5,3,5,6};
//     float m3[9];
//     strmm(m1, m2, CblasLeft, CblasUpper, CblasNoTrans, CblasUnit);
//     strmm(m1, m2, CblasRight, CblasUpper, CblasTrans, CblasUnit);
//     cblas_ssymm(CblasColMajor,
//                 CblasLeft,
//                 CblasUpper,
//                 3, 3, 1.0, m1, 3, m2, 3, 0.0, m3, 3);
//     cblas_sgemm(CblasColMajor,
//                 CblasNoTrans,
//                 CblasNoTrans,
//                 3,3,3,1.0,m1,3,m2,3,0.0,m3,3);
//     for(int i = 0;i<9;++i){
//         std::cout<<*(m2+i)<<std::endl;
//     }
// }
