#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void    MatShow(double* Mat, int row, int col);
double* MatAdd(double* A, double* B, int row, int col);
double* MatSub(double* A, double* B, int row, int col);
double* MatMul(double* A, int Arow, int Acol, double* B, int Brow, int Bcol);
double* MatMulk(double *A, int row, int col, double k);
double* MatT(double* A, int row, int col);
double  MatDet(double *A, int row);
double* MatInv(double *A, int row, int col);
double* MatPInv(double*A, int row, int col);
double  MatACof(double *A, int row, int m, int n);
double* MatAdj(double *A, int row, int col);
double *MatRead(char *csvFileName);
void MatWrite(const char *csvFileName, double *A, int row, int col);
double *MatDiag(double *A,int n);
double *MatEye(int n);

double* two_one(double** input,int row,int col);
double** one_two(double* input,int row,int col);