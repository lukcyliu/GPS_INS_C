//
// Created by aoschen on 17-10-25.
//

#ifndef RASPBERRY_SVD_H
#define RASPBERRY_SVD_H
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#define SIGN(a,b) ((b) >= 0.0 ? fabs(a) : -fabs(a))
#define MAX(a,b) (a>b?a:b)
#define MIN(a,b) (a<b?a:b)

double **Make2DArray(int row, int col);
void SetZero(double** a, int m);
void Free2DArray(double** a, int row);
void p(double**a, int m, int n);
int dsvd(double **a, int m, int n, double *w, double **v);
#endif //RASPBERRY_SVD_H
