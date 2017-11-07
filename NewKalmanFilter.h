//
// Created by aos on 17-3-14.
//

#ifndef GPS_INS_NEWKALMANFILTER_H
#define GPS_INS_NEWKALMANFILTER_H
#include <math.h>
#include "matrix_lc.h"
double* kalman_GPS_INS_pv(double* Dpv,double Ve,double Vn,double Vu,double L,double h,double* mahonyR,double* Fn,double tao,double Rm,double Rn);
void setNull();
#endif //GPS_INS_NEWKALMANFILTER_H
