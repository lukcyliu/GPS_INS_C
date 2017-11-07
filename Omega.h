//
// Created by aos on 17-3-16.
//

#ifndef GPS_INS_OMEGA_H
#define GPS_INS_OMEGA_H

#include <math.h>
#include <stdlib.h>
void initnum(double L,double h,double VN,double VE);
double getRm();
double getRn();
double* getWie();
double* getWep();
double** getWadd();
#endif //GPS_INS_OMEGA_H
