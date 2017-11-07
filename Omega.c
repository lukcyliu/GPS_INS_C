//
// Created by aos on 17-3-16.
//

#include "Omega.h"
#define M_PI 3.1415926

double L,h,VN,VE;
double* wie;
double* wep;
double** wadd;

const double e = 1 / 298.257;
const double Re = 6377830;
const double w = 0.00007292;

void initnum(double _L,double _h,double _VN,double _VE){
    L = _L;
    h = _h;
    VN = _VN;
    VE = _VE;
}

double getRm(){
    return Re * (1 - e * e) * pow(1 - e * e * sin(L * M_PI / 180),1.5);
}

double getRn(){
    return Re * sqrt(1 - e * e * pow(sin(L * M_PI / 180),2));
}

double* getWie(){
    wie = (double*)malloc(3 * sizeof(double));
    wie[0] = 0;
    wie[1] = w * cos(L * M_PI / 180);
    wie[2] = w * sin(L * M_PI / 180);
    return wie;
}

double* getWep(){
    wep = (double*)malloc(3 * sizeof(double));
    wep[0] = -VN / (getRm() + h);
    wep[1] = VE / (getRn() + h);
    wep[2] = VE / (getRn() + h) * tan(L * M_PI / 180);
    return wep;
}

double** getWadd(){
    wadd = (double**)malloc(3 * sizeof(double*));
    for(int i = 0;i < 3;i++)
        wadd[i] = (double*)malloc(3 * sizeof(double));
    wie = getWie();
    wep = getWep();
    wadd[0][0] = 0;
    wadd[0][1] = -(2 * wie[2] + wep[2]);
    wadd[0][2] = 2 * wie[1] + wep[1];
    wadd[1][0] = 2 * wie[2] + wep[2];
    wadd[1][1] = 0;
    wadd[1][2] = -(2 * wie[0] + wep[0]);
    wadd[2][0] = -(2 * wie[1] + wep[1]);
    wadd[2][1] = 2 * wie[0] + wep[0];
    wadd[2][2] = 0;
    return wadd;
}
