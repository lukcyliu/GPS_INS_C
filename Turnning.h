//
// Created by AosCh on 2017/11/3.
//

#ifndef GPS_INS_NEW_TURNNING_H
#define GPS_INS_NEW_TURNNING_H
#include <math.h>
#include <stdlib.h>

double* TurnningTest(double gyoX, double gyoY, double gyoZ, double magX, double magY, double magZ);
double fuseOrientations(double orientations[], double weights[],int size);
double calculateAngle325(double* list,int size);
double calculateAvg(double* list,int size);
#endif //GPS_INS_NEW_TURNNING_H
