//
// Created by aos on 17-3-14.
//

#ifndef GPS_INS_ENGINEINS_H
#define GPS_INS_ENGINEINS_H

#include "parameter.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#include <memory.h>
struct Velocity setFirstVcc(double speedVe,double speedVn,double speedVh);
struct Velocity addNewAccelerationSample(struct Acceleration sampleAcc);
void addNewAngularVelocity(struct AngularVelocity sampleAV);
struct Acceleration getCentripetalAcceleration();
struct Position addNewVelocitySample(struct Acceleration sampleACC,struct Velocity sampleV);
void outputCurrentState();
double* setSection(struct Acceleration A);
double getTheclosest(double num);
void writeAcceleration(char* output);
void writeVelocity(char* output);
void writePosition(char* output);
void writeCentripetalAcceleration(char* output);
char* dealDouble(double x,double y,double z);
void setLastV(struct Velocity temp);
void setCurrentPos(struct Position temp);
void UseX(double X[15][1]);
#endif //GPS_INS_ENGINEINS_H
