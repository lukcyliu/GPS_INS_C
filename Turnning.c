//
// Created by AosCh on 2017/11/3.
//

#include <stdio.h>
#include "Turnning.h"
#define false 0
#define true 1

typedef int boolean;
const double t_deg2rad = 3.1415926 / 180;
const double t_rad2deg = 180 / 3.1415926;

double lastAv[3];
boolean initAngle = false;
boolean calibrateGyr = true;
boolean checkMag = false;
boolean useGyr = true;
//static ArrayList<Double> initGyrMagList = new ArrayList<Double>();
double *initGyrMagList;
double initGyrHeading;
double lastGyrOrientation;

double lastUncalibratedGyrOrientation;


const double thetaCThreshold = 12.45;
const double thetaMThreshold = 11.110; // 11.115
double lastMagOrientation;
int flag = 0;
double lastStepOrientation;
double Weights[6][3] = {
        // lastStepOrientation, magOrientation, gyrOrientation
        {0.004, 0.488, 0.508},  // walk straight, when thetaC <= thetaCThreshold && thetaM <= thetaMThreshold
        {0.008, 0.730, 0.262},  // turning, when thetaC <= thetaCThreshold && thetaM > thetaMThreshold
        {0.904, 0.073, 0.023},  // when thetaC > thetaCThreshold && thetaM <= thetaMThreshold
        {0.850, 0.064, 0.086},
        {0.170, 0.015, 0.815},    // magnetometer disturbed, when thetaC > thetaCThreshold && thetaM > thetaMThreshold
        {0.000, 0.000, 1.000}

//            {0.004, 0.488, 0.508},  // walk straight, when thetaC <= thetaCThreshold && thetaM <= thetaMThreshold
//            {0.008, 0.730, 0.262},  // turning, when thetaC <= thetaCThreshold && thetaM > thetaMThreshold
//            {0.904, 0.073, 0.023},  // when thetaC > thetaCThreshold && thetaM <= thetaMThreshold
//            {0.850, 0.064, 0.086},
//            {0.170, 0.015, 0.815},    // magnetometer disturbed, when thetaC > thetaCThreshold && thetaM > thetaMThreshold
//            {0.000, 0.000, 1.000}

//     	{0.01, 0.5, 0.49},  // walk straight, when thetaC <= thetaCThreshold && thetaM <= thetaMThreshold
//        {  0, 0.722, 0.278},  // turning, when thetaC <= thetaCThreshold && thetaM > thetaMThreshold
//        {0.95, 0.05, 0},
//        {0.86, 0.06, 0.08},  // when thetaC > thetaCThreshold && thetaM <= thetaMThreshold
//        {0.178,   0, 0.822},   // magnetometer disturbed, when thetaC > thetaCThreshold && thetaM > thetaMThreshold
//        {0.1, 0.5, 0.4}
};
long countMethod[5];

int calibrateGyrStepCount;

double viewWidth = 960;
double midAccHeight = 120, midGyrHeight = 120, midMagHeight = 120, midPathHeight = 240;

double STEP_LENGTH = 0.6;
int magCheckStepCount;
double lastThetaC;

int count_initGyrMagList = 0;

double* TurnningTest(double gyoX, double gyoY, double gyoZ, double magX, double magY, double magZ) {
    if (initGyrMagList == NULL) {
        initGyrMagList = (double *) malloc(sizeof(double));
        count_initGyrMagList = 0;
    }
    double *resultOrientation = (double*)malloc(sizeof(double) * 4);
    double gyoTemp[3] = {gyoX,gyoY,gyoZ};
    double magOrientation;
    double magNorm = sqrt(magX * magX + magY * magY);
    if(magY > 0)
        magOrientation = asin(magX / magNorm) * t_rad2deg;
    else
        magOrientation = (M_PI - asin(magX / magNorm)) * t_rad2deg;
    if(magOrientation > 180)
        magOrientation -= 360;

    magOrientation = -magOrientation;

    double avSum[3];
    double temp1[3];temp1[0] = lastAv[0];temp1[1] = lastAv[1];temp1[2] = lastAv[2];
    double temp2[3];temp2[0] = gyoTemp[0];temp2[1] = gyoTemp[1];temp2[2] = gyoTemp[2];
    avSum[0] += (temp1[0] + temp2[0]) * 0.05;
    avSum[1] += (temp1[1] + temp2[1]) * 0.05;
    avSum[2] += (temp1[2] + temp2[2]) * 0.05;

    double gyrOrientationDiff = -avSum[2];
    lastAv[0] = gyoTemp[0];lastAv[1] = gyoTemp[1];lastAv[2] = gyoTemp[2];

    double gyrOrientation = 720;
    double uncalibratedGyrOrientation = 720;
    double calibratedMagOrientation = 720;

    if(initAngle == false){
        if(fabs(gyrOrientation) > 20) {
            free(initGyrMagList);
            initGyrMagList = (double*)malloc(sizeof(double));
            count_initGyrMagList = 0;
        }else{
            initGyrMagList[count_initGyrMagList++] = magOrientation;
            initGyrMagList = realloc(initGyrMagList,sizeof(double) * (count_initGyrMagList + 1));
        }

        if (count_initGyrMagList > 10){
            initAngle = true;
            initGyrHeading = calculateAngle325(initGyrMagList,count_initGyrMagList);
            free(initGyrMagList);
            initGyrMagList = (double*)malloc(sizeof(double));
            count_initGyrMagList = 0;
            gyrOrientation = initGyrHeading;
            uncalibratedGyrOrientation = initGyrHeading;
        }
    }else{
        gyrOrientation = lastGyrOrientation + gyrOrientationDiff;
        uncalibratedGyrOrientation = lastUncalibratedGyrOrientation + gyrOrientationDiff;
        while (gyrOrientation > 180) gyrOrientation -= 360;
        while (gyrOrientation < -180) gyrOrientation += 360;
        while (uncalibratedGyrOrientation > 180) uncalibratedGyrOrientation -= 360;
        while (uncalibratedGyrOrientation < -180) uncalibratedGyrOrientation += 360;
    }
    if(useGyr == false)
        gyrOrientation = 720;

    double stepOrientation;
    double thetaC = fabs(magOrientation - gyrOrientation);
    if(thetaC > 180) thetaC = 360 - thetaC;
    double thetaM = fabs(magOrientation - lastMagOrientation);
    if(thetaM > 180) thetaM = 360 - thetaM;
    if(gyrOrientation == 720){
        if(lastStepOrientation == 720){
            stepOrientation = magOrientation;
        }else{
            if(lastMagOrientation != 720 && thetaM > 15){
                if(fabs(gyrOrientationDiff) < 10){
                    double orientations[2] = {lastStepOrientation,magOrientation};
                    double weights[2] = {0.8,0.2};
                    stepOrientation = fuseOrientations(orientations,weights,2);
                }else{
                    double orientations[2] = {lastStepOrientation,magOrientation};
                    double weights[2] = {0.2,0.8};
                    stepOrientation = fuseOrientations(orientations,weights,2);
                }
            }else{
                double orientations[2] = {lastStepOrientation,magOrientation};
                double weights[2] = {0.5,0.5};
                stepOrientation = fuseOrientations(orientations,weights,2);
            }
        }
        gyrOrientation = stepOrientation;
        uncalibratedGyrOrientation = stepOrientation;
    }else{
        double orientations[3] = {lastStepOrientation, magOrientation, gyrOrientation};
        if (thetaC <= thetaCThreshold && thetaM <= thetaMThreshold) {
            double weights[3] = {Weights[0][0],Weights[0][1],Weights[0][2]};
//                double[] weights = {0.3, 0.4, 0.3};
            stepOrientation = fuseOrientations(orientations, weights,3);
            flag = 1;
            countMethod[0]++;
        } else if (thetaC <= thetaCThreshold && thetaM > thetaMThreshold) {
            double weights[3] = {Weights[1][0],Weights[1][1],Weights[1][2]};
            stepOrientation = fuseOrientations(orientations, weights,3);
            flag = 2;
            countMethod[1]++;
        } else if (thetaC > thetaCThreshold && thetaM <= thetaMThreshold) {
            if (fabs(gyrOrientationDiff) < 2.5) {
                flag = 3;
                countMethod[2]++;
                double weights[3] = {Weights[2][0],Weights[2][1],Weights[2][2]};
//                    double[] weights = {0.7, 0.055, 0.245};
                stepOrientation = fuseOrientations(orientations, weights,3);
            } else {
                flag = 4;
                countMethod[3]++;
                double weights[3] = {Weights[3][0],Weights[3][1],Weights[3][2]};
                stepOrientation = fuseOrientations(orientations, weights,3);
            }
        } else {
            if (fabs(gyrOrientationDiff) < 3.6) {
                flag = 5;
                countMethod[4]++;
                double weights[3] = {Weights[4][0],Weights[4][1],Weights[4][2]};
                stepOrientation = fuseOrientations(orientations, weights,3);
                calibratedMagOrientation = stepOrientation;
            } else {
                flag = 6;
                countMethod[5]++;
                double weights[3] = {Weights[5][0],Weights[5][1],Weights[5][2]};
                stepOrientation = fuseOrientations(orientations, weights,3);
            }
        }
    }

    if (calibrateGyr) {
        double calibrateMagOri = 720;
        int magFlag = 0;
        double calibrateStepOri = 720;
        int stepFlag = 0;
        double stepDiff = fabs(lastStepOrientation - stepOrientation);
        if (stepDiff > 180) stepDiff = 360 - stepDiff;
        if (lastStepOrientation != 720 && stepDiff < 5) {
            calibrateGyrStepCount++;
        } else {
            calibrateGyrStepCount = 0;
        }
        if (calibrateGyrStepCount == 18) {
            calibrateStepOri = stepOrientation; // 取当前一步的方向作为绝对方向来校准陀螺仪的方向，可以改成求前几步的加权平均，类似下面地磁的处理。
            stepFlag = 1;
            calibrateGyrStepCount = 0;
            magCheckStepCount = 0;
        }

//            if (gyrOrientationDiff < 5) {
//                calibrateGyrMagList.add(magOrientation);
//            } else {
//                calibrateGyrMagList.clear();
//            }
//            if (calibrateGyrMagList.size() == 40) {
//                calibrateMagOri = AngleUtils.calculateAngle325(calibrateGyrMagList); // 取前几步地磁的平均作为绝对方向来校准陀螺仪的方向。
//                magFlag = 1;
//                calibrateGyrMagList.clear();
//            }

        if (magFlag + stepFlag == 1) {
            gyrOrientation = calibrateMagOri * magFlag + calibrateStepOri * stepFlag;
        } else if (magFlag + stepFlag == 2) {
            double* list = (double*)malloc(sizeof(double) * 2);
            list[0] = calibrateMagOri;
            list[1] = calibrateStepOri;
            gyrOrientation = calculateAngle325(list,2);
        }
    }

    if (checkMag) {
        double thetaCDiff = fabs(thetaC - lastThetaC);
        if (thetaCDiff < 1.2) {
            magCheckStepCount++;
        } else {
            magCheckStepCount = 0;
        }
        if (magCheckStepCount == 5) {
            gyrOrientation = magOrientation;
            stepOrientation = magOrientation;
            printf("checkma\n");
            magCheckStepCount = 0;
        }
    }

    lastGyrOrientation = gyrOrientation;
    lastUncalibratedGyrOrientation = uncalibratedGyrOrientation;
    lastMagOrientation = magOrientation;
    lastStepOrientation = stepOrientation;
    lastThetaC = thetaC;

    resultOrientation[0] = gyrOrientation;
    resultOrientation[1] = uncalibratedGyrOrientation;
    resultOrientation[2] = magOrientation;
    resultOrientation[3] = stepOrientation;

    return resultOrientation;

}
double fuseOrientations(double orientations[], double weights[],int size){
    int count = 0;
    double* list = (double*)malloc(sizeof(double));
    for (int i = 0; i < size; i++) {
        int num = round(weights[i] * 1000);
        for(int j = 0;j < num;j++){
            list[count++] = orientations[i];
            list = realloc(list, sizeof(double) * (count + 1));
        }
    }
    return calculateAngle325(list,count);
}
double calculateAngle325(double* list,int size){
    int count = 0;
    int positive = 0, negative = 0;
    if(size != 0){
        double *positiveList = (double*)malloc(sizeof(double));
        double *negativeList = (double*)malloc(sizeof(double));
        for (int i = 0; i < size; i++) {
            if(list[i] >= 0){
                positiveList[positive++] = list[i];
                positiveList = realloc(positiveList, sizeof(double) * (positive + 1));
            }else{
                negativeList[negative++] = list[i];
                negativeList = realloc(negativeList, sizeof(double) * (negative + 1));
            }
            if(list[i] > 90 || list[i] < -90)
                count++;
        }
        double sum = size;
        double percent = positive / sum;
        if(positive == 0 || positive == sum)
            return calculateAvg(list,size);
        if(count > sum * 0.3){
            double avgPositive = calculateAvg(positiveList,positive);
            double avgNegative = calculateAvg(negativeList,negative);

            double angleDifferenceABS = fabs(avgPositive - avgNegative);
            double angleDifference = 0;

            if(angleDifferenceABS >= 180){
                angleDifference = 360 - angleDifferenceABS;
            }else{
                angleDifference = angleDifferenceABS;
            }
            double avgPiece = angleDifference / sum;

            if(fabs(avgNegative) + fabs(avgPositive) > 180){
                double num = avgNegative - positive * avgPiece;
                if(num < -180)
                    return num + 360;
                else
                    return num;
            }else{
                double num = avgNegative + positive * avgPiece;
                return num;
            }
        }else{
            return calculateAvg(list,size);
        }
    }else{
        return 0;
    }
}
double calculateAvg(double* list,int size){
    double sum = 0.0;
    for(int i = 0;i < size;i++){
        sum += list[i];
    }
    return sum / size;
}