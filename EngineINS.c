//
// Created by aos on 17-3-14.
//

#include "EngineINS.h"


const char* WRITTEN_DIR = "/home/luxy/Desktop/blocksTest/datasheet/";
const char* NAME = "imu";

const double SP = 0.1;
const bool CENT_ACC_ON = true;
const bool STA_ACC_ON = false;

bool hasStaticStateSet = true;
struct Acceleration staticAcc;

struct Acceleration lastAcc;
struct AngularVelocity lastAV;
struct Velocity lastV;
struct Acceleration centripetalAcc;
struct Position currentPos;
struct Position tempPos;

//拐弯部分的变量暂时先空着，那边的函数也先空着，因为要用到java中的ArrayList，目前C不知道怎么写
struct AngularVelocity* turningWindowAV;
int size_turningWindowAV = 0;
struct Angle turningAngle = {0,0,0};
bool isTurning = false;
const double TURNING_SAMPLE_NUMBER_THRESHOLD = 500;

struct Velocity setFirstVcc(double speedVe,double speedVn,double speedVh){
    lastV.x = speedVe;
    lastV.y = speedVn;
    lastV.z = speedVh;
}

struct Velocity addNewAccelerationSample(struct Acceleration sampleAcc){
    // 必须要先获得静止时测得的加速度结果，目前默认为已设置
//    if(!hasStaticStateSet)
//        return NULL;
    // 减去静止加速度
//    if(STA_ACC_ON){
//        sampleAcc.x -= staticAcc.x;
//        sampleAcc.y -= staticAcc.y;
//        sampleAcc.z -= staticAcc.z;
//    }
    // 如果检测到拐弯，则进行向心加速度校正
    if(CENT_ACC_ON && isTurning){
        centripetalAcc = getCentripetalAcceleration();
        sampleAcc.x -= centripetalAcc.x;
        sampleAcc.y -= centripetalAcc.y;
        sampleAcc.z -= centripetalAcc.z;
        isTurning = false;
        char* output = dealDouble(centripetalAcc.x,centripetalAcc.y,centripetalAcc.z);
        writeCentripetalAcceleration(output);
    }

    struct Velocity sampleV = {lastV.x,lastV.y,lastV.z,lastV.t};

    sampleV.x += (sampleAcc.x + lastAcc.x) * SP * 0.5 * 9.8;
    sampleV.y += (sampleAcc.y + lastAcc.y) * SP * 0.5 * 9.8;
    sampleV.z += (sampleAcc.z + lastAcc.z) * SP * 0.5 * 9.8;

    sampleV.t = sampleAcc.t;
    lastAcc = sampleAcc;
    char* output = dealDouble(lastAcc.x,lastAcc.y,lastAcc.y);
	output = strcat(output,"\n");
    writeAcceleration(output);
    lastV = sampleV;

    return sampleV;
}

void addNewAngularVelocity(struct AngularVelocity sampleAV){
    if(hasStaticStateSet)
        return;
    if(size_turningWindowAV == 0)
        turningWindowAV = (struct AngularVelocity*)malloc(sizeof(struct AngularVelocity));

    struct AngularVelocity listLastAV = turningWindowAV[size_turningWindowAV - 1];
    turningAngle.x += (sampleAV.x + listLastAV.x) * SP * 0.5;
    turningAngle.y += (sampleAV.y + listLastAV.y) * SP * 0.5;
    turningAngle.z += (sampleAV.z + listLastAV.z) * SP * 0.5;

    turningWindowAV[size_turningWindowAV++] = sampleAV;

    if(size_turningWindowAV > TURNING_SAMPLE_NUMBER_THRESHOLD){
        struct AngularVelocity removedAV = turningWindowAV[0];
        for(int i = 0;i < size_turningWindowAV;i++)
            turningWindowAV[i] = turningWindowAV[i+1];
        size_turningWindowAV--;
        struct AngularVelocity firstAV = turningWindowAV[0];
        turningAngle.x -= (removedAV.x + firstAV.x) * SP * 0.5;
        turningAngle.y -= (removedAV.y + firstAV.y) * SP * 0.5;
        turningAngle.z -= (removedAV.z + firstAV.z) * SP * 0.5;
    }

    if(fabs(turningAngle.x) + fabs(turningAngle.y) + fabs(turningAngle.z) > M_PI / 4){
        isTurning = true;
        free(turningWindowAV);
        size_turningWindowAV = 0;
        turningAngle.x = 0;
        turningAngle.y = 0;
        turningAngle.z = 0;
    } else
        isTurning = false;
    lastAV.x = sampleAV.x;
    lastAV.y = sampleAV.y;
    lastAV.z = sampleAV.z;
    lastAV.t = sampleAV.t;
}//这个函数先空着

// 求当前状态的向心加速度，用来对加速度进行校正
struct Acceleration getCentripetalAcceleration(){
    struct Acceleration ca;//向心加速度centripetal acceleration

    double vx = lastV.x;
    double vy = lastV.x;
    double vz = lastV.x;
    double ax = (fabs(lastAV.x) > 2 ? lastAV.x : 0);
    double ay = (fabs(lastAV.y) > 2 ? lastAV.y : 0);
    double az = (fabs(lastAV.z) > 2 ? lastAV.z : 0);
    if(vx * vx + vy * vy + vz * vz < 1) return ca;
    if(ax * ax + ay * ay + az * az < 1) return ca;

    // 向心加速度的大小用a = w · v求得，而方向为w和v向量积的方向。
    // w和v的向量积：
    ca.x = ay * vz - az * vy;
    ca.y = az * vx - ax * vz;
    ca.z = ax * vy - ay * vx;

    //向量积的模
    double _1_mod = 1 / sqrt(pow(ca.x,2)) + pow(ca.y,2) + pow(ca.z,2);
    //向心加速度的模
    double ca_mod = sqrt((vx * vx + vy * vy) * az * az
                          + (vx * vx + vz * vz) * ay * ay
                          + (vy * vy + vz * vz) * ax * ax);
    // 向量积除以向量积的模得到向量积方向的单位向量，再乘以向心加速度的模得到向心加速度。
    // 手机得到的加速度数值为与其自身加速度相反的向量
    ca.x = ca.x * ca_mod * _1_mod * -1 * 0.01;
    ca.y = ca.y * ca_mod * _1_mod * -1 * 0.01;
    ca.z = ca.z * ca_mod * _1_mod * -1 * 0.01;

    return ca;
}

struct Position addNewVelocitySample(struct Acceleration sampleACC,struct Velocity sampleV){
    tempPos.x = (sampleV.x + lastV.x) * SP * 0.5;
    tempPos.x = (sampleV.x + lastV.x) * SP * 0.5;
    tempPos.x = (sampleV.x + lastV.x) * SP * 0.5;
    tempPos.t = sampleV.t;

//    currentPos.x += tempPos.x;
//    currentPos.y += tempPos.y;
//    currentPos.z += tempPos.z;
//    currentPos.t = sampleV.t;
    lastV = sampleV;

    return tempPos;
}

void outputCurrentState(){
    //写输出的文件用
    char* v_buff = dealDouble(lastV.x,lastV.y,lastV.z);
    char* p_buff = dealDouble(currentPos.x,currentPos.y,currentPos.z);
	v_buff = strcat(v_buff,"\n");
	p_buff = strcat(p_buff,"\n");
    writePosition(p_buff);
    writeVelocity(v_buff);
}

double* setSection(struct Acceleration A){
    double* Acc = (double*)malloc(3 * sizeof(double));
    Acc[0] = getTheclosest(A.x) * 0.01;
    Acc[1] = getTheclosest(A.y) * 0.01;
    Acc[2] = getTheclosest(A.z) * 0.01;
    return Acc;
}

double getTheclosest(double num){
    double result;
    int int_num = (int)num;
    if(fabs(num - int_num) <= 0.5)
        if(num > 0)
            result = int_num + 0.5;
        else
            result = int_num = 0.5;
    else
        if(num > 0)
            result = int_num + 1;
        else
            result = int_num - 1;
    return result;
}

void writeAcceleration(char* output){
    FILE* fp;
    char* WRITTEN_NAME = (char*)malloc(1000);
    memset(WRITTEN_NAME,0,1000);
    strcat(WRITTEN_NAME,WRITTEN_DIR);
    strcat(WRITTEN_NAME,NAME);
    strcat(WRITTEN_NAME,"_accleration.csv");
    fp = fopen(WRITTEN_NAME,"a+");
    fputs(output,fp);
    fclose(fp);
}

void writeVelocity(char* output){
    FILE* fp;
    char* WRITTEN_NAME = (char*)malloc(1000);
    memset(WRITTEN_NAME,0,1000);
    strcat(WRITTEN_NAME,WRITTEN_DIR);
    strcat(WRITTEN_NAME,NAME);
    strcat(WRITTEN_NAME,"_velocity.csv");
    fp = fopen(WRITTEN_NAME,"a+");
    fputs(output,fp);
    fclose(fp);
}

void writePosition(char* output){
    FILE* fp;
    char* WRITTEN_NAME = (char*)malloc(1000);
    memset(WRITTEN_NAME,0,1000);
    strcat(WRITTEN_NAME,WRITTEN_DIR);
    strcat(WRITTEN_NAME,NAME);
    strcat(WRITTEN_NAME,"_position.csv");
    fp =  fopen(WRITTEN_NAME,"a+");
    fputs(output,fp);
    fclose(fp);
}

void writeCentripetalAcceleration(char* output){
    FILE* fp;
    char* WRITTEN_NAME = (char*)malloc(1000);
    memset(WRITTEN_NAME,0,1000);
    strcat(WRITTEN_NAME,WRITTEN_DIR);
    strcat(WRITTEN_NAME,NAME);
    strcat(WRITTEN_NAME,"_centripetalacceleration.csv");
    fp =  fopen(WRITTEN_NAME,"a+");
    fputs(output,fp);
    fclose(fp);
}

char* dealDouble(double x,double y,double z){
    char* buff_x = (char*)malloc(1000);
    char* buff_y = (char*)malloc(1000);
    char* buff_z = (char*)malloc(1000);
    memset(buff_x,0,1000);
    memset(buff_y,0,1000);
    memset(buff_z,0,1000);
    sprintf(buff_x,"%f",x);
    sprintf(buff_y,"%f",y);
    sprintf(buff_z,"%f",z);
    char* buff = strcat(buff_x,",");
    buff = strcat(buff,buff_y);
    buff = strcat(buff,",");
    buff = strcat(buff,buff_z);

    return buff;
}

void setLastV(struct Velocity temp){
    lastV.x = temp.x;
    lastV.y = temp.y;
    lastV.z = temp.z;
    lastV.t = temp.t;
}
void setCurrentPos(struct Position temp){
    currentPos.x = temp.x;
    currentPos.y = temp.y;
    currentPos.z = temp.z;
    currentPos.t = temp.t;
}

void UseX(double X[15][1]){
    lastV.x += X[3][0];
    lastV.y += X[4][0];
    lastV.z += X[5][0];
    currentPos.x += X[6][0];
    currentPos.y += X[7][0];
    currentPos.z += X[8][0];
}
