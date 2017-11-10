#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "NewKalmanFilter.h"
#include "MahonyAHRS.h"
#include "Turnning.h"

//------------------------------main thread--------------------------------------------
const char *openfile = "8.28.csv";
const char *savefile = "离线结果.csv";
const double INS_driftw_eight = 0.5;
int doTurning = 1;//是否用turning算法航位推算覆盖Vccq,0为否
int GPSoffAddMod=1;//失效是的航向选取,1为用gpsyaw,2为用TurningYaw,3为用MahonyYaw
const double distace_GPS = 10;
double ***smoothRes;
const int width = 12;//平滑窗口大小
int firstSmooth = 0;
const int data_size = 20;//每个数据的长度

double stepP[3] = {0,0,0};
double gyoP[3] = {0,0,0};
double magP[3] = {0,0,0};

const double accCalErr_X = -0.09092, accCalErr_Y = 0.081208, accCalErr_Z = 0.015632;
const double deg_rad = 3.1415926 / 180;
const double rad_deg = 180 / 3.1415926;
const int worksize = 1500;

double acc[3][12];
double gyo[3][12];
double mag[3][12];
double gps[2][12];

//acc高通低通滤波
double acc_mod = 0, acc_low = 0, acc_high = 0;
double low_value = 0.2;
double hx = 0, hy = 0, hAy = 0, hAx = 0, hVy = 0, hVx = 0, hSx = 0, hSy = 0, hV2 = 0, hS2 = 0, hSy2 = 0, hSx2 = 0, lasthA2 = 0, lasthV2 = 0;
double lasthAy = 0, lasthAx = 0, lastlastAy = 0, lastlastAx = 0, lasthVy = 0, lasthVx = 0, lastlastVy = 0, lastlastVx = 0;
double yaw = 90;
double hubuYaw = 90;
double ax, ay, az, gx, gy, gz, mx, my, mz, jyYaw, GPSYaw = 0, GPSv = 0, GPSLongitude, GPSLattitude, GPSHeight, GPS_SN;

double q[4];
double smoothAx = 0, smoothAy = 0, smoothAz = 0, smoothGx = 0, smoothGy = 0, smoothGz = 0, smoothMx = 0, smoothMy = 0, smoothMz = 0, smoothGPSYaw = 0, smoothGPSv = 0;
double GPSVn = 0, GPSVe = 0, GPSVu = 0;
double lastGPSLongtitude = 0, lastGPSLattitude = 0, lastGPSh = 0,lastGPSyaw = 0,lastGPSv = 0;
double last_L = 0, last_E = 0, last_h = 0;
int firstGPSOff = 1;
int firstGPSVcc_in = 0;
int firstGPSVcc_out = 0;
int GPSOff = 0;
double speedE = 0, speedN = 0, speedH = 0;
double pre_lamb = 0, pre_L = 0, pre_h = 0;
double INS_lamb = 0, INS_L = 0, INS_h = 0;
double lastpVx = 0, lastpVy = 0, lastpVz = 0;
double pVx = 0, pVy = 0, pVz = 0;
double lastAx = 0, lastAy = 0, lastAz = 0;
double Px = 0, Py = 0;
int insCnt = 0;
double Pitch0 = 0, Roll0 = 0, Yaw0 = 0;
double Pitch = 0, Roll = 0, Yaw = 0;
double firstPitch = 0, firstRoll = 0, firstYaw = 0;
double eInt[3] = {0, 0, 0};
//Re长半轴 r短半轴 f椭球扁率 e椭球偏心率 wie地球自转角速率
double earthRe = 6378137, earthr = 6356752.3142, earthf = 1 / 298.257, earthe = 0.0818, earthwie = 7.292e-5;
extern double G0;
double *Fn;

double Rm = 0, Rn = 0, R0 = 0, lastRm = 0, lastRn = 0;
double WieE = 0, WinE = 0, WWX = 0;
//Matrix Fn = new Matrix(3,1);
double Vccq[3] = {0, 0, 0};
double L = 39.980793 * 3.1415926 / 180, E = 116.321083 * 3.1415926 / 180, h = 51.3;
//融合迭代补偿tao
double tao = 0;

//Mahony引入变量
extern double q0, q1, q2, q3;
double *mahonyR;
double *XX;
int count = 1;

//for test
int firstGPSVcc = 0;
int GPSout = 0;



char output_string[1000];

double *EularToQuaternion(double yaw, double pitch, double roll) {
    double *Qresult = (double *) malloc(sizeof(double) * 4);
    double cosRoll = cos(roll * 0.5);
    double sinRoll = sin(roll * 0.5);
    double cosPitch = cos(pitch * 0.5);
    double sinPitch = sin(pitch * 0.5);
    double cosYaw = cos(yaw * 0.5);
    double sinYaw = sin(yaw * 0.5);

    Qresult[0] = (double) (cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw);
    Qresult[1] = (double) (sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw);
    Qresult[2] = (double) (cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw);
    Qresult[3] = (double) (cosRoll * cosPitch * sinYaw - sinRoll * sinRoll * cosYaw);

    return Qresult;
}

double **getDatas(int size) {
    FILE *file = fopen(openfile, "r");
    char temp[1000];
    char *t = (char *) malloc(1000);
    double **datas = (double **) malloc(sizeof(double *) * 1);
    datas[0] = (double *) malloc(sizeof(double) * size);
    while (fgets(temp, 1000, file) != EOF) {
        if (temp[0] == 0)
            break;
        for (int i = 0; i < size; i++) {
            if (i == 0) {
                t = strtok(temp, ",");
                datas[count - 1][0] = atof(t);
            } else {
                t = strtok(NULL, ",");
                datas[count - 1][i] = atof(t);
            }
        }
        count++;
        datas = realloc(datas, sizeof(double *) * (count));
        datas[count - 1] = (double *) malloc(sizeof(double) * size);
        memset(temp, 0, 1000);
    }
    return datas;
}

void slideWindows(double ax, double ay, double az, double gx, double gy, double gz, double mx, double my, double mz,
                  double GPSyaw, double GPSv) {
    if (firstSmooth == 0) {
        for (int i = 0; i < width; i++) {
            smoothRes[0][0][i] = ax;
            smoothRes[0][1][i] = ay;
            smoothRes[0][2][i] = az;
            smoothRes[1][0][i] = gx;
            smoothRes[1][1][i] = gy;
            smoothRes[1][2][i] = gz;
            smoothRes[2][0][i] = mx;
            smoothRes[2][1][i] = my;
            smoothRes[2][2][i] = mz;
            smoothRes[3][0][i] = GPSyaw;
            smoothRes[3][1][i] = GPSv;
        }
        firstSmooth = 1;
    } else {
        for (int i = 1; i < width; i++) {
            smoothRes[0][0][i - 1] = smoothRes[0][0][i];
            smoothRes[0][1][i - 1] = smoothRes[0][1][i];
            smoothRes[0][2][i - 1] = smoothRes[0][2][i];
            smoothRes[1][0][i - 1] = smoothRes[1][0][i];
            smoothRes[1][1][i - 1] = smoothRes[1][1][i];
            smoothRes[1][2][i - 1] = smoothRes[1][2][i];
            smoothRes[2][0][i - 1] = smoothRes[2][0][i];
            smoothRes[2][1][i - 1] = smoothRes[2][1][i];
            smoothRes[2][2][i - 1] = smoothRes[2][2][i];
            smoothRes[3][0][i - 1] = smoothRes[3][0][i];
            smoothRes[3][1][i - 1] = smoothRes[3][1][i];
        }
        smoothRes[0][0][width - 1] = ax;
        smoothRes[0][1][width - 1] = ay;
        smoothRes[0][2][width - 1] = az;
        smoothRes[1][0][width - 1] = gx;
        smoothRes[1][1][width - 1] = gy;
        smoothRes[1][2][width - 1] = gz;
        smoothRes[2][0][width - 1] = mx;
        smoothRes[2][1][width - 1] = my;
        smoothRes[2][2][width - 1] = mz;
        smoothRes[3][0][width - 1] = GPSyaw;
        smoothRes[3][1][width - 1] = GPSv;
    }
}

void quternionToCnbMatrix(double q0, double q1, double q2, double q3) {
    mahonyR = (double *) malloc(sizeof(double) * 9);

    double q0q0 = q0 * q0;
    double q1q1 = q1 * q1;
    double q2q2 = q2 * q2;
    double q3q3 = q3 * q3;
    double q0q1 = q0 * q1;
    double q0q2 = q0 * q2;
    double q0q3 = q0 * q3;
    double q1q2 = q1 * q2;
    double q1q3 = q1 * q3;
    double q2q3 = q2 * q3;
    //定义Cnb
    mahonyR[0] = q0q0 + q1q1 - q2q2 - q3q3;
    mahonyR[1] = 2 * (q1q2 - q0q3);
    mahonyR[2] = 2 * (q1q3 + q0q2);
    mahonyR[3] = 2 * (q1q2 + q0q3);
    mahonyR[4] = q0q0 - q1q1 + q2q2 - q3q3;
    mahonyR[5] = 2 * (q2q3 - q0q1);
    mahonyR[6] = 2 * (q1q3 - q0q2);
    mahonyR[7] = 2 * (q2q3 + q0q1);
    mahonyR[8] = q0q0 - q1q1 - q2q2 + q3q3;
}

void accToFn(double x, double y, double z) {
    //mahonyR = MatT(mahonyR,3,3);
    Fn = (double *) malloc(sizeof(double) * 3);
    double *m = (double *) malloc(sizeof(double) * 3);
    m[0] = x;
    m[1] = y;
    m[2] = z;
    Fn = MatMul(mahonyR, 3, 3, m, 3, 1);
}

int main(int argc, char *argv[]) {
    smoothRes = (double ***) malloc(sizeof(double **) * 4);
    for (int i = 0; i < 4; i++) {
        smoothRes[i] = (double **) malloc(sizeof(double *) * 3);
        for (int j = 0; j < 3; j++)
            smoothRes[i][j] = (double *) malloc(sizeof(double) * 12);
    }
    FILE *foutput = fopen(savefile, "w"),*f_tokf= fopen("file_tokf.csv","w"),*f_kfresult = fopen("file_kfresult.csv","w");
    //循环队列滑动滤波算法测试
    double value_buf[width];
    int slideN = 0;

    //数据读取
    double **datas = getDatas(data_size);

    //初始化定姿
    double *data0 = (double *) malloc(sizeof(double) * data_size);
    data0 = datas[0];
    ax = -(data0[0] - accCalErr_X);
    ay = -(data0[1] - accCalErr_Y);
    az = -(data0[2] - accCalErr_Z);
    gx = data0[3];
    gy = data0[4];
    gz = data0[5];
    mx = data0[6];
    my = data0[7];
    mz = -data0[8];
    lastGPSh = data0[16];

    h = lastGPSh;
    L = data0[15]  * 3.1415926 / 180;
    E = data0[14]  * 3.1415926 / 180;
    last_E = E;
    last_L = L;
    //求初始的姿态角
    Pitch0 = atan2(-ay, -az);
    Roll0 = atan2(ax, -az);
    Yaw0 = atan2(-my * cos(Roll0) + mz * sin(Roll0),
                 mx * cos(Pitch0) + my * sin(Pitch0) * sin(Roll0) - mz * sin(Pitch0) * cos(Roll0));
    firstPitch = Pitch0 * 57.29578;
    firstRoll = Roll0 * 57.29578;
    firstYaw = -Yaw0 * 57.29578;
    //计算初始四元数
    double *m_q0 = EularToQuaternion(Yaw0, Pitch0, Roll0);
//    q0 = m_q0[0];
//    q1 = m_q0[1];
//    q2 = m_q0[2];
//    q3 = m_q0[3];
//
//    q0 = data0[20];
//    q1 = data0[21];
//    q2 = data0[22];
//    q3 = -data0[23];

    int d_count = 1;
    double *data = datas[d_count];
    int datacnt;

    while (d_count <= count - 2) {
        ax = -(data[0] - accCalErr_X);
        ay = -(data[1] - accCalErr_Y);
        az = -(data[2] - accCalErr_Z);
        gx = data[3];
        gy = data[4];
        gz = data[5];
        mx = data[6];
        my = data[7];
        mz = -data[8];

        GPSLongitude = data[14];
        GPSLattitude = data[15];
        GPSHeight = data[16];
        GPSYaw = data[17];
        GPSv = data[18];
        GPS_SN = data[19];



//---------------------------------------------滑动滤波-----------------------------------------------------------------//
        //暴力法
        slideWindows(ax, ay, az, gx, gy, gz, mx, my, mz, GPSYaw, GPSv);
        for (int j = 0; j < width; ++j) {
            smoothAx += smoothRes[0][0][j];
            smoothAy += smoothRes[0][1][j];
            smoothAz += smoothRes[0][2][j];
            smoothGx += smoothRes[1][0][j];
            smoothGy += smoothRes[1][1][j];
            smoothGz += smoothRes[1][2][j];
            smoothMx += smoothRes[2][0][j];
            smoothMy += smoothRes[2][1][j];
            smoothMz += smoothRes[2][2][j];
            smoothGPSYaw += smoothRes[3][0][j];
            smoothGPSv += smoothRes[3][1][j];
        }
        smoothAx /= width;
        smoothAy /= width;
        smoothAz /= width;
        smoothGx /= width;
        smoothGy /= width;
        smoothGz /= width;
        smoothMx /= width;
        smoothMy /= width;
        smoothMz /= width;
        smoothGPSYaw /= width;
        smoothGPSv /= width;

        GPSVn = smoothGPSv * cos(smoothGPSYaw * 3.1415926 / 180) / 3.6;
        GPSVe = smoothGPSv * sin(smoothGPSYaw * 3.1415926 / 180) / 3.6;
        GPSVu = GPSHeight - lastGPSh;

//-----------------------------------------------滑动滤波结束-------------------------------------------------------------------//

//-------------------------------------------------以下是乱七八糟的测试部分------------------------------------------------//
//        //用mahony互补滤波更新四元数并计算出姿态角
        MahonyAHRSupdate((gx * deg_rad), (gy * deg_rad), (gz * deg_rad), ax,
                         ay, az, mx, my, mz);
        Yaw = -atan2(2 * q1 * q2 + 2 * q0 * q3, 2 * q0 * q0 + 2 * q0 * q2 - 1) * rad_deg;
        Pitch = asin(2 * q2 * q3 + 2 * q0 * q1) * rad_deg;
        Roll = -atan2(2 * q1 * q3 + 2 * q0 * q2, 2 * q0 * q0 + 2 * q3 * q3 - 1) * rad_deg;

        //有更新四元数得到更新旋转矩阵Cnb
//        q0 = data[20];
//        q1 = data[21];
//        q2 = data[22];
//        q3 = -data[23];

        quternionToCnbMatrix(q0, q1, q2, q3);


        double *resultOrientation = TurnningTest(gx, gy, gz, -mx, -my, mz);
        stepP[0] += sin(resultOrientation[3] * 3.1415926 / 180);
        stepP[1] += cos(resultOrientation[3] * 3.1415926 / 180);
        gyoP[0] += sin(resultOrientation[0] * 3.1415926 / 180);
        gyoP[1] += cos(resultOrientation[0] * 3.1415926 / 180);
        magP[0] += sin(resultOrientation[2] * 3.1415926 / 180);
        magP[1] += cos(resultOrientation[2] * 3.1415926 / 180);

        //测试单步长匀速路径
        Px += cos(Yaw * 3.1415926 / 180);
        Py += sin(Yaw * 3.1415926 / 180);


        //转置得到Cbn

        //计算更新的子午曲率半径Rm和卯酉曲率半径Rn以及曲率平均半径R0
        Rm = earthRe * (1 - 2 * earthf + 3 * earthf * sin(last_L) * sin(last_L));
        Rn = earthRe * (1 + earthf * sin(last_L) * sin(last_L));
        R0 = sqrt(Rm * Rm + Rn * Rn);

//-----------------------------------------------------积分测试部分--------------------------------------------------------//
        //投影到东北天坐标系下计算绝对加速度比力和计算绝对速度微分Vccq
        accToFn(smoothAx, smoothAy, smoothAz);
        Fn = MatMulk(Fn, 3, 1, G0);

//        Fn[0] = data[40];
//        Fn[1] = data[41];
//        Fn[2] = data[42];

        Vccq[0] = -Fn[0];
        Vccq[1] = Fn[1];
        Vccq[2] = Fn[2] + G0;
        if(doTurning == 1){
            Vccq[0] = ay * G0 * sin(resultOrientation[3] * 3.1415926 / 180);
            Vccq[1] = ay * G0 * cos(resultOrientation[3] * 3.1415926 / 180);
        }


        pVx += (Vccq[0]) * 0.2;
        pVy += (Vccq[1]) * 0.2;
        pVz += (Vccq[2]) * 0.2;

        L = L + (lastpVy / (Rm + last_h)) * 0.2;
        E = E + (lastpVx / (cos(last_L) * (Rn + last_h))) * 0.2;
        h = h - lastpVz * 0.2;

        lastAx = Vccq[0];
        lastAy = Vccq[1];
        lastAz = Vccq[2];



//
////---------------------------------------------------------融合------------------------------
        if (GPS_SN >= 4) {
            if(d_count % worksize == 0 ){
                lastGPSh = GPSHeight;
                h = lastGPSh;
                L = data[15]  * 3.1415926 / 180;
                E = data[14]  * 3.1415926 / 180;
                //求初始的姿态角
                Pitch0 = atan2(-ay, -az);
                Roll0 = atan2(ax, -az);
                Yaw0 = atan2(-my * cos(Roll0) + mz * sin(Roll0),
                             mx * cos(Pitch0) + my * sin(Pitch0) * sin(Roll0) - mz * sin(Pitch0) * cos(Roll0));
                firstPitch = Pitch0 * 57.29578;
                firstRoll = Roll0 * 57.29578;
                firstYaw = -Yaw0 * 57.29578;
                //计算初始四元数
                double *m_q0 = EularToQuaternion(Yaw0, Pitch0, Roll0);
                q0 = m_q0[0];
                q1 = m_q0[1];
                q2 = m_q0[2];
                q3 = m_q0[3];
                d_count++;
                data = datas[d_count];
                tao = 0;
                setNull();
                continue;
            }
            firstGPSOff = 1;
            //printf("Now we are in the GPS/INS mode...........................................session 2\n");
            if(GPSout == 1){
                E = GPSLongitude * 3.1415926 / 180;
                L = GPSLattitude * 3.1415926 / 180;
                h = GPSHeight;
                printf("Now we are setting the GPS location again after GPS lost...........................................session 2_1\n");
            }
            GPSout = 0;
//---------------------------------------------------------融合------------------------------
            tao += 0.2;
            double Dpv[6] = {L * rad_deg - GPSLattitude, E * rad_deg - GPSLongitude, h - GPSHeight, pVx - GPSVe,
                             pVy - GPSVn, pVz - GPSVu};

//            char datas_tokf[10000];
//            sprintf(datas_tokf,"%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n"
//                    ,datacnt,Dpv[0],Dpv[1],Dpv[2]
//                    ,Dpv[3],Dpv[4],Dpv[5]
//                    ,pVx,pVy,pVz
//                    ,last_L,last_h,Fn[0]
//                    ,Fn[1],Fn[2]
//                    ,tao,Rm,Rn
//                    ,mahonyR[0],mahonyR[1],mahonyR[2]
//                    ,mahonyR[3],mahonyR[4],mahonyR[5]
//                    ,mahonyR[6],mahonyR[7],mahonyR[8]);
//            fprintf(f_tokf,datas_tokf,10000);

            double *XX = kalman_GPS_INS_pv(Dpv, pVx, pVy, pVz, last_L, last_h, mahonyR, Fn, tao, Rm, Rn);

//            char datas_kfresult[10000];
//            sprintf(datas_kfresult,"%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n"
//                    ,datacnt,XX[0],XX[1],XX[2],XX[3],XX[4],XX[5],XX[6],XX[7],XX[8],XX[9]
//                    ,XX[10],XX[11],XX[12],XX[13],XX[15]);
//            fprintf(f_kfresult,datas_kfresult,10000);

            pVx = pVx - XX[3];
            pVy = pVy - XX[4];
            pVz = pVz - XX[5];
            L = L - 0.29 * XX[6];
            E = E - 0.32 * XX[7];
            h = h - XX[8];


            lastGPSLongtitude = GPSLongitude;
            lastGPSLattitude = GPSLattitude;
            lastGPSh = GPSHeight;
            lastGPSyaw = GPSYaw;
            lastGPSv = GPSv;

            double L_distance = fabs(L * rad_deg - GPSLattitude) * 111000;
            double E_distance = fabs(E * rad_deg - GPSLongitude) * 111000 * cos(GPSLattitude);
            double distance = sqrt(pow(L_distance,2) + pow(E_distance,2));

            if(distance >= distace_GPS){
                L = GPSLattitude * deg_rad;
                E = GPSLongitude * deg_rad;
            }


        }else if(GPS_SN < 4){
            printf("lost GPS..\nNow We are in INS Mode...................................session 3\n");
            if (firstGPSOff == 1){
                printf("first set lost GPS---------------------3.1");
                L = lastGPSLattitude * deg_rad;
                E = lastGPSLongtitude * deg_rad;
                h = lastGPSh;
                firstGPSOff = 0;
            }
            if (GPSoffAddMod == 1) {
                lastpVx = lastGPSv * sin(smoothGPSYaw * 3.1415926 / 180) / 3.6;
                lastpVy = lastGPSv * cos(smoothGPSYaw * 3.1415926 / 180) / 3.6;
            }else if (GPSoffAddMod == 2){
                lastpVx = lastGPSv * sin(resultOrientation[3] * 3.1415926 / 180) / 3.6;
                lastpVy = lastGPSv * cos(resultOrientation[3] * 3.1415926 / 180) / 3.6;
            } else if(GPSoffAddMod == 3){
                lastpVx = lastGPSv * sin(Yaw * 3.1415926 / 180) / 3.6;
                lastpVy = lastGPSv * cos(Yaw * 3.1415926 / 180) / 3.6;
            }
            L = L + (lastpVy / (Rm + last_h)) * 0.2 * INS_driftw_eight;
            E = E + (lastpVx / (cos(last_L) * (Rn + last_h))) * 0.2 * INS_driftw_eight;
            h = h ;
            printf("%d E = %f, L = %f, lastGPSyaw = %f\n",d_count,E * rad_deg,L * rad_deg,lastGPSyaw);
            GPSout = 1;

        }

        lastpVx = pVx;
        lastpVy = pVy;
        lastpVz = pVz;
        last_L = L;
        last_h = h;

        memset(output_string, 0, 1000);
        sprintf(output_string, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,"
                        "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d\n",
                smoothAx, smoothAy, smoothAz, smoothGx, smoothGy, smoothGz, smoothMx, smoothMy, smoothMz, smoothGPSYaw,
                smoothGPSv, GPSVe, GPSVn, Roll, Pitch, Yaw, Vccq[0], Vccq[1], Vccq[2], pVx, pVy, pVz, E * rad_deg,
                L * rad_deg, h,resultOrientation[0],resultOrientation[1],resultOrientation[2],resultOrientation[3],
                stepP[0],stepP[1],lastGPSLattitude,lastGPSyaw,d_count);

        fprintf(foutput, "%s", output_string);

        smoothAx = 0;
        smoothAy = 0;
        smoothAz = 0;
        smoothGx = 0;
        smoothGy = 0;
        smoothGz = 0;
        smoothMx = 0;
        smoothMy = 0;
        smoothMz = 0;
        smoothGPSYaw = 0;
        smoothGPSv = 0;

        d_count++;
        data = datas[d_count];
    }    fclose(foutput);
}