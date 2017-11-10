#include "NewKalmanFilter.h"
//----------------------------------------融合相关参数--------------------------------------------------------------//
//Re长半轴 r短半轴 f椭球扁率 e椭球偏心率 wie地球自转角速率
volatile double G0 = 9.8015;
double Q_wg = (1/(57*3600))*(1/(57*3600));//陀螺的随机漂移为0.5度每小时
double Tg[3] = {300,300,300};//陀螺仪误差漂移相关时间
double Ta[3] = {1000,1000,1000};//加表误差漂移相关时间
double Rlamt = 1e-5 * 3.1415926 /(60 * 180);//经纬度误差均方根，弧度制
double Rl = 1e-5 * 3.1415926 /(60 * 180);
double Rh = 1e-11;//高度误差均方根，单位米
double Rvx = 1e-7;//速度误差均方根，单位 米/秒
double Rvy = 1e-7;
double Rvz = 5e-9;
double Q_wa;//加速度计的随机偏差为0.5e-4*g
double* Q;
double* R;
double* PP;//初始误差协方差阵,加速度计的初始偏值均取1e-4*g 陀螺的常值漂移取0.1度每小时
double* X;

double* kalman_GPS_INS_pv(double* Dpv,double Ve,double Vn,double Vu,double L,double h,double* mahonyR,double* Fn,double tao,double Rm,double Rn) {
//    FILE *f_w = fopen("w.csv","a"),*f_z = fopen("z.csv","a");
    Q_wa = pow(((0.5e-4)*G0),2);//加速度计的随机偏差为0.5e-4*g
    double Q_diag[6] = {Q_wg, Q_wg, Q_wg, Q_wa, Q_wa, Q_wa};
    double Rk[6] = {Rlamt, Rl, Rh, Rvx, Rvy, Rvz};
    double PP0k[15] = {(0.1/(57))*(0.1/(57)), (0.1/(57))*(0.1/(57)), (0.1/(57))*(0.1/(57)),
                       0.01*0.01, 0.01*0.01, 0.01*0.01,0, 0, 0,
                       (0.1/(57*3600))*(0.1/(57*3600)), (0.1/(57*3600))*(0.1/(57*3600)), (0.1/(57*3600))*(0.1/(57*3600)),
                       ((1e-4)*G0)*((1e-4)*G0), ((1e-4)*G0)*((1e-4)*G0), ((1e-4)*G0)*((1e-4)*G0)};
    double wie = 7.292e-5;
    Q = MatDiag(Q_diag,6);
    R = MatDiag(Rk,6);
    if(PP == NULL)
        PP = MatDiag(PP0k,15);
    if(X == NULL){
        X = (double*)malloc(sizeof(double) * 15);
        memset(X,0,15 * sizeof(double));
    }
    
    double fe = Fn[0];
    double fn = Fn[1];
    double fu = Fn[2];
    //连续系统状态转换阵 F 的时间更新
    double secL2 = (1 / (sin(L) * sin(L)));
    double Rnh2 = (Rn + h) * (Rn + h);
    double Rmh2 = ((Rm + h) * (Rm + h));
    double tg = Tg[0];
    double ta = Ta[0];

    double* F = (double*)malloc(sizeof(double) * 225);
    double matrixF[15][15] = {
            {0, -wie * sin(L) - Ve * tan(L) / (Rn + h), Vn / (Rm + h), 0, 1 / (Rn + h), 0, -wie * sin(L), 0, -Ve / Rnh2, mahonyR[0], mahonyR[3], mahonyR[6], 0, 0, 0},
            {wie * sin(L) + Ve * tan(L) / (Rn + h), 0, wie * cos(L) + Ve / (Rn + h), -1 / (Rm + h), 0, 0, 0, 0, Vn / Rmh2, mahonyR[1], mahonyR[4], mahonyR[7], 0, 0, 0},
            {-Vn / (Rm + h), -wie * cos(L) - Ve / (Rn + h), 0, 0, -tan(L) / (Rn + h), 0, -wie * cos(L) - Ve * secL2 / (Rn + h), 0, Ve * tan(L) / Rnh2, mahonyR[2], mahonyR[5], mahonyR[8], 0, 0, 0},
            {0, -fu, fe, Vu / (Rm + h), -2 * wie * sin(L) - Ve * tan(L) / (Rn + h) - Ve * tan(L) / (Rn + h), Vn / (Rm + h), -2 * wie * cos(L) * Ve - Ve * Ve * secL2 / (Rn + h), 0, Ve * Ve * tan(L) / Rnh2 - Vn * Vu / Rmh2, 0, 0, 0, mahonyR[0], mahonyR[3], mahonyR[6]},
            {fu, 0, -fn, 2 * wie * sin(L) + Ve * tan(L) / (Rn + h), (Vn * tan(L) + Vu) / (Rn + h), 2 * wie * cos(L) + Ve / (Rn + h), (2 * wie * cos(L) + Ve * (secL2) / (Rn + h)) * Vn - 2 * wie * Vu * sin(L), 0, (Ve * Vn * tan(L) - Ve * Vu) / Rnh2, 0, 0, 0,mahonyR[1], mahonyR[4], mahonyR[7]},
            {-fe, fn, 0, -Vn / (Rm + h), -2 * wie * cos(L) - 2 * Ve / (Rn + h), 0, 2 * Ve * wie * sin(L), 0, Ve * Ve / Rnh2 + Vn * Vn / Rmh2, 0, 0, 0, mahonyR[2], mahonyR[5], mahonyR[8]},
            {0, 0, 0, 1 / (Rm + h), 0, 0, 0, 0, -Vn / Rmh2, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 1 / ((Rn + h) * cos(L)), 0, Ve * tan(L) / ((Rn + h) * cos(L)), 0, -Ve / (cos(L) * Rnh2), 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, -1 / tg, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1 / tg, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1 / tg, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1 / ta, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1 / ta, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1 / ta}
    };
    F = (double*)matrixF;//15*15

    double* G = (double*)malloc(sizeof(double) * 90);

    double matrixG[15][6] = {
            {mahonyR[0], mahonyR[3], mahonyR[6],0,0,0},
            {mahonyR[1], mahonyR[4], mahonyR[7],0,0,0},
            {mahonyR[2], mahonyR[5], mahonyR[8],0,0,0},
            {0, 0, 0, mahonyR[0], mahonyR[3], mahonyR[6]},
            {0, 0, 0, mahonyR[1], mahonyR[4], mahonyR[7]},
            {0, 0, 0, mahonyR[2], mahonyR[5], mahonyR[8]},
            {0,0,0,0,0,0},
            {0,0,0,0,0,0},
            {0,0,0,0,0,0},
            {0,0,0,0,0,0},
            {0,0,0,0,0,0},
            {0,0,0,0,0,0},
            {0,0,0,0,0,0},
            {0,0,0,0,0,0},
            {0,0,0,0,0,0}
    };
    G = (double*)matrixG;//15*6

    double* H = (double*)malloc(sizeof(double) * 90);
    double matrixH[6][15] = {
            {0, 0 , 0 , 0 , 0 , 0 , Rm+h , 0,0,0,0,0,0,0,0},
            {0 , 0 , 0 , 0 , 0 , 0 , 0 , (Rn+h)*cos(L) , 0 , 0 , 0 , 0 , 0 , 0 , 0},
            {0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 1 , 0 , 0 , 0 , 0 , 0 , 0},
            {0 , 0 , 0 , 1 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0},
            {0 , 0 , 0 , 0 , 1 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0},
            {0 , 0 , 0 , 0 , 0 , 1 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0},
    };
    H = (double*)matrixH;//6*15

    //连续系统离散化
    double* eye15 = MatEye(15);
    double* A = MatAdd(eye15,MatMulk(F,15,15,tao),15,15);
    double* B = MatMulk(G,15,6,tao);

    //卡尔曼滤波开始
   // printf("Now Here is the KF:\n");

    X = MatMul(A,15,15,X,15,1);
//    printf("X:\n");
//    MatShow(X,15,1);

    double* P = MatAdd(MatMul(MatMul(A,15,15,PP,15,15),15,15,MatT(A,15,15),15,15),MatMul(MatMul(B,15,6,Q,6,6),15,6,MatT(B,15,6),6,15),15,15);
//    printf("P:\n");
//    MatShow(P,15,15);

    double* Y = MatAdd(MatMul(MatMul(H,6,15,P,15,15),6,15,MatT(H,6,15),15,6),R,6,6);//6*6
//    printf("Y:\n");
//    MatShow(Y,6,6);

    double* K = MatMul(MatMul(P,15,15,MatT(H,6,15),15,6),15,6,MatInv(Y,6,6),6,6);//15*6
//    printf("K:\n");
//    MatShow(K,15,6);

    PP = MatMul(MatSub(eye15,MatMul(K,15,6,H,6,15),15,15),15,15,P,15,15);
//    printf("PP:\n");
//    MatShow(PP,15,15);

    double* Z = Dpv;
//    char z_string[1000];
//    sprintf(z_string,"%f,%f,%f,%f,%f,%f\n",Z[0],Z[1],Z[2],Z[3],Z[4],Z[5]);
//    fprintf(f_z,z_string,1000);
//    fclose(f_z);

    double* W = MatSub(Z,MatMul(H,6,15,X,15,1),6,1);
//    printf("W:\n");
//    MatShow(W,6,1);
//    char w_string[1000];
//    sprintf(w_string,"%f,%f,%f,%f,%f,%f\n",W[0],W[1],W[2],W[3],W[4],W[5]);
//    fprintf(f_w,w_string,1000);
//    fclose(f_w);

    double* XX = MatAdd(X,MatMul(K,15,6,W,6,1),15,1);
//    printf("XX:\n");
//    MatShow(XX,15,1);

    return XX;
}
void setNull(){
    free(X);
    X = NULL;
    PP = NULL;
}