//
// Created by aos on 17-3-13.
//

#ifndef GPS_INS_FILESTRUCT_H
#define GPS_INS_FILESTRUCT_H
struct Data{
	int Minute;
	double Second;

    double acc_x;
    double acc_y;
    double acc_z;//加速度计
    double gyo_x;
    double gyo_y;
    double gyo_z;//陀螺仪
    double angle_x;
    double angle_y;
    double angle_z;//姿态角
    int m_x;
    int m_y;
    int m_z;//磁力计

    int sDStatus_0;
    int sDStatus_1;
    int sDStatus_2;
    int sDStatus_3;
    int lPressure;
    double lAltitude;
    double Longitude;
    double Lattitude;
    double Height;
    double Yaw;
    double GPSV;

	double Q_q0;
	double Q_q1;
	double Q_q2;
	double Q_q3;

	double SN;
	double PDOP;
	double HDOP;
	double VDOP;
};
struct slideRes{
    double slideAcc_x;
    double slideAcc_y;
    double slideAcc_z;
    double slideGyo_x;
    double slideGyo_y;
    double slideGyo_z;
    double slideMag_x;
    double slideMag_y;
    double slideMag_z;
    double slideJy_yaw;
    double slideGps_yaw;
    double slideGps_v;
};
#endif //GPS_INS_FILESTRUCT_H
