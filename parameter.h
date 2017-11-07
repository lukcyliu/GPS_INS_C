//
// Created by aos on 17-3-8.
//

#ifndef GPS_INS_VARIABLE_H
#define GPS_INS_VARIABLE_H
struct Acceleration{
    double x;
    double y;
    double z;
    long t;
};
struct Angle{
    double x;
    double y;
    double z;
};
struct AngularVelocity{
    double x;
    double y;
    double z;
    long t;
};
struct Position{
    double x;
    double y;
    double z;
    long t;
};
struct Velocity{
    double x;
    double y;
    double z;
    long t;
};
#endif //GPS_INS_VARIABLE_H
