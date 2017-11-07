//=====================================================================================================
// MahonyAHRS.c
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files

#include "MahonyAHRS.h"
#include <math.h>

//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq	0.2f			// sample frequency in Hz
#define twoKpDef	2	// 2 * proportional gain
#define twoKiDef	0.01f	// 2 * integral gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile double twoKp = twoKpDef;											// 2 * proportional gain (Kp)
volatile double twoKi = twoKiDef;											// 2 * integral gain (Ki)
volatile double q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame
volatile double integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki

//---------------------------------------------------------------------------------------------------
// Function declarations

double invSqrt(double x);

//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

void MahonyAHRSupdate(double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz) {
    double recipNorm;
    double q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    double hx, hy, bx, bz;
    double vx, vy, vz, wx, wy, wz;
    double ex, ey, ez;
    double qa, qb, qc;

    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az);
        return;
    }

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = sqrt(ax * ax + ay * ay + az * az);
        recipNorm = 1.0f / recipNorm;
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = sqrt(mx * mx + my * my + mz * mz);
        recipNorm = 1.0f / recipNorm;
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        // Reference direction of Earth's magnetic field
        hx = 2.0f * mx * (0.5f - q2q2 - q3q3) + 2.0f * my * (q1q2 - q0q3) + 2.0f * mz * (q1q3 + q0q2);
        hy = 2.0f * mx * (q1q2 + q0q3) + 2.0f * my * (0.5f - q1q1 - q3q3) + 2.0f * mz * (q2q3 - q0q1);
        bx = sqrt(hx * hx + hy * hy);
        //bx = 0;
        bz = 2.0f * mx * (q1q3 - q0q2) + 2.0f * my * (q2q3 + q0q1) + 2.0f * mz * (0.5f - q1q1 - q2q2);

        // Estimated direction of gravity and magnetic field
        vx = 2.0f * (q0q2 - q1q3);
        vy = -2.0f * (q0q1 + q2q3);
        vz = -(q0q0 - q1q1 - q2q2 + q3q3);
        wx = 0.5f * (2.0f * bx * (0.5f - q2q2 - q3q3) + 2.0f * bz * (q1q3 - q0q2));
        wy = 0.5f * (2.0f * bx * (q1q2 - q0q3) + 2.0f * bz * (q0q1 + q2q3));
        wz = 0.5f * (2.0f * bx * (q0q2 + q1q3) + 2.0f * bz * (0.5f - q1q1 - q2q2));

        // Error is sum of cross product between estimated direction and measured direction of field vectors
        ex = (ay * vz - az * vy) + (my * wz - mz * wy);
        ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
        ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

        // Compute and apply integral feedback if enabled
        if(twoKi > 0.0f) {
            integralFBx += ex * sampleFreq;	// integral error scaled by Ki
            integralFBy += ey * sampleFreq;
            integralFBz += ez * sampleFreq;
        }
        else {
            integralFBx = 0.0f;	// prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKp * ex + twoKi * integralFBx;
        gy += twoKp * ey + twoKi * integralFBy;
        gz += twoKp * ez + twoKi * integralFBz;
    }

    // Integrate rate of change of quaternion
//    gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
//    gy *= (0.5f * (1.0f / sampleFreq));
//    gz *= (0.5f * (1.0f / sampleFreq));
    qa = q1;
    qb = q2;
    qc = q3;
    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * (0.5f * sampleFreq);
    q1 = qa + (q0 * gx + qb * gz - qc * gy) * (0.5f * sampleFreq);
    q2 = qb + (q0 * gy - qa * gz + qc * gx) * (0.5f * sampleFreq);
    q3 = qc + (q0 * gz + qa * gy - qb * gx) * (0.5f * sampleFreq);

    // Normalise quaternion
    recipNorm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    recipNorm = 1.0f / recipNorm;
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MahonyAHRSupdateIMU(double gx, double gy, double gz, double ax, double ay, double az) {
    double recipNorm;
    double halfvx, halfvy, halfvz;
    double halfex, halfey, halfez;
    double qa, qb, qc;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Compute and apply integral feedback if enabled
        if(twoKi > 0.0f) {
            integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
            integralFBy += twoKi * halfey * (1.0f / sampleFreq);
            integralFBz += twoKi * halfez * (1.0f / sampleFreq);
            gx += integralFBx;	// apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        }
        else {
            integralFBx = 0.0f;	// prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
    gy *= (0.5f * (1.0f / sampleFreq));
    gz *= (0.5f * (1.0f / sampleFreq));
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

double invSqrt(double x) {
    double halfx = 0.5f * x;
    double y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(double*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

//====================================================================================================
// END OF CODE
//====================================================================================================