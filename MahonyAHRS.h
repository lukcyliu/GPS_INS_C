//=====================================================================================================
// MahonyAHRS.h
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
#ifndef MahonyAHRS_h
#define MahonyAHRS_h

//----------------------------------------------------------------------------------------------------
// Variable declaration

extern volatile double twoKp;			// 2 * proportional gain (Kp)
extern volatile double twoKi;			// 2 * integral gain (Ki)

//---------------------------------------------------------------------------------------------------
// Function declarations

void MahonyAHRSupdate(double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz);
void MahonyAHRSupdateIMU(double gx, double gy, double gz, double ax, double ay, double az);

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
