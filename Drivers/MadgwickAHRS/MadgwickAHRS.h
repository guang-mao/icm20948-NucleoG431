//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h

//#ifdef __cplusplus
//extern "C" {
//#endif

#include "main.h"
//----------------------------------------------------------------------------------------------------
// Variable declaration

//extern volatile float beta;				// algorithm gain
//extern volatile float q[4];       // quaternion of sensor frame relative to auxiliary frame
//extern volatile uint8_t anglesComputed;
//extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame

//---------------------------------------------------------------------------------------------------
// Function declarations

//void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
//void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);

class MadgwickAHRS
{
  private:
    double delta_t = 0;           // Used to control display output rate
    uint32_t now = 0;             // used to calculate integration interval
    uint32_t last_update = 0;     // used to calculate integration interval
    static float invSqrt(float x);
    float beta;                   // algorithm gain
    float q0;
    float q1;
    float q2;
    float q3;                     // quaternion of sensor frame relative to auxiliary frame
    float roll;
    float pitch;
    float yaw;
    char anglesComputed;
    void computeAngles();

  public:
    MadgwickAHRS(void);
    void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);

    float getRoll()
    {
      if (!anglesComputed) computeAngles();
      return roll * 57.29578f;
    }

    float getPitch()
    {
      if (!anglesComputed) computeAngles();
      return pitch * 57.29578f;
    }

    float getYaw()
    {
      if (!anglesComputed) computeAngles();
      return yaw * 57.29578f + 180.0f;
    }

    float getRollRadians()
    {
      if (!anglesComputed) computeAngles();
      return roll;
    }

    float getPitchRadians()
    {
      if (!anglesComputed) computeAngles();
      return pitch;
    }

    float getYawRadians()
    {
      if (!anglesComputed) computeAngles();
      return yaw;
    }

    void getQuaternion(float *w, float *x, float *y, float *z)
    {
      *w = q0;
      *x = q1;
      *y = q2;
      *z = q3;
    }
};

//#ifdef __cplusplus
//}
//#endif

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
