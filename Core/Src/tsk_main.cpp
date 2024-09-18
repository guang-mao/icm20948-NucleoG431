/*
 * tsk_main.cpp
 *
 *  Created on: 2024年8月15日
 *      Author: 20210113
 */
#include "main.h"
#include "ICM_20948.h"
#include "math.h"
#include "Adafruit_AHRS_Madgwick.h"
#include "uNavAHRS.h"

/* Private variables ---------------------------------------------------------*/
float roll, pitch, yaw;

// unit g
float Fa[3][3] = {
  {+1.00021f, +0.00201f, +0.02555f},
  {+0.02085f, +1.00252f, +0.01340f},
  {-0.01943f, -0.00301f, +1.00331f}
};
float Ka[3][3] = {
  {+0.99934f, -0.00208f, -0.02542f},
  {-0.02104f, +0.99749f, -0.01279f},
  {+0.01929f, +0.00295f, +0.99617f}
};
float delta_acc[3] = {-0.00612f, -0.02053f, +0.00627f};

float filter_acc[3] = {0.0f};
float acc_result[3] = {0.0f};

// unit rad/s
float g_offset[3] = {-0.538f, -0.048, -0.249f};
float gyr_result[3] = {0.0f};
// unit uT
float mag_result[3] = {0.0f};

#if 1
//float mag_hard_iron[3] = {0.00f, 0.00f, 0.00f};
float mag_hard_iron[3] = {-12.139564f, +31.953657f, -18.385916f};
float mag_soft_iron[3][3] = {
    { +1.809586f, -0.015683f, +0.041884f},
    { -0.015683f, +1.784187f, +0.006941f},
    { +0.041884f, +0.006941f, +1.740622f},
};
#else
float mag_hard_iron[3] = {-8.64f, 17.64f, -11.71f};
float mag_soft_iron[3][3] = {
    { +1.059f, -0.070f, +0.002f},
    { -0.070f, +0.908f, +0.022f},
    { +0.002f, +0.022f, +1.045f},
};
#endif

float mag_field = 22.13f;

ICM_20948_I2C myICM;

BOOTSTAGE_t bootstage = INIT;

uint32_t bootTick = 0;

Adafruit_Madgwick madgwick_filter(0.5f);

// a uNavAHRS object
uNavAHRS ekf_Filter;

/* Private function prototypes -----------------------------------------------*/
void inverse3x3(float matrix[3][3], float result[3][3]);

void Setup()
{
  // DWT tick Init...
  if ( DWT_Init() )
  {
    printf("DWT Initialize failed...\r\n");
  }

  ekf_Filter.setInitializationDuration(5000);

  // ICM-20948 Init...
  bool initialized = false;

  while ( !initialized )
  {
    myICM.begin();
    printf("Initialization of the sensor returned: ");
    printf(myICM.statusString());
    printf("\r\n");
    if ( myICM.status != ICM_20948_Stat_Ok )
    {
      printf("Trying again...\r\n");
      HAL_Delay(500);
    }
    else
    {
      initialized = true;
    }
  }

  // Here we are doing a SW reset to make sure the device starts in a known state
  myICM.swReset();
  if ( myICM.status != ICM_20948_Stat_Ok )
  {
    printf("Software Reset returned: ");
    printf(myICM.statusString());
    printf("\r\n");
  }
  HAL_Delay(250);

  // Now wake the sensor up
  myICM.sleep(false);
  myICM.lowPower(false);

  // The next few configuration functions accept a bit-mask of sensors for which the settings should be applied.

  // Set Gyro and Accelerometer to a particular sample mode
  // options: ICM_20948_Sample_Mode_Continuous
  //          ICM_20948_Sample_Mode_Cycled
  myICM.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous);
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    printf("setSampleMode returned: ");
    printf(myICM.statusString());
    printf("\r\n");
  }

  // Set full scale ranges for both acc and gyr
  ICM_20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors

  myFSS.a = gpm4; // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                  // gpm2
                  // gpm4
                  // gpm8
                  // gpm16

  myFSS.g = dps500; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                    // dps250
                    // dps500
                    // dps1000
                    // dps2000

  myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
  if ( myICM.status != ICM_20948_Stat_Ok )
  {
    printf("setFullScale returned: ");
    printf(myICM.statusString());
    printf("\r\n");
  }

  // Set up Digital Low-Pass Filter configuration
  ICM_20948_dlpcfg_t myDLPcfg;    // Similar to FSS, this uses a configuration structure for the desired sensors
  myDLPcfg.a = acc_d246bw_n265bw; // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)
                                  // acc_d246bw_n265bw      - means 3db bandwidth is 246 hz and nyquist bandwidth is 265 hz
                                  // acc_d111bw4_n136bw
                                  // acc_d50bw4_n68bw8
                                  // acc_d23bw9_n34bw4
                                  // acc_d11bw5_n17bw
                                  // acc_d5bw7_n8bw3        - means 3 db bandwidth is 5.7 hz and nyquist bandwidth is 8.3 hz
                                  // acc_d473bw_n499bw

  myDLPcfg.g = gyr_d196bw6_n229bw8; // (ICM_20948_GYRO_CONFIG_1_DLPCFG_e)
                                    // gyr_d196bw6_n229bw8
                                    // gyr_d151bw8_n187bw6
                                    // gyr_d119bw5_n154bw3
                                    // gyr_d51bw2_n73bw3
                                    // gyr_d23bw9_n35bw9
                                    // gyr_d11bw6_n17bw8
                                    // gyr_d5bw7_n8bw9
                                    // gyr_d361bw4_n376bw5

  myICM.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg);
  if ( myICM.status != ICM_20948_Stat_Ok )
  {
    printf("setDLPcfg returned: ");
    printf(myICM.statusString());
    printf("\r\n");
  }

  // Choose whether or not to use DLPF
  // Here we're also showing another way to access the status values, and that it is OK to supply individual sensor masks to these functions
  ICM_20948_Status_e accDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Acc, false);
  ICM_20948_Status_e gyrDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Gyr, false);
  printf("Enable DLPF for Accelerometer returned: ");
  printf(myICM.statusString(accDLPEnableStat));
  printf("\r\n");
  printf("Enable DLPF for Gyroscope returned: ");
  printf(myICM.statusString(gyrDLPEnableStat));
  printf("\r\n");

  // Choose whether or not to start the magnetometer
  myICM.startupMagnetometer();
  if ( myICM.status != ICM_20948_Stat_Ok )
  {
    printf("startupMagnetometer returned: ");
    printf(myICM.statusString());
    printf("\r\n");
  }

  printf("\r\n");
  printf("Configuration complete!\r\n");

  bootTick = HAL_GetTick();
}

void Loop()
{
  if ( myICM.dataReady() )
  {
    myICM.getAGMT();  // The values are only updated when you call 'getAGMT'

    float gyr[3] = { myICM.gyrX(), myICM.gyrY(), myICM.gyrZ()};
    for ( uint8_t i = 0; i < 3; i++ )
    {
      gyr_result[i] = gyr[i] - g_offset[i];
    }

    float acc[3] = { myICM.accX() / 1000.0f, myICM.accY() / 1000.0f, myICM.accZ() / 1000.0f};

    float acc_neutral[3] = {0.0f};

    for ( uint8_t i = 0; i < 3; i++ )
    {
      acc_neutral[i] = acc[i] - delta_acc[i];
    }

    for ( uint8_t j = 0; j < 3; j++ )
    {
      acc_result[j] = Ka[j][0] * acc_neutral[0] + Ka[j][1] * acc_neutral[1] + Ka[j][2] * acc_neutral[2];
    }

//    printf("Acc X axis: %.5f\r\n", acc_result[0]);
//    printf("Acc Y axis: %.5f\r\n", acc_result[1]);
//    printf("Acc Z axis: %.5f\r\n", acc_result[2]);
   //    printf("Pitch: %.1f, ", pitch);
   //    printf("Raw Yaw: %.1f, ", raw_yaw);
   //    printf("Yaw: %.1f\n", yaw);

#if 1
    float mag[3] = {myICM.magX(), myICM.magY(), myICM.magZ()};

    float mag_neutral[3];

    for ( uint8_t i = 0; i < 3; i++ )
    {
      mag_neutral[i] = mag[i] - mag_hard_iron[i];
    }

    for ( uint8_t j = 0; j < 3; j++ )
    {
      mag_result[j] = mag_soft_iron[j][0] * mag_neutral[0] + mag_soft_iron[j][1] * mag_neutral[1] + mag_soft_iron[j][2] * mag_neutral[2];
    }
#endif

#if 0
//     'Raw' values to match expectation of MOtionCal
    printf("Raw:");
    printf("0"); printf(",");// myICM.agmt.acc.axes.x
    printf("0"); printf(",");
    printf("0"); printf(",");
    printf("0"); printf(",");// myICM.agmt.gyr.axes.x
    printf("0"); printf(",");
    printf("0"); printf(",");
    printf("%d", myICM.agmt.mag.axes.x); printf(",");
    printf("%d", myICM.agmt.mag.axes.y); printf(",");
    printf("%d", myICM.agmt.mag.axes.z); printf("\r\n");

    float i = 0.0f;
    // unified data
    printf("Uni:");
    printf("%.1f", i); printf(","); //myICM.accX() * 9.81 / 1000.0f
    printf("%.1f", i); printf(",");
    printf("%.1f", i); printf(",");
    printf("%.4f", i); printf(","); // myICM.gyrX()
    printf("%.4f", i); printf(",");
    printf("%.4f", i); printf(",");
    printf("%.1f", mag_result[0]); printf(",");
    printf("%.1f", mag_result[1]); printf(",");
    printf("%.1f", mag_result[2]); printf("\r\n");
#else
    static uint32_t lastTick = 0;

    uint32_t now = micros();

    float elapse_dt = ( now - lastTick ) / 1000000.0f;

#if 1
//    madgwick_filter.update(gyr_result[0], gyr_result[1], gyr_result[2], acc_result[0], acc_result[1], acc_result[2], mag[0], mag[1], mag[2], elapse_dt);

    // extended kalman filter...
    float gyro_rad[3], acc_mss[3];
    for ( uint8_t i = 0; i < 3; i++ )
    {
      gyro_rad[i] = gyr_result[i] * 0.0174533f;
      acc_mss[i] = acc_result[i] * 9.81f;
    }
//    acc_mss[2] = -acc_mss[2];
    ekf_Filter.update(gyro_rad[0], gyro_rad[1], gyro_rad[2], \
                      acc_mss[0], acc_mss[1], acc_mss[2], \
                      mag[0], mag[1], mag[2]);

#else
    madgwick_filter.updateIMU(myICM.gyrX(), myICM.gyrY(), myICM.gyrZ(), myICM.accX(), myICM.accY(), myICM.accZ(), elapse_dt);
#endif

    // Roll
//    roll = madgwick_filter.getRoll();
//
//    // Pitch
//    pitch = madgwick_filter.getPitch();
//
//    // Yaw
//    yaw = madgwick_filter.getYaw();

//    float raw_yaw = atan2f(mag_result[0], mag_result[1]) * 180.0 / M_PI + 180.0f;

//    printf("------Madgwick-----\r\n");
//    printf("Roll: %.1f, ", roll);
//    printf("Pitch: %.1f, ", pitch);
////    printf("Raw Yaw: %.1f, ", raw_yaw);
//    printf("Yaw: %.1f\n", yaw);

    // Roll
    float tmp = 0.0f;

    tmp = ekf_Filter.getRoll_rad() * 180.0f / M_PI + 180.0f;

    if ( tmp > 180.0f )
      tmp -= 360.0f;

    roll = tmp;

    // Pitch
    pitch = ekf_Filter.getPitch_rad() * 180.0f / M_PI;

    // Yaw
    yaw = ekf_Filter.getYaw_rad() * 180.0f / M_PI;

//    printf("------EKF-----\r\n");
    printf("Roll: %.1f, ", roll);
    printf("Pitch: %.1f, ", pitch);
    printf("Yaw: %.1f\n", yaw);

    lastTick = now;
#endif
    HAL_Delay(5);
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);

  }
  else
  {
    HAL_Delay(500);
  }

}


void inverse3x3(float matrix[3][3], float result[3][3])
{
    float det = matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) -
                matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0]) +
                matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);

    if (det == 0) {
        printf("Matrix is singular and cannot be inverted.\n");
        return;
    }

    float invDet = 1.0 / det;

    result[0][0] = (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) * invDet;
    result[0][1] = (matrix[0][2] * matrix[2][1] - matrix[0][1] * matrix[2][2]) * invDet;
    result[0][2] = (matrix[0][1] * matrix[1][2] - matrix[0][2] * matrix[1][1]) * invDet;
    result[1][0] = (matrix[1][2] * matrix[2][0] - matrix[1][0] * matrix[2][2]) * invDet;
    result[1][1] = (matrix[0][0] * matrix[2][2] - matrix[0][2] * matrix[2][0]) * invDet;
    result[1][2] = (matrix[0][2] * matrix[1][0] - matrix[0][0] * matrix[1][2]) * invDet;
    result[2][0] = (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]) * invDet;
    result[2][1] = (matrix[0][1] * matrix[2][0] - matrix[0][0] * matrix[2][1]) * invDet;
    result[2][2] = (matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0]) * invDet;
}

