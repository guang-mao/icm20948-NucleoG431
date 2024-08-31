/*
 * tsk_main.cpp
 *
 *  Created on: 2024年8月15日
 *      Author: 20210113
 */
#include "main.h"
#include "ICM_20948.h"
#include "math.h"
#include "MadgwickAHRS.h"

/* Private variables ---------------------------------------------------------*/
float roll, pitch, yaw;
// unit g
float ax, ay, az;
// unit rad/s
float gx, gy, gz;
float g_offset[3] = {-0.718654335f, -0.0836905688f, -0.171692878f};
// unit uT
float mx, my, mz;
float mag_offset[3] = {-6.58f, 20.96f, -11.87f};
float mag_softiron_matrix[3][3] = {
    { 1.039f, 0.000f, 0.001f},
    { 0.000f, 1.003f, -0.002f},
    { 0.001f, -0.002f, 0.959f},
};
float mag_field = 22.11f;
// unit second
float dt = 0.0f;

ICM_20948_I2C myICM;

BOOTSTAGE_t bootstage = INIT;

uint32_t bootTick = 0;

MadgwickAHRS filter;

/* Private function prototypes -----------------------------------------------*/
void printScaledAGMT(ICM_20948_I2C *sensor);

void Setup()
{
  // DWT tick Init...
  if ( DWT_Init() )
  {
    printf("DWT Initialize failed...\n");
  }

  // ICM-20948 Init...
  bool initialized = false;

  while ( !initialized )
  {
    myICM.begin();

    if ( myICM.status != ICM_20948_Stat_Ok )
    {
      printf("Initialized failed!...\n");
      HAL_Delay(50);
    }
    else
    {
      initialized = true;
      printf("Initialized successful!...\n");
    }
  }

  // Here we are doing a SW reset to make sure the device starts in a known state
  myICM.swReset();
  if ( myICM.status != ICM_20948_Stat_Ok )
  {
    printf("Software Reset returned: ");
    printf(myICM.statusString());
    printf("\n");
  }

  HAL_Delay(100);

  // Now wake the sensor up
  myICM.sleep(false);
  myICM.lowPower(false);

  // The next few configuration functions accept a bit-mask of sensors for which the settings should be applied.

  // Set Gyro and Accelerometer to a particular sample mode
  // options: ICM_20948_Sample_Mode_Continuous
  //          ICM_20948_Sample_Mode_Cycled
  myICM.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous);
  if ( myICM.status != ICM_20948_Stat_Ok )
  {
    printf("setSampleMode returned: ");
    printf(myICM.statusString());
    printf("\n");
  }

  // Set full scale ranges for both acc and gyr
  ICM_20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors

  myFSS.a = gpm4; // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                  // gpm2
                  // gpm4
                  // gpm8
                  // gpm16

  myFSS.g = dps250; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                    // dps250
                    // dps500
                    // dps1000
                    // dps2000

  myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);

  if ( myICM.status != ICM_20948_Stat_Ok )
  {
    printf("setFullScale returned: ");
    printf(myICM.statusString());
    printf("\n");
  }

  // Set up Digital Low-Pass Filter configuration
  ICM_20948_dlpcfg_t myDLPcfg;    // Similar to FSS, this uses a configuration structure for the desired sensors
  myDLPcfg.a = acc_d473bw_n499bw; // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)
                                  // acc_d246bw_n265bw      - means 3db bandwidth is 246 hz and nyquist bandwidth is 265 hz
                                  // acc_d111bw4_n136bw
                                  // acc_d50bw4_n68bw8
                                  // acc_d23bw9_n34bw4
                                  // acc_d11bw5_n17bw
                                  // acc_d5bw7_n8bw3        - means 3 db bandwidth is 5.7 hz and nyquist bandwidth is 8.3 hz
                                  // acc_d473bw_n499bw

  myDLPcfg.g = gyr_d361bw4_n376bw5; // (ICM_20948_GYRO_CONFIG_1_DLPCFG_e)
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
    printf("\n");
  }

  // Choose whether or not to use DLPF
  // Here we're also showing another way to access the status values, and that it is OK to supply individual sensor masks to these functions
  ICM_20948_Status_e accDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Acc, false);
  ICM_20948_Status_e gyrDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Gyr, false);
  printf("Enable DLPF for Accelerometer returned: ");
  printf(myICM.statusString(accDLPEnableStat));
  printf("\n");
  printf("Enable DLPF for Gyroscope returned: ");
  printf(myICM.statusString(gyrDLPEnableStat));
  printf("\n");

  // Choose whether or not to start the magnetometer
  myICM.startupMagnetometer();

  if ( myICM.status != ICM_20948_Stat_Ok )
  {
    printf("startupMagnetometer returned: ");
    printf(myICM.statusString());
    printf("\n");
  }

  //
  bootTick = HAL_GetTick();
}

void Loop()
{
  if ( myICM.dataReady() )
  {
    float raw_acc[3] = {0.0f}, raw_gyr[3] = {0.0f}, raw_mag[3] = {0.0f};

    myICM.getAGMT();  // The values are only updated when you call 'getAGMT'

    raw_acc[0] = myICM.accX();

    raw_acc[1] = myICM.accY();

    raw_acc[2] = myICM.accZ();

    raw_gyr[0] = myICM.gyrX();

    raw_gyr[1] = myICM.gyrY();

    raw_gyr[2] = myICM.gyrZ();

    raw_mag[0] = myICM.magX();

    raw_mag[1] = myICM.magY();

    raw_mag[2] = myICM.magZ();

//    float x = (float) ( myICM.agmt.mag.axes.x - mag_offset[0] );
//
//    float y = (float) ( myICM.agmt.mag.axes.y - mag_offset[1] );
//
//    float z = (float) ( myICM.agmt.mag.axes.z - mag_offset[2] );
//
//    raw_mag[0] = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
//
//    raw_mag[1] = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
//
//    raw_mag[2] = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

//    printf("Raw:");
//
//    printf("%d", myICM.agmt.acc.axes.x);
//    printf(",");
//    printf("%d", myICM.agmt.acc.axes.y);
//    printf(",");
//    printf("%d", myICM.agmt.acc.axes.z);
//    printf(",");
//
//    printf("%d", myICM.agmt.gyr.axes.x);
//    printf(",");
//    printf("%d", myICM.agmt.gyr.axes.y);
//    printf(",");
//    printf("%d", myICM.agmt.gyr.axes.z);
//    printf(",");

//    printf("Raw:");
    printf("%d", myICM.agmt.mag.axes.x); // xyz xzy yxz yzx zyx
    printf(",");
    printf("%d", myICM.agmt.mag.axes.y);
    printf(",");
    printf("%d", myICM.agmt.mag.axes.z);
    printf("\n");

//    printf("Cal:");
//    printf("%.1f", raw_mag[0]); // xyz xzy yxz yzx zyx
//    printf(" ");
//    printf("%.1f", raw_mag[1]);
//    printf(" ");
//    printf("%.1f", raw_mag[2]);
//    printf("\n");

//    printf("Uni:");
//
//    printf("%f", raw_acc[0]);
//    printf(",");
//    printf("%f", raw_acc[1]);
//    printf(",");
//    printf("%f", raw_acc[2]);
//    printf(",");
//
//    printf("%f", raw_gyr[0]);
//    printf(",");
//    printf("%f", raw_gyr[1]);
//    printf(",");
//    printf("%f", raw_gyr[2]);
//    printf(",");
//
//    printf("%f", raw_mag[0]);
//    printf(",");
//    printf("%f", raw_mag[1]);
//    printf(",");
//    printf("%f", raw_mag[2]);
//    printf("\n");

    switch ( bootstage )
    {
      case INIT: {
        bootstage = CALIBRA;
        bootTick = HAL_GetTick();
        break;
      }
      case CALIBRA: {
//        if ( ( HAL_GetTick() - bootTick ) > 1000 )
        {
          bootstage = RUN;
        }
//        else
//        {
//          g_offset[0] = g_offset[0] * 15.0f / 16.0f +  raw_gyr[0] / 16.0f;
//          g_offset[1] = g_offset[1] * 15.0f / 16.0f +  raw_gyr[1] / 16.0f;
//          g_offset[2] = g_offset[2] * 15.0f / 16.0f +  raw_gyr[2] / 16.0f;
//
//          m_offset[0] = m_offset[0] * 15.0f / 16.0f +  raw_mag[0] / 16.0f;
//          m_offset[1] = m_offset[1] * 15.0f / 16.0f +  raw_mag[1] / 16.0f;
//          m_offset[2] = m_offset[2] * 15.0f / 16.0f +  raw_mag[2] / 16.0f;
//        }
        break;
      }
      case RUN: {
#if 0
          ax = raw_acc[0];

          ay = raw_acc[1];

          az = raw_acc[2];

          gx = ( raw_gyr[0] - g_offset[0] );

          gy = ( raw_gyr[1] - g_offset[1] );

          gz = ( raw_gyr[2] - g_offset[2] );

          mx = raw_mag[0];

          my = raw_mag[1];

          mz = raw_mag[2];

          filter.MadgwickAHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz);

          // Roll
          roll = filter.getRoll();

          // Pitch
          pitch = filter.getPitch();

          // Yaw
          yaw = filter.getYaw();
#endif
        break;
      }
    }

    //printScaledAGMT(&myICM);      // This function takes into account the scale settings from when the measurement was made to calculate the values with units

    HAL_Delay(10);

    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
  }
  else
  {
    HAL_Delay(500);
  }
}

// Below here are some helper functions to print the data nicely!

void printPaddedInt16b(int16_t val)
{
  if (val > 0)
  {
    printf(" ");
    if (val < 10000)
    {
      printf("0");
    }
    if (val < 1000)
    {
      printf("0");
    }
    if (val < 100)
    {
      printf("0");
    }
    if (val < 10)
    {
      printf("0");
    }
  }
  else
  {
    printf("-");
    if (abs(val) < 10000)
    {
      printf("0");
    }
    if (abs(val) < 1000)
    {
      printf("0");
    }
    if (abs(val) < 100)
    {
      printf("0");
    }
    if (abs(val) < 10)
    {
      printf("0");
    }
  }
  printf("%d", abs(val));
}

void printRawAGMT(ICM_20948_AGMT_t agmt)
{
  printf("RAW. Acc [ ");
  printPaddedInt16b(agmt.acc.axes.x);
  printf(", ");
  printPaddedInt16b(agmt.acc.axes.y);
  printf(", ");
  printPaddedInt16b(agmt.acc.axes.z);
  printf(" ], Gyr [ ");
  printPaddedInt16b(agmt.gyr.axes.x);
  printf(", ");
  printPaddedInt16b(agmt.gyr.axes.y);
  printf(", ");
  printPaddedInt16b(agmt.gyr.axes.z);
  printf(" ], Mag [ ");
  printPaddedInt16b(agmt.mag.axes.x);
  printf(", ");
  printPaddedInt16b(agmt.mag.axes.y);
  printf(", ");
  printPaddedInt16b(agmt.mag.axes.z);
  printf(" ], Tmp [ ");
  printPaddedInt16b(agmt.tmp.val);
  printf(" ]");
  printf("\n");
}

void printFormattedFloat(float val, uint8_t leading, uint8_t decimals)
{
  float aval = abs(val);
  if (val < 0)
  {
    printf("-");
  }
  else
  {
    printf(" ");
  }
  for (uint8_t indi = 0; indi < leading; indi++)
  {
    uint32_t tenpow = 0;
    if (indi < (leading - 1))
    {
      tenpow = 1;
    }
    for (uint8_t c = 0; c < (leading - 1 - indi); c++)
    {
      tenpow *= 10;
    }
    if (aval < tenpow)
    {
      printf("0");
    }
    else
    {
      break;
    }
  }
  if ( val < 0 )
  {
    printf("%f%d",-val, decimals);
  }
  else
  {
    printf("%f%d",val, decimals);
  }
}

void printScaledAGMT(ICM_20948_I2C *sensor)
{
  printf("Scaled. Acc (g) [ ");
  printFormattedFloat(sensor->accX()/1000.0f, 5, 2);
  printf(", ");
  printFormattedFloat(sensor->accY()/1000.0f, 5, 2);
  printf(", ");
  printFormattedFloat(sensor->accZ()/1000.0f, 5, 2);
  printf(" ], Gyr (rad/s) [ ");
  printFormattedFloat(sensor->gyrX()*0.017453f, 5, 2);
  printf(", ");
  printFormattedFloat(sensor->gyrY()*0.017453f, 5, 2);
  printf(", ");
  printFormattedFloat(sensor->gyrZ()*0.017453f, 5, 2);
  printf(" ], Mag (uT) [ ");
  printFormattedFloat(sensor->magX(), 5, 2);
  printf(", ");
  printFormattedFloat(sensor->magY(), 5, 2);
  printf(", ");
  printFormattedFloat(sensor->magZ(), 5, 2);
  printf(" ], Tmp (C) [ ");
  printFormattedFloat(sensor->temp(), 5, 2);
  printf(" ]");
  printf("\n");
}
