#include "main.h"
#include "ICM_20948.h"

ICM_20948_I2C myICM;

// After calibration, the following two parameters will be obtained.
float mag_hard_iron[3] = { 0.0f, +0.0f, 0.0f};
float mag_soft_iron[3][3] = {
    { 1.0f, 0.0f, 0.0f},
    { 0.0f, 1.0f, 0.0f},
    { 0.0f, 0.0f, 1.0f},
};

// [Unit]: uT
float mag_result[3] = { 0.0f, 0.0f, 0.0f};

void Setup(void)
{
  // DWT tick Init...
  if ( DWT_Init() )
  {
    printf("DWT Initialize failed...\r\n");
  }

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
  myDLPcfg.a = acc_d50bw4_n68bw8; // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)
                                  // acc_d246bw_n265bw      - means 3db bandwidth is 246 hz and nyquist bandwidth is 265 hz
                                  // acc_d111bw4_n136bw
                                  // acc_d50bw4_n68bw8
                                  // acc_d23bw9_n34bw4
                                  // acc_d11bw5_n17bw
                                  // acc_d5bw7_n8bw3        - means 3 db bandwidth is 5.7 hz and nyquist bandwidth is 8.3 hz
                                  // acc_d473bw_n499bw

  myDLPcfg.g = gyr_d51bw2_n73bw3; // (ICM_20948_GYRO_CONFIG_1_DLPCFG_e)
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
  ICM_20948_Status_e accDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Acc, true);
  ICM_20948_Status_e gyrDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Gyr, true);
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
}

void Loop(void)
{
  if ( myICM.dataReady() )
  {
    myICM.getAGMT();  // The values are only updated when you call 'getAGMT'

    // Unit: [uT]
    float mag[3] = { myICM.magX(), myICM.magY(), myICM.magZ()};

    #if 1
    /*        [ Not calibrated ]
     *  After printing this format through Serial,
     *  save it as a .txt file. Obtaining parameters
     *  through Magneto12.exe.
     */
    printf("%.1f", mag[0]); printf(" ");
    printf("%.1f", mag[1]); printf(" ");
    printf("%.1f", mag[2]); printf("\r\n");
    #else
    /* Calibrated */
    float mag_neutral[3];

    for ( uint8_t i = 0; i < 3; i++ )
    {
      mag_neutral[i] = mag[i] - mag_hard_iron[i];
    }

    for ( uint8_t j = 0; j < 3; j++ )
    {
      mag_result[j] = mag_soft_iron[j][0] * mag_neutral[0] + mag_soft_iron[j][1] * mag_neutral[1] + mag_soft_iron[j][2] * mag_neutral[2];
    }
    
    printf("%.1f", mag_result[0]); printf(" ");
    printf("%.1f", mag_result[1]); printf(" ");
    printf("%.1f", mag_result[2]); printf("\r\n");
    #endif

    HAL_Delay(20);
  }
  else
  {
    HAL_Delay(500);
  }

}