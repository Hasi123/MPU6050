#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define COMPASS_ON      (0x04)

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (100)

struct rx_s {
  unsigned char header[3];
  unsigned char cmd;
};
struct hal_s {
  unsigned char lp_accel_mode;
  unsigned char sensors;
  unsigned char dmp_on;
  unsigned char wait_for_tap;
  volatile unsigned char new_gyro;
  unsigned char motion_int_mode;
  unsigned long no_dmp_hz;
  unsigned long next_pedo_ms;
  unsigned long next_temp_ms;
  unsigned long next_compass_ms;
  unsigned int report;
  unsigned short dmp_features;
  struct rx_s rx;
};
struct hal_s hal = {0};


/* The sensors can be mounted onto the board in any orientation. The mounting
   matrix seen below tells the MPL how to rotate the raw data from the
   driver(s).
   TODO: The following matrices refer to the configuration on internal test
   boards at Invensense. If needed, please modify the matrices to match the
   chip-to-body matrix for your particular set up.
*/
/*
   XYZ  010_001_000 Identity Matrix
   XZY  001_010_000
   YXZ  010_000_001
   YZX  000_010_001
   ZXY  001_000_010
   ZYX  000_001_010
*/
uint16_t orientMtx = 0b10001000;


#include "I2Cdev.h"
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "inv_dmp_uncompress.h"

void setup() {
  //init serial
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  unsigned char accel_fsr;
  unsigned short gyro_rate, gyro_fsr;
  struct int_param_s int_param;

  if (mpu_init(&int_param) < 0)
    Serial.println(F("MPU init failed"));

  /* Get/set hardware configuration. Start gyro. */
  /* Wake up all sensors. */
#ifdef COMPASS_ENABLED
  mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
#else
  mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
#endif
  /* Push both gyro and accel data into the FIFO. */
  mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  mpu_set_sample_rate(DEFAULT_MPU_HZ);
#ifdef COMPASS_ENABLED
  /* The compass sampling rate can be less than the gyro/accel sampling rate.
     Use this function for proper power management.
  */
  mpu_set_compass_sample_rate(1000 / COMPASS_READ_MS);
#endif
  /* Read back configuration in case it was set improperly. */
  mpu_get_sample_rate(&gyro_rate);
  mpu_get_gyro_fsr(&gyro_fsr);
  mpu_get_accel_fsr(&accel_fsr);
#ifdef COMPASS_ENABLED
  mpu_get_compass_fsr(&compass_fsr);
#endif

  /* Initialize HAL state variables. */
#ifdef COMPASS_ENABLED
  hal.sensors = ACCEL_ON | GYRO_ON | COMPASS_ON;
#else
  hal.sensors = ACCEL_ON | GYRO_ON;
#endif
  hal.dmp_on = 0;
  hal.report = 0;
  hal.rx.cmd = 0;
  hal.next_pedo_ms = 0;
  hal.next_compass_ms = 0;
  hal.next_temp_ms = 0;

  /* To initialize the DMP:
    1. Call dmp_load_motion_driver_firmware(). This pushes the DMP image in
       inv_mpu_dmp_motion_driver.h into the MPU memory.
    2. Push the gyro and accel orientation matrix to the DMP.
    3. Register gesture callbacks. Don't worry, these callbacks won't be
       executed unless the corresponding feature is enabled.
    4. Call dmp_enable_feature(mask) to enable different features.
    5. Call dmp_set_fifo_rate(freq) to select a DMP output rate.
    6. Call any feature-specific control functions.

    To enable the DMP, just call mpu_set_dmp_state(1). This function can
    be called repeatedly to enable and disable the DMP at runtime.

    The following is a short summary of the features supported in the DMP
    image provided in inv_mpu_dmp_motion_driver.c:
    DMP_FEATURE_LP_QUAT: Generate a gyro-only quaternion on the DMP at
    200Hz. Integrating the gyro data at higher rates reduces numerical
    errors (compared to integration on the MCU at a lower sampling rate).
    DMP_FEATURE_6X_LP_QUAT: Generate a gyro/accel quaternion on the DMP at
    200Hz. Cannot be used in combination with DMP_FEATURE_LP_QUAT.
    DMP_FEATURE_TAP: Detect taps along the X, Y, and Z axes.
    DMP_FEATURE_ANDROID_ORIENT: Google's screen rotation algorithm. Triggers
    an event at the four orientations where the screen should rotate.
    DMP_FEATURE_GYRO_CAL: Calibrates the gyro data after eight seconds of
    no motion.
    DMP_FEATURE_SEND_RAW_ACCEL: Add raw accelerometer data to the FIFO.
    DMP_FEATURE_SEND_RAW_GYRO: Add raw gyro data to the FIFO.
    DMP_FEATURE_SEND_CAL_GYRO: Add calibrated gyro data to the FIFO. Cannot
    be used in combination with DMP_FEATURE_SEND_RAW_GYRO.
  */
  if (dmp_load_motion_driver_firmware() < 0)
    Serial.println(F("DMP load failed"));
  //dmp_set_orientation(orientMtx);  //default orientation doesn't need to be set
  //dmp_register_tap_cb(tap_cb);
  //dmp_register_android_orient_cb(android_orient_cb);
  /*
     Known Bug -
     DMP when enabled will sample sensor data at 200Hz and output to FIFO at the rate
     specified in the dmp_set_fifo_rate API. The DMP will then sent an interrupt once
     a sample has been put into the FIFO. Therefore if the dmp_set_fifo_rate is at 25Hz
     there will be a 25Hz interrupt from the MPU device.

     There is a known issue in which if you do not enable DMP_FEATURE_TAP
     then the interrupts will be at 200Hz even if fifo rate
     is set at a different rate. To avoid this issue include the DMP_FEATURE_TAP

     DMP sensor fusion works only with gyro at +-2000dps and accel +-2G
  */
  hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
                     DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
                     DMP_FEATURE_GYRO_CAL;
  dmp_enable_feature(hal.dmp_features);
  dmp_set_fifo_rate(DEFAULT_MPU_HZ);

  //read back configured DMP and print
  /*Reamrks:
    If DMP is activated, it always changes
    So read back with DMP still disabled
  */
#define READ_DMP
#ifdef READ_DMP
  unsigned char curRead;
  unsigned int i = 0;
  while (i < UNCOMPRESSED_DMP_CODE_SIZE) {
    if (!(i % 256)) {
      Serial.print("//bank #");
      Serial.println(i / 256);
    }
    mpu_read_mem(i, 1, &curRead);
    Serial.print("0x");
    if (curRead < 16)
      Serial.print(0);
    Serial.print(curRead, HEX);
    Serial.print(", ");
    i++;
    if (!(i % 16))
      Serial.println();
  }
  Serial.println();
#endif

  //enable DMP
  mpu_set_dmp_state(1);
  hal.dmp_on = 1;

  //get WhoAmI, should be 0x68 or 0x69
  uint8_t whoami;
  I2Cdev::readByte(0x68, 0x75, &whoami);
  Serial.print(F("WhoAmI: 0x"));
  Serial.println(whoami, HEX);
}

void loop() {
  short gyro[3], accel_short[3], sensors;
  unsigned char more;
  long quat[4];
  unsigned long sensor_timestamp;
  dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more);

  if (sensors) { //new packet

    //#define PRINT_GYRO
    //#define PRINT_ACCEL
    //#define PRINT_QUAT

#ifdef PRINT_GYRO
    Serial.print(gyro[0]); Serial.print("\t");
    Serial.print(gyro[1]); Serial.print("\t");
    Serial.println(gyro[2]);
#endif
#ifdef PRINT_ACCEL
    Serial.print(accel_short[0]); Serial.print("\t");
    Serial.print(accel_short[1]); Serial.print("\t");
    Serial.println(accel_short[2]);
#endif
#ifdef PRINT_QUAT
    Serial.print(quat[0]); Serial.print("\t");
    Serial.print(quat[1]); Serial.print("\t");
    Serial.print(quat[2]); Serial.print("\t");
    Serial.println(quat[3]);
#endif
  }
}
