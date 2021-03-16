//minimal implementation with DMP
#include "MPU6050Helper.h"

//#define PRINT_GYRO
//#define PRINT_ACCEL
//#define PRINT_QUAT

short gyro[3], accel[3];
long quat[4];

//provide your calibration values here
int16_t gyrOffs[3] = {0, 85, 9};
int16_t accOffs[3] = { -3960, -1672, 1323};
uint8_t fineGain[3] = {177, 221, 220};

void setup() {
  //Serial.begin(57600);
  //while (!Serial); // wait for Leonardo enumeration, others continue immediately

  mpuInit(gyrOffs, accOffs, fineGain);
  //mpuInit(); // if you don't have any calibration values
}

void loop() {

  if (mpuNewDmp()) { //new DMP packet
    mpuGetFIFO(gyro, accel, quat);

    //Serial.println(getVertaccel(accel, quat)); //print vertical acceleration independent of orientation

#ifdef PRINT_GYRO
    Serial.print(gyro[0]); Serial.print("\t");
    Serial.print(gyro[1]); Serial.print("\t");
    Serial.println(gyro[2]);
#endif
#ifdef PRINT_ACCEL
    Serial.print(accel[0]); Serial.print("\t");
    Serial.print(accel[1]); Serial.print("\t");
    Serial.println(accel[2]);
#endif
#ifdef PRINT_QUAT
    Serial.print(quat[0]); Serial.print("\t");
    Serial.print(quat[1]); Serial.print("\t");
    Serial.print(quat[2]); Serial.print("\t");
    Serial.println(quat[3]);
#endif
  }
}
