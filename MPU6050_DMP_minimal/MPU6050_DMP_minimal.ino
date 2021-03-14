//minimal implementation with DMP
#include "MPU6050Helper.h"

//#define PRINT_GYRO
//#define PRINT_ACCEL
//#define PRINT_QUAT

short gyro[3], accel[3];
long quat[4];

void setup() {
  //Serial.begin(57600);
  //while (!Serial); // wait for Leonardo enumeration, others continue immediately

  mpuInit();
}

void loop() {

  if (bitRead(readByte(mpuAddr, MPU6050_RA_INT_STATUS), 1)) { //new DMP packet
    mpuGetFIFO(gyro, accel, quat);

#ifdef PRINT_GYRO
    Serial.print(gyro[0]); Serial.print("\t");
    Serial.print(gyro[1]); Serial.print("\t");
    Serial.println(gyro[2]); //Serial.print("\t");
    //Serial.println(readWord(mpuAddr, MPU6050_RA_XG_OFFS_USRH));
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
