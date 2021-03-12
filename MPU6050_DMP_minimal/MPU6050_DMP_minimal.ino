//minimal implementation with DMP
#include "MPU6050Helper.h"

//#define PRINT_GYRO
//#define PRINT_ACCEL
//#define PRINT_QUAT

void setup() {
  Serial.begin(57600);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  mpuInit();
}

void loop() {
  short gyro[3], accel_short[3], sensors;
  unsigned char more;
  long quat[4];
  unsigned long sensor_timestamp;


  if (bitRead(readByte(mpuAddr, MPU6050_RA_INT_STATUS), 1)) { //new DMP packet
    //dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more); //TODO

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
