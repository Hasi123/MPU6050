#include "I2Cdev.h"
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define MPU6050_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW

#include "MPU6050Helper.h"

#define GYRO_CAL //calibrate gyro
#define ACCEL_CAL //calibrate accel
#define GYRO_OUT //output gyro data
#define ACCEL_OUT //output accel data
//#define TEMP_OUT //output temp data

//OFFSETS: -3936, -1674, 1330, -16, 82, 9  manually calibrated values just for verification

void setup() {
  //variables
  short origAccOffs[3];
  unsigned char origGain[3]; //[7:4] accel gain, [3:0] gyro gain
  char origGyrTC[3]; //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD

  //init serial
  Serial.begin(57600);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  ////////////
  //init mpu//
  ////////////
  writeByte(mpuAddr, MPU6050_RA_PWR_MGMT_1, 0b10000000); //reset
  delay(100); //needed according to InvenSense driver
  writeByte(mpuAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_CLOCK_PLL_ZGYRO); //ClockSource to X gyro
  writeByte(mpuAddr, MPU6050_RA_CONFIG, MPU6050_DLPF_BW_42); //DLPF
  writeByte(mpuAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GYRO_FS_1000 << 3); //Gyro range
  writeByte(mpuAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACCEL_FS_16 << 3); //Accel range
  //calibration units are in 1000 gyro and 16 accel range
  delay(50);

  //get WhoAmI, should be 0x68 or 0x69
  unsigned char whoami = readByte(mpuAddr, MPU6050_RA_WHO_AM_I);
  Serial.print(F("WhoAmI: 0x"));
  Serial.println(whoami, HEX);


  //get factory calibration data
  origAccOffs[0] = readWord(mpuAddr, MPU6050_RA_XA_OFFS_H);
  origAccOffs[1] = readWord(mpuAddr, MPU6050_RA_YA_OFFS_H);
  origAccOffs[2] = readWord(mpuAddr, MPU6050_RA_ZA_OFFS_H);

  origGain[0] = readByte(mpuAddr, MPU6050_RA_X_FINE_GAIN);
  origGain[1] = readByte(mpuAddr, MPU6050_RA_Y_FINE_GAIN);
  origGain[2] = readByte(mpuAddr, MPU6050_RA_Z_FINE_GAIN);

  origGyrTC[0] = readByte(mpuAddr, MPU6050_RA_XG_OFFS_TC);
  origGyrTC[1] = readByte(mpuAddr, MPU6050_RA_YG_OFFS_TC);
  origGyrTC[2] = readByte(mpuAddr, MPU6050_RA_ZG_OFFS_TC);
  //...gyro offsets not needed, since 0

  Serial.println(F("Original biases:"));
  for (unsigned char i = 0; i < 3; i++) {
    Serial.print(origAccOffs[i]); Serial.print("\t");
    Serial.print(origGain[i]); Serial.print("\t");
    Serial.println(origGyrTC[i]);
  }

  //////////////////
  //gyro calibration
  //////////////////
#ifdef GYRO_CAL
  //TODO: dmp gyro calibration
  Serial.println(F("Calibrating gyro, don´t move!"));
  delay(500); //gyro needs some time to stabilize
  while (!isResting(5000)); //wait for resting IMU
  const unsigned char loops = 200;
  short newGyrOffs[3] = {0, 0, 0};

  newGyrOffs[0] = -readWordAveraged(mpuAddr, MPU6050_RA_GYRO_XOUT_H, loops);
  newGyrOffs[1] = -readWordAveraged(mpuAddr, MPU6050_RA_GYRO_YOUT_H, loops);
  newGyrOffs[2] = -readWordAveraged(mpuAddr, MPU6050_RA_GYRO_ZOUT_H, loops);

  writeWord(mpuAddr, MPU6050_RA_XG_OFFS_USRH, newGyrOffs[0]);
  writeWord(mpuAddr, MPU6050_RA_YG_OFFS_USRH, newGyrOffs[1]);
  writeWord(mpuAddr, MPU6050_RA_ZG_OFFS_USRH, newGyrOffs[2]);

  Serial.print(F("Gyro calib done, values: "));
  Serial.print(newGyrOffs[0]); Serial.print("\t");
  Serial.print(newGyrOffs[1]); Serial.print("\t");
  Serial.println(newGyrOffs[2]);
#endif //GYRO_CAL

  ////////////////////
  //Accel calibration
  ///////////////////
#ifdef ACCEL_CAL
  Serial.println(F("Calibrating accel, place on all 6 sides flat!"));

  //set accel fine gain to 0 for easier calculation
  I2Cdev::writeBits(mpuAddr, MPU6050_RA_X_FINE_GAIN, 7, 4, 0);
  I2Cdev::writeBits(mpuAddr, MPU6050_RA_Y_FINE_GAIN, 7, 4, 0);
  I2Cdev::writeBits(mpuAddr, MPU6050_RA_Z_FINE_GAIN, 7, 4, 0);
  delay(50); //to apply offset values

  //if not moving get averaged (max) data of active axis (TODO maybe: if new data save the higher value, or better var?)
  short accelMax[6];
  short accelData[3];
  unsigned char calibState = 0;
  unsigned char accIndex;
  while (true) {
    if (isResting()) {
      accelData[0] = readWordAveraged(mpuAddr, MPU6050_RA_ACCEL_XOUT_H, 50);
      accelData[1] = readWordAveraged(mpuAddr, MPU6050_RA_ACCEL_YOUT_H, 50);
      accelData[2] = readWordAveraged(mpuAddr, MPU6050_RA_ACCEL_ZOUT_H, 50);

      for (unsigned char i = 0; i < 3; i++) {
        accIndex = i * 2;
        if (!bitRead(calibState, accIndex) && accelData[i] > 1500) { //not set and positive
          accelMax[accIndex] = accelData[i];
          bitSet(calibState, accIndex);
          Serial.print(F("Saved "));
          Serial.println(accIndex);
        }
        accIndex++;
        if (!bitRead(calibState, accIndex) && accelData[i] < -1500) { //not set and negative
          accelMax[accIndex] = accelData[i];
          bitSet(calibState, accIndex);
          Serial.print(F("Saved "));
          Serial.println(accIndex);
        }
      }
    }

    //if all axes have data point -> calculate values -> write to registers and print
    if (calibState == 0b111111) {
      short newAccOffs[3];
      char newAccScal[3];

      Serial.println(F("Calculating..."));
      Serial.println(F("Readings: "));
      for (unsigned char i = 0; i < 6; i++) {
        Serial.print(accelMax[i]);
        Serial.print("\t");
      }
      Serial.println();

      Serial.print(F("newOffs: "));
      for (unsigned char i = 0; i < 3; i++) {
        //Offsets
        newAccOffs[i] = (accelMax[i * 2] + accelMax[i * 2 + 1]) / 2;

        //Scaling
        short diff = 2048 - (accelMax[i * 2] - newAccOffs[i]); //need to calculate offset compansated values
        if (diff >= 0) //round positive values
          newAccScal[i] = (diff + 14) / 29;
        else //round negative values
          newAccScal[i] = (diff - 14) / 29;

        //final offstes
        newAccOffs[i] = origAccOffs[i] - newAccOffs[i];

        Serial.print(newAccOffs[i]); Serial.print("\t");
      }
      Serial.println();

      //write values to registers
      writeWord(mpuAddr, MPU6050_RA_XA_OFFS_H, newAccOffs[0]);
      writeWord(mpuAddr, MPU6050_RA_YA_OFFS_H, newAccOffs[1]);
      writeWord(mpuAddr, MPU6050_RA_ZA_OFFS_H, newAccOffs[2]);

      I2Cdev::writeBits(mpuAddr, MPU6050_RA_X_FINE_GAIN, 7, 4, newAccScal[0]);
      I2Cdev::writeBits(mpuAddr, MPU6050_RA_Y_FINE_GAIN, 7, 4, newAccScal[1]);
      I2Cdev::writeBits(mpuAddr, MPU6050_RA_Z_FINE_GAIN, 7, 4, newAccScal[2]);

      Serial.print(F("newScal: "));
      Serial.print(readByte(mpuAddr, MPU6050_RA_X_FINE_GAIN)); Serial.print("\t");
      Serial.print(readByte(mpuAddr, MPU6050_RA_Y_FINE_GAIN)); Serial.print("\t");
      Serial.println(readByte(mpuAddr, MPU6050_RA_Z_FINE_GAIN));

      delay(50); //to apply offsets
      Serial.println(F("Accel calib done"));
      break;
    } //calculate calibration values
  } //while(1)

#endif //ACCEL_CAL

  //relation between Accel reading and AccGain
  /*
    for (char i = -8; i < 8; i++) {
      I2Cdev::writeBits(mpuAddr, MPU6050_RA_Z_FINE_GAIN, 7, 4, i);
      delay(50);
      Serial.print(i); Serial.print("\t");
      Serial.println(readWord(mpuAddr, MPU6050_RA_ACCEL_ZOUT_H));
    }
  */
  //Findings: only the upper 4 bits seem to affect accel scaling
  //          lower 4 bits affect gyro values, don't overwrite factory calibration
}

void loop() {
  delay(50);

#ifdef GYRO_OUT
  Serial.print(F("Gyro: "));
  Serial.print(readWordAveraged(mpuAddr, MPU6050_RA_GYRO_XOUT_H, 10)); Serial.print("\t");
  Serial.print(readWordAveraged(mpuAddr, MPU6050_RA_GYRO_YOUT_H, 10)); Serial.print("\t");
  Serial.print(readWordAveraged(mpuAddr, MPU6050_RA_GYRO_ZOUT_H, 10)); Serial.print("\t");
#endif

#ifdef ACCEL_OUT
  Serial.print(F("Accel: "));
  Serial.print(readWordAveraged(mpuAddr, MPU6050_RA_ACCEL_XOUT_H, 10)); Serial.print("\t");
  Serial.print(readWordAveraged(mpuAddr, MPU6050_RA_ACCEL_YOUT_H, 10)); Serial.print("\t");
  Serial.print(readWordAveraged(mpuAddr, MPU6050_RA_ACCEL_ZOUT_H, 10)); Serial.print("\t");
#endif

#ifdef TEMP_OUT
  Serial.print(F("Temp: "));
  Serial.print((float)readWord(mpuAddr, MPU6050_RA_TEMP_OUT_H) / 340.0 + 36.53); //in °C
#endif

#if defined(GYRO_CAL) || defined(ACCEL_CAL) || defined(TEMP_OUT)
  Serial.println();
#endif

}
