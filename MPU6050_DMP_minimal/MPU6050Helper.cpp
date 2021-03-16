#include <Arduino.h>
#include "MPU6050Helper.h"
#include "I2Cdev.h"
//Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#include "inv_dmp_uncompress.h"


unsigned char readByte(unsigned char devAddr, unsigned char regAddr) {
  unsigned char buff;
  I2Cdev::readByte(devAddr, regAddr, &buff);
  return buff;
}

bool writeByte(unsigned char devAddr, unsigned char regAddr, unsigned char buff) {
  return I2Cdev::writeByte(devAddr, regAddr, buff);
}

short readWord(unsigned char devAddr, unsigned char regAddr) {
  uint16_t buff;
  I2Cdev::readWord(devAddr, regAddr, &buff);
  return buff;
}

bool writeWord(unsigned char devAddr, unsigned char regAddr, short buff) {
  return I2Cdev::writeWord(devAddr, regAddr, buff);
}

char readBytes(unsigned char devAddr, unsigned char regAddr, unsigned char length, unsigned char *data) {
  return I2Cdev::readBytes(devAddr, regAddr, length, data);
}

bool writeBytes(unsigned char devAddr, unsigned char regAddr, unsigned char length, unsigned char *data) {
  return I2Cdev::writeBytes(devAddr, regAddr, length, data);
}

bool writeWords(unsigned char devAddr, unsigned char regAddr, unsigned char length, short *data) {
  return I2Cdev::writeWords(devAddr, regAddr, length, (uint16_t*)data);
}

//reads word "loops" times and averages the result
short readWordAveraged(unsigned char devAddr, unsigned char regAddr, unsigned short loops) {
  long sum = 0;
  readByte(mpuAddr, MPU6050_RA_INT_STATUS); //clear int status
  for (unsigned short i = 0; i < loops; i++) {
    while (!bitRead(readByte(mpuAddr, MPU6050_RA_INT_STATUS), MPU6050_INTERRUPT_DATA_RDY_BIT)); //wait for new reading
    sum += readWord(devAddr, regAddr);
  }
  return (short)(sum / loops);
}

//Print hex number
void printHex(unsigned char hexVal) {
  Serial.print("0x");
  if (hexVal < 0x10)
    Serial.print(0);
  Serial.print(hexVal, HEX);
}

//Dump all MPU regs
void mpu_dump_regs() {
  for (unsigned char i = 0; i < 128; i++) {
    printHex(i);
    Serial.print(", ");
    printHex(readByte(mpuAddr, i));
    Serial.println();
  }
}

//load calibration data into the registers
void load_calibration(short *gyro_offs, short *accel_offs, unsigned char *fine_gain) {
  writeBytes(mpuAddr, MPU6050_RA_X_FINE_GAIN, 3, fine_gain);
  writeWords(mpuAddr, MPU6050_RA_XA_OFFS_H, 3, accel_offs);
  writeWords(mpuAddr, MPU6050_RA_XG_OFFS_USRH, 3, gyro_offs);
}

//Write to the DMP memory
char mpu_write_mem(unsigned short mem_addr, unsigned short length, unsigned char *data) {

  if (!writeWord(mpuAddr, MPU6050_RA_BANK_SEL, mem_addr))
    return -1;
  if (!writeBytes(mpuAddr, MPU6050_RA_MEM_R_W, length, data))
    return -1;
  return 0;
}

//Read from the DMP memory
char mpu_read_mem(unsigned short mem_addr, unsigned short length, unsigned char *data) {

  if (!writeWord(mpuAddr, MPU6050_RA_BANK_SEL, mem_addr))
    return -1;
  if (!readBytes(mpuAddr, MPU6050_RA_MEM_R_W, length, data))
    return -1;
  return 0;
}

//dumps the dmp memory onto serial
void mpu_dump_dmp() {
  unsigned char curRead;
  unsigned int i = 0;
  writeByte(mpuAddr, MPU6050_RA_USER_CTRL, bit(MPU6050_USERCTRL_FIFO_RESET_BIT)); //pause DMP and reset FIFO
  while (i < UNCOMPRESSED_DMP_CODE_SIZE) {
    if (!(i % 256)) {
      Serial.print("//bank #");
      Serial.println(i / 256);
    }
    mpu_read_mem(i, 1, &curRead);
    printHex(curRead);
    Serial.print(", ");
    i++;
    if (!(i % 16))
      Serial.println();
  }
  Serial.println();
  writeByte(mpuAddr, MPU6050_RA_USER_CTRL, bit(MPU6050_USERCTRL_DMP_EN_BIT) | bit(MPU6050_USERCTRL_FIFO_EN_BIT)); //enable FIFO and DMP
}

//Load and verify DMP image
char load_dmp() {  //using compressed DMP firmware
  //#define VERIFY_DMP //should we verify loading the DMP?

  unsigned short ii, this_write;
  unsigned char progBuffer[MPU6050_DMP_MEMORY_CHUNK_SIZE];
#ifdef VERIFY_DMP
  unsigned char cur[MPU6050_DMP_MEMORY_CHUNK_SIZE];
#endif

  /* start loading */
  inv_dmp_uncompress_reset();

  for (ii = 0; ii < UNCOMPRESSED_DMP_CODE_SIZE; ii += this_write) {
    this_write = min(MPU6050_DMP_MEMORY_CHUNK_SIZE, UNCOMPRESSED_DMP_CODE_SIZE - ii);

    /* decompress chunk */
    for (unsigned short progIndex = 0; progIndex < this_write; progIndex++)
      progBuffer[progIndex] = inv_dmp_uncompress();

    //write
    if (mpu_write_mem(ii, this_write, progBuffer))
      return -1;
#ifdef VERIFY_DMP //check
    if (mpu_read_mem(ii, this_write, cur))
      return -1;
    if (memcmp(progBuffer, cur, this_write))
      return -2;
#endif
  }

  /* Set program start address. */
  if (!writeWord(mpuAddr, MPU6050_RA_DMP_CFG_1, MPU6050_DMP_START_ADDRESS))
    return -1;

  return 0;
}

//Init the sensor and load dmp
void mpuInit(short *gyro_offs, short *accel_offs, unsigned char *fine_gain) {
  //join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  //following order is done in InvenSense example code
  //redundant writes are deleted
#define RESET_MPU //regs get reset on power cycle, so not really needed
#ifdef RESET_MPU
  writeByte(mpuAddr, MPU6050_RA_PWR_MGMT_1, bit(MPU6050_PWR1_DEVICE_RESET_BIT)); //reset
  delay(100);
#endif
  writeByte(mpuAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_CLOCK_PLL_XGYRO); //wake up and set clock to gyro X (recomended by datasheet)
  writeByte(mpuAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GYRO_FS_2000 << 3); //Gyro range
  writeByte(mpuAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACCEL_FS << 3); //Accel range
  writeByte(mpuAddr, MPU6050_RA_CONFIG, MPU6050_DLPF_BW_42); //DLPF
  writeByte(mpuAddr, MPU6050_RA_SMPLRT_DIV, 1000 / MPU6050_SAMPLE_RATE - 1);  //sample rate divider
#ifdef MPU6050_INTERRUPT_PIN
  //writeByte(mpuAddr, MPU6050_RA_INT_ENABLE, 0); //disable interrupts, already 0 by default
  writeByte(mpuAddr, MPU6050_RA_INT_PIN_CFG, bit(MPU6050_INTCFG_LATCH_INT_EN_BIT) | bit(MPU6050_INTCFG_INT_RD_CLEAR_BIT)); //setup interrupt pin
#endif
  if (gyro_offs && accel_offs && fine_gain)
    load_calibration(gyro_offs, accel_offs, fine_gain);
  load_dmp();
  //writeByte(mpuAddr, MPU6050_RA_FIFO_EN, 0); //disable FIFO, already 0 by default
  writeByte(mpuAddr, MPU6050_RA_USER_CTRL, bit(MPU6050_USERCTRL_DMP_RESET_BIT) | bit(MPU6050_USERCTRL_FIFO_RESET_BIT)); //reset FIFO and DMP
  delay(50);
  writeByte(mpuAddr, MPU6050_RA_USER_CTRL, bit(MPU6050_USERCTRL_DMP_EN_BIT) | bit(MPU6050_USERCTRL_FIFO_EN_BIT)); //enable FIFO and DMP
#ifdef MPU6050_INTERRUPT_PIN
  writeByte(mpuAddr, MPU6050_RA_INT_ENABLE, bit(MPU6050_INTERRUPT_DMP_INT_BIT)); //hardware DMP interrupt needed? yes in this case, else set to 0
#endif

}

//read 1 FIFO packet and parse data
//should be called after interrupt or data_ready state
char mpuGetFIFO(short *gyroData, short *accelData, long *quatData) {
#define FIFO_SIZE 32
  unsigned char fifo_data[FIFO_SIZE];
  unsigned short fifo_count = readWord(mpuAddr, MPU6050_RA_FIFO_COUNTH);

  if (fifo_count != FIFO_SIZE) { //reset FIFO if more data than 1 packet
    writeByte(mpuAddr, MPU6050_RA_USER_CTRL, bit(MPU6050_USERCTRL_FIFO_RESET_BIT)); //reset FIFO
    delay(50);
    writeByte(mpuAddr, MPU6050_RA_USER_CTRL, bit(MPU6050_USERCTRL_DMP_EN_BIT) | bit(MPU6050_USERCTRL_FIFO_EN_BIT)); //enable FIFO and DMP
    return -1;
  }
  else {
    readBytes(mpuAddr, MPU6050_RA_FIFO_R_W, fifo_count, fifo_data); //get FIFO data

    //parse data
    quatData[0] = ((long)fifo_data[0] << 24) | ((long)fifo_data[1] << 16) |
                  ((long)fifo_data[2] << 8) | fifo_data[3];
    quatData[1] = ((long)fifo_data[4] << 24) | ((long)fifo_data[5] << 16) |
                  ((long)fifo_data[6] << 8) | fifo_data[7];
    quatData[2] = ((long)fifo_data[8] << 24) | ((long)fifo_data[9] << 16) |
                  ((long)fifo_data[10] << 8) | fifo_data[11];
    quatData[3] = ((long)fifo_data[12] << 24) | ((long)fifo_data[13] << 16) |
                  ((long)fifo_data[14] << 8) | fifo_data[15];
    accelData[0] = ((short)fifo_data[16] << 8) | fifo_data[17];
    accelData[1] = ((short)fifo_data[18] << 8) | fifo_data[19];
    accelData[2] = ((short)fifo_data[20] << 8) | fifo_data[21];
    gyroData[0] = ((short)fifo_data[22] << 8) | fifo_data[23];
    gyroData[1] = ((short)fifo_data[24] << 8) | fifo_data[25];
    gyroData[2] = ((short)fifo_data[26] << 8) | fifo_data[27];
    return 0;
  }
}

bool mpuNewDmp() {
  return bitRead(readByte(mpuAddr, MPU6050_RA_INT_STATUS), MPU6050_INTERRUPT_DMP_INT_BIT);
}

/* compute vertical vector and vertical accel from IMU data */
double getVertaccel(short *imuAccel, long *imuQuat) {

  /* G to ms convertion */
#define VERTACCEL_G_TO_MS 9.80665

  /* quat scale */
#define LIGHT_INVENSENSE_QUAT_SCALE_SHIFT 30
#define LIGHT_INVENSENSE_QUAT_SCALE ((double)(1LL << LIGHT_INVENSENSE_QUAT_SCALE_SHIFT))

  /* accel scale */
#if MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_2
#define LIGHT_INVENSENSE_ACCEL_SCALE_SHIFT 14
#elif MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_4
#define LIGHT_INVENSENSE_ACCEL_SCALE_SHIFT 13
#elif MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_8
#define LIGHT_INVENSENSE_ACCEL_SCALE_SHIFT 12
#elif MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_16
#define LIGHT_INVENSENSE_ACCEL_SCALE_SHIFT 11
#endif
#define LIGHT_INVENSENSE_ACCEL_SCALE ((double)(1LL << LIGHT_INVENSENSE_ACCEL_SCALE_SHIFT))

  /***************************/
  /* normalize and calibrate */
  /***************************/
  double accel[3], quat[4], vertVector[3];

  for (unsigned char i = 0; i < 3; i++)
    accel[i] = ((double)imuAccel[i]) / LIGHT_INVENSENSE_ACCEL_SCALE;

  for (unsigned char i = 0; i < 4; i++)
    quat[i] = ((double)imuQuat[i]) / LIGHT_INVENSENSE_QUAT_SCALE;


  /******************************/
  /* real and vert acceleration */
  /******************************/

  /* compute vertical direction from quaternions */
  vertVector[0] = 2 * (quat[1] * quat[3] - quat[0] * quat[2]);
  vertVector[1] = 2 * (quat[2] * quat[3] + quat[0] * quat[1]);
  vertVector[2] = 2 * (quat[0] * quat[0] + quat[3] * quat[3]) - 1;

  /* compute real acceleration (without gravity) */
  double ra[3];
  for (unsigned char i = 0; i < 3; i++)
    ra[i] = accel[i] - vertVector[i];

  /* compute vertical acceleration */
  double vertAccel = (vertVector[0] * ra[0] + vertVector[1] * ra[1] + vertVector[2] * ra[2]) * VERTACCEL_G_TO_MS;
  return vertAccel;
}
