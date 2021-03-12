#include <Arduino.h>
#include "MPU6050Helper.h"
#include "I2Cdev.h"
//Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#include "inv_dmp_uncompress.h"


uint8_t readByte(uint8_t devAddr, uint8_t regAddr) {
  uint8_t buff;
  I2Cdev::readByte(devAddr, regAddr, &buff);
  return buff;
}

bool writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t buff) {
  return I2Cdev::writeByte(devAddr, regAddr, buff);
}

int16_t readWord(uint8_t devAddr, uint8_t regAddr) {
  uint16_t buff;
  I2Cdev::readWord(devAddr, regAddr, &buff);
  return buff;
}

bool writeWord(uint8_t devAddr, uint8_t regAddr, int16_t buff) {
  return I2Cdev::writeWord(devAddr, regAddr, buff);
}

int8_t readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data) {
  return I2Cdev::readBytes(devAddr, regAddr, length, data);
}

bool writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data) {
  return I2Cdev::writeBytes(devAddr, regAddr, length, data);
}

int16_t readWordAveraged(uint8_t devAddr, uint8_t regAddr, uint8_t loops) {
  int32_t sum = 0;
  for (uint8_t i = 0; i < loops; i++) {
    sum += readWord(devAddr, regAddr);
  }
  return (int16_t)(sum / loops);
}

//Print hex number
void printHex(uint8_t hexVal) {
  Serial.print("0x");
  if (hexVal < 0x10)
    Serial.print(0);
  Serial.print(hexVal, HEX);
}

//Dump all the MPUs regs
void mpu_dump_regs() {
  for (uint8_t i = 0; i < 255; i++) {
    printHex(i);
    Serial.print(", ");
    printHex(readByte(mpuAddr, i));
    Serial.println();
  }
}

//Write to the DMP memory
int8_t mpu_write_mem(uint16_t mem_addr, uint16_t length, uint8_t *data) {

  if (!writeWord(mpuAddr, MPU6050_RA_BANK_SEL, mem_addr))
    return -1;
  if (!writeBytes(mpuAddr, MPU6050_RA_MEM_R_W, length, data))
    return -1;
  return 0;
}

//Read from the DMP memory
int8_t mpu_read_mem(uint16_t mem_addr, uint16_t length, uint8_t *data) {

  if (!writeWord(mpuAddr, MPU6050_RA_BANK_SEL, mem_addr))
    return -1;
  if (!readBytes(mpuAddr, MPU6050_RA_MEM_R_W, length, data))
    return -1;
  return 0;
}

//Load and verify DMP image
int8_t load_dmp() {  //using compressed DMP firmware
  //#define VERIFY_DMP //should we verify loading the DMP?

  uint16_t ii, this_write;
  uint8_t progBuffer[MPU6050_DMP_MEMORY_CHUNK_SIZE];
#ifdef VERIFY_DMP
  uint8_t cur[MPU6050_DMP_MEMORY_CHUNK_SIZE];
#endif

  /* start loading */
  inv_dmp_uncompress_reset();

  for (ii = 0; ii < UNCOMPRESSED_DMP_CODE_SIZE; ii += this_write) {
    this_write = min(MPU6050_DMP_MEMORY_CHUNK_SIZE, UNCOMPRESSED_DMP_CODE_SIZE - ii);

    /* decompress chunk */
    for (uint16_t progIndex = 0; progIndex < this_write; progIndex++)
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
void mpuInit() {
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
  writeByte(mpuAddr, MPU6050_RA_PWR_MGMT_1, 0b10000000); //reset
  delay(100);
#endif
  writeByte(mpuAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_CLOCK_PLL_XGYRO); //wake up and set clock to gyro X (recomended by datasheet)
  writeByte(mpuAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GYRO_FS_2000 << 3); //Gyro range
  writeByte(mpuAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACCEL_FS_2 << 3); //Accel range
  writeByte(mpuAddr, MPU6050_RA_CONFIG, MPU6050_DLPF_BW_42); //DLPF
#define SAMPLE_RATE 200 //has to be double as DMP rate
  writeByte(mpuAddr, MPU6050_RA_SMPLRT_DIV, 1000 / SAMPLE_RATE - 1);  //sample rate divider
  //writeByte(mpuAddr, MPU6050_RA_INT_ENABLE, 0); //disable interrupts, already 0 by default
  writeByte(mpuAddr, MPU6050_RA_INT_PIN_CFG, 0x80); //setup interrupt pin
  delay(50);
  load_dmp();
  //writeByte(mpuAddr, MPU6050_RA_FIFO_EN, 0); //disable FIFO, already 0 by default
  writeByte(mpuAddr, MPU6050_RA_USER_CTRL, 0b00001100); //reset FIFO and DMP
  delay(50);
  writeByte(mpuAddr, MPU6050_RA_USER_CTRL, 0b11000000); //enable FIFO and DMP
  writeByte(mpuAddr, MPU6050_RA_INT_ENABLE, 0x02); //hardware DMP interrupt needed? yes in this case else set to 0
  //writeByte(mpuAddr, MPU6050_RA_FIFO_EN, 0); //already 0

}

//read a FIFO packet and parse data
//should be called after interrupt or data_ready state
void mpuGetFIFO(short *gyro, short *accel, long *quat) {
  //get packet
#define FIFO_SIZE 32
  unsigned char fifo_data[FIFO_SIZE];
  unsigned short fifo_count = readWord(mpuAddr, MPU6050_RA_FIFO_COUNTH);

  readBytes(mpuAddr, MPU6050_RA_FIFO_R_W, fifo_count, fifo_data);

  //parse data
  quat[0] = ((long)fifo_data[0] << 24) | ((long)fifo_data[1] << 16) |
            ((long)fifo_data[2] << 8) | fifo_data[3];
  quat[1] = ((long)fifo_data[4] << 24) | ((long)fifo_data[5] << 16) |
            ((long)fifo_data[6] << 8) | fifo_data[7];
  quat[2] = ((long)fifo_data[8] << 24) | ((long)fifo_data[9] << 16) |
            ((long)fifo_data[10] << 8) | fifo_data[11];
  quat[3] = ((long)fifo_data[12] << 24) | ((long)fifo_data[13] << 16) |
            ((long)fifo_data[14] << 8) | fifo_data[15];
  accel[0] = ((short)fifo_data[16] << 8) | fifo_data[17];
  accel[1] = ((short)fifo_data[18] << 8) | fifo_data[19];
  accel[2] = ((short)fifo_data[20] << 8) | fifo_data[21];
  gyro[0] = ((short)fifo_data[22] << 8) | fifo_data[23];
  gyro[1] = ((short)fifo_data[24] << 8) | fifo_data[25];
  gyro[2] = ((short)fifo_data[26] << 8) | fifo_data[27];
}
