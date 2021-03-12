# MPU6050
I created this project to improve my understanding of the MPU6050 and write the most efficient (minimal size) code for it. Much of it is based on the I2Cdev library and prunkdump's arduino-variometer project.

InvenSenseMotionDriver is just an Arduino port of the official 6.12 DMP release. You can configure the DMP firmware with it and read back the firmware.

dmp_compress compresses the DMP firmware image (basically if there are many 0x00 after each other, it saves a count instead of the many zeros).

MPU6060_DMP_minimal ist the bare minimum with full dmp functionality. It only takes up 5508 bytes (17%) of the flash memory on a 328p.

MPU6050_calibrate is my attempt to calibrate the sensor.
