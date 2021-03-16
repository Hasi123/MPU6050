#include "inv_dmp_compress.h"

void setup() {
  Serial.begin(57600);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
  inv_dmp_compress();
}

void loop() {

}
