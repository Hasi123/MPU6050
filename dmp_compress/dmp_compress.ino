#include "inv_dmp_compress.h"

void setup() {
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
  inv_dmp_compress();
}

void loop() {

}
