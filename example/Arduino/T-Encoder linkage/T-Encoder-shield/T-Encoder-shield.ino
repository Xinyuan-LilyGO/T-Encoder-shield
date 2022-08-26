

void setup() {
  Serial0_begin(9600);
}

void loop() {

    while(USBSerial_available()) {
    char serialChar = USBSerial_read();
    Serial0_write(serialChar);
  }
  while (Serial0_available()) {
    char serialChar = Serial0_read();
    USBSerial_write(serialChar);
  }
  
}
