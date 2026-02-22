#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_TinyUSB.h"

#include "uart1_driver.h"
#include "UartRemote.h"
#include "app.h"

#define MY_I2C_ADDRESS 0x33

void ReceiveEvent(int nBytes) {
  uint8_t command;
  //LOG(LOG_DEBUG, "received I2C msg");
  if (nBytes > 0) {
    command = Wire.read();
    nBytes--;
    //LOG(LOG_DEBUG, "received command: %d\r\n", command);
  }
  if (command < 10) {  // valid command
    // read remaining bytes
    //LOG(LOG_DEBUG,"entering command=%d command MAX_CMDS=%d\r",command,MAX_CMDS);
    switch (command) {
      case 1:
        SerialTinyUSB.println("I2C cmd  =1");
        for (int i = 0; i < 4; i++) {
          uint8_t v = Wire.read();
          SerialTinyUSB.print(v, HEX);
        }
        SerialTinyUSB.println("");

        break;
    }
    while (nBytes > 0) {
      uint8_t val = Wire.read();
      nBytes--;
    }
  }
}

void RequestEvent() {

  uint8_t buf[] = { 1, 2, 3, 4, 5, 6, 7, 8 };
  Wire.write(buf, 8);
}




USART1Stream SerialA;
UartRemote uart(SerialA, app_handler);

void setup() {
  delay(1000);
  SerialTinyUSB.println("Ready to receive commands");

  Wire.begin(MY_I2C_ADDRESS);
  // Note: enable I2C slave functionality in /libraries/Wire/src/utility/twi.h to prevent error message for next two lines
  Wire.onReceive(ReceiveEvent);
  Wire.onRequest(RequestEvent);



  pinMode(PB2, OUTPUT);
  SerialA.begin(115200);
}

void loop() {
  uart.process();
}