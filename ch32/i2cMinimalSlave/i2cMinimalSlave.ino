/*
  I2C Minimal Slave - CH32 I2C Slave example

  Show the minimal code required to implement an I2C slave device with these features:
     - handle I2C scanning (i.e. acknowledge data-less transaction)
     - handle receiving data
     - return data requested by master
     - blink LED on PA2 according transfered value
  
  Compiled for CH32V003J4M6 (SOP8) using Arduino IDE v2.3.2, CH32 core v1.0.4. 
  Optimize: Smallest (-Os default), Debug symbols: none, no UART: //#define UART_MODULE_ENABLED
  Note - enable I2C slave functionality in /libraries/Wire/src/utility/twi.h
    #define OPT_I2C_SLAVE 1

  Using those options this sketch uses 9424 bytes (57%) of program storage space. Maximum is 16384 bytes.
  Global variables use 580 bytes (28%) of dynamic memory, leaving 1468 bytes for local variables. Maximum is 2048 bytes.
*/

#define MAJ_VERSION 1
#define MIN_VERSION 0


#define NUM_SENSORS 10

#include <Wire.h>
#define MY_I2C_ADDRESS 0x33
 
typedef enum {
    MODE_VAL_RAW = 0,
    MODE_VAL_CAL, //1
    MODE_VAL_DIG, //2
    MODE_CALIBRATE, //3
    MODE_GET_MIN,// 4
    MODE_GET_MAX,// 5
    MODE_GET_AVG,//6
    MODE_IS_CALIBRATED, //7 
    MODE_PRINT_CAL, //8
    MODE_SAVE_CAL, //9
    MODE_LOAD_CAL, // 10
    MODE_VERSION, //11
    MODE_DEBUG, //12
    MODE_POSITION, //13
    MODE_INVERT //14
} Mode;

Mode current_mode = MODE_VAL_RAW;
Mode old_mode = MODE_VAL_RAW;
uint8_t callibrating = 0;
bool inverted=true;
int nr=0;
unsigned long t0 = millis();


uint8_t calMin[NUM_SENSORS];
uint8_t calMax[NUM_SENSORS];
uint8_t calAvg[NUM_SENSORS];

int weights[NUM_SENSORS];

int adcPins[]={PA0,PA1,PA2,PA3,PA4,PA5,PA6,PA7,PB0,PB1};

void readSensors(uint8_t outVals[]) {
  for (int i = 0; i < NUM_SENSORS; ++i) {
     outVals[i] = 255-(analogRead(adcPins[i])>>4);
  }
}

void printCal() {
  Serial.println(F("Calibration:"));
  for (int i = 0; i < NUM_SENSORS; ++i) {
    Serial.print("S"); Serial.print(i);
    Serial.print(": min="); Serial.print(calMin[i]);
    Serial.print(" max="); Serial.print(calMax[i]);
    Serial.print(" Avg="); Serial.println(calAvg[i]);
  }
}

void initCalibrate() {
  // initialize min/max to extremes
  Serial.println("start callibrating");
  for (int i = 0; i < NUM_SENSORS; ++i) {
    calMin[i] = 255;
    calMax[i] = 0;
  }
}


void doCalibrate() {
  // initialize min/max to extremes
  unsigned long t0 = millis();
  uint8_t vals[NUM_SENSORS];
  readSensors(vals);
  for (int i = 0; i < NUM_SENSORS; ++i) {
    if (vals[i] < calMin[i]) calMin[i] = vals[i];
    if (vals[i] > calMax[i]) calMax[i] = vals[i];
  }

  // avoid zero-range
  for (int i = 0; i < NUM_SENSORS; ++i) {
    if (calMax[i] <= calMin[i]) {
      // fallback if no variation: small margin
      calMax[i] = calMin[i] + 8;
    }
    calAvg[i] = (calMin[i] + calMax[i]) / 2;
  }

}

void invertBuf(const uint8_t inp[], uint8_t outp[]) {
  if (inverted) {
    for (int i=0; i< NUM_SENSORS; i++) {
      outp[i]=inp[i];
    }
  } else {
    for (int i=0; i< NUM_SENSORS; i++) {
      outp[i]=255-inp[i];
    }
  }
}

void computeNormalized(const uint8_t raw[], uint8_t normOut[]) {
  for (int i = 0; i < NUM_SENSORS; ++i) {
    int r = raw[i];
    int minv = calMin[i];
    int maxv = calMax[i];
    int v = (r - minv) * 255 / (maxv - minv);
    if (v < 0) v = 0;
    if (v > 255) v = 255;
    normOut[i] = v;
  }
}

void computeDigital(const uint8_t raw[], uint8_t normOut[]) {
  for (int i = 0; i < NUM_SENSORS; ++i) {
    int r = raw[i];
    if (r < calAvg[i]) 
      normOut[i] = 0;
    else
      normOut[i] = 1;
  }
}


bool getLinePosition(const uint8_t raw[], uint8_t &posOut) {
  uint8_t norm[NUM_SENSORS];
  computeNormalized(raw, norm);
  long num = 0;
  long den = 0;
  for (int i = 0; i < NUM_SENSORS; ++i) {
    num += (long)norm[i] * (long)weights[i];
    den += norm[i];
  }
  if (den < 50) { // nothing detected (tunable)
    return false;
  }
  long p = num / den; // p in approx -350..350
  // scale to -1000..1000
  posOut = (int)((p * 128L) / 350L);
  if (posOut > 128) posOut = 128;
  if (posOut < -128) posOut = -128;
  posOut+=128;
  return true;
}



void ReceiveEvent(int nBytes) {
  Serial.println("received I2C msg");
  old_mode = current_mode;
  if (nBytes > 0) {
    current_mode = Wire.read();
    Serial.print("received command: ");
    Serial.println(current_mode);
  }
  // red remaining bytes
  nBytes--;
  switch (current_mode) {
    case MODE_CALIBRATE:
      initCalibrate();
      //current_mode = old_mode;
      break;
    case MODE_PRINT_CAL:
      printCal();
      //current_mode = old_mode;
      break;
    case MODE_DEBUG:
      Serial.println(millis());
      break;
    case MODE_INVERT:
      inverted = !inverted;
      Serial.print("inverted :");
      Serial.println(inverted);
      //current_mode = old_mode;
  }
  // read remaining bytes
  while(nBytes>0)
  {
    Serial.println(Wire.read());
    nBytes--;
  }
  


  

}

void RequestEvent() {
  uint8_t buf[NUM_SENSORS];
  uint8_t norm[NUM_SENSORS];
  uint8_t invert[NUM_SENSORS];
  
  nr+=1;
  if (nr==1000) {
    Serial.print("delay=");
    Serial.println(millis()-t0);
    t0 = millis();
    nr=0;
  }
  switch (current_mode) {
    case MODE_VAL_RAW:
      readSensors(buf);
      //invertBuf(buf,invert);
      Wire.write(buf,NUM_SENSORS);
      break;
    case MODE_CALIBRATE:
      readSensors(buf);
      doCalibrate();
      //invertBuf(buf,invert);
      Wire.write(buf,NUM_SENSORS);
      break;
    case MODE_VAL_CAL:
      readSensors(buf);
      computeNormalized(buf,norm);
      //invertBuf(norm,invert);
      Wire.write(norm,NUM_SENSORS);
      break;
    case MODE_VAL_DIG:
      readSensors(buf);
      computeDigital(buf,norm);
      Wire.write(norm,NUM_SENSORS);
      break;
    case MODE_PRINT_CAL:
      readSensors(buf);
      Wire.write(buf,NUM_SENSORS);
      break;
    case MODE_POSITION:
      uint8_t pos;
      readSensors(buf);
      getLinePosition(buf,pos);
      buf[0]=pos; // reuse buf
      Wire.write(buf,1);
      Serial.print("position:");
      Serial.println(pos);

      break;
    case MODE_GET_MIN:
      Wire.write(calMin, NUM_SENSORS);
      break;
    case MODE_GET_MAX:
      Wire.write(calMax, NUM_SENSORS);
      break;
    case MODE_GET_AVG:
      Wire.write(calAvg, NUM_SENSORS);
      break;
    case MODE_VERSION:
      buf[0]=MAJ_VERSION;
      buf[1]=MIN_VERSION;
      Wire.write(buf, NUM_SENSORS);
      break;
      
    default:
      readSensors(buf);
      Wire.write(buf,NUM_SENSORS);
    
  }
  
}

void setup() {
  Serial.begin(115200);
  Serial.println("I2C slave @address 0x33");

  Mode current_mode = MODE_VAL_RAW;

  Wire.begin(MY_I2C_ADDRESS);
  // Note: enable I2C slave functionality in /libraries/Wire/src/utility/twi.h to prevent error message for next two lines
  Wire.onReceive(ReceiveEvent);
  Wire.onRequest(RequestEvent);

  int dist=10;
  int start = -dist/2*(NUM_SENSORS-1);
  for (int i=0; i<NUM_SENSORS; i++) {
     weights[i]= start+i*dist;
  }


}

void loop() {

}