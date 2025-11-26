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

#define PWM_PIN PA11

typedef enum {
    CMD_SET_MODE_RAW = 0,
    CMD_SET_MODE_CAL, //1
    CMD_SET_MODE_DIG, //2
    CMD_CALIBRATE, //3
    CMD_GET_MIN,// 4
    CMD_GET_MAX,// 5
    CMD_GET_AVG,//6
    CMD_IS_CALIBRATED, //7 
    CMD_PRINT_CAL, //8
    CMD_SET_MIN, //9
    CMD_SET_MAX, // 10
    CMD_GET_VERSION, //11
    CMD_DEBUG, //12
    CMD_GET_POSITION, //13
    CMD_SET_EMITTER //14
} Commands;

typedef enum {
    MODE_RAW,
    MODE_CAL,
    MODE_DIG,
    MODE_CALIBRATING
} Modes;

Modes current_mode = MODE_RAW;
uint8_t is_calibrated = 0;
bool inverted=true;
int nr=0;
unsigned long t0 = millis();


uint8_t calMin[NUM_SENSORS];
uint8_t calMax[NUM_SENSORS];
uint8_t calAvg[NUM_SENSORS];

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
  Serial.println("start calibrating");
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
  is_calibrated = true;

}

void setEmitter(uint8_t level) {
   analogWrite(PWM_PIN, level<<4);    // 5
  //  if (level>1) 
  //   digitalWrite(PWM_PIN,HIGH);
  // else
  // digitalWrite(PWM_PIN,LOW);
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
  long weighted_sum = 0;
  long sum = 0;
  int start = -10*(NUM_SENSORS-1);
  for (int i = 0; i < NUM_SENSORS; ++i) {
    weighted_sum += (long)norm[i] * (start + i*20);
    sum += norm[i];
  }
  if (sum < 10) { // nothing detected (tunable)
    return false;
  }
  
  long p;
  if (sum!=0) {
    p = weighted_sum * 10 / sum; // p in approx -350..350
  } else
  p=0;
  // scale to -1000..1000
  if (p > 400) p = 400;
  if (p < -400) p = -400;
  posOut=(p+400)*255/800; // between 0 and 255
  return true;
}

 

void ReceiveEvent(int nBytes) {
  uint8_t command;
  uint8_t buf[NUM_SENSORS];
  Serial.println("received I2C msg");
  if (nBytes > 0) {
    command = Wire.read();
    Serial.print("received command: ");
    Serial.println(command);
  }
  if (command<4) { // then change measuring mode
    current_mode=command;
  }
  // red remaining bytes
  nBytes--;
  switch (command) {
    case CMD_SET_MODE_RAW:
      current_mode = MODE_RAW;
      break;
    case CMD_SET_MODE_CAL:
      current_mode = MODE_CAL;
      break;
    case CMD_SET_MODE_DIG:
      current_mode = MODE_DIG;
      break;
      
    case CMD_CALIBRATE:
      initCalibrate();
      current_mode = MODE_CALIBRATING;//current_mode = old_mode;
      break;
    case CMD_IS_CALIBRATED:
      buf[0]=is_calibrated;
      Wire.write(buf,1);      
      break;
    case CMD_PRINT_CAL:
      printCal();
      //current_mode = old_mode;
      break;
    case CMD_DEBUG:
      Serial.println(millis());
      break;
    case CMD_SET_EMITTER:
      if (nBytes>0) {
        Serial.print("emitter :");
        uint8_t level = Wire.read();
        nBytes--;
        Serial.println(level);
        setEmitter(level);
      }
      break;
    case CMD_SET_MIN:
      if (nBytes>=8) {
        Serial.print("set cal min ");
        for (int i=0; i<8; i++) {
          uint8_t val = Wire.read();
          calMin[i]=val;
          calAvg[i] = (calMin[i] + calMax[i]) / 2;
          Serial.print(",");
          Serial.print(val);
          nBytes--;
        }
        Serial.println();
      }
      break;
    case CMD_SET_MAX:
      if (nBytes>=8) {
        Serial.print("set cal max ");
        for (int i=0; i<8; i++) {
          uint8_t val = Wire.read();
          calMax[i]=val;
          calAvg[i] = (calMin[i] + calMax[i]) / 2;
          Serial.print(",");
          Serial.print(val);
          nBytes--;
        }
        Serial.println();
      }
      break;

    case CMD_GET_MIN:
      Wire.write(calMin, NUM_SENSORS);
      break;
    case CMD_GET_MAX:
      Wire.write(calMax, NUM_SENSORS);
      break;
    case CMD_GET_AVG:
      Wire.write(calAvg, NUM_SENSORS);
      break;
    case CMD_GET_VERSION:
      buf[0]=MAJ_VERSION;
      buf[1]=MIN_VERSION;
      Wire.write(buf, 2);
      break;
    case CMD_GET_POSITION:
      uint8_t pos;
      readSensors(buf);
      getLinePosition(buf,pos);
      buf[0]=pos; // reuse buf
      Wire.write(buf,1);
      Serial.print("position:");
      Serial.println(pos);
      break;

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
    case MODE_RAW:
      readSensors(buf);
      //invertBuf(buf,invert);
      Wire.write(buf,NUM_SENSORS);
      break;
    case MODE_CAL:
      readSensors(buf);
      computeNormalized(buf,norm);
      Serial.print("a[0]= ");
      Serial.println(norm[0]);
      //invertBuf(norm,invert);
      Wire.write(norm,NUM_SENSORS);
      break;
    case MODE_DIG:
      readSensors(buf);
      computeDigital(buf,norm);
      Wire.write(norm,NUM_SENSORS);
      break;
    case MODE_CALIBRATING:
      readSensors(buf);
      doCalibrate();
      //invertBuf(buf,invert);
      Wire.write(buf,NUM_SENSORS);
      break;
    default:
      readSensors(buf);
      Wire.write(buf,NUM_SENSORS);
    
  }
  
}

void setup() {
  Serial.begin(115200);
  Serial.println("I2C slave @address 0x33");

  current_mode = MODE_RAW;
  pinMode(PWM_PIN, OUTPUT);
  analogWriteFrequency(100000);  // set PWM to 100 kHz
  

  Wire.begin(MY_I2C_ADDRESS);
  // Note: enable I2C slave functionality in /libraries/Wire/src/utility/twi.h to prevent error message for next two lines
  Wire.onReceive(ReceiveEvent);
  Wire.onRequest(RequestEvent);

}

void loop() {

}