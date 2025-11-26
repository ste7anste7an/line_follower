/*
  I2C Slave used for reading Line Follwowing ir detectors.
*/
// include standard eeprom library for writing calibration values
#include <EEPROM.h>

#define MAJ_VERSION 1
#define MIN_VERSION 0


#define NUM_SENSORS 8
#include <Adafruit_NeoPixel.h>

#define LED_PIN  PB11     // any PAx / PBx pin works
#define LED_COUNT 1

Adafruit_NeoPixel strip(NUM_SENSORS, LED_PIN, NEO_GRB + NEO_KHZ800);

#include <Wire.h>
#define MY_I2C_ADDRESS 0x33

#define CTRL_PIN PB3

typedef enum {
  CMD_SET_MODE_RAW = 0,
  CMD_SET_MODE_CAL,   //1
  CMD_SET_MODE_DIG,   //2
  CMD_CALIBRATE,      //3
  CMD_GET_MIN,        // 4
  CMD_GET_MAX,        // 5
  CMD_GET_AVG,        //6
  CMD_IS_CALIBRATED,  //7
  CMD_PRINT_CAL,      //8
  CMD_SET_MIN,        //9
  CMD_SET_MAX,        // 10
  CMD_GET_VERSION,    //11
  CMD_DEBUG,          //12
  CMD_GET_POSITION,   //13
  CMD_SET_EMITTER,     //14
  CMD_SAVE_CAL, // 15 save calibrated values to eeprom
  CMD_LOAD_CAL  //16 load calibrated values frpom eeprom
} Commands;

typedef enum {
  MODE_RAW,
  MODE_CAL,
  MODE_DIG,
  MODE_CALIBRATING
} Modes;

Modes current_mode = MODE_RAW;
bool is_calibrated = false;
bool calc_position = true;
bool inverted = true;
int nr = 0;
unsigned long t0 = millis();
uint8_t load_cal[2]={0,0};

uint8_t calMin[NUM_SENSORS+4]; // +4 to keep these arrays as long as the measurment array buf
uint8_t calMax[NUM_SENSORS+4];
uint8_t calAvg[NUM_SENSORS+4];

int adcPins[] = { PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PB0, PB1 };


// --- IWDG register definitions (from WCH datasheet) ---
#define IWDG_BASE      0x40003000UL
#define IWDG_KR        (*(volatile uint32_t *)(IWDG_BASE + 0x00))
#define IWDG_PR        (*(volatile uint32_t *)(IWDG_BASE + 0x04))
#define IWDG_RLR       (*(volatile uint32_t *)(IWDG_BASE + 0x08))
#define IWDG_SR        (*(volatile uint32_t *)(IWDG_BASE + 0x0C))

// --- Key values ---
#define IWDG_START     0xCCCC
#define IWDG_REFRESH   0xAAAA
#define IWDG_UNLOCK    0x5555

// --- Helper functions ---
void wdt_init(uint16_t timeout_ms) {
  // LSI ≈ 40 kHz
  // Choose prescaler and reload for desired timeout
  // Example: prescaler 32 → tick = 1.25 ms
  uint8_t prescaler = 3; // PR = 3 -> divide by 32
  uint32_t tick_ms = (1000UL * 32UL) / 40000UL; // ≈ 0.8 ms/tick
  uint16_t reload = min((uint16_t)(timeout_ms / tick_ms), (uint16_t)0x0FFF);

  IWDG_KR = IWDG_START;  // Start the watchdog
  IWDG_KR = IWDG_UNLOCK; // Enable PR and RLR access
  IWDG_PR = prescaler;
  IWDG_RLR = reload;
  IWDG_KR = IWDG_REFRESH; // Reload counter
}

void wdt_feed() {
  IWDG_KR = IWDG_REFRESH;
}

void readSensors(uint8_t outVals[]) {
  for (int i = 0; i < NUM_SENSORS; ++i) {
    outVals[i] = 255 - (analogRead(adcPins[i]) >> 4);
  }
}

void printCal() {
  Serial.println(F("Calibration:"));
  for (int i = 0; i < NUM_SENSORS; ++i) {
    Serial.print("S");
    Serial.print(i);
    Serial.print(": min=");
    Serial.print(calMin[i]);
    Serial.print(" max=");
    Serial.print(calMax[i]);
    Serial.print(" Avg=");
    Serial.println(calAvg[i]);
  }
}

void initCalibrate() {
  // initialize min/max to extremes
  Serial.println("start calibrating");
  for (int i = 0; i < NUM_SENSORS+4; ++i) {
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
  pinMode(CTRL_PIN, OUTPUT);
  digitalWrite(CTRL_PIN, LOW);
  delayMicroseconds(1000);  // delay 1ms to reset level
  digitalWrite(CTRL_PIN, HIGH);
  for (int i = 0; i < level; i++) {
    digitalWrite(CTRL_PIN, LOW);
    delayMicroseconds(1);
    digitalWrite(CTRL_PIN, HIGH);
    delayMicroseconds(10);
  }
}

void show_neopixel(uint8_t buf[]) {
  for (int i=0; i<NUM_SENSORS; i++) {
    if (is_calibrated) 
      strip.setPixelColor(i, strip.Color(0, (uint8_t)(buf[i]/16), 0)); // Green
    else 
       strip.setPixelColor(i, strip.Color((uint8_t)(buf[i]/16), 0, 0)); // Red
  }  
  strip.show();
  
}

void invertBuf(const uint8_t inp[], uint8_t outp[]) {
  if (inverted) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      outp[i] = inp[i];
    }
  } else {
    for (int i = 0; i < NUM_SENSORS; i++) {
      outp[i] = 255 - inp[i];
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


bool getLinePosition(const uint8_t norm[], uint8_t &posOut, uint8_t &minOut, uint8_t &maxOut) {
  long weighted_sum = 0;
  long sum = 0;
  minOut = 255;
  maxOut = 0;
  int val;
  if (!is_calibrated) return false;
  // hardcoded num sensors !!!
  for (int i = 0; i < NUM_SENSORS; ++i) {
    val = 255 - norm[i];  // invert
    weighted_sum += val * (i + 1);
    sum += val;
    if (val > maxOut) maxOut = val;
    if (val < minOut) minOut = val;
  }
  // if (sum < 10) { // nothing detected (tunable)
  //   return false;
  // }

  long p;
  if (sum > 0) {
    p = (255 * weighted_sum);
    p /= sum;  // p in approx -350..350
  } else
    p = 0;
  // scale to -1000..1000
  // for (int i=0; i<8; i++) {
  //   Serial.print(norm[i]);
  //   Serial.print(",");
  // }
  posOut = (p - 255) / 7;
  // Serial.print("pos=");
  // Serial.println(posOut);
  return true;
}

#define N_SAMPLES 8  // number of derivative samples to average


uint8_t computeMovingAverageDerivative(uint8_t pos) {
    static uint8_t lastPos = 0;
    static uint32_t lastTime = 0;
    static bool initialized = false;

    static float derivHistory[N_SAMPLES];
    static uint8_t index = 0;
    static uint8_t count = 0;

    uint32_t now = millis();

    if (!initialized) {
        lastPos = pos;
        lastTime = now;
        initialized = true;
        return 128;  // neutral mid value
    }

    int16_t deltaPos = (int16_t)pos - (int16_t)lastPos;
    uint32_t deltaTime = now - lastTime;

    // Handle wrap-around (0↔255)
    if (deltaPos > 128) deltaPos -= 256;
    else if (deltaPos < -128) deltaPos += 256;

    float derivative = 0.0f;
    if (deltaTime > 0) {
        derivative = (float)deltaPos / (float)deltaTime;  // pos units per ms
    }

    // Store derivative in circular buffer
    derivHistory[index] = derivative;
    index = (index + 1) % N_SAMPLES;
    if (count < N_SAMPLES) count++;

    // Compute moving average
    float sum = 0.0f;
    for (uint8_t i = 0; i < count; i++) {
        sum += derivHistory[i];
    }
    float avgDerivative = sum / count;

    lastPos = pos;
    lastTime = now;

    // --- Convert to uint8_t output ---
    float scaled = avgDerivative * 100.0f + 128.0f;  // scale and offset
    if (scaled < 0.0f) scaled = 0.0f;
    if (scaled > 255.0f) scaled = 255.0f;

    return (uint8_t)lroundf(scaled);  // rounded uint8_t in [0..255]
}

void ReceiveEvent(int nBytes) {
  uint8_t command;
  uint8_t buf[NUM_SENSORS];
  //Serial.print("nbytes:"); Serial.println(nbytes);
  Serial.println("received I2C msg");
  if (nBytes > 0) {
    command = Wire.read();
    Serial.print("received command: ");
    Serial.println(command);
    //delay(1);
  }
  if (command < 4) {  // then change measuring mode
    current_mode = command;
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
      current_mode = MODE_CALIBRATING;  //current_mode = old_mode;
      break;
    case CMD_IS_CALIBRATED:
      buf[0] = is_calibrated ? 1 : 0;
      Wire.write(buf, NUM_SENSORS+4);
      break;
    case CMD_PRINT_CAL:
      printCal();
      //current_mode = old_mode;
      break;
    case CMD_DEBUG:
      Serial.println(millis());
      break;
    case CMD_SET_EMITTER:
      if (nBytes > 0) {
        Serial.print("emitter :");
        uint8_t level = Wire.read();
        nBytes--;
        Serial.println(level);
        setEmitter(level);
      }
      break;
    case CMD_SET_MIN:
      if (nBytes >= 8) {
        Serial.print("set cal min ");
        for (int i = 0; i < 8; i++) {
          uint8_t val = Wire.read();
          calMin[i] = val;
          calAvg[i] = (calMin[i] + calMax[i]) / 2;
          Serial.print(",");
          Serial.print(val);
          nBytes--;
        }
        Serial.println();
        load_cal[0]=1;
        if (load_cal[1]==1) {
          is_calibrated=true;
        }
      }
      break;
    case CMD_SET_MAX:
      if (nBytes >= 8) {
        Serial.print("set cal max ");
        for (int i = 0; i < 8; i++) {
          uint8_t val = Wire.read();
          calMax[i] = val;
          calAvg[i] = (calMin[i] + calMax[i]) / 2;
          Serial.print(",");
          Serial.print(val);
          nBytes--;
        }
        Serial.println();
        load_cal[1]=1;
        if (load_cal[0]==1) {
          is_calibrated=true;
        }
      }
      break;

    case CMD_GET_MIN:
      Wire.write(calMin, NUM_SENSORS+4);
      break;
    case CMD_GET_MAX:
      Wire.write(calMax, NUM_SENSORS+4);
      break;
    case CMD_GET_AVG:
      Wire.write(calAvg, NUM_SENSORS+4);
      break;

    case CMD_SAVE_CAL:
      Serial.println("CMD_SAVE_CAL");
      //printCal();
      if (is_calibrated==1) {
        for (int i=0; i<NUM_SENSORS; i++) {
          EEPROM.write(i,calMin[i]);
          EEPROM.write(i+NUM_SENSORS,calMax[i]);
          //Serial.print("calmax ");Serial.print(i); Serial.print("= "); Serial.println(calMax[i]);
        }
      }
      EEPROM.commit();
      break;
    case CMD_LOAD_CAL:
      Serial.println("CMD_LOAD_CAL");
      for (int i=0; i<NUM_SENSORS; i++) {
        calMin[i]=EEPROM.read(i);
        calMax[i]=EEPROM.read(i+NUM_SENSORS);
        //Serial.print("calmax ");Serial.print(i); Serial.print("= "); Serial.println(calMax[i]);
        calAvg[i] = (calMin[i] + calMax[i]) / 2;
  
      }
      is_calibrated=1;
      //printCal();
      break;
    case CMD_GET_VERSION:
      buf[0] = MAJ_VERSION;
      buf[1] = MIN_VERSION;
      Wire.write(buf, NUM_SENSORS+4);
      break;
    case CMD_GET_POSITION:
      if (nBytes > 0) {
        Serial.print("emitter :");
        uint8_t measure_pos = Wire.read();
        calc_position = (measure_pos == 1);
        if (calc_position) {
          Serial.println("calc postion Yes");
        } else {
          Serial.println("calc postion No");
        }
        nBytes--;
      }
      // uint8_t pos;
      // uint8_t minval;
      // uint8_t maxval;
      // readSensors(buf);
      // if (getLinePosition(buf,pos,minval,maxval)) {
      //   buf[0]=pos; // reuse buf
      //   buf[1]=minval;
      //   buf[2]=maxval;
      // } else {
      //   buf[0]=0;
      //   buf[1]=0;
      //   buf[2]=0;
      // }
      // // buf[0]=123;
      // // buf[1]=222;
      // // buf[2]=42;
      // Wire.write(buf,3);

      // //buf[0]=pos; // reuse buf
      // //Wire.write(buf,1);
      // //Serial.print("position:");
      // //Serial.println(pos);

      break;

      //current_mode = old_mode;
  }
  // read remaining bytes
  Serial.print("nBytes=");
  Serial.println(nBytes);

  while (nBytes > 0) {
    Serial.println("reading extra byte");
    Serial.println(Wire.read());
    nBytes--;
  }
}

void RequestEvent() {
  uint8_t buf[NUM_SENSORS + 4];
  uint8_t norm[NUM_SENSORS + 4];
  uint8_t invert[NUM_SENSORS + 4];
  uint8_t pos;
  uint8_t minval;
  uint8_t maxval;
  nr += 1;
  if (nr == 1000) {
    Serial.print("delay=");
    Serial.println(millis() - t0);
    t0 = millis();
    nr = 0;
  }
  switch (current_mode) {
    case MODE_RAW:
      readSensors(buf);
      show_neopixel(buf);
      //invertBuf(buf,invert);
      Wire.write(buf, NUM_SENSORS + 4);
      break;
    case MODE_CAL:
      readSensors(buf);
      computeNormalized(buf, norm);
      show_neopixel(norm);
      if (calc_position) {
        if (getLinePosition(norm, pos, minval, maxval)) {
          norm[NUM_SENSORS] = pos;
          norm[NUM_SENSORS + 1] = minval;
          norm[NUM_SENSORS + 2] = maxval;
        } else {
          pos=0;
          norm[NUM_SENSORS] = 0;
          norm[NUM_SENSORS + 1] = 0;
          norm[NUM_SENSORS + 2] = 0;
        }
      }
      norm[NUM_SENSORS+3]=computeMovingAverageDerivative(pos);
      //Serial.print("a[0]= ");
      //Serial.println(norm[0]);
      //invertBuf(norm,invert);
      Wire.write(norm, NUM_SENSORS + 4);
      break;
    case MODE_DIG:
      readSensors(buf);
      computeDigital(buf, norm);
      Wire.write(norm, NUM_SENSORS + 4);
      break;
    case MODE_CALIBRATING:
      readSensors(buf);
      doCalibrate();
      //invertBuf(buf,invert);
      Wire.write(buf, NUM_SENSORS+4);
      break;
    default:
      readSensors(buf);
      Wire.write(buf, NUM_SENSORS+4);
  }
}

void setup() {
  

  strip.begin();
  strip.show(); // Turn off pixels

strip.setPixelColor(0, strip.Color(25, 0, 0)); // Red
  strip.show();
  delay(500);

  strip.setPixelColor(0, strip.Color(0, 25, 0)); // Green
  strip.show();
  delay(500);

  strip.setPixelColor(0, strip.Color(0, 0, 25)); // Blue
  strip.show();
  delay(500);

  Serial.begin(115200);
  Serial.println("I2C slave @address 0x33");
  EEPROM.begin(); // setup eeprom usage
  current_mode = MODE_RAW;

  Wire.begin(MY_I2C_ADDRESS);
  // Note: enable I2C slave functionality in /libraries/Wire/src/utility/twi.h to prevent error message for next two lines
  Wire.onReceive(ReceiveEvent);
  Wire.onRequest(RequestEvent);
  
  //wdt_init(2000);  // 2 second watchdog
  //Serial.println("Watchdog active!");
}

void loop() {
  //wdt_feed();
  //Serial.println("Feeding watchdog");
  delay(100);

}
