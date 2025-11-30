/*
  I2C Slave used for reading Line Follwowing ir detectors.
*/


enum LogLevel { LOG_ERROR=0, LOG_WARN, LOG_INFO, LOG_DEBUG, LOG_VERBOSE };
static LogLevel CURRENT_LOG_LEVEL = LOG_DEBUG;

// printf-style logging macro using Serial.printf directly
#define LOG(level, fmt, ...) do { \
    if(level <= CURRENT_LOG_LEVEL) { \
        Serial.printf("[%s] " fmt "\n", logLevelStr(level), ##__VA_ARGS__); \
    } \
} while(0)

inline const char* logLevelStr(LogLevel level) {
    switch(level) {
        case LOG_ERROR:   return "ERROR";
        case LOG_WARN:    return "WARN";
        case LOG_INFO:    return "INFO";
        case LOG_DEBUG:   return "DEBUG";
        case LOG_VERBOSE: return "VERBOSE";
        default:          return "LOG";
    }
}



// include standard eeprom library for writing calibration values
#include <EEPROM.h>
//#include <stdio.h>
//#include "Adafruit_TinyUSB.h"

#define MAJ_VERSION 2
#define MIN_VERSION 13


#define NUM_SENSORS 8
#include <Adafruit_NeoPixel.h>

#define NEOPIXEL_PIN PB11  // any PAx / PBx pin works

Adafruit_NeoPixel strip(NUM_SENSORS+1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

#define FLASH_TIME 250 // is ms
#define CAL_TIME 5000 // is ms

unsigned long t_cal_on;
unsigned long t_cal_off;
unsigned long t_cal_stop;

int count=0;

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
  CMD_SET_EMITTER,    //14
  CMD_SAVE_CAL,       // 15 save calibrated values to eeprom
  CMD_LOAD_CAL,       //16 load calibrated values frpom eeprom
  CMD_NEOPIXEL,       // 17 neopixel: lednr, r, g, b, write
  CMD_LEDS            //18
} Commands;

typedef enum {
  MODE_RAW,
  MODE_CAL,
  MODE_DIG,
  MODE_CALIBRATING,
} Modes;

typedef enum {
  LEDS_OFF,
  LEDS_NORMAL,
  LEDS_INVERTED,
  LEDS_POSITION
} LedsMode;



Modes current_mode = MODE_RAW;
LedsMode current_leds_mode = LEDS_NORMAL;
bool is_calibrated = false;
bool calc_position = true;
bool inverted = true;
bool debugging = true;
int nr = 0;
unsigned long t0 = millis();
uint8_t load_cal[2] = { 0, 0 };

uint8_t rawVals[NUM_SENSORS + 4];
uint8_t calMin[NUM_SENSORS + 4];  // +4 to keep these arrays as long as the measurment array buf
uint8_t calMax[NUM_SENSORS + 4];
uint8_t calAvg[NUM_SENSORS + 4];

int adcPins[] = { PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PB0, PB1 };  // 10 GPIO's for 10 possible detectors


// --- IWDG register definitions (from WCH datasheet) ---
#define IWDG_BASE 0x40003000UL
#define IWDG_KR (*(volatile uint32_t *)(IWDG_BASE + 0x00))
#define IWDG_PR (*(volatile uint32_t *)(IWDG_BASE + 0x04))
#define IWDG_RLR (*(volatile uint32_t *)(IWDG_BASE + 0x08))
#define IWDG_SR (*(volatile uint32_t *)(IWDG_BASE + 0x0C))

// --- Key values ---
#define IWDG_START 0xCCCC
#define IWDG_REFRESH 0xAAAA
#define IWDG_UNLOCK 0x5555

// --- Helper functions ---
void wdt_init(uint16_t timeout_ms) {
  // LSI ≈ 40 kHz
  // Choose prescaler and reload for desired timeout
  // Example: prescaler 32 → tick = 1.25 ms
  uint8_t prescaler = 3;                         // PR = 3 -> divide by 32
  uint32_t tick_ms = (1000UL * 32UL) / 40000UL;  // ≈ 0.8 ms/tick
  uint16_t reload = min((uint16_t)(timeout_ms / tick_ms), (uint16_t)0x0FFF);

  IWDG_KR = IWDG_START;   // Start the watchdog
  IWDG_KR = IWDG_UNLOCK;  // Enable PR and RLR access
  IWDG_PR = prescaler;
  IWDG_RLR = reload;
  IWDG_KR = IWDG_REFRESH;  // Reload counter
}

void wdt_feed() {
  IWDG_KR = IWDG_REFRESH;
}

// read raw values and store in global rawVals buffer
void readSensors() {
  for (int i = 0; i < NUM_SENSORS; ++i) {
    rawVals[i] = 255 - (analogRead(adcPins[i]) >> 4);
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
  LOG(LOG_DEBUG, "start calibrating\r\n");
  for (int i = 0; i < NUM_SENSORS + 4; ++i) {
    calMin[i] = 255;
    calMax[i] = 0;
  }
}


void doCalibrate() {
  // initialize min/max to extremes
  unsigned long t0 = millis();
  uint8_t vals[NUM_SENSORS];
  //readSensors(vals);
  for (int i = 0; i < NUM_SENSORS; ++i) {
    if (rawVals[i] < calMin[i]) calMin[i] = rawVals[i];
    if (rawVals[i] > calMax[i]) calMax[i] = rawVals[i];
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
  LOG(LOG_DEBUG,"is_calibrated set to true\r");
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
  if (current_leds_mode == LEDS_NORMAL) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (is_calibrated)
        strip.setPixelColor(i, strip.Color(0, (uint8_t)(buf[i] / 16), 0));  // Green
      else
        strip.setPixelColor(i, strip.Color((uint8_t)(buf[i] / 16), 0, 0));  // Red
    }
    strip.show();
  } else 
  if (current_leds_mode == LEDS_INVERTED) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (is_calibrated)
        strip.setPixelColor(i, strip.Color(0, 16-(uint8_t)(buf[i] / 16), 0));  // Green
      else
        strip.setPixelColor(i, strip.Color(16-(uint8_t)(buf[i] / 16), 0, 0));  // Red
    }
    strip.show();
  }
}

// not used
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

//uint32_t cnt_pos=0;

bool getLinePosition(const uint8_t norm[], uint8_t &posOut, uint8_t &minOut, uint8_t &maxOut) {
  long weighted_sum = 0;
  long sum = 0;
  minOut = 255;
  maxOut = 0;
  posOut = 0; // default value
  int val;
  if (!is_calibrated) return false;
  // hardcoded num sensors !!!
  for (int i = 0; i < NUM_SENSORS; ++i) {
    val = 255 - norm[i];  // invert
    if (val<50) val=0;
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
  
  posOut = (p - 255) / 7;
  
  // cnt_pos++;
  // if ( cnt_pos%1000==0) {

  // for (int i=0; i<8; i++) {
  //   LOG(LOG_DEBUG,"%d, \r",norm[i]);
  // }
  // LOG(LOG_DEBUG,"sum= %d, weighted sum = %d\r\n ",sum,weighted_sum);
 
  
  // LOG(LOG_DEBUG,"\r\npos=%d\r\n",posOut);
  // }
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
  uint8_t buf[NUM_SENSORS+4] = {0};
  //Serial.print("nbytes:"); Serial.println(nbytes);
  if (debugging) Serial.println("received I2C msg");
  if (nBytes > 0) {
    command = Wire.read();
    if (debugging) {
      LOG(LOG_DEBUG,"received command: %d\r\n",command);
    }
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
      t_cal_on = millis();
      t_cal_stop = millis() + CAL_TIME;
      // show_leds = false;
      // for (int i=0; i<NUM_SENSORS; i++) {

      // }
      // flash extra NEOPIXEL
      break;
    case CMD_IS_CALIBRATED:
      buf[0] = is_calibrated ? 1 : 0;
      Wire.write(buf, NUM_SENSORS + 4);
      break;
    case CMD_PRINT_CAL:
      printCal();
      //current_mode = old_mode;
      break;
    case CMD_DEBUG:
      debugging = !debugging;  // toggle debug
      LOG(LOG_DEBUG,"Debugging enabled\r\n");
      break;
    case CMD_SET_EMITTER:
      if (nBytes > 0) {
        if (debugging) { Serial.print("emitter :"); }
        uint8_t level = Wire.read();
        nBytes--;
        if (debugging) { Serial.println(level); }
        setEmitter(level);
      }
      break;
    case CMD_LEDS:
      if (nBytes > 0) {
        uint8_t led_mode = Wire.read();
        LOG(LOG_DEBUG,"led mode= %d\r\n",led_mode);
        current_leds_mode = led_mode;
        if (current_leds_mode == LEDS_OFF) {  // switch off neopixels
          strip.clear();
          strip.show();
        }
      }
      break;
    case CMD_NEOPIXEL:
      if (nBytes >= 4) {
        uint8_t nr_led = Wire.read();
        nBytes--;
        uint8_t r = Wire.read();
        nBytes--;
        uint8_t g = Wire.read();
        nBytes--;
        uint8_t b = Wire.read();
        nBytes--;
        LOG(LOG_VERBOSE,"nr=%d, r=%d, g=%d, b=%d\r\n",nr_led,r,g,b);

        strip.setPixelColor(nr_led, strip.Color(r, g, b));
        strip.show();  //always show neopixels
      }
      break;

    case CMD_SET_MIN:
      if (nBytes >= 8) {
        if (debugging) { Serial.print("set cal min "); }
        for (int i = 0; i < 8; i++) {
          uint8_t val = Wire.read();
          calMin[i] = val;
          calAvg[i] = (calMin[i] + calMax[i]) / 2;
          LOG(LOG_DEBUG,"%d, ",val);
          nBytes--;
        }
        LOG(LOG_DEBUG,"\r\n");
        load_cal[0] = 1;
        if (load_cal[1] == 1) {
          is_calibrated = true;
        }
      }
      break;
    case CMD_SET_MAX:
      if (nBytes >= 8) {
        LOG(LOG_DEBUG,"set cal max\r\n ");
        for (int i = 0; i < 8; i++) {
          uint8_t val = Wire.read();
          calMax[i] = val;
          calAvg[i] = (calMin[i] + calMax[i]) / 2;
          if (debugging) {
            LOG(LOG_DEBUG,"%d,",val);
          }
          nBytes--;
        }
        LOG(LOG_DEBUG,"\r\n");
        load_cal[1] = 1;
        if (load_cal[0] == 1) {
          is_calibrated = true;
        }
      }
      break;

    case CMD_GET_MIN:
      Wire.write(calMin, NUM_SENSORS + 4);
      break;
    case CMD_GET_MAX:
      Wire.write(calMax, NUM_SENSORS + 4);
      break;
    case CMD_GET_AVG:
      Wire.write(calAvg, NUM_SENSORS + 4);
      break;

    case CMD_SAVE_CAL:
      LOG(LOG_DEBUG,"CMD_SAVE_CAL\r\n");
      //printCal();
      if (is_calibrated == 1) {
        for (int i = 0; i < NUM_SENSORS; i++) {
          EEPROM.write(i, calMin[i]);
          EEPROM.write(i + NUM_SENSORS, calMax[i]);
        }
      }
      EEPROM.commit();
      break;
    case CMD_LOAD_CAL:
      LOG(LOG_DEBUG,"CMD_LOAD_CAL\r\n");
      for (int i = 0; i < NUM_SENSORS; i++) {
        calMin[i] = EEPROM.read(i);
        calMax[i] = EEPROM.read(i + NUM_SENSORS);
        calAvg[i] = (calMin[i] + calMax[i]) / 2;
      }
      is_calibrated = 1;
      //printCal();
      break;
    case CMD_GET_VERSION:
      buf[0] = MAJ_VERSION;
      buf[1] = MIN_VERSION;
      Wire.write(buf, NUM_SENSORS + 4);
      break;
    // case CMD_GET_POSITION:
    //   if (nBytes > 0) {
    //     uint8_t measure_pos = Wire.read();
    //     calc_position = (measure_pos == 1);
    //     if (debugging) {
    //       if (calc_position) {
    //         LOG(LOG_DEBUG,"calc postion Yes\r\n");
    //       } else {
    //         LOG(LOG_DEBUG,"calc postion No\r\n");
    //       }
    //     }
    //     nBytes--;
    //   }


    //   break;

      //current_mode = old_mode;
  }
  // read remaining bytes
  if (debugging) {
    LOG(LOG_VERBOSE,"nBytes=%d\r\n",nBytes);
  }

  while (nBytes > 0) {
    if (debugging) {
      Serial.println("reading extra byte");
      Serial.println(Wire.read());
    }
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
    LOG(LOG_DEBUG,"delay=%d\r\n",millis() - t0);
    t0 = millis();
    nr = 0;
  }
  switch (current_mode) {
    case MODE_RAW:
      // readSensors(buf);
      // show_neopixel(buf);

      show_neopixel(rawVals);

      //invertBuf(buf,invert);
      Wire.write(rawVals, NUM_SENSORS + 4);
      break;
    case MODE_CAL:
      // readSensors(buf);
      // computeNormalized(buf, norm);
      computeNormalized(rawVals, norm);
      if (current_leds_mode!=LEDS_POSITION)
        show_neopixel(norm);
      if (calc_position) {
        if (getLinePosition(norm, pos, minval, maxval)) {
          if (current_leds_mode==LEDS_POSITION) {
            
            float fpos = pos * (8.0 / 256.0);

            int i = floor(fpos);        // left LED
            float frac = fpos - i;      // blend factor

            int j = i + 1;              // right LED
            if (j >= 8) j = 7;

            const int R = 20;
            const int G = 0;
            const int B = 0;

            // Calculate intensity
            int R_i = R * (1.0 - frac);
            int R_j = R * frac;

            // Light LEDs
            strip.clear();
            
            if (i >= 0 && i < 8)
                strip.setPixelColor(i, strip.Color(R_i, G, B));

            if (j >= 0 && j < 8)
                strip.setPixelColor(j, strip.Color(R_j, G, B));
            
            // uint8_t led_nr=pos/32;
            // strip.setPixelColor(led_nr, strip.Color(30, 0, 0));  // Red
            strip.show();
          }
          norm[NUM_SENSORS] = pos;
          norm[NUM_SENSORS + 1] = minval;
          norm[NUM_SENSORS + 2] = maxval;
        } else {
          pos = 0;
          norm[NUM_SENSORS] = 0;
          norm[NUM_SENSORS + 1] = 0;
          norm[NUM_SENSORS + 2] = 0;
        }
      }
      norm[NUM_SENSORS + 3] = computeMovingAverageDerivative(pos);
      Wire.write(norm, NUM_SENSORS + 4);
      break;
    case MODE_DIG:
      computeDigital(rawVals, norm);
      Wire.write(norm, NUM_SENSORS + 4);
      break;
    case MODE_CALIBRATING:
      show_neopixel(rawVals);
      
      doCalibrate();
      //invertBuf(buf,invert);
      Wire.write(rawVals, NUM_SENSORS + 4);
      break;
    default:
      Wire.write(rawVals, NUM_SENSORS + 4);
  }
}

uint8_t buf[NUM_SENSORS];

void setup() {
/*
 if (!TinyUSBDevice.isInitialized()) {
    TinyUSBDevice.begin(0);
  }
*/
 
  strip.begin();
  strip.show();  // Turn off pixels

  strip.setPixelColor(0, strip.Color(25, 0, 0));  // Red
  strip.show();
  delay(100);

  strip.setPixelColor(0, strip.Color(0, 25, 0));  // Green
  strip.show();
  delay(100);

  strip.setPixelColor(0, strip.Color(0, 0, 25));  // Blue
  strip.show();
  delay(100);

  Serial.begin(115200);
  Serial.println("I2C slave @address 0x33");
  Serial.printf("\nhallo %d %px\n",1234, buf);
  EEPROM.begin();  // setup eeprom usage
  current_mode = MODE_RAW;

  Wire.begin(MY_I2C_ADDRESS);
  // Note: enable I2C slave functionality in /libraries/Wire/src/utility/twi.h to prevent error message for next two lines
  Wire.onReceive(ReceiveEvent);
  Wire.onRequest(RequestEvent);

  //wdt_init(2000);  // 2 second watchdog
  //Serial.println("Watchdog active!");
}


uint32_t lastFlash = 0;
bool flashState = false;


void loop() {

  count+=1;
  if (count%2000 ==0) {
    Serial.printf("count=%d\n\r",count);
    Serial.printf("mode=%d\r\n", current_mode);
    Serial.printf("is calibrated: %d calc_position=%d\r",is_calibrated,calc_position);
  }
  // read new values
  readSensors();
  
  if (current_mode != MODE_CALIBRATING) {
    if (is_calibrated) 
      strip.setPixelColor(NUM_SENSORS, strip.Color(0, 20, 0));
    else
      strip.setPixelColor(NUM_SENSORS, strip.Color(0, 0, 0));
    strip.show();
  }
  if (current_mode == MODE_CALIBRATING) {
  //if (false) {

  //  readSensors(buf);
  // if (show_leds) {
  //   for (int i = 0; i < NUM_SENSORS; i++) {
  //     if (is_calibrated)
  //       strip.setPixelColor(i, strip.Color(0, (uint8_t)(buf[i] / 16), 0));  // Green
  //     else
  //       strip.setPixelColor(i, strip.Color((uint8_t)(buf[i] / 16), 0, 0));  // Red
  //   }
    
  // }

    //doCalibrate();

    // FLASH AFTER ALL OTHER PIXEL UPDATES
    static uint32_t lastFlash = 0;
    static bool flashState = false;
    if (millis() - lastFlash >= FLASH_TIME) {
      lastFlash = millis();
      flashState = !flashState;

      if (flashState) {
        strip.setPixelColor(NUM_SENSORS, strip.Color(40, 0, 0));  // on
      } else {
        strip.setPixelColor(NUM_SENSORS, strip.Color(0, 0, 0));   // off
      }

      strip.show();   // <-- AFTER flashing pixel changed
    }
  }
  
  delay(1);
}
