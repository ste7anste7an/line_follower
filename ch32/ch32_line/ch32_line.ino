/*
  I2C Slave used for reading Line Follwowing ir detectors.
*/

// For USB Serial, enable Adafruit TinyUSB with USBD in settings



enum LogLevel { LOG_ERROR = 0,
                LOG_WARN,
                LOG_INFO,
                LOG_DEBUG,
                LOG_VERBOSE };
static LogLevel CURRENT_LOG_LEVEL = LOG_DEBUG;

// printf-style logging macro using Serial.printf directly
#define LOG(level, fmt, ...) \
  do { \
    if (level <= CURRENT_LOG_LEVEL) { \
      Serial.printf("[%s] " fmt "\n", logLevelStr(level), ##__VA_ARGS__); \
    } \
  } while (0)

inline const char *logLevelStr(LogLevel level) {
  switch (level) {
    case LOG_ERROR: return "ERROR";
    case LOG_WARN: return "WARN";
    case LOG_INFO: return "INFO";
    case LOG_DEBUG: return "DEBUG";
    case LOG_VERBOSE: return "VERBOSE";
    default: return "LOG";
  }
}



// include standard eeprom library for writing calibration values
#include <EEPROM.h>
#include <Adafruit_NeoPixel.h>


#define MAJ_VERSION 2
#define MIN_VERSION 3


#define NUM_SENSORS 8
#define THRESHOLD 50  // ignore reading below 50 for calculating position
#define N_SAMPLES 8   // number of derivative samples to average

#define NEOPIXEL_PIN PB11  // any PAx / PBx pin works
#define CALLIBRATE_PIN PB1 // connected to BOOT0.


Adafruit_NeoPixel strip(NUM_SENSORS + 1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

#define FLASH_TIME 250  // is ms
#define CAL_TIME 5000   // is ms

unsigned long t_cal_on;
unsigned long t_cal_off;
unsigned long t_cal_stop;

int count = 0;

#include <Wire.h>
#define MY_I2C_ADDRESS 0x33

#define CTRL_PIN PB3

typedef enum {
  CMD_SET_MODE_RAW = 0,  //0
  CMD_SET_MODE_CAL,      //1
  CMD_GET_VERSION,       //2
  CMD_DEBUG,             //3
  CMD_CALIBRATE,         //4
  CMD_IS_CALIBRATED,     //5
  CMD_LOAD_CAL,          //6 load calibrated values frpom eeprom
  CMD_SAVE_CAL,          //7 save calibrated values to eeprom
  CMD_GET_MIN,           //8
  CMD_GET_MAX,           //9
  CMD_SET_MIN,           //10
  CMD_SET_MAX,           //11
  CMD_NEOPIXEL,          //12 neopixel: lednr, r, g, b, write
  CMD_LEDS,              //13
  CMD_SET_EMITTER,       //14 optional for qtr sensors
  MAX_CMDS,
} Commands;

typedef enum {
  MODE_RAW = 0,      //0
  MODE_CAL,          //1
  MODE_DIG,          //2
  MODE_CALIBRATING,  //3
} Modes;

typedef enum {
  LEDS_OFF = 0,   //0
  LEDS_NORMAL,    //1
  LEDS_INVERTED,  //2
  LEDS_POSITION,  //3
  MAX_LED_MODE,
} LedsMode;


Modes current_mode = MODE_RAW;
LedsMode current_leds_mode = LEDS_NORMAL;
bool is_calibrated = false;
bool inverted = true;
int nr = 0;
unsigned long t0 = millis();
uint8_t load_cal[2] = { 0, 0 };

bool callibrate = false;
bool lastCalPinState = LOW;

// output: [Sensor_0] [Sensor_1] ... [Sensor_N] [position] [minval] [maxval] [derivative] [shape]

uint8_t rawVals[NUM_SENSORS + 5];
uint8_t calMin[NUM_SENSORS + 5];  // +5 to keep these arrays as long as the measurement array buf
uint8_t calMax[NUM_SENSORS + 5];
uint8_t buf[NUM_SENSORS + 5];
uint8_t norm[NUM_SENSORS + 5];
uint8_t invert[NUM_SENSORS + 5];

uint8_t pos;
uint8_t minval;
uint8_t maxval;

int adcPins[] = { PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PB0, PB1 };  // 10 GPIO's for 10 possible detectors

/* watchdog stuff
// The following code is for setting the Watchdog Timer
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
*/

#define THRESHOLD_BLACK 100  // Anything below = black, above = white

typedef enum {
  LINE_NONE,
  LINE_STRAIGHT,
  LINE_T,
  LINE_L_LEFT,
  LINE_L_RIGHT,
  LINE_Y
} line_shape_t;



uint8_t detect_shape(uint8_t s[]) {
  // Convert to binary (1 = black, 0 = white)
  uint8_t b[8];
  for (int i = 0; i < 8; i++) {
    b[i] = (s[i] < THRESHOLD_BLACK) ? 1 : 0;
  }

  // Count black sensors
  int black_count = 0;
  for (int i = 0; i < 8; i++)
    black_count += b[i];

  // ---------------------------
  // Detect T-junction
  // A T typically shows wide black region
  // like ***####*** (centered block)
  // ---------------------------
  if (black_count == 0) {
    // Middle section thick?
    return LINE_NONE;
  }

  if (black_count >= 6) {
    // Middle section thick?
    if ((b[1] && b[3] && b[4] && b[6])) {
      return LINE_T;
    }
  }
  if (black_count >= 2) {
    if ((b[2] && b[3] && b[6]) || (b[2] && b[4] && b[6])) {
      return LINE_Y;
    }
  }
  // ---------------------------
  // Detect L-left
  // Pattern looks like:
  // ###.....
  // Or more black on left and almost none on right
  // ---------------------------
  if (black_count >= 3) {
    int left = b[0] + b[1] + b[2] + b[3];
    int right = b[4] + b[5] + b[6] + b[7];

    if (left >= 3 && right <= 1) {
      return LINE_L_RIGHT;
    }

    if (right >= 3 && left <= 1) {
      return LINE_L_LEFT;
    }
  }

  return LINE_STRAIGHT;
}



// read raw values and store in global rawVals buffer
void readSensors() {
  for (int i = 0; i < NUM_SENSORS; ++i) {
    pinMode(adcPins[i], INPUT_PULLUP);  // NOT INPUT_PULLUP
    rawVals[i] = 255 - (analogRead(adcPins[i]) >> 4);
  }
}

void initCalibrate() {
  // initialize min/max to extremes
  LOG(LOG_DEBUG, "start calibrating\r\n");
  for (int i = 0; i < NUM_SENSORS + 5; ++i) {
    calMin[i] = 255;
    calMax[i] = 0;
  }
}


void doCalibrate() {
  uint8_t vals[NUM_SENSORS];
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
  }
  is_calibrated = true;
  //LOG(LOG_DEBUG, "is_calibrated set to true\r");
}

// only for qtr sensor of pololu
void setEmitter(uint8_t level) {
  /*
  // code for special 31 level emitter
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
  */
  pinMode(CTRL_PIN, OUTPUT);
  if (level == 0) {
    digitalWrite(CTRL_PIN, LOW);
  } else {
    digitalWrite(CTRL_PIN, HIGH);
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
  } else if (current_leds_mode == LEDS_INVERTED) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (is_calibrated)
        strip.setPixelColor(i, strip.Color(0, 16 - (uint8_t)(buf[i] / 16), 0));  // Green
      else
        strip.setPixelColor(i, strip.Color(16 - (uint8_t)(buf[i] / 16), 0, 0));  // Red
    }
    strip.show();
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

bool getLinePosition(const uint8_t norm[], uint8_t &posOut, uint8_t &minOut, uint8_t &maxOut) {
  long weighted_sum = 0;
  long sum = 0;
  minOut = 255;
  maxOut = 0;
  posOut = 0;  // default value
  int val;
  if (!is_calibrated) return false;
  // hardcoded num sensors !!!
  for (int i = 0; i < NUM_SENSORS; ++i) {
    val = 255 - norm[i];  // invert
    if (val < THRESHOLD) val = 0;
    weighted_sum += val * (i);
    sum += val;
    if (val > maxOut) maxOut = val;
    if (val < minOut) minOut = val;
  }

  long p;
  if (sum > 0) {
    p = (255 * weighted_sum);
    p /= sum;  // p in approx -350..350
  } else
    p = 0;

  posOut = (p - 255) / 7;
  return true;
}


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
  uint8_t buf[NUM_SENSORS + 4] = { 0 };
  //LOG(LOG_DEBUG, "received I2C msg");
  if (nBytes > 0) {
    command = Wire.read();
    nBytes--;
    //LOG(LOG_DEBUG, "received command: %d\r\n", command);
  }

  if (command < MAX_CMDS) {  // valid command
    // read remaining bytes
    //LOG(LOG_DEBUG,"entering command=%d command MAX_CMDS=%d\r",command,MAX_CMDS);
    switch (command) {
      case CMD_SET_MODE_RAW:
        LOG(LOG_DEBUG, "CMD_SET_MODE_RAW\r");
        current_mode = MODE_RAW;
        break;
      case CMD_SET_MODE_CAL:
        LOG(LOG_DEBUG, "CMD_SET_MODE_CAL\r");
        current_mode = MODE_CAL;
        break;
      case CMD_GET_VERSION:
        LOG(LOG_DEBUG, "version %d.%d\r", MAJ_VERSION, MIN_VERSION);
        buf[0] = MAJ_VERSION;
        buf[1] = MIN_VERSION;
        Wire.write(buf, NUM_SENSORS + 5);
        break;
      // case CMD_DEBUG:
      //   LOG(LOG_DEBUG, "CMD_DEBUG");
      //   uint8_t debug = Wire.read();
      //   nBytes--;
      //   CURRENT_LOG_LEVEL = debug;
      //   LOG(LOG_DEBUG, "Debug = %d\r",debug);
      //   break;
      case CMD_CALIBRATE:
        LOG(LOG_DEBUG, "CMD_CALIBRATE\r");
        initCalibrate();
        current_mode = MODE_CALIBRATING;  //current_mode = old_mode;
        t_cal_on = millis();
        t_cal_stop = millis() + CAL_TIME;
        break;
      case CMD_IS_CALIBRATED:
        buf[0] = is_calibrated ? 1 : 0;
        LOG(LOG_DEBUG, "CMD_IS_CALIBRATED: %d\r", buf[0]);
        Wire.write(buf, NUM_SENSORS + 5);
        break;
      case CMD_SAVE_CAL:
        LOG(LOG_DEBUG, "CMD_SAVE_CAL\r");
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
        Serial.println("CMD_LOAD_CAL");
        LOG(LOG_DEBUG, "CMD_LOAD_CAL\r");
        for (int i = 0; i < NUM_SENSORS; i++) {
          calMin[i] = EEPROM.read(i);
          calMax[i] = EEPROM.read(i + NUM_SENSORS);
        }
        is_calibrated = 1;
        break;
      case CMD_GET_MIN:
        LOG(LOG_DEBUG, "CMD_GET_MIN\r");
        Wire.write(calMin, NUM_SENSORS + 5);
        break;
      case CMD_GET_MAX:
        LOG(LOG_DEBUG, "CMD_GET_MAX\r");
        Wire.write(calMax, NUM_SENSORS + 5);
        break;
      case CMD_SET_MIN:
        if (nBytes >= 8) {
          LOG(LOG_DEBUG, "set cal min ");
          for (int i = 0; i < 8; i++) {
            uint8_t val = Wire.read();
            calMin[i] = val;
            LOG(LOG_DEBUG, "%d, ", val);
            nBytes--;
          }
          LOG(LOG_DEBUG, "\r\n");
          load_cal[0] = 1;
          if (load_cal[1] == 1) {
            is_calibrated = true;
          }
        }
        break;
      case CMD_SET_MAX:
        if (nBytes >= 8) {
          LOG(LOG_DEBUG, "set cal max\r");
          for (int i = 0; i < 8; i++) {
            uint8_t val = Wire.read();
            calMax[i] = val;
            LOG(LOG_DEBUG, "%d,", val);
            nBytes--;
          }
          LOG(LOG_DEBUG, "\r\n");
          load_cal[1] = 1;
          if (load_cal[0] == 1) {
            is_calibrated = true;
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
          LOG(LOG_VERBOSE, "nr=%d, r=%d, g=%d, b=%d\r\n", nr_led, r, g, b);
          strip.setPixelColor(nr_led, strip.Color(r, g, b));
          strip.show();  //always show neopixels
        }
        break;
      case CMD_LEDS:
        if (nBytes > 0) {
          uint8_t led_mode = Wire.read();
          nBytes--;
          LOG(LOG_DEBUG, "led mode= %d\r\n", led_mode);
          current_leds_mode = led_mode;
          if (current_leds_mode == LEDS_OFF) {  // switch off neopixels
            strip.clear();
            strip.show();
          }
        }
        break;
      case CMD_SET_EMITTER:
        if (nBytes > 0) {
          uint8_t level = Wire.read();
          nBytes--;
          LOG(LOG_DEBUG, "Emitter level = %d\r", level);
          setEmitter(level);
        }
        break;
      default:
        Serial.println("Default switch case");
        break;
    }
  }
  // read remaining bytes
  LOG(LOG_VERBOSE, "nBytes=%d\r\n", nBytes);

  while (nBytes > 0) {
    uint8_t val = Wire.read();
    nBytes--;
    LOG(LOG_DEBUG, "reading extra byte: %d \r", val);
  }
}

void RequestEvent() {


  nr += 1;
  if (nr == 1000) {
    LOG(LOG_DEBUG, "delay=%d\r\n", millis() - t0);
    t0 = millis();
    nr = 0;
  }
  switch (current_mode) {
    case MODE_RAW:
      Wire.write(rawVals, NUM_SENSORS + 5);
      break;
    case MODE_CAL:
      Wire.write(norm, NUM_SENSORS + 5);
      break;
    case MODE_CALIBRATING:

      Wire.write(rawVals, NUM_SENSORS + 5);
      break;
    default:
      Wire.write(rawVals, NUM_SENSORS + 5);
  }
}


void setup() {
  strip.begin();
  strip.show();  // Turn off pixels
  // start with IR leds off
  setEmitter(1);
  pinMode(CALLIBRATE_PIN, INPUT_PULLDOWN);
  Serial.begin(115200);
  Serial.println("I2C slave @address 0x33");
  EEPROM.begin();  // setup eeprom usage
  current_mode = MODE_RAW;

  // remap pins
  Wire.setSDA(PB9);
  Wire.setSCL(PB8);

  Wire.begin(MY_I2C_ADDRESS);
  // Note: enable I2C slave functionality in /libraries/Wire/src/utility/twi.h to prevent error message for next two lines
  Wire.onReceive(ReceiveEvent);
  Wire.onRequest(RequestEvent);



  for (int n = 0; n < 2; n++) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      strip.setPixelColor(i, strip.Color(0, 0, 40));
      strip.show();
      delay(20);
      strip.setPixelColor(i, strip.Color(0, 0, 0));
      strip.show();
    }
    for (int i = 0; i < NUM_SENSORS; i++) {
      strip.setPixelColor(NUM_SENSORS - i - 1, strip.Color(0, 0, 40));
      strip.show();
      delay(20);
      strip.setPixelColor(NUM_SENSORS - i - 1, strip.Color(0, 0, 0));
      strip.show();
    }
  }
}


uint32_t lastFlash = 0;
bool flashState = false;


void loop() {

  count += 1;
  if (count % 5000 == 0) {
    //Serial.printf("count=%d\n\r", count);
    Serial.printf("mode=%d\r\n", current_mode);
    Serial.printf("is calibrated: %d\r\n", is_calibrated);
  }
  // read new values
  readSensors();

  //============================

  switch (current_mode) {
    case MODE_RAW:
      show_neopixel(rawVals);
      break;
    case MODE_CAL:
      computeNormalized(rawVals, norm);
      if (current_leds_mode != LEDS_POSITION)
        show_neopixel(norm);
      if (getLinePosition(norm, pos, minval, maxval)) {
        if (current_leds_mode == LEDS_POSITION) {
          float fpos = pos * (NUM_SENSORS * 1.0 / 256.0);
          int i = floor(fpos);    // left LED
          float frac = fpos - i;  // blend factor

          int j = i + 1;  // right LED
          if (j >= NUM_SENSORS) j = NUM_SENSORS - 1;

          const int R = 20;
          const int G = 0;
          const int B = 0;

          // Calculate intensity
          int R_i = R * (1.0 - frac);
          int R_j = R * frac;

          // Light LEDs
          strip.clear();
          if (i >= 0 && i < NUM_SENSORS)
            strip.setPixelColor(i, strip.Color(R_i, G, B));
          if (j >= 0 && j < NUM_SENSORS)
            strip.setPixelColor(j, strip.Color(R_j, G, B));
          strip.show();
        }
        norm[NUM_SENSORS] = pos;
        norm[NUM_SENSORS + 1] = minval;
        norm[NUM_SENSORS + 2] = maxval;
        norm[NUM_SENSORS + 4] = detect_shape(norm);
      } else {
        pos = 0;
        norm[NUM_SENSORS] = 0;
        norm[NUM_SENSORS + 1] = 0;
        norm[NUM_SENSORS + 2] = 0;
        norm[NUM_SENSORS + 4] = 0;
      }

      norm[NUM_SENSORS + 3] = computeMovingAverageDerivative(pos);
      break;
    case MODE_CALIBRATING:
      show_neopixel(rawVals);
      doCalibrate();
      //invertBuf(buf,invert);
      break;
  }

  //==========================
   bool currentCalPinState = digitalRead(CALLIBRATE_PIN);
  
  if (currentCalPinState == HIGH && lastCalPinState == LOW) {
    callibrate = !callibrate;  // toggle state
    if (callibrate) {
      
      current_mode = MODE_CALIBRATING;
    } else {
      current_mode = MODE_RAW;
    }
  }

  lastCalPinState = currentCalPinState;

  
  if (current_mode != MODE_CALIBRATING) {
    if (is_calibrated)
      strip.setPixelColor(NUM_SENSORS, strip.Color(0, 20, 0));
    else
      strip.setPixelColor(NUM_SENSORS, strip.Color(0, 0, 0));
    strip.show();
  }
  if (current_mode == MODE_CALIBRATING) {
    // FLASH AFTER ALL OTHER PIXEL UPDATES
    static uint32_t lastFlash = 0;
    static bool flashState = false;
    if (millis() - lastFlash >= FLASH_TIME) {
      lastFlash = millis();
      flashState = !flashState;

      if (flashState) {
        strip.setPixelColor(NUM_SENSORS, strip.Color(0, 0, 40));  // on
      } else {
        strip.setPixelColor(NUM_SENSORS, strip.Color(0, 0, 0));  // off
      }
      strip.show();
    }
    delay(1);
  }
  
}
