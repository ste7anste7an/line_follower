// Low-level Pololu QTR-RC array reader without using micros()
// Works for sensors like QTR-8RC

const uint8_t qtrPins[] = {PA0, PA1, PA2, PA3, PA4};
const uint8_t numSensors = sizeof(qtrPins);
uint16_t values[numSensors];

void setup() {
  Serial.begin(115200);
}

void loop() {
  readQTRArray();
  for (uint8_t i = 0; i < numSensors; i++) {
    Serial.print(values[i]);
    Serial.print('\t');
  }
  Serial.println();
  delay(100);
}

void readQTRArray() {
  // 1️⃣ Charge all sensors
  for (uint8_t i = 0; i < numSensors; i++) {
    pinMode(qtrPins[i], OUTPUT);
    digitalWrite(qtrPins[i], HIGH);
  }
  delayMicroseconds(10);

  // 2️⃣ Set as inputs to start discharge
  for (uint8_t i = 0; i < numSensors; i++) {
    pinMode(qtrPins[i], INPUT);
  }

  // 3️⃣ Measure discharge with step-based sampling
  const uint16_t maxSteps = 3000; // max count before timeout
  for (uint8_t i = 0; i < numSensors; i++) {
    values[i] = maxSteps; // default timeout
  }

  for (uint16_t t = 0; t < maxSteps; t++) {
    for (uint8_t i = 0; i < numSensors; i++) {
      if (values[i] == maxSteps && digitalRead(qtrPins[i]) == LOW) {
        values[i] = t; // record when sensor goes LOW
      }
    }
    delayMicroseconds(1); // 1 µs sampling step
  }
}
