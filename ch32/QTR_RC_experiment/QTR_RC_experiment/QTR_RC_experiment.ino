#define PIN PB3

void setup() {
  Serial.begin(115200);
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN, LOW);
}

uint8_t measure[100];
void loop() {
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
    delayMicroseconds(100);  // charge time
  pinMode(PIN, INPUT);
  for (int i=0; i<100; i++ ) {
    if (digitalRead(PIN) == HIGH) {
      measure[i]=1;
    } else
      measure[i]=0;
    delayMicroseconds(20);  // charge time
  } 
  for (int i=0; i<100; i++) {
    Serial.print(",");
    Serial.print(measure[i]);
  }
  Serial.println();
  
}