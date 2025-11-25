#include <ESP32Servo.h>
Servo servo;

const int ledPin = 33;
const int universalPin = 32;
const int servoPin = 26;

void setup() {
  // Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 2, 4);

  servo.attach(servoPin); 

  pinMode(ledPin, OUTPUT);
  pinMode(universalPin, OUTPUT);

  digitalWrite(universalPin, LOW);

  digitalWrite(ledPin, HIGH); delay(500);
  digitalWrite(ledPin, LOW);  delay(500);
  digitalWrite(ledPin, HIGH); delay(500);
  digitalWrite(ledPin, LOW);
}

void loop() {
  if (Serial1.available()) {
    String data = Serial1.readStringUntil('\n');
    float x, y, z;
    int parsed = sscanf(data.c_str(), "%f,%f,%f", &x, &y, &z);

    if (parsed == 3) {
      //магнит
      if(y > 0.9f){
        digitalWrite(ledPin, HIGH);
        digitalWrite(universalPin, HIGH);
      }
      else{
        digitalWrite(ledPin, LOW);
        digitalWrite(universalPin, LOW);
      }
      //cерво
      z = constrain(z, 0.0f, 3.14f);
      int servoAngle = map((int)(z * 100), 0, 314, 0, 180);
      servo.write(servoAngle);

      //пистолет
      if(x > 0.01f){
        digitalWrite(ledPin, HIGH);
        digitalWrite(universalPin, HIGH);
        delay((int)(x * 1000));
        digitalWrite(ledPin, LOW);
        digitalWrite(universalPin, LOW);
        
      }

      // Serial.printf("x=%.2f, y=%.2f, z=%.2f\n", x, y, z);
    }
  }
}