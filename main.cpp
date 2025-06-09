#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Servo.h>

// Motor A connections
// don't use pin 9, servo conflict
int enA = 6;
int in1 = 8;
int in2 = 7;
// Motor B connections
int enB = 3;
int in3 = 5;
int in4 = 4;

Servo myServo;
int servoPin = 10;

// Speed variable received from App Inventor
byte motorSpeed = 0;

// Servo control
int servoAngle = 20; // 50 for prev // 0이면 flywheel쪽으로 더 깊게
int servoAngleEnd = 260; // 240 for previous loading mechanism
// 흰색 기어 마운트 돌려서 서보 각도 조절할 수도 있음
bool increasing = true;
unsigned long lastServoMove = 0;
const unsigned long interval = 1000; // 800 seconds

void directionControl(byte speed);

void setup() {
  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  myServo.attach(servoPin);
  myServo.write(servoAngle);

  Serial.begin(9600); // Start serial communication
}

void loop() {
  // Read speed from App Inventor if available
  if (Serial.available() > 0) {
    motorSpeed = Serial.read();
    Serial.print("Received speed: ");
    Serial.println(motorSpeed);
  }
  directionControl(motorSpeed);
    
  // Non-blocking servo movement every 1.5 seconds
  unsigned long currentMillis = millis();
  if (currentMillis - lastServoMove >= interval) {
    lastServoMove = currentMillis;
    if (increasing) {
      servoAngle = servoAngleEnd;
    } else {
      servoAngle = 0;
    }
    myServo.write(servoAngle);
    increasing = !increasing; // Toggle direction
  }
}
