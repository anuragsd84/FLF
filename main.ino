
#include <Servo.h>
#include <QTRSensors.h>

const int calibPin = 31;      // calibration button (was requested)
const int onOffPin = 32;      // on/off toggle button
const int LEFT_ESC_PIN = 3;
const int RIGHT_ESC_PIN = 5;

QTRSensors qtr;
const uint8_t SensorCount = 16;
uint16_t sensorValues[SensorCount];

#define maxSpeed 1450
const int basespeeda = 1300;  // 180 160 // 170
const int basespeedb = 1300;  // 180 160 170

#define numCalibrationReadings 500

uint16_t lastPosition = 0;

//PID
float Kp = 0.125;
float Ki = 0.00;
float Kd = 0.000;  //0.035;

int PL;
int IL = 0; // initialize integral accumulator
int DL;

int lastError = 0;

Servo leftEsc;  // create servo object to control a servo
Servo rightEsc;

int count = 0;

int pos = 1000;    // variable to store the servo position

bool running = false;             // whether PID_line() should run
unsigned long lastToggleTime = 0; // debounce timestamp

void setup() {
  pinMode(calibPin, INPUT_PULLUP);
  pinMode(onOffPin, INPUT_PULLUP);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A0,A1,A2,A3,A4,A5,A6,A7,A8,A9,A12,A13,A14,A15,A16,A17}, SensorCount);

  leftEsc.attach(LEFT_ESC_PIN, 1000, 2000);  // attaches the servo on pin 9 to the servo object
  rightEsc.attach(RIGHT_ESC_PIN, 1000, 2000);
  leftEsc.writeMicroseconds(1000);
  rightEsc.writeMicroseconds(1000);
  delay(2000);           

  // Wait for calibration button press (calibPin = 31)
  int buttonState = digitalRead(calibPin);
  while (buttonState == HIGH) {
    buttonState = digitalRead(calibPin);
  }

  // Calibration loop (same as before)
  for (uint16_t i = 0; i < 400; i++) {
    mspeed(1000, 1300);
    qtr.calibrate();
  }
  leftEsc.writeMicroseconds(1000);
  rightEsc.writeMicroseconds(1000);
  //delay(2000);  

  // Wait for on/off button press to start (onOffPin = 32)
  buttonState = digitalRead(onOffPin);
  while (buttonState == HIGH) {
    buttonState = digitalRead(onOffPin);
  }
  // small settle
  delay(1000);

  // start running after this press
  running = true;
  lastToggleTime = millis();
}

void mspeed(int posa, int posb) {
  leftEsc.writeMicroseconds(posa);
  rightEsc.writeMicroseconds(posb);
}

void PID_line() {

    uint16_t position = qtr.readLineBlack(sensorValues);
    int error = 7500 - position;

    PL = error;
    IL = IL + error;
    DL = error - lastError;
    lastError = error;
    int motorspeed = PL * Kp + IL * Ki + DL * Kd;

    //const int trim_left =100;

    int motorspeeda = basespeeda + motorspeed ;
    int motorspeedb = basespeedb - motorspeed;

    if (motorspeeda > maxSpeed) {
      motorspeeda = maxSpeed;
    }
    if (motorspeedb > maxSpeed) {
      motorspeedb = maxSpeed;
    }
    if (motorspeeda < 0) {
      motorspeeda = 0;
    }
    if (motorspeedb < 0) {
      motorspeedb = 0;
    }
    mspeed(motorspeeda, motorspeedb);
}

void loop() {
  // Read on/off button and toggle running on each press (debounced)
  int onState = digitalRead(onOffPin);
  if (onState == LOW && (millis() - lastToggleTime) > 200) { // button pressed (active LOW)
    running = !running;             // toggle run/stop
    lastToggleTime = millis();
    if (!running) {
      // turn off motors (idle)
      leftEsc.writeMicroseconds(1000);
      rightEsc.writeMicroseconds(1000);
    }
    // small debounce wait
    delay(50);
  }

  if (running) {
    PID_line();
  } else {
    // ensure motors remain idle while stopped
    leftEsc.writeMicroseconds(1000);
    rightEsc.writeMicroseconds(1000);
  }
}
