#include <Arduino.h>

const int PIN_PWM = 9;
const int PIN_EN = 2;
const int PIN_VOLT = A0;
const int PIN_CURR = A1;

const float DIVIDER_RATIO = 124.0;
const float R_SHUNT = 0.04;
const float AMC_GAIN = 8.0;

const float MAX_CURRENT_AMPS = 5.0;
const float TARGET_VOLTAGE = 180.0;
const float MAX_DUTY_LIMIT = 0.85;

const int TARGET_ADC = (int)((TARGET_VOLTAGE / DIVIDER_RATIO) * (1023.0 / 5.0));
const int MAX_DUTY_RAW = (int)(MAX_DUTY_LIMIT * 255.0);

const float Kp = 0.5;
const float Ki = 0.04;

float setpoint = 0;
float integralError = 0;
unsigned long lastRampTime = 0;

void setup() {
  Serial.begin(115200);
  pinMode(PIN_EN, INPUT_PULLUP);
  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_VOLT, INPUT);
  pinMode(PIN_CURR, INPUT);

  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1A = _BV(COM1A1) | _BV(WGM10);
  TCCR1B = _BV(CS10);
}

void loop() {
  int rawCurrentADC = analogRead(PIN_CURR);
  float pinVoltage = rawCurrentADC * (5.0 / 1023.0);
  float shuntVoltage = pinVoltage / AMC_GAIN;
  float actualAmps = shuntVoltage / R_SHUNT;

  if (actualAmps > MAX_CURRENT_AMPS) {
    OCR1A = 0;
    while(1);
  }

  if (digitalRead(PIN_EN) == HIGH) {
    OCR1A = 0;
    setpoint = 0;
    integralError = 0;
    return;
  }

  if (millis() - lastRampTime >= 15) {
    if (setpoint < TARGET_ADC) {
      setpoint += 1.0;
    }
    lastRampTime = millis();
  }

  int inputADC = analogRead(PIN_VOLT);

  if (inputADC > (TARGET_ADC * 1.15)) {
    OCR1A = 0;
    while(1);
  }

  float error = setpoint - inputADC;
  integralError += error;

  if (integralError > 4000) integralError = 4000;
  if (integralError < -4000) integralError = -4000;

  float outputRaw = (Kp * error) + (Ki * integralError);

  if (outputRaw < 0) outputRaw = 0;
  if (outputRaw > MAX_DUTY_RAW) outputRaw = MAX_DUTY_RAW;

  OCR1A = (int)outputRaw;

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 250) {
    float realVoltage = (inputADC * (5.0 / 1023.0)) * DIVIDER_RATIO;
    Serial.print("V:");
    Serial.print(realVoltage, 1);
    Serial.print(" I:");
    Serial.println(actualAmps, 3);
    lastPrint = millis();
  }
}