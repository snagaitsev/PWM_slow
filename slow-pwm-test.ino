/*
  Slow PWM test
 */

#include "Teensy_Slow_PWM.h"

#define HW_TIMER_INTERVAL_MS        0.01f
#define HW_TIMER_INTERVAL_FREQ      100000L

#define PIN_COUNT 16


int ssr0 = 3;
int ssr1 = 2;
int ssr2 = 5;
int ssr3 = 4;
int ssr4 = 7;
int ssr5 = 6;
int ssr6 = 9;
int ssr7 = 8;
int ssr8 = 11;
int ssr9 = 10;
int ssr10 = 14;
int ssr11 = 12;
int ssr12 = 18;
int ssr13 = 15;
int ssr14 = 22;
int ssr15 = 19;

// these are in ssr order
int pins[] = {
  3,
  2,
  5,
  4,
  7,
  6,
  9,
  8,
  11,
  10,
  14,
  12,
  18,
  15,
  22,
  19
};

// Init Teensy timer TEENSY_TIMER_1
TeensyTimer ITimer(TEENSY_TIMER_1);
// Init Teensy_SLOW_PWM, each can service 16 different ISR-based PWM channels
Teensy_SLOW_PWM ISR_PWM;

void TimerHandler()
{
  ISR_PWM.run();
}

int channels[PIN_COUNT];
float dutyCycles[PIN_COUNT];

float freq = 5.0f;

// the setup routine runs once when you press reset:
void setup() {
  Serial.begin(9600);
  // initialize the digital pin as an output.
  for (int i = 0; i < PIN_COUNT; i++) {
    pinMode(pins[i], OUTPUT);
    dutyCycles[i] = 0.0f;
    channels[i] = ISR_PWM.setPWM(pins[i], freq, dutyCycles[i]);
  }

  if(ITimer.attachInterrupt(HW_TIMER_INTERVAL_FREQ, TimerHandler)) {
    Serial.println("Timer is good");
  } else {
    Serial.println("Timer is bad");
  }
    for (int i = 0; i < PIN_COUNT; i++) {
    pinMode(pins[i], OUTPUT);
    dutyCycles[i] = 0.0;
    ISR_PWM.modifyPWMChannel(channels[i], pins[i], freq, dutyCycles[i]);
  }
}

bool dutySwitch = false;

// the loop routine runs over and over again forever:
void loop() {
  delay(2000);
}
