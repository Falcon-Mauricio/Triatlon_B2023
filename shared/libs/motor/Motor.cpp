#include "motor.h"
#include <Arduino.h>

Motor::Motor(int pin_a_in, int pin_b_in, int ch_a_in, int ch_b_in) {
  pin_a = pin_a_in;
  pin_b = pin_b_in;
  ch_a = ch_a_in;
  ch_b = ch_b_in;

  ledcSetup(ch_a, frequency, resolution);
  ledcSetup(ch_b, frequency, resolution);
  ledcAttachPin(pin_a, ch_a);
  ledcAttachPin(pin_b, ch_b);
}

// Metodos motores
void Motor::Forward(int vel) {
  ledcWrite(ch_a, vel);
  ledcWrite(ch_b, 0);
}
void Motor::Backward(int vel) {
  ledcWrite(ch_a, 0);
  ledcWrite(ch_b, vel);
}
void Motor::Stop() {
  ledcWrite(ch_a, 0);
  ledcWrite(ch_b, 0);
}