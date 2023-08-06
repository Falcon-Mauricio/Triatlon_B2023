#ifndef _MOTOR_h
#define _MOTOR_h
#include <Arduino.h>

class Motor{

private:
  int pin_a;
  int pin_b;
  int ch_a;
  int ch_b;
  int frequency = 1000;
  int resolution = 8;

public:
  Motor(int pin_a_in, int pin_b_in, int ch_a_in, int ch_b_in);
  void Forward(int vel);
  void Backward(int vel);
  void Stop();
};

#endif