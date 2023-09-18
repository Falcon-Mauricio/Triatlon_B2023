#include <PS4Controller.h>
#include <BluetoothSerial.h>
#include "motor.h"

BluetoothSerial BT;

#define DEBUG 0
#define BT_DEBUG 0
#define DEBUG_VEL 0
// Motores
#define M2A 22
#define M2B 23
#define M1B 19
#define M1A 21

// Sensores
#define SENSOR_2 27
#define SENSOR_3 26
#define SENSOR_1 13
#define SENSOR_4 25
#define SENSOR_5 34
#define SENSOR_6 35
#define SENSOR_7 32
#define SENSOR_8 33

#define LEFT_CNY 27
#define RIGHT_CNY 25
#define BACK_CNY 26

int Cny70_pins[] = {SENSOR_1, SENSOR_2, SENSOR_3, SENSOR_4, SENSOR_5, SENSOR_6, SENSOR_7, SENSOR_8};

#define CNY70_CANT 8

// Control
#define BUZZER 18
#define PUSH_1 16
#define PUSH_2 17

// Constantes

#define BLACK 1
#define WHITE 0
#define YES 1
#define NO 0
#define LEFT 1
#define RIGHT 2

int black_side = BLACK;

class Buzzer
{

private:
  int pin;

public:
  Buzzer(int pin);
  void Use(int duration);
  void Beep(int n, int duration);
};

Buzzer::Buzzer(int pin_in)
{
  pin = pin_in;
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

void Buzzer::Use(int duration)
{
  digitalWrite(pin, HIGH);
  delay(duration);
  digitalWrite(pin, LOW);
}

void Buzzer::Beep(int n, int duration)
{

  for (int i = 1; i < n; i++)
  {

    digitalWrite(pin, HIGH);
    if (DEBUG)
    {
      Serial.println("beeep");
    }
    delay(duration);
    digitalWrite(pin, LOW);
    delay(duration);
  }
  digitalWrite(pin, HIGH);
  if (DEBUG)
  {
    Serial.println("beeep");
  }
  delay(duration);
  digitalWrite(pin, LOW);
}

// Definicion de clase para motores

// Definicion de clase para sensores Sharp

#define MIN_READING 200
#define MAX_READING 3000
#define MIN_MAP 0
#define MAX_MAP 100

class Sharp
{

private:
  int pin;

public:
  Sharp(int pin_in);
  int ReadDigital(int samples, int distance_deadline);
};

Sharp::Sharp(int pin_in)
{
  pin = pin_in;
}

int Sharp::ReadDigital(int samples, int distance_deadline)
{

  int sum = 0;
  int reading = 0;

  for (int i = 0; i < samples; i++)
  {
    reading = analogRead(pin);
    sum += map(reading, MIN_READING, MAX_READING, MIN_MAP, MAX_MAP);
  }

  int average = sum / samples;
  if (DEBUG)
  {
    Serial.println(average);
  }
  return average > distance_deadline;
}

class Button
{

private:
  int pin;
  int cant_pin;

public:
  Button(int pin_in);
  int IsPressed();
  void WaitPressing();
};

// Constructor clase Button
Button::Button(int pin_in)
{
  pin = pin_in;
  pinMode(pin, INPUT);
}

int Button::IsPressed()
{
  return digitalRead(pin);
}

void Button::WaitPressing()
{
  while (digitalRead(pin) == NO)
  {
    continue;
  }
}

// Definicion de clase PID

#define LIMIT_LEN 8

class CNY
{

private:
  int map_min = 0;
  int map_max = 1000;
  int cant_pins;
  int value;
  int values_read[LIMIT_LEN];
  int max_value_read[LIMIT_LEN];
  int min_value_read[LIMIT_LEN];
  int Cny70_pins[LIMIT_LEN];

public:
  CNY(int cny70_pins_in[], int cant);
  void InitializeReadings();
  void Calibrate();
  int ReadFloor();
  int ReadTatamiColor();
  int color_deadline = 500;
};

CNY::CNY(int cny70_pins_in[], int cant)
{

  cant_pins = cant;
  for (int i = 0; i < cant_pins; ++i)
  {
    Cny70_pins[i] = cny70_pins_in[i];
  }
}

void CNY::InitializeReadings()
{

  for (int i = 0; i < cant_pins; i++)
  {
    max_value_read[i] = -10000;
    min_value_read[i] = 10000;
  }
}

void CNY::Calibrate()
{

  for (int i = 0; i < cant_pins; i++)
  {
    int value = analogRead(Cny70_pins[i]);
    if (value < min_value_read[i])
      min_value_read[i] = value;
    else if (value > max_value_read[i])
      max_value_read[i] = value;
  }
  if (DEBUG)
  {
    // Serial.println("CNY_%d: %d, %d ", i, min_value_read[i], max_value_read[i]);
  }
}

int CNY::ReadFloor()
{

  for (int i = 0; i < cant_pins / 2; i++)
  {
    values_read[i] = analogRead(Cny70_pins[i]);
    values_read[i] = map(values_read[i], min_value_read[i], max_value_read[i], map_max, map_min);
    values_read[i] = constrain(values_read[i], map_min, map_max);
    // Serial.println(values_read[i]);
  }

  Serial.println();
  for (int i = cant_pins / 2; i < cant_pins; i++)
  {
    values_read[i] = analogRead(Cny70_pins[i]);
    values_read[i] = map(values_read[i], min_value_read[i], max_value_read[i], map_min, map_max);
    values_read[i] = constrain(values_read[i], map_min, map_max);
    // Serial.println(values_read[i]);
  }

  return (-16 * values_read[0]) + (-8 * values_read[1]) + (-4 * values_read[2]) + (-2 * values_read[3]) + (2 * values_read[4]) + (4 * values_read[5]) + (8 * values_read[6]) + (16 * values_read[7]);

  // return (-16 * values_read[7]) + (-8 * values_read[6]) + (-4 * values_read[5]) + (-2 * values_read[4]) + (2 * values_read[3]) + (4 * values_read[2]) + (8 * values_read[1]) + (16 * values_read[0]);
}

int CNY::ReadTatamiColor()
{
  int value;
  int detected = WHITE;
  for (int i = 0; i < cant_pins; i++)
  {
    value = analogRead(Cny70_pins[i]);

    value = map(value, min_value_read[i], max_value_read[i], map_min, map_max);
    if (value > color_deadline)
    {
      detected = Cny70_pins[i];
    }
  }
  return detected;
}

// Creacion de objetos

Motor rightMotor(M1A, M1B, 14, 13);
Motor leftMotor(M2A, M2B, 12, 11);
Button *confirm_button = new Button(PUSH_2);
Button *count_button = new Button(PUSH_1);
Sharp *left_sharp = new Sharp(SENSOR_8);
Sharp *right_sharp = new Sharp(SENSOR_5);
Sharp *center_sharp = new Sharp(SENSOR_7);
Sharp *left_side_sharp = new Sharp(SENSOR_1);
Sharp *right_side_sharp = new Sharp(SENSOR_6);
Buzzer *buzzer = new Buzzer(BUZZER);
int line_follower_array[] = {SENSOR_1, SENSOR_2, SENSOR_3, SENSOR_4, SENSOR_5, SENSOR_6, SENSOR_7, SENSOR_8};
int area_cleaner_array[] = {LEFT_CNY, RIGHT_CNY};
CNY *line_follower = new CNY(line_follower_array, 8);
CNY *area_cleaner = new CNY(area_cleaner_array, 2);

// ------------------ INICIALIZADOR ---------------------------------------------------------------------------

void initialize(void (*category_loop_function)())
{

  confirm_button->WaitPressing();
  if (DEBUG)
  {
    Serial.println("iniciando");
  }
  buzzer->Beep(3, 1000);
  if (DEBUG)
  {
    Serial.println("Arranque brrr");
  }

  while (true)
  {
    category_loop_function();
  }
}

// -------------------- VELOCISTA -----------------------------------------------------------------------------

#define VEL_WHITE_FLOOR 150
#define COMPENSATION_DEADLINE 80
int PID_VEL_MIN = 190;
bool all_white = false;
int in_straight = YES;
int right_vel = 0;
int left_vel = 0;
int base_velocity = PID_VEL_MIN;
unsigned long time_stamp;
int values_read[CNY70_CANT];
int max_value_read[CNY70_CANT];
int min_value_read[CNY70_CANT];
float kp = 10;
float kd = 4;
float ki = 0;
int error = 0;
int last_error = 0;
int actual_value = 0;
int integral = 0;
int PID_calc;
float gain = 120;
float pid_max_average = (kp * 15600) + ki * 0 + (kd * (15600 - -15600));
float pid_min_average = (kp * -15600) + ki * 0 + (kd * (-15600 - 15600));

/*
void line_follower_telemetry(){
  if(BT.available()){
    String in = BT.readStringUntil('\n');
    in.trim();
    if (in == "p"){
        while(true){
        leftMotor.Stop();
        rightMotor.Stop();

        if (BT.available()){
          in = BT.readStringUntil('\n');
          in.trim();
        }

        if(in == "kpup"){ kp += 1; BT.println(kp); }
        else if(in == "kpdw"){ kp -= 1; BT.println(kp); }
        else if(in == "kdup"){ kd += 0.2; BT.println(kd); }
        else if(in == "kddw"){ kd -= 0.2; BT.println(kd); }
        else if(in == "velup"){ PID_VEL_MIN += 5; BT.println(PID_VEL_MIN); }
        else if(in == "veldw"){ PID_VEL_MIN -= 5; BT.println(PID_VEL_MIN); }
        if(in == "s") break;
        in = "";

        }
    }

  }
}
*/
void Motor_control(int value)
{

  if (in_straight)
  {
    if (millis() > time_stamp + 500)
    {
      time_stamp = millis();
      base_velocity += 5;
    }
  }
  else
    base_velocity = PID_VEL_MIN;

  if (black_side == LEFT)
  {

    right_vel = base_velocity - value;
    left_vel = base_velocity + value;
  }
  else
  {
    right_vel = base_velocity + value;
    left_vel = base_velocity - value;
  }

  right_vel = constrain(right_vel, 0, 255);
  left_vel = constrain(left_vel, 0, 255);
  if (right_vel > 255)
  {
    right_vel = 255;
  }
  if (left_vel > 255)
  {
    right_vel = 255;
  }

  if (all_white)
  {

    if (black_side == RIGHT)
    {
      rightMotor.Backward(110);
      leftMotor.Forward(VEL_WHITE_FLOOR);
    }
    else
    {
      leftMotor.Backward(110);
      rightMotor.Forward(VEL_WHITE_FLOOR);
    }
  }
  else
  {

    if (right_vel < COMPENSATION_DEADLINE)
    {
      rightMotor.Backward(COMPENSATION_DEADLINE + (COMPENSATION_DEADLINE - right_vel));
    }
    else
    {
      rightMotor.Forward(right_vel);
    }
    if (left_vel < COMPENSATION_DEADLINE)
    {

      leftMotor.Backward(COMPENSATION_DEADLINE + (COMPENSATION_DEADLINE - left_vel));
    }
    else
    {
      leftMotor.Forward(left_vel);
    }
  }
}

void line_follower_loop()
{

  actual_value = line_follower->ReadFloor();
  // Serial.println(actual_value);

  // line_follower_telemetry();

  error = actual_value;
  integral += error;
  integral = constrain(integral, -100000, 100000);

  all_white = actual_value < -26000;

  PID_calc = (kp * error + ki * integral + kd * (error - last_error));

  PID_calc = map(PID_calc, pid_min_average, pid_max_average, -1 * gain, gain);

  if (PID_calc < 50 && PID_calc > -50)
  {
    in_straight = YES;
  }
  else
  {
    in_straight = NO;
  }

  last_error = error;

  if (actual_value < pid_min_average)
  {
    pid_min_average = actual_value;
  }
  if (actual_value > pid_max_average)
  {
    pid_max_average = actual_value;
  }
  // Serial.println("PID");
  Motor_control(PID_calc);
}

void line_follower_setup()
{

  // BT.begin("line_follower_debug");

  /*
    xTaskCreatePinnedToCore(
    ReadSensorsFun, // Function to implement the task
    "Task1",        // Name of the task
    4096,           // Stack size in words
    NULL,           // Task input parameter
    0,              // Priority of the task
    &ReadSensors,   // Task handle.
    0);             // Core where the task should run
  */

  line_follower->InitializeReadings();

  if (DEBUG)
  {
    Serial.println("CALIBRANDO, select");
  }

  while (count_button->IsPressed() == NO)
  {
    line_follower->Calibrate();
  }

  if (DEBUG)
  {
    Serial.println("Calibrado");
  }

  initialize(line_follower_loop);
}

// -------------------- SUMO RC -------------------------------------------------------------------------------
int velocity, turn_velocity;

void radio_controlled_loop()
{

  int y_axis_value = PS4.LStickY();
  int x_axis_value = PS4.RStickX();
  int l1 = PS4.L2Value();
  int r2 = PS4.R2Value();

  if (r2 >= 50)
  {
    velocity = 250;
    turn_velocity = 150;
  }

  if (l1 >= 50)
  {
    velocity = 100;
    turn_velocity = 50;
  }

  if (r2 < 50 && l1 < 50)
  {
    velocity = 170;
    turn_velocity = 70;
  }

  if (y_axis_value >= 50)
  {
    rightMotor.Forward(velocity);
    leftMotor.Forward(velocity);

    if (x_axis_value >= 50)
    {

      leftMotor.Forward(velocity);
      rightMotor.Forward(turn_velocity);
    }

    else if (x_axis_value <= -50)
    {

      leftMotor.Forward(turn_velocity);
      rightMotor.Forward(velocity);
    }
  }
  else if (y_axis_value <= -50) // Move car Backward
  {
    rightMotor.Backward(velocity);
    leftMotor.Backward(velocity);

    if (x_axis_value >= 50)
    {
      rightMotor.Backward(turn_velocity);
      leftMotor.Backward(velocity);
    }
    else if (x_axis_value <= -50)
    {
      rightMotor.Backward(velocity);
      leftMotor.Backward(turn_velocity);
    }
  }
  else if (x_axis_value >= 50) // Move car Right
  {
    rightMotor.Backward(150);
    leftMotor.Forward(250);
  }
  else if (x_axis_value <= -50) // Move car Left
  {
    rightMotor.Forward(250);
    leftMotor.Backward(150);
  }
  else // Stop the car
  {
    rightMotor.Stop();
    leftMotor.Stop();
  }
}

void radio_controlled_setup()
{

  PS4.begin();
  while (!PS4.isConnected())
    ;
  buzzer->Use(1000);
  PS4.attach(radio_controlled_loop);
}

// ------------------ DESPEJAR AREA ---------------------------------------------------------------------------

// Detection cases

enum CASES
{
  SEE_EDGE = -1,
  SEE_VOID = 0,
  SEE_LEFT = 1,
  SEE_CENTER = 2,
  SEE_LEFT_CENTER = 3,
  SEE_RIGHT = 4,
  SEE_RIGHT_CENTER = 6,
};

// Constants
#define LINE_REBOUND_TIME 800
#define LIMIT_BLIND_ADVANCING_TIME 1000
#define ON_AXIS 1
#define ON_FORWARD 0
#define SECURITY_TURN_TIME 100

int seek_velocity = 80;
int max_distance = 20;
int blind_turn_diference = 100;
#define TURN_ADJUSTMENT_DIFERENCE 100
int MAX_VELOCITY = 160;
int BACK_VEL = 160;
#define BLIND_LIMIT_TIME 2000

// timers
unsigned long seeking_time;
unsigned long last_rebound_time;
unsigned long saw_side_time;

// flags
int rebound_flag = NO;
int first_blind = YES;
int first_loop = YES;

// variables

void area_cleaner_telemetry()
{
  if (BT.available())
  {
    String in = BT.readStringUntil('\n');
    in.trim();
    if (in == "p")
    {
      while (true)
      {
        leftMotor.Stop();
        rightMotor.Stop();

        if (BT.available())
        {
          in = BT.readStringUntil('\n');
          in.trim();
        }

        if (in == "atkup")
        {
          MAX_VELOCITY += 5;
          BT.println(MAX_VELOCITY);
        }
        else if (in == "atkdw")
        {
          MAX_VELOCITY -= 5;
          BT.println(MAX_VELOCITY);
        }
        else if (in == "seekup")
        {
          seek_velocity += 5;
          BT.println(seek_velocity);
        }
        else if (in == "seekdw")
        {
          seek_velocity -= 5;
          BT.println(seek_velocity);
        }
        else if (in == "delup")
        {
          blind_turn_diference += 100;
          BT.println(blind_turn_diference);
        }
        else if (in == "deldw")
        {
          blind_turn_diference -= 100;
          BT.println(blind_turn_diference);
        }
        if (in == "s")
          break;
        in = "";
      }
    }
  }
}

int binaryAreaCleanerSum(int last_cny_value)
{

  if (last_cny_value != WHITE)
  {
    return SEE_EDGE;
  }

  return (left_sharp->ReadDigital(10, max_distance) + center_sharp->ReadDigital(10, max_distance) * 2 + right_sharp->ReadDigital(10, max_distance) * 4);
}

void rebound(int time_rebound, int back_velocity, int last_cny_value, int last_value)
{
  unsigned long time = millis();
  while (millis() < (time + time_rebound))
  {

    leftMotor.Backward(back_velocity);
    rightMotor.Backward(back_velocity);

    if (analogRead(BACK_CNY) > 2000)
    {

      if (last_cny_value == LEFT_CNY)
      {
        leftMotor.Forward(200);
        rightMotor.Forward(120);
        delay(500);
      }
      else if (last_cny_value == RIGHT_CNY)
      {
        leftMotor.Forward(120);
        rightMotor.Forward(200);
        delay(500);
      }
      break;
    }
  }

  if (last_value == LEFT)
  {
    leftMotor.Backward(MAX_VELOCITY);
    rightMotor.Forward(MAX_VELOCITY);
  }
  else if (last_value == RIGHT)
  {
    rightMotor.Backward(MAX_VELOCITY);
    leftMotor.Forward(MAX_VELOCITY);
  }
  delay(SECURITY_TURN_TIME);
}

void move_on_axis(int axis)
{
  if (axis == LEFT)
  {
    rightMotor.Forward(seek_velocity);
    leftMotor.Backward(seek_velocity);
  }
  else if (axis == RIGHT)
  {
    leftMotor.Forward(seek_velocity);
    rightMotor.Backward(seek_velocity);
  }
}

void move_with_chanfle(int axis)
{
  if (axis == LEFT)
  {
    leftMotor.Forward(seek_velocity + blind_turn_diference);
    rightMotor.Forward(seek_velocity);
  }
  else if (axis == RIGHT)
  {
    rightMotor.Forward(seek_velocity + blind_turn_diference);
    leftMotor.Forward(seek_velocity);
  }
}

int previous_value;

int modo_busqueda = ON_AXIS;
int last_value = RIGHT;

void area_cleaner_loop()
{

  /*
  if(first_loop){
  leftMotor.Forward(120);
  rightMotor.Forward(120);
  delay(1000);
  }
  */

  // last_change: Giro en su eje despues de rebotar, habiendo detectado en ataque
  area_cleaner->Calibrate();

  
  int last_cny_value = area_cleaner->ReadTatamiColor();
  int binary_area_cleaner_sum = binaryAreaCleanerSum(last_cny_value);
  bool object_side_left_detect = left_side_sharp->ReadDigital(10, max_distance);
  bool object_side_right_detect = right_side_sharp->ReadDigital(10, max_distance);

  if(binary_area_cleaner_sum == SEE_VOID){
    if(previous_value == SEE_RIGHT) last_value = RIGHT;
    else if(previous_value == SEE_LEFT) last_value = LEFT;
  }

  if (object_side_left_detect || object_side_right_detect)
  {
    modo_busqueda = ON_AXIS;
    saw_side_time = millis();
  }

  if (object_side_left_detect)
  {

    last_value = LEFT;
  }

  previous_value = binary_area_cleaner_sum;

  switch (binary_area_cleaner_sum)
  {

  case SEE_EDGE:
  {
    int time_rebound = LINE_REBOUND_TIME;
    int back_velocity = BACK_VEL;

    if (object_side_left_detect || object_side_right_detect)
    {
      time_rebound = millis() - saw_side_time;
      back_velocity = MAX_VELOCITY;
    }
    rebound(time_rebound, back_velocity, last_cny_value, last_value);

    rebound_flag = YES;
    last_rebound_time = millis();
    break;
  }
  case SEE_VOID:
  { // no ve nada

    if (millis() > last_rebound_time + BLIND_LIMIT_TIME && rebound_flag == YES)
    {

      rebound_flag = NO;
      if (area_cleaner->ReadTatamiColor() == WHITE)
      {
        modo_busqueda == ON_FORWARD;
        first_blind = NO;
      }
    }
    else
    {

      if (modo_busqueda == ON_AXIS)
      {
        move_on_axis(last_value);
      }
      else
      {
        move_with_chanfle(last_value);
      }
    }

    break;
  }

  case SEE_LEFT:
  case SEE_LEFT_CENTER:
    leftMotor.Forward(seek_velocity);
    rightMotor.Forward(seek_velocity + TURN_ADJUSTMENT_DIFERENCE);
    break;

  case SEE_RIGHT:
  case SEE_RIGHT_CENTER:
    leftMotor.Forward(seek_velocity + TURN_ADJUSTMENT_DIFERENCE);
    rightMotor.Forward(seek_velocity);
    break;

  case SEE_CENTER:
  default:
    leftMotor.Forward(MAX_VELOCITY);
    rightMotor.Forward(MAX_VELOCITY);
    break;
  }
}

void area_cleaner_setup()
{

  /*
  if(BT_DEBUG){
  BT.begin();
  while (!BT.connected())
  ;
  buzzer->Use(1000);
  }
  */
  area_cleaner->InitializeReadings();

  if (DEBUG)
  {
    Serial.println("CALIBRANDO, select");
  }

  while (count_button->IsPressed() == NO)
  {
    area_cleaner->Calibrate();
  }

  last_rebound_time = millis();
  initialize(area_cleaner_loop);
}

#define MAX_MODES 4
#define MIN_MODES 1

// Funciones para seleccion e inicializacion de modos
void printMode(int mode)
{
  int prev_mode = 1000;
  String modes[MAX_MODES] = {"LINE FOLLOW LEFT", "LINE FOLLOW RIGHT", "AREA CLEAN", "RADIO CONTROLLER"};
  String mode_text = "[mode]: ";

  if (prev_mode != mode)
  {
    mode_text.concat(modes[mode + 1]);
    Serial.println(mode_text);
  }
}

enum MODE
{
  MODE_LINE_FOLLOWER_LEFT = 1,
  MODE_LINE_FOLLOWER_RIGHT = 2,
  MODE_AREA_CLEANER = 3,
  MODE_RADIO_CONTROLLER = 4,
};

int swichModeByButtonPress()
{
  int mode = 0;
  while (true)
  {

    if (count_button->IsPressed() == YES)
    {
      if (++mode > MAX_MODES)
      {
        mode = MIN_MODES;
      }

      buzzer->Beep(mode, 100);
      delay(100);
    }
    if (confirm_button->IsPressed() == YES && mode != 0)
    {
      break;
    }
  }
  printMode(mode);
  return mode;
}

void mode_selection()
{

  int mode = swichModeByButtonPress();
  switch (mode)
  {
  case MODE_LINE_FOLLOWER_LEFT:
  {
    black_side = LEFT;
    line_follower_setup();
    break;
  }
  case MODE_LINE_FOLLOWER_RIGHT:
  {
    black_side = RIGHT;
    line_follower_setup();
    break;
  }
  case MODE_AREA_CLEANER:
  {
    area_cleaner_setup();
    break;
  }
  case MODE_RADIO_CONTROLLER:
  {
    radio_controlled_setup();
    break;
  }
  }
}

void setup()
{
  Serial.begin(115200);
  if (DEBUG)
  {
    Serial.println("inicio programa");
  }
  mode_selection();
}

void loop()
{
}
