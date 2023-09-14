#include <PS4Controller.h>
#include <BluetoothSerial.h>
#include "motor.h"

BluetoothSerial BT;

#define DEBUG 0
#define DEBUG_VEL 0
// Motores
//
#define M2A 19
#define M2B 21

// RIGHT
#define M1A 23
#define M1B 22

// Sensores
#define SENSOR_1 13
#define SENSOR_2 27
#define SENSOR_3 26
#define SENSOR_4 25
#define SENSOR_5 34
#define SENSOR_6 35
#define SENSOR_7 32
#define SENSOR_8 33

#define LEFT_CNY SENSOR_5
#define RIGHT_CNY SENSOR_4
#define BACK_CNY SENSOR_2

int Cny70_pins[] = { SENSOR_1, SENSOR_2, SENSOR_3, SENSOR_4, SENSOR_5, SENSOR_6, SENSOR_7, SENSOR_8 };

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

class Buzzer {

private:
  int pin;

public:
  Buzzer(int pin);
  void Use(int duration);
  void Beep(int n, int duration);
};

Buzzer::Buzzer(int pin_in) {
  pin = pin_in;
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

void Buzzer::Use(int duration) {
  digitalWrite(pin, HIGH);
  delay(duration);
  digitalWrite(pin, LOW);
}

void Buzzer::Beep(int n, int duration) {

  for (int i = 1; i < n; i++) {

    digitalWrite(pin, HIGH);
    if (DEBUG) Serial.println("beeep");
    delay(duration);
    digitalWrite(pin, LOW);
    delay(duration);
  }
  digitalWrite(pin, HIGH);
  if (DEBUG) Serial.println("beeep");
  delay(duration);
  digitalWrite(pin, LOW);
}

// Definicion de clase para motores


// Definicion de clase para sensores Sharp

#define MIN_READING 200
#define MAX_READING 3000
#define MIN_MAP 0
#define MAX_MAP 100


class Sharp {

private:
  int pin;

public:
  Sharp(int pin_in);
  int ReadDigital(int samples, int distance_deadline);
};

Sharp::Sharp(int pin_in) {
  pin = pin_in;
}

int Sharp::ReadDigital(int samples, int distance_deadline) {

  int sum = 0;
  int reading = 0;

  for (int i = 0; i < samples; i++) {
    reading = analogRead(pin);
    sum += map(reading, MIN_READING, MAX_READING, MIN_MAP, MAX_MAP);
  }

  int average = sum / samples;
  if (DEBUG) Serial.println(average);
  return average > distance_deadline;
}


class Button {

private:
  int pin;
  int cant_pin;

public:
  Button(int pin_in);
  int IsPressed();
  void WaitPressing();
};

// Constructor clase Button
Button::Button(int pin_in) {
  pin = pin_in;
  pinMode(pin, INPUT);
}

int Button::IsPressed() {
  return !digitalRead(pin);
}

void Button::WaitPressing() {
  while (!digitalRead(pin) == NO)
    continue;
}

// Definicion de clase PID

#define LIMIT_LEN 8

class CNY {

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

CNY::CNY(int cny70_pins_in[], int cant) {

  cant_pins = cant;
  for (int i = 0; i < cant_pins; ++i) {
    Cny70_pins[i] = cny70_pins_in[i];
  }
}


void CNY::InitializeReadings() {

  for (int i = 0; i < cant_pins; i++) {
    max_value_read[i] = -10000;
    min_value_read[i] = 10000;
  }
}

void CNY::Calibrate() {

  for (int i = 0; i < cant_pins; i++) {
    int value = analogRead(Cny70_pins[i]);
    if (value < min_value_read[i]) min_value_read[i] = value;
    else if (value > max_value_read[i]) max_value_read[i] = value;
  }
  if (DEBUG) {
    //Serial.println("CNY_%d: %d, %d ", i, min_value_read[i], max_value_read[i]);
  }
}

int CNY::ReadFloor() {

  for (int i = 0; i < cant_pins / 2; i++) {
    values_read[i] = analogRead(Cny70_pins[i]);
    values_read[i] = map(values_read[i], min_value_read[i], max_value_read[i], map_max, map_min);
    values_read[i] = constrain(values_read[i], map_min, map_max);
    //Serial.println(values_read[i]);
  }

  Serial.println();
  for (int i = cant_pins / 2; i < cant_pins; i++) {
    values_read[i] = analogRead(Cny70_pins[i]);
    values_read[i] = map(values_read[i], min_value_read[i], max_value_read[i], map_min, map_max);
    values_read[i] = constrain(values_read[i], map_min, map_max);
    //Serial.println(values_read[i]);
  }

  return (-16 * values_read[0]) + (-8 * values_read[1]) + (-4 * values_read[2]) + (-2 * values_read[3]) + (2 * values_read[4]) + (4 * values_read[5]) + (8 * values_read[6]) + (16 * values_read[7]);

  //return (-16 * values_read[7]) + (-8 * values_read[6]) + (-4 * values_read[5]) + (-2 * values_read[4]) + (2 * values_read[3]) + (4 * values_read[2]) + (8 * values_read[1]) + (16 * values_read[0]);
}


int CNY::ReadTatamiColor() {
  int value;
  int detected = WHITE;
  for (int i = 0; i < cant_pins; i++) {
    value = analogRead(Cny70_pins[i]);

    value = map(value, min_value_read[i], max_value_read[i], map_min, map_max);
    if (value > color_deadline) {
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
Sharp *left_sharp = new Sharp(SENSOR_6);
Sharp *right_sharp = new Sharp(SENSOR_8);
Sharp *center_sharp = new Sharp(SENSOR_7);
Sharp *left_side_sharp = new Sharp(SENSOR_1);
Sharp *right_side_sharp = new Sharp(SENSOR_3);
Buzzer *buzzer = new Buzzer(BUZZER);
int line_follower_array[] = { SENSOR_1, SENSOR_2, SENSOR_3, SENSOR_4, SENSOR_5, SENSOR_6, SENSOR_7, SENSOR_8 };
int area_cleaner_array[] = { LEFT_CNY, RIGHT_CNY };
CNY *line_follower = new CNY(line_follower_array, 8);
CNY *area_cleaner = new CNY(area_cleaner_array, 2);

// ------------------ INICIALIZADOR ---------------------------------------------------------------------------

void initialize(void (*category_loop_function)()) {

  confirm_button->WaitPressing();
  if (DEBUG) Serial.println("iniciando");
  buzzer->Beep(3, 1000);
  if (DEBUG) Serial.println("Arranque brrr");

  while (true) category_loop_function();
}

// -------------------- VELOCISTA -----------------------------------------------------------------------------

#define VEL_WHITE_FLOOR 200
#define PID_VEL_MIN 80

/*
#define MIN_FORWARD_VEL 105
float gain = 180;
float kp = 5;
float kd = 0;
*/

int MIN_FORWARD_VEL = 170;
#define MIN_BACKWARD_VEL 80
#define MIN_DOUBLE_FORWARD_VEL 70

#define COMPENSATION_DEADLINE 90
int base_velocity = MIN_FORWARD_VEL;
// doble forward 70
// doble forward 60

float gain = 180;
float kp = 4;
float kd = 0.5;
//#define PID_VEL_MIN 150 pista naba 7seg y pista 2019 7seg
//#define PID_VEL_MIN 220
bool all_white = false;
int in_straight = NO;
int right_vel = 0;
int left_vel = 0;

unsigned long time_stamp;
int values_read[CNY70_CANT];
int max_value_read[CNY70_CANT];
int min_value_read[CNY70_CANT];

void telemetry(){
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
        else if(in == "velup"){ MIN_FORWARD_VEL += 5; BT.println(MIN_FORWARD_VEL); }
        else if(in == "veldw"){ MIN_FORWARD_VEL -= 5; BT.println(MIN_FORWARD_VEL); }
        if(in == "s") break;
        in = "";
        
        }
    } 

  }
}

void Motor_control(int value) {


  if (in_straight) {
    if (millis() > time_stamp + 500) {
      time_stamp = millis();
      base_velocity += 5;
    }
  } else base_velocity = MIN_FORWARD_VEL;


  if (black_side == LEFT) {

    right_vel = base_velocity - value;
    left_vel = base_velocity + value;
  } else {
    right_vel = base_velocity + value;
    left_vel = base_velocity - value;
  }

  right_vel = constrain(right_vel, 0, 255);
  left_vel = constrain(left_vel, 0, 255);
  if (right_vel > 255) right_vel = 255;
  if (left_vel > 255) right_vel = 255;

  if (all_white) {

    if (black_side == RIGHT) {

      leftMotor.Forward(VEL_WHITE_FLOOR);
      rightMotor.Forward(0);
    } else {
      rightMotor.Forward(VEL_WHITE_FLOOR);
      leftMotor.Forward(0);
    }
  } else {

    if (right_vel < COMPENSATION_DEADLINE) rightMotor.Backward(MIN_BACKWARD_VEL + (COMPENSATION_DEADLINE - right_vel));
    else rightMotor.Forward(right_vel);
    if (left_vel < COMPENSATION_DEADLINE) leftMotor.Backward(MIN_BACKWARD_VEL + (COMPENSATION_DEADLINE - left_vel));
    else leftMotor.Forward(left_vel);
  }
}

float ki = 0;
int error = 0;
int last_error = 0;
int actual_value = 0;
int integral = 0;
int PID_calc;
int motor_PID;

float pid_max_average = (kp * 15600) + ki * 0 + (kd * (15600 - -15600));
float pid_min_average = (kp * -15600) + ki * 0 + (kd * (-15600 - 15600));


void line_follower_loop() {

  telemetry();

  actual_value = line_follower->ReadFloor();
  Serial.println(actual_value);

  error = actual_value;
  //integral += error;
  //integral = constrain(integral, -100000, 100000);

  //all_white = actual_value < -26000;

  PID_calc = (kp * error + ki * integral + kd * (error - last_error));

  motor_PID = map(PID_calc, pid_min_average, pid_max_average, -1 * gain, gain);
  Serial.println(motor_PID);
  //delay(400);

  //if(PID_calc < 50 && PID_calc > -50)in_straight = YES;
  //else in_straight = NO;

  last_error = error;

  if (PID_calc < pid_min_average) pid_min_average = PID_calc;
  if (PID_calc > pid_max_average) pid_max_average = PID_calc;
  //Serial.println("PID");
  Motor_control(motor_PID);
}

void line_follower_setup() {

  if (DEBUG) Serial.println("en follower");

  //BT.begin("line_follower_debug");
  BT.begin();
  while (!BT.connected());
  buzzer->Use(1000);

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

  if (DEBUG) Serial.println("CALIBRANDO, select");

  while (count_button->IsPressed() == NO) {
    line_follower->Calibrate();
  }

  if (DEBUG) Serial.println("Calibrado");

  initialize(line_follower_loop);
}

// -------------------- SUMO RC -------------------------------------------------------------------------------
int velocity, turn_velocity;

void radio_controlled_loop() {

  int y_axis_value = PS4.LStickY();
  int x_axis_value = PS4.RStickX();
  int l1 = PS4.L2Value();
  int r2 = PS4.R2Value();

  if (r2 >= 50) {
    velocity = 250;
    turn_velocity = 150;
  }

  if (l1 >= 50) {
    velocity = 100;
    turn_velocity = 50;
  }

  if (r2 < 50 && l1 < 50) {
    velocity = 170;
    turn_velocity = 70;
  }

  if (y_axis_value >= 50) {
    rightMotor.Forward(velocity);
    leftMotor.Forward(velocity);

    if (x_axis_value >= 50) {

      leftMotor.Forward(velocity);
      rightMotor.Forward(turn_velocity);

    }

    else if (x_axis_value <= -50) {

      leftMotor.Forward(turn_velocity);
      rightMotor.Forward(velocity);
    }
  } else if (y_axis_value <= -50)  //Move car Backward
  {
    rightMotor.Backward(velocity);
    leftMotor.Backward(velocity);

    if (x_axis_value >= 50) {
      rightMotor.Backward(turn_velocity);
      leftMotor.Backward(velocity);
    } else if (x_axis_value <= -50) {
      rightMotor.Backward(velocity);
      leftMotor.Backward(turn_velocity);
    }
  } else if (x_axis_value >= 50)  //Move car Right
  {
    rightMotor.Backward(150);
    leftMotor.Forward(250);

  } else if (x_axis_value <= -50)  //Move car Left
  {
    rightMotor.Forward(250);
    leftMotor.Backward(150);
  } else  //Stop the car
  {
    rightMotor.Stop();
    leftMotor.Stop();
  }
}

void radio_controlled_setup() {

  PS4.begin();
  while (!PS4.isConnected())
    ;
  buzzer->Use(1000);
  PS4.attach(radio_controlled_loop);
}

// ------------------ DESPEJAR AREA ---------------------------------------------------------------------------


#define LINE_REBOUND_TIME 600
#define LIMIT_BLIND_ADVANCING_TIME 1000

int seek_velocity = 100;
int max_distance = 20;
int blind_turn_diference = 70;
#define TURN_ADJUSTMENT_DIFERENCE 90
#define MAX_VELOCITY 100
#define BLIND_LIMIT_TIME 3000
#define BACK_VEL 180


// timers
unsigned long seeking_time;
unsigned long last_rebound_time;
unsigned long start_rebound;
unsigned long saw_right_time;

// flags
int saw_right = NO;
int last_value = LEFT;
int rebound_flag = NO;
int first_blind = YES;
int first_loop = YES;

// variables

int binary_area_cleaner_sum;
int previous_value = 0;
int last_cny_value;

int read_area(){

  return (left_sharp->ReadDigital(10, max_distance)
                               + center_sharp->ReadDigital(10, max_distance) * 2
                               + right_sharp->ReadDigital(10, max_distance) * 4);
  
}

void area_cleaner_loop() {

  /*
  if(first_loop){
  leftMotor.Forward(170);
  rightMotor.Forward(170);
  delay(1000);
  } 
  */

  area_cleaner->Calibrate();
  last_cny_value = area_cleaner->ReadTatamiColor();
  
  if (last_cny_value != WHITE) binary_area_cleaner_sum = -1;
  else {
    binary_area_cleaner_sum = read_area();
  }
  /*
  Serial.println(binary_area_cleaner_sum);
  Serial.println(analogRead(BACK_CNY));
  Serial.println();
  delay(300);
  */

  if(previous_value == 4 & binary_area_cleaner_sum == 0){
    if(read_area() == 0){
          last_value = RIGHT;
    }
  } 
  
  if (right_side_sharp->ReadDigital(10, max_distance)) last_value = RIGHT;
  else if  (left_side_sharp->ReadDigital(10, max_distance)) last_value = LEFT;

  previous_value = binary_area_cleaner_sum;

  switch (binary_area_cleaner_sum) {

    case -1:
      
      start_rebound = millis();
      while (millis() < (start_rebound + LINE_REBOUND_TIME)) {
          
        leftMotor.Backward(BACK_VEL);
        rightMotor.Backward(BACK_VEL);
         
        if (analogRead(BACK_CNY) > 2000) {

          if (last_cny_value == LEFT_CNY) {
            leftMotor.Forward(BACK_VEL + TURN_ADJUSTMENT_DIFERENCE);
            rightMotor.Forward(BACK_VEL);
            delay(500);

          } else if (last_cny_value == RIGHT_CNY) {
            leftMotor.Forward(BACK_VEL);
            rightMotor.Forward(BACK_VEL + TURN_ADJUSTMENT_DIFERENCE);
            delay(500);
          }
          break;
        }
        
      }

      rebound_flag = YES;
      last_rebound_time = millis();
      
      break;

    case 0:  //no ve nada



      if (millis() > last_rebound_time + BLIND_LIMIT_TIME && rebound_flag == YES) {
        
        if (area_cleaner->ReadTatamiColor() == WHITE) {
          leftMotor.Forward(seek_velocity);
          rightMotor.Forward(seek_velocity + TURN_ADJUSTMENT_DIFERENCE );
          //first_blind = NO;
        } else rebound_flag = NO;
      } 
      
      else if (last_value == LEFT) {

        if (first_blind == YES) {
          leftMotor.Backward(seek_velocity);
          rightMotor.Forward(seek_velocity);
        } else {
          rightMotor.Forward(seek_velocity + blind_turn_diference);
          leftMotor.Forward(seek_velocity);
        }
      } else {
        if (first_blind == YES) {
          rightMotor.Backward(seek_velocity);
          leftMotor.Forward(seek_velocity);
        } else {
          leftMotor.Forward(seek_velocity + blind_turn_diference);
          rightMotor.Forward(seek_velocity);
        }
      }


      break;

    case 1:
    case 3:
      leftMotor.Forward(seek_velocity);
      rightMotor.Forward(seek_velocity + TURN_ADJUSTMENT_DIFERENCE);
      break;

    case 4:
    case 6:
      leftMotor.Forward(seek_velocity + TURN_ADJUSTMENT_DIFERENCE);
      rightMotor.Forward(seek_velocity);
      break;

    default:
      leftMotor.Forward(MAX_VELOCITY);
      rightMotor.Forward(MAX_VELOCITY);
      break;
  }
}


void area_cleaner_setup() {

  area_cleaner->InitializeReadings();

  if (DEBUG) Serial.println("CALIBRANDO, select");

  while (count_button->IsPressed() == NO) {
    area_cleaner->Calibrate();
  }

  last_rebound_time = millis();
  initialize(area_cleaner_loop);
}
// Funciones para seleccion e inicializacion de modos

#define MODES 4

void mode_selection() {

  int counter = 0;

  while (true) {


    if (count_button->IsPressed() == YES) {

      counter++;
      if (counter > MODES)
        counter = 1;

      buzzer->Beep(counter, 100);
      //Serial.println("Counter %f", counter);
      delay(100);
    }
    if (confirm_button->IsPressed() == YES && counter != 0) break;
  }
  switch (counter) {

    case 1:
      black_side = LEFT;
      line_follower_setup();
      if (DEBUG) Serial.println("seguidor de linea");

      break;

    case 2:
      black_side = RIGHT;
      line_follower_setup();
      if (DEBUG) Serial.println("line follower");

      break;

    case 3:
      area_cleaner_setup();
      if (DEBUG) Serial.println("area cleaner");

      break;

    case 4:
      radio_controlled_setup();
      if (DEBUG) Serial.println("radio_control");

      break;
  }
}

void setup() {
  Serial.begin(115200);
  if (DEBUG) Serial.println("inicio programa");
  mode_selection();
}

void loop() {
}
