
#include <Ps3Controller.h>


#include <BluetoothSerial.h>

#define DEBUG 1
// Motores
#define M2A 22
#define M2B 23
#define M1B 19
#define M1A 21

// Sensores
#define SENSOR_1 13
#define SENSOR_2 27
#define SENSOR_3 26
#define SENSOR_4 25
#define SENSOR_5 34
#define SENSOR_6 35
#define SENSOR_7 32
#define SENSOR_8 33
#define left_CNY 27
#define right_CNY 25

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
#define RIGHT 0

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
    Serial.println("beeep");
    delay(duration);
    digitalWrite(pin, LOW);
    delay(duration);
  }
  digitalWrite(pin, HIGH);
  Serial.println("beeep");
  delay(duration);
  digitalWrite(pin, LOW);
}

// Definicion de clase para motores

class Motor {

private:
  int pinA;
  int pinB;
  int chA;
  int chB;

public:
  Motor(int pinA_in, int pinB_in, int chA_in, int chB_in);
  void Forward(int vel);
  void Backward(int vel);
  void Stop();
};

// Constructor clase motores
Motor::Motor(int pinA_in, int pinB_in, int chA_in, int chB_in) {
  pinA = pinA_in;
  pinB = pinB_in;
  chA = chA_in;
  chB = chB_in;

  ledcSetup(chA, 1000, 8);
  ledcSetup(chB, 1000, 8);
  ledcAttachPin(pinA, chA);
  ledcAttachPin(pinB, chB);
}

// Metodos motores
void Motor::Forward(int vel) {
  ledcWrite(chA, vel);
  ledcWrite(chB, 0);
}
void Motor::Backward(int vel) {
  ledcWrite(chA, 0);
  ledcWrite(chB, vel);
}
void Motor::Stop() {
  ledcWrite(chA, 0);
  ledcWrite(chB, 0);
}

// Definicion de clase para sensores Sharp
class Sharp {

private:
  int pin;

public:
  Sharp(int pin_in);
  int ReadDigital(int samples);
};

Sharp::Sharp(int pin_in) {
  pin = pin_in;
}

int Sharp::ReadDigital(int samples) {
  int sum = 0;
  int value = 0;
  for (int i = 0; i <= samples; i++) {
    value = analogRead(pin);
    sum += map(value, 200, 3000, 0, 100);
  }
  Serial.println(sum / samples);
  if (sum / samples > 20)
    return YES;
  else
    return NO;
}


class Button {

private:
  int pin;

public:
  Button(int pin_in);
  int is_pressed();
  void wait_pressing();
};

// Constructor clase Button
Button::Button(int pin_in) {
  pin = pin_in;
  pinMode(pin, INPUT);
}

int Button::is_pressed() {
  return digitalRead(pin);
}

void Button::wait_pressing() {
  while (digitalRead(pin) == NO)
    continue;
}

// Definicion de clase PID
class PID {

private:
  int map_start, map_end;
  int color_deadline = 1600;
  int values_read[CNY70_CANT];
  int max_value_read[CNY70_CANT];
  int min_value_read[CNY70_CANT];
  int Cny70_pins[8] = { SENSOR_1, SENSOR_2, SENSOR_3, SENSOR_4, SENSOR_5, SENSOR_6, SENSOR_7, SENSOR_8 };

public:
  PID();
  void Initialize_readings();
  void Calibrate();
  int Read_floor();
  int read_tatami_color();
};

// Constructor clase motores
PID::PID() {
}

int PID::read_tatami_color() {


  if (analogRead(left_CNY) > color_deadline || analogRead(right_CNY) > color_deadline) {
    return BLACK;
  } else {
    return WHITE;
  }
}

void PID::Initialize_readings() {

  for (int i = 0; i < CNY70_CANT; i++) {
    max_value_read[i] = 3500;
    min_value_read[i] = 3500;
  }
}

void PID::Calibrate() {

  int i;

  for (i = 0; i < CNY70_CANT; i++) {
    int value = analogRead(Cny70_pins[i]);
    min_value_read[i] = constrain(min_value_read[i], -100000, value);
    max_value_read[i] = constrain(max_value_read[i], value, 100000);
  }
  if (DEBUG) {
    //Serial.println("CNY_%d: %d, %d ", i, min_value_read[i], max_value_read[i]);
  }
}

int PID::Read_floor() {

  for (int i = 0; i < 4; i++) {
    values_read[i] = analogRead(Cny70_pins[i]);
    values_read[i] = map(values_read[i], min_value_read[i], max_value_read[i], 1000, 0);
    values_read[i] = constrain(values_read[i], 0, 1000);
    //Serial.println(values_read[i]);
  }

  Serial.println();
  for (int i = 4; i < 8; i++) {
    values_read[i] = analogRead(Cny70_pins[i]);
    values_read[i] = map(values_read[i], min_value_read[i], max_value_read[i], 0, 1000);
    values_read[i] = constrain(values_read[i], 0, 1000);
    //Serial.println(values_read[i]);
  }



  return (-16 * values_read[0]) + (-8 * values_read[1]) + (-4 * values_read[2]) + (-2 * values_read[3]) + (2 * values_read[4]) + (4 * values_read[5]) + (8 * values_read[6]) + (16 * values_read[7]);

  //return (-16 * values_read[7]) + (-8 * values_read[6]) + (-4 * values_read[5]) + (-2 * values_read[4]) + (2 * values_read[3]) + (4 * values_read[2]) + (8 * values_read[1]) + (16 * values_read[0]);
}

Motor *rightMotor = new Motor(M1A, M1B, 14, 13);
Motor *leftMotor = new Motor(M2A, M2B, 12, 11);
Button *confirm_button = new Button(PUSH_2);
Button *count_button = new Button(PUSH_1);
Sharp *left_sharp = new Sharp(SENSOR_7);
Sharp *right_sharp = new Sharp(SENSOR_8);
Sharp *center_sharp = new Sharp(SENSOR_6);
Sharp *left_side_sharp = new Sharp(SENSOR_1);
Sharp *right_side_sharp = new Sharp(SENSOR_5);
Buzzer *buzzer = new Buzzer(BUZZER);
PID *pid = new PID();

// ------------------ INICIALIZADOR ---------------------------------------------------------------------------

void initialize(void (*category_loop_function)()) {

  confirm_button->wait_pressing();
  Serial.println("iniciando");
  buzzer->Beep(3, 1000);
  Serial.println("Arranque brrr");

  while (true) {
    category_loop_function();
  }
}

// -------------------- VELOCISTA -----------------------------------------------------------------------------

#define VEL_WHITE_FLOOR 100
#define VEL_MIN 80
//#define PID_VEL_MIN 110
//#define PID_VEL_MIN 150 pista naba 7seg y pista 2019 7seg
#define PID_VEL_MIN 220
bool all_white = false;

int right_vel = 0;
int left_vel = 0;

int values_read[CNY70_CANT];
int max_value_read[CNY70_CANT];
int min_value_read[CNY70_CANT];

void Motor_control(int value) {

  if (black_side == LEFT) {

    right_vel = PID_VEL_MIN - value;
    left_vel = PID_VEL_MIN + value;
  } else {
    right_vel = PID_VEL_MIN + value;
    left_vel = PID_VEL_MIN - value;
  }

  right_vel = constrain(right_vel, 0, 255);
  left_vel = constrain(left_vel, 0, 255);

  Serial.println(left_vel);
  Serial.println(right_vel);

  if (all_white) {

    if (black_side == RIGHT) {

      leftMotor->Forward(0);
      rightMotor->Forward(VEL_WHITE_FLOOR);
    } else {
      rightMotor->Forward(0);
      leftMotor->Forward(VEL_WHITE_FLOOR);
    }
  } else {

    if (right_vel < VEL_MIN) rightMotor->Backward(VEL_MIN + (VEL_MIN - right_vel));
    else rightMotor->Forward(right_vel);
    if (left_vel < VEL_MIN) leftMotor->Backward(VEL_MIN + (VEL_MIN - left_vel));
    else leftMotor->Forward(left_vel);
  }
}

float kp = 10;
float kd = 0;
float ki = 0;
int error = 0;
int last_error = 0;
int actual_value = 0;
int integral = 0;
int PID_calc;
float gain = 120;
float pid_max_average = (kp * 15600) + ki * 0 + (kd * (15600 - -15600));
float pid_min_average = (kp * -15600) + ki * 0 + (kd * (-15600 - 15600));


void line_follower_loop() {

  actual_value = pid->Read_floor();
  //Serial.println(actual_value);

  error = actual_value;
  integral += error;
  integral = constrain(integral, -100000, 100000);

  all_white = actual_value < -26000;

  PID_calc = (kp * error + ki * integral + kd * (error - last_error));
  PID_calc = map(PID_calc, pid_min_average, pid_max_average, -1 * gain, gain);

  last_error = error;

  if (actual_value < pid_min_average) pid_min_average = actual_value;
  if (actual_value > pid_max_average) pid_max_average = actual_value;
  //Serial.println("PID");
  Motor_control(PID_calc);
}

void line_follower_setup() {

  Serial.println("en follower");

  //BT.begin("line_follower_debug");

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

  pid->Initialize_readings();

  Serial.println("CALIBRANDO, select");

  while (count_button->is_pressed() == NO) {
    pid->Calibrate();
  }

  Serial.println("Calibrado");

  initialize(line_follower_loop);
}

// -------------------- SUMO RC -------------------------------------------------------------------------------
int velocidad, giro;

void radio_controlled_loop() {

  int yAxisValue = (Ps3.data.analog.stick.ly);
  int xAxisValue = (Ps3.data.analog.stick.rx);
  int r2 = (Ps3.data.analog.button.r2);
  int l1 = (Ps3.data.analog.button.l1);

  if (r2 >= 50) { 
    velocidad = 250;
    giro = 150;
  }

  if (l1 >= 50) {
    velocidad = 100;
    giro = 50;
  }

  if (r2 < 50 && l1 < 50) {
    velocidad = 170;
    giro = 70;
  }

  if (yAxisValue <= -50 ) {
    rightMotor->Forward(velocidad);
    leftMotor->Forward(velocidad);

    if (xAxisValue >= 50) {

      leftMotor->Forward(velocidad);
      rightMotor->Forward(giro);

    }

    else if (xAxisValue <= -50) {

      leftMotor->Forward(giro);
      rightMotor->Forward(velocidad);
    }
  } else if (yAxisValue >= 50)  //Move car Backward
  {
    rightMotor->Backward(velocidad);
    leftMotor->Backward(velocidad);

    if (xAxisValue >= 50) {
      rightMotor->Backward(giro);
      leftMotor->Backward(velocidad);
    } else if (xAxisValue <= -50) {
      rightMotor->Backward(velocidad);
      leftMotor->Backward(giro);
    }
  } else if (xAxisValue >= 50)  //Move car Right
  {
    rightMotor->Backward(150);
    leftMotor->Forward(250);

  } else if (xAxisValue <= -50)  //Move car Left
  {
    rightMotor->Forward(250);
    leftMotor->Backward(150);
  } else  //Stop the car
  {
    rightMotor->Stop();
    leftMotor->Stop();
  }
}

void radio_controlled_setup() {

  Ps3.begin("0c:dc:7e:61:53:f2");
  while (!Ps3.isConnected())
    ;
  buzzer->Use(1000);
  Ps3.attach(radio_controlled_loop);
}

// ------------------ DESPEJAR AREA ---------------------------------------------------------------------------


#define LINE_REBOUND_TIME 150
#define TURN_VELOCITY 80
#define TURN_ADJUSTMENT_DIFERENCE 70
#define MAX_VELOCITY 220
#define LIMIT_BLIND_TIME 2500
#define LIMIT_BLIND_ADVANCING_TIME 1000
#define SEEK_VELOCITY 70

int binary_area_cleaner_sum;
int start_seeking_flag = YES;
unsigned long seeking_time;
int saw_right = NO;
int last_value = LEFT;
unsigned long last_rebound_time;
void area_cleaner_loop() {

  if (pid->read_tatami_color() == BLACK) {
    binary_area_cleaner_sum = -1;
  } else {
    binary_area_cleaner_sum = (left_sharp->ReadDigital(10)
                               + center_sharp->ReadDigital(10) * 2
                               + right_sharp->ReadDigital(10) * 4);
  }

  if (right_side_sharp->ReadDigital(10) == YES) saw_right = YES;
  switch (binary_area_cleaner_sum) {

    case -1:

      if (saw_right) {
        last_value = RIGHT;
        saw_right = NO;
      } else {
        last_value = LEFT;
      }
      leftMotor->Backward(100);
      rightMotor->Backward(100);
      delay(700);
      leftMotor->Backward(100);
      rightMotor->Forward(100);
      delay(100);
      last_rebound_time = millis();
      break;

    case 0:  //no ve nada
      /*
            if (start_seeking_flag == YES) {
              seeking_time = millis();
              start_seeking_flag = NO;

            }

            if (millis() > seeking_time + LIMIT_BLIND_TIME) {

              leftMotor->Forward(100);
              rightMotor->Forward(100);

              if (millis() > seeking_time + LIMIT_BLIND_TIME + LIMIT_BLIND_ADVANCING_TIME) start_seeking_flag == YES;
              else{
      


      */
      if (last_value == LEFT) {
        leftMotor->Backward(SEEK_VELOCITY);
        rightMotor->Forward(SEEK_VELOCITY);
      } else {
        leftMotor->Forward(SEEK_VELOCITY);
        rightMotor->Backward(SEEK_VELOCITY);
      }

      if (millis() > last_rebound_time + 5000) {
        while (pid->read_tatami_color() == WHITE) {
          leftMotor->Forward(130);
          rightMotor->Forward(200);
        }
      }
      //}
      // }

      break;

    case 1:
    case 3:
      last_value = LEFT;
      leftMotor->Forward(TURN_VELOCITY);
      rightMotor->Forward(TURN_VELOCITY + TURN_ADJUSTMENT_DIFERENCE);
      break;

    case 4:
    case 6:
      last_value = RIGHT;
      leftMotor->Forward(TURN_VELOCITY + TURN_ADJUSTMENT_DIFERENCE);
      rightMotor->Forward(TURN_VELOCITY);
      break;

    case 8:
      leftMotor->Backward(TURN_VELOCITY + TURN_ADJUSTMENT_DIFERENCE);
      rightMotor->Backward(TURN_VELOCITY);
      break;

    default:
      leftMotor->Forward(MAX_VELOCITY);
      rightMotor->Forward(MAX_VELOCITY);
      break;
  }
}


void area_cleaner_setup() {
  last_rebound_time = millis();
  initialize(area_cleaner_loop);
}
// Funciones para seleccion e inicializacion de modos

void mode_selection() {

  int counter = 0;

  while (confirm_button->is_pressed() == NO) {

    if (count_button->is_pressed() == YES) {

      counter++;
      if (counter > 4)
        counter = 1;

      buzzer->Beep(counter, 100);
      //Serial.println("Counter %f", counter);
      delay(100);
    }
  }
  switch (counter) {

    case 1:
      black_side = LEFT;
      line_follower_setup();
      Serial.println("seguidor de linea");

      break;

    case 2:
      black_side = RIGHT;
      line_follower_setup();
      Serial.println("line follower");

      break;

    case 3:
      area_cleaner_setup();
      Serial.println("area cleaner");

      break;

    case 4:
      radio_controlled_setup();
      Serial.println("radio_control");

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
