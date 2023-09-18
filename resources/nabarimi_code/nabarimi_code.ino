#include <Arduino.h>
#include <BluetoothSerial.h>


#define DEBUG 1  //Poner en 'true' o '1' para debugear por bluetooh y 'false' o 0 para que no mande

// distancia maxima de los sensores
#define DISTANCIA_MAXIMA 60
#define FALLO 100000
#define VALOR_WHITE 3500
//#define VALOR_BLACK
bool giroTatami = true;


// Tiempo de retroceso cuando toca la linea blanca
#define TIEMPO_RETROCESO 50
#define TIEMPO_GIRO 80

bool executedMilis = false;
bool isForward = false;


#define left 1
#define right 0

#define backward 0
#define Forward 255

const int SearchSpeed = 60;
int NormalSpeed = 175;
unsigned long buscando = 0;
long DELAY_TIME = 3000;

// channels
#define DER_VELOCIDAD 12
#define IZQ_VELOCIDAD 14

#define M1A 19
#define M1B 18
#define M2A 22
#define M2B 21

#define led 2
#define BUZZER 17
#define START 16

#define MAX_MOTOR_SPEED 200

#define VARIABLE_VELOCIDAD 150

// Sensores frente
// Sensores frente
#define SENSOR_ADELANTE_MEDIO 27
#define SENSOR_ADELANTE_IZQUIERDO 35
#define SENSOR_ADELANTE_DERECHO 32
#define SENSOR_DER 33
#define SENSOR_IZQ 34

// Sensores Piso
#define SENSOR_PISO_DER 13
#define SENSOR_PISO_IZQ 25
#define SENSOR_PISO_BACK 26

// TASK HANDLE

BluetoothSerial BT;

TaskHandle_t ReadSensors;
unsigned long startOfMotor;

unsigned long searchTime;

bool hasTurn = false;

int seconds = 1;
bool isRetroceso = false;

int rotate = 100;
unsigned long currentM;
int state_last = 100;

const int PWMFreq = 1000; /* 1 KHz */
const int PWMResolution = 8;
const int PWMSpeedChannel = 4;

const int turnSpeed = 255;
const int speed = 100;
bool isEnemyVal = false;
unsigned long actualTime;

bool needStop = false;
const long referenceMv = 3300;
enum State {
  BUSCANDO,
  ADELANTE,
  DERECHA,
  ADELANTE_DERECHA,
  IZQUIERDA,
  ADELANTE_IZQUIERDA,
  DERECHA_IZQUIERDA,
  ADELANTE_DERECHA_IZQUIERDA,
  TATAMI

};

enum lastMidSensor {
  MID_ADELANTE_MEDIO,
  MID_ADELANTE_DERECHA,
  MID_ADELANTE_IZQUIERDA
};
int state = BUSCANDO;
int state_last_mid = 9999;

// interpolación de la distancia a intervalos de 180mV
const int TABLE_ENTRIES = 12;
const int INTERVAL = 180;
static int distance[TABLE_ENTRIES] = { 150, 140, 130, 100, 60, 50, 40, 35, 30, 25, 20, 15 };
int cm_medio;
int cm_derecha_adelante;
int cm_izquierda_adelante;
int cm_derecha;
int cm_izquierda;
bool isTatamiVal;
bool isTatamiBack;
bool sensCostados = true;

bool rightOn = false;
bool leftOn = false;

bool crazyMode = false;
bool rebootMode = false;
int crazyEnable = 2;
int last_crazy_enable = 2;


void setUpPinModes() {
  pinMode(SENSOR_PISO_DER, INPUT);
  pinMode(SENSOR_PISO_IZQ, INPUT);
  pinMode(SENSOR_PISO_BACK, INPUT);

  ledcSetup(IZQ_VELOCIDAD, 1000, 8);

  ledcSetup(DER_VELOCIDAD, 1000, 8);

  pinMode(M1A, OUTPUT);
  ledcAttachPin(M1B, IZQ_VELOCIDAD);
  pinMode(M2A, OUTPUT);
  ledcAttachPin(M2B, DER_VELOCIDAD);

  pinMode(led, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);
  digitalWrite(led, LOW);
}




int getCentimeter(int mV) {
  if (mV > INTERVAL * TABLE_ENTRIES - 1)
    return distance[TABLE_ENTRIES - 1];
  else {
    int index = mV / INTERVAL;
    float frac = (mV % 180) / (float)INTERVAL;
    return distance[index] - ((distance[index] - distance[index + 1]) * frac);
  }
}

int getDistance(int sensorPin) {

  int val = analogRead(sensorPin);
  int mV = (val * referenceMv) / 4096;  // 4096
  int cm = getCentimeter(mV);
  return cm;
}

bool isBlack(int sensorPiso) {

  int valorCNY = analogRead(sensorPiso);
  Serial.println(valorCNY);
  bool black = valorCNY > 1300;
  return black;
}

bool isTatami() {

  bool der= isBlack(SENSOR_PISO_DER);
  bool izq= isBlack(SENSOR_PISO_IZQ);

  bool tatami =  der || izq;  //SENSOR_DER


  return tatami;
}




long timeM = 0;


void isEnemy() {
  if (state == BUSCANDO || state >= 8) {
    isEnemyVal = false;
    return;
  }

  isEnemyVal = true;
}


void updateValues() {  //actualizo todos los valores
  cm_medio = getDistance(SENSOR_ADELANTE_MEDIO);
  cm_derecha_adelante = getDistance(SENSOR_ADELANTE_DERECHO);
  cm_izquierda_adelante = getDistance(SENSOR_ADELANTE_IZQUIERDO);


  cm_derecha = getDistance(SENSOR_DER);
  cm_izquierda = getDistance(SENSOR_IZQ);

  isTatamiVal = isTatami();

}

bool frontEnemy() {
  if (cm_medio < DISTANCIA_MAXIMA || cm_izquierda_adelante < DISTANCIA_MAXIMA || cm_derecha_adelante < DISTANCIA_MAXIMA || cm_medio > FALLO || cm_izquierda_adelante > FALLO || cm_derecha_adelante > FALLO) {
    return true;
  }
  return false;
}


unsigned long tie = 0;
int selectState() {  //sumo los valores de todos los lados || state != DERECHA_IZQUIERDA
  updateValues();

  if (cm_medio < DISTANCIA_MAXIMA || cm_medio > FALLO) {
    state_last_mid = MID_ADELANTE_MEDIO;
  } else if (cm_derecha_adelante < DISTANCIA_MAXIMA || cm_derecha_adelante > FALLO) {
    state_last_mid = MID_ADELANTE_DERECHA;
    //if (DEBUG) BT.println("MID_ADELANTE_DERECHA");
  } else if (cm_izquierda_adelante < DISTANCIA_MAXIMA || cm_izquierda_adelante > FALLO) {
    state_last_mid = MID_ADELANTE_IZQUIERDA;
    // if (DEBUG) BT.println("MID_ADELANTE_IZQUIERDA");
  }


  int suma = 0;

  //Adelante = 1
  if (cm_medio < DISTANCIA_MAXIMA || cm_medio > FALLO || cm_izquierda_adelante < DISTANCIA_MAXIMA || cm_izquierda_adelante > FALLO || cm_derecha_adelante < DISTANCIA_MAXIMA) {
    suma += ADELANTE;

    isForward = true;
    rightOn = true;
    leftOn = true;


  } else {
    isForward = false;
    hasTurn = false;
  }


  //RESOLVER FALLO

  if (!hasTurn) {
    //Derecha = 2
    if (rightOn) {
      if (cm_derecha < DISTANCIA_MAXIMA || cm_derecha > FALLO) {
        suma += DERECHA;
        giroTatami = true;
      }
    }

    if (leftOn) {
      //Izquierda = 4
      if (cm_izquierda < DISTANCIA_MAXIMA || cm_izquierda > FALLO) {
        suma += IZQUIERDA;

        giroTatami = false;
      }
    }
  }

  //Tatami = 8
  if (isTatamiVal) suma += TATAMI;

  isEnemy();

  return suma;
}

bool mot = true;
void controlBluetuch() {
  if (BT.available()) {
    //     int velocidad_giro = 100;    // 4 celdas 150
    // int vel_giro_crazy = 50;
    // int vel_retroceso = 200;
    // int vel_retroceso_crazy = 100;
    // NormalSpeed
    String in = BT.readStringUntil('\n');
    in.trim();
    if (in == "m") {
      digitalWrite(BUZZER, 1);
      mot = !mot;
    } else if (in == "un") {
      digitalWrite(BUZZER, 1);
      NormalSpeed += 5;
      BT.print(NormalSpeed);
    } else if (in == "dn") {
      digitalWrite(BUZZER, 1);
      NormalSpeed -= 5;
      BT.print(NormalSpeed);
    } else if (in == "D") {
      digitalWrite(BUZZER, 1);
      // DEBUG = !DEBUG;
    }
    digitalWrite(BUZZER, 0);
  }
}


void selectCrazyM() {
  // actualTime =millis();
  // BT.print("MILLIS = ");
  // BT.println(actualTime);
  // BT.print("buscando = ");
  // BT.println(buscando);
  // if(crazyEnable == BUSCANDO){
  //     if(last_crazy_enable != crazyEnable){
  //       buscando = millis();
  //     }
  //     else if(last_crazy_enable == crazyEnable ){
  //       BT.print("crazyMode = ");
  //       crazyMode = actualTime >= (buscando + TIME_CRAZY) && buscando != 0;
  //       BT.println(crazyMode);
  //     }
  // }else buscando = 0;
  //     last_crazy_enable = crazyEnable;
}

void ReadSensorsFun(void *parameter) {

  for (;;) {

    controlBluetuch();
    state = selectState();  // le doy el valor de las sumas a 'state'
    vTaskDelay(10);
  }
}
//TOOD APAGAR EL SENSOR CONTRARIO AL QUE GIRO if gira derecha cuando sale de tatami desactivar por 2 seg


void motor(int motor, int direccion, int velocidad) {

  if (motor == left) {
    if (direccion == 0) {
      digitalWrite(M1A, 1);
      ledcWrite(IZQ_VELOCIDAD, velocidad);
    } else {
      digitalWrite(M1A, 0);
      ledcWrite(IZQ_VELOCIDAD, velocidad);
    }
    return;
  }

  else if (motor == right) {
    if (direccion == 0) {
      digitalWrite(M2A, 0);
      ledcWrite(DER_VELOCIDAD, velocidad);
    } else {
      digitalWrite(M2A, 1);
      ledcWrite(DER_VELOCIDAD, velocidad);
    }
    return;
  }
}


void stopMotors() {
  motor(right, backward, 0);
  motor(left, backward, 0);
}

void searchEnemy(int pwm, int side) {
  if (DEBUG) BT.println("SEARCHING FUNCT");
  if (side == IZQUIERDA) {
    motor(right, Forward, pwm);
    motor(left, backward, pwm);
  } else {
    motor(left, Forward, pwm);
    motor(right, backward, pwm);
  }
  needStop = true;
}


void PYD(int pwm) {
  // while (isTatamiVal == false && isForward  ) {
    switch (state_last_mid) {


      case MID_ADELANTE_MEDIO:
        if (DEBUG) BT.println("MID");
        if (needStop) {
          stopMotors();
        }
        needStop = false;
        motor(left, Forward, pwm);
        motor(right, Forward, pwm);
        hasTurn = true;
        break;
      case MID_ADELANTE_DERECHA:
        if (DEBUG) BT.println("MID_ADELANTE_DERECHA");
        motor(left, Forward, NormalSpeed - 10);
        motor(right, Forward, NormalSpeed - 50);
        break;
      case MID_ADELANTE_IZQUIERDA:
        if (DEBUG) BT.println("MID_ADELANTE_IZQUIERDA");

        motor(right, Forward, NormalSpeed - 10);
        motor(left, Forward, NormalSpeed - 50);
        break;
    }
  // }
}


void goForward(int pwm) {
  PYD(pwm);
}
void goLeft(int speed) {
  motor(right, Forward, speed);
  motor(left, backward, speed);
}




void goRight(int speed) {
  motor(left, Forward, speed);
  motor(right, backward, speed);
}





bool hasTurnBefore() {
  if (state_last != DERECHA_IZQUIERDA || state_last != IZQUIERDA || state_last != DERECHA) {
    return false;
  };
  return true;
}


void crazy() {
  while (isTatamiVal == false && isEnemyVal == false) {
    motor(left, Forward, 140);  //cambiar a 140
    motor(right, Forward, 140);
  }
  needStop = true;
}

void StateMachine() {

  //if(state != state_last){// Eliminar este 'if' y al 'state_last = state' en caso de implementar codigo en el 'switch(state)'
  BT.println(hasTurn);

  if (state != BUSCANDO) digitalWrite(BUZZER, 0);

  switch (state)  //Veo en que estado me encuentro
  {
    case BUSCANDO:
      {
        if (DEBUG) BT.println("BUSCANDO");
        currentM = millis();
        if (crazyMode) {
          if (DEBUG) BT.println("CRAZY by default");
          crazy();
        } else {
          if (currentM - timeM > DELAY_TIME && isEnemyVal == false) {
            timeM = currentM;
            if (DEBUG) BT.println("CRAZY");
            crazyMode = true;
            crazy();
          } else {
            if (rotate == DERECHA) {
              searchEnemy(SearchSpeed, DERECHA);
            } else if (rotate == IZQUIERDA) {
              searchEnemy(SearchSpeed, IZQUIERDA);
            } else {
              searchEnemy(SearchSpeed, IZQUIERDA);
            }
          }
        }
        break;
      }


    case ADELANTE:
      {
        if (DEBUG) BT.println("ADELANTE");
        goForward(NormalSpeed);
        break;
      }  //1

    case DERECHA:
      {
        if (DEBUG) BT.println("DERECHA");
        goRight(TIEMPO_GIRO);

        hasTurn = true;
        rotate = DERECHA;
        break;
      }  //2

    case ADELANTE_DERECHA:
      {
        if (DEBUG) BT.println("ADELANTE_DERECHA");
        goForward(NormalSpeed);
        giroTatami = true;
        break;
      }  //3

    case IZQUIERDA:
      {
        if (DEBUG) BT.println("IZQUIERDA");
        goLeft(TIEMPO_GIRO);  // poner search en un futuro
        hasTurn = true;
        rotate = IZQUIERDA;

        break;
      }  //4

    case ADELANTE_IZQUIERDA:
      {
        if (DEBUG) BT.println("ADELANTE_IZQUIERDA");
        // state_last = ADELANTE_IZQUIERDA;
        goForward(NormalSpeed);
        giroTatami = false;

        break;
      }  //5

    case DERECHA_IZQUIERDA:
      {
        if (DEBUG) BT.println("DERECHA_IZQUIERDA");

        goLeft(TIEMPO_GIRO);

        hasTurn = true;


        break;
      }  //6

    case ADELANTE_DERECHA_IZQUIERDA:
      {
        if (DEBUG) BT.println("ADELANTE_DERECHA_IZQUIERDA");
        if (cm_derecha > cm_izquierda) {
          giroTatami = false;
        } else {
          giroTatami = true;
        }
        goForward(NormalSpeed);
        break;
      }  //7

    default:  //en caso de que 'state' sea mayor a 7 se encontrara en el estado 'TATAMI'
      if (crazyMode) {
        motor(left, backward, 150);
        motor(right, backward, 150);
        delay(250);
        if (DEBUG) BT.println("TATAMI YES crazyMode");
        motor(left, backward, 100 + 10);
        motor(right, Forward, 100 + 10);
        delay(200);  //peligroso
        stopMotors();
        crazyMode = false;
      } else {

    
          executedMilis = false;
          hasTurn = false;
         while(isTatamiVal){
          motor(left, backward, 205);
          motor(right, backward, 205);
         }
          if (giroTatami) {
          searchEnemy(80,DERECHA);
            leftOn = false;
          } else {
                     searchEnemy(80,IZQUIERDA);
            rightOn = false;
          }
          delay(200);


      }
        rotate = 100;
      break;
  }
  state_last = state;
}

void inicio() {
  bool presion = false;
  while (true) {
    if (digitalRead(16) == 1) {
      delay(200);
      presion = true;
    } else {
      if (presion) { break; }
    }
  }
  digitalWrite(LED_BUILTIN, 1);
  delay(4000);
  digitalWrite(BUZZER, 1);

  delay(1000);
  digitalWrite(BUZZER, 0);
}
void setup() {
  if (DEBUG) BT.begin("ESPAAA");
  Serial.begin(9600);
  buscando = millis();
  setUpPinModes();



  xTaskCreatePinnedToCore(
    ReadSensorsFun, /* Function to implement the task */
    "Task1",        /* Name of the task */
    4096,           /* Stack size in words */
    NULL,           /* Task input parameter */
    0,              /* Priority of the task */
    &ReadSensors,   /* Task handle. */
    0);             /* Core where the task should run */
  inicio();
}

//bool dect = false;

void loop() {
  StateMachine();
  //unsigned long tiemp = millis();

  vTaskDelay(10);
}