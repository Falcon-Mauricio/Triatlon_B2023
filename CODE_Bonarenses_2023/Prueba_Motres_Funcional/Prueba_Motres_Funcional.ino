// pin left
#define M1A 23
#define M1B 22
// pin Right
#define M2A 21
#define M2B 19
//channels
#define DER_VELOCIDAD 12
#define IZQ_VELOCIDAD 14
#define DER_DIRECCION 13
#define IZQ_DIRECCION 15
#define left 1
#define right 0
#define backward 0
#define Forward 255
const int PWMFreq = 1000; /* 1 KHz */
const int PWMResolution = 8;
const int PWMSpeedChannel = 4;

void setUpPinModes()
{
  ledcSetup(IZQ_DIRECCION, 1000, 8);
  ledcSetup(IZQ_VELOCIDAD, 1000, 8);
  ledcSetup(DER_DIRECCION, 1000, 8);
  ledcSetup(DER_VELOCIDAD, 1000, 8);

  ledcAttachPin(M1A, IZQ_DIRECCION);
  ledcAttachPin(M1B, IZQ_VELOCIDAD);
  ledcAttachPin(M2A, DER_DIRECCION);
  ledcAttachPin(M2B, DER_VELOCIDAD);
}

void motor(int motor, int direccion, int velocidad) { 
  if (motor == left) {
    if (direccion == 0) {
      ledcWrite(IZQ_DIRECCION, velocidad);
      ledcWrite(IZQ_VELOCIDAD, 0);
      digitalWrite(2,HIGH);
    }
    else {
      ledcWrite(IZQ_DIRECCION, 0);
      ledcWrite(IZQ_VELOCIDAD, velocidad);
    }
    // return ;
  }
  else if (motor == right) {
    if (direccion == 0) {
      ledcWrite(DER_DIRECCION, 0);
      ledcWrite(DER_VELOCIDAD, velocidad);
      digitalWrite(2,LOW);

    }
    else {
      ledcWrite(DER_DIRECCION, velocidad);
      ledcWrite(DER_VELOCIDAD, 0);
    }
    // return ;
  }
}

void setup()
{
  setUpPinModes();
  Serial.begin(115200);
  motor(left, Forward, 0);
  motor(right, Forward, 0);
}

void loop()
{
    motor(left, Forward, 150);
    motor(right, backward, 0);
    delay(3000);
    motor(right, Forward, 0);
    motor(left, backward, 200);
    delay(3000);
    motor(left, Forward, 0);
    motor(right, backward, 200);
    delay(3000);
    motor(right, Forward, 150);
    motor(left, backward, 0);
    delay(3000);
}