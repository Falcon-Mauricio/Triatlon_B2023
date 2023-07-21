#define DEBUG 1

// Motores

#define M1A 21
#define M1B 19
#define M2A 22
#define M2B 23

class Motor {

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

// Constructor clase motores
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

Motor *rightMotor = new Motor(M1A, M1B, 14, 13);
Motor *leftMotor = new Motor(M2A, M2B, 12, 11);


void setup() {
  Serial.begin(115200);
  if (DEBUG) Serial.println("inicio programa");
  mode_selection();
}

void loop() {

  // Los motores son objetos "leftMotor" o "rightMotor"
  // La direccion de giro es un metodo de la clase "Motor"
  // La velocidad es el parametro de el metodo empleado
  // En el caso del metodo Stop() no se requiere parametro

  leftMotor -> Forward(200)
}
