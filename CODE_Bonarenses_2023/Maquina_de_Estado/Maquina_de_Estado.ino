#define COUNTBUTTONPIN 16
#define CONFIRMBUTTONPIN 17
#define BUZZERPIN 18

int counter;
bool statecountboton;
bool stateconfirmboton;

void first_serial_message(){
  Serial.println("Contador del boton de 1 a 4:");
}
void buzzer_on(){
  digitalWrite(BUZZERPIN, HIGH);
  delay(100);
}

void buzzer_off(){
  digitalWrite(BUZZERPIN, LOW);
}

bool read_current_count_button_state(int pin_count_button){
  statecountboton = digitalRead(COUNTBUTTONPIN);
  return statecountboton;
}

bool read_current_confirm_button_state(int pin_confirm_button){
  stateconfirmboton = digitalRead(CONFIRMBUTTONPIN);
  return stateconfirmboton;
}

void print_counter(){
  
  if(read_current_count_button_state(COUNTBUTTONPIN) == 0){
    //Aca reemplazo el while que habia por un if, por alguna razon el While era el problema que te daba los saltos de numero
    if(!read_current_count_button_state(COUNTBUTTONPIN)){ // !read_current_state_button(COUNTBUTTONPIN) significa que el while a tomar como "valor verdadero" el valor 0(cuando el boton este presionado).
      counter++;  //Sumo primero y hago la logica de counter counter>4 antes para que el buzzer no se pase de 4 pitidos
    if (counter > 4){
      counter = 1;
    }
    for(int i = 1; i<=counter;i++){  //Este es un for que compara i con counter para que el buzzer suene la misma cantidad de veces que se toco el boton
      buzzer_on();
      buzzer_off();
      delay(100);
    }
    Serial.println(counter);
    delay(200); //lo cambie a 200 estaba en 100
    }
  }
}

void while_loop(){
  
  while((read_current_confirm_button_state(CONFIRMBUTTONPIN)) == 1){
    print_counter();
  }
  Serial.print("Estoy fuera del while");
  Serial.println();
}

void maquina_de_estado(){
  
  switch(counter){
    
    case 1: while(true){
      Serial.print("Estoy en Velocista derecha");
      Serial.println();
      delay(3000);
    }
    break;
    
    case 2: while(true){
      Serial.print("Estoy en Velocista izquierda");
      Serial.println();
      delay(3000);
    }
    break;
    
    case 3: while(true){
      Serial.print("Estoy en Despejar area");
      Serial.println();
      delay(3000);
    }
    break;
    
    case 4: while(true){
      Serial.print("Estoy en Sumo rc");
      Serial.println();
      delay(3000);
    }
    break;
  }
}

void setup() {
  pinMode(CONFIRMBUTTONPIN, INPUT);
  pinMode(COUNTBUTTONPIN, INPUT);
  pinMode(BUZZERPIN, OUTPUT);
  Serial.begin(115200);
  first_serial_message();
  while_loop();
  maquina_de_estado();
}

void loop() {
  
}