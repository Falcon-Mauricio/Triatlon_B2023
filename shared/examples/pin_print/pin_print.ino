#define SENSOR_1 13
#define SENSOR_3 26
#define SENSOR_2 27
#define SENSOR_4 25
#define SENSOR_5 34
#define SENSOR_6 35
#define SENSOR_7 32
#define SENSOR_8 33

int PINS[] = { SENSOR_1, SENSOR_2, SENSOR_3, SENSOR_4, 
                SENSOR_5, SENSOR_6, SENSOR_7, SENSOR_8 };

#define PIN_CANT 8
#define DELAY 400

void setup(){

    Serial.begin(115200);

}

void loop(){

    for(int i = 0; i < PIN_CANT; i++){
        int value = analogRead(PINS[i]);
        Serial.println(value);
    }
    Serial.println();
    delay(DELAY);

}