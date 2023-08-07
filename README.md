# Triatlon_B2023

### Indice
[PINOUT PCB](#pinout)<br>
[Normas de convivencia de codigo](#normas)<br>
[Organizacion del repositorio y colaboracion](#colab)<br>
[Imagenes](#imag)<br>

<a name="pinout"></a>
## PINOUT PCB

### Kairimi
```
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
#define LEFT_CNY 27
#define RIGHT_CNY 25
#define BACK_CNY 26

// Control
#define BUZZER 18
#define PUSH_1 16
#define PUSH_2 17
```

### C5
```
// Placa en desarrollo
```
<br>

<a name="normas"></a>
## Normas de convivencia de codigo

* #### 1.  Convencion de nomenclatura:

    | Tipo      | Caso           | Ejemplo      |
    | :-------: |:--------------:|:------------:|
    | constantes| screaming snake|MAX_READING   |
    | variables | snake case     |color_deadline|
    | objetos   | snake case     |left_sharp    |
    | clases    | pascal case    |Motor         |
    | funciones | pascal case    |ReadFloor     |
    

* #### 2.  Orden del codigo:

    * Importar librerias
    * Definicion pines
    * Definicion de constantes / variables globales
    * Creacion de clases / objetos (hasta terminar la modularizacion)
    * Funciones globales
    * Velocista
    * Sumo radiocontrolado
    * Despejar area
    * Seleccion de modo / setup / loop
    
* #### 3. Legibilidad:

    * Usar define para los numeros aislados
    * Comentarios descriptivos
    * Codigo limpio y ordenado

<br>

<a name="colab"></a>
## Organizacion del repositorio y colaboracion

### Arbol de carpetas:

```
├───C5
│   ├───3D
│   ├───CODE
│   └───PCB
├───Kairimi
│   ├───3D
│   ├───CODE
│   └───PCB
├───resources
└───shared
    ├───examples
    └───libs
```

* C5: Dedicado a la version C5 (Rodri/Mauri)

* Kairimi: Dedicado a la version Kairimi (Iris/Julian).
* resources: Librerias externas necesarias para correr el codigo
* shared: Archivos compartidos para los dos robots, como librerias dedicadas

### Colaboracion:

Para colaborar es preferible forkear el repositorio, editar lo necesario en el momento y enviar un pull request. Esto para mantener la actualizacion de archivos ordenada y evitar conflictos entre los robots.

Cuando se alcance una version funcional del robot tagerla marcando las diferencias y mejoras respecto a la anterior y tiempos records de pistas.

<br>

<a name="imag"></a>
## Imagenes 3Ds / PCBs

### KAIRIMI:

[![Captura-de-pantalla-2023-04-26-111438.png](https://i.postimg.cc/HxC7txbK/Captura-de-pantalla-2023-04-26-111438.png)](https://postimg.cc/2q26kCcF)

### C5:

3d en desarrollo.

# Manolos ahi voy 🤤

