// Librerias I2C para controlar el mpu6050
// la libreria MPU6050.h necesita I2Cdev.h, I2Cdev.h necesita Wire.h
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include <Servo.h>

// La dirección del MPU6050 puede ser 0x68 o 0x69, dependiendo
// del estado de AD0. Si no se especifica, 0x68 estará implicito
MPU6050 sensor;

//Creaciñon de Objetos Servo
Servo myservo1;
Servo myservo2;

//Asignación de pines del JoyStick
const int pinJoyY = A0;
const int pinJoyZ = A1;
const int pinJoyButton = 8;

// Valores RAW (sin procesar) del acelerometro y giroscopio en los ejes x,y,z
int ax, ay, az;
int gx, gy, gz;
// Variables del Joystick
int joy_z, joy_y;
int Zvalue = 0;
int Yvalue = 0;
bool buttonValue;
// Parámetros del estabilizador
long tiempo_prev;
float dt;
unsigned long tiempo = 0;
float ang_z, ang_y;
float ang_z_prev, ang_y_prev;
//Variables para sobreescribir el lazo
int ovserv_z = 0, ovserv_y = 0;

void setup() {
  Serial.begin(9600);    //Iniciando puerto serial
  Wire.begin();     //Iniciando I2C
  sensor.initialize();    //Iniciando el sensor
  if (sensor.testConnection()) Serial.println("Sensor iniciado correctamente");
  else Serial.println("Error al iniciar el sensor");

  myservo1.attach(9);
  myservo2.attach(10);
  pinMode(pinJoyButton , INPUT_PULLUP);   //activar resistencia pull up

}

void loop() {
  //Lectura del tiempo
  tiempo = millis();

  //Lectura de los valores del joystick
  Zvalue = analogRead(pinJoyZ);
  Yvalue = analogRead(pinJoyY);
  buttonValue = digitalRead(pinJoyButton);

  if (Zvalue > 600 && joy_z < 9) joy_z--;
  if (Zvalue < 400 && joy_z > -9) joy_z++;
  if (joy_y > -9 && Yvalue > 600) joy_y--;
  if (joy_y < 9 &&  Yvalue < 400) joy_y++;

  // Lectura de las aceleraciones y velocidades angulares
  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);

  dt = (millis() - tiempo_prev) / 1000.0;
  tiempo_prev = millis();

  //Calcular los ángulos con acelerometro
  float accel_ang_y = atan(-az / sqrt(pow(ay, 2) + pow(ax, 2))) * (180.0 / 3.14);
  float accel_ang_z = atan(ay / sqrt(pow(az, 2) + pow(ax, 2))) * (180.0 / 3.14);

  //Calcular el angulo de rotación con giroscopio y filtro complemento
  ang_y = 0.98 * (ang_y_prev + (gy / 131) * dt) + 0.02 * accel_ang_y;
  ang_z = 0.98 * (ang_z_prev + (gz / 131) * dt) + 0.02 * accel_ang_z;

  ang_y_prev = ang_y;
  ang_z_prev = ang_z;

  //Impresión de los valores por el puerto serie
  Serial.print(joy_z);
  Serial.print("\t ");
  Serial.print(joy_y);
  Serial.print("\t ");

  Serial.print(tiempo);
  Serial.print("\t");
  Serial.print((gy / 131) * dt);
  Serial.print("\t");
  Serial.print(ang_y);
  Serial.print("\t");
  Serial.print(accel_ang_y);
  Serial.print("\t");
  Serial.print((gz / 131) * dt);
  Serial.print("\t");
  Serial.print(ang_z);
  Serial.print("\t");
  Serial.print(accel_ang_z);
  Serial.print("\t");
  Serial.print(ovserv_y);
  Serial.print("\t");
  Serial.print(ovserv_z);
  Serial.println("\t");


  //Variables para cerrar el lazo y sobreescribir la posición
  if (ang_y < -5 + 5 * joy_y && ovserv_y < 90) {
    ovserv_y ++;
  }

  if (ang_y > 5 + 5 * joy_y && ovserv_y > -90) {
    ovserv_y --;
  }

  if (ang_z < -5 + 5 * joy_z && ovserv_z < 90) {
    ovserv_z ++;
  }

  if (ang_z >  5 + 5 * joy_z && ovserv_z > -90) {
    ovserv_z --;
  }

  //Posicionamiento de los servomotores
  myservo2.write(90 + ang_y - ovserv_y);
  myservo1.write(90 - ang_z + ovserv_z );

}
