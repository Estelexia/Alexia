#include <ESP32Servo.h>
#include <Wire.h>
#include <MPU6050.h>
#include <PID_v1.h>

MPU6050 sensor;

// Valores RAW (sin procesar) del acelerometro  en los ejes x,y,z
int16_t gx, gy, gz;
long tiempo_prev, dt;
float girosc_ang_x, girosc_ang_y;
float girosc_ang_x_prev, girosc_ang_y_prev;

//PID
//Define Variables we'll be connecting to
double Setpoint, Input, Output;
double Setpoint2, Input2, Output2;

//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
//double Kp2=2, Ki2=5, Kd2=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
//PID myPID(&Input2, &Output2, &Setpoint2, Kp2, Ki2, Kd2, DIRECT);

/*
//Pines
#define TRIG 15
#define ECHO 4
#define MotorD1 14
#define MotorD2 27
#define pwmMD 12
#define MotorI1 25
#define MotorI2 33
#define pwmMI 32

//Servo
Servo servo;
int pinServo=13;

//Variables
const int frec = 1000, canal1 = 5,canal2=6, res = 8;
float duracion, dist=0, distA=0, distI=0, distD=0;
int velocidadD=95, velocidadI=100;
float z=0;
*/

void setup() {
  Wire.begin();
  Serial.begin(9600);
  
  //MPU
  sensor.initialize();
  if (!sensor.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (true);
  }
  sensor.CalibrateGyro();

  //initialize the variables we're linked to
  Input = 0;
  Setpoint = 0;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  /*
  //Servo
  servo.attach(pinServo,500,2500);
  servo.write(90);
  //Motor derecho
  pinMode(MotorD1, OUTPUT);
  pinMode(MotorD2, OUTPUT);
  ledcSetup(canal1, frec, res);
  ledcAttachPin(pwmMD, canal1);
  ledcWrite(canal1, velocidadD);
  //Motor izquierdo
  pinMode(MotorI1, OUTPUT);
  pinMode(MotorI2, OUTPUT);
  ledcSetup(canal2, frec, res);
  ledcAttachPin(pwmMI, canal2);
  ledcWrite(canal2, velocidadI);
  //Sensor 
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  */
}

void loop() {
  // Leer las aceleraciones 
  sensor.getRotation(&gx, &gy, &gz);
  //Calcular los angulos rotacion:
  dt = millis()-tiempo_prev;
  tiempo_prev=millis();

  girosc_ang_x = (gx/131)*dt/1000.0 + girosc_ang_x_prev;
  girosc_ang_y = (gy/131)*dt/1000.0 + girosc_ang_y_prev;

  girosc_ang_x_prev=girosc_ang_x;
  girosc_ang_y_prev=girosc_ang_y;

  //Mostrar los angulos separadas por un [tab]
  Serial.print("Rotacion en X:  ");
  Serial.print(girosc_ang_x); 
  Serial.print("tRotacion en Y: ");
  Serial.println(girosc_ang_y);

  delay(100);
  int Salida;
  if(girosc_ang_x >= 0){
    Input = girosc_ang_x*(-1);
  }
  else{
    Input = girosc_ang_x;
  }
  myPID.Compute();
  /*
  if(girosc_ang_x >= 0){
    Salida = map(Output,0,255,100,150);
  }
  else{
    Salida = map(Output,0,255,100,50);
  }
  */
  /*
  if(Input > 0){
    Output = map(Output,0,255,100,50);
  }
*/
  /*
  if(Input >= 0){
    myPID.Compute();
    Salida = map(Output,0,255,100,150);
  }
  else{
    Input = Input*(-1);
    myPID.Compute();
    Salida = map(Output,0,255,100,50);
  }
  */
  /*
  Serial.print("Salida Controlador: ");
  Serial.println(Output);
  */
  Serial.print("Salida Controlador: ");
  Serial.println(Output);
/*
  //EVASOR
  distA=distancia();
  Serial.println("Distancia: ");
  Serial.println(distA);
  if(distA>=20 || distA==0){
    avanzar();
  } else if(distA<20 && distA>0){
    detener();
    delay(100);
  }

  int PosInicial = 0;
  int Err = girosc_ang_x - PosInicial;

  //CONTROL POSICION
  if(girosc_ang_x > 0){
    //aumentar velocidad rueda
  }
*/

}
/*
int distancia(){
  digitalWrite(TRIG, LOW);
  delayMicroseconds(3);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  duracion = pulseIn(ECHO, HIGH);
  dist = (343.0 / (10000.0 * 2.0)) * duracion;
  return dist;
}
void detener(){
  digitalWrite(MotorD1,LOW);
  digitalWrite(MotorD2,LOW);
  digitalWrite(MotorI1,LOW);
  digitalWrite(MotorI2,LOW);
}
void avanzar(){
  digitalWrite(MotorD1,HIGH);
  digitalWrite(MotorD2,LOW);
  digitalWrite(MotorI1,HIGH);
  digitalWrite(MotorI2,LOW);
}
void retroceder(){
  digitalWrite(MotorD1,LOW);
  digitalWrite(MotorD2,HIGH);
  digitalWrite(MotorI1,LOW);
  digitalWrite(MotorI2,HIGH);
}
void girarD(){
  digitalWrite(MotorD1,LOW);
  digitalWrite(MotorD2,HIGH);
  digitalWrite(MotorI1,HIGH);
  digitalWrite(MotorI2,LOW);
}
void girarI(){
  digitalWrite(MotorD1,HIGH);
  digitalWrite(MotorD2,LOW);
  digitalWrite(MotorI1,LOW);
  digitalWrite(MotorI2,HIGH);
}
*/
