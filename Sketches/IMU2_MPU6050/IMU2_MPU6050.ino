#include <Wire.h>

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>

ros::NodeHandle  nh;

std_msgs::Float32 float_msg;
ros::Publisher Gz("Gz", &float_msg);

std_msgs::Float32 float_msgDD;
std_msgs::Float32 float_msgDM;
std_msgs::Float32 float_msgDI;
ros::Publisher DistanciaDerecha("DistanciaDerecha", &float_msgDD);
ros::Publisher DistanciaMedio("DistanciaMedio", &float_msgDM);
ros::Publisher DistanciaIzquierda("DistanciaIzquierda", &float_msgDI);


float RateYaw;

float RateCalibrationYaw;
int RateCalibrationNumber;

double dt, timer;
float GzAng;

bool start_publishing = false;

const int TRIG_LEFT = 13;  
const int ECHO_LEFT = 12;  
const int TRIG_RIGHT = 4;  
const int ECHO_RIGHT = 5;
const int TRIG_FRONT = 9;   
const int ECHO_FRONT = 8; 

bool startCallback(const std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  start_publishing = true;
  nh.loginfo("Start command received. Publishing will start.");
  return true;
}

void gyro_signals(){
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();

  Wire.requestFrom(0x68,6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  RateYaw = (float)GyroZ/65.5;
}

ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> start_service("start_publishing", &startCallback);

void setup() {
  Serial.begin(57600);

  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);

  Serial.print("XD");
  
  pinMode(13,OUTPUT);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  Serial.print("XD");

  nh.initNode();
  nh.advertiseService(start_service);

  while (start_publishing != true) {
    Serial.print("Waiting for Response!");
    nh.spinOnce();
  }
  
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 4000; RateCalibrationNumber ++) {
    gyro_signals();
    RateCalibrationYaw += RateYaw;
    delay(1);
  }
  
  RateCalibrationYaw /= 4000;
  digitalWrite(13,HIGH);

  nh.advertise(Gz);
  nh.advertise(DistanciaDerecha);
  nh.advertise(DistanciaMedio);
  nh.advertise(DistanciaIzquierda);
}

float es_ruta_libre(int trigPin, int echoPin) {
  // Enviar pulso ultrasónico
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Medir la duración del pulso de retorno
  float duracion = pulseIn(echoPin, HIGH);
  float distancia = duracion * 0.034 / 2;

  // Si la distancia es mayor que la distancia segura, considera que la ruta está libre
  return distancia;
}

void loop() {
  float izquierdaLibre = es_ruta_libre(TRIG_LEFT, ECHO_LEFT);
  float derechaLibre = es_ruta_libre(TRIG_RIGHT, ECHO_RIGHT);
  float frenteLibre = es_ruta_libre(TRIG_FRONT, ECHO_FRONT);
  
  gyro_signals();
 
  dt = (double) (millis()-timer)/1000;
  timer = millis();
  
  RateYaw -= RateCalibrationYaw;

  if (RateYaw < 0.07 && RateYaw > -0.07) {
    RateYaw = 0;
  }

  GzAng += RateYaw*dt;

  float_msg.data = GzAng;
  Gz.publish(&float_msg);
  
  float_msgDD.data = derechaLibre;
  DistanciaDerecha.publish(&float_msgDD);

  float_msgDM.data = frenteLibre;
  DistanciaMedio.publish(&float_msgDM);

  float_msgDI.data = izquierdaLibre;
  DistanciaIzquierda.publish(&float_msgDI);

  nh.spinOnce();
}
