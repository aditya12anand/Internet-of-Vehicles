#include "DHT.h"        // including the library of DHT11 temperature and humidity sensor
#define DHTTYPE DHT11   // DHT 11
#include <ESP8266WiFi.h>
#include <FirebaseArduino.h>
#include <Wire.h>
#include <SoftwareSerial.h> 
#include <TinyGPS.h> 

SoftwareSerial gpsSerial(D6,D5);//rx,tx 
TinyGPS gps; // create gps object 
float lat,lon;

#define dht_dpin D0
DHT dht(dht_dpin, DHTTYPE);

#define IR D2

/*
// MPU6050 Slave Device Address
const uint8_t MPU6050SlaveAddress = 0x68;
const uint8_t MPU6050_REGISTER_ACCEL_XOUT_H =  0x3B;


// Select SDA and SCL pins for I2C communication 
const uint8_t scl = D7;
const uint8_t sda = D8;

// sensitivity scale factor respective to full scale setting provided in datasheet 
const uint16_t AccelScaleFactor = 16384;
const uint16_t GyroScaleFactor = 131;


int16_t AccelX, AccelY, AccelZ, Temperature, GyroX, GyroY, GyroZ;
*/
const int button  = D4;
int temp=0;

#define FIREBASE_HOST "minor-project-70169.firebaseio.com"
#define FIREBASE_AUTH "2x1Gi4tWv53dNo0yKxuMVyslznTpZsqhQR8jALGm"
#define WIFI_SSID "Hi, what's up?"
#define WIFI_PASSWORD "123cdewsxzaq"

void setup(void)
{ 
  //dht.begin();
  Serial.begin(9600);
  gpsSerial.begin(9600); // connect gps sensor 
/*
Wire.begin(sda, scl);
  MPU6050_Init();
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
*/
WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("connecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  delay(500);
  Serial.println("connected: ");
  Serial.println(WiFi.localIP());
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);

pinMode(button,INPUT);
  delay(700);
}
void loop() {
  
    float h = dht.readHumidity();
    float t = dht.readTemperature();         
    Serial.print("Current humidity = ");
    Serial.print(h);
    Serial.print("%  ");
    Serial.print("temperature = ");
    Serial.print(t); 
    Serial.println("C  ");

    Firebase.pushFloat("t&h/temperature", t);
    Firebase.pushFloat("t&h/humidity", h);


Serial.println(digitalRead(IR));
Firebase.setInt("IR",digitalRead(IR));

    while(gpsSerial.available()){ // check for gps data 
  if(gps.encode(gpsSerial.read()))// encode gps data 
  {  
  gps.f_get_position(&lat,&lon); // get latitude and longitude 
  // display position 
  //Serial.print("Position: "); 
  //Serial.print("Latitude:"); 
  //Serial.print(lat,6); 
  //Serial.print(";"); 
  //Serial.print("Longitude:"); 
  //Serial.println(lon,6);  
  
 } 
} 
String latitude = String(lat,6); 
  String longitude = String(lon,6); 
Serial.println(latitude+";"+longitude); 
Firebase.pushString("gps/Latitude", latitude);
Firebase.pushString("gps/Longiitude", longitude);

/*
float Ax, Ay, Az, T, Gx, Gy, Gz;
  
  Read_RawValue(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H);
  
  //divide each with their sensitivity scale factor
  Ax = (float)AccelX/AccelScaleFactor;
  Ay = (float)AccelY/AccelScaleFactor;
  Az = (float)AccelZ/AccelScaleFactor;
  T = (float)Temperature/340+36.53; //temperature formula
  Gx = (float)GyroX/GyroScaleFactor;
  Gy = (float)GyroY/GyroScaleFactor;
  Gz = (float)GyroZ/GyroScaleFactor;

  Serial.print("Ax: "); Serial.print(Ax);
  Serial.print(" Ay: "); Serial.print(Ay);
  Serial.print(" Az: "); Serial.print(Az);
  Serial.print(" T: "); Serial.print(T);
  Serial.print(" Gx: "); Serial.print(Gx);
  Serial.print(" Gy: "); Serial.print(Gy);
  Serial.print(" Gz: "); Serial.println(Gz);

   Firebase.pushFloat("A&G/Ax",Ax);
   Firebase.pushFloat("A&G/Ay",Ay);
   Firebase.pushFloat("A&G/Az",Az);
   Firebase.pushFloat("A&G/Gx",Gx);
   Firebase.pushFloat("A&G/Gy",Gy);
   Firebase.pushFloat("A&G/Gz",Gz);
*/

    temp = digitalRead(button);
     
     if (temp == HIGH) {
        Serial.println("Emergency");
        Firebase.setString("Button","Emergency");
        delay(1000);
       }
     else {
        Serial.println("Normal");
        Firebase.setString("Button","Normal");
        delay(1000);
       }

    if (Firebase.failed()) {
        Serial.print("setting /number failed:");
        Serial.println(Firebase.error());
        return;
    }
  delay(100);
}
/*
void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data){
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();
}

// read all 14 register
void Read_RawValue(uint8_t deviceAddress, uint8_t regAddress){
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, (uint8_t)14);
  AccelX = (((int16_t)Wire.read()<<8) | Wire.read());
  AccelY = (((int16_t)Wire.read()<<8) | Wire.read());
  AccelZ = (((int16_t)Wire.read()<<8) | Wire.read());
  Temperature = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroX = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroY = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroZ = (((int16_t)Wire.read()<<8) | Wire.read());
}

//configure MPU6050
void MPU6050_Init(){
  // MPU6050 few configuration register addresses
const uint8_t MPU6050_REGISTER_SMPLRT_DIV   =  0x19;
const uint8_t MPU6050_REGISTER_USER_CTRL    =  0x6A;
const uint8_t MPU6050_REGISTER_PWR_MGMT_1   =  0x6B;
const uint8_t MPU6050_REGISTER_PWR_MGMT_2   =  0x6C;
const uint8_t MPU6050_REGISTER_CONFIG       =  0x1A;
const uint8_t MPU6050_REGISTER_GYRO_CONFIG  =  0x1B;
const uint8_t MPU6050_REGISTER_ACCEL_CONFIG =  0x1C;
const uint8_t MPU6050_REGISTER_FIFO_EN      =  0x23;
const uint8_t MPU6050_REGISTER_INT_ENABLE   =  0x38;
const uint8_t MPU6050_REGISTER_SIGNAL_PATH_RESET  = 0x68;
  delay(150);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SMPLRT_DIV, 0x07);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_1, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_2, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_CONFIG, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_GYRO_CONFIG, 0x00);//set +/-250 degree/second full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG, 0x00);// set +/- 2g full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_FIFO_EN, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_INT_ENABLE, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SIGNAL_PATH_RESET, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_USER_CTRL, 0x00);
}

*/

