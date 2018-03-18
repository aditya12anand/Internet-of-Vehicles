#include "DHT.h"        // including the library of DHT11 temperature and humidity sensor
#define DHTTYPE DHT11   // DHT 11
#include <ESP8266WiFi.h>
#include <FirebaseArduino.h>
#include <SoftwareSerial.h> 
#include <TinyGPS.h> 

SoftwareSerial gpsSerial(D6,D5);//rx,tx 
TinyGPS gps; // create gps object 
float lat=12.824891,lon=80.045040;
float hu=44.0,te=33.0;

#define dht_dpin D0
DHT dht(dht_dpin, DHTTYPE);

#define IR D2

#define FIREBASE_HOST "minor-project-70169.firebaseio.com"
#define FIREBASE_AUTH "2x1Gi4tWv53dNo0yKxuMVyslznTpZsqhQR8jALGm"
#define WIFI_SSID "desktop"
#define WIFI_PASSWORD "147258369"

void setup(void)
{ 
  dht.begin();
  Serial.begin(9600);

  gpsSerial.begin(9600); // connect gps sensor 

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

delay(500);
}

void loop() {
  
    float h = dht.readHumidity();
    float t = dht.readTemperature();         
    Serial.print("Current humidity = ");
    Serial.print(hu);
    Serial.print("%  ");
    Serial.print("temperature = ");
    Serial.print(te); 
    Serial.println("C  ");

    Firebase.pushFloat("t&h/temperature", te);
    Firebase.pushFloat("t&h/humidity", hu);

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

String msg = Firebase.getString("message");
Serial.println(msg);
delay(5000);
msg="";

    if (Firebase.failed()) {
        Serial.print("setting /number failed:");
        Serial.println(Firebase.error());
        return;
    }
  delay(100);
}
