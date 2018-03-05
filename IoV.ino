#include "DHT.h"        // including the library of DHT11 temperature and humidity sensor
#define DHTTYPE DHT11   // DHT 11
#include <ESP8266WiFi.h>
#include <FirebaseArduino.h>

#include <SoftwareSerial.h> 
#include <TinyGPS.h> 
float lat = 28.5458,lon = 77.1703; // create variable for latitude and longitude object  
SoftwareSerial gpsSerial(D7,D6);//rx,tx 
TinyGPS gps; // create gps object 

#define dht_dpin 0
DHT dht(dht_dpin, DHTTYPE); 

#define FIREBASE_HOST "minor-project-70169.firebaseio.com"
#define FIREBASE_AUTH "2x1Gi4tWv53dNo0yKxuMVyslznTpZsqhQR8jALGm"
#define WIFI_SSID "Rajesh"
#define WIFI_PASSWORD "8124664004"

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

  Serial.println("Humidity and temperature\n\n");
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

    //Firebase.pushString("gps/Latitude", latitude);
    //Firebase.pushString("gps/Longiitude", longitude);

    if (Firebase.failed()) {
        Serial.print("setting /number failed:");
        Serial.println(Firebase.error());
        return;
    }
  delay(1000);
}


