#include<LiquidCrystal.h>


LiquidCrystal lcd(12, 11, 5, 4, 3, 2);  // sets the interfacing pins

const int trigPin = 10;
const int echoPin = 9;
// defines variables
long duration;
int distance;
void setup() {
   lcd.begin(16, 2);  // initializes the 16x2 LCD
pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
pinMode(echoPin, INPUT); // Sets the echoPin as an Input
Serial.begin(9600); // Starts the serial communication
}
void loop() {
// Clears the trigPin
digitalWrite(trigPin, LOW);
delayMicroseconds(2);
// Sets the trigPin on HIGH state for 10 micro seconds
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);
// Reads the echoPin, returns the sound wave travel time in microseconds
duration = pulseIn(echoPin, HIGH);
// Calculating the distance
distance= duration*0.034/2;
// Prints the distance on the Serial Monitor
lcd.setCursor(0,0);           //sets the cursor at row 0 column 0
  lcd.print("IoV"); // prints 16x2 LCD MODULE
  lcd.setCursor(2,1);//sets the cursor at row 1 column 2
  lcd.print("Distance: ");
  lcd.print(distance); 
Serial.println(distance);
delay(500);
}
