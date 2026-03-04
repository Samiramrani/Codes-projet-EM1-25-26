#include <SoftwareSerial.h>

// Définition des broches de communication avec le HC-05
SoftwareSerial bluetooth(6, 7);

void setup() {
  Serial.begin(9600);         // Initialisation du port série pour le moniteur série PC
  bluetooth.begin(9600);
  Serial.println("Bridge serial <-> JDY-30");
//  pinMode(6, OUTPUT);
//  pinMode(7, INPUT);
}

void loop() {
  if(bluetooth.available()) // if the HC05 is sending something… 
  {

    char c = bluetooth.read();    
    Serial.print(c); // print in serial monitor
  }
  if(Serial.available()) // if serial monitor is outputting something… 
  {
    bluetooth.write(Serial.read()); // write to Arduino’s Tx pin
  }
  }
