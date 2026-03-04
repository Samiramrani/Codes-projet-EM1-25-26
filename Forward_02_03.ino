// Librairies
#include <Servo.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <SoftwareSerial.h>

// Constantes
const float pi = 3.14159;

// Données du proto (tt est en cm)
float Rayon = 2.1;
float Rayon_base = 19.7/2;

const int encoderPin1 = 3; // qui est connecté au moteur 1 mais sur le driver c'est 2
const int encoderPin2 = 2;
volatile long encoderTicks1 = 0;
volatile long encoderTicks2 = 0; 

const float CPR1_HIGH = 709.2; 
const float CPR2_HIGH = 353.4;
const float CPR1_LOW = 353.4;
const float CPR2_LOW = 431.3;

int PWM1 = 9; // Là c'est les bons pins avec les bons branchements (tout est connecté en 1)
int DIR1 = 8;
int PWM2 = 6;
int DIR2 = 7;

// Régulation : quelle vitesse ? intervalle de temps choisi pour la régulation ?
float vitesse = 15.0;
float Kp1 = 1; // Changer soit les Kp soit l'intervalle de temps (voir notes)
float Kp2 = 0.0001;
unsigned long currentTime; // pour utiliser millis() après
unsigned long prevTime;


void goForward(float distance){

  prevTime = millis();

  int valeur_PWM1 = 75;
  int valeur_PWM2 = 90;
  encoderTicks1 = 0;
  encoderTicks2 = 0;

  float ticks_a_faire1 = distance/(2*pi*Rayon) * CPR1_HIGH ;
  float ticks_a_faire2 = distance/(2*pi*Rayon) * CPR2_HIGH ;

  while ((encoderTicks1 < ticks_a_faire1) && (encoderTicks2 < ticks_a_faire2)){
  
    digitalWrite(DIR1, HIGH);
    analogWrite(PWM1, valeur_PWM1);
    digitalWrite(DIR2, HIGH);
    analogWrite(PWM2, valeur_PWM2);

    delay(1000);
    
    currentTime = millis();

    long delta_time = (currentTime - prevTime)/1000; // les temps sont en millisecondes

    float erreur1 = vitesse*delta_time - encoderTicks1*2*pi*Rayon/CPR1_HIGH;
    float erreur2 = vitesse*delta_time - encoderTicks2*2*pi*Rayon/CPR2_HIGH;

    valeur_PWM1 = valeur_PWM1 + Kp1*erreur1; 
    valeur_PWM2 = valeur_PWM2 + Kp2*erreur2;

    Serial.println(encoderTicks1);
    //Serial.println(erreur2);
    //Serial.println(valeur_PWM1);
    //Serial.println(valeur_PWM2);

  };

    digitalWrite(DIR1, HIGH);
    analogWrite(PWM1, 0);
    digitalWrite(DIR2, HIGH);
    analogWrite(PWM2, 0);

}

void setup() {
  
Serial.begin(9600);

pinMode(DIR1, OUTPUT);
pinMode(DIR2, OUTPUT);

pinMode(encoderPin1, INPUT_PULLUP);
pinMode(encoderPin2, INPUT_PULLUP);

attachInterrupt(digitalPinToInterrupt(encoderPin1), countTicks1, RISING);
attachInterrupt(digitalPinToInterrupt(encoderPin2), countTicks2, RISING);

}

void loop() {

  goForward(500);
  delay(1000);

}

void countTicks1(){
  encoderTicks1++;
}

void countTicks2(){
  encoderTicks2++;
}