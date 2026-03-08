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
float Rayon_base =  11.45; // Essai pour voir si rotation se passe mieux (valeur de base : 12.0)

const int encoderPin1 = 2; // qui est connecté au moteur 1 mais sur le driver c'est 2
const int encoderPin2 = 3;
volatile long encoderTicks1 = 0;
volatile long encoderTicks2 = 0; 

const float CPR1_HIGH = 709.2; // OK
const float CPR2_HIGH = 353.4; // OK
const float CPR1_LOW = 353.4; // OK
const float CPR2_LOW = 353.4; // OK mais pas le mm qu'avant

int PWM1 = 9; // Là c'est les bons pins avec les bons branchements (tout est connecté en 1)
int DIR1 = 8;
int PWM2 = 6;
int DIR2 = 7;

// Régulation : quelle vitesse ? intervalle de temps choisi pour la régulation ?
float vitesse = 15.0;

float Kp1_HIGH = 10.6; // Changer soit les Kp soit l'intervalle de temps (voir notes)
float Kp2_HIGH = 10.4;
float Kp1_LOW = 10.9;
float Kp2_LOW = 10;

// Profil trapèzoïdal
float pente = 75;


unsigned long currentTime; // pour utiliser millis() après
unsigned long prevTime;


void goForward(float distance){

  prevTime = millis();

  float valeur_PWM1 = 50;
  float valeur_PWM2 = 50;
  encoderTicks1 = 0;
  encoderTicks2 = 0;
  float prevE1 = 0;
  float prevE2 = 0;

  float ticks_a_faire1 = distance/(2*pi*Rayon) * CPR1_HIGH ;
  float ticks_a_faire2 = distance/(2*pi*Rayon) * CPR2_HIGH ;

  // Phase d'accélération

  while ((encoderTicks1 < 0.2*CPR1_HIGH) && (encoderTicks2 < 0.2*CPR2_HIGH)){

    digitalWrite(DIR1, HIGH);
    analogWrite(PWM1, valeur_PWM1);
    digitalWrite(DIR2, HIGH);
    analogWrite(PWM2, valeur_PWM2);

    delay(10);
    
    currentTime = millis();

    float delta_time = (currentTime - prevTime)/1000.0; // les temps sont en millisecondes

    float ref = constrain((pente/2)*(delta_time*delta_time), 0, 2.64);
    float reel1 = (encoderTicks1*2*pi*Rayon)/(CPR1_HIGH);
    float reel2 = (encoderTicks2*2*pi*Rayon)/(CPR2_HIGH);
    float erreur1 = (ref - reel1);
    float erreur2 = (ref - reel2);


    valeur_PWM1 = constrain(Kp1_HIGH*erreur1, 50, 200); 
    valeur_PWM2 = constrain(Kp2_HIGH*erreur2, 50, 200);
    Serial.println(erreur2);
  };

  prevTime = millis();
  prevE1 = encoderTicks1;
  prevE2 = encoderTicks2;

  while ((encoderTicks1 < ticks_a_faire1) && (encoderTicks2 < ticks_a_faire2)){
  
    digitalWrite(DIR1, HIGH);
    analogWrite(PWM1, valeur_PWM1);
    digitalWrite(DIR2, HIGH);
    analogWrite(PWM2, valeur_PWM2);

    delay(10);
    
    currentTime = millis();

    float delta_time = (currentTime - prevTime)/1000.0; // les temps sont en millisecondes

    float ref = vitesse*delta_time;
    float reel1 = ((encoderTicks1-prevE1)*2*pi*Rayon)/(CPR1_HIGH);
    float reel2 = ((encoderTicks2-prevE2)*2*pi*Rayon)/(CPR2_HIGH);
    float erreur1 = (ref - reel1);
    float erreur2 = (ref - reel2);


    valeur_PWM1 = constrain(Kp1_HIGH*erreur1, 50, 200); 
    valeur_PWM2 = constrain(Kp2_HIGH*erreur2, 50, 200);

    Serial.println(erreur2);

  };

    digitalWrite(DIR1, HIGH);
    analogWrite(PWM1, 0);
    digitalWrite(DIR2, HIGH);
    analogWrite(PWM2, 0);

}

void goBackward(float distance){

  prevTime = millis();

  float valeur_PWM1 = 50;
  float valeur_PWM2 = 50;
  encoderTicks1 = 0;
  encoderTicks2 = 0;
  float prevE1 = 0;
  float prevE2 = 0;

  float ticks_a_faire1 = distance/(2*pi*Rayon) * CPR1_LOW ;
  float ticks_a_faire2 = distance/(2*pi*Rayon) * CPR2_LOW ;

  // Phase d'accélération

  while ((encoderTicks1 < 0.2*CPR1_LOW) && (encoderTicks2 < 0.2*CPR2_LOW)){

    digitalWrite(DIR1, LOW);
    analogWrite(PWM1, valeur_PWM1);
    digitalWrite(DIR2, LOW);
    analogWrite(PWM2, valeur_PWM2);

    delay(10);
    
    currentTime = millis();

    float delta_time = (currentTime - prevTime)/1000.0; // les temps sont en millisecondes

    float ref = constrain((pente/2)*(delta_time*delta_time), 0, 2.64);
    float reel1 = (encoderTicks1*2*pi*Rayon)/(CPR1_LOW);
    float reel2 = (encoderTicks2*2*pi*Rayon)/(CPR2_LOW);
    float erreur1 = (ref - reel1);
    float erreur2 = (ref - reel2);


    valeur_PWM1 = constrain(Kp1_LOW*erreur1, 50, 200); 
    valeur_PWM2 = constrain(Kp2_LOW*erreur2, 50, 200);
    Serial.println(erreur1-erreur2);
  };

  prevTime = millis();
  prevE1 = encoderTicks1;
  prevE2 = encoderTicks2;

  while ((encoderTicks1 < ticks_a_faire1) && (encoderTicks2 < ticks_a_faire2)){
  
    digitalWrite(DIR1, LOW);
    analogWrite(PWM1, valeur_PWM1);
    digitalWrite(DIR2, LOW);
    analogWrite(PWM2, valeur_PWM2);

    delay(10);
    
    currentTime = millis();

    float delta_time = (currentTime - prevTime)/1000.0; // les temps sont en millisecondes

    float ref = vitesse*delta_time;
    float reel1 = ((encoderTicks1-prevE1)*2*pi*Rayon)/(CPR1_LOW);
    float reel2 = ((encoderTicks2-prevE2)*2*pi*Rayon)/(CPR2_LOW);
    float erreur1 = (ref - reel1);
    float erreur2 = (ref - reel2);


    valeur_PWM1 = constrain(Kp1_LOW*erreur1, 50, 200); 
    valeur_PWM2 = constrain(Kp2_LOW*erreur2, 50, 200);

    Serial.println(erreur1);

  };

    digitalWrite(DIR1, HIGH);
    analogWrite(PWM1, 0);
    digitalWrite(DIR2, HIGH);
    analogWrite(PWM2, 0);

}

void goRight(float angle){

  float distance = angle/180*pi*Rayon_base;

  prevTime = millis();

  float valeur_PWM1 = 50;
  float valeur_PWM2 = 50;
  encoderTicks1 = 0;
  encoderTicks2 = 0;
  float prevE1 = 0;
  float prevE2 = 0;

  float ticks_a_faire1 = distance/(2*pi*Rayon) * CPR1_HIGH ;
  float ticks_a_faire2 = distance/(2*pi*Rayon) * CPR2_LOW ;

  // Phase d'accélération

  while ((encoderTicks1 < 0.2*CPR1_HIGH) && (encoderTicks2 < 0.2*CPR2_LOW)){

    digitalWrite(DIR1, HIGH);
    analogWrite(PWM1, valeur_PWM1);
    digitalWrite(DIR2, LOW);
    analogWrite(PWM2, valeur_PWM2);

    delay(10);
    
    currentTime = millis();

    float delta_time = (currentTime - prevTime)/1000.0; // les temps sont en millisecondes

    float ref = constrain((pente/2)*(delta_time*delta_time), 0, 2.64);
    float reel1 = (encoderTicks1*2*pi*Rayon)/(CPR1_HIGH);
    float reel2 = (encoderTicks2*2*pi*Rayon)/(CPR2_LOW);
    float erreur1 = (ref - reel1);
    float erreur2 = (ref - reel2);


    valeur_PWM1 = constrain(Kp1_HIGH*erreur1, 50, 200); 
    valeur_PWM2 = constrain(Kp2_LOW*erreur2, 50, 200);
    Serial.println(erreur1-erreur2);
  };

  prevTime = millis();
  prevE1 = encoderTicks1;
  prevE2 = encoderTicks2;

  while ((encoderTicks1 < ticks_a_faire1) && (encoderTicks2 < ticks_a_faire2)){
  
    digitalWrite(DIR1, HIGH);
    analogWrite(PWM1, valeur_PWM1);
    digitalWrite(DIR2, LOW);
    analogWrite(PWM2, valeur_PWM2);

    delay(10);
    
    currentTime = millis();

    float delta_time = (currentTime - prevTime)/1000.0; // les temps sont en millisecondes

    float ref = vitesse*delta_time;
    float reel1 = ((encoderTicks1-prevE1)*2*pi*Rayon)/(CPR1_HIGH);
    float reel2 = ((encoderTicks2-prevE2)*2*pi*Rayon)/(CPR2_LOW);
    float erreur1 = (ref - reel1);
    float erreur2 = (ref - reel2);


    valeur_PWM1 = constrain(Kp1_HIGH*erreur1, 50, 200); 
    valeur_PWM2 = constrain(Kp2_LOW*erreur2, 50, 200);

    Serial.println(erreur1-erreur2);

  };

    digitalWrite(DIR1, HIGH);
    analogWrite(PWM1, 0);
    digitalWrite(DIR2, HIGH);
    analogWrite(PWM2, 0);


}

void goLeft(float angle){
  float distance = angle/180*pi*Rayon_base;

  prevTime = millis();

  float valeur_PWM1 = 50;
  float valeur_PWM2 = 50;
  encoderTicks1 = 0;
  encoderTicks2 = 0;
  float prevE1 = 0;
  float prevE2 = 0;

  float ticks_a_faire1 = distance/(2*pi*Rayon) * CPR1_LOW ;
  float ticks_a_faire2 = distance/(2*pi*Rayon) * CPR2_HIGH ;

  // Phase d'accélération

  while ((encoderTicks1 < 0.2*CPR1_LOW) && (encoderTicks2 < 0.2*CPR2_HIGH)){

    digitalWrite(DIR1, LOW);
    analogWrite(PWM1, valeur_PWM1);
    digitalWrite(DIR2, HIGH);
    analogWrite(PWM2, valeur_PWM2);

    delay(10);
    
    currentTime = millis();

    float delta_time = (currentTime - prevTime)/1000.0; // les temps sont en millisecondes

    float ref = constrain((pente/2)*(delta_time*delta_time), 0, 2.64);
    float reel1 = (encoderTicks1*2*pi*Rayon)/(CPR1_LOW);
    float reel2 = (encoderTicks2*2*pi*Rayon)/(CPR2_HIGH);
    float erreur1 = (ref - reel1);
    float erreur2 = (ref - reel2);


    valeur_PWM1 = constrain(Kp1_LOW*erreur1, 50, 200); 
    valeur_PWM2 = constrain(Kp2_HIGH*erreur2, 50, 200);
    Serial.println(erreur1-2);
  };

  prevTime = millis();
  prevE1 = encoderTicks1;
  prevE2 = encoderTicks2;

  while ((encoderTicks1 < ticks_a_faire1) && (encoderTicks2 < ticks_a_faire2)){
  
    digitalWrite(DIR1, LOW);
    analogWrite(PWM1, valeur_PWM1);
    digitalWrite(DIR2, HIGH);
    analogWrite(PWM2, valeur_PWM2);

    delay(10);
    
    currentTime = millis();

    float delta_time = (currentTime - prevTime)/1000.0; // les temps sont en millisecondes

    float ref = vitesse*delta_time;
    float reel1 = ((encoderTicks1-prevE1)*2*pi*Rayon)/(CPR1_LOW);
    float reel2 = ((encoderTicks2-prevE2)*2*pi*Rayon)/(CPR2_HIGH);
    float erreur1 = (ref - reel1);
    float erreur2 = (ref - reel2);


    valeur_PWM1 = constrain(Kp1_LOW*erreur1, 50, 200); 
    valeur_PWM2 = constrain(Kp2_HIGH*erreur2, 50, 200);

    Serial.println(erreur1-erreur2);

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
 
  delay(10000);
  goForward(200);
  goRight(360);
  goBackward(100);
  goLeft(180);
  goForward(100);

  digitalWrite(DIR1, HIGH);
  analogWrite(PWM1, 0);
  digitalWrite(DIR2, HIGH);
  analogWrite(PWM2, 0);
  delay(5000);

};

void countTicks1(){
  encoderTicks1++;
}

void countTicks2(){
  encoderTicks2++;
}