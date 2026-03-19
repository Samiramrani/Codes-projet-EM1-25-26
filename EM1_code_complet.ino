// Librairies
#include <Servo.h>
#include <SPI.h>
#include <RF24.h>
#include <SoftwareSerial.h>
#include <math.h>
#include <nRF24L01.h>

// Constantes
const float pi = 3.14159;

int pos = 0;

// Données du proto (tt est en cm)
float Rayon = 2.1;
float Rayon_base =  11.35; // Valeur d'avant (11.48)
float Diameter_base = 22.96;

const int encoderPin1 = 2; // qui est connecté au moteur 1 mais sur le driver c'est 2
const int encoderPin2 = 3;
volatile long encoderTicks1 = 0;
volatile long encoderTicks2 = 0; 

const float CPR1_HIGH = 709.2; // OK
const float CPR2_HIGH = 353.4; // OK
const float CPR1_LOW = 353.4; // OK
const float CPR2_LOW = 1000.0; // OK mais pas le mm qu'avant 353.5 avant

int PWM1 = 5; // Là c'est les bons pins avec les bons branchements (tout est connecté en 1)
int DIR1 = 7;
int PWM2 = 6;
int DIR2 = 8;

int IRSensor1 = 4;
int IRSensor2 = A4;

// Régulation : quelle vitesse ? intervalle de temps choisi pour la régulation ?
float vitesse = 15.0;

float Kp1_HIGH = 10; // OK
float Kp2_HIGH = 9.7; // OK
float Kp1_LOW = 10.3; // OK
float Kp2_LOW = 9.6; // OK

// Profil trapèzoïdal
float pente = 75;


unsigned long currentTime; // pour utiliser millis() après
unsigned long prevTime;

// Servo
Servo myservo;

// BT
SoftwareSerial bluetooth(A2, A3);

// Radio
#define pinCE   A0
#define pinCSN  A1
//#define tunnel1  "00001"
//#define tunnel2 "00002"
RF24 radio(pinCE, pinCSN);

const byte addresses[][6] = {"CANAL1", "CANAL2"};

unsigned long tempsPrecedent = 0;


// Configuration OXO
const int8_t VIDE = 0;
int8_t ROBOT_IA;    
int8_t ADVERSAIRE; 
char monSymbole;    // 'X' ou 'O'
bool estMonTour;

int8_t plateau[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// Les lignes gagnantes [cite: 6]
const int8_t WIN_LINES[8][3] = {
  {1,2,3}, {4,5,6}, {7,8,9}, 
  {1,4,7}, {2,5,8}, {3,6,9}, 
  {1,5,9}, {3,5,7}           
};

// Priorité en distance
const int8_t ORDRE_TEST[9] = {1, 4, 7, 2, 5, 8, 3, 6, 9}; 

// Fonctions logiques

int8_t verifierGagnant(int8_t b[10]) {
  for (int i = 0; i < 8; i++) {
    if (b[WIN_LINES[i][0]] != VIDE &&
        b[WIN_LINES[i][0]] == b[WIN_LINES[i][1]] &&
        b[WIN_LINES[i][0]] == b[WIN_LINES[i][2]]) {
      return b[WIN_LINES[i][0]];
    }
  }
  bool plein = true;
  for (int i = 1; i <= 9; i++) if (b[i] == VIDE) plein = false;
  if (plein) return -1; // Match nul [cite: 7]
  return VIDE;
}

// Algorithme Alpha-Beta avec Backtracking (pas de .copy() pour la RAM) 
int alphabeta(int8_t b[10], bool maximizing, int alpha, int beta) {
  int8_t gagnant = verifierGagnant(b);
  if (gagnant == ROBOT_IA) return 1;    // [cite: 8]
  if (gagnant == ADVERSAIRE) return -1;
  if (gagnant == -1) return 0;          // [cite: 7]

  if (maximizing) {
    int bestV = -2;
    for (int i = 0; i < 9; i++) {
      int m = ORDRE_TEST[i];
      if (b[m] == VIDE) {
        b[m] = ROBOT_IA; 
        int v = alphabeta(b, false, alpha, beta);
        b[m] = VIDE; // Annuler le coup pour économiser la RAM
        if (v > bestV) bestV = v;
        if (bestV > alpha) alpha = bestV;
        if (beta <= alpha) break; // Elagage Alpha-Beta [cite: 11]
      }
    }
    return bestV;
  } else {
    int bestV = 2;
    for (int i = 0; i < 9; i++) {
      int m = ORDRE_TEST[i];
      if (b[m] == VIDE) {
        b[m] = ADVERSAIRE;
        int v = alphabeta(b, true, alpha, beta);
        b[m] = VIDE;
        if (v < bestV) bestV = v;
        if (bestV < beta) beta = bestV;
        if (beta <= alpha) break; // Elagage Alpha-Beta [cite: 13]
      }
    }
    return bestV;
  }
}

int trouverMeilleurCoup() {
  int bestScore = -2;
  int meilleurMouve = -1;
  for (int i = 0; i < 9; i++) {
    int m = ORDRE_TEST[i];
    if (plateau[m] == VIDE) {
      plateau[m] = ROBOT_IA;
      int score = alphabeta(plateau, false, -2, 2); // [cite: 18]
      plateau[m] = VIDE;
      if (score > bestScore) {
        bestScore = score;
        meilleurMouve = m;
      }
    }
  }
  return meilleurMouve;
}

void goForward(float distance){

  prevTime = millis();

  float valeur_PWM1 = 48;
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


    valeur_PWM1 = constrain(Kp1_HIGH*erreur1, 48, 200); 
    valeur_PWM2 = constrain(Kp2_HIGH*erreur2, 50, 200);
    //Serial.println(erreur1);
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


    valeur_PWM1 = constrain(Kp1_HIGH*erreur1, 48, 200); 
    valeur_PWM2 = constrain(Kp2_HIGH*erreur2, 50, 200);

    //Serial.println(erreur1);

  };

  while (encoderTicks1 < ticks_a_faire1){
    digitalWrite(DIR1, HIGH);
    analogWrite(PWM1, valeur_PWM1);
    digitalWrite(DIR2, HIGH);
    analogWrite(PWM2, 0);
  };

  while (encoderTicks2 < ticks_a_faire2){
    digitalWrite(DIR1, HIGH);
    analogWrite(PWM1, 0);
    digitalWrite(DIR2, HIGH);
    analogWrite(PWM2, valeur_PWM2);
  }

    digitalWrite(DIR1, HIGH);
    analogWrite(PWM1, 0);
    digitalWrite(DIR2, HIGH);
    analogWrite(PWM2, 0);

}

void goBackward(float distance){

  prevTime = millis();

  float valeur_PWM1 = 54;
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


    valeur_PWM1 = constrain(Kp1_LOW*erreur1, 54, 200); 
    valeur_PWM2 = constrain(Kp2_LOW*erreur2, 50, 200);
    Serial.println(ticks_a_faire2-encoderTicks2);
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


    valeur_PWM1 = constrain(Kp1_LOW*erreur1, 54, 200); 
    valeur_PWM2 = constrain(Kp2_LOW*erreur2, 50, 200);

    Serial.println(ticks_a_faire2-encoderTicks2);

  };

  while (encoderTicks1 < ticks_a_faire1){
    digitalWrite(DIR1, LOW);
    analogWrite(PWM1, valeur_PWM1);
    digitalWrite(DIR2, LOW);
    analogWrite(PWM2, 0);
  }

  while (encoderTicks2 < ticks_a_faire2){
    digitalWrite(DIR1, LOW);
    analogWrite(PWM1, 0);
    digitalWrite(DIR2, LOW);
    analogWrite(PWM2, valeur_PWM2);
    Serial.println(ticks_a_faire2-encoderTicks2);
  }

    digitalWrite(DIR1, HIGH);
    analogWrite(PWM1, 0);
    digitalWrite(DIR2, HIGH);
    analogWrite(PWM2, 0);

}

void goRight(float angle){

  float distance = angle/180*pi*Rayon_base;

  prevTime = millis();

  float valeur_PWM1 = 48;
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


    valeur_PWM1 = constrain(Kp1_HIGH*erreur1, 48, 200); 
    valeur_PWM2 = constrain(Kp2_LOW*erreur2, 50, 200);
    //Serial.println(erreur1);
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


    valeur_PWM1 = constrain(Kp1_HIGH*erreur1, 48, 200); 
    valeur_PWM2 = constrain(Kp2_LOW*erreur2, 50, 200);

    //Serial.println(erreur1-erreur2);

  };

  while (encoderTicks1 < ticks_a_faire1) {
    digitalWrite(DIR1, HIGH);
    analogWrite(PWM1, valeur_PWM1);
    digitalWrite(DIR2, HIGH);
    analogWrite(PWM2, 0);
  };

  while (encoderTicks2 < ticks_a_faire2) {
    digitalWrite(DIR1, HIGH);
    analogWrite(PWM1, 0);
    digitalWrite(DIR2, LOW);
    analogWrite(PWM2, valeur_PWM2);
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


    valeur_PWM1 = constrain(Kp1_LOW*erreur1, 48, 200); 
    valeur_PWM2 = constrain(Kp2_HIGH*erreur2, 50, 200);
    //Serial.println(erreur1-2);
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


    valeur_PWM1 = constrain(Kp1_LOW*erreur1, 48, 200); 
    valeur_PWM2 = constrain(Kp2_HIGH*erreur2, 50, 200);

    //Serial.println(erreur1-erreur2);

  };

  while (encoderTicks1 < ticks_a_faire1){
    digitalWrite(DIR1, LOW);
    analogWrite(PWM1, valeur_PWM1);
    digitalWrite(DIR2, HIGH);
    analogWrite(PWM2, 0);
  };

  while (encoderTicks2 < ticks_a_faire2){
    digitalWrite(DIR1, HIGH);
    analogWrite(PWM1, 0);
    digitalWrite(DIR2, HIGH);
    analogWrite(PWM2, valeur_PWM2);
  };

    digitalWrite(DIR1, HIGH);
    analogWrite(PWM1, 0);
    digitalWrite(DIR2, HIGH);
    analogWrite(PWM2, 0);


}

void pivotLeft(float angle){
  float distance = angle/180*pi*Diameter_base;

  prevTime = millis();

  float valeur_PWM2 = 50;
  encoderTicks2 = 0;
  float prevE2 = 0;

  float ticks_a_faire2 = distance/(2*pi*Rayon) * CPR2_HIGH ;

  // Phase d'accélération

  while (encoderTicks2 < 0.2*CPR2_HIGH){

    digitalWrite(DIR1, LOW);
    analogWrite(PWM1, 0);
    digitalWrite(DIR2, HIGH);
    analogWrite(PWM2, valeur_PWM2);

    delay(10);
    
    currentTime = millis();

    float delta_time = (currentTime - prevTime)/1000.0; // les temps sont en millisecondes

    float ref = constrain((pente/2)*(delta_time*delta_time), 0, 2.64);
    float reel2 = (encoderTicks2*2*pi*Rayon)/(CPR2_HIGH);
    float erreur2 = (ref - reel2);

    valeur_PWM2 = constrain(Kp2_HIGH*erreur2, 50, 200);
  };

  prevTime = millis();
  prevE2 = encoderTicks2;

  while (encoderTicks2 < ticks_a_faire2){
  
    digitalWrite(DIR1, LOW);
    analogWrite(PWM1, 0);
    digitalWrite(DIR2, HIGH);
    analogWrite(PWM2, valeur_PWM2);

    delay(10);
    
    currentTime = millis();

    float delta_time = (currentTime - prevTime)/1000.0; // les temps sont en millisecondes

    float ref = vitesse*delta_time;
    float reel2 = ((encoderTicks2-prevE2)*2*pi*Rayon)/(CPR2_HIGH);
    float erreur2 = (ref - reel2);

    valeur_PWM2 = constrain(Kp2_HIGH*erreur2, 50, 200);


  };

    digitalWrite(DIR1, HIGH);
    analogWrite(PWM1, 0);
    digitalWrite(DIR2, HIGH);
    analogWrite(PWM2, 0);


}

void pivotBackLeft(float angle){
  float distance = angle/180*pi*Rayon_base;

  prevTime = millis();

  float valeur_PWM1 = 50;
  encoderTicks1 = 0;
  float prevE1 = 0;

  float ticks_a_faire1 = distance/(2*pi*Rayon) * CPR1_HIGH ;

  // Phase d'accélération

  while (encoderTicks2 < 0.2*CPR1_HIGH){

    digitalWrite(DIR1, LOW);
    analogWrite(PWM1, valeur_PWM1);
    digitalWrite(DIR2, HIGH);
    analogWrite(PWM2, 0);

    delay(10);
    
    currentTime = millis();

    float delta_time = (currentTime - prevTime)/1000.0; // les temps sont en millisecondes

    float ref = constrain((pente/2)*(delta_time*delta_time), 0, 2.64);
    float reel1 = (encoderTicks1*2*pi*Rayon)/(CPR2_HIGH);
    float erreur1 = (ref - reel1);

    valeur_PWM1 = constrain(Kp1_HIGH*erreur1, 50, 200);
  };

  prevTime = millis();
  prevE1 = encoderTicks1;

  while (encoderTicks1 < ticks_a_faire1){
  
    digitalWrite(DIR1, LOW);
    analogWrite(PWM1, valeur_PWM1);
    digitalWrite(DIR2, HIGH);
    analogWrite(PWM2, 0);

    delay(10);
    
    currentTime = millis();

    float delta_time = (currentTime - prevTime)/1000.0; // les temps sont en millisecondes

    float ref = vitesse*delta_time;
    float reel1 = ((encoderTicks1-prevE1)*2*pi*Rayon)/(CPR1_HIGH);
    float erreur1 = (ref - reel1);

    valeur_PWM1 = constrain(Kp1_HIGH*erreur1, 50, 200);


  };

    digitalWrite(DIR1, HIGH);
    analogWrite(PWM1, 0);
    digitalWrite(DIR2, HIGH);
    analogWrite(PWM2, 0);


}

void pivotRight(float angle){
  float distance = angle/180*pi*Diameter_base;

  prevTime = millis();

  float valeur_PWM1 = 50;
  encoderTicks1 = 0;
  float prevE1 = 0;

  float ticks_a_faire1 = distance/(2*pi*Rayon) * CPR1_HIGH ;

  // Phase d'accélération

  while (encoderTicks2 < 0.2*CPR1_HIGH){

    digitalWrite(DIR1, HIGH);
    analogWrite(PWM1, valeur_PWM1);
    digitalWrite(DIR2, HIGH);
    analogWrite(PWM2, 0);

    delay(10);
    
    currentTime = millis();

    float delta_time = (currentTime - prevTime)/1000.0; // les temps sont en millisecondes

    float ref = constrain((pente/2)*(delta_time*delta_time), 0, 2.64);
    float reel1 = (encoderTicks1*2*pi*Rayon)/(CPR2_HIGH);
    float erreur1 = (ref - reel1);

    valeur_PWM1 = constrain(Kp1_HIGH*erreur1, 50, 200);
  };

  prevTime = millis();
  prevE1 = encoderTicks1;

  while (encoderTicks1 < ticks_a_faire1){
  
    digitalWrite(DIR1, HIGH);
    analogWrite(PWM1, valeur_PWM1);
    digitalWrite(DIR2, HIGH);
    analogWrite(PWM2, 0);

    delay(10);
    
    currentTime = millis();

    float delta_time = (currentTime - prevTime)/1000.0; // les temps sont en millisecondes

    float ref = vitesse*delta_time;
    float reel1 = ((encoderTicks1-prevE1)*2*pi*Rayon)/(CPR1_HIGH);
    float erreur1 = (ref - reel1);

    valeur_PWM1 = constrain(Kp1_HIGH*erreur1, 50, 200);


  };

    digitalWrite(DIR1, HIGH);
    analogWrite(PWM1, 0);
    digitalWrite(DIR2, HIGH);
    analogWrite(PWM2, 0);


}
//fonctions qui bougent le marqueur
void BicUp(){
  myservo.attach(A5);
  myservo.write(60);
  delay(1000);
  myservo.detach();
}

void BicDown(){
  myservo.attach(A5);
  myservo.write(0);
  delay(1000); 
  myservo.detach();
}

//fonctions dessin des symboles
void croix(){
  BicUp();
  delay(1000);
  goRight(45);
  BicDown();
  delay(100);
  goForward(sqrt(800));
  BicUp();
  goRight(45);
  delay(100);
  goBackward(20);
  delay(100);
  goRight(45);
  delay(100);
  BicDown();
  goForward(sqrt(800));
  delay(100);
  BicUp();
  goRight(45);
  delay(1000);
};

void rond(){

  goForward(10);
  delay(1000);
  BicDown();
  pivotRight(380);
  pivotBackLeft(20);
  BicUp();
  delay(100);
  goBackward(10);
  delay(100);
  goRight(90);
  delay(100);
  goForward(20);
  delay(100);
  goRight(90);


}
// fonctions allés
void case1(){
  goLeft(48.81);
  goForward(53.15);
  goRight(48.81);
}
void case2(){
  goLeft(31.61);
  goForward(76.32);
  goRight(31.61);
  delay(100);
}
void case3(){
  goLeft(22.83);
  goForward(103.08-1);
  goRight(22.83);
}
void case4(){
  goLeft(15.95);
  goForward(36.4);
  goRight(15.95);
}
void case5(){
  goForward(65);
  goLeft(90);
  goForward(10+5);
  goRight(90);
}
void case6(){
  goForward(95-3);
  goLeft(90);
  goForward(10);
  goRight(90);
}
void case7(){
  goForward(35);
  goRight(90);
  goForward(20);
  goLeft(90);
}
void case8(){
  goForward(65-3);
  goRight(90);
  goForward(20);
  goLeft(90);
}
void case9(){
  goForward(95);
  goRight(90);
  goForward(20-2);
  goLeft(90);
}

// fonctions retours
void retour1(){
  goLeft(40.28);
  goForward(77.34);
  goLeft(49.72+5);
  ajuster();
}
void retour2(){
  goLeft(29.33);
  goForward(102.08);
  goLeft(60.67+5);
  ajuster();
}
void retour3(){
  goLeft(22.79);
  goForward(129.08);
  goLeft(63.21+5);
  ajuster();
}
void retour4(){
  goLeft(18.73);
  goForward(62.3);
  goLeft(71.27+5);
  ajuster();
}
void retour5(){
  goLeft(12.66);
  goForward(91.22);
  goLeft(77.34+5);
  ajuster();
}
void retour6(){
  goLeft(9.54);
  goForward(120.67);
  goLeft(80.46+5);
  ajuster();
}
void retour7(){
  goRight(9.63);
  goForward(59.84);
  goLeft(99.63+5);
  ajuster();
}
void retour8(){
  goRight(6.41);
  goForward(89.56);
  goLeft(96.41+5);
  ajuster();
  
}
void retour9(){
  goRight(4.8);
  goForward(119.42);
  goLeft(94.8+5);
  ajuster();
}

void aligner(){ //fonction qui aligne le robot perpendiculairment à une ligne noire
  int sensorstatus1 = digitalRead(IRSensor1);
  int sensorstatus2 = digitalRead(IRSensor2);
  int a = 5;
  int st1 = 0;
  int st2 = 0;

  while (a != 0){
    sensorstatus1 = digitalRead(IRSensor1);
    sensorstatus2 = digitalRead(IRSensor2);
    if ((sensorstatus1 == LOW) && (sensorstatus2 == LOW) && (st1 == 0) && (st2 == 0)){
      a = 12;
      digitalWrite(DIR1, HIGH);
      analogWrite(PWM1, 50);
      digitalWrite(DIR2, HIGH);
      analogWrite(PWM2, 50);
    }
    else if ((sensorstatus1 == HIGH) && (sensorstatus2 == LOW) && (st1 == 0)){
      a = 1;
      digitalWrite(DIR1, HIGH);
      analogWrite(PWM1, 0);
      digitalWrite(DIR2, HIGH);
      analogWrite(PWM2, 50);
      st1 = 1;
    }
    else if ((sensorstatus1 == LOW) && (sensorstatus2 == HIGH) && (st2 == 0)){
      a = 2;
      digitalWrite(DIR1, HIGH);
      analogWrite(PWM1, 50);
      digitalWrite(DIR2, HIGH);
      analogWrite(PWM2, 0);
      st2 = 1;
    }
    else if ((sensorstatus1 == HIGH && sensorstatus2 == HIGH) || (st1!=0 && st2!=0) ){
      a = 0;
      digitalWrite(DIR1, HIGH);
      analogWrite(PWM1, 0);
      digitalWrite(DIR2, HIGH);
      analogWrite(PWM2, 0);
    };
  }
}

void ajuster(){ // fonction qui remet l'erreur de position à zero apres chaque coup
  aligner();
  delay(1000);
  goBackward(59+1);
  goLeft(90-5);
  delay(1000);
  aligner();
  goForward(3);
}

void setup() {

  Serial.begin(9600);

  bluetooth.begin(9600);

  bool isCarteA = false;

  radio.begin();
  radio.setChannel(76); // A changer
  radio.setPALevel(RF24_PA_MIN);
  //radio.setRetries(5, 15);
  radio.setDataRate(RF24_250KBPS);
  radio.startListening();
  //radio.powerDown();

  if (isCarteA) {
    radio.openWritingPipe(addresses[0]);     // Carte A écrit sur CANAL1
    radio.openReadingPipe(1, addresses[1]);  // Carte A écoute sur CANAL2
    Serial.println("Je suis la Carte A");
  } else {
    radio.openWritingPipe(addresses[1]);     // Carte B écrit sur CANAL2
    radio.openReadingPipe(1, addresses[0]);  // Carte B écoute sur CANAL1
    Serial.println("Je suis la Carte B");
  };

  //radio.openReadingPipe(1, adresselire); //
  //radio.openWritingPipe(adresseecrire);
  
  int choix = 2;

  if (choix == 1) {
    ROBOT_IA = 1; ADVERSAIRE = 2;
    monSymbole = 'X'; estMonTour = true;
    Serial.println(F("Robot = Player 1 (X). Je commence !"));
  } else {
    ROBOT_IA = 2; ADVERSAIRE = 1;
    monSymbole = 'O'; estMonTour = false;
    Serial.println(F("Robot = Player 2 (O). L'adversaire commence !"));
  }
  Serial.println(F("------------------------------"));


  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);

  pinMode(encoderPin1, INPUT_PULLUP);
  pinMode(encoderPin2, INPUT_PULLUP);

  pinMode(IRSensor1,INPUT);
  pinMode(IRSensor2,INPUT);

  attachInterrupt(digitalPinToInterrupt(encoderPin1), countTicks1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPin2), countTicks2, RISING);

}

void loop() {
  
  if (estMonTour) {
    Serial.println(F("L'IA reflechit..."));
    int coupIA = trouverMeilleurCoup();
    if (coupIA != -1) {
      plateau[coupIA] = ROBOT_IA;
      Serial.print(F("ACTION : Aller en case ")); Serial.println(coupIA);
      Serial.print(F("ACTION : Tracer un ")); Serial.println(monSymbole);
      afficherPlateau();

    char coupIA_c = '0' + coupIA;
    bluetooth.write(coupIA_c);

      if (coupIA == 1){
        case1();
        if (monSymbole == 'X'){
          croix();
        }
        else{
          rond();
        }
        retour1();
      }

      if (coupIA == 2){
        case2();
        if (monSymbole == 'X'){
          croix();
        }
        else{
          rond();
        }
        retour2();
      }

      if (coupIA == 3){
        case3();
        if (monSymbole == 'X'){
          croix();
        }
        else{
          rond();
        }
        retour3();
      }

      if (coupIA == 4){
        case4();
        if (monSymbole == 'X'){
          croix();
        }
        else{
          rond();
        }
        retour4();
      }

      if (coupIA == 5){
        case5();
        if (monSymbole == 'X'){
          croix();
        }
        else{
          rond();
        }
        retour5();
      }

      if (coupIA == 6){
        case6();
        if (monSymbole == 'X'){
          croix();
        }
        else{
          rond();
        }
        retour6();
      }

      if (coupIA == 7){
        case7();
        if (monSymbole == 'X'){
          croix();
        }
        else{
          rond();
        }
        retour7();
      }

      if (coupIA == 8){
        case8();
        if (monSymbole == 'X'){
          croix();
        }
        else{
          rond();
        }
        retour8();
      }

      if (coupIA == 9){
        case9();
        if (monSymbole == 'X'){
          croix();
        }
        else{
          rond();
        }
        retour9();
      }
      
        unsigned long tempsActuel = millis(); 
        tempsPrecedent = millis();

        while (tempsActuel - tempsPrecedent <= 4000) {
          radio.stopListening();
          char texteAEnvoyer[32] = "0" + coupIA;
          bool succes = radio.write(&texteAEnvoyer, sizeof(texteAEnvoyer));
          if (succes) {
            Serial.println("<< Message envoyé !");
          } else {
            Serial.println("<< Échec de l'envoi...");
          }
          radio.startListening();
          tempsPrecedent = tempsActuel;
        };

      if (verifierGagnant(plateau) != VIDE) finDePartie();
      estMonTour = false;

    }
  } 
  else {

    Serial.println(F("En attente du coup de l'adversaire (1-9)..."));
    int a_recu = 0;
    int coupAdv = 0;
    while (a_recu == 0){
      if (Serial.available()){
        delay(100);
        coupAdv = Serial.parseInt();
        coupAdv = 10 - coupAdv; 
        while(Serial.available() > 0) Serial.read(); 
        a_recu += 1;
      }
      else if(bluetooth.available())
      {
        char mess = bluetooth.read();   
        if (mess >= '0' && mess <= '9'){
          coupAdv = mess-'0';
          coupAdv = 10 - coupAdv;
          Serial.print(coupAdv);
          a_recu += 1;
        }
      }
      else if (radio.available()){
        char message[32];
        radio.read(&message, sizeof(message));
        if (message >= '0' && message <= '9'){
          coupAdv = message-'0';
          coupAdv = 10 - coupAdv;
          Serial.print(coupAdv);
          a_recu += 1;
        }; 
      };
    };

    if (coupAdv > 0 && coupAdv <= 9 && plateau[coupAdv] == VIDE) {
      plateau[coupAdv] = ADVERSAIRE;
      afficherPlateau();
      if (verifierGagnant(plateau) != VIDE) finDePartie();
      estMonTour = true;
      } 
    else {
      Serial.println(F("Case invalide, reessayez."));
      }

  }


  /*BicUp();
  case1();  
  rond();
  retour1();
  case2();  
  croix();
  retour2();
  case3();  
  rond();
  retour3();
  case4();  
  croix();
  retour4();
  case5();  
  rond();
  retour5();
  case6();  
  croix();
  retour6();
  case7();  
  rond();
  retour7();
  case8();  
  croix();
  retour8();
  case9();  
  rond();
  retour9();
  delay(1000);*/

  /* Alternative au test standard
  BicUp();
  case3();  
  rond();
  retour3();
  case9();  
  croix();
  retour9();
  case7();  
  rond();
  retour7();
  case1();  
  croix();
  retour1();
  */

  
}

void countTicks1(){
  encoderTicks1++;
}

void countTicks2(){
  encoderTicks2++;
}

void afficherPlateau() {
  for (int i = 1; i <= 9; i++) {
    if (plateau[i] == VIDE) Serial.print(F(". "));
    else if (plateau[i] == 1) Serial.print(F("X "));
    else Serial.print(F("O "));
    if (i % 3 == 0) Serial.println();
  }
  Serial.println(F("-----"));
}

void finDePartie() {
  int8_t r = verifierGagnant(plateau);
  if (r == -1) Serial.println(F("MATCH NUL !"));
  else Serial.println(r == ROBOT_IA ? F("IA GAGNE !") : F("L'ADVERSAIRE A GAGNE !"));
  while (1); 
} 