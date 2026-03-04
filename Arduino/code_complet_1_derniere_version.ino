// Librairies
#include <Servo.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <SoftwareSerial.h>

// Constantes
const float pi = 3.14159;

// Données du proto (tt est en cm)
float Rayon = 4.2;
float Rayon_base = 19.7/2;

// Branchements moteurs
int DIR1 = 8;
int DIR2 = 7;
int PWM1 = 9;
int PWM2 = 6;

// Setup encodeurs
const int encoderPin1 = 2;
const int encoderPin2 = 3;
volatile long countTicks1 = 0;
volatile long countTicks2 = 0;
float CPR1 = 350;
float CPR2 = 350;

// Régulation : quelle vitesse ? intervalle de temps choisi pour la régulation ?
float vitesse = 20.0;
float Kp = 1;

// Setup servo
Servo myservo;
int pos = 0;

// Croix ou rond
char monSymbole[] = "Croix";
int joue_case; // Pas oublier de vérifier la convention choisie pour numéroter les cases


// Configuration radio
RF24 radio(9, 10); // CE, CSN
const byte address[6] = "00001"; // Adresse 1
const byte address2[6] = "00002"; // Adresse 2

// Configuration BT
SoftwareSerial bluetooth(6, 7);


// Configuration algo OXO
const int8_t VIDE = 0;
int8_t ROBOT_IA;    
int8_t ADVERSAIRE;   
bool estMonTour;

int8_t plateau[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

// Les lignes gagnantes [cite: 6]
const int8_t WIN_LINES[8][3] = {
  {0,1,2}, {3,4,5}, {6,7,8}, 
  {0,3,6}, {1,4,7}, {2,5,8}, 
  {0,4,8}, {2,4,6}           
};

// Priorité stratégique (Centre, Coins, Bords) [cite: 8]
const int8_t ORDRE_TEST[9] = {4, 0, 2, 6, 8, 1, 3, 5, 7}; 


// Fonctions déplacement + tracé

void goForward(float distance){

  int valeur_PWM1 = 100;
  int valeur_PWM2 = 100;
  countTicks1 = 0;
  countTicks2 = 0;

  float ticks_a_faire1 = distance/(2*pi*Rayon) * CPR1 ;
  float ticks_a_faire2 = distance/(2*pi*Rayon) * CPR2 ;

  int i = 0;

  while ((countTicks1 < ticks_a_faire1) && (countTicks2 < ticks_a_faire2)){
    
    i += 1;

    digitalWrite(DIR1, HIGH);
    analogWrite(PWM1, valeur_PWM1);
    digitalWrite(DIR2, HIGH);
    analogWrite(PWM2, valeur_PWM2);

    delay(100);

    float erreur1 = ref_pos[i] - countTicks1;
    float erreur2 = ref_pos[i] - countTicks2;

    valeur_PWM1 = constrain(valeur_PWM1 + Kp*erreur1, 0, 200); // PWM minimal en borne inf plutôt
    valeur_PWM2 = constrain(valeur_PWM2 + Kp*erreur2, 0, 200);

  }
};

void goBackward(float distance){

  int ref_pos[210];

  for (float i = 0.1; i < 20; i += 0.1){
    ref_pos[i*10] = i*vitesse;
  };

  int valeur_PWM1 = 100;
  int valeur_PWM2 = 100;
  countTicks1 = 0;
  countTicks2 = 0;

  float ticks_a_faire1 = distance/(2*pi*Rayon) * CPR1 ;
  float ticks_a_faire2 = distance/(2*pi*Rayon) * CPR2 ;

  int i = 0;

  while ((countTicks1 < ticks_a_faire1) && (countTicks2 < ticks_a_faire2)){
    
    i += 1;

    digitalWrite(DIR1, LOW);
    analogWrite(PWM1, valeur_PWM1);
    digitalWrite(DIR2, LOW);
    analogWrite(PWM2, valeur_PWM2);

    delay(100);

    float erreur1 = ref_pos[i] - countTicks1;
    float erreur2 = ref_pos[i] - countTicks2;

    valeur_PWM1 = constrain(valeur_PWM1 + Kp*erreur1, 0, 200);
    valeur_PWM2 = constrain(valeur_PWM2 + Kp*erreur2, 0, 200);

  };
};

void goLeft(float angle){

  float distance1 = angle/180*pi * Rayon_base;

  float ref_pos1[210];

  for (float i = 0.1; i < 20; i += 0.1){
    ref_pos1[i*10] = i*vitesse;
  };

  int valeur_PWM1 = 100;
  countTicks1 = 0;
  float ticks_a_faire1 = distance1/(2*pi*Rayon) * CPR1;

  int i = 0;

  while (countTicks1 < ticks_a_faire1){
    
    i += 1;

    digitalWrite(DIR1, HIGH);
    analogWrite(PWM1, valeur_PWM1);
    digitalWrite(DIR2, HIGH);
    analogWrite(PWM2, 0);
    delay(100);

    float erreur1 = ref_pos[i] - countTicks1;

    valeur_PWM1 = constrain(valeur_PWM1 + Kp*erreur1, 0, 200);
  };
};

void goRight(float angle){

  float distance2 = angle/180*pi*Rayon_base;

  int ref_pos[210];

  for (float i = 0.1; i < 20; i += 0.1){
    ref_pos[i*10] = i*vitesse;
  };

  int valeur_PWM2 = 100;
  countTicks2 = 0;

  float ticks_a_faire2 = distance2/(2*pi*Rayon) * CPR2 ;

  int i = 0;

  while (countTicks2 < ticks_a_faire2){
    
    i += 1;


    digitalWrite(DIR1, HIGH);
    analogWrite(PWM1, 0);
    digitalWrite(DIR2, HIGH);
    analogWrite(PWM2, valeur_PWM2);

    delay(100);

    float erreur2 = ref_pos[i] - countTicks2;

    valeur_PWM2 = constrain(valeur_PWM2 + Kp*erreur2, 0, 200);

  };
};

void pivotRight(float angle){

  float distance2 = angle/180*pi*Rayon_base/2;
  float distance1 = angle/180*pi*Rayon_base/2;

  int ref_pos[210];

  for (float i = 0.1; i < 20; i += 0.1 ){
    ref_pos[i*10] = i*vitesse;
  };

  int valeur_PWM1 = 100;
  int valeur_PWM2 = 100;
  countTicks1 = 0;
  countTicks2 = 0;

  float ticks_a_faire1 = distance1/(2*pi*Rayon) * CPR1 ;
  float ticks_a_faire2 = distance2/(2*pi*Rayon) * CPR2 ;

  int i = 0;

  while ((countTicks1 < ticks_a_faire1) && (countTicks2 < ticks_a_faire2)){
    
    i += 1;

    digitalWrite(DIR1, LOW);
    analogWrite(PWM1, valeur_PWM1);
    digitalWrite(DIR2, HIGH);
    analogWrite(PWM2, valeur_PWM2);

    delay(100);

    float erreur1 = ref_pos[i] - countTicks1;
    float erreur2 = ref_pos[i] - countTicks2;

    valeur_PWM1 = constrain(valeur_PWM1 + Kp*erreur1, 0, 200);
    valeur_PWM2 = constrain(valeur_PWM2 + Kp*erreur2, 0, 200);

  }

};

void pivotLeft(float angle){

  float distance2 = angle/180*pi*Rayon_base/2;
  float distance1 = angle/180*pi*Rayon_base/2;

  int ref_pos[210];

  for (float i = 0.1; i < 20; i += 0.1 ){
    ref_pos[i*10] = i*vitesse;
  };

  int valeur_PWM1 = 100;
  int valeur_PWM2 = 100;
  countTicks1 = 0;
  countTicks2 = 0;

  float ticks_a_faire1 = distance1/(2*pi*Rayon) * CPR1 ;
  float ticks_a_faire2 = distance2/(2*pi*Rayon) * CPR2 ;

  int i = 0;

  while ((countTicks1 < ticks_a_faire1) && (countTicks2 < ticks_a_faire2)){
    
    i += 1;

    digitalWrite(DIR1, HIGH);
    analogWrite(PWM1, valeur_PWM1);
    digitalWrite(DIR2, LOW);
    analogWrite(PWM2, valeur_PWM2);

    delay(100);

    float erreur1 = ref_pos[i] - countTicks1;
    float erreur2 = ref_pos[i] - countTicks2;

    valeur_PWM1 = constrain(valeur_PWM1 + Kp*erreur1, 0, 200);
    valeur_PWM2 = constrain(valeur_PWM2 + Kp*erreur2, 0, 200);

  };

};


void gotocasedroite(int i){
  pivotRight(90);
  goForward(10+30);
  pivotLeft(90);
  goForward(10 + (i-1)*30 + 5);
};

void gotocasemilieu(int i){
  pivotRight(90);
  goForward(10);
  pivotLeft(90);
  goForward(10 + (i-1)*30 + 5);

};

void gotocasegauche(int i){
   pivotLeft(90);
   goForward(15+5);
   pivotRight(90);
   goForward(10 + (i-1)*30 + 5);
};

void returncasegauche(int i){
  goForward(5 + (i-1)*30 + 10);
  pivotLeft(90);
  goForward(30+10);
  pivotLeft(90);
};

void returncasemilieu(int i){
  goForward(5 + (i-1)*30 + 10);
  pivotLeft(90);
  goForward(10);
  pivotLeft(90);
};

void returncasedroite(int i){
  goForward(5 + (i-1)*30 + 10);
  pivotRight(90);
  goForward(20);
  pivotRight(90);
};


// Fonctions servo
int bicDown(){
  for (pos > 100; pos < 200; pos+=1){
  myservo.write(pos);
};

int bicUp(){
  for (pos = 0; pos <= 100; pos += 1) { // goes from 0 degrees to 180 degrees
  // in steps of 1 degree
  myservo.write(pos); // tell servo to go to position in variable 'pos'
  delay(30);
  } // waits 15ms for the servo to reach the position
  for (pos > 100; pos < 200; pos+=1){
  myservo.write(pos);
};


// Fonctions tracé
void Rond(){

  goForward(10);
  bicDown();
  goLeft(360);
  bic_Up();
  goLeft(180);
  goForward(10);
};

void Croix(){
  
  pivotLeft(45);
  bicDown();
  goForward(28.28);
  pivotRight(135);
  goForward(20);
  pivotRight(135);
  goForward(28.28);
  bicUp();
  pivotLeft(45);
  
};


// Fonctions OXO
int8_t verifierGagnant(int8_t b[9]) {
  for (int i = 0; i < 8; i++) {
    if (b[WIN_LINES[i][0]] != VIDE &&
        b[WIN_LINES[i][0]] == b[WIN_LINES[i][1]] &&
        b[WIN_LINES[i][0]] == b[WIN_LINES[i][2]]) {
      return b[WIN_LINES[i][0]];
    }
  }
  bool plein = true;
  for (int i = 0; i < 9; i++) if (b[i] == VIDE) plein = false;
  if (plein) return -1; // Match nul [cite: 7]
  return VIDE;
};

int alphabeta(int8_t b[9], bool maximizing, int alpha, int beta) {
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
};

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
};

void afficherPlateau() {
  for (int i = 0; i < 9; i++) {
    if (plateau[i] == VIDE) Serial.print(F(". "));
    else if (plateau[i] == 1) Serial.print(F("X "));
    else Serial.print(F("O "));
    if (i % 3 == 2) Serial.println();
  }
  Serial.println(F("-----"));
};

void finDePartie() {
  int8_t r = verifierGagnant(plateau);
  if (r == -1) Serial.println(F("MATCH NUL !"));
  else Serial.println(r == ROBOT_IA ? F("IA GAGNE !") : F("L'ADVERSAIRE A GAGNE !"));
  while (1); 
};



void setup() {
  // Configuration moteurs
  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);

  // Configruation encodeurs
  pinMode(encoderPin1, INPUT_PULLUP);
  pinMode(encoderPin2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPin1), f_CountTicks1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPin2), f_CountTicks2, RISING);

  // Configuration servo
  myservo.attach(3); // C'est le pin ? est-ce que c'est le bon ?

  // Configuration radio  
  radio.begin();
  radio.openReadingPipe(0, address); // 00001
  radio.openWritingPipe(address2); // 00002

  // Configuration BT
  bluetooth.begin(9600);

  // Configuration algo OXO
  Serial.begin(9600);
  while (!Serial); 

  if (monSymbole == "Croix") {
    ROBOT_IA = 1; ADVERSAIRE = 2;
    monSymbole = 'X'; estMonTour = true;
    Serial.println(F("Robot = Player 1 (X). Je commence !"));
  } else {
    ROBOT_IA = 2; ADVERSAIRE = 1;
    monSymbole = 'O'; estMonTour = false;
    Serial.println(F("Robot = Player 2 (O). L'adversaire commence !"));
  }
  Serial.println(F("------------------------------"));
  }
};

void loop() {
  


  // if reçoit une une info de comm() -> dernier_coup_adversaire = ;
  // joue_case = algo_oxo(dernier_coup_adversaire)
  // if joue_case = -1 c fini;


    // Algo OXO
    if (estMonTour) {
    int joue_case = trouverMeilleurCoup();
      if (joue_case != -1) {
        plateau[joue_case] = ROBOT_IA;
        Serial.print(F("ACTION : Aller en case ")); Serial.println(joue_case);
        Serial.print(F("ACTION : Tracer un ")); Serial.println(monSymbole);
        afficherPlateau();
        if (verifierGagnant(plateau) != VIDE) finDePartie();
        estMonTour = false;
        }

          if (joue_case == 0){
            gotocasegauche(1);
            if (monSymbole == "Croix"){
              Croix();
            }
            else Rond();
            returncasegauche(1);
      };

      else if (joue_case == 1){
        gotocasegauche(2);
        if (monSymbole == "Croix"){
          Croix();
        }
        else Rond();
        returncasegauche(2);
      };

      else if (joue_case == 2){
        gotocasegauche(3);
        if (monSymbole == "Croix"){
          Croix();
        }
        else Rond();
        returncasegauche(3);
      };

      else if (joue_case == 3){
        gotocasemilieu(1);
        if (monSymbole == "Croix"){
          Croix();
        }
        else Rond();
        returncasemilieu(1);
      };

      else if (joue_case == 4){
        gotocasemilieu(2);
        if (monSymbole == "Croix"){
          Croix();
        }
        else Rond();
        returncasemilieu(2);
      };
      
      else if (joue_case == 5){
        gotocasemilieu(3);
        if (monSymbole == "Croix"){
          Croix();
        }
        else Rond();
        returncasemilieu(3);
        
      };
      
      else if (joue_case == 6){
        gotocasedroite(1);
        if (monSymbole == "Croix"){
          Croix();
        }
        else Rond();
        returncasedroite(1);
      };

      else if (joue_case == 7){
        gotocasedroite(2);
        if (monSymbole == "Croix"){
          Croix();
        }
        else Rond();
        returncasedroite(2);
      };

      else if (joue_case == 8){
        gotocasedroite(3);
        if (monSymbole == "Croix"){
          Croix();
        }
        else Rond();
        returncasedroite(3);
      };


      // Transmettre coup

      radio.stopListening();
      delay(2000);
      const char text_transmis[] = static_cast<char>(joue_case); // message
      radio.write(&text_transmis, sizeof(text_transmis));

      bluetooth.write(text_transmis);


      };



    else {
      Serial.println(F("En attente du coup de l'adversaire (0-8)..."));
      
      // Recevoir comm (rajouter une condition pour que la comm BT puisse intervenir au cas où)

      radio.startListening();
      while (!radio.available()){
      };
      radio.startListening();
      char text[32] = {0};
      radio.read(&text, sizeof(text));
      delay(2000);

      int coup_adv = static_cast<int>(text);

      if (coupAdv >= 0 && coupAdv < 9 && plateau[coupAdv] == VIDE) {
        plateau[coupAdv] = ADVERSAIRE;
        afficherPlateau();
        if (verifierGagnant(plateau) != VIDE) finDePartie();
        estMonTour = true;
      } 
      else {
        Serial.println(F("Case invalide, reessayez."));
      }
      };


};


void f_CountTicks1(){
  countTicks1++;
};

void f_CountTicks2(){
  countTicks2++;
};
