// --- CONFIGURATION ---
const int8_t VIDE = 0;
int8_t ROBOT_IA;    
int8_t ADVERSAIRE; 
char monSymbole;    // 'X' ou 'O'
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

// --- FONCTIONS LOGIQUES ---

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
}

// Algorithme Alpha-Beta avec Backtracking (pas de .copy() pour la RAM) 
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

// --- ARDUINO SETUP & LOOP ---

void setup() {
  Serial.begin(9600);
  while (!Serial); 
  
  Serial.println(F("--- CONFIGURATION DU ROBOT ---"));
  Serial.println(F("Le robot est-il Player 1 ou 2 ? (Tapez 1 ou 2)"));
  
  while (Serial.available() == 0);
  int choix = Serial.parseInt();
  while(Serial.available() > 0) Serial.read(); 

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
      if (verifierGagnant(plateau) != VIDE) finDePartie();
      estMonTour = false;
    }
  } else {
    Serial.println(F("En attente du coup de l'adversaire (0-8)..."));
    while (Serial.available() == 0);
    int coupAdv = Serial.parseInt();
    while(Serial.available() > 0) Serial.read(); 

    if (coupAdv >= 0 && coupAdv < 9 && plateau[coupAdv] == VIDE) {
      plateau[coupAdv] = ADVERSAIRE;
      afficherPlateau();
      if (verifierGagnant(plateau) != VIDE) finDePartie();
      estMonTour = true;
    } else {
      Serial.println(F("Case invalide, reessayez."));
    }
  }
}

void afficherPlateau() {
  for (int i = 0; i < 9; i++) {
    if (plateau[i] == VIDE) Serial.print(F(". "));
    else if (plateau[i] == 1) Serial.print(F("X "));
    else Serial.print(F("O "));
    if (i % 3 == 2) Serial.println();
  }
  Serial.println(F("-----"));
}

void finDePartie() {
  int8_t r = verifierGagnant(plateau);
  if (r == -1) Serial.println(F("MATCH NUL !"));
  else Serial.println(r == ROBOT_IA ? F("IA GAGNE !") : F("L'ADVERSAIRE A GAGNE !"));
  while (1); 
}