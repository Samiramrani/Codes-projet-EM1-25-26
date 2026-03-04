const int encoderPin1 = 3; // Qui est connecté au moteur 1 mais sur le driver c'est 2
const int encoderPin2 = 2;
volatile long encoderTicks1 = 0;
volatile long encoderTicks2 = 0; 

const float CPR1_HIGH = 709.2; 
const float CPR2_HIGH = 353.4;
const float CPR1_LOW = 431.3;
const float CPR2_LOW = 353.4;

int PWM1 = 9;
int DIR1 = 8;
int PWM2 = 6;
int DIR2 = 7;

void setup() {

Serial.begin(9600);

pinMode(DIR1, OUTPUT);
pinMode(DIR2, OUTPUT);

pinMode(encoderPin1, INPUT_PULLUP);
pinMode(encoderPin2, INPUT_PULLUP);

attachInterrupt(digitalPinToInterrupt(encoderPin1), countTicks1, RISING);
attachInterrupt(digitalPinToInterrupt(encoderPin2), countTicks2, RISING);

};

void loop() {

  digitalWrite(DIR1, HIGH);
  analogWrite(PWM1, 100);
  digitalWrite(DIR2, HIGH);
  analogWrite(PWM2, 100);
  Serial.println(encoderTicks1);
  Serial.println(encoderTicks2);
  //Serial.println(encoderTicks2/CPR2_HIGH);

};

void countTicks1(){
  encoderTicks1++;
};

void countTicks2(){
  encoderTicks2++;
};
