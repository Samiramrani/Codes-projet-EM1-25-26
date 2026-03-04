//RADIO : UNO (reçoit, transmet)

// Configuration radio:
#include <SPI.h>
#include <RF24.h>
#define pinCE   9
#define pinCSN  10
#define tunnel  "PIPE1"

RF24 radio(pinCE, pinCSN);

const byte adresse[6] = tunnel;

bool role = 1;   // 1 = emetteur  /  0 = recepteur

char message[32] = "Heo World !!!";

void setup() {
 
  Serial.begin(9600);
 
  if (role == 0) {
    Serial.println("Recepteur NRF24L01");
  } else {
    Serial.println("Emetteur NRF24L01");
  }
  Serial.println("");

  radio.begin();
  radio.setChannel(109);
  radio.setPALevel(RF24_PA_MIN);
  radio.setRetries(5, 15);

  if (role == 0) {
    radio.openReadingPipe(0, adresse);
    radio.startListening();
  } else {
    radio.openWritingPipe(adresse);
    radio.stopListening();
  }
}

void loop() {

  if (role == 1) {

      radio.write(&message, sizeof(message));
      Serial.println("Message envoye");
      delay(1000);

  }
  else {

    if (radio.available()) {
      radio.read(&message, sizeof(message));
      Serial.print("Message recu : ");
      Serial.println(message);
    }
  }
}
