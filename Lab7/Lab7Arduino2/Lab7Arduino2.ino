/*
 * Lab7Arduino2.ino
 * 
 * Programme de test servant à tester l'implantation de la classe
 * Calculateur_Leq. La quantité Leq d'un signal est le niveau d'énergie 
 * sonore équivalent pour durée déterminée. 
 *
 * Pour ce programme la fréquence d'échantillonnage est
 * réglée à fs = 16 Hz (62.5 ms) et la valeur Li sera calculée
 * à 1 Hz (1 seconde). Finalement, Leq sera calculé pour une
 * durée de 5 minutes.
 * 
 * Convention:
 *  Variables -> camelCase
 *  Classes, fonctions -> PascalCase
 *  Constantes, types utilisateurs -> SNAKE_CASE
 * 
 * 
 * GPA788 - ETS
 * T. Wong
 * 09-2018
 * 08-2020
 */

// Pour pouvoir utiliser un objet de type Calculateur_Leq
#include "calculateur_leq.h"

// Pour la communication I2C
#include <Wire.h>

/* -------------------------------------------------------------
   Constantes et variables globales
   -------------------------------------------------------------- */
const uint32_t SERIAL_BAUD_RATE{115200};    // Terminal serie
const uint8_t PIN{A0};                      // Broche du Capteur sonore
const uint8_t ADR_NOEUD {0x45};        // Adresse I2C du noeud (Hex 44 -> décimal 68)
const uint8_t NB_REGISTRES{9};         // Nombre de registres sur ce noeud
const uint8_t MIN_Ts{3};               //Période d'échantillonage minimum
const uint8_t MAX_Ts{200};            //Période d'échantillonage maximum
uint8_t adrReg;                        // Pour mémoriser le numéro de registre reçu du coordonnateur
int16_t cmd;                          // Commande à exécuter
//uint32_t TS{62};                     // Période d'échantillionnage (ms)
//const uint16_t NB_SAMPLE{32};              // 32 x 62 ms ~ 2 secondes
//const uint16_t NB_LI{30};                  // 150 x 2 secondes = 5 minutes (*)

/* --------------------------------------------------------------------------------- 
 * Commande du coordonateur
 CMD:Stop //Arrète la lecture de LEQ
 CMD:Stop //Active la lecture de LEQ
 * --------------------------------------------------------------------------------- */
enum  CMD : int16_t {Stop=0,Go=1};  //


uint32_t countMillis;                       // Compter les minutes (pour debug seulement)
// (*) Évidemment vous pouvez réduire la période d'échantillonnage durant la phase
//     de déboggage (;-))

/* ------------------------------------------------------------- */
/* La carte des regisgtres                                       */
/* ------------------------------------------------------------- */
/* La communication entre le Pi et l'Arduino passe par des re-   */
/* gistres. Le Pi envoie un numéro de registre à l'Arduino puis  */
/* il envoie une commande de lecture/écriture pour signifier     */
/* ce qu'il veut faire avec le registre.                         */
/* ------------------------------------------------------------- */
/* Pour cet exemple, la cart des registres occupe 7 octets:      */
/* - gpa[3]: 3 octets                                            */
/* - float:  4 octets                                            */
/* ------------------------------------------------------------- */
union CarteRegistres {
  // Du côté de l'Arduino
  // On écrit/lit dans les registres par cette structure
  struct {
   //Taux d'échantillonage (1 octet)
   volatile uint8_t Ts;
   //Nombre d'échantillion (2 octets)
   volatile uint16_t Nb_Sample;
    //Leq (4 octets)
   volatile float Leq_value;
    //Nombre de Li (2 octets)
   volatile uint16_t Nb_Li;
  } champs;

  // Du côté du Pi
  // On écrit/lit les registres par ce tableau d'octets
  uint8_t regs[NB_REGISTRES];
};

/* ------------------------------------------------------------- */
/* Créer la carte des registres                                  */
/* ------------------------------------------------------------- */
union CarteRegistres cr;

/* -------------------------------------------------------------
   Créer un objet Calculateur_Leq en utilisant des paramètres
   spécifiques
   ------------------------------------------------------------- */
Calculateur_Leq leq;
 
/* ---
----------------------------------------------------------
   Initialisation de l'ADC du microcontrôleur et affciher des
   messages sur le terminal série.
   ------------------------------------------------------------- */
// Macro pour extraire le nom de ce fichier (facultatif)
#include <string.h>
#define __FILENAME__ (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)
void setup() 
{
  // Initialiser le terminal série
  Serial.begin(SERIAL_BAUD_RATE);

  // Sur le VS Code, l'ouverture du port série prend du temps et on
  // peut perdre des caractères. Ce problème n'existe pas sur l'include
  // de l'Arduino.
  waitUntil(2000);

 // Initialiser les champs de la carte des registres
  cr.champs.Ts = MIN_Ts;
  cr.champs.Nb_Sample = 0;
  cr.champs.Nb_Li = -1;
  cr.champs.Leq_value = -1;
  // Initialiser les variables de contrôle de la
  // communication I2C
  cmd = CMD::Stop;
  adrReg = -1;
  // Initaliser Wire avec le numéro du noeud
  Wire.begin(ADR_NOEUD);
  
   // Régler la fonction de rappel (callback) pour traiter une requête de données
  // venant du coordonnateur
  Wire.onRequest(i2c_requestEvent);
  // Régler la fonction de rappel (callback) pour traiter la réception de données
  // venant du coordonnateur
  Wire.onReceive(i2c_receiveEvent);
  waitUntil(1500);

  // Afficher les paramètres de fonctionnement du programme sur le terminal série
  Serial.print(F("<")); Serial.print(__FILENAME__); Serial.print(F(">"));
  Serial.println(F(" Démonstration de calcul Leq"));
  
  Serial.print(F("Ts = ")); Serial.print(leq.GetTs()); Serial.print(F("ms\t"));
  Serial.print(F("ti = ")); Serial.print(leq.GetTs() * leq.GetVrmSamples() / 1000.0);
  Serial.print(F("s\t"));
  Serial.print(F("tp = ")); Serial.print((leq.GetTs() * leq.GetVrmSamples() / 1000.0) * leq.GetLiSamples() / 60.0);
  Serial.println(F("min"));
  Serial.println("Leq (dB SPL)\tMinutes écoulées");

    // Indiquer que le noeud est prêt
  Serial.print(F("Noeud à l'adresse 0x")); Serial.print(ADR_NOEUD, HEX);
  Serial.println(F(" prêt à recevoir des commandes"));

  // Pour l'ADC du microcontrôleur...
  analogReference(EXTERNAL);                // utiliser VREF externe pour l'ADC
  pinMode(PIN, INPUT);                      // capteur sonore à la broche PIN
  // Pour debug seulement
  countMillis = millis();                   // compter le nb. de ms écoulées
}
 
/* -------------------------------------------------------------
   À chaque exécution de loop(), on exécute les fonctions
   membres Accumulate et Compute() du Calculateur_Leq.

   La temporisation s'effectue dans ces fonctions membres
   simplifiant ainsi leur utilisantion.
   ------------------------------------------------------------- */
void loop() 
{
   if(cmd){
// L'objet leq "sait" à quel moment il doit accumuler les valeurs
  // du signal sonore.
  leq.Accumulate();
  // L'objet leq sait à quels moments il faut calculer Vrms, Li et Leq
  if (leq.Compute() ) {
     cr.champs.Leq_value=leq.GetLeq();
     cr.champs.Nb_Sample= leq.GetNb_Leq();
     Serial.print(F("Échantillion #:  ")); Serial.print(leq.GetNb_Leq()); Serial.print(leq.GetLeq(), 3); Serial.print(F("\t\t\t"));
    Serial.println((1.0 * millis() - countMillis) / 60000);
    countMillis = millis();
  }

   }
  
}

/* ------------------------------------------------------------- */
/* i2c_receiveFunc(int x)                                        */
/* Fonction de rappel exécutée par la bibliothèque Wire lors de  */
/* la réception des données venant du coordonnateur Pi.          */
/*                                                               */
/* combien: un entier indiquant le nombre d'octets reçus.        */
/* ------------------------------------------------------------- */
/* Note: La communication I2C s'effectue par interruptions (#25  */
/*       TWI_vect).                                              */
/* ------------------------------------------------------------- */
void i2c_receiveEvent(int combien) {
// Traiter les commandes ou les adresses de registre (1 octet)
if (combien == 1) {
// Un seul octet reçu. C'est probablement une commande.
uint8_t data = Wire.read();
switch (data) {
case 0xA1:
cmd = CMD::Stop;
Serial.println(F("commande 'Arrêter' reçue"));
break;
case 0xA2:
cmd = CMD::Go;
Serial.println(F("Commande 'Démarrer' reçue"));
break;
default:
// Sinon, c'est probablement une adresse de registre
if ((data >= 0) && (data < NB_REGISTRES)) {
adrReg = data;
}
else adrReg = -1; // Il y sans doute une erreur!
}
} 
  else if (combien == 2) {
// Deux octets reçu. C'est probablement pour changer le
// taux d'échantillonnage.
uint8_t data1 = Wire.read();
uint8_t data2 = Wire.read();
Serial.println(F("Commande 'Changer Ts' reçue"));
if ((data1 == 0xA0) && (data2 >= MIN_Ts) && (data2 <= MAX_Ts
)) {
cr.champs.Ts = data2;
Serial.print(F("La nouvelle valeur est: "));
Serial.print(cr.champs.Ts); Serial.println(F(" secondes"))
;
}
}
else {
    // Ignorer la réception n > 2 octets.
    Serial.println(F("Erreur: ce noeud n'accepte\
    pas de communication/commande à trois octets"));
  }

}

/* ------------------------------------------------------------- */
/* i2c_requestEvent()                                            */
/* Fonction de rappel exécutée par la bibliothèque Wire lors     */
/* d'une requête de données venant du coordonnateur Pi.          */
/* ------------------------------------------------------------- */
/* Note: La communication I2C s'effectue par interruptions (#25  */
/*       TWI_vect).                                              */
/* ------------------------------------------------------------- */
void i2c_requestEvent(){
// Le coordonnateur veut la valeur d'un registre. L'adresse du
// registre a été reçue précédemment.
if ((adrReg >= 0) && (adrReg < NB_REGISTRES)){
Serial.print("");
// Envoyer le contenu du registre au coordonnateur
Wire.write(cr.regs[adrReg]);
}
}

/* ---------------------------------------------------------------
   Fonction pour créer un delai de w millisecondes
   
   La fonction delay() est utilisée dans bien des tutoriels pour
   créer un delai temporel. On peut aussi créer notre propre délai
   et utiliser une unité de temps à notre guise.
   --------------------------------------------------------------- */
void waitUntil(uint32_t w) {
  uint32_t t{millis()};
  // Attendre w millisecondes
  while (millis() < t + w) {}
}

