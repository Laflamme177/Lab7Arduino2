#include <Arduino.h>
#line 1 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino"
/*
 * test_calculateur_Leq.ino
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

/* -------------------------------------------------------------
   Constantes et variables globales
   -------------------------------------------------------------- */
const uint32_t SERIAL_BAUD_RATE{115200};    // Terminal serie
const uint8_t PIN{A0};                      // Broche du Capteur sonore
const uint32_t TS = 62;                     // Période d'échantillionnage (ms)
const uint16_t NB_SAMPLE = 32;              // 32 x 62 ms ~ 2 secondes
const uint16_t NB_LI = 30;                  // 150 x 2 secondes = 5 minutes (*)
uint32_t countMillis;                       // Compter les minutes (pour debug seulement)
// (*) Évidemment vous pouvez réduire la période d'échantillonnage durant la phase
//     de déboggage (;-))

/* -------------------------------------------------------------
   Créer un objet Calculateur_Leq en utilisant des paramètres
   spécifiques.
   ------------------------------------------------------------- */
Calculateur_Leq leq(TS, NB_SAMPLE, NB_LI);
 
/* -------------------------------------------------------------
   Initialisation de l'ADC du microcontrôleur et affciher des
   messages sur le terminal série.
   ------------------------------------------------------------- */
// Macro pour extraire le nom de ce fichier (facultatif)
#include <string.h>
#define __FILENAME__ (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)
#line 53 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino"
void setup();
#line 87 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino"
void loop();
#line 107 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino"
void waitUntil(uint32_t w);
#line 53 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino"
void setup() 
{
  // Initialiser le terminal série
  Serial.begin(SERIAL_BAUD_RATE);

  // Sur le VS Code, l'ouverture du port série prend du temps et on
  // peut perdre des caractères. Ce problème n'existe pas sur l'include
  // de l'Arduino.
  waitUntil(2000);

  // Afficher les paramètres de fonctionnement du programme sur le terminal série
  Serial.print(F("<")); Serial.print(__FILENAME__); Serial.print(F(">"));
  Serial.println(F(" Démonstration de calcul Leq"));
  Serial.print(F("Ts = ")); Serial.print(leq.GetTs()); Serial.print(F("ms\t"));
  Serial.print(F("ti = ")); Serial.print(leq.GetTs() * leq.GetVrmSamples() / 1000.0);
  Serial.print(F("s\t"));
  Serial.print(F("tp = ")); Serial.print((leq.GetTs() * leq.GetVrmSamples() / 1000.0) * leq.GetLiSamples() / 60.0);
  Serial.println(F("min"));
  Serial.println("Leq (dB SPL)\tMinutes écoulées");

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
  // L'objet leq "sait" à quel moment il doit accumuler les valeurs
  // du signal sonore.
  leq.Accumulate();
  // L'objet leq sait à quels moments il faut calculer Vrms, Li et Leq
  if (leq.Compute() ) {
    Serial.print(leq.GetLeq(), 3); Serial.print(F("\t\t\t"));
    Serial.println((1.0 * millis() - countMillis) / 60000);
    countMillis = millis();
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


