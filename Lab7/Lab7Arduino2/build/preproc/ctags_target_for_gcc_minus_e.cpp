# 1 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino"
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
# 25 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino"
// Pour pouvoir utiliser un objet de type Calculateur_Leq
# 27 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino" 2

/* -------------------------------------------------------------

   Constantes et variables globales

   -------------------------------------------------------------- */
# 31 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino"
const uint32_t SERIAL_BAUD_RATE{115200}; // Terminal serie
const uint8_t PIN{A0}; // Broche du Capteur sonore
const uint32_t TS = 62; // Période d'échantillionnage (ms)
const uint16_t NB_SAMPLE = 32; // 32 x 62 ms ~ 2 secondes
const uint16_t NB_LI = 30; // 150 x 2 secondes = 5 minutes (*)
uint32_t countMillis; // Compter les minutes (pour debug seulement)
// (*) Évidemment vous pouvez réduire la période d'échantillonnage durant la phase
//     de déboggage (;-))

/* -------------------------------------------------------------

   Créer un objet Calculateur_Leq en utilisant des paramètres

   spécifiques.

   ------------------------------------------------------------- */
# 44 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino"
Calculateur_Leq leq(TS, NB_SAMPLE, NB_LI);

/* -------------------------------------------------------------

   Initialisation de l'ADC du microcontrôleur et affciher des

   messages sur le terminal série.

   ------------------------------------------------------------- */
# 50 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino"
// Macro pour extraire le nom de ce fichier (facultatif)
# 52 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino" 2


# 53 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino"
void setup()
{
  // Initialiser le terminal série
  Serial.begin(SERIAL_BAUD_RATE);

  // Sur le VS Code, l'ouverture du port série prend du temps et on
  // peut perdre des caractères. Ce problème n'existe pas sur l'include
  // de l'Arduino.
  waitUntil(2000);

  // Afficher les paramètres de fonctionnement du programme sur le terminal série
  Serial.print((reinterpret_cast<const __FlashStringHelper *>(
# 64 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino" 3
              (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 64 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino"
              "<"
# 64 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino" 3
              ); &__c[0];}))
# 64 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino"
              ))); Serial.print((strrchr("c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino", '\\') ? strrchr("c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino", '\\') + 1 : "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino")); Serial.print((reinterpret_cast<const __FlashStringHelper *>(
# 64 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino" 3
                                                                (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 64 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino"
                                                                ">"
# 64 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino" 3
                                                                ); &__c[0];}))
# 64 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino"
                                                                )));
  Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 65 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino" 3
                (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 65 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino"
                " Démonstration de calcul Leq"
# 65 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino" 3
                ); &__c[0];}))
# 65 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino"
                )));
  Serial.print((reinterpret_cast<const __FlashStringHelper *>(
# 66 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino" 3
              (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 66 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino"
              "Ts = "
# 66 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino" 3
              ); &__c[0];}))
# 66 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino"
              ))); Serial.print(leq.GetTs()); Serial.print((reinterpret_cast<const __FlashStringHelper *>(
# 66 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino" 3
                                                                   (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 66 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino"
                                                                   "ms\t"
# 66 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino" 3
                                                                   ); &__c[0];}))
# 66 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino"
                                                                   )));
  Serial.print((reinterpret_cast<const __FlashStringHelper *>(
# 67 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino" 3
              (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 67 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino"
              "ti = "
# 67 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino" 3
              ); &__c[0];}))
# 67 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino"
              ))); Serial.print(leq.GetTs() * leq.GetVrmSamples() / 1000.0);
  Serial.print((reinterpret_cast<const __FlashStringHelper *>(
# 68 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino" 3
              (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 68 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino"
              "s\t"
# 68 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino" 3
              ); &__c[0];}))
# 68 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino"
              )));
  Serial.print((reinterpret_cast<const __FlashStringHelper *>(
# 69 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino" 3
              (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 69 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino"
              "tp = "
# 69 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino" 3
              ); &__c[0];}))
# 69 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino"
              ))); Serial.print((leq.GetTs() * leq.GetVrmSamples() / 1000.0) * leq.GetLiSamples() / 60.0);
  Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 70 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino" 3
                (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 70 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino"
                "min"
# 70 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino" 3
                ); &__c[0];}))
# 70 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino"
                )));
  Serial.println("Leq (dB SPL)\tMinutes écoulées");

  // Pour l'ADC du microcontrôleur...
  analogReference(0); // utiliser VREF externe pour l'ADC
  pinMode(PIN, 0x0); // capteur sonore à la broche PIN
  // Pour debug seulement
  countMillis = millis(); // compter le nb. de ms écoulées
}

/* -------------------------------------------------------------

   À chaque exécution de loop(), on exécute les fonctions

   membres Accumulate et Compute() du Calculateur_Leq.



   La temporisation s'effectue dans ces fonctions membres

   simplifiant ainsi leur utilisantion.

   ------------------------------------------------------------- */
# 87 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino"
void loop()
{
  // L'objet leq "sait" à quel moment il doit accumuler les valeurs
  // du signal sonore.
  leq.Accumulate();
  // L'objet leq sait à quels moments il faut calculer Vrms, Li et Leq
  if (leq.Compute() ) {
    Serial.print(leq.GetLeq(), 3); Serial.print((reinterpret_cast<const __FlashStringHelper *>(
# 94 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino" 3
                                               (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 94 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino"
                                               "\t\t\t"
# 94 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino" 3
                                               ); &__c[0];}))
# 94 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino"
                                               )));
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
# 107 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\test_calculateur_Leq.ino"
void waitUntil(uint32_t w) {
  uint32_t t{millis()};
  // Attendre w millisecondes
  while (millis() < t + w) {}
}
