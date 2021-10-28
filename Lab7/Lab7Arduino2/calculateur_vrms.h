/*
 * Calculateur_VRMS
 * 
 * Une classe pour réaliser le calcul de la tension rms qui
 * représente le niveau sonore à l'entrée du capteur Electret
 * MAX4466.
 *
 * Voir les notes de cours "Conception des objets (IIB)" pour les
 * calculs à effectuer.
 *
 *  
 * Note: La tension rms calculée est disponible en volt rms et
 *       en dBV.
 *
 * Convention:
 *  Variables -> camelCase
 *  Classes, fonctions -> PascalCase
 *  Constantes, types utilisateurs -> SNAKE_CASE
 * 
 * GPA788 - ETS
 * T. Wong
 * 09-2018
 * 08-2020
 */
#ifndef CALCULATEUR_VRMS_H
#define CALCULATEUR_VRMS_H

class Calculateur_VRMS {
public:
  // Constructeur
  // Paramètres: aPin  - broche reliée au capteur sonore.
  //             vMax - tension max à la sortie du capteur sonore.
  //           maxADC - valeur max à la sortie de l'ADC du ucontrôleur
  Calculateur_VRMS(uint8_t aPin = A0, double vMax = 3.3, int16_t maxADC = 1024) {
    m_APin = aPin; m_VMax = vMax; m_AdcMax = maxADC;
    m_Amplitude =  m_Vrms = m_dBV = m_TmpVrms = 0.0;
    m_NbTotalSamples = m_NbSamples = 0;
    m_VDC_OFFSET = m_VMax / 2.0;
    m_C1 = m_VMax / m_AdcMax;
  }
  // Empêcher l'utilisation du constructeur de copie
  Calculateur_VRMS(const Calculateur_VRMS& other) = delete;
  // Empêcher l'utilisation de l'opérateur d'assignation
  Calculateur_VRMS& operator=(const Calculateur_VRMS& other) = delete;
  // Empêcher l'utilisation du constructeur par déplacement
  Calculateur_VRMS(Calculateur_VRMS&& other) = delete;
  // Empêcher l'utilisation de l'opérateur de déplacement
  Calculateur_VRMS& operator=(Calculateur_VRMS&& other) = delete;

  ~Calculateur_VRMS() {}                    // Destructeur

  /* -------------------------------------------------------------
     Accesseurs des données membres de la classe
     -------------------------------------------------------------- */
  inline uint16_t GetnbSamples() const { return m_NbSamples; }
  inline uint16_t GetTotalSamples() const { return m_NbTotalSamples; }
  inline double GetVrms() const { return m_Vrms; }
  inline int16_t GetAmplitude() const { return m_Amplitude; }
  inline double GetdBV() const { return m_dBV; }
  inline uint8_t GetAPin() const { return m_APin; }
  inline double GetVMax() const { return m_VMax; }
  inline int16_t GetAdcMax() const { return m_AdcMax; }

  /* -------------------------------------------------------------
     Services publics offerts
     -------------------------------------------------------------- */
  // Accumuler les valeurs lues du capteur sonore dans le but de
  // calculer la valeur rms du signal sonore.
  // Note: La temporisation est la responsabilité de l'utilisateur.
  void Accumulate() {
    m_Amplitude = analogRead(m_APin);
    // Convertir en volts
    double v = (m_Amplitude * m_C1) - m_VDC_OFFSET;
    // Accumuler v^2
    m_TmpVrms += (v * v);
    ++m_NbSamples;   
  }

  // Calculer la valeur rms du signal sonore.
  // Note: La temporisation est la responsabilité de l'utilisateur.
  void Compute() {
    m_NbTotalSamples += m_NbSamples;    
    m_Vrms = sqrt(m_TmpVrms / m_NbSamples);
    m_dBV = 20.0 * log10(m_Vrms);
    // RAZ le décompte des échantillons
    m_NbSamples = 0;
    // RAZ le cumule des v^2
    m_TmpVrms = 0.0;
  }
  
private:
  uint8_t m_APin;                           // Broche du Capteur sonore
  double m_VMax;                            // VREF est à 3.3 V par défaut
  int16_t m_AdcMax;                         // ADC à 10 bits
  double m_VDC_OFFSET;                      // Tension décalage (niveau CC) 
  double m_C1;                              // Conversion bits -> volt
  int16_t m_Amplitude;                      // Signal échantillonné en bits
  double m_Vrms;                            // Valeur Vrms
  double m_TmpVrms;                         // Pour acumuler v au carré
  double m_dBV;                             // Valeur dBV
  uint16_t m_NbTotalSamples;                // Nb. total des échantillons
  uint16_t m_NbSamples;                     // Nb. d'échantillons
};

#endif