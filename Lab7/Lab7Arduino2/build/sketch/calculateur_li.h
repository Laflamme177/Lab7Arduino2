#line 1 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\calculateur_li.h"
/*
 * Calculateur_Li
 * 
 * Une classe pour réaliser le calcul de la valeur Li en
 * utilisant la sensibilité et le gain du capteur Electret
 * MAX4466.
 * 
 * Voir les notes de cours "Conception des objets (IIB)" pour les
 * calculs à effectuer.
 * 
 * Note: Cette classe contient un objet de classe
 *       Calculateur_VRMS pour calculer la valeur dBV du signal
 *       échantillonné.
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
#ifndef CALCULATEUR_LI_H
#define CALCULATEUR_LI_H

// Pour pouvoir utiliser un objet de type Calculateur_VRMS
#include "calculateur_vrms.h"

class Calculateur_Li {
public:
  // Pour le microphone Electret une application de 94 dB SPL
  // produit -44 dBV/Pa à sa sortie. Le gain du MAX4466 est par
  // défaut réglé à 125 ou 42 dBV.
  Calculateur_Li(double P = 94.0, double M = -44, double G = 52.0)
    : m_P(P), m_M(M), m_G(G)
  {
  }
    // Empêcher l'utilisation du constructeur de copie
  Calculateur_Li(const Calculateur_Li& other) = delete;
  // Empêcher l'utilisation de l'opérateur d'assignation
  Calculateur_Li& operator=(const Calculateur_Li& other) = delete;
  // Empêcher l'utilisation du constructeur par déplacement
  Calculateur_Li(Calculateur_Li&& other) = delete;
  // Empêcher l'utilisation de l'opérateur de déplacement
  Calculateur_Li& operator=(Calculateur_Li&& other) = delete;

  ~Calculateur_Li() = default;  // Destructeur

  /* -------------------------------------------------------------
     Accesseurs des données membres de la classe
     -------------------------------------------------------------- */  
  inline double GetLi() const { return m_Li; }
  inline double GetP() const { return m_P; }
  inline double GetM() const { return m_M; }
  inline double GetG() const { return m_G; }
  
  inline uint16_t GetNbSamples() const { return c.GetnbSamples(); }
  inline uint16_t GetTotalSamples() const { return c.GetTotalSamples(); }
  inline double GetVrms() const { return c.GetVrms(); }
  inline double GetdBV() const { return c.GetdBV(); }
  inline uint8_t GetAPin() const { return c.GetAPin(); }
  inline double GetVMax() const { return c.GetVMax(); }
  inline int16_t GetAdcMax() const { return c.GetAdcMax(); }


  /* -------------------------------------------------------------
     Services publics offerts
     -------------------------------------------------------------- */
  // Utiliser Accumulate() de l'objet de classe Calculateur_VRMS
  // pour accumuler les valeurs du capteur sonore.
  // Note: La temporisation est la responsabilité de l'utilisateur.
  void Accumulate() {
    c.Accumulate();
  }
  // Utiliser Compute() de l'objet de classe Calculateur_VRMS
  // pour calculer la valeur rms du signal sonore et ensuite
  // calculer Li du signal.
  // Note: La temporisation est la responsabilité de l'utilisateur.
  double Compute() {
    c.Compute();
    m_Li = GetdBV() + m_P - m_M - m_G;
    return m_Li;
  }
  
private:
  // Objet de classe Calculateur_VRMS pour réaliser les calculs
  // Vrms et dBV du signal échantillonné.
  // La relation entre la classe Calculateur_VRMS et la classe 
  // Calculateur_Li est une relation de "composition".
  Calculateur_VRMS c;
  // Pour le calcul de Li
  double m_Li;                         // Niveau d'énergie sonore au temps ti
  double m_P;                          // Sensibilité Electret en dB SPL
  double m_M;                          // Sensibilité Electret en dBV/Pa
  double m_G;                          // Gain du MAX4466 en dBV
};

#endif
