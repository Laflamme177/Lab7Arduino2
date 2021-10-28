#line 1 "c:\\Users\\Utilisateur\\OneDrive - ETS\\GPA 788\\Laboratoire\\test_calculateur_Leq\\calculateur_leq.h"
//
// FICHIER: calculateur_leq.cpp
// AUTEURS: Samuel Joly & Olivier Laflamme & Pier-Olivier Jolin & Benjamin Hotte
// VERSION: 0.1
// OBJECTIF: Electret Sensor library for Arduino
//

#ifndef CALCULATEUR_LEQ_H
#define CALCULATEUR_LEQ_H

// Pour pouvoir utiliser un objet de type Calculateur_LI
#include "calculateur_li.h"

class Calculateur_Leq {
  /*
 * Classe pour réaliser le calcul du bruit équivalent Leq en
 * utilisant plusieurs niveaux d'énergies sonores Li. l'uti-
 * lisateur de cette classe doit indiquer le temps d'échantil-
 * lonnage et le nombre d'échantillons pour calculer un niveau
 * d'énergie sonore Li. Celui-ci peut ensuite:
 * 
 * - Lire la valeur actuelle de LEQ
 * - Obtenir le nombre d'échantillons par Li
 * - Obtenir le nombre de Li dans un LEQ
 * - Obtenir le temps d'échantillonnage
 * - Aller chercher temps du dernier échantillonnage
 * 
 */

public:
    // CONSTRUCTEURS -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
   
    // Avec paramètres
    Calculateur_Leq(uint32_t TS, uint16_t NB_SAMPLE, uint16_t NB_LI):
      mTS{TS},
      mNB_SAMPLE{NB_SAMPLE},
      mNB_LI{NB_LI},
      mLeq{0},
      mLeqTemp{0},
      mNB_LEQ{0},
      mLi()
    {
      mTimer=millis();
    }

    // Interdits
    Calculateur_Leq(const Calculateur_Leq& other) = delete;
    Calculateur_Leq& operator=(const Calculateur_Leq& other) = delete;
    Calculateur_Leq(Calculateur_Leq&& other) = delete;
    Calculateur_Leq& operator=(Calculateur_Leq&& other) = delete;

    // DESTRUCTEURS -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    ~Calculateur_Leq() = default;

    // ACCESSEURS -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    float GetLeq() { return mLeq; };                // Leq le plus récent
    uint32_t GetTs() { return  mTS; };              // Période d'échantillonnage
    uint16_t GetVrmSamples() { return mNB_SAMPLE;}; // Nombre d'échantillons
    uint16_t GetLiSamples() { return mNB_LI;};      // Nombres de Li
    uint32_t GetTimer(){return mTimer;};            // Temps du dernier échantillonnage


    // PUBLIC FUNCTIONS -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 

    void Accumulate() { 
      /*
       * Cette fonction permet l'échantillonnage de la pression acoustique
       * avec l'objet vrms du Li. l'échantillonnage se fait uniquement
       * lorsque a période d'échantillonage est respectée. Une fois la valeur
       * du capteur obtenue, le "timer" est mis à jour et la valeur cumulée du 
       * vrms mise à jour
       */

      while((millis()-mTimer)<mTS)
      {

      }
      mTimer=millis();
      mLi.Accumulate();
    } ;
    bool Compute() { 
      /*
       * Cette fonction permet doit être appelée juste après Accumulate. Elle
       * vérifie le nombre d'échantillons pris jusqu'à date et vérifie s'il 
       * correspond au nombre compris dans un niveau d'énergie sonore Li. Si 
       * cela s'avère vrai, le calcul du Li s'effectue. La valeur est ajoutée 
       * à la somme LeqTemp
       * 
       * Si le nombre d'échantillons total correspond au nombre dans un Leq,
       * le calcul du Leq s'effectue lui-aussi. La valeur est mise dans mLeq
       * 
       * Valeur de retour: 1 si la valeur du LEQ est mise à jour, 0 sinon
       *
       */

        if(mLi.GetNbSamples()==mNB_SAMPLE) 
        {
            mLi.Compute();
            mLeqTemp += (mNB_SAMPLE*(mTS/1000.0)*pow(10,0.1*mLi.GetLi()));
            Serial.print(mLi.GetLi()); Serial.print("\n");
        } 
        if(mLi.GetTotalSamples()==mNB_SAMPLE*mNB_LI*(1+mNB_LEQ))
        {
          mLeq = 10.0* log10(mLeqTemp/(mNB_SAMPLE*(mTS/1000.0)*mNB_LI));
          mLeqTemp = 0;
          mNB_LEQ++;
          return true;
        } 
        
        return false;

        } ;

  private:

    // MEMBRES -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    uint32_t mTS;                // Période d'échantillonnage
    uint16_t mNB_SAMPLE, mNB_LI; // Nombre d'échantillons, de Li
    float mLeqTemp;              // Valeur cumulée temporaire du LEQ
    float mLeq;                  // LEQ
    Calculateur_Li mLi;          // Instance Li
    uint32_t mTimer;             // Temps du dernier échantillonnage
    uint32_t mNB_LEQ;            // nombre de Leq calculés depuis le début

};

#endif