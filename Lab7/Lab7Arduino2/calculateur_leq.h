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
    Calculateur_Leq():
      mTS{62},
      mNB_SAMPLE{32},
      mNB_LI{30},
      mLeq{0},
      mLeqTemp{0},
      mNb_Leq{0},
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
    uint32_t GetNb_Leq(){return mNb_Leq;};            // Temps du dernier échantillonnage
    // MUTATEURS -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
  void setTs(uint32_t new_Ts){mTS=new_Ts;};
  void setLi(uint32_t new_Li){mLi.setLi(new_Li);};


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
            //Serial.print(mLi.GetLi()); Serial.print("\n");
        } 
        if(mLi.GetTotalSamples()==mNB_SAMPLE*mNB_LI*(1+mNb_Leq))
        {
          mLeq = 10.0* log10(mLeqTemp/(mNB_SAMPLE*(mTS/1000.0)*mNB_LI));
          mLeqTemp = 0;
          mNb_Leq++;
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
    uint32_t mNb_Leq;            // nombre de Leq calculés depuis le début

};

#endif