/*
 * File:   mrinterpoltypes.h
 * Author: frank
 *
 * Created on 2. Februar 2012, 10:53
 */

#ifndef MRINTERPOLTYPES_H
#define MRINTERPOLTYPES_H

#include <math.h>
#include "mrclient_common.h"
#include <stdio.h>

enum hubpos {
  HUB_UNGEAENDERT,
  HUB_UNTEN,
  HUB_OBEN
};

enum laserschutzfeld {
  SCHUTZ_FERN,
  SCHUTZ_NAH
};

enum mca_modus {
  MCA_UNGEANDERT,
  MCA_AUS,
  MCA_LOKAL_WEITER,
  MCA_LOKAL
};

enum fein_modus {
  UPDATE,
  PERIODE
};

/* Bahn-Parameter für das Geschwindigkeitsprofil eines mobilen Roboters */
class BahnParameter {
  public:
    double v_max, // maximale Bahngeschwindigkeit
      a,          // Beschleunigung auf der Bahn
      alpha_max,  // Winkelbeschleunigung von Theta
      omega_max,  // maximale Winkelgeschwindigkeit von Theta
      r;          // Radius bei einem Kreissegment

    unsigned int warten, // Anzahl der mS, die nach dem Abfahren des Segmentes zu warten sind
      n;                 // Anzahl der Punkte (z.B. Spline mit Zwischenpunkten)

    uint8_t noactuators; // Anzahl der Actuatorpositionen für das Bahn-Segment

    mrc_interpol_seg_type_t segmenttyp; // Art des Bahnsegment
};

class Orientierung {
  private:
    double winkel;

  public:
    Orientierung()
      : winkel(0.0)
    {
    }

    Orientierung(double w)
      : winkel(w)
    {
    }

    void Schreibe(double Wert)
    {
      winkel = Wert;
      // in den Wertebereich -PI...+PI bringen
      while (winkel > M_PI)
        winkel -= 2 * M_PI;
      while (winkel < -M_PI)
        winkel += 2 * M_PI;
    }

    /* Methode zum Lesen des Winkels in RAD.
     */
    virtual double inRAD()
    {
      return winkel;
    }

    virtual double inGRAD()
    {
      return winkel * 180.0 / M_PI;
    }

    /** Operator = mit Schreibe ueberladen.
     * der Wert auf der rechten Seite muss vom Typ double sein.
     */
    void operator=(double Wert)
    {
      Schreibe(Wert);
    }

    void operator-=(double Wert)
    {
      Schreibe(winkel - Wert);
    }

    void operator+=(double Wert)
    {
      Schreibe(winkel + Wert);
    }

    bool operator==(Orientierung &rhs)
    {
      bool ret = false;
      if (this->winkel == rhs.winkel)
        ret = true;
      if (((round(this->winkel * 100) == 314) && (round(rhs.winkel * 100) == -314)) || ((round(this->winkel * 100) == -314) && (round(rhs.winkel * 100) == 314)))
        ret = true;
      return ret;
    }

    /** Oberator-Ueberladung der Umwandlung in double.
     */
    virtual operator double()
    {
      return winkel;
    }
};

/* Klasse fuer die Pose eines mobilen Roboters
 * der Winkel theta wird in Rad gespeichert
 */
class Pose {
  public:
    double x,           // X-Position
      y;                // Y-Position
    Orientierung theta; // Drehwinkel um die Z-Achse (Theta)

    // leerer Konstruktor
    Pose()
      : x(0)
      , y(0)
      , theta(0)
    {
    }

    // Copy-Konstruktor
    Pose(const Pose &p)
      : x(p.x)
      , y(p.y)
      , theta(p.theta)
    {
    }

    // Konstruktor mit Vorgabe der Werte
    Pose(double X, double Y, Orientierung Theta)
      : x(X)
      , y(Y)
      , theta(Theta)
    {
    }

    // Konstruktor mit Vorgabe der Werte
    Pose(double X, double Y, double Theta)
      : x(X)
      , y(Y)
      , theta(Theta)
    {
    }

    // euklidischer Abstand zu einer anderen Pose
    double AbstandZu(Pose p)
    {
      return sqrt((p.x - x) * (p.x - x) + (p.y - y) * (p.y - y));
    }

    // Winkel der Geraden zu einer anderen Pose
    double BahnWinkelZu(Pose p)
    {
      return atan2(p.y - y, p.x - x);
    }

    // Drehwinkel zwischen zwei Posen, mit Begrenzung 0...2PI
    double DrehWinkelZu(Pose p)
    {
      double t;
      t = p.theta - theta;
      if (t > M_PI)
        t -= 2 * M_PI;
      if (t < -M_PI)
        t += 2 * M_PI;
      return t;
    }

    operator mrc_pose()
    {
      mrc_pose pose;

      pose.x = x;
      pose.y = y;
      pose.theta = theta.inRAD();

      return pose;
    }
};

/* Klasse fuer einen Bahnpunkt mit den Geschwindigkeiten in den drei Komponenten
 * bei einem omnidirektionalen Roboter sind die Geschwindigkeiten unabhaengig von Theta
 */
class Bahnpunkt : public Pose {
  public:
    double vx,                                // Geschwindigkeit in X-Richtung
      vy,                                     // Geschwindigkeit in Y-Richtung
      normvx,                                 // Normalisierte X-Richtung der Bahn (1. Ableitung)
      normvy,                                 // Normalisierte Y-Richtung der Bahn (1. Ableitung)
      normddx,                                // Normalisierte Krümmung der Bahn (2. Ableitung)
      normddy,                                // Normalisierte Krümmung der Bahn (2. Ableitung)
      vbahn,                                  // Bahngeschwindigkeit
      omega;                                  // Drehgeschwindigkeit um die Z-Achse (d Theta/dt)
    enum hubpos hub;                          // Position des Hubzelinders
    enum laserschutzfeld schutzfeld;          // Art des Schutzfelds
    mrc_localization_modes localizationmodus; // Modus des Monte Carlo Algorithmus
    double distanz;                           // Distanz seit letztem Bahnpunkt

    // leerer Konstruktor
    Bahnpunkt()
      : vx(0)
      , vy(0)
      , normvx(0)
      , normvy(0)
      , normddx(0)
      , normddy(0)
      , vbahn(0)
      , omega(0)
      , hub(HUB_UNGEAENDERT)
      , schutzfeld(SCHUTZ_NAH)
      , localizationmodus(MRC_LOCALIZATION_NO_CHANGE)
      , distanz(0)
    {
    }

    // Copy-Konstruktor
    Bahnpunkt(const Bahnpunkt &p)
      : Pose(p.x, p.y, p.theta)
      , vx(p.vx)
      , vy(p.vy)
      , normvx(p.normvx)
      , normvy(p.normvy)
      , normddx(p.normddx)
      , normddy(p.normddy)
      , vbahn(p.vbahn)
      , omega(p.omega)
      , hub(p.hub)
      , schutzfeld(p.schutzfeld)
      , localizationmodus(p.localizationmodus)
      , distanz(p.distanz)
    {
    }

    // Konstruktor mit Vorgabe der Positionswerte
    Bahnpunkt(double x, double y, double theta)
      : Pose(x, y, theta)
      , vx(0)
      , vy(0)
      , normvx(0)
      , normvy(0)
      , normddx(0)
      , normddy(0)
      , omega(0)
      , hub(HUB_UNGEAENDERT)
      , schutzfeld(SCHUTZ_FERN)
      , localizationmodus(MRC_LOCALIZATION_NO_CHANGE)
      , distanz(0)
    {
    }

    // Konstruktor mit Vorgabe der Geschwindigkeiten in Vx, Vy und Omega
    Bahnpunkt(double x, double y, double theta, double Vx, double Vy, double Omega)
      : Pose(x, y, theta)
      , vx(Vx)
      , vy(Vy)
      , normvx(0)
      , normvy(0)
      , normddx(0)
      , normddy(0)
      , omega(Omega)
      , hub(HUB_UNGEAENDERT)
      , schutzfeld(SCHUTZ_FERN)
      , localizationmodus(MRC_LOCALIZATION_NO_CHANGE)
      , distanz(0)
    {
      vbahn = 0;
    }

    // Konstruktor mit Vorgabe der Bahngeschwindigkeiten in Vbahn, Omega
    Bahnpunkt(double x, double y, double theta, double Vbahn, double Omega)
      : Pose(x, y, theta)
      , normvx(0)
      , normvy(0)
      , normddx(0)
      , normddy(0)
      , vbahn(Vbahn)
      , omega(Omega)
      , hub(HUB_UNGEAENDERT)
      , schutzfeld(SCHUTZ_FERN)
      , localizationmodus(MRC_LOCALIZATION_NO_CHANGE)
      , distanz(0)
    {
      vx = 0;
      vy = 0;
    }

    void setPoseV(double x, double y, double theta, double Vbahn, double Omega)
    {
      this->x = x;
      this->y = y;
      this->theta = theta;
      vbahn = Vbahn;
      omega = Omega;
    }

    void setPoseV(double X, double Y, double Theta, double Vx, double Vy, double Omega)
    {
      x = X;
      y = Y;
      theta = Theta;
      vx = Vx;
      vy = Vy;
      omega = Omega;
    }

    double BahnV()
    {
      if (vbahn == 0)
        vbahn = sqrt(vx * vx + vy * vy);
      return vbahn;
    }

    bool operator==(Bahnpunkt &rhs)
    {
      bool ret = false;
      if ((this->x == rhs.x) && (this->y == rhs.y) && (this->theta == rhs.theta))
        ret = true;
      return ret;
    }
};

#endif /* MRINTERPOLTYPES_H */
