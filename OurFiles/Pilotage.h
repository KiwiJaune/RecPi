#ifndef __PILOTAGE_H__
#define __PILOTAGE_H__

#include "Types.h"
#include "UserUdp.h"


Trame PiloteGotoXY(int x, int y, unsigned char x_negatif, unsigned char y_negatif);

Trame PilotePIDRessource();
void PilotPIDInit(void);
Trame PiloteGetPosition(unsigned char cote);
Trame PiloteGetRawPosition(void);
Trame PiloteGetLongPosition(void);
Trame PiloteGetBuffPosition(void);
void PilotePIDCoeffs(unsigned int new_kp, unsigned int new_ki, unsigned int new_kd);
void PilotePIDManual(unsigned int gauche,unsigned int droite);
void PilotePIDBridage(unsigned int value);
Trame PilotePIDErreurs(void);
void PilotePIDFeedforward(unsigned int value);

void PiloteResetAx();
Trame PilotePositionXYT(void);
Trame PiloteDemandeCapteurs(char numCapteur);

void PiloteAlimentationCamera(char onOff);

void PilotePositionAx(char numAx, int position);

void PiloteLedAx(char numAx, char allume);

void PiloteVitesseAx(char numAx, int vitesse);

void PiloteAvalageSens(char avantOuArriere, char sensRotation);

void PiloteAvalageVitesse(char avantOuArriere, int vitesse);

void PiloteLevagePosition(char avantOuArriere, int position);

void PiloteLevageVitesse(char avantOuArriere, int vitesse);

Trame PiloteSwitchEnclanche(char numSwitch);

Trame PiloteDemandeCapteur(char numCapteur);

void PilotePositionPetitBras(char numPetitBras, int position);

void PiloteVitessePetitBras(char numPetitBras, int vitesse);


Trame PiloteSwitchTour();
Trame PiloteMesureBalise();
void PiloteLevagePosition(char avantOuArriere, int position);

Trame PiloteCapteurs(char cote);

int Coupure(void);

// Constantes des fonctions des actionneurs
#define ON  1
#define OFF 0

#define BRAS_DEPLOYE 145
#define BRAS_RETRACTE 280

#define DEBLOQUE_BAS 550
#define DEBLOQUE_HAUT 250

#define INIT_TURBINE 312 //Previous Value 5000
#define INIT_CANON 5000

#define CANON_ACTIVE 5500
#define CANON_DESACTIVE 5000
#define CANON_ACTIVE_MAX 10000

#define ASPIRATION_ACTIVE 625
#define ASPIRATION_DESACTIVE 312

#define ID_SERVO_ASPIRATEUR 16 //0
#define ID_SERVO_DEBLOQUEUR 0  //16 ou 2
#define ALL_SERVO 254

#define SHUTTER_BLOQUE 0
#define SHUTTER_PAS_BLOQUE 1
#define SHUTTER LATAbits.LATA1

#define SIGNAL_TURBINE LATCbits.LATC7
#define SIGNAL_CANON LATCbits.LATC6

#define RISING_EDGE 1
#define FALLING_EDGE 0 

//Fonction de test des actionneurs

void Init_Turbine(void);

void Init_Servos(void);

void Canon_Vitesse(unsigned int vitesse);

void Aspirateur_Vitesse(unsigned int vitesse);

unsigned int Send_Variable_Capteur_Couleur(void);

//Initialisation timer 3,4 et 5
void Init_Timer (void);

//ASSERVISSEMENT

int PiloteVitesse(int vitesse);

int PiloteAcceleration(int acceleration);

int PiloteAvancer(double distance);

int PiloteReculer(double distance);

int PilotePivoter(double angle, Cote direction);

int PiloteVirage(unsigned char reculer, unsigned char direction, double rayon, double angle);

int PiloteStop(unsigned char stopmode);

int PiloteRecallage(Sens s);

int PiloteAvancerEtapes(int nombreEtapes, Etape etape);

int PiloteValiderEtapes(int numEtape);

int PiloteOffsetAsserv(int x, int y, int teta);

Trame AnalyseTrame(Trame t);

#endif // __PILOTAGE_H__

// Déplacements
#define	CMD_AVANCER					0x01
#define	CMD_PIVOTER					0x03
#define	CMD_VIRAGE					0x04
#define	CMD_STOP					0x05
#define	CMD_GOTOXY					0x06
#define	CMD_RECALLAGE				0x10
#define CMD_FINRECALLAGE			0x11
#define CMD_FINDEPLACEMENT			0x12

// Asservissement
#define CMD_DEMANDEPOSITION			0x30
#define CMD_RETOURPOSITION			0x31
#define	CMD_VITESSE_LIGNE			0x32
#define	CMD_ACCELERATION_LIGNE		0x33
#define	CMD_VITESSE_PIVOT			0x34
#define	CMD_ACCELERATION_PIVOT		0x35
#define CMD_ENVOI_PID 				0x36
#define CMD_OFFSETASSERV			0x37

// Actionneurs
#define CMD_VITESSE_ASPIRATEUR      0x53
#define CMD_VITESSE_CANON           0x54
#define CMD_SHUTTER                 0x55

// Servomoteurs
#define CMD_SERVO_POSITION          0x60
#define CMD_SERVO_VITESSE           0x61

// Capteurs
#define CMD_REPONSE_PRESENCE_JACK	0x71
#define CMD_DEMANDE_COULEUR_EQUIPE	0x72
#define CMD_REPONSE_COULEUR_EQUIPE	0x73
#define CMD_DEMANDE_COULEUR			0x75
#define CMD_REPONSE_COULEUR_BALLE   0x76
#define CMD_DEMANDE_PRESENCE_BALLE	0x77
#define CMD_REPONSE_PRESENCE_BALLE	0x78
#define CMD_DEMANDE_PRESENCE_ASSIETTE	0x7A
#define CMD_REPONSE_PRESENCE_ASSIETTE	0x7B
#define CMD_DEMANDE_PRESENCE_ASPIRATEUR	0x7C
#define CMD_REPONSE_PRESENCE_ASPIRATEUR	0x7D

// Diagnostic
#define	CMD_ECHO					0xF0
#define	CMD_RESET_CARTE				0xF1
#define	CMD_ALIMENTATION			0xF3
