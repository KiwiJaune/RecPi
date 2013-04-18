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

#define	CMD_AVANCER					0x01
#define	CMD_PIVOTER					0x03
#define	CMD_VIRAGE					0x04
#define	CMD_STOP					0x05
#define	CMD_GOTOXY					0x06
#define	CMD_RECALLAGE				0x10
#define	CMD_VITESSE_LIGNE			0x32
#define	CMD_ACCELERATION_LIGNE		0x33
#define	CMD_VITESSE_PIVOT			0x34
#define	CMD_ACCELERATION_PIVOT		0x35
#define CMD_ENVOI_PID 				0x45
#define	CMD_ECHO					0xF0
#define	CMD_RESET_CARTE				0xF1
#define	CMD_ALIMENTATION			0xF3
#define CMD_OFFSETASSERV			0x46


