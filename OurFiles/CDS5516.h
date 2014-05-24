#ifndef __CDS5516_H__
#define __CDS5516_H__

#include "UserUdp.h"


static Trame trame;
static BYTE msg[20];

// Bauderate = 1   pour 1000000
// Bauderate = 3   pour 500000
// Bauderate = 4   pour 400000
// Bauderate = 7   pour 250000
// Bauderate = 9   pour 200000
// Bauderate = 16  pour 115200
// Bauderate = 34  pour 57600
// Bauderate = 103 pour 19200
// Bauderate = 207 pour 9600

// Fonctions de communication servomoteur
void CDS5516Reset(char baud, char id);
int CDS5516Ping(char baud, char id);
void CDS5516EnvoiMessage(char baud, char id, char fonction, int valeur, char nbChar);
int CDS5516DemandeMessage(char baud, char id, char fonction, char nbChar);

// Fonctions demandant des valeurs du servomoteur
int CDS5516DemandeConfigAlarmeLED(char baud, char id);
int CDS5516DemandeConfigAlarmeShutdown(char baud, char id);
int CDS5516DemandeConfigEcho(char baud, char id);
int CDS5516DemandeCoupleActive(char baud, char id);
int CDS5516DemandeCoupleMaximum(char baud, char id);
int CDS5516DemandeLed(char baud, char id);
int CDS5516DemandeMouvement(char baud, char id);
int CDS5516DemandeModele(char baud, char id);
int CDS5516DemandePositionActuelle(char baud, char id);
int CDS5516DemandePositionCible(char baud, char id);
int CDS5516DemandePositionMax(char baud, char id);
int CDS5516DemandePositionMin(char baud, char id);
int CDS5516DemandeTemperature(char baud, char id);
int CDS5516DemandeTension(char baud, char id);
int CDS5516DemandeVersionFirmware(char baud, char id);
int CDS5516DemandeVitesseActuelle(char baud, char id);
int CDS5516DemandeVitesseMax(char baud, char id);
int CDS5516DemandeCCWMargin(char baud, char id);
int CDS5516DemandeCWMargin(char baud, char id);
int CDS5516DemandeCCWSlope(char baud, char id);
int CDS5516DemandeCWSlope(char baud, char id);
int CDS5516DemandeBaudrate(char id);
int CDS5516DemandeID(char baudrate);

// Fonctions d'envoi de valeurs
void CDS5516EnvoiComplianceParams(char baud, char id, char CCWSlope, char CCWMargin, char CWSlope, char CWMargin);
void CDS5516EnvoiAlarmeShutdown(char baud, char id, char inputVoltage, char angleLimit, char overheating, char range, char checksum, char overload, char instruction);
void CDS5516EnvoiAlarmeLED(char baud, char id, char inputVoltage, char angleLimit, char overheating, char range, char checksum, char overload, char instruction);
void CDS5516EnvoiPositionMin(char baud, char id, unsigned int positionMin);
void CDS5516EnvoiPositionMax(char baud, char id, unsigned int positionMax);
void CDS5516EnvoiPosistionCible(char baud, char id, unsigned int positionCible);
void CDS5516EnvoiVitesseMax(char baud, char id, unsigned int vitesseMax);
void CDS5516EnvoiLed(char baud, char id, char ledAllume);
void CDS5516EnvoiCoupleActive(char baud, char id, char enableTorque);
void CDS5516EnvoiBauderate(char baud, char id, char finalBauderate);
void CDS5516EnvoiId(char baud, char id, char nouvelId);
void CDS5516EnvoiCoupleMaximum(char baud, char id, unsigned int coupleMax);

#endif // __CDS5516_H__
