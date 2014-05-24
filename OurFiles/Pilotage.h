#ifndef __PILOTAGE_H__
#define __PILOTAGE_H__

#include "Types.h"
#include "UserUdp.h"


Trame PiloteGotoXY(int x, int y, unsigned char x_negatif, unsigned char y_negatif);
Trame ReponseEcho(void);
Trame StatusMonitor(void);

Trame PilotePIDRessource();
void PilotePIDInit(void);
Trame PiloteGetPosition(unsigned char cote);
Trame PiloteGetRawPosition(void);
Trame PiloteGetLongPosition(void);
Trame PiloteGetBuffPosition(void);
void PilotePIDCoeffs(unsigned int new_kp, unsigned int new_ki, unsigned int new_kd);
void PilotePIDManual(unsigned int gauche,unsigned int droite);
Trame PilotePIDErreurs(void);
void PilotePIDFeedforward(unsigned int value);
void Assiette_Position(unsigned int vitesse);

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
Trame PiloteMesureCanon();

int Coupure(void);

// Constantes des fonctions des actionneurs
#define ON  1
#define OFF 0

#define BRAS_DEPLOYE 145
#define BRAS_RETRACTE 280

#define DEBLOQUE_BAS 550
#define DEBLOQUE_HAUT 250

#define INIT_TURBINE 312			
#define INIT_CANON 5000
#define INIT_ASSIETTE 312

#define CPT_PERIODE_20MS 6250

#define CANON_ACTIVE 5500
#define CANON_DESACTIVE 5000
#define CANON_ACTIVE_MAX 10000

#define ASPIRATION_ACTIVE 625	
#define ASPIRATION_DESACTIVE 312	

#define ID_SERVO_ASPIRATEUR 16 	//0
#define ID_SERVO_ASSIETTE 6  	//16 ou 2
#define ID_SERVO_DEBLOQUEUR 0  	//16 ou 2
#define ALL_SERVO 254

#define SHUTTER_BLOQUE 0
#define SHUTTER_PAS_BLOQUE 1
#define SHUTTER LATAbits.LATA1

#define SIGNAL_TURBINE LATCbits.LATC7
#define SIGNAL_CANON LATCbits.LATC6
#define SIGNAL_ASSIETTE LATBbits.LATB11

#define POMPE_ACTIVE 1
#define POMPE_DESACTIVE 0
#define POMPE LATBbits.LATB10

#define RISING_EDGE 1
#define FALLING_EDGE 0 


//Fonction de test des actionneurs
void Init_Turbine(void);
void Init_Servos(void);
void Init_Pompe(void);

void Canon_Vitesse(unsigned int vitesse);
void Aspirateur_Vitesse(unsigned int vitesse);
void Assiette_Position(unsigned int vitesse);

void Commande_Pompe(unsigned char Etat_Pompe);
unsigned int Send_Variable_Capteur_Couleur(void);

//Initialisation timer 3,4 et 5
void Init_Timer (void);

//Initialisation Alimentation
void Init_Alimentation(void);

//Delay
void delay(void);
void delays(void);
void delayms(void);
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

//Trame servo
static Trame trameServo;
static BYTE msgServo[20];

// SERVOMOTEURS
Trame PiloteServoDemandeBaudrate(char id);
Trame PiloteServoDemandeConfigAlarmeLED(char id);
Trame PiloteServoDemandeConfigAlarmeShutdown(char id);
Trame PiloteServoDemandeConfigEcho(char id);
Trame PiloteServoDemandeCompliance(char id);
Trame PiloteServoDemandeCoupleActive(char id);
Trame PiloteServoDemandeCoupleMaximum(char id);
Trame PiloteServoPing(char id);
Trame PiloteServoTrouveID();
Trame PiloteServoDemandeLed(char id);
Trame PiloteServoDemandeMouvement(char id);
Trame PiloteServoDemandeModele(char id);
Trame PiloteServoDemandePositionActuelle(char id);
Trame PiloteServoDemandePositionCible(char id);
Trame PiloteServoDemandePositionMax(char id);
Trame PiloteServoDemandePositionMin(char id);
Trame PiloteServoDemandeTemperature(char id);
Trame PiloteServoDemandeTension(char id);
Trame PiloteServoDemandeVersionFirmware(char id);
Trame PiloteServoDemandeVitesseActuelle(char id);
Trame PiloteServoDemandeVitesseMax(char id);
int PiloteServoEnvoiBauderate(char id, char baudrate);
int PiloteServoEnvoiAlarmeLED(char id, char inputVoltage, char angleLimit, char overheating, char range, char checksum, char overload, char instruction);
int PiloteServoEnvoiAlarmeShutdown(char id, char inputVoltage, char angleLimit, char overheating, char range, char checksum, char overload, char instruction);
int PiloteServoEnvoiComplianceParams(char id, char CCWSlope, char CCWMargin, char CWSlope, char CWMargin);
int PiloteServoEnvoiCoupleActive(char id, char coupleActive);
int PiloteServoEnvoiCoupleMaximum(char id, unsigned int coupleMax);
int PiloteServoEnvoiId(char id, char nouvelId);
int PiloteServoEnvoiLed(char id, char ledAllume);
int PiloteServoEnvoiPosistionCible(char id, unsigned int positionCible);
int PiloteServoEnvoiPosistionMax(char id, unsigned int positionMax);
int PiloteServoEnvoiPosistionMin(char id, unsigned int positionMin);
int PiloteServoEnvoiVitesseMax(char id, unsigned int vitesseMax);
int PiloteServoReset(char id);

// DEBUG
Trame PiloteDebug0(Trame t);
Trame PiloteDebug1(Trame t);
Trame PiloteDebug2(Trame t);
Trame PiloteDebug3(Trame t);
Trame PiloteDebug4(Trame t);
Trame PiloteDebug5(Trame t);
Trame PiloteDebug6(Trame t);
Trame PiloteDebug7(Trame t);
Trame PiloteDebug8(Trame t);
Trame PiloteDebug9(Trame t);

//Analyse Trame
Trame AnalyseTrame(Trame t);

#endif // __PILOTAGE_H__

// Deplacements
#define	CMD_AVANCER						0x01
#define	CMD_PIVOTER						0x03
#define	CMD_VIRAGE						0x04
#define	CMD_STOP						0x05
#define	CMD_GOTOXY						0x06
#define	CMD_RECALLAGE					0x10
#define CMD_FINRECALLAGE				0x11
#define CMD_FINDEPLACEMENT				0x12

// Asservissement
#define CMD_DEMANDEPOSITION				0x30
#define CMD_RETOURPOSITION				0x31
#define	CMD_VITESSE_LIGNE				0x32
#define	CMD_ACCELERATION_LIGNE			0x33
#define	CMD_VITESSE_PIVOT				0x34
#define	CMD_ACCELERATION_PIVOT			0x35
#define CMD_ENVOI_PID 					0x36
#define CMD_OFFSETASSERV				0x37

// Debug asservissement
#define CMD_DEMANDE_BUFF_POSITION		0x43
#define CMD_REPONSE_BUFF_POSITION		0x44
#define CMD_CONSIGNE_POSITION			0x45
#define CMD_DEMANDE_BUFF_STATUS			0x46
#define CMD_REPONSE_BUFF_STATUS			0x47
#define CMD_PRD_ENVOI_POSITION			0x48

// Actionneurs
#define CMD_VITESSE_ASPIRATEUR      	0x53
#define CMD_VITESSE_CANON           	0x54
#define CMD_SHUTTER                 	0x55
#define CMD_POMPE_A_VIDE				0x56
#define CMD_CONSIGNE_CANON				0x57

// Autre
#define CMD_SERVOMOTEUR					0x60

// Servomoteurs
#define CMD_SERVO_DEMANDE_POSITION_CIBLE		0x01
#define CMD_SERVO_RETOUR_POSITION_CIBLE			0x02
#define CMD_SERVO_ENVOI_POSITION_CIBLE			0x03
#define CMD_SERVO_DEMANDE_BAUDRATE				0x04
#define CMD_SERVO_RETOUR_BAUDRATE				0x05
#define CMD_SERVO_ENVOI_BAUDRATE				0x06
#define CMD_SERVO_DEMANDE_VITESSE_MAX			0x07
#define CMD_SERVO_RETOUR_VITESSE_MAX			0x08
#define CMD_SERVO_ENVOI_VITESSE_MAX				0x09
#define CMD_SERVO_DEMANDE_ID					0x10
#define CMD_SERVO_RETOUR_ID						0x11
#define CMD_SERVO_ENVOI_ID						0x12
#define CMD_SERVO_RESET							0x13
#define CMD_SERVO_DEMANDE_COUPLE_MAX			0x14
#define CMD_SERVO_RETOUR_COUPLE_MAX				0x15
#define CMD_SERVO_ENVOI_COUPLE_MAX				0x16
#define CMD_SERVO_DEMANDE_COUPLE_ACTIVE			0x17
#define CMD_SERVO_RETOUR_COUPLE_ACTIVE			0x18
#define CMD_SERVO_ENVOI_COUPLE_ACTIVE			0x19
#define CMD_SERVO_DEMANDE_TENSION				0x20
#define CMD_SERVO_RETOUR_TENSION				0x21
#define CMD_SERVO_DEMANDE_TEMPTERATURE			0x22
#define CMD_SERVO_RETOUR_TEMPERATURE			0x23
#define CMD_SERVO_DEMANDE_MOUVEMENT				0x24
#define CMD_SERVO_RETOUR_MOUVEMENT				0x25
#define CMD_SERVO_DEMANDE_POSITION_MIN			0x26
#define CMD_SERVO_RETOUR_POSITION_MIN			0x27
#define CMD_SERVO_ENVOI_POSITION_MIN			0x28
#define CMD_SERVO_DEMANDE_POSITION_MAX			0x29
#define CMD_SERVO_RETOUR_POSITION_MAX			0x30
#define CMD_SERVO_ENVOI_POSITION_MAX			0x31
#define CMD_SERVO_DEMANDE_NUMERO_MODELE			0x32
#define CMD_SERVO_RETOUR_NUMERO_MODELE			0x33
#define CMD_SERVO_DEMANDE_VERSION_FIRMWARE		0x34
#define CMD_SERVO_RETOUR_VERSION_FIRMWARE		0x35
#define CMD_SERVO_DEMANDE_LED					0x36
#define CMD_SERVO_RETOUR_LED					0x37
#define CMD_SERVO_ENVOI_LED						0x38
#define CMD_SERVO_DEMANDE_CFG_ALARME_LED		0x42
#define CMD_SERVO_RETOUR_CFG_ALARME_LED			0x43
#define CMD_SERVO_ENVOI_CFG_ALARME_LED			0x44
#define CMD_SERVO_DEMANDE_CFG_ALARME_SHUTDOWN	0x45
#define CMD_SERVO_RETOUR_CFG_ALARME_SHUTDOWN	0x46
#define CMD_SERVO_ENVOI_CFG_ALARME_SHUTDOWN		0x47
#define CMD_SERVO_DEMANDE_CFG_ECHO				0x48
#define CMD_SERVO_RETOUR_CFG_ECHO				0x49
#define CMD_SERVO_ENVOI_CFG_ECHO				0x50
#define CMD_SERVO_DEMANDE_COMPLIANCE_PARAMS		0x51
#define CMD_SERVO_RETOUR_COMPLIANCE_PARAMS		0x52
#define CMD_SERVO_ENVOI_COMPLIANCE_PARAMS		0x53
#define CMD_SERVO_DEMANDE_POSITION_ACTUELLE		0x54
#define CMD_SERVO_RETOUR_POSITION_ACTUELLE		0x55
#define CMD_SERVO_DEMANDE_VITESSE_ACTUELLE		0x56
#define CMD_SERVO_RETOUR_VITESSE_ACTUELLE		0x57
#define CMD_SERVO_DEMANDE_ERREURS				0x58
#define CMD_SERVO_RETOUR_ERREURS				0x59

// Capteurs
#define CMD_DEPART_JACK					0x71	
#define CMD_DEMANDE_COULEUR_EQUIPE		0x72
#define CMD_REPONSE_COULEUR_EQUIPE		0x73
#define CMD_DEMANDE_COULEUR				0x75
#define CMD_REPONSE_COULEUR_BALLE   	0x76
#define CMD_DEMANDE_PRESENCE_BALLE		0x77
#define CMD_REPONSE_PRESENCE_BALLE		0x78
#define CMD_DEMANDE_PRESENCE_ASSIETTE	0x7A
#define CMD_REPONSE_PRESENCE_ASSIETTE	0x7B
#define CMD_DEMANDE_PRESENCE_ASPIRATEUR	0x7C
#define CMD_REPONSE_PRESENCE_ASPIRATEUR	0x7D
#define CMD_DEMANDE_MESURE_CANON		0x7E
#define CMD_REPONSE_MESURE_CANON		0x7F
#define CMD_ARME_JACK					0x70

// Alimentation
#define CMD_ALIMENTATION		0x80
#define CMD_ALIMENTATION_CAMERA 0x81	

// Diagnostic
#define	CMD_DEBUG						0xEE
#define	CMD_ECHO						0xF0
#define	CMD_RESET_CARTE					0xF1
#define CMD_DEMANDE_PRESENCE_JACK		0xF3
#define CMD_REPONSE_PRESENCE_JACK		0xF4
#define CMD_REPONSE_ECHO				0xF5
