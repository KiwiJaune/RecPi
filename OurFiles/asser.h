
#define N				2						// Nombre de moteurs
#define DEFAULT_KP		25						// Coefficient proporionnel 
#define DEFAULT_KI		0						// Coefficient integral (inverse !)
#define DEFAULT_KD		500						// Coefficient derive
#define CODEUR			1024 					// Nombre de pas par tour moteur (sans le ratio x4)
#define REDUCTEUR		1						// Reducteur utilise en sortie d'arbre moteur (=1 si roue codeuse indépendante)
#define DIAMETRE_ROUE 	35						// Diametre de la roue motrice (ou roue codeuse si indépendante) en mm 
#define PI 				3.1415926535897932384	// Ben pi quoi
#define VOIE			280*1.010200964882001*0.9458333333333336//1.01*1.00051*0.9995701848205272*1.000119*1.05726872246696						// Distance entre les deux roues en mm
#define COEFF_ROUE		1.083264470266402		// Coeff d'ajustement pour le diametre de la roue
#define COEFF_VOIE		1.01		// Coeff d'ajustement pour la voie
#define MM_SCALER		COEFF_ROUE*DIAMETRE_ROUE*PI/(4*CODEUR*REDUCTEUR) // Formule de conversion [pas]<==>[mm]
#define MM_INVSCALER	4*CODEUR*REDUCTEUR/(COEFF_ROUE*DIAMETRE_ROUE*PI)
#define DEFAULT_SPEED	700				// Vitesse par défaut en mm/s
#define DEFAULT_ACCEL	2500			// Acceleration par défaut en mm/s^2
#define ERROR_ALLOWED	0				// En cas de sifflement moteur intempestif (en pas)


#define DIRG 	LATBbits.LATB12
#define DIRD 	LATBbits.LATB14

#define STOP			0x01
#define VITESSE			0x02
#define ACCELERATION	0x03
#define AVANCE			0x04
#define PIVOT			0x05
#define VIRAGE			0x06
#define DISTANCE		0x08
#define ARRIVE			0x10
#define PIVOTG			0x11
#define PIVOTD			0x12
#define RECULE			0x13
#define COUPURE			0x90
#define COEFF_P			0x20
#define COEFF_I			0x21
#define COEFF_D			0x22
#define ROULEAU_AV		0x81
#define ROULEAU_AR		0x82

#define FALSE			0x00
#define TRUE			0x01


#define AVANT 			0			// Convention 
#define ARRIERE 		1
#define MOTEUR_GAUCHE 	0	// Convetion de merde !		
#define MOTEUR_DROIT 	1
#define GAUCHE 			2			
#define DROITE 			3
#define ON				1
#define OFF				0
#define FREELY			0
#define SMOOTH			1
#define ABRUPT			2

void InitProp(void);
void Avance(double distance, unsigned char wait);
void Pivot(double angle,unsigned char wait);
void Virage(unsigned char cote, double rayon, double angle, unsigned char wait);
double Stop(unsigned char stopmode);


double Motors_GetPosition(unsigned char moteur);
void Motors_GetRawPosition(unsigned int *buffer);

void Motors_DefineHome(unsigned char moteur);
void Motors_Stop(unsigned char stopmode, unsigned char moteur);

unsigned char Motors_IsRunning(unsigned char moteur);
void Motors_SetPosition(double position, unsigned char moteur);
void Motors_SetAcceleration(double acceleration, unsigned char moteur);

void Motors_SetAcceleration_Pivot(double acceleration);
void Motors_SetSpeed_Pivot(double vitesse);
void Motors_SetAcceleration_Ligne(double acceleration);
void Motors_SetSpeed_Ligne(double vitesse);

void Motors_SetSpeed(double vitesse, unsigned char moteur);
void Motors_Power(unsigned char power);
void Motors_Start(unsigned char moteur);


void GotoXY(double x, double y, unsigned char reculer);
unsigned char Motor_Task(void);
void set_pid(double * coeffs);
double pid(unsigned char power, double * targ_pos,double * real_pos);
char pwm(unsigned char motor, double valeur);
//double _abs(double value);
void Calage(unsigned char reculer);


void Sleepms(unsigned int nbr);
void Sleepus(unsigned int nbr);
