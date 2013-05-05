#include "Pilotage.h"
#include "asser.h"
#include "uart2.h"
#include <uart2.h>
#include <math.h>
#include "CDS5516.h"

// ATTENTION /!\ Ces fonctions ne doivent pas être bloquantes

extern unsigned int ADC_Results[8];
extern unsigned int position_buffer[6];
extern unsigned char buff_position_ptr,last_send_ptr;
extern long buff_position[N][256];
extern long raw_position[2];
double BAUDRATE, BRGVAL, FCY = 40000000;
extern double bridage;
extern double erreur[N];
extern double targ_pos[N];
extern double real_pos[N];
extern double cons_pos[N];
extern double etape_pos[5];
extern unsigned char nbr_etapes;
extern unsigned char num_etape;
extern double pwm_cor[N];
extern double feedforward;
extern unsigned int PID_ressource_used ;
extern double xydistance;
extern double xyangle;
extern double position_lock;
extern double pos_x;
extern double pos_y;
extern double pos_teta;
extern double offset_teta;

extern double erreur_allowed;

unsigned char scan;

//Extern Capteur Couleur
extern unsigned int Tab_Capteur_Couleur[8];

//Variable Extern pour turbine
unsigned int Periode_Turbine  = INIT_TURBINE;
unsigned int Periode_Canon    = INIT_CANON;

//Variable Servo Assiette
unsigned int Periode_Assiette = INIT_ASSIETTE;

//Variable Capteur Couleur
unsigned int Valeur_Capteur_Couleur = 24;
unsigned int Old_IC1Buf = 0;

unsigned int Cpt_Tmr_Periode = 0;
unsigned int Cpt_Tmr_Pwm_Turbine = 0;
unsigned int Cpt_Tmr_Pwm_Assiette = 0;

void delay(void)
{
    long i = 10; 
    while(i--);
}
void delayms(void) 
{
	long i = 1600000; //400ms
    while(i--);
}
//Delay seconde
void delays(void) 
{
	long i = 4000000; //seconde
    while(i--);
}

//Initialisation des servos moteurs selon les positions suivantes: 
//Aspirateur/Turbine : Desactive
//Canon : Desactive
void Init_Turbine(void)
{		
	Aspirateur_Vitesse(INIT_TURBINE);	
 	Canon_Vitesse(INIT_CANON);
	delays();
	delays();
}
//Initialisation des servos moteurs selon les positions suivantes: 
//Bras_Aspirateur : Retracte
//Bras_Debloqueur : Bas
//Assiette : Position Haut (0.5ms)
void Init_Servos(void)
{
	CDS5516Pos(19100,ID_SERVO_ASPIRATEUR,BRAS_RETRACTE);
	CDS5516Pos(19100,ID_SERVO_DEBLOQUEUR,DEBLOQUE_BAS);
	Assiette_Position(INIT_ASSIETTE);
}

//Initalisation Alimentation
void Init_Alimentation(void)
{
	LATAbits.LATA3 = 1;
}
//Initialisation de la pompe en mode desactive
void Init_Pompe(void)
{
	POMPE = POMPE_DESACTIVE;
}

//Fonction qui commande le shutter/Debloqueur 
//Etat pompe : 0 Desactive
//			   1 Active
void Shutter_Pos(unsigned char Pos)
{
	if(Pos)
		SHUTTER = SHUTTER_BLOQUE;
	else
		SHUTTER = SHUTTER_PAS_BLOQUE;
}

//Fonction qui commande la pompe à vide 
//Etat pompe : 0 Desactive
//			   1 Active
void Commande_Pompe(unsigned char Etat_Pompe)
{
	if(Etat_Pompe)
		POMPE = POMPE_ACTIVE;
	else
		POMPE = POMPE_DESACTIVE;
}

//Fonction qui renvoie sous forme de trame(UDP) la couleur de l'equipe
Trame Couleur_Equipe(void)
{
	Trame Etat_Couleur_Equipe;
	static BYTE Couleur[3];
	Etat_Couleur_Equipe.nbChar = 3;

	Couleur[0] = 0xC1;
	Couleur[1] = CMD_REPONSE_COULEUR_EQUIPE;
	Couleur[2] = PORTBbits.RB4;

	Etat_Couleur_Equipe.message = Couleur;
	
	return Etat_Couleur_Equipe;
}

//Fonction qui renvoie sous forme de trame(UDP) l'etat du switch Presence Assiette
Trame Presence_Assiette(void)
{
	Trame Etat_Presence_Assiette;
	static BYTE Presence[3];
	Etat_Presence_Assiette.nbChar = 3;

	Presence[0] = 0xC1;
	Presence[1] = CMD_REPONSE_PRESENCE_ASSIETTE;
	Presence[2] = !PORTBbits.RB6;

	Etat_Presence_Assiette.message = Presence;
	
	return Etat_Presence_Assiette;
}

//Fonction qui renvoie sous forme de trame(UDP) l'etat du switch Presence Aspirateur
Trame Presence_Aspirateur (void)
{
	Trame Etat_Presence_Aspirateur;
	static BYTE Presence[3];
	Etat_Presence_Aspirateur.nbChar = 3;

	Presence[0] = 0xC1;
	Presence[1] = CMD_REPONSE_PRESENCE_ASPIRATEUR;
	Presence[2] = !PORTBbits.RB9;

	Etat_Presence_Aspirateur.message = Presence;
	
	return Etat_Presence_Aspirateur;
}

//Fonction commande vitesse Aspirateur/Turbine
//Value range 1(0.5ms) <--> 4(2.0ms)
void Aspirateur_Vitesse(unsigned int vitesse)
{
	Periode_Turbine = vitesse;
}

//Fonction commande vitesse canon
//Value range 5000(1ms) <--> 10000(2ms) 
void Canon_Vitesse(unsigned int vitesse)
{
	Periode_Canon = vitesse;
}

//Function controls the position of assiette servo
//Value range 1(0.5ms) <--> 4(2.0ms)
void Assiette_Position(unsigned int vitesse)
{	
	Periode_Assiette = vitesse;
}

//Function generates PWM through Timer 2 ISR, induced every 1ms
void __attribute__((__interrupt__,__auto_psv__)) _T2Interrupt(void)
{
	Cpt_Tmr_Periode++;			
	Cpt_Tmr_Pwm_Turbine++;
	Cpt_Tmr_Pwm_Assiette++;
	
	if(Cpt_Tmr_Pwm_Turbine == Periode_Turbine)
	{
		SIGNAL_TURBINE = FALLING_EDGE;		
	}
		
	if(Cpt_Tmr_Pwm_Assiette == Periode_Assiette)
	{
		SIGNAL_ASSIETTE = FALLING_EDGE;		
	}
	if(Cpt_Tmr_Periode == CPT_PERIODE_20MS)
	{
		SIGNAL_ASSIETTE = RISING_EDGE;
		SIGNAL_TURBINE  = RISING_EDGE;
		SIGNAL_CANON    = ~PORTCbits.RC6;

		TMR5 = 0; 					
		PR5  = Periode_Canon; 			
		T5CONbits.TON = 1;

		Cpt_Tmr_Periode = 0;
		Cpt_Tmr_Pwm_Turbine = 0;
		Cpt_Tmr_Pwm_Assiette = 0;
	}
	IFS0bits.T2IF = 0; 		//Clear Timer1 Interrupt flag
}

//Interrupt induced every ~160us
void __attribute__((__interrupt__,__auto_psv__)) _T5Interrupt(void){
	
	T5CONbits.TON = 0;
	IFS1bits.T5IF = 0; 		//Clear Timer1 Interrupt flag

	SIGNAL_CANON = ~PORTCbits.RC6;
}

//Interruption Input Capture
//Interruption induced on every 16th Rising Edge
void __attribute__((__interrupt__,__auto_psv__)) _IC1Interrupt(void)
{
	unsigned int t1,t2;
	t2=IC1BUF;
	t1=IC1BUF;

	IFS0bits.IC1IF=0;

	if(t2>t1)
		Valeur_Capteur_Couleur = t2-t1;
	else
		Valeur_Capteur_Couleur = (PR3 - t1) + t2;
}



Trame PiloteGotoXY(int x,int y, unsigned char x_negatif, unsigned char y_negatif)
{
	//double x,y,teta;
	Trame trame;
	static BYTE tableau[6];
	trame.nbChar = 6;

	GotoXY((double)x,(double)y,0);
	
	tableau[0] = 1;
	tableau[1] = 0x13;
	tableau[2] = (int)xyangle>>8;
	tableau[3] = (int)xyangle&0x00FF;
	tableau[4] = (int)xydistance>>8;
	tableau[5] = (int)xydistance&0x00FF;
	
	trame.message = tableau;
	
	return trame;
	
}

void PilotPIDInit(void)
{
	InitProp();
}


Trame PilotePositionXYT()
{
	//double x,y,teta;
	Trame trame;
	static BYTE tableau[8];
	trame.nbChar = 8;
	
	tableau[0] = 0xC1;
	tableau[1] = CMD_RETOURPOSITION;
	tableau[2] = (int)(pos_x * 10)>>8;
	tableau[3] = (int)(pos_x * 10)&0x00FF;
	tableau[4] = (int)(pos_y * 10)>>8;
	tableau[5] = (int)(pos_y * 10)&0x00FF;
	tableau[6] = (unsigned int)(pos_teta*36000/(2*PI)+18000)>>8;
	tableau[7] = (unsigned int)(pos_teta*36000/(2*PI)+18000)&0x00FF;
	
	trame.message = tableau;
	
	return trame;
}

Trame PilotePIDRessource()
{
	Trame trame;
	static BYTE tableau[4];
	trame.nbChar = 4;
	
	tableau[0] = 1;
	tableau[1] = 0x66;
	tableau[2] = PID_ressource_used>>8;
	tableau[3] = PID_ressource_used&0x00FF;
	
	trame.message = tableau;
	
	return trame;
}
Trame Presence_Balle(void)
{
	Trame Etat_Presence_Balle;
	static BYTE Presence[3];
	Etat_Presence_Balle.nbChar = 3;

	Presence[0] = 0xC1;
	Presence[1] = CMD_REPONSE_PRESENCE_BALLE;
	Presence[2] = !PORTBbits.RB5;

	Etat_Presence_Balle.message = Presence;
	
	return Etat_Presence_Balle;
}

unsigned int Send_Variable_Capteur_Couleur(void){
	return Valeur_Capteur_Couleur;		
}

Trame Couleur_Balle(void)
{

	Trame Couleur_Balle;
	static BYTE Couleur[18];
	Couleur_Balle.nbChar = 18;

	Couleur[0] = 0xC1;
	Couleur[1] = CMD_REPONSE_COULEUR_BALLE;

	Couleur[2] = Tab_Capteur_Couleur[0]>>8;
	Couleur[3] = Tab_Capteur_Couleur[0]&0x00FF;

	Couleur[4] = Tab_Capteur_Couleur[1]>>8;
	Couleur[5] = Tab_Capteur_Couleur[1]&0x00FF;

	Couleur[6] = Tab_Capteur_Couleur[2]>>8;
	Couleur[7] = Tab_Capteur_Couleur[2]&0x00FF;

	Couleur[8] = Tab_Capteur_Couleur[3]>>8;
	Couleur[9] = Tab_Capteur_Couleur[3]&0x00FF;

	Couleur[10] = Tab_Capteur_Couleur[4]>>8;
	Couleur[11] = Tab_Capteur_Couleur[4]&0x00FF;

	Couleur[12] = Tab_Capteur_Couleur[5]>>8;
	Couleur[13] = Tab_Capteur_Couleur[5]&0x00FF;

	Couleur[14] = Tab_Capteur_Couleur[6]>>8;
	Couleur[15] = Tab_Capteur_Couleur[6]&0x00FF;

	Couleur[16] = Tab_Capteur_Couleur[7]>>8;
	Couleur[17] = Tab_Capteur_Couleur[7]&0x00FF;

	Couleur_Balle.message = Couleur;
	
	return Couleur_Balle;

}

void PilotePIDManual(unsigned int gauche,unsigned int droite)
{
	manual_pid((double)gauche,(double)droite);
}

void PilotePIDBridage(unsigned int value)
{
	bridage = (double)value;
}

void PilotePIDFeedforward(unsigned int value)
{
	feedforward = (double)value;
}

Trame PilotePIDErreurs()
{
	Trame trame;
	static BYTE tableau[10];
	trame.nbChar = 10;
	unsigned int data[4];
	
	tableau[0] = 1;
	tableau[1] = 0x48;
	
	data[0] = (unsigned int)fabs(pwm_cor[0]);
	data[1] = (unsigned int)fabs(pwm_cor[1]);
	data[2] = (unsigned int)fabs(real_pos[0]);
	data[3] = (unsigned int)fabs(real_pos[1]);


	tableau[2] = (int)pwm_cor[0]>>8;
	tableau[3] = (int)pwm_cor[0]&0x00FF;
	tableau[4] = (int)pwm_cor[1]>>8;
	tableau[5] = (int)pwm_cor[1]&0x00FF;
	tableau[6] = (int)raw_position[0]>>8;
	tableau[7] = (int)raw_position[0]&0x00FF;
	tableau[8] = (int)raw_position[1]>>8;
	tableau[9] = (int)raw_position[1]&0x00FF;
	
	trame.message = tableau;
	
	return trame;
}
void PilotePIDCoeffs(unsigned int new_kp, unsigned int new_ki, unsigned int new_kd)
{
	double coeffs[N*3]={DEFAULT_KP,DEFAULT_KI,DEFAULT_KD,DEFAULT_KP,DEFAULT_KI,DEFAULT_KD};
	coeffs[0] = (double)new_kp;
	coeffs[1] = (double)new_ki;
	coeffs[2] = (double)new_kd;
	coeffs[3] = (double)new_kp;
	coeffs[4] = (double)new_ki;
	coeffs[5] = (double)new_kd;
	
	set_pid(coeffs);
}
	

int Coupure(void)
{
	Stop(0); // FREELY
	return 0;
}

Trame PiloteGetPosition(unsigned char cote)
{
	Trame trame;
	static BYTE tableau[6];
	trame.nbChar = 6;
	double position;	

	tableau[0] = 1;
	tableau[1] = 0x42;
	tableau[2] = cote;
	
	switch(cote)
	{
		case 3:	position = Motors_GetPosition(0);
				break;
		case 4:	position = Motors_GetPosition(1);
				break;
		default:	break;
	}

	tableau[3] = ((unsigned int)fabs(position))>>8;
	tableau[4] = ((unsigned int)fabs(position))&0x00FF;
	if(position<0) 	tableau[5] = 1;
	else			tableau[5] = 0;
	trame.message = tableau;
	
	return trame;
}

Trame PiloteGetRawPosition()
{
	Trame trame;
	static BYTE tableau[15];
	trame.nbChar = 15;
	
	tableau[0] = 1;
	tableau[1] = 0x43;
	tableau[2] = 0;
	

	tableau[3] = position_buffer[0]>>8;
	tableau[4] = position_buffer[0]&0x00FF;
	tableau[5] = position_buffer[1]>>8;
	tableau[6] = position_buffer[1]&0x00FF;
	tableau[7] = position_buffer[2]>>8;
	tableau[8] = position_buffer[2]&0x00FF;
	tableau[9] = position_buffer[3]>>8;
	tableau[10] = position_buffer[3]&0x00FF;
	tableau[11] = position_buffer[4]>>8;
	tableau[12] = position_buffer[4]&0x00FF;
	tableau[13] = position_buffer[5]>>8;
	tableau[14] = position_buffer[5]&0x00FF;
	
	trame.message = tableau;
	
	return trame;
}

Trame PiloteGetLongPosition()
{
	Trame trame;
	static BYTE tableau[10];
	trame.nbChar = 10;
	
	tableau[0] = 1;
	tableau[1] = 0x44;

	tableau[2] = raw_position[0]>>24;
	tableau[3] = raw_position[0]>>16;
	tableau[4] = raw_position[0]>>8;
	tableau[5] = raw_position[0]&0x00FF;
	
	tableau[6] = raw_position[1]>>24;
	tableau[7] = raw_position[1]>>16;
	tableau[8] = raw_position[1]>>8;
	tableau[9] = raw_position[1]&0x00FF;
	
	trame.message = tableau;
	
	return trame;
}

Trame PiloteGetBuffPosition()
{
	Trame trame;
	static BYTE tableau[211];
	trame.nbChar = 102;
	unsigned char i,current_send_ptr,nbr_to_send;
	
	tableau[0] = 1;
	tableau[1] = 0x44;
	tableau[2] = buff_position_ptr;
	nbr_to_send = buff_position_ptr - last_send_ptr;
	last_send_ptr=buff_position_ptr;
	if(nbr_to_send>15) nbr_to_send=15;
	tableau[3] = nbr_to_send;
	trame.nbChar = nbr_to_send*8+4;
	//trame.nbChar = 18;
	//nbr_to_send=2;
	//current_send_ptr = buff_position_ptr;
	for(i=0;i<nbr_to_send;i++)
	{
		current_send_ptr = buff_position_ptr-nbr_to_send+i;
		tableau[2+2+(i*8)] = buff_position[0][current_send_ptr]>>24;
		tableau[2+3+(i*8)] = buff_position[0][current_send_ptr]>>16;
		tableau[2+4+(i*8)] = buff_position[0][current_send_ptr]>>8;
		tableau[2+5+(i*8)] = buff_position[0][current_send_ptr]&0x00FF;
		
		tableau[2+6+(i*8)] = buff_position[1][current_send_ptr]>>24;
		tableau[2+7+(i*8)] = buff_position[1][current_send_ptr]>>16;
		tableau[2+8+(i*8)] = buff_position[1][current_send_ptr]>>8;
		tableau[2+9+(i*8)] = buff_position[1][current_send_ptr]&0x00FF;
	}
	
	trame.message = tableau;
	
	return trame;
}

void PiloteAlimentation(char onOff)
{
	TRISAbits.TRISA3 = 0;
	LATAbits.LATA3 = onOff; 
}

void PiloteAlimentationCamera(char onOff)
{
	TRISAbits.TRISA0 = 0;
	LATAbits.LATA0 = onOff; 
}
int PiloteVitesse(int vitesse)
{
	Motors_SetSpeed(vitesse,MOTEUR_GAUCHE);
	Motors_SetSpeed(vitesse,MOTEUR_DROIT);
	return 1;
}

int PiloteAcceleration(int acceleration)
{
	Motors_SetAcceleration(acceleration,MOTEUR_GAUCHE);
	Motors_SetAcceleration(acceleration,MOTEUR_DROIT);
	return 1;
}

int PiloteAvancer(double distance)
{
	Avance(distance,0);
	return 1;
}

// Recule de la distance spécifiée
// distance : distance à reculer
int PiloteReculer(double distance)
{
	Avance(-distance,0);
	return 1;
}

// Pivote de l'angle spécifié
// angle : angle de rotation (en degrés)
// direction : coté du pivot (Gauche ou Droite)
int PilotePivoter(double angle, Cote direction)
{
	if(direction==Gauche)	Pivot( angle/100.0,0);
	else					Pivot(-angle/100.0,0);
	return 1;
}

// Effectuer un virage
// angle : angle de rotation (en degrés)
// rayon : rayon du virage
// direction : coté du virage (Gauche ou Droite)
int PiloteVirage(unsigned char reculer, unsigned char direction, double rayon, double angle)
{
	if(reculer) Virage(direction, rayon, angle/100, 0);
	else	 	Virage(direction, rayon, -angle/100, 0);

	return 1;
}

// Stoppe le robot
// mode : mode de stop (Abrupt, Smooth, Freely)
int PiloteStop(unsigned char stopmode)
{
	int distanceRestante;
	Trame envoiReste;
	static BYTE messReste[2];
	messReste[0] = 0xC1;
	messReste[1] = 0x60;
	envoiReste.nbChar = 4;
	
	distanceRestante = (int)Stop(stopmode);	

	messReste[2] = distanceRestante >> 8;
	messReste[3] = distanceRestante & 0xFF;
	
	envoiReste.message = messReste;

	while(Motors_IsRunning(MOTEUR_GAUCHE) || Motors_IsRunning(MOTEUR_DROIT));

	EnvoiUserUdp(envoiReste);
	
	return 1;
}

// Recallage du robot
// s : sens du recallage (Avant ou Arriere)
int PiloteRecallage(Sens s)
{	
	// TODO : Bah alors, c'est pas encore codé feignasse ?

	return 1;
}

// Lance une trajectoire par étapes
// nombreEtapes : nombre de points de passage
// etape : premiere étape du trajet
int PiloteAvancerEtapes(int nombreEtapes, Etape etape)
{
	// TODO : Bah alors, c'est pas encore codé feignasse ?

	return 1;
}

int PiloteOffsetAsserv(int x, int y, int teta) // marche pas
{
	pos_x = -y;
	pos_y = -x;
	offset_teta = (teta/180*PI) / 100.0 - pos_teta;

	return 1;
}

// Analyse la trame recue et renvoie vers la bonne fonction de pilotage
// Trame t : Trame ethernet recue
Trame AnalyseTrame(Trame t)
{
	Trame retour;
	unsigned int param1, param2, param3, param4;
	
	retour = t;

	// Les messages ne commencant pas par 0xC1 ne nous sont pas adressés (RecMove)
	if(t.message[0] != 0xC1)
		return t;

	switch(t.message[1])
	{
		case CMD_AVANCER:
			param1 = t.message[3] * 256 + t.message[4];		// Distance
			if(t.message[2])								// Sens
				PiloteAvancer(param1);
			else
				PiloteReculer(param1);
		break;

		case CMD_GOTOXY:
			param1 = t.message[2] * 256 + t.message[3];		// X
			param2 = t.message[4] * 256 + t.message[5];		// Y
			param3 = t.message[6];							// X positif
			param4 = t.message[7];							// Y positif
			retour = PiloteGotoXY(param1,param2,(unsigned char)param3,(unsigned char)param4);
		break;

		case CMD_RECALLAGE:
			Calage(t.message[2]);							// Sens
		break;

		case CMD_PIVOTER:
			param1 = t.message[3] * 256 + t.message[4];		// Angle
			param2 = t.message[2];							// Sens
			PilotePivoter(param1, (Cote)param2);
		break;

		case CMD_VIRAGE:
			param1 = t.message[4] * 256 + t.message[5];		// Rayon
			param2 = t.message[6] * 256 + t.message[7];		// Angle
			param3 = t.message[3];							// Direction
			param4 = t.message[2];							// Sens
			PiloteVirage((unsigned char)param4,(unsigned char)param3, (double)param1, (double)param2);
		break;

		case CMD_STOP:
			param1 = t.message[2];							// StopMode
			PiloteStop((unsigned char)param1);
		break;

		case CMD_VITESSE_PIVOT:
			param1 = t.message[2] * 256 + t.message[3];		// Vitesse
			Motors_SetSpeed_Pivot(param1);
		break;

		case CMD_VITESSE_LIGNE:
			param1 = t.message[2] * 256 + t.message[3];		// Vitesse
			Motors_SetSpeed_Ligne((double)param1);
		break;

		case CMD_ACCELERATION_PIVOT:
			param1 = t.message[2] * 256 + t.message[3];		// Accélération
			Motors_SetAcceleration_Pivot((double)param1);
		break;

		case CMD_ACCELERATION_LIGNE:
			param1 = t.message[2] * 256 + t.message[3];		// Accélération
			Motors_SetAcceleration_Ligne((double)param1);
		break;

		case CMD_ENVOI_PID:
			param1 = t.message[2]*256+t.message[3];			// P
			param2 = t.message[4]*256+t.message[5];			// I
			param3 = t.message[6]*256+t.message[7];			// D
			PilotePIDCoeffs(param1,param2,param3);
		break;

		case CMD_ECHO:
			retour = t;
		break;

		case CMD_ALIMENTATION:
			// Commande alimentation
			param1 = !t.message[2];							// On ou Off
			PiloteAlimentation(param1);
		break;

		case CMD_ALIMENTATION_CAMERA:
			// Commande alimentation
			param1 = t.message[2];							// On ou Off
			PiloteAlimentationCamera(param1);
		break;
			
		case CMD_DEMANDE_COULEUR_EQUIPE:
			// Interrupteur couleur Equipe
			retour = Couleur_Equipe();
			break;
			
		case CMD_DEMANDE_PRESENCE_ASSIETTE:
			// Switchs en série présence Assiette
			retour = Presence_Assiette();
			break;

		case CMD_DEMANDE_PRESENCE_ASPIRATEUR:
			// Interrupteur couleur Equipe
			retour = Presence_Aspirateur();
		break;

		case CMD_OFFSETASSERV:

			param1 = t.message[2]*256+t.message[3];			// X
			param2 = t.message[4]*256+t.message[5];			// Y
			param3 = t.message[6]*256+t.message[7];			// TETA
			PiloteOffsetAsserv(param1,param2,param3);

		break;

		case CMD_DEMANDEPOSITION: // Demande POS X Y TETA
			retour = PilotePositionXYT();
		break;

		case 0x90: //CMD_COUPURE:
			Coupure();
		break;

		case CMD_VITESSE_ASPIRATEUR:
			param1 = ((t.message[2]*256+t.message[3]) + 312);			
			Aspirateur_Vitesse(param1);
		break;

		case CMD_VITESSE_CANON:
			param1 = (t.message[2]*256+t.message[3]) + 5000;
			Canon_Vitesse(param1);
		break;

		case CMD_SHUTTER: 
			if(t.message[2])
				Shutter_Pos(SHUTTER_BLOQUE);
			else 
				Shutter_Pos(SHUTTER_PAS_BLOQUE);
		break;
	
		case CMD_SERVO_POSITION:
			if(t.message[2] == ID_SERVO_ASSIETTE)
			{
				Assiette_Position((t.message[3]*256+t.message[4])+312);
			}
			else 
			{
				param1 = t.message[2];						// Id servo
				param2 = t.message[3] * 256 + t.message[4];	// Position
				param3 = 19100;
				CDS5516Pos(param3,param1,param2);
			}
		break;	

		case CMD_SERVO_VITESSE:
			// Bouge servomoteur
			param1 = t.message[2];						// Id servo
			param2 = t.message[3] * 256 + t.message[4];	// Position
			param3 = 19100;
			CDS5516Vit(param3, param1,param2);
		break;

		case CMD_DEMANDE_PRESENCE_BALLE:
			return Presence_Balle();
		break;

		case CMD_DEMANDE_COULEUR:
			return Couleur_Balle();
		break;

		case CMD_POMPE_A_VIDE:
			Commande_Pompe(t.message[2]);
		break;

		case 0xF2:
			Reset();
		break;
	}
	return retour;
}
