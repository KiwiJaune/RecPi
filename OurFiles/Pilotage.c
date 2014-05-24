#include "Pilotage.h"
#include "asser.h"
#include "uart2.h"
#include <uart2.h>
#include <math.h>
#include "CDS5516.h"

// ATTENTION /!\ Ces fonctions ne doivent pas être bloquantes
extern unsigned char bridage;
extern unsigned int prd_envoi_position;
extern unsigned char jackAvant;
extern unsigned int capteur_vitesse;
extern unsigned char desactive_interrupt;
extern unsigned int cpt_capteur_vitesse;
extern double vitesse_canon;
extern unsigned int consigne_canon;
extern unsigned int ADC_Results[8];
extern unsigned int position_buffer[6];
extern unsigned char buff_position_ptr,last_send_ptr;
extern long buff_position[N][256];
extern unsigned char buff_status_ptr,last_send_status_ptr;
extern unsigned int buff_status[3][64];
extern long raw_position[2];
double BAUDRATE, BRGVAL, FCY = 40000000;
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
extern unsigned char flag_capteur_vitesse;

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

// DEBUG

Trame PiloteDebug0(Trame t)
{
	return t;
}

Trame PiloteDebug1(Trame t)
{
	return t;
}

Trame PiloteDebug2(Trame t)
{
	return t;
}

Trame PiloteDebug3(Trame t)
{
	return t;
}

Trame PiloteDebug4(Trame t)
{
	return t;
}

Trame PiloteDebug5(Trame t)
{
	return t;
}

Trame PiloteDebug6(Trame t)
{
	return t;
}

Trame PiloteDebug7(Trame t)
{
	return t;
}

Trame PiloteDebug8(Trame t)
{
	return t;
}

Trame PiloteDebug9(Trame t)
{
	return t;
}


//Initialisation des servos moteurs selon les positions suivantes: 
//Aspirateur/Turbine : Desactive
//Canon : Desactive
void Init_Turbine(void)
{		
	Aspirateur_Vitesse(INIT_TURBINE);	
 	Canon_Vitesse(INIT_CANON);
}
//Initialisation des servos moteurs selon les positions suivantes: 
//Bras_Aspirateur : Retracte
//Bras_Debloqueur : Bas
//Assiette : Position Haut (0.5ms)
void Init_Servos(void)
{
//	CDS5516EnvoiMessage(19100,ID_SERVO_ASPIRATEUR,BRAS_RETRACTE);
//	CDS5516EnvoiMessage(19100,ID_SERVO_DEBLOQUEUR,DEBLOQUE_BAS);
//	Assiette_Position(INIT_ASSIETTE);
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

	Couleur[0] = 0xC3;
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

	Presence[0] = 0xC3;
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

	Presence[0] = 0xC3;
	Presence[1] = CMD_REPONSE_PRESENCE_ASPIRATEUR;
	Presence[2] = !PORTBbits.RB9;

	Etat_Presence_Aspirateur.message = Presence;
	
	return Etat_Presence_Aspirateur;
}

Trame Presence_Jack(void)
{
	Trame Etat_Jack;
	static BYTE Jack[3];
	Etat_Jack.nbChar = 3;

	Jack[0] = 0xC3;
	Jack[1] = CMD_REPONSE_PRESENCE_JACK;
	Jack[2] = !PORTAbits.RA8;	

	Etat_Jack.message = Jack;

	return Etat_Jack;
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

//Function generates PWM through Timer 2 ISR, induced every 3.2us
void __attribute__((__interrupt__,__auto_psv__)) _T2Interrupt(void)
{
	Cpt_Tmr_Periode++;			
		
	if(cpt_capteur_vitesse<8000) // 8000 car 8000*8 < 65536
		cpt_capteur_vitesse++;

	if(flag_capteur_vitesse)
	{
		desactive_interrupt=3; 	// Desactive l'interuption du captuer vitesse pendant 3 ms (protection anti rebonds)
		flag_capteur_vitesse=0;	// ACK
		capteur_vitesse=cpt_capteur_vitesse; // Stock la vitesse (période)
		cpt_capteur_vitesse=0;	// Reinitialise le compteur soft
	}

	
	if(Cpt_Tmr_Periode == Periode_Turbine)
	{
		//SIGNAL_TURBINE = FALLING_EDGE;		
	}
		
	if(Cpt_Tmr_Periode == Periode_Assiette)
	{
		//SIGNAL_ASSIETTE = FALLING_EDGE;		
	}
	if(Cpt_Tmr_Periode == CPT_PERIODE_20MS)
	{
		//SIGNAL_ASSIETTE = RISING_EDGE;
		//SIGNAL_TURBINE  = RISING_EDGE;
		//SIGNAL_CANON    = RISING_EDGE;

		TMR5 = 0; 					
		PR5  = Periode_Canon; 			
		T5CONbits.TON = 1;

		Cpt_Tmr_Periode = 0;
	}
	IFS0bits.T2IF = 0; 		//Clear Timer1 Interrupt flag
}

//Interrupt induced every ~160us
void __attribute__((__interrupt__,__auto_psv__)) _T5Interrupt(void){
	
	T5CONbits.TON = 0;
	IFS1bits.T5IF = 0; 		//Clear Timer1 Interrupt flag

	//SIGNAL_CANON = FALLING_EDGE;
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

//Interruption Input Capture
//Interruption induced on every Falling Edge
void __attribute__((__interrupt__,__auto_psv__)) _IC2Interrupt(void)
{
	IEC0bits.IC2IE = 0;
	IFS0bits.IC2IF = 0;
	flag_capteur_vitesse = 1;
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

Trame StatusMonitor(void)
{
	Trame trame;
	static BYTE tableau[512];
	unsigned char i,current_send_ptr,nbr_to_send;
	
	tableau[0] = 0xC3; // identifiant trame
	tableau[1] = CMD_REPONSE_BUFF_STATUS;

	

	if(buff_status_ptr > last_send_status_ptr)
		nbr_to_send = buff_status_ptr - last_send_status_ptr;
	else
		nbr_to_send = 64 - last_send_status_ptr + buff_status_ptr;

	//nbr_to_send = (buff_status_ptr - last_send_status_ptr)%64;


	last_send_status_ptr=buff_status_ptr;
	if(nbr_to_send>35) nbr_to_send=35;
	tableau[2] = nbr_to_send;
	trame.nbChar = nbr_to_send*6+4;

	current_send_ptr = last_send_status_ptr + 1;

	for(i=0;i<nbr_to_send;i++)
	{
		tableau[1+2+(i*6)] = buff_status[0][current_send_ptr]>>8; // Status
		tableau[1+3+(i*6)] = buff_status[0][current_send_ptr]&0x00FF;		
		tableau[1+4+(i*6)] = buff_status[1][current_send_ptr]>>8; // PWM gauche
		tableau[1+5+(i*6)] = buff_status[1][current_send_ptr]&0x00FF;
		tableau[1+6+(i*6)] = buff_status[2][current_send_ptr]>>8; // PWM droite
		tableau[1+7+(i*6)] = buff_status[2][current_send_ptr]&0x00FF;
		current_send_ptr = (current_send_ptr + 1)%64;
	}
	
	trame.message = tableau;
	
	return trame;
}


void PilotePIDInit(void)
{
	InitProp();
}


Trame PilotePositionXYT()
{
	//double x,y,teta;
	Trame trame;
	static BYTE tableau[8];
	trame.nbChar = 8;
	
	tableau[0] = 0xC3;
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

Trame PiloteMesureCanon()
{
	Trame trame;
	static BYTE tableau[4];
	trame.nbChar = 4;
	
	tableau[0] = 0xC3;
	tableau[1] = CMD_REPONSE_MESURE_CANON;
	tableau[2] = (unsigned int)vitesse_canon>>8;
	tableau[3] = (unsigned int)vitesse_canon&0x00FF;
	
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

	Presence[0] = 0xC3;
	Presence[1] = CMD_REPONSE_PRESENCE_BALLE;
	Presence[2] = !PORTBbits.RB5;

	Etat_Presence_Balle.message = Presence;
	
	return Etat_Presence_Balle;
}

Trame ReponseEcho()
{
	BYTE vbat[6];
	static Trame trameEcho;
	static BYTE msgEcho[6];

	long bat1 = (float)(ADC_Results[5] * 0.012693 + 0.343777) * 100;
	long bat2 = (float)(ADC_Results[1] * 0.012693 + 0.343777) * 100;

	vbat[0] = bat1 >> 8; // Etalonnage de compétition !
	vbat[1] = bat1 & 0xFF;	
	vbat[2] = bat2 >> 8; // Etalonnage de compétition !
	vbat[3] = bat2 & 0xFF;				
	  	
	trameEcho.nbChar = 6;
	trameEcho.message = msgEcho;
	msgEcho[0] = 0xC3;
	msgEcho[1] = CMD_REPONSE_ECHO;
	msgEcho[2] = (BYTE) vbat[0];
	msgEcho[3] = (BYTE) vbat[1];
	msgEcho[4] = (BYTE) vbat[2];
	msgEcho[5] = (BYTE) vbat[3];
	return trameEcho;
}

unsigned int Send_Variable_Capteur_Couleur(void){
	return Valeur_Capteur_Couleur;		
}

Trame Couleur_Balle(void)
{

	Trame Couleur_Balle;
	static BYTE Couleur[18];
	Couleur_Balle.nbChar = 18;

	Couleur[0] = 0xC3;
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
	tableau[1] = CMD_DEMANDE_BUFF_POSITION;
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

// A faire : envoie consigne posiiton brut en pas codeur 2 parametres : [sens]8u ; [pascodeur]16u ;

Trame PiloteGetBuffPosition()
{
	Trame trame;
	static BYTE tableau[512];
	unsigned char i,current_send_ptr,nbr_to_send;
	
	tableau[0] = 0xC3; // identifiant trame
	tableau[1] = CMD_REPONSE_BUFF_POSITION;
	nbr_to_send = buff_position_ptr - last_send_ptr;
	last_send_ptr=buff_position_ptr;
	if(nbr_to_send>60) nbr_to_send=60;
	tableau[2] = nbr_to_send;
	trame.nbChar = nbr_to_send*8+4;
	for(i=0;i<nbr_to_send;i++)
	{
		current_send_ptr = buff_position_ptr-nbr_to_send+i;
		tableau[1+2+(i*8)] = buff_position[0][current_send_ptr]>>24;
		tableau[1+3+(i*8)] = buff_position[0][current_send_ptr]>>16;
		tableau[1+4+(i*8)] = buff_position[0][current_send_ptr]>>8;
		tableau[1+5+(i*8)] = buff_position[0][current_send_ptr]&0x00FF;
		
		tableau[1+6+(i*8)] = buff_position[1][current_send_ptr]>>24;
		tableau[1+7+(i*8)] = buff_position[1][current_send_ptr]>>16;
		tableau[1+8+(i*8)] = buff_position[1][current_send_ptr]>>8;
		tableau[1+9+(i*8)] = buff_position[1][current_send_ptr]&0x00FF;
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
	/*int distanceRestante;
	Trame envoiReste;
	static BYTE messReste[2];
	messReste[0] = 0xC3;
	messReste[1] = 0x60;
	envoiReste.nbChar = 4;
	*/
	Stop(stopmode);	

	//envoiReste.message = messReste;

	//while(Motors_IsRunning(MOTEUR_GAUCHE) || Motors_IsRunning(MOTEUR_DROIT));

	
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

int PiloteOffsetAsserv(int x, int y, int teta)
{
	double toto;
	pos_x = -y;
	pos_y = -x;
	toto = (((double)teta)/180*PI) / 100.0;
	offset_teta = toto - pos_teta + offset_teta;

	return 1;
}

//Fonctions servo
Trame PiloteServoDemandeBaudrate(char id)
{
	trameServo.nbChar = 5;
	msgServo[2] = id;
	msgServo[3] = CMD_SERVO_RETOUR_BAUDRATE;
	msgServo[4] = CDS5516DemandeBaudrate(id);
	return trameServo;
}

Trame PiloteServoDemandeConfigAlarmeLED(char id)
{
	int config = CDS5516DemandeConfigAlarmeLED(103, id);
	trameServo.nbChar = 11;
	msgServo[2] = CMD_SERVO_RETOUR_CFG_ALARME_LED;
	msgServo[3] = id;
	if(config & 0b01000000)
		msgServo[4] = 1;
	else
		msgServo[4] = 0;
	if(config & 0b00100000)
		msgServo[5] = 1;
	else
		msgServo[5] = 0;
	if(config & 0b00010000)
		msgServo[6] = 1;
	else
		msgServo[6] = 0;
	if(config & 0b00001000)
		msgServo[7] = 1;
	else
		msgServo[7] = 0;
	if(config & 0b00000100)
		msgServo[8] = 1;
	else
		msgServo[8] = 0;
	if(config & 0b00000010)
		msgServo[9] = 1;
	else
		msgServo[9] = 0;
	if(config & 0b00000001)
		msgServo[10] = 1;
	else
		msgServo[10] = 0;
		
	return trameServo;
}

Trame PiloteServoDemandeConfigAlarmeShutdown(char id)
{
	int config = CDS5516DemandeConfigAlarmeShutdown(103, id);
	trameServo.nbChar = 11;
	msgServo[2] = CMD_SERVO_RETOUR_CFG_ALARME_SHUTDOWN;
	msgServo[3] = id;
	if(config & 0b01000000)
		msgServo[4] = 1;
	else
		msgServo[4] = 0;
	if(config & 0b00100000)
		msgServo[5] = 1;
	else
		msgServo[5] = 0;
	if(config & 0b00010000)
		msgServo[6] = 1;
	else
		msgServo[6] = 0;
	if(config & 0b00001000)
		msgServo[7] = 1;
	else
		msgServo[7] = 0;
	if(config & 0b00000100)
		msgServo[8] = 1;
	else
		msgServo[8] = 0;
	if(config & 0b00000010)
		msgServo[9] = 1;
	else
		msgServo[9] = 0;
	if(config & 0b00000001)
		msgServo[10] = 1;
	else
		msgServo[10] = 0;
		
	return trameServo;
}

Trame PiloteServoDemandeConfigEcho(char id)
{
	trameServo.nbChar = 5;
	msgServo[2] = CMD_SERVO_RETOUR_CFG_ECHO;
	msgServo[3] = id;
	msgServo[4] = CDS5516DemandeConfigEcho(103, id);
	return trameServo;
}

Trame PiloteServoDemandeCompliance(char id)
{
	trameServo.nbChar = 8;
	msgServo[2] = CMD_SERVO_RETOUR_COMPLIANCE_PARAMS;
	msgServo[3] = id;
	msgServo[4] = CDS5516DemandeCCWSlope(103, id);
	msgServo[5] = CDS5516DemandeCCWMargin(103, id);
	msgServo[6] = CDS5516DemandeCWSlope(103, id);
	msgServo[7] = CDS5516DemandeCWMargin(103, id);
	return trameServo;
}

Trame PiloteServoDemandeCoupleActive(char id)
{
	trameServo.nbChar = 5;
	msgServo[2] = CMD_SERVO_RETOUR_COUPLE_ACTIVE;
	msgServo[3] = id;
	msgServo[4] = CDS5516DemandeCoupleActive(103, id);
	return trameServo;
}

Trame PiloteServoDemandeCoupleMaximum(char id)
{
	int valeur = CDS5516DemandeCoupleMaximum(103, id);
	trameServo.nbChar = 6;
	msgServo[2] = CMD_SERVO_RETOUR_COUPLE_MAX;
	msgServo[3] = id;
	msgServo[4] = valeur >> 8;
	msgServo[5] = valeur & 0xFF;
	return trameServo;
}

Trame PiloteServoPing(char id)
{
	int valeur=CDS5516Ping(103, id);
	trameServo.nbChar = 11;
	msgServo[2] = CMD_SERVO_RETOUR_ERREURS;
	msgServo[3] = id;
	msgServo[4] = (valeur>>1)&0x01;
	msgServo[5] = (valeur>>4)&0x01;
	msgServo[6] = (valeur)&0x01;
	msgServo[7] = (valeur>>6)&0x01;
	msgServo[8] = (valeur>>2)&0x01;
	msgServo[9] = (valeur>>5)&0x01;
	msgServo[10] = (valeur>>3)&0x01;
	
	return trameServo;
}

Trame PiloteServoTrouveID()
{
	trameServo.nbChar = 4;
	msgServo[2] = CMD_SERVO_RETOUR_ID;
	msgServo[3] = CDS5516DemandeID(103);
	return trameServo;
}

Trame PiloteServoDemandeLed(char id)
{
	trameServo.nbChar = 5;
	msgServo[2] = CMD_SERVO_RETOUR_LED;
	msgServo[3] = id;
	msgServo[4] = CDS5516DemandeLed(103, id);
	return trameServo;
}

Trame PiloteServoDemandeMouvement(char id)
{
	trameServo.nbChar = 5;
	msgServo[2] = CMD_SERVO_RETOUR_MOUVEMENT;
	msgServo[3] = id;
	msgServo[4] = CDS5516DemandeMouvement(103, id);
	return trameServo;
}

Trame PiloteServoDemandeModele(char id)
{
	int valeur = CDS5516DemandeModele(103, id);
	trameServo.nbChar = 6;
	msgServo[2] = CMD_SERVO_RETOUR_NUMERO_MODELE;
	msgServo[3] = id;
	msgServo[4] = valeur >> 8;
	msgServo[5] = valeur & 0xFF;
	return trameServo;
}

Trame PiloteServoDemandePositionActuelle(char id)
{
	int valeur = CDS5516DemandePositionActuelle(103, id);
	trameServo.nbChar = 6;
	msgServo[2] = CMD_SERVO_RETOUR_POSITION_ACTUELLE;
	msgServo[3] = id;
	msgServo[4] = valeur >> 8;
	msgServo[5] = valeur & 0xFF;
	return trameServo;
}

Trame PiloteServoDemandePositionCible(char id)
{
	int valeur = CDS5516DemandePositionCible(103, id);
	trameServo.nbChar = 6;
	msgServo[2] = CMD_SERVO_RETOUR_POSITION_CIBLE;
	msgServo[3] = id;
	msgServo[4] = valeur >> 8;
	msgServo[5] = valeur & 0xFF;
	return trameServo;
}

Trame PiloteServoDemandePositionMax(char id)
{
	int valeur = CDS5516DemandePositionMax(103, id);
	trameServo.nbChar = 6;
	msgServo[2] = CMD_SERVO_RETOUR_POSITION_MAX;
	msgServo[3] = id;
	msgServo[4] = (int)valeur >> 8;
	msgServo[5] = valeur & 0xFF;
	return trameServo;
}

Trame PiloteServoDemandePositionMin(char id)
{
	int valeur = CDS5516DemandePositionMin(103, id);
	trameServo.nbChar = 6;
	msgServo[2] = CMD_SERVO_RETOUR_POSITION_MIN;
	msgServo[3] = id;
	msgServo[4] = valeur >> 8;
	msgServo[5] = valeur & 0xFF;
	return trameServo;
}

Trame PiloteServoDemandeTemperature(char id)
{
	trameServo.nbChar = 5;
	msgServo[2] = CMD_SERVO_RETOUR_TEMPERATURE;
	msgServo[3] = id;
	msgServo[4] = CDS5516DemandeTemperature(103, id);
	return trameServo;
}

Trame PiloteServoDemandeTension(char id)
{
	trameServo.nbChar = 5;
	msgServo[2] = CMD_SERVO_RETOUR_TENSION;
	msgServo[3] = id;
	msgServo[4] = CDS5516DemandeTension(103, id);
	return trameServo;
}

Trame PiloteServoDemandeVersionFirmware(char id)
{
	trameServo.nbChar = 5;
	msgServo[2] = CMD_SERVO_RETOUR_VERSION_FIRMWARE;
	msgServo[3] = id;
	msgServo[4] = CDS5516DemandeVersionFirmware(103, id);
	return trameServo;
}

Trame PiloteServoDemandeVitesseActuelle(char id)
{
	int valeur = CDS5516DemandeVitesseActuelle(103, id);
	trameServo.nbChar = 6;
	msgServo[2] = CMD_SERVO_RETOUR_VITESSE_ACTUELLE;
	msgServo[3] = id;
	msgServo[4] = valeur >> 8;
	msgServo[5] = valeur & 0xFF;
	return trameServo;
}

Trame PiloteServoDemandeVitesseMax(char id)
{
	int valeur = CDS5516DemandeVitesseMax(103, id);
	trameServo.nbChar = 6;
	msgServo[2] = CMD_SERVO_RETOUR_VITESSE_MAX;
	msgServo[3] = id;
	msgServo[4] = valeur >> 8;
	msgServo[5] = valeur & 0xFF;
	return trameServo;
}

int PiloteServoEnvoiBauderate(char id, char baudrate)
{
	CDS5516EnvoiBauderate(103 , id, baudrate);
	return 1;
}

int PiloteServoEnvoiAlarmeLED(char id, char inputVoltage, char angleLimit, char overheating, char range, char checksum, char overload, char instruction)
{
	CDS5516EnvoiAlarmeLED(103 , id, inputVoltage, angleLimit, overheating, range, checksum, overload, instruction);
	return 1;
}

int PiloteServoEnvoiAlarmeShutdown(char id, char inputVoltage, char angleLimit, char overheating, char range, char checksum, char overload, char instruction)
{
	CDS5516EnvoiAlarmeShutdown(103, id, inputVoltage, angleLimit, overheating, range, checksum, overload, instruction);
	return 1;
}

int PiloteServoEnvoiComplianceParams(char id, char CCWSlope, char CCWMargin, char CWSlope, char CWMargin)
{
	CDS5516EnvoiComplianceParams(103, id, CCWSlope, CCWMargin, CWSlope, CWMargin);
	return 1;
}

int PiloteServoEnvoiCoupleActive(char id, char coupleActive)
{
	CDS5516EnvoiCoupleActive(103, id, coupleActive);
	return 1;
}

int PiloteServoEnvoiCoupleMaximum(char id, unsigned int coupleMax)
{
	CDS5516EnvoiCoupleMaximum(103, id, coupleMax);
	return 1;
}

int PiloteServoEnvoiId(char id, char nouvelId)
{
	CDS5516EnvoiId(103, id, nouvelId);
	return 1;
}

int PiloteServoEnvoiLed(char id, char ledAllume)
{
	CDS5516EnvoiLed(103, id, ledAllume);
	return 1;
}

int PiloteServoEnvoiPosistionCible(char id, unsigned int positionCible)
{
	CDS5516EnvoiPosistionCible(103, id, positionCible);
	return 1;
}

int PiloteServoEnvoiPosistionMax(char id, unsigned int positionMax)
{
	CDS5516EnvoiPositionMax(103, id, positionMax);
	return 1;
}

int PiloteServoEnvoiPosistionMin(char id, unsigned int positionMin)
{
	CDS5516EnvoiPositionMin(103, id, positionMin);
	return 1;
}

int PiloteServoEnvoiVitesseMax(char id, unsigned int vitesseMax)
{
	CDS5516EnvoiVitesseMax(103, id, vitesseMax);
	return 1;
}

int PiloteServoReset(char id)
{
	CDS5516Reset(103, id);
	return 1;
}

// Analyse la trame recue et renvoie vers la bonne fonction de pilotage
// Trame t : Trame ethernet recue
Trame AnalyseTrame(Trame t)
{
	Trame retour;
	unsigned int param1, param2, param3, param4, param5, param6, param7, param8;

	retour = t;
	
	// Les messages ne commencant pas par 0xC3 ne nous sont pas adressés (RecMove)
	if(t.message[0] != 0xC3)
		return t;

	switch(t.message[1])
	{
		case CMD_DEBUG:
			param1 = t.message[3];							// Numero
			switch(param1)
			{
				case 0:
					retour = PiloteDebug0(t);
					break;
				case 1:
					retour = PiloteDebug1(t);
					break;
				case 2:
					retour = PiloteDebug2(t);
					break;
				case 3:
					retour = PiloteDebug3(t);
					break;
				case 4:
					retour = PiloteDebug4(t);
					break;
				case 5:
					retour = PiloteDebug5(t);
					break;
				case 6:
					retour = PiloteDebug6(t);
					break;
				case 7:
					retour = PiloteDebug7(t);
					break;
				case 8:
					retour = PiloteDebug8(t);
					break;
				case 9:
					retour = PiloteDebug9(t);
					break;
			}
		break;
		
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
			bridage = t.message[2];
			retour = ReponseEcho();
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
	
		case CMD_SERVOMOTEUR:
	
				msgServo[0] = 0xC3;
				msgServo[1] = CMD_SERVOMOTEUR;
				trameServo.message = msgServo;
				
				//Addr doublon 14 devient 4 
				if(t.message[3] == 14)
					 t.message[3] = 4;
			
				switch(t.message[2])
				{
					case CMD_SERVO_DEMANDE_BAUDRATE:
						param1 = t.message[3];
						return PiloteServoDemandeBaudrate(param1);
						break; 
					case CMD_SERVO_DEMANDE_CFG_ALARME_LED:
						param1 = t.message[3];
						return PiloteServoDemandeConfigAlarmeLED(param1);
						break; 
					case CMD_SERVO_DEMANDE_CFG_ALARME_SHUTDOWN:
						param1 = t.message[3];
						return PiloteServoDemandeConfigAlarmeShutdown(param1);
						break; 
					case CMD_SERVO_DEMANDE_CFG_ECHO:
						param1 = t.message[3];
						return PiloteServoDemandeConfigEcho(param1);
						break; 
					case CMD_SERVO_DEMANDE_COMPLIANCE_PARAMS:
						param1 = t.message[3];
						return PiloteServoDemandeCompliance(param1);
						break; 
					case CMD_SERVO_DEMANDE_COUPLE_ACTIVE:
						param1 = t.message[3];
						return PiloteServoDemandeCoupleActive(param1);
						break; 
					case CMD_SERVO_DEMANDE_COUPLE_MAX:
						param1 = t.message[3];
						return PiloteServoDemandeCoupleMaximum(param1);
						break; 
					case CMD_SERVO_DEMANDE_ERREURS:
						param1 = t.message[3];
						return PiloteServoPing(param1);
						break; 
					case CMD_SERVO_DEMANDE_ID:
						return PiloteServoTrouveID();
						break;
					case CMD_SERVO_DEMANDE_LED:
						param1 = t.message[3];
						return PiloteServoDemandeLed(param1);
						break; 
					case CMD_SERVO_DEMANDE_MOUVEMENT:
						param1 = t.message[3];
						return PiloteServoDemandeMouvement(param1);
						break; 
					case CMD_SERVO_DEMANDE_NUMERO_MODELE:
						param1 = t.message[3];
						return PiloteServoDemandeModele(param1);
						break; 
					case CMD_SERVO_DEMANDE_POSITION_ACTUELLE:
						param1 = t.message[3];
						return PiloteServoDemandePositionActuelle(param1);
						break; 
					case CMD_SERVO_DEMANDE_POSITION_CIBLE:
						param1 = t.message[3];
						return PiloteServoDemandePositionCible(param1);
						break; 
					case CMD_SERVO_DEMANDE_POSITION_MAX:
						param1 = t.message[3];
						return PiloteServoDemandePositionMax(param1);
						break; 
					case CMD_SERVO_DEMANDE_POSITION_MIN:
						param1 = t.message[3];
						return PiloteServoDemandePositionMin(param1);
						break; 
					case CMD_SERVO_DEMANDE_TEMPTERATURE:
						param1 = t.message[3];
						return PiloteServoDemandeTemperature(param1);
						break; 
					case CMD_SERVO_DEMANDE_TENSION:
						param1 = t.message[3];
						return PiloteServoDemandeTension(param1);
						break; 
					case CMD_SERVO_DEMANDE_VERSION_FIRMWARE:
						param1 = t.message[3];
						return PiloteServoDemandeVersionFirmware(param1);
						break; 
					case CMD_SERVO_DEMANDE_VITESSE_ACTUELLE:
						param1 = t.message[3];
						return PiloteServoDemandeVitesseActuelle(param1);
						break; 
					case CMD_SERVO_DEMANDE_VITESSE_MAX:
						param1 = t.message[3];
						return PiloteServoDemandeVitesseMax(param1);
						break; 
					case CMD_SERVO_ENVOI_BAUDRATE:
						param1 = t.message[3];
						param2 = t.message[4];
						PiloteServoEnvoiBauderate(param1, param2);
						break;
					case CMD_SERVO_ENVOI_CFG_ALARME_LED:
						param1 = t.message[3];
						param2 = t.message[4];
						param3 = t.message[5];
						param4 = t.message[6];
						param5 = t.message[7];
						param6 = t.message[8];
						param7 = t.message[9];
						param8 = t.message[10];
						PiloteServoEnvoiAlarmeLED(param1, param2, param3, param4, param5, param6, param7, param8);
						break;
					case CMD_SERVO_ENVOI_CFG_ALARME_SHUTDOWN:
						param1 = t.message[3];
						param2 = t.message[4];
						param3 = t.message[5];
						param4 = t.message[6];
						param5 = t.message[7];
						param6 = t.message[8];
						param7 = t.message[9];
						param8 = t.message[10];
						PiloteServoEnvoiAlarmeShutdown(param1, param2, param3, param4, param5, param6, param7, param8);
						break;
					case CMD_SERVO_ENVOI_CFG_ECHO:
						// TODO
						break;
					case CMD_SERVO_ENVOI_COMPLIANCE_PARAMS:
						param1 = t.message[3];
						param2 = t.message[4];
						param3 = t.message[5];
						param4 = t.message[6];
						param5 = t.message[7];
						PiloteServoEnvoiComplianceParams(param1, param2, param3, param4, param5);
						break;
					case CMD_SERVO_ENVOI_COUPLE_ACTIVE:
						param1 = t.message[3];
						param2 = t.message[4];
						PiloteServoEnvoiCoupleActive(param1, param2);
						break;
					case CMD_SERVO_ENVOI_COUPLE_MAX:
						param1 = t.message[3];
						param2 = t.message[4] * 256 + t.message[5];
						PiloteServoEnvoiCoupleMaximum(param1, param2);
						break;
					case CMD_SERVO_ENVOI_ID:
						param1 = t.message[3];
						param2 = t.message[4];
						PiloteServoEnvoiId(param1, param2);
						break;
					case CMD_SERVO_ENVOI_LED:
						param1 = t.message[3];
						param2 = t.message[4];
						PiloteServoEnvoiLed(param1, param2);
						break;
					case CMD_SERVO_ENVOI_POSITION_CIBLE:
						param1 = t.message[3];
						param2 = t.message[4] * 256 + t.message[5];
						PiloteServoEnvoiPosistionCible(param1, param2);
						break;
					case CMD_SERVO_ENVOI_POSITION_MAX:
						param1 = t.message[3];
						param2 = t.message[4] * 256 + t.message[5];
						PiloteServoEnvoiPosistionMax(param1, param2);
						break;
					case CMD_SERVO_ENVOI_POSITION_MIN:
						param1 = t.message[3];
						param2 = t.message[4] * 256 + t.message[5];
						PiloteServoEnvoiPosistionMin(param1, param2);
						break;
					case CMD_SERVO_ENVOI_VITESSE_MAX:
						param1 = t.message[3];
						param2 = t.message[4] * 256 + t.message[5];
						PiloteServoEnvoiVitesseMax(param1, param2);
						break;
					case CMD_SERVO_RESET:
						param1 = t.message[3];
						PiloteServoReset(param1);
						break;
				}
//		case CMD_DEMANDE_PRESENCE_BALLE:
//			return Presence_Balle();
//		break;
//
//		case CMD_DEMANDE_COULEUR:
//			return Couleur_Balle();
//		break;
//
//		case CMD_POMPE_A_VIDE:
//			Commande_Pompe(t.message[2]);
//		break;
//
//		case CMD_DEMANDE_MESURE_CANON:	
//			retour = PiloteMesureCanon();
//		break;
//		
//		case CMD_CONSIGNE_CANON:
//			consigne_canon= t.message[2] * 256 + t.message[3];
//			if(consigne_canon==0) Canon_Vitesse(5000);
//		break;
		
		case CMD_RESET_CARTE:
			Reset();
		break;
//		case CMD_ARME_JACK:
//			jackAvant=1;
//		break;
//		case CMD_DEMANDE_PRESENCE_JACK:
//			return Presence_Jack();
//		break;
		case CMD_CONSIGNE_POSITION:
			if(t.message[2] == AVANT)
			{
				cons_pos[0] += MM_SCALER * (t.message[3] * 256 + t.message[4]);
				cons_pos[1] += MM_SCALER * (t.message[3] * 256 + t.message[4]);
			}
			else
			{
				cons_pos[0] -= MM_SCALER * (t.message[3] * 256 + t.message[4]);
				cons_pos[1] -= MM_SCALER * (t.message[3] * 256 + t.message[4]);
			}
			break;
		case CMD_DEMANDE_BUFF_POSITION:
			return PiloteGetBuffPosition();
			break;
		case CMD_DEMANDE_BUFF_STATUS:
			return StatusMonitor();
			break;
		case CMD_PRD_ENVOI_POSITION:
			prd_envoi_position = 10*(unsigned int)t.message[2];
			break;
	}
	return retour;
}
