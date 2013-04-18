#include "Pilotage.h"
#include "asser.h"
#include "uart2.h"
#include <uart2.h>
#include <math.h>

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

extern double erreur_allowed;

unsigned char scan;

Trame PiloteGotoXY(int x,int y, unsigned char x_negatif, unsigned char y_negatif)
{
	//double x,y,teta;
	Trame trame;
	static BYTE tableau[6];
	trame.nbChar = 6;
	double xfinal,yfinal;
	
	/*if(x_negatif)	xfinal = (double)(-x);
	else			xfinal = (double)(x);
	if(y_negatif)	yfinal = (double)(-y);
	else			yfinal = (double)(y);*/
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
	tableau[1] = 0x67;
	tableau[2] = (int)pos_x>>8;
	tableau[3] = (int)pos_x&0x00FF;
	tableau[4] = (int)pos_y>>8;
	tableau[5] = (int)pos_y&0x00FF;
	tableau[6] = (int)(pos_teta*360/(2*PI))>>8;
	tableau[7] = (int)(pos_teta*360/(2*PI))&0x00FF;
	
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
	TRISBbits.TRISB0 = 0;
	LATBbits.LATB0 = onOff;
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
	if(direction==Gauche)	Pivot( angle,0);
	else					Pivot(-angle,0);
	return 1;
}

// Effectuer un virage
// angle : angle de rotation (en degrés)
// rayon : rayon du virage
// direction : coté du virage (Gauche ou Droite)
int PiloteVirage(unsigned char reculer, unsigned char direction, double rayon, double angle)
{
	if(reculer) Virage(direction, rayon, angle, 0);
	else	 	Virage(direction, rayon, -angle, 0);

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
	pos_teta = 0;
}

// Analyse la trame recue et renvoie vers la bonne fonction de pilotage
// Trame t : Trame ethernet recue
Trame AnalyseTrame(Trame t)
{
	Trame retour;
	int param1, param2, param3, param4;
	
	unsigned char i;
	retour = t;

	// Les messages ne commencant pas par 0xC1 ne nous sont pas adressés (RecMove)
	if(t.message[0] != 0xC1)
		return;

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
			param1 = t.message[2];							// On ou Off
			PiloteAlimentation(param1);
			break;

		// Commandes spéciales Pwet Debug


		case CMD_OFFSETASSERV:

			param1 = t.message[2]*256+t.message[3];			// X
			param2 = t.message[4]*256+t.message[5];			// Y
			param3 = t.message[6]*256+t.message[7];			// TETA
			PiloteOffsetAsserv(param1,param2,param3);

		break;

		case 0x37: // INIT
			PilotPIDInit();
		break;

		case 0x67: // POS X Y TETA
			retour = PilotePositionXYT();
		break;

		case 0x39: // PID RESSOURCE
			retour = PilotePIDRessource();
		break;
			
		case 0x41: // CMD_GET_POSITION
			param1 = t.message[2];
			retour = PiloteGetPosition(param1);
		break;

		case 0x42: // CMD_GET_RAW_POSITION
			retour = PiloteGetRawPosition();
		break;

		case 0x43: // CMD_GET_LONG_POSITION
			retour = PiloteGetLongPosition();
		break;

		case 0x44: // CMD_GET_BUFF_POSITION
			retour = PiloteGetBuffPosition();
		break;

		/*case 0x46:
			param1 = t.message[2]*256+t.message[3];
			param2 = t.message[4]*256+t.message[5];
			PilotePIDManual(param1,param2);
		break;*/

		case 0x47:
			param1 = t.message[2]*256+t.message[3];
			PilotePIDBridage(param1);
		break;

		case 0x48:
			retour = PilotePIDErreurs();
		break;

		case 0x49: 
			param1 = t.message[2]*256+t.message[3];
			feedforward = (double)param1;
		break;
		case 0x90: //CMD_COUPURE:
			Coupure();
		break;
		case 0xF2:
			Reset();
			break;


	}
	return retour;
}