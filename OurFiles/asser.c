#include <p33FJ128MC804.h>
#include "asser.h"
#include <math.h>

double kp[N],ki[N],kd[N];
int revolutions[N];
unsigned char kd_cancel;
unsigned int pid_count;
double cons_pos[N]={0};
long raw_position[N];
long buff_position[N][256];
unsigned char buff_position_ptr=0,last_send_ptr=0;
double real_pos[N];
double targ_pos[N];
double erreur_allowed=2;
double erreur[N],pwm_cor[N];
double feedforward=0;//825;
unsigned int position_buffer[6];
double bridage=0,xydistance,xyangle;
unsigned char xymotion=0;
double pos_x,pos_y,pos_teta, offset_teta = 0;
double ratio;
unsigned char moteur1, moteur2,motiontype=0; // motiontype : 0: Avance 1: Pivot 2: Virage 3: Calage
unsigned char cpt_calage[N];

double etape_pos[5];
unsigned char nbr_etapes,num_etape;


double speed_def[N]={DEFAULT_SPEED};
double accel_def[N]={DEFAULT_ACCEL};
double speed_def_ligne=DEFAULT_SPEED;
double speed_def_pivot=DEFAULT_SPEED;
double accel_def_ligne=DEFAULT_ACCEL;
double accel_def_pivot=DEFAULT_ACCEL;

unsigned char pid_power=0,motion[N]={0},flagdefinehome;

/*************************** Couche utilisateur ***************************/

void manual_pid(double moteurga, double moteurdr)
{
	cons_pos[0] = moteurga;
	cons_pos[1] = moteurdr;
}

void InitProp(void)
{
	double coeffs[N*3]={DEFAULT_KP,DEFAULT_KI,DEFAULT_KD,DEFAULT_KP,DEFAULT_KI,DEFAULT_KD};
	Motors_Power(OFF);
	set_pid(coeffs);
	Motors_DefineHome(MOTEUR_GAUCHE);
	Motors_DefineHome(MOTEUR_DROIT);
	Motors_SetAcceleration(DEFAULT_ACCEL,MOTEUR_GAUCHE);
	Motors_SetAcceleration(DEFAULT_ACCEL,MOTEUR_DROIT);
	Motors_SetSpeed(DEFAULT_SPEED,MOTEUR_GAUCHE);
	Motors_SetSpeed(DEFAULT_SPEED,MOTEUR_DROIT);
	accel_def_ligne = DEFAULT_ACCEL;
	speed_def_ligne = DEFAULT_SPEED;
	accel_def_pivot = DEFAULT_ACCEL;
	speed_def_pivot = DEFAULT_SPEED;
	pos_x=0;
	pos_y=0;
	pos_teta=0;
	Motors_Power(ON);
}

void Avance(double distance, unsigned char wait)
{
	motiontype = 0; // Avance
	Motors_SetAcceleration(accel_def_ligne, MOTEUR_GAUCHE);  
	Motors_SetAcceleration(accel_def_ligne, MOTEUR_DROIT);
	
	Motors_SetSpeed(speed_def_ligne, MOTEUR_GAUCHE); 
	Motors_SetSpeed(speed_def_ligne, MOTEUR_DROIT);
	
	Motors_SetPosition(distance, MOTEUR_GAUCHE);
	Motors_SetPosition(distance, MOTEUR_DROIT);
	
	Motors_Start(MOTEUR_GAUCHE);
	Motors_Start(MOTEUR_DROIT);

	if(wait) 
	{
		while(Motors_IsRunning(MOTEUR_GAUCHE) || Motors_IsRunning(MOTEUR_DROIT));
		Sleepms(100);
	}

}

void Calage(unsigned char reculer)
{
	Motors_SetAcceleration(accel_def_ligne, MOTEUR_GAUCHE);  
	Motors_SetAcceleration(accel_def_ligne, MOTEUR_DROIT);
	
	Motors_SetSpeed(speed_def_ligne, MOTEUR_GAUCHE); 
	Motors_SetSpeed(speed_def_ligne, MOTEUR_DROIT);
	
	if(reculer)
	{
		Motors_SetPosition(3000, MOTEUR_GAUCHE);
		Motors_SetPosition(3000, MOTEUR_DROIT);
	}
	else
	{
		Motors_SetPosition(-3000, MOTEUR_GAUCHE);
		Motors_SetPosition(-3000, MOTEUR_DROIT);
	}

	
	Motors_Start(MOTEUR_GAUCHE);
	Motors_Start(MOTEUR_DROIT);
	motiontype = 4; // Fake
	Sleepms(400);
//	cpt_calage[0]=0;
//	cpt_calage[1]=0;
	motiontype = 3; // Calage
	
}

void GotoXY(double x, double y, unsigned char reculer)
{
	double dx,dy,angle;

	dx = x-pos_x;
	dy = y-pos_y;

	//angle = atan2f(dx,dy)+pos_teta; // = arctan(dx/dy) + pos_teta
	angle = atan(dx/dy)+pos_teta; // = arctan(dx/dy) + pos_teta
	angle = -angle;
	if(dy<0)
	{
		if(dx>0)	angle += PI;
		else		angle -= PI;
	}	
	if(angle> PI) angle -=2*PI;
	if(angle<-PI) angle +=2*PI;	

	xydistance = fabs(sqrt(pow(dx,2)+pow(dy,2)));
	if(xydistance<0) while(1);
	xyangle = angle*180/PI;
	xymotion=1;
	//Pivot(angle,0);
}

void Pivot(double angle,unsigned char wait)
{
	motiontype = 1;
	Motors_SetAcceleration(accel_def_pivot, MOTEUR_GAUCHE);  
	Motors_SetAcceleration(accel_def_pivot, MOTEUR_DROIT);
	
	Motors_SetSpeed(speed_def_pivot, MOTEUR_GAUCHE); 
	Motors_SetSpeed(speed_def_pivot, MOTEUR_DROIT);
	
	Motors_SetPosition(-angle*(PI/180.0)*VOIE/2.0, MOTEUR_GAUCHE);
	Motors_SetPosition( angle*(PI/180.0)*VOIE/2.0, MOTEUR_DROIT);
	
	Motors_Start(MOTEUR_GAUCHE);
	Motors_Start(MOTEUR_DROIT);
	
	if(wait) 
	{
		while(Motors_IsRunning(MOTEUR_GAUCHE) || Motors_IsRunning(MOTEUR_DROIT));
		Sleepms(100);
	}
}

void Virage(unsigned char cote, double rayon, double angle, unsigned char wait)
{
	motiontype = 2;
	ratio = (rayon - VOIE/2.0) / (rayon + VOIE/2.0);
	
	moteur1 = ((cote == GAUCHE) ? MOTEUR_DROIT : MOTEUR_GAUCHE);
	moteur2 = ((cote == GAUCHE) ? MOTEUR_GAUCHE : MOTEUR_DROIT);
	
	//Motors_SetAcceleration(fabs(accel_def_ligne * ratio), moteur1);  
	//Motors_SetAcceleration(accel_def_ligne, moteur2);
	
	//Motors_SetSpeed(fabs(speed_def_ligne * ratio), moteur1); 
	//Motors_SetSpeed(speed_def_ligne, moteur2);
	
	Motors_SetAcceleration(fabs(1000 * ratio), moteur1);  
	Motors_SetAcceleration(1000, moteur2);
	
	Motors_SetSpeed(fabs(700 * ratio), moteur1); 
	Motors_SetSpeed(700, moteur2);
	
	Motors_SetPosition((angle * PI / 180.0) * (rayon - VOIE/2.0),moteur1); 
	Motors_SetPosition((angle * PI / 180.0) * (rayon + VOIE/2.0),moteur2);
	
	Motors_Start(MOTEUR_GAUCHE);
	Motors_Start(MOTEUR_DROIT);

	if(wait) while(Motors_IsRunning(MOTEUR_GAUCHE) || Motors_IsRunning(MOTEUR_DROIT));
}


double Stop(unsigned char stopmode)
{
	Motors_Stop(stopmode,MOTEUR_GAUCHE);
	Motors_Stop(stopmode,MOTEUR_DROIT);

	while(Motors_IsRunning(MOTEUR_GAUCHE) || Motors_IsRunning(MOTEUR_DROIT));

	return fabs((targ_pos[0]-real_pos[0]+targ_pos[1]-real_pos[1])/2);
}


/*************************** Couche intermédiaire ***************************/

double Motors_GetPosition(unsigned char moteur)
{
	return real_pos[moteur];
}

void Motors_GetRawPosition(unsigned int *buffer)
{
	buffer = position_buffer;
}

void Motors_DefineHome(unsigned char moteur)
{
	flagdefinehome = 1;
	real_pos[moteur]=0;
	cons_pos[moteur]=0;
	Motors_Stop(ABRUPT, moteur);
}

void Motors_Stop(unsigned char stopmode, unsigned char moteur)
{
	switch(stopmode)
	{
		case FREELY :	cons_pos[moteur] = real_pos[moteur]; // Commande stop FREELY
						motion[moteur] = 0;
						pid_power = 0;
						pwm(moteur,0);
						break;
		case SMOOTH :	cons_pos[moteur] = real_pos[moteur]; // Commande stop SMOOTH
						motion[moteur] = 30;
						pid_power = 1;
						break;
		case ABRUPT :	cons_pos[moteur] = real_pos[moteur]; // Commande stop ABRUPT
						motion[moteur] = 0;
						pid_power = 1;
						break;
		default :		break;
	}
}

unsigned char Motors_IsRunning(unsigned char moteur)
{
	if((motion[0] == 0) && (motion[1] == 0))	return FALSE;
	else										return TRUE;
}	

void Motors_SetPosition(double position, unsigned char moteur)
{
	targ_pos[moteur] = position;
	targ_pos[moteur] += cons_pos[moteur];
}	

void Motors_SetAcceleration(double acceleration, unsigned char moteur)
{
	accel_def[moteur] = acceleration;
}	

void Motors_SetSpeed(double vitesse, unsigned char moteur)
{
	speed_def[moteur] = vitesse;
}	

void Motors_SetAcceleration_Pivot(double acceleration)
{
	accel_def_pivot= acceleration;
}	

void Motors_SetSpeed_Pivot(double vitesse)
{
	speed_def_pivot = vitesse;
}	

void Motors_SetAcceleration_Ligne(double acceleration)
{
	accel_def_ligne = acceleration;
}	

void Motors_SetSpeed_Ligne(double vitesse)
{
	speed_def_ligne = vitesse;
}	

void Motors_Power(unsigned char power)
{
	pid_power = power;
	if(pid_power == ON)
	{
		cons_pos[0] = real_pos[0];
		cons_pos[1] = real_pos[1];
	}
}

void Motors_Start(unsigned char moteur)
{
	motion[moteur] = 1;
}

/*************************** Couche basse ***************************/

unsigned char Motors_Task(void)
{
	unsigned char i;
	double motors_ok;
	static unsigned char sens[N];
	static double decel_point[N],origi_point[N];
	static double speed_max[N],accel_max[N];
	static double speed[N],accel[N];
	double delta_x, delta_y, lcurvi, vitesse;
	static double lcurvi_old;
	unsigned char retour = 0;
	static double old_posi[N]={0};
	static unsigned char cpt_blocage[N] = {0};

	lcurvi   = (real_pos[1] + real_pos[0])/2;
	vitesse  = lcurvi - lcurvi_old;
	pos_teta = (real_pos[1] - real_pos[0]) / (VOIE) + offset_teta;
	while(pos_teta >  PI) pos_teta -= 2*PI;
	while(pos_teta < -PI) pos_teta += 2*PI;
	delta_x  = -vitesse * sinf(pos_teta);
	delta_y  =  vitesse * cosf(pos_teta);
	pos_x   += delta_x;
	pos_y   += delta_y;
	lcurvi_old = lcurvi;
	
	motors_ok=pid(1,cons_pos,real_pos); // 150µs Max

	for(i=0;i<N;i++)
		if(motion[i] == 11 || motion[i] == 21 || motion[i] == 22)
			if((cpt_calage[0] > 20) && (cpt_calage[1] > 20) && motiontype ==3) // 50	
			{
				motiontype = 0;
				Stop(ABRUPT);
				retour = 0x30;
				return retour;
			}

	for(i=0;i<N;i++) switch(motion[i]) // 10:12 -> Triangulaire | 20:23 -> Trapezoidale
	{
		case 0	:	break;

		case 1	:	accel_max[i] = accel_def[i];
					speed_max[i] = speed_def[i];
					if(fabs(targ_pos[i]-real_pos[i]) < (speed_max[i]*speed_max[i]/accel_max[i]))  // Triangulaire ou trapezoidale ?
						motion[i] = 10;
					else
						motion[i] = 20;
					break;
		case 10	:	speed[i] = 0;												// Acceleration triangulaire
					accel[i] =  accel_max[i];
					if(targ_pos[i] > real_pos[i])	sens[i] = 1; // Sens positif
					else							sens[i] = 0; // Sens négatif
					decel_point[i] = (targ_pos[i]-real_pos[i])/2+real_pos[i];
					motion[i]++;
		case 11	:	speed[i] += accel[i] * 0.001;								
					if(speed[i] > speed_max[i])	speed[i] =  speed_max[i]; // Ne devrais pas arriver, mais on ne sait jamais
					if( sens[i] && real_pos[i] > decel_point[i]) motion[i] = 23;
					if(!sens[i] && real_pos[i] < decel_point[i]) motion[i] = 23;
					if(sens[i])	cons_pos[i] += speed[i] * 0.001;
					else		cons_pos[i] -= speed[i] * 0.001;
					break;

		case 20	:	speed[i] = 0;												// Acceleration trapézoidale
					accel[i] = accel_max[i];
					decel_point[i] = 0;
					origi_point[i] = real_pos[i];
					if(targ_pos[i] > real_pos[i])	sens[i] = 1; // Sens positif
					else							sens[i] = 0; // Sens négatif
					motion[i]++;
		case 21 :	speed[i] += accel[i] * 0.001;								
					if(speed[i] > speed_max[i])	
					{
						speed[i] = speed_max[i];
						decel_point[i] = targ_pos[i]-real_pos[i]+origi_point[i];
						motion[i]++;
					}
					if(sens[i])	cons_pos[i] += speed[i] * 0.001;
					else		cons_pos[i] -= speed[i] * 0.001;
					break;

		case 22	:	if( sens[i] && real_pos[i] > decel_point[i]) motion[i]++;	// Croisière
					if(!sens[i] && real_pos[i] < decel_point[i]) motion[i]++;
					if(sens[i])	cons_pos[i] += speed[i] * 0.001;
					else		cons_pos[i] -= speed[i] * 0.001;
					break;

		case 23	:	speed[i] -= accel[i] * 0.001; 								// Deceleration
					if(speed[i] < 10)// 50 (07/05/2013)
						{
							speed[i]=10; //50	 (07/05/2013)			// Ne devrais pas arriver, mais on ne sait jamais
							//motion[i] = 24;
						}
					if(sens[i])	cons_pos[i] += speed[i] * 0.001;
					else		cons_pos[i] -= speed[i] * 0.001;
					if( sens[i] && (cons_pos[i] - targ_pos[i]) >= 0) motion[i] = 24;
					if(!sens[i] && (cons_pos[i] - targ_pos[i]) <= 0) motion[i] = 24;

					break;

		case 24	:	cons_pos[i] = targ_pos[i]; // On force l'exactitude du deplacement
					if(i==0 && motion[0] == 24 && motion[1] == 24)
					{
						motion[0]=0;//1;
						motion[1]=0;
						// Envoi au pc la fin de la trajectoire, seulement si i=0 pour ne pas envoyer 2 fois (un seul moteur)
						return(0x10);	
					}
					break;
					

		case 30	:	// Deceleration sur arret imprévu (commande STOP SMOOTH)
					//if(speed[i] >  MAX_SPEED) speed[i] =  MAX_SPEED; 
					//if(speed[i] < -MAX_SPEED) speed[i] = -MAX_SPEED;
					speed[i] -= accel[i] * 0.001;
					if(sens[i])	cons_pos[i] += speed[i] * 0.001;
					else		cons_pos[i] -= speed[i] * 0.001;
					if(speed[i] <= 0) 	
					{
						motion[i] = 0;
						speed[i]  = 0;
						if(i==0) retour = (0x10); // 10/05/2013 ajout du if(i==0) 
					}
					break;

		default :	break;
	}
	
	if(motiontype==3) // calage
	{
		for(i=0;i<N;i++)
		{
			if(old_posi[i] == real_pos[i])
			{
				if(cpt_calage[i]<255) 
					cpt_calage[i]++;
			}
			else
			{
				cpt_calage[i]=0;
			}
			old_posi[i] = real_pos[i];
		}
	}
	else
	{
		if(motiontype!=4)
		{
			for(i=0;i<N;i++)
				if(fabs(cons_pos[i] - real_pos[i]) > 150)
				{
					if(cpt_blocage[i]++>20)
					{
						Stop(FREELY);
						retour = 0x40;
						return retour;
					}
				}
				else
				{
					cpt_blocage[i] =0;
				}
		}	
		cpt_calage[0]=0;
		cpt_calage[1]=0;
	}
	

	return retour;
}

void set_pid(double *coeffs)
{
	unsigned char i,j=0;
	for(i=0;i<N;i++)
	{
		kp[i] = coeffs[j++];
		ki[i] = coeffs[j++];
		kd[i] = coeffs[j++];
	}
}



// Calcul PID a reiterer a chaque milliseconde
double pid(unsigned char power,double * targ_pos,double * real_pos)
{
	unsigned char i;
	double cor[N];
	static double erreur_old[N]={0};
	

	if(flagdefinehome)
	{
		flagdefinehome=0;

		QEI1CONbits.QEIM = 0;
		POS1CNT=0;
		revolutions[0]=0;
		QEI1CONbits.QEIM = QEI2CONbits.QEIM;

		QEI2CONbits.QEIM = 0;
		POS2CNT=0;
		revolutions[1]=0;
		QEI2CONbits.QEIM = QEI1CONbits.QEIM;
	}

	
	if(revolutions[0]<0)	position_buffer[0] = 1;
	else					position_buffer[0] = 0;
	if(revolutions[1]<0)	position_buffer[3] = 1;
	else					position_buffer[3] = 0;
	
	position_buffer[1] = POS1CNT;
	position_buffer[4] = POS2CNT;
	
	if(position_buffer[0])	position_buffer[2] = -revolutions[0];
	else					position_buffer[2] =  revolutions[0];
	if(position_buffer[3])	position_buffer[5] = -revolutions[1];
	else					position_buffer[5] =  revolutions[1];
	
	
	raw_position[0] = (long)POS1CNT + (long)(revolutions[0]*0x10000);
	raw_position[1] = (long)POS2CNT + (long)(revolutions[1]*0x10000);

	buff_position[0][buff_position_ptr]   = raw_position[0];
	buff_position[1][buff_position_ptr++] = raw_position[1];

	// Calcul de la position reelle en mm
	real_pos[0] = 1.000 * MM_SCALER * (double)raw_position[0]; // Roue droite
	real_pos[1] = 1.000 * MM_SCALER * (double)raw_position[1]; // Roue gauche


	if(pid_power)
	{
		for(i=0;i<N;i++)
		{
			erreur[i] = targ_pos[i]* MM_INVSCALER - (double)raw_position[i];// ; // Calcul de l'erreur en pas codeur
			if((motion[0] == 0) && (motion[1] == 0))
			{
				if(pid_count < 1000)
					pid_count++;
			}
			else
			{
				pid_count = 0;
			}
			
			if(pid_count == 1000) 	cor[i] = erreur[i]*(kp[i]-5);
			else					cor[i] = erreur[i]*kp[i] + (erreur[i] - erreur_old[i])*kd[i];
			
			erreur_old[i] = erreur[i]; // Mise a jour necessaire pour le terme derive
			pwm_cor[i] = cor[i];
		}
		pwm(GAUCHE,cor[0]); // a dequote (test)
		pwm(DROITE,cor[1]); // a dequote (test)
	}
	return fabs(cor[0]) + fabs(cor[1]);
}

char pwm(unsigned char motor, double valeur) // Value = +/- 4000
{
	double value;
	value = valeur;
	
	if(value >0)
	{
		if(value >  erreur_allowed) value -= erreur_allowed;
		else			value = 0;
	}
	else
	{
		if(value < -erreur_allowed) value += erreur_allowed;
		else			value = 0;
	}
	if(value >  4095) value =  4095;
	if(value < -4095) value = -4095;	
	

	if(motor==GAUCHE) value = -value;

	switch(motor)
	{
		case AVANT:
		case GAUCHE:	if(value > 0)	// Moteur Gauche
						{
							DIRG  = 1;		// Position incremente
							P1DC2 = (unsigned int)(4095 - value);		
						}
						else
						{
							DIRG  = 0;		// Position decremente
							P1DC2 = (unsigned int)(4095 + value);		
						}
						break;
		case ARRIERE:
		case DROITE: 	if(value > 0)	// Moteur Droit
						{
							DIRD  = 1;		// Position incremente
							P1DC1 = (unsigned int)(4095 - value);		
						}
						else
						{
							DIRD  = 0;		// Position decremente
							P1DC1 = (unsigned int)(4095 + value);		
						}
						break;
		default : 		return -1;
	}
	return 0;
}


/*double fabs(double value)
{
	if(value<0) return -value;
	else		return  value;
}*/

//----------------------------------------------------------------------------


void Sleepms(unsigned int nbr)
{
	unsigned int i,j;

	for(i=0;i<nbr;i++)
		for(j=0;j<8000;j++);
}

void Sleepus(unsigned int nbr) // Défaut constant de +0.5us
{
	unsigned int i;

	for(i=0;i<nbr;i++)
	{
		Nop();	Nop();	Nop();	Nop();
		Nop();	Nop();	Nop();	Nop();
		Nop();	Nop();	Nop();	Nop();
		Nop();	Nop();	Nop();	Nop();
		Nop();	Nop();	Nop();	Nop();
		Nop();	Nop();	Nop();	Nop();
		Nop();	Nop();	Nop();	Nop();
		Nop();	Nop();	Nop();	Nop();
	}
}


void __attribute__ ((interrupt, no_auto_psv)) _QEI1Interrupt(void) 
{/* moteur GAUCHE*/
	QEI1CONbits.QEIM = 0;
	IFS3bits.QEI1IF = 0;
	if((QEI1CONbits.UPDN==1) && POS1CNT < 0x8000)	revolutions[0]++; // rollover
	if((QEI1CONbits.UPDN==0) && POS1CNT > 0x8000)	revolutions[0]--; // underflow
	QEI1CONbits.QEIM = QEI2CONbits.QEIM;
}

void __attribute__ ((interrupt, no_auto_psv)) _QEI2Interrupt(void) 
{/* moteur DROIT*/
	QEI2CONbits.QEIM = 0;
	IFS4bits.QEI2IF = 0;
	if((QEI2CONbits.UPDN==1) && POS2CNT < 0x8000)	revolutions[1]++; // rollover
	if((QEI2CONbits.UPDN==0) && POS2CNT > 0x8000)	revolutions[1]--; // underflow
	QEI2CONbits.QEIM = QEI1CONbits.QEIM;
}
