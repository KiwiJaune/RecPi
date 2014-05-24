#include <p33FJ128MC804.h>
#include "CDS5516.h"
#include "FonctionsUc.h"

extern unsigned char trame_recu[20],flag_servo,recu_nbr,timeout_servo;

/////////////////////////////////////////////////
//                                             //
//              Communication Servo            //
//                                             //
/////////////////////////////////////////////////
//delay ajouté par Mouly
void delay_us_uart(float mult){
    long i = 40000*mult;
    while(i--);
}

// Envoi d'un message série au servomoteur
void CDS5516EnvoiMessage(char baud, char id, char fonction, int valeur, char nbChar)
{
	char taille = 0, instruction = 0, adresse = 0, val = 0,val1 = 0, checksum = 0;
	int i;

	LATBbits.LATB2 = 1;	// 1 J'envoie et 0 je réceptionne
	//InitUART2();	//Initialisation de la liaison série 2 (UART2)
	//U2BRG = ((FCY/baud)/4)-1;
	U2BRG = 522;													
	//Instruction écriture valeur
	instruction = 0x03;  	
	adresse = fonction; 
	if(nbChar == 2)
	{	
		taille = 0x05; 
		val = valeur & 0b0000000011111111;
		val1 = valeur >> 8;
		checksum = (char)(0xFF-(char)(id + taille + instruction + adresse + val + val1));
	}
	else if(nbChar == 1)
	{
		taille = 0x04; 
		val = valeur;
		checksum = (char)(0xFF-(char)(id + taille + instruction + adresse + val));
	}

	for(i=0;i<2;i++)
	{
		UART2PutChar(0xFF);
		UART2PutChar(0xFF);
		UART2PutChar(id);
		UART2PutChar(taille);
		UART2PutChar(instruction);
		UART2PutChar(adresse);
		UART2PutChar(val);	
		if(nbChar == 2)
			UART2PutChar(val1);
		UART2PutChar(checksum);
	}
}

// Envoi d'un ordre série de reset au servomoteur
void CDS5516Reset(char baud, char id)
{
	char taille = 0, instruction = 0, checksum = 0;
	int i;

	LATBbits.LATB2 = 1;	// 1 J'envoie et 0 je réceptionne
	//InitUART2();	//Initialisation de la liaison série 2 (UART2)
	//U2BRG = ((FCY/baud)/4)-1;
	U2BRG = 522; 
	taille = 0x02;
	instruction = 0x06;  
	checksum = (char)(0xFF-(char)(id + taille + instruction));

	for(i=0;i<2;i++)
	{
		UART2PutChar(0xFF);
		UART2PutChar(0xFF);
		UART2PutChar(id);
		UART2PutChar(taille);
		UART2PutChar(instruction);
		UART2PutChar(checksum);
	}
}

// Envoi d'un ping série au servomoteur auquel il est censé répondre par un état de ses erreurs
int CDS5516Ping(char baud, char id)
{
	char taille = 0, checksum = 0, instruction = 0;
	int i,valeurRecue;
	char checksumRecu;
	char messageRecu[10];

	LATBbits.LATB2 = 1;	// 1 J'envoie et 0 je réceptionne

	//InitUART2();	//Initialisation de la liaison série 2 (UART2)
	//U2BRG = ((FCY/baud)/4)-1;
	U2BRG = 522; 
	taille = 0x02;  															//Nbr de paramètres + 2
	instruction = 0x01; 
	checksum = (char)(0xFF-(char)(id + taille + instruction));

	UART2PutChar(0xFF);
	UART2PutChar(0xFF);
	UART2PutChar(id);
	UART2PutChar(taille);
	UART2PutChar(instruction);
	UART2PutChar(checksum);


/*	// Réception de la réponse
	LATBbits.LATB2 = 0;	// 1 J'envoie et 0 je réceptionne
	while(!U1STAbits.URXDA);
	
	checksum = 0;
	while(i < 10 && U1STAbits.URXDA)
	{
		messageRecu[i] = U1RXREG;
		checksum += messageRecu[i];
		i++;
	}
*/
	timeout_servo = 0;
	while(flag_servo==0) if(timeout_servo>10) return(42); // ajouter un timeout !!!!
	for(i=0;i<20;i++)
		messageRecu[i] = trame_recu[i];
	i=recu_nbr+4;
	checksum -= messageRecu[i - 1];
	checksum = (char)(0xFF-(char)(checksum));


	//if(messageRecu[0] != 0xFF || messageRecu[1] != 0xFF)// || messageRecu[2] != id || messageRecu[3] != 2 || checksum != messageRecu[i - 1])
	//	return 42;

	valeurRecue = messageRecu[i - 2];
	return valeurRecue;
}

// Envoi un message série au servomoteur qui demande un retour qui est une lecture d'une valeur registre
int CDS5516DemandeMessage(char baud, char id, char fonction, char nbChar)
{
	char taille = 0, instruction = 0, adresse = 0,checksum = 0;
	int i, j,valeurRecue;
	char checksumRecu;
	char messageRecu[20];

	LATBbits.LATB2 = 1;	// 1 J'envoie et 0 je réceptionne
	//InitUART2();	//Initialisation de la liaison série 2 (UART2)
	//U2BRG = ((FCY/baud)/4)-1;
	U2BRG = 522; 
	taille = 0x04;  															//Nbr de paramètres + 2
	instruction = 0x02;  														//Instruction écriture valeur
	adresse = fonction;
	checksum = (char)(0xFF-(char)(id + taille + instruction + nbChar + adresse));

	UART2PutChar(0xFF);
	UART2PutChar(0xFF);
	UART2PutChar(id);
	UART2PutChar(taille);
	UART2PutChar(instruction);
	UART2PutChar(adresse);
	UART2PutChar(nbChar);
	UART2PutChar(checksum);

	// Réception de la réponse
	LATBbits.LATB2 = 0;	// 1 J'envoie et 0 je réceptionne
	



/*return;
	while(!U1STAbits.URXDA);

	trame.message = msg;
	msg[0] = 0xC4;
	msg[1] = 0x60;
	msg[2] = 0x02;
	
	j = i = 0;
	checksum = 0;

	while(j < 20 && U1STAbits.URXDA)
	{
		messageRecu[i] = U1RXREG;
		checksum += messageRecu[i];
		i++;		
		delay_us_uart(0.1);

		// Hack
		msg[j + 4] = messageRecu[i];
		j++;	
	}

	msg[3] = j;
	trame.nbChar = j + 4;
	EnvoiUserUdp(trame);
	return;
*/
	// Ici il faut que messageRecu contienne la trame servo recu
	timeout_servo = 0;
	while(flag_servo==0) if(timeout_servo>10) return(42); // ajouter un timeout !!!!
	flag_servo=0;
	for(i=0;i<20;i++)
		messageRecu[i] = trame_recu[i];
	i=recu_nbr+4;
	checksum -= messageRecu[i - 1];
	checksum = (char)(0xFF-(char)(checksum));

	// pour debug
	/*msg[0] = 0xC4;
	msg[1] = 0x60;
	msg[2] = 0x02;
	for(i=0;i<17;i++)
		msg[i+3] = trame_recu[i];
	trame.nbChar=3+recu_nbr+4;
	trame.message = msg;
	EnvoiUserUdp(trame);
	
	i=recu_nbr+4;
	*/
	//if(messageRecu[0] != 0xFF || messageRecu[1] != 0xFF || messageRecu[2] != id )//|| messageRecu[3] != 2 )//|| checksum != messageRecu[i - 1])
	//	return 42;

	if(nbChar == 1)
	{
		valeurRecue = messageRecu[i - 2];
		return valeurRecue;
	}
	else
	{
		valeurRecue  = (int)messageRecu[i - 2];
		valeurRecue  = valeurRecue<<8;
		valeurRecue  = valeurRecue+(((int)(messageRecu[i - 3]))&0x00FF); // canard en plastique d'or
		return valeurRecue;
	}
}

/////////////////////////////////////////////////
//                                             //
//       Fonctions de demande de valeurs       //
//                                             //
/////////////////////////////////////////////////

int CDS5516DemandeCCWMargin(char baud, char id)
{
	return CDS5516DemandeMessage(baud, id, 0x1B, 1);
}

int CDS5516DemandeCWMargin(char baud, char id)
{
	return CDS5516DemandeMessage(baud, id, 0x1A, 1);
}

int CDS5516DemandeCCWSlope(char baud, char id)
{
	return CDS5516DemandeMessage(baud, id, 0x1D, 1);
}

int CDS5516DemandeCWSlope(char baud, char id)
{
	return CDS5516DemandeMessage(baud, id, 0x1C, 1);
}

int CDS5516DemandeConfigAlarmeLED(char baud, char id)
{
	return CDS5516DemandeMessage(baud, id, 0x11, 1);
}

int CDS5516DemandeConfigAlarmeShutdown(char baud, char id)
{
	return CDS5516DemandeMessage(baud, id, 0x12, 1);
}

int CDS5516DemandeConfigEcho(char baud, char id)
{
	return CDS5516DemandeMessage(baud, id, 0x10, 1);
}

int CDS5516DemandeCoupleActive(char baud, char id)
{
	return CDS5516DemandeMessage(baud, id, 0x18, 1);
}

int CDS5516DemandeCoupleMaximum(char baud, char id)
{
	return CDS5516DemandeMessage(baud, id, 0x0E, 2);
}

int CDS5516DemandeLed(char baud, char id)
{
	return CDS5516DemandeMessage(baud, id, 0x19, 1);
}

int CDS5516DemandeMouvement(char baud, char id)
{
	return CDS5516DemandeMessage(baud, id, 0x2E, 1);
}

int CDS5516DemandeModele(char baud, char id)
{
	return CDS5516DemandeMessage(baud, id, 0x00, 1);
}

int CDS5516DemandePositionActuelle(char baud, char id)
{
	return CDS5516DemandeMessage(baud, id, 0x24, 2);
}

int CDS5516DemandePositionCible(char baud, char id)
{
	return CDS5516DemandeMessage(baud, id, 0x1E, 2);
}

int CDS5516DemandePositionMax(char baud, char id)
{
	return CDS5516DemandeMessage(baud, id, 0x08, 2);
}

int CDS5516DemandePositionMin(char baud, char id)
{
	return CDS5516DemandeMessage(baud, id, 0x06, 2);
}

int CDS5516DemandeTemperature(char baud, char id)
{
	return CDS5516DemandeMessage(baud, id, 0x2B, 1);
}

int CDS5516DemandeTension(char baud, char id)
{
	return CDS5516DemandeMessage(baud, id, 0x2A, 1);
}

int CDS5516DemandeVersionFirmware(char baud, char id)
{
	return CDS5516DemandeMessage(baud, id, 0x02, 1);
}

int CDS5516DemandeVitesseActuelle(char baud, char id)
{
	return CDS5516DemandeMessage(baud, id, 0x26, 2);
}

int CDS5516DemandeVitesseMax(char baud, char id)
{
	return CDS5516DemandeMessage(baud, id, 0x20, 1);
}

int CDS5516DemandeBaudrate(char id)
{
	// TODO : Boucler sur tous les baudrate et trouver celui qui est bon
}

int CDS5516DemandeID(char baudrate)
{
	// TODO : Envoyer un ping en broadcast (ou à tous les ID un par un si ça ne fonctionne pas) et écouter les réponses
	// TODO : Traiter le fait de devoir renvoyer plusieurs ID si nécessaire et utile
}

/////////////////////////////////////////////////
//                                             //
//      Fonctions envoi valeur servo           //
//                                             //
/////////////////////////////////////////////////

//Contrôle des paramètres de compliance
void CDS5516EnvoiComplianceParams(char baud, char id, char CCWSlope, char CCWMargin, char CWSlope, char CWMargin)
{
	CDS5516EnvoiMessage(baud, id, 0x1A, CWMargin, 1);
	CDS5516EnvoiMessage(baud, id, 0x1B, CCWMargin, 1);
	CDS5516EnvoiMessage(baud, id, 0x1C, CWSlope, 1);
	CDS5516EnvoiMessage(baud, id, 0x1D, CCWSlope, 1);
}

//Contrôle de la configuration du déclenchement de l'alarme Shutdown
void CDS5516EnvoiAlarmeShutdown(char baud, char id, char inputVoltage, char angleLimit, char overheating, char range, char checksum, char overload, char instruction)
{
	int valeur = 0;
	if(inputVoltage)
		valeur = valeur | 0b00000001;
	if(angleLimit)
		valeur = valeur | 0b00000010;
	if(overheating)
		valeur = valeur | 0b00000100;
	if(range)
		valeur = valeur | 0b00001000;
	if(checksum)
		valeur = valeur | 0b00010000;
	if(overload)
		valeur = valeur | 0b00100000;
	if(instruction)
		valeur = valeur | 0b01000000;

	CDS5516EnvoiMessage(baud, id, 0x12, valeur, 1);
}

//Contrôle de la configuration du déclenchement de l'alarme LED
void CDS5516EnvoiAlarmeLED(char baud, char id, char inputVoltage, char angleLimit, char overheating, char range, char checksum, char overload, char instruction)
{
	int valeur = 0;
	if(inputVoltage)
		valeur = valeur | 0b00000001;
	if(angleLimit)
		valeur = valeur | 0b00000010;
	if(overheating)
		valeur = valeur | 0b00000100;
	if(range)
		valeur = valeur | 0b00001000;
	if(checksum)
		valeur = valeur | 0b00010000;
	if(overload)
		valeur = valeur | 0b00100000;
	if(instruction)
		valeur = valeur | 0b01000000;

	CDS5516EnvoiMessage(baud, id, 0x11, valeur, 1);
}

//Contrôle de la position minimum de 0 à 1023
void CDS5516EnvoiPositionMin(char baud, char id, unsigned int positionMin)
{
	CDS5516EnvoiMessage(baud, id, 0x06, positionMin, 2);
}

//Contrôle de la réponse retournée à chaque message reçu
void CDS5516EnvoiConfigEcho(char baud, char id, char config)
{
	CDS5516EnvoiMessage(baud, id, 0x10, config, 1);
}

//Contrôle de la position maximum de 0 à 1023
void CDS5516EnvoiPositionMax(char baud, char id, unsigned int positionMax)
{
	CDS5516EnvoiMessage(baud, id, 0x08, positionMax, 2);
}

//Contrôle en angle du cds5516 (0 à 1023 soit 0° à 300°)
void CDS5516EnvoiPosistionCible(char baud, char id, unsigned int positionCible)
{
	CDS5516EnvoiMessage(baud, id, 0x1E, positionCible, 2);
}

//Paramètre de la vitesse du CDS5516 (0 à 1023)
void CDS5516EnvoiVitesseMax(char baud, char id, unsigned int vitesseMax)
{
	CDS5516EnvoiMessage(baud, id, 0x20, vitesseMax, 2);
}

//Allumer la led du CDS5516
void CDS5516EnvoiLed(char baud, char id, char ledAllume)
{
	CDS5516EnvoiMessage(baud, id, 0x19, ledAllume, 1);
}

//Activation du couple du CDS5516 (0 ou 1)
void CDS5516EnvoiCoupleActive(char baud, char id, char enableTorque)
{
	CDS5516EnvoiMessage(baud, id, 0x18, enableTorque, 1);
}

//Changer le bauderate du CDS5516 
void CDS5516EnvoiBauderate(char baud, char id, char finalBauderate)
{
	CDS5516EnvoiMessage(baud, id, 0x04, finalBauderate, 1);
}

//Changer l'ID du CDS5516 
void CDS5516EnvoiId(char baud, char id, char nouvelId)
{
	CDS5516EnvoiMessage(baud, id, 0x03, nouvelId, 1);
}

//Régler le maximum de couple en EEPROM (0 à 1023)
void CDS5516EnvoiCoupleMaximum(char baud, char id, unsigned int coupleMax)
{
	CDS5516EnvoiMessage(baud, id, 0x0E, coupleMax, 2);
}
