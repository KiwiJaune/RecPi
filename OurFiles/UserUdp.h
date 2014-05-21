/************************************************
*
* UserUdp.h
* Fichier permettant de gérer une communication UDP
* Couche intermédiaire entre le main et le fichier udp.c
*
*
*****************************************************/

#ifndef __USER_UDP__
#define __USER_UDP__

//#include "TCPIP Stack/UDP.h"
#include "TCPIP Stack/TCPIP.h"

typedef struct Trame
{
	BYTE* message;
	int nbChar;
} Trame;

#define portReception 12313
#define portEmission 12323

void InitUserUdp();
Trame ReceptionUserUdp();
void EnvoiUserUdp(Trame trame, unsigned char bloquant);
void EnvoiStringUdp(const char *string);
void memclr(void * dest, WORD size);

#endif
