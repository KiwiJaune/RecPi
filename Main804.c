#define THIS_IS_STACK_APPLICATION
#include "TCPIP Stack/TCPIP.h"
#include "TCPIP Stack/UDPPerformanceTest.h"
#include "Main804.h"
#include "OurFiles/UserUdp.h"
#include "OurFiles/asser.h"
#include "OurFiles/init.h"
#include <stdio.h>
#include <stdlib.h>
#include <uart.h>
#include <math.h>
#include "OurFiles/Pilotage.h"
#include "OurFiles/CDS5516.h"
#include "OurFiles/FonctionsUc.h"

// Configuration Clock Par Mouly
_FOSCSEL(FNOSC_FRC)
_FOSC(FCKSM_CSECMD & OSCIOFNC_ON)
_FPOR(FPWRT_PWR1)
_FWDT(FWDTEN_OFF)// & SWDTEN_OFF)
_FICD(ICS_PGD3 & JTAGEN_OFF)

//Define Capteur Couleur
#define LED LATAbits.LATA7
#define S2 LATBbits.LATB0 
#define S3 LATAbits.LATA10 

// TODO list
// * Self-calibration de l'odométrie
// * Self-check de l'asservissement avant chaque début de match
// * Self-placement du robot
// * Procédure de calibration odométrique auto
// * Asservissement polaire
// * Localisation x,y,teta
// * Navigation x,y,teta

// * Detection blocage, plusieurs pistes :
//   - On detecte le blocage, on envoi simplement au PC
//	 - On detecte le blocage, on arrete le moteur (ABRUPT) et on envoi au PC le reste de la distance à parcourir
//	 - On detecte le blocage, on revient à la position précédente mieux encore avec le magnetoscope



APP_CONFIG AppConfig;

// Use UART2 instead of UART1 for stdout (printf functions).  Explorer 16 
// serial port hardware is on PIC UART2 module.
//#if defined(EXPLORER_16)//TODO
// int __C30_UART = 2;
//#endif

// Private helper functions.
// These may or may not be present in all applications.
static void InitAppConfig(void);

extern unsigned int ADC_Results[8];
extern double cons_pos[N];
extern double real_pos[N];
extern unsigned char scan;

unsigned char motor_flag=0,datalogger_blocker=0;
double position_lock;
//double datalogger_ga[500]={0};
//double datalogger_dr[500]={0};
unsigned int datalogger_counter=0,flag=0,courrier=0,PID_ressource_used;
unsigned char flag_envoi=0,flag_distance=0,flag_blocage=0,flag_calage=0;

//Variable Capteur Couleur
unsigned int Cpt_Tmr2_Capteur_Couleur = 0;
unsigned int Tab_Capteur_Couleur[8] = {0};
unsigned char etat_Capteur_Couleur = 0;

void _ISR __attribute__((__no_auto_psv__)) _AddressError(void)
{
    Nop();
	Nop();
}
void _ISR __attribute__((__no_auto_psv__)) _StackError(void)
{
    Nop();
	Nop();
}

int main(void)
{
	unsigned char demo=0;
	unsigned char hold_blocage;
	unsigned char jackAvant = 1, etatCouleur = 2;
	static DWORD dwLastIP = 0;

	Trame Jack;
	static BYTE Presence[2];
	Jack.nbChar = 2;
	Presence[0] = 0xC1;
	Presence[1] = CMD_REPONSE_PRESENCE_JACK;
	Jack.message = Presence;

	Trame Couleur_Equipe;
	static BYTE Couleur[3];
	Couleur_Equipe.nbChar = 3;
	Couleur[0] = 0xC1;
	Couleur[1] = CMD_REPONSE_COULEUR_EQUIPE;
	Couleur[2] = PORTBbits.RB4;
	Couleur_Equipe.message = Couleur;

	Trame envoiFin;
	static BYTE mess[2];
	mess[0] = 0xC1;
	mess[1] = CMD_FINDEPLACEMENT;
	envoiFin.message = mess;
	envoiFin.nbChar = 2;
	
	/*Trame envoiDistance;
	static BYTE messdistance[4];
	messdistance[0] = 0xC1;
	messdistance[1] = 0x50;
	envoiDistance.message = messdistance;
	envoiDistance.nbChar = 4;*/
	
	Trame envoiBlocage;
	static BYTE messblocage[2];
	messblocage[0] = 0xC1;
	messblocage[1] = 0x70;
	envoiBlocage.message = messblocage;
	envoiBlocage.nbChar = 2;
	
	Trame envoiCalage;
	static BYTE messcalage[2];
	messcalage[0] = 0xC1;
	messcalage[1] = CMD_FINRECALLAGE;
	envoiCalage.message = messcalage;
	envoiCalage.nbChar = 2;

	Trame envoiDebugAsser;
	static BYTE Debug_Asser[4];
//	Debug_Asser[0] = erreur[0];
//	Debug_Asser[1] = erreur[1];
//	Debug_Asser[2] = cor[0];
//	Debug_Asser[3] = cor[1];
	envoiDebugAsser.message = Debug_Asser;
	Trame trame;	

	InitClk(); 		// Initialisation de l'horloge
	InitPorts(); 	// Initialisation des ports E/S

    Init_Timer();	
    
	InitQEI(); 		// Initialisation des entrées en quadrature
	InitPWM();		// Configuration du module PWM 
	InitT2();		// Configuration du timer 2	
//	InitADC();
//	InitDMA();
	InitProp();
	
	// Initialize stack-related hardware components that may be 
	// required by the UART configuration routines
    TickInit();
	#if defined(STACK_USE_MPFS) || defined(STACK_USE_MPFS2)
		MPFSInit();
	#endif
	
	// Initialize Stack and application related NV variables into AppConfig.
	InitAppConfig();
	
	UDPInit();
    StackInit();	
 	UDPPerformanceTask();
	InitUserUdp();
	
	//INTERRUPT PRIORITY
	IPC4bits.SI2C1IP = 7;
//	IPC1bits.T2IP = 4;
	IPC14bits.QEI1IP = 5;
	IPC18bits.QEI2IP = 5;								
	
	demo=4;

	hold_blocage=0;
	
// Initialisation de la liaison série 2 (UART2)
	InitUART2();
	Init_Turbine(); //Initialisé après Init_Timer
	Init_Servos();
	Init_Input_Capture();



	while(1)
  	{	
	  	if(!PORTAbits.RA8 && jackAvant)
	  	{
		  	EnvoiUserUdp (Jack);
		  	jackAvant = 0;
		}
		if(etatCouleur != PORTBbits.RB4)
		{
			Couleur[2] = PORTBbits.RB4;
  			EnvoiUserUdp (Couleur_Equipe);
  			etatCouleur = PORTBbits.RB4;
  		}
		if(flag_envoi) 
		{	
			scan=0;
			EnvoiUserUdp(envoiFin);
			flag_envoi = 0;
		}
		if(flag_distance) 
		{
			/*distanceRestante = (int)Stop(2);	// Abrupt
			messdistance[2] = distanceRestante >> 8;
			messdistance[3] = distanceRestante & 0xFF;
			EnvoiUserUdp(envoiDistance);*/
			flag_distance = 0;
		}
		if(flag_blocage)
		{
			EnvoiUserUdp(envoiBlocage);
			flag_blocage = 0;
		}
		if(flag_calage)
		{
			EnvoiUserUdp(envoiCalage);
			flag_calage = 0;
		}
		
//		//	motor_flag=Motors_Task(); // Si prend trop de ressource sur l'udp, inclure motortask dans le main	
//		/*
//		if(courrier)
//		{
//			switch(motor_flag)
//			{
//				case 0x00:		break;
//				case 0x01:
//				case 0x02:
//				case 0x03:
//				case 0x04:
//				case 0x05:	messEtape[0] = 1;
//							messEtape[1] = 0x21;
//							messEtape[2] = motor_flag;
//							flagEtape.message = messEtape;
//							flagEtape.nbChar = 3;
//							//EnvoiUserUdp(flagEtape);			//A remettre pour etape
//							break;
//						
//		
//				case 0x10:	EnvoiUserUdp(envoiFin);
//							break;
//				case 0x20:	//if(hold_blocage==0)                // A remettre pour blocage 
//							//EnvoiUserUdp(envoiBlocage);
//							//hold_blocage=1;
//							break;
//				case 0xFF:	//EnvoiUserUdp(envoiBlocage); // A coder...
//							datalogger_blocker=1;
//							Stop(ABRUPT);
//							for(dataplayer_counter=0;dataplayer_counter<500;dataplayer_counter++)
//							{
//								if(datalogger_counter>0)	datalogger_counter--;
//								else						datalogger_counter=499;
//								manual_pid(datalogger_ga[datalogger_counter],datalogger_dr[datalogger_counter]);
//								Tempo1mS(1);
//							}
//							datalogger_blocker=0;
//							break;
//				default:
//							break;
//			}
//			if(motor_flag != 0x20) hold_blocage=0;
//			courrier=0;
//		}
//		*/
//		
//
///*		for(i=0;i<N;i++) // Constat, au bout de quelques run demo=4, il y a un depassement systématique de l'erreur ; peut être que c'était a cause de abs() au lieu de fabs()
//		{
//			if((fabs(cons_pos[0]-real_pos[0]) > 10) || (fabs(cons_pos[1]-real_pos[1]) > 10))
//			{
//				// Seuil de detection blocage
//				distanceRestante = fabs(((targ_pos[0]-real_pos[0])+targ_pos[1]-real_pos[1])/2);
//
//				while(1)
//				{
//					pwm(GAUCHE,-500);
//					pwm(DROITE,500);
//				}
//			}
//		}
//
//		*/
//    	
		StackTask();
		trame = ReceptionUserUdp();
		if(trame.nbChar != 0)
		{
			trame = AnalyseTrame(trame);
			EnvoiUserUdp(trame);
		}
        StackApplications();

		if(dwLastIP != AppConfig.MyIPAddr.Val)
		{
			dwLastIP = AppConfig.MyIPAddr.Val;
			
			#if defined(STACK_USE_UART)
				putrsUART((ROM char*)"\r\nNew IP Address: ");
			#endif

			DisplayIPValue(AppConfig.MyIPAddr);

			#if defined(STACK_USE_UART)
				putrsUART((ROM char*)"\r\n");
			#endif


			#if defined(STACK_USE_ANNOUNCE)
				AnnounceIP();
			#endif
		}
	}
}

// Writes an IP address to the LCD display and the UART as available
void DisplayIPValue(IP_ADDR IPVal)
{
//	printf("%u.%u.%u.%u", IPVal.v[0], IPVal.v[1], IPVal.v[2], IPVal.v[3]);
    BYTE IPDigit[4];
	BYTE i;
#ifdef USE_LCD
	BYTE j;
	BYTE LCDPos=16;
#endif

	for(i = 0; i < sizeof(IP_ADDR); i++)
	{
	    uitoa((WORD)IPVal.v[i], IPDigit);

		#if defined(STACK_USE_UART)
			putsUART(IPDigit);
		#endif

		#ifdef USE_LCD
			for(j = 0; j < strlen((char*)IPDigit); j++)
			{
				LCDText[LCDPos++] = IPDigit[j];
			}
			if(i == sizeof(IP_ADDR)-1)
				break;
			LCDText[LCDPos++] = '.';
		#else
			if(i == sizeof(IP_ADDR)-1)
				break;
		#endif

		#if defined(STACK_USE_UART)
			while(BusyUART());
			WriteUART('.');
		#endif
	}

	#ifdef USE_LCD
		if(LCDPos < 32u)
			LCDText[LCDPos] = 0;
		LCDUpdate();
	#endif
}

//#pragma romdata MACROM=0x1FFF0
static ROM BYTE SerializedMACAddress[6] = {MY_DEFAULT_MAC_BYTE1, MY_DEFAULT_MAC_BYTE2, MY_DEFAULT_MAC_BYTE3, MY_DEFAULT_MAC_BYTE4, MY_DEFAULT_MAC_BYTE5, MY_DEFAULT_MAC_BYTE6};
//#pragma romdata

static void InitAppConfig(void)
{
	AppConfig.Flags.bIsDHCPEnabled = TRUE;
	AppConfig.Flags.bInConfigMode = TRUE;
	memcpypgm2ram((void*)&AppConfig.MyMACAddr, (ROM void*)SerializedMACAddress, sizeof(AppConfig.MyMACAddr));
//	{
//		_prog_addressT MACAddressAddress;
//		MACAddressAddress.next = 0x157F8;
//		_memcpy_p2d24((char*)&AppConfig.MyMACAddr, MACAddressAddress, sizeof(AppConfig.MyMACAddr));
//	}
	AppConfig.MyIPAddr.Val = MY_DEFAULT_IP_ADDR_BYTE1 | MY_DEFAULT_IP_ADDR_BYTE2<<8ul | MY_DEFAULT_IP_ADDR_BYTE3<<16ul | MY_DEFAULT_IP_ADDR_BYTE4<<24ul;
	AppConfig.DefaultIPAddr.Val = AppConfig.MyIPAddr.Val;
	AppConfig.MyMask.Val = MY_DEFAULT_MASK_BYTE1 | MY_DEFAULT_MASK_BYTE2<<8ul | MY_DEFAULT_MASK_BYTE3<<16ul | MY_DEFAULT_MASK_BYTE4<<24ul;
	AppConfig.DefaultMask.Val = AppConfig.MyMask.Val;
	AppConfig.MyGateway.Val = MY_DEFAULT_GATE_BYTE1 | MY_DEFAULT_GATE_BYTE2<<8ul | MY_DEFAULT_GATE_BYTE3<<16ul | MY_DEFAULT_GATE_BYTE4<<24ul;
	AppConfig.PrimaryDNSServer.Val = MY_DEFAULT_PRIMARY_DNS_BYTE1 | MY_DEFAULT_PRIMARY_DNS_BYTE2<<8ul  | MY_DEFAULT_PRIMARY_DNS_BYTE3<<16ul  | MY_DEFAULT_PRIMARY_DNS_BYTE4<<24ul;
	AppConfig.SecondaryDNSServer.Val = MY_DEFAULT_SECONDARY_DNS_BYTE1 | MY_DEFAULT_SECONDARY_DNS_BYTE2<<8ul  | MY_DEFAULT_SECONDARY_DNS_BYTE3<<16ul  | MY_DEFAULT_SECONDARY_DNS_BYTE4<<24ul;


	// SNMP Community String configuration
	#if defined(STACK_USE_SNMP_SERVER)
	{
		BYTE i;
		static ROM char * ROM cReadCommunities[] = SNMP_READ_COMMUNITIES;
		static ROM char * ROM cWriteCommunities[] = SNMP_WRITE_COMMUNITIES;
		ROM char * strCommunity;
		
		for(i = 0; i < SNMP_MAX_COMMUNITY_SUPPORT; i++)
		{
			// Get a pointer to the next community string
			strCommunity = cReadCommunities[i];
			if(i >= sizeof(cReadCommunities)/sizeof(cReadCommunities[0]))
				strCommunity = "";

			// Ensure we don't buffer overflow.  If your code gets stuck here, 
			// it means your SNMP_COMMUNITY_MAX_LEN definition in TCPIPConfig.h 
			// is either too small or one of your community string lengths 
			// (SNMP_READ_COMMUNITIES) are too large.  Fix either.
			if(strlenpgm(strCommunity) >= sizeof(AppConfig.readCommunity[0]))
				while(1);
			
			// Copy string into AppConfig
			strcpypgm2ram((char*)AppConfig.readCommunity[i], strCommunity);

			// Get a pointer to the next community string
			strCommunity = cWriteCommunities[i];
			if(i >= sizeof(cWriteCommunities)/sizeof(cWriteCommunities[0]))
				strCommunity = "";

			// Ensure we don't buffer overflow.  If your code gets stuck here, 
			// it means your SNMP_COMMUNITY_MAX_LEN definition in TCPIPConfig.h 
			// is either too small or one of your community string lengths 
			// (SNMP_WRITE_COMMUNITIES) are too large.  Fix either.
			if(strlenpgm(strCommunity) >= sizeof(AppConfig.writeCommunity[0]))
				while(1);

			// Copy string into AppConfig
			strcpypgm2ram((char*)AppConfig.writeCommunity[i], strCommunity);
		}
	}
	#endif

	// Load the default NetBIOS Host Name
	memcpypgm2ram(AppConfig.NetBIOSName, (ROM void*)MY_DEFAULT_HOST_NAME, 16);
	FormatNetBIOSName(AppConfig.NetBIOSName);

	#if defined(ZG_CS_TRIS)
		// Load the default SSID Name
		if (sizeof(MY_DEFAULT_SSID_NAME) > sizeof(AppConfig.MySSID))
		{
		    ZGSYS_DRIVER_ASSERT(5, (ROM char *)"AppConfig.MySSID[] too small.\n");
		}
		memcpypgm2ram(AppConfig.MySSID, (ROM void*)MY_DEFAULT_SSID_NAME, sizeof(MY_DEFAULT_SSID_NAME));
	#endif

	#if defined(EEPROM_CS_TRIS)
	{
		BYTE c;
		
	    // When a record is saved, first byte is written as 0x60 to indicate
	    // that a valid record was saved.  Note that older stack versions
		// used 0x57.  This change has been made to so old EEPROM contents
		// will get overwritten.  The AppConfig() structure has been changed,
		// resulting in parameter misalignment if still using old EEPROM
		// contents.
		XEEReadArray(0x0000, &c, 1);
	    if(c == 0x60u)
		    XEEReadArray(0x0001, (BYTE*)&AppConfig, sizeof(AppConfig));
	    else
	        SaveAppConfig();
	}
	#elif defined(SPIFLASH_CS_TRIS)
	{
		BYTE c;
		
		SPIFlashReadArray(0x0000, &c, 1);
		if(c == 0x60u)
			SPIFlashReadArray(0x0001, (BYTE*)&AppConfig, sizeof(AppConfig));
		else
			SaveAppConfig();
	}
	#endif
}

#if defined(EEPROM_CS_TRIS) || defined(SPIFLASH_CS_TRIS)
void SaveAppConfig(void)
{
	// Ensure adequate space has been reserved in non-volatile storage to 
	// store the entire AppConfig structure.  If you get stuck in this while(1) 
	// trap, it means you have a design time misconfiguration in TCPIPConfig.h.
	// You must increase MPFS_RESERVE_BLOCK to allocate more space.
	#if defined(STACK_USE_MPFS) || defined(STACK_USE_MPFS2)
		if(sizeof(AppConfig) > MPFS_RESERVE_BLOCK)
			while(1);
	#endif

	#if defined(EEPROM_CS_TRIS)
	    XEEBeginWrite(0x0000);
	    XEEWrite(0x60);
	    XEEWriteArray((BYTE*)&AppConfig, sizeof(AppConfig));
    #else
	    SPIFlashBeginWrite(0x0000);
	    SPIFlashWrite(0x60);
	    SPIFlashWriteArray((BYTE*)&AppConfig, sizeof(AppConfig));
    #endif
}
#endif


void __attribute__ ((interrupt, no_auto_psv)) _T2Interrupt(void) 
{
	flag = 0;
	courrier = 1;
	motor_flag = Motors_Task(); // Si prend trop de ressource sur l'udp, inclure motortask dans le main	
	if(motor_flag == 0x10)
	{
		motor_flag=0;
		flag_envoi=1;
	}
	if(motor_flag == 0x20)
	{
		motor_flag=0;
		flag_distance=1;
	}
	if(motor_flag == 0x30)
	{
		motor_flag=0;
		flag_calage=1;
	}
	if(motor_flag == 0x40)
	{
		motor_flag=0;
		flag_blocage=1;
	}
	
	/*if(datalogger_blocker==0)
	{
		if(++datalogger_counter>500) 
			datalogger_counter = 0;
		datalogger_ga[datalogger_counter] = (int)Motors_GetPosition(MOTEUR_GAUCHE);
		datalogger_dr[datalogger_counter] = (int)Motors_GetPosition(MOTEUR_DROIT);
	}*/
	PID_ressource_used = TMR2;

	Cpt_Tmr2_Capteur_Couleur++;

	if(Cpt_Tmr2_Capteur_Couleur == 20)
	{
		Cpt_Tmr2_Capteur_Couleur = 0;

		switch(++etat_Capteur_Couleur){

			case 1:
				Tab_Capteur_Couleur[0] = Send_Variable_Capteur_Couleur();
				S3  = 0; 
				S2  = 0;
				LED = 0;
			break;
			case 2:
				Tab_Capteur_Couleur[1] = Send_Variable_Capteur_Couleur();
				S3  = 1; 
				S2  = 0;
				LED = 0;
			break;
			case 3:
				Tab_Capteur_Couleur[2] = Send_Variable_Capteur_Couleur();
				S3  = 0; 
				S2  = 1;
				LED = 0;
			break;
			case 4:
				Tab_Capteur_Couleur[3] = Send_Variable_Capteur_Couleur();
				S3  = 1; 
				S2  = 1;
				LED = 0;
			break;
			case 5:
				Tab_Capteur_Couleur[4] = Send_Variable_Capteur_Couleur();
				S3  = 0; 
				S2  = 0;
				LED = 1;
			break;
			case 6:
				Tab_Capteur_Couleur[5] = Send_Variable_Capteur_Couleur();
				S3  = 1; 
				S2  = 0;
				LED = 1;
			break;
			case 7:
				Tab_Capteur_Couleur[6] = Send_Variable_Capteur_Couleur();
				S3  = 0; 
				S2  = 1;
				LED = 1;
			break;
			case 8:
				Tab_Capteur_Couleur[7] = Send_Variable_Capteur_Couleur();
				S3  = 1; 
				S2  = 1;
				LED = 1;
				etat_Capteur_Couleur = 0;
			break;
		}
	}
	IFS0bits.T2IF = 0;
}
