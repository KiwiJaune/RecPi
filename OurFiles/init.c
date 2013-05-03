#include <p33FJ128MC804.h>
#include "init.h"

unsigned int  BufferA[MAX_CHNUM+1][SAMP_BUFF_SIZE] __attribute__((space(dma),aligned(256)));
unsigned int  BufferB[MAX_CHNUM+1][SAMP_BUFF_SIZE] __attribute__((space(dma),aligned(256)));
unsigned int ADC_Results[8],DmaBuffer = 0;

void InitClk(void)
{	
	PLLFBD = 38;				// Multiply by 40 for 160MHz VCO output (8MHz XT oscillator)
	CLKDIV = 0x0000;			// FRC: divide by 2, PLLPOST: divide by 2, PLLPRE: divide by 2
	
	__builtin_write_OSCCONH(0x03);
	__builtin_write_OSCCONL(OSCCON | 0x01);
	while(OSCCONbits.COSC != 0b011);
	while(OSCCONbits.LOCK != 1);
}

void Init_Input_Capture(void)
{
	
	//Set Timer3 to Capture
	T3CONbits.TON 	= 0;	//Stops the timer
	T3CONbits.TSIDL = 0;
	T3CONbits.TGATE = 0;
	T3CONbits.TCS	= 0;
	T3CONbits.TCKPS = 0b10; //Prescaler set to 1:1

//	IPC2bits.T3IP = 7; 		//Set Timer2 Interrupt Priority Level
//	IFS0bits.T3IF = 0; 		//Clear Timer2 Interrupt Flag
	IEC0bits.T3IE = 0; 		//Enable Timer2 interrupt	
	
	T3CONbits.TON = 1;

	//Configurer le pin Out en RP
	RPINR7bits.IC1R 	= 1;	//RP1

	IC1CONbits.ICM		= 0;
	IC1CONbits.ICSIDL 	= 1;
	IC1CONbits.ICTMR	= 0;		//TM3
	IC1CONbits.ICI		= 0b00;
	IC1CONbits.ICM		= 0b101;		
		
	IPC0bits.IC1IP = 7;
	IFS0bits.IC1IF = 0;
	IEC0bits.IC1IE = 1;
}

void InitPWM(void)
{
	P1TCONbits.PTEN = 1; 		// PWM Time base is On
	P1TPER = 2000 - 1; 			// 20kHz PWM (2000 counts @40MIPS)
	PWM1CON1bits.PEN1L = 1;		// PWM1L1 pin is enabled for PWM output
	PWM1CON1bits.PEN2L = 1;		// PWM1L2 pin is enabled for PWM output

	P1DC1 = 0xFFFF;	// 0x0000 = 100.00% Power
	P1DC2 = 0xFFFF;	// 0xFFFF =   0.00% Power
}

void InitQEI(void)
{
	QEI1CONbits.QEIM  = 0b111;
	QEI2CONbits.QEIM  = 0b111;
	QEI1CONbits.SWPAB = 1;
	QEI2CONbits.SWPAB = 0;
	POS1CNT = 0x0000;
	POS2CNT = 0x0000;
	//DFLT1CONbits.QECK = 0b111;
	//DFLT2CONbits.QECK = 0b111;
	IFS3bits.QEI1IF = 0;
	IFS4bits.QEI2IF = 0;
	IEC3bits.QEI1IE = 1;
	IEC4bits.QEI2IE = 1;
}

void InitPorts() 
{
	// Pinout date : 08_04_13
	// Ports A
	// 0 | I | RA0  : Coupure alim cam�ra
	// 1 | O | RA1  : shutter tuy�re
	// 2 | I | RA2  : OSC
	// 3 | O | RA3  : Coupure_Alim
	// 4 | I | RA4  : ETH_RST
	// 5 
	// 6 
	// 7 | ? | RA7  : capteur couleur
	// 8 | I | RA8  : Jack
	// 9 | O | RA9  : ETH_CS
	// A | ? | RA10 : capteur couleur
	
	// Ports B
	// 0 | ? | RP0  | RB0  : capteur couleur
	// 1 | ? | RP1  | RB1  : capteur couleur
	// 2 | O | RP2  | RB2  : DIR_CDS
	// 3 | I | RP3  | RB3  : codeur_Droit_A
	// 4 | I | RP4  | RB4  : Interrupteur_Couleur
	// 5 | I | RP5  | RB5  : PROG_PGD & Capteur pr�sence balle
	// 6 | I | RP6  | RB6  : PROG_PGC & Switchs assiette
	// 7 | I | RP7  | RB7  : ETH_INT
	// 8 | O | RP8  | RB8  : I2C_SCL
	// 9 | X | RP9  | RB9  : I2C_SDA
	// A | O | RP10 | RB10 : Pompe � vide
	// B | O | RP11 | RB11 : Servo assiette
	// C | O | RP12 | RB12 : Moteur_Gauche_DIR
	// D | O | RP13 | RB13 : Moteur_Gauche_PWM
	// E | O | RP14 | RB14 : Moteur_Droit_DIR
	// F | O | RP15 | RB15 : Moteur_Droit_PWM
	
	// Ports C
	// 0 | I | RP16 | RC0  : Codeur_Droit_B
	// 1 | I | RP17 | RC1  : Codeur_Gauche_A
	// 2 | I | RP18 | RC2  : Codeur_Gauche_B
	// 3 | O | RP19 | RC3  : Ethernet_SDO1
	// 4 | O | RP20 | RC4  : Ethernet_SCK1
	// 5 | I | RP21 | RC5  : Ethernet_SDI1
	// 6 | O | RP22 | RC6  : Moteur_Canon
	// 7 | O | RP23 | RC7  : Moteur_Turbine
	// 8 | I | RP24 | RC8  : CDS_RX
	// 9 | O | RP25 | RC9  : CDS_TX

	// * : Non utilis� pour le moment, donc configure en entree

	TRISA 	= 0b1111100101110101;
	//          FEDCBA9876543210
	TRISB 	= 0b0000001011111010;
	//          FEDCBA9876543210
	TRISC 	= 0b1111111000100111; //Modif de C9 et C8 : Valeur par default C9 = 0;C8 = 1
	//          FEDCBA9876543210
	
	AD1PCFGL=0xFFFF;	//Tous les ports Analogiques configur�s en num�rique
	
	
	RPINR16bits.QEA2R = 17;			// CHAd 		<==> RP17  //2013
	RPINR16bits.QEB2R = 18;			// CHBd			<==> RP18  //2013

	RPINR14bits.QEA1R = 3;			// CHAg			<==> RP3 
	RPINR14bits.QEB1R = 16;			// CHBg			<==> RP16 

	RPOR10bits.RP20R = 0b01000;  // SCK1 <==> RP20 RC4  //23/01/2013
	RPINR20bits.SDI1R = 0b10101; // SDI1 <==> RP21 RC5  //23/01/2013
	RPOR9bits.RP19R = 0b00111;   // SDO1 <==> RP19 RC3  //23/01/2013
	
	//Configuration des ports pour la liaison UART2 des CDS
//	RPINR19bits.U2RXR	= 24;	// Rx	<==> RP24-RC8
//	TRISCbits.TRISC9 	= 1;
//	RPOR12bits.RP25R	= 0b00101;	// Tx	<==> RP25-RC9
//	
	RPOR12bits.RP25R = 0b00011;     //TX RP24
    RPINR18 = 0b11000;              //RX RP25
    TRISCbits.TRISC8 = 0;
    TRISCbits.TRISC9 = 1;


	LATAbits.LATA3 = 0; 	// Alimentation OFF
	
	//Initialisation du sens de communication pour les AX12
	LATBbits.LATB2 = 1;	// 1 J'envoie et 0 je r�ceptionne
}

void InitT2(void)
{
	T2CONbits.TON 	= 0;	//Stops the timer
	T2CONbits.TSIDL = 0;
	T2CONbits.TGATE = 0;
	T2CONbits.TCS	= 0;
	T2CONbits.T32	= 0;
	T2CONbits.TCKPS = 0b01; //Prescaler set to 1:1
	
	TMR2 = 0; 				//Clear timer register
	PR2  = 5000;			//Load the period value (5000 = 0.5ms)

	IPC1bits.T2IP = 4; 		//Set Timer2 Interrupt Priority Level
	IFS0bits.T2IF = 0; 		//Clear Timer2 Interrupt Flag
	IEC0bits.T2IE = 1; 		//Enable Timer2 interrupt
	T2CONbits.TON = 1;		// Timer enabled
}

void InitADC(void)
{
	AD1CON1bits.FORM   = 0;		// Data Output Format: Integer
	AD1CON1bits.SSRC   = 7;		// Sample Clock Source: Conversion autostart
	AD1CON1bits.ASAM   = 1;		// ADC Sample Control: Sampling begins immediately after conversion
	AD1CON1bits.AD12B  = 1;		// 12-bit ADC operation

	AD1CON2bits.CSCNA = 1;		// Scan Input Selections for CH0+ during Sample A bit
	AD1CON2bits.CHPS  = 0;		// Converts CH0

	AD1CON3bits.ADRC = 0;		// ADC Clock is derived from Systems Clock
	AD1CON3bits.ADCS = 63;		// ADC Conversion Clock Tad=Tcy*(ADCS+1)= (1/40M)*64 = 1.6us (625Khz)
								// ADC Conversion Time for 10-bit Tc=12*Tab = 19.2us	

	AD1CON1bits.ADDMABM = 0; 	// DMA buffers are built in scatter/gather mode
	AD1CON2bits.SMPI    = (NUM_CHS2SCAN-1);	// 6 ADC Channel is scanned
	AD1CON4bits.DMABL   = 3;	// Each buffer contains 8 words

	//AD1CSSH/AD1CSSL: A/D Input Scan Selection Register
	AD1CSSLbits.CSS0=1;		// Enable AN0 for channel scan
	AD1CSSLbits.CSS1=1;		// Enable AN1 for channel scan
	AD1CSSLbits.CSS2=1;		// Enable AN2 for channel scan
	AD1CSSLbits.CSS3=1;		// Enable AN3 for channel scan
	AD1CSSLbits.CSS6=1;		// Enable AN6 for channel scan
	AD1CSSLbits.CSS7=1;		// Enable AN7 for channel scan
	
 	//AD1PCFGH/AD1PCFGL: Port Configuration Register
	AD1PCFGL=0xFFFF;
	AD1PCFGLbits.PCFG0 = 0;	// AN0 as Analog Input
	AD1PCFGLbits.PCFG1 = 0;	// AN1 as Analog Input 
 	AD1PCFGLbits.PCFG2 = 0;	// AN2 as Analog Input
	AD1PCFGLbits.PCFG3 = 0;	// AN3 as Analog Input 
	AD1PCFGLbits.PCFG6 = 0;	// AN6 as Analog Input
	AD1PCFGLbits.PCFG7 = 0;	// AN7 as Analog Input 
	
	IFS0bits.AD1IF   = 0;		// Clear the A/D interrupt flag bit
	IEC0bits.AD1IE   = 0;		// Do Not Enable A/D interrupt 
	AD1CON1bits.ADON = 1;		// Turn on the A/D converter
}

void InitDMA(void)
{
	DMA5CONbits.AMODE = 2;			// Configure DMA for Peripheral indirect mode
	DMA5CONbits.MODE  = 2;			// Configure DMA for Continuous Ping-Pong mode
	DMA5PAD=(int)&ADC1BUF0;
	DMA5CNT = (SAMP_BUFF_SIZE*NUM_CHS2SCAN)-1;					
	DMA5REQ = 13;					// Select ADC1 as DMA Request source

	DMA5STA = __builtin_dmaoffset(BufferA);		
	DMA5STB = __builtin_dmaoffset(BufferB);

	IFS3bits.DMA5IF = 0; //Clear the DMA interrupt flag bit
	IEC3bits.DMA5IE = 1; //Set the DMA interrupt enable bit
	
	DMA5CONbits.CHEN=1;				// Enable DMA
}

void __attribute__((interrupt, no_auto_psv)) _DMA5Interrupt(void)
{
	unsigned char i;
	ADC_Results[0]=0;
	ADC_Results[1]=0;
	ADC_Results[2]=0;
	ADC_Results[3]=0;
	ADC_Results[4]=0;
	ADC_Results[5]=0;
	if(DmaBuffer == 0)
	{
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[0] += BufferA[0][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[1] += BufferA[1][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[2] += BufferA[2][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[3] += BufferA[3][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[4] += BufferA[6][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[5] += BufferA[7][i];
		ADC_Results[0] /= SAMP_BUFF_SIZE;
		ADC_Results[1] /= SAMP_BUFF_SIZE;
		ADC_Results[2] /= SAMP_BUFF_SIZE;
		ADC_Results[3] /= SAMP_BUFF_SIZE;
		ADC_Results[4] /= SAMP_BUFF_SIZE;
		ADC_Results[5] /= SAMP_BUFF_SIZE;
	}
	else
	{
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[0] += BufferB[0][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[1] += BufferB[1][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[2] += BufferB[2][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[3] += BufferB[3][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[4] += BufferB[6][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[5] += BufferB[7][i];
		ADC_Results[0] /= SAMP_BUFF_SIZE;
		ADC_Results[1] /= SAMP_BUFF_SIZE;
		ADC_Results[2] /= SAMP_BUFF_SIZE;
		ADC_Results[3] /= SAMP_BUFF_SIZE;
		ADC_Results[4] /= SAMP_BUFF_SIZE;
		ADC_Results[5] /= SAMP_BUFF_SIZE;
	}
	
	DmaBuffer ^= 1;

	IFS3bits.DMA5IF = 0;		// Clear the DMA0 Interrupt Flag
}
