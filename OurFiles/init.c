#include <p33FJ128MC804.h>
#include "init.h"
#include "Pilotage.h"

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

void Init_Interrupt_Priority(void)
{
	
	IPC0bits.IC1IP  = 6;			//Input Capture used by Capteur Couleur	
	IPC1bits.IC2IP  = 6;
	IPC0bits.T1IP   = 4;	//5		//Timer 1 used by Ethernet (Default value = 2)
	IPC1bits.T2IP   = 5;	//6		//Timer 2 used to generate PWM
//	IPC2bits.T3IP   = 7;			//Interrupt UNUSED
	IPC2bits.U1RXIP = 4;			//UART RX Interrupt
	IPC3bits.U1TXIP = 4;			//UART TX Interrupt
	IPC6bits.T4IP   = 3;			//Timer 4 Used by Asser
	IPC7bits.T5IP   = 6;	//5		//Timer 5 Used by Canon
	IPC14bits.QEI1IP = 7;			//Quad Encoder Interrupt
	IPC18bits.QEI2IP = 7;			//Quad Encoder Interrupt
}


void InitPorts() 
{
	TRISAbits.TRISA0=1; //  - 
	TRISAbits.TRISA1=1; // VadcV2 (AN1) - Gestion d'alim
	TRISAbits.TRISA2=1; // Oscillateur 8MHz - Oscillateur
	TRISAbits.TRISA3=0; // Electroaimant - Lanceur
	TRISAbits.TRISA4=0; // RST - Ethernet
	TRISAbits.TRISA7=1; // switch - 
	TRISAbits.TRISA8=1; // switch - 
	TRISAbits.TRISA9=0; // CS - Ethernet
	TRISAbits.TRISA10=1; // switch - 
	TRISBbits.TRISB0=0; // Electroaimant - Lanceur
	TRISBbits.TRISB1=1; // CHA - Propulsion gauche
	TRISBbits.TRISB2=0; // Direction - CDS_AX12
	TRISBbits.TRISB3=1; // CHA - Propulsion droite
	TRISBbits.TRISB4=1; // switch - 
	TRISBbits.TRISB5=1; // PGD - JTAG 
	TRISBbits.TRISB6=1; // PGC - JTAG 
	TRISBbits.TRISB7=1; // INT - Ethernet
	TRISBbits.TRISB8=1; // SCL - I²C
	TRISBbits.TRISB9=1; // SDA - I²C
	TRISBbits.TRISB10=1; //  - 
	TRISBbits.TRISB11=1; //  - 
	TRISBbits.TRISB12=0; // DIR - Propulsion gauche
	TRISBbits.TRISB13=0; // PWM1L2 - Propulsion gauche
	TRISBbits.TRISB14=0; // DIR - Propulsion droite
	TRISBbits.TRISB15=0; // PWM1L1 - Propulsion droite
	TRISCbits.TRISC0=1; // CHB - Propulsion droite
	TRISCbits.TRISC1=1; // VadcV1 (AN7) - Gestion d'alim
	TRISCbits.TRISC2=1; // CHB - Propulsion gauche
	TRISCbits.TRISC3=0; // SDO1 - Ethernet
	TRISCbits.TRISC4=0; // SCK1 - Ethernet
	TRISCbits.TRISC5=1; // SDI1 - Ethernet
	TRISCbits.TRISC6=0; // servo - Bras drap bouchon
	TRISCbits.TRISC7=0; // servo - Canon filet
	TRISCbits.TRISC8=1; // RX - CDS_AX12
	TRISCbits.TRISC9=0; // TX - CDS_AX12
		
	AD1PCFGL=0xFFFF;	//Tous les ports Analogiques configurés en numérique
	
	RPINR14bits.QEA1R = 3;			// CHAd 		<==> RP3
	RPINR14bits.QEB1R = 16;			// CHBd			<==> RP16

	RPINR16bits.QEA2R = 1;			// CHAg			<==> RP1 
	RPINR16bits.QEB2R = 18;			// CHBg			<==> RP18

	RPOR10bits.RP20R = 0b01000; // SCK1 <==> RP20  
	RPINR20bits.SDI1R = 21; 	// SDI1 <==> RP21
	RPOR9bits.RP19R = 0b00111;  // SDO1 <==> RP19 
	
	//Configuration des ports pour la liaison UART2 des CDS
	RPINR19bits.U2RXR	= 24;		// Rx	<==> RP24-RC8
	RPOR12bits.RP25R	= 0b00101;	// Tx	<==> RP25-RC9
	
	
	//Initialisation du sens de communication pour les AX12
	LATBbits.LATB2 = 1;	// 1 J'envoie et 0 je réceptionne
}

void Init_Input_Capture(void)
{
	//Set Timer3 to Capture
	T3CONbits.TON 	= 0;	//Stops the timer
	T3CONbits.TSIDL = 0;
	T3CONbits.TGATE = 0;
	T3CONbits.TCS	= 0;
	T3CONbits.TCKPS = 0b10; //Prescaler set to 1:64

//	IPC2bits.T3IP = 7; 		//Set Timer3 Interrupt Priority Level
//	IFS0bits.T3IF = 0; 		//Clear Timer3 Interrupt Flag
	IEC0bits.T3IE = 0; 		//Disable Timer3 interrupt	
	
	T3CONbits.TON = 1;		//Starts the timer

	//Configurer le pin Out en RP
	RPINR7bits.IC1R 	= 1;	//RP1

	IC1CONbits.ICM		= 0;
	IC1CONbits.ICSIDL 	= 1;
	IC1CONbits.ICTMR	= 0;		//TM3
	IC1CONbits.ICI		= 0b00;
	IC1CONbits.ICM		= 0b101;		
		
//	IPC0bits.IC1IP = 7;
	IFS0bits.IC1IF = 0;
	IEC0bits.IC1IE = 1;
	
	// pour capteur vitesse
	//Configurer la pin d'entree
	RPINR7bits.IC2R 	= 8;	//RP8

	IC2CONbits.ICM		= 0;
	IC2CONbits.ICSIDL 	= 1;
	IC2CONbits.ICTMR	= 0;		//TM3
	IC2CONbits.ICI		= 0b00;
	IC2CONbits.ICM		= 0b010;		
		
//	IPC0bits.IC2IP = 7;
	IFS0bits.IC2IF = 0;
	IEC0bits.IC2IE = 1;

}
void Init_Timer(void)
{
	Init_Timer2();
	Init_Timer4();
	Init_Timer5();
}
void Init_Timer2(void)
{
	T2CONbits.TON 	= 0;	//Stops the timer
	T2CONbits.TSIDL = 0;
	T2CONbits.TGATE = 0;
	T2CONbits.TCS	= 0;
	T2CONbits.T32	= 0;
	T2CONbits.TCKPS = 0b10; //Prescaler set to 1:64
	
	TMR2 = 0; 				//Clear timer register
	PR2  = 1;				//Load the period value (1 = 3.2us)

//	IPC1bits.T2IP = 4; 		//Set Timer2 Interrupt Priority Level
	IFS0bits.T2IF = 0; 		//Clear Timer2 Interrupt Flag
	IEC0bits.T2IE = 1; 		//Enable Timer2 interrupt
	T2CONbits.TON = 1;		//Timer enabled
}

void Init_Timer4(void)
{
	//--Timer4
	T4CONbits.TON 	= 0;	//Stops the timer
	T4CONbits.TSIDL = 0;
	T4CONbits.TGATE = 0;
	T4CONbits.TCS	= 0;
	T4CONbits.TCKPS = 0b01; //Prescaler set to 1:8
	
	TMR4 = 0; 				//Clear timer register
	PR4  = 5000; 			//Load the period value (Pas) 1/(40e6/8/1250) = 1ms

//	IPC6bits.T4IP = 6; 		//Set Timer4 Interrupt Priority Level
	IFS1bits.T4IF = 0; 		//Clear Timer4 Interrupt Flag
	IEC1bits.T4IE = 1; 		//Enable Timer4 interrupt
	
	T4CONbits.TON = 1;		//Starts the timer
}

void Init_Timer5(void)
{
	//--Timer5
	T5CONbits.TON 	= 0;	//Stops the timer
	T5CONbits.TSIDL = 0;
	T5CONbits.TGATE = 0;
	T5CONbits.TCS	= 0;
	T5CONbits.TCKPS = 0b01; //Prescaler set to 1:8
	
	TMR5 = 0; 				//Clear timer register
	PR5  = 5000; 	    		//Load the period value (Pas) 1/(40e6/8/2500) = 1.6us

//	IPC7bits.T5IP = 7; 		//Set Timer5 Interrupt Priority Level
	IFS1bits.T5IF = 0; 		//Clear Timer5 Interrupt Flag
	IEC1bits.T5IE = 1; 		//Enable Timer5 interrupt

	//SIGNAL_CANON    = 0;
	//SIGNAL_TURBINE  = 0;
	//SIGNAL_ASSIETTE = 0;
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
	QEI1CONbits.SWPAB = 0;
	QEI2CONbits.SWPAB = 1;
	POS1CNT = 0x0000;
	POS2CNT = 0x0000;
	IFS3bits.QEI1IF = 0;
	IFS4bits.QEI2IF = 0;
	IEC3bits.QEI1IE = 1;
	IEC4bits.QEI2IE = 1;
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
	AD1CSSLbits.CSS2=0;		// Enable AN2 for channel scan
	AD1CSSLbits.CSS3=0;		// Enable AN3 for channel scan
	AD1CSSLbits.CSS6=0;		// Enable AN6 for channel scan
	AD1CSSLbits.CSS7=1;		// Enable AN7 for channel scan
	
 	//AD1PCFGH/AD1PCFGL: Port Configuration Register
	AD1PCFGL=0xFFFF;
	AD1PCFGLbits.PCFG0 = 1;	// AN0 as Digital Input
	AD1PCFGLbits.PCFG1 = 0;	// AN1 as Digital Input 
 	AD1PCFGLbits.PCFG2 = 1;	// AN2 as Digital Input
	AD1PCFGLbits.PCFG3 = 1;	// AN3 as Digital Input 
	AD1PCFGLbits.PCFG6 = 1;	// AN6 as Digital Input
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
