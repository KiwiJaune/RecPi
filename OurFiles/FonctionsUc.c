#include <p33FJ128MC804.h>
#include "FonctionsUc.h"

void InitUART2()
{

    U1BRG = 522;			// 9600
	U1MODEbits.UARTEN = 1;		// UART2 is Enabled
	U1MODEbits.USIDL = 0;		// Continue operation at Idlestate
	U1MODEbits.IREN = 0;		// IrDA En/Decoder is disabled
	U1MODEbits.RTSMD = 0; 		// flow control mode
	U1MODEbits.UEN = 0b00;		// UTX, RTX, U2CTS, U2RTS are enable and on use.
	U1MODEbits.WAKE = 0;		// Wake-up on start bit is enabled
	U1MODEbits.LPBACK = 0;		// Loop-back is disabled
	U1MODEbits.ABAUD = 0;		// auto baud is disabled
	U1MODEbits.URXINV = 0;		// No RX inversion
	U1MODEbits.BRGH = 1;		// low boud rate
	U1MODEbits.PDSEL = 0b00; 	// 8bit no parity
	U1MODEbits.STSEL = 0;		// one stop bit

	U1STAbits.UTXISEL1 = 0b00;
	U1STA &= 0xDFFF;			// clear TXINV by bit masking
	U1STAbits.UTXBRK = 0;		// sync break tx is disabled
	U1STAbits.UTXEN = 1;		// transmit  is enabled
	U1STAbits.URXISEL = 0b00;	// interrupt flag bit is set when RXBUF is filled whith 1 character
	U1STAbits.ADDEN = 0;		// address detect mode is disabled

//  IPC3bits.U1TXIP = 2;         // set UART Tx interrupt priority
	IFS0bits.U1TXIF = 0;         // clear UART Tx interrupt flag
	IEC0bits.U1TXIE = 0;         // enable UART Tx interrupt

	IFS0bits.U1RXIF = 0;		 // clear interrupt flag of rx
	IEC0bits.U1RXIE = 1;		 // enable rx recieved data
//	IPC2bits.U1RXIP = 1;
   
    RPOR12bits.RP25R = 0b00011;     //TX RP24
    RPINR18 = 0b11000;              //RX RP25
    TRISCbits.TRISC8 = 1;
    TRISCbits.TRISC9 = 0;
    AD1PCFGL = 0xFFFF;
}

/*******************************************************************************
Function: UART2PutChar( char ch )

Precondition:
    UART2Init must be called prior to calling this routine.

Overview:
    This routine writes a character to the transmit FIFO, and then waits for the
    transmit FIFO to be empty.

Input: Byte to be sent.

Output: None.

*******************************************************************************/
void UART2PutChar( char ch )
{
    U1TXREG = ch;
    while(U1STAbits.TRMT == 0);
}


