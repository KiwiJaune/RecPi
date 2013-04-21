void InitClk(void);
void InitPorts(void);
void InitPWM(void); 
void InitQEI(void);
void InitT2(void);
void InitADC(void);
void InitDMA(void);
void Init_Clk(void);

#define  MAX_CHNUM	 			7		// Highest Analog input number in Channel Scan
#define  SAMP_BUFF_SIZE	 		8		// Size of the input buffer per analog input
#define  NUM_CHS2SCAN			8		// Number of channels enabled for channel scan
