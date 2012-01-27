/* ********************************************************************** */
/*                      Pre-Fire Mapping System                           */
/*                   Hokuyo LIDAR Communication Code                      */
/*                                                                        */
/* Authors : William Etter (MSE '11)                                      */
/*                                                                        */
/*                      University of Pennsylvania                        */
/* mLab - Real-Time Embedded Systems Laboratory                           */
/* Date : September  30, 2011                                             */
/* Version : 1.0                                                          */
/* Hardware : Hoyuko Laser RangeFinder, Pandaboard                        */
/* Copyright William Etter 2011 (Etterw@seas.upenn.edu)                   */
/* ********************************************************************** */

/* ****************************************************************************** */
/* *******************************   About  ************************************* */
/* ****************************************************************************** */
// This code contains LIDAR communication protocols

/* ****************************************************************************** */
/* ********************   Configuration Definitions  **************************** */
/* ****************************************************************************** */
/*	(Information obtained from Hokuyo "Communicatin Protocol Specification for 
	SCIP2.0 Standard", C-42-03320B)

	LF = Line Feed, CR = Carriage Return 	

	HOST -> SENSOR
	Command Symbol + Parameter + String Characters (Max 16) + LF/CR (or both)

	SENSOR -> HOST
	Command Symbol + Parameter + String Characters + LF + Status + Sum + LF + Data + Sum + LF + LF
	(Note: Command Echo followed by LF + Status + Sum + LF + Data + Sum + LF + LF)
	
	Command Symbol: 2 bytes at the beginning of every command.
	Parameters: information required fo change sensors settings or request additional data.
	String Characters: optional information in the command.  Used to verify the reply when the
		same command is repeated more than once, such as by sending different String
		Characters and checking the echo.  Max length is 16 characters made up of
		A-Z,a-z,0-9,(blank space), ., _, +, @.  Seperated by a semicolon ';' at the beginning
		to seperate from parameter.
	Line Feed (LF) or Carriage Return (CR): termination code.  Command will have either or both
		but reply will aways have two LF termination code.
	Status: 2 bytes data in the reply that indicates normal processing or error status.
		Status other than 00 or 99 are error codes
	Sum: 1 byte data used for authentication of the sent data.  Calculated by adding data between
		two LFs, taking the lower 6 bits and adding 0x30.
	Data: the main information related to the command.  If over 64 bytes, seperate by LF and sum
		every 64 bytes.
	IMPORTANT: DO NOT USE COMMAND $(24H).  THIS IS A SPECIAL RESERVED COMMAND.

	
	
	Sensor Commands-------------------------------------------------------------
	MDMS-Command: Sensor Data Acquisition Control
		M + (D or S) + (Starting Step) + (End Step) + (Cluster Count) + (Scan Interval) + 
			(Number of Scans) + (String Characters) + LF		

		Sensor transmits command echo with status '00' followed by reply with measurement data.
		Laser switches on automatically before the measurement and switches off after completing
		the number of scans defined in the command.
		
		MD = 3-Character encoded data (M=4dH D=44H)
		MS = 2-Character encoded data (M=4dH S=53H)
		Starting Step and End Step = 4-bytes each, between 0 and maximum step (End > Starting)
			Example: From 10 to 750
			10 = 0010 = (0x30,0x30,0x31,0x30)
			750 = 0750 = (0x30,0x37,0x35,0x30)
		Cluster Count = number of adjacent steps that can be merged into single data point
			Example: Cluster Count = 3, if 3 adjacents steps are 3059, 3055, and 3062
			received data will be 3055.  2-bytes
		Scan Interval = 1-byte decimal, number of skipped scans when obtaining multiple scan data
		Number of Scans = 2-bytes decimal, number of scans to receive data for.  If set to '00' then
			data is supplied indefinitely until canceled using [QT-Command] or [RS-Command].
		String Character = Command verification for reply (max of 16 letters)
		
		
		
*/

/* ****************************************************************************** */
/* ******************************   TO DO  ************************************** */
/* ****************************************************************************** */



/* ****************************************************************************** */
/* ****************************** Includes ************************************** */
/* ****************************************************************************** */
#ifndef GUARD
	#define GUARD
	#include "prefiremapping.h"
#endif

char dataheader[26] = "0000000000000000000000000\n";
uint32_t datanum;
uint32_t datablocks;
uint32_t remaining;
size_t temp;
char mainblock[64];
char remblock[64];
char remainingblock[REMBLOCK];
char csum[1];
char chartemp[64];
uint8_t checksum;
int i;

/*************************************************************************
Function: twocharEncode()
Purpose:  Encodes 12-bit data into 2 Characters.  Separates into lower and higher 6-bit 
          numbers, adds 0x30 to convert to ASCII, and returns the chars.
Input:    Data having a maximum length of 12 bits, location of the chars to output 
**************************************************************************/
void twocharEncode(uint16_t input,char * output){
	uint8_t low = input & 0x003F;
	uint8_t high = (input & 0x0FC0) >> 6;
	output[0] = high + 0x30;
	output[1] = low + 0x30;
}

/*************************************************************************
Function: twocharDecode()
Purpose:  Decodes 12-bit data (2-Characters) into decimal equivalent.  Subtracts
          0x30 from each and then merges to form decimal output.
Input:    2-Character Encoding of Data, location of decimal to output
**************************************************************************/
void twocharDecode(char * input,uint16_t * output){
	uint8_t high = input[0] - 0x30;
	uint8_t low = input[1] - 0x30;
	*output = (high << 6) + low;
}

/*************************************************************************
Function: threecharEncode()
Purpose:  Encodes 18-bit data into 3 Characters.  Separates into lower, middle,
          and higher 6-bit numbers, adds 0x30 to convert to ASCII, and returns the chars.
Input:    Data having a maximum length of 18 bits, location of the chars to output  
**************************************************************************/
void threecharEncode(uint16_t input,char * output){
	uint8_t low = input & 0x003F;
	uint8_t middle = (input & 0x0FC0) >> 6;
	uint8_t high = (input & 0x03F000) >> 12;
	output[0] = high + 0x30;
	output[1] = middle + 0x30;
	output[2] = low + 0x30;
}

/*************************************************************************
Function: threecharDecode()
Purpose:  Decodes 18-bit data (3-Characters) into decimal equivalent.  Subtracts
          0x30 from each and then merges to form decimal output.
Input:    3-Character Encoding of Data, location of decimal to output
**************************************************************************/
void threecharDecode(char * input,uint16_t * output){
	uint8_t high = input[0] - 0x30;
	uint8_t middle = input[1] - 0x30;
	uint8_t low = input[2] - 0x30;
	*output = (high << 12) + (middle << 6) + low;
}

/*************************************************************************
Function: fourcharEncode()
Purpose:  Encodes 24-bit data into 4 Characters.  Separates into lower, midlow,
          midhigh, and higher 6-bit numbers, adds 0x30 to convert to ASCII, and
          returns the chars.
Input:    Data having a maximum length of 24 bits, location of the chars to output
**************************************************************************/
void fourcharEncode(uint32_t input,char * output){
	uint8_t low = input & 0x003F;
	uint8_t midlow = (input & 0x0FC0) >> 6;
	uint8_t midhigh = (input & 0x03F000) >> 12;
	uint8_t high = (input & 0x00FC0000) >> 18;
	output[0] = high + 0x30;
	output[1] = midhigh + 0x30;
	output[2] = midlow + 0x30;
	output[3] = low + 0x30;
}

/*************************************************************************
Function: fourcharDecode()
Purpose:  Decodes 24-bit data (4-Characters) into decimal equivalent.  Subtracts
          0x30 from each and then merges to form decimal output.
Input:    4-Character Encoding of Data, location of decimal to output
**************************************************************************/
void fourcharDecode(char * input,uint32_t * output){
	uint8_t high = input[0] - 0x30;
	uint8_t midhigh = input[1] - 0x30;
	uint8_t midlow = input[2] - 0x30;
	uint8_t low = input[3] - 0x30;
	*output = (high << 18) + (midhigh << 12) + (midlow << 6) + low;
}

/*************************************************************************
Function: checkSum()
Purpose:  Computes the LIDAR checksum
Input:    Input string, length
Output:	  Checksum value
**************************************************************************/
uint8_t checkSum(char * input,int length){
	uint32_t sum = 0;
	int i = 0;
	for (i = 0; i < length; i++){
		sum += (uint8_t)input[i];
	}
	return ((sum & 0x3F) + 0x30);
}

/*************************************************************************
Function: commandNumber()
Purpose:  Computes the LIDAR Command Number to ensure data is received correctly
Input:    Input command string
Output:	  Command Number
**************************************************************************/
uint8_t commandNumber(char * input){	
	if((strncmp(input,"MD",2)) == 0){
		// MD Laser Acquisition Command (3-Character Encoded Data)
		return 0;
	}
	else if((strncmp(input,"MS",2)) == 0){
		// MS Laser Acquisition Command (2-Character Encoded Data)
		return 1;
	}
	else if((strncmp(input,"GD",2)) == 0){
		// GD Laser Acquition Command (3-Character Encoded Data)
		return 2;
	}
	else if((strncmp(input,"GS",2)) == 0){
		// GS Laser Acquition Command (2-Character Encoded Data)
		return 3;
	}
	else if((strncmp(input,"BM",2)) == 0){
		// BM Laser ON Command
		return 4;
	}
	else if((strncmp(input,"QT",2)) == 0){
		// QT Laser OFF Command
		return 5;
	}
	else if((strncmp(input,"RS",2)) == 0){
		// RS Laser Reset to Default Command
		return 6;
	}
	else if((strncmp(input,"TM",2)) == 0){
		// TM Laser Time Adjust Command
		return 7;
	}
	else if((strncmp(input,"SS",2)) == 0){
		// SS Laser Bit Rate Command
		return 8;
	}
	else if((strncmp(input,"CR",2)) == 0){
		// CR Laser Motor Speed Command
		return 9;
	}
	else if((strncmp(input,"HS",2)) == 0){
		// HS Laser Sensitivity Command
		return 10;
	}
	else if((strncmp(input,"DB",2)) == 0){
		// DB Laser Malfunction Simulation Command
		return 11;
	}
	else if((strncmp(input,"VV",2)) == 0){
		// VV Laser Detail Command
		return 12;
	}
	else if((strncmp(input,"PP",2)) == 0){
		// PP Laser Specifications Command
		return 13;
	}
	else if((strncmp(input,"II",2)) == 0){
		// II Laser Running State Specifications Command
		return 14;
	}
	else
		return 20;
}

/*************************************************************************
Function: computeTime()
Purpose:  Computes the LIDAR Time from 4 Byte Value
Input:    Input string
Output:	  Time value
**************************************************************************/
int computeTime(char * input){
	int time = 0;
	time = (int)input[3] + (int)input[2]*16 + (int)input[1]*32 + (int)input[0]*64;
	return time;
}

/*************************************************************************
Function: readData()
Purpose:  Computes the LIDAR sensor data
Input:    Encoding, StartStep, EndStep, Cluster, ScanInterval, Device to Read From
**************************************************************************/
void readData(uint8_t encoding, uint32_t startstep, uint32_t endstep, uint32_t cluster, uint32_t scaninterval, FILE * filedescriptor){
	// Number of data points is equal to (End Step - Startstep + 1) divided by the  cluster number. (For example, 3 -> 7 = 7-3+1 = 5 data points (3,4,5,6,7)
	datanum = (endstep-startstep+1)/cluster;
	
	// Decode base on encoding method
	if(encoding == 2){ // 2-Character Encoding
		datanum*=2;
	}
	else if(encoding == 3){ // 3-Character Encoding
		datanum *=3;
	}

	datablocks = datanum/64;
	remaining = datanum%64;
	// Read in data header
	temp = fread(dataheader,1,26,filedescriptor);

	// Loop through full blocks of data
	for (i = 0; i < datablocks; i++){
		// Copy Data
		temp = fread(mainblock,1,64,filedescriptor);
		// Get Sum
		temp = fread(csum,1,1,filedescriptor);
	
		checksum = checkSum(mainblock,64);
		printf("Checksum = %d\n",checksum);
		printf("Csum = %d\n",(uint8_t)csum[0]);
		if((uint8_t)csum[0] == checksum){
			// Checksum matches - If so, decode data and put into array
			printf("CHECKSUM MATCHES!!!!!\n");
			printf("Mainblock = %s\n",mainblock);
		}
		else{
			// Check sum didn't match, report error 
		}
		temp = fread(chartemp,1,1,filedescriptor);	// Read in last LF
	}

	// Read remaining data
		temp = fread(remblock,1,47,filedescriptor);
		temp = fread(csum,1,1,filedescriptor);
		printf("Remaining block: %s\n",remblock);
		checksum = checkSum(remblock,47);
		//checksum = checkSum(remblock,remaining);
		printf("Rem Checksum = %d\n",checksum);
		printf("Rem Csum = %d\n",(uint8_t)csum[0]);
	
		temp = fread(chartemp,1,2,filedescriptor);	// Read in last LF LF
		
		printf("Test!\n");
		printf("Datablocks = %d\n",datablocks);
		printf("Remaining blocks = %d\n",remaining);



/*

char dataheader[27] = "00000000000000000000000000\n";
int datanum;
int datablocks;
int remaining;
size_t temp;
*/
}

/* ****************************************************************************** */
// End of HOKUYO_COMM.C
/* ****************************************************************************** */
