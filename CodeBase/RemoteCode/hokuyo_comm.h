/* ********************************************************************** */
/*                      Pre-Fire Mapping System                           */
/*               Hokuyo LIDAR Communication Code Header                   */
/*                            Remote Unit                                 */
/*                                                                        */
/* Authors : William Etter (MSE '11)                                      */
/*                                                                        */
/*                      University of Pennsylvania                        */
/* mLab - Real-Time Embedded Systems Laboratory                           */
/* Date : October  01, 2011                                               */
/* Version : 1.0                                                          */
/* Hardware : Hoyuko Laser RangeFinder, Pandaboard                        */
/* Copyright William Etter 2011 (Etterw@seas.upenn.edu)                   */
/* ********************************************************************** */
#ifndef _HOKUYO_COMM_H_
#define _HOKUYO_COMM_H_

/* ****************************************************************************** */
/* ****************************** Includes ************************************** */
/* ****************************************************************************** */



/* ****************************************************************************** */
/* ************************ Function Declarations ******************************* */
/* ****************************************************************************** */


/*************************************************************************
Function: twocharEncode()
Purpose:  Encodes 12-bit data into 2 Characters.  Separates into lower and higher 6-bit 
          numbers, adds 0x30 to convert to ASCII, and returns the chars.
Input:    Data having a maximum length of 12 bits, location of chars to output
**************************************************************************/
void twocharEncode(uint16_t input,char * output);

/*************************************************************************
Function: twocharDecode()
Purpose:  Decodes 12-bit data (2-Characters) into decimal equivalent.  Subtracts
          0x30 from each and then merges to form decimal output.
Input:    2-Character Encoding of Data, location of decimal to output
**************************************************************************/
void twocharDecode(char * input,uint16_t * output);

/*************************************************************************
Function: threecharEncode()
Purpose:  Encodes 18-bit data into 3 Characters.  Separates into lower, middle,
          and higher 6-bit numbers, adds 0x30 to convert to ASCII, and returns the chars.
Input:    Data having a maximum length of 18 bits, location of the chars to output
**************************************************************************/
void threecharEncode(uint16_t input,char * output);

/*************************************************************************
Function: threecharDecode()
Purpose:  Decodes 18-bit data (3-Characters) into decimal equivalent.  Subtracts
          0x30 from each and then merges to form decimal output.
Input:    3-Character Encoding of Data
Returns:  16-bit decimal representation of data, location of the decimals to output
**************************************************************************/
void threecharDecode(char * input,uint16_t * output);

/*************************************************************************
Function: fourcharEncode()
Purpose:  Encodes 24-bit data into 4 Characters.  Separates into lower, midlow,
          midhigh, and higher 6-bit numbers, adds 0x30 to convert to ASCII, and
          returns the chars.
Input:    Data having a maximum length of 24 bits, location of the chars to output   
**************************************************************************/
void fourcharEncode(uint32_t input,char * output);

/*************************************************************************
Function: fourcharDecode()
Purpose:  Decodes 24-bit data (4-Characters) into decimal equivalent.  Subtracts
          0x30 from each and then merges to form decimal output.
Input:    4-Character Encoding of Data, location of the decimals to output
**************************************************************************/
void fourcharDecode(char * input,uint32_t * output);

/*************************************************************************
Function: checkSum()
Purpose:  Computes the LIDAR checksum
Input:    Input string, length
Output:	  Checksum value
**************************************************************************/
uint8_t checkSum(char * input,int length);

/*************************************************************************
Function: commandNumber()
Purpose:  Computes the LIDAR Command Number to ensure data is received correctly
Input:    Input command string
Output:	  Command Number
**************************************************************************/
uint8_t commandNumber(char * input);

/*************************************************************************
Function: computeTime()
Purpose:  Computes the LIDAR Time from 4 Byte Value
Input:    Input string
Output:	  Time value
**************************************************************************/
int computeTime(char * input);

/*************************************************************************
Function: readData()
Purpose:  Computes the LIDAR sensor data
Input:    Encoding, StartStep, EndStep, Cluster, ScanInterval, Device to Read From
**************************************************************************/
void readData(uint8_t encoding, uint32_t startstep, uint32_t endstep, uint32_t cluster, uint32_t scaninterval, FILE * filedescriptor);

#endif
/* ****************************************************************************** */
// End of HOKUYO_COMM.H
/* ****************************************************************************** */
