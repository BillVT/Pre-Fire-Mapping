/* ********************************************************************** */
/*                      Pre-Fire Mapping System                           */
/*                        Hokuyo LIDAR Code                               */
/*                            Remote Unit                                 */
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
// This code contains LIDAR configurations and command actions

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

	GDGS-Command: Sensor Latest Data Acquisition Command
		G + (D or S) + (Starting Step) + (End Step) + (Cluster Count) + (String Characters) + LF

		Sensor transmits command echo when status is not '00'.  Otherwise returns data packet with latest
		measurement information.

		GD = 3-Character encoded data (G=47H D=44H)
		GS = 2-Character encoded data (G=47H S=53H)
		Starting Step and End Step = 4-bytes each, between 0 and maximum step (End > Starting)
			Example: From 10 to 750
			10 = 0010 = (0x30,0x30,0x31,0x30)
			750 = 0750 = (0x30,0x37,0x35,0x30)
		Cluster Count = number of adjacent steps that can be merged into single data point
			Example: Cluster Count = 3, if 3 adjacents steps are 3059, 3055, and 3062
			received data will be 3055.  2-bytes
		String Character = Command verification for reply (max of 16 letters)

	BM-Command: Sensor Laser Enable Command
		B + M + (String Characters) + LF

		Illuminates (enables) the sensor's laser and enables measurement.

	QT-Command: Sensor Laser Disable COmmand
		Q + T + (String Characters) + LF
		
		Turns off (disables) the sensor's laser and disables measurement

	RS-Command: Sensor Reset to Default Settings Command
		R + S + (String Characters) + LF

		Turns off the sensor's laser, sets motor speed and bit rate back to default and resets the sensor's internal timer

	TM-Command: Sensor Time Adjust Command
		T + M + (Control Byte) + (String Characters) + LF

		Used to match the host's time with the sensor's.  Sensor should be switched to adjust mode before requesting its time
		and switched off after the adjustment.
		
		Control Byte:	0 = Adust Mode ON
				1 = Time Request
				2 = Adjust Mode OFF

	SS-Command: Sensor Communication Bit Rate Command
		S + S + (Bit Rate) + (String Characters) + LF

		Used to change the communication bit rate of the sensor when connected with RS232C.  Does not have any effect when
		connected with USB, but if both connections exist on the sensor then the commands will be effective if the
		communication changes to RS232.

		Bit Rate (6 bytes):	019200 = 19.2 Kbps
					038400 = 38.4 Kbps (Some sensor models are not compatible with this speed)
					057600 = 57.6 Kbps
					115200 = 115.2 Kbps
					250000 = 250.0 Kbps
					500000 = 500.0 Kbps
					750000 = 750.0 Kbps

	CR-Command: Sensor Motor Speed Command
		C + R + (Speed Parameter) + (String Characters) + LF

		Used to change the sensor motor speed.  Whem multiple sensors are used in the same environment, motor speeds can be
		set to diferent values to avoid light interference.  (UTM-30LX is NOT compatible with this command).

		Speed Parameter (2 bytes):	00 = Default Speed
						01 ~ 10 = Changes speed to 10 different levels
						99 = Reset to intial speed		
*/


/* ****************************************************************************** */
/* ******************************   TO DO  ************************************** */
/* ****************************************************************************** */



/* ****************************************************************************** */
/* ****************************** Includes ************************************** */
/* ****************************************************************************** */
#include "prefiremapping.h"
#include "hokuyo.h"

/* ****************************************************************************** */
/* ****************************** Variables ************************************* */
/* ****************************************************************************** */

char mdreturn[19] = "000000000000000000\n";
char bmreturn[6] = "00000\n";
char qtreturn[6] = "00000\n";
char rsreturn[6] = "00000\n";
char tm1return[12] = "00000000000\n";
char tmreturn[6] = "00000\n";
char ssreturn[6] = "00000\n";
char crreturn[8] = "000000\n";


char command[2] = "0\n";
uint8_t commnum = 0;
uint32_t startstep = 0;
char startchar[4] = "000\n";
uint32_t endstep = 0;
char endchar[4] = "000\n";
uint32_t clustercount = 0;
char clusterchar[2] = "0\n";
uint32_t scaninterval = 0;
char scanchar[1] = "\n";
uint32_t numscans = 0;
char numscanchar[2] = "0\n";
int status = 0;
char statuschar[2] = "0\n";
int sum = 0;
char sumchar[1] = "\n";
size_t temp;
int controlcode = 0;
char controlchar[1] = "\n";
uint8_t checksum = 0;
char speedchar[2] = "0\n";
int speed = 0;
char tempchar[1] = "\n";
char statchar1;
char statchar2;

/* ****************************************************************************** */
/* ****************************** Functions ************************************* */
/* ****************************************************************************** */

/********************************************//**
 *  Function: lidar_open()
 *  Purpose:  Opens LIDAR Communication and flushes buffer
 *  Input:    Device name to open
 *  Returns:  File Descriptor (FD) if successful, -1 if not
 ***********************************************/
/*! \brief Move a chess piece
 * Precondition: it's the owner's turn and the move is valid.
 * Postcondition: the piece will be moved.
 * \param name Device name to open
 * \return File Descriptor (FD) if successful, -1 if not */
FILE * lidar_open(char * name){
	if(DEBUGGING_MODE == 1 || LIDAR_OPEN == 1){
		printf("***In Debugging Mode - Not Opening LIDAR***\n");
		return NULL;
	}
	else{
		FILE * stream = fopen(name, "rw+");
		usleep(OPEN_WAIT);
		int flushed = fflush(stream);
		if(stream != NULL && flushed == 0){
			LIDAR_OPEN = 1;
			if(VERBOSE_MODE == 1)
				printf("Opened Laser Connection\n");
			return stream;
		}
		problem = -1;		
		return stream;
	}
}

/*************************************************************************
Function: lidar_close()
Purpose:  Closes LIDAR Communication
Input:    Device name to close 
Returns:  0 if successful, <0 if not
**************************************************************************/
int lidar_close(FILE * filedescriptor){
	if(DEBUGGING_MODE == 1){
		printf("***In Debugging Mode - Not Closing LIDAR***\n");
		return 0;
	}
	else{
		if(LIDAR_OPEN == 1){
			LIDAR_OPEN = 0;
			if(VERBOSE_MODE == 1)
				printf("Closed Laser Connection\n");
			return fclose(filedescriptor);
		}
		else{
			problem = -2;		
			return -1;
		}
	}
}

/*************************************************************************
Function: lidar_flush()
Purpose:  Flushes LIDAR Communication
Input:    Device name to close 
**************************************************************************/
void lidar_flush(FILE * filedescriptor){
	int flushed = 0;
	if(DEBUGGING_MODE == 1){
		printf("***In Debugging Mode - Not Flushing LIDAR***\n");
	}
	else{
		if(LIDAR_OPEN == 1){
			flushed = fflush(filedescriptor);
			if(VERBOSE_MODE == 1){
				printf("Flushing LIDAR Connection, flushed %d\n",flushed);
			}
			
		}
	}
}

/*************************************************************************
Function: lidar_sendMD()
Purpose:  Sends a MD (3-character) acquisition command to the sensor. 
Input:    Device name to send to, Number of scans
**************************************************************************/
void lidar_sendMD(FILE * filedescriptor, int scannum){
	int commandlength = 0;	
	char acq[16] = "000000000000000\n";
	int printlength = 0;
	commandlength = sprintf(acq, "MD%04d%04d%02d%01d%02d\n",START_STEP, END_STEP, CLUSTER_COUNT, SCAN_INTERVAL, scannum);
	if(DEBUGGING_MODE == 1)
		printf("%s",acq);
	else{
		printlength = fprintf(filedescriptor, "%s", acq);
		if(VERBOSE_MODE == 1)
				printf("LIDAR MD Command Sent\n");
		if(commandlength != printlength){
			problem = 1;
		}
	}	
}

/*************************************************************************
Function: lidar_sendMS()
Purpose:  Sends a MS (2-Character) acquisition command to the sensor.
Input:    Device name to send to, Number of scans
**************************************************************************/
void lidar_sendMS(FILE * filedescriptor, int scannum){
	int commandlength = 0;	
	char acq[16] = "000000000000000\n";
	int printlength = 0;
	commandlength = sprintf(acq, "MS%04d%04d%02d%01d%02d\n",START_STEP, END_STEP, CLUSTER_COUNT, SCAN_INTERVAL, scannum);
	if(DEBUGGING_MODE == 1)
		printf("%s",acq);
	else{
		printlength = fprintf(filedescriptor, "%s", acq);
		if(VERBOSE_MODE == 1)
				printf("LIDAR MS Command Sent\n");
		if(commandlength != printlength){
			problem = 2;
		}
	}
}

/*************************************************************************
Function: lidar_contiuousScanMD()
Purpose:  Sends a MD (3-character) acquisition command to the sensor to continuously
          scan (doesn't stop until QT-Command or RS-Command is received). 
Input:    Device name to send to
**************************************************************************/
void lidar_contiuousScanMD(FILE * filedescriptor){
	int commandlength = 0;	
	char acq[16] = "000000000000000\n";
	int printlength = 0;
	commandlength = sprintf(acq, "MD%04d%04d%02d%01d%02d\n",START_STEP, END_STEP, CLUSTER_COUNT, SCAN_INTERVAL, 0);
	if(DEBUGGING_MODE == 1)
		printf("%s",acq);
	else{
		printlength = fprintf(filedescriptor, "%s", acq);
		if(VERBOSE_MODE == 1)
				printf("LIDAR Continous MD Command Sent\n");
		if(commandlength != printlength){
			problem = 3;
		}
	}
}

/*************************************************************************
Function: lidar_contiuousScanMS()
Purpose:  Sends a MS (2-character) acquisition command to the sensor to continuously
          scan (doesn't stop until QT-Command or RS-Command is received). 
Input:    Device name to send to
**************************************************************************/
void lidar_contiuousScanMS(FILE * filedescriptor){
	int commandlength = 0;	
	char acq[16] = "000000000000000\n";
	int printlength = 0;
	commandlength = sprintf(acq, "MS%04d%04d%02d%01d%02d\n",START_STEP, END_STEP, CLUSTER_COUNT, SCAN_INTERVAL, 0);
	if(DEBUGGING_MODE == 1)
		printf("%s",acq);
	else{
		printlength = fprintf(filedescriptor, "%s", acq);
		if(VERBOSE_MODE == 1)
				printf("LIDAR Continous MS Command Sent\n");
		if(commandlength != printlength){
			problem = 4;
		}
	}
}

/*************************************************************************
Function: lidar_sendGD()
Purpose:  Sends a GD (3-character) acquisition command to the sensor to obtain
          the last sensor data. 
Input:    Device name to send to
**************************************************************************/
void lidar_sendGD(FILE * filedescriptor){
	int commandlength = 0;	
	char acq[13] = "000000000000\n";
	int printlength = 0;
	commandlength = sprintf(acq, "GD%04d%04d%02d\n",START_STEP, END_STEP, CLUSTER_COUNT);
	if(DEBUGGING_MODE == 1)
		printf("%s",acq);
	else{
		printlength = fprintf(filedescriptor, "%s", acq);
		if(VERBOSE_MODE == 1)
				printf("LIDAR GD Command Sent\n");
		if(commandlength != printlength){
			problem = 5;
		}
	}
}

/*************************************************************************
Function: lidar_sendGS()
Purpose:  Sends a GS (2-character) acquisition command to the sensor to obtain
          the last sensor data. 
Input:    Device name to send to
**************************************************************************/
void lidar_sendGS(FILE * filedescriptor){
	int commandlength = 0;	
	char acq[13] = "000000000000\n";
	int printlength = 0;
	commandlength = sprintf(acq, "GS%04d%04d%02d\n",START_STEP, END_STEP, CLUSTER_COUNT);
	if(DEBUGGING_MODE == 1)
		printf("%s",acq);
	else{
		printlength = fprintf(filedescriptor, "%s", acq);
		if(VERBOSE_MODE == 1)
				printf("LIDAR GS Command Sent\n");
		if(commandlength != printlength){
			problem = 6;
		}
	}
}

/*************************************************************************
Function: lidar_laserON()
Purpose:  Sends a BM command to activate the sensor's laser
Input:    Device name to send to
**************************************************************************/
void lidar_laserON(FILE * filedescriptor){
	int commandlength = 0;	
	char com[3] = "00\n";
	int printlength = 0;
	commandlength = sprintf(com, "BM\n");
	if(DEBUGGING_MODE == 1)
		printf("%s",com);
	else{
		printlength = fprintf(filedescriptor, "%s", com);
		if(commandlength != printlength){
			problem = 7;
		}
		if(VERBOSE_MODE == 1)
			printf("LASER ON\n");
	}
}

/*************************************************************************
Function: lidar_laserOFF()
Purpose:  Sends a BM command to deactivate the sensor's laser
Input:    Device name to send to
**************************************************************************/
void lidar_laserOFF(FILE * filedescriptor){
	int commandlength = 3;	
	char com[3] = "00\n";
	int printlength = 0;
	commandlength = sprintf(com, "QT\n");
	if(DEBUGGING_MODE == 1)
		printf("%s",com);
	else{
		printlength = fprintf(filedescriptor, "%s", com);
		if(commandlength != printlength){
			problem = 8;
		}
		if(VERBOSE_MODE == 1)
			printf("LASER OFF\n");
	}
}

/*************************************************************************
Function: lidar_RESET()
Purpose:  Sends a RS command to reset all lidar settings to default values
Input:    Device name to send to
**************************************************************************/
void lidar_RESET(FILE * filedescriptor){
	int commandlength = 0;	
	char com[3] = "00\n";
	int printlength = 0;
	commandlength = sprintf(com, "RS\n");
	if(DEBUGGING_MODE == 1)
		printf("%s",com);
	else{
		printlength = fprintf(filedescriptor, "%s", com);
		if(VERBOSE_MODE == 1)
				printf("LIDAR Reset to Default Command Sent\n");
		if(commandlength != printlength){
			problem = 9;
		}
	}
}

/*************************************************************************
Function: lidar_adjustON()
Purpose:  Sends a TM command to enable the lidar's time adjustment mode
Input:    Device name to send to
**************************************************************************/
void lidar_adjustON(FILE * filedescriptor){
	int commandlength = 0;	
	char com[4] = "000\n";
	int printlength = 0;
	commandlength = sprintf(com, "TM0\n");
	if(DEBUGGING_MODE == 1)
		printf("%s",com);
	else{
		printlength = fprintf(filedescriptor, "%s", com);
		if(VERBOSE_MODE == 1)
				printf("LIDAR Adjust Mode ON Command Sent\n");
		if(commandlength != printlength){
			problem = 10;
		}
	}
}

/*************************************************************************
Function: lidar_adjustTIME()
Purpose:  Sends a TM command to obtain the lidar's time in adjustment mode
Input:    Device name to send to
**************************************************************************/
void lidar_adjustTIME(FILE * filedescriptor){
	int commandlength = 0;	
	char com[4] = "000\n";
	int printlength = 0;
	commandlength = sprintf(com, "TM1\n");
	if(DEBUGGING_MODE == 1)
		printf("%s",com);
	else{
		printlength = fprintf(filedescriptor, "%s", com);
		if(VERBOSE_MODE == 1)
				printf("LIDAR Adjust TIME Command Sent\n");
		if(commandlength != printlength){
			problem = 11;
		}
	}
}

/*************************************************************************
Function: lidar_adjustOFF()
Purpose:  Sends a TM command to disable the lidar's time adjustment mode
Input:    Device name to send to
**************************************************************************/
void lidar_adjustOFF(FILE * filedescriptor){
	int commandlength = 0;	
	char com[4] = "000\n";
	int printlength = 0;
	commandlength = sprintf(com, "TM2\n");
	if(DEBUGGING_MODE == 1)
		printf("%s",com);
	else{
		printlength = fprintf(filedescriptor, "%s", com);
		if(VERBOSE_MODE == 1)
				printf("LIDAR Adjust Mode OFF Command Sent\n");
		if(commandlength != printlength){
			problem = 12;
		}
	}
}

/*************************************************************************
Function: lidar_bitRate()
Purpose:  Sends a SS command to set the sensor's RS232 communication Bit Rate
Input:    Device name to send to, Bit Rate (see info above for values)
**************************************************************************/
void lidar_bitRate(FILE * filedescriptor, int speed){
	int commandlength = 0;	
	char com[9] = "00000000\n";
	int printlength = 0;
	if(speed != 19200 && speed != 38400 && speed != 57600 && speed != 115200 && speed != 250000 && speed != 50000 && speed != 750000){
		problem = 11;
		return;
	}
	commandlength = sprintf(com, "SS%06d\n",speed);
	if(DEBUGGING_MODE == 1)
		printf("%s",com);
	else{
		printlength = fprintf(filedescriptor, "%s", com);
		if(VERBOSE_MODE == 1)
				printf("LIDAR Bit Rate Command Sent\n");
		if(commandlength != printlength){
			problem = 13;
		}
	}
}

/*************************************************************************
Function: lidar_motorSpeed()
Purpose:  Sends a CR command to set the sensor's motor speed
Input:    Device name to send to, motor speed (0 = default, 1-10, 99 = Reset to initial speed)
**************************************************************************/
void lidar_motorSpeed(FILE * filedescriptor, int speed){
	int commandlength = 0;	
	char com[5] = "0000\n";
	int printlength = 0;
	if(((speed < 0) || (speed > 10)) && (speed != 99))
		speed = 0;
	commandlength = sprintf(com, "CR%02d\n",speed);
	if(DEBUGGING_MODE == 1)
		printf("%s",com);
	else{
		printlength = fprintf(filedescriptor, "%s", com);
		if(VERBOSE_MODE == 1)
				printf("LIDAR Motor Speed Command Sent\n");
		if(commandlength != printlength){
			problem = 14;
		}
	}
}

/*************************************************************************
Function: lidar_sensitivity()
Purpose:  Sends a HS command to set the sensor's sensitivity
Input:    Device name to send to, sensitivity (0 = Normal, 1 = High Sensitivity)
**************************************************************************/
void lidar_sensitivity(FILE * filedescriptor, int sensitivity){
	int commandlength = 0;	
	char com[4] = "000\n";
	int printlength = 0;
	if((sensitivity != 0) && (sensitivity != 1))
		return;
	commandlength = sprintf(com, "HS%01d\n",sensitivity);
	if(DEBUGGING_MODE == 1)
		printf("%s",com);
	else{
		printlength = fprintf(filedescriptor, "%s", com);
		if(VERBOSE_MODE == 1)
				printf("LIDAR Sensitivity Command Sent\n");
		if(commandlength != printlength){
			problem = 15;
		}
	}
}

/*************************************************************************
Function: lidar_malfunctionSim()
Purpose:  Sends a DB command to simulate a sensor malfunction
Input:    Device name to send to, malfunction to simulate
**************************************************************************/
void lidar_malfunctionSim(FILE * filedescriptor, int malfunction){
	int commandlength = 0;	
	char com[5] = "0000\n";
	int printlength = 0;
	if((malfunction < 1) && (malfunction > 5) && (malfunction != 10))
		return;
	commandlength = sprintf(com, "DB%02d\n",malfunction);
	if(DEBUGGING_MODE == 1)
		printf("%s",com);
	else{
		printlength = fprintf(filedescriptor, "%s", com);
		if(VERBOSE_MODE == 1)
				printf("LIDAR Malfunction Simulation Command Sent\n");
		if(commandlength != printlength){
			problem = 16;
		}
	}
}

/*************************************************************************
Function: lidar_version()
Purpose:  Sends a VV command to receive the Sensor Version information
Input:    Device name to send to
**************************************************************************/
void lidar_version(FILE * filedescriptor){
	int commandlength = 0;	
	char com[3] = "00\n";
	int printlength = 0;
	commandlength = sprintf(com, "VV\n");
	if(DEBUGGING_MODE == 1)
		printf("%s",com);
	else{
		printlength = fprintf(filedescriptor, "%s", com);
		if(VERBOSE_MODE == 1)
				printf("LIDAR Version Command Sent\n");
		if(commandlength != printlength){
			problem = 17;
		}
	}
}

/*************************************************************************
Function: lidar_specs()
Purpose:  Sends a PP command to receive the Sensor specifications
Input:    Device name to send to
**************************************************************************/
void lidar_specs(FILE * filedescriptor){
	int commandlength = 0;	
	char com[3] = "00\n";
	int printlength = 0;
	commandlength = sprintf(com, "PP\n");
	if(DEBUGGING_MODE == 1)
		printf("%s",com);
	else{
		printlength = fprintf(filedescriptor, "%s", com);
		if(VERBOSE_MODE == 1)
				printf("LIDAR Specification Command Sent\n");
		if(commandlength != printlength){
			problem = 18;
		}
	}
}

/*************************************************************************
Function: lidar_state()
Purpose:  Sends a II command to receive the Sensor running state
Input:    Device name to send to
**************************************************************************/
void lidar_state(FILE * filedescriptor){
	int commandlength = 0;	
	char com[3] = "00\n";
	int printlength = 0;
	commandlength = sprintf(com, "II\n");
	if(DEBUGGING_MODE == 1)
		printf("%s",com);
	else{
		printlength = fprintf(filedescriptor, "%s", com);
		if(VERBOSE_MODE == 1)
				printf("LIDAR State Command Sent\n");
		if(commandlength != printlength){
			problem = 19;
		}
	}
}

/*************************************************************************
Function: lidar_read()
Purpose:  Reads the laser's response and checks for the appropriate values
Input:    Device name to read from
**************************************************************************/
void lidar_read(FILE * filedescriptor){
	temp = fread(command,1,2,filedescriptor);
	commnum = commandNumber(command);
	switch(commnum){
		case 0:	// MD Command - 3-Character Laser Acquisiton
			temp = fread(mdreturn,1,19,filedescriptor);
			sscanf(mdreturn,"%04d%04d%02d%01d%02d\n%02c%c\n\n",&startstep,&endstep,&clustercount,&scaninterval,&numscans,statuschar,sumchar);
			sum = (uint8_t)sumchar[0];
			checksum = checkSum(statuschar,2);
			if(checksum != sum){
				if(VERBOSE_MODE == 1)
					printf("Checksums Do Not Match!\n");
				problem = 20;
			}
			if(status == 0){
				if(VERBOSE_MODE == 1)
					printf("Need to read data\n");
				readData(3,startstep,endstep,clustercount,scaninterval,filedescriptor);
			}
			else if(status == 1){
				if(VERBOSE_MODE == 1)
					printf("Starting Step has Non-Numeric Value\n");
				problem = 20;
			}
			else if(status == 2){
				if(VERBOSE_MODE == 1)
					printf("End Step has Non-Numeric Value\n");
				problem = 20;
			}
			else if(status == 3){
				if(VERBOSE_MODE == 1)
					printf("Cluster Count has Non-Numeric Value\n");
				problem = 20;
			}
			else if(status == 4){
				if(VERBOSE_MODE == 1)
					printf("End Step is Out of Range\n");
				problem = 20;
			}
			else if(status == 5){
				if(VERBOSE_MODE == 1)
					printf("End Step is Smaller Than Starting Step\n");
				problem = 20;
			}
			else if(status == 6){
				if(VERBOSE_MODE == 1)
					printf("Scan Interval has Non-Numeric Value\n");
				problem = 20;
			}
			else if(status == 7){
				if(VERBOSE_MODE == 1)
					printf("Cluster Count has Non-Numeric Value\n");
				problem = 20;
			}
			else if((status > 20) && (status < 50)){
				if(VERBOSE_MODE == 1)
					printf("Processing Stopped to Verify Error\n");
				problem = 20;
			}
			else if((status > 49) && (status < 98)){
				if(VERBOSE_MODE == 1)
					printf("Hardware Trouble\n");
				problem = 20;
			}
			else if(status == 98){
				if(VERBOSE_MODE == 1)
					printf("Resumption of Process After Normal Operation\n");
				problem = 20;
			}
			break;
		case 1: // MS Command - 2-Character Laser Acquisiton
			break;
		case 2:	// GD Command - Single 3-Character Laser Acquisiton
			break;
		case 3: // GS Command - Single 2-Character Laser Acquisiton
			break;
		case 4:	// BM Command - Laser ON
			temp = fread(bmreturn,1,6,filedescriptor);
			sscanf(bmreturn,"\n%02c%01c\n\n",statuschar,sumchar);
			checksum = checkSum(statuschar,2);
			sum = (uint8_t)sumchar[0];
			status = atoi(statuschar);
			if(checksum != sum){
				if(VERBOSE_MODE == 1)
					printf("Checksums Do Not Match!\n");
				problem = 24;
			}
			if(status == 1){
				if(VERBOSE_MODE == 1)
					printf("Unable to Control Due to Laser Malfunction\n");
				problem = 24;
			}
			if(status == 2){
				if(VERBOSE_MODE == 1)
					printf("Laser is already ON\n");
				problem = 24;
			}
			break;
		case 5: // QT Command - Laser OFF
			temp = fread(qtreturn,1,6,filedescriptor);
			sscanf(qtreturn,"\n%02c%01c\n\n",statuschar,sumchar);
			checksum = checkSum(statuschar,2);
			sum = (uint8_t)sumchar[0];
			if(checksum != sum){
				if(VERBOSE_MODE == 1)
					printf("Checksums Do Not Match!\n");
				problem = 25;
			}
			break;
		case 6:	// RS Command - LIDAR Reset to Default
			temp = fread(rsreturn,1,6,filedescriptor);
			sscanf(rsreturn,"\n%02c%01c\n\n",statuschar,sumchar);
			checksum = checkSum(statuschar,2);
			sum = (uint8_t)sumchar[0];
			if(checksum != sum){
				if(VERBOSE_MODE == 1)
					printf("Checksums Do Not Match!\n");
				problem = 26;
			}
			break;
		case 7: // TM Command - Laser Time Adjust Command
			temp = fread(controlchar,1,1,filedescriptor);	// Control Code
			controlcode = atoi(controlchar);
			if(controlcode == 1){
				char time[4] = "000\n";
				// Time value returned;
				temp = fread(tm1return,1,12,filedescriptor);
				sscanf(tm1return,"\n00P\n%04c%01c\n\n",time,sumchar);
				// NEED TO FIX THIS
				lidar_time = (uint32_t)time;
				sum = (uint8_t)sumchar[0];
				checksum = checkSum(time,4);
				if(checksum != sum){
					if(VERBOSE_MODE == 1)
						printf("Checksums Do Not Match!\n");
					problem = 27;
				}
			}
			else{
				temp = fread(tmreturn,1,6,filedescriptor);
				sscanf(tmreturn,"\n%02c%01c\n\n",statuschar,sumchar);
				checksum = checkSum(statuschar,2);
				sum = (uint8_t)sumchar[0];
				status = atoi(statuschar);
				if(checksum != sum){
					if(VERBOSE_MODE == 1)
						printf("Checksums Do Not Match!\n");
					problem = 27;
				}
				if(status == 1){
					if(VERBOSE_MODE == 1)
						printf("Invalid Control Code\n");
					problem = 27;
				}
				else if(status == 2){
					if(VERBOSE_MODE == 1)
						printf("Adjust Mode ON Command is received when Sensor mode is already ON\n");
					problem = 27;
				}
				else if(status == 3){
					if(VERBOSE_MODE == 1)
						printf("Adjust Mode OFF Command is received when Sensor mode is already OFF\n");
					problem = 27;
				}
				else if(status == 4){
					if(VERBOSE_MODE == 1)
						printf("Adjust Mode is OFF when Time was requested\n");
					problem = 27;
				}
			}
			break;
		case 8:	// SS Command - Bit Rate Command
			temp = fread(ssreturn,1,6,filedescriptor);
			sscanf(ssreturn,"\n%02c%01c\n\n",statuschar,sumchar);
			checksum = checkSum(statuschar,2);
			sum = (uint8_t)sumchar[0];
			status = atoi(statuschar);
			if(checksum != sum){
				if(VERBOSE_MODE == 1)
					printf("Checksums Do Not Match!\n");
				problem = 28;
			}
			if(status == 1){
					if(VERBOSE_MODE == 1)
						printf("Bit Rate has Non-Numeric Value\n");
					problem = 28;
			}
			else if(status == 2){
					if(VERBOSE_MODE == 1)
						printf("Invalid Bit Rate\n");
					problem = 28;
			}
			else if(status == 3){
					if(VERBOSE_MODE == 1)
						printf("Sensor is Already Running at the Defined Rate\n");
					problem = 28;
			}
			else if(status == 4){
					if(VERBOSE_MODE == 1)
						printf("Not Compatible with the Sensor Model\n");
					problem = 28;
			}			
			break;
		case 9: // CR Command - Motor Speed Command
			temp = fread(crreturn,1,8,filedescriptor);
			sscanf(crreturn,"%02c\n%02c%01c\n\n",speedchar,statuschar,sumchar);
			checksum = checkSum(statuschar,2);
			sum = (uint8_t)sumchar[0];
			status = atoi(statuschar);
			speed = atoi(speedchar);
			if(checksum != sum){
				if(VERBOSE_MODE == 1)
					printf("Checksums Do Not Match!\n");
				problem = 29;
			}
			if(status == 1){
					if(VERBOSE_MODE == 1)
						printf("Invalid Speed Ratio\n");
					problem = 29;
			}
			else if(status == 2){
					if(VERBOSE_MODE == 1)
						printf("Speed Out of Range\n");
					problem = 29;
			}
			else if(status == 3){
					if(VERBOSE_MODE == 1)
						printf("Motor is Already Running at the Defined Speed\n");
					problem = 29;
			}
			else if(status == 4){
					if(VERBOSE_MODE == 1)
						printf("Not Compatible with the Sensor Model\n");
					problem = 29;
			}			
			break;
		case 10: // HS Command - Sensitivity Command
			break;
		case 11: // DB Command
			break;
		case 12: // VV Command
			break;
		case 13: // PP Command
			break;
		case 14: // II Command
		default:
			break;
	}		
}

/* ****************************************************************************** */
// End of HOKUYO.C
/* ****************************************************************************** */
