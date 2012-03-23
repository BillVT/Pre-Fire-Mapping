/* ********************************************************************** */
/*                      Pre-Fire Mapping System                           */
/*                      Hokuyo LIDAR Code Header                          */
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


/* ****************************************************************************** */
/* ****************************** Includes ************************************** */
/* ****************************************************************************** */
#ifndef GUARD
	#define GUARD
	#include "prefiremapping.h"
#endif

#include "hokuyo_comm.h"

/* ****************************************************************************** */
/* ************************ Function Declarations ******************************* */
/* ****************************************************************************** */

/*************************************************************************
Function: lidar_open()
Purpose:  Opens LIDAR Communication
Input:    Device name to open 
Returns:  File Descriptor (FD) if successful, -1 if not
**************************************************************************/
FILE * lidar_open(char * name);

/*************************************************************************
Function: lidar_close()
Purpose:  Closes LIDAR Communication
Input:    Device name to close 
Returns:  0 if successful, <0 if not
**************************************************************************/
int lidar_close(FILE * filedescriptor);

/*************************************************************************
Function: lidar_flush()
Purpose:  Flushes LIDAR Communication
Input:    Device name to close 
**************************************************************************/
void lidar_flush(FILE * filedescriptor);

/*************************************************************************
Function: lidar_sendMD()
Purpose:  Sends a MD (3-character) acquisition command to the sensor. 
Input:    Device name to send to, Number of scans
**************************************************************************/
void lidar_sendMD(FILE * filedescriptor, int scannum);

/*************************************************************************
Function: lidar_sendMS()
Purpose:  Sends a MS (2-Character) acquisition command to the sensor.
Input:    Device name to send to, Number of scans
**************************************************************************/
void lidar_sendMS(FILE * filedescriptor, int scannum);

/*************************************************************************
Function: lidar_contiuousScanMD()
Purpose:  Sends a MD (3-character) acquisition command to the sensor to continuously
          scan (doesn't stop until QT-Command or RS-Command is received). 
Input:    Device name to send to
**************************************************************************/
void lidar_contiuousScanMD(FILE * filedescriptor);

/*************************************************************************
Function: lidar_contiuousScanMS()
Purpose:  Sends a MD (2-character) acquisition command to the sensor to continuously
          scan (doesn't stop until QT-Command or RS-Command is received). 
Input:    Device name to send to
**************************************************************************/
void lidar_contiuousScanMS(FILE * filedescriptor);

/*************************************************************************
Function: lidar_sendGD()
Purpose:  Sends a GD (3-character) acquisition command to the sensor to obtain
          the last sensor data. 
Input:    Device name to send to
**************************************************************************/
void lidar_sendGD(FILE * filedescriptor);

/*************************************************************************
Function: lidar_sendGS()
Purpose:  Sends a GS (2-character) acquisition command to the sensor to obtain
          the last sensor data. 
Input:    Device name to send to
**************************************************************************/
void lidar_sendGS(FILE * filedescriptor);

/*************************************************************************
Function: lidar_laserON()
Purpose:  Sends a BM command to activate the sensor's laser
Input:    Device name to send to
**************************************************************************/
void lidar_laserON(FILE * filedescriptor);

/*************************************************************************
Function: lidar_laserOFF()
Purpose:  Sends a BM command to deactivate the sensor's laser
Input:    Device name to send to
**************************************************************************/
void lidar_laserOFF(FILE * filedescriptor);

/*************************************************************************
Function: lidar_RESET()
Purpose:  Sends a RS command to reset all lidar settings to default values
Input:    Device name to send to
**************************************************************************/
void lidar_RESET(FILE * filedescriptor);

/*************************************************************************
Function: lidar_adjustON()
Purpose:  Sends a TM command to enable the lidar's time adjustment mode
Input:    Device name to send to
**************************************************************************/
void lidar_adjustON(FILE * filedescriptor);

/*************************************************************************
Function: lidar_adjustTIME()
Purpose:  Sends a TM command to obtain the lidar's time in adjustment mode
Input:    Device name to send to
**************************************************************************/
void lidar_adjustTIME(FILE * filedescriptor);

/*************************************************************************
Function: lidar_adjustOFF()
Purpose:  Sends a TM command to disable the lidar's time adjustment mode
Input:    Device name to send to
**************************************************************************/
void lidar_adjustOFF(FILE * filedescriptor);

/*************************************************************************
Function: lidar_bitRate()
Purpose:  Sends a SS command to set the sensor's RS232 communication Bit Rate
Input:    Device name to send to, Bit Rate (see info above for values)
**************************************************************************/
void lidar_bitRate(FILE * filedescriptor, int speed);

/*************************************************************************
Function: lidar_motorSpeed()
Purpose:  Sends a CR command to set the sensor's motor speed
Input:    Device name to send to, motor speed (0 = default, 1-10, 99 = Reset to initial speed
**************************************************************************/
void lidar_motorSpeed(FILE * filedescriptor, int speed);

/*************************************************************************
Function: lidar_sensitivity()
Purpose:  Sends a HS command to set the sensor's sensitivity
Input:    Device name to send to, sensitivity (0 = Normal, 1 = High Sensitivity)
**************************************************************************/
void lidar_sensitivity(FILE * filedescriptor, int sensitivity);

/*************************************************************************
Function: lidar_malfunctionSim()
Purpose:  Sends a DB command to simulate a sensor malfunction
Input:    Device name to send to, malfunction to simulate
**************************************************************************/
void lidar_malfunctionSim(FILE * filedescriptor, int malfunction);

/*************************************************************************
Function: lidar_version()
Purpose:  Sends a VV command to receive the Sensor Version information
Input:    Device name to send to
**************************************************************************/
void lidar_version(FILE * filedescriptor);

/*************************************************************************
Function: lidar_specs()
Purpose:  Sends a PP command to receive the Sensor specifications
Input:    Device name to send to
**************************************************************************/
void lidar_specs(FILE * filedescriptor);

/*************************************************************************
Function: lidar_state()
Purpose:  Sends a II command to receive the Sensor running state
Input:    Device name to send to
**************************************************************************/
void lidar_state(FILE * filedescriptor);

/*************************************************************************
Function: lidar_read()
Purpose:  Reads the laser's response and checks for the appropriate values
Input:    Device name to read from
**************************************************************************/
void lidar_read(FILE * filedescriptor);

/* ****************************************************************************** */
// End of HOKUYO.H
/* ****************************************************************************** */
