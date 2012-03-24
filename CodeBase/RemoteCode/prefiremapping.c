/* ********************************************************************** */
/*                      Pre-Fire Mapping System                           */
/*                        Main Program Code                               */
/*                            Remote Unit                                 */
/*                                                                        */
/* Authors : William Etter (MSE '11)                                      */
/*                                                                        */
/*                      University of Pennsylvania                        */
/* mLab - Real-Time Embedded Systems Laboratory                           */
/* Date : October  08, 2011                                               */
/* Version : 1.0                                                          */
/* Hardware : Hoyuko Laser RangeFinder, Pandaboard                        */
/* Copyright William Etter 2011 (Etterw@seas.upenn.edu)                   */
/* ********************************************************************** */

/* ****************************************************************************** */
/* *******************************   About  ************************************* */
/* ****************************************************************************** */
// This is the main program code for the Pre-Fire Mapping System.


/* ****************************************************************************** */
/* ******************************   TO DO  ************************************** */
/* ****************************************************************************** */
// Connect to LIDAR
// Configure LIDAR settings
// Obtain data from LIDAR
// Process Data
// Apply SLAM algorithm
// Close Connection with LIDAR
// Run optimization and line detection
// Convert to structure required for 3D graphing program
// Build in 3D


/* ****************************************************************************** */
/* ****************************** Includes ************************************** */
/* ****************************************************************************** */
#include "prefiremapping.h"


/* ****************************************************************************** */
/* ********************   Configuration Definitions  **************************** */
/* ****************************************************************************** */

char * lidarname = "/dev/ttyACM0";		// LIDAR Connection Name
FILE * fd;					// LIDAR File Descriptor (I/O Stream)
int status;					// LIDAR File Descriptor Status

char teststr[8] = "0000000\n";

/* ****************************************************************************** */
/* **************************** Main Program ************************************ */
/* ****************************************************************************** */
int main(int argc,char **argv){
	
	/*** SCAN PROPERTIES ***/
	problem = 0;				// Startup with No Problem
	fd = NULL;
	START_STEP = 10;
	END_STEP = 750;
	CLUSTER_COUNT = 1;
	SCAN_INTERVAL = 1;
	/***********************/
	
	/***  DEBUGGING MODE ***/
	if(DEBUGGING_MODE == 1)
		printf("-------IN DEBUGGING MODE - NOT CONNECTED TO LIDAR-------\n\n\n");
	/***********************/

	/***   VERBOSE MODE  ***/
	if(VERBOSE_MODE == 1)
		printf("-------IN VERBOSE MODE - OUTPUTTING DATA TO TERMINAL-------\n\n\n");
	/***********************/
	
	
	/***    Open LIDAR   ***/
	if(DEBUGGING_MODE == 0)
		fd = lidar_open(lidarname);
	if(fd == NULL && DEBUGGING_MODE == 0){
		// Problem opening LIDAR
		problem = -1;
		if(VERBOSE_MODE == 1)	
			printf("Problem Opening LIDAR\n");
	}

	

	usleep(1000000);
	lidar_sendMD(fd,1);
	lidar_read(fd);

	usleep(2000000);
	//Laser OFF
	//lidar_laserOFF(fd);
	//lidar_read(fd);

	// Close LIDAR
	status = lidar_close(fd);

	return 0;
}
/* ****************************************************************************** */
// End of PREFIREMAPPING.C
/* ****************************************************************************** */
