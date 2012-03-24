/* ********************************************************************** */
/*                      Pre-Fire Mapping System                           */
/*                      Main Program Code Header                          */
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
#ifndef _PREFIREMAPPING_H_
#define _PREFIREMAPPING_H_

/* ****************************************************************************** */
/* ****************************** Includes ************************************** */
/* ****************************************************************************** */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <string.h>
#include "hokuyo_comm.h"
#include "hokuyo.h"

/* ****************************************************************************** */
/* *****************************   Definitions  ********************************* */
/* ****************************************************************************** */
// PROGRAM
#define DEBUGGING_MODE 0				// Debugging Mode: Outputs commands to terminal if 1
#define VERBOSE_MODE 1					// Verbose Mode: Outputs Errors to terminal if 1

int problem;						// Problem is occurring.  Value indicates problem
		// 0 = No Problem
		// -2 = Problem Closing LIDAR Communication (Already closed)
		// -1 = Problem with Opening LIDAR Communication		
		// 01 = Problem with LIDAR MD command
		// 02 = Problem with LIDAR MS command
		// 03 = Problem with LIDAR Continous Scan MD command
		// 04 = Problem with LIDAR Continous Scan MS command
		// 05 = Problem with LIDAR GD command
		// 06 = Problem with LIDAR GS command
		// 07 = Problem with LIDAR Laser ON Command (BM command)
		// 08 = Problem with LIDAR Laser OFF Command (QT command)
		// 09 = Problem with LIDAR Reset Command (RS command)
		// 10 = Problem with LIDAR Adjust ON Command (TM Command)
		// 11 = Problem with LIDAR Adjust TIME Command (TM Command)
		// 12 = Problem with LIDAR Adjust OFF Command (TM Command)
		// 13 = Problem with LIDAR Bit Rate Command (SS Command)
		// 14 = Problem with LIDAR Motor Speed Command
		// 15 = Problem with LIDAR Sensitivy Command (HS Command)
		// 16 = Problem with LIDAR Malfunction Simulator Command (DB Command)
		// 17 = Problem with LIDAR Version Details Command (VV Command)
		// 18 = Problem with LIDAR Specifications Command (PP Command)
		// 19 = Problem with LIDAR Running State Command (II Command)
		// 20 = Problem with received MD Command
		// 21 = Problem with received MS Command
		// 22 = Problem with received GD Command
		// 23 = Problem with received GS Command
		// 24 = Problem with received BM Command
		// 25 = Problem with received QT Command
		// 26 = Problem with received RS Command
		// 27 = Problem with received TM Command
		// 28 = Problem with received SS Command
		// 29 = Problem with received CR Command
		// 30 = Problem with received HS Command
		// 31 = Problem with received DB Command
		// 32 = Problem with received VV Command
		// 33 = Problem with received PP Command
		// 34 = Problem with received II Command

int status;						// LIDAR Status
		// GD/GS STATUS
		// 00 = Normal Operation
		// 01 = Starting Step has Non-Numeric Value
		// 02 = End Step has Non-Numeric Value
		// 03 = Cluster Count has Non-Numeric Value
		// 04 = End Step is Out of Range
		// 05 = End Step is Smaller than Starting Step
		// 10 = Laser is off.
		// 50-98 = Hardware Trouble (such as Laser, Motor Malfunction, etc.)

		// BM STATUS
		// 00 = Command Received without any Error (Normal Operation)
		// 01 = Unable to Control Laser due to Malfunction
		// 02 = Laser is Already Turned On

		// TM STATUS
		// 00 = Command Received without any Error (Normal Operation)
		// 01 = Invalid Control Code
		// 02 = Adjust Mode ON Command is Received when Sensor's Adjust Mode is Already ON
		// 03 = Adjust Mode OFF Command is Received when Sensor's Adjust Mode is Already OFF
		// 04 = Adjust Mode is OFF when Requested for Time

		// SS STATUS
		// 00 = Command Received without any Error (Normal Operation)
		// 01 = Bit Rate has Non-Numeric Value
		// 02 = Invalid Bit Rate
		// 03 = Sensor is already running at the defined bit rate
		// 04 = Bit Rate is not compatible with this sensor model

		// CR STATUS
		// 00 = Command Received without any Error (Normal Operation)
		// 01 = Invalid Speed Ratio
		// 02 = Speed Ratio is out of range
		// 03 = Motor is already running at the defined speed
		// 04 = Motor Speed is not compatible with this sensor model


// LIDAR
#define OPEN_WAIT 1000000
int START_STEP;
int END_STEP;
int CLUSTER_COUNT;
int SCAN_INTERVAL;
int LIDAR_OPEN;
uint32_t lidar_time;
uint16_t data[740];
#define REMBLOCK 36


#endif
/* ****************************************************************************** */
// End of PREFIREMAPPING.H
/* ****************************************************************************** */
