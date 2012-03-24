# ********************************************************************** #
#                      Pre-Fire Mapping System                           #
#                              Makefile                                  #
#                                                                        #
# Authors : William Etter (MSE '11)                                      #
#                                                                        #
#                      University of Pennsylvania                        #
# mLab - Real-Time Embedded Systems Laboratory                           #
# Date : March 23, 2011                                                  #
# Version : 1.0                                                          #
# Hardware : Hoyuko Laser RangeFinder, BeagleBone, CHRobotics IMU        #
# Copyright William Etter 2011 (Etterw@seas.upenn.edu)                   #
# ********************************************************************** #

PROGN := pfm

CC := gcc

CFLAGS := -Wall -g

SOURCES := prefiremapping.c hokuyo.c hokuyo_comm.c

all: prefiremapping


prefiremapping: $(SOURCES)
	$(CC) $(CFLAGS) $(SOURCES) -o $(PROGN)

clean :
	rm -f ./$(PROGN)
