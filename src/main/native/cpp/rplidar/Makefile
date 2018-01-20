#/*
# * Copyright (C) 2014  RoboPeak
# *
# * This program is free software: you can redistribute it and/or modify
# * it under the terms of the GNU General Public License as published by
# * the Free Software Foundation, either version 3 of the License, or
# * (at your option) any later version.
# *
# * This program is distributed in the hope that it will be useful,
# * but WITHOUT ANY WARRANTY; without even the implied warranty of
# * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# * GNU General Public License for more details.
# *
# * You should have received a copy of the GNU General Public License
# * along with this program.  If not, see <http://www.gnu.org/licenses/>.
# *
# */
#

#Run this to create RPLidar.o, rplidar,and librplidar_sdk.a
# HOME_TREE := $(CURDIR)

# MODULE_NAME := $(notdir $(CURDIR))

# include mak_def.inc

# CXX = arm-frc-linux-gnueabi-g++
# CXXSRC += RPLidar.cpp
# C_INCLUDES += -I/Library/Java/JavaVirtualMachines/jdk1.8.0_141.jdk/Contents/Home/include/ -I/Library/Java/JavaVirtualMachines/jdk1.8.0_141.jdk/Contents/Home/include/darwin

# EXTRA_OBJ := 
# LD_LIBS += -lstdc++ -lpthread -lrplidar_sdk

# FLAGS        = -std=gnu99 -Iinclude
# CFLAGS       = -fPIC -pedantic -Wall -Wextra -march=native -ggdb3
# LDFLAGS      = -lm -pthread -static -L./ -lrplidar_sdk -shared

# TARGET  = librplidarjni.so

# all: build_app

# include mak_common.inc

# clean: clean_app

#Run this to create Rlibrplidarjni.so
CC=arm-frc-linux-gnueabi-g++

ifeq ($(DEBUG),yes)
	CXXFLAGS=-Wall -g
	LDFLAGS=-Wall -g
else
	CXXFLAGS=-Wall
	LDFLAGS=-Wall
endif

INCPATH=include
SRCPATH=
OBJPATH=obj/Linux/Release/rplidar
LIBPATH=output/Linux/Release
BINPATH=bin

INC=$(INCPATH)/rplidar.h
SRC=$(SRCPATH)/RPLidar.cpp
OBJ=$(OBJPATH)/RPLidar.o
OUT=$(LIBPATH)/librplidarjni.so

INCLUDES=-I ./$(INCPATH)

default: $(OUT)

$(OUT): $(OBJ)
	$(CC) $(LDFLAGS) -shared -o $@ $^

$(OBJPATH)/%.o: $(SRCPATH)/%.cpp $(INC)
	$(CC) $(CXXFLAGS) $(INCLUDES) -fpic -c $< -o $@

.PHONY: clean cleanall

clean:
	rm -f $(OBJPATH)/*.o

cleanall: clean
	rm -f $(OUT)