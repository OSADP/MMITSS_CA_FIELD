# common definitions for VMware builds on Savari RSU and OBU devices

# add the following line into individual user's ~/.bashrc
#  export SAVARI_MK_DEFS=/home/MMITSS-CA/build/Savari.mk
# then source ~/.bashrc
# can use 'echo $SAVARI_MK_DEFS' from the command line to check whether SAVARI_MK_DEFS has been set.

BASE_DIR      := /home/MMITSS-CA
RSU_DIR       := $(BASE_DIR)/rsu
OBU_DIR       := $(BASE_DIR)/obu

# Savari uClibc toolchain
RSU_TOOLCHAIN_DIR  := /home/Sobos-SDK-for-i686-Linux/RSE_SDK-3.1.2/toolchain
OBU_TOOLCHAIN_DIR  := /home/Sobos-SDK-for-i686-Linux/ASD_SDK-3.1.2/toolchain

# common path
SOURCE_DIR      := src
HEADER_DIR      := include
OBJ_DIR         := obj
LIB_DIR         := lib

# compiler flags
CC              := gcc
C++             := g++
LOCAL_INCL      := -I$(RSU_TOOLCHAIN_DIR)/usr/$(HEADER_DIR) -I/usr/$(HEADER_DIR) -I$(HEADER_DIR)
CFLAGS          := -Wall -W -Wshadow -Wcast-qual -Wwrite-strings -O3 -fPIC $(LOCAL_INCL)
C++FLAGS        := -std=c++11 -Wconversion -fno-strict-aliasing $(CFLAGS)

# compiler flags to use with Savari toolchain
RSU_CC       := $(RSU_TOOLCHAIN_DIR)/bin/i386-linux-gcc
RSU_C++      := $(RSU_TOOLCHAIN_DIR)/bin/i386-linux-g++
RSU_INCL     := -I$(RSU_TOOLCHAIN_DIR)/$(HEADER_DIR) -I$(RSU_TOOLCHAIN_DIR)/usr/$(HEADER_DIR) -I$(HEADER_DIR)
RSU_CFLAGS   := -Wall -W -Wshadow -Wcast-qual -Wwrite-strings -O3 -fPIC $(RSU_INCL)
RSU_C++FLAGS := -std=c++11 -Wconversion -fno-strict-aliasing $(RSU_CFLAGS)

OBU_CC       := $(OBU_TOOLCHAIN_DIR)/bin/i386-linux-gcc
OBU_C++      := $(OBU_TOOLCHAIN_DIR)/bin/i386-linux-g++
OBU_INCL     := -I$(OBU_TOOLCHAIN_DIR)/$(HEADER_DIR) -I$(OBU_TOOLCHAIN_DIR)/usr/$(HEADER_DIR) -I$(HEADER_DIR)
OBU_CFLAGS   := -Wall -W -Wshadow -Wcast-qual -Wwrite-strings -O3 -fPIC $(OBU_INCL)
OBU_C++FLAGS := -std=c++11 -Wconversion -fno-strict-aliasing $(OBU_CFLAGS)
