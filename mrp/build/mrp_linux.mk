# common definitions for MRP builds on Linux-like systems

# add the following line into individual user's ~/.bashrc
#  export MRP_MK_DEFS=/home/MMITSS-CA/mrp/build/mrp_linux.mk
# then source ~/.bashrc
# can use 'echo $MRP_MK_DEFS' from the command line to check whether MRP_MK_DEFS has been set

MRP_DIR       := /home/MMITSS-CA/mrp
ASN1_DIR      := $(MRP_DIR)/asn1
J2735_DIR	    := $(MRP_DIR)/asn1j2735
LOCAWARE_DIR  := $(MRP_DIR)/locationAware
UTILS_DIR     := $(MRP_DIR)/utils
TCI_DIR       := $(MRP_DIR)/tci
DATAMGR_DIR   := $(MRP_DIR)/dataMgr
MRPAWARE_DIR  := $(MRP_DIR)/mrpAware
SCRIPT_DIR    := $(MRP_DIR)/script

MRP_EXEC_DIR  := $(MRP_DIR)/bin
MRP_SO_DIR    := $(MRP_DIR)/lib
TIMECARD_DIR  := $(MRP_DIR)/timingCard
MRP_LOG_DIR   := $(MRP_DIR)/logs
AWRLOG_DIR    := $(MRP_LOG_DIR)/awr
MGRLOG_DIR    := $(MRP_LOG_DIR)/mgr
TCILOG_DIR    := $(MRP_LOG_DIR)/tci

SOURCE_DIR    := src
HEADER_DIR    := include
OBJ_DIR       := obj
LIB_DIR       := lib

# compiler flags
MRP_CC        := gcc
MRP_C++       := g++
LOCAL_INCL    := -I/usr/$(HEADER_DIR) -I$(HEADER_DIR)
MRP_CFLAGS    := -g -Wall -W -Wshadow -Wcast-qual -Wwrite-strings -O3 -fPIC $(LOCAL_INCL)
MRP_C++FLAGS  := -std=c++11 -Wconversion -fno-strict-aliasing $(MRP_CFLAGS)
