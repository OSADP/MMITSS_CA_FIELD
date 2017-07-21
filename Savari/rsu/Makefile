# Makefile for 'rsu' directory

include $(SAVARI_MK_DEFS)

MSGTRANS_DIR := $(RSU_DIR)/msgTransceiver
BIN_DIR      := $(RSU_DIR)/bin
EXEC         := $(MSGTRANS_DIR)/$(OBJ_DIR)/msgTransceiver

.PHONY: all install

all:
	(cd $(MSGTRANS_DIR); make clean; make all)

install:
	(mkdir -p $(BIN_DIR))
	(cp $(EXEC) $(BIN_DIR))
