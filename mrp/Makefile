# Makefile for 'mrp' directory

include $(MRP_MK_DEFS)

SYSTEMD_PATH := /etc/systemd/system
MRP_SERVICE  := mmitss.mrp.service

EXEC := $(TCI_DIR)/$(OBJ_DIR)/tci $(DATAMGR_DIR)/$(OBJ_DIR)/dataMgr $(MRPAWARE_DIR)/$(OBJ_DIR)/mrpAware

.PHONY: all asn directory mrp install startup

all: asn mrp

asn:
	(cd $(ASN1_DIR); make clean; make all)

directory:
	(mkdir -p $(MRP_EXEC_DIR))
	(mkdir -p $(TIMECARD_DIR))
	(mkdir -p $(MRP_LOG_DIR))
	(mkdir -p $(AWRLOG_DIR))
	(mkdir -p $(MGRLOG_DIR))
	(mkdir -p $(TCILOG_DIR))

mrp:
	(cd $(J2735_DIR); make clean; make all)
	(cd $(LOCAWARE_DIR); make clean; make all)
	(cd $(UTILS_DIR); make clean; make all)
	(cd $(TCI_DIR); make clean; make all)
	(cd $(DATAMGR_DIR); make clean; make all)
	(cd $(MRPAWARE_DIR); make clean; make all)

install: directory
	(sudo systemctl stop $(MRP_SERVICE))
	(cp $(EXEC) $(MRP_EXEC_DIR))
	(cp $(SCRIPT_DIR)/*.sh $(MRP_EXEC_DIR))
	(cd $(MRP_EXEC_DIR); chmod +x $(EXEC); chmod +x *.sh)

startup:
	(sudo cp -f $(SCRIPT_DIR)/$(MRP_SERVICE) $(SYSTEMD_PATH))
	(sudo chmod +x $(SYSTEMD_PATH)/$(MRP_SERVICE))
	(sudo systemctl daemon-reload)
	(sudo systemctl enable $(MRP_SERVICE))

