# Makefile for 'mrpAware' directory

include $(MRP_MK_DEFS)

TARGET  := $(OBJ_DIR)/mrpAware
OBJ     := $(OBJ_DIR)/mrpAware.o
OBJS    := $(OBJ) $(TCI_DIR)/$(OBJ_DIR)/msgDefs.o
ADDINC  := -I$(J2735_DIR)/$(HEADER_DIR) -I$(LOCAWARE_DIR)/$(HEADER_DIR) -I$(UTILS_DIR)/$(HEADER_DIR) -I$(TCI_DIR)/$(HEADER_DIR)
LINKSO  := -Wl,-rpath,$(MRP_SO_DIR) -L$(MRP_SO_DIR) -Wl,--as-needed -llocAware -ldsrc -lasn -lutils

all: $(OBJ_DIR) $(OBJ) $(TARGET)

$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

$(OBJ): $(SOURCE_DIR)/mrpAware.cpp
	$(MRP_C++) $(MRP_C++FLAGS) $(ADDINC) -c $(SOURCE_DIR)/mrpAware.cpp -o $(OBJ)

$(TARGET): $(OBJS)
	$(MRP_C++) $(MRP_C++FLAGS) -o $(TARGET) $(OBJS) $(LINKSO)

clean:
	rm -f $(OBJ) $(TARGET)
