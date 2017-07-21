# Makefile for 'tci' directory

include $(MRP_MK_DEFS)

TARGET  := $(OBJ_DIR)/tci
OBJS    := $(patsubst $(SOURCE_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(wildcard $(SOURCE_DIR)/*.cpp))
ADDINC  := -I$(J2735_DIR)/$(HEADER_DIR) -I$(UTILS_DIR)/$(HEADER_DIR)
LINKSO  := -Wl,-rpath,$(MRP_SO_DIR) -L$(MRP_SO_DIR) -lutils

all: $(OBJ_DIR) $(OBJS) $(TARGET)

$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

$(OBJ_DIR)/%.o: $(SOURCE_DIR)/%.cpp
	$(MRP_C++) $(MRP_C++FLAGS) $(ADDINC) -c -o $@ $<

$(TARGET): $(OBJS)
	$(MRP_C++) $(MRP_C++FLAGS) -o $(TARGET) $(OBJS) $(LINKSO)

clean:
	rm -f $(OBJS) $(TARGET)

