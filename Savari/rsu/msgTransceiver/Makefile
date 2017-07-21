# Makefile for 'msgTransceiver' directory

include $(SAVARI_MK_DEFS)

TARGET     := $(OBJ_DIR)/msgTransceiver
OBJS       := $(patsubst $(SOURCE_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(wildcard $(SOURCE_DIR)/*.cpp))
SAVARILIBS := -L$(RSU_TOOLCHAIN_DIR)/lib -L$(RSU_TOOLCHAIN_DIR)/usr/lib -lstdc++ -luClibc++ -lwme -lradio -lsocket
#LINKSO  := -Wl,-rpath,$(RSU_TOOLCHAIN_DIR)/$(LIB_DIR) -L$(RSU_TOOLCHAIN_DIR)/$(LIB_DIR)
#LINKSO  += -Wl,-rpath,$(RSU_TOOLCHAIN_DIR)/usr/$(LIB_DIR) -L$(RSU_TOOLCHAIN_DIR)/usr/$(LIB_DIR) -luClibc++ -lwme -lradio -lsocket

all: $(OBJ_DIR) $(OBJS) $(TARGET)

$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

$(OBJ_DIR)/%.o: $(SOURCE_DIR)/%.cpp
	$(RSU_C++) $(RSU_CFLAGS) -c -o $@ $<

$(TARGET): $(OBJS)
	$(RSU_C++)  $(RSU_CFLAGS) -o $(TARGET) $(OBJS) $(SAVARILIBS)

clean:
	rm -f $(OBJS) $(TARGET)
