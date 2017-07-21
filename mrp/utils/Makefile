# Makefile for 'utils' directory

include $(MRP_MK_DEFS)

LIBNAME := libutils.so
VERSION := 0
SONAME  := $(LIBNAME).1
SOFLAGS := -shared -fPIC -Wl,-soname,$(SONAME)
TARGET  := $(LIB_DIR)/$(SONAME).$(VERSION)
OBJS    := $(patsubst $(SOURCE_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(wildcard $(SOURCE_DIR)/*.cpp))

all: $(OBJ_DIR) $(LIB_DIR) $(OBJS) $(TARGET)

$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

$(LIB_DIR):
	mkdir -p $(LIB_DIR)

$(OBJ_DIR)/%.o: $(SOURCE_DIR)/%.cpp
	$(MRP_C++) $(MRP_C++FLAGS) -c -o $@ $<

$(TARGET): $(OBJS)
	($(MRP_C++) $(SOFLAGS) -o $(TARGET) $(OBJS))
	(mkdir -p $(MRP_SO_DIR); cp $(TARGET) $(MRP_SO_DIR))
	(cd $(MRP_SO_DIR); rm -f $(SONAME); ln -s $(SONAME).$(VERSION) $(SONAME))
	(cd $(MRP_SO_DIR); rm -f $(LIBNAME); ln -s $(SONAME).$(VERSION) $(LIBNAME))

clean:
	rm -f $(OBJS) $(TARGET)
