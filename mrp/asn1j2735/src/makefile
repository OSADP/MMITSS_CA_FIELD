# makefile
ASNINDIR := ../../asn1/include
ASNLIBDIR := ../../asn1/release
INCDIR  := ../include
RELDIR  := ../release
OBJDIR  := ../obj

ASNLIB := -L$(ASNLIBDIR) -lasn

COMP := -Wall -W -Wconversion -Wshadow -Wcast-qual -Wwrite-strings -O3 -fPIC -fno-strict-aliasing 
# -DNDEBUG to disable -DNDEBUG assert

ALLINC := -I/usr/include -I$(INCDIR) -I$(ASNINDIR) 
COMPFLAGS := $(COMP) $(ALLINC)

C++ := g++
AR := ar
ARFLAGS := -rv

OBJS := $(patsubst %.cpp,$(OBJDIR)/%.o,$(wildcard *.cpp))

TARGET := $(OBJDIR)/libdsrc.a
TARLIB := $(RELDIR)/libdsrc.a
	
all: $(OBJDIR) $(RELDIR) $(OBJS) $(TARGET)

$(OBJDIR):
	mkdir -p $(OBJDIR)

$(RELDIR):
	mkdir -p $(RELDIR)	

$(OBJDIR)/%.o: %.cpp
	$(C++) $(COMPFLAGS) -c -o $@ $<
	
$(TARGET): $(OBJS)
	$(AR) $(ARFLAGS) -o $(TARGET) $(OBJS)	

clean:
	rm -f $(OBJS) $(TARGET) $(TARLIB)

install:
	cp $(TARGET) $(RELDIR)
	chmod +x $(TARLIB)



