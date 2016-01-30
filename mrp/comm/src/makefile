# makefile
ASNJ2375INDIR		:= ../../asn1j2735/include
INCDIR  				:= ../include
RELDIR  				:= ../release
OBJDIR  				:= ../obj

COMP := -Wall -W -Wconversion -Wshadow -Wcast-qual -Wwrite-strings -O3 -fPIC -fno-strict-aliasing 
# -DNDEBUG to disable -DNDEBUG assert

ALLINC := -I/usr/include -I$(INCDIR) -I$(ASNJ2375INDIR) 
COMPFLAGS := $(COMP) $(ALLINC)

C++ := g++
AR := ar
ARFLAGS := -rv

OBJS := $(patsubst %.cpp,$(OBJDIR)/%.o,$(wildcard *.cpp))

TARGET := $(OBJDIR)/libmmitsscomm.a
TARLIB := $(RELDIR)/libmmitsscomm.a
	
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

