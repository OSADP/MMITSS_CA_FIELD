//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************
# Makefile
INCDIR := ../include
OBJDIR := ../obj
RELDIR := ../release

TARGET := $(OBJDIR)/libasn.a
TARLIB := $(RELDIR)/libasn.a
OBJS   := $(patsubst %.c,$(OBJDIR)/%.o,$(wildcard *.c))

C++ := gcc
AR := ar
ARFLAGS := -rv
COMP := -Wall -W -Wshadow -Wcast-qual -O3 -fPIC
#-Wconversion -Wwrite-strings
ALLINC := -I$(INCDIR)
COMPFLAGS := $(COMP) $(ALLINC) 

all: $(OBJDIR) $(RELDIR) $(OBJS) $(TARGET)

$(OBJDIR):
	mkdir -p $(OBJDIR)
  
$(RELDIR):
	mkdir -p $(RELDIR)

$(TARGET): $(OBJS)
	$(AR) $(ARFLAGS) -o $(TARGET) $(OBJS)	
	
$(OBJDIR)/%.o: %.c
	$(C++) $(COMPFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJS) $(TARGET) $(TARLIB)

install:
	cp $(TARGET) $(RELDIR)
	chmod +x $(TARLIB)
