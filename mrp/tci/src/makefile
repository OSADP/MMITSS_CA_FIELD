# makefile
ASNJ2375INDIR			:= ../../asn1j2735/include
MMITSSCOMMINDIR		:= ../../comm/include
MMITSSCOMMLIBDIR	:= ../../comm/release
INCDIR  					:= ../include
RELDIR  					:= ../release
OBJDIR  					:= ../obj
MMITSSCOMMLIB			:= -L$(MMITSSCOMMLIBDIR) -lmmitsscomm

COMP := -Wall -W -Wconversion -Wshadow -Wcast-qual -Wwrite-strings -O3 -fPIC -fno-strict-aliasing 
# -DNDEBUG to disable -DNDEBUG assert

ALLINC := -I/usr/include -I$(INCDIR) -I$(ASNJ2375INDIR) -I$(MMITSSCOMMINDIR)
COMPFLAGS := $(COMP) $(ALLINC)

C++ := g++

TARGET := $(OBJDIR)/tci 
TAREXE := $(RELDIR)/tci 
	
all: $(OBJDIR) $(RELDIR) $(TARGET)

$(OBJDIR):
	mkdir -p $(OBJDIR)

$(RELDIR):
	mkdir -p $(RELDIR)	

$(OBJDIR)/tci: tci.cpp 
	$(C++)  $(COMPFLAGS) tci.cpp -o $(OBJDIR)/tci $(MMITSSCOMMLIB)
	
clean:
	rm -f $(TARGET) $(TAREXE)

install:
	cp $(TARGET) $(RELDIR)
	chmod +x $(TAREXE)
