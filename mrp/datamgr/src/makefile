# makefile
ASNINDIR 					:= ../../asn1/include
ASNLIBDIR 				:= ../../asn1/release
ASNJ2375INDIR			:= ../../asn1j2735/include
ASNJ2375LIBDIR 		:= ../../asn1j2735/release
MMITSSCOMMINDIR		:= ../../comm/include
MMITSSCOMMLIBDIR	:= ../../comm/release
INCDIR  					:= ../include
RELDIR  					:= ../release
OBJDIR  					:= ../obj
ASNLIB 						:= -L$(ASNLIBDIR) -lasn
ASNJ2735LIB 			:= -L$(ASNJ2375LIBDIR) -ldsrc
MMITSSCOMMLIB			:= -L$(MMITSSCOMMLIBDIR) -lmmitsscomm

COMP := -Wall -W -Wconversion -Wshadow -Wcast-qual -Wwrite-strings -O3 -fPIC -fno-strict-aliasing 
# -DNDEBUG to disable -DNDEBUG assert

ALLINC := -I/usr/include -I$(INCDIR) -I$(ASNINDIR) -I$(ASNJ2375INDIR) -I$(MMITSSCOMMINDIR)
COMPFLAGS := $(COMP) $(ALLINC)

C++ := g++

TARGET := $(OBJDIR)/dataMgr
TAREXE := $(RELDIR)/dataMgr
	
all: $(OBJDIR) $(RELDIR) $(TARGET)

$(OBJDIR):
	mkdir -p $(OBJDIR)

$(RELDIR):
	mkdir -p $(RELDIR)	

$(OBJDIR)/dataMgr: dataMgr.cpp 
	$(C++)  $(COMPFLAGS) dataMgr.cpp -o $(OBJDIR)/dataMgr $(MMITSSCOMMLIB) $(ASNJ2735LIB) $(ASNLIB)
	
clean:
	rm -f $(TARGET) $(TAREXE)

install:
	cp $(TARGET) $(RELDIR)
	chmod +x $(TAREXE)


