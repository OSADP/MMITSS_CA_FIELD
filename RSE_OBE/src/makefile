# makefile
HOMEDIR      := /home/user
ASDTOOLCHAIN := $(HOMEDIR)/Sobos-SDK-for-i686-Linux/ASD_SDK-3.1.2/toolchain
RSUTOOLCHAIN := $(HOMEDIR)/Sobos-SDK-for-i686-Linux/RSE_SDK-3.1.2/toolchain

MRPBASEDIR  := $(HOMEDIR)/dsrc/mrp

APPBASEDIR  := $(HOMEDIR)/dsrc/apps
APPMRPDIR   := $(APPBASEDIR)/mrp

APPASNDIR     := $(APPMRPDIR)/asn1
ASNDIR        := $(MRPBASEDIR)/asn1
ASNSRCDIR     := $(ASNDIR)/src
ASNINCDIR     := $(ASNDIR)/include
ASDASNDIR     := $(APPASNDIR)/ASD
ASDASNOBJDIR  := $(ASDASNDIR)/obj
ASDASNRELDIR  := $(ASDASNDIR)/release
RSUASNDIR     := $(APPASNDIR)/RSU
RSUASNOBJDIR  := $(RSUASNDIR)/obj
RSUASNRELDIR  := $(RSUASNDIR)/release

APPJ2735DIR  := $(APPMRPDIR)/asn1j2735
J2375DIR     := $(MRPBASEDIR)/asn1j2735
J2375SRCDIR  := $(J2375DIR)/src
J2375INCDIR  := $(J2375DIR)/include
ASDJ2375DIR  := $(APPJ2735DIR)/ASD
ASDJ2375OBJDIR := $(ASDJ2375DIR)/obj
ASDJ2375RELDIR := $(ASDJ2375DIR)/release
RSUJ2375DIR  := $(APPJ2735DIR)/RSU
RSUJ2375OBJDIR := $(RSUJ2375DIR)/obj
RSUJ2375RELDIR := $(RSUJ2375DIR)/release

APPCOMMDIR   := $(APPMRPDIR)/comm
COMMDIR      := $(MRPBASEDIR)/comm
COMMSRCDIR   := $(COMMDIR)/src
COMMINCDIR   := $(COMMDIR)/include
ASDCOMMDIR   := $(APPCOMMDIR)/ASD
ASDCOMMOBJDIR := $(ASDCOMMDIR)/obj
ASDCOMMRELDIR := $(ASDCOMMDIR)/release
RSUCOMMDIR   := $(APPCOMMDIR)/RSU
RSUCOMMOBJDIR := $(RSUCOMMDIR)/obj
RSUCOMMRELDIR := $(RSUCOMMDIR)/release

# apps
SRCDIR := $(APPBASEDIR)/savariApps/src
INCDIR := $(APPBASEDIR)/savariApps/include
ifdef ASD
  RELDIR            := $(APPBASEDIR)/savariApps/ASD/release
  OBJDIR            := $(APPBASEDIR)/savariApps/ASD/obj
  ASNLIBDIR         := $(ASDASNRELDIR)
  ASNJ2375LIBDIR    := $(ASDJ2375RELDIR)
  MMITSSCOMMLIBDIR  := $(ASDCOMMRELDIR)
  ASNJ2375OBJDIR    := $(ASDJ2375OBJDIR)
  MMITSSCOMMOBJDIR  := $(ASDCOMMOBJDIR)
  TOOLCHAIN         := $(ASDTOOLCHAIN)
  OBJS              := $(OBJDIR)/obuAware.o  $(OBJDIR)/wmeConf.o  $(OBJDIR)/wmerx.o  $(OBJDIR)/wmetx.o
  CFLAGS            := -Wall -W -Wshadow -Wcast-qual -O3 -fPIC -DASD 
  TARGET            := $(OBJDIR)/obuAware $(OBJDIR)/wmetx $(OBJDIR)/wmerx
  TAREXE            := $(RELDIR)/obuAware $(RELDIR)/wmetx $(RELDIR)/wmerx
else
  RELDIR            := $(APPBASEDIR)/savariApps/RSU/release
  OBJDIR            := $(APPBASEDIR)/savariApps/RSU/obj
  ASNLIBDIR         := $(RSUASNRELDIR)
  ASNJ2375LIBDIR    := $(RSUJ2375RELDIR)
  MMITSSCOMMLIBDIR  := $(RSUCOMMRELDIR)
  ASNJ2375OBJDIR    := $(RSUJ2375OBJDIR)
  MMITSSCOMMOBJDIR  := $(RSUCOMMOBJDIR)
  TOOLCHAIN         := $(RSUTOOLCHAIN)
  OBJS              := $(OBJDIR)/wmeConf.o  $(OBJDIR)/wmerx.o  $(OBJDIR)/wmetx.o
  CFLAGS            := -Wall -W -Wshadow -Wcast-qual -O3 -fPIC -DRSU 
  TARGET            := $(OBJDIR)/wmetx $(OBJDIR)/wmerx
  TAREXE            := $(RELDIR)/wmetx $(RELDIR)/wmerx
endif

SAVARI_CC    := $(TOOLCHAIN)/bin/i386-linux-gcc
SAVARI_G++   := $(TOOLCHAIN)/bin/i386-linux-g++
SAVARI_C++   := $(TOOLCHAIN)/bin/i386-linux-c++
SAVARI_AR    := $(TOOLCHAIN)/bin/i386-linux-ar

ALLINC    := -I$(TOOLCHAIN)/include -I$(TOOLCHAIN)/usr/include -I$(ASNINCDIR) -I$(J2375INCDIR) -I$(COMMINCDIR) -I$(INCDIR)
COMPFLAGS := $(CFLAGS) $(ALLINC)

ASNLIB              := -L$(ASNLIBDIR) -lasn
ASNJ2735LIB         := -L$(ASNJ2375LIBDIR) -ldsrc
MMITSSCOMMLIB       := -L$(MMITSSCOMMLIBDIR) -lmmitsscomm
SAVARILIBWMEFWD     := -L$(TOOLCHAIN)/lib -L$(TOOLCHAIN)/usr/lib -lstdc++ -luClibc++ -leloop -lwme
SAVARILIBAWARE      := -L$(TOOLCHAIN)/lib -L$(TOOLCHAIN)/usr/lib -lstdc++ -luClibc++ -leloop -lgps -lgpsapi -lsavarimath

AROBJS := $(MMITSSCOMMOBJDIR)/socketUtils.o $(MMITSSCOMMOBJDIR)/timeUtils.o $(MMITSSCOMMOBJDIR)/msgUtils.o $(ASNJ2375OBJDIR)/wmeUtils.o

all: $(OBJDIR) $(RELDIR) $(OBJS) $(TARGET)

$(OBJDIR):
	mkdir -p $(OBJDIR)

$(RELDIR):
	mkdir -p $(RELDIR)
  
$(OBJDIR)/%.o: $(SRCDIR)/%.cpp
	${SAVARI_G++} $(COMPFLAGS) -c -o $@ $<
  
$(OBJDIR)/obuAware: $(OBJDIR)/obuAware.o
	${SAVARI_G++}  -o $(OBJDIR)/obuAware $(OBJDIR)/obuAware.o $(MMITSSCOMMLIB) $(ASNJ2735LIB) $(ASNLIB) $(SAVARILIBAWARE)

$(OBJDIR)/wmetx: $(OBJDIR)/wmetx.o $(OBJDIR)/wmeConf.o
	${SAVARI_G++} -o $(OBJDIR)/wmetx $(OBJDIR)/wmetx.o $(OBJDIR)/wmeConf.o $(AROBJS) $(SAVARILIBWMEFWD)
	
$(OBJDIR)/wmerx: $(OBJDIR)/wmerx.o $(OBJDIR)/wmeConf.o
	${SAVARI_G++} -o $(OBJDIR)/wmerx $(OBJDIR)/wmerx.o $(OBJDIR)/wmeConf.o $(AROBJS) $(SAVARILIBWMEFWD)
  
clean:
	rm -f $(OBJS) $(TARGET)

install:
	cp $(TARGET) $(RELDIR)
	chmod +x $(TAREXE)  

