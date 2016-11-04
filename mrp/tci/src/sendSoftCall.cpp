//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************
#include <iostream>
#include <fstream> 
#include <sstream>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <cstring>
#include <unistd.h>
#include <termios.h> 
#include <iomanip> 
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/poll.h>
#include <fcntl.h>
#include <csetjmp>
#include <csignal>
#include <stdint.h>
#include <bitset>

#include "socketUtils.h"
#include "msgUtils.h"
#include "tciConf.h"
#include "catchSignal.h"

using namespace std;

void do_usage(const char* progname)
{
  cerr << "Usage" << progname << endl;
  cerr << "\t-s socket.conf" << endl;
  exit (1);
}

int main(int argc, char** argv) 
{
  int option;
  char* f_socketConfig = NULL;
  
  // getopt
  while ((option = getopt(argc, argv, "s:")) != EOF) 
  {
    switch(option) 
    {
    case 's':
      f_socketConfig = strdup(optarg);
      break;
    default:
      do_usage(argv[0]);
      break;
    }
  }
  if (f_socketConfig == NULL)
    do_usage(argv[0]);
  
  /* ----------- preparation -------------------------------------*/
  /// instance TCIconfig::configTCI to read configuration files
  TCIconfig::configTCI s_tciConfig(f_socketConfig);
  if (!s_tciConfig.isConfigSucceed())
  {
    cerr << argv[0] << ": failed construct TCIconfig::configTCI";
    cerr << ", f_socketConfig = " << f_socketConfig << endl;
    delete f_socketConfig;
    exit(1);
  }
  delete f_socketConfig;
  
  /// open socket
  int fd_tciSend = socketUtils::client(s_tciConfig.getSocketAddr(TCIconfig::socketHandleEnum_t::MGRLISTEN),false);
  if (fd_tciSend < 0)
  {
    cerr << argv[0] << ": failed create send softcall socket" << endl;
    exit(1);
  }
  
  /* ----------- intercepts signals -------------------------------------*/
  if (setjmp(exit_env) != 0) 
  {
    close(fd_tciSend);
    return (0);
  }
  else
    sig_ign( sig_list, sig_hand );
  
  /* ----------- local variables -------------------------------------*/  
  char sendbuf[MAXUDPMSGSIZE];
  mmitss_udp_header_t header;
  header.msgheader = msg_header;
  header.msgid = msgid_softcall;
  header.ms_since_midnight = 0;
  header.length = static_cast<uint16_t>(sizeof(softcall_request_t));
  memcpy(&sendbuf[0],&header,sizeof(mmitss_udp_header_t));
  
  softcall_request_t request;
  request.callphase = 0;
  request.callobj = static_cast<uint8_t>(softcall_enum_t::NONE);
  request.calltype  = static_cast<uint8_t>(softcall_enum_t::UNKNOWN);
  
  bitset<8> phases2call;
  
  while(1)
  {
    int softcallObj = 0;
    int softcallType = 0;
    int softcallPhase = 0;
    phases2call.reset();
    
		while (!(softcallObj >= 1 && softcallObj <=3))
		{
			fprintf(stdout,"\nInput call object: 1-Ped, 2-Veh,3-TSP: ");
      if (scanf("%d",&softcallObj) == 1) {request.callobj = static_cast<uint8_t>(softcallObj);}
      else {softcallObj = 0;}
		}
		while (!(softcallType >= 1 && softcallType <=3))
		{
			fprintf(stdout,"\nInput call type: 1-Call, 2-Extension,3-Cancel: ");
      if (scanf("%d",&softcallType) == 1) {request.calltype = static_cast<uint8_t>(softcallType);}
      else {softcallType = 0;}
		}
		while (!(softcallPhase >= 1 && softcallPhase <=8))
		{
			fprintf(stdout,"\nInput call phase: 1 ... 8: ");
      if (scanf("%d",&softcallPhase) == 1)
      {
        phases2call.set(softcallPhase-1);
        request.callphase = static_cast<uint8_t>(phases2call.to_ulong());
      }
      else
        softcallPhase = 0;
		}
     
    cout << "You input: softcallObj = " << static_cast<int>(request.callobj);
    cout << " softcallType = " << static_cast<int>(request.calltype);
    cout << " softcallPhase = " << softcallPhase << endl;
    
    memcpy(&sendbuf[sizeof(mmitss_udp_header_t)],&request,sizeof(softcall_request_t));
    socketUtils::sendall(fd_tciSend,sendbuf,sizeof(mmitss_udp_header_t) + sizeof(softcall_request_t));
    cout << "Softcall sent" << endl << endl;
  }
}
