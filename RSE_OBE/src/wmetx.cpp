/* wmetx.cpp
    receives MRP packets on eth0 and forwards the packets to wme for broadcast
*/
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <vector>
#include <getopt.h>
#include <iomanip> 			// setw, setfill

#include "libeloop.h"	  // eloop_* functions

#include "wmetx.h"
#include "wmeConf.h"
#include "socketUtils.h"
#include "wmeUtils.h"
#include "timeUtils.h"
#include "msgUtils.h"

#ifdef RSU
#include "savariWAVE_api.h"
#endif

using namespace std;

WMEconfig::appHandler_t appHandle;
std::vector<savariWmeHandler_t> savariHandle;
savari_wme_handler_t handler[2];
std::map<uint32_t,size_t> wmePSIDmap; /// key: network byte psid, value: index in applications
map<uint32_t,size_t>::const_iterator ite;
map<uint8_t,uint32_t>::const_iterator it;

int fd_listen;
uint8_t recvbuf[MAXDSRCMSGSIZE];

ofstream OS_log;
string s_logfilename;
long long logFileInterval;
long long logfile_tms_minute;
unsigned long l_logRows;

timeUtils::fullTimeStamp_t fullTimeStamp;

bool verbose = false;
bool logFile = false;
bool logDetails = false;

void do_usage(const char* progname)
{
	cerr << progname << "Usage: " << endl;
	cerr << "\t-c wmefwd.conf" << endl;
	cerr << "\t-f log flag: 0 - no log; 1 - simple log; 2 - detail log" << endl;
	cerr << "\t-v turn on verbose" << endl;
	exit (1);
}

int main (int argc, char** argv)
{
	int option;
	char* f_wmefwdConfig = NULL;
  
	/// getopt
	while ((option = getopt(argc, argv, "c:f:v")) != EOF) 
	{
		switch(option) 
		{
		case 'c':
			f_wmefwdConfig = strdup(optarg);
			break;
    case 'f':
      {
        int flag = atoi(optarg);
        if (flag > 0) 
          logFile = true;
        if (flag > 1)
          logDetails = true;
      }
      break;
		case 'v':
			verbose = true;
			break;
		default:
			do_usage(argv[0]);
			break;
		}
	}
	if (f_wmefwdConfig == NULL)
		do_usage(argv[0]);
	
	/* ----------- preparing -------------------------------------*/  
  timeUtils::getFullTimeStamp(fullTimeStamp);  
	/// instance WMEconfig::configWme to read configuration files
	WMEconfig::configWme s_appConfig(f_wmefwdConfig);
	if (!s_appConfig.isConfigSucceed())
	{
    cerr << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
		cerr << ", failed construct WMEconfig::configWme, filename " << f_wmefwdConfig << endl;
		delete f_wmefwdConfig;
		exit(1);
	}
	delete f_wmefwdConfig;
  
	appHandle = s_appConfig.getAppHandler();
  if (appHandle.applications.empty())
  {
    cerr << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    cerr << ", appHandle: no applications" << endl;
    exit(1);
  }
  
  /// open log file
  logFileInterval = static_cast<long long>(appHandle.i_logInterval) * 60 * 1000LL; 
  s_logfilename = appHandle.s_logPath + std::string("/wmetx.") + timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp) + std::string(".log");
  if (logFile)
  {
    OS_log.open(s_logfilename.c_str());
    if (!OS_log.is_open())
    {
      cerr << "Failed open logfile " << s_logfilename << endl;
      exit(1);      
    }
    logfile_tms_minute = fullTimeStamp.tms_minute;
    l_logRows = 0;
  }
	if (verbose)
	{
		cout << "appHandle " << appHandle.applications.size() << " applications, map size " << appHandle.appPSIDmap.size() << endl;		
		for (ite = appHandle.appPSIDmap.begin(); ite != appHandle.appPSIDmap.end(); ite++)
		{
      if (appHandle.applications[ite->second].e_direction == WMEconfig::appConfigEnum_t::RX)
        continue;
			cout << "Application " << appHandle.applications[ite->second].s_name;
			cout << " psid: " << "0x" << hex << uppercase << ite->first << dec;
      cout << " network byte order psid: " << "0x" << hex << uppercase << wme_convert_psid_be(ite->first) << dec;
			cout << ", application index: " << ite->second << endl;
		}
	}
  if (logFile)
	{
		OS_log << "appHandle " << appHandle.applications.size() << " applications, map size " << appHandle.appPSIDmap.size() << endl;		
		for (ite = appHandle.appPSIDmap.begin(); ite != appHandle.appPSIDmap.end(); ite++)
		{
      if (appHandle.applications[ite->second].e_direction == WMEconfig::appConfigEnum_t::RX)
        continue;
			OS_log << "Application " << appHandle.applications[ite->second].s_name;
			OS_log << " psid: " << "0x" << hex << uppercase << ite->first << dec;
      OS_log << " network byte order psid: " << "0x" << hex << uppercase << wme_convert_psid_be(ite->first) << dec;
			OS_log << ", application index: " << ite->second << endl;
		}
    l_logRows++;
	}
  
	/// open eth0 listen socket
	fd_listen = socketUtils::server(*(appHandle.mpListenAddr),false);
	if (fd_listen < 0)
	{
    cerr << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
		cerr << ", failed create listen socket" << endl;
    if (logFile)
    {
      OS_log << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
      OS_log << ", failed create listen socket" << endl;
      OS_log.close();
    }
		exit (1);
	}
  if (verbose)
  {
    cout << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    cout << ", fd_listen = " << fd_listen << ", listen on " << appHandle.mpListenAddr->name << ":" << appHandle.mpListenAddr->port << endl;
  }
  if (logFile)
  {
    OS_log << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    OS_log << ", fd_listen = " << fd_listen << ", listen on " << appHandle.mpListenAddr->name << ":" << appHandle.mpListenAddr->port << endl;
    l_logRows++;
  }
	  
	/// fill structure savariWmeHandler_t
	uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	savariHandle.resize(appHandle.applications.size());
	for (size_t i = 0; i < appHandle.applications.size(); i++)
	{
		savariHandle[i].isRegistered = false;
    /// wmereq
		memset(&(savariHandle[i].wmereq),0,sizeof(struct savariwme_reg_req));
		savariHandle[i].wmereq.channel = appHandle.applications[i].channel;
		memcpy(savariHandle[i].wmereq.destmacaddr, broadcast_mac, SAVARI1609_IEEE80211_ADDR_LEN);
		savariHandle[i].wmereq.psid = wme_convert_psid_be(appHandle.applications[i].psid);
    wmePSIDmap[savariHandle[i].wmereq.psid] = i;
		savariHandle[i].wmereq.priority = appHandle.applications[i].priority;
#ifdef RSU		
		if (appHandle.applications[i].e_channelMode == wmeUtils::channelmode_enum_t::CONTINUOUS)
		{			
      /* type of user application request:
          SAVARI1609_USER_AUTOACCESS_ONMATCH (Switch Between 178 and SCH after receiving Matching WSA from RSE)
          SAVARI1609_USER_AUTOACCESS_UNCOND (Start Switching between 178 and SCH without Waiting for Matching WSA from RSEs)
          SAVARI1609_USER_AUTOACCESS_NOSCHACCESS(CCH Only Mode. No Switching). Only applicable if channel_access is ALTERNATING
      */
			savariHandle[i].wmereq.request_type = SAVARI1609_USER_AUTOACCESS_ONMATCH;
			savariHandle[i].wmereq.extended_access = 0xFFFF;
			savariHandle[i].wmereq.channel_access = SAVARI1609_CHANNEL_ACCESS_CONTINUOUS;   // value 0
		}
		else
		{
			savariHandle[i].wmereq.request_type = SAVARI1609_USER_AUTOACCESS_UNCOND;
			savariHandle[i].wmereq.extended_access = 0;
			savariHandle[i].wmereq.channel_access = SAVARI1609_CHANNEL_ACCESS_ALTERNATING;  // value 1
		}	
#else
//    savariHandle[i].wmereq.channel_access = LIBWME_CHANNEL_ACCESS_CONTINUOUS; // default LIBWME_CHANNEL_ACCESS_CONTINUOUS = 0
		savariHandle[i].wmereq.request_type = LIBWME_USER_AUTOACCESS_ONMATCH;
		savariHandle[i].wmereq.extended_access = 0xFFFF;    // continuous access
#endif		
		savariHandle[i].wmereq.immediate_access = 0;
//    savariHandle[i].wmereq.wsatype = SAVARI1609_WSA_UNSECURED;  /// default SAVARI1609_WSA_UNSECURED = 0
		savariHandle[i].wmereq.psc_length = static_cast<int>(appHandle.applications[i].s_psc.size() + 1);
		memcpy(savariHandle[i].wmereq.psc, appHandle.applications[i].s_psc.c_str(), savariHandle[i].wmereq.psc_length);
    savariHandle[i].wmereq.local_service_index = static_cast<int>(i+1);
    if (appHandle.applications[i].e_iface == wmeUtils::channelmode_enum_t::ATH0_)
      savariHandle[i].wmereq.secondradio = 0;
    else
      savariHandle[i].wmereq.secondradio = 1;
    /// wmetx
		memset(&(savariHandle[i].wmetx),0, sizeof(struct savariwme_tx_req));
		savariHandle[i].wmetx.channel = appHandle.applications[i].channel;
		savariHandle[i].wmetx.psid = savariHandle[i].wmereq.psid;
		savariHandle[i].wmetx.priority = appHandle.applications[i].priority;
		savariHandle[i].wmetx.datarate = 6; //3Mbps
		savariHandle[i].wmetx.txpower = 15; //in dbM
		memcpy(savariHandle[i].wmetx.mac, broadcast_mac, SAVARI1609_IEEE80211_ADDR_LEN);
		savariHandle[i].wmetx.expiry_time = 0;
		savariHandle[i].wmetx.element_id = WAVE_ELEMID_WSMP;
		savariHandle[i].wmetx.tx_length = 0;    // fill tx_length on-the-fly
		savariHandle[i].wmetx.supp_enable = 0;  // do not consider the safetysupp to transmit in the WSMP header
    savariHandle[i].wmetx.safetysupp = 0;
  }
    
  /// connect to WME
  for (int i=0;i<2;i++)
  {
    handler[i] = wme_init("::1", (char*)wmeUtils::iface[i]);
    if (handler[i] < 0)
    {
      cerr << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);          
      cerr << ", failed wme_init for interface " << wmeUtils::iface[i] << endl;
      if (logFile)
      {
        OS_log << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);          
        OS_log << ", failed wme_init for interface " << wmeUtils::iface[i] << endl;
        OS_log.close();
      }      
      close(fd_listen);
      exit(1);
    }
    if (verbose)
    {
      cout << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);          
      cout << ", handler = " << handler[i] << ", wme_init succeed for interface " << wmeUtils::iface[i] << endl;
    }
    if (logFile)
    {
      OS_log << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);          
      OS_log << ", handler = " << handler[i] << ", wme_init succeed for interface " << wmeUtils::iface[i] << endl;
      l_logRows++;
    }
  }
  
	/// initial event loop
	if (eloop_init(0) < 0)
	{
    cerr << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);          
		cerr << ", failed eloop_init" << endl;
    if (logFile)
    {
      OS_log << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);          
      OS_log << ", failed eloop_init" << endl;
      OS_log.close();
    }
		close(fd_listen);
		exit(1);
	}
  
  /// register applications to WME
  int ret;
	for (size_t i = 0; i < appHandle.applications.size(); i++)
	{
    if (appHandle.applications[i].e_direction == WMEconfig::appConfigEnum_t::RX)
      continue;
    if (appHandle.applications[i].e_registerType == WMEconfig::appConfigEnum_t::USER)
      ret = wme_register_user(handler[appHandle.applications[i].e_iface],&savariHandle[i].wmereq);
    else
      ret = wme_register_provider(handler[appHandle.applications[i].e_iface],&savariHandle[i].wmereq);
    
    if (ret < 0)
    {
      cerr << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);                
      cerr << ", failed wme_register for " << appHandle.applications[i].s_name << endl;
      if (logFile)
      {
        OS_log << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);                
        OS_log << ", failed wme_register for " << appHandle.applications[i].s_name << endl;
      }
      close_all();
    }
    
    savariHandle[i].isRegistered = true;
    if (verbose)
    {
      cout << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);                
      cout << ", wme_register succeed for " << appHandle.applications[i].s_name << endl;
    }
    if (logFile)
    {
      OS_log << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);                
      OS_log << ", wme_register succeed for " << appHandle.applications[i].s_name << endl;
      l_logRows++;
    }
  }
    
  /// fd_listen eloop_register_read_sock
  ret = eloop_register_read_sock(fd_listen,wsm_tx,0,0);
  if (ret < 0)
  {
    cerr << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);                
    cerr << ", failed eloop_register_read_sock for listening socket" << endl;
    if (logFile)
    {
      OS_log << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);                
      OS_log << ", failed eloop_register_read_sock for listening socket" << endl;
    }
    close_all();
  }
  if (verbose)
  {
    cout << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);                
    cout << ", eloop_register_read_sock succeed for listening socket" << endl;
  }
  if (logFile)
  {
    OS_log << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);                
    OS_log << ", eloop_register_read_sock succeed for listening socket" << endl;
    l_logRows++;
  } 
  
	/* ----------- intercepts signals -------------------------------------*/
	if (setjmp(exit_env) != 0) 
	{
    timeUtils::getFullTimeStamp(fullTimeStamp);
    cerr << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);          
		cerr << ", received user termination signal, quit!" << endl;
    if (logFile)
    {
      OS_log << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
      OS_log << ", received user termination signal, quit!" << endl;
    }
    close_all();
	}
	else
		sig_ign( sig_list, sig_hand );  
  
	if (verbose)
	{
    cout << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
		cout << ", start eloop_run waiting for data ..." << endl;
	}
  if (logFile)
  {
    OS_log << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    OS_log << ", start eloop_run waiting for data ..." << endl;
    l_logRows++;
  }
	eloop_run();
	
	/// should not be here
  if (logFile)
  {
    timeUtils::getFullTimeStamp(fullTimeStamp);
    OS_log << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    OS_log << ", exited from eloop_run!" << endl;
  }  
  close_all();
	return 0;
}

/// wsm_tx - wait for data on eth0 interface socket fd_listen
void wsm_tx(int sock_t, void *eloop_data, void *user_data)
{
  timeUtils::getFullTimeStamp(fullTimeStamp);
	ssize_t bytesReceived = recv(sock_t,recvbuf,sizeof(recvbuf),0);
  /// recvbuf: mmitss header + payload
  
  if (bytesReceived > 0 && (size_t)bytesReceived > sizeof(mmitss_udp_header_t))
  {
    mmitss_udp_header_t* header = (mmitss_udp_header_t*)&recvbuf[0];
    if (header->msgheader != msg_header)
      return;
    if (verbose)
    {
      cout << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);              
      cout << ", receive on eth0 " << bytesReceived << " bytes, 0x" << hex << uppercase << static_cast<int>(header->msgid) << dec << endl;
    }
    if (logDetails)
    {
      OS_log << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);              
      OS_log << ", receive on eth0 " << bytesReceived << " bytes, 0x" << hex << uppercase << static_cast<int>(header->msgid) << dec << endl;
      l_logRows++;
    }
    
		it = wmeUtils::mmitssMsgid2psidMap.find(header->msgid);
    if (it == wmeUtils::mmitssMsgid2psidMap.end())
    {
      cerr << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);              
			cerr << ", failed finding msgid 0x" << hex << uppercase << static_cast<int>(header->msgid) << dec << " in mmitssMsgid2psidMap" << endl;
      if (logFile)
      {
        OS_log << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);              
        OS_log << ", failed finding msgid 0x" << hex << uppercase << static_cast<int>(header->msgid) << dec << " in mmitssMsgid2psidMap" << endl;
        l_logRows++;
      }  
    }  
		else
		{
			ite = appHandle.appPSIDmap.find(it->second);
			if (ite != appHandle.appPSIDmap.end() && savariHandle[ite->second].isRegistered)
			{
				savariHandle[ite->second].wmetx.tx_length = static_cast<int>(header->length);
				if (wme_wsm_tx(handler[appHandle.applications[ite->second].e_iface],&(savariHandle[ite->second].wmetx),&recvbuf[sizeof(mmitss_udp_header_t)]) < 0)
				{
          cerr << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);              
					cerr << ", failed wme_wsm_tx for " << appHandle.applications[ite->second].s_name;
          if (logFile)
          {
            OS_log << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);              
            OS_log << ", failed wme_wsm_tx for " << appHandle.applications[ite->second].s_name;
            l_logRows++;
          }
				}		
				else 
        {
          if(verbose)
          {
            cout << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);              
            cout << ", send msgid 0x" << hex << uppercase << static_cast<int>(header->msgid) << dec << ", application " << appHandle.applications[ite->second].s_name;
            cout << " to wme psid 0x" << hex << uppercase << it->second << dec << " for broadcast" << endl;
          }
          if (logDetails)
          {
            OS_log << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);              
            OS_log << ", send msgid 0x" << hex << uppercase << static_cast<int>(header->msgid) << dec << ", application " << appHandle.applications[ite->second].s_name;
            OS_log << " to wme psid 0x" << hex << uppercase << it->second << dec << " for broadcast" << endl;
            l_logRows++;
          }  
        }
			}
    }
	}
  
  /// check reopen log files
  if (logFile && fullTimeStamp.tms_minute > logfile_tms_minute + logFileInterval)
  {
    OS_log.close();
    if (l_logRows == 0)
      remove(s_logfilename.c_str());
    s_logfilename = appHandle.s_logPath + std::string("/wmetx.") + timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp) + std::string(".log");    
    OS_log.open(s_logfilename.c_str());
    l_logRows = 0;
    
    logfile_tms_minute = fullTimeStamp.tms_minute;
  }
}

void close_all(void)
{
  timeUtils::getFullTimeStamp(fullTimeStamp);
  for (size_t i = 0; i < savariHandle.size(); i++)
  {
    if (savariHandle[i].isRegistered)
    {
      if (appHandle.applications[i].e_registerType == WMEconfig::appConfigEnum_t::USER)
        wme_unregister_user(handler[appHandle.applications[i].e_iface],&(savariHandle[i].wmereq));
      else
        wme_unregister_provider(handler[appHandle.applications[i].e_iface],&(savariHandle[i].wmereq));
    }
  }
  eloop_unregister_read_sock(fd_listen);
  eloop_terminate();
  sleep(1);    
  eloop_destroy();
  wme_deinit(handler[0]);
  wme_deinit(handler[1]);  
  close(fd_listen);
  if (logFile)
  {
    OS_log << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    OS_log << ": close_all()" << endl;
    OS_log.close();
  }
  exit (0);
}
