//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************
/* wmerx.cpp
    receives over-the-air wme packets and forwards to to the MRP
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

#include "wmerx.h"
#include "wmeConf.h"
#include "socketUtils.h"
#include "wmeUtils.h"
#include "timeUtils.h"
#include "msgUtils.h"

#ifdef RSU
#include "savariWAVE_api.h"
#endif

#ifndef MAXDSRCMSGSIZE
#define MAXDSRCMSGSIZE	2000
#endif

using namespace std;

WMEconfig::appHandler_t appHandle;
std::vector<savariWmeHandler_t> savariHandle;
savari_wme_handler_t handler[2];
std::map<uint32_t,size_t> wmePSIDmap; /// key: network byte psid, value: index in applications
map<uint32_t,size_t>::const_iterator ite;
wmeUtils::wsmp_header_t wsmpHeader;

int fd_send;
bool send2earapp = false;
int fd_earSend;
std::string s_earSend = string("UDP:10.0.0.141:15030");
uint8_t sendbuf[MAXDSRCMSGSIZE];

ofstream OS_log;
string s_logfilename;
long long logFileInterval;
long long logfile_tms_minute;
unsigned long l_logRows;

timeUtils::fullTimeStamp_t fullTimeStamp;

struct savariwme_cbs wme_cbs;

bool verbose = false;
bool logFile = false;
bool logDetails = false;

void do_usage(const char* progname)
{
	cerr << progname << "Usage: " << endl;
	cerr << "\t-c wmefwd.conf" << endl;
  cerr << "\t-d turn on udp send to ear app" << endl;
	cerr << "\t-f log flag: 0 - no log; 1 - simple log; 2 - detail log" << endl;
	cerr << "\t-v turn on verbose" << endl;
	exit (1);
}

int main (int argc, char** argv)
{
	int option;
	char* f_wmefwdConfig = NULL;
  
	/// getopt
	while ((option = getopt(argc, argv, "c:f:dv")) != EOF) 
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
		case 'd':
			send2earapp = true;
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
  s_logfilename = appHandle.s_logPath + std::string("/wmerx.") + timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp) + std::string(".log");
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
      if (appHandle.applications[ite->second].e_direction == WMEconfig::appConfigEnum_t::TX)
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
      if (appHandle.applications[ite->second].e_direction == WMEconfig::appConfigEnum_t::TX)
        continue;      
			OS_log << "Application " << appHandle.applications[ite->second].s_name;
			OS_log << " psid: " << "0x" << hex << uppercase << ite->first << dec;
      OS_log << " network byte order psid: " << "0x" << hex << uppercase << wme_convert_psid_be(ite->first) << dec;
			OS_log << ", application index: " << ite->second << endl;
		}
    l_logRows++;
	}
  
	/// open eth0 send socket
	fd_send = socketUtils::client(*(appHandle.mpSendAddr),false);
	if (fd_send < 0)
	{
    cerr << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);  
		cerr << ", failed create send socket" << endl;
    if (logFile)
    {
      OS_log << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);  
      OS_log << ", failed create send socket" << endl;
      OS_log.close();
    }
		exit (1);
	}
  if (verbose)
  {
    cout << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);      
    cout << ", fd_send = " << fd_send << ", send to " << appHandle.mpSendAddr->name << ":" << appHandle.mpSendAddr->port << endl;
  }
  if (logFile)
  {
    OS_log << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);      
    OS_log << ", fd_send = " << fd_send << ", send to " << appHandle.mpSendAddr->name << ":" << appHandle.mpSendAddr->port << endl;
    l_logRows++;
  }  
  
  if (send2earapp)
  {
    socketUtils::socketAddress_t earSendAddr;
    socketUtils::setSocketAddress(s_earSend.c_str(),earSendAddr);
    fd_earSend = socketUtils::client(earSendAddr,false);
    if (fd_earSend < 0)
    {
      cerr << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);  
      cerr << ", failed create fd_earSend" << endl;
      if (logFile)
      {
        OS_log << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);  
        OS_log << ", failed create fd_earSend" << endl;
        OS_log.close();
      }
      close(fd_send);      
      exit (1);
    }
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
		savariHandle[i].wmereq.extended_access = 0xFFFF;
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
  }
  
  /// set up WME callback functions
  wme_cbs.wme_user_confirm = &savari_user_confirm;
  wme_cbs.wme_provider_confirm = &savari_provider_confirm;
	wme_cbs.wme_wsm_indication = &savari_wsm_indication; 
  
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
      close(fd_send);
      if (send2earapp)
        close(fd_earSend);
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
		close(fd_send);
    if (send2earapp)
      close(fd_earSend);
		exit(1);
	}
  
  /// register applications to WME
  int ret;
	for (size_t i = 0; i < appHandle.applications.size(); i++)
	{
    if (appHandle.applications[i].e_direction == WMEconfig::appConfigEnum_t::TX)
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
    
  /// WME eloop_register_read_sock
	for (size_t i = 0; i < appHandle.applications.size(); i++)
	{
    if (appHandle.applications[i].e_direction == WMEconfig::appConfigEnum_t::TX)
      continue;
    ret = eloop_register_read_sock(handler[appHandle.applications[i].e_iface],savari_rx,0,&(savariHandle[i].wmereq));
    if (ret < 0)
    {
      cerr << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);                
      cerr << ", failed eloop_register_read_sock for " << appHandle.applications[i].s_name << endl;
      if (logFile)
      {
        OS_log << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);                
        OS_log << ", failed eloop_register_read_sock for " << appHandle.applications[i].s_name << endl;
      }
      close_all();
    }
    if (verbose)
    {
      cout << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);                
      cout << ", eloop_register_read_sock succeed for " << appHandle.applications[i].s_name << endl;
    }
    if (logFile)
    {
      OS_log << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);                
      OS_log << ", eloop_register_read_sock succeed for " << appHandle.applications[i].s_name << endl;
      l_logRows++;
    } 
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

/// savari_rx - wait for data on the savari_wme_handler returned by the wme_init. 
/// When data is available wme_rx invokes registered callbacks (wme_user_confirm, wme_provider_confirm & savari_wsm_indication)
void savari_rx(int sock_t, void *eloop_data, void *user_data)
{
  wme_rx(sock_t,&wme_cbs,user_data);
}

/// savari_user_confirm - invoked by the WME layer to indicate the status of wme_register_user.
void savari_user_confirm(void *ctx, int conf_result_indication)
{
	struct savariwme_reg_req *wme_req = (struct savariwme_reg_req *)ctx;
  timeUtils::getFullTimeStamp(fullTimeStamp);
	ite = wmePSIDmap.find(wme_req->psid);
  if (ite == wmePSIDmap.end())
  {
    cerr << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    cerr << ", failed savari_user_confirm, can't find psid " << "0x" << hex << uppercase << wme_req->psid << dec << " in wmePSIDmap" << endl;
    if (logFile)
    {
      OS_log << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
      OS_log << ", failed savari_user_confirm, can't find psid " << "0x" << hex << uppercase << wme_req->psid << dec << " in wmePSIDmap" << endl;
    }  
    close_all();
  }
#ifdef RSU	
	if (conf_result_indication == SAVARI1609_RC_ACCEPTED)
	{
		wme_user_service_confirm(handler[appHandle.applications[ite->second].e_iface],SAVARI1609_ACTION_ADD,wme_req);
		if (verbose)
    {
      cout << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
			cout << ", wme_user_service_confirm for " << appHandle.applications[ite->second].s_name << endl;
    }
		if (logFile)
    {
      OS_log << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
			OS_log << ", wme_user_service_confirm for " << appHandle.applications[ite->second].s_name << endl;
      l_logRows++;
    }
	}
#else
	if (conf_result_indication == LIBWME_RC_ACCEPTED)
	{
		wme_user_service_confirm(handler[appHandle.applications[ite->second].e_iface],LIBWME_ACTION_ADD,wme_req);
		if (verbose)
    {
      cout << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);    
			cout << ", wme_user_service_confirm for " << appHandle.applications[ite->second].s_name << endl;
    }
		if (logFile)
    {
      OS_log << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);    
			OS_log << ", wme_user_service_confirm for " << appHandle.applications[ite->second].s_name << endl;
      l_logRows++;
    }    
	}
#endif  
}

/// savari_provider_confirm - invoked by the WME layer to indicate the status of wme_register_provider.
void savari_provider_confirm(void *ctx, int conf_result_indication)
{
	struct savariwme_reg_req *wme_req = (struct savariwme_reg_req *)ctx;
  timeUtils::getFullTimeStamp(fullTimeStamp);  
	ite = wmePSIDmap.find(wme_req->psid);
  if (ite == wmePSIDmap.end())
  {
    cerr << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);      
    cerr << ", failed savari_provider_confirm, can't find psid " << "0x" << hex << uppercase << wme_req->psid << dec << " in wmePSIDmap" << endl;
    if (logFile)
    {
      OS_log << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);      
      OS_log << ", failed savari_provider_confirm, can't find psid " << "0x" << hex << uppercase << wme_req->psid << dec << " in wmePSIDmap" << endl;
    }
    close_all();
  }
#ifdef RSU	
	if (conf_result_indication == SAVARI1609_RC_ACCEPTED)
	{
		wme_provider_service_confirm(handler[appHandle.applications[ite->second].e_iface],SAVARI1609_ACTION_ADD,wme_req);
		if (verbose)
    {
      cout << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);      
			cout << ", wme_provider_service_confirm for " << appHandle.applications[ite->second].s_name << endl;
    }
		if (logFile)
    {
      OS_log << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);      
			OS_log << ", wme_provider_service_confirm for " << appHandle.applications[ite->second].s_name << endl;
      l_logRows++;
    }    
	}
#else
	if (conf_result_indication == LIBWME_RC_ACCEPTED)
	{
		wme_provider_service_confirm(handler[appHandle.applications[ite->second].e_iface],LIBWME_ACTION_ADD,wme_req);
		if (verbose)
    {
      cout << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);          
			cout << ", wme_provider_service_confirm for " << appHandle.applications[ite->second].s_name << endl;
    }
		if (logFile)
    {
      OS_log << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);          
			OS_log << ", wme_provider_service_confirm for " << appHandle.applications[ite->second].s_name << endl;
      l_logRows++;
    }
	}
#endif  
}

/// savari_wsm_indication - invoked by the WME layer indicating the application about the WSM packet matching based on psid.
void savari_wsm_indication(void *ctx, struct savariwme_rx_indication *rxind)
{
  timeUtils::getFullTimeStamp(fullTimeStamp);    
    
  wsmpHeader.version = static_cast<uint8_t>(rxind->version);
  wsmpHeader.psidlen = static_cast<uint8_t>(wme_getpsidlen(&(rxind->psid[0])));
  if (wsmpHeader.psidlen <= PSID_LEN)
  {
    wsmpHeader.psid = 0;
    for (uint8_t i = 0; i < wsmpHeader.psidlen; i++)
    {
      wsmpHeader.psid = (wsmpHeader.psid << 8) | rxind->psid[i];
    }
    wsmpHeader.channel = static_cast<uint8_t>(rxind->channel);
    wsmpHeader.rate = static_cast<uint8_t>(rxind->datarate);
    wsmpHeader.txpower = static_cast<int8_t>(rxind->txpower);
    wsmpHeader.waveId = WAVE_ELEMID_WSMP;
    wsmpHeader.txlength = static_cast<uint16_t>(rxind->num_rx & 0xFFFF);  
    /// encode wsmp header    
    ite = appHandle.appPSIDmap.find(wsmpHeader.psid);
    if (ite != appHandle.appPSIDmap.end() && savariHandle[ite->second].isRegistered)
    {
      size_t headerLen = wmeUtils::encode_wsmp_header(&wsmpHeader,sendbuf,sizeof(sendbuf)); 
      memcpy(&sendbuf[headerLen],&rxind->rx_buf[0],rxind->num_rx);
      size_t bytesSend = static_cast<size_t>(headerLen + rxind->num_rx);    
      socketUtils::sendall(fd_send,(char*)sendbuf,bytesSend);
      if (send2earapp && wsmpHeader.psidlen == 2 && (rxind->psid[0] == 0xBF && rxind->psid[1] == 0xE0)
        || (rxind->psid[1] == 0xBF && rxind->psid[0] == 0xE0))
      {
        socketUtils::sendall(fd_earSend,(char*)&sendbuf[headerLen],(size_t)(rxind->num_rx));
      }
      if (verbose)
      {
        cout << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);              
        cout << ", sent wme to application, psid 0x" << hex << uppercase << wsmpHeader.psid << dec << endl;
      }
      if (logDetails)
      {
        OS_log << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);              
        OS_log << ", sent wme to application, psid 0x" << hex << uppercase << wsmpHeader.psid << dec << endl;
        l_logRows++;
      }  
    }
  }
  
  /// check reopen log files
  if (logFile && fullTimeStamp.tms_minute > logfile_tms_minute + logFileInterval)
  {
    OS_log.close();
    if (l_logRows == 0)
      remove(s_logfilename.c_str());
    s_logfilename = appHandle.s_logPath + std::string("/wmerx.") + timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp) + std::string(".log");    
    OS_log.open(s_logfilename.c_str());
    logfile_tms_minute = fullTimeStamp.tms_minute;
    l_logRows = 0;
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
  eloop_unregister_read_sock(handler[0]);
  eloop_unregister_read_sock(handler[1]);
  eloop_terminate();
  sleep(1);    
  eloop_destroy();
  wme_deinit(handler[0]);
  wme_deinit(handler[1]);
  close(fd_send);
  if (send2earapp)
    close(fd_earSend);
  if (logFile)
  {
    OS_log << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    OS_log << ": close_all()" << endl;
    OS_log.close();
  }
  exit (0);
}
