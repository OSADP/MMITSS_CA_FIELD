/* dataMgr.cpp - MRP_DataMgr
 * functions:
 * 1. send MAP to RSE_MessageTX and the cloud server
 * 2. receive WSMs (BSM & SRM) from RSE_MessageRX and forward the WSMs to RSE_Aware
 * 3. receive PSRM from the cloud server and foward it to RSE_Aware
 * 4. receive UDP messages from MRP_TrafficControllerInterface
 *    msgid_cntrlstatus   - encode SPaT, send to RSE_MessageTX and the cloud server
 *                        - send signal status message (msgid_signalstatus) to MRP_Aware
 *    msgid_detCnt        - log to file
 *    msgid_detPres       - log to file 
 * 5. receive UDP messages from MRP_Aware
 *    msgid_ssm           - forward to RSE_MessageTX
 *    msgid_softcall      - forward to MRP_TrafficControllerInterface
 *    msgid_troj          - log to file 
 * 6. receive poll request (msgid_pollReq) and send poll response back
 * UDP message structures are defined in timeCard.h
 * logs:
 * 1. logFile:
 *    - received SRM, PSRM
 *    - received softcall request
 *    - received vehicle trajectory
 * 2. logDetails:
 *    - logFile plus
 *    - BSM, SSM and SPaT
 *    - detector count/occupancy
 *    - detector presence
 *    - signal status
*/

#include <iostream>
#include <fstream> 
#include <sstream>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <string>
#include <cstring>
#include <bitset>  
#include <map>
#include <unistd.h>
#include <termios.h> 
#include <iomanip> 
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/poll.h>
#include <fcntl.h>

#include "wmeUtils.h"
#include "AsnJ2735Lib.h"
#include "comm.h"
#include "socketUtils.h"
#include "timeCard.h"
#include "mgrConf.h"
#include "catchSignal.h"
#include "dataMgr.h"

using namespace std;

void do_usage(const char* progname)
{
  cerr << "Usage" << progname << endl;
  cerr << "\t-n intersection name" << endl;
  cerr << "\t-s socket.conf" << endl;
  cerr << "\t-f log flag: 0 - no log; 1 - simple log; 2 - detail log" << endl;
  cerr << "\t-v turn on verbose" << endl;
  exit (1);
}

int main(int argc, char** argv) 
{
  int option;
  char* f_intName = NULL;
  char* f_socketConfig = NULL;
  volatile bool logFile = false;
  volatile bool logDetails = false;
  volatile bool verbose = false;
  
  // getopt
  while ((option = getopt(argc, argv, "s:n:f:cpv")) != EOF) 
  {
    switch(option) 
    {
    case 's':
      f_socketConfig = strdup(optarg);
      break;
    case 'n':
      f_intName = strdup(optarg);
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
  if (f_socketConfig == NULL || f_intName == NULL)
    do_usage(argv[0]);
    
  /* ----------- preparation -------------------------------------*/
  /// get timestamp
  timeUtils::fullTimeStamp_t fullTimeStamp;
  timeUtils::getFullTimeStamp(fullTimeStamp);
  
  /// instance MGRconfig::configMgr to read configuration files
  MGRconfig::configMgr s_mgrConfig(f_socketConfig);
  if (!s_mgrConfig.isConfigSucceed())
  {
    cerr << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    cerr << ", failed construct MGRconfig::configMgr";
    cerr << ", f_socketConfig = " << f_socketConfig << endl;
    delete f_socketConfig;
    exit(1);
  }
  delete f_socketConfig;

  /// store intersectionName
  string intersectionName = f_intName;
  delete f_intName;
  string timecardName = s_mgrConfig.getTimecardPath() + string("/") + intersectionName + string(".timecard");
  
  /// instance AsnJ2735Lib to read nmap
  AsnJ2735Lib asnJ2735Lib(s_mgrConfig.getNmapFileName().c_str());  
  /// get intersectionId
  uint32_t intersectionId = asnJ2735Lib.getIntersectionIdByName(intersectionName.c_str());
  if (intersectionId == 0)
  {
    cerr << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    cerr << ", failed getIntersectionIdByName: " << intersectionName << endl;
    exit (1);
  }
  /// get MAP payload
  char mapPayload[MAXDSRCMSGSIZE];
  size_t mapPayload_size = asnJ2735Lib.get_mapdata_payload(intersectionId,mapPayload,MAXDSRCMSGSIZE,false);
  if (mapPayload_size == 0)
  {
    cerr << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    cerr << ", failed get_mapdata_payload: " << intersectionId << endl;
    exit(1);    
  }
  const long long mapInterval = 1000LL; // in milliseconds
  volatile long long sentMap_tms = 0;
  
  /// open log file
  string logFilePath = s_mgrConfig.getLogPath();
  long long logFileInterval = static_cast<long long>(s_mgrConfig.getLogInterval()) * 60 * 1000LL;  
  volatile long long logfile_tms_minute = fullTimeStamp.tms_minute;
  /// err log file
  string errLogFile = logFilePath + string("/mgr.err");
  ofstream OS_ERR(errLogFile.c_str(),ios::app);
  if (!OS_ERR.is_open()) 
  {
    cerr << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    cerr << ", failed open err log" << endl;
    exit(1);
  }

  /// data types to log
  vector<string> logtypes;
  logtypes.push_back(string("mgr"));      // general log
  logtypes.push_back(string("sig"));      // log controller state data received from MRP_TCI
  logtypes.push_back(string("cnt"));      // log count/volume data received from MRP_TCI
  logtypes.push_back(string("pres"));     // log pres data received from MRP_TCI
  logtypes.push_back(string("req"));      // log softcall request data received from MRP_Aware
  logtypes.push_back(string("traj"));     // log vehicle trajectory data received from MRP_Aware
  logtypes.push_back(string("perm"));     // log performance measures
  logtypes.push_back(string("payload"));  // log wsm payload
  /// build log file list
  vector<Logfile_t> logFiles;
  for (size_t i = 0; i < logtypes.size(); i++) 
    {logFiles.push_back(Logfile_t(logFilePath,intersectionName,logtypes[i]));}
  /// open log files
  if (logFile && !openLogFiles(logFiles,fullTimeStamp.localDateTimeStamp))
  {
    OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    OS_ERR << ", failed openLogFiles" << endl;
    closeLogFiles(logFiles);
    OS_ERR.close();
    exit(1);
  }    
      
  /// open sockets
  int fd_wmeListen = socketUtils::server(s_mgrConfig.getSocketAddr(MGRconfig::socketHandleEnum_t::WMELISTEN),true);
  if (fd_wmeListen < 0)
  {
    OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    OS_ERR << ", failed create WMELISTEN socket" << endl;
    if (logFile) {closeLogFiles(logFiles);}
    OS_ERR.close();
    exit(1);
  }  
  
  int fd_wmeSend = socketUtils::client(s_mgrConfig.getSocketAddr(MGRconfig::socketHandleEnum_t::WMESEND),false);
  if (fd_wmeSend < 0)
  {
    OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    OS_ERR << ", failed create WMESEND socket" << endl;
    if (logFile) {closeLogFiles(logFiles);}
    OS_ERR.close();
    close(fd_wmeListen);
    exit(1);
  }  
  
  int fd_cloudListen = socketUtils::server(s_mgrConfig.getSocketAddr(MGRconfig::socketHandleEnum_t::CLOUDLISTEN),true);
  if (fd_cloudListen < 0)
  {
    OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    OS_ERR << ", failed create CLOUDLISTEN socket" << endl;
    if (logFile) {closeLogFiles(logFiles);}
    OS_ERR.close();
    close(fd_wmeListen);
    close(fd_wmeSend);
    exit(1);
  }  
  
  int fd_cloudSend = socketUtils::client(s_mgrConfig.getSocketAddr(MGRconfig::socketHandleEnum_t::CLOUDSEND),false);
  if (fd_cloudSend < 0)
  {
    OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    OS_ERR << ", failed create CLOUDSEND socket" << endl;
    if (logFile) {closeLogFiles(logFiles);}
    OS_ERR.close();
    close(fd_wmeListen);
    close(fd_wmeSend);
    close(fd_cloudListen);
    exit(1);
  }  
  
  int fd_tciListen = socketUtils::server(s_mgrConfig.getSocketAddr(MGRconfig::socketHandleEnum_t::TCILISTEN),true);
  if (fd_tciListen < 0)
  {
    OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    OS_ERR << ", failed create TCILISTEN socket" << endl;
    if (logFile) {closeLogFiles(logFiles);}
    OS_ERR.close();
    close(fd_wmeListen);
    close(fd_wmeSend);
    close(fd_cloudListen);
    close(fd_cloudSend);    
    exit(1);
  }  
  
  int fd_tciSend = socketUtils::client(s_mgrConfig.getSocketAddr(MGRconfig::socketHandleEnum_t::TCISEND),false);
  if (fd_tciSend < 0)
  {
    OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    OS_ERR << ", failed create TCISEND socket" << endl;
    if (logFile) {closeLogFiles(logFiles);}
    OS_ERR.close();
    close(fd_wmeListen);
    close(fd_wmeSend);
    close(fd_cloudListen);
    close(fd_cloudSend);    
    close(fd_tciListen);    
    exit(1);
  }  
  
  int fd_awareListen = socketUtils::server(s_mgrConfig.getSocketAddr(MGRconfig::socketHandleEnum_t::AWARELISTEN),true);
  if (fd_awareListen < 0)
  {
    OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    OS_ERR << ", failed create AWARELISTEN socket" << endl;
    if (logFile) {closeLogFiles(logFiles);}
    OS_ERR.close();
    close(fd_wmeListen);
    close(fd_wmeSend);
    close(fd_cloudListen);
    close(fd_cloudSend);    
    close(fd_tciListen);    
    close(fd_tciSend);    
    exit(1);
  }
  
  int fd_awareSend = socketUtils::client(s_mgrConfig.getSocketAddr(MGRconfig::socketHandleEnum_t::AWARESEND),false);
  if (fd_awareSend < 0)
  {
    OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    OS_ERR << ", failed create AWARESEND socket" << endl;
    if (logFile) {closeLogFiles(logFiles);}
    OS_ERR.close();
    close(fd_wmeListen);
    close(fd_wmeSend);
    close(fd_cloudListen);
    close(fd_cloudSend);    
    close(fd_tciListen);    
    close(fd_tciSend);    
    close(fd_awareListen);    
    exit(1);
  }
    
  int fd_obsvListen = socketUtils::server(s_mgrConfig.getSocketAddr(MGRconfig::socketHandleEnum_t::OBSLISTEN),true);
  if (fd_obsvListen < 0)
  {
    OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    OS_ERR << ", failed create OBSLISTEN socket" << endl;
    if (logFile) {closeLogFiles(logFiles);}
    OS_ERR.close();
    close(fd_wmeListen);
    close(fd_wmeSend);
    close(fd_cloudListen);
    close(fd_cloudSend);    
    close(fd_tciListen);    
    close(fd_tciSend);  
    close(fd_awareListen);    
    close(fd_awareSend);        
    exit(1);
  }
  
  int fd_obsvSend = socketUtils::client(s_mgrConfig.getSocketAddr(MGRconfig::socketHandleEnum_t::OBSSEND),false);
  if (fd_obsvSend < 0)
  {
    OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    OS_ERR << ", failed create OBSSEND socket" << endl;
    if (logFile) {closeLogFiles(logFiles);}
    OS_ERR.close();
    close(fd_wmeListen);
    close(fd_wmeSend);
    close(fd_cloudListen);
    close(fd_cloudSend);    
    close(fd_tciListen);    
    close(fd_tciSend);    
    close(fd_awareListen);    
    close(fd_awareSend);        
    close(fd_obsvListen);        
    exit(1);
  }
    
  /* ----------- intercepts signals -------------------------------------*/
  if (setjmp(exit_env) != 0) 
  {
    timeUtils::getFullTimeStamp(fullTimeStamp);
    OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    OS_ERR << ", received user termination signal, quit!" << endl;
    if (logFile) {closeLogFiles(logFiles);}
    OS_ERR.close();    
    close(fd_wmeListen);
    close(fd_wmeSend);
    close(fd_cloudListen);
    close(fd_cloudSend);    
    close(fd_tciListen);    
    close(fd_tciSend);    
    close(fd_awareListen);    
    close(fd_awareSend);
    close(fd_obsvListen);        
    close(fd_obsvSend);        
    return (0);
  }
  else
    sig_ign( sig_list, sig_hand );

  /* ----------- local variables -------------------------------------*/    
  const useconds_t usleepms = 5000;     // in microseconds  
  
  /// for performance measure
  long long perm_tms_minute = fullTimeStamp.tms_minute;
  long long perm_intv = static_cast<long long>(s_mgrConfig.getPermInterval()) * 60 * 1000LL;  // in milliseconds
  const uint8_t speedLimit[] = {25,35,25,25,25,35,25,25}; // in mph  
  intPerm_t intPerm;
  
  /// structure to hold static phase timing, flags, and control plan parameters
  timing_card_t timingcard; 
  bool iscardready = false;
  
  /// buf to hold SPaT payload
  char spatPayload[MAXDSRCMSGSIZE];
  
 /// structures to hold messages received from other MMITSS components
  det_cnt_t     detCnt;     // from MRP_TCI
  det_pres_t    detPres;    // from MRP_TCI
  cntrl_state_t cntrlState; // from MRP_TCI  
  softreq_t     softReq;    // from MRP_Aware
  veh_troj_t    vehTroj;    // from MRP_Aware
  vector< vector<vehTroj_t> > a_vehTroj;
  a_vehTroj.resize(8);  // by control phase that the trajectory is associated with
  
  /// socket recv & send buf
  char recvbuf[MAXUDPMSGSIZE];
  char sendbuf[MAXUDPMSGSIZE];
  
  /// set up sockets poll structure
  struct pollfd ufds[5];
  ufds[0].fd = fd_wmeListen;
  ufds[0].events = POLLIN;
  ufds[1].fd = fd_cloudListen;
  ufds[1].events = POLLIN;
  ufds[2].fd = fd_tciListen;
  ufds[2].events = POLLIN;
  ufds[3].fd = fd_awareListen;
  ufds[3].events = POLLIN;
  ufds[4].fd = fd_obsvListen;
  ufds[4].events = POLLIN;
  nfds_t nfds = 5;
  int pollTimeout = 50;     // in milliseconds

  int retval;
  ssize_t bytesReceived;
  bool doSpat;
  
  if (verbose)
  {
    cout << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    cout << ", waiting on udp message..." << endl;
  }

  while(1)
  {
    /// wait for events
    doSpat = false;
    retval = poll(ufds,nfds,pollTimeout);
    if (retval > 0)
    {
      timeUtils::getFullTimeStamp(fullTimeStamp);
      for (nfds_t i = 0; i < nfds; i++)
      {
        if ((ufds[i].revents & POLLIN) == POLLIN)
        {
          bytesReceived = recv(ufds[i].fd,recvbuf,sizeof(recvbuf),0);
          if (bytesReceived > 0)
          {
            switch (i)
            {
            case 0:
              /// events on fd_wmeListen: wsmp header + payload (BSM, SRM)
              /// action: replace wsmp header with mmitss header and forward to MRP_Aware
              {
                wmeUtils::wsmp_header_t wsmp;
                /// decode wsmp header to determine wsm type
                size_t offset = wmeUtils::decode_wsmp_header((const uint8_t*)&recvbuf[0],bytesReceived,&wsmp);
                if (offset > 0)
                {               
                  /// get msgid
                  uint8_t msgid = wmeUtils::getmsgiddbypsid(wmeUtils::psid2mmitssMsgidMap,wsmp.psid);
                  if (msgid == wmeUtils::msgid_bsm || msgid == wmeUtils::msgid_srm)
                  {
                    /// adjust offset and payload size (Arada includes security TLV at the begginging of a payload)
                    size_t offset_adjust = 0;
                    for (size_t j = offset; j < (size_t)bytesReceived; j++)
                    {
                      /// all dsrc payloads start with 0x30
                      if (recvbuf[j] != 0x30) {offset_adjust++;}
                      else {break;}
                    }
                    offset += offset_adjust;
                    wsmp.txlength = static_cast<uint16_t>( ((size_t)(wsmp.txlength) > offset_adjust) ? ((size_t)(wsmp.txlength) - offset_adjust) : 0 );
                    if (wsmp.txlength > 0)
                    {
                      sendMMITSSmsg(fd_awareSend,sendbuf,&recvbuf[offset],(size_t)wsmp.txlength,fullTimeStamp.ms_since_midnight_local,msgid);
                      if (logFile)
                      {
                        int fileIdx = findFileIdx(logtypes,"mgr");
                        if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
                        {
                          *(logFiles[fileIdx].OS) << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
                          *(logFiles[fileIdx].OS) << ", forward " << wmeUtils::MSGNAME[msgid - wmeUtils::msgid_bsm + 1];
                          *(logFiles[fileIdx].OS) << " to MRP_Aware" << endl;
                          logFiles[fileIdx].logrows++;
                        }
                        
                        fileIdx = findFileIdx(logtypes,"payload");
                        if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
                        {
                          *(logFiles[fileIdx].OS) << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp) << ",";
                          *(logFiles[fileIdx].OS) << wmeUtils::MSGNAME[msgid - wmeUtils::msgid_bsm + 1] << ",payload=";
                          logPayloadHex(*(logFiles[fileIdx].OS),&recvbuf[offset],(size_t)wsmp.txlength);
                          logFiles[fileIdx].logrows++;
                        }                        
                      }
                      if (verbose)
                      {
                        cout << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
                        cout << ", forward " << wmeUtils::MSGNAME[msgid - wmeUtils::msgid_bsm + 1];
                        cout << " to MRP_Aware" << endl;
                      }                      
                    }
                  }
                }
                else
                {
                  OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
                  OS_ERR << ", failed decode_wsmp_header, wsm=";
                  logPayloadHex(OS_ERR,recvbuf,(size_t)bytesReceived);
                }
              }
              break;
            case 1:
              /// event on fd_cloudListen: payload (PSRM)
              /// action: add mmitss header and forward to MRP_Aware
              {
                sendMMITSSmsg(fd_awareSend,sendbuf,recvbuf,(size_t)bytesReceived,fullTimeStamp.ms_since_midnight_local,wmeUtils::msgid_psrm);
                if (logFile)
                {
                  int fileIdx = findFileIdx(logtypes,"mgr");
                  if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
                  {
                    *(logFiles[fileIdx].OS) << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
                    *(logFiles[fileIdx].OS) << ", forward PSRM to MRP_Aware" << endl;
                    logFiles[fileIdx].logrows++;
                  }
                  
                  fileIdx = findFileIdx(logtypes,"payload");
                  if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
                  {
                    *(logFiles[fileIdx].OS) << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp) << ",PSRM,payload=";
                    logPayloadHex(*(logFiles[fileIdx].OS),recvbuf,(size_t)bytesReceived);
                    logFiles[fileIdx].logrows++;
                  }                  
                }
                if (verbose)
                {
                  cout << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
                  cout << ", forward PSRM to MRP_Aware" << endl;
                }                                      
              }
              break;
            case 2:
              /// event on fd_tciListen: msgid_detCnt, msgid_detPres, msgid_cntrlstatus
              {
                mmitss_udp_header_t* pheader = (mmitss_udp_header_t*)&recvbuf[0];
                switch(pheader->msgid)
                {
                case msgid_detCnt:
                  /// action: store in memory
                  detCnt.mssent = pheader->ms_since_midnight;
                  memcpy(&detCnt.data,&recvbuf[sizeof(mmitss_udp_header_t)],sizeof(count_data_t));
                  if (logDetails)
                  {
                    int fileIdx = findFileIdx(logtypes,"cnt");
                    if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
                    {
                      logDetCntMsg(*(logFiles[fileIdx].OS),&detCnt.data,pheader->ms_since_midnight,
                        timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp));
                      logFiles[fileIdx].logrows++; 
                    }
                  }
                  break;
                case msgid_detPres:
                  /// action: store in memory
                  detPres.mssent = pheader->ms_since_midnight;
                  memcpy(&detPres.data,&recvbuf[sizeof(mmitss_udp_header_t)],sizeof(pres_data_t));
                  if (logDetails)
                  {
                    int fileIdx = findFileIdx(logtypes,"pres");
                    if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
                    {
                      logDetPresMsg(*(logFiles[fileIdx].OS),&detPres.data,pheader->ms_since_midnight,
                        timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp));
                      logFiles[fileIdx].logrows++; 
                    }
                  }
                  break;
                case msgid_cntrlstatus:
                  /// action: set doSpat flag for encoding SPaT and send signal status message (msgid_signalstatus) to MRP_Aware
                  doSpat = true;
                  cntrlState.mssent = pheader->ms_since_midnight;
                  memcpy(&cntrlState.data,&recvbuf[sizeof(mmitss_udp_header_t)],sizeof(controller_state_t));
                  if (logDetails)
                  {
                    int fileIdx = findFileIdx(logtypes,"sig");
                    if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
                    {
                      logSigStateMsg(*(logFiles[fileIdx].OS),&cntrlState.data,pheader->ms_since_midnight,
                        timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp));
                      logFiles[fileIdx].logrows++; 
                    }
                  }
                  break;
                default:
                  break;
                } // switch
              } // case 2
              break;
            case 3:
              /// event on fd_awareListen: msgid_ssm, msgid_softcall, msgid_troj
              {
                mmitss_udp_header_t* pheader = (mmitss_udp_header_t*)&recvbuf[0];
                switch(pheader->msgid)
                {
                case wmeUtils::msgid_ssm:
                  /// action: forward to RSU_MessageTX
                  socketUtils::sendall(fd_wmeSend,recvbuf,bytesReceived);
                  if (verbose)
                  {
                    cout << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
                    cout << ", forward SSM to RSU_MessageTX" << endl;
                  }           
                  if (logDetails)
                  {                 
                    int fileIdx = findFileIdx(logtypes,"payload");
                    if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
                    {
                      *(logFiles[fileIdx].OS) << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp) << ",SSM,payload=";
                      logPayloadHex(*(logFiles[fileIdx].OS),&recvbuf[sizeof(mmitss_udp_header_t)],(size_t)(pheader->length));
                      logFiles[fileIdx].logrows++;
                    }
                  }   
                  break;
                case msgid_softcall:
                  /// action: forward to MRP_TCI
                  socketUtils::sendall(fd_tciSend,recvbuf,bytesReceived);
                  if (verbose)
                  {
                    cout << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
                    cout << ", forward softcall request to MRP_TCI" << endl;
                  }                                                                          
                  softReq.mssent = pheader->ms_since_midnight;
                  memcpy(&softReq.data,&recvbuf[sizeof(mmitss_udp_header_t)],sizeof(softcall_request_t));
                  if (logFile)
                  {
                    int fileIdx = findFileIdx(logtypes,"req");
                    if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
                    {
                      logSoftReqMsg(*(logFiles[fileIdx].OS),&softReq.data,pheader->ms_since_midnight,
                        timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp));
                      logFiles[fileIdx].logrows++; 
                    }
                  }
                  break;
                case msgid_troj:
                  /// action: store in memory
                  vehTroj.mssent = pheader->ms_since_midnight;
                  memcpy(&vehTroj.data,&recvbuf[sizeof(mmitss_udp_header_t)],sizeof(vehTroj_t));
                  if (vehTroj.data.entryControlPhase >= 1 && vehTroj.data.entryControlPhase <= 8)
                  {
                    a_vehTroj[vehTroj.data.entryControlPhase - 1].push_back(vehTroj.data);
                  }
                  if (logFile)
                  {
                    int fileIdx = findFileIdx(logtypes,"traj");
                    if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
                    {
                      logVehTroj(*(logFiles[fileIdx].OS),&vehTroj.data,pheader->ms_since_midnight,
                        timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp));
                      logFiles[fileIdx].logrows++; 
                    }
                  }
                  break;
                default:
                  break;
                } // switch
              } // case 3
              break;
            case 4:
              /// event on fd_obsvListen: msgid_pollReq
              break;
            default:
              break;
            } // swicth 
          } // if (bytesReceived > 0)
        } // if POLLIN
      } // for
    } // if retval 
    
    /// get timestamp
    timeUtils::getFullTimeStamp(fullTimeStamp);      
    
    /// encoding SPaT
    if (doSpat)
    {
      if (!iscardready)
      {
        if (verbose)
        {
          cout << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
          cout << ", read timecard: " << timecardName << endl;
        }                                                                                  
        if  (!readTimeCard(timecardName.c_str(),timingcard))
        {
          OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
          OS_ERR << ", failed reading timecard " << timecardName;
          OS_ERR << ", quit!" << endl;
          if (logFile) {closeLogFiles(logFiles);}
          OS_ERR.close();    
          close(fd_wmeListen);
          close(fd_wmeSend);
          close(fd_cloudListen);
          close(fd_cloudSend);    
          close(fd_tciListen);    
          close(fd_tciSend);    
          close(fd_awareListen);    
          close(fd_awareSend);
          close(fd_obsvListen);        
          close(fd_obsvSend);        
          exit(1);          
        }
        iscardready = true;
        if (verbose)
        {
          cout << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
          cout << ", read timecard succeed" << endl;
        }                                                                                  
        string tmpstr = timecardName + string(".check");
        logTimeCard(tmpstr.c_str(),timingcard);
      }
      
      /// encode SPaT
      SPAT_element_t SPAT_element;
      formSPaT(SPAT_element,cntrlState.data,timingcard,intersectionId);
      
      /// encode SPaT
      ssize_t spatPayload_size = asnJ2735Lib.encode_spat_payload(&SPAT_element,spatPayload,MAXDSRCMSGSIZE,false);
      if (spatPayload_size > 0)
      {
        /// send to wmetx
        sendMMITSSmsg(fd_wmeSend,sendbuf,spatPayload,(size_t)spatPayload_size,fullTimeStamp.ms_since_midnight_local,wmeUtils::msgid_spat);
        /// send to cloud server
        send2cloud(fd_cloudSend,sendbuf,spatPayload,(size_t)spatPayload_size,fullTimeStamp.localDateTimeStamp.timeStamp,intersectionId,savari_cloud_spat);
        /// form msgid_signalstatus message and send to MRP_Aware
        sendSignalStateMsg(fd_awareSend,sendbuf,cntrlState.data.sigState,spatPayload,(size_t)spatPayload_size,
          fullTimeStamp.ms_since_midnight_local,msgid_signalstatus);
        if (verbose)
        {
          cout << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
          cout << ", sent SPaT" << endl;
        }                                                   
        /// log
        if (logDetails)
        {
          int fileIdx = findFileIdx(logtypes,"mgr");
          if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
          {
            *(logFiles[fileIdx].OS) << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
            *(logFiles[fileIdx].OS) << ", send SPaT" << endl;
            logFiles[fileIdx].logrows++;
          }
          
          fileIdx = findFileIdx(logtypes,"payload");
          if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
          {
            *(logFiles[fileIdx].OS) << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp) << ",SPaT,payload=";
            logPayloadHex(*(logFiles[fileIdx].OS),spatPayload,(size_t)spatPayload_size);
            logFiles[fileIdx].logrows++;
          }
        }        
      }
      else
      {
        OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
        OS_ERR << ", failed encode_spat_payload" << endl;
      }
    }
    
    /// check sending MAP
    if (fullTimeStamp.tms > sentMap_tms + mapInterval)
    {
      /// send to wmetx
      sendMMITSSmsg(fd_wmeSend,sendbuf,mapPayload,mapPayload_size,fullTimeStamp.ms_since_midnight_local,wmeUtils::msgid_map);
      /// send to cloud server
      send2cloud(fd_cloudSend,sendbuf,mapPayload,mapPayload_size,fullTimeStamp.localDateTimeStamp.timeStamp,intersectionId,savari_cloud_map);
      /// reset sentMap_tms
      sentMap_tms = fullTimeStamp.tms;
      if (verbose)
      {
        cout << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
        cout << ", sent MAP" << endl;
      }                                                                                                
      /// log
      if (logDetails)
      {
        int fileIdx = findFileIdx(logtypes,"mgr");
        if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
        {
          *(logFiles[fileIdx].OS) << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
          *(logFiles[fileIdx].OS) << ", send MAP" << endl;
          logFiles[fileIdx].logrows++;
        }
      }              
    }
    
    /// check calculate performance measures
    if (fullTimeStamp.tms_minute > perm_tms_minute + perm_intv)
    {
      formPerm(intPerm,a_vehTroj,fullTimeStamp.localDateTimeStamp.timeStamp,intersectionId,cntrlState.data.sigState.permitted_phases,&speedLimit[0]);
      if (logFile)
      {
        int fileIdx = findFileIdx(logtypes,"perm");
        if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
        {
          logPermMsg(*(logFiles[fileIdx].OS),&intPerm,fullTimeStamp.ms_since_midnight_local);
          logFiles[fileIdx].logrows++;           
        }
      }                    
      perm_tms_minute = fullTimeStamp.tms_minute;
    }
    
    /// check reopen log files
    if (logFile && fullTimeStamp.tms_minute > logfile_tms_minute + logFileInterval)
    {
      reOpenLogFiles(logFiles,fullTimeStamp.localDateTimeStamp);     
      logfile_tms_minute = fullTimeStamp.tms_minute;
    }
    
    /// sleep
    usleep(usleepms);

  } // while  
}

bool sendMMITSSmsg(int fd,char* buf,const char* msg,const size_t msglen,const uint32_t ms,const uint8_t msgid)
{
  /// mmitss msg: mmitss header + msg
  mmitss_udp_header_t header;
  header.msgheader = msg_header;
  header.msgid = msgid;
  header.ms_since_midnight = ms;
  header.length = static_cast<uint16_t>(msglen);
  memcpy(buf,&header,sizeof(mmitss_udp_header_t));
  memcpy(buf+sizeof(mmitss_udp_header_t),msg,msglen);  
  return (socketUtils::sendall(fd,buf,sizeof(mmitss_udp_header_t) + msglen)); 
}

bool sendSignalStateMsg(int fd,char* buf,const signal_state_t& sigState,
  const char* payload,size_t payloadlen,const uint32_t ms,const uint8_t msgid)
{
  /// mmitss header + sigState + payload
  mmitss_udp_header_t header;
  header.msgheader = msg_header;
  header.msgid = msgid;
  header.ms_since_midnight = ms;
  header.length = static_cast<uint16_t>(payloadlen);
  memcpy(buf,&header,sizeof(mmitss_udp_header_t));
  memcpy(buf+sizeof(mmitss_udp_header_t),&sigState,sizeof(signal_state_t));
  memcpy(buf+sizeof(mmitss_udp_header_t)+sizeof(signal_state_t),payload,payloadlen);
  return (socketUtils::sendall(fd,buf,sizeof(mmitss_udp_header_t)+sizeof(signal_state_t)+payloadlen)); 
}

bool send2cloud(int fd,char* buf,const char* msg,const size_t msglen,const timeUtils::timeStamp_t& ts,const uint32_t intID,const uint8_t msgtype)
{
  /// savari header + msg
  savari_udp_header_t header;
  header.type = msgtype;
  header.len = static_cast<uint32_t>(msglen);
  header.seconds = static_cast<uint32_t>(ts.hour * 3600 + ts.min * 60 + ts.sec);
  header.msecs = ts.millisec;
  header.intersectionID = intID;
  memcpy(buf,&header,sizeof(savari_udp_header_t));
  memcpy(buf+sizeof(savari_udp_header_t),msg,msglen);  
  return (socketUtils::sendall(fd,buf,sizeof(savari_udp_header_t) + msglen));   
}

void formPerm(intPerm_t& intPerm,std::vector< std::vector<vehTroj_t> >& a_vehTroj,const timeUtils::timeStamp_t& timeStamp,
  const uint32_t intID,const uint8_t permitted_phases,const uint8_t* pSpeedLimits)
{
  intPerm.hr = static_cast<uint8_t>(timeStamp.hour);
  intPerm.min = static_cast<uint8_t>(timeStamp.min);
  intPerm.sec = static_cast<uint8_t>(timeStamp.sec);
  intPerm.millisec = timeStamp.millisec;
  intPerm.intId = intID;
  intPerm.permitted_phases = permitted_phases;
  for (int i = 0; i < 8; i++)
  {
    memset(&intPerm.apchPerm[i],0,sizeof(apchPerm_t));
    intPerm.apchPerm[i].sampleNums = static_cast<uint16_t>(a_vehTroj[i].size());
    if (intPerm.apchPerm[i].sampleNums > 0)
    {
      double freeflow_v = (double)pSpeedLimits[i] * 0.447;   // m/s
      vector<double> sample_tt(intPerm.apchPerm[i].sampleNums);
      vector<double> sample_daly(intPerm.apchPerm[i].sampleNums);
      double sum_tt = 0;
      double sum_delay = 0;
      uint32_t stopnums = 0;
      for (uint16_t j = 0; j < intPerm.apchPerm[i].sampleNums; j++)
      {
        double freeflow_tt = (double)a_vehTroj[i][j].ingressLen / freeflow_v; // decisecond
        sample_tt[j] = (double)( (a_vehTroj[i][j].distTraveled > 0) ? 
          ((double)a_vehTroj[i][j].ingressLen * a_vehTroj[i][j].timeTraveled / a_vehTroj[i][j].distTraveled ) : 0 );
        sample_daly[j] = ((sample_tt[j] > freeflow_tt) ? (sample_tt[j] - freeflow_tt) : 0);
        sum_tt += sample_tt[j];
        sum_delay += sample_daly[j];
        if (a_vehTroj[i][j].stoppedTime >= 50)   // 5 seconds
          stopnums++;
      }
      double tt_avg = sum_tt / intPerm.apchPerm[i].sampleNums;
      double delay_avg = sum_delay / intPerm.apchPerm[i].sampleNums;
      double sum_tt_std = 0;
      double sum_delay_std = 0;
      for (uint16_t j = 0; j < intPerm.apchPerm[i].sampleNums; j++)
      {
        sum_tt_std += (sample_tt[j] - tt_avg) * (sample_tt[j] - tt_avg);
        sum_delay_std += (sample_daly[j] - delay_avg) * (sample_daly[j] - delay_avg);
      }
      intPerm.apchPerm[i].tt_avg = static_cast<uint16_t>(tt_avg);
      intPerm.apchPerm[i].tt_std = static_cast<uint16_t>(sqrt(sum_tt_std / intPerm.apchPerm[i].sampleNums));
      intPerm.apchPerm[i].delay_avg = static_cast<uint16_t>(delay_avg);
      intPerm.apchPerm[i].delay_std = static_cast<uint16_t>(sqrt(sum_delay_std / intPerm.apchPerm[i].sampleNums));
      intPerm.apchPerm[i].stoppedSamples = static_cast<uint16_t>(stopnums);
      /// clear records
      a_vehTroj[i].clear();
    }
  }
}  
  
uint32_t getNextLight(uint32_t cur_light)
{
  switch(cur_light)
  {
  case BALL_GREEN:
    return BALL_YELLOW;
    break;
  case BALL_YELLOW:
    return BALL_RED;
    break;
  case BALL_RED:
    return BALL_GREEN;
    break;
  case BALL_FLASHING:
    return BALL_FLASHING;
    break;
  case LEFTARROW_GREEN:
    return LEFTARROW_YEWLLOW;
    break;
  case LEFTARROW_YEWLLOW:
    return LEFTARROW_RED;
    break;
  case LEFTARROW_RED:
    return LEFTARROW_GREEN;
    break;
  case LEFTARROW_FLASHING:
    return LEFTARROW_FLASHING;
    break;
  case RIGHTARROW_GREEN:
    return RIGHTARROW_YEWLLOW;
    break;
  case RIGHTARROW_YEWLLOW:
    return RIGHTARROW_RED;
    break;
  case RIGHTARROW_RED:
    return RIGHTARROW_GREEN;
    break;
  case RIGHTARROW_FLASHING:
    return RIGHTARROW_FLASHING;
    break;
  case STRAIGHTARROW_GREEN:
    return STRAIGHTARROW_YEWLLOW;
    break;
  case STRAIGHTARROW_YEWLLOW:
    return STRAIGHTARROW_RED;
    break;
  case STRAIGHTARROW_RED:
    return STRAIGHTARROW_GREEN;
    break;
  case STRAIGHTARROW_FLASHING:
    return STRAIGHTARROW_FLASHING;
    break;
  case UTURNARROW_GREEN:
    return UTURNARROW_YEWLLOW;
    break;
  case UTURNARROW_YEWLLOW:
    return UTURNARROW_RED;
    break;
  case UTURNARROW_RED:
    return UTURNARROW_GREEN;
    break;
  default:
    return UTURNARROW_FLASHING;
  }
}

uint32_t getNextPedSign(uint32_t cur_sign)
{
  switch(cur_sign)
  {
  case CROSS_WALK:
    return CROSS_FLASH_DONOT_WALK;
    break;
  case CROSS_FLASH_DONOT_WALK:
    return CROSS_DONT_WALK;
    break;
  case CROSS_DONT_WALK:
    return CROSS_WALK;
    break;
  default:
    return CROSS_FLASHING;
    break;
  }
}

void formSPaT(SPAT_element_t& SPAT_element,const controller_state_t& cntrState,const timing_card_t& timingcard,const uint32_t intID)
{
  SPAT_element.id = intID;
  SPAT_element.permittedPhases = cntrState.sigState.permitted_phases;
  SPAT_element.permittedPedPhases = cntrState.sigState.permitted_ped_phases;
  SPAT_element.status = cntrState.status;
  SPAT_element.timeStamp = cntrState.timeStamp;
  bitset<8> permitted_phases(cntrState.sigState.permitted_phases);
  bitset<8> permitted_ped_phases(cntrState.sigState.permitted_phases);
  switch(cntrState.sigState.cntlrMode)
  {
  case (uint8_t)(operation_mode_enum_t::FLASHING):  
    {
      for (uint8_t i = 0; i < 8; i++)
      {
        /// phaseState
        if (permitted_phases.test(i))
        {
          SPAT_element.phaseState[i].currState = BALL_FLASHING;
          if (i % 2 == 0 && permitted_phases.test(i+1))
            SPAT_element.phaseState[i].currState = LEFTARROW_FLASHING;
          SPAT_element.phaseState[i].timeToChange = 0;
          SPAT_element.phaseState[i].stateConfidence = SPAT_StateConfidence_minTime;
          SPAT_element.phaseState[i].nextState = getNextLight(SPAT_element.phaseState[i].currState);          
          SPAT_element.phaseState[i].clearanceIntv = static_cast<uint16_t>(timingcard.phasetiming[i].yellow_interval);
          SPAT_element.phaseState[i].yellStateConfidence = SPAT_StateConfidence_minTime;          
        }
        /// pedPhaseState
        if (permitted_ped_phases.test(i))
        {
          SPAT_element.pedPhaseState[i].currState = CROSS_FLASHING;
          SPAT_element.pedPhaseState[i].timeToChange = 0;
          SPAT_element.pedPhaseState[i].stateConfidence = SPAT_StateConfidence_minTime;
          SPAT_element.pedPhaseState[i].nextState = getNextPedSign(SPAT_element.pedPhaseState[i].currState);
          SPAT_element.pedPhaseState[i].clearanceIntv = static_cast<uint16_t>(timingcard.phasetiming[i].walk_clearance * 10);
          SPAT_element.pedPhaseState[i].yellStateConfidence = SPAT_StateConfidence_minTime;          
        }
      }    
    }
    break;
  default:
    /// free, coordination, 
    {
      for (uint8_t i = 0; i < 8; i++)
      {
        /// phaseState
        if (permitted_phases.test(i))
        {
          switch(cntrState.phaseState[i].state)
          {
          case operation_mode_enum_t::GREEN:
            SPAT_element.phaseState[i].currState = BALL_GREEN;
            if (i % 2 == 0 && permitted_phases.test(i+1))
              SPAT_element.phaseState[i].currState = LEFTARROW_GREEN;
            SPAT_element.phaseState[i].timeToChange = cntrState.phaseState[i].time2next.bound_L;
            SPAT_element.phaseState[i].stateConfidence = cntrState.phaseState[i].time2next.confidence;                
            break;
          case operation_mode_enum_t::YELLOW:
            SPAT_element.phaseState[i].currState = BALL_YELLOW;
            if (i % 2 == 0 && permitted_phases.test(i+1))
              SPAT_element.phaseState[i].currState = LEFTARROW_YEWLLOW;
            SPAT_element.phaseState[i].timeToChange = cntrState.phaseState[i].time2next.bound_L;
            SPAT_element.phaseState[i].stateConfidence = cntrState.phaseState[i].time2next.confidence;
            break;
          default:
            SPAT_element.phaseState[i].currState = BALL_RED;
            if (i % 2 == 0 && permitted_phases.test(i+1))
              SPAT_element.phaseState[i].currState = LEFTARROW_RED;
            if (cntrState.phaseState[i].call_status != operation_mode_enum_t::NOCALL 
              || cntrState.phaseState[i].recall_status != operation_mode_enum_t::NORECALL)
            {
              SPAT_element.phaseState[i].timeToChange = cntrState.phaseState[i].time2next.bound_U;
              SPAT_element.phaseState[i].stateConfidence = SPAT_StateConfidence_maxTime;                
            }
            else
            {
              SPAT_element.phaseState[i].timeToChange = cntrState.phaseState[i].time2next.bound_L;
              SPAT_element.phaseState[i].stateConfidence = cntrState.phaseState[i].time2next.confidence;                
            }
            break;
          }
          SPAT_element.phaseState[i].nextState = getNextLight(SPAT_element.phaseState[i].currState);
          SPAT_element.phaseState[i].clearanceIntv = static_cast<uint16_t>(timingcard.phasetiming[i].yellow_interval);
          SPAT_element.phaseState[i].yellStateConfidence = SPAT_StateConfidence_minTime;
        }
        /// pedPhaseState
        if (permitted_ped_phases.test(i))
        {
          switch(cntrState.phaseState[i].pedstate)
          {
          case operation_mode_enum_t::WALK:
            SPAT_element.pedPhaseState[i].currState = CROSS_WALK;
            SPAT_element.pedPhaseState[i].timeToChange = cntrState.phaseState[i].pedtime2next.bound_L;
            SPAT_element.pedPhaseState[i].stateConfidence = cntrState.phaseState[i].pedtime2next.confidence;              
            break;
          case operation_mode_enum_t::FLASH_DONOT_WALK:
            SPAT_element.pedPhaseState[i].currState = CROSS_FLASH_DONOT_WALK;
            SPAT_element.pedPhaseState[i].timeToChange = cntrState.phaseState[i].pedtime2next.bound_L;
            SPAT_element.pedPhaseState[i].stateConfidence = cntrState.phaseState[i].pedtime2next.confidence;
            break;
          default:
            SPAT_element.pedPhaseState[i].currState = CROSS_DONT_WALK;
            if (cntrState.phaseState[i].recall_status == operation_mode_enum_t::PEDRECALL 
              || cntrState.phaseState[i].call_status == operation_mode_enum_t::CALLPED) 
            {    
              SPAT_element.pedPhaseState[i].timeToChange = cntrState.phaseState[i].pedtime2next.bound_U;
              SPAT_element.pedPhaseState[i].stateConfidence = SPAT_StateConfidence_maxTime;
            }
            else
            {
              SPAT_element.pedPhaseState[i].timeToChange = cntrState.phaseState[i].pedtime2next.bound_L;
              SPAT_element.pedPhaseState[i].stateConfidence = cntrState.phaseState[i].pedtime2next.confidence;                
            }              
            break;
          }
          SPAT_element.pedPhaseState[i].nextState = getNextPedSign(SPAT_element.pedPhaseState[i].currState);
          SPAT_element.pedPhaseState[i].clearanceIntv = static_cast<uint16_t>(timingcard.phasetiming[i].walk_clearance * 10);
          SPAT_element.pedPhaseState[i].yellStateConfidence = SPAT_StateConfidence_minTime;          
        }
      }      
    }
  }
}