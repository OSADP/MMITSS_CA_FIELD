//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************
/* mrpAware.cpp
 * functions:
 * 1. receive BSM (msgid_bsm), SRM (msgid_srm), PSRM (msgid_psrm) from MRP_DataMgr, decode messages
 * 2. receive signal status messages (msgid_signalstatus) from MRP_DataMgr
 * 3. track BSMs on the map and identifies those are on an approach at the intersection
 * 4. track SRM and associates with BSM
 * 5. manage BSM, SRM & PSRM, and send soft-call request to MRP_DataMgr
 * 6. send SSM (msgid_ssm) to MRP_DataMgr
 * 7. send vehicle trjoectory data (msgid_troj) to MRP_DataMgr
 * UDP message structures are defined in msgUtils.h
 * logs:
 * 1. display log for debugging purpose
 * 2. logFile:
 *    - received SRM, PSRM
 *    - vehicle trojectory
 *    - softcall request
 *    - SSM
 * 3. logDetails:
 *    - logFile plus
 *    - received BSM
 *  Softcall logic:
 *    1. ped phase call: received from Savari cloud, send it to TCI if the calling ped phase has not already been called (ped_call)
 *    2. vehicle phase call: generated here based on BSM. 
 *      each cv can make a phase call once per intersection approach, when:
 *        1) vehicle is on an approach to the intersection
 *        2) its speed is greater than softcallSpeed (2 m/s)
 *        3) its control phase is not in green
 *        4) its control phase has not been called (veh_call)
 *        5) its time-to-intersection is less than maxTime2goPhaseCall (20 sec)  
 *      soft vehicle phase call is determined at a time combining needs from all vehicles, so multiple phases can be called with one softcall.
 *      softcall on a particular phase can not be faster than vehPhaseCallInterval (1 sec)
 *    3. vehicle phase extension call:  generated here based on BSM. 
 *      each cv can make a phase extension call once per intersection approach, when:
 *        1) signal is running free or under coordination
 *        2) controller is not serving a priority treatment 
 *        3) vehicle is on an approach to the intersection
 *        4) its speed is greater than softcallSpeed (2 m/s)
 *        5) its control phase is in green (not a coordinated phase when under coordination)
 *        6) its control phase is not serving an ongoing phase extension 
 *        7) the vehicle is expected to arrive within maxTime2phaseExt (5 sec) after the remaing green
 *        8) control phase green is about to be terminated (within maxTime2changePhaseExt = 4 sec)
 *      soft vehicle phase extension call is determined at a time combining needs from all vehicles, so concurrent phases can be requested with one softcall,
 *      and call on one phase could cover multiple vehicles.
 *      vehicle phase extension request is cancel when:
 *        1) the requested phase green has been terminated, or
 *        2) vehicles contributed to the phase extension request have passed the stopbar
 *    4. priority call:  generated here based on SRM
 *      each priority-eligible cv can make a priority request once per intersection approach, when:
 *        1) signal is under coordination
 *        2) controller is not serving a priority treatment 
 *        3) vehicle is on an approach to the intersection
 *        4) its speed is greater than softcallSpeed (2 m/s)
 *        5) its control phase is a coordinated phase
 *      Early green
 *        EG 6) its control phase is in yellow and the vehicle can't pass during the first half of yellow, or
 *        EG 7) its control phase is in red and the vehicle is expected to arrive at the stopbar within red
 *      Green extension
 *        GE 6) its control phase is in green and the vehicle is expecting to arrive within maxGreenExtenstion after the remaining green
 *        GE 7) synch phase green is about to be terminated (with maxTime2changePhaseExt)
 *      priority request is determined at a time combining needs from all vehicles, green extension has higher priority than early green
 *      priority request is cancel when:
 *        GE 1) the requested phase green has been terminated
 *        EG 1) the requested phase has turned green, or
 *        2) vehicle contributed to the priority request has passed the stopbar
*/

#include <iostream>
#include <fstream> 
#include <sstream>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <cstring>
#include <unistd.h>

#include "wmeUtils.h"
#include "AsnJ2735Lib.h"
#include "comm.h"
#include "socketUtils.h"
#include "awrConf.h"
#include "catchSignal.h"
#include "mrpAware.h"

using namespace std;

void do_usage(const char* progname)
{
  cout << "Usage" << progname << endl;
  cout << "\t-n intersection name" << endl;
  cout << "\t-s socket.config" << endl;
  cerr << "\t-f log flag: 0 - no log; 1 - simple log; 2 - detail log" << endl;
  exit (1);
}

int main(int argc, char** argv) 
{
  int option;
  char* f_intName = NULL;
  char* f_socketConfig = NULL;
  volatile bool logFile = false;
  volatile bool logDetails = false;
  
  // getopt
  while ((option = getopt(argc, argv, "s:n:f:")) != EOF) 
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
  
  /// instance AWRconfig::configAwr to read configuration files  
  AWRconfig::configAwr s_awrConfig(f_socketConfig);
  if (!s_awrConfig.isConfigSucceed())
  {
    cerr << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    cerr << ", failed construct AWRconfig::configAwr";
    cerr << ", f_socketConfig = " << f_socketConfig << endl;
    delete f_socketConfig;
    exit(1);
  }
  delete f_socketConfig;

  /// store intersectionName
  string intersectionName = f_intName;
  delete f_intName;
  string timecardName = s_awrConfig.getTimecardPath() + string("/") + intersectionName + string(".timecard");

  /// instance AsnJ2735Lib to read nmap
  AsnJ2735Lib asnJ2735Lib(s_awrConfig.getNmapFileName().c_str()); 
  /// get intersectionId
  uint32_t intersectionId = asnJ2735Lib.getIntersectionIdByName(intersectionName.c_str());
  if (intersectionId == 0)
  {
    cerr << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    cerr << ", failed getIntersectionIdByName: " << intersectionName << endl;
    exit (1);
  }
  /// get index of this intersection
  int8_t intersectionIndex = static_cast<int8_t>(asnJ2735Lib.getIndexByIntersectionId(intersectionId));
  if (intersectionIndex < 0)
  {
    cerr << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);    
    cerr << ", failed getIndexByIntersectionId: " << intersectionId << endl;
    exit (1);
  }
  
  /// log files
  string logFilePath = s_awrConfig.getLogPath();
  long long logFileInterval = static_cast<long long>(s_awrConfig.getLogInterval()) * 60 * 1000LL;  
  volatile long long logfile_tms_minute = fullTimeStamp.tms_minute;
  /// err log file
  string errLogFile = logFilePath + string("/awr.err");
  ofstream OS_ERR(errLogFile.c_str(),ios::app);
  if (!OS_ERR.is_open()) 
  {
    cerr << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    cerr << ", failed open err log" << endl;
    exit(1);
  }
  /// display log file (always on for debugging purpose, keep logFileInterval records)
  string s_DisplayLogFileName = logFilePath + std::string("/display.log");
  ofstream OS_Display(s_DisplayLogFileName.c_str());
  volatile long long displayfile_tms_minute = fullTimeStamp.tms_minute;
  if (!OS_Display.is_open())
  {
    OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    OS_ERR << ", failed open display log file" << endl;
    OS_ERR.close();
    exit(1);
  }
  
  /// data types to log
  vector<string> logtypes;
  logtypes.push_back(string("awr"));      // general log
  logtypes.push_back(string("veh"));      // log tracked bsm locationAware & signalAware
  logtypes.push_back(string("req"));      // log softcall request send to MRP_DataMgr
  logtypes.push_back(string("ssm"));      // log priority request server status (ssm send to MRP_DataMgr)
  logtypes.push_back(string("traj"));      // log vehicle trajectory data received from MRP_Aware
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
    OS_Display.close();
    exit(1);
  }    
  
  /// open sockets
  int fd_Listen = socketUtils::server(s_awrConfig.getSocketAddr(AWRconfig::socketHandleEnum_t::MGRLISTEN),true);
  if (fd_Listen < 0)
  {
    OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    OS_ERR << ", failed create fd_Listen socket" << endl;
    OS_Display << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    OS_Display << ", failed create fd_Listen socket" << endl;
    if (logFile) {closeLogFiles(logFiles);}
    OS_ERR.close();
    OS_Display.close();    
    exit(1);
  }
  
  int fd_Send = socketUtils::client(s_awrConfig.getSocketAddr(AWRconfig::socketHandleEnum_t::MGRSEND),false);
  if (fd_Send < 0)
  {
    OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    OS_ERR << ", failed create fd_Send socket" << endl;
    OS_Display << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    OS_Display << ", failed create fd_Send socket" << endl;
    if (logFile) {closeLogFiles(logFiles);}
    OS_ERR.close();
    OS_Display.close();    
    close(fd_Listen);    
    exit(1);
  }
    
  /* ----------- intercepts signals -------------------------------------*/
  if (setjmp(exit_env) != 0) 
  {
    timeUtils::getFullTimeStamp(fullTimeStamp);
    OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    OS_ERR << ", received user termination signal, quit!" << endl;
    OS_Display << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    OS_Display << ", received user termination signal, quit!" << endl;
    if (logFile) {closeLogFiles(logFiles);}
    OS_ERR.close();    
    OS_Display.close();
    close(fd_Listen);
    close(fd_Send);
    return (0);
  }
  else
    sig_ign( sig_list, sig_hand );
    
  /* ----------- local variables -------------------------------------*/
  /// control parameters
  const useconds_t usleepms = 5000;     // in microseconds
  const double  stopSpeed = 2;         // in m/s
  const double  stopDist = 5.0;         // in meters, no need to re-do mapping when speed is lower than stopSpeed & distance travelled is smaller than stopDist 
  const double  softcallSpeed = 2.0;    // in m/s, no softcall if vehicle's speed is lower than softcallSpeed 
  const long long ssmInterval = 1000LL; // in milliseconds
  const long long vehPhaseCallInterval = 1000LL;  // in milliseconds
  long long timeOutms = static_cast<long long>(s_awrConfig.getDsrcTimeOut()) * 1000LL;                    // in milliseconds
  uint16_t maxTime2goPhaseCall = static_cast<uint16_t>(s_awrConfig.getMaxTime2goPhaseCall() * 10);        // in deciseconds (200)
  uint16_t maxTime2changePhaseExt = static_cast<uint16_t>(s_awrConfig.getMaxTime2changePhaseExt() * 10);  // in deciseconds (40)
  uint16_t maxTime2phaseExt = static_cast<uint16_t>(s_awrConfig.getMaxTime2phaseExt() * 10);              // in deciseconds (50)
  uint16_t maxGreenExtenstion = static_cast<uint16_t>(s_awrConfig.getMaxGreenExtenstion() * 10);          // in deciseconds (100)
  
  /// structure to hold static phase timing, flags, and control plan parameters
  timing_card_t timingcard; 

  /// socket recv & send buf
  char recvbuf[MAXUDPMSGSIZE];
  char sendbuf[MAXUDPMSGSIZE];
  ssize_t bytesReceived;
      
  /// maps to store latest inbound dsrc messages (BSM, SRM, SPaT)
  map<uint32_t,BSM_List_t>  bsmList;    // key vehId
	map<uint32_t,SRM_List_t>	srmList;		// key requested intersectionId
	map<uint32_t,SPAT_List_t> spatList;		// key intersectionId
  
  /// structures to hold decoded dsrc messages
  BSM_List_t bsmIn;
  SRM_List_t srmIn;
	SPAT_List_t spatIn;
  /// decoded SPaT of this intersection (payload inside msgid_signalstatus messages sending from MRP_DataMgr)
  SPAT_element_t ownSpat;
    
  /// for tracking vehicle on map, locationAware, signalAware, phase call/extension status
  map<uint32_t,cvStatusAware_t> vehList;  // key vehId
  map<uint32_t,cvStatusAware_t>::iterator itVehList;
  cvStatusAware_t cvStatusAware;      
  GeoUtils::connectedVehicle_t cvIn;  
  GeoUtils::connectedVehicle_t cv;
  
  /// for tracking MMITSS signal control status data at this intersection
  mmitssSignalControlStatus_t signalControlStatus;
  signalControlStatus.reset();
  map<uint32_t,priorityRequestTableEntry_t>::iterator itPrsList;
  map<uint32_t,uint8_t>::iterator itPhaseExtVidMap;
  priorityRequestTableEntry_t prsTableEntry;
  
  /// for tracking signal cycle count 
	uint32_t prevSyncPhaseState = MsgEnum::phasestate_enum_t::UNKNOWN;
  
  /// softcall request phases
  bitset<NEMAPHASES> phases2call;
  bitset<NEMAPHASES> pedphases2call;
  
  /// buf to hold SSM payload
  char ssmPayload[MAXDSRCMSGSIZE];
	SSM_element_t ssm2send; 
  
  softcall_request_t softReq;
  vehTroj_t vehTroj;
  uint32_t trojCnt = 0;
  
  bool processBSM;  // if ture, tracking BSM on the MAP
  bool processSRM;  // if ture, performing priority request server functions
    
  while(1)
  {
    processBSM = false;
    processSRM = false;
    
    /// receive udp message (non-blocking)
    bytesReceived = recv(fd_Listen,recvbuf,sizeof(recvbuf),0);   
    if (bytesReceived > 0)
    {
      /// allmessages include mmitss header
      timeUtils::getFullTimeStamp(fullTimeStamp);
      mmitss_udp_header_t* pheader = (mmitss_udp_header_t*)&recvbuf[0];
      switch(pheader->msgid)
      {
      case wmeUtils::msgid_bsm:
        /// action decode BSM, set processBSM flag (vehicle tracking), updateBsmList
        {
          if (asnJ2735Lib.decode_bsm_payload(&recvbuf[sizeof(mmitss_udp_header_t)],(size_t)(pheader->length),&bsmIn.bsmMsg,NULL))
          {
            processBSM = true;
            bsmIn.tms = fullTimeStamp.tms;
            updateBsmList(bsmList,bsmIn);
            /// log
            if (logFile)
            {
              int fileIdx = findFileIdx(logtypes,"awr");
              if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
              {
                *(logFiles[fileIdx].OS) << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
                *(logFiles[fileIdx].OS) << ", received BSM from vehicle " << bsmIn.bsmMsg.bolb1_element.id;
                *(logFiles[fileIdx].OS) << ", msgCnt " << static_cast<int>(bsmIn.bsmMsg.bolb1_element.msgCnt) << endl;
                logFiles[fileIdx].logrows++;
              }
            }
          }
          else
          {
            OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
            OS_ERR << ", failed decode_bsm_payload, payload=";
            logPayloadHex(OS_ERR,&recvbuf[sizeof(mmitss_udp_header_t)],(size_t)(pheader->length));
            OS_Display << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
            OS_Display << ", failed decode_bsm_payload" << endl;          
          }
        }
        break;        
      case wmeUtils::msgid_psrm:
        /// pedestrian phase request (PSRM)
        /// action: set pedphases2call
        {
          SRM_element_t pedSrm;
          if (asnJ2735Lib.decode_srm_payload(&recvbuf[sizeof(mmitss_udp_header_t)],(size_t)(pheader->length),&pedSrm,NULL))
          {
            /// validate PSRM
/// PSRM id sometimes is wrong, disable id checking            
            pedSrm.signalRequest_element.id = intersectionId;            
            if (pedSrm.signalRequest_element.id == intersectionId)
            {
              uint8_t pedRequestPhase = asnJ2735Lib.getControlPhaseByLaneId(pedSrm.signalRequest_element.id,pedSrm.signalRequest_element.inLaneId);            
              if (pedRequestPhase >= 1 && pedRequestPhase <= NEMAPHASES)
              {
                pedphases2call.set(pedRequestPhase-1);
                /// log
                OS_Display << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
                OS_Display << ", received PSRM on phase " << static_cast<int>(pedRequestPhase) << endl;
                if (logFile)
                {
                  int fileIdx = findFileIdx(logtypes,"awr");
                  if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
                  {
                    *(logFiles[fileIdx].OS) << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
                    *(logFiles[fileIdx].OS) << ", received PSRM on phase " << static_cast<int>(pedRequestPhase) << endl;
                    logFiles[fileIdx].logrows++;
                  }
                }
              }
            }
          }
          else
          {
            OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
            OS_ERR << ", failed ped decode_srm_payload, payload=";
            logPayloadHex(OS_ERR,&recvbuf[sizeof(mmitss_udp_header_t)],(size_t)(pheader->length));
            OS_Display << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
            OS_Display << ", failed ped decode_srm_payload" << endl;
          }        
        }
        break;
      case wmeUtils::msgid_srm:  
        /// action: decode SRM, set processSRM flag (priority request server), updateSrmList
        {
          srmIn.srmMsg.reset();
          if (asnJ2735Lib.decode_srm_payload(&recvbuf[sizeof(mmitss_udp_header_t)],(size_t)(pheader->length),&srmIn.srmMsg,NULL))
          {
            /// validate SRM
            if (srmIn.srmMsg.signalRequest_element.id == intersectionId)
            {
              processSRM = true;
              asnJ2735Lib.decode_bsmblob1_payload((const u_char*)srmIn.srmMsg.BSMblob,&srmIn.srmBlob1);
              srmIn.tms = fullTimeStamp.tms;
              srmIn.isCancel = false;
              if (srmIn.srmMsg.signalRequest_element.requestedAction == MsgEnum::priorityrequest_enum_t::CANCELPRIORITY 
                || srmIn.srmMsg.signalRequest_element.requestedAction == MsgEnum::priorityrequest_enum_t::CANCELPREEMP)
              {
                srmIn.isCancel = true;
              }         
              srmIn.requestedPhase = asnJ2735Lib.getControlPhaseByLaneId(srmIn.srmMsg.signalRequest_element.id,
                srmIn.srmMsg.signalRequest_element.inLaneId);
              srmIn.requestedServiceStartTms = fullTimeStamp.tms_midnight + srmIn.srmMsg.timeOfService.sec      ///DE_DSecond in unit of milliseconds
                + (srmIn.srmMsg.timeOfService.hour * 3600LL + srmIn.srmMsg.timeOfService.min * 60LL) * 1000LL;
              srmIn.requestedServiceEndTms = fullTimeStamp.tms_midnight + srmIn.srmMsg.endOfService.sec
                + (srmIn.srmMsg.endOfService.hour * 3600LL + srmIn.srmMsg.endOfService.min * 60LL) * 1000LL;          
              updateSrmList(srmList,srmIn); 
              /// log
              if (logFile)
              {
                int fileIdx = findFileIdx(logtypes,"awr");
                if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
                {
                  *(logFiles[fileIdx].OS) << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
                  *(logFiles[fileIdx].OS) << ", received SRM from vehicle " << srmIn.srmBlob1.id;
                  *(logFiles[fileIdx].OS) << ", msgCnt " << static_cast<int>(srmIn.srmMsg.msgCnt);
                  *(logFiles[fileIdx].OS) << ", on phase " << static_cast<int>(srmIn.requestedPhase);
                  *(logFiles[fileIdx].OS) << ", action " << static_cast<int>(srmIn.srmMsg.signalRequest_element.requestedAction) << endl;
                  logFiles[fileIdx].logrows++;
                }
              }              
            }
          }
          else
          {
            OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
            OS_ERR << ", failed decode_srm_payload, payload=";
            logPayloadHex(OS_ERR,&recvbuf[sizeof(mmitss_udp_header_t)],(size_t)(pheader->length));
            OS_Display << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
            OS_Display << ", failed decode_srm_payload" << endl;
          }                
        }
        break;
      case wmeUtils::msgid_spat: 
        /// action: decode SPaT, updateSpatList
        {
          if (asnJ2735Lib.decode_spat_payload(&recvbuf[sizeof(mmitss_udp_header_t)],(size_t)(pheader->length),&spatIn.spatMsg,NULL))
          {
            spatIn.tms = fullTimeStamp.tms;
            updateSpatList(spatList,spatIn);
            /// log
            if (logDetails)
            {
              int fileIdx = findFileIdx(logtypes,"awr");
              if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
              {
                *(logFiles[fileIdx].OS) << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
                *(logFiles[fileIdx].OS) << ", received SPaT from intersection " << asnJ2735Lib.getIntersectionNameById(spatIn.spatMsg.id) << endl;
                logFiles[fileIdx].logrows++;
              }
            }
          }
          else
          {
            OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
            OS_ERR << ", failed decode_spat_payload, payload=";
            logPayloadHex(OS_ERR,&recvbuf[sizeof(mmitss_udp_header_t)],(size_t)(pheader->length));
            OS_Display << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
            OS_Display << ", failed decode_spat_payload" << endl;
          }
        }
        break;
      case msgid_signalstatus:
        /// action: read timecard if has not been read, decode SPaT payload inside the message, and updateSignalState
        {
          if (!signalControlStatus.isPlantimingReady)
          {
            OS_Display << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
            OS_Display << ", read timecard: " << timecardName << endl;
            if  (!readTimeCard(timecardName.c_str(),timingcard))
            {
              OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
              OS_ERR << ", failed reading timecard " << timecardName;
              OS_ERR << ", quit!" << endl;
              OS_Display << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
              OS_Display << ", failed reading timecard " << timecardName;
              OS_Display << ", quit!" << endl;
              if (logFile) {closeLogFiles(logFiles);}
              OS_ERR.close();    
              OS_Display.close();
              close(fd_Listen);
              close(fd_Send);
              exit(1);          
            }
            signalControlStatus.isPlantimingReady = true;
            OS_Display << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
            OS_Display << ", read timecard succeed" << endl;
          }
          /// decode ownSpat
          if (asnJ2735Lib.decode_spat_payload(&recvbuf[sizeof(mmitss_udp_header_t)+sizeof(signal_state_t)],
            (size_t)(pheader->length),&ownSpat,NULL))
          {
            spatIn.tms = fullTimeStamp.tms;
            memcpy(&(spatIn.spatMsg),&ownSpat,sizeof(SPAT_element_t));									
            updateSpatList(spatList,spatIn);
          }
          else
          {
            OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
            OS_ERR << ", failed own decode_spat_payload, payload=";
            logPayloadHex(OS_ERR,&recvbuf[sizeof(mmitss_udp_header_t)+sizeof(signal_state_t)],(size_t)(pheader->length));   
            OS_Display << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
            OS_Display << ", failed decode own spat" << endl;          
          }
          /// update signal status
          signal_state_t* psigState = (signal_state_t*)&recvbuf[sizeof(mmitss_udp_header_t)];        
          if (!updateSignalState(signalControlStatus,psigState,timingcard))
          {
            OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
            OS_ERR << ", failed updateSignalState, quit!" << endl;
            OS_Display << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
            OS_Display << ", failed updateSignalState, quit!" << endl;
            if (logFile) {closeLogFiles(logFiles);}
            OS_ERR.close();    
            OS_Display.close();
            close(fd_Listen);
            close(fd_Send);
            exit(1);                    
          }
          /// update cycleCtn
          if (prevSyncPhaseState == MsgEnum::phasestate_enum_t::UNKNOWN)
            prevSyncPhaseState = ownSpat.phaseState[signalControlStatus.synch_phase-1].currState;
          if (ownSpat.phaseState[signalControlStatus.synch_phase-1].currState != prevSyncPhaseState)
          {
            if (ownSpat.phaseState[signalControlStatus.synch_phase-1].currState == MsgEnum::phasestate_enum_t::YELLOW)
            {
              signalControlStatus.cycleCtn = (uint8_t)((signalControlStatus.cycleCtn + 1) % 128);
              OS_Display << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);              
              OS_Display << " cycleCtn = " << static_cast<int>(signalControlStatus.cycleCtn) << endl;
            }
            prevSyncPhaseState = ownSpat.phaseState[signalControlStatus.synch_phase-1].currState;
          }
        }
        break;
      default:
        break;
      } // switch
    } // if (bytesReceived > 0)
    
    if (!signalControlStatus.isPlantimingReady)
    {
      usleep(usleepms);
      continue;
    }
    
    /// get timestamp
    timeUtils::getFullTimeStamp(fullTimeStamp);      
    
    if (pedphases2call.any())  
    {
      softReq.callphase = static_cast<uint8_t>(pedphases2call.to_ulong());
      softReq.callobj = static_cast<uint8_t>(softcall_enum_t::PED);
      softReq.calltype = static_cast<uint8_t>(softcall_enum_t::CALL);
      sendSoftReq(fd_Send,sendbuf,softReq,fullTimeStamp.ms_since_midnight_local,msgid_softcall);
      OS_Display << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp); 
      OS_Display << ", call ped phase " << static_cast<int>(softReq.callphase) << endl;
      if (logFile)
      {
        int fileIdx = findFileIdx(logtypes,"awr");
        if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
        {
          *(logFiles[fileIdx].OS) << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
          *(logFiles[fileIdx].OS) << ", call ped phase " << static_cast<int>(softReq.callphase) << endl;
          logFiles[fileIdx].logrows++;
        }
        
        fileIdx = findFileIdx(logtypes,"req");
        if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
        {
          logSoftReqMsg(*(logFiles[fileIdx].OS),&softReq,fullTimeStamp.ms_since_midnight_local);
          logFiles[fileIdx].logrows++; 
        }
      }
      pedphases2call.reset();      
    }    
    
    if (processBSM)
    {
      /// get cvIn geo location from bsmIn (id,ts,geoPoint,motionState)
      cvIn.reset();           
      cvIn.id = bsmIn.bsmMsg.bolb1_element.id;
      cvIn.ts = fullTimeStamp.tms;
      cvIn.geoPoint.latitude = bsmIn.bsmMsg.bolb1_element.lat;
      cvIn.geoPoint.longitude = bsmIn.bsmMsg.bolb1_element.lon;
      cvIn.geoPoint.elevation = bsmIn.bsmMsg.bolb1_element.elev;
      cvIn.motionState.speed = bsmIn.bsmMsg.bolb1_element.speed;
      cvIn.motionState.heading = bsmIn.bsmMsg.bolb1_element.heading;
      /// get last cvStatusAware (map mapping result, priority request and soft-call status)
      itVehList = vehList.find(cvIn.id);
      if (itVehList != vehList.end())
      {
        // get cvStatus from vehList
        cvStatusAware = itVehList->second;
      }
      else
      {
        // add to vehList
        cvStatusAware.reset();
        cvStatusAware.cvStatus.push_back(cvIn);
        vehList[cvIn.id] = cvStatusAware;
      }
      /// get latest map matching result
      cv = cvStatusAware.cvStatus.back();
      /// determine whether it's an outdated BSM (it's UDP)         
      bool proceed = (cvIn.ts < cv.ts) ? false : true;
      /// process new BSM
      if (proceed)
      {
        /// get distance travelled from the last geoPoint
        double distTravelled = GeoUtils::distlla2lla(cvIn.geoPoint,cv.geoPoint);
        /// check whether need to locate the vehicle on map
        bool doMapping = (cvIn.motionState.speed  < stopSpeed && fabs(distTravelled) < stopDist) ? false : true;
        /// update cvIn geo location (id, ts, geoPoint & motionState) to cv
        cv.id = cvIn.id;
        cv.ts = cvIn.ts;
        /// keep same geoPoint & motionState if not doing mapping
        if(doMapping)
        {
          cv.geoPoint = cvIn.geoPoint;
          cv.motionState = cvIn.motionState;
        }         
        /// do mapping to get current cv mapping result (isVehicleInMap & vehicleTrackingState)
        if (!doMapping)
        {
          // keep the same mapping info. 
          // Note that cv.ts has been updated but cv.geoPoint & cv.motionState remained as the last record,
          //  to ensure working for the case that vehicle is moving in geoPoint but speed is low (seen this on OBU pcap file)
          cvIn.isVehicleInMap = cv.isVehicleInMap;
          cvIn.vehicleTrackingState = cv.vehicleTrackingState;
          cvIn.vehicleLocationAware = cv.vehicleLocationAware;
        }
        else
        {
          // call locateVehicleInMap
          //  cvIn & cv have the same ts, geoPoint & motionState but different isVehicleInMap & vehicleTrackingState
          //  cv consonants the last mapping result & cvIn is going to get the current mapping result
          cvIn.isVehicleInMap = asnJ2735Lib.locateVehicleInMap(cv,cvIn.vehicleTrackingState);   
          // convert mapping result cvIn to locationAware & signalAware
          asnJ2735Lib.updateLocationAware(cvIn.vehicleTrackingState,cvIn.vehicleLocationAware);
        }
        updateSignalAware(spatList,cvIn.vehicleLocationAware,cvIn.vehicleSignalAware);
        
        /// update vehList with the current mapping result      
        if (cvStatusAware.isOnApproach) 
        {
          /// previous on an ingress of this intersection, possible current vehicleIntersectionStatus:
          ///   case 1: on the same ingress (ON_APPROACH or moved into AT_INTERSECTION_BOX)
          if (cvIn.isVehicleInMap // in one of the intersection map 
            && cvIn.vehicleTrackingState.intsectionTrackingState.intersectionIndex == intersectionIndex // at this intersection
            && (cvIn.vehicleTrackingState.intsectionTrackingState.vehicleIntersectionStatus == GeoUtils::vehicleInMap_enum_t::ON_APPROACH
              || cvIn.vehicleTrackingState.intsectionTrackingState.vehicleIntersectionStatus == GeoUtils::vehicleInMap_enum_t::AT_INTERSECTION_BOX)
            && cvIn.vehicleTrackingState.intsectionTrackingState.approachIndex == cv.vehicleTrackingState.intsectionTrackingState.approachIndex) // on the same ingress
          {
            cvStatusAware.cvStatus.push_back(cvIn); // add tracking point cvIn 
            vehList[cvIn.id] = cvStatusAware;
          }
          /// case 2: still at this intersection but changed approach (moves to egress)
          else if (cvIn.isVehicleInMap // in one of the intersection map
            && cvIn.vehicleTrackingState.intsectionTrackingState.intersectionIndex == intersectionIndex // at this intersection
            && cvIn.vehicleTrackingState.intsectionTrackingState.vehicleIntersectionStatus == GeoUtils::vehicleInMap_enum_t::ON_EGRESS
            && cvIn.vehicleTrackingState.intsectionTrackingState.approachIndex != cv.vehicleTrackingState.intsectionTrackingState.approachIndex) // changed approach
          {
            /// log tracked points
            trojCnt++;
            if (cvStatusAware.cvStatus.size() >= 10)
            {
              uint32_t laneLen = asnJ2735Lib.getLaneLength(intersectionId,cvStatusAware.cvStatus[0].vehicleLocationAware.laneId);
              formTroj(vehTroj,cvStatusAware,laneLen,trojCnt,stopSpeed);
              sendVehTroj(fd_Send,sendbuf,vehTroj,fullTimeStamp.ms_since_midnight_local,msgid_troj);
              if (logFile)
              {
                int fileIdx = findFileIdx(logtypes,"veh");
                if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
                {
                  logVehTrack(*(logFiles[fileIdx].OS),cvStatusAware,trojCnt);
                  logFiles[fileIdx].logrows++;
                }
                
                fileIdx = findFileIdx(logtypes,"traj");
                if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
                {
                  logVehTroj(*(logFiles[fileIdx].OS),&vehTroj,fullTimeStamp.ms_since_midnight_local);
                  logFiles[fileIdx].logrows++;
                }
              }
            }
            cvStatusAware.isOnApproach = false;
            cvStatusAware.cvStatus.clear();
            cvStatusAware.cvStatus.push_back(cvIn);
            vehList[cvIn.id] = cvStatusAware;
            OS_Display << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);                      
            OS_Display << " vehicle " << cvIn.id << " entered egress lane ";
            OS_Display << static_cast<int>(cvIn.vehicleLocationAware.laneId) << endl;
          }
          /// case 3: changed intersection (moved to ingress of the next intersection)
          else if (cvIn.isVehicleInMap // in one of the intersection map
            && cvIn.vehicleTrackingState.intsectionTrackingState.intersectionIndex != intersectionIndex) // changed intersection
          {
            /// log tracked points
            trojCnt++;
            if (cvStatusAware.cvStatus.size() >= 10)
            {
              uint32_t laneLen = asnJ2735Lib.getLaneLength(intersectionId,cvStatusAware.cvStatus[0].vehicleLocationAware.laneId);
              formTroj(vehTroj,cvStatusAware,laneLen,trojCnt,stopSpeed);
              sendVehTroj(fd_Send,sendbuf,vehTroj,fullTimeStamp.ms_since_midnight_local,msgid_troj);
              if (logFile)
              {
                int fileIdx = findFileIdx(logtypes,"veh");
                if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
                {
                  logVehTrack(*(logFiles[fileIdx].OS),cvStatusAware,trojCnt);
                  logFiles[fileIdx].logrows++;
                }
                
                fileIdx = findFileIdx(logtypes,"traj");
                if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
                {
                  logVehTroj(*(logFiles[fileIdx].OS),&vehTroj,fullTimeStamp.ms_since_midnight_local);
                  logFiles[fileIdx].logrows++;
                }
              }              
            }
            cvStatusAware.isOnApproach = false;
            cvStatusAware.cvStatus.clear();
            cvStatusAware.cvStatus.push_back(cvIn);
            vehList[cvIn.id] = cvStatusAware;
            OS_Display << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);                      
            OS_Display << " vehicle " << cvIn.id << " entered intersection ";
            OS_Display << asnJ2735Lib.getIntersectionNameByIndex(cvIn.vehicleTrackingState.intsectionTrackingState.intersectionIndex);
            OS_Display << " on lane " << static_cast<int>(cvIn.vehicleLocationAware.laneId);
            OS_Display << ", phase " << static_cast<int>(cvIn.vehicleLocationAware.controlPhase) << endl;
          }
          /// case 4: NOT_IN_MAP (moved outside of the map)
          else if (!cvIn.isVehicleInMap)
          {
            trojCnt++;
            /// log tracked points
            if (cvStatusAware.cvStatus.size() >= 10)
            {
              uint32_t laneLen = asnJ2735Lib.getLaneLength(intersectionId,cvStatusAware.cvStatus[0].vehicleLocationAware.laneId);
              formTroj(vehTroj,cvStatusAware,laneLen,trojCnt,stopSpeed);
              sendVehTroj(fd_Send,sendbuf,vehTroj,fullTimeStamp.ms_since_midnight_local,msgid_troj);
              if (logFile)
              {
                int fileIdx = findFileIdx(logtypes,"veh");
                if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
                {
                  logVehTrack(*(logFiles[fileIdx].OS),cvStatusAware,trojCnt);
                  logFiles[fileIdx].logrows++;
                }
                
                fileIdx = findFileIdx(logtypes,"traj");
                if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
                {
                  logVehTroj(*(logFiles[fileIdx].OS),&vehTroj,fullTimeStamp.ms_since_midnight_local);
                  logFiles[fileIdx].logrows++;
                }
              }              
            }  
            cvStatusAware.isOnApproach = false;
            cvStatusAware.cvStatus.clear();
            cvStatusAware.cvStatus.push_back(cvIn);
            vehList[cvIn.id] = cvStatusAware;
            OS_Display << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);                      
            OS_Display << " vehicle " << cvIn.id << " left map area" << endl;
          }
        }
        /// previous not on any ingress of this intersection, current on an ingress of this intersection
        else if (cvIn.isVehicleInMap // in one of the intersection map
          && cvIn.vehicleTrackingState.intsectionTrackingState.intersectionIndex == intersectionIndex // at this intersection           
          && cvIn.vehicleTrackingState.intsectionTrackingState.vehicleIntersectionStatus == GeoUtils::vehicleInMap_enum_t::ON_APPROACH) // on an ingress of this intersection
        {
          // vehicle first enters an approach of this intersection
          cvStatusAware.isOnApproach = true;
          cvStatusAware.cvStatus.clear();
          cvStatusAware.cvStatus.push_back(cvIn);
          vehList[cvIn.id] = cvStatusAware;
          OS_Display << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);                      
          OS_Display << " vehicle " << cvIn.id << " entered ingress lane ";
          OS_Display << static_cast<int>(cvIn.vehicleLocationAware.laneId) << ", phase ";
          OS_Display << static_cast<int>(cvIn.vehicleLocationAware.controlPhase) << ", dist2go ";
          OS_Display << cvIn.vehicleLocationAware.dist2go.distLong << endl;
        }
        /// both previous and current are not on any ingress of this intersection. 
        else
        {
          cvStatusAware.cvStatus.clear();
          cvStatusAware.cvStatus.push_back(cvIn);
          vehList[cvIn.id] = cvStatusAware;
        }
      }
    }
    
    if (processSRM)
    {
      // srmIn maintains the latest SRM data
      itPrsList = signalControlStatus.prsList.find(srmIn.srmBlob1.id);
      if (itPrsList == signalControlStatus.prsList.end())
      {
        // this is a new SRM
        prsTableEntry.requestVehId = srmIn.srmBlob1.id;
        prsTableEntry.msgCnt = srmIn.srmMsg.msgCnt;
        prsTableEntry.requestUpdateStatus = priorityRequestServer_enum_t::INITIATE;
        prsTableEntry.initiatedTms = srmIn.tms;
        prsTableEntry.receivedTms = prsTableEntry.initiatedTms;
        prsTableEntry.requestedServiceStartTms = srmIn.requestedServiceStartTms;
        prsTableEntry.requestedServiceEndTms = srmIn.requestedServiceEndTms;
        prsTableEntry.requestPhase = srmIn.requestedPhase;
        prsTableEntry.requestedAction = srmIn.srmMsg.signalRequest_element.requestedAction;
        prsTableEntry.requestInLaneId = srmIn.srmMsg.signalRequest_element.inLaneId;
        prsTableEntry.requestOutLaneId = srmIn.srmMsg.signalRequest_element.outLaneId;
        prsTableEntry.requestVehicleClassType = srmIn.srmMsg.signalRequest_element.NTCIPVehicleclass.NTCIPvehicleClass_type;
        prsTableEntry.requestVehicleClassLevel = srmIn.srmMsg.signalRequest_element.NTCIPVehicleclass.NTCIPvehicleClass_level;
        prsTableEntry.requestVehicleTransitStatus = srmIn.srmMsg.transitStatus;
        memcpy(&(prsTableEntry.requestTimeOfService),&(srmIn.srmMsg.timeOfService),sizeof(DTime_element_t));
        memcpy(&(prsTableEntry.requestEndOfService),&(srmIn.srmMsg.endOfService),sizeof(DTime_element_t));
        // determine requestStatus
        itVehList = vehList.find(prsTableEntry.requestVehId);         
        if (itVehList == vehList.end() 						// not on vehList (no BSM)
          || !(itVehList->second.isOnApproach)		// not on approach of this intersection
          || srmIn.isCancel												// cancel of nothing
          || srmIn.srmMsg.signalRequest_element.requestedAction != MsgEnum::priorityrequest_enum_t::REQUESTPRIORITY) // not request priority 
        {
          prsTableEntry.requestStatus = MsgEnum::priorityrequest_enum_t::NOTVALID;
        }
        else if (!signalControlStatus.coordinatedPhases.test(prsTableEntry.requestPhase - 1))			// request on non-coordinated phase
          {prsTableEntry.requestStatus = MsgEnum::priorityrequest_enum_t::REJECTED;}
        else if (itVehList->second.cvStatus.back().vehicleSignalAware.currState == MsgEnum::phasestate_enum_t::GREEN // request phase in green
          && getTime2goDeci(itVehList->second.cvStatus.back().vehicleLocationAware.dist2go.distLong,itVehList->second.cvStatus.back().motionState.speed)
            < itVehList->second.cvStatus.back().vehicleSignalAware.timeToChange)	// can pass in green
        {
          prsTableEntry.requestStatus = MsgEnum::priorityrequest_enum_t::NOTNEEDED;
        }    
        else
          prsTableEntry.requestStatus = MsgEnum::priorityrequest_enum_t::QUEUED;
        // add to prsList
        signalControlStatus.prsList[srmIn.srmBlob1.id] = prsTableEntry;
        if(!srmIn.isCancel)
        {
          /// first SRM from this vehicle
          OS_Display << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);                      
          OS_Display << " received SRM request from vehicle " << srmIn.srmBlob1.id << " on phase " << static_cast<int>(srmIn.requestedPhase);
          OS_Display << ", requestedAction " <<  static_cast<int>(srmIn.srmMsg.signalRequest_element.requestedAction);
          OS_Display << ", requestStatus " << static_cast<int>(prsTableEntry.requestStatus);
          OS_Display << ", adding SRM to PRS list" << endl;
        }
      }
      else if (itPrsList->second.msgCnt == srmIn.srmMsg.msgCnt)
      {
        itPrsList->second.requestUpdateStatus = priorityRequestServer_enum_t::MAINTAIN;
      }
      else
      {
        // this is an update of existing SRM
        itPrsList->second.msgCnt = srmIn.srmMsg.msgCnt;
        itPrsList->second.requestUpdateStatus = priorityRequestServer_enum_t::UPDATE;
        itPrsList->second.receivedTms = srmIn.tms;
        itPrsList->second.requestedServiceStartTms = srmIn.requestedServiceStartTms;
        itPrsList->second.requestedServiceEndTms = srmIn.requestedServiceEndTms;
        itPrsList->second.requestPhase = srmIn.requestedPhase;
        itPrsList->second.requestedAction = srmIn.srmMsg.signalRequest_element.requestedAction;
        itPrsList->second.requestInLaneId = srmIn.srmMsg.signalRequest_element.inLaneId;
        itPrsList->second.requestOutLaneId = srmIn.srmMsg.signalRequest_element.outLaneId;
        itPrsList->second.requestVehicleClassType = srmIn.srmMsg.signalRequest_element.NTCIPVehicleclass.NTCIPvehicleClass_type;
        itPrsList->second.requestVehicleClassLevel = srmIn.srmMsg.signalRequest_element.NTCIPVehicleclass.NTCIPvehicleClass_level;
        itPrsList->second.requestVehicleTransitStatus = srmIn.srmMsg.transitStatus;
        memcpy(&(itPrsList->second.requestTimeOfService),&(srmIn.srmMsg.timeOfService),sizeof(DTime_element_t));
        memcpy(&(itPrsList->second.requestEndOfService),&(srmIn.srmMsg.endOfService),sizeof(DTime_element_t));
        // determine requestStatus
        MsgEnum::priorityrequest_enum_t::requeststatus prevRequestStatus = itPrsList->second.requestStatus;
        itVehList = vehList.find(itPrsList->second.requestVehId);
        switch(prevRequestStatus)
        {
        case MsgEnum::priorityrequest_enum_t::ACTIVE:
          // it was the cause of the ongoing signal priority treatment, possible next requestStatus: COMPLETED or CANCELLED
          if (itVehList == vehList.end() || !(itVehList->second.isOnApproach))	// moved out from the approach
            itPrsList->second.requestStatus = MsgEnum::priorityrequest_enum_t::COMPLETED;
          else if (srmIn.isCancel)			
          {
            // vehicle requested cancel
            itPrsList->second.requestStatus = MsgEnum::priorityrequest_enum_t::CANCELLED;
            /// last SRM of this vehicle
            OS_Display << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);                      
            OS_Display << " received cancel SRM request from vehicle " << srmIn.srmBlob1.id << endl;
          }  
          break;
        default:
          // for other cases, next requestStatus is determined the same as INITIATE state
          if (itVehList == vehList.end() 						// not on vehList
            || !(itVehList->second.isOnApproach)		// not on approach of this intersection
            || srmIn.isCancel												// cancel of nothing
            || srmIn.srmMsg.signalRequest_element.requestedAction != MsgEnum::priorityrequest_enum_t::REQUESTPRIORITY) // not request priority 
          {
            itPrsList->second.requestStatus = MsgEnum::priorityrequest_enum_t::NOTVALID;
          }
          else if (!signalControlStatus.coordinatedPhases.test(itPrsList->second.requestPhase - 1))			// request on non-coordinated phase
            itPrsList->second.requestStatus = MsgEnum::priorityrequest_enum_t::REJECTED;
          else if (itVehList->second.cvStatus.back().vehicleSignalAware.currState == MsgEnum::phasestate_enum_t::GREEN // request phase in green
            && getTime2goDeci(itVehList->second.cvStatus.back().vehicleLocationAware.dist2go.distLong,itVehList->second.cvStatus.back().motionState.speed) 
              < itVehList->second.cvStatus.back().vehicleSignalAware.timeToChange)	// can pass in green
          {
            itPrsList->second.requestStatus = MsgEnum::priorityrequest_enum_t::NOTNEEDED;
          }
          else
            itPrsList->second.requestStatus = MsgEnum::priorityrequest_enum_t::QUEUED;
        }
      }
    }
        
    /// check signal priority call: check cancel first
    phases2call.reset();
    if (signalControlStatus.prsAction != priorityRequestServer_enum_t::NONE)
    {
      /// get SPaT info for this intersection   
      spatIn = getSpatFromList(spatList,intersectionId);
      if (spatIn.tms > 0)
      {
        /// get vehicle info for the priority vehicle
        itVehList = vehList.find(signalControlStatus.priorityVehicleId);
        if ((signalControlStatus.prsAction == priorityRequestServer_enum_t::EARLYGREEN
              && spatIn.spatMsg.phaseState[signalControlStatus.priorityPhaseId - 1].currState == MsgEnum::phasestate_enum_t::GREEN)   // phase turned green
          || (signalControlStatus.prsAction == priorityRequestServer_enum_t::GREENEXTENSION
              && spatIn.spatMsg.phaseState[signalControlStatus.priorityPhaseId - 1].currState != MsgEnum::phasestate_enum_t::GREEN) // green expired
          || (itVehList == vehList.end() || !itVehList->second.isOnApproach)) // priority cause vehicle passed
        {
          phases2call.set(signalControlStatus.priorityPhaseId - 1,1);      
          signalControlStatus.prsAction = priorityRequestServer_enum_t::NONE;
          signalControlStatus.priorityPhaseId = 0;
          signalControlStatus.priorityVehicleId = 0;
          itPrsList = signalControlStatus.prsList.find(itVehList->first);
          if (itPrsList != signalControlStatus.prsList.end())
          {
            itPrsList->second.requestStatus = MsgEnum::priorityrequest_enum_t::COMPLETED;
          }
        }
      }
    }
    if (phases2call.any())
    {
      /// send cancel priority call
      timeUtils::getFullTimeStamp(fullTimeStamp);   
      softReq.callphase = static_cast<uint8_t>(phases2call.to_ulong());
      softReq.callobj = static_cast<uint8_t>(softcall_enum_t::PRIORITY);
      softReq.calltype = static_cast<uint8_t>(softcall_enum_t::CANCEL);
      sendSoftReq(fd_Send,sendbuf,softReq,fullTimeStamp.ms_since_midnight_local,msgid_softcall);
      OS_Display << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp); 
      OS_Display << ", cancel priority on phase " << static_cast<int>(softReq.callphase) << endl;
      if (logFile)
      {
        int fileIdx = findFileIdx(logtypes,"awr");
        if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
        {
          *(logFiles[fileIdx].OS) << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
          *(logFiles[fileIdx].OS) << ", cancel priority on phase " << static_cast<int>(softReq.callphase) << endl;
          logFiles[fileIdx].logrows++;
        }
        
        fileIdx = findFileIdx(logtypes,"req");
        if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
        {
          logSoftReqMsg(*(logFiles[fileIdx].OS),&softReq,fullTimeStamp.ms_since_midnight_local);
          logFiles[fileIdx].logrows++; 
        }
      }
      phases2call.reset();
    }      
        
    /// check signal priority call: check whether need to initiate priority treatment
    ///   Note that no new treatment will be initiated if the signal is in an active priority treatment,
    ///   in the field, there is additional constrain of how many cycles per priority treatment, which is 
    ///   not considered here as that needs local cycle clock info
    phases2call.reset();
    softcall_enum_t::callType priorityCall = softcall_enum_t::UNKNOWN;
    if (signalControlStatus.prsAction == priorityRequestServer_enum_t::NONE      // signal is not serving a priority
      && signalControlStatus.cntlrMode == operation_mode_enum_t::COORDINATION    // signal is under coordinated control
      && signalControlStatus.cycleCtn != signalControlStatus.priorityCycleCnt)   // at most one priority request per signal cycle
    {
      /// get SPaT info for this intersection   
      spatIn = getSpatFromList(spatList,intersectionId);
      if (spatIn.tms > 0)
      {
        timeUtils::getFullTimeStamp(fullTimeStamp);
        for (itPrsList = signalControlStatus.prsList.begin(); itPrsList != signalControlStatus.prsList.end();++itPrsList)
        {
          if (itPrsList->second.requestStatus != MsgEnum::priorityrequest_enum_t::QUEUED)
            continue;
          itVehList = vehList.find(itPrsList->first);
          if (itVehList == vehList.end() || !(itVehList->second.isOnApproach))
            continue;
//          if (itVehList->second.cvStatus.back().motionState.speed < softcallSpeed)
//            continue;    // not initiate priority request for stopped vehicle
          uint16_t time2go = getTime2goDeci(itVehList->second.cvStatus.back().vehicleLocationAware.dist2go.distLong,
            itVehList->second.cvStatus.back().motionState.speed);  // in deciseconds           
          if (spatIn.spatMsg.phaseState[itPrsList->second.requestPhase - 1].currState == MsgEnum::phasestate_enum_t::GREEN // control phase in green
            && time2go > spatIn.spatMsg.phaseState[itPrsList->second.requestPhase - 1].timeToChange // can't pass duing green
            && time2go < spatIn.spatMsg.phaseState[itPrsList->second.requestPhase - 1].timeToChange + maxGreenExtenstion  // expecting to arrive within maxGreenExtenstion
            && spatIn.spatMsg.phaseState[signalControlStatus.synch_phase - 1].currState == MsgEnum::phasestate_enum_t::GREEN // sync phase in green
            && spatIn.spatMsg.phaseState[signalControlStatus.synch_phase - 1].timeToChange < maxTime2changePhaseExt)  // close to the yield point
          {
            // green extension has to be requested before termination of syncPhase green
            phases2call.set(signalControlStatus.synch_phase - 1, 1);
            priorityCall = softcall_enum_t::EXTENSION;
            itPrsList->second.requestStatus = MsgEnum::priorityrequest_enum_t::ACTIVE;
            signalControlStatus.prsAction = priorityRequestServer_enum_t::GREENEXTENSION;
            signalControlStatus.priorityPhaseId = signalControlStatus.synch_phase;
            signalControlStatus.priorityVehicleId = itPrsList->first;
            signalControlStatus.priorityCycleCnt = signalControlStatus.cycleCtn;
            signalControlStatus.priorityTms = fullTimeStamp.tms; 
            break;
          }
          else if (spatIn.spatMsg.phaseState[itPrsList->second.requestPhase - 1].currState == MsgEnum::phasestate_enum_t::YELLOW  // control phase in yellow
            && time2go > spatIn.spatMsg.phaseState[itPrsList->second.requestPhase - 1].timeToChange / 2)    // can't pass within yellow
          {
            phases2call.set(itPrsList->second.requestPhase - 1, 1);            
            priorityCall = softcall_enum_t::CALL;
            itPrsList->second.requestStatus = MsgEnum::priorityrequest_enum_t::ACTIVE;
            signalControlStatus.prsAction = priorityRequestServer_enum_t::EARLYGREEN;
            signalControlStatus.priorityPhaseId = itPrsList->second.requestPhase;
            signalControlStatus.priorityVehicleId = itPrsList->first;
            signalControlStatus.priorityCycleCnt = signalControlStatus.cycleCtn;
            signalControlStatus.priorityTms = fullTimeStamp.tms; 
            break;
          }
          else if (spatIn.spatMsg.phaseState[itPrsList->second.requestPhase - 1].currState == MsgEnum::phasestate_enum_t::RED  // control phase in red
            && time2go < spatIn.spatMsg.phaseState[itPrsList->second.requestPhase - 1].timeToChange)    // expecting to arrive at the stopbar within red
          {
            phases2call.set(itPrsList->second.requestPhase - 1, 1);            
            priorityCall = softcall_enum_t::CALL;
            itPrsList->second.requestStatus = MsgEnum::priorityrequest_enum_t::ACTIVE;
            signalControlStatus.prsAction = priorityRequestServer_enum_t::EARLYGREEN;
            signalControlStatus.priorityPhaseId = itPrsList->second.requestPhase;
            signalControlStatus.priorityVehicleId = itPrsList->first;
            signalControlStatus.priorityCycleCnt = signalControlStatus.cycleCtn;
            signalControlStatus.priorityTms = fullTimeStamp.tms; 
            break;
          }
        }
      }
    }
    if (phases2call.any())
    {
      /// send priority call
      softReq.callphase = static_cast<uint8_t>(phases2call.to_ulong());
      softReq.callobj = static_cast<uint8_t>(softcall_enum_t::PRIORITY);
      softReq.calltype = static_cast<uint8_t>(priorityCall);
      sendSoftReq(fd_Send,sendbuf,softReq,fullTimeStamp.ms_since_midnight_local,msgid_softcall);
      OS_Display << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp); 
      if (priorityCall == softcall_enum_t::EXTENSION)
        OS_Display << ", call PriorityGreenExtension on phase " << static_cast<int>(softReq.callphase) << endl;
      else
        OS_Display << ", call PriorityEarlyGreen on phase " << static_cast<int>(softReq.callphase) << endl;
      if (logFile)
      {
        int fileIdx = findFileIdx(logtypes,"awr");
        if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
        {
          *(logFiles[fileIdx].OS) << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
          if (priorityCall == softcall_enum_t::EXTENSION)
            *(logFiles[fileIdx].OS) << ", call PriorityGreenExtension on phase " << static_cast<int>(softReq.callphase) << endl;
          else
            *(logFiles[fileIdx].OS) << ", call PriorityEarlyGreen on phase " << static_cast<int>(softReq.callphase) << endl;
          logFiles[fileIdx].logrows++;
        }
        
        fileIdx = findFileIdx(logtypes,"req");
        if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
        {
          logSoftReqMsg(*(logFiles[fileIdx].OS),&softReq,fullTimeStamp.ms_since_midnight_local);
          logFiles[fileIdx].logrows++; 
        }
      }
      phases2call.reset();
    }

    /// check vehicle phase call
    phases2call.reset();
    timeUtils::getFullTimeStamp(fullTimeStamp);   
    for (itVehList = vehList.begin(); itVehList != vehList.end(); ++itVehList)
    {
      if (itVehList->second.isOnApproach && !itVehList->second.isPhaseCalled      // at an approach of this intersection and has not been called before
        && itVehList->second.cvStatus.back().motionState.speed > softcallSpeed)   // vehicle is not stopped
      {
        uint8_t controlPhase = (uint8_t)(itVehList->second.cvStatus.back().vehicleLocationAware.controlPhase - 1);
        uint16_t time2go = getTime2goDeci(itVehList->second.cvStatus.back().vehicleLocationAware.dist2go.distLong,itVehList->second.cvStatus.back().motionState.speed);                                         // in deciseconds
        if (itVehList->second.cvStatus.back().vehicleSignalAware.currState != MsgEnum::phasestate_enum_t::GREEN   // phase not in Green
          && !signalControlStatus.veh_call.test(controlPhase - 1)                                       // phase has not been called
          && time2go < maxTime2goPhaseCall                                                              // time2go within maxTime2goPhaseCall requirement   
          && fullTimeStamp.tms - signalControlStatus.phaseCallTms[controlPhase] > vehPhaseCallInterval) // within vehPhaseCallInterval requirement
        {
          // phase is not green and within the maximum bound for soft phase call
          phases2call.set(controlPhase,1);
          itVehList->second.isPhaseCalled = true;
          signalControlStatus.phaseCallTms[controlPhase] = fullTimeStamp.tms;
        }
      }
    } 
    if (phases2call.any())
    {
      /// send vehicle phase call
      softReq.callphase = static_cast<uint8_t>(phases2call.to_ulong());
      softReq.callobj = static_cast<uint8_t>(softcall_enum_t::VEH);
      softReq.calltype = static_cast<uint8_t>(softcall_enum_t::CALL);
      sendSoftReq(fd_Send,sendbuf,softReq,fullTimeStamp.ms_since_midnight_local,msgid_softcall);
      OS_Display << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp); 
      OS_Display << ", call vehicle on phase " << static_cast<int>(softReq.callphase) << endl;
      if (logFile)
      {
        int fileIdx = findFileIdx(logtypes,"awr");
        if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
        {
          *(logFiles[fileIdx].OS) << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
          *(logFiles[fileIdx].OS) << ", call vehicle on phase " << static_cast<int>(softReq.callphase) << endl;
          logFiles[fileIdx].logrows++;
        }
        
        fileIdx = findFileIdx(logtypes,"req");
        if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
        {
          logSoftReqMsg(*(logFiles[fileIdx].OS),&softReq,fullTimeStamp.ms_since_midnight_local);
          logFiles[fileIdx].logrows++; 
        }
      }
      phases2call.reset();
    }  
    
    /// check vehicle green extension call: check cancel of active call first
    phases2call.reset();
    spatIn = getSpatFromList(spatList,intersectionId);  // get SPaT info for this intersection    
    if (spatIn.tms > 0)
    {
      timeUtils::getFullTimeStamp(fullTimeStamp);         
      for (int i = 0; i < NEMAPHASES; i++)
      {
        /// check phase that has been called for extension
        if (signalControlStatus.phaseExtStatus[i].callAction == softcall_enum_t::CALLED)
        {
          /// loop through callVehicles, remove vehicles that has passed the intersection
          for (itPhaseExtVidMap = signalControlStatus.phaseExtStatus[i].callVehicles.begin(); itPhaseExtVidMap != signalControlStatus.phaseExtStatus[i].callVehicles.end();)
          {
            itVehList = vehList.find(itPhaseExtVidMap->first);
            if (itVehList == vehList.end() || !itVehList->second.isOnApproach)
              signalControlStatus.phaseExtStatus[i].callVehicles.erase(itPhaseExtVidMap++);
            else
              ++itPhaseExtVidMap;
          }       
          if (spatIn.spatMsg.phaseState[i].currState == MsgEnum::phasestate_enum_t::GREEN   // phase still in green
            && signalControlStatus.phaseExtStatus[i].callVehicles.empty())        // no vehicle needs extension any more
          {
            /// needs to send soft cancel call
            phases2call.set(i,1);
            signalControlStatus.phaseExtStatus[i].callAction = softcall_enum_t::CANCELLED;
          }
          else if (spatIn.spatMsg.phaseState[i].currState != MsgEnum::phasestate_enum_t::GREEN) // phase no longer in green
          {
            /// phase changed from green, got to cancel regardless whether the vehicles passed the intersection or not
            /// no need to send soft cancel call but send it anyway
            phases2call.set(i,1);
            signalControlStatus.phaseExtStatus[i].callAction = softcall_enum_t::CANCELLED;
            signalControlStatus.phaseExtStatus[i].callVehicles.clear();
          }
        }
      }
    }
    if (phases2call.any())
    {
      /// send cancel vehicle phase extension
      softReq.callphase = static_cast<uint8_t>(phases2call.to_ulong());
      softReq.callobj = static_cast<uint8_t>(softcall_enum_t::VEH);
      softReq.calltype = static_cast<uint8_t>(softcall_enum_t::CANCEL);
      sendSoftReq(fd_Send,sendbuf,softReq,fullTimeStamp.ms_since_midnight_local,msgid_softcall);
      OS_Display << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp); 
      OS_Display << ", cancel vehicle extension on phase " << static_cast<int>(softReq.callphase) << endl;
      if (logFile)
      {
        int fileIdx = findFileIdx(logtypes,"awr");
        if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
        {
          *(logFiles[fileIdx].OS) << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
          *(logFiles[fileIdx].OS) << ", cancel vehicle extension on phase " << static_cast<int>(softReq.callphase) << endl;
          logFiles[fileIdx].logrows++;
        }
        
        fileIdx = findFileIdx(logtypes,"req");
        if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
        {
          logSoftReqMsg(*(logFiles[fileIdx].OS),&softReq,fullTimeStamp.ms_since_midnight_local);
          logFiles[fileIdx].logrows++; 
        }
      }
      phases2call.reset();
    }
          
    /// check vehicle green extension call: check whether need to initiate a call
    ///   (when coordinated, non-coordination phases only as coordinated phase green extension involving signal priority green extension treatment)
    ///   Note that vehicle green extension can only be initiated when signal priority control is not active, as it may conflict with priority treatment
    phases2call.reset();    
    if ((signalControlStatus.cntlrMode == operation_mode_enum_t::RUNNINGFREE 
    || signalControlStatus.cntlrMode == operation_mode_enum_t::COORDINATION)
    && signalControlStatus.prsAction == priorityRequestServer_enum_t::NONE)
    {
      timeUtils::getFullTimeStamp(fullTimeStamp);   
      for (itVehList = vehList.begin(); itVehList != vehList.end(); ++itVehList)
      {
        if (itVehList->second.isOnApproach && !itVehList->second.isExtensionCalled  // at an approach of this intersection and has not been called before
          && itVehList->second.cvStatus.back().motionState.speed > softcallSpeed)             // vehicle is not stopped
        {
          uint8_t controlPhase = (uint8_t)(itVehList->second.cvStatus.back().vehicleLocationAware.controlPhase - 1);
          uint16_t time2go = getTime2goDeci(itVehList->second.cvStatus.back().vehicleLocationAware.dist2go.distLong,
            itVehList->second.cvStatus.back().motionState.speed);                   // in deciseconds
          if (signalControlStatus.phaseExtStatus[controlPhase].callAction != softcall_enum_t::CALLED  // no multiple extension calls on one green phase
            && (signalControlStatus.cntlrMode != operation_mode_enum_t::COORDINATION    // when not under coordination, can request any phase
              || !signalControlStatus.coordinatedPhases.test(controlPhase))                      // when under coordination, can not request a coordinated phase
            && itVehList->second.cvStatus.back().vehicleSignalAware.currState == MsgEnum::phasestate_enum_t::GREEN // phase in green
            && itVehList->second.cvStatus.back().vehicleSignalAware.timeToChange < maxTime2changePhaseExt // time2change within maxTime2changePhaseExt
            && time2go > itVehList->second.cvStatus.back().vehicleSignalAware.timeToChange                // can't pass duing the green
            && time2go < itVehList->second.cvStatus.back().vehicleSignalAware.timeToChange + maxTime2phaseExt) // expecting to arrive within maxTime2phaseExt
          {
            phases2call.set(controlPhase,1);
            itVehList->second.isExtensionCalled = true;
            signalControlStatus.phaseExtStatus[controlPhase].callAction = softcall_enum_t::PENDING;
            signalControlStatus.phaseExtStatus[controlPhase].callTms = fullTimeStamp.tms;
            signalControlStatus.phaseExtStatus[controlPhase].callVehicles[itVehList->second.cvStatus.back().id] =
              static_cast<uint8_t>(time2go - itVehList->second.cvStatus.back().vehicleSignalAware.timeToChange);
          }
        }
      }
    }
    if (phases2call.any())
    {      
      for (int i = 0; i < 8; i++)
      {
        if (phases2call.test(i))
        {
          signalControlStatus.phaseExtStatus[i].callAction = softcall_enum_t::CALLED;
        }
      }
      /// send cancel vehicle phase extension
      softReq.callphase = static_cast<uint8_t>(phases2call.to_ulong());
      softReq.callobj = static_cast<uint8_t>(softcall_enum_t::VEH);
      softReq.calltype = static_cast<uint8_t>(softcall_enum_t::EXTENSION);
      sendSoftReq(fd_Send,sendbuf,softReq,fullTimeStamp.ms_since_midnight_local,msgid_softcall);
      OS_Display << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp); 
      OS_Display << ", call vehicle extension on phase " << static_cast<int>(softReq.callphase) << endl;
      if (logFile)
      {
        int fileIdx = findFileIdx(logtypes,"awr");
        if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
        {
          *(logFiles[fileIdx].OS) << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
          *(logFiles[fileIdx].OS) << ", call vehicle extension on phase " << static_cast<int>(softReq.callphase) << endl;
          logFiles[fileIdx].logrows++;
        }
        
        fileIdx = findFileIdx(logtypes,"req");
        if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
        {
          logSoftReqMsg(*(logFiles[fileIdx].OS),&softReq,fullTimeStamp.ms_since_midnight_local);
          logFiles[fileIdx].logrows++; 
        }
      }
      phases2call.reset();
    }
          
    /// check whether need to send SSM
    timeUtils::getFullTimeStamp(fullTimeStamp);   
    if (fullTimeStamp.tms > signalControlStatus.ssmTms +  ssmInterval)
    {
      spatIn = getSpatFromList(spatList,intersectionId);
      if (spatIn.tms > 0)
      {
        /// fill ssm2send structure
        signalControlStatus.ssmMsgCnt = (uint8_t)((signalControlStatus.ssmMsgCnt + 1) % 128);
        signalControlStatus.ssmTms = fullTimeStamp.tms;
        ssm2send.msgCnt = signalControlStatus.ssmMsgCnt;
        ssm2send.id = intersectionId;
        ssm2send.status = static_cast<uint8_t>(signalControlStatus.prsAction);
        ssm2send.ms = static_cast<uint16_t>(fullTimeStamp.localDateTimeStamp.timeStamp.sec * 10 + fullTimeStamp.localDateTimeStamp.timeStamp.millisec /100);
        ssm2send.priorityCause = signalControlStatus.priorityVehicleId;
        int i = 0;
        long long etaOffset;
        /// fill priorityVehicleId on the top of the request entries
        if (signalControlStatus.prsAction != priorityRequestServer_enum_t::NONE)
        {
          itPrsList = signalControlStatus.prsList.find(signalControlStatus.priorityVehicleId);
          if (itPrsList != signalControlStatus.prsList.end())
          {
            ssm2send.request[i].id = itPrsList->first;
            ssm2send.request[i].inLaneId = itPrsList->second.requestInLaneId;
            ssm2send.request[i].outLaneId = itPrsList->second.requestOutLaneId;
            ssm2send.request[i].contolPhase = itPrsList->second.requestPhase;
            ssm2send.request[i].classType = itPrsList->second.requestVehicleClassType;
            ssm2send.request[i].classLevel = itPrsList->second.requestVehicleClassLevel;
            ssm2send.request[i].requestStatus = static_cast<uint8_t>(itPrsList->second.requestStatus);
            ssm2send.request[i].transitStatus = itPrsList->second.requestVehicleTransitStatus;
            memcpy(&(ssm2send.request[i].timeOfService),&(itPrsList->second.requestTimeOfService),sizeof(DTime_element_t));
            memcpy(&(ssm2send.request[i].endOfService),&(itPrsList->second.requestEndOfService),sizeof(DTime_element_t));
            etaOffset = (itPrsList->second.requestedServiceEndTms - fullTimeStamp.tms)/100;     // in deciseconds
            if (etaOffset < 0)
              etaOffset = 0;
            ssm2send.request[i].etaOffset = static_cast<uint16_t>(etaOffset);
            i++;
          }
        }
        /// add other requests  
        for (itPrsList = signalControlStatus.prsList.begin(); itPrsList != signalControlStatus.prsList.end();++itPrsList)
        {
          if (itPrsList->first == signalControlStatus.priorityVehicleId)
            continue;
          ssm2send.request[i].id = itPrsList->first;
          ssm2send.request[i].inLaneId = itPrsList->second.requestInLaneId;
          ssm2send.request[i].outLaneId = itPrsList->second.requestOutLaneId;
          ssm2send.request[i].contolPhase = itPrsList->second.requestPhase;
          ssm2send.request[i].classType = itPrsList->second.requestVehicleClassType;
          ssm2send.request[i].classLevel = itPrsList->second.requestVehicleClassLevel;
          ssm2send.request[i].requestStatus = static_cast<uint8_t>(itPrsList->second.requestStatus);
          ssm2send.request[i].transitStatus = itPrsList->second.requestVehicleTransitStatus;
          memcpy(&(ssm2send.request[i].timeOfService),&(itPrsList->second.requestTimeOfService),sizeof(DTime_element_t));
          memcpy(&(ssm2send.request[i].endOfService),&(itPrsList->second.requestEndOfService),sizeof(DTime_element_t));
          etaOffset = (itPrsList->second.requestedServiceEndTms - fullTimeStamp.tms)/100;     // in deciseconds
          if (etaOffset < 0)
            etaOffset = 0;
          ssm2send.request[i].etaOffset = static_cast<uint16_t>(etaOffset);
          i++;
          if (i == MAXPRIOREQUESTS)
            break;
        }
        ssm2send.requestNums = static_cast<uint8_t>(i);
        
        // encode SSM
        ssize_t ssmPayload_size = asnJ2735Lib.encode_ssm_payload(&ssm2send,ssmPayload,MAXDSRCMSGSIZE,false);
        if (ssmPayload_size > 0)
        {
          /// send to MRP_DataMgr
          sendSSM(fd_Send,sendbuf,ssmPayload,(size_t)ssmPayload_size,fullTimeStamp.ms_since_midnight_local,wmeUtils::msgid_ssm);
          /// log
          if (logFile && ssm2send.requestNums > 0)
          {
            int fileIdx = findFileIdx(logtypes,"ssm");
            if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
            {
              logSSM(*(logFiles[fileIdx].OS),ssm2send,fullTimeStamp.localDateTimeStamp);
              logFiles[fileIdx].logrows++; 
            }
          }
        }
        else
        {
          OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
          OS_ERR << ", failed encode_ssm_payload" << endl;     
          OS_Display << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
          OS_Display << ", failed encode_ssm_payload" << endl;               
        }
      }
    }

    /// update lists (remove entry if it's outdated by timeOutms milliseconds)
    cleanBsmList(bsmList,fullTimeStamp.tms,timeOutms);
    cleanSrmList(srmList,fullTimeStamp.tms,timeOutms);
    cleanSpatList(spatList,fullTimeStamp.tms,timeOutms);

    cleanVehList(vehList,fullTimeStamp.tms,timeOutms);
    cleanPrsList(signalControlStatus.prsList,fullTimeStamp.tms,timeOutms);
    
    /// check reopen log files
    if (logFile && fullTimeStamp.tms_minute > logfile_tms_minute + logFileInterval)
    {
      reOpenLogFiles(logFiles,fullTimeStamp.localDateTimeStamp);     
      logfile_tms_minute = fullTimeStamp.tms_minute;
    }
    
    /// check reopen display log file
    if (fullTimeStamp.tms_minute > displayfile_tms_minute + logFileInterval)
    {
      OS_Display.close();
      OS_Display.open(s_DisplayLogFileName.c_str());
      displayfile_tms_minute = fullTimeStamp.tms_minute;
    }
    
    // sleep 
    usleep(usleepms);
  }
}

bool updateSignalState(mmitssSignalControlStatus_t& cntrStatus,const signal_state_t* psigState,const timing_card_t& timingcard)
{
  /// trace pattern_num change
  if (cntrStatus.cntlrMode == operation_mode_enum_t::UNKNOWN || cntrStatus.pattern_num != psigState->pattern_num)
  {
    uint8_t plan_num;
    uint8_t offset_index;
    ssize_t coordplan_index;
    pattern2planOffset(plan_num,offset_index,psigState->pattern_num);
    if (!getPatnIdx(coordplan_index,(operation_mode_enum_t::controlmode)(psigState->cntlrMode),plan_num,timingcard.plannum2planindex_map))
      return false;
    cntrStatus.cntlrMode = (operation_mode_enum_t::controlmode)(psigState->cntlrMode);
    cntrStatus.pattern_num = psigState->pattern_num;
    cntrStatus.coordplan_index = coordplan_index;
    if (coordplan_index >= 0)
    {
      cntrStatus.coordinatedPhases = timingcard.coordplans[coordplan_index].sync_phases;
      cntrStatus.synch_phase = timingcard.coordplans[coordplan_index].coordinated_phases[timingcard.coordplans[coordplan_index].sync_ring];      
    }
    else
    {
      cntrStatus.coordinatedPhases = bitset<8>(string("00100010"));
      cntrStatus.synch_phase = 2;
    }
  } 
  cntrStatus.permitted_phases = bitset<8>(psigState->permitted_phases);
  cntrStatus.permitted_ped_phases = bitset<8>(psigState->permitted_ped_phases);
  cntrStatus.preempt = bitset<8>(psigState->preempt);
  cntrStatus.ped_call = bitset<8>(psigState->ped_call);
  cntrStatus.veh_call = bitset<8>(psigState->veh_call);
  return true;
}

void cleanVehList(map<uint32_t,cvStatusAware_t>& list,const long long tms,const long long timeout)
{
  for(map<uint32_t,cvStatusAware_t>::iterator it = list.begin(); it != list.end();) 
  {
    if (tms > it->second.cvStatus.back().ts + timeout)
    {
      list.erase(it++);
    }
    else
      ++it;
  }
}

void cleanPrsList(map<uint32_t,priorityRequestTableEntry_t>& list,const long long tms,const long long timeout)
{
  for(map<uint32_t,priorityRequestTableEntry_t>::iterator it = list.begin(); it != list.end();) 
  {
    if (tms > it->second.receivedTms + timeout)
    {
      list.erase(it++);
    }
    else
      ++it;
  }
}

void logSSM(std::ofstream& OS,const SSM_element_t& ssm,const timeUtils::dateTimeStamp_t& ts)
{
  OS << timeUtils::getTimestampStr(ts) << ",";
  OS << static_cast<int>(ssm.msgCnt) << ",";
  OS << ssm.id << ",";
  OS << static_cast<int>(ssm.status) << ",";
  OS << static_cast<int>(ssm.ms) << ",";
  OS << ssm.priorityCause << ",";
  OS << static_cast<int>(ssm.requestNums);
  for (uint8_t i = 0; i < ssm.requestNums; i++)
  {
    OS << "," << ssm.request[i].id;
    OS << "," << static_cast<int>(ssm.request[i].inLaneId);
    OS << "," << static_cast<int>(ssm.request[i].outLaneId);
    OS << "," << static_cast<int>(ssm.request[i].contolPhase);
    OS << "," << static_cast<int>(ssm.request[i].classType);
    OS << "," << static_cast<int>(ssm.request[i].classLevel);
    OS << "," << static_cast<int>(ssm.request[i].requestStatus);
    OS << "," << static_cast<int>(ssm.request[i].transitStatus);
    OS << "," << static_cast<int>(ssm.request[i].timeOfService.hour);
    OS << ":" << static_cast<int>(ssm.request[i].timeOfService.min);
    OS << ":" << static_cast<int>(ssm.request[i].timeOfService.sec);
    OS << "," << static_cast<int>(ssm.request[i].endOfService.hour);
    OS << ":" << static_cast<int>(ssm.request[i].endOfService.min);
    OS << ":" << static_cast<int>(ssm.request[i].endOfService.sec);
    OS << "," << static_cast<int>(ssm.request[i].etaOffset);
  }
  OS << endl;
}

void logVehTrack(std::ofstream& OS,const cvStatusAware_t& cvStatusAware,const uint32_t trojCnt)
{
  timeUtils::dateTimeStamp_t ts_local;
  for (size_t i = 0; i < cvStatusAware.cvStatus.size(); i++)
  {
    timeUtils::timeStampFrom_ms(cvStatusAware.cvStatus[i].ts,ts_local,true);
    OS << trojCnt << ",";
    OS << static_cast<int>(i) << ",";
    OS << timeUtils::getTimestampStr(ts_local) << ",";
    OS << cvStatusAware.cvStatus[i].id << ",";
    OS << cvStatusAware.cvStatus[i].geoPoint.latitude << ",";
    OS << cvStatusAware.cvStatus[i].geoPoint.longitude << ",";
    OS << cvStatusAware.cvStatus[i].geoPoint.elevation << ",";
    OS << cvStatusAware.cvStatus[i].motionState.speed << ",";
    OS << cvStatusAware.cvStatus[i].motionState.heading << ",";
    OS << static_cast<int>(cvStatusAware.cvStatus[i].vehicleTrackingState.intsectionTrackingState.vehicleIntersectionStatus) << ",";
    OS << cvStatusAware.cvStatus[i].vehicleLocationAware.intersectionId << ",";
    OS << static_cast<int>(cvStatusAware.cvStatus[i].vehicleLocationAware.laneId) << ",";
    OS << static_cast<int>(cvStatusAware.cvStatus[i].vehicleLocationAware.controlPhase) << ",";  
    OS << cvStatusAware.cvStatus[i].vehicleLocationAware.dist2go.distLong << ",";
    OS << cvStatusAware.cvStatus[i].vehicleLocationAware.dist2go.distLat << ",";
    OS << static_cast<int>(cvStatusAware.cvStatus[i].vehicleSignalAware.currState) << ",";
    OS << static_cast<int>(cvStatusAware.cvStatus[i].vehicleSignalAware.timeToChange) << ",";
    OS << static_cast<int>(cvStatusAware.cvStatus[i].vehicleSignalAware.stateConfidence) << ",";
    OS << static_cast<int>(cvStatusAware.cvStatus[i].vehicleSignalAware.nextState) << ",";
    OS << static_cast<int>(cvStatusAware.cvStatus[i].vehicleSignalAware.clearanceIntv) << ",";
    OS << static_cast<int>(cvStatusAware.cvStatus[i].vehicleSignalAware.yellStateConfidence) << endl;    
  }
}

void formTroj(vehTroj_t& vehTroj,const cvStatusAware_t& cvStatusAware,const uint32_t laneLen,const uint32_t trojCnt,const double stopSpeed)
{
  vehTroj.trojCnt = trojCnt;
  vehTroj.vehId = cvStatusAware.cvStatus[0].id;
  vehTroj.entryLaneId = cvStatusAware.cvStatus[0].vehicleLocationAware.laneId;
  vehTroj.entryControlPhase = cvStatusAware.cvStatus[0].vehicleLocationAware.controlPhase;
  vehTroj.leaveLaneId = cvStatusAware.cvStatus[cvStatusAware.cvStatus.size() - 1].vehicleLocationAware.laneId;
  vehTroj.leaveControlPhase = cvStatusAware.cvStatus[cvStatusAware.cvStatus.size() - 1].vehicleLocationAware.controlPhase;
  vehTroj.distTraveled = static_cast<uint16_t>((cvStatusAware.cvStatus[0].vehicleLocationAware.dist2go.distLong
    - cvStatusAware.cvStatus[cvStatusAware.cvStatus.size() - 1].vehicleLocationAware.dist2go.distLong)*10);
  vehTroj.timeTraveled = static_cast<uint16_t>((cvStatusAware.cvStatus[cvStatusAware.cvStatus.size() - 1].ts
    - cvStatusAware.cvStatus[0].ts)/100);
  vehTroj.ingressLen = static_cast<uint16_t>(laneLen/10);
  uint32_t stoppedTime = 0;
  for (size_t i = 0; i < cvStatusAware.cvStatus.size(); i++)
  {
    if (cvStatusAware.cvStatus[i].motionState.speed <= stopSpeed)
    {
      stoppedTime++;
    }
  }  
  vehTroj.stoppedTime = static_cast<uint16_t>(stoppedTime);
}

bool sendVehTroj(int fd,char* buf,const vehTroj_t& vehTroj,const uint32_t ms,const uint8_t msgid)
{
  /// mmitss header + vehTroj
  mmitss_udp_header_t header;
  header.msgheader = msg_header;
  header.msgid = msgid;
  header.ms_since_midnight = ms;
  header.length = static_cast<uint16_t>(sizeof(vehTroj_t));
  memcpy(buf,&header,sizeof(mmitss_udp_header_t));
  memcpy(buf+sizeof(mmitss_udp_header_t),&vehTroj,sizeof(vehTroj_t));
  return (socketUtils::sendall(fd,buf,sizeof(mmitss_udp_header_t)+sizeof(vehTroj_t)));   
}

bool sendSoftReq(int fd,char* buf,const softcall_request_t& softReq,const uint32_t ms,const uint8_t msgid)
{
  /// mmitss header + softReq
  mmitss_udp_header_t header;
  header.msgheader = msg_header;
  header.msgid = msgid;
  header.ms_since_midnight = ms;
  header.length = static_cast<uint16_t>(sizeof(softcall_request_t));
  memcpy(buf,&header,sizeof(mmitss_udp_header_t));
  memcpy(buf+sizeof(mmitss_udp_header_t),&softReq,sizeof(softcall_request_t));
  return (socketUtils::sendall(fd,buf,sizeof(mmitss_udp_header_t)+sizeof(softcall_request_t))); 
}

bool sendSSM(int fd,char* buf,const char* msg,const size_t msglen,const uint32_t ms,const uint8_t msgid)
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