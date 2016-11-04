//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************
/* tci.cpp - MRP_TrafficControllerInterface
 * functions:
 * 1. poll controller configuration data (poll_list ==> timing_card_t) and save it to file sharing with MRP_DataMgr and MRP_Aware
 * 2. receive controller pushing out msgs: signal_status_mess_t, status8e_mess_t, longstatus8e_mess_t
 * 3. estimate phase and ped phase remaining time (controller_status_t::phase_status_t)
 * 4. send UDP messages (msgid_cntrlstatus, msgid_detCnt & msgid_detPres) to MRP_DataMgr
 * 5. receive UDP messages (msgid_softcall) from MRP_DataMgr and manage sending soft-call to the traffic controller
 * UDP message structures are defined in msgUtils.h
 * logs:
 * 1. logFile
 *    - controller polls and response
 *    - received softcall requests, and sending to the traffic controller
 * 2. logDetails
 *    - logFile plus
 *    - signal status data received from the controller
 *    - detector count/occupancy data received from the controller
 *    - detector presence data received from the controller
*/

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

#include "ab3418msgs.h"
#include "ab3418fcs.h"
#include "comm.h"
#include "socketUtils.h"
#include "msgUtils.h"
#include "tciConf.h"
#include "tci.h"
#include "catchSignal.h"

using namespace std;

void do_usage(const char* progname)
{
  cerr << "Usage" << progname << endl;
  cerr << "\t-n intersection name" << endl;
  cerr << "\t-s socket.conf" << endl;
  cerr << "\t-c turn on sending soft-call to 2070 controller" << endl;
  cerr << "\t-p turn on polling 2070 when tci starts, default reads from timing-card" << endl;
  cerr << "\t-f log flag: 0 - no log; 1 - simple log; 2 - detail log" << endl;
  cerr << "\t-v turn on verbose" << endl;
  exit (1);
}

int main(int argc, char** argv) 
{
  int option;
  char* f_intName = NULL;
  char* f_socketConfig = NULL;
  volatile bool sendControl2controller = false;
  volatile bool pollTimeCard = false;
  volatile bool saveTimeCard = false;
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
    case 'c':
      sendControl2controller = true;
      break;      
    case 'p':
      pollTimeCard = true;
      saveTimeCard = true;
      break;
    case 'v':
      verbose = true;
      break;
    default:
      do_usage(argv[0]);
      break;
    }
  }
  if (f_socketConfig == NULL || f_intName == NULL) {do_usage(argv[0]);}
    
  /* ----------- preparation -------------------------------------*/
  /// get timestamp
  timeUtils::fullTimeStamp_t fullTimeStamp;
  timeUtils::getFullTimeStamp(fullTimeStamp);
  
  /// instance TCIconfig::configTCI to read configuration files
  TCIconfig::configTCI s_tciConfig(f_socketConfig);
  if (!s_tciConfig.isConfigSucceed())
  {
    cerr << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    cerr << ", failed construct TCIconfig::configTCI";
    cerr << ", f_socketConfig = " << f_socketConfig << endl;
    delete f_socketConfig;
    exit(1);
  }
  delete f_socketConfig;

  /// store intersectionName
  string intersectionName = f_intName;
  delete f_intName;
  string timecardName = s_tciConfig.getTimecardPath() + string("/") + intersectionName + string(".timecard");
 
  /// log files
  string logFilePath = s_tciConfig.getLogPath();
  long long logFileInterval = static_cast<long long>(s_tciConfig.getLogInterval()) * 60 * 1000LL;  
  volatile long long logfile_tms_minute = fullTimeStamp.tms_minute;
  /// err log file
  string errLogFile = logFilePath + string("/tci.err");
  ofstream OS_ERR(errLogFile.c_str(),ios::app);
  if (!OS_ERR.is_open()) 
  {
    cerr << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    cerr << ", failed open err log" << endl;
    exit(1);
  }
    
  /// data types to log
  vector<string> logtypes;
  logtypes.push_back(string("tci"));  // genenal log, controller polls and response
  logtypes.push_back(string("sig"));  // log ontroller pushing out signal_status message
  logtypes.push_back(string("cnt"));  // log count/volume data sending to MRP_DataMgr
  logtypes.push_back(string("pres")); // log precense data sending to MRP_DataMgr
  logtypes.push_back(string("req"));  // log softcall received from MRP_DataMgr and acivities sending to cotroller
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
  int fd_mgrSend = socketUtils::client(s_tciConfig.getSocketAddr(TCIconfig::socketHandleEnum_t::MGRSEND),false);
  if (fd_mgrSend < 0)
  {
    OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    OS_ERR << ", failed create MGRSEND socket" << endl;
    if (logFile) {closeLogFiles(logFiles);}
    OS_ERR.close();
    exit(1);
  }  
      
  int fd_mgrListen = socketUtils::server(s_tciConfig.getSocketAddr(TCIconfig::socketHandleEnum_t::MGRLISTEN),true);
  if (fd_mgrListen < 0)
  {
    OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    OS_ERR << ", failed create MGRLISTEN socket" << endl;
    if (logFile) {closeLogFiles(logFiles);}
    OS_ERR.close();
    close(fd_mgrSend);
    exit(1);
  }
  
  /// open serial ports
  int fd_spat = open_port(OS_ERR,s_tciConfig.getSpatPort().c_str(),true);
  if (fd_spat < 0)
  {
    OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    OS_ERR << ", failed open spat port" << endl;
    if (logFile) {closeLogFiles(logFiles);}
    OS_ERR.close();
    close(fd_mgrSend);
    close(fd_mgrListen);
    exit(1);
  }
  
  int fd_spat2 = open_port(OS_ERR,s_tciConfig.getSpat2Port().c_str(),false);
  if (fd_spat2 < 0)
  {
    OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    OS_ERR << ", failed open spat2 port" << endl;
    if (logFile) {closeLogFiles(logFiles);}
    OS_ERR.close();
    close(fd_mgrSend);
    close(fd_mgrListen);
    close_port(fd_spat,true);
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
    close(fd_mgrSend);
    close(fd_mgrListen);
    close_port(fd_spat,true);
    close_port(fd_spat2,false);
    return (0);
  }
  else
    sig_ign( sig_list, sig_hand );

  /* ----------- local variables -------------------------------------*/   
  const useconds_t usleepms = 5000;     // in microseconds  
  /// build poll list
  vector<poll_conf_t> poll_list;
  buildPollList(poll_list);  
  int poll_nums = (int)poll_list.size();
  if (poll_nums == 0)
  {
    OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    OS_ERR << ", failed buildPollList" << endl;
    if (logFile) {closeLogFiles(logFiles);}
    OS_ERR.close();    
    close(fd_mgrSend);
    close(fd_mgrListen);
    close_port(fd_spat,true);
    close_port(fd_spat2,false);
    exit(1);    
  }
  long long poll_interval = 500LL;    // in milliseconds
  int maxpolls_per_request = 10;      // maximum repeat polls for one poll_conf_t
  /// poll control variables
  poll_trace_t poll_trace(pollTimeCard,saveTimeCard,0,0,0LL);

  /// structure to hold static phase timing, flags, and control plan parameters
  timing_card_t timingcard;
  timingcard.controller_addr = 0x00;
  
  /// structure to trace controller status 
  controller_status_t controller_status;
  memset(&controller_status,0,sizeof(controller_status));
  controller_status.isPlantimingReady = false;
  controller_status.mode = operation_mode_enum_t::UNKNOWN;
  controller_status.coordplan_index = -1; 
  
  /// serial port read buffer
  uint8_t recvbuf_spat[MAXAB3418MSGSIZE];
  uint8_t recvbuf_spat2[MAXAB3418MSGSIZE];
  memset(&recvbuf_spat[0],0,sizeof(recvbuf_spat));
  memset(&recvbuf_spat2[0],0,sizeof(recvbuf_spat2));
  /// read all available bytes from serial port, variables to manage received half of a packet 
  uint8_t msgbuf[MAXAB3418MSGSIZE];
  vector< pair<size_t, size_t> > ab3418Frame_spat;
  vector< pair<size_t, size_t> > ab3418Frame_spat2;  
  size_t bytenums_spat = 0;
  size_t bytenums_spat2 = 0;
  
  /// serial port write buffer
  uint8_t sendbuf_spat2[MAXAB3418MSGSIZE];
  memset(&sendbuf_spat2[0],0,sizeof(sendbuf_spat2));
  
  /// structure to hold inbound pushing out ab3418 messages on serial ports
  signal_status_mess_t  signal_status_mess; // fd_spat 
  status8e_mess_t       status8e_mess;      // fd_spat2
  longstatus8e_mess_t   longstatus8e_mess;  // fd_spat2
  /// initial controller status
  status8e_mess.status.reset();
 
  /// buffer to receive on fd_mgrListen
  char recvbuf_socket[MAXUDPMSGSIZE];
  /// interval to send softcall to the traffic controller
  long long softcall_interval = 20;     // in milliseconds
  /// struct to trace softcall state
  softcall_state_t softcall_state;
  softcall_state.reset();
  
  /// buffer to send to fd_mgrSend
  char sendbuf_socket[MAXUDPMSGSIZE];
  
  /// set up serial ports and sockets poll structure
  struct pollfd ufds[3];
  ufds[0].fd = fd_spat;
  ufds[0].events = POLLIN;
  ufds[1].fd = fd_spat2;
  ufds[1].events = POLLIN;
  ufds[2].fd = fd_mgrListen;
  ufds[2].events = POLLIN;
  nfds_t nfds = 3;
  int pollTimeout = 40; // in milliseconds, B48300 = 48 bytes in 10 milliseconds
  
  /// flow control parameters  
  bool process_recvbuf_spat;
  bool process_recvbuf_spat2;
  bool isNewSpat;  
  int retval;
  ssize_t bytesread;
  size_t bytesrequested2read;
  size_t bytesrequested2write;
  ssize_t bytesReceived;
  
  /// read timing card if not polling at the beginning 
  if (!poll_trace.pollTimeCard)
  {
    if (!readTimeCard(timecardName.c_str(),timingcard))
    {
      OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
      OS_ERR << ", failed reading timing card: " << timecardName;
      OS_ERR << ", change to poll instead" << endl;
      /// set to poll 2070 instead
      timingcard.reset(); 
      poll_trace.resetpolls();
    }
  }
  
  while(1)
  {
    /// wait for events
    process_recvbuf_spat = false;
    process_recvbuf_spat2 = false;
    retval = poll(ufds,nfds,pollTimeout);
    if (retval > 0)
    {
      timeUtils::getFullTimeStamp(fullTimeStamp);      
      for (nfds_t i = 0; i < nfds; i++)
      {
        if ((ufds[i].revents & POLLIN) == POLLIN)
        {
          switch (i)
          {
          case 0:
            /// events on fd_spat, read all available bytes
            bytesrequested2read = static_cast<size_t>(MAXAB3418MSGSIZE - bytenums_spat);
            bytesread = read(fd_spat,(uint8_t*)&recvbuf_spat[bytenums_spat],bytesrequested2read);
            if (bytesread > 0)
            {
              bytenums_spat += bytesread;
              ab3418Frame_spat.clear();
              processRecvBuff(recvbuf_spat,bytenums_spat,ab3418Frame_spat);
              if (!ab3418Frame_spat.empty()) {process_recvbuf_spat = true;}
            }
            break;
          case 1:
            /// events on fd_spat2, read all available bytes
            bytesrequested2read = static_cast<size_t>(MAXAB3418MSGSIZE - bytenums_spat2);
            bytesread = read(fd_spat2,(uint8_t*)&recvbuf_spat2[bytenums_spat2],bytesrequested2read);
            if (bytesread > 0)
            {
              bytenums_spat2 += bytesread;
              ab3418Frame_spat2.clear();          
              processRecvBuff(recvbuf_spat2,bytenums_spat2,ab3418Frame_spat2);
              if (!ab3418Frame_spat2.empty()) {process_recvbuf_spat2 = true;}
            }
            break;
          case 2:
            /// events on fd_mgrListen
            bytesReceived = recv(fd_mgrListen,recvbuf_socket,sizeof(recvbuf_socket),0);
            if (bytesReceived > 0 && (size_t)bytesReceived > sizeof(mmitss_udp_header_t))
            {
              mmitss_udp_header_t* pheader = (mmitss_udp_header_t*)&recvbuf_socket[0];
              if (pheader->msgid == msgid_softcall)
              {
                softcall_request_t* prequest = (softcall_request_t*)&recvbuf_socket[sizeof(mmitss_udp_header_t)];
                updateSoftcallState(softcall_state,prequest);
                if (logFile)
                {
                  int fileIdx = findFileIdx(logtypes,"req");
                  if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
                  {
                    *(logFiles[fileIdx].OS) << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp) << ",recv,";
                    *(logFiles[fileIdx].OS) << static_cast<int>(prequest->callphase) << ",";
                    *(logFiles[fileIdx].OS) << static_cast<int>(prequest->callobj) << ",";
                    *(logFiles[fileIdx].OS) << static_cast<int>(prequest->calltype) << endl;
                    logFiles[fileIdx].logrows++;
                  }
                }
              }
            }  
            break;  
          default:
            break;
          }
        }
      }
    }

    isNewSpat = false;
    if (process_recvbuf_spat)
    {
      timeUtils::getFullTimeStamp(fullTimeStamp);
      for (size_t i = 0; i < ab3418Frame_spat.size(); i++)
      {
        size_t msglen = ab3418Frame_spat[i].second - ab3418Frame_spat[i].first + 1;
        if (msglen > 5)
        {
          if (processAB3418Msg(OS_ERR,msgbuf,&recvbuf_spat[ab3418Frame_spat[i].first],msglen))
          {
            switch(msgbuf[4])
            {
            case AB3418MSG::rawspatRes_messType:
              // only expect rawspatRes_messType on fd_spat
              isNewSpat = true;
              parseSignalStatusMsg(signal_status_mess,&msgbuf[0]);
              if (timingcard.controller_addr == 0x00) {timingcard.controller_addr = signal_status_mess.controller_addr;}
              if (logDetails)
              {
                int fileIdx = findFileIdx(logtypes,"sig");
                if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
                {
                  logSignalStatusMsg(*(logFiles[fileIdx].OS),signal_status_mess);
                  logFiles[fileIdx].logrows++;
                }
              }
              break;
            default:
              OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
              OS_ERR << ", fd_spat unknown mess_type " << hex << uppercase << setw(2) << setfill('0');
              OS_ERR << static_cast<unsigned int>(msgbuf[4]) << nouppercase << dec << endl;
            } 
          }
        }
      }
      /// reset bytenums_spat
      size_t endIndex = ab3418Frame_spat[ab3418Frame_spat.size()-1].second;
      if (endIndex >= bytenums_spat - 1)
        bytenums_spat = 0;
      else if (recvbuf_spat[endIndex +1] == AB3418MSG::flag)
      {
        size_t offset = 0;
        for (size_t i = endIndex +1; i < bytenums_spat; i++)
        {
          recvbuf_spat[offset] = recvbuf_spat[i];
          offset++;
        }
        bytenums_spat = offset;
      }
      else
        bytenums_spat = 0;
    }
  
    if (process_recvbuf_spat2)
    {
      timeUtils::getFullTimeStamp(fullTimeStamp);      
      for (size_t i = 0; i < ab3418Frame_spat2.size(); i++)
      {
        size_t msglen = ab3418Frame_spat2[i].second - ab3418Frame_spat2[i].first + 1;
        if (msglen > 5)
        {
          if (processAB3418Msg(OS_ERR,msgbuf,&recvbuf_spat2[ab3418Frame_spat2[i].first],msglen))
          {
            switch(msgbuf[4])
            {
            case AB3418MSG::status8eRes_messType:
              parseStatus8eMsg(status8e_mess,&msgbuf[0]);
              controller_status.status = status8e_mess.status;
              /// send to MRP_DataMgr
              sendPresMsg(fd_mgrSend,sendbuf_socket,status8e_mess);
              /// log to file
              if (logDetails)
              {                
                int fileIdx = findFileIdx(logtypes,"pres");
                if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
                {
                  pres_data_t* pPresData = (pres_data_t*)&sendbuf_socket[sizeof(mmitss_udp_header_t)];
                  logDetPresMsg(*(logFiles[fileIdx].OS),pPresData,status8e_mess.fullTimeStamp.ms_since_midnight_local);
                  logFiles[fileIdx].logrows++; 
                }
              }                
              break;
            case AB3418MSG::longStatus8eRes_messType:
              parseLongStatus8eMsg(longstatus8e_mess,&msgbuf[0]);
              /// send to MRP_DataMgr
              sendCntMsg(fd_mgrSend,sendbuf_socket,longstatus8e_mess);
              /// log to file
              if (logDetails)
              {
                int fileIdx = findFileIdx(logtypes,"cnt");
                if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
                {
                  count_data_t* pcntData = (count_data_t*)&sendbuf_socket[sizeof(mmitss_udp_header_t)];
                  logDetCntMsg(*(logFiles[fileIdx].OS),pcntData,longstatus8e_mess.fullTimeStamp.ms_since_midnight_local);
                  logFiles[fileIdx].logrows++; 
                }
              }
              break;
            case AB3418MSG::getBlockMsgRes_errMessType:
              /// getBlock request returned error (message includes pageId & blockId of getBlockMsg)
              if (poll_trace.pollTimeCard)
              {
                int pollIdx = getPollIdx(poll_list,&msgbuf[5],AB3418MSG::getBlockMsgRes_errMessType);
                if (pollIdx >=0 && pollIdx < poll_nums)
                {             
                  OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
                  OS_ERR << ", getBlockMsg error: " << poll_list[pollIdx].poll_desc;
                  OS_ERR << ", err_num = " << static_cast<int>(msgbuf[7]);
                  OS_ERR << ", err_code " << ab3418msgErrCode(msgbuf[7]) << endl;
                }
              }
              break;
            case AB3418MSG::getTimingDataRes_errMessType:
              /// getTimingData request returned error (message includes error number and index number)           
              OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
              OS_ERR << ", getTimingData err_num = " << static_cast<int>(msgbuf[5]);
              OS_ERR << ", err_code " << ab3418msgErrCode(msgbuf[5]) << endl;
              break;
            case AB3418MSG::setSoftcallRes_errMessType:
              /// setSoftcall request returned error (message includes error number and index number) 
              OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
              OS_ERR << ", setSoftcall err_num = " << static_cast<int>(msgbuf[5]);
              OS_ERR << ", err_code " << ab3418msgErrCode(msgbuf[5]) << endl;
              if (logFile)
              {
                int fileIdx = findFileIdx(logtypes,"req");
                if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
                {
                  *(logFiles[fileIdx].OS) << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp) << ",failed" << endl;
                  logFiles[fileIdx].logrows++;
                }
              }              
              break;
            case AB3418MSG::setSoftcallRes_messType:
              if (logFile)
              {
                int fileIdx = findFileIdx(logtypes,"req");
                if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
                {
                  *(logFiles[fileIdx].OS) << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp) << ",succeed" << endl;
                  logFiles[fileIdx].logrows++;
                }
              }
              break; 
            case AB3418MSG::getTimingDataRes_messType:
              /// getTimingData request returned success
              if (poll_trace.pollTimeCard)
              {
                int pollIdx = getPollIdx(poll_list,&msgbuf[5],AB3418MSG::getTimingDataRes_messType);
                if (pollIdx >=0 && pollIdx < poll_nums)
                {
                  updateTimingCard(timingcard,&msgbuf[0],poll_list[pollIdx].poll_desc);
                  poll_list[pollIdx].pollReturned = true;
                  if (verbose)
                  {
                    cout << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
                    cout << "," << poll_list[pollIdx].poll_desc << " returned success" << endl;              
                  }
                  if (logFile)
                  {
                    int fileIdx = findFileIdx(logtypes,"tci");
                    if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
                    {
                      *(logFiles[fileIdx].OS) << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
                      *(logFiles[fileIdx].OS) << "," << poll_list[pollIdx].poll_desc << " returned success" << endl;
                      logFiles[fileIdx].logrows++;
                    }
                  }
                }  
              }  
              break;
            case AB3418MSG::getBlockMsgRes_messType:
              /// getBlock request returned success
              if (poll_trace.pollTimeCard)
              {
                int pollIdx = getPollIdx(poll_list,&msgbuf[5],AB3418MSG::getBlockMsgRes_messType);                
                if (pollIdx >=0 && pollIdx < poll_nums)
                {
                  updateTimingCard(timingcard,&msgbuf[0],poll_list[pollIdx].poll_desc);
                  poll_list[pollIdx].pollReturned = true;
                  if (verbose)
                  {
                    cout << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
                    cout << "," << poll_list[pollIdx].poll_desc << " returned success" << endl; 
                  }
                  if (logFile)
                  {
                    int fileIdx = findFileIdx(logtypes,"tci");
                    if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
                    {
                      *(logFiles[fileIdx].OS) << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
                      *(logFiles[fileIdx].OS) << "," << poll_list[pollIdx].poll_desc << " returned success" << endl;
                      logFiles[fileIdx].logrows++;
                    }
                  }
                }
              }
              break;
            default:
              OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
              OS_ERR << ", fd_spat2 unknown mess_type " << hex << uppercase << setw(2) << setfill('0');
              OS_ERR << static_cast<unsigned int>(msgbuf[4]) << nouppercase << dec << endl;
            }
          }
        }
      }
      /// reset bytenums_spat2
      size_t endIndex = ab3418Frame_spat2[ab3418Frame_spat2.size()-1].second;
      if (endIndex >= bytenums_spat2 - 1)
        bytenums_spat2 = 0;
      else if (recvbuf_spat2[endIndex +1] == AB3418MSG::flag)
      {
        size_t offset = 0;
        for (size_t i = endIndex +1; i < bytenums_spat2; i++)
        {
          recvbuf_spat2[offset] = recvbuf_spat2[i];
          offset++;
        }
        bytenums_spat2 = offset;
      }
      else
        bytenums_spat2 = 0;
    }
    
    /// deal with poll_list polls
    if (poll_trace.pollTimeCard && (poll_list[poll_trace.poll_seq].pollReturned || poll_trace.numspolled >= maxpolls_per_request))
    {
      /// move to the next poll
      poll_trace.poll_seq++;
      poll_trace.numspolled = 0;
    }
    if (poll_trace.pollTimeCard && poll_trace.poll_seq >= poll_nums)
    {
      /// have looped though poll_list
      poll_trace.resetcnt();
      /// check whether all polls have returned success 
      if (isAllPollReturned(poll_list))
      {
        poll_trace.pollTimeCard = false;
        /// freeplan_mess_t
        getPlanParameters(timingcard.freeplan,timingcard.phaseflags.permitted_phases,timingcard.phaseflags.permitted_ped_phases);
        /// coordplan_mess_t
        getPlanParameters(timingcard.coordplans,timingcard.TSPconf,timingcard.phasetiming,timingcard.phaseflags.permitted_phases,
          timingcard.phaseflags.permitted_ped_phases,timingcard.phaseflags.maximum_recall_phases);
        /// build plannum2planindex_map
        getPlanIdxMap(timingcard.plannum2planindex_map,timingcard.coordplans);        
        if (poll_trace.saveTimeCard)
        {
          if (!logTimeCard(timecardName.c_str(),timingcard))
          {
            OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
            OS_ERR << ", failed logging timing card : " << timecardName << endl;
          }
        }
        if (verbose)
        {
          cout << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp) << ", finished polling controller" << endl;
        }
        if (logFile)
        {
          int fileIdx = findFileIdx(logtypes,"tci");
          if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
          {
            *(logFiles[fileIdx].OS) << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp) << ", finished polling controller" << endl;
            logFiles[fileIdx].logrows++;
          }
        }
      }
    }  
    if (poll_trace.pollTimeCard && timingcard.controller_addr != 0x00 && !poll_list[poll_trace.poll_seq].pollReturned 
      && fullTimeStamp.tms - poll_trace.poll_tms > poll_interval)
    {
      bytesrequested2write = formpollrequest(sendbuf_spat2,timingcard.controller_addr,poll_list[poll_trace.poll_seq]);
      if (write(fd_spat2,(uint8_t*)&sendbuf_spat2,bytesrequested2write) > 0)
      {
        if (verbose)
        {
          cout << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp) << ", polling " << poll_list[poll_trace.poll_seq].poll_desc << endl;
        }
        if (logFile)
        {
          int fileIdx = findFileIdx(logtypes,"tci");
          if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
          {
            *(logFiles[fileIdx].OS) << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp) << ", polling " << poll_list[poll_trace.poll_seq].poll_desc << endl;  
            logFiles[fileIdx].logrows++;
          }
        }
        poll_trace.poll_tms = fullTimeStamp.tms;
        poll_trace.numspolled++;
      }
    }
    if (!poll_trace.pollTimeCard && !controller_status.isPlantimingReady)
    {
      /// get current timing parameters
      controller_status.tms = fullTimeStamp.tms;
      controller_status.status = status8e_mess.status;      
      for (uint8_t ring = 0; ring < 2; ring++)
      {
        if (signal_status_mess.active_phases[ring] > 0)
          controller_status.active_force_off[signal_status_mess.active_phases[ring] - 1] = signal_status_mess.active_force_off[ring];
      }
      controller_status.mode = get_control_mode(controller_status.status,signal_status_mess.preempt,signal_status_mess.pattern_num);
      if (!getPatnIdx(controller_status.coordplan_index,controller_status.mode,signal_status_mess.plan_num,timingcard.plannum2planindex_map))
      {
        /// should not be here as all plans have been polled
        OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
        OS_ERR << ", failed find coordplan_index in timingcard for plan_num " << static_cast<int>(signal_status_mess.plan_num);
        OS_ERR << ", re-poll coord plans" << endl;
        /// poll coordination plans
        timingcard.resetplans(); 
        resetPlanPolls(poll_list);
        poll_trace.resetpolls();
        continue;
      }
      /// initial controller status tracing
      updatePermitPhases(controller_status.permitted_phases,controller_status.permitted_ped_phases,
        controller_status.mode,controller_status.coordplan_index,timingcard);      
      controller_status.curbarrier = barrier_phases_on(signal_status_mess.active_phase);
      controller_status.curbarrier_start_time = fullTimeStamp.tms;
      controller_status.timer_time[0] = fullTimeStamp.tms;
      controller_status.timer_time[1] = fullTimeStamp.tms;
      controller_status.cycle_start_time = fullTimeStamp.tms;
      controller_status.cycle_clock_time = fullTimeStamp.tms;
      /// phase_status
      for (uint8_t i = 1; i <= 8; i++)
      {
        if (!controller_status.permitted_phases.test(i-1))
          controller_status.phase_status[i-1].state = operation_mode_enum_t::NOTPERMITTED;
        else
        {
          controller_status.phase_status[i-1].state = get_light_color(signal_status_mess.active_phases[ring_phase_on(i)],signal_status_mess.active_interval[ring_phase_on(i)],i);
          controller_status.phase_status[i-1].state_start_time = fullTimeStamp.tms;
          controller_status.phase_status[i-1].call_status = operation_mode_enum_t::NOCALL;         
          controller_status.phase_status[i-1].recall_status = operation_mode_enum_t::NORECALL;         
          if (controller_status.mode == operation_mode_enum_t::RUNNINGFREE)
            controller_status.phase_status[i-1].recall_status = getPhaseRecallType(timingcard.phaseflags,timingcard.freeplan,(uint8_t)(i-1));            
          else if (controller_status.coordplan_index >= 0)
            controller_status.phase_status[i-1].recall_status = getPhaseRecallType(timingcard.phaseflags,timingcard.coordplans[controller_status.coordplan_index],(uint8_t)(i-1));
        }
        if (!controller_status.permitted_ped_phases.test(i-1))
          controller_status.phase_status[i-1].pedstate = operation_mode_enum_t::NOTEQUIPPED;
        else
        {
          controller_status.phase_status[i-1].pedstate = get_ped_state(signal_status_mess.active_phases[ring_phase_on(i)],signal_status_mess.active_interval[ring_phase_on(i)],i);
          controller_status.phase_status[i-1].pedstate_start_time = fullTimeStamp.tms;
        }
      }
      memcpy(&controller_status.signal_status,&signal_status_mess,sizeof(signal_status_mess_t));
      controller_status.isPlantimingReady = true;
      if (verbose)
      {
        cout << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
        cout << ", starting, signal is running mode " << static_cast<int>(controller_status.mode);
        if (controller_status.mode == operation_mode_enum_t::COORDINATION)
          cout << ", coordination plan_num " << static_cast<int>(timingcard.coordplans[controller_status.coordplan_index].plan_num);
        else
          cout << ", plan_num " << static_cast<int>(controller_status.signal_status.plan_num);
        cout << endl;  
      }
      if (logFile)
      {
        int fileIdx = findFileIdx(logtypes,"tci");
        if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
        {
          *(logFiles[fileIdx].OS) << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
          *(logFiles[fileIdx].OS) << ", starting, signal is running mode " << static_cast<int>(controller_status.mode);
          if (controller_status.mode == operation_mode_enum_t::COORDINATION)
            *(logFiles[fileIdx].OS) << ", coordination plan_num " << static_cast<int>(timingcard.coordplans[controller_status.coordplan_index].plan_num);
          else
            *(logFiles[fileIdx].OS) << ", plan_num " << static_cast<int>(controller_status.signal_status.plan_num);
          *(logFiles[fileIdx].OS) << endl;
          logFiles[fileIdx].logrows++;
        }  
      }
    }
    
    if(poll_trace.pollTimeCard || !controller_status.isPlantimingReady)
      continue;
    
    /// finished all polls
    timeUtils::getFullTimeStamp(fullTimeStamp);
    if (isNewSpat)
    {      
      controller_status.tms = fullTimeStamp.tms;
      for (uint8_t ring = 0; ring < 2; ring++)
      {
        if (signal_status_mess.active_phases[ring] > 0)
          controller_status.active_force_off[signal_status_mess.active_phases[ring] - 1] = signal_status_mess.active_force_off[ring];
      }
      /// trace pattern_num change
      if (signal_status_mess.pattern_num != controller_status.signal_status.pattern_num)
      {
        /// reset control plan parameters
        controller_status.isPlantimingReady = false;
        controller_status.mode = get_control_mode(controller_status.status,signal_status_mess.preempt,signal_status_mess.pattern_num);
        if (!getPatnIdx(controller_status.coordplan_index,controller_status.mode,signal_status_mess.plan_num,timingcard.plannum2planindex_map))
        {
          /// should not be here as all plans have been polled
          OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
          OS_ERR << ", plan change, failed find coordplan_index in timingcard for plan_num " << static_cast<int>(signal_status_mess.plan_num);
          OS_ERR << ", re-poll coord plans" << endl;
          /// poll coordination plans
          timingcard.resetplans(); 
          resetPlanPolls(poll_list);
          poll_trace.resetpolls();
          continue;
        }
        /// reset permitted_phases & permitted_ped_phases
        updatePermitPhases(controller_status.permitted_phases,controller_status.permitted_ped_phases,
          controller_status.mode,controller_status.coordplan_index,timingcard);
        /// reset phase recall_status
        for (uint8_t i = 0; i < 8; i++)
        {
          if (controller_status.permitted_phases.test(i))
          {
            controller_status.phase_status[i].recall_status = operation_mode_enum_t::NORECALL;
            if (controller_status.mode == operation_mode_enum_t::RUNNINGFREE)
              controller_status.phase_status[i].recall_status = getPhaseRecallType(timingcard.phaseflags,timingcard.freeplan,i);            
            else if (controller_status.coordplan_index >= 0)
              controller_status.phase_status[i].recall_status = getPhaseRecallType(timingcard.phaseflags,timingcard.coordplans[controller_status.coordplan_index],i);
          }
        }
        controller_status.isPlantimingReady = true;
        if (logFile)
        {
          int fileIdx = findFileIdx(logtypes,"tci");
          if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
          {
            *(logFiles[fileIdx].OS) << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
            *(logFiles[fileIdx].OS) << ", control pattern_num changed from " << static_cast<int>(controller_status.signal_status.pattern_num);
            *(logFiles[fileIdx].OS) << " to " << static_cast<int>(signal_status_mess.pattern_num);
            *(logFiles[fileIdx].OS) << ", signal is running mode " << static_cast<int>(controller_status.mode);
            *(logFiles[fileIdx].OS) << endl;
            logFiles[fileIdx].logrows++;
          }
        }                
      }      
      /// trace barrier change
      uint8_t curbarrier = barrier_phases_on(signal_status_mess.active_phase);
      if (curbarrier != controller_status.curbarrier)
      {
        controller_status.curbarrier = curbarrier;
        controller_status.curbarrier_start_time = fullTimeStamp.tms;
      }
      /// trace local_cycle_clock change (local_cycle_clock stays 0 when runnung free)
      if (signal_status_mess.local_cycle_clock != controller_status.signal_status.local_cycle_clock)
      {
        controller_status.cycle_clock_time = fullTimeStamp.tms;
      }
      /// trace cycle change (under coordination)
      if (signal_status_mess.local_cycle_clock < controller_status.signal_status.local_cycle_clock && signal_status_mess.local_cycle_clock < 3)
      {
        controller_status.cycle_start_time = fullTimeStamp.tms - (long long)(signal_status_mess.local_cycle_clock) * 1000LL;        
      }
      /// trace active interval countdown timer change
      for (uint8_t ring = 0; ring < 2; ring++)
      {
        if (signal_status_mess.active_phases[ring] != controller_status.signal_status.active_phases[ring]
          || signal_status_mess.active_interval[ring] != controller_status.signal_status.active_interval[ring]
          || signal_status_mess.interval_timer[ring] != controller_status.signal_status.interval_timer[ring])
        {
          controller_status.timer_time[ring] = fullTimeStamp.tms;
        }
      }
      /// trace phase_status change
      for (uint8_t i = 1; i <= 8; i++)
      {
        if (controller_status.permitted_phases.test(i-1))
        {
          operation_mode_enum_t::lightcolor state = get_light_color(signal_status_mess.active_phases[ring_phase_on(i)],signal_status_mess.active_interval[ring_phase_on(i)],i);
          /// state & state_start_time
          if (state != controller_status.phase_status[i-1].state)
          {
            controller_status.phase_status[i-1].state = state;
            controller_status.phase_status[i-1].state_start_time = fullTimeStamp.tms;
          }
          /// call_status
          controller_status.phase_status[i-1].call_status = operation_mode_enum_t::NOCALL;
          if (signal_status_mess.veh_call.test(i-1))
            controller_status.phase_status[i-1].call_status = operation_mode_enum_t::CALLVEH;
          /// ped_call has higher priority than veh_call
          if (signal_status_mess.ped_call.test(i-1))
            controller_status.phase_status[i-1].call_status = operation_mode_enum_t::CALLPED;        
        }
        if (controller_status.permitted_ped_phases.test(i-1))
        {
          operation_mode_enum_t::pedsign pedstate = get_ped_state(signal_status_mess.active_phases[ring_phase_on(i)],signal_status_mess.active_interval[ring_phase_on(i)],i);
          /// pedstate & pedstate_start_time
          if (pedstate != controller_status.phase_status[i-1].pedstate)
          {
            controller_status.phase_status[i-1].pedstate = pedstate;
            controller_status.phase_status[i-1].pedstate_start_time = fullTimeStamp.tms;
          }
        }
      }
      
      /// update controller_status.phase_status.time2next & pedtime2next (no need for red flashing mode)
      for (uint8_t i = 0; i < 8; i++)
      {
        controller_status.phase_status[i].resetTime2next();
      }
      /// phase_status.time2next
      predicted_bound_t time2start[2]; // phase green onset by ring
      time2start[0].reset();
      time2start[1].reset();
      switch(controller_status.mode)
      {
      case operation_mode_enum_t::RUNNINGFREE:
        {
          /// when running free, there is no local_cycle_clock (stays at 0) and no force-off logic
          /// phase green is terminated by either gap-out or max-out
          freeplan_mess_t* pfreeplan = &timingcard.freeplan;
          /// start with active phases
          for (uint8_t ring = 0; ring < 2; ring++)
          {
            uint8_t phaseOnRing = signal_status_mess.active_phases[ring];
            if (phaseOnRing > 0)
            {
              updateActivePhaseTime2next(controller_status.phase_status[phaseOnRing-1],time2start[ring],signal_status_mess,timingcard.phasetiming[phaseOnRing-1],
                timingcard.phaseflags,ring,controller_status.timer_time[ring],fullTimeStamp.tms);              
            }
          }
          /// determine startbarrier & startphases for moving barrier-to-barrier, phase-to-phase
          uint8_t startbarrier = curbarrier;
          uint8_t startphases[2] = {signal_status_mess.active_phases[0],signal_status_mess.active_phases[1]};
          /// when next_phase is on (active phase in yellow or red clearance), update time2next for next_phase & time2start for the phases after
          if (signal_status_mess.next_phase.any())
          {
            /// next_phase is on when at least one active phases is in yellow or red clearance
            uint8_t nextbarrier = barrier_phases_on(signal_status_mess.next_phase);
            if (nextbarrier != curbarrier)
            {
              barrierCrossAdjust(time2start);
              startbarrier = nextbarrier;
            }
            for (uint8_t ring = 0; ring < 2; ring++)
            {
              startphases[ring] = signal_status_mess.next_phases[ring];
              if (startphases[ring] > 0)
              {
                /// update time2next (i.e., red to green) for next_phases 
                if (controller_status.phase_status[startphases[ring]-1].state == operation_mode_enum_t::RED)
                {
                  controller_status.phase_status[startphases[ring]-1].time2next.bound_L = time2start[ring].bound_L;
                  controller_status.phase_status[startphases[ring]-1].time2next.bound_U = time2start[ring].bound_U;
                  controller_status.phase_status[startphases[ring]-1].time2next.confidence = SPAT_StateConfidence_maxTime;
                }
                /// next_phase should be on
                if (controller_status.phase_status[startphases[ring]-1].call_status == operation_mode_enum_t::NOCALL)
                {
                  controller_status.phase_status[startphases[ring]-1].call_status = operation_mode_enum_t::CALLVEH;
                }  
                /// update time2start for phases after the next_phase
                getNextPhaseStartBound(time2start[ring],timingcard.phasetiming[startphases[ring]-1],timingcard.phaseflags,
                  (uint8_t)(startphases[ring]-1),controller_status.phase_status[startphases[ring]-1]);
              }
            }
          }
          /// phase after startphases and on startbarrier & barrier cross
          for (uint8_t ring = 0; ring < 2; ring++)
          {
            if (startphases[ring] > 0 && !pfreeplan->lag_phases.test(startphases[ring]-1))
            {
              uint8_t lagphase = pfreeplan->leadlag_phases[startbarrier][ring][1];
              if (lagphase > 0 && lagphase != startphases[ring])
              {
                controller_status.phase_status[lagphase-1].time2next.bound_L = time2start[ring].bound_L;
                controller_status.phase_status[lagphase-1].time2next.bound_U = time2start[ring].bound_U;
                controller_status.phase_status[lagphase-1].time2next.confidence = SPAT_StateConfidence_minTime;              
                getNextPhaseStartBound(time2start[ring],timingcard.phasetiming[lagphase-1],timingcard.phaseflags,
                  (uint8_t)(lagphase-1),controller_status.phase_status[lagphase-1]);
              }
            }
          }
          barrierCrossAdjust(time2start);
          if (startbarrier == curbarrier)
          {
            /// for phases on the next barrier
            for (uint8_t ring = 0; ring < 2; ring++)
            {
              uint8_t leadphase = pfreeplan->leadlag_phases[next_barrier(startbarrier)][ring][0];
              uint8_t lagphase = pfreeplan->leadlag_phases[next_barrier(startbarrier)][ring][1];
              if (leadphase > 0)
              {
                controller_status.phase_status[leadphase-1].time2next.bound_L = time2start[ring].bound_L;
                controller_status.phase_status[leadphase-1].time2next.bound_U = time2start[ring].bound_U;
                controller_status.phase_status[leadphase-1].time2next.confidence = SPAT_StateConfidence_minTime;                          
                getNextPhaseStartBound(time2start[ring],timingcard.phasetiming[leadphase-1],timingcard.phaseflags,
                  (uint8_t)(leadphase-1),controller_status.phase_status[leadphase-1]);
              }
              if (lagphase > 0 && lagphase != leadphase)
              {
                controller_status.phase_status[lagphase-1].time2next.bound_L = time2start[ring].bound_L;
                controller_status.phase_status[lagphase-1].time2next.bound_U = time2start[ring].bound_U;
                controller_status.phase_status[lagphase-1].time2next.confidence = SPAT_StateConfidence_minTime;                          
                getNextPhaseStartBound(time2start[ring],timingcard.phasetiming[lagphase-1],timingcard.phaseflags,
                  (uint8_t)(lagphase-1),controller_status.phase_status[lagphase-1]);
              }
            }  
            barrierCrossAdjust(time2start);
            /// for remaining phases on the startbarrier (i.e. curbarrier)
            for (uint8_t ring = 0; ring < 2; ring++)
            {
              uint8_t leadphase = pfreeplan->leadlag_phases[startbarrier][ring][0];
              uint8_t lagphase = pfreeplan->leadlag_phases[startbarrier][ring][1];
              if (leadphase > 0 && controller_status.phase_status[leadphase-1].time2next.confidence == SPAT_StateConfidence_unKnownEstimate)
              {
                controller_status.phase_status[leadphase-1].time2next.bound_L = time2start[ring].bound_L;
                controller_status.phase_status[leadphase-1].time2next.bound_U = time2start[ring].bound_U;
                controller_status.phase_status[leadphase-1].time2next.confidence = SPAT_StateConfidence_minTime;                                      
                getNextPhaseStartBound(time2start[ring],timingcard.phasetiming[leadphase-1],timingcard.phaseflags,
                  (uint8_t)(leadphase-1),controller_status.phase_status[leadphase-1]);
              }
              if (lagphase > 0 && lagphase != leadphase && controller_status.phase_status[lagphase-1].time2next.confidence == SPAT_StateConfidence_unKnownEstimate)
              {
                controller_status.phase_status[lagphase-1].time2next.bound_L = time2start[ring].bound_L;
                controller_status.phase_status[lagphase-1].time2next.bound_U = time2start[ring].bound_U;
                controller_status.phase_status[lagphase-1].time2next.confidence = SPAT_StateConfidence_minTime;                                      
                getNextPhaseStartBound(time2start[ring],timingcard.phasetiming[lagphase-1],timingcard.phaseflags,
                  (uint8_t)(lagphase-1),controller_status.phase_status[lagphase-1]);
              }
            }
          }
          else
          {
            /// for remaining phases on the curbarrier
            for (uint8_t ring = 0; ring < 2; ring++)
            {
              uint8_t leadphase = pfreeplan->leadlag_phases[curbarrier][ring][0];
              uint8_t lagphase = pfreeplan->leadlag_phases[curbarrier][ring][1];
              if (leadphase > 0 && controller_status.phase_status[leadphase-1].time2next.confidence == SPAT_StateConfidence_unKnownEstimate)
              {
                controller_status.phase_status[leadphase-1].time2next.bound_L = time2start[ring].bound_L;
                controller_status.phase_status[leadphase-1].time2next.bound_U = time2start[ring].bound_U;
                controller_status.phase_status[leadphase-1].time2next.confidence = SPAT_StateConfidence_minTime;                                      
                getNextPhaseStartBound(time2start[ring],timingcard.phasetiming[leadphase-1],timingcard.phaseflags,
                  (uint8_t)(leadphase-1),controller_status.phase_status[leadphase-1]);
              }
              if (lagphase > 0 && lagphase != leadphase && controller_status.phase_status[lagphase-1].time2next.confidence == SPAT_StateConfidence_unKnownEstimate)
              {
                controller_status.phase_status[lagphase-1].time2next.bound_L = time2start[ring].bound_L;
                controller_status.phase_status[lagphase-1].time2next.bound_U = time2start[ring].bound_U;
                controller_status.phase_status[lagphase-1].time2next.confidence = SPAT_StateConfidence_minTime;                                      
                getNextPhaseStartBound(time2start[ring],timingcard.phasetiming[lagphase-1],timingcard.phaseflags,
                  (uint8_t)(lagphase-1),controller_status.phase_status[lagphase-1]);
              }
            }            
          }
        }
        break;
      case operation_mode_enum_t::COORDINATION:
        {
          /// when under coordination, phase green is terminated by gap-out, force-off or max-out        
          coordplan_mess_t* pcoordplan = &timingcard.coordplans[controller_status.coordplan_index];
          /// get the cur_local_cycle_clock and cycle_length (in deciseconds)
          uint16_t cycle_length = static_cast<uint16_t>(pcoordplan->cycle_length * 10);
          uint16_t cur_local_cycle_clock = static_cast<uint16_t>((fullTimeStamp.tms + 30 - controller_status.cycle_clock_time)/100LL 
            + signal_status_mess.local_cycle_clock * 10); 
          /// 0 < cur_local_cycle_clock <= cycle_length
          cur_local_cycle_clock = static_cast<uint16_t>((cur_local_cycle_clock > cycle_length) ? (cur_local_cycle_clock - cycle_length) : cur_local_cycle_clock);
          /// get concurrent phase combination type (MINOR_MINOR, MINOR_MAJOR, or MAJOR_MAJOR)
          operation_mode_enum_t::concurrentphasetype concurrentType = getConcurrentPhaseType(signal_status_mess.active_phase,pcoordplan->sync_phases);
          /// active_force_off adjustment
          if (concurrentType == operation_mode_enum_t::MINOR_MAJOR)
          {
            /// lagging MINOR_MAJOR should have the same force-off point
            if (pcoordplan->lag_phases.test(signal_status_mess.active_phases[0]-1) && signal_status_mess.active_force_off[0] != signal_status_mess.active_force_off[1])
            {
              signal_status_mess.active_force_off[0] = (signal_status_mess.active_force_off[0] > signal_status_mess.active_force_off[1]) ?
                signal_status_mess.active_force_off[1] : signal_status_mess.active_force_off[0];
              signal_status_mess.active_force_off[1] = signal_status_mess.active_force_off[0];
            }
          }
          else if (concurrentType == operation_mode_enum_t::MAJOR_MAJOR)
          {
            /// both coordinated phases have passed the yield point, they should have the same force-off point
            if (pcoordplan->sync_phases.count() == 2 && signal_status_mess.active_force_off[0] > 0 && signal_status_mess.active_force_off[1] > 0
              && signal_status_mess.active_force_off[0] != signal_status_mess.active_force_off[1])
            {
              signal_status_mess.active_force_off[0] = (signal_status_mess.active_force_off[0] > signal_status_mess.active_force_off[1]) ?
                signal_status_mess.active_force_off[1] : signal_status_mess.active_force_off[0];
              signal_status_mess.active_force_off[1] = signal_status_mess.active_force_off[0];            
            }          
          }
          /// coordinated phases should be on
          for (uint8_t ring = 0; ring < 2; ring++)
          {
            if (signal_status_mess.active_phases[ring] == pcoordplan->coordinated_phases[ring] && signal_status_mess.active_phases[ring] > 0
              && controller_status.phase_status[signal_status_mess.active_phases[ring]-1].call_status == operation_mode_enum_t::NOCALL)
            {
              controller_status.phase_status[signal_status_mess.active_phases[ring]-1].call_status = operation_mode_enum_t::CALLVEH;
            }
          }
          /// phase_status.time2next: start with active phases
          for (uint8_t ring = 0; ring < 2; ring++)
          {
            uint8_t phaseOnRing = signal_status_mess.active_phases[ring];
            if (phaseOnRing > 0)
            {
              updateActivePhaseTime2next(controller_status.phase_status[phaseOnRing-1],time2start[ring],pcoordplan,signal_status_mess,
                timingcard.phasetiming[phaseOnRing-1],timingcard.phaseflags,ring,cur_local_cycle_clock,cycle_length,concurrentType,
                controller_status.timer_time[ring],fullTimeStamp.tms);              
            }
          }
          /// determine startbarrier & startphases for moving barrier-to-barrier, phase-to-phase
          uint8_t startbarrier = curbarrier;
          uint8_t startphases[2] = {signal_status_mess.active_phases[0],signal_status_mess.active_phases[1]};
          /// when next_phase is on (active phase in yellow or red clearance), update time2next for next_phase & time2start for the phases after
          if (signal_status_mess.next_phase.any())
          {
            /// next_phase is on when at least one active phases is in yellow or red clearance
            uint8_t nextbarrier = barrier_phases_on(signal_status_mess.next_phase);
            if (nextbarrier != curbarrier)
            {
              barrierCrossAdjust(time2start);
              startbarrier = nextbarrier;
            }
            for (uint8_t ring = 0; ring < 2; ring++)
            {
              startphases[ring] = signal_status_mess.next_phases[ring];
              if (startphases[ring] > 0)
              {
                /// update time2next (i.e., red to green) for next_phases 
                if (controller_status.phase_status[startphases[ring]-1].state == operation_mode_enum_t::RED)
                {
                  controller_status.phase_status[startphases[ring]-1].time2next.bound_L = time2start[ring].bound_L;
                  controller_status.phase_status[startphases[ring]-1].time2next.bound_U = time2start[ring].bound_U;
                  controller_status.phase_status[startphases[ring]-1].time2next.confidence = SPAT_StateConfidence_maxTime;
                }
                /// next_phase should be on
                if (controller_status.phase_status[startphases[ring]-1].call_status == operation_mode_enum_t::NOCALL)
                {
                  controller_status.phase_status[startphases[ring]-1].call_status = operation_mode_enum_t::CALLVEH;
                }
                /// update time2start for phases after the next_phase
                getNextPhaseStartBound(time2start[ring],timingcard.phasetiming[startphases[ring]-1],timingcard.phaseflags,(uint8_t)(startphases[ring]-1),
                  controller_status.phase_status[startphases[ring]-1],pcoordplan,cur_local_cycle_clock,cycle_length);                
              }
            }
          }
          /// phase after startphases and on startbarrier & barrier cross
          for (uint8_t ring = 0; ring < 2; ring++)
          {
            if (startphases[ring] > 0 && !pcoordplan->lag_phases.test(startphases[ring]-1))
            {
              uint8_t lagphase = pcoordplan->leadlag_phases[startbarrier][ring][1];
              if (lagphase > 0 && lagphase != startphases[ring])
              {
                controller_status.phase_status[lagphase-1].time2next.bound_L = time2start[ring].bound_L;
                controller_status.phase_status[lagphase-1].time2next.bound_U = time2start[ring].bound_U;
                controller_status.phase_status[lagphase-1].time2next.confidence = SPAT_StateConfidence_minTime;              
                getNextPhaseStartBound(time2start[ring],timingcard.phasetiming[lagphase-1],timingcard.phaseflags,(uint8_t)(lagphase-1),
                  controller_status.phase_status[lagphase-1],pcoordplan,cur_local_cycle_clock,cycle_length);
              }
            }
          }
          barrierCrossAdjust(time2start);
          if (startbarrier == curbarrier)
          {
            /// for phases on the next barrier
            for (uint8_t ring = 0; ring < 2; ring++)
            {
              uint8_t leadphase = pcoordplan->leadlag_phases[next_barrier(startbarrier)][ring][0];
              uint8_t lagphase = pcoordplan->leadlag_phases[next_barrier(startbarrier)][ring][1];
              if (leadphase > 0)
              {
                controller_status.phase_status[leadphase-1].time2next.bound_L = time2start[ring].bound_L;
                controller_status.phase_status[leadphase-1].time2next.bound_U = time2start[ring].bound_U;
                controller_status.phase_status[leadphase-1].time2next.confidence = SPAT_StateConfidence_minTime;                          
                getNextPhaseStartBound(time2start[ring],timingcard.phasetiming[leadphase-1],timingcard.phaseflags,(uint8_t)(leadphase-1),
                  controller_status.phase_status[leadphase-1],pcoordplan,cur_local_cycle_clock,cycle_length);
              }
              if (lagphase > 0 && lagphase != leadphase)
              {
                controller_status.phase_status[lagphase-1].time2next.bound_L = time2start[ring].bound_L;
                controller_status.phase_status[lagphase-1].time2next.bound_U = time2start[ring].bound_U;
                controller_status.phase_status[lagphase-1].time2next.confidence = SPAT_StateConfidence_minTime;                          
                getNextPhaseStartBound(time2start[ring],timingcard.phasetiming[lagphase-1],timingcard.phaseflags,(uint8_t)(lagphase-1),
                  controller_status.phase_status[lagphase-1],pcoordplan,cur_local_cycle_clock,cycle_length);
              }
            }  
            barrierCrossAdjust(time2start);
            /// for remaining phases on the startbarrier (i.e. curbarrier)
            for (uint8_t ring = 0; ring < 2; ring++)
            {
              uint8_t leadphase = pcoordplan->leadlag_phases[startbarrier][ring][0];
              uint8_t lagphase = pcoordplan->leadlag_phases[startbarrier][ring][1];
              if (leadphase > 0 && controller_status.phase_status[leadphase-1].time2next.confidence == SPAT_StateConfidence_unKnownEstimate)
              {
                controller_status.phase_status[leadphase-1].time2next.bound_L = time2start[ring].bound_L;
                controller_status.phase_status[leadphase-1].time2next.bound_U = time2start[ring].bound_U;
                controller_status.phase_status[leadphase-1].time2next.confidence = SPAT_StateConfidence_minTime;                                      
                getNextPhaseStartBound(time2start[ring],timingcard.phasetiming[leadphase-1],timingcard.phaseflags,(uint8_t)(leadphase-1),
                  controller_status.phase_status[leadphase-1],pcoordplan,cur_local_cycle_clock,cycle_length);
              }
              if (lagphase > 0 && lagphase != leadphase && controller_status.phase_status[lagphase-1].time2next.confidence == SPAT_StateConfidence_unKnownEstimate)
              {
                controller_status.phase_status[lagphase-1].time2next.bound_L = time2start[ring].bound_L;
                controller_status.phase_status[lagphase-1].time2next.bound_U = time2start[ring].bound_U;
                controller_status.phase_status[lagphase-1].time2next.confidence = SPAT_StateConfidence_minTime;                                      
                getNextPhaseStartBound(time2start[ring],timingcard.phasetiming[lagphase-1],timingcard.phaseflags,(uint8_t)(lagphase-1),
                  controller_status.phase_status[lagphase-1],pcoordplan,cur_local_cycle_clock,cycle_length);
              }
            }
          }
          else
          {
            /// for remaining phases on the curbarrier
            for (uint8_t ring = 0; ring < 2; ring++)
            {
              uint8_t leadphase = pcoordplan->leadlag_phases[curbarrier][ring][0];
              uint8_t lagphase = pcoordplan->leadlag_phases[curbarrier][ring][1];
              if (leadphase > 0 && controller_status.phase_status[leadphase-1].time2next.confidence == SPAT_StateConfidence_unKnownEstimate)
              {
                controller_status.phase_status[leadphase-1].time2next.bound_L = time2start[ring].bound_L;
                controller_status.phase_status[leadphase-1].time2next.bound_U = time2start[ring].bound_U;
                controller_status.phase_status[leadphase-1].time2next.confidence = SPAT_StateConfidence_minTime;                                      
                getNextPhaseStartBound(time2start[ring],timingcard.phasetiming[leadphase-1],timingcard.phaseflags,(uint8_t)(leadphase-1),
                  controller_status.phase_status[leadphase-1],pcoordplan,cur_local_cycle_clock,cycle_length);
              }
              if (lagphase > 0 && lagphase != leadphase && controller_status.phase_status[lagphase-1].time2next.confidence == SPAT_StateConfidence_unKnownEstimate)
              {
                controller_status.phase_status[lagphase-1].time2next.bound_L = time2start[ring].bound_L;
                controller_status.phase_status[lagphase-1].time2next.bound_U = time2start[ring].bound_U;
                controller_status.phase_status[lagphase-1].time2next.confidence = SPAT_StateConfidence_minTime;                                      
                getNextPhaseStartBound(time2start[ring],timingcard.phasetiming[lagphase-1],timingcard.phaseflags,(uint8_t)(lagphase-1),
                  controller_status.phase_status[lagphase-1],pcoordplan,cur_local_cycle_clock,cycle_length);
              }
            }            
          }
        }
        break;
      default:
        /// do nothing here
        /// 1. type of preemption is included in signal_status_mess.preempt: bits 0-3 <==> EV A-D, bits 4-5 <==> RR 1-2
        ///    should not have RR preemption for CA testbed
        /// 2. no need to do software flashing 
        break;
      }
      
      /// phase_status.pedtime2next
      if (controller_status.mode != operation_mode_enum_t::FLASHING)
      {
        for (uint8_t i = 1; i <= 8; i++)
        {
          if (controller_status.permitted_ped_phases.test(i-1))
          {
            if (controller_status.phase_status[i-1].pedstate != operation_mode_enum_t::DONT_WALK)
            {
              /// WALK or FLASH_DONOT_WALK
              controller_status.phase_status[i-1].pedtime2next.bound_L = getPedIntervalLeft(signal_status_mess.interval_timer[ring_phase_on(i)],
                controller_status.timer_time[ring_phase_on(i)],fullTimeStamp.tms);
              controller_status.phase_status[i-1].pedtime2next.bound_U = controller_status.phase_status[i-1].pedtime2next.bound_L;
              controller_status.phase_status[i-1].pedtime2next.confidence = SPAT_StateConfidence_minTime;
            }
            else if (controller_status.phase_status[i-1].state == operation_mode_enum_t::RED)
            {
              controller_status.phase_status[i-1].pedtime2next.bound_L = controller_status.phase_status[i-1].time2next.bound_L;
              controller_status.phase_status[i-1].pedtime2next.bound_U = controller_status.phase_status[i-1].time2next.bound_U;
              if ((controller_status.phase_status[i-1].recall_status == operation_mode_enum_t::PEDRECALL 
                || controller_status.phase_status[i-1].call_status == operation_mode_enum_t::CALLPED)
                && controller_status.phase_status[i-1].time2next.confidence == SPAT_StateConfidence_maxTime)
              {
                controller_status.phase_status[i-1].pedtime2next.confidence = SPAT_StateConfidence_maxTime;
              }
              else
                controller_status.phase_status[i-1].pedtime2next.confidence = SPAT_StateConfidence_minTime;
            }  
            else if (controller_status.phase_status[i-1].state == operation_mode_enum_t::YELLOW && i == signal_status_mess.next_phases[ring_phase_on(i)])
            {
              uint32_t red_clearance = ((timingcard.phaseflags.red_revert_interval > timingcard.phasetiming[i-1].red_clearance) ?
                timingcard.phaseflags.red_revert_interval : timingcard.phasetiming[i-1].red_clearance);
              controller_status.phase_status[i-1].pedtime2next.bound_L = static_cast<uint16_t>(controller_status.phase_status[i-1].time2next.bound_L + red_clearance);
              controller_status.phase_status[i-1].pedtime2next.bound_U = static_cast<uint16_t>(controller_status.phase_status[i-1].time2next.bound_U + red_clearance);
              if (controller_status.phase_status[i-1].recall_status == operation_mode_enum_t::PEDRECALL 
                || controller_status.phase_status[i-1].call_status == operation_mode_enum_t::CALLPED)
              {
                controller_status.phase_status[i-1].pedtime2next.confidence = SPAT_StateConfidence_maxTime;
              }
              else
                controller_status.phase_status[i-1].pedtime2next.confidence = SPAT_StateConfidence_minTime;
            }
            else
            {
              controller_status.phase_status[i-1].pedtime2next.bound_U = time2start[ring_phase_on(i)].bound_U;
              controller_status.phase_status[i-1].pedtime2next.bound_L = time2start[ring_phase_on(i)].bound_L;
              controller_status.phase_status[i-1].pedtime2next.confidence = SPAT_StateConfidence_minTime;
            }
          }
        }
      }
      
      /// update controller_status.signal_status
      memcpy(&controller_status.signal_status,&signal_status_mess,sizeof(signal_status_mess));
      
      /// send to MRP_DataMgr
      sendSigStaMsg(fd_mgrSend,sendbuf_socket,controller_status,timingcard.manualplan.planOn);
           
      /// update softcall_state with current phase_status
      updateSoftcallState(softcall_state,&controller_status.phase_status[0]);
    }
   
    /// check sending softcall
    if (fullTimeStamp.tms > softcall_state.tms + softcall_interval)
    {
      if (softcall_state.ped_call.any() || softcall_state.veh_call.any() || softcall_state.veh_ext.any()
        || softcall_state.prio_call.any() || softcall_state.prio_ext.any())
      {
        bitset<8> pedCallPhases;
        bitset<8> vehCallPahses;
        bitset<8> prioCallPhases;
        for (int i = 0; i < 8; i++)
        {
          if (softcall_state.ped_call.test(i))
            pedCallPhases.set(i);
          if (softcall_state.veh_call.test(i) && controller_status.phase_status[i].state != operation_mode_enum_t::GREEN)
            vehCallPahses.set(i);
          if (softcall_state.veh_ext.test(i) && controller_status.phase_status[i].state == operation_mode_enum_t::GREEN)
            vehCallPahses.set(i);
          if (softcall_state.prio_call.test(i) && controller_status.phase_status[i].state != operation_mode_enum_t::GREEN)
            prioCallPhases.set(i);
          if (softcall_state.prio_ext.test(i) && controller_status.phase_status[i].state == operation_mode_enum_t::GREEN)
            prioCallPhases.set(i);
        }
        if (pedCallPhases.any() || vehCallPahses.any() || prioCallPhases.any())
        {
          bytesrequested2write = formsoftcallrequest(sendbuf_spat2,timingcard.controller_addr,vehCallPahses,pedCallPhases,prioCallPhases);
          if (sendControl2controller)
          {
            if (write(fd_spat2,(uint8_t*)&sendbuf_spat2,bytesrequested2write) > 0)
            {
              softcall_state.ped_call.reset();
              softcall_state.tms = fullTimeStamp.tms;
              if (verbose)
              {
                cout << timeUtils::getTimestampStr(signal_status_mess.fullTimeStamp.localDateTimeStamp);
                cout << " send softcall to controller: ped_call=" << pedCallPhases.to_string();
                cout << " veh_call=" << vehCallPahses.to_string();
                cout << " prio_call=" << prioCallPhases.to_string() << endl;
              }
              if (logFile)
              {
                int fileIdx = findFileIdx(logtypes,"req");
                if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
                {
                  *(logFiles[fileIdx].OS) << timeUtils::getTimestampStr(signal_status_mess.fullTimeStamp.localDateTimeStamp) << ",sent,";
                  *(logFiles[fileIdx].OS) << static_cast<int>(pedCallPhases.to_ulong()) << ",";
                  *(logFiles[fileIdx].OS) << static_cast<int>(vehCallPahses.to_ulong()) << ",";   
                  *(logFiles[fileIdx].OS) << static_cast<int>(prioCallPhases.to_ulong()) << endl;
                  logFiles[fileIdx].logrows++;
                }
              }
            }
            else
            {
              OS_ERR << timeUtils::getTimestampStr(signal_status_mess.fullTimeStamp.localDateTimeStamp);
              OS_ERR << ", failed sending softcall to controller: ped_call=" << pedCallPhases.to_string();
              OS_ERR << ", veh_call=" << vehCallPahses.to_string();
              OS_ERR << ", prio_call=" << prioCallPhases.to_string() << endl;
            }
          }    
        }
      }
    }
      
    /// check reopen log files
    if (logFile && fullTimeStamp.tms_minute > logfile_tms_minute + logFileInterval)
    {
      reOpenLogFiles(logFiles,fullTimeStamp.localDateTimeStamp);     
      logfile_tms_minute = fullTimeStamp.tms_minute;
    }
    
    /// sleep
    usleep(usleepms);
  }  
}

void buildPollList(std::vector<poll_conf_t>& list)
{  
  list.push_back(poll_conf_t(1,string("phase flags"),0x33,0x87,2,1,0,0xC7,0xE7,true));
  list.push_back(poll_conf_t(1,string("ped flags"),0x33,0x87,2,3,0,0xC7,0xE7,true));
  list.push_back(poll_conf_t(2,string("red revert"),0x33,0x89,0x72,0x00,1,0xC9,0xE9,true));
  list.push_back(poll_conf_t(1,string("phase timing 1"),0x33,0x87,3,1,0,0xC7,0xE7,true));
  list.push_back(poll_conf_t(1,string("phase timing 2"),0x33,0x87,3,2,0,0xC7,0xE7,true));
  list.push_back(poll_conf_t(1,string("phase timing 3"),0x33,0x87,3,3,0,0xC7,0xE7,true));
  list.push_back(poll_conf_t(1,string("phase timing 4"),0x33,0x87,3,4,0,0xC7,0xE7,true));
  list.push_back(poll_conf_t(1,string("phase timing 5"),0x33,0x87,3,5,0,0xC7,0xE7,true));
  list.push_back(poll_conf_t(1,string("phase timing 6"),0x33,0x87,3,6,0,0xC7,0xE7,true));
  list.push_back(poll_conf_t(1,string("phase timing 7"),0x33,0x87,3,7,0,0xC7,0xE7,true));
  list.push_back(poll_conf_t(1,string("phase timing 8"),0x33,0x87,3,8,0,0xC7,0xE7,true));
  list.push_back(poll_conf_t(1,string("free plan"),0x33,0x87,4,10,0,0xC7,0xE7,true));
  list.push_back(poll_conf_t(1,string("CIC plan"),0x33,0x87,7,13,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("detector group 1"),0x33,0x87,7,1,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("detector group 2"),0x33,0x87,7,2,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("detector group 3"),0x33,0x87,7,3,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("detector group 4"),0x33,0x87,7,4,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("detector group 5"),0x33,0x87,7,5,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("detector group 6"),0x33,0x87,7,6,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("detector group 7"),0x33,0x87,7,7,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("detector group 8"),0x33,0x87,7,8,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("detector group 9"),0x33,0x87,7,9,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("detector group 10"),0x33,0x87,7,10,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("detector group 11"),0x33,0x87,7,11,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("system detector"),0x33,0x87,7,12,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("TOD table 1"),0x33,0x87,8,1,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("TOD table 2"),0x33,0x87,8,2,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("TOD table 3"),0x33,0x87,8,3,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("TOD table 4"),0x33,0x87,8,4,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("TOD table 5"),0x33,0x87,8,5,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("TOD table 6"),0x33,0x87,8,6,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("TOD table 7"),0x33,0x87,8,7,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("TOD table 8"),0x33,0x87,8,8,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("TOD table 9"),0x33,0x87,8,9,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("TOD table 10"),0x33,0x87,8,10,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("TOD table 11"),0x33,0x87,8,11,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("TOD table 12"),0x33,0x87,8,12,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("TOD table 13"),0x33,0x87,8,13,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("TOD table 14"),0x33,0x87,8,14,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("TOD table 15"),0x33,0x87,8,15,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("TOD table 16"),0x33,0x87,8,16,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("TOD table 17"),0x33,0x87,8,17,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("TOD table 18"),0x33,0x87,8,18,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("TOD table 19"),0x33,0x87,8,19,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("TOD table 20"),0x33,0x87,8,20,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("TOD table 21"),0x33,0x87,8,21,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("TOD table 22"),0x33,0x87,8,22,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("TOD table 23"),0x33,0x87,8,23,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("TOD table 24"),0x33,0x87,8,24,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("weekday"),0x33,0x87,8,25,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("TOD Function 1"),0x33,0x87,9,6,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("TOD Function 2"),0x33,0x87,9,7,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("TOD Function 3"),0x33,0x87,9,8,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("TOD Function 4"),0x33,0x87,9,9,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("EVA"),0x33,0x87,11,11,0,0xC7,0xE7,true));
  list.push_back(poll_conf_t(1,string("EVB"),0x33,0x87,11,12,0,0xC7,0xE7,true));
  list.push_back(poll_conf_t(1,string("EVC"),0x33,0x87,11,13,0,0xC7,0xE7,true));
  list.push_back(poll_conf_t(1,string("EVD"),0x33,0x87,11,14,0,0xC7,0xE7,true));
  list.push_back(poll_conf_t(1,string("RR1 phase flags"),0x33,0x87,11,1,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("RR1 ped flags"),0x33,0x87,11,2,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("RR1 overlap flags"),0x33,0x87,11,3,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("RR1 exit parameters"),0x33,0x87,11,4,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("RR1 Configuration"),0x33,0x87,11,5,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("RR2 phase flags"),0x33,0x87,11,6,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("RR2 ped flags"),0x33,0x87,11,7,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("RR2 overlap flags"),0x33,0x87,11,8,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("RR2 exit parameters"),0x33,0x87,11,9,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("RR2 Configuration"),0x33,0x87,11,10,0,0xC7,0xE7,false));  
  list.push_back(poll_conf_t(1,string("TSP enable plans"),0x33,0x87,13,8,0,0xC7,0xE7,true));
  list.push_back(poll_conf_t(1,string("TSP plan group 1"),0x33,0x87,13,2,0,0xC7,0xE7,true));  // plan 1 - 3
  list.push_back(poll_conf_t(1,string("TSP plan group 2"),0x33,0x87,13,3,0,0xC7,0xE7,true));  // plan 4 - 6
  list.push_back(poll_conf_t(1,string("TSP plan group 3"),0x33,0x87,13,4,0,0xC7,0xE7,true));  // plan 7 - 9
  list.push_back(poll_conf_t(1,string("TSP plan group 4"),0x33,0x87,13,5,0,0xC7,0xE7,false)); // plan 11 - 13
  list.push_back(poll_conf_t(1,string("TSP plan group 5"),0x33,0x87,13,6,0,0xC7,0xE7,false)); // plan 14 - 16
  list.push_back(poll_conf_t(1,string("TSP plan group 6"),0x33,0x87,13,7,0,0xC7,0xE7,false)); // plan 17 - 19
  list.push_back(poll_conf_t(1,string("coord plan 1"),0x33,0x87,4,1,0,0xC7,0xE7,true));
  list.push_back(poll_conf_t(1,string("coord plan 2"),0x33,0x87,4,2,0,0xC7,0xE7,true));
  list.push_back(poll_conf_t(1,string("coord plan 3"),0x33,0x87,4,3,0,0xC7,0xE7,true));
  list.push_back(poll_conf_t(1,string("coord plan 4"),0x33,0x87,4,4,0,0xC7,0xE7,true));
  list.push_back(poll_conf_t(1,string("coord plan 5"),0x33,0x87,4,5,0,0xC7,0xE7,true));
  list.push_back(poll_conf_t(1,string("coord plan 6"),0x33,0x87,4,6,0,0xC7,0xE7,true));
  list.push_back(poll_conf_t(1,string("coord plan 7"),0x33,0x87,4,7,0,0xC7,0xE7,true));
  list.push_back(poll_conf_t(1,string("coord plan 8"),0x33,0x87,4,8,0,0xC7,0xE7,true));
  list.push_back(poll_conf_t(1,string("coord plan 9"),0x33,0x87,4,9,0,0xC7,0xE7,true));
  list.push_back(poll_conf_t(1,string("coord plan 11"),0x33,0x87,5,1,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("coord plan 12"),0x33,0x87,5,2,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("coord plan 13"),0x33,0x87,5,3,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("coord plan 14"),0x33,0x87,5,4,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("coord plan 15"),0x33,0x87,5,5,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("coord plan 16"),0x33,0x87,5,6,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("coord plan 17"),0x33,0x87,5,7,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("coord plan 18"),0x33,0x87,5,8,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("coord plan 19"),0x33,0x87,5,9,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("coord plan 21"),0x33,0x87,6,1,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("coord plan 22"),0x33,0x87,6,2,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("coord plan 23"),0x33,0x87,6,3,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("coord plan 24"),0x33,0x87,6,4,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("coord plan 25"),0x33,0x87,6,5,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("coord plan 26"),0x33,0x87,6,6,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("coord plan 27"),0x33,0x87,6,7,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("coord plan 28"),0x33,0x87,6,8,0,0xC7,0xE7,false));
  list.push_back(poll_conf_t(1,string("coord plan 29"),0x33,0x87,6,9,0,0xC7,0xE7,false));
}

void resetPlanPolls(std::vector<poll_conf_t>& list)
{
  for (size_t i = 0; i < list.size(); i++)
  {
    if (list[i].poll_desc.find("coord plan") == 0) {list[i].pollReturned = false;}
  }
}

bool isAllPollReturned(const std::vector<poll_conf_t>& poll_list_)
{
  bool retn = true;
  for (std::vector<poll_conf_t>::const_iterator itPolls = poll_list_.begin(); itPolls != poll_list_.end(); ++itPolls)
  {
    if (itPolls->pollRequired && !itPolls->pollReturned)
    {
      retn = false;
      break;
    }
  }
  return retn;
}

int open_port(std::ofstream& OS,const char *port_name,const bool isReadOnly)
{
  struct termios settings;
  speed_t baudrate = B38400;
  int fd;
  int ret;
  
  /// open port
  if (isReadOnly)
    fd = open(port_name, O_RDONLY | O_NOCTTY | O_NDELAY);
  else
    fd = open(port_name, O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd == -1) 
  {
    OS << "Failed to open serial port " << port_name << endl;
    return (-1);
  }
  if (!isatty(fd))
  {
    OS << "This is not a terminal" << endl;
    close_port(fd,isReadOnly);
    return (-1);
  }
  /// get serial interface attributes
  memset(&settings,0,sizeof(settings));
  ret = tcgetattr(fd,&settings);
  if (ret < 0)
  {
    OS << "Failed tcgetattr on serial port " << port_name << endl;
    close_port(fd,isReadOnly);
    return (-1);
  }
  /// set to raw mode
  cfmakeraw(&settings);
    /*  c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
        c_oflag &= ~OPOST
        c_lflag &= ~(ICANON | ECHO | ECHONL | ISIG | IEXTEN)
        c_cflag &= ~(CSIZE | PARENB)
        c_cflag |= CS8
    */
  /// disable input parity checking
  settings.c_iflag &= ~INPCK;
  /// disable replacing output line feeds with carriage return and line feed
  settings.c_oflag &= ~ONLCR;
  /// ignore modem control lines, enable receiver
  settings.c_cflag |= (CLOCAL | CREAD);
  /// ignore modem disconnect, one stop bit, no flow control
        settings.c_cflag &= ~(HUPCL | CSTOPB | CRTSCTS);
  /// immediate read return
  settings.c_cc[VMIN] = 1;
  settings.c_cc[VTIME] = 0;
  /// input/output baud rate
  ret = cfsetspeed(&settings,baudrate);   
  if (ret < 0)
  {
    OS << "Failed cfsetspeed for serial port " << port_name << endl;
    close_port(fd,isReadOnly);
    return (-1);
  }
  /// set serial interface attributes
  if (isReadOnly)
    tcflush(fd,TCIFLUSH);
  else
    tcflush(fd,TCIOFLUSH);
  ret = tcsetattr(fd,TCSANOW,&settings);
  if (ret < 0)
  {
    OS << "Failed tcsetattr for serial port " << port_name << endl;
    close_port(fd,isReadOnly);
    return (-1);
  }
  return fd;
}

void close_port(int fd,const bool isReadOnly)
{
  if (isReadOnly)
    tcflush(fd,TCIFLUSH);
  else
    tcflush(fd,TCIOFLUSH);
  close(fd);
}

void processRecvBuff(uint8_t* buf, size_t& count,vector< pair<size_t, size_t> >& vec)
{
  /// serial port recv buf
  vector<size_t> flagloc;  
  for (size_t i = 0; i < count; i++)
  {
    if (buf[i] == AB3418MSG::flag)
      flagloc.push_back(i);
  }  
  if (flagloc.empty())
  {
    // no 0x7E found, discard bytes in buf
    count = 0;
    return;
  }
  
  size_t flagnums = flagloc.size();
  size_t offset;
  if (flagnums == 1)
  {
    // 1 0x7E found, move to the beginning of buf
    offset = 0;
    for (size_t i = flagloc[0]; i < count; i++)
    {
      buf[offset] = buf[i];
      offset++;
    }
    count = offset;
    return;
  }
  if (flagnums == 2)
  {
    // 2 0x7E found, two cases: ...0x7E 0x7E ... and ...0x7E...0x7E
    if (flagloc[1] - flagloc[0] == 1)
    {
      // case 1, no completed ab3418 frame found, move 2nd 0x7E to the beginning of buf
      offset = 0;
      for (size_t i = flagloc[1]; i < count; i++)
      {
        buf[offset] = buf[i];
        offset++;
      }
      count = offset;
      return;
    }
    else  
    {
      // case 2, found one completed ab3418 frame
      vec.push_back(make_pair(flagloc[0],flagloc[1]));
      return;
    }
  }
  // found at least 3 0x7E, there must be a pair of consective 0x7Es
  bool foundpair = false;
  size_t pairIndex;
  for (size_t i = 0; i < flagnums-1; i++)
  {
    if (flagloc[i+1] - flagloc[i] == 1)
    {
      foundpair = true;
      pairIndex = i;  // this is the end of a completed ab3418 frame
      break;
    }
  }
  if(!foundpair)
  {
    // error in received buff, move the last 0x7E to the beginning of buf
    offset = 0;
    for (size_t i = flagloc[flagnums-1]; i < count; i++)
    {
      buf[offset] = buf[i];
      offset++;
    }
    count = offset;
    return;
  }
  if (pairIndex > 0)  
  {
    // add the previous completed ab3418 frame to vec
    vec.push_back(make_pair(flagloc[pairIndex-1],flagloc[pairIndex]));
  }
  while(1)
  {
    pairIndex += 2;
    if (pairIndex < flagnums)
    {
      // add the following completed ab3418 frame to vec
      vec.push_back(make_pair(flagloc[pairIndex-1],flagloc[pairIndex]));
    }   
    else
      break;
  }
}

bool processAB3418Msg(std::ofstream& OS,uint8_t* buf,const uint8_t* pmsg,const size_t msglen)
{
  size_t offset_msg = 0;
  size_t offset_buf = 0;
  buf[offset_buf] = pmsg[offset_msg];
  offset_buf++;
  offset_msg++;
  while (1)
  {
    if (offset_buf >= MAXAB3418MSGSIZE)
      return false;
    if (offset_msg == msglen-1)
    {
      buf[offset_buf] = pmsg[offset_msg];
      break;
    }  
    else if (pmsg[offset_msg] == 0x7D)
    {
      if (pmsg[offset_msg+1] == 0x5E)
      {
        buf[offset_buf] = 0x7E;
        offset_buf++;
        offset_msg += 2;
      }
      else if (pmsg[offset_msg+1] == 0x5D)
      {
        buf[offset_buf] = pmsg[offset_msg];
        offset_buf++;
        offset_msg += 2;
      }
      else
        return false;
    }
    else
    {
      buf[offset_buf] = pmsg[offset_msg];
      offset_buf++;
      offset_msg++;
    }
  }   
  /// checksum
  uint16_t oldfcs = static_cast<uint16_t>( ~((uint16_t)buf[offset_buf-2] << 8) | ~(uint16_t)buf[offset_buf-1] );
  uint16_t newfcs = AB3418checksum::pppfcs( oldfcs, &buf[1], (int)(offset_buf-1));
  if (newfcs != AB3418checksum::PPPGOODFCS)
  {
    timeUtils::fullTimeStamp_t ts;
    timeUtils::getFullTimeStamp(ts);
    OS << timeUtils::getTimestampStr(ts.localDateTimeStamp);
    OS << ", FCS error, msg: ";
    hexPrint(OS,(const char*)pmsg,msglen);
    OS << ", buf: ";
    hexPrint(OS,(char*)buf,offset_buf);    
    OS << endl;
    return false;
  }
  return true;
}

string ab3418msgErrCode(const uint8_t err_num)
{
  switch (err_num)
  {
  case 0:
    return string("ERROR_NO_ERROR");
    break;
  case 1:
    return string("ERROR_TOO_BIG");
    break;
  case 2:
    return string("ERROR_NO_SUCH_NAME");
    break;
  case 3:
    return string("ERROR_BAD_VALUE");
    break;
  case 4:
    return string("ERROR_READ_ONLY");
    break;
  case 5:
    return string("ERROR_GEN_ERR");
    break;
  case 6:
    return string("ERROR_MESS_LEN");
    break;
  case 10:
    return string("ERROR_INVALID_PLAN");
    break;
  case 11:
    return string("ERROR_INVALID_PACKET_SIZE");
    break;
  case 12:
    return string("ERROR_OUT_OF_RANGE");
    break;
  default:
    return string("ERROR_UNKNOWN_MSG");
  }
}

void parseSignalStatusMsg(signal_status_mess_t& signalstatus,const uint8_t* pmsgbuf)
{             
  timeUtils::getFullTimeStamp(signalstatus.fullTimeStamp);
  signalstatus.controller_addr = pmsgbuf[1];
  int offset = 5;
  signalstatus.active_phase = bitset<8>(pmsgbuf[offset++]);
  bitsetphases2ringphases(signalstatus.active_phases,signalstatus.active_phase);
  signalstatus.active_interval[0] = pmsgbuf[offset++];
  signalstatus.active_interval[1] = pmsgbuf[offset++];
  signalstatus.interval_timer[0] = pmsgbuf[offset++];
  signalstatus.interval_timer[1] = pmsgbuf[offset++];
  signalstatus.next_phase = bitset<8>(pmsgbuf[offset++]);
  bitsetphases2ringphases(signalstatus.next_phases,signalstatus.next_phase);
  signalstatus.ped_call = bitset<8>(pmsgbuf[offset++]);
  signalstatus.veh_call = bitset<8>(pmsgbuf[offset++]);
  signalstatus.pattern_num = pmsgbuf[offset++];
  pattern2planOffset(signalstatus.plan_num,signalstatus.offset_index,signalstatus.pattern_num);
  signalstatus.local_cycle_clock = pmsgbuf[offset++];  
  signalstatus.master_cycle_clock = pmsgbuf[offset++];  
  signalstatus.preempt = bitset<8>(pmsgbuf[offset++]);  
  for (int j = 0; j < 8; j++)
  {
    signalstatus.permissive[j] = pmsgbuf[17+j];
    signalstatus.ped_permissive[j] = pmsgbuf[27+j];
  }
  signalstatus.active_force_off[0] = pmsgbuf[25];  
  signalstatus.active_force_off[1] = pmsgbuf[26];       
}

void parseStatus8eMsg(status8e_mess_t& status8e,const uint8_t* pmsgbuf)
{
  timeUtils::getFullTimeStamp(status8e.fullTimeStamp);
  status8e.controller_addr = pmsgbuf[1];
  status8e.hour = pmsgbuf[5];
  status8e.minute = pmsgbuf[6];
  status8e.sec = pmsgbuf[7];
  status8e.flag = bitset<8>(pmsgbuf[8]);
  status8e.status = bitset<8>(pmsgbuf[9]);  
  status8e.pattern_num = pmsgbuf[10];
  // 1-8, 9-16, 17-24, 25-32, 33-40
  status8e.detector_presences = bitset<40>(pmsgbuf[22]);
  for (int j = 21; j> 17; j--)
  {
    (status8e.detector_presences <<= 8) ^= (bitset<40>(pmsgbuf[j]));
  }
  status8e.master_cycle_clock = pmsgbuf[23];
  status8e.local_cycle_clock = pmsgbuf[24];
  status8e.prio_busId = static_cast<uint16_t>(((uint16_t)pmsgbuf[25] << 8) + pmsgbuf[26]);
  status8e.prio_busDirection = pmsgbuf[27];
  status8e.prio_type = pmsgbuf[28];
}

void parseLongStatus8eMsg(longstatus8e_mess_t& longstatus8e,const uint8_t* pmsgbuf)
{
  timeUtils::getFullTimeStamp(longstatus8e.fullTimeStamp);
  longstatus8e.controller_addr = pmsgbuf[1];
  longstatus8e.hour = pmsgbuf[5];
  longstatus8e.minute = pmsgbuf[6];
  longstatus8e.sec = pmsgbuf[7];              
  longstatus8e.flag = bitset<8>(pmsgbuf[8]);
  longstatus8e.status = bitset<8>(pmsgbuf[9]);
  longstatus8e.pattern_num = pmsgbuf[10];
  longstatus8e.master_cycle_clock = pmsgbuf[23];
  longstatus8e.local_cycle_clock = pmsgbuf[24];
  longstatus8e.seq_num = pmsgbuf[25];
  for (int j=0; j<16; j++)
  {
    longstatus8e.volume[j] = pmsgbuf[26+j*2];
    longstatus8e.occupancy[j] = pmsgbuf[27+j*2];
  }
}
 
int getPollIdx(const std::vector<poll_conf_t>& poll_list_,const uint8_t* pResp,const uint8_t messType)
{
  int idx = -1;
  switch(messType)
  {
  case AB3418MSG::getBlockMsgRes_errMessType:
    for (std::vector<poll_conf_t>::const_iterator itPolls = poll_list_.begin(); itPolls != poll_list_.end(); ++itPolls)
    {
      if (itPolls->err_messType == AB3418MSG::getBlockMsgRes_errMessType 
        && itPolls->poll_data1 == pResp[0] && itPolls->poll_data2 == pResp[1])
      {
        if (!itPolls->pollReturned) {idx = static_cast<int>(distance(poll_list_.begin(),itPolls));}
        break;
      }
    }
    break;
  case AB3418MSG::getTimingDataRes_messType:
    for (std::vector<poll_conf_t>::const_iterator itPolls = poll_list_.begin(); itPolls != poll_list_.end(); ++itPolls)
    {
      if (itPolls->res_messType == AB3418MSG::getTimingDataRes_messType && itPolls->poll_data1 == pResp[0] 
        && itPolls->poll_data2 == pResp[1] && itPolls->poll_data3 == pResp[2])
      {
        if (!itPolls->pollReturned) {idx = static_cast<int>(distance(poll_list_.begin(),itPolls));}
        break;
      }
    }
    break;
  case AB3418MSG::getBlockMsgRes_messType:
    for (std::vector<poll_conf_t>::const_iterator itPolls = poll_list_.begin(); itPolls != poll_list_.end(); ++itPolls)
    {
      if (itPolls->res_messType == AB3418MSG::getBlockMsgRes_messType 
        && itPolls->poll_data1 == pResp[0] && itPolls->poll_data2 == pResp[1])
      {
        if (!itPolls->pollReturned) {idx = static_cast<int>(distance(poll_list_.begin(),itPolls));}
        break;
      } 
    }
    break;
  default:
    break;  
  }
  return (idx);
}

void parsePhaseFlags(phaseflags_mess_t& phaseflags,const uint8_t* pmsgbuf)
{
  int offset = 7;
  phaseflags.permitted_phases = bitset<8>(pmsgbuf[offset++]);
  phaseflags.restricted_phases = bitset<8>(pmsgbuf[offset++]);
  phaseflags.minimum_recall_phases = bitset<8>(pmsgbuf[offset++]);
  phaseflags.maximum_recall_phases = bitset<8>(pmsgbuf[offset++]);
  phaseflags.ped_recall_phases = bitset<8>(pmsgbuf[offset++]);
  phaseflags.bike_recall_phases = bitset<8>(pmsgbuf[offset++]);
  phaseflags.redlock_phases = bitset<8>(pmsgbuf[offset++]);
  phaseflags.yewlock_phases = bitset<8>(pmsgbuf[offset++]);
  phaseflags.fomaxlock_phases = bitset<8>(pmsgbuf[offset++]);
  phaseflags.doubleEntry_phases = bitset<8>(pmsgbuf[offset++]);
  phaseflags.restInWalk_phases = bitset<8>(pmsgbuf[offset++]);
  phaseflags.restInRed_phases = bitset<8>(pmsgbuf[offset++]);
  phaseflags.walk2_phases = bitset<8>(pmsgbuf[offset++]);
  phaseflags.maxgreen2_phases = bitset<8>(pmsgbuf[offset++]);
  phaseflags.maxgreen3_phases = bitset<8>(pmsgbuf[offset++]);
  phaseflags.startup_green_phases = bitset<8>(pmsgbuf[offset++]);
  phaseflags.startup_yellow_phases = bitset<8>(pmsgbuf[offset++]);
  phaseflags.startup_vehCalls = bitset<8>(pmsgbuf[offset++]);
  phaseflags.startup_pedCalls = bitset<8>(pmsgbuf[offset++]);
  phaseflags.startup_yellowOverlaps = bitset<8>(pmsgbuf[offset++]);
  phaseflags.startup_allred = pmsgbuf[offset++];  
}

void parsePermittedPedPhases(std::bitset<8>& permitted_ped_phases,const uint8_t* pmsgbuf)
{
  for (int j = 7; j < 15; j++)
  {
    permitted_ped_phases |= bitset<8>(pmsgbuf[j]);
  }  
}

void parsePhaseTiming(phasetiming_mess_t& phasetiming,const uint8_t* pmsgbuf)
{
  int offset = 6;
  phasetiming.phase_num = pmsgbuf[offset++];
  phasetiming.walk1_interval = pmsgbuf[offset++];
  phasetiming.walk_clearance = pmsgbuf[offset++];
  phasetiming.minimum_green = pmsgbuf[offset++];
  phasetiming.detector_limit = pmsgbuf[offset++];
  phasetiming.maximum_initial = pmsgbuf[offset++];
  phasetiming.maximum_extensions[0] = pmsgbuf[offset++];
  phasetiming.maximum_extensions[1] = pmsgbuf[offset++];
  phasetiming.maximum_extensions[2] = pmsgbuf[offset++];
  phasetiming.passage = pmsgbuf[offset++];
  phasetiming.maximum_gap = pmsgbuf[offset++];
  phasetiming.minimum_gap = pmsgbuf[offset++];
  phasetiming.added_initial_per_vehicle = pmsgbuf[offset++];
  phasetiming.reduce_gap_by = pmsgbuf[offset++];
  phasetiming.reduce_gap_every = pmsgbuf[offset++];
  phasetiming.yellow_interval = pmsgbuf[offset++];
  phasetiming.red_clearance = pmsgbuf[offset++];
  phasetiming.walk2_interval = pmsgbuf[offset++];
  phasetiming.delay_early_walk_time = pmsgbuf[offset++];
  phasetiming.solid_walk_clearance = pmsgbuf[offset++];
  phasetiming.bike_green = pmsgbuf[offset++];
  phasetiming.bike_red_clearance = pmsgbuf[offset++];  
}

void parseFreePlanTiming(freeplan_mess_t& freeplan,manualplan_mess_t& manualplan, const uint8_t* pmsgbuf)
{
  int offset = 15;
  /// freeplan
  freeplan.lag_phases = bitset<8>(pmsgbuf[offset++]);
  freeplan.omit_phases = bitset<8>(pmsgbuf[offset++]);
  freeplan.minimum_recall_phases = bitset<8>(pmsgbuf[offset++]);
  freeplan.maximum_recall_phases = bitset<8>(pmsgbuf[offset++]);
  freeplan.ped_recall_phases = bitset<8>(pmsgbuf[offset++]);
  freeplan.bike_recall_phases = bitset<8>(pmsgbuf[offset++]);
  freeplan.conditional_service_phases = bitset<8>(pmsgbuf[offset++]);
  freeplan.conditional_service_minimum_green = pmsgbuf[offset++];
  /// manualplan
  manualplan.plan_num = pmsgbuf[offset++];
  manualplan.offset_index = pmsgbuf[offset++];
  if (manualplan.offset_index >= 10)
    manualplan.offset_index = static_cast<uint8_t>(manualplan.offset_index - 10);
  if (manualplan.plan_num == 0)
    manualplan.planOn = false;
  else
    manualplan.planOn = true;  
}

void parseCICplans(cicplan_mess_t& cicplans,const uint8_t* pmsgbuf)
{
  // CIC Enable in Plans: Byte #8: Bit 0 = plan 9, Byte #9: Bits 0-7 = plans 1-8
  unsigned long ul = (unsigned long)((pmsgbuf[7] & 0x01) << 8) + pmsgbuf[8];
  cicplans.enabled_plans = bitset<9>( ul );
  int offset = 9;
  cicplans.smoothing_volume = pmsgbuf[offset++];
  cicplans.smoothing_occupancy = pmsgbuf[offset++];
  cicplans.smoothing_demand = pmsgbuf[offset++];
  cicplans.multiplier_volume = pmsgbuf[offset++];
  cicplans.multiplier_occupancy = pmsgbuf[offset++];
  cicplans.exponent_volume = pmsgbuf[offset++];
  cicplans.exponent_occupancy = pmsgbuf[offset++];
  for (int ii = 0; ii < 16; ii++)
  {
    cicplans.phase_assignment[ii] = pmsgbuf[offset++];
  }  
}

void parseDetectorConf(vector<detectorconf_mess_t>& detectorconf,const uint8_t* pmsgbuf)
{
  detectorconf_mess_t detectorconf_mess;
  int offset = 7;
  for (int entry = 0; entry < 4; entry++)
  {
    detectorconf_mess.type = pmsgbuf[offset++];
    detectorconf_mess.phaseAssignment = bitset<8>(pmsgbuf[offset++]);
    detectorconf_mess.lock = pmsgbuf[offset++];
    detectorconf_mess.delayTime = pmsgbuf[offset++];
    detectorconf_mess.extendTime = pmsgbuf[offset++];
    detectorconf_mess.recallTime = pmsgbuf[offset++];
    detectorconf_mess.inputPort = pmsgbuf[offset++];
    detectorconf.push_back(detectorconf_mess);
  }  
}

void parseDetectorAssignment(system_detector_assignment_mess_t& assignment,const uint8_t* pmsgbuf)
{
  assignment.maxOnTime = pmsgbuf[7];
  assignment.failResetTime = pmsgbuf[8];
  // 1-8, 9-16, 17-24, 25-32, 33-40, 41-44
  assignment.failOverride = bitset<44>((pmsgbuf[14] & 0x07));
  for (int i = 13; i > 8; i--)
  {
    (assignment.failOverride <<= 8) ^= (bitset<44>(pmsgbuf[i]));
  }
  int offset = 15;
  for (int i = 0; i < 16; i++)
  {
    assignment.detectorInput[i] = pmsgbuf[offset++];
  }  
}

void parseTODtables(vector<TODtable_mess_t>& TODtables,const uint8_t* pmsgbuf)
{
  TODtable_mess_t TODtable_mess;
  TODtable_mess.table_num = static_cast<uint8_t>((pmsgbuf[6] - 1)/4 + 1);
  int offset = 7;
  for (int entry = 0; entry < 4; entry++)
  {
    TODtable_mess.start_hour = pmsgbuf[offset++];
    TODtable_mess.start_min = pmsgbuf[offset++];
    TODtable_mess.plan_num = pmsgbuf[offset++];
    TODtable_mess.offset_index = pmsgbuf[offset++];
    if (TODtable_mess.offset_index >= 10)
      TODtable_mess.offset_index = static_cast<uint8_t>(TODtable_mess.offset_index  - 10);
    if (!(TODtable_mess.start_hour == 0 && TODtable_mess.start_min == 0 && TODtable_mess.plan_num == 0))
      TODtables.push_back(TODtable_mess);
  }
}

void parseDOWtable(weekday_plan_assignment_mess_t& assignment,const uint8_t* pmsgbuf)
{
  int offset = 7;
  for (int i = 0; i < 7; i++)
  {
    assignment.dayofweek[i] = pmsgbuf[offset++];
  }  
}

void parseTODfunctions(vector<TODfunction_mess_t>& TODfunctions,const uint8_t* pmsgbuf)
{
  TODfunction_mess_t TODfunction_mess;
  int offset = 7;
  for (int entry = 0; entry < 4; entry++)
  {
    TODfunction_mess.start_hour = pmsgbuf[offset++];
    TODfunction_mess.start_min = pmsgbuf[offset++];
    TODfunction_mess.end_hour = pmsgbuf[offset++];
    TODfunction_mess.end_min = pmsgbuf[offset++];
    TODfunction_mess.dayofweek = bitset<8>(pmsgbuf[offset++]);
    TODfunction_mess.action_code = pmsgbuf[offset++];
    TODfunction_mess.affect_phases = bitset<8>(pmsgbuf[offset++]);
    if (TODfunction_mess.action_code > 0)
      TODfunctions.push_back(TODfunction_mess);
  }  
}

void parseEVpreempt(EVpreemption_mess_t& EVpreemption,const uint8_t* pmsgbuf)
{
  int offset = 7;                
  EVpreemption.delay_time = pmsgbuf[offset++];
  EVpreemption.green_hold_time = pmsgbuf[offset++];
  EVpreemption.maximum_clearance_time = pmsgbuf[offset++];
  EVpreemption.clearance_phase_green = bitset<8>(pmsgbuf[offset++]);
  EVpreemption.clearance_overlap_green = bitset<8>(pmsgbuf[offset++]);
  EVpreemption.input_port = pmsgbuf[offset++];
  EVpreemption.latching_flag = pmsgbuf[offset++];
  EVpreemption.phase_termination_flag = pmsgbuf[offset++];  
}

void parseRRpreempt(RRpreemption_mess_t& RRpreemption,const uint8_t* pmsgbuf,const int step)
{
  int offset = 7;
  switch(step)
  {
  case 0:
    RRpreemption.RRpreemption_steps[0].green_hold_phases = bitset<8>(pmsgbuf[offset++]);
    RRpreemption.RRpreemption_steps[0].yew_flashing_phases = bitset<8>(pmsgbuf[offset++]);
    RRpreemption.RRpreemption_steps[0].red_flashing_phases = bitset<8>(pmsgbuf[offset++]);
    RRpreemption.RRpreemption_steps[1].green_hold_phases = bitset<8>(pmsgbuf[offset++]);
    RRpreemption.RRpreemption_steps[1].yew_flashing_phases = bitset<8>(pmsgbuf[offset++]);
    RRpreemption.RRpreemption_steps[1].red_flashing_phases = bitset<8>(pmsgbuf[offset++]);
    RRpreemption.RRpreemption_steps[2].green_hold_phases = bitset<8>(pmsgbuf[offset++]);
    RRpreemption.RRpreemption_steps[2].yew_flashing_phases = bitset<8>(pmsgbuf[offset++]);
    RRpreemption.RRpreemption_steps[2].red_flashing_phases = bitset<8>(pmsgbuf[offset++]);
    RRpreemption.RRpreemption_steps[3].green_hold_phases = bitset<8>(pmsgbuf[offset++]);
    RRpreemption.RRpreemption_steps[3].yew_flashing_phases = bitset<8>(pmsgbuf[offset++]);
    RRpreemption.RRpreemption_steps[3].red_flashing_phases = bitset<8>(pmsgbuf[offset++]);  
    break;
  case 1:
    RRpreemption.RRpreemption_steps[0].ped_walk_phases = bitset<8>(pmsgbuf[offset++]);
    RRpreemption.RRpreemption_steps[0].ped_clear_phases = bitset<8>(pmsgbuf[offset++]);
    RRpreemption.RRpreemption_steps[0].ped_red_phases = bitset<8>(pmsgbuf[offset++]);
    RRpreemption.RRpreemption_steps[1].ped_walk_phases = bitset<8>(pmsgbuf[offset++]);
    RRpreemption.RRpreemption_steps[1].ped_clear_phases = bitset<8>(pmsgbuf[offset++]);
    RRpreemption.RRpreemption_steps[1].ped_red_phases = bitset<8>(pmsgbuf[offset++]);
    RRpreemption.RRpreemption_steps[2].ped_walk_phases = bitset<8>(pmsgbuf[offset++]);
    RRpreemption.RRpreemption_steps[2].ped_clear_phases = bitset<8>(pmsgbuf[offset++]);
    RRpreemption.RRpreemption_steps[2].ped_red_phases = bitset<8>(pmsgbuf[offset++]);
    RRpreemption.RRpreemption_steps[3].ped_walk_phases = bitset<8>(pmsgbuf[offset++]);
    RRpreemption.RRpreemption_steps[3].ped_clear_phases = bitset<8>(pmsgbuf[offset++]);
    RRpreemption.RRpreemption_steps[3].ped_red_phases = bitset<8>(pmsgbuf[offset++]);
    break;
  case 2:
    RRpreemption.RRpreemption_steps[0].green_hold_overlaps = bitset<8>(pmsgbuf[offset++]);
    RRpreemption.RRpreemption_steps[0].yew_flashing_overlaps = bitset<8>(pmsgbuf[offset++]);
    RRpreemption.RRpreemption_steps[0].red_flashing_overlaps = bitset<8>(pmsgbuf[offset++]);
    RRpreemption.RRpreemption_steps[1].green_hold_overlaps = bitset<8>(pmsgbuf[offset++]);
    RRpreemption.RRpreemption_steps[1].yew_flashing_overlaps = bitset<8>(pmsgbuf[offset++]);
    RRpreemption.RRpreemption_steps[1].red_flashing_overlaps = bitset<8>(pmsgbuf[offset++]);
    RRpreemption.RRpreemption_steps[2].green_hold_overlaps = bitset<8>(pmsgbuf[offset++]);
    RRpreemption.RRpreemption_steps[2].yew_flashing_overlaps = bitset<8>(pmsgbuf[offset++]);
    RRpreemption.RRpreemption_steps[2].red_flashing_overlaps = bitset<8>(pmsgbuf[offset++]);
    RRpreemption.RRpreemption_steps[3].green_hold_overlaps = bitset<8>(pmsgbuf[offset++]);
    RRpreemption.RRpreemption_steps[3].yew_flashing_overlaps = bitset<8>(pmsgbuf[offset++]);
    RRpreemption.RRpreemption_steps[3].red_flashing_overlaps = bitset<8>(pmsgbuf[offset++]);
    break;
  case 3:
    RRpreemption.exit_phases_green = bitset<8>(pmsgbuf[offset++]);
    RRpreemption.exit_overlaps_green = bitset<8>(pmsgbuf[offset++]);
    RRpreemption.exit_veh_call = bitset<8>(pmsgbuf[offset++]);
    RRpreemption.exit_ped_call = bitset<8>(pmsgbuf[offset++]);
    break;
  case 4:
    RRpreemption.delay_time = pmsgbuf[offset++];
    RRpreemption.RRpreemption_steps[0].step_time = pmsgbuf[offset++];
    RRpreemption.RRpreemption_steps[1].step_time = pmsgbuf[offset++];
    RRpreemption.RRpreemption_steps[2].step_time = pmsgbuf[offset++];
    RRpreemption.RRpreemption_steps[3].step_time = pmsgbuf[offset++];
    RRpreemption.exit_time = pmsgbuf[offset++];
    RRpreemption.minimum_green = pmsgbuf[offset++];
    RRpreemption.ped_clear_time = pmsgbuf[offset++];
    RRpreemption.input_port = pmsgbuf[offset++];
    RRpreemption.gate_port = pmsgbuf[offset++];
    RRpreemption.latching_flag = pmsgbuf[offset++];
    RRpreemption.power_up = pmsgbuf[offset++]; 
    break;
  default:
    break;
  }  
}

void parseTSPenablePlans(TSPconf_mess_t& TSPconf,freeplan_mess_t& freeplan,const uint8_t* pmsgbuf)
{
  // Byte #8: Bit 0 = plan 9, Byte #9: Bits 0-7 = plans 1-8
  // Byte #10: Bit 0 = plan 19, Byte #11: Bits 0-7 = plans 11-18
  // plan 10 is skipped
  unsigned long ul = (unsigned long)((pmsgbuf[7] & 0x01) << 8) + pmsgbuf[8];
  ul += ((((pmsgbuf[9] & 0x01) << 8) + pmsgbuf[10]) << 9);
  TSPconf.enable_coordination_plans = bitset<18>(ul);
  freeplan.TSP_max_green_hold_time = pmsgbuf[11];
  freeplan.TSP_hold_phases = bitset<8>(pmsgbuf[12]);
  if (freeplan.TSP_hold_phases.any() && freeplan.TSP_max_green_hold_time > 0)
    freeplan.isTSPenabled = true;
  else
    freeplan.isTSPenabled = false;
}

void parseTSPplans(TSPconf_mess_t& TSPconf,const uint8_t* pmsgbuf,const int planGroupIdx)
{
  int offset = 7;
  for (int entry = 0; entry < 3; entry++)
  {
    int planIdx = planGroupIdx + entry;
    TSPconf.TSPplan[planIdx].max_early_green = pmsgbuf[offset++];
    TSPconf.TSPplan[planIdx].max_green_extension = pmsgbuf[offset++];
    TSPconf.TSPplan[planIdx].inhibit_cycles = pmsgbuf[offset++];
    for (int i = 0; i < 8; i++)
    {
      TSPconf.TSPplan[planIdx].green_factor[i] = pmsgbuf[offset++];
    }
  }  
}

void parseCoordPlans(vector<coordplan_mess_t>& coordplans,const uint8_t* pmsgbuf,const uint8_t plan_num)
{
  coordplan_mess_t coordplan_mess;
  coordplan_mess.plan_num = plan_num;
  int offset = 7;
  coordplan_mess.cycle_length = pmsgbuf[offset++];
  if (coordplan_mess.cycle_length > 0)
  {
    /// only record non-empty coordination plans
    for (int i = 0; i < 8; i++)
    {
      coordplan_mess.green_factor[i] = pmsgbuf[offset++];
    }
    coordplan_mess.cycle_multiplier = pmsgbuf[offset++];
    for (int i = 0; i < 3; i++)
    {
      coordplan_mess.offsets[i] = pmsgbuf[offset++];
    }
    coordplan_mess.laggapout_phase = pmsgbuf[offset++];
    coordplan_mess.lag_phases = bitset<8>(pmsgbuf[offset++]);
    coordplan_mess.sync_phases = bitset<8>(pmsgbuf[offset++]);
    coordplan_mess.hold_phases = bitset<8>(pmsgbuf[offset++]);
    coordplan_mess.omit_phases = bitset<8>(pmsgbuf[offset++]);
    coordplan_mess.minimum_recall_phases = bitset<8>(pmsgbuf[offset++]);
    coordplan_mess.maximum_recall_phases = bitset<8>(pmsgbuf[offset++]);
    coordplan_mess.ped_recall_phases = bitset<8>(pmsgbuf[offset++]);
    coordplan_mess.bike_recall_phases = bitset<8>(pmsgbuf[offset++]);
    coordplan_mess.force_off_flag = pmsgbuf[offset++];  
    coordplans.push_back(coordplan_mess);
  }  
}

void updateTimingCard(timing_card_t& card,const uint8_t* pmsgbuf,const std::string& pollDesc)
{
  if (pollDesc.find("phase flags") == 0)
    parsePhaseFlags(card.phaseflags,pmsgbuf);
  else if (pollDesc.find("ped flags") == 0)
    parsePermittedPedPhases(card.phaseflags.permitted_ped_phases,pmsgbuf);
  else if (pollDesc.find("red revert") == 0)
    card.phaseflags.red_revert_interval = pmsgbuf[8];
  else if (pollDesc.find("phase timing") == 0)
    parsePhaseTiming(card.phasetiming[pmsgbuf[6]-1],pmsgbuf);
  else if (pollDesc.find("free plan") == 0)
    parseFreePlanTiming(card.freeplan,card.manualplan,pmsgbuf);
  else if (pollDesc.find("CIC plan") == 0)
    parseCICplans(card.cicplans,pmsgbuf);
  else if (pollDesc.find("detector group") == 0)
    parseDetectorConf(card.detectorconf,pmsgbuf);
  else if (pollDesc.find("system detector") == 0)
    parseDetectorAssignment(card.system_detector_assignment,pmsgbuf);
  else if (pollDesc.find("TOD table") == 0)
    parseTODtables(card.TODtables,pmsgbuf);
  else if (pollDesc.find("weekday") == 0)
    parseDOWtable(card.weekday_plan_assignment,pmsgbuf);
  else if (pollDesc.find("TOD Function") == 0)
    parseTODfunctions(card.TODfunctions,pmsgbuf);
  else if (pollDesc.find("EVA") == 0)
    parseEVpreempt(card.EVpreemption[0],pmsgbuf);
  else if (pollDesc.find("EVB") == 0)
    parseEVpreempt(card.EVpreemption[1],pmsgbuf);
  else if (pollDesc.find("EVC") == 0)
    parseEVpreempt(card.EVpreemption[2],pmsgbuf);
  else if (pollDesc.find("EVD") == 0)
    parseEVpreempt(card.EVpreemption[3],pmsgbuf);
  else if (pollDesc.find("EVD") == 0)
    parseEVpreempt(card.EVpreemption[3],pmsgbuf);
  else if (pollDesc.find("RR1 phase flags") == 0)
    parseRRpreempt(card.RRpreemption[0],pmsgbuf,0);
  else if (pollDesc.find("RR1 ped flags") == 0)
    parseRRpreempt(card.RRpreemption[0],pmsgbuf,1);
  else if (pollDesc.find("RR1 overlap flags") == 0)
    parseRRpreempt(card.RRpreemption[0],pmsgbuf,2);
  else if (pollDesc.find("RR1 exit parameters") == 0)
    parseRRpreempt(card.RRpreemption[0],pmsgbuf,3);
  else if (pollDesc.find("RR1 Configuration") == 0)
    parseRRpreempt(card.RRpreemption[0],pmsgbuf,4);
  else if (pollDesc.find("RR2 phase flags") == 0)
    parseRRpreempt(card.RRpreemption[1],pmsgbuf,0);
  else if (pollDesc.find("RR2 ped flags") == 0)
    parseRRpreempt(card.RRpreemption[1],pmsgbuf,1);
  else if (pollDesc.find("RR2 overlap flags") == 0)
    parseRRpreempt(card.RRpreemption[1],pmsgbuf,2);
  else if (pollDesc.find("RR2 exit parameters") == 0)
    parseRRpreempt(card.RRpreemption[1],pmsgbuf,3);
  else if (pollDesc.find("RR2 Configuration") == 0)
    parseRRpreempt(card.RRpreemption[1],pmsgbuf,4);  
  else if (pollDesc.find("TSP enable plans") == 0)
    parseTSPenablePlans(card.TSPconf,card.freeplan,pmsgbuf);
  else if (pollDesc.find("TSP plan group") == 0)
    parseTSPplans(card.TSPconf,pmsgbuf,(pmsgbuf[6] - 2) * 3);
  else if (pollDesc.find("coord plan") == 0)
    parseCoordPlans(card.coordplans,pmsgbuf,(uint8_t)((pmsgbuf[5] - 4) * 10 + pmsgbuf[6]));
}

void updatePermitPhases(std::bitset<8>& permitted_phases,std::bitset<8>& permitted_ped_phases,
  const operation_mode_enum_t::controlmode cntrMode,const ssize_t planIdx,const timing_card_t& timingcard)
{
  permitted_phases = timingcard.phaseflags.permitted_phases;
  permitted_ped_phases = timingcard.phaseflags.permitted_ped_phases;
  if (cntrMode == operation_mode_enum_t::RUNNINGFREE)
  {
    permitted_phases = timingcard.freeplan.permitted_phases;                
    permitted_ped_phases = timingcard.freeplan.permitted_ped_phases;                
  }  
  else if (planIdx >= 0)
  {
    permitted_phases = timingcard.coordplans[planIdx].permitted_phases;
    permitted_ped_phases = timingcard.coordplans[planIdx].permitted_ped_phases;
  }  
}

size_t formpollrequest(uint8_t* buf,const uint8_t addr,const poll_conf_t& poll_conf)
{
  int msg_len;
  buf[0] = AB3418MSG::flag;
  buf[1] = addr;
  buf[2] = poll_conf.poll_controlByte;
  buf[3] = AB3418MSG::ipi;
  buf[4] = poll_conf.poll_messType;
  buf[5] = poll_conf.poll_data1;
  buf[6] = poll_conf.poll_data2;
  msg_len = 6;
  if (poll_conf.poll_type == 2)
  {
    // getTimingData
    buf[7] = poll_conf.poll_data3;
    msg_len = 7;
  }
  /// append the FCS //
  AB3418checksum::get_modframe_string(buf+1,&msg_len);
  /// byte stuffing: from byte 2 to the end of checksum (bytes between 0x7E), 
  /// replace 0x7E or 0x7D to 2-byte sequence 0x7D5E and 0x7D5D, respectively
  AB3418checksum::get_byte_stuffing(buf,&msg_len);
  /// add the end flag
  buf[msg_len+1] = AB3418MSG::flag;
  return (static_cast<size_t>(msg_len+2));
}

size_t formsoftcallrequest(uint8_t* buf,const uint8_t addr,const bitset<8>& veh_call,const bitset<8>& ped_call,const bitset<8>& prio_call)
{
  int msg_len;
  buf[0] = AB3418MSG::flag;
  buf[1] = addr;
  buf[2] = AB3418MSG::set_controlByte;
  buf[3] = AB3418MSG::ipi;
  buf[4] = AB3418MSG::setSoftcall_messType;
  buf[5] = static_cast<uint8_t>(veh_call.to_ulong());
  buf[6] = static_cast<uint8_t>(ped_call.to_ulong());
  buf[7] = static_cast<uint8_t>(prio_call.to_ulong());
  buf[8] = 0;
  buf[9] = 0;
  buf[10] = 0;
  buf[11] = 0;
  buf[12] = 0;
  msg_len = 12;
  /// append the FCS //
  AB3418checksum::get_modframe_string(buf+1,&msg_len);
  /// byte stuffing: from byte 2 to the end of checksum (bytes between 0x7E), 
  /// replace 0x7E or 0x7D to 2-byte sequence 0x7D5E and 0x7D5D, respectively
  AB3418checksum::get_byte_stuffing(buf,&msg_len);
  /// add the end flag
  buf[msg_len+1] = AB3418MSG::flag;
  return (static_cast<size_t>(msg_len+2));
}

void bitsetphases2ringphases(uint8_t (&ringphases)[2],const std::bitset<8>& bitsetPhases)
{
  ringphases[0] = 0;
  ringphases[1] = 0;
  for (int i = 0; i < 8; i++)
  {
    if (bitsetPhases.test(i))
    {
      if (i < 4)
        ringphases[0] = static_cast<uint8_t>(i + 1);
      else
      {
        ringphases[1] = static_cast<uint8_t>(i + 1);
        break;
      }
    }
  }    
}

uint8_t ring_phase_on(const uint8_t check_phase)
{
  /// check_phase != 0
  if (check_phase <= 4)
    return (0);
  else
    return (1);
}

uint8_t barrier_phases_on(const std::bitset<8>& check_phases)
{
  bitset<8> left_barrier (std::string("00110011"));
  left_barrier &= check_phases;
  if (left_barrier.any())
    return (0);
  else 
    return (1);
}

uint8_t next_ring(const uint8_t check_ring)
{
  return static_cast<uint8_t>((check_ring+1) % 2);
}

uint8_t next_barrier(const uint8_t check_barrier)
{
  return static_cast<uint8_t>((check_barrier+1) % 2);  
}

operation_mode_enum_t::leadlagoperation getLeadlagOperationMode(const std::bitset<8>& sync_phases,const std::bitset<8>& lag_phases,const std::bitset<8>& permitted_phases)
{
  bitset<8> sync_lag_phases = (sync_phases & (lag_phases & permitted_phases));
  switch(sync_lag_phases.count())
  {
  case 0:
    return operation_mode_enum_t::LEAD_LEAD;
    break;
  case 2:
    return operation_mode_enum_t::LAG_LAG;
    break;
  default:
    if (sync_phases.count() == 2)
    {
      /// sync phase on both rings are set
      bitset<8> check_phases (std::string("00001111"));
      check_phases &= sync_lag_phases;
      if (check_phases.count() == 1)
        return operation_mode_enum_t::LAG_LEAD;
      else
        return operation_mode_enum_t::LEAD_LAG;
    }
    else
      return operation_mode_enum_t::LAG_LAG;
    break;
  }
}

void getLeadLagPhasebyBarrierAndRing(uint8_t (&leadlag_phases)[2][2][2],const std::bitset<8>& lag_phases,const std::bitset<8>& permitted_phases)
{
  bitset<8> barrier_phases;
  bitset<8> ring_phases;
  bitset<8> seq_phases;
  
  for (uint8_t barrier = 0; barrier < 2; barrier++)
  {
    if (barrier == 0)
      barrier_phases = bitset<8>(std::string("00110011"));
    else
      barrier_phases = bitset<8>(std::string("11001100"));
    for (uint8_t ring = 0; ring < 2; ring++)
    {
      if (ring == 0)
        ring_phases = bitset<8>(std::string("00001111"));
      else
        ring_phases = bitset<8>(std::string("11110000"));
      for (uint8_t seq = 0; seq < 2; seq++)
      {
        if (seq == 0)
          seq_phases = (~lag_phases) & permitted_phases & barrier_phases & ring_phases;
        else
          seq_phases = lag_phases & permitted_phases & barrier_phases & ring_phases;
        if (seq_phases.none())
          leadlag_phases[barrier][ring][seq] = 0;
        else
        {
          for (uint8_t i = 0; i < 8; i++)
          {
            if (seq_phases.test(i))
            {
              leadlag_phases[barrier][ring][seq] = static_cast<uint8_t>(i+1);
              break;
            }
          }
        }
      }
      if (leadlag_phases[barrier][ring][0] == 0 && leadlag_phases[barrier][ring][1] > 0)
        leadlag_phases[barrier][ring][0] = leadlag_phases[barrier][ring][1];
      else if (leadlag_phases[barrier][ring][1] == 0 && leadlag_phases[barrier][ring][0] > 0)
        leadlag_phases[barrier][ring][1] = leadlag_phases[barrier][ring][0];
    }
  }
}

void getForceOffbyRing(uint8_t (&force_off)[8],uint32_t &count,const uint8_t (&greenfactor)[8],const phasetiming_mess_t (&phasetiming)[8],
  const uint8_t leadPhase,const uint8_t lagPhase,bool greenOnset)
{
  if (leadPhase > 0)
  {
    if (greenOnset)
      count += greenfactor[leadPhase-1] * 10;
    force_off[leadPhase-1] = static_cast<uint8_t>(count / 10);
    count = force_off[leadPhase-1] * 10 + phasetiming[leadPhase-1].yellow_interval + phasetiming[leadPhase-1].red_clearance;
    if (force_off[leadPhase-1] == 0)
      count += 10;  // local_cycle clock starts at 1 not 0
  }
  if (lagPhase >0 && lagPhase != leadPhase)
  {
    count += greenfactor[lagPhase-1] * 10;
    force_off[lagPhase-1] = static_cast<uint8_t>(count / 10);
    count = force_off[lagPhase-1] * 10 +  phasetiming[lagPhase-1].yellow_interval + phasetiming[lagPhase-1].red_clearance;
  } 
}

void getForceOffPoints(uint8_t (&force_off)[8],const uint8_t (&greenfactor)[8],const uint8_t (&leadlag_phases)[2][2][2],const phasetiming_mess_t (&phasetiming)[8],
  const uint8_t sync_barrier,const uint8_t sync_ring,const operation_mode_enum_t::leadlagoperation leadLagMode)
{
  memset(&force_off[0],0,8);
  uint32_t count[2] = {0,0};
  uint8_t leadPhase,lagPhase;
  uint8_t cur_barrier = sync_barrier;
  
  /// start with sync_barrier
  for (uint8_t ring = sync_ring; ring < sync_ring + 2; ring++)
  {
    uint8_t cur_ring = static_cast<uint8_t>(ring % 2);
    leadPhase = leadlag_phases[cur_barrier][cur_ring][0];
    lagPhase = leadlag_phases[cur_barrier][cur_ring][1];
    if (leadLagMode == operation_mode_enum_t::LEAD_LEAD)
      getForceOffbyRing(force_off,count[cur_ring],greenfactor,phasetiming,leadPhase,lagPhase,false);
    else if (leadLagMode == operation_mode_enum_t::LAG_LAG)
      getForceOffbyRing(force_off,count[cur_ring],greenfactor,phasetiming,lagPhase,0,false);
    else if (cur_ring == sync_ring)
      getForceOffbyRing(force_off,count[cur_ring],greenfactor,phasetiming,leadPhase,lagPhase,false);
    else
    {
      count[cur_ring] = force_off[leadlag_phases[cur_barrier][next_ring(cur_ring)][1] - 1] * 10;
      getForceOffbyRing(force_off,count[cur_ring],greenfactor,phasetiming,lagPhase,0,false);
    }
  }
  /// move to non sync_barrier
  cur_barrier = next_barrier(sync_barrier);
  count[0] = (count[0] < count[1]) ? count[1] : count[0];
  count[1] = count[0];
  for (uint8_t cur_ring = 0; cur_ring < 2; cur_ring++)
  {
    leadPhase = leadlag_phases[cur_barrier][cur_ring][0];
    lagPhase = leadlag_phases[cur_barrier][cur_ring][1];
    getForceOffbyRing(force_off,count[cur_ring],greenfactor,phasetiming,leadPhase,lagPhase,true);    
  }
  /// move to sync_barrier
  cur_barrier = sync_barrier;
  count[0] = (count[0] < count[1]) ? count[1] : count[0];
  count[1] = count[0];
  for (uint8_t ring = sync_ring; ring < sync_ring + 2; ring++)
  {
    uint8_t cur_ring = static_cast<uint8_t>(ring % 2);
    leadPhase = leadlag_phases[cur_barrier][cur_ring][0];
    lagPhase = leadlag_phases[cur_barrier][cur_ring][1];
    if (leadLagMode == operation_mode_enum_t::LAG_LAG)
    {
      if (leadPhase > 0 && leadPhase != lagPhase)
        getForceOffbyRing(force_off,count[cur_ring],greenfactor,phasetiming,leadPhase,0,true);
    }
    else if (leadLagMode != operation_mode_enum_t::LEAD_LEAD)
    {
      if (cur_ring != sync_ring && leadPhase > 0 && leadPhase != lagPhase)
        getForceOffbyRing(force_off,count[cur_ring],greenfactor,phasetiming,leadPhase,0,true);
    }
  }
}

void getPlanParameters(freeplan_mess_t& freeplan,const std::bitset<8>& permitted_phases,const std::bitset<8>& permitted_ped_phases)
{
  freeplan.permitted_phases = (permitted_phases & (~freeplan.omit_phases));
  freeplan.permitted_ped_phases = (permitted_ped_phases & (~freeplan.omit_phases));
  getLeadLagPhasebyBarrierAndRing(freeplan.leadlag_phases,freeplan.lag_phases,freeplan.permitted_phases);
}

void getPlanParameters(std::vector<coordplan_mess_t>& coordplans,const TSPconf_mess_t& TSPconf,const phasetiming_mess_t (&phasetiming)[8],
  const std::bitset<8>& permitted_phases,const std::bitset<8>& permitted_ped_phases,const std::bitset<8>& maximum_recall_phases)
{
  for (size_t i = 0; i < coordplans.size(); i++)
  {
    coordplan_mess_t* pcoordplan = &coordplans[i];
    /// permitted_phases
    pcoordplan->permitted_phases = (permitted_phases & (~pcoordplan->omit_phases));
    pcoordplan->permitted_ped_phases = (permitted_ped_phases & (~pcoordplan->omit_phases));
    /// coordinated_phases
    bitsetphases2ringphases(pcoordplan->coordinated_phases,pcoordplan->sync_phases);
    /// leadLagMode
    pcoordplan->leadLagMode = getLeadlagOperationMode(pcoordplan->sync_phases,pcoordplan->lag_phases,pcoordplan->permitted_phases);
    /// sync_ring
    pcoordplan->sync_ring = 0;
    if (pcoordplan->coordinated_phases[0] == 0 || pcoordplan->leadLagMode == operation_mode_enum_t::LAG_LEAD)
      pcoordplan->sync_ring = 1;
    /// sync_barrier
    pcoordplan->sync_barrier = barrier_phases_on(pcoordplan->sync_phases);
    /// leadlag_phases
    getLeadLagPhasebyBarrierAndRing(pcoordplan->leadlag_phases,pcoordplan->lag_phases,pcoordplan->permitted_phases);
    /// force_off
    if (pcoordplan->force_off_flag == 1)
      memcpy(&pcoordplan->force_off[0],&pcoordplan->green_factor[0],8);
    else
    {
      getForceOffPoints(pcoordplan->force_off,pcoordplan->green_factor,pcoordplan->leadlag_phases,phasetiming,
        pcoordplan->sync_barrier,pcoordplan->sync_ring,pcoordplan->leadLagMode);
    }    
    /// permissive & ped_permissive
    for (uint8_t j = 0; j < 8; j++)
    {
      pcoordplan->permissive[j] = 0;
      pcoordplan->ped_permissive[j] = 0;
      if (pcoordplan->permitted_phases.test(j) && !pcoordplan->sync_phases.test(j))
      {
        pcoordplan->permissive[j] = static_cast<uint8_t>((pcoordplan->force_off[j] > phasetiming[j].minimum_green) ?
          (pcoordplan->force_off[j] - phasetiming[j].minimum_green) : 0);
      }
      if (pcoordplan->permitted_ped_phases.test(j) && !pcoordplan->sync_phases.test(j))
      {
        pcoordplan->ped_permissive[j] = static_cast<uint8_t>((pcoordplan->force_off[j] > phasetiming[j].walk1_interval + phasetiming[i].walk_clearance) ?
          (pcoordplan->force_off[j] - phasetiming[j].walk1_interval - phasetiming[j].walk_clearance) : 0);
      }
    }
    /// noncoordBarrierGreenOnset - green onset for non-coordinated barrier
    uint32_t count[2];
    for (uint8_t ring = 0; ring < 2; ring++)
    {
      count[ring] = 0;
      uint8_t lagphase = pcoordplan->leadlag_phases[pcoordplan->sync_barrier][ring][1];
      if (lagphase > 0)
        count[ring] = pcoordplan->force_off[lagphase-1] * 10 + phasetiming[lagphase-1].yellow_interval + phasetiming[lagphase-1].red_clearance;
    }
    pcoordplan->noncoordBarrierGreenOnset = (count[0] > count[1]) ? count[0] : count[1];
    /// coordBarrierGreenOnset - green onset for coordinated barrier
    for (uint8_t ring = 0; ring < 2; ring++)
    {
      count[ring] = 0;
      uint8_t lagphase = pcoordplan->leadlag_phases[next_barrier(pcoordplan->sync_barrier)][ring][1];
      if (lagphase > 0)
        count[ring] = pcoordplan->force_off[lagphase-1] * 10 + phasetiming[lagphase-1].yellow_interval + phasetiming[lagphase-1].red_clearance;
    }
    pcoordplan->coordBarrierGreenOnset = (count[0] > count[1]) ? count[0] : count[1];
    /// coordPhaseGreenOnset - green onset for coordinated phase by ring
    /// coordPhaseGreenEnd - yellow onset for coordinated phase by ring
    for (uint8_t ring = 0; ring < 2; ring++)
    {
      pcoordplan->coordPhaseGreenOnset[ring] = pcoordplan->coordBarrierGreenOnset;
      uint8_t leadphase = pcoordplan->leadlag_phases[pcoordplan->sync_barrier][ring][0];
      uint8_t syncphase = pcoordplan->coordinated_phases[ring];
      if (syncphase > 0 && pcoordplan->lag_phases.test(syncphase-1) && leadphase > 0 && leadphase != syncphase)
        pcoordplan->coordPhaseGreenOnset[ring] = pcoordplan->force_off[leadphase-1] * 10 + phasetiming[leadphase-1].yellow_interval + phasetiming[leadphase-1].red_clearance;
      if (syncphase > 0)
        pcoordplan->coordPhaseGreenEnd[ring] = pcoordplan->force_off[syncphase-1] * 10;
      else
        pcoordplan->coordPhaseGreenEnd[ring] = 0;
    }
    /// coordLagphaseGapout and adjustment on coordPhaseGreenEnd
    pcoordplan->coordLagphaseGapout = false;
    if (pcoordplan->leadLagMode != operation_mode_enum_t::LEAD_LEAD && pcoordplan->leadLagMode != operation_mode_enum_t::LAG_LAG)
    {
      uint8_t lagphase = pcoordplan->leadlag_phases[pcoordplan->sync_barrier][pcoordplan->sync_ring][1];
      uint8_t leadphase = pcoordplan->leadlag_phases[pcoordplan->sync_barrier][pcoordplan->sync_ring][0];
      uint8_t coordlagphase = pcoordplan->leadlag_phases[pcoordplan->sync_barrier][next_ring(pcoordplan->sync_ring)][1];
      if (lagphase > 0 && leadphase > 0 && coordlagphase > 0 && lagphase == pcoordplan->laggapout_phase 
        && !pcoordplan->maximum_recall_phases.test(lagphase-1) && !maximum_recall_phases.test(lagphase-1))
      {
        pcoordplan->coordLagphaseGapout = true;
        pcoordplan->coordPhaseGreenEnd[next_ring(pcoordplan->sync_ring)] = pcoordplan->force_off[leadphase-1] * 10 
          + phasetiming[leadphase-1].yellow_interval + phasetiming[leadphase-1].red_clearance
          + phasetiming[lagphase-1].minimum_green * 10;
      }
    }
    /// TSP conf (for plan 1 to 9 and 11 to 19)
    pcoordplan->isTSPenabled = false;
    pcoordplan->max_early_green = 0;
    pcoordplan->max_green_extension = 0;
    pcoordplan->inhibit_cycles = 0;
    memset(&(pcoordplan->TSP_force_off[0]),0,8);
    if (pcoordplan->plan_num > 0 && pcoordplan->plan_num < 20)
    {
      int planIdx;
      if (pcoordplan->plan_num > 10)
        planIdx = pcoordplan->plan_num - 2;
      else
        planIdx = pcoordplan->plan_num - 1;
      pcoordplan->max_early_green = TSPconf.TSPplan[planIdx].max_early_green;
      pcoordplan->max_green_extension = TSPconf.TSPplan[planIdx].max_green_extension;
      pcoordplan->inhibit_cycles = TSPconf.TSPplan[planIdx].inhibit_cycles;
      memcpy(&pcoordplan->TSP_force_off[0],&TSPconf.TSPplan[planIdx].green_factor[0],8);      
      if (TSPconf.enable_coordination_plans.test(planIdx))
      {
        pcoordplan->isTSPenabled = true;
        uint8_t syncphase = pcoordplan->coordinated_phases[pcoordplan->sync_ring];
        if (TSPconf.TSPplan[planIdx].green_factor[syncphase-1] != 0)
        {
          getForceOffPoints(pcoordplan->TSP_force_off,TSPconf.TSPplan[planIdx].green_factor,pcoordplan->leadlag_phases,phasetiming,
            pcoordplan->sync_barrier,pcoordplan->sync_ring,pcoordplan->leadLagMode);          
        }
      }
    }
  }
}

operation_mode_enum_t::controlmode get_control_mode(const std::bitset<8>& controller_status,const std::bitset<8>& preempt,const uint8_t pattern_num)
{
  bitset<8> preemption(std::string("00111111"));
  preemption &= preempt;
  
  if (controller_status.test(0)) 
    return (operation_mode_enum_t::PREEMPTION);
  if (controller_status.test(1) || pattern_num == pattern_flashing)
    return (operation_mode_enum_t::FLASHING);
  if (pattern_num == pattern_free || pattern_num == 0)
    return (operation_mode_enum_t::RUNNINGFREE);
  else
    return (operation_mode_enum_t::COORDINATION);
}

operation_mode_enum_t::lightcolor get_light_color(const uint8_t active_phase,const uint8_t active_intv,const uint8_t check_phase)
{
  ///interval decoding
  /// 0x00 = Walk
  /// 0x01 = Don’t Walk
  /// 0x02 = Min Green
  /// 0x04 = Added Initial
  /// 0x05 = Passage
  /// 0x06 = Max Gap
  /// 0x07 = Min Gap
  /// 0x08 = Red Rest
  /// 0x09 = Preemption 
  /// 0x0A = Stop Time (remian indication)
  /// 0x0B = Red Revert
  /// 0x0C = Max Termination
  /// 0x0D = Gap Termination
  /// 0x0E = Force Off
  /// 0x0F = Red Clearance
  if (check_phase != active_phase)
    return operation_mode_enum_t::RED;
  if (active_intv < 0x08 || active_intv == 0x09 || active_intv == 0x0A)
    return operation_mode_enum_t::GREEN;    
  if (active_intv >= 0x0C && active_intv <= 0x0E)
    return operation_mode_enum_t::YELLOW;    
  else
    return operation_mode_enum_t::RED;
}

operation_mode_enum_t::pedsign get_ped_state(const uint8_t active_phase,const uint8_t active_intv,const uint8_t check_phase)
{
  ///interval decoding
  /// 0x00 = Walk
  /// 0x01 = Don’t Walk
  if (check_phase != active_phase)
    return operation_mode_enum_t::DONT_WALK;
  if (active_intv == 0x00)
    return operation_mode_enum_t::WALK;    
  if (active_intv == 0x01)
    return operation_mode_enum_t::FLASH_DONOT_WALK;    
  else
    return operation_mode_enum_t::DONT_WALK;
}

operation_mode_enum_t::recalltype getPhaseRecallType(const phaseflags_mess_t& phaseflags,const coordplan_mess_t& coordplan,const uint8_t phaseIdx)
{
  if (phaseflags.maximum_recall_phases.test(phaseIdx) || coordplan.maximum_recall_phases.test(phaseIdx))
    return operation_mode_enum_t::MAXIMUMRECALL;
  else if (phaseflags.ped_recall_phases.test(phaseIdx) || coordplan.ped_recall_phases.test(phaseIdx))
    return operation_mode_enum_t::PEDRECALL;
  else if (phaseflags.minimum_recall_phases.test(phaseIdx) || coordplan.minimum_recall_phases.test(phaseIdx))
    return operation_mode_enum_t::MINIMUMRECALL;
  else if (phaseflags.bike_recall_phases.test(phaseIdx) || coordplan.bike_recall_phases.test(phaseIdx))
    return operation_mode_enum_t::BIKERECALL;
  else
    return operation_mode_enum_t::NORECALL;
}

operation_mode_enum_t::recalltype getPhaseRecallType(const phaseflags_mess_t& phaseflags,const freeplan_mess_t& freeplan,const uint8_t phaseIdx)
{
  if (phaseflags.maximum_recall_phases.test(phaseIdx) || freeplan.maximum_recall_phases.test(phaseIdx))
    return operation_mode_enum_t::MAXIMUMRECALL;
  else if (phaseflags.ped_recall_phases.test(phaseIdx) || freeplan.ped_recall_phases.test(phaseIdx))
    return operation_mode_enum_t::PEDRECALL;
  else if (phaseflags.minimum_recall_phases.test(phaseIdx) || freeplan.minimum_recall_phases.test(phaseIdx))
    return operation_mode_enum_t::MINIMUMRECALL;
  else if (phaseflags.bike_recall_phases.test(phaseIdx) || freeplan.bike_recall_phases.test(phaseIdx))
    return operation_mode_enum_t::BIKERECALL;
  else
    return operation_mode_enum_t::NORECALL;
}

operation_mode_enum_t::concurrentphasetype getConcurrentPhaseType(const std::bitset<8>& active_phase,const std::bitset<8>& sync_phase)
{
  bitset<8> active_sync_phase = (active_phase & sync_phase);
  size_t active_sync_phase_count = active_sync_phase.count();
  size_t sync_phase_count = sync_phase.count();
  if (active_sync_phase_count == 0)
    return operation_mode_enum_t::MINOR_MINOR;
  else if (active_sync_phase_count == sync_phase_count)
    return operation_mode_enum_t::MAJOR_MAJOR;
  else
    return operation_mode_enum_t::MINOR_MAJOR;
}

uint32_t getPhaseGreenLeft(const uint8_t active_interval,const uint8_t interval_timer,const uint32_t intervalTimeInto,const phasetiming_mess_t& phasetiming)
{
  uint32_t countdownTime = interval_timer * 10;
  
  if (active_interval == 0x00)  /// walk
    return (((countdownTime > intervalTimeInto) ? (countdownTime - intervalTimeInto) : countdownTime) + phasetiming.walk_clearance * 10);
  else if (active_interval < 0x05) /// walk clearance or minimum green or added initial
    return ((countdownTime > intervalTimeInto) ? (countdownTime - intervalTimeInto) : countdownTime);
  else /// passage, map gap, min gap
    return (interval_timer); 
}

uint16_t getPedIntervalLeft(const uint8_t interval_timer,const long long timer_time,const long long tms)
{
  uint16_t timeinto = static_cast<uint16_t>((tms+30-timer_time)/100LL);
  uint16_t timecountdown = static_cast<uint16_t>(interval_timer * 10);
  return (static_cast<uint16_t>((timecountdown > timeinto) ? (timecountdown - timeinto) : timecountdown));
}

uint32_t getPhaseGreen2Maxout(const phasetiming_mess_t& phasetiming,const phaseflags_mess_t& phaseflags,const uint8_t phaseIdx)
{
  uint32_t maximum_extension = phasetiming.maximum_extensions[0];
  if (phaseflags.maxgreen2_phases.test(phaseIdx))
    maximum_extension = phasetiming.maximum_extensions[1];
  else if (phaseflags.maxgreen3_phases.test(phaseIdx))
    maximum_extension = phasetiming.maximum_extensions[2];
  return ((phasetiming.minimum_green + maximum_extension) * 10);
}

uint8_t getPhaseWalkInterval(const phasetiming_mess_t& phasetiming,const phaseflags_mess_t& phaseflags,const uint8_t phaseIdx)
{
  return static_cast<uint8_t>((phaseflags.walk2_phases.test(phaseIdx)) ?  phasetiming.walk2_interval : phasetiming.walk1_interval);
}

uint32_t getPhaseGuaranteedGreen(const phasetiming_mess_t& phasetiming,const phaseflags_mess_t& phaseflags,const uint8_t phaseIdx,const uint8_t active_interval)
{
  /// for active phase
  if (active_interval <= 0x01)
    return ((getPhaseWalkInterval(phasetiming,phaseflags,phaseIdx) + phasetiming.walk_clearance) * 10);
  else if (active_interval < 0x05)
    return (phasetiming.minimum_green * 10);
  else
    return 0;
}

uint32_t getPhaseGuaranteedGreen(const phasetiming_mess_t& phasetiming,const phaseflags_mess_t& phaseflags,const uint8_t phaseIdx,const phase_status_t& phase_status)
{
  /// for phase is not active and not with continous recall
  if (phase_status.recall_status == operation_mode_enum_t::PEDRECALL || phase_status.call_status == operation_mode_enum_t::CALLPED)
    return ((getPhaseWalkInterval(phasetiming,phaseflags,phaseIdx) + phasetiming.walk_clearance) * 10);
  else 
    return (phasetiming.minimum_green * 10);    
}

void barrierCrossAdjust(predicted_bound_t (&time2start)[2])
{
  time2start[0].bound_L = static_cast<uint16_t>((time2start[0].bound_L < time2start[1].bound_L) ? time2start[1].bound_L : time2start[0].bound_L);
  time2start[1].bound_L = time2start[0].bound_L;
  time2start[0].bound_U = static_cast<uint16_t>((time2start[0].bound_U < time2start[1].bound_U) ? time2start[1].bound_U : time2start[0].bound_U);
  time2start[1].bound_U = time2start[0].bound_U;  
}

void updateActivePhaseTime2next(phase_status_t& phase_status,predicted_bound_t& time2start,const signal_status_mess_t& signalstatus,
  const phasetiming_mess_t& phasetiming,const phaseflags_mess_t& phaseflags,const uint8_t ring,const long long timer_time,const long long tms)
{
  /// running free, no force-off constraint
  uint32_t intervalTimeInto = static_cast<uint32_t>((tms + 30 - timer_time)/100LL);
  switch(phase_status.state)
  {
  case operation_mode_enum_t::GREEN:
    {
      uint32_t stateTimeInto = static_cast<uint32_t>((tms + 30 - phase_status.state_start_time)/100LL);
      /// green left based on active_interval
      uint32_t timeleft = getPhaseGreenLeft(signalstatus.active_interval[ring],signalstatus.interval_timer[ring],intervalTimeInto,phasetiming);
      /// time2maxout (till max-out point)
      uint32_t maxgreen = getPhaseGreen2Maxout(phasetiming,phaseflags,(uint8_t)(signalstatus.active_phases[ring]-1));
      uint32_t time2maxout = (maxgreen > stateTimeInto) ? (maxgreen - stateTimeInto) : 0;
      /// time2gapout (till end of the guaranteed green where the phase could be gapped-out)
      uint32_t guaranteedgreen = ((phase_status.recall_status == operation_mode_enum_t::MAXIMUMRECALL) ?
        maxgreen : getPhaseGuaranteedGreen(phasetiming,phaseflags,(uint8_t)(signalstatus.active_phases[ring]-1),signalstatus.active_interval[ring]));
      uint32_t time2gapout = (guaranteedgreen > stateTimeInto) ? (guaranteedgreen - stateTimeInto) : 0;
      /// minimum green is guranteed
      time2maxout = (time2maxout < time2gapout) ? time2gapout : time2maxout;
      /// time2next
      phase_status.time2next.bound_L = static_cast<uint16_t>( (time2gapout == 0) ? timeleft : time2gapout);
      phase_status.time2next.bound_U = static_cast<uint16_t>( (time2maxout == 0) ? timeleft : time2maxout);
      phase_status.time2next.confidence = SPAT_StateConfidence_minTime;
      /// time2start for phases after (next_phase not known yet)
      time2start.bound_L = static_cast<uint16_t>(phase_status.time2next.bound_L + phasetiming.yellow_interval + phasetiming.red_clearance);
      time2start.bound_U = static_cast<uint16_t>(phase_status.time2next.bound_U + phasetiming.yellow_interval + phasetiming.red_clearance);
      time2start.confidence = SPAT_StateConfidence_minTime; 
    }
    break;
  case operation_mode_enum_t::YELLOW:
    {
      /// yellow interval is fixed
      uint32_t timeleft = signalstatus.interval_timer[ring];
      phase_status.time2next.bound_L = static_cast<uint16_t>((timeleft >= intervalTimeInto) ? (timeleft - intervalTimeInto) : timeleft);
      phase_status.time2next.bound_U = phase_status.time2next.bound_L;
      phase_status.time2next.confidence = SPAT_StateConfidence_minTime;
      if (signalstatus.next_phases[ring] == signalstatus.active_phases[ring])
      {
        time2start.bound_L = static_cast<uint16_t>(phase_status.time2next.bound_L +
          ((phaseflags.red_revert_interval > phasetiming.red_clearance) ? phaseflags.red_revert_interval : phasetiming.red_clearance));
      }
      else
        time2start.bound_L = static_cast<uint16_t>(phase_status.time2next.bound_L + phasetiming.red_clearance);
      time2start.bound_U = time2start.bound_L;
      time2start.confidence = SPAT_StateConfidence_minTime;
    }
    break;
  default:
    {
      /// red clearance or red revert intervals are fixed
      uint32_t timeleft = signalstatus.interval_timer[ring];      
      phase_status.time2next.bound_L = static_cast<uint16_t>((timeleft >= intervalTimeInto) ? (timeleft - intervalTimeInto) : timeleft);
      phase_status.time2next.bound_U = phase_status.time2next.bound_U;
      phase_status.time2next.confidence = SPAT_StateConfidence_unKnownEstimate;
      /// time2start for the next_phase (next_phase already known)
      time2start.bound_L = phase_status.time2next.bound_L;
      time2start.bound_U = time2start.bound_L;
      time2start.confidence = SPAT_StateConfidence_minTime;
    }
    break;
  }
}

void getNextPhaseStartBound(predicted_bound_t& time2start,const phasetiming_mess_t& phasetiming,const phaseflags_mess_t& phaseflags,
  const uint8_t phaseIdx,const phase_status_t& phase_status)
{
  /// running free, for phase is not active
  uint32_t maxgreen = getPhaseGreen2Maxout(phasetiming,phaseflags,phaseIdx);
  uint32_t guaranteedgreen = ((phase_status.recall_status == operation_mode_enum_t::MAXIMUMRECALL) ?
    maxgreen : getPhaseGuaranteedGreen(phasetiming,phaseflags,phaseIdx,phase_status));
  maxgreen = (maxgreen < guaranteedgreen) ? guaranteedgreen : maxgreen;
  if (phase_status.recall_status != operation_mode_enum_t::NORECALL || phase_status.call_status != operation_mode_enum_t::NOCALL)
  {
    /// phase should be on, move back time2start.bound_L
    time2start.bound_L = static_cast<uint16_t>(time2start.bound_L + guaranteedgreen + phasetiming.yellow_interval + phasetiming.red_clearance);
  }
  time2start.bound_U = static_cast<uint16_t>(time2start.bound_U + maxgreen + phasetiming.yellow_interval + phasetiming.red_clearance);
  time2start.confidence = SPAT_StateConfidence_minTime;
}

uint32_t getTime2Forceoff(const uint8_t force_off,const uint16_t local_cycle_clock,const uint16_t cycle_length,const bool is_sync_phase)
{
  /// sync phase that terminates at yield point actually terminates at the offset of a second
  uint32_t forceoff = (is_sync_phase && force_off == 0) ? 10 : force_off * 10;
  return ((is_sync_phase) ? ((forceoff > local_cycle_clock) ? (forceoff - local_cycle_clock) : (forceoff + cycle_length - local_cycle_clock))
    : ((forceoff > local_cycle_clock) ? (forceoff - local_cycle_clock) : 0));
}

bool isPhaseForceoffOnly(const operation_mode_enum_t::concurrentphasetype type,const bool is_sync_phase,const bool is_lag_phase)
{
  if (type == operation_mode_enum_t::MINOR_MINOR ||
    (type == operation_mode_enum_t::MINOR_MAJOR && !is_sync_phase && !is_lag_phase))
  {
    /// MINOR_MINOR or lead left-turn phase in MINOR_MAJOR
    return false;
  }
  else 
    return true;
}

bool isPhaseForceoffOnly(const bool is_sync_phase,const bool is_leadlag_mode,const bool is_minor_lagphase)
{
  if (is_sync_phase || (is_leadlag_mode && is_minor_lagphase))
    return true;
  else 
    return false;
}

uint32_t getTime2GreenEnd(const uint32_t time2maxout,const uint32_t time2forceoff,const bool is_forceoff_Only)
{
  /// green end at either max-out point or force-off point, whichever comes first
  if (is_forceoff_Only)
    return time2forceoff;
  else
    return ((time2maxout > time2forceoff) ? time2forceoff : time2maxout);
}

void getNextPhaseStartBound(predicted_bound_t& time2start,const phasetiming_mess_t& phasetiming,const phaseflags_mess_t& phaseflags,const uint8_t phaseIdx,
  const phase_status_t& phase_status,const coordplan_mess_t* pcoordplan,const uint16_t local_cycle_clock,const uint16_t cycle_length)
{
  /// under coodination, for phase is not active
  uint32_t maxgreen = getPhaseGreen2Maxout(phasetiming,phaseflags,phaseIdx);
  bool terminateByforceoffOnly = isPhaseForceoffOnly(pcoordplan->sync_phases.test(phaseIdx),
    (pcoordplan->leadLagMode == operation_mode_enum_t::LEAD_LAG || pcoordplan->leadLagMode == operation_mode_enum_t::LAG_LEAD),
    (pcoordplan->leadlag_phases[pcoordplan->sync_barrier][pcoordplan->sync_ring][1] == (uint8_t)(phaseIdx+1)));
  /// when phase green starts at time2start.bound_L
  uint16_t start_local_cycle_clock_L = static_cast<uint16_t>(time2start.bound_L + local_cycle_clock);
  start_local_cycle_clock_L = static_cast<uint16_t>((start_local_cycle_clock_L > cycle_length) ? (start_local_cycle_clock_L - cycle_length) : start_local_cycle_clock_L);
  uint32_t time2forceoff_L = getTime2Forceoff(pcoordplan->force_off[phaseIdx],start_local_cycle_clock_L,cycle_length,pcoordplan->sync_phases.test(phaseIdx));
  uint32_t time2terminate_L = getTime2GreenEnd(maxgreen,time2forceoff_L,terminateByforceoffOnly);
  uint32_t guaranteedgreen_L = ((phase_status.recall_status == operation_mode_enum_t::MAXIMUMRECALL) ?
    time2terminate_L : getPhaseGuaranteedGreen(phasetiming,phaseflags,phaseIdx,phase_status));
  time2terminate_L = (time2terminate_L < guaranteedgreen_L) ? guaranteedgreen_L : time2terminate_L;
  /// when phase green starts at time2start.bound_U
  uint16_t start_local_cycle_clock_U = static_cast<uint16_t>(time2start.bound_U + local_cycle_clock);
  start_local_cycle_clock_U = static_cast<uint16_t>((start_local_cycle_clock_U > cycle_length) ? (start_local_cycle_clock_U - cycle_length) : start_local_cycle_clock_U);
  uint32_t time2forceoff_U = getTime2Forceoff(pcoordplan->force_off[phaseIdx],start_local_cycle_clock_U,cycle_length,pcoordplan->sync_phases.test(phaseIdx));
  uint32_t time2terminate_U = getTime2GreenEnd(maxgreen,time2forceoff_U,terminateByforceoffOnly);
  uint32_t guaranteedgreen_U = ((phase_status.recall_status == operation_mode_enum_t::MAXIMUMRECALL) ?
    time2terminate_U : getPhaseGuaranteedGreen(phasetiming,phaseflags,phaseIdx,phase_status));
  time2terminate_U = (time2terminate_U < guaranteedgreen_U) ? guaranteedgreen_U : time2terminate_U;
  if (terminateByforceoffOnly)
  {
    time2start.bound_L = static_cast<uint16_t>(time2start.bound_L + time2terminate_L + phasetiming.yellow_interval + phasetiming.red_clearance);
    time2start.bound_U = static_cast<uint16_t>(time2start.bound_U + time2terminate_U + phasetiming.yellow_interval + phasetiming.red_clearance);
  }
  else 
  {
    time2start.bound_U = static_cast<uint16_t>(time2start.bound_U + time2terminate_U + phasetiming.yellow_interval + phasetiming.red_clearance);
    if (phase_status.recall_status != operation_mode_enum_t::NORECALL || phase_status.call_status != operation_mode_enum_t::NOCALL)
      time2start.bound_L = static_cast<uint16_t>(time2start.bound_L + guaranteedgreen_L + phasetiming.yellow_interval + phasetiming.red_clearance);
  }
  time2start.confidence = SPAT_StateConfidence_minTime;
}

void updateActivePhaseTime2next(phase_status_t& phase_status,predicted_bound_t& time2start,const coordplan_mess_t* pcoordplan,const signal_status_mess_t& signalstatus,
  const phasetiming_mess_t& phasetiming,const phaseflags_mess_t& phaseflags,const uint8_t ring,const uint16_t local_cycle_clock,const uint16_t cycle_length,
  const operation_mode_enum_t::concurrentphasetype concurrentType,const long long timer_time,const long long tms)
{  
  /// under coordination, needs to consider force-off constraint
  uint32_t intervalTimeInto = static_cast<uint32_t>((tms + 30 - timer_time)/100LL);
  switch(phase_status.state)
  {
  case operation_mode_enum_t::GREEN:
    {
      /// stateTimeInto & intervalTimeInto
      uint32_t stateTimeInto = static_cast<uint32_t>((tms + 30 - phase_status.state_start_time)/100LL);
      /// green left based on active_interval
      uint32_t timeleft = getPhaseGreenLeft(signalstatus.active_interval[ring],signalstatus.interval_timer[ring],intervalTimeInto,phasetiming);
      /// time2maxout (till max-out point)
      uint32_t maxgreen = getPhaseGreen2Maxout(phasetiming,phaseflags,(uint8_t)(signalstatus.active_phases[ring]-1));
      uint32_t time2maxout = (maxgreen > stateTimeInto) ? (maxgreen - stateTimeInto) : 0;
      /// time2forceoff (till force-off point)
      uint32_t time2forceoff = getTime2Forceoff(signalstatus.active_force_off[ring],local_cycle_clock,cycle_length,
        (signalstatus.active_phases[ring] == pcoordplan->coordinated_phases[ring]));
      /// time2terminate (max-out or foece-off whichever comes first)
      bool terminateByforceoffOnly = isPhaseForceoffOnly(concurrentType,(signalstatus.active_phases[ring] == pcoordplan->coordinated_phases[ring]),
        pcoordplan->lag_phases.test(signalstatus.active_phases[ring]-1));
      uint32_t time2terminate = getTime2GreenEnd(time2maxout,time2forceoff,terminateByforceoffOnly);
      /// time2gapout (till the end of the guaranteed green)
      uint32_t guaranteedgreen = ((phase_status.recall_status == operation_mode_enum_t::MAXIMUMRECALL) ?
        time2terminate : getPhaseGuaranteedGreen(phasetiming,phaseflags,(uint8_t)(signalstatus.active_phases[ring]-1),signalstatus.active_interval[ring]));
      uint32_t time2gapout = (guaranteedgreen > stateTimeInto) ? (guaranteedgreen - stateTimeInto) : 0;
      /// minimum green is guranteed
      time2terminate = (time2terminate < time2gapout) ? time2gapout : time2terminate;
      /// time2next
      phase_status.time2next.bound_U = static_cast<uint16_t>( (time2terminate == 0) ? timeleft : time2terminate);
      if (terminateByforceoffOnly)
        phase_status.time2next.bound_L = phase_status.time2next.bound_U;
      else
        phase_status.time2next.bound_L = static_cast<uint16_t>( (time2gapout == 0) ? timeleft : time2gapout);
      phase_status.time2next.confidence = SPAT_StateConfidence_minTime;
      /// time2start for the next_phase (next_phase not known yet)
      time2start.bound_L = static_cast<uint16_t>(phase_status.time2next.bound_L + phasetiming.yellow_interval + phasetiming.red_clearance);
      time2start.bound_U = static_cast<uint16_t>(phase_status.time2next.bound_U + phasetiming.yellow_interval + phasetiming.red_clearance);
      time2start.confidence = SPAT_StateConfidence_minTime; 
    }
    break;
  case operation_mode_enum_t::YELLOW:
    {
      /// yellow interval is fixed
      uint32_t timeleft = signalstatus.interval_timer[ring];
      phase_status.time2next.bound_L = static_cast<uint16_t>((timeleft >= intervalTimeInto) ? (timeleft - intervalTimeInto) : timeleft);
      phase_status.time2next.bound_U = phase_status.time2next.bound_L;
      phase_status.time2next.confidence = SPAT_StateConfidence_minTime;
      if (signalstatus.next_phases[ring] == signalstatus.active_phases[ring])
      {
        time2start.bound_L = static_cast<uint16_t>(phase_status.time2next.bound_L +
          ((phaseflags.red_revert_interval > phasetiming.red_clearance) ? phaseflags.red_revert_interval : phasetiming.red_clearance));
      }
      else
        time2start.bound_L = static_cast<uint16_t>(phase_status.time2next.bound_L + phasetiming.red_clearance);
      time2start.bound_U = time2start.bound_L;
      time2start.confidence = SPAT_StateConfidence_minTime;
    }
    break;
  default:
    {
      /// red clearance or red revert intervals are fixed
      uint32_t timeleft = signalstatus.interval_timer[ring];
      phase_status.time2next.bound_L = static_cast<uint16_t>((timeleft >= intervalTimeInto) ? (timeleft - intervalTimeInto) : timeleft);
      phase_status.time2next.bound_U = phase_status.time2next.bound_U;
      phase_status.time2next.confidence = SPAT_StateConfidence_unKnownEstimate;
      /// time2start for the next_phase (next_phase already known)
      time2start.bound_L = phase_status.time2next.bound_L;
      time2start.bound_U = time2start.bound_L;
      time2start.confidence = SPAT_StateConfidence_minTime;
    }
    break;
  }
}

void softcallPhaseReset(bitset<8>& callPhases,const uint8_t requestPhases)
{
  bitset<8> resetPhases = bitset<8>(requestPhases);
  for (int i = 0; i < 8; i++)
  {
    if (resetPhases.test(i) && callPhases.test(i))
      callPhases.reset(i);
  }
}

void softcallPhaseSet(bitset<8>& callPhases,const uint8_t requestPhases)
{
  bitset<8> resetPhases = bitset<8>(requestPhases);
  for (int i = 0; i < 8; i++)
  {
    if (resetPhases.test(i) && !callPhases.test(i))
      callPhases.set(i);
  }
}

void updateSoftcallState(softcall_state_t& softcallState,const softcall_request_t* pRequest)
{  
  switch(pRequest->calltype)
  {
  case softcall_enum_t::CANCEL:
    if (pRequest->callobj == softcall_enum_t::VEH)
      softcallPhaseReset(softcallState.veh_ext,pRequest->callphase);
    else if (pRequest->callobj == softcall_enum_t::PRIORITY)
    {
      softcallPhaseReset(softcallState.prio_call,pRequest->callphase);
      softcallPhaseReset(softcallState.prio_ext,pRequest->callphase);
    }
    break;
  case softcall_enum_t::CALL:
    if (pRequest->callobj == softcall_enum_t::PED)
      softcallPhaseSet(softcallState.ped_call,pRequest->callphase);
    else if (pRequest->callobj == softcall_enum_t::VEH)
      softcallPhaseSet(softcallState.veh_call,pRequest->callphase);
    else if (pRequest->callobj == softcall_enum_t::PRIORITY)
      softcallPhaseSet(softcallState.prio_call,pRequest->callphase);
    break;
  case softcall_enum_t::EXTENSION:
    if (pRequest->callobj == softcall_enum_t::VEH)
      softcallPhaseSet(softcallState.veh_ext,pRequest->callphase);
    else if (pRequest->callobj == softcall_enum_t::PRIORITY)
      softcallPhaseSet(softcallState.prio_ext,pRequest->callphase);
    break;
  default:
    break;
  }
}  

void updateSoftcallState(softcall_state_t& softcallState,const phase_status_t* pPhase_status)
{
  for (int i = 0; i < 8; i++)
  {
    if (softcallState.veh_call.test(i) && pPhase_status[i].state == operation_mode_enum_t::GREEN)
      softcallState.veh_call.reset(i);
    if (softcallState.veh_ext.test(i) && pPhase_status[i].state != operation_mode_enum_t::GREEN)
      softcallState.veh_ext.reset(i);
    if (softcallState.prio_call.test(i) && pPhase_status[i].state == operation_mode_enum_t::GREEN)
      softcallState.prio_call.reset(i);
    if (softcallState.prio_ext.test(i) && pPhase_status[i].state != operation_mode_enum_t::GREEN)
      softcallState.prio_ext.reset(i);
  }
}

bool sendPresMsg(int fd,char* buf,const status8e_mess_t& status8e)
{
  mmitss_udp_header_t header;
  header.msgheader = msg_header;
  header.msgid = msgid_detPres;
  header.ms_since_midnight = status8e.fullTimeStamp.ms_since_midnight_local;
  header.length = static_cast<uint16_t>(sizeof(pres_data_t));
  memcpy(buf,&header,sizeof(mmitss_udp_header_t));
  pres_data_t presData;
  presData.flag = static_cast<uint8_t>(status8e.flag.to_ulong());
  presData.status = static_cast<uint8_t>(status8e.status.to_ulong());
  presData.pattern_num = status8e.pattern_num;
  presData.master_cycle_clock = status8e.master_cycle_clock;
  presData.local_cycle_clock = status8e.local_cycle_clock;
  presData.prio_busId = status8e.prio_busId;
  presData.prio_busDirection = status8e.prio_busDirection;
  presData.prio_type = status8e.prio_type;
  std::string pres = status8e.detector_presences.to_string();
  strcpy(presData.presences,pres.c_str());
  memcpy(buf+sizeof(mmitss_udp_header_t),&presData,sizeof(pres_data_t));
  return (socketUtils::sendall(fd,buf,sizeof(mmitss_udp_header_t) + sizeof(pres_data_t)));
}

bool sendCntMsg(int fd,char* buf,const longstatus8e_mess_t& longstatus8e)
{
  mmitss_udp_header_t header;
  header.msgheader = msg_header;
  header.msgid = msgid_detCnt;
  header.ms_since_midnight = longstatus8e.fullTimeStamp.ms_since_midnight_local;
  header.length = static_cast<uint16_t>(sizeof(count_data_t));
  memcpy(buf,&header,sizeof(mmitss_udp_header_t));
  count_data_t cntData;
  cntData.seq_num = longstatus8e.seq_num;
  cntData.flag = static_cast<uint8_t>(longstatus8e.flag.to_ulong());
  cntData.status = static_cast<uint8_t>(longstatus8e.status.to_ulong());
  cntData.pattern_num = longstatus8e.pattern_num;
  cntData.master_cycle_clock = longstatus8e.master_cycle_clock;
  cntData.local_cycle_clock = longstatus8e.local_cycle_clock;
  for (int i = 0; i < 16; i++)
  {
    cntData.vol[i] = longstatus8e.volume[i];
    cntData.occ[i] = longstatus8e.occupancy[i];
  }
  memcpy(buf+sizeof(mmitss_udp_header_t),&cntData,sizeof(count_data_t));
  return (socketUtils::sendall(fd,buf,sizeof(mmitss_udp_header_t) + sizeof(count_data_t)));  
} 

bool sendSigStaMsg(int fd,char* buf,const controller_status_t& cntrstatus,const bool isManualCntr)
{
  mmitss_udp_header_t header;
  header.msgheader = msg_header;
  header.msgid = msgid_cntrlstatus;
  header.ms_since_midnight = cntrstatus.signal_status.fullTimeStamp.ms_since_midnight_local;
  header.length = static_cast<uint16_t>(sizeof(controller_state_t));
  memcpy(buf,&header,sizeof(mmitss_udp_header_t));
  controller_state_t cntlrState;
  cntlrState.sigState.cntlrMode = static_cast<uint8_t>(cntrstatus.mode);
  cntlrState.sigState.pattern_num = cntrstatus.signal_status.pattern_num;
  cntlrState.sigState.permitted_phases = static_cast<uint8_t>(cntrstatus.permitted_phases.to_ulong());
  cntlrState.sigState.permitted_ped_phases = static_cast<uint8_t>(cntrstatus.permitted_ped_phases.to_ulong());
  cntlrState.sigState.preempt = static_cast<uint8_t>(cntrstatus.signal_status.preempt.to_ulong());
  cntlrState.sigState.ped_call = static_cast<uint8_t>(cntrstatus.signal_status.ped_call.to_ulong());
  cntlrState.sigState.veh_call = static_cast<uint8_t>(cntrstatus.signal_status.veh_call.to_ulong());
  bitset<8> spat_status;
  if (isManualCntr)
      spat_status.set(0);
  if(cntrstatus.signal_status.active_interval[0] == 0x0A || cntrstatus.signal_status.active_interval[1] == 0x0A)
    spat_status.set(1);
  if (cntrstatus.mode == operation_mode_enum_t::FLASHING)
    spat_status.set(2);
  if (cntrstatus.mode == operation_mode_enum_t::PREEMPTION)
    spat_status.set(3);
  if(cntrstatus.signal_status.preempt.test(7))
    spat_status.set(4);
  cntlrState.status = static_cast<uint8_t>(spat_status.to_ulong());
  cntlrState.timeStamp = static_cast<uint16_t>(cntrstatus.signal_status.fullTimeStamp.utcDateTimeStamp.timeStamp.sec * 10 
    + cntrstatus.signal_status.fullTimeStamp.utcDateTimeStamp.timeStamp.millisec/100);
  if ((cntrstatus.signal_status.fullTimeStamp.utcDateTimeStamp.timeStamp.min % 2) == 1)
    cntlrState.timeStamp = static_cast<uint16_t>(cntlrState.timeStamp + 600);      
  for (int i = 0; i < 8; i++)
  {
    cntlrState.phaseState[i].state = static_cast<uint8_t>(cntrstatus.phase_status[i].state);
    cntlrState.phaseState[i].pedstate = static_cast<uint8_t>(cntrstatus.phase_status[i].pedstate);
    cntlrState.phaseState[i].call_status = static_cast<uint8_t>(cntrstatus.phase_status[i].call_status);
    cntlrState.phaseState[i].recall_status = static_cast<uint8_t>(cntrstatus.phase_status[i].recall_status);
    cntlrState.phaseState[i].time2next.bound_L = cntrstatus.phase_status[i].time2next.bound_L;
    cntlrState.phaseState[i].time2next.bound_U = cntrstatus.phase_status[i].time2next.bound_U;
    cntlrState.phaseState[i].time2next.confidence = cntrstatus.phase_status[i].time2next.confidence;
    cntlrState.phaseState[i].pedtime2next.bound_L = cntrstatus.phase_status[i].pedtime2next.bound_L;
    cntlrState.phaseState[i].pedtime2next.bound_U = cntrstatus.phase_status[i].pedtime2next.bound_U;
    cntlrState.phaseState[i].pedtime2next.confidence = cntrstatus.phase_status[i].pedtime2next.confidence;
  }
  memcpy(buf+sizeof(mmitss_udp_header_t),&cntlrState,sizeof(controller_state_t));
  return (socketUtils::sendall(fd,buf,sizeof(mmitss_udp_header_t) + sizeof(controller_state_t)));  
}

void logSignalStatusMsg(std::ofstream& OS,const signal_status_mess_t& signalstatus)
{
  OS << timeUtils::getTimestampStr(signalstatus.fullTimeStamp.localDateTimeStamp) << ",";
  OS << static_cast<int>(signalstatus.active_phases[0]) << ",";
  OS << static_cast<int>(signalstatus.active_phases[1]) << ",";
  OS << static_cast<int>(signalstatus.active_interval[0]) << ",";
  OS << static_cast<int>(signalstatus.active_interval[1]) << ",";
  OS << static_cast<int>(signalstatus.interval_timer[0]) << ",";
  OS << static_cast<int>(signalstatus.interval_timer[1]) << ",";
  OS << static_cast<int>(signalstatus.next_phases[0]) << ",";
  OS << static_cast<int>(signalstatus.next_phases[1]) << ",";
  OS << static_cast<int>(signalstatus.active_force_off[0]) << ",";
  OS << static_cast<int>(signalstatus.active_force_off[1]) << ",";
  OS << static_cast<int>(signalstatus.pattern_num) << ",";  
  OS << static_cast<int>(signalstatus.local_cycle_clock) << ",";  
  OS << static_cast<int>(signalstatus.master_cycle_clock) << ",";  
  OS << signalstatus.ped_call.to_string() << ",";   
  OS << signalstatus.veh_call.to_string() << ",";   
  OS << signalstatus.preempt.to_string() << endl;
} 
