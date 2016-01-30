/* obuAware.cpp
 *  MMITSS OBU main process, tasks include
 *    1. get gps_fixes from OBU gpsd, pack BSM and send to wmefwd for broadcast
 *    2. receives WME messages forwarded by wmefwd, including BSM from other OBUs, and SPaT, MAP and SSM from RSUs
 *    3. decodes WME messages
 *    4. tracks OBU location on the map and associates intersection-lane with control phase,
 *        also automatically switches map when the OBU crossed an intersection
 *    5.  sends SRM (including cancel) to RSU if the OBU is priority eligible (determined by configuration file)
*/
 
#include <cstdio>
#include <cstdlib>
#include <bitset>  
#include <vector>
#include <sys/time.h>

#include "msgenum.h"
#include "AsnJ2735Lib.h"
#include "comm.h"
#include "socketUtils.h"
#include "wmeUtils.h"
#include "msgUtils.h"
#include "catchSignal.h"

#include "obuAware.h"

#include "libgps.h"

using namespace std;

static bool time2query = false;
static void set_alram_timer(struct itimerval *ptout_val)
{
  ptout_val->it_interval.tv_sec = 0;
  ptout_val->it_interval.tv_usec = 100000;
  ptout_val->it_value.tv_sec = 0; 
  ptout_val->it_value.tv_usec = 100000;   
};
static void timer_alarm_hand(int code)
{
  if (code == SIGALRM)
    time2query = true;
};

void do_usage(const char* progname)
{
  cout << "Usage" << progname << endl;
  cout << "\t-s socket.config" << endl;
  cout << "\t-c vin.config" << endl;
  cout << "\t-d turn on send udp packet to display program" << endl;
  cout << "\t-f turn on log files" << endl;
  cout << "\t-v turn on verbose" << endl;
  exit (1);
}

int main(int argc, char** argv) 
{
  int option;
  char* f_socketConfig = NULL;
  char* f_vinConfig = NULL;
  volatile bool send2disp = false;
  volatile bool logFile = false;
  volatile bool logDetails = false;
  volatile bool verbose = false;
  
  /// getopt
  while ((option = getopt(argc, argv, "s:c:f:dv")) != EOF) 
  {
    switch(option) 
    {
    case 's':
      f_socketConfig = strdup(optarg);
      break;
    case 'c':
      f_vinConfig = strdup(optarg);
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
      send2disp = true;
      break;
    case 'v':
      verbose = true;
      break;
    default:
      do_usage(argv[0]);
    }
  }
  if (f_socketConfig == NULL || f_vinConfig == NULL)
  {
    do_usage(argv[0]);
  }
  
  /* ----------- preparation -------------------------------------*/
  /// get timestamp
  timeUtils::fullTimeStamp_t fullTimeStamp;
  timeUtils::getFullTimeStamp(fullTimeStamp);
  
  /// instance OBUconfig::configObu to read configuration files
  OBUconfig::configObu s_obuConfig(f_socketConfig,f_vinConfig);
  if (!s_obuConfig.isConfigSucceed())
  {
    cerr << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    cerr << ", failed construct OBUconfig::configObu";
    cerr << ", f_socketConfig = " << f_socketConfig;
    cerr << ", f_vinConfig = " << f_vinConfig << endl;
    delete f_socketConfig;
    delete f_vinConfig;
    exit(1);
  }
  delete f_socketConfig;
  delete f_vinConfig;
  
  /// get OBU vin
  OBUconfig::VinConfig myVin = s_obuConfig.getVinConfig();
  string vehName = myVin.name;
  
  /// instance AsnJ2735Lib to read nmap
  AsnJ2735Lib asnJ2735Lib(s_obuConfig.getNmapFileName().c_str());
  if (asnJ2735Lib.getIntersectionNums() == 0)
  {
    cerr << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    cerr << ", failed open nmap file = " << s_obuConfig.getNmapFileName() << endl;
    exit (1);
  }
    
  /// open log files
  string logFilePath = s_obuConfig.getLogPath();
  long long logFileInterval = static_cast<long long>(s_obuConfig.getLogInterval()) * 60 * 1000LL;
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
  /// display log file (always on for debugging purpose)
  string s_DisplayLogFileName = logFilePath + std::string("/display.log");
  ofstream OS_Display(s_DisplayLogFileName.c_str());
  if (!OS_Display.is_open())
  {
    OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    OS_ERR << ", failed open display log file" << endl;
    OS_ERR.close();
    exit(1);
  }
  /// data types to log
  vector<string> logtypes;
  logtypes.push_back(string("veh"));      // log tracked own bsm locationAware & signalAware
  logtypes.push_back(string("srm"));      // log sending srm
  logtypes.push_back(string("ssm"));      // log received ssm
  logtypes.push_back(string("ack"));      // log ssm acknowledgement
  logtypes.push_back(string("payload"));  
  /// build log file list
  vector<Logfile_t> logFiles;
  for (size_t i = 0; i < logtypes.size(); i++) 
    {logFiles.push_back(Logfile_t(logFilePath,vehName,logtypes[i]));}
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
  int fd_wmeListen = socketUtils::server(s_obuConfig.getSocketAddr(OBUconfig::socketHandleEnum_t::WMELISTEN),true);
  if (fd_wmeListen < 0)
  {
    OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    OS_ERR << ", failed create WMELISTEN socket" << endl;
    OS_Display << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    OS_Display << ", failed create WMELISTEN socket" << endl;
    if (logFile) {closeLogFiles(logFiles);}
    OS_ERR.close();
    OS_Display.close();    
    exit(1);
  }
  
  int fd_wmeSend = socketUtils::client(s_obuConfig.getSocketAddr(OBUconfig::socketHandleEnum_t::WMESEND),false);
  if (fd_wmeSend < 0)
  {
    OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    OS_ERR << ", failed create WMESEND socket" << endl;
    OS_Display << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    OS_Display << ", failed create WMESEND socket" << endl;
    if (logFile) {closeLogFiles(logFiles);}
    OS_ERR.close();
    OS_Display.close();    
    close(fd_wmeListen);        
    exit(1);
  }
  
  volatile int fd_dispSend = -1;
  if (send2disp)
  {
    fd_dispSend = socketUtils::client(s_obuConfig.getSocketAddr(OBUconfig::socketHandleEnum_t::DISPSEND),false);
    if (fd_dispSend < 0)
    {
      OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
      OS_ERR << ", failed create DISPSEND socket" << endl;
      OS_Display << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
      OS_Display << ", failed create DISPSEND socket" << endl;
      if (logFile) {closeLogFiles(logFiles);}
      OS_ERR.close();
      OS_Display.close();    
      close(fd_wmeListen);     
      close(fd_wmeSend);             
      exit(1);
    }
  }  
  
  /// initiate GPS
  int fd_gps;  
  savari_gps_data_t gpsdata;
  struct gps_data_t *gps_handler;
  gps_handler = savari_gps_open(&fd_gps, 0);
  if (gps_handler == 0)
  {
    OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    OS_ERR << ", failed connect to GPS" << endl;
    OS_Display << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    OS_Display << ", failed connect to GPS" << endl;
    if (logFile) {closeLogFiles(logFiles);}
    OS_ERR.close();
    OS_Display.close();    
    close(fd_wmeListen);        
    close(fd_wmeSend);     
    if (send2disp)
      close(fd_dispSend);
      
    exit(1);
  }
  memset(&gpsdata, 0, sizeof(savari_gps_data_t)); 

  /* ----------- intercepts signals -------------------------------------*/
  if (setjmp(exit_env) != 0) 
  {
    OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    OS_ERR << ", received user termination signal, quit!" << endl;
    OS_Display << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
    OS_Display << ", received user termination signal, quit!" << endl;
    if (logFile) {closeLogFiles(logFiles);}
    OS_ERR.close();
    OS_Display.close();    
    close(fd_wmeListen);        
    close(fd_wmeSend);        
    if (send2disp)
      close(fd_dispSend);
    savari_gps_close(gps_handler);    
    return (0);
  }
  else
    sig_ign( sig_list, sig_hand );
    
  /* ----------- gps -------------------------------------*/
  /// set up callback timer for every 100 milliseconds
  struct sigaction sa;  
  struct itimerval tout_val;      
  memset(&sa,0,sizeof(sa));
  sa.sa_handler = &timer_alarm_hand;
  sigaction(SIGALRM,&sa,NULL);
  set_alram_timer(&tout_val);
  setitimer(ITIMER_REAL, &tout_val,0);  
    
  /* ----------- local variables -------------------------------------*/
  const useconds_t usleepms = 10000;    // in microseconds
  /// control parameters
  const double    stopSpeed = 2;        // in m/s
  const double    stopDist = 5.0;       // in meters, no need to re-do mapping when speed is lower than stopSpeed & distance travelled is smaller than stopDist 
  const long long srmInterval = 1000LL; // in milliseconds
  long long timeOutms = static_cast<long long>(s_obuConfig.getDsrcTimeOut()) * 1000LL; // in milliseconds
  
  /// recv and send socket buffer
  char recvbuf[MAXDSRCMSGSIZE];
  char sendbuf[MAXDSRCMSGSIZE];
    
  /// dsrc message decoding
  SPAT_List_t spatIn;
  MAP_List_t  mapIn;
  SSM_List_t  ssmIn;
  
  /// maps to store latest inbound dsrc messages (SPaT, Map, SSM from RSUs, BSM & SRM from other cars)
  map<uint32_t,SPAT_List_t> spatList;   // key intersectionId
  map<uint32_t,MAP_List_t>  mapList;    // key intersectionId
  map<uint32_t,SSM_List_t>  ssmList;    // key intersectionId
  map<uint32_t,SPAT_List_t>::iterator iteSpatList;
  
  /// bsm encoding
  BSM_element_t bsmout;
  memset(&bsmout,0,sizeof(bsmout));
  bsmout.bolb1_element.msgCnt = 0; 
  bsmout.bolb1_element.id = myVin.vehId;
  bsmout.bolb1_element.vehLen = myVin.vehLen;
  bsmout.bolb1_element.vehWidth = myVin.vehWidth;
  bsmout.bolb1_element.steeringAngle = AsnJ2735Lib::msUnavailableElement;
  bsmout.bolb1_element.transState = 7;
  bsmout.bolb1_element.brakes = static_cast<uint16_t>(0x0800);
  bsmout.bolb1_element.semiMajorAccuracy = AsnJ2735Lib::msUnavailableElement;
  bsmout.bolb1_element.semiMinorAccuracy = AsnJ2735Lib::msUnavailableElement;
  bsmout.bolb1_element.orientation = AsnJ2735Lib::msUnavailableElement;
  
  /// for tracking own bsm on map
  GeoUtils::connectedVehicle_t cvIn;  // input
  GeoUtils::connectedVehicle_t cv;    // latest tracking
  cv.reset();
  bool changedIntersection = false;
  
  /// for tracking own priority request
  SRM_element_t cvSRM;
  initialSRM(cvSRM,myVin);
  PriorityRequest_t cvRequest;
  cvRequest.reset();
  priorityRequestAction_enum_t::Action priorityRequestAction = priorityRequestAction_enum_t::NONE;
  
  while(1)
  {       
    if (time2query)
    {
      timeUtils::getFullTimeStamp(fullTimeStamp);      
      /// time to savari_gps_read      
      if (savari_gps_read(&gpsdata,gps_handler) == SUCCESS)
      {
        if (isnan(gpsdata.latitude) == 0 && isnan(gpsdata.longitude) == 0 && isnan(gpsdata.altitude) == 0)
        {
          bsmout.bolb1_element.msgCnt = (uint8_t)((bsmout.bolb1_element.msgCnt + 1) % 128);
          bsmout.bolb1_element.ms = static_cast<uint16_t>(gpsdata.dsecond);
          bsmout.bolb1_element.lat = gpsdata.latitude;
          bsmout.bolb1_element.lon = gpsdata.longitude;
          bsmout.bolb1_element.elev = gpsdata.altitude;
          bsmout.bolb1_element.speed = gpsdata.speed / 3.6;    // convert kph to m/s
          bsmout.bolb1_element.heading = gpsdata.heading;
          bsmout.bolb1_element.accelLon = gpsdata.lonaccel;
          bsmout.bolb1_element.accelLat = gpsdata.lataccel;
          bsmout.bolb1_element.accelVert = gpsdata.vertaccel;
          bsmout.bolb1_element.yawRate = gpsdata.yawrate;
          /// encode BSM
          ssize_t payload_size = asnJ2735Lib.encode_bsm_payload(&bsmout,&sendbuf[sizeof(mmitss_udp_header_t)],sizeof(sendbuf),false);
          if (payload_size > 0)
          {
            mmitss_udp_header_t header;
            header.msgheader = msg_header;
            header.msgid = wmeUtils::msgid_bsm;
            header.ms_since_midnight = fullTimeStamp.ms_since_midnight;
            header.length = static_cast<uint16_t>(payload_size);
            memcpy(&sendbuf[0],&header,sizeof(mmitss_udp_header_t));
            socketUtils::sendall(fd_wmeSend,sendbuf,(size_t)(payload_size) + sizeof(mmitss_udp_header_t));
            if (logDetails)
            {              
              int fileIdx = findFileIdx(logtypes,"payload");
              if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
              {
                *(logFiles[fileIdx].OS) << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp) << ",BSM,payload=";
                logPayloadHex(*(logFiles[fileIdx].OS),&sendbuf[sizeof(mmitss_udp_header_t)],(size_t)payload_size);
                logFiles[fileIdx].logrows++;
              }
            }
          
            /// track vehicle on the map
            cvIn.reset();
            cvIn.id = bsmout.bolb1_element.id;
            cvIn.ts = fullTimeStamp.tms;
            cvIn.geoPoint.latitude = bsmout.bolb1_element.lat;
            cvIn.geoPoint.longitude = bsmout.bolb1_element.lon;
            cvIn.geoPoint.elevation = bsmout.bolb1_element.elev;
            cvIn.motionState.speed = bsmout.bolb1_element.speed;
            cvIn.motionState.heading = bsmout.bolb1_element.heading;
            /// get distance travelled from the last geoPoint
            double distTravelled = GeoUtils::distlla2lla(cvIn.geoPoint,cv.geoPoint);
            /// check whether need to locate vehicle on map
            bool doMapping = ((cvIn.motionState.speed  < stopSpeed && fabs(distTravelled) < stopDist) ? false : true);
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
            /// log
            if (logFile && cvIn.vehicleTrackingState.intsectionTrackingState.vehicleIntersectionStatus != GeoUtils::vehicleInMap_enum_t::NOT_IN_MAP)
            {
              /// only log tracking data when vehicle is in a MAP
              int fileIdx = findFileIdx(logtypes,"veh");
              if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
              {
                logAware(*(logFiles[fileIdx].OS),fullTimeStamp.localDateTimeStamp,bsmout.bolb1_element.msgCnt,cvIn);
                logFiles[fileIdx].logrows++;
              }
            }
            /// check change of map
            if (cv.vehicleTrackingState.intsectionTrackingState.vehicleIntersectionStatus == GeoUtils::vehicleInMap_enum_t::NOT_IN_MAP
              && cvIn.vehicleTrackingState.intsectionTrackingState.vehicleIntersectionStatus != GeoUtils::vehicleInMap_enum_t::NOT_IN_MAP)
            {
              changedIntersection = true;
              OS_Display << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
              OS_Display << ", entered MAP area, at intersection ";
              OS_Display << asnJ2735Lib.getIntersectionNameByIndex(cvIn.vehicleTrackingState.intsectionTrackingState.intersectionIndex) << endl;
            }
            else if (cv.vehicleTrackingState.intsectionTrackingState.vehicleIntersectionStatus != GeoUtils::vehicleInMap_enum_t::NOT_IN_MAP
              && cvIn.vehicleTrackingState.intsectionTrackingState.vehicleIntersectionStatus == GeoUtils::vehicleInMap_enum_t::NOT_IN_MAP)
            {
              changedIntersection = false;
              OS_Display << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);  
              OS_Display << ", left MAP area at intersection ";
              OS_Display << asnJ2735Lib.getIntersectionNameByIndex(cv.vehicleTrackingState.intsectionTrackingState.intersectionIndex) << endl;
            }
            else if (cv.vehicleTrackingState.intsectionTrackingState.vehicleIntersectionStatus != GeoUtils::vehicleInMap_enum_t::NOT_IN_MAP
              && cvIn.vehicleTrackingState.intsectionTrackingState.vehicleIntersectionStatus != GeoUtils::vehicleInMap_enum_t::NOT_IN_MAP
              && cvIn.vehicleTrackingState.intsectionTrackingState.intersectionIndex != cv.vehicleTrackingState.intsectionTrackingState.intersectionIndex)
            {
              changedIntersection = true;
              OS_Display << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);  
              OS_Display << ", intersection changed from ";
              OS_Display << asnJ2735Lib.getIntersectionNameByIndex(cv.vehicleTrackingState.intsectionTrackingState.intersectionIndex);
              OS_Display << " to ";
              OS_Display << asnJ2735Lib.getIntersectionNameByIndex(cvIn.vehicleTrackingState.intsectionTrackingState.intersectionIndex) << endl;              
            }
            
            // set cvIn to cv and continue;
            cv.isVehicleInMap = cvIn.isVehicleInMap;
            cv.vehicleTrackingState = cvIn.vehicleTrackingState;  
            cv.vehicleLocationAware = cvIn.vehicleLocationAware;
            cv.vehicleSignalAware = cvIn.vehicleSignalAware;
          }
          else
          {
            OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);  
            OS_ERR << ", failed encode_bsm_payload" << endl;
            OS_Display << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);  
            OS_Display << ", failed encode_bsm_payload" << endl;
          }            
        }
      }
      time2query = false;
    }
    
    /// check whether there is inbound dsrc messages
    ssize_t bytesReceived = recv(fd_wmeListen,recvbuf,sizeof(recvbuf),0);
    if (bytesReceived > 0)
    {
      timeUtils::getFullTimeStamp(fullTimeStamp);
      /// decode wsmp header
      wmeUtils::wsmp_header_t wsmp;
      size_t offset = wmeUtils::decode_wsmp_header((const uint8_t*)&recvbuf[0],bytesReceived,&wsmp);
      if (offset > 0)
      {
        /// get msgid
        uint8_t msgid = wmeUtils::getmsgiddbypsid(wmeUtils::psid2mmitssMsgidMap,wsmp.psid);
        if (msgid == wmeUtils::msgid_spat || msgid == wmeUtils::msgid_map || msgid == wmeUtils::msgid_ssm)
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
            if (logDetails && msgid != wmeUtils::msgid_map)
            {
              int fileIdx = findFileIdx(logtypes,"payload");
              if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
              {
                *(logFiles[fileIdx].OS) << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp) << ",";
                *(logFiles[fileIdx].OS) << wmeUtils::MSGNAME[msgid - wmeUtils::msgid_bsm + 1] << ",payload=";
                logPayloadHex(*(logFiles[fileIdx].OS),&recvbuf[offset],(size_t)wsmp.txlength);
                logFiles[fileIdx].logrows++;
              }     
            }
            switch(msgid)
            {
            case wmeUtils::msgid_spat:
              if (asnJ2735Lib.decode_spat_payload(&recvbuf[offset],(size_t)(wsmp.txlength),&spatIn.spatMsg,NULL))
              {
                spatIn.tms = fullTimeStamp.tms;
                updateSpatList(spatList,spatIn);
                if (changedIntersection && spatIn.spatMsg.id == cv.vehicleLocationAware.intersectionId)
                {
                  changedIntersection = false;
                  OS_Display << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);  
                  OS_Display << ", received SPaT from ";
                  OS_Display << asnJ2735Lib.getIntersectionNameById(spatIn.spatMsg.id) << endl;
                }
              }
              else
              {
                OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
                OS_ERR << ", failed decode_spat_payload, payload=";
                logPayloadHex(OS_ERR,&recvbuf[offset],(size_t)(wsmp.txlength));
                OS_Display << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
                OS_Display << ", failed decode_spat_payload" << endl;                          
              }
              break;
            case wmeUtils::msgid_map:   
              if (asnJ2735Lib.decode_mapdata_payload(&recvbuf[offset],(size_t)(wsmp.txlength),&mapIn.mapMsg,NULL))
              {
                mapIn.tms = fullTimeStamp.tms;
                mapList[mapIn.mapMsg.id] = mapIn;
              }
              else
              {
                OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
                OS_ERR << ", failed decode_mapdata_payload, payload=";
                logPayloadHex(OS_ERR,&recvbuf[offset],(size_t)(wsmp.txlength));
                OS_Display << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
                OS_Display << ", failed decode_mapdata_payload" << endl;                          
              }            
              break;
            case wmeUtils::msgid_ssm:
              if (asnJ2735Lib.decode_ssm_payload(&recvbuf[offset],sizeof(recvbuf),&ssmIn.ssmMsg))
              {
                ssmIn.tms = fullTimeStamp.tms;
                updateSsmList(ssmList,ssmIn);
                if (logFile && ssmIn.ssmMsg.requestNums > 0)
                {
                  int fileIdx = findFileIdx(logtypes,"ssm");
                  if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
                  {
                    logSSM(*(logFiles[fileIdx].OS),ssmIn.ssmMsg,fullTimeStamp.localDateTimeStamp);
                    logFiles[fileIdx].logrows++;
                  }                       
                }
                /// send udp message to display
                if (send2disp && cv.vehicleTrackingState.intsectionTrackingState.vehicleIntersectionStatus != GeoUtils::vehicleInMap_enum_t::NOT_IN_MAP   // vehicle in map
                  && ssmIn.ssmMsg.id == cv.vehicleLocationAware.intersectionId)  // ssm from the intersection that the vehicle is on
                {
                  iteSpatList = spatList.find(cv.vehicleLocationAware.intersectionId);
                  if (iteSpatList != spatList.end())
                  {
                    /// found SPaT from spatList
                    SPAT_element_t intSpat = iteSpatList->second.spatMsg;
                    // 1. number of intersections that the vehicle is receiving SPaT
                    string disppacket = toString<size_t>(spatList.size()) + string(",");      
                    // 2. intersection id
                    uint32_t intId = cv.vehicleLocationAware.intersectionId;
// to test at RFS intersection, set intId as stanford
//intId = 1000;                    
                    disppacket += toString<uint32_t>(intId) + string(","); 
                    // 3. intersection name
                    disppacket += asnJ2735Lib.getIntersectionNameById(intId) + string (","); 
                    // 4. vehicleIntersectionStatus
                    int locStatue;
                    switch(cv.vehicleTrackingState.intsectionTrackingState.vehicleIntersectionStatus)
                    {
                    case GeoUtils::vehicleInMap_enum_t::ON_APPROACH:
                      locStatue = 1;
                      break;
                    case GeoUtils::vehicleInMap_enum_t::ON_EGRESS:
                      locStatue = 3;
                      break;
                    default:
                      locStatue = 2;
                    }
                    disppacket += toString<int>(locStatue) + string (",");
                    // 5. priority request status
                    int prioReqStatus = 0;
                    if (cvRequest.isPriorityRequested)
                      prioReqStatus = 1;
                    else if (cvRequest.isPriorityCancelled)
                      prioReqStatus = 2;
                    disppacket += toString<int>(prioReqStatus) + string (",");
                    // 6-13. signalState
                    bitset<8> permittedPhases = bitset<8>(intSpat.permittedPhases);
                    for (int i = 0; i < 8; i++)
                    {
                      uint32_t signalState = 0;   // not permitted
                      if (permittedPhases.test(i))
                        signalState = intSpat.phaseState[i].currState;  // 1-GREEN, 2-YELLOW, 3-RED
                      disppacket += toString<uint32_t>(signalState) + string (",");
                    }
                    // 14-21 pedSignalState 
                    bitset<8> permittedPedPhases = bitset<8>(intSpat.permittedPedPhases);
                    for (int i = 0; i < 8; i++)
                    {
                      uint32_t pedSignalState = 0;   // not equipped
                      if (permittedPedPhases.test(i))
                        pedSignalState = intSpat.pedPhaseState[i].currState; // 1-STOP, 2-CAUTION, 3-WALK
                      disppacket += toString<uint32_t>(pedSignalState) + string (",");
                    }
                    // 22 ssm requestNums
                    disppacket += toString<int>((int)ssmIn.ssmMsg.requestNums);
                    for (uint8_t i = 0; i < ssmIn.ssmMsg.requestNums; i++)
                    {
                      // 1. tableRowSeq
                      disppacket += string("|") + toString<int>((int)i+1) + string (",");
                      // 2. isPriorityActive
                      int isPriorityActive = 0; // not active
                      if (ssmIn.ssmMsg.status != 0 && ssmIn.ssmMsg.priorityCause == ssmIn.ssmMsg.request[i].id)
                        isPriorityActive = 1;
                      disppacket += toString<int>(isPriorityActive) + string (",");
                      // 3. vehId
                      disppacket += toString<uint32_t>(ssmIn.ssmMsg.request[i].id) + string (",");
                      // 4. vehType
                      int vehType = 1; // bus
                      if (ssmIn.ssmMsg.request[i].classType > 6)
                        vehType = 2; // truck
                      else if (ssmIn.ssmMsg.request[i].classType == 2)
                        vehType = 4; // EV
                      disppacket += toString<int>(vehType) + string (",");
                      // 5. inLaneID
                      disppacket += toString<int>((int)ssmIn.ssmMsg.request[i].inLaneId) + string (",");
                      // 6. outLaneId
                      disppacket += toString<int>((int)ssmIn.ssmMsg.request[i].outLaneId) + string (",");
                      // 7. priorityPhase
                      disppacket += toString<int>((int)ssmIn.ssmMsg.request[i].contolPhase) + string (",");
                      // 8. vehArrvTime
                      disppacket += toString<int>((int)ssmIn.ssmMsg.request[i].etaOffset / 10) + string (",");
                      // 9. vehServiceStartTime
                      disppacket += toString<int>((int)ssmIn.ssmMsg.request[i].timeOfService.hour) + ":";
                      disppacket += toString<int>((int)ssmIn.ssmMsg.request[i].timeOfService.min) + ":";
                      disppacket += toString<int>((int)ssmIn.ssmMsg.request[i].timeOfService.sec / 1000) + string(",");
                      // 10. vehServiceEndTime
                      disppacket += toString<int>((int)ssmIn.ssmMsg.request[i].endOfService.hour) + ":";
                      disppacket += toString<int>((int)ssmIn.ssmMsg.request[i].endOfService.min) + ":";
                      disppacket += toString<int>((int)ssmIn.ssmMsg.request[i].endOfService.sec / 1000) + string(",");
/*                      
                      // 11. vehLocStatus (1 - ingress, 2 - egress)
                      int vehLocStatus = 1;
                      disppacket += toString<int>(vehLocStatus);
*/                      
                      // 11. requestStatus {NOTVALID,REJECTED,NOTNEEDED,QUEUED,ACTIVE,CANCELLED,COMPLETED};
                      disppacket += toString<int>((int)ssmIn.ssmMsg.request[i].requestStatus);
                    }
                    socketUtils::sendall(fd_dispSend,disppacket.c_str(),disppacket.size()+1);     
OS_Display << "send display packet: " << disppacket << endl;
                  }
                }
                
                /// acknowledgement for srm
                if (cvRequest.isPriorityRequested && ssmIn.ssmMsg.id == cvRequest.requestedIntersectionId && ssmIn.ssmMsg.requestNums > 0)
                {                  
                  int ackIdx = - 1;
                  for (uint8_t i = 0; i < ssmIn.ssmMsg.requestNums; i++)
                  {
                    if (ssmIn.ssmMsg.request[i].id == cvRequest.requestedVehicleId)
                    {
                      ackIdx = i;
                      break;
                    }
                  }
                  if (ackIdx >= 0)
                  {
                    if (!cvRequest.isReqAck)
                    {
                      OS_Display << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
                      OS_Display << ", received ssm acknowledgement, priorityStatus ";
                      OS_Display << static_cast<int>(ssmIn.ssmMsg.status);
                      OS_Display << ", requestStatus " <<  static_cast<int>(ssmIn.ssmMsg.request[ackIdx].requestStatus) << endl;
                    }
                    cvRequest.isReqAck = true;
                    cvRequest.requestStatus = ssmIn.ssmMsg.request[ackIdx].requestStatus;
                    cvRequest.priorityStatus = ssmIn.ssmMsg.status;
                    if (logFile)
                    {
                      int fileIdx = findFileIdx(logtypes,"ack");
                      if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
                      {
                        *(logFiles[fileIdx].OS) << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp) << ",";
                        *(logFiles[fileIdx].OS) << cvRequest.isReqAck << ",";
                        *(logFiles[fileIdx].OS) << static_cast<int>(cvRequest.requestStatus) << ",";
                        *(logFiles[fileIdx].OS) << static_cast<int>(cvRequest.priorityStatus) << endl;
                        logFiles[fileIdx].logrows++;
                      }                           
                    }
                  }
                }
              }
              else
              {
                OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
                OS_ERR << ", failed decode_ssm_payload, payload=";
                logPayloadHex(OS_ERR,&recvbuf[offset],(size_t)(wsmp.txlength));
                OS_Display << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
                OS_Display << ", failed decode_ssm_payload" << endl;                          
              }            
              break;
            default:
              break;
            }
          }
        }
      }
      else
      {
        OS_ERR << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
        OS_ERR << ", failed decode_wsmp_header, wsm=";
        logPayloadHex(OS_ERR,recvbuf,(size_t)bytesReceived);        
        OS_Display << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
        OS_Display << ", failed decode_wsmp_header" << endl;
      }
    }

    /// check whether need to send SRM
    timeUtils::getFullTimeStamp(fullTimeStamp);   
    if (myVin.isPrioEligible && fullTimeStamp.tms > cvRequest.requestedTms + srmInterval)
    {
      // determine priority request action
      priorityRequestAction = priorityRequestAction_enum_t::NONE;
      if (!cvRequest.isPriorityRequested)
      {
        if (cv.isVehicleInMap && cv.vehicleTrackingState.intsectionTrackingState.vehicleIntersectionStatus == GeoUtils::vehicleInMap_enum_t::ON_APPROACH)
        {
          // has not requested priority yet, now OBU is on an approach
          iteSpatList = spatList.find(cv.vehicleLocationAware.intersectionId);
//          if (iteSpatList != spatList.end())
//          {
            // has received SPaT from the approach intersection
            priorityRequestAction = priorityRequestAction_enum_t::INITIATE; // needs to initiate SRM
//          }
        }
      }
      else if (cv.isVehicleInMap && cv.vehicleTrackingState.intsectionTrackingState.vehicleIntersectionStatus != GeoUtils::vehicleInMap_enum_t::INSIDE_INTERSECTION_BOX
        && cv.vehicleTrackingState.intsectionTrackingState.intersectionIndex == cvRequest.requestedIntersectionIndex
        && cv.vehicleTrackingState.intsectionTrackingState.approachIndex == cvRequest.requestedApproachedIndex)
      {
        // remains on the same approach
        priorityRequestAction = priorityRequestAction_enum_t::KEEPGOING;
      }
      else
      {
        // changed approach or intersection
        priorityRequestAction = priorityRequestAction_enum_t::CANCEL;
      }

      if (priorityRequestAction != priorityRequestAction_enum_t::NONE)
      {
        //  update cvRequest & cvSRM based on latest map matching result stored in cv and latest BSM bolb1 stored in bsmout
        updatePriorityRequest(cvRequest,cv,fullTimeStamp.tms,priorityRequestAction);
        updateSRM(cvSRM,cvRequest,asnJ2735Lib,&(bsmout.bolb1_element),priorityRequestAction);
        // encode SRM        
        ssize_t payload_size = asnJ2735Lib.encode_srm_payload(&cvSRM,&sendbuf[sizeof(mmitss_udp_header_t)],sizeof(sendbuf),false);
        if (payload_size > 0)
        {
          mmitss_udp_header_t header;
          header.msgheader = msg_header;
          header.msgid = wmeUtils::msgid_srm;
          header.ms_since_midnight = fullTimeStamp.ms_since_midnight;
          header.length = static_cast<uint16_t>(payload_size);
          memcpy(&sendbuf[0],&header,sizeof(mmitss_udp_header_t));
          socketUtils::sendall(fd_wmeSend,sendbuf,(size_t)payload_size+sizeof(mmitss_udp_header_t));
          if (logDetails)
          {              
            int fileIdx = findFileIdx(logtypes,"payload");
            if (fileIdx >= 0 && (size_t)fileIdx < logtypes.size() && logFiles[fileIdx].isOpened)
            {
              *(logFiles[fileIdx].OS) << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp) << ",SRM,payload=";
              logPayloadHex(*(logFiles[fileIdx].OS),&sendbuf[sizeof(mmitss_udp_header_t)],(size_t)payload_size);
              logFiles[fileIdx].logrows++;
            }
          }
          if (priorityRequestAction == priorityRequestAction_enum_t::INITIATE)
          {
            OS_Display << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
            OS_Display << ", start sending SRM to ";
            OS_Display << asnJ2735Lib.getIntersectionNameById(cvSRM.signalRequest_element.id) << endl;
          }
          else if (priorityRequestAction == priorityRequestAction_enum_t::CANCEL)
          {
            OS_Display << timeUtils::getTimestampStr(fullTimeStamp.localDateTimeStamp);
            OS_Display << ", cancel priority at ";
            OS_Display << asnJ2735Lib.getIntersectionNameById(cvSRM.signalRequest_element.id) << endl;
          }
        }
      }
    }
    
    /// update lists (remove entry if it's outdated by timeOutms milliseconds)
    timeUtils::getFullTimeStamp(fullTimeStamp);   
    cleanSpatList(spatList,fullTimeStamp.tms,timeOutms);
    cleanMapList(mapList,fullTimeStamp.tms,timeOutms);
    cleanSsmList(ssmList,fullTimeStamp.tms,timeOutms);
    
    /// reset cv if own BSM timeout expiries to make sure SRM can be cancelled 
    if (fullTimeStamp.tms > cv.ts + timeOutms)
      cv.reset();
    
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

void initialSRM(SRM_element_t& cvSRM,OBUconfig::VinConfig& vehvin)
{
  cvSRM.msgCnt = 0;
  cvSRM.signalRequest_element.id = 0;
  cvSRM.signalRequest_element.requestedAction = MsgEnum::priorityrequest_enum_t::UNKNOWREQUEST;
  cvSRM.signalRequest_element.inLaneId = 0;
  cvSRM.signalRequest_element.outLaneId = 0;
  cvSRM.signalRequest_element.NTCIPVehicleclass.NTCIPvehicleClass_type = vehvin.vehType;
  cvSRM.signalRequest_element.NTCIPVehicleclass.NTCIPvehicleClass_level = vehvin.prioLevel;
  strcpy(cvSRM.signalRequest_element.codeWord,vehvin.codeWord.c_str());
  cvSRM.timeOfService.hour = 0;
  cvSRM.timeOfService.min = 0;
  cvSRM.timeOfService.sec = 0;
  cvSRM.endOfService.hour = 0;
  cvSRM.endOfService.min = 0;
  cvSRM.endOfService.sec = 0;
  cvSRM.transitStatus = 0;
  strcpy(cvSRM.vehIdent_element.vehName,vehvin.name.c_str());
  strcpy(cvSRM.vehIdent_element.vehVin,vehvin.vin.c_str());
  strcpy(cvSRM.vehIdent_element.vehOwnerCode,vehvin.ownerCode.c_str());
  cvSRM.vehIdent_element.vehId = 0;
  cvSRM.vehIdent_element.vehType = vehvin.vehType;
  memset(cvSRM.BSMblob,0,BSMBLOB1SIZE);
  cvSRM.requestStatus = 0;
}

uint8_t getOutlaneId(const std::vector<GeoUtils::connectTo_t>& aConnectTo)
{
  uint8_t ret = 0;
  if (aConnectTo.empty())
    return (0);
  for (size_t i = 0; i < aConnectTo.size(); i++)
  {
    if (aConnectTo[i].laneManeuver == MsgEnum::map_enum_t::STRAIGHTAHEAD 
      || aConnectTo[i].laneManeuver == MsgEnum::map_enum_t::STRAIGHT)
    {
      ret = aConnectTo[i].laneId;
      break;
    }
  }
  if (ret > 0)
    return (ret);
  for (size_t i = 0; i < aConnectTo.size(); i++)
  {
    if (aConnectTo[i].laneManeuver == MsgEnum::map_enum_t::LEFTTURN) 
    {
      ret = aConnectTo[i].laneId;
      break;
    }
  }
  if (ret > 0)
    return (ret);
  for (size_t i = 0; i < aConnectTo.size(); i++)
  {
    if (aConnectTo[i].laneManeuver == MsgEnum::map_enum_t::RIGHTTURN) 
    {
      ret = aConnectTo[i].laneId;
      break;
    }
  }
  if (ret > 0)
    return (ret);
  return (aConnectTo[0].laneId);
}

void updatePriorityRequest(PriorityRequest_t& request,const GeoUtils::connectedVehicle_t& cv,const long long tms,const priorityRequestAction_enum_t::Action action)
{
  request.requestedmsgCnt = (uint8_t)((request.requestedmsgCnt + 1) % 128);
  switch(action)
  {
  case priorityRequestAction_enum_t::CANCEL:
    request.isPriorityRequested = false;
    request.isPriorityCancelled = true;
    request.isReqAck = false;
    // obu may moved out the intersection, keep the info for the last priority request
    break;
  default:
    request.isPriorityRequested = true;
    request.isPriorityCancelled = false;
    if (action == priorityRequestAction_enum_t::INITIATE) {request.isReqAck = false;}
    // update request
    request.requestedTms = tms;
    request.requestedEta = tms + getTime2goLL(cv.vehicleLocationAware.dist2go.distLong,cv.motionState.speed);
    request.requestedIntersectionIndex = static_cast<uint8_t>(cv.vehicleTrackingState.intsectionTrackingState.intersectionIndex);
    request.requestedApproachedIndex = static_cast<uint8_t>(cv.vehicleTrackingState.intsectionTrackingState.approachIndex);
    request.requestedLaneIndex = static_cast<uint8_t>(cv.vehicleTrackingState.intsectionTrackingState.laneIndex); 
    request.requestedVehicleId = cv.id;
    request.requestedIntersectionId = cv.vehicleLocationAware.intersectionId;
    request.requestedInlaneId = cv.vehicleLocationAware.laneId;
    request.requestedOutlaneId = getOutlaneId(cv.vehicleLocationAware.connect2go);
    request.requestedControlPhase = cv.vehicleLocationAware.controlPhase;   
  }
}

void updateSRM(SRM_element_t& cvSRM,const PriorityRequest_t& request,const AsnJ2735Lib& lib,const BOLB1_element_t* ps_bsmblob1,const priorityRequestAction_enum_t::Action action)
{
  cvSRM.msgCnt = request.requestedmsgCnt;
  cvSRM.signalRequest_element.id = request.requestedIntersectionId;
  if (action == priorityRequestAction_enum_t::CANCEL)
  {
    cvSRM.signalRequest_element.requestedAction = static_cast<uint8_t>(MsgEnum::priorityrequest_enum_t::CANCELPRIORITY);
  }
  else
  {
    cvSRM.signalRequest_element.requestedAction = static_cast<uint8_t>(MsgEnum::priorityrequest_enum_t::REQUESTPRIORITY);
  }
  cvSRM.signalRequest_element.inLaneId = request.requestedInlaneId;
  cvSRM.signalRequest_element.outLaneId = request.requestedOutlaneId; 
  timeUtils::dateTimeStamp_t ts;
  timeUtils::timeStampFrom_time_t(static_cast<time_t>(request.requestedTms/1000LL),ts,false);
  dts2DTime(cvSRM.timeOfService,ts,static_cast<uint16_t>(request.requestedTms - (request.requestedTms/1000LL) * 1000LL));
  timeUtils::timeStampFrom_time_t(static_cast<time_t>(request.requestedEta/1000LL),ts,false);
  dts2DTime(cvSRM.endOfService,ts,static_cast<uint16_t>(request.requestedEta - (request.requestedEta/1000LL) * 1000LL));
  cvSRM.transitStatus = 0;
  cvSRM.vehIdent_element.vehId = request.requestedVehicleId;
  lib.encode_bsmblob1_payload(ps_bsmblob1,(u_char*)cvSRM.BSMblob);
  cvSRM.requestStatus = 0;
}

void dts2DTime(DTime_element_t& dt,const timeUtils::dateTimeStamp_t& ts,const uint16_t millitm)
{
  /// in utc time
  dt.hour = static_cast<uint8_t>(ts.timeStamp.hour);
  dt.min = static_cast<uint8_t>(ts.timeStamp.min);
  dt.sec = static_cast<uint16_t>(ts.timeStamp.sec * 1000 + millitm);  /// DE_DSecond in milliseconds
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
