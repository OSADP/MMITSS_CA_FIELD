#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <algorithm>  // find
#include <iterator>   // distance
#include <iomanip>    // setw, setfill

#include "socketUtils.h"
#include "comm.h"

using namespace std;

string getLogfileName(const timeUtils::dateTimeStamp_t& ts,const string& pathName,const string& name,const char* type)
{
  return (pathName + string("/") + name + string(".") + string(type) + string(".") + timeUtils::getTimestampStr(ts) + string(".txt"));
}

bool openLogFile(ofstream& OS,string& fileName,const string& logFilePath,
  const timeUtils::dateTimeStamp_t& localts,const string& name,const char* type)
{
  fileName = getLogfileName(localts,logFilePath,name,type);
  OS.open(fileName.c_str());
  if (!OS.is_open())
    return false;
  OS.precision(8);
  OS.setf(ios::showpoint | ios::fixed );    
  return true;
}

bool openLogFiles(std::vector<Logfile_t>& filelist,const timeUtils::dateTimeStamp_t& ts)
{
  bool ret = true;
  for (size_t i = 0; i < filelist.size(); i++)
  { 
    filelist[i].OS = new ofstream();
    filelist[i].isOpened = openLogFile(*(filelist[i].OS),filelist[i].fullname,filelist[i].logpath,ts,
      filelist[i].filename,filelist[i].filetype.c_str());
    filelist[i].logrows = 0;
    if (!filelist[i].isOpened) {ret = false;delete filelist[i].OS;}
  }
  return (ret);
}

bool reOpenLogFiles(std::vector<Logfile_t>& filelist,const timeUtils::dateTimeStamp_t& ts)
{
  bool ret = true;  
  for (size_t i = 0; i < filelist.size(); i++)
  {
    if (filelist[i].isOpened)
    {
      filelist[i].OS->close();
      if (filelist[i].logrows == 0) {std::remove(filelist[i].fullname.c_str());}
      delete filelist[i].OS;
    }
    filelist[i].OS = new ofstream();    
    filelist[i].isOpened = openLogFile(*(filelist[i].OS),filelist[i].fullname,filelist[i].logpath,ts,
      filelist[i].filename,filelist[i].filetype.c_str());
    filelist[i].logrows = 0;  
    if (!filelist[i].isOpened) {ret = false;delete filelist[i].OS;}
  }
  return (ret);
}

void closeLogFiles(std::vector<Logfile_t>& filelist)
{
  for (size_t i = 0; i < filelist.size(); i++)
  {
    if (filelist[i].isOpened) {filelist[i].OS->close();delete filelist[i].OS;}
  }
}

int findFileIdx(const std::vector<std::string>& types,const char* type)
{
  int idx = -1;
  vector<string>::const_iterator ite = find (types.begin(),types.end(),type);
  if (ite != types.end()) {idx = static_cast<int>(std::distance(types.begin(),ite));}
  return idx;
}

void updateBsmList(map<uint32_t,BSM_List_t>& list,const BSM_List_t& bsmIn)
{
  map<uint32_t, BSM_List_t>::iterator it = list.find(bsmIn.bsmMsg.bolb1_element.id);
  if (it == list.end() || it->second.tms < bsmIn.tms)
  {
    // only update if spat not in list or it's a new update
    list[bsmIn.bsmMsg.bolb1_element.id] = bsmIn;
  }
}

void updateSpatTms(const uint16_t spat_timeStamp,const timeUtils::fullTimeStamp_t& timestamp,long long& tms)
{
  if (spat_timeStamp > 12000)
    tms = timestamp.tms;
  else
  {
    long long spat_dsec = static_cast<long long>(spat_timeStamp % 600) * 100LL; // milliseconds in minute
    if (spat_dsec > timestamp.tms - timestamp.tms_minute)
    {
      // spat was at the previous minute
      tms = timestamp.tms_minute - (60 * 1000LL) + spat_dsec;
    }
    else
    {
      // spat was at the current minute
      tms = timestamp.tms_minute + spat_dsec;
    }
  }
}

void updateSpatList(map<uint32_t,SPAT_List_t>& list,const SPAT_List_t& spatIn)
{
  map<uint32_t, SPAT_List_t>::iterator it = list.find(spatIn.spatMsg.id);
  if (it == list.end() || it->second.tms < spatIn.tms)
  {
    // only update if spat not in list or it's a new update
    list[spatIn.spatMsg.id] = spatIn;
  }
}

void updateSrmList(std::map<uint32_t,SRM_List_t>& list,const SRM_List_t& srmIn)
{
  map<uint32_t, SRM_List_t>::iterator it = list.find(srmIn.srmMsg.signalRequest_element.id);
  if (it == list.end() || it->second.tms < srmIn.tms)
  {
    // only update if spat not in list or it's a new update
    list[srmIn.srmMsg.signalRequest_element.id] = srmIn;
  }
}

void updateSsmList(std::map<uint32_t,SSM_List_t>& list,const SSM_List_t& ssmIn)
{
  map<uint32_t, SSM_List_t>::iterator it = list.find(ssmIn.ssmMsg.id);
  if (it == list.end() || it->second.tms < ssmIn.tms)
  {
    // only update if spat not in list or it's a new update
    list[ssmIn.ssmMsg.id] = ssmIn;
  }
}

void cleanBsmList(map<uint32_t,BSM_List_t>& list,const long long tms, const long long timeout)
{
  for(map<uint32_t,BSM_List_t>::iterator it = list.begin(); it != list.end();) 
  {
    if (tms > it->second.tms + timeout)
      list.erase(it++);
    else
      ++it;
  }
}

void cleanSrmList(std::map<uint32_t,SRM_List_t>& list,const long long tms, const long long timeout)
{
  for(map<uint32_t,SRM_List_t>::iterator it = list.begin(); it != list.end();) 
  {
    if (tms > it->second.tms + timeout)
      list.erase(it++);
    else
      ++it;
  }
}

void cleanSpatList(map<uint32_t,SPAT_List_t>& list,const long long tms, const long long timeout)
{
  for(map<uint32_t,SPAT_List_t>::iterator it = list.begin(); it != list.end();) 
  {
    if (tms > it->second.tms + timeout)
      list.erase(it++);
    else
      ++it;
  }
}

void cleanMapList(map<uint32_t,MAP_List_t>& list,const long long tms,const long long timeout)
{
  for(map<uint32_t,MAP_List_t>::iterator it = list.begin(); it != list.end();) 
  {
    if (tms > it->second.tms + timeout)
      list.erase(it++);
    else
      ++it;
  }
}

void cleanSsmList(std::map<uint32_t,SSM_List_t>& list,const long long tms,const long long timeout)
{
  for(map<uint32_t,SSM_List_t>::iterator it = list.begin(); it != list.end();) 
  {
    if (tms > it->second.tms + timeout)
      list.erase(it++);
    else
      ++it;
  }
}

SPAT_List_t getSpatFromList(const map<uint32_t,SPAT_List_t>& list,const uint32_t id)
{
  SPAT_List_t spatOut;
  spatOut.tms = 0;
  map<uint32_t,SPAT_List_t>::const_iterator it = list.find(id);
  if (it == list.end())
  {
    return spatOut;
  }
  else
  {
    return (it->second);
  }
}

void updateSignalAware(const map<uint32_t,SPAT_List_t>& list,const GeoUtils::locationAware_t& locationAware,GeoUtils::signalAware_t& signalAware)
{
  signalAware.reset();
  int phasenum = locationAware.controlPhase;
  if (phasenum > 0)
  {
    phasenum--;
    // find latest spat from spatList
    SPAT_List_t spatOut = getSpatFromList(list,locationAware.intersectionId);
    if (spatOut.tms > 0)
    {
      PhaseState_element_t phaseState = spatOut.spatMsg.phaseState[phasenum];
      signalAware.currState = static_cast<MsgEnum::phasestate_enum_t::vehicular>(phaseState.currState);
      signalAware.timeToChange = phaseState.timeToChange;
      signalAware.stateConfidence = static_cast<MsgEnum::phasestate_enum_t::confidentce>(phaseState.stateConfidence);
      signalAware.nextState = static_cast<MsgEnum::phasestate_enum_t::vehicular>(phaseState.nextState);
      signalAware.clearanceIntv = phaseState.clearanceIntv;           
      signalAware.yellStateConfidence = static_cast<MsgEnum::phasestate_enum_t::confidentce>(phaseState.yellStateConfidence);
    }
  }
}

uint16_t getTime2goDeci(const double& dist2go, const double& speed)
{
  return (static_cast<uint16_t>(fabs(dist2go)/getSpeed(speed) * 10)); // in deciseconds
}

long long getTime2goLL(const double& dist2go, const double& speed)
{
  return (static_cast<long long>(fabs(dist2go)/getSpeed(speed) * 1000LL));  // in milliseconds
}

double getSpeed(const double& speed)
{
  if (speed > 2.0)
    return (speed);
  else
    return (defaultSpeed);
}

void logAware(std::ofstream& OS,const timeUtils::dateTimeStamp_t& ts,const uint8_t msgCnt,const GeoUtils::connectedVehicle_t &cv)
{
  OS << dec;
  OS << timeUtils::getTimestampStr(ts) << ",";
  OS << static_cast<int>(msgCnt) << ",";
  OS << cv.id << ",";
  OS << cv.geoPoint.latitude << ",";
  OS << cv.geoPoint.longitude << ",";
  OS << cv.geoPoint.elevation << ",";
  OS << cv.motionState.speed << ",";
  OS << cv.motionState.heading << ",";
  OS << static_cast<int>(cv.vehicleTrackingState.intsectionTrackingState.vehicleIntersectionStatus) << ",";
  OS << cv.vehicleLocationAware.intersectionId << ",";
  OS << static_cast<int>(cv.vehicleLocationAware.laneId) << ",";
  OS << static_cast<int>(cv.vehicleLocationAware.controlPhase) << ",";  
  OS << cv.vehicleLocationAware.dist2go.distLong << ",";
  OS << cv.vehicleLocationAware.dist2go.distLat << ",";
  OS << static_cast<int>(cv.vehicleSignalAware.currState) << ",";
  OS << static_cast<int>(cv.vehicleSignalAware.timeToChange) << ",";
  OS << static_cast<int>(cv.vehicleSignalAware.stateConfidence) << ",";
  OS << static_cast<int>(cv.vehicleSignalAware.nextState) << ",";
  OS << static_cast<int>(cv.vehicleSignalAware.clearanceIntv) << ",";
  OS << static_cast<int>(cv.vehicleSignalAware.yellStateConfidence);
  OS << dec << endl;
}

void logPayload(std::ofstream& OS,const timeUtils::dateTimeStamp_t& ts,const char* buf,const size_t size,const char* msgName)
{
  OS << timeUtils::getTimestampStr(ts) << ", msgName = " << msgName << ", payload=";
  hexPrint(OS,buf,size);
} 

void hexPrint(std::ofstream& OS,const char* buf,const size_t size)
{
  OS << hex;
  for (size_t i=0; i<size;i++)
  {
    OS << uppercase << setw(2) << setfill('0') << static_cast<unsigned int>((unsigned char)buf[i]);
  }
  OS << dec << endl;
}

