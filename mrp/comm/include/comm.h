//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************
#ifndef _MMITSSCOMM_H
#define _MMITSSCOMM_H

#include <fstream>
#include <stdint.h> // c++11 <cstdint>
#include <string>
#include <cstring>
#include <map>
#include <vector>

#include "dsrcMsgs.h"
#include "timeUtils.h"
#include "geoUtils.h"

const double defaultSpeed = 10.0; //in m/s

// structure to hold received BSM (could be multiple cars)
struct BSM_List_t
{
  long long tms;
  BSM_element_t bsmMsg;
};

// structure to hold received SRM (could be multiple cars)
struct SRM_List_t
{
  long long tms;
  SRM_element_t   srmMsg;
  BOLB1_element_t srmBlob1;
  bool            isCancel;
  uint8_t         requestedPhase;
  long long       requestedServiceStartTms; /// utc time
  long long       requestedServiceEndTms;   /// utc time
};

// structure to hold received SPaT (could be multiple intersections)
struct SPAT_List_t
{
  long long tms;
  SPAT_element_t spatMsg;
};

// structure to hold received MAP (could be multiple intersections)
struct MAP_List_t
{
  long long tms;
  Mapdata_element_t mapMsg;
};

// structure to hold received SSM (could be multiple intersections)
struct SSM_List_t
{
  long long tms;
  SSM_element_t ssmMsg;
};

// structure for log file
struct Logfile_t
{
  bool isOpened;
  std::ofstream* OS;
  std::string logpath;
  std::string filename;
  std::string filetype;
  std::string fullname;
  unsigned long logrows;
  Logfile_t(std::string logpath_,std::string filename_,std::string filetype_)
  {
    isOpened = false;
    logpath = logpath_;
    filename = filename_;
    filetype = filetype_;
    logrows = 0;
  };
};

std::string getLogfileName(const timeUtils::dateTimeStamp_t& ts,const std::string& pathName,
  const std::string& name,const char* type);
bool openLogFile(std::ofstream& OS,std::string& fileName,const std::string& logFilePath,
  const timeUtils::dateTimeStamp_t& localts,const std::string& name,const char* type);
bool openLogFiles(std::vector<Logfile_t>& filelist,const timeUtils::dateTimeStamp_t& ts);
bool reOpenLogFiles(std::vector<Logfile_t>& filelist,const timeUtils::dateTimeStamp_t& ts);
void closeLogFiles(std::vector<Logfile_t>& filelist);
int  findFileIdx(const std::vector<std::string>& types,const char* type);
void updateBsmList(std::map<uint32_t,BSM_List_t>& list,const BSM_List_t& bsmIn);
void updateSpatTms(const uint16_t spat_timeStamp,const timeUtils::fullTimeStamp_t& timestamp,long long& tms);
void updateSpatList(std::map<uint32_t,SPAT_List_t>& list,const SPAT_List_t& spatIn);
void updateSrmList(std::map<uint32_t,SRM_List_t>& list,const SRM_List_t& srmIn);  
void updateSsmList(std::map<uint32_t,SSM_List_t>& list,const SSM_List_t& ssmIn);          
void cleanBsmList(std::map<uint32_t,BSM_List_t>& list,const long long tms, const long long timeout);
void cleanSrmList(std::map<uint32_t,SRM_List_t>& list,const long long tms, const long long timeout);
void cleanSpatList(std::map<uint32_t,SPAT_List_t>& list,const long long tms, const long long timeout);
void cleanMapList(std::map<uint32_t,MAP_List_t>& list,const long long tms,const long long timeout);
void cleanSsmList(std::map<uint32_t,SSM_List_t>& list,const long long tms,const long long timeout);
SPAT_List_t getSpatFromList(const std::map<uint32_t,SPAT_List_t>& list,const uint32_t id);
void updateSignalAware(const std::map<uint32_t,SPAT_List_t>& list,const GeoUtils::locationAware_t& locationAware,GeoUtils::signalAware_t& signalAware);
uint16_t getTime2goDeci(const double& dist2go, const double& speed);
long long getTime2goLL(const double& dist2go, const double& speed);
double getSpeed(const double& speed);
void logAware(std::ofstream& OS,const timeUtils::dateTimeStamp_t& ts,const uint8_t msgCnt,const GeoUtils::connectedVehicle_t &cv);
void logPayload(std::ofstream& OS,const timeUtils::dateTimeStamp_t& ts,const char* buf,const size_t size,const char* msgName);
void hexPrint(std::ofstream& OS,const char* buf,const size_t size);

#endif
