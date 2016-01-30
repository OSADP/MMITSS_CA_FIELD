#ifndef _TIME_UITLS_H
#define _TIME_UITLS_H

#include <stdint.h>   // c++11 <cstdint>
#include <ctime>
#include <string>
#include <cstring>

namespace timeUtils
{
  struct dateStamp_t
  {
    int year;
    int month;
    int day;
  };
  
  struct timeStamp_t
  {
    int hour;
    int min;
    int sec;
    unsigned short millisec;
  };
  
  struct dateTimeStamp_t
  {
    timeUtils::dateStamp_t dateStamp;
    timeUtils::timeStamp_t timeStamp;
  };

  struct fullTimeStamp_t
  {
    long long tms;                        // UTC milliseconds since epoch begin
    long long tms_minute;                 // UTC milliseconds until minute
    long long tms_midnight;               // UTC milliseconds until midnight
    long long tms_midnight_local;         // milliseconds until midnight (local time)
    unsigned int ms_since_midnight;       // UTC milliseconds since midnight
    unsigned int ms_since_midnight_local; // milliseconds since midnight (local time)
    timeUtils::dateTimeStamp_t utcDateTimeStamp;
    timeUtils::dateTimeStamp_t localDateTimeStamp;
  };
  
  void getFullTimeStamp(timeUtils::fullTimeStamp_t& fullTimeStamp);
  void timeStampFrom_time_t(const time_t rawTime,timeUtils::dateTimeStamp_t& convectedTime,const bool isLocal);
  void timeStampFrom_ms(const long long ms,dateTimeStamp_t& convectedTime,const bool isLocal);
  std::string getTimestampStr(const timeUtils::dateTimeStamp_t& ts);
  std::string getTimestampStr(const unsigned int ms_since_midnight);
  unsigned int getMsSinceMidnight(const timeUtils::timeStamp_t& ts);
} 

#endif
