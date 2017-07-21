//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************

#include <ctime>
#include "timeUtils.h"

void timeUtils::getFullTimeStamp(fullTimeStamp_t& fullTimeStamp)
{
	struct timeb timeptr_raw;
	ftime(&timeptr_raw);
	timeUtils::timeStampFrom_timeb(timeptr_raw, fullTimeStamp.utcDateTimeStamp, false);
	timeUtils::timeStampFrom_timeb(timeptr_raw, fullTimeStamp.localDateTimeStamp, true);
	fullTimeStamp.msec = (unsigned long long)timeptr_raw.time * 1000 + timeptr_raw.millitm;
}

void timeUtils::timeStampFrom_timeb(const struct timeb& rawTime, dateTimeStamp_t& convectedTime, bool isLocal)
{
	struct tm lcl;
	if (isLocal)
		localtime_r(&(rawTime.time),&lcl);
	else
		gmtime_r(&(rawTime.time),&lcl);
	convectedTime.dateStamp.year  = lcl.tm_year + 1900;
	convectedTime.dateStamp.month = lcl.tm_mon + 1;
	convectedTime.dateStamp.day   = lcl.tm_mday;
	convectedTime.timeStamp.hour  = lcl.tm_hour;
	convectedTime.timeStamp.min   = lcl.tm_min;
	convectedTime.timeStamp.sec   = lcl.tm_sec;
	convectedTime.timeStamp.millisec = rawTime.millitm;
	convectedTime.minuteOfYear = static_cast<uint32_t>((rawTime.time % 31536000) / 60);
	convectedTime.msOfMinute = static_cast<uint16_t>(((lcl.tm_min * 60) + lcl.tm_sec) * 1000 + rawTime.millitm);
	convectedTime.msOfDay = static_cast<uint32_t>(((lcl.tm_hour * 3600) + (lcl.tm_min * 60) + lcl.tm_sec) * 1000 + rawTime.millitm);
}
