//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************
#ifndef _TIME_UITLS_H
#define _TIME_UITLS_H

#include <cstdint>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <sys/timeb.h>

namespace timeUtils
{
	struct dateStamp_t
	{
		int year;
		int month;
		int day;
		bool operator!=(const timeUtils::dateStamp_t& p) const
		{
			return ((year != p.year) || (month != p.month) || (day != p.day));
		};
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
		uint32_t  minuteOfYear;
		uint32_t  msOfDay;
		uint16_t  msOfMinute;
		std::string to_timeStr(char delimiter) const
		{
			std::stringstream ss;
			ss << std::setw(2) << std::setfill('0') << timeStamp.hour << delimiter;
			ss << std::setw(2) << std::setfill('0') << timeStamp.min << delimiter;
			ss << std::setw(2) << std::setfill('0') << timeStamp.sec << '.';
			ss << std::setw(3) << std::setfill('0') << timeStamp.millisec;
			return(ss.str());
		};
		std::string to_dateStr(char delimiter) const
		{
			std::stringstream ss;
			ss << std::setw(4) << std::setfill('0') << dateStamp.year << delimiter;
			ss << std::setw(2) << std::setfill('0') << dateStamp.month << delimiter;
			ss << std::setw(2) << std::setfill('0') << dateStamp.day;
			return(ss.str());
		};
		std::string to_dateTimeStr(char date_delimiter,char time_delimiter) const
		{
			std::stringstream ss;
			ss << std::setw(4) << std::setfill('0') << dateStamp.year << date_delimiter;
			ss << std::setw(2) << std::setfill('0') << dateStamp.month << date_delimiter;
			ss << std::setw(2) << std::setfill('0') << dateStamp.day << 'T';
			ss << std::setw(2) << std::setfill('0') << timeStamp.hour << time_delimiter;
			ss << std::setw(2) << std::setfill('0') << timeStamp.min << time_delimiter;
			ss << std::setw(2) << std::setfill('0') << timeStamp.sec << '.';
			ss << std::setw(3) << std::setfill('0') << timeStamp.millisec;
			return(ss.str());
		};
		std::string to_fileName(void) const
		{
			std::stringstream ss;
			ss << std::setw(4) << std::setfill('0') << dateStamp.year;
			ss << std::setw(2) << std::setfill('0') << dateStamp.month;
			ss << std::setw(2) << std::setfill('0') << dateStamp.day << '-';
			ss << std::setw(2) << std::setfill('0') << timeStamp.hour;
			ss << std::setw(2) << std::setfill('0') << timeStamp.min;
			ss << std::setw(2) << std::setfill('0') << timeStamp.sec;
			return(ss.str() + std::string(".txt"));
		};
	};

	struct fullTimeStamp_t
	{
		timeUtils::dateTimeStamp_t utcDateTimeStamp;
		timeUtils::dateTimeStamp_t localDateTimeStamp;
		unsigned long long msec;  // milliseconds since the UNIX epoch
	};

	void getFullTimeStamp(timeUtils::fullTimeStamp_t& fullTimeStamp);
	void timeStampFrom_timeb(const struct timeb& rawTime, timeUtils::dateTimeStamp_t& convectedTime, bool isLocal);
};

#endif
