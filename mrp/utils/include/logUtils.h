//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************
#ifndef _LOGFILE_UTILS_H
#define _LOGFILE_UTILS_H

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

namespace logUtils
{
	enum class logType {none, simpleLog, detailLog};

	struct Logfile_t
	{
		bool isOpened;
		std::ofstream* OS;
		std::string prefix;
		std::string type;
		std::string suffix;
		std::string fullname;
		unsigned long logrows;
		Logfile_t(std::string prefix_, std::string type_)
		{
			isOpened = false;
			prefix = prefix_;
			type = type_;
			logrows = 0;
		};
	};

	bool openLogFiles(std::vector<logUtils::Logfile_t>& filelist, const std::string& suffix);
	bool reOpenLogFiles(std::vector<logUtils::Logfile_t>& filelist, const std::string& suffix);
	void closeLogFiles(std::vector<logUtils::Logfile_t>& filelist);
	void logMsg(std::vector<logUtils::Logfile_t>& filelist, const std::string& type, std::vector<uint8_t>& msg, size_t size, bool receive=false);
	void logMsg(std::vector<logUtils::Logfile_t>& filelist, const std::string& type, std::vector<uint8_t>& msg, size_t size, uint32_t  msOfDay);
	void logMsgHex(std::ofstream& OS, const uint8_t* buf, size_t size);
}

#endif
