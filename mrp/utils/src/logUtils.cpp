//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************
#include <algorithm>
#include <cstdio>
#include <iomanip>
#include <iostream>

#include "logUtils.h"

bool logUtils::openLogFiles(std::vector<logUtils::Logfile_t>& filelist, const std::string& suffix)
{
	bool has_error = false;
	for (auto& log : filelist)
	{
		log.OS = new std::ofstream();
		log.logrows = 0;
		log.suffix = suffix;
		log.fullname = log.prefix + std::string(".") + log.type + std::string(".")  + suffix;
		log.OS->open(log.fullname, std::ofstream::out | std::ofstream::binary);
		log.isOpened = log.OS->is_open();
		if (!log.isOpened)
		{
			has_error = true;
			delete log.OS;
			break;
		}
	}
	return(!has_error);
}

bool logUtils::reOpenLogFiles(std::vector<logUtils::Logfile_t>& filelist, const std::string& suffix)
{
	bool has_error = false;
	for (auto& log : filelist)
	{
		if (log.isOpened)
		{
			log.OS->close();
			if (log.logrows == 0)
				std::remove(log.fullname.c_str());
			delete log.OS;
		}
		log.OS = new std::ofstream();
		log.logrows = 0;
		log.suffix = suffix;
		log.fullname = log.prefix + std::string(".") + log.type + std::string(".")  + suffix;
		log.OS->open(log.fullname, std::ofstream::out | std::ofstream::binary);
		log.isOpened = log.OS->is_open();
		if (!log.isOpened)
		{
			has_error = true;
			delete log.OS;
		}
	}
	return(!has_error);
}

void logUtils::closeLogFiles(std::vector<logUtils::Logfile_t>& filelist)
{
	for (auto& log : filelist)
	{
		if (log.isOpened)
		{
			log.OS->close();
			if (log.logrows == 0)
				std::remove(log.fullname.c_str());
			delete log.OS;
		}
	}
}

void logUtils::logMsg(std::vector<logUtils::Logfile_t>& filelist, const std::string& type, std::vector<uint8_t>& msg, size_t size, bool receive /*=false*/)
{
	auto it = std::find_if(filelist.begin(), filelist.end(), [&type](Logfile_t& log){return(log.type == type);});
	if ((it != filelist.end()) && (it->isOpened))
	{
		char flag = receive ? 1 : 0;
		it->OS->write(&flag, 1);
		it->OS->write((char *)&msg[0], size);
		*(it->OS) << std::endl;
		(it->logrows)++;
	}
}

void logUtils::logMsg(std::vector<logUtils::Logfile_t>& filelist, const std::string& type, std::vector<uint8_t>& msg, size_t size, uint32_t  msOfDay)
{ /// add message received msOfDay before message sent msOfDay
	std::vector<uint8_t> timestampe(4);
	for (int i = 0; i < 4; i++)
		timestampe[i] = (uint8_t)((msOfDay >> (3-i)*8) & 0xFF);
	msg.insert(msg.begin() + 3, timestampe.begin(), timestampe.end());
	logUtils::logMsg(filelist, type, msg, size + 4, true);
}

void logUtils::logMsgHex(std::ofstream& OS, const uint8_t* buf, size_t size)
{
	OS << std::hex;
	for (size_t i = 0; i < size; i++)
		OS << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<unsigned int>(buf[i]);
	OS << std::dec << std::endl;
}

