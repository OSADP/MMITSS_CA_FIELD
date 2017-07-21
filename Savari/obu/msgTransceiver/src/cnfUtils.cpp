//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <cstring>

#include "cnfUtils.h"

ComponentCnf::ComponentCnf(const std::string& fname)
	{success = readConf(fname);}

bool ComponentCnf::readConf(const std::string& fname)
{
	std::ifstream IS_F(fname.c_str());
	if (!IS_F.is_open())
	{
		std::cerr << "ComponentCnf: could not open " << fname << std::endl;
		return(false);
	}
	unsigned int max_entries = 20;
	std::istringstream iss;
	std::string line;
	bool has_error = false;
	while (std::getline(IS_F,line))
	{
		if (line.empty())
			continue;
		if (line.find("STRING_PARAMETERS") == 0)
		{ // start of string parameters
			unsigned int cnt = 0;
			std::string variableName, variableValue;
			while(1)
			{
				std::getline(IS_F,line);
				cnt++;
				if (cnt > max_entries)
				{
					has_error = true;
					break;
				}
				if (line.empty())
					continue;
				if (line.find("END_STRING_PARAMETERS") == 0)
					break;
				iss.str(line);
				iss >> std::skipws >> variableName >> variableValue;
				iss.clear();
				std::map<std::string, std::string>::iterator it = stringParas.find(variableName);
				if (it != stringParas.end())
				{
					std::cerr << "ComponentCnf: " << fname << " has duplicated string variable " << variableName << std::endl;
					has_error = true;
					break;
				}
				stringParas[variableName] = variableValue;
			}
			if (has_error)
			{
				std::cerr << "ComponentCnf: " << fname << ", failed reading STRING_PARAMETERS" << std::endl;
				break;
			}
		}
		else if (line.find("INTEGER_PARAMETERS") == 0)
		{ // start of integer parameters
			unsigned int cnt = 0;
			std::string variableName;
			int variableValue;
			while(1)
			{
				std::getline(IS_F,line);
				cnt++;
				if (cnt > max_entries)
				{
					has_error = true;
					break;
				}
				if (line.empty())
					continue;
				if (line.find("END_INTEGER_PARAMETERS") == 0)
						break;
				iss.str(line);
				iss >> std::skipws >> variableName >> variableValue;
				iss.clear();
				std::map<std::string, int>::iterator it = integerParas.find(variableName);
				if (it != integerParas.end())
				{
					std::cerr << "ComponentCnf: " << fname << " has duplicated integer variable " << variableName << std::endl;
					has_error = true;
					break;
				}
				integerParas[variableName] = variableValue;
			}
			if (has_error)
			{
				std::cerr << "ComponentCnf: " << fname << ", failed reading INTEGER_PARAMETERS" << std::endl;
				break;
			}
		}
		else if (line.find("SOCKETS") == 0)
		{ // start of socket configurations
			unsigned int cnt = 0;
			while(1)
			{
				std::getline(IS_F,line);
				cnt++;
				if (cnt > max_entries)
				{
					has_error = true;
					break;
				}
				if (line.empty())
					continue;
				if (line.find("END_SOCKETS") == 0)
					break;
				ComponentCnf::Address_t addr;
				iss.str(line);
				iss >> std::skipws >> addr.comp >> addr.listenIP >> addr.listenPort >> addr.sendIP >> addr.sendPort;
				iss.clear();
				if (addr.valid())
					sAddr.push_back(addr);
				else
				{
					has_error = true;
					break;
				}
			}
			if (has_error)
			{
				std::cerr << "ComponentCnf: " << fname << ", failed reading SOCKETS" << std::endl;
				break;
			}
		}
	}
	IS_F.close();
	return(!has_error);
}

bool ComponentCnf::isInitiated(void) const
	{return(success);}

std::string ComponentCnf::getStringParaValue(const std::string& variableName) const
{
	std::map<std::string, std::string>::const_iterator it = stringParas.find(variableName);
	return((it != stringParas.end()) ? it->second : std::string());
}

int ComponentCnf::getIntegerParaValue(const std::string& variableName) const
{
	std::map<std::string, int>::const_iterator it = integerParas.find(variableName);
	return((it != integerParas.end()) ? it->second : -1);
}

ComponentCnf::Address_t ComponentCnf::getAddr(const std::string& comp) const
{
	ComponentCnf::Address_t addr;
	for(std::vector<ComponentCnf::Address_t>::const_iterator it = sAddr.begin(); it != sAddr.end(); ++it)
	{
		if (it->comp == comp)
		{
			addr = *it;
			break;
		}
	}
	return(addr);
}
