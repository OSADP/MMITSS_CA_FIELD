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

#include "cnfUtils.h"

ComponentCnf::ComponentCnf(const std::string& fname)
	{success = readConf(fname);}

ComponentCnf::~ComponentCnf(void)
	{ComponentCnf::disconnectAll();}

bool ComponentCnf::readConf(const std::string& fname)
{
	std::ifstream IS_F(fname);
	if (!IS_F.is_open())
	{
		std::cerr << "ComponentCnf: could not open " << fname << std::endl;
		return(false);
	}
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
				if (cnt > 20)
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
				auto it = stringParas.find(variableName);
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
				if (cnt > 20)
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
				auto it = integerParas.find(variableName);
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
				if (cnt > 20)
				{
					has_error = true;
					break;
				}
				if (line.empty())
					continue;
				if (line.find("END_SOCKETS") == 0)
					break;
				socketUtils::Address_t addr;
				if (socketUtils::setAddress(line, addr))
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

bool ComponentCnf::connectAll(void)
{
	bool has_error = false;
	for (auto& item : sAddr)
	{
		if(!socketUtils::create(item))
		{
			has_error = true;
			break;
		}
	}
	if (has_error)
		ComponentCnf::disconnectAll();
	return(!has_error);
}

void ComponentCnf::disconnectAll(void)
{
	for (auto& item : sAddr)
	{
		if (item.conn.fd != -1)
		{
			socketUtils::destroy(item.conn.fd);
			item.conn.fd = -1;
		}
	}
}

std::string ComponentCnf::getStringParaValue(const std::string& variableName) const
{
	auto it = stringParas.find(variableName);
	return((it != stringParas.end()) ? it->second : std::string());
}

int ComponentCnf::getIntegerParaValue(const std::string& variableName) const
{
	auto it = integerParas.find(variableName);
	return((it != integerParas.end()) ? it->second : -1);
}

socketUtils::Conn_t ComponentCnf::getSocketConn(const std::string& id) const
{
	socketUtils::Conn_t retn;
	retn.fd = -1;
	auto it = std::find_if(sAddr.begin(), sAddr.end(),
		[&id](const socketUtils::Address_t& item){return(item.id == id);});
	return((it != sAddr.end()) ? (it->conn) : retn);
}

int ComponentCnf::getSocketDescriptor(const std::string& id) const
{
	socketUtils::Conn_t retn = ComponentCnf::getSocketConn(id);
	return(retn.fd);
}
