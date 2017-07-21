//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************
#ifndef _CONFIGURATION_UTILS_H
#define _CONFIGURATION_UTILS_H

#include <map>
#include <string>
#include <vector>

#include "socketUtils.h"

class ComponentCnf
{
	private:
		bool success;
		std::map<std::string, std::string> stringParas;
		std::map<std::string, int> integerParas;
		std::vector<socketUtils::Address_t> sAddr;

		bool readConf(const std::string& fname);

	public:
		ComponentCnf(const std::string& fname);
		~ComponentCnf(void);

		bool isInitiated(void) const;
		bool connectAll(void);
		void disconnectAll(void);
		std::string getStringParaValue(const std::string& variableName) const;
		int getIntegerParaValue(const std::string& variableName) const;
		socketUtils::Conn_t getSocketConn(const std::string& id) const;
		int getSocketDescriptor(const std::string& id) const;
};

#endif

