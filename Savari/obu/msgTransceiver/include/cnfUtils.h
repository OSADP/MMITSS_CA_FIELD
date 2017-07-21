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

class ComponentCnf
{
	public:
		struct Address_t
		{
			std::string comp;     /// component to communicate with
			std::string listenIP;
			std::string listenPort;
			std::string sendIP;
			std::string sendPort;
			bool valid(void)
			{
				return(!comp.empty() && !listenIP.empty() && !listenPort.empty() && !sendIP.empty() && !sendPort.empty());
			};
		};

	private:
		bool success;
		std::map<std::string, std::string> stringParas;
		std::map<std::string, int> integerParas;
		std::vector<ComponentCnf::Address_t> sAddr;

		bool readConf(const std::string& fname);

	public:
		ComponentCnf(const std::string& fname);
		~ComponentCnf(void){};

		bool isInitiated(void) const;
		std::string getStringParaValue(const std::string& variableName) const;
		int getIntegerParaValue(const std::string& variableName) const;
		ComponentCnf::Address_t getAddr(const std::string& comp) const;
};

#endif

