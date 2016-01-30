#ifndef _MRPTCICONFIG_H
#define _MRPTCICONFIG_H

#include <string>
#include <cstring>
#include <stdint.h>   // c++11 <cstdint>

// forward declaration
namespace socketUtils
{
  struct socketAddress_t;
};

namespace TCIconfig
{   
  struct SocketConfig
  {
    std::string s_spatPort;
    std::string s_spat2Port;    
    std::string s_timecardPath;
    std::string s_logPath;
    int         i_logInterval;              // interval of log file, in minutes
    std::string s_mgrSendSocket;            // to dataMgr
    std::string s_mgrListenSocket;          // from dataMgr
  };
      
  struct socketHandleEnum_t
  {
    enum socketHandle{MGRSEND,MGRLISTEN};
  };

  class configTCI
  {
    private:      
      bool configSucceed;
      TCIconfig::SocketConfig       mSocketConfig;
      socketUtils::socketAddress_t* mpMGRsendAddr;      
      socketUtils::socketAddress_t* mpMGRlistenAddr;
      
      bool readSocketConfigFile(const char* ptr);
      bool setupSocketAddress(void);
      
    public:
      configTCI(const char* ptr);
      ~configTCI(void);
            
      bool isConfigSucceed(void) const;
      std::string getSpatPort(void) const;
      std::string getSpat2Port(void) const;
      std::string getTimecardPath(void) const;
      std::string getLogPath(void) const;
      int getLogInterval(void) const;     
      socketUtils::socketAddress_t getSocketAddr(const TCIconfig::socketHandleEnum_t::socketHandle type) const;
  };
};

#endif

