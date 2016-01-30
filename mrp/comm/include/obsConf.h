#ifndef _MRPPERFOBSCONFIG_H
#define _MRPPERFOBSCONFIG_H

#include <string>
#include <cstring>
#include <stdint.h>   // c++11 <cstdint>

// forward declaration
namespace socketUtils
{
  struct socketAddress_t;
};

namespace OBSconfig
{   
  struct SocketConfig
  {
    std::string s_logPath;
    int         i_logInterval;              // interval of log file, in minutes
    std::string s_mgrListenSocket;          // from mrp_dataMgr
    std::string s_mgrSendSocket;            // to mrp_dataMgr
  };
      
  struct socketHandleEnum_t
  {
    enum socketHandle{MGRLISTEN,MGRSEND};
  };

  class configObs
  {
    private:      
      bool configSucceed;
      OBSconfig::SocketConfig   mSocketConfig;
      socketUtils::socketAddress_t* mpMGRlistenAddr;      
      socketUtils::socketAddress_t* mpMGRsendAddr;      
      
      bool readSocketConfigFile(const char* ptr);
      bool setupSocketAddress(void);
      
    public:
      configObs(const char* ptr);
      ~configObs(void);
            
      bool isConfigSucceed(void) const;
      std::string getLogPath(void) const;
      int getLogInterval(void) const;     
      socketUtils::socketAddress_t getSocketAddr(const OBSconfig::socketHandleEnum_t::socketHandle type) const;
  };
};

#endif

