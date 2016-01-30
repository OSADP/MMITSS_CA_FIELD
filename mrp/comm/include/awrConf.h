#ifndef _MRPAWARECONFIG_H
#define _MRPAWARECONFIG_H

#include <string>
#include <cstring>
#include <stdint.h>   // c++11 <cstdint>

// forward declaration
namespace socketUtils
{
  struct socketAddress_t;
};

namespace AWRconfig
{   
  struct SocketConfig
  {
    std::string s_nmapFile;
    std::string s_timecardPath;    
    std::string s_logPath;
    int         i_logInterval;              // interval of log file, in minutes
    int         i_dsrcTimeoutSec;           // interval of timeout for dsrc messages, in seconds
    int         i_maxTime2goPhaseCall;      // maximum travel time to stop-bar for allowing vehicular phase call, in seconds
    int         i_maxTime2changePhaseExt;   // maximum green phase time2change for considering vehicular extension requests, in seconds
    int         i_maxTime2phaseExt;         // maximum time a green phase can be extended, in seconds
    int         i_maxGreenExtenstion;       // maximum time a priority phase can be extended, in seconds
    std::string s_mgrListenSocket;          // from mrp_dataMgr
    std::string s_mgrSendSocket;            // to mrp_dataMgr
  };
      
  struct socketHandleEnum_t
  {
    enum socketHandle{MGRLISTEN,MGRSEND};
  };

  class configAwr
  {
    private:      
      bool configSucceed;
      AWRconfig::SocketConfig   mSocketConfig;
      socketUtils::socketAddress_t* mpMGRlistenAddr;      
      socketUtils::socketAddress_t* mpMGRsendAddr;      
      
      bool readSocketConfigFile(const char* ptr);
      bool setupSocketAddress(void);
      
    public:
      configAwr(const char* ptr);
      ~configAwr(void);
            
      bool isConfigSucceed(void) const;
      std::string getNmapFileName(void) const;
      std::string getTimecardPath(void) const;            
      std::string getLogPath(void) const;
      int getLogInterval(void) const;     
      int getDsrcTimeOut(void) const;
      int getMaxTime2goPhaseCall(void) const;
      int getMaxTime2changePhaseExt(void) const;
      int getMaxTime2phaseExt(void) const;
      int getMaxGreenExtenstion(void) const;
      socketUtils::socketAddress_t getSocketAddr(const AWRconfig::socketHandleEnum_t::socketHandle type) const;
  };
};

#endif

