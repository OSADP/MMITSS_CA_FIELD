#ifndef _MMITSSMINICONFIG_H
#define _MMITSSMINICONFIG_H

#include <string>
#include <cstring>
#include <stdint.h>   // c++11 <cstdint>

// forward declaration
namespace socketUtils
{
  struct socketAddress_t;
};

namespace MINIconfig
{   
  struct SocketConfig
  {
    std::string s_nmapFile;
    std::string s_logPath;
    int         i_logInterval;
    int         i_dsrcTimeoutSec;   
    std::string s_wmeListenSocket;          // from wmefwd
    std::string s_bsmSendSocket;            // to wmefwd
    std::string s_srmSendSocket;            // to wmefwd
    std::string s_gpsSocket;
  };
  
  struct VinConfig
  {
    bool isPrioEligible;
    uint32_t    vehId;
    double      vehLen;
    double      vehWidth;
    std::string codeWord;   // (1..16)
    std::string name;       // (1..63)
    std::string vin;        // (1..17)
    std::string ownerCode;  // (1..32)
    uint8_t vehType;        // (4 bits)(0..15) 
    uint8_t prioLevel;      // (4 bits)(0..15)
  };
    
  struct socketHandleEnum_t
  {
    enum socketHandle{WMELISTEN,BSMSEND,SRMSEND,GPSDEV};
  };

  class configObu
  {
    private:      
      bool configSucceed;
      MINIconfig::SocketConfig       mSocketConfig;
      MINIconfig::VinConfig          mVinConfig;
      socketUtils::socketAddress_t* mpWMElistenAddr;
      socketUtils::socketAddress_t* mpBSMsendAddr;
      socketUtils::socketAddress_t* mpSRMsendAddr;
      socketUtils::socketAddress_t* mpGpsAddr;

      bool readSocketConfigFile(const char* ptr1);
      bool readVinConfigFile(const char* ptr2);
      bool setupSocketAddress(void);
      
    public:
      configObu(const char* ptr1,const char* ptr2);
      ~configObu(void);
            
      bool isConfigSucceed(void) const;
      std::string getNmapFileName(void) const;
      std::string getLogPath(void) const;
      int getLogInterval(void) const;
      int getDsrcTimeOut(void) const;     
      struct MINIconfig::VinConfig getVinConfig(void) const;
      socketUtils::socketAddress_t getSocketAddr(const MINIconfig::socketHandleEnum_t::socketHandle type) const;
  };
};

#endif

