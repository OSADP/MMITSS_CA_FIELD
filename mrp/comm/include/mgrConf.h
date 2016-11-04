//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************
#ifndef _MRPDATAMGRCONFIG_H
#define _MRPDATAMGRCONFIG_H

#include <string>
#include <cstring>
#include <stdint.h>   // c++11 <cstdint>

// forward declaration
namespace socketUtils
{
  struct socketAddress_t;
};

namespace MGRconfig
{   
  struct SocketConfig
  {
    std::string s_nmapFile;
    std::string s_timecardPath;    
    std::string s_logPath;
    int         i_logInterval;              // interval of log file, in minutes
    int         i_permInterval;             // interval of calculating performance measures
    std::string s_wmeListenSocket;          // from wmerx
    std::string s_wmeSendSocket;            // to wmetx
    std::string s_cloudListenSocket;        // from pedestrian cloud server
    std::string s_cloudSendSocket;          // to pedestrian cloud server
    std::string s_tciListenSocket;          // from traffic controller interface
    std::string s_tciSendSocket;            // to traffic controller interface
    std::string s_awareListenSocket;        // from mrp_aware
    std::string s_awareSendSocket;          // to mrp_aware
    std::string s_obsListenSocket;          // from mrp_perfObs
    std::string s_obsSendSocket;            // to mrp_perfObs
  };
      
  struct socketHandleEnum_t
  {
    enum socketHandle{WMELISTEN,WMESEND,CLOUDLISTEN,CLOUDSEND,TCILISTEN,TCISEND,AWARELISTEN,AWARESEND,OBSLISTEN,OBSSEND};
  };

  class configMgr
  {
    private:      
      bool configSucceed;
      MGRconfig::SocketConfig   mSocketConfig;
      socketUtils::socketAddress_t* mpWMElistenAddr;      
      socketUtils::socketAddress_t* mpWMEsendAddr;      
      socketUtils::socketAddress_t* mpCLOUDlistenAddr;
      socketUtils::socketAddress_t* mpCLOUDsendAddr;      
      socketUtils::socketAddress_t* mpTCIlistenAddr;
      socketUtils::socketAddress_t* mpTCIsendAddr;
      socketUtils::socketAddress_t* mpAWARElistenAddr;
      socketUtils::socketAddress_t* mpAWAREsendAddr;      
      socketUtils::socketAddress_t* mpOBSlistenAddr;
      socketUtils::socketAddress_t* mpOBSsendAddr;
      bool readSocketConfigFile(const char* ptr);
      bool setupSocketAddress(void);
      
    public:
      configMgr(const char* ptr);
      ~configMgr(void);
            
      bool isConfigSucceed(void) const;
      std::string getNmapFileName(void) const;
      std::string getTimecardPath(void) const;      
      std::string getLogPath(void) const;
      int getLogInterval(void) const;    
      int getPermInterval(void) const;
      socketUtils::socketAddress_t getSocketAddr(const MGRconfig::socketHandleEnum_t::socketHandle type) const;
  };
};

#endif

