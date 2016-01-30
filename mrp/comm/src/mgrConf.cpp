#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <cstring>

#include "mgrConf.h"
#include "socketUtils.h"

using namespace std;

MGRconfig::configMgr::configMgr(const char* ptr)
{
  // default value, can be changed when readSocketConfigFile
  configSucceed = false;
  mSocketConfig.i_logInterval = 15;     
  mSocketConfig.i_permInterval = 5;
  mpWMElistenAddr   = new socketUtils::socketAddress_t;
  mpWMEsendAddr     = new socketUtils::socketAddress_t;
  mpCLOUDlistenAddr = new socketUtils::socketAddress_t;
  mpCLOUDsendAddr   = new socketUtils::socketAddress_t;
  mpTCIlistenAddr   = new socketUtils::socketAddress_t;
  mpTCIsendAddr     = new socketUtils::socketAddress_t;
  mpAWARElistenAddr = new socketUtils::socketAddress_t;
  mpAWAREsendAddr   = new socketUtils::socketAddress_t;
  mpOBSlistenAddr   = new socketUtils::socketAddress_t;
  mpOBSsendAddr     = new socketUtils::socketAddress_t;
  
  bool isSocketConfigSucceed = readSocketConfigFile(ptr);
  bool isSetupAddrSucceed = setupSocketAddress();
  if (!isSocketConfigSucceed || !isSetupAddrSucceed)
    configSucceed = false;
  else
    configSucceed = true;
}

MGRconfig::configMgr::~configMgr(void)
{
  delete mpWMElistenAddr;
  delete mpWMEsendAddr;
  delete mpCLOUDlistenAddr;
  delete mpCLOUDsendAddr;
  delete mpTCIlistenAddr;
  delete mpTCIsendAddr;
  delete mpAWARElistenAddr;
  delete mpAWAREsendAddr;
  delete mpOBSlistenAddr;
  delete mpOBSsendAddr;
}

bool MGRconfig::configMgr::readSocketConfigFile(const char* ptr)
{
  ifstream IS_F(ptr);
  if (!IS_F.is_open())
  {
    cerr << "MGRconfig::readSocketConfigFile failed open: " << ptr << endl;
    return false;
  }
  istringstream iss;
  string line,s;
  while (std::getline(IS_F,line))
  {
    if (!line.empty())
    {
      if (line.find("nmapFile") == 0)
      {
        iss.str(line);
        iss >> std::skipws >> s >> s >> mSocketConfig.s_nmapFile;
        iss.clear();
      }
      else if (line.find("timeCardPath") == 0)
      {
        iss.str(line);
        iss >> std::skipws >> s >> s >> mSocketConfig.s_timecardPath;
        iss.clear();
      }      
      else if (line.find("logPath") == 0)
      {
        iss.str(line);
        iss >> std::skipws >> s >> s >> mSocketConfig.s_logPath;
        iss.clear();
      }
      else if (line.find("logInterval") == 0)
      {
        iss.str(line);
        iss >> std::skipws >> s >> s >> mSocketConfig.i_logInterval;
        iss.clear();
      }
      else if (line.find("permInterval") == 0)
      {
        iss.str(line);
        iss >> std::skipws >> s >> s >> mSocketConfig.i_permInterval;
        iss.clear();
      }      
      else if (line.find("wmeListenSocket") == 0)
      {
        iss.str(line);
        iss >> std::skipws >> s >> s >> mSocketConfig.s_wmeListenSocket;
        iss.clear();
      }
      else if (line.find("wmeSendSocket") == 0)
      {
        iss.str(line);
        iss >> std::skipws >> s >> s >> mSocketConfig.s_wmeSendSocket;
        iss.clear();
      }
      else if (line.find("cloudListenSocket") == 0)
      {
        iss.str(line);
        iss >> std::skipws >> s >> s >> mSocketConfig.s_cloudListenSocket;
        iss.clear();
      }
      else if (line.find("cloudSendSocket") == 0)
      {
        iss.str(line);
        iss >> std::skipws >> s >> s >> mSocketConfig.s_cloudSendSocket;
        iss.clear();
      }
      else if (line.find("tciListenSocket") == 0)
      {
        iss.str(line);
        iss >> std::skipws >> s >> s >> mSocketConfig.s_tciListenSocket;
        iss.clear();
      }
      else if (line.find("tciSendSocket") == 0)
      {
        iss.str(line);
        iss >> std::skipws >> s >> s >> mSocketConfig.s_tciSendSocket;
        iss.clear();
      }
      else if (line.find("awrListenSocket") == 0)
      {
        iss.str(line);
        iss >> std::skipws >> s >> s >> mSocketConfig.s_awareListenSocket;
        iss.clear();
      }
      else if (line.find("awrSendSocket") == 0)
      {
        iss.str(line);
        iss >> std::skipws >> s >> s >> mSocketConfig.s_awareSendSocket;
        iss.clear();
      }
      else if (line.find("obsListenSocket") == 0)
      {
        iss.str(line);
        iss >> std::skipws >> s >> s >> mSocketConfig.s_obsListenSocket;
        iss.clear();
      }
      else if (line.find("obsSendSocket") == 0)
      {
        iss.str(line);
        iss >> std::skipws >> s >> s >> mSocketConfig.s_obsSendSocket;
        iss.clear();
      }
    }
  }
  IS_F.close();
  if (mSocketConfig.s_nmapFile.empty() 
    || mSocketConfig.s_timecardPath.empty()
    || mSocketConfig.s_logPath.empty() 
    || mSocketConfig.s_wmeListenSocket.empty() 
    || mSocketConfig.s_wmeSendSocket.empty() 
    || mSocketConfig.s_cloudListenSocket.empty()
    || mSocketConfig.s_cloudSendSocket.empty()
    || mSocketConfig.s_tciListenSocket.empty()
    || mSocketConfig.s_tciSendSocket.empty()
    || mSocketConfig.s_awareListenSocket.empty()
    || mSocketConfig.s_awareSendSocket.empty()
    || mSocketConfig.s_obsListenSocket.empty()
    || mSocketConfig.s_obsSendSocket.empty())
    return false;
  else
    return true;
}

bool MGRconfig::configMgr::setupSocketAddress(void)
{
  if(!socketUtils::setSocketAddress(mSocketConfig.s_wmeListenSocket.c_str(),(*mpWMElistenAddr))
    || !socketUtils::setSocketAddress(mSocketConfig.s_wmeSendSocket.c_str(),(*mpWMEsendAddr))
    || !socketUtils::setSocketAddress(mSocketConfig.s_cloudListenSocket.c_str(),(*mpCLOUDlistenAddr))
    || !socketUtils::setSocketAddress(mSocketConfig.s_cloudSendSocket.c_str(),(*mpCLOUDsendAddr))    
    || !socketUtils::setSocketAddress(mSocketConfig.s_tciListenSocket.c_str(),(*mpTCIlistenAddr))
    || !socketUtils::setSocketAddress(mSocketConfig.s_tciSendSocket.c_str(),(*mpTCIsendAddr))
    || !socketUtils::setSocketAddress(mSocketConfig.s_awareListenSocket.c_str(),(*mpAWARElistenAddr))
    || !socketUtils::setSocketAddress(mSocketConfig.s_awareSendSocket.c_str(),(*mpAWAREsendAddr))
    || !socketUtils::setSocketAddress(mSocketConfig.s_obsListenSocket.c_str(),(*mpOBSlistenAddr))
    || !socketUtils::setSocketAddress(mSocketConfig.s_obsSendSocket.c_str(),(*mpOBSsendAddr)))
    return false;
  else
    return true;
}

bool MGRconfig::configMgr::isConfigSucceed(void) const
{
  return configSucceed;
}

string MGRconfig::configMgr::getNmapFileName(void) const
{
  return mSocketConfig.s_nmapFile;
}

string MGRconfig::configMgr::getTimecardPath(void) const
{
  return mSocketConfig.s_timecardPath;
}

string MGRconfig::configMgr::getLogPath(void) const
{
  return mSocketConfig.s_logPath;
}

int MGRconfig::configMgr::getLogInterval(void) const
{
  return mSocketConfig.i_logInterval;
}

int MGRconfig::configMgr::getPermInterval(void) const
{
  return mSocketConfig.i_permInterval;
}

socketUtils::socketAddress_t MGRconfig::configMgr::getSocketAddr(const MGRconfig::socketHandleEnum_t::socketHandle type) const
{
  socketUtils::socketAddress_t p;
  switch(type)
  {
  case MGRconfig::socketHandleEnum_t::WMELISTEN:
    p = *mpWMElistenAddr;
    break;
  case MGRconfig::socketHandleEnum_t::WMESEND:
    p = *mpWMEsendAddr;
    break;
  case MGRconfig::socketHandleEnum_t::CLOUDLISTEN:
    p = *mpCLOUDlistenAddr;
    break;
  case MGRconfig::socketHandleEnum_t::CLOUDSEND:
    p = *mpCLOUDsendAddr;
    break;
  case MGRconfig::socketHandleEnum_t::TCILISTEN:
    p = *mpTCIlistenAddr;
    break;
  case MGRconfig::socketHandleEnum_t::TCISEND:
    p = *mpTCIsendAddr;
    break;
  case MGRconfig::socketHandleEnum_t::AWARELISTEN:
    p = *mpAWARElistenAddr;
    break;
  case MGRconfig::socketHandleEnum_t::AWARESEND:
    p = *mpAWAREsendAddr;
    break;
  case MGRconfig::socketHandleEnum_t::OBSLISTEN:
    p = *mpOBSlistenAddr;
    break;
  default:
    p = *mpOBSsendAddr;
  }
  return (p);
}
