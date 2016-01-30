#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <cstring>

#include "awrConf.h"
#include "socketUtils.h"

using namespace std;

AWRconfig::configAwr::configAwr(const char* ptr)
{
  // default value, can be changed when readSocketConfigFile
  configSucceed = false;
  mSocketConfig.i_logInterval = 15;     
  mSocketConfig.i_dsrcTimeoutSec = 2;   
  mSocketConfig.i_maxTime2goPhaseCall = 20;
  mSocketConfig.i_maxTime2changePhaseExt = 2;
  mSocketConfig.i_maxTime2phaseExt = 5;
  mSocketConfig.i_maxGreenExtenstion = 10;
  mpMGRlistenAddr   = new socketUtils::socketAddress_t;
  mpMGRsendAddr     = new socketUtils::socketAddress_t;
  
  bool isSocketConfigSucceed = readSocketConfigFile(ptr);
  bool isSetupAddrSucceed = setupSocketAddress();
  if (!isSocketConfigSucceed || !isSetupAddrSucceed)
    configSucceed = false;
  else
    configSucceed = true;
}

AWRconfig::configAwr::~configAwr(void)
{
  delete mpMGRlistenAddr;
  delete mpMGRsendAddr;
}

bool AWRconfig::configAwr::readSocketConfigFile(const char* ptr)
{
  ifstream IS_F(ptr);
  if (!IS_F.is_open())
  {
    cerr << "AWRconfig::readSocketConfigFile failed open: " << ptr << endl;
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
      else if (line.find("dsrcTimeout") == 0)
      {
        iss.str(line);
        iss >> std::skipws >> s >> s >> mSocketConfig.i_dsrcTimeoutSec;
        iss.clear();
      }
      else if (line.find("maxTime2goPhaseCall") == 0)
      {
        iss.str(line);
        iss >> std::skipws >> s >> s >> mSocketConfig.i_maxTime2goPhaseCall;
        iss.clear();
      }
      else if (line.find("maxTime2change4Ext") == 0)
      {
        iss.str(line);
        iss >> std::skipws >> s >> s >> mSocketConfig.i_maxTime2changePhaseExt;
        iss.clear();
      }
      else if (line.find("maxTime4phaseExt") == 0)
      {
        iss.str(line);
        iss >> std::skipws >> s >> s >> mSocketConfig.i_maxTime2phaseExt;
        iss.clear();
      }
      else if (line.find("maxGreenExtenstion") == 0)
      {
        iss.str(line);
        iss >> std::skipws >> s >> s >> mSocketConfig.i_maxGreenExtenstion;
        iss.clear();
      }
      else if (line.find("mgrSendSocket") == 0)
      {
        iss.str(line);
        iss >> std::skipws >> s >> s >> mSocketConfig.s_mgrSendSocket;
        iss.clear();
      }
      else if (line.find("mgrListenSocket") == 0)
      {
        iss.str(line);
        iss >> std::skipws >> s >> s >> mSocketConfig.s_mgrListenSocket;
        iss.clear();
      }
    }
  }
  IS_F.close();
  if (mSocketConfig.s_nmapFile.empty() 
    || mSocketConfig.s_timecardPath.empty()
    || mSocketConfig.s_logPath.empty() 
    || mSocketConfig.s_mgrListenSocket.empty() 
    || mSocketConfig.s_mgrSendSocket.empty())
    return false;
  else
    return true;
}

bool AWRconfig::configAwr::setupSocketAddress(void)
{
  if(!socketUtils::setSocketAddress(mSocketConfig.s_mgrListenSocket.c_str(),(*mpMGRlistenAddr))
    || !socketUtils::setSocketAddress(mSocketConfig.s_mgrSendSocket.c_str(),(*mpMGRsendAddr)))
    return false;
  else
    return true;
}

bool AWRconfig::configAwr::isConfigSucceed(void) const
{
  return configSucceed;
}

string AWRconfig::configAwr::getNmapFileName(void) const
{
  return mSocketConfig.s_nmapFile;
}

string AWRconfig::configAwr::getTimecardPath(void) const
{
  return mSocketConfig.s_timecardPath;
}

string AWRconfig::configAwr::getLogPath(void) const
{
  return mSocketConfig.s_logPath;
}

int AWRconfig::configAwr::getLogInterval(void) const
{
  return mSocketConfig.i_logInterval;
}

int AWRconfig::configAwr::getDsrcTimeOut(void) const
{
  return mSocketConfig.i_dsrcTimeoutSec;
}

int AWRconfig::configAwr::getMaxTime2goPhaseCall(void) const
{
  return mSocketConfig.i_maxTime2goPhaseCall;
}

int AWRconfig::configAwr::getMaxTime2changePhaseExt(void) const
{
  return mSocketConfig.i_maxTime2changePhaseExt;
}

int AWRconfig::configAwr::getMaxTime2phaseExt(void) const
{
  return mSocketConfig.i_maxTime2phaseExt;
}

int AWRconfig::configAwr::getMaxGreenExtenstion(void) const
{
  return mSocketConfig.i_maxGreenExtenstion;
}

socketUtils::socketAddress_t AWRconfig::configAwr::getSocketAddr(const AWRconfig::socketHandleEnum_t::socketHandle type) const
{
  socketUtils::socketAddress_t p;
  switch(type)
  {
  case AWRconfig::socketHandleEnum_t::MGRLISTEN:
    p = *mpMGRlistenAddr;
    break;
  default:
    p = *mpMGRsendAddr;
  }
  return (p);
}
