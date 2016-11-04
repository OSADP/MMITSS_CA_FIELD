//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <cstring>

#include "tciConf.h"
#include "socketUtils.h"

using namespace std;

TCIconfig::configTCI::configTCI(const char* ptr)
{
  // default value, can be changed when readSocketConfigFile
  configSucceed = false;
  mSocketConfig.i_logInterval = 15;     
  mpMGRsendAddr   = new socketUtils::socketAddress_t;
  mpMGRlistenAddr = new socketUtils::socketAddress_t;
  
  bool isSocketConfigSucceed = readSocketConfigFile(ptr);
  bool isSetupAddrSucceed = setupSocketAddress();
  if (!isSocketConfigSucceed || !isSetupAddrSucceed)
    configSucceed = false;
  else
    configSucceed = true;
}

TCIconfig::configTCI::~configTCI(void)
{
  delete mpMGRsendAddr;
  delete mpMGRlistenAddr;
}

bool TCIconfig::configTCI::readSocketConfigFile(const char* ptr)
{
  ifstream IS_F(ptr);
  if (!IS_F.is_open())
  {
    cerr << "TCIconfig::readSocketConfigFile failed open: " << ptr << endl;
    return false;
  }
  istringstream iss;
  string line,s;
  while (std::getline(IS_F,line))
  {
    if (!line.empty())
    {
      if (line.find("spatPort") == 0)
      {
        iss.str(line);
        iss >> std::skipws >> s >> s >> mSocketConfig.s_spatPort;
        iss.clear();
      }
      else if (line.find("spat2Port") == 0)
      {
        iss.str(line);
        iss >> std::skipws >> s >> s >> mSocketConfig.s_spat2Port;
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
  if (mSocketConfig.s_spatPort.empty()
    || mSocketConfig.s_spat2Port.empty()
    || mSocketConfig.s_timecardPath.empty() 
    || mSocketConfig.s_logPath.empty() 
    || mSocketConfig.s_mgrSendSocket.empty() 
    || mSocketConfig.s_mgrListenSocket.empty())
    return false;
  else
    return true;
}

bool TCIconfig::configTCI::setupSocketAddress(void)
{
  if(!socketUtils::setSocketAddress(mSocketConfig.s_mgrSendSocket.c_str(),(*mpMGRsendAddr))
    || !socketUtils::setSocketAddress(mSocketConfig.s_mgrListenSocket.c_str(),(*mpMGRlistenAddr)))
    return false;
  else
    return true;
}

bool TCIconfig::configTCI::isConfigSucceed(void) const
{
  return configSucceed;
}

string TCIconfig::configTCI::getSpatPort(void) const
{
  return mSocketConfig.s_spatPort;
}

string TCIconfig::configTCI::getSpat2Port(void) const
{
  return mSocketConfig.s_spat2Port;
}

string TCIconfig::configTCI::getTimecardPath(void) const
{
  return mSocketConfig.s_timecardPath;
}

string TCIconfig::configTCI::getLogPath(void) const
{
  return mSocketConfig.s_logPath;
}

int TCIconfig::configTCI::getLogInterval(void) const
{
  return mSocketConfig.i_logInterval;
}

socketUtils::socketAddress_t TCIconfig::configTCI::getSocketAddr(const TCIconfig::socketHandleEnum_t::socketHandle type) const
{
  socketUtils::socketAddress_t p;
  switch(type)
  {
  case TCIconfig::socketHandleEnum_t::MGRSEND:
    p = *mpMGRsendAddr;
    break;
  default:
    p = *mpMGRlistenAddr;
  }
  return (p);
}
