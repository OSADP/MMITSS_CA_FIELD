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

#include "obsConf.h"
#include "socketUtils.h"

using namespace std;

OBSconfig::configObs::configObs(const char* ptr)
{
  // default value, can be changed when readSocketConfigFile
  configSucceed = false;
  mSocketConfig.i_logInterval = 15;     
  mpMGRlistenAddr = new socketUtils::socketAddress_t;
  mpMGRsendAddr   = new socketUtils::socketAddress_t;
  
  bool isSocketConfigSucceed = readSocketConfigFile(ptr);
  bool isSetupAddrSucceed = setupSocketAddress();
  if (!isSocketConfigSucceed || !isSetupAddrSucceed)
    configSucceed = false;
  else
    configSucceed = true;
}

OBSconfig::configObs::~configObs(void)
{
  delete mpMGRlistenAddr;
  delete mpMGRsendAddr;
}

bool OBSconfig::configObs::readSocketConfigFile(const char* ptr)
{
  ifstream IS_F(ptr);
  if (!IS_F.is_open())
  {
    cerr << "OBSconfig::readSocketConfigFile failed open: " << ptr << endl;
    return false;
  }
  istringstream iss;
  string line,s;
  while (std::getline(IS_F,line))
  {
    if (!line.empty())
    {
      if (line.find("logPath") == 0)
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
  if (mSocketConfig.s_logPath.empty() 
    || mSocketConfig.s_mgrListenSocket.empty() 
    || mSocketConfig.s_mgrSendSocket.empty())
    return false;
  else
    return true;
}

bool OBSconfig::configObs::setupSocketAddress(void)
{
  if(!socketUtils::setSocketAddress(mSocketConfig.s_mgrListenSocket.c_str(),(*mpMGRlistenAddr))
    || !socketUtils::setSocketAddress(mSocketConfig.s_mgrSendSocket.c_str(),(*mpMGRsendAddr)))
    return false;
  else
    return true;
}

bool OBSconfig::configObs::isConfigSucceed(void) const
{
  return configSucceed;
}

string OBSconfig::configObs::getLogPath(void) const
{
  return mSocketConfig.s_logPath;
}

int OBSconfig::configObs::getLogInterval(void) const
{
  return mSocketConfig.i_logInterval;
}

socketUtils::socketAddress_t OBSconfig::configObs::getSocketAddr(const OBSconfig::socketHandleEnum_t::socketHandle type) const
{
  socketUtils::socketAddress_t p;
  switch(type)
  {
  case OBSconfig::socketHandleEnum_t::MGRLISTEN:
    p = *mpMGRlistenAddr;
    break;
  default:
    p = *mpMGRsendAddr;
  }
  return (p);
}
