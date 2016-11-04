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
#include <iomanip> 

#include "timeUtils.h"
#include "msgUtils.h"

using namespace std;

void logDetPresMsg(std::ofstream& OS,const pres_data_t* pdata,const uint32_t ms,const std::string& dateStr)
{
  OS << dateStr << ",";
  logDetPresMsg(OS,pdata,ms);
}

void logDetPresMsg(std::ofstream& OS,const pres_data_t* pdata,const uint32_t ms)
{
  OS << timeUtils::getTimestampStr(ms) << ",";
  OS << static_cast<int>(pdata->flag) << ",";
  OS << static_cast<int>(pdata->status) << ",";  
  OS << static_cast<int>(pdata->pattern_num) << ",";
  OS << static_cast<int>(pdata->local_cycle_clock) << ",";
  OS << static_cast<int>(pdata->master_cycle_clock) << ",";
  OS << static_cast<int>(pdata->prio_busId) << ",";
  OS << static_cast<int>(pdata->prio_busDirection) << ",";
  OS << static_cast<int>(pdata->prio_type) << ",";
  OS << pdata->presences << endl;
}

void logDetCntMsg(std::ofstream& OS,const count_data_t* pdata,const uint32_t ms,const std::string& dateStr)
{
  OS << dateStr << ",";
  logDetCntMsg(OS,pdata,ms);
}

void logDetCntMsg(std::ofstream& OS,const count_data_t* pdata,const uint32_t ms)
{
  OS << timeUtils::getTimestampStr(ms) << ",";
  OS << static_cast<int>(pdata->seq_num) << ",";
  OS << static_cast<int>(pdata->flag) << ",";
  OS << static_cast<int>(pdata->status) << ",";
  OS << static_cast<int>(pdata->pattern_num) << ",";
  OS << static_cast<int>(pdata->local_cycle_clock) << ",";
  OS << static_cast<int>(pdata->master_cycle_clock);
  for (int i = 0; i < 16; i++) {OS << "," << static_cast<int>(pdata->vol[i]);}
  for (int i = 0; i < 16; i++) {OS << "," << static_cast<int>(pdata->occ[i]);}
  OS << endl;
}

void logSigStateMsg(std::ofstream& OS,const controller_state_t* pdata,const uint32_t ms,const std::string& dateStr)
{
  OS << dateStr << ",";
  logSigStateMsg(OS,pdata,ms);  
}

void logSigStateMsg(std::ofstream& OS,const controller_state_t* pdata,const uint32_t ms)
{
  OS << timeUtils::getTimestampStr(ms) << ",";
  OS << static_cast<int>(pdata->sigState.cntlrMode) << ",";
  OS << static_cast<int>(pdata->sigState.pattern_num) << ",";
  OS << static_cast<int>(pdata->sigState.permitted_phases) << ",";
  OS << static_cast<int>(pdata->sigState.permitted_ped_phases) << ",";  
  OS << static_cast<int>(pdata->sigState.preempt) << ",";
  OS << static_cast<int>(pdata->sigState.ped_call) << ",";
  OS << static_cast<int>(pdata->sigState.veh_call) << ",";
  OS << static_cast<int>(pdata->status) << ",";
  OS << static_cast<int>(pdata->timeStamp);
  for (int i = 0; i < 8; i++)
  {
    OS << "," << static_cast<int>(pdata->phaseState[i].state);
    OS << "," << static_cast<int>(pdata->phaseState[i].pedstate);
    OS << "," << static_cast<int>(pdata->phaseState[i].call_status);
    OS << "," << static_cast<int>(pdata->phaseState[i].recall_status);
    OS << "," << static_cast<int>(pdata->phaseState[i].time2next.bound_L);
    OS << "," << static_cast<int>(pdata->phaseState[i].time2next.bound_U);
    OS << "," << static_cast<int>(pdata->phaseState[i].time2next.confidence);
    OS << "," << static_cast<int>(pdata->phaseState[i].pedtime2next.bound_L);
    OS << "," << static_cast<int>(pdata->phaseState[i].pedtime2next.bound_U);
    OS << "," << static_cast<int>(pdata->phaseState[i].pedtime2next.confidence);
  }
  OS << endl;
}

void logSoftReqMsg(std::ofstream& OS,const softcall_request_t* pdata,const uint32_t ms,const std::string& dateStr)
{
  OS << dateStr << ",";
  logSoftReqMsg(OS,pdata,ms);  
}

void logSoftReqMsg(std::ofstream& OS,const softcall_request_t* pdata,const uint32_t ms)
{
  OS << timeUtils::getTimestampStr(ms) << ",";
  OS << static_cast<int>(pdata->callphase) << ",";
  OS << static_cast<int>(pdata->callobj) << ",";
  OS << static_cast<int>(pdata->calltype) << endl;
}

void logVehTroj(std::ofstream& OS,const vehTroj_t* pdata,const uint32_t ms,const std::string& dateStr)
{
  OS << dateStr << ",";
  logVehTroj(OS,pdata,ms);    
}

void logVehTroj(std::ofstream& OS,const vehTroj_t* pdata,const uint32_t ms)
{
  OS << timeUtils::getTimestampStr(ms) << ",";
  OS << pdata->trojCnt << ",";
  OS << pdata->vehId << ",";
  OS << static_cast<int>(pdata->entryLaneId) << ",";
  OS << static_cast<int>(pdata->entryControlPhase) << ",";
  OS << static_cast<int>(pdata->leaveLaneId) << ",";
  OS << static_cast<int>(pdata->leaveControlPhase) << ",";
  OS << static_cast<int>(pdata->distTraveled) << ",";
  OS << static_cast<int>(pdata->timeTraveled) << ",";
  OS << static_cast<int>(pdata->stoppedTime) << ",";
  OS << static_cast<int>(pdata->ingressLen) << endl;
}

void logPermMsg(std::ofstream& OS,const intPerm_t* pdata, const uint32_t ms,const std::string& dateStr)
{
  OS << dateStr << ",";
  logPermMsg(OS,pdata,ms);    
}

void logPermMsg(std::ofstream& OS,const intPerm_t* pdata, const uint32_t ms)
{
  OS << timeUtils::getTimestampStr(ms) << ",";
  OS << pdata->intId << ",";
  OS << static_cast<int>(pdata->permitted_phases);
  for (int i = 0; i < 8; i++)
  {
    OS << "," << static_cast<int>(pdata->apchPerm[i].sampleNums);
    OS << "," << static_cast<int>(pdata->apchPerm[i].tt_avg);
    OS << "," << static_cast<int>(pdata->apchPerm[i].tt_std);
    OS << "," << static_cast<int>(pdata->apchPerm[i].delay_avg);
    OS << "," << static_cast<int>(pdata->apchPerm[i].delay_std);
    OS << "," << static_cast<int>(pdata->apchPerm[i].stoppedSamples);
  }
  OS << endl;
}

void logPayloadHex(std::ofstream& OS,const char* buf,const size_t size)
{
  OS << hex;
  for (size_t i=0; i<size;i++)
  {
    OS << uppercase << setw(2) << setfill('0') << static_cast<unsigned int>((unsigned char)buf[i]);
  }
  OS << dec << endl;  
}