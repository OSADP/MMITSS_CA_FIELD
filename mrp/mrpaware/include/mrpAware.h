#ifndef _MRPAWARE_H
#define _MRPAWARE_H

#include <fstream>
#include <stdint.h>
#include <map>
#include <vector>
#include <bitset>

#include "msgenum.h"
#include "dsrcMsgs.h"
#include "geoUtils.h"
#include "timeUtils.h"
#include "timeCard.h"
#include "msgUtils.h"

struct priorityRequestServer_enum_t
{
	enum PRGupdate {INITIATE=0,MAINTAIN=1,UPDATE=2,CANCEL=3};
	enum PRSstate {NONE=0,EARLYGREEN=1,GREENEXTENSION=2};
};

/// structure to hold connected vehicle status including geo-location, map matched location, location/signal aware,
///		and priority request and soft-call status
struct cvStatusAware_t
{
	bool isOnApproach;				// in an approach of this intersection (given by intersection name)
	bool isPhaseCalled;				// call vehicle phase at most once for each vehicle if it's needed
	bool isExtensionCalled;		// extend vehicle phase green at most once for each vehicle if it's needed
	std::vector<GeoUtils::connectedVehicle_t> cvStatus;                                                       
	void reset(void)
	{
		isOnApproach = false;
		isPhaseCalled = false;
		isExtensionCalled = false;
		cvStatus.clear();
	};
};

/// structure to hold vehicular phase extension call status at this intersection
struct signalPhaseExtension_t
{
	softcall_enum_t::action callAction;
	long long callTms;
  uint8_t callduration; // in deciseconds
	std::map<uint32_t,uint8_t> callVehicles; // key vehId, value green extension time
	void reset(void)
	{
		callAction = softcall_enum_t::NOACTION;
		callTms = 0;
    callVehicles.clear();
	};
};

/// structure to hold PRS' request table entry
struct priorityRequestTableEntry_t
{
	uint32_t 	requestVehId;
	uint8_t		msgCnt;
	priorityRequestServer_enum_t::PRGupdate requestUpdateStatus;
	long long initiatedTms;               // utc time
  long long receivedTms;                // utc time
	long long requestedServiceStartTms;   // utc time
	long long requestedServiceEndTms;     // utc time
	uint8_t		requestedAction;
	uint8_t		requestInLaneId;
	uint8_t		requestOutLaneId;
	uint8_t		requestPhase;
	uint8_t		requestVehicleClassType;
	uint8_t		requestVehicleClassLevel;
	uint8_t 	requestVehicleTransitStatus;	
	DTime_element_t requestTimeOfService;									// optional in SRM_element_t
	DTime_element_t requestEndOfService;									// optional in SRM_element_t
	MsgEnum::priorityrequest_enum_t::requeststatus	requestStatus;	// determined here
};

/// structure to hold MMITSS signal control status
struct mmitssSignalControlStatus_t
{
  /// signal status 
  bool isPlantimingReady; 
  operation_mode_enum_t::controlmode cntlrMode;
  uint8_t pattern_num; 
  std::bitset<8> permitted_phases;
  std::bitset<8> permitted_ped_phases;
  std::bitset<8> preempt;
  std::bitset<8> ped_call;
  std::bitset<8> veh_call;
  /// generate from received signal status msg combining with timecard
  ssize_t coordplan_index;  // index in timing_card_t::coordplans  
  std::bitset<8> coordinatedPhases;
  uint8_t synch_phase;   
  uint8_t cycleCtn;
  /// ssm
	uint8_t ssmMsgCnt;
	long long ssmTms;
  /// priority request server
	priorityRequestServer_enum_t::PRSstate prsAction;
	uint8_t priorityPhaseId;
	uint32_t priorityVehicleId; 
	uint8_t priorityCycleCnt;	 		
	long long priorityTms;
	std::map<uint32_t,priorityRequestTableEntry_t> prsList; // key vehId
  /// vehicular phase call
	long long phaseCallTms[NEMAPHASES];
  /// vehicular phase extension
	signalPhaseExtension_t phaseExtStatus[NEMAPHASES];
	void reset(void)
	{
    isPlantimingReady = false;
    cntlrMode = operation_mode_enum_t::UNKNOWN;
    pattern_num = 0;
    coordplan_index = -1;
    permitted_phases.reset();
    permitted_ped_phases.reset();
    preempt.reset();
    ped_call.reset();
    veh_call.reset();
    coordinatedPhases.reset();
    synch_phase = 2;
    cycleCtn = 0;
		prsAction = priorityRequestServer_enum_t::NONE;
		ssmMsgCnt = 0;
		priorityPhaseId = 0;
		priorityVehicleId = 0;
		priorityCycleCnt = 0;
		ssmTms = 0;
		priorityTms = 0;
		prsList.clear();
		for (int i=0; i<NEMAPHASES; i++)
		{
			phaseCallTms[i] = 0;
			phaseExtStatus[i].reset();
		};
	};
};

bool updateSignalState(mmitssSignalControlStatus_t& cntrStatus,const signal_state_t* psigState,const timing_card_t& timingcard);
bool sendSoftReq(int fd,char* buf,const softcall_request_t& softReq,const uint32_t ms,const uint8_t msgid);
bool sendVehTroj(int fd,char* buf,const vehTroj_t& vehTroj,const uint32_t ms,const uint8_t msgid);
bool sendSSM(int fd,char* buf,const char* msg,const size_t msglen,const uint32_t ms,const uint8_t msgid);
void cleanVehList(std::map<uint32_t,cvStatusAware_t>& list,const long long tms,const long long timeout);
void cleanPrsList(std::map<uint32_t,priorityRequestTableEntry_t>& list,const long long tms,const long long timeout);
void logSSM(std::ofstream& OS,const SSM_element_t& ssm,const timeUtils::dateTimeStamp_t& ts);
void logVehTrack(std::ofstream& OS,const cvStatusAware_t& cvStatusAware,const uint32_t trojCnt);
void formTroj(vehTroj_t& vehTroj,const cvStatusAware_t& cvStatusAware,const uint32_t laneLen,const uint32_t trojCnt,const double stopSpeed);

#endif
