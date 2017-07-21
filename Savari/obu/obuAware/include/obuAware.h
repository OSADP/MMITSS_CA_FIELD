//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************
#ifndef _OBUAWARE_H
#define _OBUAWARE_H

#include <iostream> 
#include <sstream>
#include <cstring>
#include <string>
#include <unistd.h>
#include <stdint.h>

#include "dsrcMsgs.h"
#include "timeUtils.h"
#include "geoUtils.h"
#include "obuConf.h"

template <class T>
std::string toString (const T& t)
{
	std::ostringstream ss;
	ss << t;
		
	return ss.str();
};

template<class T> 
T fromString(const std::string& s)
{
	std::istringstream stream (s);
	T t;
	stream >> t;
	return t;
};

struct priorityRequestAction_enum_t
{
	enum Action {NONE,INITIATE,KEEPGOING,CANCEL};
};

//structure to hold priority request info
struct PriorityRequest_t
{ /// request
	bool 			isPriorityRequested;
  bool      isPriorityCancelled;
	long long requestedTms;
	long long requestedEta;
	uint8_t		requestedmsgCnt;
	uint32_t	requestedVehicleId;
	uint32_t	requestedIntersectionId;
	uint8_t		requestedInlaneId;
	uint8_t 	requestedOutlaneId;
	uint8_t 	requestedControlPhase;
	uint8_t 	requestedIntersectionIndex;
	uint8_t 	requestedApproachedIndex;
	uint8_t 	requestedLaneIndex;
  /// acknowledgement
  bool      isReqAck;
  uint8_t   requestStatus;  // priorityrequest_enum_t::requeststatus  {NOTVALID,REJECTED,NOTNEEDED,QUEUED,ACTIVE,CANCELLED,COMPLETED}
  uint8_t   priorityStatus; // 0 - noPriority, 1 - earlyGreen, 2 - greenExtension
	void reset(void)
	{
		isPriorityRequested = false;
    isPriorityCancelled = false;
		requestedTms = 0;
		requestedmsgCnt = 0;
    isReqAck = false;
	};
};

void initialSRM(SRM_element_t& cvSRM,OBUconfig::VinConfig& vehvin);
uint8_t getOutlaneId(const std::vector<GeoUtils::connectTo_t>& aConnectTo);
void updatePriorityRequest(PriorityRequest_t& request,const GeoUtils::connectedVehicle_t& cv,const long long tms,const priorityRequestAction_enum_t::Action action);
void updateSRM(SRM_element_t& cvSRM,const PriorityRequest_t& request,const AsnJ2735Lib& lib,const BOLB1_element_t* ps_bsmblob1,const priorityRequestAction_enum_t::Action action);
void dts2DTime(DTime_element_t& dt,const timeUtils::dateTimeStamp_t& ts,const uint16_t millitm);
void logSSM(std::ofstream& OS,const SSM_element_t& ssm,const timeUtils::dateTimeStamp_t& ts);

#endif
