//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************
#ifndef _MRPAWARE_H
#define _MRPAWARE_H

#include <bitset>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "dsrcBSM.h"
#include "dsrcSRM.h"
#include "geoUtils.h"
#include "msgDefs.h"
#include "msgEnum.h"
#include "timeUtils.h"

enum class prioGrantType : uint8_t {none, earlyGreen, greenExtension};
enum class phaseExtType  : uint8_t {none, called, cancelled};

/// structure to hold connected vehicle status including geo-location, map matched location,
/// location/signal aware, and status of vehicular phase call and/or phase extension request
struct cvStatusAware_t
{
	bool isOnInbound;				 // on one of the inbound approaches of this intersection
	bool isPhaseCalled;      // call vehicle phase at most once for each vehicle when it's needed
	bool isExtensionCalled;  // extend vehicle phase green at most once for each vehicle when it's needed
	unsigned long long msec;
	BSM_element_t bsm;
	std::vector<GeoUtils::connectedVehicle_t> cvStatus;
	void reset(void)
	{
		isOnInbound = false;
		isPhaseCalled = false;
		isExtensionCalled = false;
		cvStatus.clear();
	};
};

/// structure to hold SRM and status of priority request
struct srmStatus_t
{
	unsigned long long msec;
	uint8_t requestedPhase;
	MsgEnum::requestStatus status;
	SRM_element_t srm;
};

struct prioRequestCadidate_t
{
	uint32_t vehId;
	uint8_t  requestedPhase;
	uint16_t duration;
};

struct prioServingStatus_t
{
	prioGrantType grantingType;
	unsigned long long granting_msec;
	uint8_t  grantingPhase;
	uint32_t grantingVehId;
	uint8_t  grantingCycleCnt;
};

/// structure to hold vehicular phase extension call status
struct signalPhaseExtension_t
{
	phaseExtType extType;
	unsigned long long extCall_msec;
	std::vector<uint32_t> servingVehIds;
	void reset(void)
	{
		extType = phaseExtType::none;
    servingVehIds.clear();
	};
};

/// structure to hold status of MRP_Aware
struct aware_status_t
{ /// controller and signal status
	msgDefs::controller_state_t cntrlState;
	MsgEnum::phaseState syncPhaseState;
  uint8_t cycleCtn;
	/// performance measures
	msgDefs::intPerm_t intPerm;
  /// priority request server
	bool requestStatusUpdated;
	prioServingStatus_t prioServingStatus;
  /// vehicular phase call
	unsigned long long phaseCall_mec[8];
  /// vehicular phase extension (non-TSP)
	signalPhaseExtension_t phaseExtStatus[8];
	void reset(void)
	{
		cntrlState.signalStatus.mode = MsgEnum::controlMode::unavailable;
		syncPhaseState = MsgEnum::phaseState::unavailable;
		cycleCtn = 0;
		intPerm.mode = MsgEnum::controlMode::unavailable;
		requestStatusUpdated = false;
		prioServingStatus.grantingType = prioGrantType::none;
		for (int i = 0; i < 8; i++)
			phaseExtStatus[i].reset();
	};
};

size_t packMsg(std::vector<uint8_t>& buf, uint8_t callPhase, MsgEnum::softCallObj callObj,
	MsgEnum::softCallType callType, uint32_t msOfDay, uint8_t msgid);
size_t packMsg(std::vector<uint8_t>& buf, std::bitset<8> callPhases, MsgEnum::softCallObj callObj,
	MsgEnum::softCallType callType, uint32_t msOfDay, uint8_t msgid);
size_t packMsg(std::vector<uint8_t>& buf, const cvStatusAware_t& cvStatusAware, const timeUtils::dateStamp_t& curDateStamp,
	uint32_t msOfDay, uint32_t laneLen, double stopSpeed, uint8_t msgid);
void packMsg(SSM_element_t& ssm, const std::vector<srmStatus_t> list, const timeUtils::dateTimeStamp_t utcDateTimeStamp);
uint16_t getTime2Go(double dist2go, double speed, double stopSpeed);
bool withinTimeWindow(uint16_t windowStartTime, uint16_t windowLength, uint16_t arrivalTime);
bool isTimeBefore(uint16_t timePoint_1, uint16_t timePoint_2);
uint16_t getDuration(uint16_t fromTimePoint, uint16_t toTimePoint);
size_t getCandidateIndex(const std::vector<prioRequestCadidate_t>& candidates);
void setGrantStatus(std::vector<srmStatus_t>& list, const std::vector<prioRequestCadidate_t>& candidates);

#endif
