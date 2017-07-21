//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************
#ifndef _MMITSSMSGDEFS_H
#define _MMITSSMSGDEFS_H

#include <bitset>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "dsrcSPAT.h"
#include "msgEnum.h"

namespace msgDefs
{ /// MMITSS UDP messages (between MMITSS components)
	/*--------------------------------------------------------------------------------------------
	soft-call data structure (msgid_softcall) (send from MRP_Aware via MRP_DataMgr to MRP_TCI)
	---------------------------------------------------------------------------------------------*/
	struct softcall_request_t
	{
		uint32_t ms_since_midnight;
		std::bitset<8> callphase;
		MsgEnum::softCallObj  callobj;
		MsgEnum::softCallType calltype;
	};

	/*--------------------------------------------------------------------------------------------
	detector count/occupancy data structure (msgid_detCnt) (send from MRP_TCI to MRP_DataMgr)
	---------------------------------------------------------------------------------------------*/
	struct count_data_t
	{
		uint32_t ms_since_midnight;
		uint8_t  seq_num;
		std::bitset<8> flag;
		std::bitset<8> status;
		uint8_t  pattern_num;
		uint8_t  master_cycle_clock;
		uint8_t  local_cycle_clock;
		uint8_t  vol[16];
		uint8_t  occ[16];
	};

	/*--------------------------------------------------------------------------------------------
	detector presence structure (msgid_detPres) (send from MRP_TCI to MRP_DataMgr)
	--------------------------------------------------------------------------------------------*/
	struct pres_data_t
	{
		uint32_t ms_since_midnight;
		std::bitset<8> flag;
		std::bitset<8> status;
		uint8_t pattern_num;
		uint8_t master_cycle_clock;
		uint8_t local_cycle_clock;
		uint16_t prio_busId;
		uint8_t prio_busDirection;
		uint8_t prio_type;
		std::bitset<40> presences;
	};

	/*--------------------------------------------------------------------------------------------
	controller status data structure (msgid_cntrlstatus) (send from MRP_TCI to MRP_DataMgr)
	--------------------------------------------------------------------------------------------*/
	struct signal_status_t
	{
		MsgEnum::controlMode mode;
		uint8_t patternNum;
		uint8_t synch_phase;
		uint16_t cycle_length;       // in tenths of a second
		uint16_t local_cycle_clock;  // in tenths of a second
		std::bitset<8> coordinated_phases;
		std::bitset<8> preempt;
		std::bitset<8> ped_call;
		std::bitset<8> veh_call;
		MsgEnum::phaseCallType   call_status[8];
		MsgEnum::phaseRecallType recall_status[8];
	};

	struct controller_state_t
	{
		uint32_t ms_since_midnight;
		SPAT_element_t spatRaw;
		msgDefs::signal_status_t signalStatus;
	};

	/*--------------------------------------------------------------------------------------------
	vehicle trajectory structure (msgid_traj) (send from MRP_Aware to MRP_DataMgr)
	--------------------------------------------------------------------------------------------*/
	struct vehTraj_t
	{
		uint32_t ms_since_midnight;
		uint32_t count;
		uint32_t vehId;
		uint8_t  entryLaneId;
		uint8_t  entryControlPhase;  // 1..8
		uint8_t  leaveLaneId;
		uint8_t  leaveControlPhase;
		uint16_t distTraveled;       // in decimeter
		uint16_t timeTraveled;       // in deciseconds
		uint16_t stoppedTime;        // in deciseconds
		uint16_t inboundLaneLen;     // in decimeter
	};

	/*--------------------------------------------------------------------------------------------
	performance measure structure (msgid_perm) (send from MRP_DataMgr to MRP server)
	--------------------------------------------------------------------------------------------*/
	struct apchPerm_t
	{
		uint16_t sampleNums;
		uint16_t travelSpeed_mean; // in mph
		uint16_t travelSpeed_std;
		uint16_t travelTime_mean;  // in deciseconds
		uint16_t travelTime_std;
		uint16_t delay_mean;       // in deciseconds
		uint16_t delay_std;       // in deciseconds
		uint16_t stoppedNums;
	};

	struct intPerm_t
	{
		uint32_t ms_since_midnight;
		MsgEnum::controlMode mode;
		uint8_t  patternNum;
		std::bitset<8> permittedPhases;
		std::bitset<8> observedPhases;
		msgDefs::apchPerm_t apchPerm[8];
	};

	size_t packMsg(std::vector<uint8_t>& buf, const msgDefs::softcall_request_t& data, uint8_t msgid);
	size_t packMsg(std::vector<uint8_t>& buf, const msgDefs::vehTraj_t& data, uint8_t msgid);
	size_t packMsg(std::vector<uint8_t>& buf, const msgDefs::intPerm_t& data, uint8_t msgid);

	void unpackMsg(const std::vector<uint8_t>& buf, size_t& offset, msgDefs::count_data_t& data);
	void unpackMsg(const std::vector<uint8_t>& buf, size_t& offset, msgDefs::pres_data_t& data);
	void unpackMsg(const std::vector<uint8_t>& buf, size_t& offset, msgDefs::controller_state_t& data);
	void unpackMsg(const std::vector<uint8_t>& buf, size_t& offset, msgDefs::softcall_request_t& data);
	void unpackMsg(const std::vector<uint8_t>& buf, size_t& offset, msgDefs::vehTraj_t& data);
	void unpackMsg(const std::vector<uint8_t>& buf, size_t& offset, msgDefs::intPerm_t& data);
}

#endif
