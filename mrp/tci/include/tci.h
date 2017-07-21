//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************
#ifndef _MRPTCI_H
#define _MRPTCI_H

#include <bitset>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "ab3418msgs.h"
#include "msgDefs.h"
#include "msgEnum.h"
#include "timeCard.h"
#include "timeUtils.h"

/// trace controller state
struct predicted_bound_t
{
  uint16_t bound_L;   /// in deciseconds
  uint16_t bound_U;   /// in deciseconds
};

struct phase_status_t
{
	MsgEnum::phaseState  state;
	MsgEnum::phaseState  pedstate;
  MsgEnum::phaseCallType   call_status;
  MsgEnum::phaseRecallType recall_status;
  unsigned long long state_start_time;     /// milliseconds since the UNIX epoch
  unsigned long long pedstate_start_time;  /// milliseconds since the UNIX epoch
  predicted_bound_t time2next;
  predicted_bound_t pedtime2next;
};

struct controller_status_t
{
	uint8_t controller_addr;
	/// timestamps
  unsigned long long msec;         // milliseconds since the UNIX epoch
  bool isPlantimingReady;          // need to switch current plan if received signal_status indicating a plan transition
  /// data received from the controller
  AB3418MSG::signal_status_mess_t  signal_status;  // received signal_status_mess_t
  std::bitset<8> status;           // from received status8e_mess_t
  uint8_t active_force_off[8];     // from received signal_status_mess_t
  /// current control mode
  MsgEnum::controlMode mode;
  std::bitset<8> permitted_phases;
  std::bitset<8> permitted_ped_phases;
  uint8_t curbarrier;
  unsigned long long curbarrier_start_time; // in milliseconds, onset of barrier change
  unsigned long long timer_time[2];         // in milliseconds, onset of active phase/active interval/interval_timer change
  /// parameters related with coordination
  ssize_t coordplan_index;                  // index in Card::timingcard::coordplans
	std::bitset<8> coordinated_phases;
	uint8_t  synch_phase;
	uint16_t cycle_length;                    // in deciseconds
	uint16_t cur_local_cycle_clock;           // in deciseconds
  unsigned long long cycle_start_time;      // in milliseconds, onset of cycle change, cycle starts at the yield point
  unsigned long long cycle_clock_time;      // in milliseconds, onset of local cycle clock change
  /// vehicular and pedestrian phase status
  phase_status_t phase_status[8];
};

/// trace soft-call state
struct softcall_state_t
{
  unsigned long long msec;  /// timestamps that the last soft-call command is sent to the controller
  std::bitset<8> ped_call;  /// only need send to the controller once
  std::bitset<8> veh_call;  /// continue send to the controller when phases are not in green
  std::bitset<8> veh_ext;   /// continue send to the controller when phases are in green
  std::bitset<8> prio_call; /// continue send to the controller when phases are not in green
  std::bitset<8> prio_ext;  /// continue send to the controller when phases are in green
  void reset(void)
  {
    msec = 0;
    ped_call.reset();
    veh_call.reset();
    veh_ext.reset();
    prio_call.reset();
    prio_ext.reset();
  };
	bool any(void)
	{
		return(ped_call.any() || veh_call.any() || veh_ext.any() || prio_call.any() || prio_ext.any());
	};
};

int  open_port(const std::string& port_name, bool isReadOnly);
void close_port(int fd, bool isReadOnly);
void processRecvBuff(std::vector<uint8_t>& buf, size_t& count, std::vector< std::pair<size_t, size_t> >& vec);
void updateActivePhaseTime2next(phase_status_t& phase_status, predicted_bound_t& time2start,
	const AB3418MSG::signal_status_mess_t& signalstatus, const Card::phasetiming_mess_t& phasetiming,
	const Card::phaseflags_mess_t& phaseflags, uint8_t ring, unsigned long long timer_time, unsigned long long msec);
void updateActivePhaseTime2next(phase_status_t& phase_status, predicted_bound_t& time2start, const Card::coordplan_mess_t& coordplan,
	const AB3418MSG::signal_status_mess_t& signalstatus, const Card::phasetiming_mess_t& phasetiming,
	const Card::phaseflags_mess_t& phaseflags, uint8_t ring, uint16_t local_cycle_clock, uint16_t cycle_length,
	Card::ConcurrentType concurrentType, unsigned long long timer_time, unsigned long long msec);
void getNextPhaseStartBound(predicted_bound_t& time2start, const Card::phasetiming_mess_t& phasetiming,
	const Card::phaseflags_mess_t& phaseflags, uint8_t phaseIdx, const phase_status_t& phase_status);
void getNextPhaseStartBound(predicted_bound_t& time2start, const Card::phasetiming_mess_t& phasetiming,
	const Card::phaseflags_mess_t& phaseflags, uint8_t phaseIdx, const phase_status_t& phase_status,
	const Card::coordplan_mess_t& coordplan, uint16_t local_cycle_clock, uint16_t cycle_length);
void barrierCrossAdjust(predicted_bound_t (&time2start)[2]);
uint16_t getPedIntervalLeft(uint8_t interval_timer, unsigned long long timer_time, unsigned long long msec);
uint8_t  getPhaseWalkInterval(const Card::phasetiming_mess_t& phasetiming, const Card::phaseflags_mess_t& phaseflags, uint8_t phaseIdx);
void updateSoftcallState(softcall_state_t& softcallState, const msgDefs::softcall_request_t& request);
void updateSoftcallState(softcall_state_t& softcallState, const phase_status_t (&status)[8]);
size_t packMsg(std::vector<uint8_t>& buf, const controller_status_t& cntrstatus, uint8_t msgid,
	const timeUtils::fullTimeStamp_t& fullTimeStamp);

#endif
