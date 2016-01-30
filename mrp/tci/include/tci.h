#ifndef _MRPTCI_H
#define _MRPTCI_H

#include <stdint.h>
#include <vector> 
#include <map> 
#include <utility> 
#include <bitset>  

#include "timeUtils.h"
#include "timeCard.h"

/// controller pushing out signal status message
struct signal_status_mess_t
{
  timeUtils::fullTimeStamp_t fullTimeStamp;
  uint8_t controller_addr; 
  std::bitset<8> active_phase;
  uint8_t active_phases[2];
  uint8_t active_interval[2]; 
  uint8_t interval_timer[2]; 
  std::bitset<8> next_phase;  // the next phase on ringA and ringB (0 means the controller does not know yet)
  uint8_t next_phases[2];     
  std::bitset<8> ped_call;
  std::bitset<8> veh_call;
  uint8_t pattern_num;
  uint8_t local_cycle_clock;  
  uint8_t master_cycle_clock; 
  std::bitset<8> preempt;   // bits 0-3 <==> EV A-D
                            // bits 4-5 <==> RR 1-2
                            // bit 6 - pattern transition
                            // bit 7 – Transit Vehicle Priority
  uint8_t permissive[8]; 
  uint8_t ped_permissive[8];
  uint8_t active_force_off[2];
  /// plan_num & offset_index are decoded from pattern_num
  uint8_t plan_num;
  uint8_t offset_index;
};

/// controller pushing out status8 message
struct status8e_mess_t
{
  timeUtils::fullTimeStamp_t fullTimeStamp;
  uint8_t controller_addr; 
  uint8_t hour;
  uint8_t minute;
  uint8_t sec;
  std::bitset<8> flag;    // bit 0 = focus mode
                          // bit 2 = advance input status
                          // bit 3 = spare 3 input status
                          // bit 4 = spare 2 input status
                          // bit 5 = spare 1 input status
                          // bit 7 = transit vehicle call
  std::bitset<8> status;  // bit 0 = in preemption
                          // bit 1 = cabinet flash
                          // bit 2 = passed local zero since last request
                          // bit 3 = in local override mode
                          // bit 4 = coordination alarm pending
                          // bit 5 = detector fault pending
                          // bit 6 = non-critical alarm pending
                          // bit 7 = critical alarm pending
  uint8_t pattern_num;                        
  std::bitset<40> detector_presences;
  uint8_t master_cycle_clock;
  uint8_t local_cycle_clock;  
  uint16_t prio_busId;
  uint8_t  prio_busDirection;       // 5 = Phase2 opticom ON
                                    // 13 = Phase2 opticom OFF
                                    // 21 = Phase6 opticom ON
                                    // 17 = Phase6 opticom OFF
  uint8_t  prio_type;               // 0 = no Priority
                                    // 1 = Early Green
                                    // 2 = Green Extension
};

/// controller pushing out long status8 message
struct longstatus8e_mess_t
{
  timeUtils::fullTimeStamp_t fullTimeStamp;
  uint8_t controller_addr; 
  uint8_t hour;
  uint8_t minute;
  uint8_t sec;
  std::bitset<8> flag;
  std::bitset<8> status;
  uint8_t pattern_num;
  uint8_t master_cycle_clock;
  uint8_t local_cycle_clock;  
  uint8_t seq_num;
  uint8_t volume[16];
  uint8_t occupancy[16];  // 0-200 = Detector occupancy in 0.5% increments
                          // 210 = Stuck ON fault
                          // 211 = Stuck OFF fault
                          // 212 = Open Loop fault
                          // 213 = Shorted Loop fault
                          // 214 = Excessive Inductance fault
                          // 215 = Over Count fault
};

/// polls to get controller configuration data
struct poll_conf_t
{
  uint8_t poll_type;      // 1/2 - getBlockMsg/getTimingData
  std::string poll_desc;
  uint8_t poll_controlByte;
  uint8_t poll_messType;
  uint8_t poll_data1;     // pageId for getBlockMsg, memory_msb for getTimingData
  uint8_t poll_data2;     // blockId for getBlockMsg, memory_lsb for getTimingData
  uint8_t poll_data3;     // not used for getBlockMsg, num_bytes for getTimingData
  uint8_t res_messType;
  uint8_t err_messType;
  bool    pollRequired;
  bool    pollReturned;
  poll_conf_t(uint8_t poll_type_,std::string poll_desc_,uint8_t poll_controlByte_,uint8_t poll_messType_,
    uint8_t poll_data1_,uint8_t poll_data2_,uint8_t poll_data3_,uint8_t res_messType_,uint8_t err_messType_,bool pollRequired_)
  {
    this->poll_type = poll_type_;
    this->poll_desc = poll_desc_;
    this->poll_controlByte = poll_controlByte_;
    this->poll_messType = poll_messType_;
    this->poll_data1 = poll_data1_;
    this->poll_data2 = poll_data2_;
    this->poll_data3 = poll_data3_;
    this->res_messType = res_messType_;
    this->err_messType = err_messType_;
    this->pollRequired = pollRequired_;
    this->pollReturned = false;
  };
};

/// trace and manage controller polls
struct poll_trace_t
{
  bool pollTimeCard;
  bool saveTimeCard;
  int poll_seq;
  int numspolled;
  long long poll_tms;
  poll_trace_t(bool pollTimeCard_,bool saveTimeCard_,int poll_seq_,int numspolled_,long long poll_tms_)
  {
    this->pollTimeCard = pollTimeCard_;
    this->saveTimeCard = saveTimeCard_;
    this->poll_seq = poll_seq_;
    this->numspolled = numspolled_;
    this->poll_tms = poll_tms_;
  };
  void resetcnt(void)
  {
    poll_seq = 0;
    numspolled = 0;
    poll_tms = 0;    
  };
  void resetpolls(void)
  {
    pollTimeCard = true;
    saveTimeCard = true;
    resetcnt();
  };
};

/// trace controller state
struct predicted_bound_t
{
  uint16_t bound_L;   /// in deciseconds
  uint16_t bound_U;   /// in deciseconds
  uint8_t  confidence;
  void reset(void)
  {
    bound_L = 0;
    bound_U = 0;
    confidence = SPAT_StateConfidence_unKnownEstimate;
  };
};

struct phase_status_t
{
  operation_mode_enum_t::lightcolor state;  /// traffic light color
  long long state_start_time;               /// in milliseconds
  operation_mode_enum_t::pedsign pedstate;  /// pedestrian light state
  long long pedstate_start_time;            /// in milliseconds
  operation_mode_enum_t::calltype call_status;
  operation_mode_enum_t::recalltype recall_status;
  predicted_bound_t time2next;
  predicted_bound_t pedtime2next;
  void resetTime2next(void)
  {
    time2next.reset();
    pedtime2next.reset();
  };
};

struct controller_status_t
{
  /// timestamp
  long long tms;                   // in milliseconds
  bool isPlantimingReady;          // need to switch curplan if received signal_status indicating a plan transition
  /// data received from the controller
  signal_status_mess_t  signal_status;  // received signal_status_mess_t
  std::bitset<8> status;           // from received status8e_mess_t
  uint8_t active_force_off[8];     // from received signal_status_mess_t
  /// current control mode
  operation_mode_enum_t::controlmode mode;
  std::bitset<8> permitted_phases;
  std::bitset<8> permitted_ped_phases;  
  uint8_t curbarrier;
  long long curbarrier_start_time; // in milliseconds, onset of barrier change  
  long long timer_time[2];         // in milliseconds, onset of active phase/active interval/interval_timer change  
  /// parameters related with coordination
  ssize_t coordplan_index;         // index in timing_card_t::coordplans
  long long cycle_start_time;      // in milliseconds, onset of cycle change, cycle starts at the yield point
  long long cycle_clock_time;      // in milliseconds, onset of local cycle clock change
  /// vehicular and ped phase status
  phase_status_t phase_status[8];
};

/// trace softcall state
struct softcall_state_t
{
  long long tms;            /// timestamp that the last softcall command is sent to the controller
  std::bitset<8> ped_call;  /// only need send to the controller once
  std::bitset<8> veh_call;  /// continue send to the controller when phases are not in green
  std::bitset<8> veh_ext;   /// continue send to the controller when phases are in green
  std::bitset<8> prio_call; /// continue send to the controller when phases are not in green
  std::bitset<8> prio_ext;  /// continue send to the controller when phases are in green
  void reset(void)
  {
    tms = 0;
    ped_call.reset();
    veh_call.reset();
    veh_ext.reset();
    prio_call.reset();
    prio_ext.reset();
  };  
};

int open_port(std::ofstream& OS,const char *port_name,const bool isReadOnly);
void close_port(int fd,const bool isReadOnly);
void buildPollList(std::vector<poll_conf_t>& list);
bool isAllPollReturned(const std::vector<poll_conf_t>& poll_list_);
void resetPlanPolls(std::vector<poll_conf_t>& list);
void processRecvBuff(uint8_t* buf,size_t& count,std::vector< std::pair<size_t, size_t> >& vec);
bool processAB3418Msg(std::ofstream& OS,uint8_t* buf,const uint8_t* pmsg,const size_t msglen);
void parseSignalStatusMsg(signal_status_mess_t& signalstatus,const uint8_t* pmsgbuf);
void parseStatus8eMsg(status8e_mess_t& status8e,const uint8_t* pmsgbuf);
void parseLongStatus8eMsg(longstatus8e_mess_t& longstatus8e,const uint8_t* pmsgbuf);
int getPollIdx(const std::vector<poll_conf_t>& poll_list_,const uint8_t* pResp,const uint8_t messType);
void updateTimingCard(timing_card_t& card,const uint8_t* pmsgbuf,const std::string& pollDesc);
size_t formpollrequest(uint8_t* buf,const uint8_t addr,const poll_conf_t& poll_conf);
size_t formsoftcallrequest(uint8_t* buf,const uint8_t addr,const std::bitset<8>& veh_call,
  const std::bitset<8>& ped_call,const std::bitset<8>& prio_call);
std::string ab3418msgErrCode(const uint8_t err_num);
void updatePermitPhases(std::bitset<8>& permitted_phases,std::bitset<8>& permitted_ped_phases,
  const operation_mode_enum_t::controlmode cntrMode,const ssize_t planIdx,const timing_card_t& timingcard);
void bitsetphases2ringphases(uint8_t (&ringphases)[2],const std::bitset<8>& bitsetPhases);
uint8_t ring_phase_on(const uint8_t check_phase);
uint8_t barrier_phases_on(const std::bitset<8>& check_phases);
uint8_t next_ring(const uint8_t check_ring);
uint8_t next_barrier(const uint8_t check_barrier);
void getPlanParameters(freeplan_mess_t& freeplan,const std::bitset<8>& permitted_phases,const std::bitset<8>& permitted_ped_phases);
void getPlanParameters(std::vector<coordplan_mess_t>& coordplans,const TSPconf_mess_t& TSPconf,const phasetiming_mess_t (&phasetiming)[8],
  const std::bitset<8>& permitted_phases,const std::bitset<8>& permitted_ped_phases,const std::bitset<8>& maximum_recall_phases);  
operation_mode_enum_t::controlmode get_control_mode(const std::bitset<8>& controller_status,const std::bitset<8>& preempt,const uint8_t pattern_num);  
operation_mode_enum_t::lightcolor get_light_color(const uint8_t active_phase,const uint8_t active_intv,const uint8_t check_phase);
operation_mode_enum_t::pedsign get_ped_state(const uint8_t active_phase,const uint8_t active_intv,const uint8_t check_phase);
operation_mode_enum_t::recalltype getPhaseRecallType(const phaseflags_mess_t& phaseflags,const coordplan_mess_t& coordplan,const uint8_t phaseIdx);
operation_mode_enum_t::recalltype getPhaseRecallType(const phaseflags_mess_t& phaseflags,const freeplan_mess_t& freeplan,const uint8_t phaseIdx);
operation_mode_enum_t::concurrentphasetype getConcurrentPhaseType(const std::bitset<8>& active_phase,const std::bitset<8>& sync_phase);
uint8_t getPhaseWalkInterval(const phasetiming_mess_t& phasetiming,const phaseflags_mess_t& phaseflags,const uint8_t phaseIdx);
uint16_t getPedIntervalLeft(const uint8_t interval_timer,const long long timer_time,const long long tms);
void barrierCrossAdjust(predicted_bound_t (&time2start)[2]);
void updateActivePhaseTime2next(phase_status_t& phase_status,predicted_bound_t& time2start,const signal_status_mess_t& signalstatus,
  const phasetiming_mess_t& phasetiming,const phaseflags_mess_t& phaseflags,const uint8_t ring,const long long timer_time,const long long tms);  
void updateActivePhaseTime2next(phase_status_t& phase_status,predicted_bound_t& time2start,const coordplan_mess_t* pcoordplan,const signal_status_mess_t& signalstatus,
  const phasetiming_mess_t& phasetiming,const phaseflags_mess_t& phaseflags,const uint8_t ring,const uint16_t local_cycle_clock,const uint16_t cycle_length,
  const operation_mode_enum_t::concurrentphasetype concurrentType,const long long timer_time,const long long tms);
void getNextPhaseStartBound(predicted_bound_t& time2start,const phasetiming_mess_t& phasetiming,const phaseflags_mess_t& phaseflags,
  const uint8_t phaseIdx,const phase_status_t& phase_status);
void getNextPhaseStartBound(predicted_bound_t& time2start,const phasetiming_mess_t& phasetiming,const phaseflags_mess_t& phaseflags,const uint8_t phaseIdx,
  const phase_status_t& phase_status,const coordplan_mess_t* pcoordplan,const uint16_t local_cycle_clock,const uint16_t cycle_length);
void softcallPhaseReset(std::bitset<8>& softcall,uint8_t requestPhases);
void softcallPhaseSet(std::bitset<8>& callPhases,const uint8_t requestPhases);
void updateSoftcallState(softcall_state_t& softcallState,const softcall_request_t* pRequest);
void updateSoftcallState(softcall_state_t& softcallState,const phase_status_t* pPhase_status);

bool sendPresMsg(int fd,char* buf,const status8e_mess_t& status8e);
bool sendCntMsg(int fd,char* buf,const longstatus8e_mess_t& longstatus8e);
bool sendSigStaMsg(int fd,char* buf,const controller_status_t& cntrstatus,const bool isManualCntr);

void logSignalStatusMsg(std::ofstream& OS,const signal_status_mess_t& signalstatus);

#endif
