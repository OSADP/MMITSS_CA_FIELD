#ifndef _MMITSSMSGUTILS_H
#define _MMITSSMSGUTILS_H

#include <fstream>
#include <stdint.h> // c++11 <cstdint>
#include <string>
#include <cstring>

#define MAXUDPMSGSIZE 2000

/// MMITSS header 
static const uint16_t msg_header          = 0xFFFF;
/// MMITSS header msgid
static const uint8_t msgid_softcall       = 0x50; 
static const uint8_t msgid_troj           = 0x51; 
static const uint8_t msgid_phaseflags     = 0x52; 
static const uint8_t msgid_phasetiming    = 0x53;
static const uint8_t msgid_coordplan      = 0x54;
static const uint8_t msgid_freeplan       = 0x55;
static const uint8_t msgid_detAsgmnt      = 0x56;
static const uint8_t msgid_detCnt         = 0x57;
static const uint8_t msgid_detPres        = 0x58;
static const uint8_t msgid_cntrlstatus    = 0x59;
static const uint8_t msgid_signalstatus   = 0x60;
static const uint8_t msgid_perm           = 0x61;
/// message poll request
static const uint8_t msgid_pollReq        = 0x80;
/// savari cloud msg type
static const uint8_t savari_cloud_map     = 0x01;
static const uint8_t savari_cloud_spat    = 0x02;
/// StateConfidence
static const uint8_t SPAT_StateConfidence_unKnownEstimate      = 0;
static const uint8_t SPAT_StateConfidence_minTime              = 1;
static const uint8_t SPAT_StateConfidence_maxTime              = 2;
static const uint8_t SPAT_StateConfidence_timeLikeklyToChange  = 3;

/// enum for soft-call status
struct softcall_enum_t
{
  enum action   {NOACTION = 0, PENDING = 1, CALLED = 2, CANCELLED = 3};
  enum callObj  {NONE = 0, PED = 1, VEH = 2, PRIORITY = 3};
  enum callType {UNKNOWN = 0,CALL = 1,EXTENSION = 2, CANCEL = 3};
};

/// Savari udp message header (between MRP_DataMgr and the cloud server)
struct savari_udp_header_t 
{
  uint8_t   type;
  uint32_t  len;
  uint32_t  seconds;
  uint16_t  msecs;
  uint32_t  intersectionID;
}__attribute__((packed));

/// MMITSS udp message header (between MMITSS components)
struct mmitss_udp_header_t
{
  uint16_t  msgheader;            
  uint8_t   msgid;
  uint32_t  ms_since_midnight;
  uint16_t  length;    
}__attribute__((packed));

/// MMITSS udp messages (between MMITSS components)
/// sharing of (msgid_phaseflags, msgid_phasetiming, msgid_coordplan, msgid_freeplan, and msgid_detAsgmnt)
/// are through sharing controller timecard file. MRP_TCI polls the controller to generate the timecard file,
/// and MRP_Aware and MRP_DataMgr reads controller configuration data from the timecard
/*--------------------------------------------------------------------------------------------
  softcall data structure (msgid_softcall) (send from MRP_Aware via MRP_DataMgr to MRP_TCI)
  ---------------------------------------------------------------------------------------------*/
struct softcall_request_t
{
  uint8_t callphase;      // bit mapped phase, bits 0-7 phase 1-8 
  uint8_t callobj;        // softcall_enum_t::callObj
  uint8_t calltype;       // softcall_enum_t::callType
}__attribute__((packed));

/*--------------------------------------------------------------------------------------------
  detector count/occupany data structure (msgid_detCnt) (send from MRP_TCI to MRP_DataMgr)
  ---------------------------------------------------------------------------------------------*/
struct count_data_t
{
  uint8_t seq_num;
  uint8_t flag;
  uint8_t status;
  uint8_t pattern_num;
  uint8_t master_cycle_clock;
  uint8_t local_cycle_clock;
  uint8_t vol[16];
  uint8_t occ[16];
}__attribute__((packed));

/*--------------------------------------------------------------------------------------------
  detctor presence structure (msgid_detPres) (send from MRP_TCI to MRP_DataMgr)
  --------------------------------------------------------------------------------------------*/
struct pres_data_t
{
  uint8_t flag;
  uint8_t status;
  uint8_t pattern_num;
  uint8_t master_cycle_clock;
  uint8_t local_cycle_clock;
  uint16_t prio_busId;
  uint8_t prio_busDirection;
  uint8_t prio_type;
  char presences[41];
}__attribute__((packed));

/*--------------------------------------------------------------------------------------------
  controller status data structure (msgid_cntrlstatus) (send from MRP_TCI to MRP_DataMgr)
  --------------------------------------------------------------------------------------------*/
struct change_state_t
{
  uint16_t bound_L;   /// in deciseconds
  uint16_t bound_U;   /// in deciseconds
  uint8_t  confidence;
}__attribute__((packed));

struct phase_state_t
{
  uint8_t state;           // operation_mode_enum_t::lightcolor
  uint8_t pedstate;        // operation_mode_enum_t::pedsign
  uint8_t call_status;     // operation_mode_enum_t::calltype
  uint8_t recall_status;   // operation_mode_enum_t::recalltype
  change_state_t time2next;
  change_state_t pedtime2next;
}__attribute__((packed));

struct signal_state_t
{
  uint8_t cntlrMode;       // operation_mode_enum_t::controlmode
  uint8_t pattern_num;     // plan_num + offset
  uint8_t permitted_phases;
  uint8_t permitted_ped_phases;
  uint8_t preempt;
  uint8_t ped_call;
  uint8_t veh_call;
}__attribute__((packed));
  
struct controller_state_t
{
  signal_state_t sigState;
  uint8_t status;          // SPaT status  
  uint16_t timeStamp;      // SPaT timeStamp
  phase_state_t phaseState[8];
}__attribute__((packed));

/*--------------------------------------------------------------------------------------------
  vehicle trajectory structure (msgid_troj) (send from MRP_Aware to MRP_DataMgr)
  --------------------------------------------------------------------------------------------*/
struct vehTroj_t
{
  uint32_t  trojCnt;
  uint32_t  vehId;
  uint8_t   entryLaneId;
  uint8_t   entryControlPhase;
  uint8_t   leaveLaneId;
  uint8_t   leaveControlPhase;
  uint16_t  distTraveled;       // in decimeter
  uint16_t  timeTraveled;       // in decisecond
  uint16_t  stoppedTime;        // in decisecond
  uint16_t  ingressLen;         // in decimeter
}__attribute__((packed));

/*--------------------------------------------------------------------------------------------
  performance measure structure (msgid_perm) (send from MRP_DataMgr to MRP server)
  --------------------------------------------------------------------------------------------*/
struct apchPerm_t
{
  uint16_t sampleNums;
  uint16_t tt_avg;        // in  decisecond
  uint16_t tt_std;
  uint16_t delay_avg;     // in  decisecond
  uint16_t delay_std;     // in  decisecond
  uint16_t stoppedSamples;
}__attribute__((packed));  

struct intPerm_t
{
  uint8_t  hr;              // local time
  uint8_t  min;
  uint8_t  sec;
  uint16_t millisec;
  uint32_t intId;
  uint8_t  permitted_phases; // bit set on for permitted phases;
  apchPerm_t apchPerm[8];
}__attribute__((packed)); 
  
void logDetPresMsg(std::ofstream& OS,const pres_data_t* pdata,const uint32_t ms);
void logDetPresMsg(std::ofstream& OS,const pres_data_t* pdata,const uint32_t ms,const std::string& dateStr);
void logDetCntMsg(std::ofstream& OS,const count_data_t* pdata,const uint32_t ms);
void logDetCntMsg(std::ofstream& OS,const count_data_t* pdata,const uint32_t ms,const std::string& dateStr);
void logSigStateMsg(std::ofstream& OS,const controller_state_t* pdata,const uint32_t ms);
void logSigStateMsg(std::ofstream& OS,const controller_state_t* pdata,const uint32_t ms,const std::string& dateStr);
void logSoftReqMsg(std::ofstream& OS,const softcall_request_t* pdata,const uint32_t ms);
void logSoftReqMsg(std::ofstream& OS,const softcall_request_t* pdata,const uint32_t ms,const std::string& dateStr);
void logPayloadHex(std::ofstream& OS,const char* buf,const size_t size);
void logVehTroj(std::ofstream& OS,const vehTroj_t* pdata,const uint32_t ms);
void logVehTroj(std::ofstream& OS,const vehTroj_t* pdata,const uint32_t ms,const std::string& dateStr);
void logPermMsg(std::ofstream& OS,const intPerm_t* pdata, const uint32_t ms);
void logPermMsg(std::ofstream& OS,const intPerm_t* pdata, const uint32_t ms,const std::string& dateStr);

#endif
