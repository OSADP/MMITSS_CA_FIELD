#ifndef _MRPDATAMGR_H
#define _MRPDATAMGR_H

#include <stdint.h>
#include <vector>

#include "timeUtils.h"
#include "msgUtils.h"
#include "dsrcMsgs.h"

/// current phase state bit mapping (Battelle's version)
static const uint32_t BALL_GREEN              = 0x00000001;
static const uint32_t BALL_YELLOW             = 0x00000002;
static const uint32_t BALL_RED                = 0x00000004;
static const uint32_t BALL_FLASHING           = 0x00000008;
static const uint32_t LEFTARROW_GREEN         = 0x00000010;
static const uint32_t LEFTARROW_YEWLLOW       = 0x00000020;
static const uint32_t LEFTARROW_RED           = 0x00000040;
static const uint32_t LEFTARROW_FLASHING      = 0x00000080;
static const uint32_t RIGHTARROW_GREEN        = 0x00000100;
static const uint32_t RIGHTARROW_YEWLLOW      = 0x00000200;
static const uint32_t RIGHTARROW_RED          = 0x00000400;
static const uint32_t RIGHTARROW_FLASHING     = 0x00000800;
static const uint32_t STRAIGHTARROW_GREEN     = 0x00001000;
static const uint32_t STRAIGHTARROW_YEWLLOW   = 0x00002000;
static const uint32_t STRAIGHTARROW_RED       = 0x00004000;
static const uint32_t STRAIGHTARROW_FLASHING  = 0x00008000;
static const uint32_t UTURNARROW_GREEN        = 0x01000000;
static const uint32_t UTURNARROW_YEWLLOW      = 0x02000000;
static const uint32_t UTURNARROW_RED          = 0x04000000;
static const uint32_t UTURNARROW_FLASHING     = 0x08000000;
/// crosswalk indication (Battelle's version)
static const uint32_t CROSS_NOTEQUIPPED       = 0x00;
static const uint32_t CROSS_DONT_WALK         = 0x01;
static const uint32_t CROSS_FLASH_DONOT_WALK  = 0x02;
static const uint32_t CROSS_WALK              = 0x03;
static const uint32_t CROSS_FLASHING          = 0x02;

struct det_cnt_t
{
  uint32_t mssent;
  count_data_t data;
};

struct det_pres_t
{
  uint32_t mssent;
  pres_data_t data;
};

struct cntrl_state_t
{
  uint32_t mssent;
  controller_state_t data;
};

struct softreq_t
{
  uint32_t mssent;
  softcall_request_t data;
};

struct veh_troj_t
{
  uint32_t mssent;
  vehTroj_t data;
};

bool sendMMITSSmsg(int fd,char* buf,const char* msg,const size_t msglen,const uint32_t ms,const uint8_t msgid);
bool send2cloud(int fd,char* buf,const char* msg,const size_t msglen,
  const timeUtils::timeStamp_t& ts,const uint32_t intID,const uint8_t msgtype);
bool sendSignalStateMsg(int fd,char* buf,const signal_state_t& sigState,
  const char* payload,size_t payloadlen,const uint32_t ms,const uint8_t msgid);
void formSPaT(SPAT_element_t& SPAT_element,const controller_state_t& cntrState,const timing_card_t& timingcard,const uint32_t intID);
void formPerm(intPerm_t& intPerm,std::vector< std::vector<vehTroj_t> >& a_vehTroj,const timeUtils::timeStamp_t& timeStamp,
  const uint32_t intID,const uint8_t permitted_phases,const uint8_t* pSpeedLimits);

#endif
