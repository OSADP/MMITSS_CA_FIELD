//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************
#ifndef _DSRCMSGENUM_H
#define _DSRCMSGENUM_H

namespace MsgEnum
{
  struct msg_enum_t
  {
    enum MSGTYPE {UNKNOWN_PKT = 0,BSM_PKT = 1,SPAT_PKT = 2,MAP_PKT = 3,SRM_PKT = 4,SSM_PKT = 5};
  };
  struct map_enum_t
  {
    enum approachType { APPROACH = 1, EGRESS, BARRIER, CROSSWALK };
    enum laneType {TRAFFICLANE = 1,CROSSWALKLANE = 4};    
    enum maneuverType { UNKNOWN = 0, UTURN = 1, LEFTTURN, RIGHTTURN, STRAIGHTAHEAD, STRAIGHT };    
  };
  struct phasestate_enum_t
  { 
    enum vehicular {UNKNOWN, GREEN, YELLOW, RED, FLASHING};
    enum pedestrian {UNAVAILABLE, STOP, CAUTION, WALK};
    enum confidentce {UNKNOWN_ESTIMATE, MINTIME, MAXTIME, TIME_LIKELY_TO_CHANGE};
  };    
  struct priorityrequest_enum_t
  {
    enum vehicularrequest {CANCELPRIORITY = 0x00, REQUESTPRIORITY = 0x10, CANCELPREEMP = 0x80, REQUESTPREEMP = 0x90,UNKNOWREQUEST = 0xF0};
    enum requeststatus  {NOTVALID,REJECTED,NOTNEEDED,QUEUED,ACTIVE,CANCELLED,COMPLETED};
  };
  /* NTCIP priorityRequestStatusInPRS OBJECT-TYPE SYNTAX INTEGER 
      idleNotValid (1), readyQueued (2), readyOverridden (3), activeProcessing (4), activeCancel (5),
      activeOverride (6), activeNotOverridden (7), closedCanceled (8), ReserviceError (9), closedTimeToLiveError (10),
      closedTimerError (11), closedStrategyError (12), closedCompleted (13), activeAdjustNotNeeded (14), closedFlash (15) */  
}

#endif
