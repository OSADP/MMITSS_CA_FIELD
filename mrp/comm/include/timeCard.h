#ifndef _CONTROLLERTIMECARD_H
#define _CONTROLLERTIMECARD_H

#include <string>
#include <cstring>
#include <stdint.h>
#include <vector> 
#include <map> 
#include <bitset>  

static const uint8_t pattern_flashing = 0xFE;   // 254
static const uint8_t pattern_free     = 0xFF;   // 255

/// enum for traffic signal control operations
struct operation_mode_enum_t
{
  enum controlmode          {UNKNOWN=0,FLASHING=1,PREEMPTION=2,RUNNINGFREE=3,COORDINATION=4};  
  enum leadlagoperation     {LEAD_LEAD=0,LEAD_LAG=1,LAG_LEAD=2,LAG_LAG=3};
  enum recalltype           {NORECALL,MINIMUMRECALL,MAXIMUMRECALL,PEDRECALL,BIKERECALL};
  enum calltype             {NOCALL,CALLVEH,CALLPED,CALLBIKE};
  enum lightcolor           {NOTPERMITTED,GREEN,YELLOW,RED};
  enum pedsign              {NOTEQUIPPED,WALK,FLASH_DONOT_WALK,DONT_WALK};
  enum concurrentphasetype  {MINOR_MINOR,MINOR_MAJOR,MAJOR_MAJOR};
};

/// struct for timing_card_t (shared among MRP_TCI, MRP_DataMgr & MRP_Aware)
struct phaseflags_mess_t  
{
  std::bitset<8> permitted_phases;
  std::bitset<8> restricted_phases;     // phases can not be timed concurrently
  std::bitset<8> permitted_ped_phases;            
  /// recall
  std::bitset<8> minimum_recall_phases;
  std::bitset<8> maximum_recall_phases; 
  std::bitset<8> ped_recall_phases;     
  std::bitset<8> bike_recall_phases;
  /// lock
  std::bitset<8> redlock_phases;      // retain calls received duing the red interval
  std::bitset<8> yewlock_phases;      // retain calls received during the yellow and red intervals
  std::bitset<8> fomaxlock_phases;    // retain calls received during the force-off and maxed-out intervals
  /// feature
  std::bitset<8> doubleEntry_phases;  // phase is timed if not call on its ring and a compatible phase is timed on the other ring 
  std::bitset<8> restInWalk_phases;    
  std::bitset<8> restInRed_phases;    // phase will be rest in red in the absence of any calls to the phase
  std::bitset<8> walk2_phases;
  std::bitset<8> maxgreen2_phases;
  std::bitset<8> maxgreen3_phases;
  /// start up
  std::bitset<8> startup_green_phases;
  std::bitset<8> startup_yellow_phases;
  std::bitset<8> startup_vehCalls;
  std::bitset<8> startup_pedCalls;
  std::bitset<8> startup_yellowOverlaps;
  uint8_t startup_allred;               // in deciseconds
  /// red revert
  uint8_t red_revert_interval;          // red interval following yellow and before returning to green on the same phase. in deciseconds, need getTimingData poll (0x7200)
};

struct phasetiming_mess_t
{
  uint8_t phase_num;    
  /// the following variables are in sec increment
  uint8_t walk1_interval;
  uint8_t walk_clearance;
  uint8_t minimum_green;
  uint8_t detector_limit;
  uint8_t maximum_initial;
  uint8_t maximum_extensions[3];
  /// the following variables are in decisecond increment
  uint8_t passage;
  uint8_t maximum_gap;
  uint8_t minimum_gap;
  uint8_t added_initial_per_vehicle;
  uint8_t reduce_gap_by;
  uint8_t reduce_gap_every;
  uint8_t yellow_interval;
  uint8_t red_clearance;
  /// the following variables are in sec increment
  uint8_t walk2_interval;
  uint8_t delay_early_walk_time;
  uint8_t solid_walk_clearance;
  uint8_t bike_green;
  /// the following variables are in decisecond increment    
  uint8_t bike_red_clearance;
};

struct coordplan_mess_t
{
  /// control plan poll
  uint8_t plan_num;                     // 1 - 9, 11 - 19, 21 - 29
  uint8_t cycle_length;                 // in sec
  uint8_t green_factor[8];              // in sec
  uint8_t cycle_multiplier;             // 0.1
  uint8_t offsets[3];                   // in sec
  uint8_t laggapout_phase;              // 1 - 8
  std::bitset<8> lag_phases;
  std::bitset<8> sync_phases;
  std::bitset<8> hold_phases;           // rest in walk at the end of walk, can't max out
  std::bitset<8> omit_phases;           // will not be served in local plan
  std::bitset<8> minimum_recall_phases;
  std::bitset<8> maximum_recall_phases; // continuous call
  std::bitset<8> ped_recall_phases;
  std::bitset<8> bike_recall_phases;
  uint8_t force_off_flag;               // = 0, green_factor is green factor
                                        // = 1, green_factor is force-off
  /// TSP plan poll
  bool isTSPenabled;
  uint8_t max_early_green;              // in sec
  uint8_t max_green_extension;          // in sec
  uint8_t inhibit_cycles;
  uint8_t TSP_force_off[8];             // in sec
  /// the following variables are processed based on control plan poll, phase flag poll and phase timing poll
  bool coordLagphaseGapout;
  std::bitset<8> permitted_phases;
  std::bitset<8> permitted_ped_phases;
  uint8_t coordinated_phases[2];        // could be only one phase on two rings
  operation_mode_enum_t::leadlagoperation leadLagMode;
  uint8_t sync_ring;
  uint8_t sync_barrier;
  uint8_t leadlag_phases[2][2][2];      // i - barrier, j - ring, k - lead and lag phase  
  uint8_t force_off[8];                 // in sec  
  uint8_t permissive[8];                // in sec
  uint8_t ped_permissive[8];            // in sec
  uint32_t noncoordBarrierGreenOnset;   // in deciseconds, green onset for non-coordinated barrier
  uint32_t coordBarrierGreenOnset;      // in deciseconds, green onset for coordinated barrier
  uint32_t coordPhaseGreenOnset[2];     // in deciseconds, green onset for coordinated phases
  uint32_t coordPhaseGreenEnd[2];       // in deciseconds, yellow onset for coordinated phases, consider lag phase gap-out feature
};

struct freeplan_mess_t
{
  /// free plan poll
  std::bitset<8> lag_phases;
  std::bitset<8> omit_phases;
  std::bitset<8> minimum_recall_phases;
  std::bitset<8> maximum_recall_phases;
  std::bitset<8> ped_recall_phases;
  std::bitset<8> bike_recall_phases;
  std::bitset<8> conditional_service_phases;  // odd phases, can be served more than once in a cycle
                                              // setting here overwrites TOD setting
  uint8_t conditional_service_minimum_green;  // in seconds
  /// TSP free running poll
  bool isTSPenabled;
  std::bitset<8> TSP_hold_phases;
  uint8_t TSP_max_green_hold_time;            // in sec
  /// the following variables are processed based on free plan poll and phase flag poll
  std::bitset<8> permitted_phases;
  std::bitset<8> permitted_ped_phases;  
  uint8_t leadlag_phases[2][2][2];            // i - barrier, j - ring, k - lead and lag phase  
};

struct manualplan_mess_t
{
  bool planOn;
  uint8_t plan_num;   
  uint8_t offset_index;
};

struct cicplan_mess_t
{
  std::bitset<9> enabled_plans;   // 9 local plans
  uint8_t smoothing_volume;       // 0.01
  uint8_t smoothing_occupancy;    // 0.01
  uint8_t smoothing_demand;       // 0.01
  uint8_t multiplier_volume;      // 0.1
  uint8_t multiplier_occupancy;   // 0.01
  uint8_t exponent_volume;        // 0.01
  uint8_t exponent_occupancy;     // 0.01
  uint8_t phase_assignment[16];   // detector-to-phase assignment (1-8) for 16 system detectors
                                  // 0 means the detector is not used for CIC operation
};

struct detectorconf_mess_t
{
  uint8_t type;                   // 0 = None, 1 = Count, 2 = Call, 3 = Extend, 4 = Count + Call
                                  // 5 = Call + Extend, 6 = Count + Call + Extend, 7 = Count + Extend
                                  // 8 = Limited, 9 = Bicycle, 10 = Pedestrian
  std::bitset<8> phaseAssignment; // usually assign to one phase but can be multiple
  uint8_t lock;                   // 0 = No, 1 = Red, 2 = Yellow
  uint8_t delayTime;
  uint8_t extendTime;             // deciseconds
  uint8_t recallTime;             // recall assigned phase when detector in failure
  uint8_t inputPort;              // MS Digit = port, LS Digit = Bit
                                  // e.g. 13 (1101):  port = 1, bit = 3
};

struct system_detector_assignment_mess_t
{
  uint8_t maxOnTime;              // minute (identify a detector failure)
  uint8_t failResetTime;          // minute (interval for controller to reset failed detector)
  std::bitset<44> failOverride;   // bits mapped with 44 detectors
  uint8_t detectorInput[16];      // detector input to each of the 16 system detectors, 0 means unused system detector
};

struct TODtable_mess_t
{
  uint8_t table_num;
  uint8_t start_hour;
  uint8_t start_min;
  uint8_t plan_num;
  uint8_t offset_index;
};

struct weekday_plan_assignment_mess_t
{
  uint8_t dayofweek[7]; // 0 = Monday, 6 = Sunday
};

struct TODfunction_mess_t
{
  uint8_t start_hour;
  uint8_t start_min;
  uint8_t end_hour;
  uint8_t end_min;
  std::bitset<8> dayofweek; // bit 0 - Monday
  uint8_t action_code;      // 0 – None, 1 – Permitted, 2 – Restricted, 4 – Veh Min Recall, 5 – Veh Max Recall
                            // 6 – Ped Recall, 7 – Bike Recall, 8 – Red Lock, 9 – Yellow Lock, 10 – Force/Max Lock
                            // 11 – Double Entry, 12 – Y-Coord C, 13 – Y-Coord D, 14 – Free, 15 – Flashing
                            // 16 – Walk 2, 17 – Max Green 2, 18 – Max Green 3, 19 – Rest in Walk, 20 – Rest in Red
                            // 21 – Free Lag Phases, 22 – Special Functions, 23 – Truck Priority, 24 – Conditional Services
                            // 25 – Conditional Services, 26 – Leading Ped, 27 – Traffic Actuated Max2, 41 – Protected Permissive
                            // 42 – Protected Permissive, 100+Action Code = Phases removed, 200+Action Code = Phases replaced
  std::bitset<8> affect_phases;
};

struct RRpreemption_steps_mess_t
{
  uint8_t step_time;      // in sec
  std::bitset<8> ped_walk_phases;
  std::bitset<8> ped_clear_phases;
  std::bitset<8> ped_red_phases;
  std::bitset<8> green_hold_phases;
  std::bitset<8> yew_flashing_phases;
  std::bitset<8> red_flashing_phases;  
  std::bitset<8> green_hold_overlaps;
  std::bitset<8> yew_flashing_overlaps;
  std::bitset<8> red_flashing_overlaps;
};

struct RRpreemption_mess_t
{
  uint8_t delay_time;       // in deciseconds
  /// terminating active phase
  uint8_t minimum_green;    // in sec
  uint8_t ped_clear_time;   // in sec
  /// 4 steps after delay_time, clear 1, 2, 3 and hold
  RRpreemption_steps_mess_t RRpreemption_steps[4];
  /// exit step
  uint8_t exit_time;        // in sec
  std::bitset<8> exit_phases_green;
  std::bitset<8> exit_overlaps_green;
  std::bitset<8> exit_veh_call;
  std::bitset<8> exit_ped_call;
  /// conf
  uint8_t input_port;
  uint8_t gate_port;
  uint8_t latching_flag;
  uint8_t power_up;
};

struct EVpreemption_mess_t
{
  uint8_t delay_time;             // in sec
  uint8_t green_hold_time;        // in sec (green time after the preempt input is deactivated)
  uint8_t maximum_clearance_time; // in sec (maximum green time for the clearance phase)
  std::bitset<8> clearance_phase_green;
  std::bitset<8> clearance_overlap_green;
  uint8_t input_port;
  uint8_t latching_flag;         // 0 - no, 1 - yes 
                                 // if latched, EV will be initiated even if the input is deactivated prior to the completion of delay_time
  uint8_t phase_termination_flag; // 0 - force-off, 2 - advance (termination type of active phase prior to clearance phase)
                                  // advance abort walking interval, force-off not, both not shortern ped clearance interval
};

struct TSPplan_mess_t
{
  uint8_t max_early_green;      // in sec
  uint8_t max_green_extension;  // in sec
  uint8_t inhibit_cycles;       
  uint8_t green_factor[8];      // in sec
};

struct TSPconf_mess_t
{
  std::bitset<18> enable_coordination_plans;  // plan 1 - 9, 11 - 19, bit 0 = plan 1
  TSPplan_mess_t TSPplan[18];
};

struct timing_card_t
{
  uint8_t             controller_addr;
  phaseflags_mess_t   phaseflags;
  phasetiming_mess_t  phasetiming[8];
  freeplan_mess_t     freeplan;
  manualplan_mess_t   manualplan;
  cicplan_mess_t      cicplans;
  std::vector<detectorconf_mess_t> detectorconf;
  system_detector_assignment_mess_t system_detector_assignment;
  std::vector<TODtable_mess_t> TODtables;
  weekday_plan_assignment_mess_t weekday_plan_assignment;
  std::vector<TODfunction_mess_t> TODfunctions;
  RRpreemption_mess_t RRpreemption[2];
  EVpreemption_mess_t EVpreemption[4];
  TSPconf_mess_t TSPconf;
  std::vector<coordplan_mess_t> coordplans;
  std::map<uint8_t,size_t> plannum2planindex_map; // key: plan_num; value: index in coordplans
  void reset(void)
  {
    detectorconf.clear();
    TODtables.clear();
    coordplans.clear();
    plannum2planindex_map.clear();
  };
  void resetplans(void)
  {
    coordplans.clear();
    plannum2planindex_map.clear();    
  }
};

void getPlanIdxMap(std::map<uint8_t,size_t>& plannum2planindex_map,const std::vector<coordplan_mess_t>& coordplans);
ssize_t get_coordplan_index(const uint8_t plan_num,const std::map<uint8_t,size_t> plannum2planindex_map);
bool getPatnIdx(ssize_t& planIdx,const operation_mode_enum_t::controlmode cntrMode,
  const uint8_t plan_num,const std::map<uint8_t,size_t>& plannum2planindex_map);
void pattern2planOffset(uint8_t& plan_num,uint8_t& offset_index,const uint8_t pattern_num);
uint8_t planOffset2pattern(const uint8_t plan_num,const uint8_t offset_index);
bool logTimeCard(const char* ptr,const timing_card_t& timingcard);
bool readTimeCard(const char* ptr,timing_card_t& timingcard);

#endif
