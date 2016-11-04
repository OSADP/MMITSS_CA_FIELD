//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************
#ifndef AB3418MSGS_H
#define AB3418MSGS_H

#include <stdint.h> 
#include <bitset>  

#define MAXAB3418MSGSIZE 256

namespace AB3418MSG
{
  static const uint8_t flag = 0x7E;
  static const uint8_t ipi = 0xC0;
  
  /* send to controller */
  static const uint8_t get_controlByte = 0x33; 
  static const uint8_t set_controlByte = 0x13;
  /* receive from controller */
  static const uint8_t res_controlByte = 0x13;
  
  /* poll controller */
  static const uint8_t getBlockMsg_messType = 0x87;
  static const uint8_t getBlockMsgRes_messType = 0xC7;
  static const uint8_t getBlockMsgRes_errMessType = 0xE7;
  
  static const uint8_t getTimingData_messType = 0x89;
  static const uint8_t getTimingDataRes_messType = 0xC9;
  static const uint8_t getTimingDataRes_errMessType = 0xE9;
  
  /* set controller */
  static const uint8_t setBlockMsg_messType = 0x96;
  static const uint8_t setBlockMsgRes_messType = 0xD6;
  static const uint8_t setBlockMsgRes_errMessType = 0xF6;
 
  static const uint8_t setTimingData_messType = 0x99;
  static const uint8_t setTimingDataRes_messType = 0xD9;
  static const uint8_t setTimingDataRes_errMessType = 0xF9;
  
  static const uint8_t setSoftcall_messType = 0x9A;
  static const uint8_t setSoftcallRes_messType = 0xDA;
  static const uint8_t setSoftcallRes_errMessType = 0xFA;
  
  /* controller pushing out */
  static const uint8_t status8eRes_messType = 0xC8;  
  static const uint8_t longStatus8eRes_messType = 0xCD;
  static const uint8_t rawspatRes_messType = 0xCE;
  
  
#if 0    
  /// ab3418 message frames below are for reference. Message encoding/decoding is done in tci.cpp
  
  /* ab3418 messages sending to controller */
  struct getBlockMsg_t
  {
    // 10 bytes
    uint8_t start_flag;
    uint8_t controller_addr;
    uint8_t control;            
    uint8_t ipi;               
    uint8_t mess_type;          
    uint8_t pageId;
    uint8_t blockId;
    uint8_t FCSmsb;
    uint8_t FCSlsb;
    uint8_t end_flag;    
  };
  struct getTimingData_t
  {
    // 11 bytes
    uint8_t start_flag;
    uint8_t controller_addr;
    uint8_t control;
    uint8_t ipi;
    uint8_t mess_type;          
    uint8_t memory_msb;
    uint8_t memory_lsb;
    uint8_t num_bytes;          // 1 to 32 bytes
    uint8_t FCSmsb;
    uint8_t FCSlsb;
    uint8_t end_flag;
  };
  struct setSoftcall_t  
  {
    // 16 bytes  
    uint8_t start_flag;  
    uint8_t controller_addr;  
    uint8_t control;    
    uint8_t ipi; 
    uint8_t mess_type;   
    uint8_t veh_call;   
    uint8_t ped_call;
    uint8_t prio_call;   
    uint8_t spare[5]; 
    uint8_t FCSmsb;        
    uint8_t FCSlsb;  
    uint8_t end_flag; 
  };
  
  /* ab3418 messages pushed out from controller */
  struct rawSPaTRes_t      
  {
    // 38 bytes
    uint8_t start_flag;    
    uint8_t controller_addr;
    uint8_t control;  
    uint8_t ipi;         
    uint8_t mess_type; 
    uint8_t active_phase;  
    uint8_t interval[2];    
    uint8_t intv_timer[2];
    uint8_t next_phase; 
    uint8_t ped_call; 
    uint8_t veh_call; 
    uint8_t pattern_num;      // control plan + offset_index
    uint8_t local_cycle_clock;  
    uint8_t master_cycle_clock; 
    uint8_t preempt; 
    uint8_t permissive[8]; 
    uint8_t force_off[2];   
    uint8_t ped_permissive[8];
    uint8_t FCSmsb; 
    uint8_t FCSlsb; 
    uint8_t end_flag;
  };
  struct status8eRes_t    
  {
    // 35 bytes
    uint8_t start_flag;    
    uint8_t controller_addr;
    uint8_t control;  
    uint8_t ipi;         
    uint8_t mess_type; 
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint8_t falg;     // bit 0 = focus mode
                      // bit 2 = advance input status
                      // bit 3 = spare 3 input status
                      // bit 4 = spare 2 input status
                      // bit 5 = spare 1 input status
                      // bit 7 = transit vehicle call
    uint8_t controller_status;  // bit 0 = in preempt
                                // bit 1 = cabinet flash
                                // bit 2 = passed local zero since last request
                                // bit 3 = in local override mode
                                // bit 4 = coordination alarm pending
                                // bit 5 = detector fault pending
                                // bit 6 = non-critical alarm pending
                                // bit 7 = critical alarm pending
    uint8_t pattern_num;    
    uint8_t green_overlaps;     // bits 0-5 <--> Green Overlaps A-F 
    uint8_t yellow_overlaps;    // bits 0-5 <--> Yellow Overlaps A-F 
    uint8_t preempt;            // bits 0-3 <--> EV A-D
                                // bits 4-5 <--> RR 1-2
                                // bit 6 - pattern transition
                                // bit 7 – Transit Vehicle Priority
    uint8_t veh_call;
    uint8_t ped_call;
    uint8_t active_phase;
    uint8_t interval;           // bits 0-3 <--> Ring A interval
                                // bits 4-7 <--> Ring B interval
    uint8_t presence[5];        // Detector 1-8, 9-16,17-24,25-32,33-40
    uint8_t master_cycle_clock;
    uint8_t local_cycle_clock;
    uint8_t prioBusIdmsb;
    uint8_t prioBusIdlsb;
    uint8_t prioBusDirection;   //  5 = Phase2 opticom ON
                                // 13 = Phase2 opticom OFF
                                // 21 = Phase6 opticom ON
                                // 17 = Phase6 opticom OFF
    uint8_t prioType;           //  0 = no Priority
                                //  1 = Early Green
                                //  2 = Green Extension
    uint8_t spare[3];
    uint8_t FCSmsb;
    uint8_t FCSlsb;
    uint8_t end_flag;
  };
  struct volOccup_t
  {
    uint8_t volume;
    uint8_t occupancy;
  };
  struct longStatus8eRes_t   
  {
    // 68 bytes
    uint8_t start_flag;    
    uint8_t controller_addr;
    uint8_t control;  
    uint8_t ipi;         
    uint8_t mess_type; 
    uint8_t hour;
    uint8_t minute;
    uint8_t second;    
    uint8_t falg; 
    uint8_t controller_status;
    uint8_t pattern_num;    
    uint8_t green_overlaps; 
    uint8_t yellow_overlaps;
    uint8_t preempt;
    uint8_t veh_call;
    uint8_t ped_call;
    uint8_t active_phase;
    uint8_t interval; 
    uint8_t presence[5];  
    uint8_t master_cycle_clock;
    uint8_t local_cycle_clock;
    uint8_t seq_num;
    volOccup_t volumeOccupancy[16];
    uint8_t prioBusIdmsb;
    uint8_t prioBusIdlsb;
    uint8_t prioBusDirection;
    uint8_t prioType;
    uint8_t spare[3];
    uint8_t FCSmsb;
    uint8_t FCSlsb;
    uint8_t end_flag;
  };  
  
  /* ab3418 messages received from controller (polling response) */
  struct blockMsgErrorRes_t
  {
    // 12 bytes
    uint8_t start_flag;
    uint8_t controller_addr;
    uint8_t control;            
    uint8_t ipi;                
    uint8_t mess_type;  
    uint8_t pageId;
    uint8_t blockId;
    uint8_t err_num;
    uint8_t index_num;
    uint8_t FCSmsb;
    uint8_t FCSlsb;
    uint8_t end_flag;        
  };
  struct timingDataErrorRes_t
  {
    // 10 bytes
    uint8_t start_flag;
    uint8_t controller_addr;
    uint8_t control;            
    uint8_t ipi;                
    uint8_t mess_type;  
    uint8_t err_num;
    uint8_t index_num;
    uint8_t FCSmsb;
    uint8_t FCSlsb;
    uint8_t end_flag;        
  };
  struct setBlockMsgRes_t   
  {
    // 8 bytes
    uint8_t start_flag;
    uint8_t controller_addr;
    uint8_t control;            
    uint8_t ipi;                
    uint8_t mess_type;
    uint8_t FCSmsb;
    uint8_t FCSlsb;
    uint8_t end_flag;        
  };
  struct setTimingDataRes_t   
  {
    // 10 bytes
    uint8_t start_flag;
    uint8_t controller_addr;
    uint8_t control;            
    uint8_t ipi;                
    uint8_t mess_type;
    uint8_t pageId;
    uint8_t blockId;
    uint8_t FCSmsb;
    uint8_t FCSlsb;
    uint8_t end_flag;        
  };
  
  struct getBlockMsgRes_PhaseFlags_t   
  {
    // 31 byes (pageId = 2, blockId = 1)
    uint8_t start_flag;
    uint8_t controller_addr;
    uint8_t control;            
    uint8_t ipi;               
    uint8_t mess_type;          
    uint8_t pageId;
    uint8_t blockId;
    uint8_t permitted_phases;
    uint8_t restricted_phases;
    uint8_t vehicular_minimum_recall;
    uint8_t vehicular_maximum_recall;
    uint8_t pedestrian_recall;
    uint8_t bike_recall;
    uint8_t red_detector_lock;
    uint8_t yellow_detector_lock;
    uint8_t fo_maxout_lock;
    uint8_t double_entry;
    uint8_t rest_in_walk;
    uint8_t rest_in_red;
    uint8_t walk2_phases;
    uint8_t maxgreen2_phases;
    uint8_t maxgreen3_phases;
    uint8_t startup_greenPhases;
    uint8_t startup_yellowPhases;
    uint8_t startup_vehCalls;
    uint8_t startup_pedCalls;
    uint8_t startup_yellowOverlaps;
    uint8_t startup_allRed;          // in decisecond
    uint8_t FCSmsb;
    uint8_t FCSlsb;
    uint8_t end_flag;    
  };
  struct getBlockMsgRes_PhaseTiming_t   
  {
    // 31 byes (pageId = 3, blockId = 1 to 8 for 8 phases)
    uint8_t start_flag;
    uint8_t controller_addr;
    uint8_t control;            
    uint8_t ipi;               
    uint8_t mess_type;          
    uint8_t pageId;
    uint8_t blockId;
    /// the folllowing variables are in sec increment
    uint8_t walk1_interval;
    uint8_t walk_clearance;
    uint8_t minimum_green;
    uint8_t type3_detector_disconnect;
    uint8_t maximum_initial;
    uint8_t maximum_extensions[3];
    /// the folllowing variables are in decisecond increment
    uint8_t passage;
    uint8_t minimum_gap;
    uint8_t maximum_gap;
    uint8_t added_initial_per_vehicle;
    uint8_t reduce_gap_by;
    uint8_t reduce_gap_every;
    uint8_t yellow_interval;
    uint8_t red_clearance;
    /// the folllowing variables are in sec increment
    uint8_t walk2_interval;
    uint8_t delay_early_walk_time;
    uint8_t solid_walk_clearance;
    uint8_t bike_green;
    /// the folllowing variables are in decisecond increment    
    uint8_t bike_red_clearance;
    uint8_t FCSmsb;
    uint8_t FCSlsb;
    uint8_t end_flag;    
  };
  struct getBlockMsgRes_CoordPlan_t   
  {
    // 36 byes 
    //  pageId = 4, blockId = 1 to 9, plan 1 to 9
    //  pageId = 5, blockId = 1 to 9, plan 11 to 19
    //  pageId = 6, blockId = 1 to 9, plan 21 to 29    
    uint8_t start_flag;
    uint8_t controller_addr;
    uint8_t control;            
    uint8_t ipi;               
    uint8_t mess_type;          
    uint8_t pageId;
    uint8_t blockId;
    /// the folllowing variables are in sec increment    
    uint8_t cycle_length;
    uint8_t green_factor[8];
    uint8_t cycle_multiplier;
    uint8_t offsets[3];
    uint8_t permissive_periods;
    uint8_t lag_phases;
    uint8_t sync_phases;
    uint8_t hold_phases;
    uint8_t omit_phases;
    uint8_t vehicular_minimum_recall;
    uint8_t vehicular_maximum_recall;
    uint8_t pedestrian_recall;
    uint8_t bike_recall;
    uint8_t fo_flag;        // = 0, green_factor is green factor
                            // = 1, green_factor is force-off points
    uint8_t spare[3];
    uint8_t FCSmsb;
    uint8_t FCSlsb;
    uint8_t end_flag;    
  };
  struct getBlockMsgRes_FreePlan_t   
  {
    // 28 byes (pageId = 4, blockId = 10)
    uint8_t start_flag;
    uint8_t controller_addr;
    uint8_t control;            
    uint8_t ipi;               
    uint8_t mess_type;          
    uint8_t pageId;
    uint8_t blockId;
    uint8_t master_time_sync[6];
    uint8_t master_input_port;
    uint8_t master_output_port;
    uint8_t freeplan_lag_phases;
    uint8_t freeplan_omit_phases;
    uint8_t freeplan_vehicular_minimum_recall;
    uint8_t freeplan_vehicular_maximum_recall;
    uint8_t freeplan_pedestrian_recall;
    uint8_t freeplan_bike_recall;
    uint8_t freeplan_conditional_service;
    uint8_t freeplan_conditional_service_minimum_green;
    uint8_t manualplan_selection;     // 0 disable
                                      // 1 - 9 , select coordination plan 1 to 9
                                      // 11 - 19, select coordination plan 11 to 19
                                      // 21 - 29, select coordination plan 21 to 29
                                      // 254, software flashing operation
                                      // otherwise, free operation
    uint8_t manualplan_offset_index;  // 10 - 12, offset A, B ,C
    uint8_t FCSmsb;
    uint8_t FCSlsb;
    uint8_t end_flag;    
  };
  struct timeofday_plan_t
  {
    uint8_t plan_start_hour;
    uint8_t plan_start_min;
    uint8_t plan_num;
    uint8_t offset_index;            // 10 - 12, offset A, B ,C
  };
  struct getBlockMsgRes_TOD_table_t   
  {
    // 26 byes (pageId = 8, blockId = 1 to 24 for 6 tables, each table has 16 plans)
    uint8_t start_flag;
    uint8_t controller_addr;
    uint8_t control;            
    uint8_t ipi;               
    uint8_t mess_type;          
    uint8_t pageId;
    uint8_t blockId;
    timeofday_plan_t timeofday_plans[4];
    uint8_t FCSmsb;
    uint8_t FCSlsb;
    uint8_t end_flag;    
  };
  struct getBlockMsgRes_weekday_table_t   
  {
    // 17 byes (pageId = 8, blockId = 25)
    uint8_t start_flag;
    uint8_t controller_addr;
    uint8_t control;            
    uint8_t ipi;               
    uint8_t mess_type;          
    uint8_t pageId;
    uint8_t blockId;
    uint8_t weekday_assignments[7];   // starts Monday (0) to Sunday (6)
    uint8_t FCSmsb;
    uint8_t FCSlsb;
    uint8_t end_flag;    
  };  
  
  /* getBlockMsg 
   *  page 2, block 1 - phase flags                             need to poll for phaseflags_mess_t
   *  page 2, block 2 - special flags
   *  page 2, block 3 - pedestrian flags                        need to poll for phaseflags_mess_t
   *  page 2, block 4 - overlap flags   
   *  page 3, block 1 to 8 - phase x timing                     need to poll for phasetiming_mess_t
   *  page 3, block 9 - Overlap timing                          (red revert time is also here)
   *  page 4, block 1 to 9 - local plan x                       need to poll for coordplan_mess_t
   *  page 4, block 10                                          need to poll for freeplan_mess_t & manualplan_mess_t
   *  page 4, block 11 - Special Function Override
   *  page 5, block 1 to 9 - local plan 11 to 19                need to poll for coordplan_mess_t
   *  page 6, block 1 to 9 - local plan 21 to 29                need to poll for coordplan_mess_t
   *  page 7, block 1 to 11 - 44 detectors attributes & conf    need to poll for detectorconf_mess_t
   *  page 7, block 12 - 44 detectors failure times & override  need to poll for system_detector_assignment_mess_t
   *                   - 16 system detectors assignment         
   *  page 7, block 13 - CIC enabled plans, values              need to poll for cicplan_mess_t
   *                   - phase assignment
   *  page 8, block 1 to 24 - 6 TOD tables                      need to poll for TODtable_mess_t      
   *                        - each table with 16 entries
   *  page 8, block 25 - weekday table                          need to poll for weekday_plan_assignment_mess_t
   *  page 9, block 1 to 2 - floating holiday table            
   *                       - 16 entries 
   *  page 9, block 3 to 4 - fixed holiday table               
   *                       - 16 entries 
   *  page 9, block 5 - solar clock
   *  page 9, block 6 to 9 - TOD functions                      need to poll for TODfunction_mess_t
   *  page 10, block 1 to 3 - 3 serial ports
   *  page 10, block 4 to 7 - 16 soft logic
   *  page 10, block 8 to 10 - 3 communications callback numbers
   *  page 10, block 11 - communications network
   *  page 11, block 1 to 5 - RR1 preemption operation conf       need to poll for RRpreemption_mess_t
   *  page 11, block 6 to 10 - RR2 preemption operation conf      need to poll for RRpreemption_mess_t
   *  page 11, block 11 to 14 - EVA/D preemption operation conf   need to poll for EVpreemption_mess_t   
   *  page 12, block 1 to 2 - controller input/output
   *  page 13, block 1 - yellow yield coordination plans
   *  page 13, block 2 to 7 - 18 TSP plans                        need to poll for TSPconf_mess_t
   *  page 13, block 8 - TSP enabled local plans, free plan       need to poll for TSPconf_mess_t
   *  page 13, block 9 - access utility
   *  page 13, block 10 - truck priority conf
   */   
#endif

}
#endif
