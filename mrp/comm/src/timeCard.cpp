//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************
#include <iostream>
#include <fstream> 
#include <sstream>
#include <cstdio>
#include <cstdlib>
#include <iomanip> 

#include "timeCard.h"

using namespace std;

void getPlanIdxMap(std::map<uint8_t,size_t>& plannum2planindex_map,const std::vector<coordplan_mess_t>& coordplans)
{
  for (size_t i = 0; i < coordplans.size(); i++)
  {
    plannum2planindex_map[coordplans[i].plan_num] = i;
  }
}

void pattern2planOffset(uint8_t& plan_num,uint8_t& offset_index,const uint8_t pattern_num)
{
  switch(pattern_num)
  {
  case 0:
    plan_num = 0;
    offset_index = 0;
    break;
  case pattern_flashing:
    // software flash
    plan_num = pattern_flashing;
    offset_index = 0;
    break;
  case pattern_free:
    // free operation
    plan_num = pattern_free;
    offset_index = 0;
    break;
  default:
    // coordination (plan 10 & 20 are skipped)
    // pattern_num 1 to 27 = plan 1 to 9, each plan has 3 offsets
    // pattern_num 31 to 57 = plan 11 to 19
    // pattern_num 61 to 87 = plan 21 to 29    
    plan_num = static_cast<uint8_t>((pattern_num - 1) / 3 + 1);
    offset_index = static_cast<uint8_t>((pattern_num - 1) % 3);
  }
}

uint8_t planOffset2pattern(const uint8_t plan_num,const uint8_t offset_index)
{
  uint8_t pattern_num;
  switch(plan_num)
  {
  case 0:
    pattern_num = 0;
    break;
  case pattern_flashing:
    pattern_num = pattern_flashing;
    break;
  case pattern_free:
    pattern_num = pattern_free;
    break;
  default:
    // coordination
    pattern_num = static_cast<uint8_t>((plan_num - 1)* 3 + offset_index + 1);
  }
  return (pattern_num);
}

ssize_t get_coordplan_index(const uint8_t plan_num,const std::map<uint8_t,size_t> plannum2planindex_map)
{
  map<uint8_t,size_t>::const_iterator ite = plannum2planindex_map.find(plan_num);
  if (ite == plannum2planindex_map.end())
    return (-1);
  else
    return static_cast<ssize_t>(ite->second);
}

bool getPatnIdx(ssize_t& planIdx,const operation_mode_enum_t::controlmode cntrMode,
  const uint8_t plan_num,const std::map<uint8_t,size_t>& plannum2planindex_map)
{
  if (cntrMode == operation_mode_enum_t::FLASHING || cntrMode == operation_mode_enum_t::RUNNINGFREE)
  {
    planIdx = -1;
    return true;
  }
  planIdx = get_coordplan_index(plan_num,plannum2planindex_map);
  if (planIdx < 0)
    return false;
  else
    return true;
}

bool logTimeCard(const char* ptr,const timing_card_t& timingcard)
{
  ofstream OS_CARD(ptr);
  if (!OS_CARD.is_open())
  {
    return false;
  }
  streambuf* strm_buffer = cout.rdbuf();  
  cout.rdbuf(OS_CARD.rdbuf());
  
  cout << "controller_addr " << static_cast<int>(timingcard.controller_addr) << endl << endl;
  
  cout << "Phase_falgs" << endl;
  cout << "permitted_phases " << timingcard.phaseflags.permitted_phases.to_string() << endl;
  cout << "restricted_phases " << timingcard.phaseflags.restricted_phases.to_string() << endl;
  cout << "permitted_ped_phases " << timingcard.phaseflags.permitted_ped_phases.to_string() << endl;
  cout << "minimum_recall_phases " << timingcard.phaseflags.minimum_recall_phases.to_string() << endl;
  cout << "maximum_recall_phases " << timingcard.phaseflags.maximum_recall_phases.to_string() << endl;
  cout << "ped_recall_phases " << timingcard.phaseflags.ped_recall_phases.to_string() << endl;
  cout << "bike_recall_phases " << timingcard.phaseflags.bike_recall_phases.to_string() << endl;
  cout << "redlock_phases " << timingcard.phaseflags.redlock_phases.to_string() << endl;
  cout << "yewlock_phases " << timingcard.phaseflags.yewlock_phases.to_string() << endl;
  cout << "fomaxlock_phases " << timingcard.phaseflags.fomaxlock_phases.to_string() << endl;
  cout << "doubleEntry_phases " << timingcard.phaseflags.doubleEntry_phases.to_string() << endl;
  cout << "restInWalk_phases " << timingcard.phaseflags.restInWalk_phases.to_string() << endl;
  cout << "restInRed_phases " << timingcard.phaseflags.restInRed_phases.to_string() << endl;
  cout << "walk2_phases " << timingcard.phaseflags.walk2_phases.to_string() << endl;
  cout << "maxgreen2_phases " << timingcard.phaseflags.maxgreen2_phases.to_string() << endl;
  cout << "maxgreen3_phases " << timingcard.phaseflags.maxgreen3_phases.to_string() << endl;
  cout << "startup_green_phases " << timingcard.phaseflags.startup_green_phases.to_string() << endl;
  cout << "startup_yellow_phases " << timingcard.phaseflags.startup_yellow_phases.to_string() << endl;
  cout << "startup_vehCalls " << timingcard.phaseflags.startup_vehCalls.to_string() << endl;
  cout << "startup_pedCalls " << timingcard.phaseflags.startup_pedCalls.to_string() << endl;
  cout << "startup_yellowOverlaps " << timingcard.phaseflags.startup_yellowOverlaps.to_string() << endl;
  cout << "startup_allred " << static_cast<int>(timingcard.phaseflags.startup_allred) << endl;
  cout << "red_revert_interval " << static_cast<int>(timingcard.phaseflags.red_revert_interval) << endl <<endl;
  
  cout << "Phase_timing" << endl;
  for (int i = 0; i < 8; i++)
  {
    cout << "phase_num " << static_cast<int>(timingcard.phasetiming[i].phase_num) << endl;
    cout << "walk1_interval " << static_cast<int>(timingcard.phasetiming[i].walk1_interval) << endl;
    cout << "walk_clearance " << static_cast<int>(timingcard.phasetiming[i].walk_clearance) << endl;
    cout << "minimum_green " << static_cast<int>(timingcard.phasetiming[i].minimum_green) << endl;
    cout << "detector_limit " << static_cast<int>(timingcard.phasetiming[i].detector_limit) << endl;
    cout << "maximum_initial " << static_cast<int>(timingcard.phasetiming[i].maximum_initial) << endl;
    cout << "maximum_extensions_I " << static_cast<int>(timingcard.phasetiming[i].maximum_extensions[0]) << endl;
    cout << "maximum_extensions_II " << static_cast<int>(timingcard.phasetiming[i].maximum_extensions[1]) << endl;
    cout << "maximum_extensions_III " << static_cast<int>(timingcard.phasetiming[i].maximum_extensions[2]) << endl;
    cout << "passage " << static_cast<int>(timingcard.phasetiming[i].passage) << endl;
    cout << "maximum_gap " << static_cast<int>(timingcard.phasetiming[i].maximum_gap) << endl;
    cout << "minimum_gap " << static_cast<int>(timingcard.phasetiming[i].minimum_gap) << endl;
    cout << "added_initial_per_vehicle " << static_cast<int>(timingcard.phasetiming[i].added_initial_per_vehicle) << endl;
    cout << "reduce_gap_by " << static_cast<int>(timingcard.phasetiming[i].reduce_gap_by) << endl;
    cout << "reduce_gap_every " << static_cast<int>(timingcard.phasetiming[i].reduce_gap_every) << endl;
    cout << "yellow_interval " << static_cast<int>(timingcard.phasetiming[i].yellow_interval) << endl;
    cout << "red_clearance " << static_cast<int>(timingcard.phasetiming[i].red_clearance) << endl;
    cout << "walk2_interval " << static_cast<int>(timingcard.phasetiming[i].walk2_interval) << endl;
    cout << "delay_early_walk_time " << static_cast<int>(timingcard.phasetiming[i].delay_early_walk_time) << endl;
    cout << "solid_walk_clearance " << static_cast<int>(timingcard.phasetiming[i].solid_walk_clearance) << endl;
    cout << "bike_green " << static_cast<int>(timingcard.phasetiming[i].bike_green) << endl;
    cout << "bike_red_clearance " << static_cast<int>(timingcard.phasetiming[i].bike_red_clearance) << endl << endl;
  }
  
  cout << "Free_plan" << endl;
  cout << "lag_phases " << timingcard.freeplan.lag_phases.to_string() << endl;
  cout << "omit_phases " << timingcard.freeplan.omit_phases.to_string() << endl;
  cout << "minimum_recall_phases " << timingcard.freeplan.minimum_recall_phases.to_string() << endl;
  cout << "maximum_recall_phases " << timingcard.freeplan.maximum_recall_phases.to_string() << endl;
  cout << "ped_recall_phases " << timingcard.freeplan.ped_recall_phases.to_string() << endl;
  cout << "bike_recall_phases " << timingcard.freeplan.bike_recall_phases.to_string() << endl;
  cout << "conditional_service_phases " << timingcard.freeplan.conditional_service_phases.to_string() << endl;
  cout << "conditional_service_minimum_green " << static_cast<int>(timingcard.freeplan.conditional_service_minimum_green) << endl;
  cout << "permitted_phases " << timingcard.freeplan.permitted_phases.to_string() << endl;
  cout << "permitted_ped_phases " << timingcard.freeplan.permitted_ped_phases.to_string() << endl;    
  cout << "leadlag_phases";
  for (int barrier = 0; barrier < 2; barrier++)
  {
    for (int ring = 0; ring < 2; ring++)
    {
      cout << " " << static_cast<int>(timingcard.freeplan.leadlag_phases[barrier][ring][0]);
      cout << " " << static_cast<int>(timingcard.freeplan.leadlag_phases[barrier][ring][1]);
    }
  }
  cout << endl;
  cout << "TSP_conf " << std::boolalpha << timingcard.freeplan.isTSPenabled;
  cout << " " << timingcard.freeplan.TSP_hold_phases.to_string();
  cout << " " << static_cast<int>(timingcard.freeplan.TSP_max_green_hold_time) << endl << endl;
  
  cout << "Manual_plan " << std::boolalpha << timingcard.manualplan.planOn;
  cout << " " << static_cast<int>(timingcard.manualplan.plan_num);
  cout << " " << static_cast<int>(timingcard.manualplan.offset_index) << endl << endl;
  
  cout << "CIC_plan " << timingcard.cicplans.enabled_plans.to_string();
  cout << " " << static_cast<int>(timingcard.cicplans.smoothing_volume);
  cout << " " << static_cast<int>(timingcard.cicplans.smoothing_occupancy);
  cout << " " << static_cast<int>(timingcard.cicplans.smoothing_demand);
  cout << " " << static_cast<int>(timingcard.cicplans.multiplier_volume);
  cout << " " << static_cast<int>(timingcard.cicplans.multiplier_occupancy);
  cout << " " << static_cast<int>(timingcard.cicplans.exponent_volume);
  cout << " " << static_cast<int>(timingcard.cicplans.exponent_occupancy) << endl;
  for (int i=0; i<16;i++)
  {
    cout << static_cast<int>(timingcard.cicplans.phase_assignment[i]) << " ";
  }
  cout << endl << endl;
  
  cout << "detector_conf " << timingcard.detectorconf.size() << endl;
  for (size_t i = 0; i < timingcard.detectorconf.size(); i++)
  {
    cout << static_cast<int>(timingcard.detectorconf[i].type) << " ";
    cout << timingcard.detectorconf[i].phaseAssignment.to_string() << " ";
    cout << static_cast<int>(timingcard.detectorconf[i].lock) << " ";
    cout << static_cast<int>(timingcard.detectorconf[i].delayTime) << " ";
    cout << static_cast<int>(timingcard.detectorconf[i].extendTime) << " ";
    cout << static_cast<int>(timingcard.detectorconf[i].recallTime) << " ";
    cout << static_cast<int>(timingcard.detectorconf[i].inputPort) << endl;
  }  
  cout << endl;

  cout << "system_detectors";
  cout << " " << static_cast<int>(timingcard.system_detector_assignment.maxOnTime);
  cout << " " << static_cast<int>(timingcard.system_detector_assignment.failResetTime);
  cout << " " << timingcard.system_detector_assignment.failOverride.to_string() << endl;
  for (int i = 0; i < 16; i++)
  {
    cout << static_cast<int>(timingcard.system_detector_assignment.detectorInput[i]) << " ";
  }
  cout << endl << endl;
  
  cout << "TODtables " << timingcard.TODtables.size() << endl;
  for (size_t i = 0; i < timingcard.TODtables.size(); i++)
  {
    cout << static_cast<int>(timingcard.TODtables[i].table_num) << " ";
    cout << static_cast<int>(timingcard.TODtables[i].start_hour) << " "; 
    cout << static_cast<int>(timingcard.TODtables[i].start_min) << " ";
    cout << static_cast<int>(timingcard.TODtables[i].plan_num) << " ";
    cout << static_cast<int>(timingcard.TODtables[i].offset_index) << endl;
  }
  cout << endl;

  cout << "weekday_tables";
  for (int i = 0; i < 7; i++)
  {
    cout << " " << static_cast<int>(timingcard.weekday_plan_assignment.dayofweek[i]);
  }
  cout << endl << endl;
  
  cout << "TODfunctions " << timingcard.TODfunctions.size() << endl;
  for (size_t i = 0; i < timingcard.TODfunctions.size(); i++)
  {
    cout << static_cast<int>(timingcard.TODfunctions[i].start_hour) << " ";
    cout << static_cast<int>(timingcard.TODfunctions[i].start_min) << " ";
    cout << static_cast<int>(timingcard.TODfunctions[i].end_hour) << " ";
    cout << static_cast<int>(timingcard.TODfunctions[i].end_min) << " ";
    cout << timingcard.TODfunctions[i].dayofweek.to_string() << " ";
    cout << static_cast<int>(timingcard.TODfunctions[i].action_code) << " ";
    cout << timingcard.TODfunctions[i].affect_phases.to_string() << endl;
  }
  cout << endl;
  
  cout << "EVpreemption" << endl;
  for (int i = 0; i < 4; i++)
  {
    cout << static_cast<int>(timingcard.EVpreemption[i].delay_time) << " ";
    cout << static_cast<int>(timingcard.EVpreemption[i].green_hold_time) << " ";
    cout << static_cast<int>(timingcard.EVpreemption[i].maximum_clearance_time) << " ";
    cout << timingcard.EVpreemption[i].clearance_phase_green.to_string() << " ";
    cout << timingcard.EVpreemption[i].clearance_overlap_green.to_string() << " ";
    cout << static_cast<int>(timingcard.EVpreemption[i].input_port) << " ";
    cout << static_cast<int>(timingcard.EVpreemption[i].latching_flag) << " ";
    cout << static_cast<int>(timingcard.EVpreemption[i].phase_termination_flag) << endl;
  }
  cout << endl;
  
  cout << "RRpreemption" << endl;
  for (int i = 0; i < 2; i++)
  {
    cout << static_cast<int>(timingcard.RRpreemption[i].delay_time) << " ";
    cout << static_cast<int>(timingcard.RRpreemption[i].minimum_green) << " ";
    cout << static_cast<int>(timingcard.RRpreemption[i].ped_clear_time) << " ";
    cout << static_cast<int>(timingcard.RRpreemption[i].exit_time) << " ";
    cout << timingcard.RRpreemption[i].exit_phases_green.to_string() << " ";
    cout << timingcard.RRpreemption[i].exit_overlaps_green.to_string() << " ";
    cout << timingcard.RRpreemption[i].exit_veh_call.to_string() << " ";
    cout << timingcard.RRpreemption[i].exit_ped_call.to_string() << " ";
    cout << static_cast<int>(timingcard.RRpreemption[i].input_port) << " ";
    cout << static_cast<int>(timingcard.RRpreemption[i].gate_port) << " ";
    cout << static_cast<int>(timingcard.RRpreemption[i].latching_flag) << " ";
    cout << static_cast<int>(timingcard.RRpreemption[i].power_up) << endl;
    for (int j = 0; j < 4; j++)
    {
      cout << static_cast<int>(timingcard.RRpreemption[i].RRpreemption_steps[j].step_time) << " ";
      cout << timingcard.RRpreemption[i].RRpreemption_steps[j].ped_walk_phases.to_string() << " ";    
      cout << timingcard.RRpreemption[i].RRpreemption_steps[j].ped_clear_phases.to_string() << " ";
      cout << timingcard.RRpreemption[i].RRpreemption_steps[j].ped_red_phases.to_string() << " ";
      cout << timingcard.RRpreemption[i].RRpreemption_steps[j].green_hold_phases.to_string() << " ";    
      cout << timingcard.RRpreemption[i].RRpreemption_steps[j].yew_flashing_phases.to_string() << " ";
      cout << timingcard.RRpreemption[i].RRpreemption_steps[j].red_flashing_phases.to_string() << " ";
      cout << timingcard.RRpreemption[i].RRpreemption_steps[j].green_hold_overlaps.to_string() << " ";    
      cout << timingcard.RRpreemption[i].RRpreemption_steps[j].yew_flashing_overlaps.to_string() << " ";
      cout << timingcard.RRpreemption[i].RRpreemption_steps[j].red_flashing_overlaps.to_string() << endl;
    }
  }
  cout << endl;
  
  cout << "TSPconf " << timingcard.TSPconf.enable_coordination_plans.to_string() << endl;
  for (int i = 0; i < 18; i++)
  {
    cout << static_cast<int>(timingcard.TSPconf.TSPplan[i].max_early_green) << " ";
    cout << static_cast<int>(timingcard.TSPconf.TSPplan[i].max_green_extension) << " ";
    cout << static_cast<int>(timingcard.TSPconf.TSPplan[i].inhibit_cycles);
    for (int j = 0; j < 8; j++)
    {
      cout << " " << static_cast<int>(timingcard.TSPconf.TSPplan[i].green_factor[j]);
    }
    cout << endl;
  }
  cout << endl;
  
  cout << "Coordination_plans " << timingcard.coordplans.size() << endl;
  for (size_t i = 0; i < timingcard.coordplans.size(); i++)
  {
    cout << "plan_num " << static_cast<int>(timingcard.coordplans[i].plan_num) << endl;
    cout << "cycle_length " << static_cast<int>(timingcard.coordplans[i].cycle_length) << endl;
    cout << "green_factor";
    for (int j = 0; j < 8; j++)
    {
      cout << " " << static_cast<int>(timingcard.coordplans[i].green_factor[j]);
    }
    cout << endl;
    cout << "cycle_multiplier " << static_cast<int>(timingcard.coordplans[i].cycle_multiplier) << endl;
    cout << "offsets";
    for (int j = 0; j < 3; j++)
    {
      cout << " " << static_cast<int>(timingcard.coordplans[i].offsets[j]);
    }
    cout << endl;
    cout << "laggapout_phase " << static_cast<int>(timingcard.coordplans[i].laggapout_phase) << endl;
    cout << "lag_phases " << timingcard.coordplans[i].lag_phases.to_string() << endl;
    cout << "sync_phases " << timingcard.coordplans[i].sync_phases.to_string() << endl;
    cout << "hold_phases " << timingcard.coordplans[i].hold_phases.to_string() << endl;
    cout << "omit_phases " << timingcard.coordplans[i].omit_phases.to_string() << endl;
    cout << "minimum_recall_phases " << timingcard.coordplans[i].minimum_recall_phases.to_string() << endl;
    cout << "maximum_recall_phases " << timingcard.coordplans[i].maximum_recall_phases.to_string() << endl;
    cout << "ped_recall_phases " << timingcard.coordplans[i].ped_recall_phases.to_string() << endl;
    cout << "bike_recall_phases " << timingcard.coordplans[i].bike_recall_phases.to_string() << endl;
    cout << "force_off_flag " << static_cast<int>(timingcard.coordplans[i].force_off_flag) << endl;
    cout << "permitted_phases " << timingcard.coordplans[i].permitted_phases.to_string() << endl;
    cout << "permitted_ped_phases " << timingcard.coordplans[i].permitted_ped_phases.to_string() << endl;    
    cout << "coordinated_phases " << static_cast<int>(timingcard.coordplans[i].coordinated_phases[0]);
    cout << " " << static_cast<int>(timingcard.coordplans[i].coordinated_phases[1]) << endl;
    cout << "leadLagMode " << static_cast<int>(timingcard.coordplans[i].leadLagMode) << endl;
    cout << "sync_ring " << static_cast<int>(timingcard.coordplans[i].sync_ring) << endl;
    cout << "sync_barrier " << static_cast<int>(timingcard.coordplans[i].sync_barrier) << endl;
    cout << "leadlag_phases";
    for (int barrier = 0; barrier < 2; barrier++)
    {
      for (int ring = 0; ring < 2; ring++)
      {
        cout << " " << static_cast<int>(timingcard.coordplans[i].leadlag_phases[barrier][ring][0]);
        cout << " " << static_cast<int>(timingcard.coordplans[i].leadlag_phases[barrier][ring][1]);
      }
    }
    cout << endl;
    cout << "force_off";
    for (int j = 0; j < 8; j++)
    {
      cout << " " << static_cast<int>(timingcard.coordplans[i].force_off[j]);
    }
    cout << endl;
    cout << "permissive";
    for (int j = 0; j < 8; j++)
    {
      cout << " " << static_cast<int>(timingcard.coordplans[i].permissive[j]);
    }    
    cout << endl;
    cout << "ped_permissive";
    for (int j = 0; j < 8; j++)
    {
      cout << " " << static_cast<int>(timingcard.coordplans[i].ped_permissive[j]);
    }    
    cout << endl;    
    cout << "noncoordBarrierGreenOnset " << static_cast<int>(timingcard.coordplans[i].noncoordBarrierGreenOnset) << endl;
    cout << "coordBarrierGreenOnset " << static_cast<int>(timingcard.coordplans[i].coordBarrierGreenOnset) << endl;
    cout << "coordPhaseGreenOnset " << static_cast<int>(timingcard.coordplans[i].coordPhaseGreenOnset[0]);
    cout << " " << static_cast<int>(timingcard.coordplans[i].coordPhaseGreenOnset[1]) << endl;
    cout << "coordPhaseGreenEnd " << std::boolalpha << timingcard.coordplans[i].coordLagphaseGapout;
    cout << " " << static_cast<int>(timingcard.coordplans[i].coordPhaseGreenEnd[0]);
    cout << " " << static_cast<int>(timingcard.coordplans[i].coordPhaseGreenEnd[1]) << endl;
    cout << "isTSPenabled " << std::boolalpha << timingcard.coordplans[i].isTSPenabled;
    cout << " " << static_cast<int>(timingcard.coordplans[i].max_early_green);
    cout << " " << static_cast<int>(timingcard.coordplans[i].max_green_extension);
    cout << " " << static_cast<int>(timingcard.coordplans[i].inhibit_cycles) << endl;
    cout << "TSP_force_off";
    for (int j = 0; j < 8; j++)
    {
      cout << " " << static_cast<int>(timingcard.coordplans[i].TSP_force_off[j]);
    }
    cout << endl << endl;
  }
  cout.rdbuf(strm_buffer);  
  OS_CARD.close();
  return true;
} 

bool readTimeCard(const char* ptr,timing_card_t& timingcard)
{
  ifstream IS_CARD(ptr);
  if (!IS_CARD.is_open())
  {
    cerr << "Failed open timing card: " << ptr;
    return false;
  }
  istringstream iss;
  string line,s;
  
  while (std::getline(IS_CARD,line))
  {
    if (line.empty())
      continue;
    if (line.find("controller_addr") == 0)
    {
      iss.str(line);
      iss >> std::skipws >> s >> s;
      iss.clear();
      timingcard.controller_addr = static_cast<uint8_t>(atoi(s.c_str()));
    }
    else if (line.find("Phase_falgs") == 0)
    {
      /// phaseflags
      //  permitted_phases
      std::getline(IS_CARD,line);
      iss.str(line);
      iss >> std::skipws >> s >> s;
      iss.clear();
      timingcard.phaseflags.permitted_phases = bitset<8>(s);
      //  restricted_phases
      std::getline(IS_CARD,line);
      iss.str(line);
      iss >> std::skipws >> s >> s;
      iss.clear();
      timingcard.phaseflags.restricted_phases = bitset<8>(s);
      //  permitted_ped_phases
      std::getline(IS_CARD,line);
      iss.str(line);
      iss >> std::skipws >> s >> s;
      iss.clear();
      timingcard.phaseflags.permitted_ped_phases = bitset<8>(s);
      //  minimum_recall_phases
      std::getline(IS_CARD,line);
      iss.str(line);
      iss >> std::skipws >> s >> s;
      iss.clear();
      timingcard.phaseflags.minimum_recall_phases = bitset<8>(s);
      //  maximum_recall_phases
      std::getline(IS_CARD,line);
      iss.str(line);
      iss >> std::skipws >> s >> s;
      iss.clear();
      timingcard.phaseflags.maximum_recall_phases = bitset<8>(s);
      //  ped_recall_phases
      std::getline(IS_CARD,line);
      iss.str(line);
      iss >> std::skipws >> s >> s;
      iss.clear();
      timingcard.phaseflags.ped_recall_phases = bitset<8>(s);
      //  bike_recall_phases
      std::getline(IS_CARD,line);
      iss.str(line);
      iss >> std::skipws >> s >> s;
      iss.clear();
      timingcard.phaseflags.bike_recall_phases = bitset<8>(s);
      //  redlock_phases
      std::getline(IS_CARD,line);
      iss.str(line);
      iss >> std::skipws >> s >> s;
      iss.clear();
      timingcard.phaseflags.redlock_phases = bitset<8>(s);
      //  yewlock_phases
      std::getline(IS_CARD,line);
      iss.str(line);
      iss >> std::skipws >> s >> s;
      iss.clear();
      timingcard.phaseflags.yewlock_phases = bitset<8>(s);
      //  fomaxlock_phases
      std::getline(IS_CARD,line);
      iss.str(line);
      iss >> std::skipws >> s >> s;
      iss.clear();
      timingcard.phaseflags.fomaxlock_phases = bitset<8>(s);
      //  doubleEntry_phases
      std::getline(IS_CARD,line);
      iss.str(line);
      iss >> std::skipws >> s >> s;
      iss.clear();
      timingcard.phaseflags.doubleEntry_phases = bitset<8>(s);
      //  restInWalk_phases
      std::getline(IS_CARD,line);
      iss.str(line);
      iss >> std::skipws >> s >> s;
      iss.clear();
      timingcard.phaseflags.restInWalk_phases = bitset<8>(s);
      //  restInRed_phases
      std::getline(IS_CARD,line);
      iss.str(line);
      iss >> std::skipws >> s >> s;
      iss.clear();
      timingcard.phaseflags.restInRed_phases = bitset<8>(s);
      //  walk2_phases
      std::getline(IS_CARD,line);
      iss.str(line);
      iss >> std::skipws >> s >> s;
      iss.clear();
      timingcard.phaseflags.walk2_phases = bitset<8>(s);
      //  maxgreen2_phases
      std::getline(IS_CARD,line);
      iss.str(line);
      iss >> std::skipws >> s >> s;
      iss.clear();
      timingcard.phaseflags.maxgreen2_phases = bitset<8>(s);
      //  maxgreen3_phases
      std::getline(IS_CARD,line);
      iss.str(line);
      iss >> std::skipws >> s >> s;
      iss.clear();
      timingcard.phaseflags.maxgreen3_phases = bitset<8>(s);
      //  startup_green_phases
      std::getline(IS_CARD,line);
      iss.str(line);
      iss >> std::skipws >> s >> s;
      iss.clear();
      timingcard.phaseflags.startup_green_phases = bitset<8>(s);
      //  startup_yellow_phases
      std::getline(IS_CARD,line);
      iss.str(line);
      iss >> std::skipws >> s >> s;
      iss.clear();
      timingcard.phaseflags.startup_yellow_phases = bitset<8>(s);
      //  startup_vehCalls
      std::getline(IS_CARD,line);
      iss.str(line);
      iss >> std::skipws >> s >> s;
      iss.clear();
      timingcard.phaseflags.startup_vehCalls = bitset<8>(s);
      //  startup_pedCalls
      std::getline(IS_CARD,line);
      iss.str(line);
      iss >> std::skipws >> s >> s;
      iss.clear();
      timingcard.phaseflags.startup_pedCalls = bitset<8>(s);
      //  startup_yellowOverlaps
      std::getline(IS_CARD,line);
      iss.str(line);
      iss >> std::skipws >> s >> s;
      iss.clear();
      timingcard.phaseflags.startup_yellowOverlaps = bitset<8>(s);
      //  startup_allred
      std::getline(IS_CARD,line);
      iss.str(line);
      iss >> std::skipws >> s >> s;
      iss.clear();
      timingcard.phaseflags.startup_allred = static_cast<uint8_t>(atoi(s.c_str()));
      //  red_revert_interval
      std::getline(IS_CARD,line);
      iss.str(line);
      iss >> std::skipws >> s >> s;
      iss.clear();
      timingcard.phaseflags.red_revert_interval = static_cast<uint8_t>(atoi(s.c_str()));
    }
    else if (line.find("Phase_timing") == 0)
    {
      for (int phase_idx = 0; phase_idx < 8; phase_idx++)
      {
        /// phase_num
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> s;
        iss.clear();
        timingcard.phasetiming[phase_idx].phase_num = static_cast<uint8_t>(atoi(s.c_str()));      
        /// walk1_interval
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> s;
        iss.clear();
        timingcard.phasetiming[phase_idx].walk1_interval = static_cast<uint8_t>(atoi(s.c_str()));
        /// walk_clearance
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> s;
        iss.clear();
        timingcard.phasetiming[phase_idx].walk_clearance = static_cast<uint8_t>(atoi(s.c_str()));      
        /// minimum_green
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> s;
        iss.clear();
        timingcard.phasetiming[phase_idx].minimum_green = static_cast<uint8_t>(atoi(s.c_str()));
        /// detector_limit
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> s;
        iss.clear();
        timingcard.phasetiming[phase_idx].detector_limit = static_cast<uint8_t>(atoi(s.c_str()));
        /// maximum_initial
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> s;
        iss.clear();
        timingcard.phasetiming[phase_idx].maximum_initial = static_cast<uint8_t>(atoi(s.c_str()));
        /// maximum_extensions_I
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> s;
        iss.clear();
        timingcard.phasetiming[phase_idx].maximum_extensions[0] = static_cast<uint8_t>(atoi(s.c_str()));
        /// maximum_extensions_II
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> s;
        iss.clear();
        timingcard.phasetiming[phase_idx].maximum_extensions[1] = static_cast<uint8_t>(atoi(s.c_str()));
        /// maximum_extensions_III
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> s;
        iss.clear();
        timingcard.phasetiming[phase_idx].maximum_extensions[2] = static_cast<uint8_t>(atoi(s.c_str()));
        /// passage
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> s;
        iss.clear();
        timingcard.phasetiming[phase_idx].passage = static_cast<uint8_t>(atoi(s.c_str()));
        /// maximum_gap
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> s;
        iss.clear();
        timingcard.phasetiming[phase_idx].maximum_gap = static_cast<uint8_t>(atoi(s.c_str()));
        /// minimum_gap
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> s;
        iss.clear();
        timingcard.phasetiming[phase_idx].minimum_gap = static_cast<uint8_t>(atoi(s.c_str()));
        /// added_initial_per_vehicle
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> s;
        iss.clear();
        timingcard.phasetiming[phase_idx].added_initial_per_vehicle = static_cast<uint8_t>(atoi(s.c_str()));
        /// reduce_gap_by
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> s;
        iss.clear();
        timingcard.phasetiming[phase_idx].reduce_gap_by = static_cast<uint8_t>(atoi(s.c_str()));
        /// reduce_gap_every
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> s;
        iss.clear();
        timingcard.phasetiming[phase_idx].reduce_gap_every = static_cast<uint8_t>(atoi(s.c_str()));
        /// yellow_interval
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> s;
        iss.clear();
        timingcard.phasetiming[phase_idx].yellow_interval = static_cast<uint8_t>(atoi(s.c_str()));
        /// red_clearance
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> s;
        iss.clear();
        timingcard.phasetiming[phase_idx].red_clearance = static_cast<uint8_t>(atoi(s.c_str()));
        /// walk2_interval
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> s;
        iss.clear();
        timingcard.phasetiming[phase_idx].walk2_interval = static_cast<uint8_t>(atoi(s.c_str()));
        /// delay_early_walk_time
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> s;
        iss.clear();
        timingcard.phasetiming[phase_idx].delay_early_walk_time = static_cast<uint8_t>(atoi(s.c_str()));
        /// solid_walk_clearance
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> s;
        iss.clear();
        timingcard.phasetiming[phase_idx].solid_walk_clearance = static_cast<uint8_t>(atoi(s.c_str()));
        /// bike_green
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> s;
        iss.clear();
        timingcard.phasetiming[phase_idx].bike_green = static_cast<uint8_t>(atoi(s.c_str()));
        /// bike_red_clearance
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> s;
        iss.clear();
        timingcard.phasetiming[phase_idx].bike_red_clearance = static_cast<uint8_t>(atoi(s.c_str()));
        /// empty line
        std::getline(IS_CARD,line);        
      }
    }
    else if (line.find("Free_plan") == 0)
    {
      string ds[8];      
      //  lag_phases
      std::getline(IS_CARD,line);
      iss.str(line);
      iss >> std::skipws >> s >> s;
      iss.clear();
      timingcard.freeplan.lag_phases = bitset<8>(s);
      //  omit_phases
      std::getline(IS_CARD,line);
      iss.str(line);
      iss >> std::skipws >> s >> s;
      iss.clear();
      timingcard.freeplan.omit_phases = bitset<8>(s);
      //  minimum_recall_phases
      std::getline(IS_CARD,line);
      iss.str(line);
      iss >> std::skipws >> s >> s;
      iss.clear();
      timingcard.freeplan.minimum_recall_phases = bitset<8>(s);
      //  maximum_recall_phases
      std::getline(IS_CARD,line);
      iss.str(line);
      iss >> std::skipws >> s >> s;
      iss.clear();
      timingcard.freeplan.maximum_recall_phases = bitset<8>(s);
      //  ped_recall_phases
      std::getline(IS_CARD,line);
      iss.str(line);
      iss >> std::skipws >> s >> s;
      iss.clear();
      timingcard.freeplan.ped_recall_phases = bitset<8>(s);
      //  bike_recall_phases
      std::getline(IS_CARD,line);
      iss.str(line);
      iss >> std::skipws >> s >> s;
      iss.clear();
      timingcard.freeplan.bike_recall_phases = bitset<8>(s);
      //  conditional_service_phases
      std::getline(IS_CARD,line);
      iss.str(line);
      iss >> std::skipws >> s >> s;
      iss.clear();
      timingcard.freeplan.conditional_service_phases = bitset<8>(s);
      //  conditional_service_minimum_green
      std::getline(IS_CARD,line);
      iss.str(line);
      iss >> std::skipws >> s >> s;
      iss.clear();
      timingcard.freeplan.conditional_service_minimum_green = static_cast<uint8_t>(atoi(s.c_str()));
      //  permitted_phases
      std::getline(IS_CARD,line);
      iss.str(line);
      iss >> std::skipws >> s >> s;
      iss.clear();
      timingcard.freeplan.permitted_phases = bitset<8>(s);
      //  permitted_ped_phases
      std::getline(IS_CARD,line);
      iss.str(line);
      iss >> std::skipws >> s >> s;
      iss.clear();
      timingcard.freeplan.permitted_ped_phases = bitset<8>(s);      
      //  leadlag_phases
      std::getline(IS_CARD,line);
      iss.str(line);
      iss >> std::skipws >> s >> ds[0] >> ds[1] >> ds[2] >> ds[3] >> ds[4] >> ds[5] >> ds[6] >> ds[7];
      iss.clear();
      int count = 0;
      for (int barrier = 0; barrier < 2; barrier++)
      {
        for (int ring = 0; ring < 2; ring++)
        {
          for (int seq = 0; seq < 2; seq++)
          {
            timingcard.freeplan.leadlag_phases[barrier][ring][seq] = static_cast<uint8_t>(atoi(ds[count].c_str()));
            count++;
          }
        }
      }
      // TSP_conf
      std::getline(IS_CARD,line);
      iss.str(line);
      iss >> std::skipws >> s >> s >> ds[0] >> ds[1];
      iss.clear();
      if (s.compare("true") == 0)
        timingcard.freeplan.isTSPenabled = true;
      else
        timingcard.freeplan.isTSPenabled = false;
      timingcard.freeplan.TSP_hold_phases = bitset<8>(ds[0]);
      timingcard.freeplan.TSP_max_green_hold_time = static_cast<uint8_t>(atoi(ds[1].c_str()));
    }
    else if (line.find("Manual_plan") == 0)
    {
      string ds[3];
      iss.str(line);
      iss >> std::skipws >> s >> s >> ds[0] >> ds[1];
      iss.clear();
      if (s.compare("true") == 0)
        timingcard.manualplan.planOn = true;
      else
        timingcard.manualplan.planOn = false;
      timingcard.manualplan.plan_num = static_cast<uint8_t>(atoi(ds[0].c_str()));
      timingcard.manualplan.offset_index = static_cast<uint8_t>(atoi(ds[1].c_str()));
    }
    else if (line.find("CIC_plan") == 0)
    {
      string ds[16];
      iss.str(line);
      iss >> std::skipws >> s >> ds[0] >> ds[1] >> ds[2] >> ds[3] >> ds[4] >> ds[5] >> ds[6] >> ds[7];
      iss.clear();
      timingcard.cicplans.enabled_plans = bitset<9>(ds[0]);
      timingcard.cicplans.smoothing_volume = static_cast<uint8_t>(atoi(ds[1].c_str()));
      timingcard.cicplans.smoothing_occupancy = static_cast<uint8_t>(atoi(ds[2].c_str()));
      timingcard.cicplans.smoothing_demand = static_cast<uint8_t>(atoi(ds[3].c_str()));
      timingcard.cicplans.multiplier_volume = static_cast<uint8_t>(atoi(ds[4].c_str()));
      timingcard.cicplans.multiplier_occupancy = static_cast<uint8_t>(atoi(ds[5].c_str()));
      timingcard.cicplans.exponent_volume = static_cast<uint8_t>(atoi(ds[6].c_str()));
      timingcard.cicplans.exponent_occupancy = static_cast<uint8_t>(atoi(ds[7].c_str()));
      std::getline(IS_CARD,line);
      iss.str(line);
      iss >> std::skipws >> ds[0] >> ds[1] >> ds[2] >> ds[3] >> ds[4] >> ds[5] >> ds[6] >> ds[7] >> ds[8] >> ds[9] >> ds[10] >> ds[11] >> ds[12] >> ds[13] >> ds[14] >> ds[15];
      iss.clear();
      for (int i = 0; i< 16; i++)
      {
        timingcard.cicplans.phase_assignment[i] = static_cast<uint8_t>(atoi(ds[i].c_str()));
      }
    }
    else if (line.find("detector_conf") == 0)
    {
      iss.str(line);
      iss >> std::skipws >> s >> s;
      iss.clear();
      int detnums = atoi(s.c_str());
      timingcard.detectorconf.resize(detnums);
      string ds[7];
      for (int i = 0; i < detnums; i++)
      {
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> ds[0] >> ds[1] >> ds[2] >> ds[3] >> ds[4] >> ds[5] >> ds[6];
        iss.clear();
        timingcard.detectorconf[i].type = static_cast<uint8_t>(atoi(ds[0].c_str()));
        timingcard.detectorconf[i].phaseAssignment = bitset<8>(ds[1]);
        timingcard.detectorconf[i].lock = static_cast<uint8_t>(atoi(ds[2].c_str()));
        timingcard.detectorconf[i].delayTime = static_cast<uint8_t>(atoi(ds[3].c_str()));
        timingcard.detectorconf[i].extendTime = static_cast<uint8_t>(atoi(ds[4].c_str()));
        timingcard.detectorconf[i].recallTime = static_cast<uint8_t>(atoi(ds[5].c_str()));
        timingcard.detectorconf[i].inputPort = static_cast<uint8_t>(atoi(ds[6].c_str()));
      }
    }
    else if (line.find("system_detectors") == 0)
    {
      string ds[16];
      iss.str(line); 
      iss >> std::skipws >> s >> ds[0] >> ds[1] >> ds[2];
      iss.clear();
      timingcard.system_detector_assignment.maxOnTime = static_cast<uint8_t>(atoi(ds[0].c_str()));
      timingcard.system_detector_assignment.failResetTime = static_cast<uint8_t>(atoi(ds[1].c_str()));
      timingcard.system_detector_assignment.failOverride = bitset<44>(ds[2]);
      std::getline(IS_CARD,line);
      iss.str(line); 
      iss >> std::skipws >> ds[0] >> ds[1] >> ds[2] >> ds[3] >> ds[4] >> ds[5] >> ds[6] >> ds[7] >> ds[8] >> ds[9] >> ds[10] >> ds[11] >> ds[12] >> ds[13] >> ds[14] >> ds[15];
      iss.clear();
      for (int i = 0; i < 16; i++)
      {
        timingcard.system_detector_assignment.detectorInput[i] = static_cast<uint8_t>(atoi(ds[i].c_str()));
      }
    }
    else if (line.find("TODtables") == 0)
    {
      iss.str(line);
      iss >> std::skipws >> s >> s;
      iss.clear();
      int tablerows = atoi(s.c_str());
      timingcard.TODtables.resize(tablerows);
      string ds[5];      
      for (int i = 0; i < tablerows; i++)
      {
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> ds[0] >> ds[1] >> ds[2] >> ds[3] >> ds[4];
        iss.clear();
        timingcard.TODtables[i].table_num = static_cast<uint8_t>(atoi(ds[0].c_str()));
        timingcard.TODtables[i].start_hour = static_cast<uint8_t>(atoi(ds[1].c_str()));
        timingcard.TODtables[i].start_min = static_cast<uint8_t>(atoi(ds[2].c_str()));
        timingcard.TODtables[i].plan_num = static_cast<uint8_t>(atoi(ds[3].c_str()));
        timingcard.TODtables[i].offset_index = static_cast<uint8_t>(atoi(ds[4].c_str()));
      }
    }
    else if (line.find("weekday_tables") == 0)
    {
      string ds[7];
      iss.str(line);
      iss >> std::skipws >> s >> ds[0] >> ds[1] >> ds[2] >> ds[3] >> ds[4] >> ds[5] >> ds[6];
      iss.clear();
      for (int i = 0; i < 7; i++)
      {
        timingcard.weekday_plan_assignment.dayofweek[i] = static_cast<uint8_t>(atoi(ds[i].c_str()));
      }
    }
    else if (line.find("TODfunctions") == 0) 
    {
      iss.str(line);
      iss >> std::skipws >> s >> s;
      iss.clear();
      int functionrows = atoi(s.c_str());
      timingcard.TODfunctions.resize(functionrows);
      string ds[7];      
      for (int i = 0; i < functionrows; i++)
      {
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> ds[0] >> ds[1] >> ds[2] >> ds[3] >> ds[4] >> ds[5] >> ds[6];
        iss.clear();
        timingcard.TODfunctions[i].start_hour = static_cast<uint8_t>(atoi(ds[0].c_str()));
        timingcard.TODfunctions[i].start_min = static_cast<uint8_t>(atoi(ds[1].c_str()));
        timingcard.TODfunctions[i].end_hour = static_cast<uint8_t>(atoi(ds[2].c_str()));
        timingcard.TODfunctions[i].end_min = static_cast<uint8_t>(atoi(ds[3].c_str()));
        timingcard.TODfunctions[i].dayofweek = bitset<8>(ds[4]);
        timingcard.TODfunctions[i].action_code = static_cast<uint8_t>(atoi(ds[5].c_str()));
        timingcard.TODfunctions[i].affect_phases = bitset<8>(ds[6]);
      }
    }
    else if (line.find("EVpreemption") == 0)
    {
      string ds[8];
      for (int i = 0; i < 4; i++)
      {
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> ds[0] >> ds[1] >> ds[2] >> ds[3] >> ds[4] >> ds[5] >> ds[6] >> ds[7];
        iss.clear();        
        timingcard.EVpreemption[i].delay_time = static_cast<uint8_t>(atoi(ds[0].c_str()));
        timingcard.EVpreemption[i].green_hold_time = static_cast<uint8_t>(atoi(ds[1].c_str()));
        timingcard.EVpreemption[i].maximum_clearance_time = static_cast<uint8_t>(atoi(ds[2].c_str()));
        timingcard.EVpreemption[i].clearance_phase_green = bitset<8>(ds[3]);
        timingcard.EVpreemption[i].clearance_overlap_green = bitset<8>(ds[4]);
        timingcard.EVpreemption[i].input_port = static_cast<uint8_t>(atoi(ds[5].c_str()));
        timingcard.EVpreemption[i].latching_flag = static_cast<uint8_t>(atoi(ds[6].c_str()));
        timingcard.EVpreemption[i].phase_termination_flag = static_cast<uint8_t>(atoi(ds[7].c_str()));
      }
    }    
    else if (line.find("RRpreemption") == 0)
    {
      string ds[12];
      for (int i = 0; i < 2; i++)
      {
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> ds[0] >> ds[1] >> ds[2] >> ds[3] >> ds[4] >> ds[5] >> ds[6] >> ds[7] >> ds[8] >> ds[9] >> ds[10] >> ds[11];
        iss.clear();        
        timingcard.RRpreemption[i].delay_time = static_cast<uint8_t>(atoi(ds[0].c_str()));
        timingcard.RRpreemption[i].minimum_green = static_cast<uint8_t>(atoi(ds[1].c_str()));
        timingcard.RRpreemption[i].ped_clear_time = static_cast<uint8_t>(atoi(ds[2].c_str()));
        timingcard.RRpreemption[i].exit_time = static_cast<uint8_t>(atoi(ds[3].c_str()));
        timingcard.RRpreemption[i].exit_phases_green = bitset<8>(ds[4]);
        timingcard.RRpreemption[i].exit_overlaps_green = bitset<8>(ds[5]);
        timingcard.RRpreemption[i].exit_veh_call = bitset<8>(ds[6]);
        timingcard.RRpreemption[i].exit_ped_call = bitset<8>(ds[7]);
        timingcard.RRpreemption[i].input_port = static_cast<uint8_t>(atoi(ds[8].c_str()));
        timingcard.RRpreemption[i].gate_port = static_cast<uint8_t>(atoi(ds[9].c_str()));
        timingcard.RRpreemption[i].latching_flag = static_cast<uint8_t>(atoi(ds[10].c_str()));
        timingcard.RRpreemption[i].power_up = static_cast<uint8_t>(atoi(ds[11].c_str()));
        for (int j = 0; j < 4; j++)
        {
          std::getline(IS_CARD,line);
          iss.str(line);
          iss >> std::skipws >> ds[0] >> ds[1] >> ds[2] >> ds[3] >> ds[4] >> ds[5] >> ds[6] >> ds[7] >> ds[8] >> ds[9];
          iss.clear();        
          timingcard.RRpreemption[i].RRpreemption_steps[j].step_time = static_cast<uint8_t>(atoi(ds[0].c_str()));
          timingcard.RRpreemption[i].RRpreemption_steps[j].ped_walk_phases = bitset<8>(ds[1]);
          timingcard.RRpreemption[i].RRpreemption_steps[j].ped_clear_phases = bitset<8>(ds[2]);
          timingcard.RRpreemption[i].RRpreemption_steps[j].ped_red_phases = bitset<8>(ds[3]);
          timingcard.RRpreemption[i].RRpreemption_steps[j].green_hold_phases = bitset<8>(ds[4]);
          timingcard.RRpreemption[i].RRpreemption_steps[j].yew_flashing_phases = bitset<8>(ds[5]);
          timingcard.RRpreemption[i].RRpreemption_steps[j].red_flashing_phases = bitset<8>(ds[6]);
          timingcard.RRpreemption[i].RRpreemption_steps[j].green_hold_overlaps = bitset<8>(ds[7]);
          timingcard.RRpreemption[i].RRpreemption_steps[j].yew_flashing_overlaps = bitset<8>(ds[8]);
          timingcard.RRpreemption[i].RRpreemption_steps[j].red_flashing_overlaps = bitset<8>(ds[9]);
        }
      }    
    }
    else if (line.find("TSPconf") == 0)
    {
      string ds[11];
      iss.str(line);
      iss >> std::skipws >> s >> s;
      iss.clear();
      timingcard.TSPconf.enable_coordination_plans = bitset<18>(s);
      for (int i = 0; i < 18; i++)
      {
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> ds[0] >> ds[1] >> ds[2] >> ds[3] >> ds[4] >> ds[5] >> ds[6] >> ds[7] >> ds[8] >> ds[9] >> ds[10];
        iss.clear();
        timingcard.TSPconf.TSPplan[i].max_early_green = static_cast<uint8_t>(atoi(ds[0].c_str()));
        timingcard.TSPconf.TSPplan[i].max_green_extension = static_cast<uint8_t>(atoi(ds[1].c_str()));
        timingcard.TSPconf.TSPplan[i].inhibit_cycles = static_cast<uint8_t>(atoi(ds[2].c_str()));
        for (int j = 0; j < 8; j++)
        {
          timingcard.TSPconf.TSPplan[i].green_factor[j] = static_cast<uint8_t>(atoi(ds[3+j].c_str())); 
        }
      }
    }
    else if (line.find("Coordination_plans") == 0)
    {
      string ds[8];
      iss.str(line);
      iss >> std::skipws >> s >> s;
      iss.clear();
      int plannums = atoi(s.c_str());
      timingcard.coordplans.resize(plannums);
      for (int plan_idx = 0; plan_idx < plannums; plan_idx++)
      {
        // plan_num
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> s;
        iss.clear();
        timingcard.coordplans[plan_idx].plan_num = static_cast<uint8_t>(atoi(s.c_str()));
        // cycle_length
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> s;
        iss.clear();
        timingcard.coordplans[plan_idx].cycle_length = static_cast<uint8_t>(atoi(s.c_str()));
        // green_factor
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> ds[0] >> ds[1] >> ds[2] >> ds[3] >> ds[4] >> ds[5] >> ds[6] >> ds[7];
        iss.clear();
        for (int i = 0; i < 8; i++)
        {
          timingcard.coordplans[plan_idx].green_factor[i] = static_cast<uint8_t>(atoi(ds[i].c_str()));
        }
        // cycle_multiplier
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> s;
        iss.clear();
        timingcard.coordplans[plan_idx].cycle_multiplier = static_cast<uint8_t>(atoi(s.c_str()));
        // offsets
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> ds[0] >> ds[1] >> ds[2];
        iss.clear();
        for (int i = 0; i < 3; i++)
        {
          timingcard.coordplans[plan_idx].offsets[i] = static_cast<uint8_t>(atoi(ds[i].c_str()));
        }
        // laggapout_phase
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> s;
        iss.clear();
        timingcard.coordplans[plan_idx].laggapout_phase = static_cast<uint8_t>(atoi(s.c_str()));
        // lag_phases
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> s;
        iss.clear();
        timingcard.coordplans[plan_idx].lag_phases = bitset<8>(s);
        // sync_phases
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> s;
        iss.clear();
        timingcard.coordplans[plan_idx].sync_phases = bitset<8>(s);
        // hold_phases
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> s;
        iss.clear();
        timingcard.coordplans[plan_idx].hold_phases = bitset<8>(s);
        // omit_phases
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> s;
        iss.clear();
        timingcard.coordplans[plan_idx].omit_phases = bitset<8>(s);
        // minimum_recall_phases
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> s;
        iss.clear();
        timingcard.coordplans[plan_idx].minimum_recall_phases = bitset<8>(s);
        // maximum_recall_phases
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> s;
        iss.clear();
        timingcard.coordplans[plan_idx].maximum_recall_phases = bitset<8>(s);
        // ped_recall_phases
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> s;
        iss.clear();
        timingcard.coordplans[plan_idx].ped_recall_phases = bitset<8>(s);
        // bike_recall_phases
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> s;
        iss.clear();
        timingcard.coordplans[plan_idx].bike_recall_phases = bitset<8>(s);
        // force_off_flag
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> s;
        iss.clear();
        timingcard.coordplans[plan_idx].force_off_flag = static_cast<uint8_t>(atoi(s.c_str()));
        // permitted_phases
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> s;
        iss.clear();
        timingcard.coordplans[plan_idx].permitted_phases = bitset<8>(s);
        // permitted_ped_phases
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> s;
        iss.clear();
        timingcard.coordplans[plan_idx].permitted_ped_phases = bitset<8>(s);      
        // coordinated_phases
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> ds[0] >> ds[1];
        iss.clear();
        timingcard.coordplans[plan_idx].coordinated_phases[0] = static_cast<uint8_t>(atoi(ds[0].c_str()));
        timingcard.coordplans[plan_idx].coordinated_phases[1] = static_cast<uint8_t>(atoi(ds[1].c_str()));
        // leadLagMode
        std::getline(IS_CARD,line);
        int leadLagMode;
        iss.str(line);
        iss >> std::skipws >> s >> leadLagMode;
        iss.clear();
        switch(leadLagMode)
        {
        case 0:
          timingcard.coordplans[plan_idx].leadLagMode = operation_mode_enum_t::LEAD_LEAD;
          break;
        case 1:
          timingcard.coordplans[plan_idx].leadLagMode = operation_mode_enum_t::LEAD_LAG;
          break;
        case 2:
          timingcard.coordplans[plan_idx].leadLagMode = operation_mode_enum_t::LAG_LEAD;
          break;
        default:
          timingcard.coordplans[plan_idx].leadLagMode = operation_mode_enum_t::LAG_LAG;
        }
        // sync_ring
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> s;
        iss.clear();
        timingcard.coordplans[plan_idx].sync_ring = static_cast<uint8_t>(atoi(s.c_str()));
        // sync_barrier
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> s;
        iss.clear();
        timingcard.coordplans[plan_idx].sync_barrier = static_cast<uint8_t>(atoi(s.c_str()));
        //  leadlag_phases
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> ds[0] >> ds[1] >> ds[2] >> ds[3] >> ds[4] >> ds[5] >> ds[6] >> ds[7];
        iss.clear();
        int count = 0;
        for (int barrier = 0; barrier < 2; barrier++)
        {
          for (int ring = 0; ring < 2; ring++)
          {
            for (int seq = 0; seq < 2; seq++)
            {
              timingcard.coordplans[plan_idx].leadlag_phases[barrier][ring][seq] = static_cast<uint8_t>(atoi(ds[count].c_str()));
              count++;
            }
          }
        }
        // force_off
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> ds[0] >> ds[1] >> ds[2] >> ds[3] >> ds[4] >> ds[5] >> ds[6] >> ds[7];
        iss.clear();
        for (int i = 0; i < 8; i++)
        {
          timingcard.coordplans[plan_idx].force_off[i] = static_cast<uint8_t>(atoi(ds[i].c_str()));
        }
        // permissive
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> ds[0] >> ds[1] >> ds[2] >> ds[3] >> ds[4] >> ds[5] >> ds[6] >> ds[7];
        iss.clear();
        for (int i = 0; i < 8; i++)
        {
          timingcard.coordplans[plan_idx].permissive[i] = static_cast<uint8_t>(atoi(ds[i].c_str()));
        }
        // ped_permissive
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> ds[0] >> ds[1] >> ds[2] >> ds[3] >> ds[4] >> ds[5] >> ds[6] >> ds[7];
        iss.clear();
        for (int i = 0; i < 8; i++)
        {
          timingcard.coordplans[plan_idx].ped_permissive[i] = static_cast<uint8_t>(atoi(ds[i].c_str()));
        }
        //  noncoordBarrierGreenOnset
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> s;
        iss.clear();
        timingcard.coordplans[plan_idx].noncoordBarrierGreenOnset = static_cast<uint32_t>(atoi(s.c_str()));
        //  coordBarrierGreenOnset
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> s;
        iss.clear();
        timingcard.coordplans[plan_idx].coordBarrierGreenOnset = static_cast<uint32_t>(atoi(s.c_str()));
        //  coordPhaseGreenOnset
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> ds[0] >> ds[1];
        iss.clear();
        timingcard.coordplans[plan_idx].coordPhaseGreenOnset[0] = static_cast<uint32_t>(atoi(ds[0].c_str()));
        timingcard.coordplans[plan_idx].coordPhaseGreenOnset[1] = static_cast<uint32_t>(atoi(ds[1].c_str()));
        //  coordPhaseGreenEnd
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> s >> ds[0] >> ds[1];
        iss.clear();
        if (s.compare("true") == 0)
          timingcard.coordplans[plan_idx].coordLagphaseGapout = true;
        else
          timingcard.coordplans[plan_idx].coordLagphaseGapout = false;
        timingcard.coordplans[plan_idx].coordPhaseGreenEnd[0] = static_cast<uint32_t>(atoi(ds[0].c_str()));
        timingcard.coordplans[plan_idx].coordPhaseGreenEnd[1] = static_cast<uint32_t>(atoi(ds[1].c_str()));
        //  isTSPenabled
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> s >> ds[0] >> ds[1] >> ds[2];
        iss.clear();
        if (s.compare("true") == 0)
          timingcard.coordplans[plan_idx].isTSPenabled = true;
        else
          timingcard.coordplans[plan_idx].isTSPenabled = false;
        timingcard.coordplans[plan_idx].max_early_green = static_cast<uint8_t>(atoi(ds[0].c_str()));
        timingcard.coordplans[plan_idx].max_green_extension = static_cast<uint8_t>(atoi(ds[1].c_str()));
        timingcard.coordplans[plan_idx].inhibit_cycles = static_cast<uint8_t>(atoi(ds[2].c_str()));
        // TSP_force_off
        std::getline(IS_CARD,line);
        iss.str(line);
        iss >> std::skipws >> s >> ds[0] >> ds[1] >> ds[2] >> ds[3] >> ds[4] >> ds[5] >> ds[6] >> ds[7];
        iss.clear();
        for (int i = 0; i < 8; i++)
        {
          timingcard.coordplans[plan_idx].TSP_force_off[i] = static_cast<uint8_t>(atoi(ds[i].c_str()));
        }
        // empty line
        std::getline(IS_CARD,line);
      }
    }
  }
  IS_CARD.close();
  if (timingcard.controller_addr == 0 || timingcard.phaseflags.permitted_phases.count() == 0
    || (timingcard.coordplans.size() == 0 && !timingcard.manualplan.planOn))
  {
    return false;
  }
  /// build plannum2planindex_map
  getPlanIdxMap(timingcard.plannum2planindex_map,timingcard.coordplans);        
  return true;
}
