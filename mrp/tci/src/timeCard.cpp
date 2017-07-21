//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>

#include "timeCard.h"

auto barrier_phases_on = [](const std::bitset<8>& check_phases)->uint8_t
{
	std::bitset<8> left_barrier(std::string("00110011"));
	left_barrier &= check_phases;
	return((left_barrier.any()) ? 0 : 1);
};
auto next_ring     = [](uint8_t check_ring){return(static_cast<uint8_t>((check_ring + 1) % 2));};
auto next_barrier  = [](uint8_t check_barrier){return(static_cast<uint8_t>((check_barrier + 1) % 2));};
auto bitset2array  = [](uint8_t (&ringphases)[2], const std::bitset<8>& bitsetPhases)->void
{
	ringphases[0] = 0;
	ringphases[1] = 0;
	for (int i = 0; i < 8; i++)
	{
		if (bitsetPhases.test(i))
		{
			if (i < 4)
				ringphases[0] = static_cast<uint8_t>(i + 1);
			else
			{
				ringphases[1] = static_cast<uint8_t>(i + 1);
				break;
			}
		}
	}
};

Card::Card(void)
{
	initiated = false;
	timingcard.reset();
}

void Card::setInitiated(void)
	{initiated = true;}

bool Card::isInitiated(void) const
	{return(initiated);}

uint8_t Card::getControllerAddr(void) const
	{return(timingcard.controller_addr);}

void Card::setControllerAddr(uint8_t addr)
	{timingcard.controller_addr = addr;}

void Card::resetPlans(void)
	{timingcard.resetplans();}

Card::phaseflags_mess_t Card::getPhaseFlags(void) const
	{return(timingcard.phaseflags);}

Card::freeplan_mess_t Card::getFreePlan(void) const
	{return(timingcard.freeplan);}

std::vector<Card::coordplan_mess_t> Card::getCoordPlans(void) const
	{return(timingcard.coordplans);}

std::vector<Card::phasetiming_mess_t> Card::getPhaseTiming(void) const
{
	std::vector<Card::phasetiming_mess_t> ret;
	for (int i = 0; i < 8; i++)
		ret.push_back(timingcard.phasetiming[i]);
	return(ret);
}

ssize_t Card::get_coordplan_index(uint8_t plan_num) const
{
	auto it = std::find_if(timingcard.coordplans.begin(), timingcard.coordplans.end(),
		[&plan_num](const Card::coordplan_mess_t& item){return(item.plan_num == plan_num);});
	return((it == timingcard.coordplans.end()) ? (-1) : (ssize_t)(it - timingcard.coordplans.begin()));
}

bool Card::getPatnIdx(ssize_t& planIdx, MsgEnum::controlMode mode, uint8_t plan_num) const
{
	if ((mode == MsgEnum::controlMode::flashing) || (mode == MsgEnum::controlMode::runningFree))
	{
		planIdx = -1;
		return(true);
	}
	planIdx = Card::get_coordplan_index(plan_num);
	return(planIdx >= 0);
}

bool Card::logTimeCard(const std::string& fname) const
{
	std::ofstream OS_CARD(fname);
	if (!OS_CARD.is_open())
	{
		std::cerr << "logTimeCard: could not open " << fname << std::endl;
		return(false);
	}
	OS_CARD << "controller_addr " << static_cast<unsigned int>(timingcard.controller_addr) << std::endl << std::endl;

	OS_CARD << "Phase_falgs" << std::endl;
	OS_CARD << "permitted_phases " << timingcard.phaseflags.permitted_phases.to_string() << std::endl;
	OS_CARD << "restricted_phases " << timingcard.phaseflags.restricted_phases.to_string() << std::endl;
	OS_CARD << "permitted_ped_phases " << timingcard.phaseflags.permitted_ped_phases.to_string() << std::endl;
	OS_CARD << "minimum_recall_phases " << timingcard.phaseflags.minimum_recall_phases.to_string() << std::endl;
	OS_CARD << "maximum_recall_phases " << timingcard.phaseflags.maximum_recall_phases.to_string() << std::endl;
	OS_CARD << "ped_recall_phases " << timingcard.phaseflags.ped_recall_phases.to_string() << std::endl;
	OS_CARD << "bike_recall_phases " << timingcard.phaseflags.bike_recall_phases.to_string() << std::endl;
	OS_CARD << "redlock_phases " << timingcard.phaseflags.redlock_phases.to_string() << std::endl;
	OS_CARD << "yewlock_phases " << timingcard.phaseflags.yewlock_phases.to_string() << std::endl;
	OS_CARD << "fomaxlock_phases " << timingcard.phaseflags.fomaxlock_phases.to_string() << std::endl;
	OS_CARD << "doubleEntry_phases " << timingcard.phaseflags.doubleEntry_phases.to_string() << std::endl;
	OS_CARD << "restInWalk_phases " << timingcard.phaseflags.restInWalk_phases.to_string() << std::endl;
	OS_CARD << "restInRed_phases " << timingcard.phaseflags.restInRed_phases.to_string() << std::endl;
	OS_CARD << "walk2_phases " << timingcard.phaseflags.walk2_phases.to_string() << std::endl;
	OS_CARD << "maxgreen2_phases " << timingcard.phaseflags.maxgreen2_phases.to_string() << std::endl;
	OS_CARD << "maxgreen3_phases " << timingcard.phaseflags.maxgreen3_phases.to_string() << std::endl;
	OS_CARD << "startup_green_phases " << timingcard.phaseflags.startup_green_phases.to_string() << std::endl;
	OS_CARD << "startup_yellow_phases " << timingcard.phaseflags.startup_yellow_phases.to_string() << std::endl;
	OS_CARD << "startup_vehCalls " << timingcard.phaseflags.startup_vehCalls.to_string() << std::endl;
	OS_CARD << "startup_pedCalls " << timingcard.phaseflags.startup_pedCalls.to_string() << std::endl;
	OS_CARD << "startup_yellowOverlaps " << timingcard.phaseflags.startup_yellowOverlaps.to_string() << std::endl;
	OS_CARD << "startup_allred " << static_cast<unsigned int>(timingcard.phaseflags.startup_allred) << std::endl;
	OS_CARD << "red_revert_interval " << static_cast<unsigned int>(timingcard.phaseflags.red_revert_interval) << std::endl << std::endl;

	OS_CARD << "Phase_timing" << std::endl;
	for (int i = 0; i < 8; i++)
	{
		auto& timing = timingcard.phasetiming[i];
		OS_CARD << "phase_num " << static_cast<unsigned int>(timing.phase_num) << std::endl;
		OS_CARD << "walk1_interval " << static_cast<unsigned int>(timing.walk1_interval) << std::endl;
		OS_CARD << "walk_clearance " << static_cast<unsigned int>(timing.walk_clearance) << std::endl;
		OS_CARD << "minimum_green " << static_cast<unsigned int>(timing.minimum_green) << std::endl;
		OS_CARD << "detector_limit " << static_cast<unsigned int>(timing.detector_limit) << std::endl;
		OS_CARD << "maximum_initial " << static_cast<unsigned int>(timing.maximum_initial) << std::endl;
		OS_CARD << "maximum_extensions_I " << static_cast<unsigned int>(timing.maximum_extensions[0]) << std::endl;
		OS_CARD << "maximum_extensions_II " << static_cast<unsigned int>(timing.maximum_extensions[1]) << std::endl;
		OS_CARD << "maximum_extensions_III " << static_cast<unsigned int>(timing.maximum_extensions[2]) << std::endl;
		OS_CARD << "passage " << static_cast<unsigned int>(timing.passage) << std::endl;
		OS_CARD << "maximum_gap " << static_cast<unsigned int>(timing.maximum_gap) << std::endl;
		OS_CARD << "minimum_gap " << static_cast<unsigned int>(timing.minimum_gap) << std::endl;
		OS_CARD << "added_initial_per_vehicle " << static_cast<unsigned int>(timing.added_initial_per_vehicle) << std::endl;
		OS_CARD << "reduce_gap_by " << static_cast<unsigned int>(timing.reduce_gap_by) << std::endl;
		OS_CARD << "reduce_gap_every " << static_cast<unsigned int>(timing.reduce_gap_every) << std::endl;
		OS_CARD << "yellow_interval " << static_cast<unsigned int>(timing.yellow_interval) << std::endl;
		OS_CARD << "red_clearance " << static_cast<unsigned int>(timing.red_clearance) << std::endl;
		OS_CARD << "walk2_interval " << static_cast<unsigned int>(timing.walk2_interval) << std::endl;
		OS_CARD << "delay_early_walk_time " << static_cast<unsigned int>(timing.delay_early_walk_time) << std::endl;
		OS_CARD << "solid_walk_clearance " << static_cast<unsigned int>(timing.solid_walk_clearance) << std::endl;
		OS_CARD << "bike_green " << static_cast<unsigned int>(timing.bike_green) << std::endl;
		OS_CARD << "bike_red_clearance " << static_cast<unsigned int>(timing.bike_red_clearance) << std::endl << std::endl;
	}

	OS_CARD << "Free_plan" << std::endl;
	auto& freeplan = timingcard.freeplan;
	OS_CARD << "lag_phases " << freeplan.lag_phases.to_string() << std::endl;
	OS_CARD << "omit_phases " << freeplan.omit_phases.to_string() << std::endl;
	OS_CARD << "minimum_recall_phases " << freeplan.minimum_recall_phases.to_string() << std::endl;
	OS_CARD << "maximum_recall_phases " << freeplan.maximum_recall_phases.to_string() << std::endl;
	OS_CARD << "ped_recall_phases " << freeplan.ped_recall_phases.to_string() << std::endl;
	OS_CARD << "bike_recall_phases " << freeplan.bike_recall_phases.to_string() << std::endl;
	OS_CARD << "conditional_service_phases " << freeplan.conditional_service_phases.to_string() << std::endl;
	OS_CARD << "conditional_service_minimum_green " << static_cast<unsigned int>(freeplan.conditional_service_minimum_green) << std::endl;
	OS_CARD << "permitted_phases " << freeplan.permitted_phases.to_string() << std::endl;
	OS_CARD << "permitted_ped_phases " << freeplan.permitted_ped_phases.to_string() << std::endl;
	OS_CARD << "leadlag_phases";
	for (int barrier = 0; barrier < 2; barrier++)
	{
		for (int ring = 0; ring < 2; ring++)
		{
			OS_CARD << " " << static_cast<unsigned int>(freeplan.leadlag_phases[barrier][ring][0]);
			OS_CARD << " " << static_cast<unsigned int>(freeplan.leadlag_phases[barrier][ring][1]);
		}
	}
	OS_CARD << std::endl;
	OS_CARD << "TSP_conf " << std::boolalpha << freeplan.isTSPenabled;
	OS_CARD << " " << freeplan.TSP_hold_phases.to_string();
	OS_CARD << " " << static_cast<unsigned int>(freeplan.TSP_max_green_hold_time) << std::endl << std::endl;

	OS_CARD << "Manual_plan " << std::boolalpha << timingcard.manualplan.planOn;
	OS_CARD << " " << static_cast<unsigned int>(timingcard.manualplan.plan_num);
	OS_CARD << " " << static_cast<unsigned int>(timingcard.manualplan.offset_index) << std::endl << std::endl;

	OS_CARD << "CIC_plan " << timingcard.cicplans.enabled_plans.to_string();
	OS_CARD << " " << static_cast<unsigned int>(timingcard.cicplans.smoothing_volume);
	OS_CARD << " " << static_cast<unsigned int>(timingcard.cicplans.smoothing_occupancy);
	OS_CARD << " " << static_cast<unsigned int>(timingcard.cicplans.smoothing_demand);
	OS_CARD << " " << static_cast<unsigned int>(timingcard.cicplans.multiplier_volume);
	OS_CARD << " " << static_cast<unsigned int>(timingcard.cicplans.multiplier_occupancy);
	OS_CARD << " " << static_cast<unsigned int>(timingcard.cicplans.exponent_volume);
	OS_CARD << " " << static_cast<unsigned int>(timingcard.cicplans.exponent_occupancy) << std::endl;
	for (int i = 0; i < 16; i++)
		OS_CARD << static_cast<unsigned int>(timingcard.cicplans.phase_assignment[i]) << " ";
	OS_CARD << std::endl << std::endl;

	OS_CARD << "detector_conf " << timingcard.detectorconf.size() << std::endl;
	for (auto& item : timingcard.detectorconf)
	{
		OS_CARD << static_cast<unsigned int>(item.id) << " ";
		OS_CARD << static_cast<unsigned int>(item.type) << " ";
		OS_CARD << item.phaseAssignment.to_string() << " ";
		OS_CARD << static_cast<unsigned int>(item.lock) << " ";
		OS_CARD << static_cast<unsigned int>(item.delayTime) << " ";
		OS_CARD << static_cast<unsigned int>(item.extendTime) << " ";
		OS_CARD << static_cast<unsigned int>(item.recallTime) << " ";
		OS_CARD << static_cast<unsigned int>(item.inputPort) << std::endl;
	}
	OS_CARD << std::endl;

	OS_CARD << "system_detectors";
	OS_CARD << " " << static_cast<unsigned int>(timingcard.system_detector_assignment.maxOnTime);
	OS_CARD << " " << static_cast<unsigned int>(timingcard.system_detector_assignment.failResetTime);
	OS_CARD << " " << timingcard.system_detector_assignment.failOverride.to_string() << std::endl;
	for (int i = 0; i < 16; i++)
		OS_CARD << static_cast<unsigned int>(timingcard.system_detector_assignment.detectorInput[i]) << " ";
	OS_CARD << std::endl << std::endl;

	OS_CARD << "TODtables " << timingcard.TODtables.size() << std::endl;
	for (auto& item : timingcard.TODtables)
	{
		OS_CARD << static_cast<unsigned int>(item.table_num) << " ";
		OS_CARD << static_cast<unsigned int>(item.start_hour) << " ";
		OS_CARD << static_cast<unsigned int>(item.start_min) << " ";
		OS_CARD << static_cast<unsigned int>(item.plan_num) << " ";
		OS_CARD << static_cast<unsigned int>(item.offset_index) << std::endl;
	}
	OS_CARD << std::endl;

	OS_CARD << "weekday_tables";
	for (int i = 0; i < 7; i++)
		OS_CARD << " " << static_cast<unsigned int>(timingcard.weekday_plan_assignment[i]);
	OS_CARD << std::endl << std::endl;

	OS_CARD << "TODfunctions " << timingcard.TODfunctions.size() << std::endl;
	for (auto& item : timingcard.TODfunctions)
	{
		OS_CARD << static_cast<unsigned int>(item.start_hour) << " ";
		OS_CARD << static_cast<unsigned int>(item.start_min) << " ";
		OS_CARD << static_cast<unsigned int>(item.end_hour) << " ";
		OS_CARD << static_cast<unsigned int>(item.end_min) << " ";
		OS_CARD << item.dayofweek.to_string() << " ";
		OS_CARD << static_cast<unsigned int>(item.action_code) << " ";
		OS_CARD << item.affect_phases.to_string() << std::endl;
	}
	OS_CARD << std::endl;

	OS_CARD << "EVpreemption" << std::endl;
	for (int i = 0; i < 4; i++)
	{
		auto& item = timingcard.EVpreemption[i];
		OS_CARD << static_cast<unsigned int>(item.delay_time) << " ";
		OS_CARD << static_cast<unsigned int>(item.green_hold_time) << " ";
		OS_CARD << static_cast<unsigned int>(item.maximum_clearance_time) << " ";
		OS_CARD << item.clearance_phase_green.to_string() << " ";
		OS_CARD << item.clearance_overlap_green.to_string() << " ";
		OS_CARD << static_cast<unsigned int>(item.input_port) << " ";
		OS_CARD << static_cast<unsigned int>(item.latching_flag) << " ";
		OS_CARD << static_cast<unsigned int>(item.phase_termination_flag) << std::endl;
	}
	OS_CARD << std::endl;

	OS_CARD << "RRpreemption" << std::endl;
	for (int i = 0; i < 2; i++)
	{
		auto& item = timingcard.RRpreemption[i];
		OS_CARD << static_cast<unsigned int>(item.delay_time) << " ";
		OS_CARD << static_cast<unsigned int>(item.minimum_green) << " ";
		OS_CARD << static_cast<unsigned int>(item.ped_clear_time) << " ";
		OS_CARD << static_cast<unsigned int>(item.exit_time) << " ";
		OS_CARD << item.exit_phases_green.to_string() << " ";
		OS_CARD << item.exit_overlaps_green.to_string() << " ";
		OS_CARD << item.exit_veh_call.to_string() << " ";
		OS_CARD << item.exit_ped_call.to_string() << " ";
		OS_CARD << static_cast<unsigned int>(item.input_port) << " ";
		OS_CARD << static_cast<unsigned int>(item.gate_port) << " ";
		OS_CARD << static_cast<unsigned int>(item.latching_flag) << " ";
		OS_CARD << static_cast<unsigned int>(item.power_up) << std::endl;
		for (int j = 0; j < 4; j++)
		{
			auto& step = item.RRpreemption_steps[j];
			OS_CARD << static_cast<unsigned int>(step.step_time) << " ";
			OS_CARD << step.ped_walk_phases.to_string() << " ";
			OS_CARD << step.ped_clear_phases.to_string() << " ";
			OS_CARD << step.ped_red_phases.to_string() << " ";
			OS_CARD << step.green_hold_phases.to_string() << " ";
			OS_CARD << step.yew_flashing_phases.to_string() << " ";
			OS_CARD << step.red_flashing_phases.to_string() << " ";
			OS_CARD << step.green_hold_overlaps.to_string() << " ";
			OS_CARD << step.yew_flashing_overlaps.to_string() << " ";
			OS_CARD << step.red_flashing_overlaps.to_string() << std::endl;
		}
	}
	OS_CARD << std::endl;

	OS_CARD << "TSPconf " << timingcard.TSPconf.enable_coordination_plans.to_string() << std::endl;
	for (int i = 0; i < 18; i++)
	{
		auto& plan = timingcard.TSPconf.TSPplan[i];
		OS_CARD << static_cast<unsigned int>(plan.max_early_green) << " ";
		OS_CARD << static_cast<unsigned int>(plan.max_green_extension) << " ";
		OS_CARD << static_cast<unsigned int>(plan.inhibit_cycles);
		for (int j = 0; j < 8; j++)
			OS_CARD << " " << static_cast<unsigned int>(plan.green_factor[j]);
		OS_CARD << std::endl;
	}
	OS_CARD << std::endl;

	OS_CARD << "Coordination_plans " << timingcard.coordplans.size() << std::endl;
	for (auto& plan : timingcard.coordplans)
	{
		OS_CARD << "plan_num " << static_cast<unsigned int>(plan.plan_num) << std::endl;
		OS_CARD << "cycle_length " << static_cast<unsigned int>(plan.cycle_length) << std::endl;
		OS_CARD << "green_factor";
		for (int j = 0; j < 8; j++)
			OS_CARD << " " << static_cast<unsigned int>(plan.green_factor[j]);
		OS_CARD << std::endl;
		OS_CARD << "cycle_multiplier " << static_cast<unsigned int>(plan.cycle_multiplier) << std::endl;
		OS_CARD << "offsets";
		for (int j = 0; j < 3; j++)
			OS_CARD << " " << static_cast<unsigned int>(plan.offsets[j]);
		OS_CARD << std::endl;
		OS_CARD << "laggapout_phase " << static_cast<unsigned int>(plan.laggapout_phase) << std::endl;
		OS_CARD << "lag_phases " << plan.lag_phases.to_string() << std::endl;
		OS_CARD << "sync_phases " << plan.sync_phases.to_string() << std::endl;
		OS_CARD << "hold_phases " << plan.hold_phases.to_string() << std::endl;
		OS_CARD << "omit_phases " << plan.omit_phases.to_string() << std::endl;
		OS_CARD << "minimum_recall_phases " << plan.minimum_recall_phases.to_string() << std::endl;
		OS_CARD << "maximum_recall_phases " << plan.maximum_recall_phases.to_string() << std::endl;
		OS_CARD << "ped_recall_phases " << plan.ped_recall_phases.to_string() << std::endl;
		OS_CARD << "bike_recall_phases " << plan.bike_recall_phases.to_string() << std::endl;
		OS_CARD << "force_off_flag " << static_cast<unsigned int>(plan.force_off_flag) << std::endl;
		OS_CARD << "permitted_phases " << plan.permitted_phases.to_string() << std::endl;
		OS_CARD << "permitted_ped_phases " << plan.permitted_ped_phases.to_string() << std::endl;
		OS_CARD << "coordinated_phases " << static_cast<unsigned int>(plan.coordinated_phases[0]);
		OS_CARD << " " << static_cast<unsigned int>(plan.coordinated_phases[1]) << std::endl;
		OS_CARD << "leadLagMode " << static_cast<unsigned int>(plan.leadLagMode) << std::endl;
		OS_CARD << "sync_ring " << static_cast<unsigned int>(plan.sync_ring) << std::endl;
		OS_CARD << "sync_barrier " << static_cast<unsigned int>(plan.sync_barrier) << std::endl;
		OS_CARD << "leadlag_phases";
		for (int barrier = 0; barrier < 2; barrier++)
		{
			for (int ring = 0; ring < 2; ring++)
			{
				OS_CARD << " " << static_cast<unsigned int>(plan.leadlag_phases[barrier][ring][0]);
				OS_CARD << " " << static_cast<unsigned int>(plan.leadlag_phases[barrier][ring][1]);
			}
		}
		OS_CARD << std::endl;
		OS_CARD << "force_off";
		for (int j = 0; j < 8; j++)
			OS_CARD << " " << static_cast<unsigned int>(plan.force_off[j]);
		OS_CARD << std::endl;
		OS_CARD << "permissive";
		for (int j = 0; j < 8; j++)
			OS_CARD << " " << static_cast<unsigned int>(plan.permissive[j]);
		OS_CARD << std::endl;
		OS_CARD << "ped_permissive";
		for (int j = 0; j < 8; j++)
			OS_CARD << " " << static_cast<unsigned int>(plan.ped_permissive[j]);
		OS_CARD << std::endl;
		OS_CARD << "noncoordBarrierGreenOnset " << static_cast<unsigned int>(plan.noncoordBarrierGreenOnset) << std::endl;
		OS_CARD << "coordBarrierGreenOnset " << static_cast<unsigned int>(plan.coordBarrierGreenOnset) << std::endl;
		OS_CARD << "coordPhaseGreenOnset " << static_cast<unsigned int>(plan.coordPhaseGreenOnset[0]);
		OS_CARD << " " << static_cast<unsigned int>(plan.coordPhaseGreenOnset[1]) << std::endl;
		OS_CARD << "coordPhaseGreenEnd " << std::boolalpha << plan.coordLagphaseGapout;
		OS_CARD << " " << static_cast<unsigned int>(plan.coordPhaseGreenEnd[0]);
		OS_CARD << " " << static_cast<unsigned int>(plan.coordPhaseGreenEnd[1]) << std::endl;
		OS_CARD << "isTSPenabled " << std::boolalpha << plan.isTSPenabled;
		OS_CARD << " " << static_cast<unsigned int>(plan.max_early_green);
		OS_CARD << " " << static_cast<unsigned int>(plan.max_green_extension);
		OS_CARD << " " << static_cast<unsigned int>(plan.inhibit_cycles) << std::endl;
		OS_CARD << "TSP_force_off";
		for (int j = 0; j < 8; j++)
			OS_CARD << " " << static_cast<unsigned int>(plan.TSP_force_off[j]);
		OS_CARD << std::endl << std::endl;
	}
	OS_CARD.close();
	return(true);
}

bool Card::readTimeCard(const std::string& fname)
{
	timingcard.reset();
	std::ifstream IS_CARD(fname);
	if (!IS_CARD.is_open())
	{
		std::cerr << "readTimeCard: could not open " << fname << std::endl;
		return(false);
	}
	std::istringstream iss;
	std::string line, s;
	unsigned int i_tmp;
	while (std::getline(IS_CARD,line))
	{
		if (line.empty())
			continue;
		if (line.find("controller_addr") == 0)
		{
			iss.str(line);
			iss >> std::skipws >> s >> i_tmp;
			iss.clear();
			timingcard.controller_addr = static_cast<uint8_t>(i_tmp);
		}
		else if (line.find("Phase_falgs") == 0)
		{ //  permitted_phases
			std::getline(IS_CARD,line);
			iss.str(line);
			iss >> std::skipws >> s >> s;
			iss.clear();
			timingcard.phaseflags.permitted_phases = std::bitset<8>(s);
			//  restricted_phases
			std::getline(IS_CARD,line);
			iss.str(line);
			iss >> std::skipws >> s >> s;
			iss.clear();
			timingcard.phaseflags.restricted_phases = std::bitset<8>(s);
			//  permitted_ped_phases
			std::getline(IS_CARD,line);
			iss.str(line);
			iss >> std::skipws >> s >> s;
			iss.clear();
			timingcard.phaseflags.permitted_ped_phases = std::bitset<8>(s);
			//  minimum_recall_phases
			std::getline(IS_CARD,line);
			iss.str(line);
			iss >> std::skipws >> s >> s;
			iss.clear();
			timingcard.phaseflags.minimum_recall_phases = std::bitset<8>(s);
			//  maximum_recall_phases
			std::getline(IS_CARD,line);
			iss.str(line);
			iss >> std::skipws >> s >> s;
			iss.clear();
			timingcard.phaseflags.maximum_recall_phases =std::bitset<8>(s);
			//  ped_recall_phases
			std::getline(IS_CARD,line);
			iss.str(line);
			iss >> std::skipws >> s >> s;
			iss.clear();
			timingcard.phaseflags.ped_recall_phases = std::bitset<8>(s);
			//  bike_recall_phases
			std::getline(IS_CARD,line);
			iss.str(line);
			iss >> std::skipws >> s >> s;
			iss.clear();
			timingcard.phaseflags.bike_recall_phases = std::bitset<8>(s);
			//  redlock_phases
			std::getline(IS_CARD,line);
			iss.str(line);
			iss >> std::skipws >> s >> s;
			iss.clear();
			timingcard.phaseflags.redlock_phases = std::bitset<8>(s);
			//  yewlock_phases
			std::getline(IS_CARD,line);
			iss.str(line);
			iss >> std::skipws >> s >> s;
			iss.clear();
			timingcard.phaseflags.yewlock_phases = std::bitset<8>(s);
			//  fomaxlock_phases
			std::getline(IS_CARD,line);
			iss.str(line);
			iss >> std::skipws >> s >> s;
			iss.clear();
			timingcard.phaseflags.fomaxlock_phases = std::bitset<8>(s);
			//  doubleEntry_phases
			std::getline(IS_CARD,line);
			iss.str(line);
			iss >> std::skipws >> s >> s;
			iss.clear();
			timingcard.phaseflags.doubleEntry_phases = std::bitset<8>(s);
			//  restInWalk_phases
			std::getline(IS_CARD,line);
			iss.str(line);
			iss >> std::skipws >> s >> s;
			iss.clear();
			timingcard.phaseflags.restInWalk_phases = std::bitset<8>(s);
			//  restInRed_phases
			std::getline(IS_CARD,line);
			iss.str(line);
			iss >> std::skipws >> s >> s;
			iss.clear();
			timingcard.phaseflags.restInRed_phases = std::bitset<8>(s);
			//  walk2_phases
			std::getline(IS_CARD,line);
			iss.str(line);
			iss >> std::skipws >> s >> s;
			iss.clear();
			timingcard.phaseflags.walk2_phases = std::bitset<8>(s);
			//  maxgreen2_phases
			std::getline(IS_CARD,line);
			iss.str(line);
			iss >> std::skipws >> s >> s;
			iss.clear();
			timingcard.phaseflags.maxgreen2_phases = std::bitset<8>(s);
			//  maxgreen3_phases
			std::getline(IS_CARD,line);
			iss.str(line);
			iss >> std::skipws >> s >> s;
			iss.clear();
			timingcard.phaseflags.maxgreen3_phases = std::bitset<8>(s);
			//  startup_green_phases
			std::getline(IS_CARD,line);
			iss.str(line);
			iss >> std::skipws >> s >> s;
			iss.clear();
			timingcard.phaseflags.startup_green_phases = std::bitset<8>(s);
			//  startup_yellow_phases
			std::getline(IS_CARD,line);
			iss.str(line);
			iss >> std::skipws >> s >> s;
			iss.clear();
			timingcard.phaseflags.startup_yellow_phases = std::bitset<8>(s);
			//  startup_vehCalls
			std::getline(IS_CARD,line);
			iss.str(line);
			iss >> std::skipws >> s >> s;
			iss.clear();
			timingcard.phaseflags.startup_vehCalls = std::bitset<8>(s);
			//  startup_pedCalls
			std::getline(IS_CARD,line);
			iss.str(line);
			iss >> std::skipws >> s >> s;
			iss.clear();
			timingcard.phaseflags.startup_pedCalls = std::bitset<8>(s);
			//  startup_yellowOverlaps
			std::getline(IS_CARD,line);
			iss.str(line);
			iss >> std::skipws >> s >> s;
			iss.clear();
			timingcard.phaseflags.startup_yellowOverlaps = std::bitset<8>(s);
			//  startup_allred
			std::getline(IS_CARD,line);
			iss.str(line);
			iss >> std::skipws >> s >> i_tmp;
			iss.clear();
			timingcard.phaseflags.startup_allred = static_cast<uint8_t>(i_tmp);
			//  red_revert_interval
			std::getline(IS_CARD,line);
			iss.str(line);
			iss >> std::skipws >> s >> i_tmp;
			iss.clear();
			timingcard.phaseflags.red_revert_interval = static_cast<uint8_t>(i_tmp);
		}
		else if (line.find("Phase_timing") == 0)
		{
			for (int phase_idx = 0; phase_idx < 8; phase_idx++)
			{
				auto& timing = timingcard.phasetiming[phase_idx];
				/// phase_num
				std::getline(IS_CARD,line);
				iss.str(line);
				iss >> std::skipws >> s >> i_tmp;
				iss.clear();
				timing.phase_num = static_cast<uint8_t>(i_tmp);
				/// walk1_interval
				std::getline(IS_CARD,line);
				iss.str(line);
				iss >> std::skipws >> s >> i_tmp;
				iss.clear();
				timing.walk1_interval = static_cast<uint8_t>(i_tmp);
				/// walk_clearance
				std::getline(IS_CARD,line);
				iss.str(line);
				iss >> std::skipws >> s >> i_tmp;
				iss.clear();
				timing.walk_clearance = static_cast<uint8_t>(i_tmp);
				/// minimum_green
				std::getline(IS_CARD,line);
				iss.str(line);
				iss >> std::skipws >> s >> i_tmp;
				iss.clear();
				timing.minimum_green = static_cast<uint8_t>(i_tmp);
				/// detector_limit
				std::getline(IS_CARD,line);
				iss.str(line);
				iss >> std::skipws >> s >> i_tmp;
				iss.clear();
				timing.detector_limit = static_cast<uint8_t>(i_tmp);
				/// maximum_initial
				std::getline(IS_CARD,line);
				iss.str(line);
				iss >> std::skipws >> s >> i_tmp;
				iss.clear();
				timing.maximum_initial = static_cast<uint8_t>(i_tmp);
				/// maximum_extensions_I
				std::getline(IS_CARD,line);
				iss.str(line);
				iss >> std::skipws >> s >> i_tmp;
				iss.clear();
				timing.maximum_extensions[0] = static_cast<uint8_t>(i_tmp);
				/// maximum_extensions_II
				std::getline(IS_CARD,line);
				iss.str(line);
				iss >> std::skipws >> s >> i_tmp;
				iss.clear();
				timing.maximum_extensions[1] = static_cast<uint8_t>(i_tmp);
				/// maximum_extensions_III
				std::getline(IS_CARD,line);
				iss.str(line);
				iss >> std::skipws >> s >> i_tmp;
				iss.clear();
				timing.maximum_extensions[2] = static_cast<uint8_t>(i_tmp);
				/// passage
				std::getline(IS_CARD,line);
				iss.str(line);
				iss >> std::skipws >> s >> i_tmp;
				iss.clear();
				timing.passage = static_cast<uint8_t>(i_tmp);
				/// maximum_gap
				std::getline(IS_CARD,line);
				iss.str(line);
				iss >> std::skipws >> s >> i_tmp;
				iss.clear();
				timing.maximum_gap = static_cast<uint8_t>(i_tmp);
				/// minimum_gap
				std::getline(IS_CARD,line);
				iss.str(line);
				iss >> std::skipws >> s >> i_tmp;
				iss.clear();
				timing.minimum_gap = static_cast<uint8_t>(i_tmp);
				/// added_initial_per_vehicle
				std::getline(IS_CARD,line);
				iss.str(line);
				iss >> std::skipws >> s >> i_tmp;
				iss.clear();
				timing.added_initial_per_vehicle = static_cast<uint8_t>(i_tmp);
				/// reduce_gap_by
				std::getline(IS_CARD,line);
				iss.str(line);
				iss >> std::skipws >> s >> i_tmp;
				iss.clear();
				timing.reduce_gap_by = static_cast<uint8_t>(i_tmp);
				/// reduce_gap_every
				std::getline(IS_CARD,line);
				iss.str(line);
				iss >> std::skipws >> s >> i_tmp;
				iss.clear();
				timing.reduce_gap_every = static_cast<uint8_t>(i_tmp);
				/// yellow_interval
				std::getline(IS_CARD,line);
				iss.str(line);
				iss >> std::skipws >> s >> i_tmp;
				iss.clear();
				timing.yellow_interval = static_cast<uint8_t>(i_tmp);
				/// red_clearance
				std::getline(IS_CARD,line);
				iss.str(line);
				iss >> std::skipws >> s >> i_tmp;
				iss.clear();
				timing.red_clearance = static_cast<uint8_t>(i_tmp);
				/// walk2_interval
				std::getline(IS_CARD,line);
				iss.str(line);
				iss >> std::skipws >> s >> i_tmp;
				iss.clear();
				timing.walk2_interval = static_cast<uint8_t>(i_tmp);
				/// delay_early_walk_time
				std::getline(IS_CARD,line);
				iss.str(line);
				iss >> std::skipws >> s >> i_tmp;
				iss.clear();
				timing.delay_early_walk_time = static_cast<uint8_t>(i_tmp);
				/// solid_walk_clearance
				std::getline(IS_CARD,line);
				iss.str(line);
				iss >> std::skipws >> s >> i_tmp;
				iss.clear();
				timing.solid_walk_clearance = static_cast<uint8_t>(i_tmp);
				/// bike_green
				std::getline(IS_CARD,line);
				iss.str(line);
				iss >> std::skipws >> s >> i_tmp;
				iss.clear();
				timing.bike_green = static_cast<uint8_t>(i_tmp);
				/// bike_red_clearance
				std::getline(IS_CARD,line);
				iss.str(line);
				iss >> std::skipws >> s >> i_tmp;
				iss.clear();
				timing.bike_red_clearance = static_cast<uint8_t>(i_tmp);
				/// empty line
				std::getline(IS_CARD,line);
			}
		}
		else if (line.find("Free_plan") == 0)
		{
			auto& freeplan = timingcard.freeplan;
			//  lag_phases
			std::getline(IS_CARD,line);
			iss.str(line);
			iss >> std::skipws >> s >> s;
			iss.clear();
			freeplan.lag_phases = std::bitset<8>(s);
			//  omit_phases
			std::getline(IS_CARD,line);
			iss.str(line);
			iss >> std::skipws >> s >> s;
			iss.clear();
			freeplan.omit_phases = std::bitset<8>(s);
			//  minimum_recall_phases
			std::getline(IS_CARD,line);
			iss.str(line);
			iss >> std::skipws >> s >> s;
			iss.clear();
			freeplan.minimum_recall_phases = std::bitset<8>(s);
			//  maximum_recall_phases
			std::getline(IS_CARD,line);
			iss.str(line);
			iss >> std::skipws >> s >> s;
			iss.clear();
			freeplan.maximum_recall_phases = std::bitset<8>(s);
			//  ped_recall_phases
			std::getline(IS_CARD,line);
			iss.str(line);
			iss >> std::skipws >> s >> s;
			iss.clear();
			freeplan.ped_recall_phases = std::bitset<8>(s);
			//  bike_recall_phases
			std::getline(IS_CARD,line);
			iss.str(line);
			iss >> std::skipws >> s >> s;
			iss.clear();
			freeplan.bike_recall_phases = std::bitset<8>(s);
			//  conditional_service_phases
			std::getline(IS_CARD,line);
			iss.str(line);
			iss >> std::skipws >> s >> s;
			iss.clear();
			freeplan.conditional_service_phases = std::bitset<8>(s);
			//  conditional_service_minimum_green
			std::getline(IS_CARD,line);
			iss.str(line);
			iss >> std::skipws >> s >> i_tmp;
			iss.clear();
			freeplan.conditional_service_minimum_green = static_cast<uint8_t>(i_tmp);
			//  permitted_phases
			std::getline(IS_CARD,line);
			iss.str(line);
			iss >> std::skipws >> s >> s;
			iss.clear();
			freeplan.permitted_phases = std::bitset<8>(s);
			//  permitted_ped_phases
			std::getline(IS_CARD,line);
			iss.str(line);
			iss >> std::skipws >> s >> s;
			iss.clear();
			freeplan.permitted_ped_phases = std::bitset<8>(s);
			//  leadlag_phases
			unsigned int ds[8];
			std::getline(IS_CARD,line);
			iss.str(line);
			iss >> std::skipws >> s >> ds[0] >> ds[1] >> ds[2] >> ds[3] >> ds[4] >> ds[5] >> ds[6] >> ds[7];
			iss.clear();
			int count = 0;
			for (int barrier = 0; barrier < 2; barrier++)
				for (int ring = 0; ring < 2; ring++)
					for (int seq = 0; seq < 2; seq++)
						freeplan.leadlag_phases[barrier][ring][seq] = static_cast<uint8_t>(ds[count++]);
			// TSP_conf
			std::string s_tmp;
			std::getline(IS_CARD,line);
			iss.str(line);
			iss >> std::skipws >> s >> s >> s_tmp >> i_tmp;
			iss.clear();
			freeplan.isTSPenabled = (s == "true");
			freeplan.TSP_hold_phases = std::bitset<8>(s_tmp);
			freeplan.TSP_max_green_hold_time = static_cast<uint8_t>(i_tmp);
		}
		else if (line.find("Manual_plan") == 0)
		{
			auto& manualplan = timingcard.manualplan;
			unsigned int ds[2];
			iss.str(line);
			iss >> std::skipws >> s >> s >> ds[0] >> ds[1];
			iss.clear();
			manualplan.planOn = (s == "true");
			manualplan.plan_num = static_cast<uint8_t>(ds[0]);
			manualplan.offset_index = static_cast<uint8_t>(ds[1]);
		}
		else if (line.find("CIC_plan") == 0)
		{
			auto& cicplans = timingcard.cicplans;
			unsigned int ds[16];
			iss.str(line);
			iss >> std::skipws >> s >> s >> ds[0] >> ds[1] >> ds[2] >> ds[3] >> ds[4] >> ds[5] >> ds[6];
			iss.clear();
			cicplans.enabled_plans = std::bitset<9>(s);
			cicplans.smoothing_volume = static_cast<uint8_t>(ds[0]);
			cicplans.smoothing_occupancy = static_cast<uint8_t>(ds[1]);
			cicplans.smoothing_demand = static_cast<uint8_t>(ds[2]);
			cicplans.multiplier_volume = static_cast<uint8_t>(ds[3]);
			cicplans.multiplier_occupancy = static_cast<uint8_t>(ds[4]);
			cicplans.exponent_volume = static_cast<uint8_t>(ds[5]);
			cicplans.exponent_occupancy = static_cast<uint8_t>(ds[6]);
			std::getline(IS_CARD,line);
			iss.str(line);
			iss >> std::skipws >> ds[0] >> ds[1] >> ds[2] >> ds[3] >> ds[4] >> ds[5] >> ds[6] >> ds[7];
			iss >> ds[8] >> ds[9] >> ds[10] >> ds[11] >> ds[12] >> ds[13] >> ds[14] >> ds[15];
			iss.clear();
			for (int i = 0; i< 16; i++)
				cicplans.phase_assignment[i] = static_cast<uint8_t>(ds[i]);
		}
		else if (line.find("detector_conf") == 0)
		{
			unsigned int detnums;
			iss.str(line);
			iss >> std::skipws >> s >> detnums;
			iss.clear();
			if (detnums > 0)
			{
				timingcard.detectorconf.resize(detnums);
				unsigned int ds[7];
				for (unsigned int i = 0; i < detnums; i++)
				{
					auto& detector = timingcard.detectorconf[i];
					std::getline(IS_CARD,line);
					iss.str(line);
					iss >> std::skipws >> ds[0] >> ds[1] >> s >> ds[2] >> ds[3] >> ds[4] >> ds[5] >> ds[6];
					iss.clear();
					detector.id = static_cast<uint8_t>(ds[0]);
					detector.type = static_cast<uint8_t>(ds[1]);
					detector.phaseAssignment = std::bitset<8>(s);
					detector.lock = static_cast<uint8_t>(ds[2]);
					detector.delayTime = static_cast<uint8_t>(ds[3]);
					detector.extendTime = static_cast<uint8_t>(ds[4]);
					detector.recallTime = static_cast<uint8_t>(ds[5]);
					detector.inputPort = static_cast<uint8_t>(ds[6]);
				}
			}
		}
		else if (line.find("system_detectors") == 0)
		{
			auto& detector = timingcard.system_detector_assignment;
			unsigned int ds[16];
			iss.str(line);
			iss >> std::skipws >> s >> ds[0] >> ds[1] >> s;
			iss.clear();
			detector.maxOnTime = static_cast<uint8_t>(ds[0]);
			detector.failResetTime = static_cast<uint8_t>(ds[1]);
			detector.failOverride = std::bitset<44>(s);
			std::getline(IS_CARD,line);
			iss.str(line);
			iss >> std::skipws >> ds[0] >> ds[1] >> ds[2] >> ds[3] >> ds[4] >> ds[5] >> ds[6] >> ds[7];
			iss >> ds[8] >> ds[9] >> ds[10] >> ds[11] >> ds[12] >> ds[13] >> ds[14] >> ds[15];
			iss.clear();
			for (int i = 0; i < 16; i++)
				detector.detectorInput[i] = static_cast<uint8_t>(ds[i]);
		}
		else if (line.find("TODtables") == 0)
		{
			unsigned int tablerows;
			iss.str(line);
			iss >> std::skipws >> s >> tablerows;
			iss.clear();
			if (tablerows > 0)
			{
				timingcard.TODtables.resize(tablerows);
				unsigned int ds[5];
				for (unsigned int i = 0; i < tablerows; i++)
				{
					auto& table = timingcard.TODtables[i];
					std::getline(IS_CARD,line);
					iss.str(line);
					iss >> std::skipws >> ds[0] >> ds[1] >> ds[2] >> ds[3] >> ds[4];
					iss.clear();
					table.table_num = static_cast<uint8_t>(ds[0]);
					table.start_hour = static_cast<uint8_t>(ds[1]);
					table.start_min = static_cast<uint8_t>(ds[2]);
					table.plan_num = static_cast<uint8_t>(ds[3]);
					table.offset_index = static_cast<uint8_t>(ds[4]);
				}
			}
		}
		else if (line.find("weekday_tables") == 0)
		{
			unsigned int ds[7];
			iss.str(line);
			iss >> std::skipws >> s >> ds[0] >> ds[1] >> ds[2] >> ds[3] >> ds[4] >> ds[5] >> ds[6];
			iss.clear();
			for (int i = 0; i < 7; i++)
				timingcard.weekday_plan_assignment[i] = static_cast<uint8_t>(ds[i]);
		}
		else if (line.find("TODfunctions") == 0)
		{
			unsigned functionrows;
			iss.str(line);
			iss >> std::skipws >> s >> functionrows;
			iss.clear();
			if (functionrows > 0)
			{
				timingcard.TODfunctions.resize(functionrows);
				unsigned int ds[5];
				std::string s_tmp;
				for (unsigned int i = 0; i < functionrows; i++)
				{
					auto& func = timingcard.TODfunctions[i];
					std::getline(IS_CARD,line);
					iss.str(line);
					iss >> std::skipws >> ds[0] >> ds[1] >> ds[2] >> ds[3] >> s >> ds[4] >> s_tmp;
					iss.clear();
					func.start_hour = static_cast<uint8_t>(ds[0]);
					func.start_min = static_cast<uint8_t>(ds[1]);
					func.end_hour = static_cast<uint8_t>(ds[2]);
					func.end_min = static_cast<uint8_t>(ds[3]);
					func.dayofweek = std::bitset<8>(s);
					func.action_code = static_cast<uint8_t>(ds[4]);
					func.affect_phases = std::bitset<8>(s_tmp);
				}
			}
		}
		else if (line.find("EVpreemption") == 0)
		{
			std::string s_tmp;
			unsigned int ds[6];
			for (int i = 0; i < 4; i++)
			{
				auto& ev = timingcard.EVpreemption[i];
				std::getline(IS_CARD,line);
				iss.str(line);
				iss >> std::skipws >> ds[0] >> ds[1] >> ds[2] >> s >> s_tmp >> ds[3] >> ds[4] >> ds[5];
				iss.clear();
				ev.delay_time = static_cast<uint8_t>(ds[0]);
				ev.green_hold_time = static_cast<uint8_t>(ds[1]);
				ev.maximum_clearance_time = static_cast<uint8_t>(ds[2]);
				ev.clearance_phase_green = std::bitset<8>(s);
				ev.clearance_overlap_green = std::bitset<8>(s_tmp);
				ev.input_port = static_cast<uint8_t>(ds[3]);
				ev.latching_flag = static_cast<uint8_t>(ds[4]);
				ev.phase_termination_flag = static_cast<uint8_t>(ds[5]);
			}
		}
		else if (line.find("RRpreemption") == 0)
		{
			std::string s_tmp[9];
			unsigned int ds[8];
			for (int i = 0; i < 2; i++)
			{
				auto& rr = timingcard.RRpreemption[i];
				std::getline(IS_CARD,line);
				iss.str(line);
				iss >> std::skipws >> ds[0] >> ds[1] >> ds[2] >> ds[3] >> s_tmp[0] >> s_tmp[1];
				iss	>> s_tmp[2] >> s_tmp[3] >> ds[4] >> ds[5] >> ds[6] >> ds[7];
				iss.clear();
				rr.delay_time = static_cast<uint8_t>(ds[0]);
				rr.minimum_green = static_cast<uint8_t>(ds[1]);
				rr.ped_clear_time = static_cast<uint8_t>(ds[2]);
				rr.exit_time = static_cast<uint8_t>(ds[3]);
				rr.exit_phases_green = std::bitset<8>(s_tmp[0]);
				rr.exit_overlaps_green = std::bitset<8>(s_tmp[1]);
				rr.exit_veh_call = std::bitset<8>(s_tmp[2]);
				rr.exit_ped_call = std::bitset<8>(s_tmp[3]);
				rr.input_port = static_cast<uint8_t>(ds[4]);
				rr.gate_port = static_cast<uint8_t>(ds[5]);
				rr.latching_flag = static_cast<uint8_t>(ds[6]);
				rr.power_up = static_cast<uint8_t>(ds[7]);
				for (int j = 0; j < 4; j++)
				{
					auto& steps = rr.RRpreemption_steps[j];
					std::getline(IS_CARD,line);
					iss.str(line);
					iss >> std::skipws >> i_tmp >> s_tmp[0] >> s_tmp[1] >> s_tmp[2] >> s_tmp[3];
					iss	>> s_tmp[4] >> s_tmp[5] >> s_tmp[6] >> s_tmp[7] >> s_tmp[8];
					iss.clear();
					steps.step_time = static_cast<uint8_t>(i_tmp);
					steps.ped_walk_phases = std::bitset<8>(s_tmp[0]);
					steps.ped_clear_phases = std::bitset<8>(s_tmp[1]);
					steps.ped_red_phases = std::bitset<8>(s_tmp[2]);
					steps.green_hold_phases = std::bitset<8>(s_tmp[3]);
					steps.yew_flashing_phases = std::bitset<8>(s_tmp[4]);
					steps.red_flashing_phases = std::bitset<8>(s_tmp[5]);
					steps.green_hold_overlaps = std::bitset<8>(s_tmp[6]);
					steps.yew_flashing_overlaps = std::bitset<8>(s_tmp[7]);
					steps.red_flashing_overlaps = std::bitset<8>(s_tmp[8]);
				}
			}
		}
		else if (line.find("TSPconf") == 0)
		{
			unsigned int ds[11];
			iss.str(line);
			iss >> std::skipws >> s >> s;
			iss.clear();
			timingcard.TSPconf.enable_coordination_plans = std::bitset<18>(s);
			for (int i = 0; i < 18; i++)
			{
				auto& plan = timingcard.TSPconf.TSPplan[i];
				std::getline(IS_CARD,line);
				iss.str(line);
				iss >> std::skipws >> ds[0] >> ds[1] >> ds[2] >> ds[3] >> ds[4] >> ds[5] >> ds[6] >> ds[7] >> ds[8] >> ds[9] >> ds[10];
				iss.clear();
				plan.max_early_green = static_cast<uint8_t>(ds[0]);
				plan.max_green_extension = static_cast<uint8_t>(ds[1]);
				plan.inhibit_cycles = static_cast<uint8_t>(ds[2]);
				for (int j = 0; j < 8; j++)
					plan.green_factor[j] = static_cast<uint8_t>(ds[3+j]);
			}
		}
		else if (line.find("Coordination_plans") == 0)
		{
			unsigned int plannums;
			iss.str(line);
			iss >> std::skipws >> s >> plannums;
			iss.clear();
			if (plannums > 0)
			{
				timingcard.coordplans.resize(plannums);
				unsigned int ds[8];
				for (unsigned int plan_idx = 0; plan_idx < plannums; plan_idx++)
				{
					auto& plan = timingcard.coordplans[plan_idx];
					// plan_num
					std::getline(IS_CARD,line);
					iss.str(line);
					iss >> std::skipws >> s >> i_tmp;
					iss.clear();
					plan.plan_num = static_cast<uint8_t>(i_tmp);
					// cycle_length
					std::getline(IS_CARD,line);
					iss.str(line);
					iss >> std::skipws >> s >> i_tmp;
					iss.clear();
					plan.cycle_length = static_cast<uint8_t>(i_tmp);
					// green_factor
					std::getline(IS_CARD,line);
					iss.str(line);
					iss >> std::skipws >> s >> ds[0] >> ds[1] >> ds[2] >> ds[3] >> ds[4] >> ds[5] >> ds[6] >> ds[7];
					iss.clear();
					for (int i = 0; i < 8; i++)
						plan.green_factor[i] = static_cast<uint8_t>(ds[i]);
					// cycle_multiplier
					std::getline(IS_CARD,line);
					iss.str(line);
					iss >> std::skipws >> s >> i_tmp;
					iss.clear();
					plan.cycle_multiplier = static_cast<uint8_t>(i_tmp);
					// offsets
					std::getline(IS_CARD,line);
					iss.str(line);
					iss >> std::skipws >> s >> ds[0] >> ds[1] >> ds[2];
					iss.clear();
					for (int i = 0; i < 3; i++)
						plan.offsets[i] = static_cast<uint8_t>(ds[i]);
					// laggapout_phase
					std::getline(IS_CARD,line);
					iss.str(line);
					iss >> std::skipws >> s >> i_tmp;
					iss.clear();
					plan.laggapout_phase = static_cast<uint8_t>(i_tmp);
					// lag_phases
					std::getline(IS_CARD,line);
					iss.str(line);
					iss >> std::skipws >> s >> s;
					iss.clear();
					plan.lag_phases = std::bitset<8>(s);
					// sync_phases
					std::getline(IS_CARD,line);
					iss.str(line);
					iss >> std::skipws >> s >> s;
					iss.clear();
					plan.sync_phases = std::bitset<8>(s);
					// hold_phases
					std::getline(IS_CARD,line);
					iss.str(line);
					iss >> std::skipws >> s >> s;
					iss.clear();
					plan.hold_phases = std::bitset<8>(s);
					// omit_phases
					std::getline(IS_CARD,line);
					iss.str(line);
					iss >> std::skipws >> s >> s;
					iss.clear();
					plan.omit_phases = std::bitset<8>(s);
					// minimum_recall_phases
					std::getline(IS_CARD,line);
					iss.str(line);
					iss >> std::skipws >> s >> s;
					iss.clear();
					plan.minimum_recall_phases = std::bitset<8>(s);
					// maximum_recall_phases
					std::getline(IS_CARD,line);
					iss.str(line);
					iss >> std::skipws >> s >> s;
					iss.clear();
					plan.maximum_recall_phases = std::bitset<8>(s);
					// ped_recall_phases
					std::getline(IS_CARD,line);
					iss.str(line);
					iss >> std::skipws >> s >> s;
					iss.clear();
					plan.ped_recall_phases = std::bitset<8>(s);
					// bike_recall_phases
					std::getline(IS_CARD,line);
					iss.str(line);
					iss >> std::skipws >> s >> s;
					iss.clear();
					plan.bike_recall_phases = std::bitset<8>(s);
					// force_off_flag
					std::getline(IS_CARD,line);
					iss.str(line);
					iss >> std::skipws >> s >> i_tmp;
					iss.clear();
					plan.force_off_flag = static_cast<uint8_t>(i_tmp);
					// permitted_phases
					std::getline(IS_CARD,line);
					iss.str(line);
					iss >> std::skipws >> s >> s;
					iss.clear();
					plan.permitted_phases = std::bitset<8>(s);
					// permitted_ped_phases
					std::getline(IS_CARD,line);
					iss.str(line);
					iss >> std::skipws >> s >> s;
					iss.clear();
					plan.permitted_ped_phases = std::bitset<8>(s);
					// coordinated_phases
					std::getline(IS_CARD,line);
					iss.str(line);
					iss >> std::skipws >> s >> ds[0] >> ds[1];
					iss.clear();
					plan.coordinated_phases[0] = static_cast<uint8_t>(ds[0]);
					plan.coordinated_phases[1] = static_cast<uint8_t>(ds[1]);
					// leadLagMode
					std::getline(IS_CARD,line);
					iss.str(line);
					iss >> std::skipws >> s >> i_tmp;
					iss.clear();
					plan.leadLagMode = static_cast<Card::LeadLagType>(i_tmp);
					// sync_ring
					std::getline(IS_CARD,line);
					iss.str(line);
					iss >> std::skipws >> s >> i_tmp;
					iss.clear();
					plan.sync_ring = static_cast<uint8_t>(i_tmp);
					// sync_barrier
					std::getline(IS_CARD,line);
					iss.str(line);
					iss >> std::skipws >> s >> i_tmp;
					iss.clear();
					plan.sync_barrier = static_cast<uint8_t>(i_tmp);
					//  leadlag_phases
					std::getline(IS_CARD,line);
					iss.str(line);
					iss >> std::skipws >> s >> ds[0] >> ds[1] >> ds[2] >> ds[3] >> ds[4] >> ds[5] >> ds[6] >> ds[7];
					iss.clear();
					int count = 0;
					for (int barrier = 0; barrier < 2; barrier++)
						for (int ring = 0; ring < 2; ring++)
							for (int seq = 0; seq < 2; seq++)
								plan.leadlag_phases[barrier][ring][seq] = static_cast<uint8_t>(ds[count++]);
					// force_off
					std::getline(IS_CARD,line);
					iss.str(line);
					iss >> std::skipws >> s >> ds[0] >> ds[1] >> ds[2] >> ds[3] >> ds[4] >> ds[5] >> ds[6] >> ds[7];
					iss.clear();
					for (int i = 0; i < 8; i++)
						plan.force_off[i] = static_cast<uint8_t>(ds[i]);
					// permissive
					std::getline(IS_CARD,line);
					iss.str(line);
					iss >> std::skipws >> s >> ds[0] >> ds[1] >> ds[2] >> ds[3] >> ds[4] >> ds[5] >> ds[6] >> ds[7];
					iss.clear();
					for (int i = 0; i < 8; i++)
						plan.permissive[i] = static_cast<uint8_t>(ds[i]);
					// ped_permissive
					std::getline(IS_CARD,line);
					iss.str(line);
					iss >> std::skipws >> s >> ds[0] >> ds[1] >> ds[2] >> ds[3] >> ds[4] >> ds[5] >> ds[6] >> ds[7];
					iss.clear();
					for (int i = 0; i < 8; i++)
						plan.ped_permissive[i] = static_cast<uint8_t>(ds[i]);
					//  noncoordBarrierGreenOnset
					std::getline(IS_CARD,line);
					iss.str(line);
					iss >> std::skipws >> s >> plan.noncoordBarrierGreenOnset;
					iss.clear();
					//  coordBarrierGreenOnset
					std::getline(IS_CARD,line);
					iss.str(line);
					iss >> std::skipws >> s >> plan.coordBarrierGreenOnset;
					iss.clear();
					//  coordPhaseGreenOnset
					std::getline(IS_CARD,line);
					iss.str(line);
					iss >> std::skipws >> s >> plan.coordPhaseGreenOnset[0] >> plan.coordPhaseGreenOnset[1];
					iss.clear();
					//  coordPhaseGreenEnd
					std::getline(IS_CARD,line);
					iss.str(line);
					iss >> std::skipws >> s >> s >> plan.coordPhaseGreenEnd[0] >> plan.coordPhaseGreenEnd[1];
					iss.clear();
					plan.coordLagphaseGapout = (s == "true");
					//  isTSPenabled
					std::getline(IS_CARD,line);
					iss.str(line);
					iss >> std::skipws >> s >> s >> ds[0] >> ds[1] >> ds[2];
					iss.clear();
					plan.isTSPenabled = (s == "true");
					plan.max_early_green = static_cast<uint8_t>(ds[0]);
					plan.max_green_extension = static_cast<uint8_t>(ds[1]);
					plan.inhibit_cycles = static_cast<uint8_t>(ds[2]);
					// TSP_force_off
					std::getline(IS_CARD,line);
					iss.str(line);
					iss >> std::skipws >> s >> ds[0] >> ds[1] >> ds[2] >> ds[3] >> ds[4] >> ds[5] >> ds[6] >> ds[7];
					iss.clear();
					for (int i = 0; i < 8; i++)
						plan.TSP_force_off[i] = static_cast<uint8_t>(ds[i]);
					// empty line
					std::getline(IS_CARD,line);
				}
			}
		}
	}
	IS_CARD.close();

	if ((timingcard.controller_addr == 0) || timingcard.phaseflags.permitted_phases.none()
			|| (timingcard.coordplans.empty() && !timingcard.manualplan.planOn))
	{
		initiated = false;
		timingcard.reset();
		return(false);
	}
	initiated = true;
	return(true);
}

void parsePhaseFlags(const std::vector<uint8_t>& buf, Card::phaseflags_mess_t& phaseflags)
{
	int offset = 7;
	phaseflags.permitted_phases       = std::bitset<8>(buf[offset++]);
	phaseflags.restricted_phases      = std::bitset<8>(buf[offset++]);
	phaseflags.minimum_recall_phases  = std::bitset<8>(buf[offset++]);
	phaseflags.maximum_recall_phases  = std::bitset<8>(buf[offset++]);
	phaseflags.ped_recall_phases      = std::bitset<8>(buf[offset++]);
	phaseflags.bike_recall_phases     = std::bitset<8>(buf[offset++]);
	phaseflags.redlock_phases         = std::bitset<8>(buf[offset++]);
	phaseflags.yewlock_phases         = std::bitset<8>(buf[offset++]);
	phaseflags.fomaxlock_phases       = std::bitset<8>(buf[offset++]);
	phaseflags.doubleEntry_phases     = std::bitset<8>(buf[offset++]);
	phaseflags.restInWalk_phases      = std::bitset<8>(buf[offset++]);
	phaseflags.restInRed_phases       = std::bitset<8>(buf[offset++]);
	phaseflags.walk2_phases           = std::bitset<8>(buf[offset++]);
	phaseflags.maxgreen2_phases       = std::bitset<8>(buf[offset++]);
	phaseflags.maxgreen3_phases       = std::bitset<8>(buf[offset++]);
	phaseflags.startup_green_phases   = std::bitset<8>(buf[offset++]);
	phaseflags.startup_yellow_phases  = std::bitset<8>(buf[offset++]);
	phaseflags.startup_vehCalls       = std::bitset<8>(buf[offset++]);
	phaseflags.startup_pedCalls       = std::bitset<8>(buf[offset++]);
	phaseflags.startup_yellowOverlaps = std::bitset<8>(buf[offset++]);
	phaseflags.startup_allred         = buf[offset];
}

void parsePermittedPedPhases(const std::vector<uint8_t>& buf, std::bitset<8>& permitted_ped_phases)
{
	permitted_ped_phases.reset();
	for (int i = 7; i < 15; i++)
		permitted_ped_phases |= std::bitset<8>(buf[i]);
}

void parsePhaseTiming(const std::vector<uint8_t>& buf, Card::phasetiming_mess_t& phasetiming)
{
	int offset = 6;
	phasetiming.phase_num                 = buf[offset++];
	phasetiming.walk1_interval            = buf[offset++];
	phasetiming.walk_clearance            = buf[offset++];
	phasetiming.minimum_green             = buf[offset++];
	phasetiming.detector_limit            = buf[offset++];
	phasetiming.maximum_initial           = buf[offset++];
	phasetiming.maximum_extensions[0]     = buf[offset++];
	phasetiming.maximum_extensions[1]     = buf[offset++];
	phasetiming.maximum_extensions[2]     = buf[offset++];
	phasetiming.passage                   = buf[offset++];
	phasetiming.maximum_gap               = buf[offset++];
	phasetiming.minimum_gap               = buf[offset++];
	phasetiming.added_initial_per_vehicle = buf[offset++];
	phasetiming.reduce_gap_by             = buf[offset++];
	phasetiming.reduce_gap_every          = buf[offset++];
	phasetiming.yellow_interval           = buf[offset++];
	phasetiming.red_clearance             = buf[offset++];
	phasetiming.walk2_interval            = buf[offset++];
	phasetiming.delay_early_walk_time     = buf[offset++];
	phasetiming.solid_walk_clearance      = buf[offset++];
	phasetiming.bike_green                = buf[offset++];
	phasetiming.bike_red_clearance        = buf[offset];
}

void parseFreePlanTiming(const std::vector<uint8_t>& buf, Card::freeplan_mess_t& freeplan, Card::manualplan_mess_t& manualplan)
{
	int offset = 15;
	freeplan.lag_phases                        = std::bitset<8>(buf[offset++]);
	freeplan.omit_phases                       = std::bitset<8>(buf[offset++]);
	freeplan.minimum_recall_phases             = std::bitset<8>(buf[offset++]);
	freeplan.maximum_recall_phases             = std::bitset<8>(buf[offset++]);
	freeplan.ped_recall_phases                 = std::bitset<8>(buf[offset++]);
	freeplan.bike_recall_phases                = std::bitset<8>(buf[offset++]);
	freeplan.conditional_service_phases        = std::bitset<8>(buf[offset++]);
	freeplan.conditional_service_minimum_green = buf[offset++];
	manualplan.plan_num                        = buf[offset++];
	manualplan.offset_index = static_cast<uint8_t>((buf[offset] < 10) ? buf[offset] : (buf[offset] - 10));
	manualplan.planOn = (manualplan.plan_num == 0) ? false : true;
}

void parseCICplans(const std::vector<uint8_t>& buf, Card::cicplan_mess_t& cicplans)
{	// CIC Enable in Plans: Byte #8: Bit 0 = plan 9, Byte #9: Bits 0-7 = plans 1-8
	cicplans.enabled_plans = std::bitset<9>(((buf[7] & 0x01) << 8) | buf[8]);
	int offset = 9;
	cicplans.smoothing_volume      = buf[offset++];
	cicplans.smoothing_occupancy   = buf[offset++];
	cicplans.smoothing_demand      = buf[offset++];
	cicplans.multiplier_volume     = buf[offset++];
	cicplans.multiplier_occupancy  = buf[offset++];
	cicplans.exponent_volume       = buf[offset++];
	cicplans.exponent_occupancy    = buf[offset++];
	for (int i = 0; i < 16; i++)
		cicplans.phase_assignment[i] = buf[offset++];
}

void parseDetectorConf(const std::vector<uint8_t>& buf, std::vector<Card::detectorconf_mess_t>& detectorconf)
{
	Card::detectorconf_mess_t detectorconf_mess;
	int offset = 6;
	uint8_t detectorId = (uint8_t)((buf[offset++] - 1) * 4);
	for (int i = 0; i < 4; i++)
	{
		detectorconf_mess.id              = ++detectorId;
		detectorconf_mess.type            = buf[offset++];
		detectorconf_mess.phaseAssignment = std::bitset<8>(buf[offset++]);
		detectorconf_mess.lock            = buf[offset++];
		detectorconf_mess.delayTime       = buf[offset++];
		detectorconf_mess.extendTime      = buf[offset++];
		detectorconf_mess.recallTime      = buf[offset++];
		detectorconf_mess.inputPort       = buf[offset++];
		auto it = std::find_if(detectorconf.begin(), detectorconf.end(),
			[&detectorId](const Card::detectorconf_mess_t& obj){return(obj.id == detectorId);});
		if (it == detectorconf.end())
			detectorconf.push_back(detectorconf_mess);
	}
}

void parseDetectorAssignment(const std::vector<uint8_t>& buf, Card::system_detector_assignment_mess_t& assignment)
{
	assignment.maxOnTime = buf[7];
	assignment.failResetTime = buf[8];
	// 1-8, 9-16, 17-24, 25-32, 33-40, 41-44
	assignment.failOverride = std::bitset<44>(buf[14] & 0x0F);
	for (int i = 13; i > 8; i--)
		(assignment.failOverride <<= 8) ^= std::bitset<44>(buf[i]);
	int offset = 15;
	for (int i = 0; i < 16; i++)
		assignment.detectorInput[i] = buf[offset++];
}

void parseTODtables(const std::vector<uint8_t>& buf, std::vector<Card::TODtable_mess_t>& TODtables)
{
	Card::TODtable_mess_t TODtable_mess;
	int offset = 6;
	TODtable_mess.table_num = static_cast<uint8_t>((buf[offset++] - 1)/4 + 1);
	for (int i = 0; i < 4; i++)
	{
		TODtable_mess.start_hour   = buf[offset++];
		TODtable_mess.start_min    = buf[offset++];
		TODtable_mess.plan_num     = buf[offset++];
		TODtable_mess.offset_index = buf[offset++];
		if (TODtable_mess.offset_index >= 10)
			TODtable_mess.offset_index = static_cast<uint8_t>(TODtable_mess.offset_index  - 10);
		if (!((TODtable_mess.start_hour == 0) && (TODtable_mess.start_min == 0) && (TODtable_mess.plan_num == 0)))
		{
			auto it = std::find_if(TODtables.begin(), TODtables.end(),
				[&TODtable_mess](const Card::TODtable_mess_t& obj) {return(obj == TODtable_mess);});
			if (it == TODtables.end())
				TODtables.push_back(TODtable_mess);
		}
	}
}

void parseDOWtable(const std::vector<uint8_t>& buf, uint8_t (&assignment)[7])
{
	int offset = 7;
	for (int i = 0; i < 7; i++)
		assignment[i] = buf[offset++];
}

void parseTODfunctions(const std::vector<uint8_t>& buf, std::vector<Card::TODfunction_mess_t>& TODfunctions)
{
	Card::TODfunction_mess_t TODfunction_mess;
	int offset = 7;
	for (int i = 0; i < 4; i++)
	{
		TODfunction_mess.start_hour    = buf[offset++];
		TODfunction_mess.start_min     = buf[offset++];
		TODfunction_mess.end_hour      = buf[offset++];
		TODfunction_mess.end_min       = buf[offset++];
		TODfunction_mess.dayofweek     = std::bitset<8>(buf[offset++]);
		TODfunction_mess.action_code   = buf[offset++];
		TODfunction_mess.affect_phases = std::bitset<8>(buf[offset++]);
		if (TODfunction_mess.action_code > 0)
		{
			auto it = std::find_if(TODfunctions.begin(), TODfunctions.end(),
				[&TODfunction_mess](const Card::TODfunction_mess_t& obj) {return(obj == TODfunction_mess);});
			if (it == TODfunctions.end())
				TODfunctions.push_back(TODfunction_mess);
		}
	}
}

void parseEVpreempt(const std::vector<uint8_t>& buf, Card::EVpreemption_mess_t& EVpreemption)
{
	int offset = 7;
	EVpreemption.delay_time              = buf[offset++];
	EVpreemption.green_hold_time         = buf[offset++];
	EVpreemption.maximum_clearance_time  = buf[offset++];
	EVpreemption.clearance_phase_green   = std::bitset<8>(buf[offset++]);
	EVpreemption.clearance_overlap_green = std::bitset<8>(buf[offset++]);
	EVpreemption.input_port              = buf[offset++];
	EVpreemption.latching_flag           = buf[offset++];
	EVpreemption.phase_termination_flag  = buf[offset++];
}

void parseRRpreempt(const std::vector<uint8_t>& buf, Card::RRpreemption_mess_t& RRpreemption, int step)
{
	int offset = 7;
	switch(step)
	{
	case 0:
		RRpreemption.RRpreemption_steps[0].green_hold_phases   = std::bitset<8>(buf[offset++]);
		RRpreemption.RRpreemption_steps[0].yew_flashing_phases = std::bitset<8>(buf[offset++]);
		RRpreemption.RRpreemption_steps[0].red_flashing_phases = std::bitset<8>(buf[offset++]);
		RRpreemption.RRpreemption_steps[1].green_hold_phases   = std::bitset<8>(buf[offset++]);
		RRpreemption.RRpreemption_steps[1].yew_flashing_phases = std::bitset<8>(buf[offset++]);
		RRpreemption.RRpreemption_steps[1].red_flashing_phases = std::bitset<8>(buf[offset++]);
		RRpreemption.RRpreemption_steps[2].green_hold_phases   = std::bitset<8>(buf[offset++]);
		RRpreemption.RRpreemption_steps[2].yew_flashing_phases = std::bitset<8>(buf[offset++]);
		RRpreemption.RRpreemption_steps[2].red_flashing_phases = std::bitset<8>(buf[offset++]);
		RRpreemption.RRpreemption_steps[3].green_hold_phases   = std::bitset<8>(buf[offset++]);
		RRpreemption.RRpreemption_steps[3].yew_flashing_phases = std::bitset<8>(buf[offset++]);
		RRpreemption.RRpreemption_steps[3].red_flashing_phases = std::bitset<8>(buf[offset++]);
		break;
	case 1:
		RRpreemption.RRpreemption_steps[0].ped_walk_phases  = std::bitset<8>(buf[offset++]);
		RRpreemption.RRpreemption_steps[0].ped_clear_phases = std::bitset<8>(buf[offset++]);
		RRpreemption.RRpreemption_steps[0].ped_red_phases   = std::bitset<8>(buf[offset++]);
		RRpreemption.RRpreemption_steps[1].ped_walk_phases  = std::bitset<8>(buf[offset++]);
		RRpreemption.RRpreemption_steps[1].ped_clear_phases = std::bitset<8>(buf[offset++]);
		RRpreemption.RRpreemption_steps[1].ped_red_phases   = std::bitset<8>(buf[offset++]);
		RRpreemption.RRpreemption_steps[2].ped_walk_phases  = std::bitset<8>(buf[offset++]);
		RRpreemption.RRpreemption_steps[2].ped_clear_phases = std::bitset<8>(buf[offset++]);
		RRpreemption.RRpreemption_steps[2].ped_red_phases   = std::bitset<8>(buf[offset++]);
		RRpreemption.RRpreemption_steps[3].ped_walk_phases  = std::bitset<8>(buf[offset++]);
		RRpreemption.RRpreemption_steps[3].ped_clear_phases = std::bitset<8>(buf[offset++]);
		RRpreemption.RRpreemption_steps[3].ped_red_phases   = std::bitset<8>(buf[offset++]);
		break;
	case 2:
		RRpreemption.RRpreemption_steps[0].green_hold_overlaps   = std::bitset<8>(buf[offset++]);
		RRpreemption.RRpreemption_steps[0].yew_flashing_overlaps = std::bitset<8>(buf[offset++]);
		RRpreemption.RRpreemption_steps[0].red_flashing_overlaps = std::bitset<8>(buf[offset++]);
		RRpreemption.RRpreemption_steps[1].green_hold_overlaps   = std::bitset<8>(buf[offset++]);
		RRpreemption.RRpreemption_steps[1].yew_flashing_overlaps = std::bitset<8>(buf[offset++]);
		RRpreemption.RRpreemption_steps[1].red_flashing_overlaps = std::bitset<8>(buf[offset++]);
		RRpreemption.RRpreemption_steps[2].green_hold_overlaps   = std::bitset<8>(buf[offset++]);
		RRpreemption.RRpreemption_steps[2].yew_flashing_overlaps = std::bitset<8>(buf[offset++]);
		RRpreemption.RRpreemption_steps[2].red_flashing_overlaps = std::bitset<8>(buf[offset++]);
		RRpreemption.RRpreemption_steps[3].green_hold_overlaps   = std::bitset<8>(buf[offset++]);
		RRpreemption.RRpreemption_steps[3].yew_flashing_overlaps = std::bitset<8>(buf[offset++]);
		RRpreemption.RRpreemption_steps[3].red_flashing_overlaps = std::bitset<8>(buf[offset++]);
		break;
	case 3:
		RRpreemption.exit_phases_green   = std::bitset<8>(buf[offset++]);
		RRpreemption.exit_overlaps_green = std::bitset<8>(buf[offset++]);
		RRpreemption.exit_veh_call       = std::bitset<8>(buf[offset++]);
		RRpreemption.exit_ped_call       = std::bitset<8>(buf[offset++]);
		break;
	case 4:
		RRpreemption.delay_time = buf[offset++];
		RRpreemption.RRpreemption_steps[0].step_time = buf[offset++];
		RRpreemption.RRpreemption_steps[1].step_time = buf[offset++];
		RRpreemption.RRpreemption_steps[2].step_time = buf[offset++];
		RRpreemption.RRpreemption_steps[3].step_time = buf[offset++];
		RRpreemption.exit_time      = buf[offset++];
		RRpreemption.minimum_green  = buf[offset++];
		RRpreemption.ped_clear_time = buf[offset++];
		RRpreemption.input_port     = buf[offset++];
		RRpreemption.gate_port      = buf[offset++];
		RRpreemption.latching_flag  = buf[offset++];
		RRpreemption.power_up       = buf[offset++];
		break;
	default:
		break;
	}
}

void parseTSPenablePlans(const std::vector<uint8_t>& buf, Card::TSPconf_mess_t& TSPconf, Card::freeplan_mess_t& freeplan)
{ // Byte #8: Bit 0 = plan 9, Byte #9: Bits 0-7 = plans 1-8
	// Byte #10: Bit 0 = plan 19, Byte #11: Bits 0-7 = plans 11-18
	// plan 10 is skipped
	unsigned long ul = (unsigned long)((buf[7] & 0x01) << 8) | buf[8];
	ul += ((((buf[9] & 0x01) << 8) | buf[10]) << 9);
	TSPconf.enable_coordination_plans = std::bitset<18>(ul);
	freeplan.TSP_max_green_hold_time  = buf[11];
	freeplan.TSP_hold_phases          = std::bitset<8>(buf[12]);
	freeplan.isTSPenabled = (freeplan.TSP_hold_phases.any() && (freeplan.TSP_max_green_hold_time > 0)) ? true : false;
}

void parseTSPplans(const std::vector<uint8_t>& buf, Card::TSPconf_mess_t& TSPconf)
{
	int planGroupIdx = (buf[6] - 2) * 3;
	int offset = 7;
	for (int i = 0; i < 3; i++)
	{
		auto& TSPplan = TSPconf.TSPplan[planGroupIdx + i];
		TSPplan.max_early_green     = buf[offset++];
		TSPplan.max_green_extension = buf[offset++];
		TSPplan.inhibit_cycles      = buf[offset++];
		for (int j = 0; j < 8; j++)
			TSPplan.green_factor[j]   = buf[offset++];
	}
}

void parseCoordPlans(const std::vector<uint8_t>& buf, std::vector<Card::coordplan_mess_t>& coordplans)
{
	Card::coordplan_mess_t coordplan_mess;
	uint8_t plan_num = (uint8_t)((buf[5] - 4) * 10 + buf[6]);
	coordplan_mess.plan_num = plan_num;
	int offset = 7;
	coordplan_mess.cycle_length = buf[offset++];
	if (coordplan_mess.cycle_length > 0)
	{	/// only record non-empty coordination plans
		for (int i = 0; i < 8; i++)
			coordplan_mess.green_factor[i] = buf[offset++];
		coordplan_mess.cycle_multiplier = buf[offset++];
		for (int i = 0; i < 3; i++)
			coordplan_mess.offsets[i]          = buf[offset++];
		coordplan_mess.laggapout_phase       = buf[offset++];
		coordplan_mess.lag_phases            = std::bitset<8>(buf[offset++]);
		coordplan_mess.sync_phases           = std::bitset<8>(buf[offset++]);
		coordplan_mess.hold_phases           = std::bitset<8>(buf[offset++]);
		coordplan_mess.omit_phases           = std::bitset<8>(buf[offset++]);
		coordplan_mess.minimum_recall_phases = std::bitset<8>(buf[offset++]);
		coordplan_mess.maximum_recall_phases = std::bitset<8>(buf[offset++]);
		coordplan_mess.ped_recall_phases     = std::bitset<8>(buf[offset++]);
		coordplan_mess.bike_recall_phases    = std::bitset<8>(buf[offset++]);
		coordplan_mess.force_off_flag        = buf[offset++];
		auto it = std::find_if(coordplans.begin(), coordplans.end(),
			[&plan_num](const Card::coordplan_mess_t& obj) {return(obj.plan_num == plan_num);});
		if (it == coordplans.end())
			coordplans.push_back(coordplan_mess);
	}
}

void Card::updateTimeCard(const std::vector<uint8_t>& buf, const std::string& pollDesc)
{
	if (pollDesc.find("phase flags") == 0)
		parsePhaseFlags(buf, timingcard.phaseflags);
	else if (pollDesc.find("ped flags") == 0)
		parsePermittedPedPhases(buf, timingcard.phaseflags.permitted_ped_phases);
	else if (pollDesc.find("red revert") == 0)
		timingcard.phaseflags.red_revert_interval = buf[8];
	else if (pollDesc.find("phase timing") == 0)
		parsePhaseTiming(buf, timingcard.phasetiming[buf[6]-1]);
	else if (pollDesc.find("free plan") == 0)
		parseFreePlanTiming(buf, timingcard.freeplan, timingcard.manualplan);
	else if (pollDesc.find("CIC plan") == 0)
		parseCICplans(buf, timingcard.cicplans);
	else if (pollDesc.find("detector group") == 0)
		parseDetectorConf(buf, timingcard.detectorconf);
	else if (pollDesc.find("system detector") == 0)
		parseDetectorAssignment(buf, timingcard.system_detector_assignment);
	else if (pollDesc.find("TOD table") == 0)
		parseTODtables(buf, timingcard.TODtables);
	else if (pollDesc.find("weekday") == 0)
		parseDOWtable(buf, timingcard.weekday_plan_assignment);
	else if (pollDesc.find("TOD Function") == 0)
		parseTODfunctions(buf, timingcard.TODfunctions);
	else if (pollDesc.find("EVA") == 0)
		parseEVpreempt(buf, timingcard.EVpreemption[0]);
	else if (pollDesc.find("EVB") == 0)
		parseEVpreempt(buf, timingcard.EVpreemption[1]);
	else if (pollDesc.find("EVC") == 0)
		parseEVpreempt(buf, timingcard.EVpreemption[2]);
	else if (pollDesc.find("EVD") == 0)
		parseEVpreempt(buf, timingcard.EVpreemption[3]);
	else if (pollDesc.find("RR1 phase flags") == 0)
		parseRRpreempt(buf, timingcard.RRpreemption[0], 0);
	else if (pollDesc.find("RR1 ped flags") == 0)
		parseRRpreempt(buf, timingcard.RRpreemption[0], 1);
	else if (pollDesc.find("RR1 overlap flags") == 0)
		parseRRpreempt(buf, timingcard.RRpreemption[0], 2);
	else if (pollDesc.find("RR1 exit parameters") == 0)
		parseRRpreempt(buf, timingcard.RRpreemption[0], 3);
	else if (pollDesc.find("RR1 Configuration") == 0)
		parseRRpreempt(buf, timingcard.RRpreemption[0], 4);
	else if (pollDesc.find("RR2 phase flags") == 0)
		parseRRpreempt(buf, timingcard.RRpreemption[1], 0);
	else if (pollDesc.find("RR2 ped flags") == 0)
		parseRRpreempt(buf, timingcard.RRpreemption[1], 1);
	else if (pollDesc.find("RR2 overlap flags") == 0)
		parseRRpreempt(buf, timingcard.RRpreemption[1], 2);
	else if (pollDesc.find("RR2 exit parameters") == 0)
		parseRRpreempt(buf, timingcard.RRpreemption[1], 3);
	else if (pollDesc.find("RR2 Configuration") == 0)
		parseRRpreempt(buf, timingcard.RRpreemption[1], 4);
	else if (pollDesc.find("TSP enable plans") == 0)
		parseTSPenablePlans(buf, timingcard.TSPconf, timingcard.freeplan);
	else if (pollDesc.find("TSP plan group") == 0)
		parseTSPplans(buf, timingcard.TSPconf);
	else if (pollDesc.find("coord plan") == 0)
		parseCoordPlans(buf, timingcard.coordplans);
}

void getLeadLagPhasebyBarrierAndRing(uint8_t (&leadlag_phases)[2][2][2], const std::bitset<8>& lag_phases,
	const std::bitset<8>& permitted_phases)
{
	std::bitset<8> barrier_phases;
	std::bitset<8> ring_phases;
	std::bitset<8> seq_phases;

	for (uint8_t barrier = 0; barrier < 2; barrier++)
	{
		if (barrier == 0)
			barrier_phases = std::bitset<8>(std::string("00110011"));
		else
			barrier_phases = std::bitset<8>(std::string("11001100"));
		for (uint8_t ring = 0; ring < 2; ring++)
		{
			if (ring == 0)
				ring_phases = std::bitset<8>(std::string("00001111"));
			else
				ring_phases = std::bitset<8>(std::string("11110000"));
			for (uint8_t seq = 0; seq < 2; seq++)
			{
				if (seq == 0)
					seq_phases = (~lag_phases) & permitted_phases & barrier_phases & ring_phases;
				else
					seq_phases = lag_phases & permitted_phases & barrier_phases & ring_phases;
				if (seq_phases.none())
					leadlag_phases[barrier][ring][seq] = 0;
				else
				{
					for (uint8_t i = 0; i < 8; i++)
					{
						if (seq_phases.test(i))
						{
							leadlag_phases[barrier][ring][seq] = static_cast<uint8_t>(i+1);
							break;
						}
					}
				}
			}
			if ((leadlag_phases[barrier][ring][0] == 0) && (leadlag_phases[barrier][ring][1] > 0))
				leadlag_phases[barrier][ring][0] = leadlag_phases[barrier][ring][1];
			else if ((leadlag_phases[barrier][ring][1] == 0) && (leadlag_phases[barrier][ring][0] > 0))
				leadlag_phases[barrier][ring][1] = leadlag_phases[barrier][ring][0];
		}
	}
}

void Card::setFreePlanParameters(void)
{
	auto& freeplan   = timingcard.freeplan;
	const auto& phaseflags = timingcard.phaseflags;
	freeplan.permitted_phases     = (phaseflags.permitted_phases & (~freeplan.omit_phases));
	freeplan.permitted_ped_phases = (phaseflags.permitted_ped_phases & (~freeplan.omit_phases));
	getLeadLagPhasebyBarrierAndRing(freeplan.leadlag_phases, freeplan.lag_phases, freeplan.permitted_phases);
}

Card::LeadLagType getLeadlagOperationMode(const std::bitset<8>& sync_phases, const std::bitset<8>& lag_phases,
	const std::bitset<8>& permitted_phases)
{
	std::bitset<8> sync_lag_phases = (sync_phases & (lag_phases & permitted_phases));
	switch(sync_lag_phases.count())
	{
	case 0:
		return(Card::LeadLagType::leadLead);
		break;
	case 2:
		return(Card::LeadLagType::lagLag);
		break;
	default:
		if (sync_phases.count() == 2)
		{	/// sync phase on both rings are set
			std::bitset<8> check_phases(std::string("00001111"));
			check_phases &= sync_lag_phases;
			if (check_phases.count() == 1)
				return(Card::LeadLagType::lagLead);
			else
				return(Card::LeadLagType::leadLag);
		}
		else
			return(Card::LeadLagType::lagLag);
		break;
	}
}

void getForceOffbyRing(Card::timing_card_t& card, uint8_t (&force_off)[8], const uint8_t (&greenfactor)[8],
	uint32_t& count, uint8_t leadPhase, uint8_t lagPhase, bool greenOnset)
{
	if (leadPhase > 0)
	{
		uint8_t indx = (uint8_t)(leadPhase - 1);
		const auto& phasetiming = card.phasetiming[indx];
		if (greenOnset)
			count += greenfactor[indx] * 10;
		force_off[indx] = static_cast<uint8_t>(count / 10);
		count = force_off[indx] * 10 + phasetiming.yellow_interval + phasetiming.red_clearance;
		if (force_off[indx] == 0)
			count += 10;  // local_cycle clock starts at 1 not 0
	}
	if (lagPhase > 0 && (lagPhase != leadPhase))
	{
		uint8_t indx = (uint8_t)(lagPhase - 1);
		const auto& phasetiming = card.phasetiming[indx];
		count += greenfactor[indx] * 10;
		force_off[indx] = static_cast<uint8_t>(count / 10);
		count = force_off[indx] * 10 +  phasetiming.yellow_interval + phasetiming.red_clearance;
	}
}

void getForceOffPoints(Card::timing_card_t& card, Card::coordplan_mess_t& plan, uint8_t (&force_off)[8], const uint8_t (&greenfactor)[8])
{
	uint32_t count[2] = {0,0};
	uint8_t cur_barrier = plan.sync_barrier;
	for (int i = 0; i < 8; i++)
		force_off[i] = 0;
	/// start with sync_barrier
	for (uint8_t ring = plan.sync_ring; ring < plan.sync_ring + 2; ring++)
	{
		uint8_t cur_ring  = static_cast<uint8_t>(ring % 2);
		uint8_t leadPhase = plan.leadlag_phases[cur_barrier][cur_ring][0];
		uint8_t lagPhase  = plan.leadlag_phases[cur_barrier][cur_ring][1];
		if (plan.leadLagMode == Card::LeadLagType::leadLead)
			getForceOffbyRing(card, force_off, greenfactor, count[cur_ring], leadPhase, lagPhase, false);
		else if (plan.leadLagMode == Card::LeadLagType::lagLag)
			getForceOffbyRing(card, force_off, greenfactor, count[cur_ring], lagPhase, 0, false);
		else if (cur_ring == plan.sync_ring)
			getForceOffbyRing(card, force_off, greenfactor, count[cur_ring], leadPhase, lagPhase, false);
		else
		{
			count[cur_ring] = force_off[plan.leadlag_phases[cur_barrier][next_ring(cur_ring)][1] - 1] * 10;
			getForceOffbyRing(card, force_off, greenfactor, count[cur_ring], lagPhase, 0, false);
		}
	}
	/// move to non sync_barrier
	cur_barrier = next_barrier(plan.sync_barrier);
	count[0] = (count[0] < count[1]) ? count[1] : count[0];
	count[1] = count[0];
	for (uint8_t cur_ring = 0; cur_ring < 2; cur_ring++)
	{
		uint8_t leadPhase = plan.leadlag_phases[cur_barrier][cur_ring][0];
		uint8_t lagPhase  = plan.leadlag_phases[cur_barrier][cur_ring][1];
		getForceOffbyRing(card, force_off, greenfactor, count[cur_ring], leadPhase, lagPhase, true);

	}
	/// move to sync_barrier
	cur_barrier = plan.sync_barrier;
	count[0] = (count[0] < count[1]) ? count[1] : count[0];
	count[1] = count[0];
	for (uint8_t ring = plan.sync_ring; ring < plan.sync_ring + 2; ring++)
	{
		uint8_t cur_ring  = static_cast<uint8_t>(ring % 2);
		uint8_t leadPhase = plan.leadlag_phases[cur_barrier][cur_ring][0];
		uint8_t lagPhase  = plan.leadlag_phases[cur_barrier][cur_ring][1];
		if (plan.leadLagMode == Card::LeadLagType::lagLag)
		{
			if ((leadPhase > 0) && (leadPhase != lagPhase))
				getForceOffbyRing(card, force_off, greenfactor, count[cur_ring], leadPhase, 0, true);
		}
		else if (plan.leadLagMode != Card::LeadLagType::leadLead)
		{
			if ((cur_ring != plan.sync_ring) && (leadPhase > 0) && (leadPhase != lagPhase))
				getForceOffbyRing(card, force_off, greenfactor, count[cur_ring], leadPhase, 0, true);
		}
	}
}

void Card::setCoordPlanParameters(void)
{
	auto& coordplans = timingcard.coordplans;
	const auto& phaseflags = timingcard.phaseflags;
	for (auto& plan : coordplans)
	{
		plan.permitted_phases = (phaseflags.permitted_phases & (~plan.omit_phases));
		plan.permitted_ped_phases = (phaseflags.permitted_ped_phases & (~plan.omit_phases));
		bitset2array(plan.coordinated_phases, plan.sync_phases);
		plan.leadLagMode = getLeadlagOperationMode(plan.sync_phases, plan.lag_phases, plan.permitted_phases);
		plan.sync_ring = ((plan.coordinated_phases[0] == 0) || (plan.leadLagMode == Card::LeadLagType::lagLead)) ? 1 : 0;
		plan.sync_barrier = barrier_phases_on(plan.sync_phases);
		getLeadLagPhasebyBarrierAndRing(plan.leadlag_phases, plan.lag_phases, plan.permitted_phases);
		if (plan.force_off_flag == 1)
		{
			for (int i = 0; i < 8; i++)
				plan.force_off[i] = plan.green_factor[i];
		}
		else
			getForceOffPoints(timingcard, plan, plan.force_off, plan.green_factor);
		for (uint8_t i = 0; i < 8; i++)
		{
			const auto& phasetiming = timingcard.phasetiming[i];
			const auto& force_off = plan.force_off[i];
			auto& permissive     = plan.permissive[i];
			auto& ped_permissive = plan.ped_permissive[i];
			permissive     = 0;
			ped_permissive = 0;
			if (plan.permitted_phases.test(i) && !plan.sync_phases.test(i))
				permissive = static_cast<uint8_t>((force_off > phasetiming.minimum_green) ? (force_off - phasetiming.minimum_green) : 0);
			if (plan.permitted_ped_phases.test(i) && !plan.sync_phases.test(i))
			{
				ped_permissive = static_cast<uint8_t>((force_off > phasetiming.walk1_interval + phasetiming.walk_clearance) ?
					(force_off - phasetiming.walk1_interval - phasetiming.walk_clearance) : 0);
			}
		}
		/// noncoordBarrierGreenOnset - green onset for non-coordinated barrier
		uint32_t count[2];
		for (uint8_t ring = 0; ring < 2; ring++)
		{
			count[ring] = 0;
			uint8_t lagphase = plan.leadlag_phases[plan.sync_barrier][ring][1];
			if (lagphase > 0)
			{
				uint8_t indx = (uint8_t)(lagphase - 1);
				const auto& phasetiming = timingcard.phasetiming[indx];
				const auto& force_off = plan.force_off[indx];
				count[ring] = force_off * 10 + phasetiming.yellow_interval + phasetiming.red_clearance;
			}
		}
		plan.noncoordBarrierGreenOnset = (count[0] > count[1]) ? count[0] : count[1];
		/// coordBarrierGreenOnset - green onset for coordinated barrier
		for (uint8_t ring = 0; ring < 2; ring++)
		{
			count[ring] = 0;
			uint8_t lagphase = plan.leadlag_phases[next_barrier(plan.sync_barrier)][ring][1];
			if (lagphase > 0)
			{
				uint8_t indx = (uint8_t)(lagphase - 1);
				const auto& phasetiming = timingcard.phasetiming[indx];
				const auto& force_off = plan.force_off[indx];
				count[ring] = force_off * 10 + phasetiming.yellow_interval + phasetiming.red_clearance;
			}
		}
		plan.coordBarrierGreenOnset = (count[0] > count[1]) ? count[0] : count[1];
		/// coordPhaseGreenOnset - green onset for coordinated phase by ring
		/// coordPhaseGreenEnd   - yellow onset for coordinated phase by ring
		for (uint8_t ring = 0; ring < 2; ring++)
		{
			plan.coordPhaseGreenOnset[ring] = plan.coordBarrierGreenOnset;
			uint8_t leadphase = plan.leadlag_phases[plan.sync_barrier][ring][0];
			uint8_t syncphase = plan.coordinated_phases[ring];
			if ((syncphase > 0) && plan.lag_phases.test(syncphase-1) && (leadphase > 0) && (leadphase != syncphase))
			{
				uint8_t indx = (uint8_t)(leadphase - 1);
				const auto& phasetiming = timingcard.phasetiming[indx];
				const auto& force_off = plan.force_off[indx];
				plan.coordPhaseGreenOnset[ring] = force_off * 10 + phasetiming.yellow_interval + phasetiming.red_clearance;
			}
			plan.coordPhaseGreenEnd[ring] = (syncphase > 0) ? plan.force_off[syncphase-1] * 10 : 0;
		}
		/// coordLagphaseGapout and adjustment on coordPhaseGreenEnd
		plan.coordLagphaseGapout = false;
		if ((plan.leadLagMode != Card::LeadLagType::leadLead) && (plan.leadLagMode != Card::LeadLagType::lagLag))
		{
			uint8_t lagphase = plan.leadlag_phases[plan.sync_barrier][plan.sync_ring][1];
			uint8_t leadphase = plan.leadlag_phases[plan.sync_barrier][plan.sync_ring][0];
			uint8_t coordlagphase = plan.leadlag_phases[plan.sync_barrier][next_ring(plan.sync_ring)][1];
			if ((lagphase > 0) && (leadphase > 0) && (coordlagphase > 0) && (lagphase == plan.laggapout_phase)
				&& !plan.maximum_recall_phases.test(lagphase - 1) && !plan.maximum_recall_phases.test(lagphase - 1))
			{
				uint8_t indx = (uint8_t)(leadphase - 1);
				const auto& phasetiming = timingcard.phasetiming[indx];
				const auto& force_off = plan.force_off[indx];
				plan.coordLagphaseGapout = true;
				plan.coordPhaseGreenEnd[next_ring(plan.sync_ring)] = force_off * 10 + phasetiming.yellow_interval
					+ phasetiming.red_clearance + timingcard.phasetiming[lagphase - 1].minimum_green * 10;
			}
		}
		/// TSP configuration (for plan 1 to 9 and 11 to 19)
		plan.isTSPenabled = false;
		plan.max_early_green = 0;
		plan.max_green_extension = 0;
		plan.inhibit_cycles = 0;
		for (int i = 0; i < 8; i++)
			plan.TSP_force_off[i] = 0;
		if ((plan.plan_num > 0) && (plan.plan_num < 20))
		{
			int planIdx = (plan.plan_num > 10) ? (plan.plan_num - 2) : (plan.plan_num - 1);
			const auto& TSPplan = timingcard.TSPconf.TSPplan[planIdx];
			plan.max_early_green = TSPplan.max_early_green;
			plan.max_green_extension = TSPplan.max_green_extension;
			plan.inhibit_cycles = TSPplan.inhibit_cycles;
			for (int i = 0; i < 8; i++)
				plan.TSP_force_off[i] = TSPplan.green_factor[i];
			if (timingcard.TSPconf.enable_coordination_plans.test(planIdx))
			{
				plan.isTSPenabled = true;
				uint8_t syncphase = plan.coordinated_phases[plan.sync_ring];
				if (TSPplan.green_factor[syncphase - 1] != 0)
					getForceOffPoints(timingcard, plan, plan.TSP_force_off, TSPplan.green_factor);
			}
		}
	}
}

MsgEnum::controlMode Card::getControlMode(const std::bitset<8>& controller_status,
	const std::bitset<8>& preempt, uint8_t pattern_num) const
{
	std::bitset<8> preemption(std::string("00111111"));
	preemption &= preempt;
	if (controller_status.test(0))
		return(MsgEnum::controlMode::preemption);
	if (controller_status.test(1) || (pattern_num == MsgEnum::patternFlashing))
		return(MsgEnum::controlMode::flashing);
	if ((pattern_num == MsgEnum::patternFree) || (pattern_num == 0))
		return(MsgEnum::controlMode::runningFree);
	return(MsgEnum::controlMode::coordination);
}

void Card::getPermitPhases(std::bitset<8>& permitted_phases, std::bitset<8>& permitted_ped_phases, ssize_t planIdx) const
{
	permitted_phases = timingcard.phaseflags.permitted_phases;
	permitted_ped_phases = timingcard.phaseflags.permitted_ped_phases;
	if (planIdx < 0)
	{
		permitted_phases = timingcard.freeplan.permitted_phases;
		permitted_ped_phases = timingcard.freeplan.permitted_ped_phases;
	}
	else if (planIdx >= 0)
	{
		const auto& plan = timingcard.coordplans[planIdx];
		permitted_phases = plan.permitted_phases;
		permitted_ped_phases = plan.permitted_ped_phases;
	}
}

void Card::getSyncPhase(std::bitset<8>& coordinated_phases, uint8_t& synch_phase, ssize_t planIdx) const
{
	if (planIdx < 0)
	{ /// running free, coordinated phases are phase 2 & 6
		coordinated_phases = std::bitset<8>(std::string("00100010"));
		synch_phase = 2;
	}
	else
	{
		const auto& plan = timingcard.coordplans[planIdx];
		coordinated_phases = plan.sync_phases;
		synch_phase = plan.coordinated_phases[plan.sync_ring];
	}
}

uint16_t Card::getCycleLength(ssize_t planIdx) const
{
	return((planIdx < 0) ? 600 : (uint16_t)(timingcard.coordplans[planIdx].cycle_length * 10));
}

MsgEnum::phaseState Card::getPhaseState(MsgEnum::controlMode mode, uint8_t active_phase, uint8_t active_intv, uint8_t check_phase) const
{	/// check_phase [1...8]
	/// interval decoding:
	/// 0x00 = Walk
	/// 0x01 = Don’t Walk
	/// 0x02 = Min Green
	/// 0x04 = Added Initial
	/// 0x05 = Passage
	/// 0x06 = Max Gap
	/// 0x07 = Min Gap
	/// 0x08 = Red Rest
	/// 0x09 = Preemption
	/// 0x0A = Stop Time (remain indication)
	/// 0x0B = Red Revert
	/// 0x0C = Max Termination
	/// 0x0D = Gap Termination
	/// 0x0E = Force Off
	/// 0x0F = Red Clearance
	if (mode == MsgEnum::controlMode::unavailable)
		return(MsgEnum::phaseState::unavailable);
	if (mode == MsgEnum::controlMode::flashing)
		return(MsgEnum::phaseState::flashingRed);
	if (check_phase != active_phase)
		return(MsgEnum::phaseState::redLight);
	if (active_intv < 0x05)
		return(MsgEnum::phaseState::protectedGreen);
	if ((active_intv < 0x08) || (active_intv == 0x09) || (active_intv == 0x0A))
		return(MsgEnum::phaseState::permissiveGreen);
	if ((active_intv >= 0x0C) && (active_intv <= 0x0E))
		return(MsgEnum::phaseState::protectedYellow);
	return(MsgEnum::phaseState::redLight);
}

MsgEnum::phaseState Card::getPedState(MsgEnum::controlMode mode, uint8_t active_phase, uint8_t active_intv, uint8_t check_phase) const
{ /// check_phase [1...8]
	if (mode == MsgEnum::controlMode::unavailable)
		return(MsgEnum::phaseState::unavailable);
	if (mode == MsgEnum::controlMode::flashing)
		return(MsgEnum::phaseState::flashingRed);
	if (check_phase != active_phase)
		return(MsgEnum::phaseState::redLight);
	if (active_intv == 0x00)
		return(MsgEnum::phaseState::protectedGreen);
	if (active_intv == 0x01)
		return(MsgEnum::phaseState::protectedYellow);
	return(MsgEnum::phaseState::redLight);
}

MsgEnum::phaseRecallType Card::getPhaseRecallType(MsgEnum::controlMode mode, ssize_t planIdx, uint8_t phaseIdx) const
{
	const auto& phaseflags = timingcard.phaseflags;
	if (mode == MsgEnum::controlMode::runningFree)
	{
		const auto& freeplan = timingcard.freeplan;
		if (phaseflags.maximum_recall_phases.test(phaseIdx) || freeplan.maximum_recall_phases.test(phaseIdx))
			return(MsgEnum::phaseRecallType::maximum);
		else if (phaseflags.ped_recall_phases.test(phaseIdx) || freeplan.ped_recall_phases.test(phaseIdx))
			return(MsgEnum::phaseRecallType::ped);
		else if (phaseflags.minimum_recall_phases.test(phaseIdx) || freeplan.minimum_recall_phases.test(phaseIdx))
			return(MsgEnum::phaseRecallType::minimum);
		else if (phaseflags.bike_recall_phases.test(phaseIdx) || freeplan.bike_recall_phases.test(phaseIdx))
			return(MsgEnum::phaseRecallType::bike);
		return(MsgEnum::phaseRecallType::none);
	}
	if (planIdx >= 0)
	{
		const auto& coordplan = timingcard.coordplans[planIdx];
		if (phaseflags.maximum_recall_phases.test(phaseIdx) || coordplan.maximum_recall_phases.test(phaseIdx))
			return(MsgEnum::phaseRecallType::maximum);
		else if (phaseflags.ped_recall_phases.test(phaseIdx) || coordplan.ped_recall_phases.test(phaseIdx))
			return(MsgEnum::phaseRecallType::ped);
		else if (phaseflags.minimum_recall_phases.test(phaseIdx) || coordplan.minimum_recall_phases.test(phaseIdx))
			return(MsgEnum::phaseRecallType::minimum);
		else if (phaseflags.bike_recall_phases.test(phaseIdx) || coordplan.bike_recall_phases.test(phaseIdx))
			return(MsgEnum::phaseRecallType::bike);
		return(MsgEnum::phaseRecallType::none);
	}
	return(MsgEnum::phaseRecallType::none);
}

Card::ConcurrentType Card::getConcurrentPhaseType(const std::bitset<8>& active_phase, const std::bitset<8>& sync_phase) const
{
	std::bitset<8> active_sync_phase = (active_phase & sync_phase);
	std::size_t active_sync_phase_count = active_sync_phase.count();
	std::size_t sync_phase_count = sync_phase.count();
	if (active_sync_phase_count == 0)
		return(Card::ConcurrentType::minorMinor);
	return((active_sync_phase_count == sync_phase_count) ? Card::ConcurrentType::majorMajor : Card::ConcurrentType::minorMajor);
}
