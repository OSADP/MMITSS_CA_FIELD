//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************
#ifndef AB3418MSGS_H
#define AB3418MSGS_H

#include <bitset>
#include <cstdint>
#include <string>
#include <vector>

namespace AB3418MSG
{
	static const uint8_t flag = 0x7E;
	static const uint8_t ipi  = 0xC0;

	/// send to traffic signal controller
	static const uint8_t get_controlByte = 0x33;
	static const uint8_t set_controlByte = 0x13;
	/// receive from traffic signal controller
	static const uint8_t res_controlByte = 0x13;

	/// poll COMMAND to the traffic signal controller
	static const uint8_t getBlockMsg_messType       = 0x87;
	static const uint8_t getBlockMsgRes_messType    = 0xC7;
	static const uint8_t getBlockMsgRes_errMessType = 0xE7;

	static const uint8_t getTimingData_messType       = 0x89;
	static const uint8_t getTimingDataRes_messType    = 0xC9;
	static const uint8_t getTimingDataRes_errMessType = 0xE9;

	/// set COMMAND to the traffic signal controller
	static const uint8_t setBlockMsg_messType       = 0x96;
	static const uint8_t setBlockMsgRes_messType    = 0xD6;
	static const uint8_t setBlockMsgRes_errMessType = 0xF6;

	static const uint8_t setTimingData_messType       = 0x99;
	static const uint8_t setTimingDataRes_messType    = 0xD9;
	static const uint8_t setTimingDataRes_errMessType = 0xF9;

	static const uint8_t setSoftcall_messType       = 0x9A;
	static const uint8_t setSoftcallRes_messType    = 0xDA;
	static const uint8_t setSoftcallRes_errMessType = 0xFA;

	/// pushing out messages from the traffic signal controller pushing out
	static const uint8_t status8eRes_messType     = 0xC8;
	static const uint8_t longStatus8eRes_messType = 0xCD;
	static const uint8_t rawspatRes_messType      = 0xCE;

	/// size of pushing out messages
	static const size_t status8eRes_size     = 35;
	static const size_t longStatus8eRes_size = 68;
	static const size_t rawspatRes_size      = 38;
	/// size of error response messages
	static const size_t errGetBlockRes_size = 12;
	static const size_t errGetDataRes_size  = 10;
	static const size_t errSetDataRes_size  = 10;
	
	/// controller pushing out signal status message
	struct signal_status_mess_t
	{
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
		std::bitset<8> preempt; // bits 0-3 <==> EV A-D
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
		uint8_t controller_addr;
		uint8_t hour;
		uint8_t minute;
		uint8_t sec;
		std::bitset<8> flag;   // bit 0 = focus mode
		                       // bit 2 = advance input status
		                       // bit 3 = spare 3 input status
		                       // bit 4 = spare 2 input status
		                       // bit 5 = spare 1 input status
		                       // bit 7 = transit vehicle call
		std::bitset<8> status; // bit 0 = in preemption
		                       // bit 1 = cabinet flash
		                       // bit 2 = passed local zero since last request
		                       // bit 3 = in local override mode
		                       // bit 4 = coordination alarm pending
		                       // bit 5 = detector fault pending
		                       // bit 6 = non-critical alarm pending
		                       // bit 7 = critical alarm pending
		uint8_t pattern_num;   // plan_num plus offset_index
		std::bitset<40> detector_presences;
		uint8_t master_cycle_clock;
		uint8_t local_cycle_clock;
		uint16_t prio_busId;
		uint8_t prio_busDirection; //  5 = Phase2 opticom ON
		                           // 13 = Phase2 opticom OFF
		                           // 21 = Phase6 opticom ON
		                           // 17 = Phase6 opticom OFF
		uint8_t  prio_type;        // 0 = no Priority
		                           // 1 = Early Green
		                           // 2 = Green Extension
	};

	/// controller pushing out long status8 message
	struct longstatus8e_mess_t
	{
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

	void bitsetphases2ringphases(uint8_t (&ringphases)[2], const std::bitset<8>& bitsetPhases);
	void pattern2planOffset(uint8_t& plan_num, uint8_t& offset_index, uint8_t pattern_num);
	uint8_t planOffset2pattern(uint8_t plan_num, uint8_t offset_index);
	void parseMsg(AB3418MSG::signal_status_mess_t& signalstatus, const std::vector<uint8_t>& msgbuf);
	void parseMsg(AB3418MSG::status8e_mess_t& status8e, const std::vector<uint8_t>& msgbuf);
	void parseMsg(AB3418MSG::longstatus8e_mess_t& longstatus8e, const std::vector<uint8_t>& msgbuf);
	size_t packMsg(std::vector<uint8_t>& buf, const AB3418MSG::signal_status_mess_t& signalstatus, uint8_t msgid, uint32_t  msOfDay);
	size_t packMsg(std::vector<uint8_t>& buf, const AB3418MSG::status8e_mess_t& status8e, uint8_t msgid, uint32_t  msOfDay);
	size_t packMsg(std::vector<uint8_t>& buf, const AB3418MSG::longstatus8e_mess_t& longstatus8e, uint8_t msgid, uint32_t  msOfDay);
	size_t packMsg(std::vector<uint8_t>& buf, const std::bitset<8>& veh_call, const std::bitset<8>& ped_call,
		const std::bitset<8>& prio_call, uint8_t msgid, uint32_t  msOfDay);
	size_t packRequest(std::vector<uint8_t>& buf, uint8_t addr, const std::bitset<8>& veh_call,
		const std::bitset<8>& ped_call, const std::bitset<8>& prio_call);
	std::string errCode(uint8_t err_num);
}
#endif
