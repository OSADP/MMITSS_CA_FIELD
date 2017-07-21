//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************

#include "ab3418fcs.h"
#include "ab3418msgs.h"
#include "msgEnum.h"
#include "msgUtils.h"

void AB3418MSG::bitsetphases2ringphases(uint8_t (&ringphases)[2], const std::bitset<8>& bitsetPhases)
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
}

void AB3418MSG::pattern2planOffset(uint8_t& plan_num, uint8_t& offset_index, uint8_t pattern_num)
{
	switch(pattern_num)
	{
	case 0:
		plan_num = 0;
		offset_index = 0;
		break;
	case MsgEnum::patternFlashing:
		plan_num = pattern_num;
		offset_index = 0;
		break;
	case MsgEnum::patternFree:
		plan_num = pattern_num;
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

uint8_t AB3418MSG::planOffset2pattern(uint8_t plan_num, uint8_t offset_index)
{
	if ((plan_num == 0) || (plan_num == MsgEnum::patternFlashing) || (plan_num == MsgEnum::patternFree))
		return(plan_num);
	return(static_cast<uint8_t>((plan_num - 1)* 3 + offset_index + 1));
}

void AB3418MSG::parseMsg(AB3418MSG::signal_status_mess_t& signalstatus, const std::vector<uint8_t>& msgbuf)
{
	size_t offset = 5;
	signalstatus.controller_addr = msgbuf[1];
	signalstatus.active_phase = std::bitset<8>(msgbuf[offset++]);
	AB3418MSG::bitsetphases2ringphases(signalstatus.active_phases, signalstatus.active_phase);
	signalstatus.active_interval[0] = msgbuf[offset++];
	signalstatus.active_interval[1] = msgbuf[offset++];
	signalstatus.interval_timer[0]  = msgbuf[offset++];
	signalstatus.interval_timer[1]  = msgbuf[offset++];
	signalstatus.next_phase = std::bitset<8>(msgbuf[offset++]);
	AB3418MSG::bitsetphases2ringphases(signalstatus.next_phases, signalstatus.next_phase);
	signalstatus.ped_call = std::bitset<8>(msgbuf[offset++]);
	signalstatus.veh_call = std::bitset<8>(msgbuf[offset++]);
	signalstatus.pattern_num = msgbuf[offset++];
	AB3418MSG::pattern2planOffset(signalstatus.plan_num, signalstatus.offset_index, signalstatus.pattern_num);
	signalstatus.local_cycle_clock = msgbuf[offset++];
	signalstatus.master_cycle_clock = msgbuf[offset++];
	signalstatus.preempt = std::bitset<8>(msgbuf[offset++]);
	for (int i = 0; i < 8; i++)
		signalstatus.permissive[i] = msgbuf[offset++];
	signalstatus.active_force_off[0] = msgbuf[offset++];
	signalstatus.active_force_off[1] = msgbuf[offset++];
	for (int i = 0; i < 8; i++)
		signalstatus.ped_permissive[i] = msgbuf[offset++];
}

void AB3418MSG::parseMsg(AB3418MSG::status8e_mess_t& status8e, const std::vector<uint8_t>& msgbuf)
{
	size_t offset = 5;
	status8e.controller_addr = msgbuf[1];
	status8e.hour   = msgbuf[offset++];
	status8e.minute = msgbuf[offset++];
	status8e.sec    = msgbuf[offset++];
	status8e.flag   = std::bitset<8>(msgbuf[offset++]);
	status8e.status = std::bitset<8>(msgbuf[offset++]);
	status8e.pattern_num = msgbuf[offset++];
	offset += 7;
	unsigned long long ull_presences = msgbuf[offset++];
	ull_presences |= msgbuf[offset++] << 8;
	ull_presences |= msgbuf[offset++] << 16;
	ull_presences |= msgbuf[offset++] << 24;
	ull_presences |= (unsigned long long)msgbuf[offset++] << 32;
	status8e.detector_presences = std::bitset<40>(ull_presences);
	status8e.master_cycle_clock = msgbuf[offset++];
	status8e.local_cycle_clock = msgbuf[offset++];
	status8e.prio_busId = msgUtils::unpack2bytes(msgbuf, offset);
	status8e.prio_busDirection = msgbuf[offset++];
	status8e.prio_type = msgbuf[offset++];
}

void AB3418MSG::parseMsg(AB3418MSG::longstatus8e_mess_t& longstatus8e, const std::vector<uint8_t>& msgbuf)
{
	size_t offset = 5;
	longstatus8e.controller_addr = msgbuf[1];
	longstatus8e.hour = msgbuf[offset++];
	longstatus8e.minute = msgbuf[offset++];
	longstatus8e.sec = msgbuf[offset++];
	longstatus8e.flag = std::bitset<8>(msgbuf[offset++]);
	longstatus8e.status = std::bitset<8>(msgbuf[offset++]);
	longstatus8e.pattern_num = msgbuf[offset++];
	offset += 12;
	longstatus8e.master_cycle_clock = msgbuf[offset++];
	longstatus8e.local_cycle_clock = msgbuf[offset++];
	longstatus8e.seq_num = msgbuf[offset++];
	for (int i = 0; i < 16; i++)
	{
		longstatus8e.volume[i]    = msgbuf[offset++];
		longstatus8e.occupancy[i] = msgbuf[offset++];
	}
}

size_t AB3418MSG::packMsg(std::vector<uint8_t>& buf, const AB3418MSG::signal_status_mess_t& signalstatus, uint8_t msgid, uint32_t  msOfDay)
{
	size_t offset = 0;
	msgUtils::packHeader(buf, offset, msgid, msOfDay, 31);
	buf[offset++] = signalstatus.controller_addr;
	buf[offset++] = (uint8_t)signalstatus.active_phase.to_ulong();
	buf[offset++] = signalstatus.active_interval[0];
	buf[offset++] = signalstatus.active_interval[1];
	buf[offset++] = signalstatus.interval_timer[0];
	buf[offset++] = signalstatus.interval_timer[1];
	buf[offset++] = (uint8_t)signalstatus.next_phase.to_ulong();
	buf[offset++] = (uint8_t)signalstatus.ped_call.to_ulong();
	buf[offset++] = (uint8_t)signalstatus.veh_call.to_ulong();
	buf[offset++] = signalstatus.pattern_num;
	buf[offset++] = signalstatus.local_cycle_clock;
	buf[offset++] = signalstatus.master_cycle_clock;
	buf[offset++] = (uint8_t)signalstatus.preempt.to_ulong();
	for (int i = 0; i < 8; i++)
		buf[offset++] = signalstatus.permissive[i];
	buf[offset++] = signalstatus.active_force_off[0];
	buf[offset++] = signalstatus.active_force_off[1];
	for (int i = 0; i < 8; i++)
		buf[offset++] = signalstatus.ped_permissive[i];
	return(offset);
}

size_t AB3418MSG::packMsg(std::vector<uint8_t>& buf, const AB3418MSG::status8e_mess_t& status8e, uint8_t msgid, uint32_t  msOfDay)
{
	size_t offset = 0;
	msgUtils::packHeader(buf, offset, msgid, msOfDay, 14);
	buf[offset++] = (uint8_t)status8e.flag.to_ulong();
	buf[offset++] = (uint8_t)status8e.status.to_ulong();
	buf[offset++] = status8e.pattern_num;
	buf[offset++] = status8e.master_cycle_clock;
	buf[offset++] = status8e.local_cycle_clock;
	msgUtils::pack2bytes(buf, offset, status8e.prio_busId);
	buf[offset++] = status8e.prio_busDirection;
	buf[offset++] = status8e.prio_type;
	msgUtils::packMultiBytes(buf, offset, status8e.detector_presences.to_ullong(), 5);
	return(offset);
}

size_t AB3418MSG::packMsg(std::vector<uint8_t>& buf, const AB3418MSG::longstatus8e_mess_t& longstatus8e, uint8_t msgid, uint32_t  msOfDay)
{
	size_t offset = 0;
	msgUtils::packHeader(buf, offset, msgid, msOfDay, 38);
	buf[offset++] = longstatus8e.seq_num;
	buf[offset++] = (uint8_t)longstatus8e.flag.to_ulong();
	buf[offset++] = (uint8_t)longstatus8e.status.to_ulong();
	buf[offset++] = longstatus8e.pattern_num;
	buf[offset++] = longstatus8e.master_cycle_clock;
	buf[offset++] = longstatus8e.local_cycle_clock;
	for (int i = 0; i < 16; i++)
		buf[offset++] = longstatus8e.volume[i];
	for (int i = 0; i < 16; i++)
		buf[offset++] = longstatus8e.occupancy[i];
	return(offset);
}

size_t AB3418MSG::packMsg(std::vector<uint8_t>& buf, const std::bitset<8>& veh_call, const std::bitset<8>& ped_call,
	const std::bitset<8>& prio_call, uint8_t msgid, uint32_t  msOfDay)
{
	size_t offset = 0;
	msgUtils::packHeader(buf, offset, msgid, msOfDay, 3);
	buf[offset++] = (uint8_t)veh_call.to_ulong();
	buf[offset++] = (uint8_t)ped_call.to_ulong();
	buf[offset++] = (uint8_t)prio_call.to_ulong();
	return(offset);
}

size_t AB3418MSG::packRequest(std::vector<uint8_t>& buf, uint8_t addr, const std::bitset<8>& veh_call,
	const std::bitset<8>& ped_call, const std::bitset<8>& prio_call)
{
	size_t offset = 0;
	buf[offset++] = AB3418MSG::flag;
	buf[offset++] = addr;
	buf[offset++] = AB3418MSG::set_controlByte;
	buf[offset++] = AB3418MSG::ipi;
	buf[offset++] = AB3418MSG::setSoftcall_messType;
	buf[offset++] = (uint8_t)veh_call.to_ulong();
	buf[offset++] = (uint8_t)ped_call.to_ulong();
	buf[offset++] = (uint8_t)prio_call.to_ulong();
	for (int i = 0; i < 5; i++)
		buf[offset++] = 0;
	AB3418checksum::append_FCS(buf, offset);
	AB3418checksum::get_byte_stuffing(buf, offset);
	buf[offset++] = AB3418MSG::flag;
	return(offset);
}

std::string AB3418MSG::errCode(uint8_t err_num)
{
	switch (err_num)
	{
	case 0:
		return(std::string("ERROR_NO_ERROR"));
		break;
	case 1:
		return(std::string("ERROR_TOO_BIG"));
		break;
	case 2:
		return(std::string("ERROR_NO_SUCH_NAME"));
		break;
	case 3:
		return(std::string("ERROR_BAD_VALUE"));
		break;
	case 4:
		return(std::string("ERROR_READ_ONLY"));
		break;
	case 5:
		return(std::string("ERROR_GEN_ERR"));
		break;
	case 6:
		return(std::string("ERROR_MESS_LEN"));
		break;
	case 10:
		return(std::string("ERROR_INVALID_PLAN"));
		break;
	case 11:
		return(std::string("ERROR_INVALID_PACKET_SIZE"));
		break;
	case 12:
		return(std::string("ERROR_OUT_OF_RANGE"));
		break;
	default:
		return(std::string("ERROR_UNKNOWN_MSG"));
	}
}
