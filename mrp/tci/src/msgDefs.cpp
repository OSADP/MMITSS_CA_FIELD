//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************

#include "msgUtils.h"
#include "msgDefs.h"

size_t msgDefs::packMsg(std::vector<uint8_t>& buf, const msgDefs::intPerm_t& data, uint8_t msgid)
{
	size_t offset = 9;
	buf[offset++] = static_cast<uint8_t>(data.mode);
	buf[offset++] = data.patternNum;
	buf[offset++] = (uint8_t)data.permittedPhases.to_ulong();
	buf[offset++] = (uint8_t)data.observedPhases.to_ulong();
	for (int i = 0; i < 8; i++)
	{
		if (data.observedPhases.test(i))
		{
			const auto& apchPerm = data.apchPerm[i];
			msgUtils::pack2bytes(buf, offset, apchPerm.sampleNums);
			msgUtils::pack2bytes(buf, offset, apchPerm.travelSpeed_mean);
			msgUtils::pack2bytes(buf, offset, apchPerm.travelSpeed_std);
			msgUtils::pack2bytes(buf, offset, apchPerm.travelTime_mean);
			msgUtils::pack2bytes(buf, offset, apchPerm.travelTime_std);
			msgUtils::pack2bytes(buf, offset, apchPerm.delay_mean);
			msgUtils::pack2bytes(buf, offset, apchPerm.delay_std);
			msgUtils::pack2bytes(buf, offset, apchPerm.stoppedNums);
		}
	}
	/// add MMITSS header
	size_t header_offset = 0;
	msgUtils::packHeader(buf, header_offset, msgid, data.ms_since_midnight, (uint16_t)(offset - 9));
	return(offset);
}

size_t msgDefs::packMsg(std::vector<uint8_t>& buf, const msgDefs::softcall_request_t& data, uint8_t msgid)
{
	size_t offset = 0;
	msgUtils::packHeader(buf, offset, msgid, data.ms_since_midnight, 3);
	buf[offset++] = static_cast<uint8_t>(data.callphase.to_ulong());
	buf[offset++] = static_cast<uint8_t>(data.callobj);
	buf[offset++] = static_cast<uint8_t>(data.calltype);
	return(offset);
}

size_t msgDefs::packMsg(std::vector<uint8_t>& buf, const msgDefs::vehTraj_t& data, uint8_t msgid)
{
	size_t offset = 0;
	msgUtils::packHeader(buf, offset, msgid, data.ms_since_midnight, 20);
	msgUtils::pack4bytes(buf, offset, data.count);
	msgUtils::pack4bytes(buf, offset, data.vehId);
	buf[offset++] = data.entryLaneId;
	buf[offset++] = data.entryControlPhase;
	buf[offset++] = data.leaveLaneId;
	buf[offset++] = data.leaveControlPhase;
	msgUtils::pack2bytes(buf, offset, data.distTraveled);
	msgUtils::pack2bytes(buf, offset, data.timeTraveled);
	msgUtils::pack2bytes(buf, offset, data.stoppedTime);
	msgUtils::pack2bytes(buf, offset, data.inboundLaneLen);
	return(offset);
}

void msgDefs::unpackMsg(const std::vector<uint8_t>& buf, size_t& offset, msgDefs::count_data_t& data)
{
	data.seq_num = buf[offset++];
	data.flag = std::bitset<8>(buf[offset++]);
	data.status = std::bitset<8>(buf[offset++]);
	data.pattern_num = buf[offset++];
	data.master_cycle_clock = buf[offset++];
	data.local_cycle_clock  = buf[offset];
	for (int i = 0; i < 16; i++)
		data.vol[i] = buf[++offset];
	for (int i = 0; i < 16; i++)
		data.occ[i] = buf[++offset];
}

void msgDefs::unpackMsg(const std::vector<uint8_t>& buf, size_t& offset, msgDefs::pres_data_t& data)
{
	data.flag = std::bitset<8>(buf[offset++]);
	data.status = std::bitset<8>(buf[offset++]);
	data.pattern_num = buf[offset++];
	data.master_cycle_clock = buf[offset++];
	data.local_cycle_clock = buf[offset++];
	data.prio_busId = msgUtils::unpack2bytes(buf, offset);
	data.prio_busDirection = buf[offset++];
	data.prio_type = buf[offset++];
	data.presences = std::bitset<40>(msgUtils::unpackMultiBytes(buf, offset, 5));
}

void msgDefs::unpackMsg(const std::vector<uint8_t>& buf, size_t& offset, msgDefs::controller_state_t& data)
{
	auto& spat = data.spatRaw;
	spat.msgCnt = buf[offset++];
	spat.timeStampMinute = msgUtils::unpack4bytes(buf, offset);
	spat.timeStampSec = msgUtils::unpack2bytes(buf, offset);
	spat.permittedPhases = std::bitset<8>(buf[offset++]);
	spat.permittedPedPhases = std::bitset<8>(buf[offset++]);
	spat.status = std::bitset<16>(msgUtils::unpack2bytes(buf, offset));
	for (int i = 0; i < 8; i++)
	{
		auto& phaseState = spat.phaseState[i];
		if (!spat.permittedPhases.test(i))
			phaseState.reset();
		else
		{
			phaseState.currState = static_cast<MsgEnum::phaseState>(buf[offset++]);
			phaseState.startTime = msgUtils::unpack2bytes(buf, offset);
			phaseState.minEndTime = msgUtils::unpack2bytes(buf, offset);
			phaseState.maxEndTime = msgUtils::unpack2bytes(buf, offset);
		}
	}
	for (int i = 0; i < 8; i++)
	{
		auto& pedPhaseState = spat.pedPhaseState[i];
		if (!spat.permittedPedPhases.test(i))
			pedPhaseState.reset();
		else
		{
			pedPhaseState.currState = static_cast<MsgEnum::phaseState>(buf[offset++]);
			pedPhaseState.startTime = msgUtils::unpack2bytes(buf, offset);
			pedPhaseState.minEndTime = msgUtils::unpack2bytes(buf, offset);
			pedPhaseState.maxEndTime = msgUtils::unpack2bytes(buf, offset);
		}
	}
	auto& signalStatus = data.signalStatus;
	signalStatus.mode = static_cast<MsgEnum::controlMode>(buf[offset++]);
	signalStatus.patternNum = buf[offset++];
	signalStatus.synch_phase = buf[offset++];
	signalStatus.cycle_length = msgUtils::unpack2bytes(buf, offset);
	signalStatus.local_cycle_clock = msgUtils::unpack2bytes(buf, offset);
	signalStatus.coordinated_phases = std::bitset<8>(buf[offset++]);
	signalStatus.preempt = std::bitset<8>(buf[offset++]);
	signalStatus.ped_call = std::bitset<8>(buf[offset++]);
	signalStatus.veh_call = std::bitset<8>(buf[offset++]);
	for (int i = 0; i < 8; i++)
	{
		signalStatus.call_status[i] = static_cast<MsgEnum::phaseCallType>(buf[offset++]);
		signalStatus.recall_status[i] = static_cast<MsgEnum::phaseRecallType>(buf[offset++]);
	}
}

void msgDefs::unpackMsg(const std::vector<uint8_t>& buf, size_t& offset, msgDefs::softcall_request_t& data)
{
	data.callphase = std::bitset<8>(buf[offset++]);
	data.callobj  = static_cast<MsgEnum::softCallObj>(buf[offset++]);
	data.calltype = static_cast<MsgEnum::softCallType>(buf[offset]);
}

void msgDefs::unpackMsg(const std::vector<uint8_t>& buf, size_t& offset, msgDefs::vehTraj_t& data)
{
	data.count = msgUtils::unpack4bytes(buf, offset);
	data.vehId = msgUtils::unpack4bytes(buf, offset);
	data.entryLaneId = buf[offset++];
	data.entryControlPhase = buf[offset++];
	data.leaveLaneId = buf[offset++];
	data.leaveControlPhase = buf[offset++];
	data.distTraveled = msgUtils::unpack2bytes(buf, offset);
	data.timeTraveled = msgUtils::unpack2bytes(buf, offset);
	data.stoppedTime = msgUtils::unpack2bytes(buf, offset);
	data.inboundLaneLen = msgUtils::unpack2bytes(buf, offset);
}

void msgDefs::unpackMsg(const std::vector<uint8_t>& buf, size_t& offset, msgDefs::intPerm_t& data)
{
	data.mode = static_cast<MsgEnum::controlMode>(buf[offset++]);
	data.patternNum = buf[offset++];
	data.permittedPhases = std::bitset<8>(buf[offset++]);
	data.observedPhases = std::bitset<8>(buf[offset++]);
	for (int i = 0; i < 8; i++)
	{
		if (data.observedPhases.test(i))
		{
			auto& apchPerm = data.apchPerm[i];
			apchPerm.sampleNums = msgUtils::unpack2bytes(buf, offset);
			apchPerm.travelSpeed_mean = msgUtils::unpack2bytes(buf, offset);
			apchPerm.travelSpeed_std  = msgUtils::unpack2bytes(buf, offset);
			apchPerm.travelTime_mean  = msgUtils::unpack2bytes(buf, offset);
			apchPerm.travelTime_std   = msgUtils::unpack2bytes(buf, offset);
			apchPerm.delay_mean       = msgUtils::unpack2bytes(buf, offset);
			apchPerm.delay_std        = msgUtils::unpack2bytes(buf, offset);
			apchPerm.stoppedNums      = msgUtils::unpack2bytes(buf, offset);
		}
	}
}
