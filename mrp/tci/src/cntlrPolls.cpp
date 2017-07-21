//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************
#include <algorithm>
#include <iostream>

#include "ab3418msgs.h"
#include "ab3418fcs.h"
#include "cntlrPolls.h"

Polls::Polls(int maxpolls_per_request, unsigned long long poll_per_interval)
	{poll_trace.set(maxpolls_per_request, poll_per_interval);}

void Polls::getNextPoll(bool fromStart /*=false*/)
{
	poll_trace.pcurPoll	= nullptr;
	poll_trace.index = fromStart ? 0 : (poll_trace.index + 1);
	for (size_t i = poll_trace.index, j = poll_list.size(); i < j; i++)
	{
		auto& item = poll_list[i];
		if (!item.pollReturned)
		{
			poll_trace.index = i;
			poll_trace.pcurPoll = &item;
			poll_trace.numspolled = 0;
			break;
		}
	}
}

void Polls::resetPollReturn(void)
{
	for (auto& item : poll_list)
		item.pollReturned = false;
}

bool Polls::setPollReturn(void)
{
	poll_trace.cyclenums++;
	if (poll_trace.cyclenums >= poll_trace.maxpolls)
		return(false);
	for (auto& item : poll_list)
	{
		if (!item.pollRequired && !item.pollReturned)
			item.pollReturned = true;
	}
	return(true);
}

void Polls::setPollReturn(const std::string& poll_desc)
{
	auto it = std::find_if(poll_list.begin(), poll_list.end(), [&poll_desc](Polls::poll_conf_t& item)
		{return(item.poll_desc == poll_desc);});
	if (it != poll_list.end())
		it->pollReturned = true;
}

bool Polls::sendPoll(unsigned long long msec)
{
	if ((poll_trace.pcurPoll != nullptr) && !poll_trace.pcurPoll->pollReturned
		&& (msec >= poll_trace.poll_msec + poll_trace.poll_interval))
	{
		poll_trace.poll_msec = msec;
		poll_trace.numspolled++;
		return(true);
	}
	return(false);
}

void Polls::resetPlanPolls(void)
{
	for (auto& item : poll_list)
	{
		if ((item.poll_desc.find("coord plan") == 0) && (item.pollRequired))
			item.pollReturned = false;
	}
}

bool Polls::nextPoll(void) const
{
	return((poll_trace.pcurPoll != nullptr) &&
		((poll_trace.pcurPoll->pollReturned) || (poll_trace.numspolled >= poll_trace.maxpolls)));
}

bool Polls::allReturned(void) const
{
	int open_required_poll_num = (int)std::count_if(poll_list.begin(), poll_list.end(),[](const Polls::poll_conf_t& item)
		{return(item.pollRequired && !item.pollReturned);});
	return(open_required_poll_num <= 0);
}

bool Polls::atEnd(void) const
	{return(poll_trace.pcurPoll == nullptr);}

std::string Polls::getPollDesc(void) const
	{return((poll_trace.pcurPoll != nullptr) ? poll_trace.pcurPoll->poll_desc : std::string());}

std::string Polls::getPollDesc(const uint8_t* pResp, uint8_t messType) const
{
	switch(messType)
	{
	case AB3418MSG::getBlockMsgRes_errMessType:
		{
			auto it = std::find_if(poll_list.begin(), poll_list.end(), [&messType, pResp](const Polls::poll_conf_t& item)
				{return((item.err_messType == messType) && (item.poll_data1 == pResp[0]) && (item.poll_data2 == pResp[1]));});
			return((it != poll_list.end()) ? it->poll_desc : std::string());
		}
		break;
	case AB3418MSG::getTimingDataRes_messType:
		{
			auto it = std::find_if(poll_list.begin(), poll_list.end(), [&messType, pResp](const Polls::poll_conf_t& item)
				{return((item.res_messType == messType) && (item.poll_data1 == pResp[0])
					&& (item.poll_data2 == pResp[1]) && (item.poll_data3 == pResp[2]));});
			return((it != poll_list.end()) ? it->poll_desc : std::string());
		}
		break;
	case AB3418MSG::getBlockMsgRes_messType:
		{
			auto it = std::find_if(poll_list.begin(), poll_list.end(), [&messType, pResp](const Polls::poll_conf_t& item)
				{return((item.res_messType == messType) && (item.poll_data1 == pResp[0]) && (item.poll_data2 == pResp[1]));});
			return((it != poll_list.end()) ? it->poll_desc : std::string());
		}
		break;
	default:
		return(std::string());
		break;
	}
}

std::string Polls::getPollDesc(const uint8_t* pResp, uint8_t messType, size_t frameSize, bool fcsValidated) const
{
	switch(messType)
	{
	case AB3418MSG::getBlockMsgRes_errMessType:
		{
			auto it = std::find_if(poll_list.begin(), poll_list.end(), [&messType, pResp](const Polls::poll_conf_t& item)
				{return((item.err_messType == messType) && (item.poll_data1 == pResp[0]) && (item.poll_data2 == pResp[1]));});
			return(((it != poll_list.end()) && !it->pollReturned && (frameSize == it->res_size) && (!it->fcsRequired || fcsValidated))
				? it->poll_desc : std::string());
		}
		break;
	case AB3418MSG::getTimingDataRes_messType:
		{
			auto it = std::find_if(poll_list.begin(), poll_list.end(), [&messType, pResp](const Polls::poll_conf_t& item)
				{return((item.res_messType == messType) && (item.poll_data1 == pResp[0])
					&& (item.poll_data2 == pResp[1]) && (item.poll_data3 == pResp[2]));});
			return(((it != poll_list.end()) && !it->pollReturned && (frameSize == it->res_size) && (!it->fcsRequired || fcsValidated))
				? it->poll_desc : std::string());
		}
		break;
	case AB3418MSG::getBlockMsgRes_messType:
		{
			auto it = std::find_if(poll_list.begin(), poll_list.end(), [&messType, pResp](const Polls::poll_conf_t& item)
				{return((item.res_messType == messType) && (item.poll_data1 == pResp[0]) && (item.poll_data2 == pResp[1]));});
			return(((it != poll_list.end()) && !it->pollReturned && (frameSize == it->res_size) && (!it->fcsRequired || fcsValidated))
				? it->poll_desc : std::string());
		}
		break;
	default:
		return(std::string());
		break;
	}
}

size_t Polls::packRequest(std::vector<uint8_t>& buf, uint8_t addr) const
{
	if (poll_trace.pcurPoll == nullptr)
		return(0);
	size_t offset = 0;
	buf[offset++] = AB3418MSG::flag;
	buf[offset++] = addr;
	buf[offset++] = poll_trace.pcurPoll->poll_controlByte;
	buf[offset++] = AB3418MSG::ipi;
	buf[offset++] = poll_trace.pcurPoll->poll_messType;
	buf[offset++] = poll_trace.pcurPoll->poll_data1;
	buf[offset++] = poll_trace.pcurPoll->poll_data2;
	if (poll_trace.pcurPoll->poll_type == 2) // getTimingData
		buf[offset++] = poll_trace.pcurPoll->poll_data3;
	AB3418checksum::append_FCS(buf, offset);
	AB3418checksum::get_byte_stuffing(buf, offset);
	buf[offset++] = AB3418MSG::flag;
	return(offset);
}
