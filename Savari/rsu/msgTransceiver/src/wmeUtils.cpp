//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************
#include <algorithm>
#include <utility>

#include "wmeUtils.h"

uint8_t wmeUtils::getpsidlen(const uint8_t& ch)
{ /// PSID has variable length between 1 and 4
	uint8_t len = 0;
	if ((ch & 0xF0) == 0xE0)
		len = 4;
	else if ((ch & 0xE0) == 0xC0)
		len = 3;
	else if ((ch & 0xC0) == 0x80)
		len = 2;
	else if ((ch & 0x80) == 0x00)
		len = 1;
	return(len);
}

size_t wmeUtils::upack_wsmp_header(const uint8_t* buf, size_t size, wmeUtils::wsmp_header_t& header)
{ /// buffer is in network byte order
	if ((buf == NULL) || (size < sizeof(wmeUtils::wsmp_header_t)))
		return(0);
	size_t offset = 0;
	header.version = buf[offset++];
	header.psidlen = wmeUtils::getpsidlen(buf[offset]);
	if (header.psidlen == 0)
		return(0);
	header.psid = 0;
	for (uint8_t i = 0; i < header.psidlen; i++)
		header.psid = (header.psid << 8) | buf[offset++];
	/// find WSMPWAVEID
	header.waveId = 0;
	while(1)
	{
		switch(buf[offset])
		{
		case wmeUtils::WSMPWAVEID:
			header.waveId = buf[offset];
			break;
		case wmeUtils::CHANNUM:
			offset += 2; /// skip elementLen
			header.channel = buf[offset];
			break;
		case wmeUtils::DATARATE:
			offset += 2; /// skip elementLen
			header.rate = buf[offset];
			break;
		case wmeUtils::TRANSMITPW:
			offset += 2; /// skip elementLen
			header.txpower = static_cast<int8_t>(buf[offset]);
			break;
		default:
			break;
		}
		offset++;
		if ((header.waveId == wmeUtils::WSMPWAVEID) || (offset >= size) || (offset > 20))
			break;
	}
	if (header.waveId != wmeUtils::WSMPWAVEID)
		return(0);
	header.txlength = static_cast<uint16_t>((buf[offset] << 8) | buf[offset+1]);
	offset += 2;
	return(offset);
}

size_t wmeUtils::pack_wsmp_header(const wmeUtils::wsmp_header_t& header, uint8_t* buf, size_t size)
{
	if ((header.psidlen == 0) || (header.psidlen > 4)
			|| (buf == NULL) || (size < 20))
		return(0);
	size_t offset = 0;
	buf[offset++] = header.version;
	for (uint8_t i = 0; i < header.psidlen; i++)
	{
		unsigned int shift_bits = (header.psidlen - 1 - i) * 8;
		buf[offset++] = (uint8_t)((header.psid >> shift_bits) & 0xFF);
	}
	buf[offset++] = CHANNUM;
	buf[offset++] = 1;
	buf[offset++] = header.channel;
	buf[offset++] = DATARATE;
	buf[offset++] = 1;
	buf[offset++] = header.rate;
	buf[offset++] = TRANSMITPW;
	buf[offset++] = 1;
	buf[offset++] = static_cast<uint8_t>(header.txpower);
	buf[offset++] = header.waveId;
	buf[offset++] = (uint8_t)((header.txlength >> 8) & 0xFF);
	buf[offset++] = (uint8_t)(header.txlength & 0xFF);
	return(offset);
}

uint32_t wmeUtils::getpsidbymsgid(uint8_t id)
{
	std::map<uint8_t, uint32_t>::const_iterator it = wmeUtils::mmitssMsgid2psidMap.find(id);
	return((it != wmeUtils::mmitssMsgid2psidMap.end()) ? it->second : 0);
}

uint8_t wmeUtils::getmsgidbypsid(uint32_t id)
{
	uint8_t retn = 0;
	std::map<uint8_t, uint32_t>::const_iterator it;
	for (it = wmeUtils::mmitssMsgid2psidMap.begin(); it != wmeUtils::mmitssMsgid2psidMap.end(); ++it)
	{
		if (it->second == id)
		{
			retn = it->first;
			break;
		}
	}
	return(retn);
}
