//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************

#include "msgUtils.h"

void msgUtils::pack4bytes(std::vector<uint8_t>& buf, size_t& offset, uint32_t value)
{
	for (int i = 0; i < 4; i++)
		buf[offset++] = (uint8_t)((value >> (3-i)*8) & 0xFF);
}

void msgUtils::pack2bytes(std::vector<uint8_t>& buf, size_t& offset, uint16_t value)
{
	buf[offset++] = (uint8_t)((value >> 8) & 0xFF);
	buf[offset++] = (uint8_t)(value & 0xFF);
}

void msgUtils::packMultiBytes(std::vector<uint8_t>& buf, size_t& offset, unsigned long long value, int bytenums)
{
	for (int i = 0; i < bytenums; i++)
		buf[offset++] = (uint8_t)((value >> (bytenums-1-i)*8) & 0xFF);
}

void msgUtils::packHeader(std::vector<uint8_t>& buf, size_t& offset, uint8_t msgid, uint32_t msec, uint16_t length)
{ /// MMITSS UDP header
	msgUtils::pack2bytes(buf, offset, msgUtils::msg_header);
	buf[offset++] = msgid;
	msgUtils::pack4bytes(buf, offset, msec);
	msgUtils::pack2bytes(buf, offset, length);
}

void msgUtils::packHeader(std::vector<uint8_t>& buf, size_t& offset, uint8_t msgtype, uint16_t intersectionID, uint32_t msec, uint16_t length)
{ /// Savari UDP header
	buf[offset++] = msgtype;
	msgUtils::pack2bytes(buf, offset, length);
	msgUtils::pack4bytes(buf, offset, msec);
	msgUtils::pack2bytes(buf, offset, intersectionID);
}

uint32_t msgUtils::unpack4bytes(const std::vector<uint8_t>& buf, size_t& offset)
{
	uint32_t retn = 0;
	for (int i = 0; i < 4; i++)
		retn = (retn << 8) | buf[offset++];
	return(retn);
}

uint16_t msgUtils::unpack2bytes(const std::vector<uint8_t>& buf, size_t& offset)
{
	uint16_t retn = (uint16_t)((buf[offset] << 8) | buf[offset+1]);
	offset += 2;
	return(retn);
}

unsigned long long msgUtils::unpackMultiBytes(const std::vector<uint8_t>& buf, size_t& offset, int bytenums)
{
	unsigned long long retn = 0;
	for (int i = 0; i < bytenums; i++)
		retn = (retn << 8) | buf[offset++];
	return(retn);
}

void msgUtils::unpackHeader(const std::vector<uint8_t>& buf, size_t& offset, msgUtils::mmitss_udp_header_t& header)
{ /// MMITSS UDP header
	header.msgheader = msgUtils::unpack2bytes(buf, offset);
	header.msgid = buf[offset++];
	header.ms_since_midnight = msgUtils::unpack4bytes(buf, offset);
	header.length = msgUtils::unpack2bytes(buf, offset);
}

void msgUtils::unpackHeader(const std::vector<uint8_t>& buf, size_t& offset, msgUtils::savari_udp_header_t& header)
{ /// Savari UDP header
	header.type = buf[offset++];
	header.length = msgUtils::unpack2bytes(buf, offset);
	header.ms_since_midnight = msgUtils::unpack4bytes(buf, offset);
	header.intersectionID = msgUtils::unpack2bytes(buf, offset);
}
