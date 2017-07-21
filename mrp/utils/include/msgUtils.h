//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************
#ifndef _MMITSSMSGUTILS_H
#define _MMITSSMSGUTILS_H

#include <cstddef>
#include <cstdint>
#include <vector>

namespace msgUtils
{ /// MMITSS header identifier
	static const uint16_t msg_header          = 0xFFFF;
	/// MMITSS header message ID
	static const uint8_t msgid_bsm            = 0x40;
	static const uint8_t msgid_spat           = 0x41;
	static const uint8_t msgid_map            = 0x42;
	static const uint8_t msgid_srm            = 0x43;
	static const uint8_t msgid_ssm            = 0x44;
	static const uint8_t msgid_psrm           = 0x45;
	static const uint8_t msgid_softcall       = 0x50;
	static const uint8_t msgid_traj           = 0x51;
	static const uint8_t msgid_phaseflags     = 0x52;
	static const uint8_t msgid_phasetiming    = 0x53;
	static const uint8_t msgid_coordplan      = 0x54;
	static const uint8_t msgid_freeplan       = 0x55;
	static const uint8_t msgid_detAsgmnt      = 0x56;
	static const uint8_t msgid_timeCard       = 0x57;
	static const uint8_t msgid_detCnt         = 0x58;
	static const uint8_t msgid_detPres        = 0x59;
	static const uint8_t msgid_cntrlstatus    = 0x60;
	static const uint8_t msgid_perm           = 0x61;
	static const uint8_t msgid_signalraw      = 0x62;
	static const uint8_t msgid_placecall      = 0x63;
	/// message poll request
	static const uint8_t msgid_pollReq        = 0x80;
	/// Savari cloud message type
	static const uint8_t savari_cloud_map     = 0x01;
	static const uint8_t savari_cloud_spat    = 0x02;
	static const uint8_t savari_cloud_srm     = 0x03;

	/// all messages are in network byte order

	/// Savari UDP message header (between MRP_DataMgr and the cloud server)
	struct savari_udp_header_t
	{
		uint8_t   type;
		uint16_t  length;
		uint32_t  ms_since_midnight;
		uint16_t  intersectionID;
	};

	/// MMITSS UDP message header (between MMITSS components)
	struct mmitss_udp_header_t
	{
		uint16_t  msgheader;
		uint8_t   msgid;
		uint32_t  ms_since_midnight;
		uint16_t  length;
	};

	void pack4bytes(std::vector<uint8_t>& buf, size_t& offset, uint32_t value);
	void pack2bytes(std::vector<uint8_t>& buf, size_t& offset, uint16_t value);
	void packMultiBytes(std::vector<uint8_t>& buf, size_t& offset, unsigned long long value, int bytenums);
	void packHeader(std::vector<uint8_t>& buf, size_t& offset, uint8_t msgid, uint32_t msec, uint16_t length);
	void packHeader(std::vector<uint8_t>& buf, size_t& offset, uint8_t msgtype, uint16_t intersectionID, uint32_t msec, uint16_t length);

	uint32_t unpack4bytes(const std::vector<uint8_t>& buf, size_t& offset);
	uint16_t unpack2bytes(const std::vector<uint8_t>& buf, size_t& offset);
	unsigned long long unpackMultiBytes(const std::vector<uint8_t>& buf, size_t& offset, int bytenums);
	void unpackHeader(const std::vector<uint8_t>& buf, size_t& offset, msgUtils::mmitss_udp_header_t& header);
	void unpackHeader(const std::vector<uint8_t>& buf, size_t& offset, msgUtils::savari_udp_header_t& header);
}

#endif
