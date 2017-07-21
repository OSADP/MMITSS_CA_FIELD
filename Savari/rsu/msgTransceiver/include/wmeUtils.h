//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************
#ifndef _WME_UITLS_H
#define _WME_UITLS_H

#include <cstddef>
#include <stdint.h>     /// c++11 <cstdint>
#include <string>
#include <map>
#include <vector>

#include "msgUtils.h"

namespace wmeUtils
{
	enum ifaceType {ath0, ath1};
	enum transMode {continuous, alternating};
	enum appRole   {user, provider};
	enum direction {TX, RX};
	/// WME PSID
	static const uint32_t WSM_PSID_BSM    = 0x20;
	static const uint32_t WSM_PSID_SPAT   = 0xBFE0;
	static const uint32_t WSM_PSID_MAP    = 0xBFF0;
	static const uint32_t WSM_PSID_SRM    = 0x40;
	static const uint32_t WSM_PSID_SSM    = 0x60;
	/// WSMP header tag
	static const uint8_t WSMPWAVEID       = 0x80;
	static const uint8_t TRANSMITPW       = 0x04;
	static const uint8_t CHANNUM          = 0x0F;
	static const uint8_t DATARATE         = 0x10;
	static const uint8_t WSMPMAXLEN       = 0x11;
	/// MMITSS message id (internal use)
	static const uint8_t msgid_bsm        = 0x40;
	static const uint8_t msgid_spat       = 0x41;
	static const uint8_t msgid_map        = 0x42;
	static const uint8_t msgid_srm        = 0x43;
	static const uint8_t msgid_ssm        = 0x44;

	/// WSMP header
	struct wsmp_header_t
	{
		uint8_t   version;
		uint32_t  psid;
		uint8_t   psidlen;
		uint8_t   channel;
		uint8_t   rate;
		int8_t    txpower;
		uint8_t   waveId;
		uint16_t  txlength;
	};

	struct wme_iface_cfg_t
	{
		wmeUtils::ifaceType iface;
		wmeUtils::appRole   role;
	};

	struct wme_channel_cfg_t
	{
		wmeUtils::ifaceType iface;
		std::string msgName;
		uint32_t    psid;
		uint8_t     channel;
		wmeUtils::transMode txMode;
		uint8_t     priority;
	};

  inline std::map<uint8_t,uint32_t> creat_mmitssMsgid2psidMap(void)
  {
    std::map<uint8_t,uint32_t> retn;
    retn[msgUtils::msgid_bsm]  = wmeUtils::WSM_PSID_BSM;
    retn[msgUtils::msgid_map]  = wmeUtils::WSM_PSID_MAP;
    retn[msgUtils::msgid_spat] = wmeUtils::WSM_PSID_SPAT;
    retn[msgUtils::msgid_srm]  = wmeUtils::WSM_PSID_SRM;
    retn[msgUtils::msgid_ssm]  = wmeUtils::WSM_PSID_SSM;
    return(retn);
  };
	static const std::map<uint8_t, uint32_t> mmitssMsgid2psidMap = wmeUtils::creat_mmitssMsgid2psidMap();

	inline std::vector<wmeUtils::wme_channel_cfg_t> creat_wmeAppChannelMap(void)
	{
		std::vector<wmeUtils::wme_channel_cfg_t> retn;
		wmeUtils::wme_channel_cfg_t mBSM  = {wmeUtils::ath1,  "BSM", wmeUtils::WSM_PSID_BSM,  172, wmeUtils::continuous, 7};
		wmeUtils::wme_channel_cfg_t mSPaT = {wmeUtils::ath1, "SPaT", wmeUtils::WSM_PSID_SPAT, 172, wmeUtils::continuous, 7};
		wmeUtils::wme_channel_cfg_t mMAP  = {wmeUtils::ath1,  "MAP", wmeUtils::WSM_PSID_MAP,  172, wmeUtils::continuous, 7};
		wmeUtils::wme_channel_cfg_t mSRM  = {wmeUtils::ath0,  "SRM", wmeUtils::WSM_PSID_SRM,  182, wmeUtils::continuous, 7};
		wmeUtils::wme_channel_cfg_t mSSM  = {wmeUtils::ath0,  "SSM", wmeUtils::WSM_PSID_SSM,  182, wmeUtils::continuous, 7};
		retn.push_back(mBSM);
		retn.push_back(mSPaT);
		retn.push_back(mMAP);
		retn.push_back(mSRM);
		retn.push_back(mSSM);
		return(retn);
	};
	static const std::vector<wmeUtils::wme_channel_cfg_t> wmeAppChannelMap = wmeUtils::creat_wmeAppChannelMap();

	static const wmeUtils::wme_iface_cfg_t wme_iface_cfg[2] = {{wmeUtils::ath0, wmeUtils::user}, {wmeUtils::ath1, wmeUtils::user}};

	uint8_t   getpsidlen(const uint8_t& ch);
	size_t    upack_wsmp_header(const uint8_t* buf, size_t size, wmeUtils::wsmp_header_t& header);
	size_t    pack_wsmp_header(const wmeUtils::wsmp_header_t& header, uint8_t* buf, size_t size);
	uint32_t  getpsidbymsgid(uint8_t id);
	uint8_t   getmsgidbypsid(uint32_t id);
};

#endif
