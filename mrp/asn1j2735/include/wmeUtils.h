#ifndef _WME_UITLS_H
#define _WME_UITLS_H

#include <stdint.h>   // c++11 <cstdint>
#include <string>
#include <cstring>
#include <map>

#include "msgenum.h"

#define MAXDSRCMSGSIZE  2000

namespace wmeUtils
{
  struct channelmode_enum_t
  {
    enum channelMode{CONTINUOUS = 0,ALTERNATING};
    enum channelInterface{ATH0_ = 0,ATH1_ = 1};
  };
  
  /// DSRC message Id for ssm (not in asn1)
  static const uint8_t DSRCmsgID_SSM        = 20;
  /// WME interface
  static const char *iface[]                = {"ath0","ath1"};
  /// WME channel
  static const uint8_t wmechannel[2]        = {182,172};
  /// WME PSID
  static const uint32_t WSM_PSID_BSM        = 0x00000020;
  static const uint32_t WSM_PSID_SPAT       = 0x0000BFE0;
  static const uint32_t WSM_PSID_MAP        = 0x0000BFF0;
  static const uint32_t WSM_PSID_SRM        = 0x00000040;
  static const uint32_t WSM_PSID_SSM        = 0x00000060;
  /// WME priority
  static const uint8_t txPriority           = 7;
  /// WSMP header tag
  static const uint8_t WSMPWAVEID           = 0x80;
  static const uint8_t TRANSMITPW           = 0x04;
  static const uint8_t CHANNUM              = 0x0F;
  static const uint8_t DATARATE             = 0x10;
  static const uint8_t WSMPMAXLEN           = 0x11;
  /// MMITSS msgid for over-the-air messages
  static const uint8_t msgid_bsm            = 0x40;
  static const uint8_t msgid_spat           = 0x41;
  static const uint8_t msgid_map            = 0x42;  
  static const uint8_t msgid_srm            = 0x43;
  static const uint8_t msgid_ssm            = 0x44;    
  static const uint8_t msgid_psrm           = 0x45;

  /// arada wme version
  static const uint8_t arada_wme_version    = 2;
    
  static const char *MSGNAME[] = {"Unknown","BSM","SPaT","MAP","SRM","SSM"};
  
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
  }__attribute__((packed));
    
  /// Arada WSMP UDP header
  struct arada_udp_header_t
  {
    uint8_t     version;
    std::string name;
    std::string iface;
    uint32_t    psid;
    uint8_t     txChannel;
    uint8_t     priority; 
    wmeUtils::channelmode_enum_t::channelMode txMode;
    uint8_t     txInterval;
    bool        signature;
    bool        encryption;
  };
  
  struct wme_channel_config_t
  {
    wmeUtils::channelmode_enum_t::channelInterface txIface;
    std::string iface;
    uint32_t    psid;
    uint8_t     channel;
    wmeUtils::channelmode_enum_t::channelMode txMode;
    uint8_t     priority;     
  };
  
  inline std::map<MsgEnum::msg_enum_t::MSGTYPE,wmeUtils::arada_udp_header_t> creat_arada_header_map(void)
  {
    std::map<MsgEnum::msg_enum_t::MSGTYPE,wmeUtils::arada_udp_header_t> m;
    MsgEnum::msg_enum_t::MSGTYPE msgtype;
    
    msgtype = MsgEnum::msg_enum_t::BSM_PKT;
    wmeUtils::arada_udp_header_t bsmUdpHeader = {arada_wme_version,std::string(wmeUtils::MSGNAME[msgtype]),std::string(wmeUtils::iface[1]),WSM_PSID_BSM,
      wmeUtils::wmechannel[1],txPriority,wmeUtils::channelmode_enum_t::CONTINUOUS,0,false,false};
    m[msgtype] = bsmUdpHeader;
    msgtype = MsgEnum::msg_enum_t::MAP_PKT;
    wmeUtils::arada_udp_header_t mapUdpHeader = {arada_wme_version,std::string(wmeUtils::MSGNAME[msgtype]),std::string(wmeUtils::iface[1]),WSM_PSID_MAP,
      wmeUtils::wmechannel[1],txPriority,wmeUtils::channelmode_enum_t::CONTINUOUS,0,false,false};
    m[msgtype] = mapUdpHeader;
    msgtype = MsgEnum::msg_enum_t::SPAT_PKT;
    wmeUtils::arada_udp_header_t spatUdpHeader = {arada_wme_version,std::string(wmeUtils::MSGNAME[msgtype]),std::string(wmeUtils::iface[1]),WSM_PSID_SPAT,
      wmeUtils::wmechannel[1],txPriority,wmeUtils::channelmode_enum_t::CONTINUOUS,0,false,false};
    m[msgtype] = spatUdpHeader;
    msgtype = MsgEnum::msg_enum_t::SRM_PKT; 
    wmeUtils::arada_udp_header_t srmUdpHeader = {arada_wme_version,std::string(wmeUtils::MSGNAME[msgtype]),std::string(wmeUtils::iface[0]),
      WSM_PSID_SRM,wmeUtils::wmechannel[0],txPriority,wmeUtils::channelmode_enum_t::CONTINUOUS,0,false,false};
    m[msgtype] = srmUdpHeader;
    msgtype = MsgEnum::msg_enum_t::SSM_PKT;
    arada_udp_header_t ssmUdpHeader = {arada_wme_version,std::string(wmeUtils::MSGNAME[msgtype]),std::string(wmeUtils::iface[0]),
      WSM_PSID_SSM,wmeUtils::wmechannel[0],txPriority,wmeUtils::channelmode_enum_t::CONTINUOUS,0,false,false};
    m[msgtype] = ssmUdpHeader;
    return m;
  };  
  static const std::map<MsgEnum::msg_enum_t::MSGTYPE,wmeUtils::arada_udp_header_t> aradaUdpHeaderMap = wmeUtils::creat_arada_header_map();
  
  inline std::map<uint8_t,uint32_t> creat_msgid2psid_map(void)
  {
    std::map<uint8_t,uint32_t> m;
    m[msgid_bsm] = WSM_PSID_BSM;
    m[msgid_srm] = WSM_PSID_SRM;
    m[msgid_spat] = WSM_PSID_SPAT;
    m[msgid_map] = WSM_PSID_MAP;
    m[msgid_ssm] = WSM_PSID_SSM;
    return m; 
  };
  static const std::map<uint8_t,uint32_t> mmitssMsgid2psidMap = wmeUtils::creat_msgid2psid_map();
  
  inline std::map<uint32_t,uint8_t> creat_psid2msg_map(void)
  {
    std::map<uint32_t,uint8_t> m;
    m[WSM_PSID_BSM] = msgid_bsm;
    m[WSM_PSID_SRM] = msgid_srm;
    m[WSM_PSID_SPAT] = msgid_spat;
    m[WSM_PSID_MAP] = msgid_map;
    m[WSM_PSID_SSM] = msgid_ssm;
    return m; 
  };
  static const std::map<uint32_t,uint8_t> psid2mmitssMsgidMap = wmeUtils::creat_psid2msg_map();
  
  inline std::map<std::string,wmeUtils::wme_channel_config_t> creat_app2wmechannel_map(void)
  { 
    std::map<std::string,wmeUtils::wme_channel_config_t> m;
    MsgEnum::msg_enum_t::MSGTYPE msgtype;
    
    msgtype = MsgEnum::msg_enum_t::BSM_PKT;
    wmeUtils::wme_channel_config_t bsmChannelConfg = {wmeUtils::channelmode_enum_t::ATH1_,std::string(wmeUtils::iface[wmeUtils::channelmode_enum_t::ATH1_]),
      WSM_PSID_BSM,wmeUtils::wmechannel[wmeUtils::channelmode_enum_t::ATH1_],wmeUtils::channelmode_enum_t::CONTINUOUS,txPriority};
    m[std::string(wmeUtils::MSGNAME[msgtype])] = bsmChannelConfg;
    msgtype = MsgEnum::msg_enum_t::SPAT_PKT;    
    wmeUtils::wme_channel_config_t spatChannelConfg = {wmeUtils::channelmode_enum_t::ATH1_,std::string(wmeUtils::iface[wmeUtils::channelmode_enum_t::ATH1_]),
      WSM_PSID_SPAT,wmeUtils::wmechannel[wmeUtils::channelmode_enum_t::ATH1_],wmeUtils::channelmode_enum_t::CONTINUOUS,txPriority};
    m[std::string(wmeUtils::MSGNAME[msgtype])] = spatChannelConfg;
    msgtype = MsgEnum::msg_enum_t::MAP_PKT;        
    wmeUtils::wme_channel_config_t mapChannelConfg = {wmeUtils::channelmode_enum_t::ATH1_,std::string(wmeUtils::iface[wmeUtils::channelmode_enum_t::ATH1_]),
      WSM_PSID_MAP,wmeUtils::wmechannel[wmeUtils::channelmode_enum_t::ATH1_],wmeUtils::channelmode_enum_t::CONTINUOUS,txPriority};
    m[std::string(wmeUtils::MSGNAME[msgtype])] = mapChannelConfg;
    msgtype = MsgEnum::msg_enum_t::SRM_PKT;            
    wmeUtils::wme_channel_config_t srmChannelConfg = {wmeUtils::channelmode_enum_t::ATH0_,std::string(wmeUtils::iface[wmeUtils::channelmode_enum_t::ATH0_]),
      WSM_PSID_SRM,wmeUtils::wmechannel[wmeUtils::channelmode_enum_t::ATH0_],wmeUtils::channelmode_enum_t::CONTINUOUS,txPriority};
    m[std::string(wmeUtils::MSGNAME[msgtype])] = srmChannelConfg;
    msgtype = MsgEnum::msg_enum_t::SSM_PKT;                
    wmeUtils::wme_channel_config_t ssmChannelConfg = {wmeUtils::channelmode_enum_t::ATH0_,std::string(wmeUtils::iface[wmeUtils::channelmode_enum_t::ATH0_]),
      WSM_PSID_SSM,wmeUtils::wmechannel[wmeUtils::channelmode_enum_t::ATH0_],wmeUtils::channelmode_enum_t::CONTINUOUS,txPriority};
    m[std::string(wmeUtils::MSGNAME[msgtype])] = ssmChannelConfg;
    return m;
  };  
  static const std::map<std::string,wmeUtils::wme_channel_config_t> wmwAppChannelMap = wmeUtils::creat_app2wmechannel_map();

  uint8_t getpsidlen(const uint8_t* ptr);
  size_t decode_wsmp_header(const uint8_t* ptr,const size_t size,wmeUtils::wsmp_header_t *pwsmp);
  size_t encode_wsmp_header(const wmeUtils::wsmp_header_t *pwsmp,uint8_t* ptr,const size_t size);
  uint32_t getpsidbymsgid(const std::map<uint8_t,uint32_t>& idmap,const uint8_t id);
  uint8_t getmsgiddbypsid(const std::map<uint32_t,uint8_t>& idmap,const uint32_t id);
  void getwsmheaderinfo(const uint8_t* ptr,const size_t size,size_t& offset,size_t& payloadLen);
  std::string fillAradaUdp(MsgEnum::msg_enum_t::MSGTYPE type,const char* ptr,const size_t size);
};

#endif
