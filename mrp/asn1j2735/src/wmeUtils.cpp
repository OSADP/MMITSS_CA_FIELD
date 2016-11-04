//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************
#include <iostream>
#include <sstream>
#include <iomanip>      // setw, setfill

#include "wmeUtils.h"

using namespace std;

uint8_t wmeUtils::getpsidlen(const uint8_t* ptr)
{
  uint8_t len = 0;
  
  if ((ptr[0] & 0xF0) == 0xE0)
    len = 4;
  else if ((ptr[0] & 0xE0) == 0xC0)
    len = 3;
  else if ((ptr[0] & 0xC0) == 0x80)
    len = 2;
  else if ((ptr[0] & 0x80) == 0x00)
    len = 1;
  
  return len;
}

size_t wmeUtils::decode_wsmp_header(const uint8_t* ptr,const size_t size,wmeUtils::wsmp_header_t *pwsmp)
{
  ///  ptr in network byte order (big-endian)
  size_t offset = 0;
  
  if (!pwsmp)
    return (0);
    
  memset(pwsmp,0,sizeof(wmeUtils::wsmp_header_t));
  
  pwsmp->version = ptr[offset];
  ++offset;
  // wsmp::psid (size 1..4)
  pwsmp->psidlen = wmeUtils::getpsidlen(&ptr[offset]);
  if (pwsmp->psidlen == 0)
    return (0);
  for (size_t i = 0; i < pwsmp->psidlen; i++)
  {
    pwsmp->psid = (pwsmp->psid << 8) | ptr[offset+i];
  }
  offset += pwsmp->psidlen;
    
  // find WSMP waveid
  while(1)
  {
    switch(ptr[offset])
    {
    case wmeUtils::WSMPWAVEID:      
      pwsmp->waveId = ptr[offset];
      break;
    case wmeUtils::CHANNUM:
      offset += 2; // skip elementLen
      pwsmp->channel = ptr[offset];
      break;
    case wmeUtils::DATARATE:
      offset += 2; // skip elementLen
      pwsmp->rate = ptr[offset];
      break;
    case wmeUtils::TRANSMITPW:
      offset += 2; // skip elementLen
      pwsmp->txpower = static_cast<int8_t>(ptr[offset]);
      break;
    default:
      ;
    }
    ++offset;
    if (pwsmp->waveId == wmeUtils::WSMPWAVEID || offset > size || offset > 20)
      break;
  }
  if (pwsmp->waveId != wmeUtils::WSMPWAVEID)
    return 0;
  pwsmp->txlength = static_cast<uint16_t>( ((ptr[offset] << 8) + ptr[offset+1]) & 0xFFFF );
  offset += 2;
  return offset;
}

size_t wmeUtils::encode_wsmp_header(const wmeUtils::wsmp_header_t *pwsmp,uint8_t* ptr,const size_t size)
{
  if (pwsmp->psidlen == 0 || size < WSMPMAXLEN)
    return 0;
    
  size_t offset = 0;
  ptr[offset] = pwsmp->version;
  ++offset;
  for (size_t i = 0; i < pwsmp->psidlen; i++)
  {
    ptr[offset+i] = static_cast<uint8_t>( (pwsmp->psid >> ((pwsmp->psidlen - i - 1)*8)) & 0xFF );
  }
  offset += pwsmp->psidlen; 
  ptr[offset] = CHANNUM;
  ++offset;
  ptr[offset] = 1;
  ++offset; 
  ptr[offset] = pwsmp->channel;
  ++offset; 
  ptr[offset] = DATARATE;
  ++offset; 
  ptr[offset] = 1;
  ++offset; 
  ptr[offset] = pwsmp->rate;
  ++offset; 
  ptr[offset] = TRANSMITPW;
  ++offset; 
  ptr[offset] = 1;
  ++offset; 
  ptr[offset] = static_cast<uint8_t>(pwsmp->txpower);
  ++offset; 
  ptr[offset] = pwsmp->waveId;
  ++offset;
  ptr[offset] = static_cast<uint8_t>((pwsmp->txlength >> 8) & 0xFF);
  ptr[offset+1] = static_cast<uint8_t> (pwsmp->txlength & 0xFF);
  return (offset+2);
}

uint32_t wmeUtils::getpsidbymsgid(const std::map<uint8_t,uint32_t>& idmap,const uint8_t id)
{
  uint32_t psid = 0;
   map<uint8_t,uint32_t>::const_iterator ite = idmap.find(id);
   if (ite != idmap.end()) {psid = ite->second;}
   return (psid);
}

uint8_t wmeUtils::getmsgiddbypsid(const std::map<uint32_t,uint8_t>& idmap,const uint32_t id)
{
  uint8_t msgid = 0;
   map<uint32_t,uint8_t>::const_iterator ite = idmap.find(id);
   if (ite != idmap.end()) {msgid = ite->second;}
   return (msgid);
}

void wmeUtils::getwsmheaderinfo(const uint8_t* ptr,const size_t size,size_t& offset,size_t& payloadLen)
{
  wmeUtils::wsmp_header_t wsmp;
  payloadLen = 0;
  
  offset = wmeUtils::decode_wsmp_header(ptr,size,&wsmp);
  if (offset > 0)
    payloadLen = wsmp.txlength;
}

std::string wmeUtils::fillAradaUdp(MsgEnum::msg_enum_t::MSGTYPE type,const char* ptr,const size_t size)
{
  // for working with Arada OBU/RSU immediate UDP forwarding.  
  // type should be one of the broadcast messages: 
  //  SPaT, MAP and ART on RSU and BSM & SRM on OBU
  wmeUtils::arada_udp_header_t udpHeader;
  map<MsgEnum::msg_enum_t::MSGTYPE,wmeUtils::arada_udp_header_t>::const_iterator it = wmeUtils::aradaUdpHeaderMap.find(type);
  if (it == wmeUtils::aradaUdpHeaderMap.end())
  {
    cerr << "Can't find entry in aradaUdpHeaderMap for message type " << MSGNAME[type] << ", aradaUdpHeaderMap size = " << wmeUtils::aradaUdpHeaderMap.size() << endl;
    return std::string();
  }
  
  udpHeader = it->second;
  ostringstream oss;
  oss << "Version=" << static_cast<int>(udpHeader.version) << endl;
  oss << "Type=" << udpHeader.name << endl;
  oss << "PSID=" << "0x" << hex << uppercase << udpHeader.psid << nouppercase << dec << endl;
  oss << "Priority=" << static_cast<int>(udpHeader.priority) << endl;
  if (udpHeader.txMode == wmeUtils::channelmode_enum_t::CONTINUOUS)
    oss << "TxMode=CONT" << endl;
  else
    oss << "TxMode=ALT" << endl;  
  oss << "TxChannel=" << static_cast<int>(udpHeader.txChannel) << endl;
  oss << "TxInterval=" << static_cast<int>(udpHeader.txInterval) << endl; 
  oss << "DeliveryStart=" << endl;
  oss << "DeliveryStop=" << endl;
  if (udpHeader.signature)
    oss << "Signature=True" << endl;
  else  
    oss << "Signature=False" << endl;
  if (udpHeader.encryption)
    oss << "Encryption=True" << endl;
  else
    oss << "Encryption=False" << endl;
  oss << "Payload=" << hex;
  for (size_t i=0; i<size;i++)
  {
    oss << uppercase << setw(2) << setfill('0') << static_cast<unsigned int>((unsigned char)ptr[i]);
  }
  oss << dec << endl;
  return (oss.str());
} 

