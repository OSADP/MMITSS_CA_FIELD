//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************
#ifndef _MRPDATAMGR_H
#define _MRPDATAMGR_H

#include <cstddef>
#include <cstdint>
#include <vector>

#include "msgDefs.h"

size_t packMapMsg(std::vector<uint8_t>& buf, const std::vector<uint8_t>& payload, uint8_t msgid);
size_t packMapMsg(std::vector<uint8_t>& buf, const std::vector<uint8_t>& payload, uint16_t intersectionID, uint8_t msgtype);
bool   getApchPerm(msgDefs::apchPerm_t& apchPerm, std::vector<msgDefs::vehTraj_t>& vehTraj, uint8_t speedLimit, bool permitted);

#endif
