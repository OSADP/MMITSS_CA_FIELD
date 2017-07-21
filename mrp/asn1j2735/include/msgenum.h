//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************
#ifndef _DSRCMSGENUM_H
#define _DSRCMSGENUM_H

#include <cstdint>

namespace MsgEnum
{
	static const uint8_t patternFlashing = 0xFE;
	static const uint8_t patternFree = 0xFF;
	static const unsigned long long mapInterval = 1000;  /// in milliseconds
	static const unsigned long long ssmInterval = 1000;  /// in milliseconds
	static const uint32_t invalid_timeStampMinute = 527040;
	static const int32_t  unknown_elevation = -4096;
	static const uint16_t unknown_timeDetail = 36001;
	static const uint16_t unknown_speed = 8191;

	enum class approachType    : uint8_t {inbound = 1, outbound, crosswalk = 4};
	enum class laneType        : uint8_t {traffic = 1, crosswalk = 4};
	enum class maneuverType    : uint8_t {unavailable, uTurn, leftTurn, rightTurn, straightAhead, straight};
	enum class polygonType     : uint8_t {colinear, concave, convex};
	enum class phaseState      : uint8_t {unavailable, dark, flashingRed, redLight, preMovement,
			permissiveGreen, protectedGreen, permissiveYellow, protectedYellow, flashingYellow};
	enum class requestType     : uint8_t {reserved, priorityRequest, requestUpdate, priorityCancellation};
	enum class requestStatus   : uint8_t {unavailable, requested, processing, watchOtherTraffic, granted,
			rejected, maxPresence, reserviceLocked};
	enum class basicRole       : uint8_t {unavailable = 8, truck, motorcycle, roadsideSource, police, fire, ambulance,
			DOT, transit, slowMoving, stopNgo, cyclist, pedestrian, nonMotorized, military};
	enum class vehicleType     : uint8_t {unavailable, notApply, special, moto, car, carOther, bus, axleCnt2, axleCnt3, axleCnt4,
			axleCnt4Trailer, axleCnt5Trailer, axleCnt6Trailer, axleCnt5MultiTrailer, axleCnt6MultiTrailer, axleCnt7MultiTrailer};
	enum class engageStatus    : uint8_t {unavailable, off, on, engaged};
	enum class transGear       : uint8_t {neutral, park, forward, reverse, unavailable = 7};
	enum class mapLocType      : uint8_t {outside, insideIntersectionBox, onInbound, atIntersectionBox, onOutbound};
	enum class laneLocType     : uint8_t {outside, approaching, inside, leaving};
	enum class controlMode     : uint8_t {unavailable, flashing, preemption, runningFree, coordination};
	enum class phaseCallType   : uint8_t {none, vehicle, ped, bike};
	enum class phaseRecallType : uint8_t {none, minimum, maximum, ped, bike};
	enum class softCallObj     : uint8_t {none, ped, vehicle, priority};
	enum class softCallType    : uint8_t {none, call, extension, cancel};
}

#endif
