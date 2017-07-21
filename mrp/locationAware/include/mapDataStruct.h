//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************
#ifndef _MAP_DATA_STRUCT_H
#define _MAP_DATA_STRUCT_H

#include <cstdint>
#include <bitset>
#include <string>
#include <vector>

#include "msgEnum.h"
#include "geoUtils.h"

namespace NmapData
{
	static const double lowSpeedThreshold = 0.2;    // in m/s; speed lower than this heading is questionable
	static const double headingErrorBoundNormal = 45.0;     // in degree
	static const double headingErrorBoundLowSpeed = 200.0;  // in degree
	static const double laneWidthRatio = 1.5;       // for geofencing

	struct ConnectStruct
	{
		uint16_t  intersectionId;
		uint8_t   laneId;
		MsgEnum::maneuverType laneManeuver;
	};

	struct NodeStruct
	{ // order of node sequence starts from near intersection box towards downstream (egress) or upstream (approach)
		GeoUtils::geoRefPoint_t geoNode;
		GeoUtils::point2D_t ptNode;   // in centimeter, reference to intersection reference point
		uint32_t dTo1stNode;          // in centimeter, reference to the first node on the same lane
		uint16_t heading;             // in decidegree
	};

	struct LaneStruct
	{ // an approach can have variable lane objectives.
		// order of lane array starts at the curb lane.
		// lanes within an approach can be controlled by different traffic lights
		// (e.g., left-turn, through, and right-turn movements on one approach).
		uint8_t   id;     // starts from 1
		MsgEnum::laneType type;
		std::bitset<20> attributes;
			/*  vehicular traffic lane with bits as defined:
			 *    isVehicleRevocableLane                (0)
			 *    isVehicleFlyOverLane                  (1)
			 *    hovLaneUseOnly                        (2)
			 *    restrictedToBusUse                    (3)
			 *    restrictedToTaxiUse                   (4)
			 *    restrictedFromPublicUse               (5)
			 *    hasIRbeaconCoverage                   (6)
			 *    permissionOnRequest                   (7)
			 *    maneuverStraightAllowed               (8)
			 *    maneuverLeftAllowed                   (9)
			 *    maneuverRightAllowed                  (10)
			 *    maneuverUTurnAllowed                  (11)
			 *    maneuverLeftTurnOnRedAllowed          (12)
			 *    maneuverRightTurnOnRedAllowed         (13)
			 *    maneuverLaneChangeAllowed             (14)
			 *    maneuverNoStoppingAllowed             (15)
			 *    yieldAllwaysRequired                  (16)
			 *    goWithHalt                            (17)
			 *    caution                               (18)
			 *    reserved                              (19)
			 *
			 *  crosswalk with bits as defined:
			 *    crosswalkRevocableLane                (0)
			 *    bicyleUseAllowed                      (1)
			 *    isXwalkFlyOverLane                    (2)
			 *    fixedCycleTime                        (3) - recall
			 *    biDirectionalCycleTimes               (4)
			 *    hasPushToWalkButton                   (5)
			 *    audioSupport                          (6)
			 *    rfSignalRequestPresent                (7)
			 *    unsignalizedSegmentsPresent           (8)
			 *    reserved                              (9-19)
			 */
		uint16_t  width;        // in centimeter
		uint8_t   controlPhase; // 1 - 8
		std::vector<NmapData::ConnectStruct> mpConnectTo;
		std::vector<NmapData::NodeStruct> mpNodes;
	};

	struct ApproachStruct
	{ // an regular intersection has 12 approach objects: 8 for motor vehicles and 4 crosswalks.
		// an approach with 0 number of lanes does not physically exist, and is not sent.
		// order of approach array starts at westbound inbound and goes clockwise to northbound outbound.
		uint8_t id;           // starts from 1
		uint8_t speed_limit;  // in mph
		MsgEnum::approachType type;
		std::vector<NmapData::LaneStruct> mpLanes;
		std::vector<GeoUtils::point2D_t> mpPolygon;
		MsgEnum::polygonType mpPolygonType;
		uint32_t mindist2intsectionCentralLine; // in centimeter
	};

	struct IntersectionStruct
	{
		uint8_t       mapVersion;
		std::string   name;
		std::string   rsuId;
		uint16_t      id;
		std::bitset<8> attributes;
			/*  with bits as defined:
			 *    Elevation data is included            (0)
			 *    Geometric data is included            (1)
			 *    Speed limit data is included          (2)
			 *    Navigational data is included         (3)
			 *    Reserved                              (4-7)
			 */
		GeoUtils::geoRefPoint_t geoRef;
		GeoUtils::enuCoord_t    enuCoord;
		uint32_t  radius;  // in centimeter
		std::vector<uint8_t> speeds;
		std::vector<NmapData::ApproachStruct> mpApproaches;
		std::vector<GeoUtils::point2D_t> mpPolygon;
		MsgEnum::polygonType mpPolygonType;
		std::vector<uint8_t> mapPayload;
	};
};

#endif
