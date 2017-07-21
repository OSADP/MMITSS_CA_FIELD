//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************
#ifndef _MRPLOCAWARE_H
#define _MRPLOCAWARE_H

#include <map>
#include "AsnJ2735Lib.h"
#include "mapDataStruct.h"

class LocAware
{
	private:
		bool initiated;
		bool mapUpdated;
		// store intersection MAP data
		std::vector<NmapData::IntersectionStruct> mpIntersection;
		// map between (intersectionId, laneId) and (intersection, approach, lane)
		// indexes (start from 0) in mpIntersection.
		// key:   (intersectionId << 8) | laneId
		// value: (intIndx << 16) | (appIndx << 8) | laneIndx
		std::map<uint32_t, uint32_t> IndexMap;
		// for saving updated MapData into file
		std::string nmapFileName;

		// processing intersection nmap file
		bool readNmap(const std::string& fname);
		void saveNmap(const std::string& fname) const;
		void setLocalOffsetAndHeading(void);
		void buildPolygons(void);
		void buildPolygons(NmapData::IntersectionStruct& intObj);

		// UPER encoding MapData
		size_t encode_mapdata_payload(void);
		// add new MAP
		void addIntersection(const NmapData::IntersectionStruct& intObj);

		// get static map data elements
		uint8_t getMapVersion(uint16_t intersectionId) const;
		std::vector<uint8_t> getIndexesByIds(uint16_t intersectionId, uint8_t laneId) const;
		uint8_t getControlPhaseByLaneId(uint16_t intersectionId, uint8_t laneId) const;
		uint8_t getControlPhaseByAprochId(uint16_t intersectionId, uint8_t approachId) const;
		std::vector<uint16_t> getIdsByIndexes(uint8_t intersectionIndx, uint8_t approachIndx, uint8_t laneIndx) const;
		// locating vehicle BSM on intersection Map
		bool isPointNearIntersection(uint8_t intersectionIndex, const GeoUtils::geoPoint_t& geoPoint) const;
		bool isPointInsideIntersectionBox(uint8_t intersectionIndex, const GeoUtils::point2D_t& ptENU) const;
		bool isPointOnApproach(uint8_t intersectionIndex, uint8_t approachIndex, const GeoUtils::point2D_t& ptENU) const;
		std::vector<uint8_t> nearedIntersections(const GeoUtils::geoPoint_t& geoPoint) const;
		std::vector<uint8_t> onApproaches(uint8_t intersectionIndex, const GeoUtils::point2D_t& ptENU) const;
		GeoUtils::laneTracking_t projectPt2Lane(uint8_t intersectionIndex, uint8_t approachIndex, uint8_t laneIndex,
			const GeoUtils::point2D_t& ptENU, const GeoUtils::motion_t& motionState) const;
		bool locateVehicleOnApproach(uint8_t intersectionIndex, uint8_t approachIndex, const GeoUtils::point2D_t& ptENU,
			const GeoUtils::motion_t& motionState, GeoUtils::vehicleTracking_t& vehicleTrackingState) const;
		bool isEgressConnect2Ingress(uint8_t intersectionIndex, uint8_t approachIndex, uint8_t laneIndex, const GeoUtils::geoPoint_t& geoPoint,
			const GeoUtils::motion_t& motionState, GeoUtils::vehicleTracking_t& vehicleTrackingState) const;
		double getPtDist2egress(uint8_t intersectionIndex, uint8_t approachIndex, const GeoUtils::point2D_t& ptENU) const;


	public:
		LocAware(const std::string& fname);
		~LocAware(void);

		// check MAP update based on encoded MAP payload
		void checkNmapUpdate(const MapData_element_t& mapData);

		// get static map data elements
		bool isInitiated(void) const;
		std::vector<uint16_t> getIntersectionIds(void) const;
		std::string getIntersectionNameById(uint16_t intersectionId) const;
		uint16_t getIntersectionIdByName(const std::string& name) const;
		uint8_t  getIndexByIntersectionId(uint16_t intersectionId) const;
		uint8_t  getLaneIdByIndexes(uint8_t intersectionIndx, uint8_t approachIndx, uint8_t laneIndx) const;
		uint8_t  getControlPhaseByIds(uint16_t intersectionId, uint8_t approachId, uint8_t laneId) const;
		uint8_t  getApproachIdByLaneId(uint16_t intersectionId, uint8_t laneId) const;
		uint32_t getLaneLength(uint16_t intersectionId, uint8_t laneId) const;
		GeoUtils::geoRefPoint_t getIntersectionRefPoint(uint8_t intersectionIndx) const;
		std::string getIntersectionNameByIndex(uint8_t intersectionIndex) const;
		std::vector<uint8_t> getMapdataPayload(uint16_t intersectionId) const;
		bool getSpeedLimits(std::vector<uint8_t>& speedLimits, uint16_t intersectionId) const;
		// locating vehicle BSM on intersection Map
		bool locateVehicleInMap(const GeoUtils::connectedVehicle_t& cv, GeoUtils::vehicleTracking_t& cvTrackingState) const;
		void updateLocationAware(const GeoUtils::vehicleTracking_t& vehicleTrackingState, GeoUtils::locationAware_t& vehicleLocationAware) const;
		void getPtDist2D(const GeoUtils::vehicleTracking_t& vehicleTrackingState, GeoUtils::point2D_t& pt) const;
};

#endif
