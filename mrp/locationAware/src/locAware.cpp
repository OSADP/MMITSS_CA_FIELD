//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <utility>

#include "dsrcConsts.h"
#include "locAware.h"

LocAware::LocAware(const std::string& fname)
{
	initiated = false;
	mapUpdated = false;
	// read nmap file
	nmapFileName = fname;
	if (!LocAware::readNmap(fname))
	{
		mpIntersection.clear();
		std::cerr << "Failed reading nmap file " << fname << std::endl;
	}
	else
	{
		// calculate local offsets and heading for way-points
		LocAware::setLocalOffsetAndHeading();
		// build approach boxes
		LocAware::buildPolygons();
		// encode MAP payload
		std::cout << "Read " << mpIntersection.size() << " intersections" << std::endl;
		size_t encoded_interections = LocAware::encode_mapdata_payload();
		if (encoded_interections != mpIntersection.size())
		{
			std::cerr << "Encoded " << encoded_interections << " out of " << mpIntersection.size() << " intersections" << std::endl;
			mpIntersection.clear();
		}
		else
		{
			std::cout << "Encoded all intersections" << std::endl;
			initiated = true;
		}
	}
}

LocAware::~LocAware(void)
{
	if(mapUpdated)
	{ // rename nmap file
		std::string backupFileName = nmapFileName + std::string(".old");
		std::rename(nmapFileName.c_str(), backupFileName.c_str());
		// save new MapData into file
		LocAware::saveNmap(nmapFileName);
	}
	mpIntersection.clear();
}

/// --- start of functions to process the intersection nmap file --- ///
std::vector<uint8_t> LocAware::getIndexesByIds(uint16_t intersectionId, uint8_t laneId) const
{
	std::vector<uint8_t> ret;
	uint32_t key = (intersectionId << 8) | laneId;
	auto it = IndexMap.find(key);
	if (it != IndexMap.end())
	{
		uint32_t value = it->second;
		ret.push_back(((value >> 16) & 0xFF));
		ret.push_back(((value >> 8) & 0xFF));
		ret.push_back((value & 0xFF));
	}
	return(ret);
}

bool LocAware::readNmap(const std::string& fname)
{ // open nmap file
	std::ifstream IS_NMAP(fname);
	if (!IS_NMAP.is_open())
	{
		std::cerr << "readNmap: failed open " << fname << std::endl;
		return(false);
	}
	// read nmap
	std::istringstream iss;
	std::string line, s;
	NmapData::IntersectionStruct* pIntersection = nullptr;
	NmapData::ApproachStruct*     pApproach = nullptr;
	NmapData::LaneStruct*         pLane = nullptr;
	uint8_t intersectionCount = 0;
	uint8_t approachCount = 0;
	uint8_t laneCount = 0;
	unsigned int iTmp;
	GeoUtils::geoPoint_t geoPoint;

	bool has_error = false;
	while (!has_error && std::getline(IS_NMAP,line))
	{
		if (line.empty())
			continue;
		if (line.find("MAP_Name") == 0)
		{ // beginning of an intersection nmap
			pIntersection = new NmapData::IntersectionStruct;
			iss.str(line);
			iss >> std::skipws >> s >> pIntersection->name;
			iss.clear();
		}
		else if (line.find("RSU_ID") == 0)
		{
			iss.str(line);
			iss >> std::skipws >> s >> pIntersection->rsuId;
			iss.clear();
		}
		else if (line.find("MAP_Version") == 0)
		{
			iss.str(line);
			iss >> std::skipws >> s >> iTmp;
			iss.clear();
			pIntersection->mapVersion = static_cast<uint8_t>(iTmp & 0xFF);
		}
		else if (line.find("IntersectionID") == 0)
		{
			iss.str(line);
			iss >> std::skipws >> s >> pIntersection->id;
			iss.clear();
		}
		else if (line.find("Intersection_attributes") == 0)
		{
			iss.str(line);
			iss >> std::skipws >> s >> s;
			pIntersection->attributes = std::bitset<8>(s);
			iss.clear();
		}
		else if (line.find("Reference_point") == 0)
		{
			iss.str(line);
			if (pIntersection->attributes.test(0))
			{ // reference point includes elevation
				iss >> std::skipws >> s >> geoPoint.latitude >> geoPoint.longitude >> geoPoint.elevation;
				geoPoint.elevation /= DsrcConstants::deca;
			}
			else
			{
				iss >> std::skipws >> s >> geoPoint.latitude >> geoPoint.longitude;
				geoPoint.elevation = 0.0;
			}
			iss.clear();
			GeoUtils::geoPoint2geoRefPoint(geoPoint,pIntersection->geoRef);
			GeoUtils::setEnuCoord(geoPoint,pIntersection->enuCoord);
		}
		else if (line.find("No_Approach") == 0)
		{ // beginning of ApproachStruct
			approachCount = 0;
			iss.str(line);
			iss >> std::skipws >> s >> iTmp;
			iss.clear();
			if (iTmp > 0)
			{
				pIntersection->mpApproaches.resize(iTmp);
				pApproach = &(pIntersection->mpApproaches[approachCount]);
			}
			else
			{
				std::cerr << "readNmap: invalid No_Approach for intersection " << pIntersection->name << std::endl;
				has_error = true;
			}
		}
		else if (line.find("Approach_type") == 0)
		{
			iss.str(line);
			iss >> std::skipws >> s >> iTmp;
			iss.clear();
			MsgEnum::approachType type = static_cast<MsgEnum::approachType>(iTmp);
			if ((type >= MsgEnum::approachType::inbound) && (type <= MsgEnum::approachType::crosswalk))
			{
				pApproach->type = type;
				if (!pIntersection->attributes.test(2))
					pApproach->speed_limit = (type == MsgEnum::approachType::crosswalk) ? 0 : 0xFF;
			}
			else
			{
				std::cerr << "readNmap: invalid Approach_type for intersection " << pIntersection->name;
				std::cerr << " approach " << static_cast<int>(pApproach->id) << std::endl;
				has_error = true;
			}
		}
		else if (line.find("Speed_limit") == 0)
		{
			iss.str(line);
			iss >> std::skipws >> s >> iTmp;
			iss.clear();
			pApproach->speed_limit = static_cast<uint8_t>(iTmp & 0xFF);
		}
		else if (line.find("Approach") == 0)
		{
			iss.str(line);
			iss >> std::skipws >> s >> iTmp;
			iss.clear();
			pApproach->id = static_cast<uint8_t>(iTmp & 0xFF);
		}
		else if (line.find("No_lane") == 0)
		{ // beginning of LaneStruct
			iss.str(line);
			iss >> std::skipws >> s >> iTmp;
			iss.clear();
			laneCount = 0;
			if (iTmp > 0)
			{ // not to include faked approaches with 0 number of lanes
				pApproach->mpLanes.resize(iTmp);
				pLane = &(pApproach->mpLanes[laneCount]);
				// add distinct speed limits to speeds array
				if (std::find(pIntersection->speeds.begin(), pIntersection->speeds.end(),
						pApproach->speed_limit) == pIntersection->speeds.end())
					pIntersection->speeds.push_back(pApproach->speed_limit);
			}
		}
		else if (line.find("Lane_ID") == 0)
		{
			iss.str(line);
			iss >> std::skipws >> s >> iTmp;
			iss.clear();
			pLane->id = static_cast<uint8_t>(iTmp & 0xFF);
			uint32_t key = (pIntersection->id << 8) | pLane->id;
			IndexMap[key] = (intersectionCount << 16) | (approachCount << 8) | laneCount;
		}
		else if (line.find("Lane_type") == 0)
		{
			iss.str(line);
			iss >> std::skipws >> s >> iTmp;
			iss.clear();
			MsgEnum::laneType type = static_cast<MsgEnum::laneType>(iTmp);
			if ((type >= MsgEnum::laneType::traffic) && (type <= MsgEnum::laneType::crosswalk))
				pLane->type = type;
			else
			{
				std::cerr << "readNmap: invalid Lane_type for intersection " << pIntersection->name;
				std::cerr << " approach " << static_cast<int>(pApproach->id);
				std::cerr << " lane " << static_cast<int>(pLane->id) << std::endl;
				has_error = true;
			}
		}
		else if (line.find("Lane_attributes") == 0)
		{
			iss.str(line);
			iss >> std::skipws >> s >> s;
			iss.clear();
			pLane->attributes = std::bitset<20>(s);
		}
		else if (line.find("Lane_width") == 0)
		{
			iss.str(line);
			iss >> std::skipws >> s >> pLane->width;
			iss.clear();
		}
		else if (line.find("Lane") == 0)
		{
			iss.str(line);
			iss >> std::skipws >> s >> s >> iTmp;
			iss.clear();
			if (iTmp <= 8)
				pLane->controlPhase = static_cast<uint8_t>(iTmp & 0xFF);
			else
			{
				std::cerr << "readNmap: invalid control phase for intersection " << pIntersection->name;
				std::cerr << " approach " << static_cast<int>(pApproach->id);
				std::cerr << " lane " << static_cast<int>(pLane->id) << std::endl;
				has_error = true;
			}
		}
		else if (line.find("No_nodes") == 0)
		{ // beginning of NodeStruct
			unsigned int nodeNums;
			iss.str(line);
			iss >> std::skipws >> s >> nodeNums;
			iss.clear();
			if ((nodeNums >= 2) && (nodeNums <= 63))
			{ // read nodeNums of nodes
				pLane->mpNodes.resize(nodeNums);
				for (unsigned int i = 0; i < nodeNums; i++)
				{
					std::getline(IS_NMAP,line);
					iss.str(line);
					iss >> std::skipws >> s >> geoPoint.latitude >> geoPoint.longitude;
					iss.clear();
					auto& nodeObj = pLane->mpNodes[i];
					geoPoint.elevation = DsrcConstants::hecto2unit<int32_t>(pIntersection->geoRef.elevation);
					GeoUtils::geoPoint2geoRefPoint(geoPoint, nodeObj.geoNode);
				}
			}
			else
			{
				std::cerr << "readNmap: invalid No_nodes phase for intersection " << pIntersection->name;
				std::cerr << " approach " << static_cast<int>(pApproach->id);
				std::cerr << " lane " << static_cast<int>(pLane->id) << std::endl;
				has_error = true;
			}
		}
		else if (line.find("No_Conn_lane") == 0)
		{ // beginning of ConnectStruct
			unsigned int connectionNums;
			iss.str(line);
			iss >> std::skipws >> s >> connectionNums;
			iss.clear();
			if (connectionNums > 0)
			{ // read connectionNums of connectTo
				unsigned int connectToIntersetionId;
				unsigned short connectToApproachIndx; // start from 1
				unsigned short connectToLaneIndx;     // start from 1
				pLane->mpConnectTo.resize(connectionNums);
				for (unsigned int i = 0; i < connectionNums; i++)
				{
					std::getline(IS_NMAP,line);
					iss.str(line);
					iss >> std::skipws >> s >> iTmp;
					iss.clear();
					MsgEnum::maneuverType type = static_cast<MsgEnum::maneuverType>(iTmp);
					if (type > MsgEnum::maneuverType::straight)
					{
						std::cerr << "readNmap: invalid connectTo laneManeuver for intersection " << pIntersection->name;
						std::cerr << " approach " << static_cast<int>(pApproach->id);
						std::cerr << " lane " << static_cast<int>(pLane->id) << std::endl;
						has_error = true;
						break;
					}
					if (std::sscanf(s.c_str(), "%u.%hu.%hu", &connectToIntersetionId, &connectToApproachIndx, &connectToLaneIndx) != 3)
					{
						std::cerr << "readNmap: failed parsing connectTo for intersection " << pIntersection->name;
						std::cerr << " approach " << static_cast<int>(pApproach->id);
						std::cerr << " lane " << static_cast<int>(pLane->id) << std::endl;
						has_error = true;
						break;
					}
					auto& connObj = pLane->mpConnectTo[i];
					connObj.intersectionId = static_cast<uint16_t>(connectToIntersetionId);
					connObj.laneId = static_cast<uint8_t>(((--connectToApproachIndx & 0x0F) << 4) | (--connectToLaneIndx & 0x0F));
					connObj.laneManeuver = type;
				}
			}
		}
		else if (line.find("end_lane") == 0)
		{ // end of LaneStruct
			laneCount++;
			pLane = (laneCount < pApproach->mpLanes.size()) ? &(pApproach->mpLanes[laneCount]) : nullptr;
		}
		else if (line.find("end_approach") == 0)
		{ // end of ApproachStruct
			approachCount++;
			pApproach = (approachCount < pIntersection->mpApproaches.size()) ? &(pIntersection->mpApproaches[approachCount]) : nullptr;
		}
		else if (line.find("end_map") == 0)
		{ // end of IntersectionStruct
			if (pIntersection->speeds.empty())
			{
				std::cerr << "readNmap: missing speed limit for intersection " << pIntersection->name << std::endl;
				has_error = true;
			}
			if (!has_error)
			{
				mpIntersection.push_back(*pIntersection);
				delete pIntersection;
				pIntersection = nullptr;
				pApproach = nullptr;
				pLane = nullptr;
				intersectionCount++;
			}
		}
	}
	IS_NMAP.close();
	delete pIntersection;
	if (has_error)
		return(false);
	/// assign ConnectStruct::laneId
	for (auto& intObj : mpIntersection)
	{
		for (auto& appObj : intObj.mpApproaches)
		{
			for (auto& laneObj : appObj.mpLanes)
			{
				for (auto& conn2obj : laneObj.mpConnectTo)
				{
					uint8_t intersectionIndx = LocAware::getIndexByIntersectionId(conn2obj.intersectionId);
					if (intersectionIndx == 0xFF)
					{
						std::cerr << "readNmap: getIndexByIntersectionId " << conn2obj.intersectionId << std::endl;
						return(false);
					}
					uint8_t laneId = LocAware::getLaneIdByIndexes(intersectionIndx, (conn2obj.laneId >> 4) & 0x0F, conn2obj.laneId & 0x0F);
					if (laneId == 0)
					{
						std::cerr << "readNmap: getLaneIdByIndexes " << conn2obj.intersectionId;
						std::cerr << ", appIndx " << (unsigned)(((conn2obj.laneId) >> 4) & 0x0F);
						std::cerr << ", laneIndx " << (unsigned)((conn2obj.laneId) & 0x0F) << std::endl;
						return(false);
					}
					conn2obj.laneId = laneId;
				}
			}
		}
	}
	return(true);
}

void LocAware::saveNmap(const std::string& fname) const
{
	std::ofstream OS_NMAP(fname);
	if (!OS_NMAP.is_open())
	{
		std::cerr << "saveNmap: failed open " << fname << std::endl;
		return;
	}
	OS_NMAP << mpIntersection.size() << " Intersections" << std::endl;
	OS_NMAP << "--------------------------------------------------------------------------------" << std::endl;
	for (const auto& intObj : mpIntersection)
	{
		GeoUtils::geoPoint_t geoPoint;
		GeoUtils::geoRefPoint2geoPoint(intObj.geoRef, geoPoint);
		OS_NMAP << "MAP_Name " << intObj.name << std::endl;
		OS_NMAP << "RSU_ID " << intObj.rsuId << std::endl;
		OS_NMAP << "MAP_Version " << static_cast<int>(intObj.mapVersion) << std::endl;
		OS_NMAP << "IntersectionID " << intObj.id << std::endl;
		OS_NMAP << "Intersection_attributes " << intObj.attributes.to_string() << std::endl;
		OS_NMAP << "Reference_point " <<  std::fixed << std::setprecision(9);
		OS_NMAP << geoPoint.latitude << " " << geoPoint.longitude << " ";
		OS_NMAP << std::setprecision(2) << geoPoint.elevation * DsrcConstants::deca << std::endl;
		OS_NMAP << "No_Approach " << intObj.mpApproaches.size() << std::endl;
		for (const auto& appObj : intObj.mpApproaches)
		{
			OS_NMAP << "Approach " << static_cast<int>(appObj.id) << std::endl;
			OS_NMAP << "Approach_type " << static_cast<int>(appObj.type) << std::endl;
			OS_NMAP << "Speed_limit " <<   static_cast<int>(appObj.speed_limit) << std::endl;
			OS_NMAP << "No_lane " << appObj.mpLanes.size() << std::endl;
			unsigned int laneSeq = 0;
			for (const auto& laneObj : appObj.mpLanes)
			{
				laneSeq++;
				OS_NMAP << "Lane " << static_cast<int>(appObj.id) << "." << laneSeq << " ";
				OS_NMAP << static_cast<int>(laneObj.controlPhase) << std::endl;
				OS_NMAP << "Lane_ID " << static_cast<int>(laneObj.id) << std::endl;
				OS_NMAP << "Lane_type " << static_cast<int>(laneObj.type) << std::endl;
				OS_NMAP << "Lane_attributes " << laneObj.attributes.to_string() << std::endl;
				OS_NMAP << "Lane_width " << static_cast<int>(laneObj.width) << std::endl;
				OS_NMAP << "No_nodes " << laneObj.mpNodes.size() << std::endl;
				unsigned int nodeSeq = 0;
				for (const auto& nodeObj : laneObj.mpNodes)
				{
					nodeSeq++;
					GeoUtils::geoRefPoint2geoPoint(nodeObj.geoNode, geoPoint);
					OS_NMAP << static_cast<int>(appObj.id) << "." << laneSeq << "." << nodeSeq << " ";
					OS_NMAP << std::fixed << std::setprecision(9) << geoPoint.latitude;
					OS_NMAP << " " << geoPoint.longitude << std::endl;
				}
				OS_NMAP << "No_Conn_lane " << laneObj.mpConnectTo.size() << std::endl;
				for (const auto& connObj : laneObj.mpConnectTo)
				{
					auto inds = LocAware::getIndexesByIds(connObj.intersectionId, connObj.laneId);
					if (!inds.empty())
					{
						OS_NMAP << connObj.intersectionId << "." << static_cast<int>(inds[1] + 1) << ".";
						OS_NMAP << static_cast<int>(inds[2] + 1) << " " << static_cast<int>(connObj.laneManeuver) << std::endl;
					}
				}
				OS_NMAP << "end_lane" << std::endl;
			}
			OS_NMAP << "end_approach" << std::endl;
		}
		OS_NMAP << "end_map" << std::endl;
		OS_NMAP << "--------------------------------------------------------------------------------" << std::endl;
	}
	OS_NMAP.close();
}

void LocAware::setLocalOffsetAndHeading(void)
{
	GeoUtils::point2D_t origin{0,0};
	// ptNode, dTo1stNode
	for (auto& intObj : mpIntersection)
	{
		uint32_t radius = 0;
		for (auto& appObj : intObj.mpApproaches)
		{
			for (auto& laneObj : appObj.mpLanes)
			{
				uint32_t dTo1stNode = 0;
				for (size_t i = 0, j = laneObj.mpNodes.size(); i < j; i++)
				{
					GeoUtils::lla2enu(intObj.enuCoord, laneObj.mpNodes[i].geoNode, laneObj.mpNodes[i].ptNode);
					if (i > 0)
						dTo1stNode += laneObj.mpNodes[i].ptNode.distance2pt(laneObj.mpNodes[i-1].ptNode);
					laneObj.mpNodes[i].dTo1stNode = dTo1stNode;
					uint32_t ptLength = laneObj.mpNodes[i].ptNode.length();
					if (ptLength > radius)
						radius = ptLength;
				}
				// reset dTo1stNode for the first node:
				// For traffic lanes, project intersection ref point (i.e., origin) onto the closed segment on lane,
				//  set distance from the closed way-point to the projected point as dTo1stNode for the closed way-point
				//  this is helpful for applications that crossing the stop-bar will be an event trigger, such as to cancel TSP request.
				//  GPS overshot at stop-bar could cause wrong cancel request. This projected distance to intersection center can be used
				//  to ensure the vehicle has crossed the stop-bar.
				//  This value won't affect calculation of distance to the stop-bar.
				GeoUtils::projection_t proj2segment;
				switch (appObj.type)
				{
				case MsgEnum::approachType::inbound:
					GeoUtils::projectPt2Line(laneObj.mpNodes[1].ptNode, laneObj.mpNodes[0].ptNode, origin, proj2segment);
					laneObj.mpNodes[0].dTo1stNode  = static_cast<uint32_t>(std::fabs((proj2segment.t -1)* proj2segment.length));
					break;
				case MsgEnum::approachType::outbound:
					GeoUtils::projectPt2Line(laneObj.mpNodes[0].ptNode, laneObj.mpNodes[1].ptNode, origin, proj2segment);
					laneObj.mpNodes[0].dTo1stNode  = static_cast<uint32_t>(std::fabs((proj2segment.t) * proj2segment.length));
					break;
				case MsgEnum::approachType::crosswalk:
					break;
				}
			}
			// get mindist2intsectionCentralLine in centimeter
			appObj.mindist2intsectionCentralLine = (appObj.mpLanes.empty()) ? 2000 : appObj.mpLanes[0].mpNodes[0].dTo1stNode;
			for (auto& laneObj : appObj.mpLanes)
			{
				if (laneObj.mpNodes[0].dTo1stNode < appObj.mindist2intsectionCentralLine)
					appObj.mindist2intsectionCentralLine = laneObj.mpNodes[0].dTo1stNode;
			}
		}
		intObj.radius = radius;
	}
	// heading
	for (auto& intObj : mpIntersection)
	{
		for (auto& appObj : intObj.mpApproaches)
		{
			if (appObj.type != MsgEnum::approachType::outbound)
			{ // order of node sequence on inbound lanes start at stop-bar towards upstream
				for (auto& laneObj : appObj.mpLanes)
				{
					for (size_t i = 1, j = laneObj.mpNodes.size(); i < j; i++)
						laneObj.mpNodes[i].heading = laneObj.mpNodes[i].ptNode.direction2pt(laneObj.mpNodes[i-1].ptNode);
					laneObj.mpNodes[0].heading = laneObj.mpNodes[1].heading;
				}
			}
			else
			{ // order of node sequence on outbound lanes starts at cross-walk towards downstream
				for (auto& laneObj : appObj.mpLanes)
				{
					for (size_t i = 0, j = laneObj.mpNodes.size() - 1; i < j; i++)
						laneObj.mpNodes[i].heading = laneObj.mpNodes[i].ptNode.direction2pt(laneObj.mpNodes[i+1].ptNode);
					laneObj.mpNodes.back().heading = laneObj.mpNodes[laneObj.mpNodes.size() - 2].heading;
				}
			}
		}
	}
}

void LocAware::buildPolygons(void)
{
	for (auto& intObj :  mpIntersection)
		LocAware::buildPolygons(intObj);
}

void LocAware::buildPolygons(NmapData::IntersectionStruct& intObj)
{
	const GeoUtils::point2D_t origin{0,0};
	std::vector< std::pair<GeoUtils::point2D_t, GeoUtils::point2D_t> > nearestWayPointPair(4, std::make_pair(origin,origin));
	// build ApproachPolygon
	for (auto appObj : intObj.mpApproaches)
	{
		std::vector<GeoUtils::point2D_t> farthestWaypoints;
		if ((appObj.type == MsgEnum::approachType::crosswalk) || (appObj.mpLanes.empty()))
			continue;
		uint8_t crosswalkIndx = uint8_t((appObj.id - 1) / 2);
		auto p1 = origin;
		auto p2 = origin;
		for (size_t i = 0, j = appObj.mpLanes.size(); i < j; i++)
		{ // lane starts from curb lane to central lane
			const auto& laneObj = appObj.mpLanes[i];
			double halfWidth = laneObj.width * NmapData::laneWidthRatio;
			if ((i == 0) || (i == j - 1))
			{ // add half lane width to the curb and central lanes
				int32_t direction_flag = (i == 0) ? 1 : (-1);
				double alpha = DsrcConstants::deg2rad(DsrcConstants::deca2unit<uint16_t>(laneObj.mpNodes[0].heading));
				GeoUtils::point2D_t waypoint;
				waypoint.x = laneObj.mpNodes[0].ptNode.x + direction_flag * static_cast<int32_t>(halfWidth * cos(alpha));
				waypoint.y = laneObj.mpNodes[0].ptNode.y - direction_flag * static_cast<int32_t>(halfWidth * sin(alpha));
				appObj.mpPolygon.push_back(waypoint);
				if (i == 0)
					p1 = waypoint;
				else
					p2 = waypoint;

				alpha = DsrcConstants::deg2rad(DsrcConstants::deca2unit<uint16_t>(laneObj.mpNodes.back().heading));
				waypoint.x = laneObj.mpNodes.back().ptNode.x + direction_flag * static_cast<int32_t>(halfWidth * cos(alpha));
				waypoint.y = laneObj.mpNodes.back().ptNode.y - direction_flag * static_cast<int32_t>(halfWidth * sin(alpha));
				farthestWaypoints.push_back(waypoint);
			}
			else
			{ // keep middle lane nearest and furthest way-points
				appObj.mpPolygon.push_back(laneObj.mpNodes.front().ptNode);
				farthestWaypoints.push_back(laneObj.mpNodes.back().ptNode);
			}
		}
		if(nearestWayPointPair[crosswalkIndx].first == origin)
		{
			nearestWayPointPair[crosswalkIndx].first = p1;
			nearestWayPointPair[crosswalkIndx].second = p2;
		}
		else
			nearestWayPointPair[crosswalkIndx].second = p1;
		// add farthestWaypoints to ApproachPolygon
		appObj.mpPolygon.insert(appObj.mpPolygon.end(), farthestWaypoints.rbegin(),farthestWaypoints.rend());
		if (GeoUtils::convexcave(appObj.mpPolygon) != MsgEnum::polygonType::convex)
			appObj.mpPolygon = GeoUtils::convexHullAndrew(appObj.mpPolygon);
		// get polygon type
		appObj.mpPolygonType = GeoUtils::convexcave(appObj.mpPolygon);
	}
	// build IntersectionPolygon
	for (int i = 0; i < 4; i++)
	{
		intObj.mpPolygon.push_back(nearestWayPointPair[i].first);
		intObj.mpPolygon.push_back(nearestWayPointPair[i].second);
	}
	if (GeoUtils::convexcave(intObj.mpPolygon) != MsgEnum::polygonType::convex)
		intObj.mpPolygon = GeoUtils::convexHullAndrew(intObj.mpPolygon);
	intObj.mpPolygonType = GeoUtils::convexcave(intObj.mpPolygon);
}
/// --- end of functions to process the intersection nmap file --- ///



/// --- start of functions to encode MAP and update decoded MAP --- ///
auto speed_mph2mps = [](const uint8_t& speed_limit)->uint16_t
{
	return((speed_limit == 0xFF) ? MsgEnum::unknown_speed :
		((speed_limit == 0) ? 0 : static_cast<uint16_t>(round(DsrcConstants::mph2mps * speed_limit / 0.02))));
};

auto speed_mps2mph = [](const uint16_t& speed_limit)->uint8_t
{
	return((speed_limit ==  MsgEnum::unknown_speed) ? 0xFF :
		((speed_limit == 0) ? 0 : static_cast<uint8_t>(round((double)speed_limit * 0.02 / DsrcConstants::mph2mps))));
};

auto IntObj2MapData = [](const NmapData::IntersectionStruct& intObj, MapData_element_t& mapData)->void
{
	if (intObj.mpApproaches.empty())
		return;
	mapData.reset();
	mapData.id               = intObj.id;
	mapData.mapVersion       = intObj.mapVersion;
	mapData.attributes       = intObj.attributes;
	mapData.geoRef.latitude  = intObj.geoRef.latitude;
	mapData.geoRef.longitude = intObj.geoRef.longitude;
	mapData.geoRef.elevation = (intObj.attributes.test(0)) ?
		static_cast<int32_t>(round((double)intObj.geoRef.elevation / 10.0)) : MsgEnum::unknown_elevation;
	mapData.mpApproaches.resize(intObj.mpApproaches.size());
	size_t appCnt = 0;
	for (const auto& appObj : intObj.mpApproaches)
	{
		auto& appData = mapData.mpApproaches[appCnt++];
		if (appObj.mpLanes.empty())
			continue;
		appData.id          = appObj.id;
		appData.type        = appObj.type;
		appData.speed_limit = speed_mph2mps(appObj.speed_limit);
		if (std::find(mapData.speeds.begin(), mapData.speeds.end(),	appData.speed_limit) == mapData.speeds.end())
			mapData.speeds.push_back(appData.speed_limit);
		appData.mpLanes.resize(appObj.mpLanes.size());
		size_t laneCnt = 0;
		for (const auto& laneObj : appObj.mpLanes)
		{
			auto& laneData = appData.mpLanes[laneCnt++];
			laneData.id           = laneObj.id;
			laneData.type         = laneObj.type;
			laneData.attributes   = laneObj.attributes;
			laneData.width        = laneObj.width;
			laneData.controlPhase = laneObj.controlPhase;
			if (!laneObj.mpConnectTo.empty())
			{
				laneData.mpConnectTo.resize(laneObj.mpConnectTo.size());
				size_t connCnt = 0;
				for (const auto& connObj : laneObj.mpConnectTo)
				{
					auto& connData = laneData.mpConnectTo[connCnt++];
					connData.intersectionId = connObj.intersectionId;
					connData.laneId         = connObj.laneId;
					connData.laneManeuver   = connObj.laneManeuver;
				}
			}
			if (!laneObj.mpNodes.empty())
			{
				laneData.mpNodes.resize(laneObj.mpNodes.size());
				GeoUtils::point2D_t prev_ptNode{0,0};
				size_t nodeCnt = 0;
				for (const auto& nodeObj : laneObj.mpNodes)
				{
					auto& nodeData = laneData.mpNodes[nodeCnt++];
					nodeData.offset_x = nodeObj.ptNode.x - prev_ptNode.x;
					nodeData.offset_y = nodeObj.ptNode.y - prev_ptNode.y;
					prev_ptNode = nodeObj.ptNode;
				}
			}
		}
	}
};

size_t LocAware::encode_mapdata_payload(void)
{
	size_t ret = 0;
	MapData_element_t mapData;
	for (auto& intObj : mpIntersection)
	{
		intObj.mapPayload.resize(2000);
		IntObj2MapData(intObj, mapData);
		size_t payload_size = AsnJ2735Lib::encode_mapdata_payload(mapData, &intObj.mapPayload[0], intObj.mapPayload.size());
		if (payload_size > 0)
		{
			intObj.mapPayload.resize(payload_size);
			intObj.mapPayload.shrink_to_fit();
			ret++;
		}
		else
			intObj.mapPayload.clear();
	}
	return(ret);
}

void LocAware::checkNmapUpdate(const MapData_element_t& mapData)
{
	if (mapData.mpApproaches.empty())
		return;
	// check whether mapData is the same version as that is stored in mpIntersection
	uint8_t mapVersion = LocAware::getMapVersion(mapData.id);
	if (mapVersion == mapData.mapVersion) // same version
		return;
	// this is either a new (mapVersion = 0) or an updated (mapVersion > 0) MapData
	mapUpdated = true;
	NmapData::IntersectionStruct* pIntObj = new NmapData::IntersectionStruct;
	pIntObj->id = mapData.id;
	pIntObj->mapVersion = mapData.mapVersion;
	if (mapVersion > 0)
	{ // mpIntersection has an older version of MapData
		pIntObj->rsuId = LocAware::getIntersectionNameById(mapData.id);
		pIntObj->name = pIntObj->rsuId + std::string(".nmap");
	}
	pIntObj->attributes = mapData.attributes;
	pIntObj->geoRef.latitude  = mapData.geoRef.latitude;
	pIntObj->geoRef.longitude = mapData.geoRef.longitude;
	pIntObj->geoRef.elevation = mapData.geoRef.elevation * 10;
	GeoUtils::setEnuCoord(pIntObj->geoRef, pIntObj->enuCoord);
	pIntObj->mapPayload = mapData.mapPayload;
	pIntObj->radius = 0;
	pIntObj->mpApproaches.resize(mapData.mpApproaches.size());
	auto& speeds = pIntObj->speeds;
	size_t appCnt = 0;
	for (const auto& appData : mapData.mpApproaches)
	{
		auto& appObj = pIntObj->mpApproaches[appCnt++];
		if (appData.mpLanes.empty())
			continue;
		appObj.id = appData.id;
		appObj.speed_limit =  speed_mps2mph(appData.speed_limit);
		if (std::find(speeds.begin(), speeds.end(),	appObj.speed_limit) == speeds.end())
			speeds.push_back(appObj.speed_limit);
		appObj.type = appData.type;
		appObj.mindist2intsectionCentralLine = 0;
		appObj.mpLanes.resize(appData.mpLanes.size());
		size_t laneCnt = 0;
		for (const auto& laneData : appData.mpLanes)
		{
			auto& laneObj = appObj.mpLanes[laneCnt++];
			laneObj.id           = laneData.id;
			laneObj.type         = laneData.type;
			laneObj.attributes   = laneData.attributes;
			laneObj.width        = laneData.width;
			laneObj.controlPhase = laneData.controlPhase;
			if (!laneData.mpConnectTo.empty())
			{
				laneObj.mpConnectTo.resize(laneData.mpConnectTo.size());
				size_t connCnt = 0;
				for (const auto& connData : laneData.mpConnectTo)
				{
					auto& connObj = laneObj.mpConnectTo[connCnt++];
					connObj.intersectionId = connData.intersectionId;
					connObj.laneId = connData.laneId;
					connObj.laneManeuver = connData.laneManeuver;
				}
			}
			if (!laneData.mpNodes.empty())
			{
				laneObj.mpNodes.resize(laneData.mpNodes.size());
				int32_t  pt_x = 0;
				int32_t  pt_y = 0;
				uint32_t dTo1stNode = 0;
				size_t nodeCnt = 0;
				for (const auto& nodeData : laneData.mpNodes)
				{
					auto& nodeObj = laneObj.mpNodes[nodeCnt];
					uint32_t offset_dist = static_cast<uint32_t>(std::sqrt(nodeData.offset_x * nodeData.offset_x + nodeData.offset_y * nodeData.offset_y));
					pt_x += nodeData.offset_x;
					pt_y += nodeData.offset_y;
					if (nodeCnt > 0)
						dTo1stNode += offset_dist;
					double heading = DsrcConstants::rad2deg(atan2(nodeData.offset_x, nodeData.offset_y));
					if (appObj.type != MsgEnum::approachType::outbound)
						heading *= -1.0;
					if (heading < 0.0)
						heading += 360.0;
					uint16_t nodeHeading = static_cast<uint16_t>(heading * 10.0);
					nodeObj.ptNode.x = pt_x;
					nodeObj.ptNode.y = pt_y;
					GeoUtils::enu2lla(pIntObj->enuCoord, nodeObj.ptNode, nodeObj.geoNode);
					nodeObj.dTo1stNode = (nodeCnt == 0) ? offset_dist : dTo1stNode;
					if ((appObj.type == MsgEnum::approachType::outbound) && (nodeCnt > 0))
						laneObj.mpNodes[nodeCnt - 1].heading = nodeHeading;
					else
						nodeObj.heading = nodeHeading;
					if (nodeCnt == 0)
					{
						if (appObj.mindist2intsectionCentralLine == 0)
							appObj.mindist2intsectionCentralLine = nodeObj.dTo1stNode;
						else if (nodeObj.dTo1stNode < appObj.mindist2intsectionCentralLine)
							appObj.mindist2intsectionCentralLine = nodeObj.dTo1stNode;
					}
					uint32_t ptLength = nodeObj.ptNode.length();
					if (ptLength > pIntObj->radius)
						pIntObj->radius = ptLength;
					nodeCnt++;
				}
				if (appObj.type == MsgEnum::approachType::outbound)
					laneObj.mpNodes.back().heading = laneObj.mpNodes[laneObj.mpNodes.size() - 2].heading;
				else
					laneObj.mpNodes[0].heading = laneObj.mpNodes[1].heading;
			}
		}
	}
	LocAware::buildPolygons(*pIntObj);
	LocAware::addIntersection(*pIntObj);
	delete pIntObj;
}

void LocAware::addIntersection(const NmapData::IntersectionStruct& intObj)
{
	uint16_t intersectionId = intObj.id;
	auto it = std::find_if(mpIntersection.begin(), mpIntersection.end(),
		[&intersectionId](const NmapData::IntersectionStruct& obj){return(obj.id == intersectionId);});
	if (it != mpIntersection.end())
		it = mpIntersection.erase(it);
	it = mpIntersection.insert(it, intObj);
	// update IndexMap
	uint8_t intIndx = (uint8_t)(it - mpIntersection.begin());
	for (auto itMap = IndexMap.begin(); itMap != IndexMap.end();)
	{
		if ((uint16_t)((itMap->first) >> 8) == intersectionId)
			itMap = IndexMap.erase(itMap);
		else
			++itMap;
	}
	uint8_t appIndx = 0;
	for (const auto& appObj : intObj.mpApproaches)
	{
		uint8_t laneIndx = 0;
		for (const auto& laneObj : appObj.mpLanes)
		{
			uint32_t key = (intersectionId << 8) | laneObj.id;
			IndexMap[key] = (intIndx << 16) | (appIndx << 8) | laneIndx;
			laneIndx++;
		}
		appIndx++;
	}
}
/// --- end of functions to encode MAP and update decoded MAP --- ///



/// --- start of functions to get static map data elements --- ///
bool LocAware::isInitiated(void) const
	{return(initiated);}

uint8_t LocAware::getMapVersion(uint16_t intersectionId) const
{
	auto it = std::find_if(mpIntersection.begin(), mpIntersection.end(),
		[&intersectionId](const NmapData::IntersectionStruct& obj){return(obj.id == intersectionId);});
	return((it != mpIntersection.end()) ? (it->mapVersion) : 0);
}

std::vector<uint16_t> LocAware::getIntersectionIds(void) const
{
	std::vector<uint16_t> ids;
	for (const auto& item : mpIntersection)
		ids.push_back(item.id);
	return(ids);
}

std::string LocAware::getIntersectionNameById(uint16_t intersectionId) const
{
	auto it = std::find_if(mpIntersection.begin(), mpIntersection.end(),
		[&intersectionId](const NmapData::IntersectionStruct& obj){return(obj.id == intersectionId);});
	return ((it != mpIntersection.end())	? (it->rsuId) : std::string());
}

uint16_t LocAware::getIntersectionIdByName(const std::string& name) const
{
	auto it = std::find_if(mpIntersection.begin(), mpIntersection.end(),
		[&name](const NmapData::IntersectionStruct& obj){return(obj.rsuId.compare(name) == 0);});
	return((it != mpIntersection.end()) ? (it->id) : 0);
}

uint8_t LocAware::getIndexByIntersectionId(uint16_t intersectionId) const
{
	auto it = std::find_if(mpIntersection.begin(), mpIntersection.end(),
		[&intersectionId](const NmapData::IntersectionStruct& obj){return(obj.id == intersectionId);});
	return((uint8_t)((it != mpIntersection.end()) ? (it - mpIntersection.begin()) : 0xFF));
}

std::string LocAware::getIntersectionNameByIndex(uint8_t intersectionIndex) const
{
	return((intersectionIndex < mpIntersection.size()) ? mpIntersection[intersectionIndex].rsuId : std::string());
}

uint8_t LocAware::getControlPhaseByLaneId(uint16_t intersectionId, uint8_t laneId) const
{
	auto inds = LocAware::getIndexesByIds(intersectionId, laneId);
	return((!inds.empty()) ? mpIntersection[inds[0]].mpApproaches[inds[1]].mpLanes[inds[2]].controlPhase : 0);
}

uint8_t LocAware::getControlPhaseByAprochId(uint16_t intersectionId, uint8_t approachId) const
{
	uint8_t intIndex = getIndexByIntersectionId(intersectionId);
	if ((intIndex == 0xFF) || (size_t)approachId > mpIntersection[intIndex].mpApproaches.size())
		return(0);
	auto& lanes = mpIntersection[intIndex].mpApproaches[approachId - 1].mpLanes;
	auto it = std::find_if(lanes.begin(), lanes.end(),
		[](const NmapData::LaneStruct& obj){return(obj.attributes.test(1) == true);});
	return((it == lanes.end()) ? 0 : it->controlPhase);
}

uint8_t  LocAware::getControlPhaseByIds(uint16_t intersectionId, uint8_t approachId, uint8_t laneId) const
{
	if ((approachId == 0) && (laneId == 0))
		return(0);
	if (laneId > 0)
		return(LocAware::getControlPhaseByLaneId(intersectionId, laneId));
	return(LocAware::getControlPhaseByAprochId(intersectionId, approachId));
}

uint8_t LocAware::getApproachIdByLaneId(uint16_t intersectionId, uint8_t laneId) const
{
	auto inds = LocAware::getIndexesByIds(intersectionId, laneId);
	return((!inds.empty()) ? mpIntersection[inds[0]].mpApproaches[inds[1]].id : 0);
}

uint32_t LocAware::getLaneLength(uint16_t intersectionId, uint8_t laneId) const
{
	auto inds = LocAware::getIndexesByIds(intersectionId,laneId);
	return ((!inds.empty()) ? mpIntersection[inds[0]].mpApproaches[inds[1]].mpLanes[inds[2]].mpNodes.back().dTo1stNode : 0);
}

GeoUtils::geoRefPoint_t LocAware::getIntersectionRefPoint(uint8_t intersectionIndx) const
{
	return(mpIntersection[intersectionIndx].geoRef);
}

std::vector<uint16_t> LocAware::getIdsByIndexes(uint8_t intersectionIndx, uint8_t approachIndx, uint8_t laneIndx) const
{
	std::vector<uint16_t> ret;
	if ((intersectionIndx < mpIntersection.size())
		&& (approachIndx < mpIntersection[intersectionIndx].mpApproaches.size())
		&& (laneIndx < mpIntersection[intersectionIndx].mpApproaches[approachIndx].mpLanes.size()))
	{
		ret.push_back(mpIntersection[intersectionIndx].id);
		ret.push_back(mpIntersection[intersectionIndx].mpApproaches[approachIndx].id);
		ret.push_back(mpIntersection[intersectionIndx].mpApproaches[approachIndx].mpLanes[laneIndx].id);
	}
	return(ret);
}

uint8_t LocAware::getLaneIdByIndexes(uint8_t intersectionIndx, uint8_t approachIndx, uint8_t laneIndx) const
{
	auto ids = LocAware::getIdsByIndexes(intersectionIndx, approachIndx, laneIndx);
	return((!ids.empty()) ? (uint8_t)(ids[2] & 0xFF) : 0);
}

std::vector<uint8_t> LocAware::getMapdataPayload(uint16_t intersectionId) const
{
	auto it = std::find_if(mpIntersection.begin(), mpIntersection.end(),
		[&intersectionId](const NmapData::IntersectionStruct& obj){return(obj.id == intersectionId);});
	return((it != mpIntersection.end()) ? it->mapPayload : std::vector<uint8_t>());
}

bool LocAware::getSpeedLimits(std::vector<uint8_t>& speedLimits, uint16_t intersectionId) const
{
	auto it = std::find_if(mpIntersection.begin(), mpIntersection.end(),
		[&intersectionId](const NmapData::IntersectionStruct& obj){return(obj.id == intersectionId);});
	if (it == mpIntersection.end())
		return(false);
	for (auto& appObj : it->mpApproaches)
	{ /// one inbound approach could have multiple control phases (e.g., straight through & protected left-turn)
		std::vector<uint8_t> controlPhases;
		if ((appObj.type == MsgEnum::approachType::inbound) && !appObj.mpLanes.empty())
		{
			for (auto& laneObj : appObj.mpLanes)
			{
				if ((laneObj.controlPhase > 0) && (std::find(controlPhases.begin(), controlPhases.end(), laneObj.controlPhase) == controlPhases.end()))
					controlPhases.push_back(laneObj.controlPhase);
			}
			for (auto& phase : controlPhases)
				speedLimits[phase - 1] = appObj.speed_limit;
		}
	}
	return(true);
}
/// --- end of functions to get static map data elements --- ///



/// --- start of functions to locate BSM on MAP --- ///
bool LocAware::isPointNearIntersection(uint8_t intersectionIndex, const GeoUtils::geoPoint_t& geoPoint) const
{ // check whether geoPt is inside radius of an intersection
	GeoUtils::point3D_t ptENU;
	const auto& intObj = mpIntersection[intersectionIndex];
	GeoUtils::lla2enu(intObj.enuCoord, geoPoint, ptENU);
	return(sqrt(ptENU.x * ptENU.x + ptENU.y * ptENU.y) <= DsrcConstants::hecto2unit<int32_t>(intObj.radius));
}

bool LocAware::isPointInsideIntersectionBox(uint8_t intersectionIndex, const GeoUtils::point2D_t& ptENU) const
{
	return(GeoUtils::isPointInsidePolygon(mpIntersection[intersectionIndex].mpPolygon, ptENU));
}

bool LocAware::isPointOnApproach(uint8_t intersectionIndex, uint8_t approachIndex, const GeoUtils::point2D_t& ptENU) const
{ // Approach polygon is convex. For point inside (or on the polygon edge) a convex polygon,
	// it should be on the same side of all polygon edges when moving counter-clockwise or
	// clockwise along vertices. Check side with vector cross product.
	return (GeoUtils::isPointInsidePolygon(mpIntersection[intersectionIndex].mpApproaches[approachIndex].mpPolygon, ptENU));
}

std::vector<uint8_t> LocAware::nearedIntersections(const GeoUtils::geoPoint_t& geoPoint) const
{
	std::vector<uint8_t> ret;
	for (uint8_t i = 0, j = (uint8_t)(mpIntersection.size()); i < j; i++)
	{
		if (LocAware::isPointNearIntersection(i, geoPoint))
			ret.push_back(i);
	}
	return(ret);
}

std::vector<uint8_t> LocAware::onApproaches(uint8_t intersectionIndex, const GeoUtils::point2D_t& ptENU) const
{ // also do this when geoPoint is near the intersection (check first with isPointNearIntersection)
	std::vector<uint8_t> ret;
	const auto& intObj = mpIntersection[intersectionIndex];
	for (uint8_t i = 0, j = (uint8_t)(intObj.mpApproaches.size()); i < j; i++)
	{
		if (!intObj.mpApproaches[i].mpPolygon.empty() && LocAware::isPointOnApproach(intersectionIndex, i, ptENU))
			ret.push_back(i);
	}
	return(ret);
}

auto getHeadingDifference = [](uint16_t nodeHeading, double ptHeading)->double
{ // headingDiff in [-180..180];
	double d = ptHeading - DsrcConstants::deca2unit<uint16_t>(nodeHeading);
	if (d > 180.0)
		d -= 360.0;
	else if (d < -180.0)
		d += 360.0;
	return(d);
};

auto getHeadingErrorBound = [](double ptSpeed)->double
{
	return((ptSpeed < NmapData::lowSpeedThreshold) ? NmapData::headingErrorBoundLowSpeed : NmapData::headingErrorBoundNormal);
};

auto getIdxByLocationType = [](const std::vector<GeoUtils::vehicleTracking_t>& aVehicleTrackingState)->int
{
	std::vector<int> indexes(3, -1);
	int size = (int)aVehicleTrackingState.size();
	for (int i = 0 ; i < size; i++)
	{
		if ((aVehicleTrackingState[i].intsectionTrackingState.vehicleIntersectionStatus == MsgEnum::mapLocType::onInbound)
				&& (indexes[0] == -1))
			indexes[0] = i;
		else if ((aVehicleTrackingState[i].intsectionTrackingState.vehicleIntersectionStatus == MsgEnum::mapLocType::onOutbound)
				&& (indexes[1] == -1))
			indexes[1] = i;
		else if ((aVehicleTrackingState[i].intsectionTrackingState.vehicleIntersectionStatus == MsgEnum::mapLocType::insideIntersectionBox)
				&& (indexes[2] == -1))
			indexes[2] = i;
	}
	return((indexes[0] != -1)? indexes[0] : ((indexes[1] != -1)? indexes[1] : ((indexes[2] != -1)? indexes[2] : -1)));
};

auto getIdx4minLatApproch = [](const std::vector<GeoUtils::vehicleTracking_t>& aApproachTrackingState)->int
{
	double dmax = 1000.0; // in centimetres
	if (aApproachTrackingState.empty())
		return(-1);
	auto it = std::min_element(aApproachTrackingState.begin(), aApproachTrackingState.end(),
		[](const GeoUtils::vehicleTracking_t& s1, const GeoUtils::vehicleTracking_t& s2)
		{return(std::abs(s1.laneProj.proj2segment.d) < std::abs(s2.laneProj.proj2segment.d));});
	if (it == aApproachTrackingState.end())
		return(-1);
	return((std::abs(it->laneProj.proj2segment.d) < dmax) ? (int)(it - aApproachTrackingState.begin()) : -1);
};

auto getIdx4minLatLane = [](const std::vector<GeoUtils::laneTracking_t>& aLaneTrackingState)->int
{
	double dmax = 1000.0; // in centimetres
	if (aLaneTrackingState.empty())
		return(-1);
	std::vector<double> vd;
	for (const auto& item : aLaneTrackingState)
	{
		double d = std::abs(item.laneProj.proj2segment.d);
		if ((item.vehicleLaneStatus == MsgEnum::laneLocType::inside) && (d < dmax))
			vd.push_back(d);
	}
	if (vd.empty())
		return(-1);
	auto it = std::min_element(vd.begin(), vd.end());
	return((it != vd.end()) ? (int)(it - vd.begin()) : -1);
};

auto getIdx4minLatNode = [](const std::vector<GeoUtils::laneProjection_t>& aProj2Lane, uint16_t laneWidth)->int
{
	double dwidth	= laneWidth * NmapData::laneWidthRatio;
	if (aProj2Lane.empty())
		return(-1);
	std::vector<double> vd;
	for (const auto& item : aProj2Lane)
	{
		double d = std::abs(item.proj2segment.d);
		if ((item.proj2segment.t >= 0.0) && (item.proj2segment.t <= 1.0) && (d < dwidth))
			vd.push_back(d);
	}
	if (vd.empty())
		return(-1);
	auto it = std::min_element(vd.begin(), vd.end());
	return((it != vd.end()) ? (int)(it - vd.begin()) : -1);
};

auto getIdx4specicalCase = [](const std::vector<GeoUtils::laneProjection_t>& aProj2Lane, uint16_t laneWidth)->int
{
	double dwidth = laneWidth * NmapData::laneWidthRatio;
	if (aProj2Lane.empty())
		return(-1);
	std::vector< std::pair<size_t, double> > candidates;
	for (size_t i = 0, j = aProj2Lane.size(); i + 1 < j; i++)
	{
		double d1 = std::abs(aProj2Lane[i].proj2segment.d);
		double d2 = std::abs(aProj2Lane[i+1].proj2segment.d);
		if ((aProj2Lane[i].proj2segment.t > 1.0) && (d1 < dwidth)
			&& (aProj2Lane[i+1].proj2segment.t < 0.0) && (d2 < dwidth))
		{
			if (d1 < d2)
				candidates.push_back(std::make_pair(i, d1));
			else
				candidates.push_back(std::make_pair(i+1, d2));
		}
	}
	if (candidates.empty())
		return(-1);
	auto it = std::min_element(candidates.begin(), candidates.end(),
		[](const std::pair<size_t, double>& s1, const std::pair<size_t, double>& s2){return(s1.second < s2.second);});
	return((it != candidates.end()) ? (int)(it->first) : -1);
};

GeoUtils::laneTracking_t LocAware::projectPt2Lane(uint8_t intersectionIndex, uint8_t approachIndex, uint8_t laneIndex,
	const GeoUtils::point2D_t& ptENU, const GeoUtils::motion_t& motionState) const
{
	const auto& appObj  = mpIntersection[intersectionIndex].mpApproaches[approachIndex];
	const auto& laneObj = appObj.mpLanes[laneIndex];
	double headingErrorBound = getHeadingErrorBound(motionState.speed);
	std::vector<GeoUtils::laneProjection_t> aProj2Lane;
	GeoUtils::laneProjection_t proj2lane;

	if (appObj.type == MsgEnum::approachType::inbound)
	{
		for (uint8_t i = (uint8_t)(laneObj.mpNodes.size() - 1); i > 0; i--)
		{
			if (std::abs(getHeadingDifference(laneObj.mpNodes[i].heading, motionState.heading)) > headingErrorBound)
				continue;
			proj2lane.nodeIndex = i;
			GeoUtils::projectPt2Line(laneObj.mpNodes[i].ptNode, laneObj.mpNodes[i-1].ptNode, ptENU, proj2lane.proj2segment);
			aProj2Lane.push_back(proj2lane);
		}
	}
	else
	{
		for (uint8_t i = 0, j = (uint8_t)(laneObj.mpNodes.size() - 1); i < j; i++)
		{
			if (std::abs(getHeadingDifference(laneObj.mpNodes[i].heading, motionState.heading)) > headingErrorBound)
				continue;
			proj2lane.nodeIndex = i;
			GeoUtils::projectPt2Line(laneObj.mpNodes[i].ptNode, laneObj.mpNodes[i+1].ptNode, ptENU, proj2lane.proj2segment);
			aProj2Lane.push_back(proj2lane);
		}
	}

	GeoUtils::laneTracking_t laneTrackingState{MsgEnum::laneLocType::outside,{0, {0.0, 0.0, 0.0}}};
	if (aProj2Lane.empty())
		return(laneTrackingState);
	if (aProj2Lane.front().proj2segment.t < 0)
	{
		laneTrackingState.vehicleLaneStatus = MsgEnum::laneLocType::approaching;
		laneTrackingState.laneProj = aProj2Lane.front();
		return(laneTrackingState);
	}
	if (aProj2Lane.back().proj2segment.t > 1)
	{
		laneTrackingState.vehicleLaneStatus = MsgEnum::laneLocType::leaving;
		laneTrackingState.laneProj = aProj2Lane.back();
		return(laneTrackingState);
	}
	// get index of the lane segment on which projection_t.t is between [0,1],
	// projection_t.d is within laneWidth * laneWidthRatio, and has the minimum
	// projection_t.d among all success projected segments
	int idx = getIdx4minLatNode(aProj2Lane, laneObj.width);
	if (idx >= 0)
	{
		laneTrackingState.vehicleLaneStatus = MsgEnum::laneLocType::inside;
		laneTrackingState.laneProj = aProj2Lane[idx];
		return(laneTrackingState);
	}
	// special case:
	idx = getIdx4specicalCase(aProj2Lane, laneObj.width);
	if (idx >= 0)
	{
		laneTrackingState.vehicleLaneStatus = MsgEnum::laneLocType::inside;
		laneTrackingState.laneProj = aProj2Lane[idx];
		return(laneTrackingState);
	}
	return(laneTrackingState);
}

bool LocAware::locateVehicleOnApproach(uint8_t intersectionIndex, uint8_t approachIndex, const GeoUtils::point2D_t& ptENU,
	const GeoUtils::motion_t& motionState, GeoUtils::vehicleTracking_t& vehicleTrackingState) const
{
	vehicleTrackingState.reset();
	const auto& appObj = mpIntersection[intersectionIndex].mpApproaches[approachIndex];
	std::vector<GeoUtils::laneTracking_t> aLaneTrackingState(appObj.mpLanes.size());
		// one record per lane regardless whether the vehicle is on lane or not

	for (uint8_t i = 0, j = (uint8_t)appObj.mpLanes.size(); i < j; i++)
		aLaneTrackingState[i] = LocAware::projectPt2Lane(intersectionIndex, approachIndex, i, ptENU, motionState);
	// when project to multiple lanes, find the lane with minimum distance away from it
	int idx = getIdx4minLatLane(aLaneTrackingState);
	if (idx >= 0)
	{
		if (appObj.type == MsgEnum::approachType::inbound)
			vehicleTrackingState.intsectionTrackingState.vehicleIntersectionStatus = MsgEnum::mapLocType::onInbound;
		else
			vehicleTrackingState.intsectionTrackingState.vehicleIntersectionStatus = MsgEnum::mapLocType::onOutbound;
		vehicleTrackingState.intsectionTrackingState.intersectionIndex = intersectionIndex;
		vehicleTrackingState.intsectionTrackingState.approachIndex = approachIndex;
		vehicleTrackingState.intsectionTrackingState.laneIndex = static_cast<uint8_t>(idx);
		vehicleTrackingState.laneProj = aLaneTrackingState[idx].laneProj;
		return(true);
	}
	return(false);
}

bool LocAware::isEgressConnect2Ingress(uint8_t intersectionIndex, uint8_t approachIndex, uint8_t laneIndex,
	const GeoUtils::geoPoint_t& geoPoint, const GeoUtils::motion_t& motionState, GeoUtils::vehicleTracking_t& vehicleTrackingState) const
{
	const auto& connectTo = mpIntersection[intersectionIndex].mpApproaches[approachIndex].mpLanes[laneIndex].mpConnectTo;

	if (!connectTo.empty())
	{ // egress lane has at most one connectTo, get indexes (intersection, approach & lane) of connectTo ingress lane
		std::vector<uint8_t> connectToindex = LocAware::getIndexesByIds(connectTo[0].intersectionId, connectTo[0].laneId);
		// convert geoPoint to ptENU at connectTo intersection
		GeoUtils::point2D_t ptENU;
		GeoUtils::lla2enu(mpIntersection[connectToindex[0]].enuCoord, geoPoint, ptENU);
		// check whether ptENU is onInbound
		if (LocAware::isPointOnApproach(connectToindex[0], connectToindex[1], ptENU)
				&& LocAware::locateVehicleOnApproach(connectToindex[0], connectToindex[1], ptENU, motionState, vehicleTrackingState))
			return(true);
	}
	return(false);
}

double LocAware::getPtDist2egress(uint8_t intersectionIndex, uint8_t approachIndex, const GeoUtils::point2D_t& ptENU) const
{ // project ptENU onto the closet lane segment on egress approach
	const auto& appObj = mpIntersection[intersectionIndex].mpApproaches[approachIndex];
	double dminimum = 2000.0; // in centimetres
	GeoUtils::projection_t proj2segment;

	for (const auto& laneObj : appObj.mpLanes)
	{
		GeoUtils::projectPt2Line(laneObj.mpNodes[0].ptNode, laneObj.mpNodes[1].ptNode, ptENU, proj2segment);
		double d = proj2segment.t * proj2segment.length;
		if ((d <= 0) && (std::abs(d) < dminimum))
			dminimum = std::abs(d);
	}
	return(dminimum);
}

bool LocAware::locateVehicleInMap(const GeoUtils::connectedVehicle_t& cv, GeoUtils::vehicleTracking_t& cvTrackingState) const
{
	cvTrackingState.reset();
	GeoUtils::point2D_t ptENU;
	GeoUtils::vehicleTracking_t vehicleTrackingState;

	if (!cv.isVehicleInMap)
	{ // find target intersections that geoPoint is on
		auto intersectionList = LocAware::nearedIntersections(cv.geoPoint);
		if (intersectionList.empty())
			return(false);
		std::vector<GeoUtils::vehicleTracking_t> aVehicleTrackingState; // at most one record per intersection
		for (const auto& intIndx : intersectionList)
		{ // convert cv.geoPoint to ptENU
			GeoUtils::lla2enu(mpIntersection[intIndx].enuCoord, cv.geoPoint, ptENU);
			// check whether ptENU is inside intersection box first
			if (LocAware::isPointInsideIntersectionBox(intIndx,ptENU))
			{
				vehicleTrackingState.reset();
				vehicleTrackingState.intsectionTrackingState.vehicleIntersectionStatus
					= MsgEnum::mapLocType::insideIntersectionBox;
				vehicleTrackingState.intsectionTrackingState.intersectionIndex = intIndx;
				aVehicleTrackingState.push_back(vehicleTrackingState);
				continue;
			}
			// find target approaches that ptENU is on
			auto approachList = LocAware::onApproaches(intIndx, ptENU);
			if (approachList.empty())
				continue;
			std::vector<GeoUtils::vehicleTracking_t> aApproachTrackingState; // at most one record per approach
			// find target lanes that ptENU is on
			for (const auto& appIndx : approachList)
			{
				if (LocAware::locateVehicleOnApproach(intIndx, appIndx, ptENU, cv.motionState, vehicleTrackingState))
					aApproachTrackingState.push_back(vehicleTrackingState);
			}
			if (!aApproachTrackingState.empty())
			{ // when project to multiple approaches, find the approach/lane with minimum distance away from it
				int idx = getIdx4minLatApproch(aApproachTrackingState);
				if (idx >= 0)
					aVehicleTrackingState.push_back(aApproachTrackingState[idx]);
			}
		}
		if (aVehicleTrackingState.empty())
			return(false);
		// when ptENU in multiple intersections, find the intersection in the order of
		// onInbound, onOutbound, insideIntersectionBox
		int idx = getIdxByLocationType(aVehicleTrackingState);
		if (idx < 0)
			return(false);
		cvTrackingState = aVehicleTrackingState[idx];
		return(true);
	}

	// vehicle was already in map, so intersectionIndex is known
	const auto& intersectionIndex = cv.vehicleTrackingState.intsectionTrackingState.intersectionIndex;
	const auto& intObj = mpIntersection[intersectionIndex];
	// convert cv.geoPoint to ptENU
	GeoUtils::lla2enu(intObj.enuCoord, cv.geoPoint,ptENU);

	if (cv.vehicleTrackingState.intsectionTrackingState.vehicleIntersectionStatus == MsgEnum::mapLocType::insideIntersectionBox)
	{ // vehicle was initiated inside the intersection box, so approachIndex & laneIndex are unknown.
		// next vehicleIntersectionStatus:
		//  1. remain insideIntersectionBox
		//  2. on one of the outbound lanes (onOutbound)
		//  3. outside the Map - GPS error could cause distance away from the center of outbound lane greater than half lane width
		//    (may need to loss the constraint by 3/4 of lane width) or due to the gap between intersection and approach mpPolygon
		//    This is not a problem for vehicle initiated insideIntersectionBox as it needs to be reset to start tracking anyway.

		// check whether vehicle remains insideIntersectionBox
		if (LocAware::isPointInsideIntersectionBox(intersectionIndex, ptENU))
		{ // no change on vehicleTracking_t, return
			cvTrackingState = cv.vehicleTrackingState;
			return(true);
		}

		// vehicle not insideIntersectionBox, check whether it is on outbound lane (onOutbound)
		std::vector<GeoUtils::vehicleTracking_t> aApproachTrackingState;
		auto approachList = LocAware::onApproaches(intersectionIndex, ptENU);
		if (!approachList.empty())
		{
			for (const auto& appIndx: approachList)
			{
				if ((intObj.mpApproaches[appIndx].type == MsgEnum::approachType::outbound)
						&& LocAware::locateVehicleOnApproach(intersectionIndex, appIndx, ptENU, cv.motionState, vehicleTrackingState))
					aApproachTrackingState.push_back(vehicleTrackingState);
			}
		}
		// find the approach with minimum distance away from the center of the lane
		int idx = aApproachTrackingState.empty() ? -1 : getIdx4minLatApproch(aApproachTrackingState);
		if (idx < 0) // vehicle is not insideIntersectionBox nor onOutbound, reset tracking to outside
			return(false);

		// vehicle onOutbound, check whether it is on connecting inbound lane (onInbound)
		if (LocAware::isEgressConnect2Ingress(aApproachTrackingState[idx].intsectionTrackingState.intersectionIndex,
			aApproachTrackingState[idx].intsectionTrackingState.approachIndex,
			aApproachTrackingState[idx].intsectionTrackingState.laneIndex,
			cv.geoPoint, cv.motionState, vehicleTrackingState))
		{
			cvTrackingState = vehicleTrackingState;
			return(true);
		}

		// vehicle remains onOutbound
		cvTrackingState = aApproachTrackingState[idx];
		return(true);
	}
	else if (cv.vehicleTrackingState.intsectionTrackingState.vehicleIntersectionStatus == MsgEnum::mapLocType::onInbound)
	{ // vehicle was onInbound, so approachIndex & laneIndex are known.
		// next vehicleIntersectionStatus:
		//  1. remain onInbound
		//  2. enters intersection box (atIntersectionBox)
		//  3. enters connecting outbound lane (onOutbound) - for bypass lanes. not likely to occur for lanes
		//     going inside the intersection box as it would take at least 1 sec to cross the intersection box,
		//     while GPS updating rate is 10 Hz.
		//  4. outside the Map - due to GPS error (distance away from the lane center is greater than half lane width),
		//    or due to the gap between intersection mpPolygon and approach mpPolygon. For the second cause,
		//    the projected distance onto the closed lane segment is compare against gapDist. If it's less than gapDist,
		//    vehicle is set as atIntersectionBox, otherwise set as outside
		const auto& approachIndex = cv.vehicleTrackingState.intsectionTrackingState.approachIndex;
		const auto& laneIndex = cv.vehicleTrackingState.intsectionTrackingState.laneIndex;
		const auto& laneObj = intObj.mpApproaches[approachIndex].mpLanes[laneIndex];
		// check whether vehicle remains onInbound
		if (LocAware::isPointOnApproach(intersectionIndex, approachIndex, ptENU)
			&& LocAware::locateVehicleOnApproach(intersectionIndex, approachIndex, ptENU, cv.motionState, vehicleTrackingState))
		{
			cvTrackingState = vehicleTrackingState;
			return(true);
		}

		// vehicle is not onInbound, check whether it is on connecting outbound lane (onOutbound)
		std::vector<uint8_t> connectToApproachIndex;
		for (const auto& connObj : laneObj.mpConnectTo)
		{
			auto connectToindex = LocAware::getIndexesByIds(connObj.intersectionId, connObj.laneId);
			connectToApproachIndex.push_back(connectToindex[1]);
		}
		std::vector<GeoUtils::vehicleTracking_t> aApproachTrackingState;
		auto approachList = LocAware::onApproaches(intersectionIndex, ptENU);
		if (!approachList.empty())
		{
			for (const auto& appIndx : approachList)
			{
				if ((intObj.mpApproaches[appIndx].type == MsgEnum::approachType::outbound)
						&& (std::find(connectToApproachIndex.begin(), connectToApproachIndex.end(), appIndx) != connectToApproachIndex.end())
						&& LocAware::locateVehicleOnApproach(intersectionIndex, appIndx, ptENU, cv.motionState, vehicleTrackingState))
					aApproachTrackingState.push_back(vehicleTrackingState);
			}
		}
		// find the approach with minimum distance away from the center of the lane
		int idx = aApproachTrackingState.empty() ? -1 : getIdx4minLatApproch(aApproachTrackingState);
		if (idx >= 0)
		{ // vehicle onOutbound, check whether is is on connecting inbound lane of the next intersection (onInbound)
			if (LocAware::isEgressConnect2Ingress(aApproachTrackingState[idx].intsectionTrackingState.intersectionIndex,
				aApproachTrackingState[idx].intsectionTrackingState.approachIndex,
				aApproachTrackingState[idx].intsectionTrackingState.laneIndex,
				cv.geoPoint, cv.motionState, vehicleTrackingState))
			{
				cvTrackingState = vehicleTrackingState;
				return(true);
			}
			else
			{ // vehicle remains onOutbound (can't find connecting inbound lane)
				cvTrackingState = aApproachTrackingState[idx];
				return(true);
			}
		}

		// vehicle is not onInbound nor onOutbound; project ptENU onto the closet lane segment on which
		// the vehicle entered the intersection box
		GeoUtils::projection_t proj2segment;
		GeoUtils::projectPt2Line(laneObj.mpNodes[1].ptNode, laneObj.mpNodes[0].ptNode, ptENU, proj2segment);
		double distInto = proj2segment.t * proj2segment.length - laneObj.mpNodes[1].dTo1stNode;
		if (LocAware::isPointInsideIntersectionBox(intersectionIndex, ptENU) || (std::abs(distInto) < laneObj.mpNodes[0].dTo1stNode / 2.0))
		{ // if inside intersection polygon or distance into is less than gapDist,
			// set vehicleIntersectionStatus to atIntersectionBox
			cvTrackingState = cv.vehicleTrackingState;
			cvTrackingState.intsectionTrackingState.vehicleIntersectionStatus = MsgEnum::mapLocType::atIntersectionBox;
			//  update cvTrackingState.laneProj
			cvTrackingState.laneProj.nodeIndex = 1;
			cvTrackingState.laneProj.proj2segment = proj2segment;
			return(true);
		}
		return(false);
	}
	else if (cv.vehicleTrackingState.intsectionTrackingState.vehicleIntersectionStatus == MsgEnum::mapLocType::onOutbound)
	{ // vehicle was onOutbound, so approachIndex & laneIndex are known.
		// next vehicleIntersectionStatus:
		//  1. switch to inbound lane to the downstream intersection (onInbound)
		//  2. remain onOutbound
		//  3. outside (finished onOutbound and there is no downstream intersections)
		const auto& approachIndex = cv.vehicleTrackingState.intsectionTrackingState.approachIndex;
		const auto& laneIndex = cv.vehicleTrackingState.intsectionTrackingState.laneIndex;

		// vehicle onOutbound, check whether the vehicle is on connecting inbound lane (onInbound)
		if (LocAware::isEgressConnect2Ingress(intersectionIndex, approachIndex, laneIndex, cv.geoPoint, cv.motionState, vehicleTrackingState))
		{
			cvTrackingState = vehicleTrackingState;
			return(true);
		}
		// vehicle not switching to onInbound, check whether it remains onOutbound
		if (LocAware::isPointOnApproach(intersectionIndex, approachIndex, ptENU)
			&& LocAware::locateVehicleOnApproach(intersectionIndex, approachIndex, ptENU, cv.motionState, vehicleTrackingState))
		{ // remains onOutbound
			cvTrackingState = vehicleTrackingState;
			return(true);
		}
		return(false); // outside
	}
	else
	{ // vehicle was atIntersectionBox, so approachIndex & laneIndex are known.
		// next vehicleIntersectionStatus:
		//  1. remain atIntersectionBox
		//  2. on one of the outbound lanes (onOutbound)
		//  3. onInbound - due to GPS overshooting when stopped near the stop-bar, need to give the chance for correction
		//  4. outside a Map - due to GPS error (distance away from the lane center is greater than half lane width),
		//    or due to the gap between intersection mpPolygon and approach mpPolygon. Same method for case onInbound
		//    is used to check whether need to keep vehicle atIntersectionBox or not
		const auto& approachIndex = cv.vehicleTrackingState.intsectionTrackingState.approachIndex;
		const auto& laneIndex = cv.vehicleTrackingState.intsectionTrackingState.laneIndex;
		const auto& laneObj = intObj.mpApproaches[approachIndex].mpLanes[laneIndex];

		// check whether vehicle is on connecting outbound lane (onOutbound)
		std::vector<uint8_t> connectToApproachIndex;
		for (const auto& connObj : laneObj.mpConnectTo)
		{
			auto connectToindex = LocAware::getIndexesByIds(connObj.intersectionId, connObj.laneId);
			connectToApproachIndex.push_back(connectToindex[1]);
		}
		std::vector<GeoUtils::vehicleTracking_t> aApproachTrackingState;
		std::vector<double> dist2egress;
		auto approachList = LocAware::onApproaches(intersectionIndex, ptENU);
		if (!approachList.empty())
		{
			for (const auto& appIndx : approachList)
			{
				if ((intObj.mpApproaches[appIndx].type == MsgEnum::approachType::outbound)
					&& (std::find(connectToApproachIndex.begin(), connectToApproachIndex.end(), appIndx)
						!= connectToApproachIndex.end()))
				{
					if (LocAware::locateVehicleOnApproach(intersectionIndex, appIndx, ptENU, cv.motionState, vehicleTrackingState))
						aApproachTrackingState.push_back(vehicleTrackingState);
					double d = LocAware::getPtDist2egress(intersectionIndex, appIndx, ptENU);
					if (d < intObj.mpApproaches[appIndx].mindist2intsectionCentralLine / 2)
					 dist2egress.push_back(d);
				}
			}
		}
		// find the approach with minimum distance away from the center of the lane
		int idx = aApproachTrackingState.empty() ? -1 : getIdx4minLatApproch(aApproachTrackingState);
		if (idx >= 0)
		{ // vehicle onOutbound, check whether the vehicle is on connecting inbound lane (onInbound)
			if (LocAware::isEgressConnect2Ingress(aApproachTrackingState[idx].intsectionTrackingState.intersectionIndex,
				aApproachTrackingState[idx].intsectionTrackingState.approachIndex,
				aApproachTrackingState[idx].intsectionTrackingState.laneIndex,
				cv.geoPoint,cv.motionState,vehicleTrackingState))
			{
				cvTrackingState = vehicleTrackingState;
				return(true);
			}
			else
			{ // vehicle onOutbound
				cvTrackingState = aApproachTrackingState[idx];
				return(true);
			}
		}

		// vehicle not onOutbound, check whether ptENU is on the approach it enters the intersection box (onInbound)
		if (LocAware::isPointOnApproach(intersectionIndex, approachIndex, ptENU)
			&& LocAware::locateVehicleOnApproach(intersectionIndex, approachIndex, ptENU, cv.motionState, vehicleTrackingState))
		{
			cvTrackingState = vehicleTrackingState;
			return(true);
		}

		//  vehicle not onOutbound nor onInbound, go to be atIntersectionBox
		//  project ptENU onto the inbound lane that the vehicle entered the intersection box
		GeoUtils::projection_t proj2segment;
		GeoUtils::projectPt2Line(laneObj.mpNodes[1].ptNode, laneObj.mpNodes[0].ptNode, ptENU, proj2segment);
		double distInto = proj2segment.t * proj2segment.length - laneObj.mpNodes[1].dTo1stNode;
		if (LocAware::isPointInsideIntersectionBox(intersectionIndex, ptENU) || (std::abs(distInto) < laneObj.mpNodes[0].dTo1stNode))
		{ // if inside intersection polygon or distance into is less than gapDist,
			// set vehicleIntersectionStatus to atIntersectionBox
			cvTrackingState = cv.vehicleTrackingState;
			cvTrackingState.intsectionTrackingState.vehicleIntersectionStatus = MsgEnum::mapLocType::atIntersectionBox;
			//  update cvTrackingState.laneProj
			cvTrackingState.laneProj.nodeIndex = 1;
			cvTrackingState.laneProj.proj2segment = proj2segment;
			return(true);
		}

		// vehicle not near onInbound, check whether it's near onOutbound
		if (!dist2egress.empty())
		{ // set vehicleIntersectionStatus to atIntersectionBox
			cvTrackingState = cv.vehicleTrackingState;
			cvTrackingState.intsectionTrackingState.vehicleIntersectionStatus = MsgEnum::mapLocType::atIntersectionBox;
			//  update cvTrackingState.laneProj
			cvTrackingState.laneProj.nodeIndex = 1;
			cvTrackingState.laneProj.proj2segment = proj2segment;
			return(true);
		}
		return(false);
	}
}

void LocAware::updateLocationAware(const GeoUtils::vehicleTracking_t& vehicleTrackingState, GeoUtils::locationAware_t& vehicleLocationAware) const
{
	vehicleLocationAware.reset();
	switch(vehicleTrackingState.intsectionTrackingState.vehicleIntersectionStatus)
	{
	case MsgEnum::mapLocType::outside:
		break;
	case MsgEnum::mapLocType::insideIntersectionBox:
		vehicleLocationAware.intersectionId = mpIntersection[vehicleTrackingState.intsectionTrackingState.intersectionIndex].id;
		break;
	default: // onInbound, atIntersectionBox, onOutbound
		const auto& intObj  = mpIntersection[vehicleTrackingState.intsectionTrackingState.intersectionIndex];
		const auto& appObj  = intObj.mpApproaches[vehicleTrackingState.intsectionTrackingState.approachIndex];
		const auto& laneObj = appObj.mpLanes[vehicleTrackingState.intsectionTrackingState.laneIndex];
		vehicleLocationAware.intersectionId = intObj.id;
		vehicleLocationAware.laneId = laneObj.id;
		vehicleLocationAware.controlPhase = laneObj.controlPhase;
		for (const auto& connObj : laneObj.mpConnectTo)
		{
			GeoUtils::connectTo_t connectTo{connObj.intersectionId, connObj.laneId, connObj.laneManeuver};
			vehicleLocationAware.connect2go.push_back(connectTo);
		}
	}
	GeoUtils::point2D_t pt;
	LocAware::getPtDist2D(vehicleTrackingState, pt);
	vehicleLocationAware.dist2go.distLong = DsrcConstants::hecto2unit<int32_t>(pt.x);
	vehicleLocationAware.dist2go.distLat = DsrcConstants::hecto2unit<int32_t>(pt.y);
}

void LocAware::getPtDist2D(const GeoUtils::vehicleTracking_t& vehicleTrackingState, GeoUtils::point2D_t& pt) const
{ // return distance to stop-bar
	if ((vehicleTrackingState.intsectionTrackingState.vehicleIntersectionStatus == MsgEnum::mapLocType::outside)
		|| (vehicleTrackingState.intsectionTrackingState.vehicleIntersectionStatus == MsgEnum::mapLocType::insideIntersectionBox))
	{
		pt.x = 0;
		pt.y = 0;
		return;
	}

	const auto& intIndx = vehicleTrackingState.intsectionTrackingState.intersectionIndex;
	const auto& appIndx = vehicleTrackingState.intsectionTrackingState.approachIndex;
	const auto& laneIndx = vehicleTrackingState.intsectionTrackingState.laneIndex;
	const auto& nodeIndx = vehicleTrackingState.laneProj.nodeIndex;
	uint32_t nodeDistTo1stNode = (nodeIndx == 0) ? 0
		: mpIntersection[intIndx].mpApproaches[appIndx].mpLanes[laneIndx].mpNodes[nodeIndx].dTo1stNode;
	double ptIntoLine = vehicleTrackingState.laneProj.proj2segment.t * vehicleTrackingState.laneProj.proj2segment.length;
	pt.y = static_cast<int32_t>(vehicleTrackingState.laneProj.proj2segment.d);
	if (vehicleTrackingState.intsectionTrackingState.vehicleIntersectionStatus == MsgEnum::mapLocType::onOutbound)
	{ // pt.x is distance downstream from the intersection box boundary,
		// node index has the same direction of lane projection
		pt.x = static_cast<int32_t>(nodeDistTo1stNode + ptIntoLine);
	}
	else
	{ // pt.x is distance to stop-bar, node index has opposite direction from lane projection
		// positive pt.x onOutbound, negative atIntersectionBox
		pt.x = static_cast<int32_t>(nodeDistTo1stNode - ptIntoLine);
	}
}
/// --- end of functions to locate BSM on MAP --- ///
