//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************
#include <algorithm>
#include <bitset>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

// asn1
#include <asn_application.h>
#include "BasicSafetyMessage.h"
#include "MapData.h"
#include "SPAT.h"
#include "SignalRequestMessage.h"
#include "SignalStatusMessage.h"

// asn1j2735
#include "AsnJ2735Lib.h"

/// convert number of bits to number of bytes
auto numbits2numbytes = [](ssize_t bit_nums)->size_t
	{return((bit_nums <= 0) ? (0) : ((bit_nums + 7) >> 3));};

/// convert unsigned long to OCTET STRING with preallocated buffer
auto ul2octString = [](uint8_t* pbuf, int size, unsigned long value)->void
{ // OCTET STRING is in network byte order
	for (int i = 0; i < size; i++)
	{
		unsigned int shift_bits = (size - 1 - i) * 8;
		pbuf[i] = (uint8_t)((value >> shift_bits) & 0xFF);
	}
};

/// convert OCTET STRING to unsigned long
auto octString2ul = [](const uint8_t* pbuf, int size)->unsigned long
{ // OCTET STRING is in network byte order
	unsigned long ret = 0;
	for (int i = 0; i < size; i++)
		ret = (ret << 8) | pbuf[i];
	return(ret);
};

/// convert BIT STRING to unsigned long
auto bitString2ul = [](const uint8_t* pbuf, int size, int bits_unused)->unsigned long
{
	unsigned long ret = octString2ul(pbuf, size);
	return(ret >> bits_unused);
};

/// convert unsigned long to BIT STRING
auto ul2bitString = [](uint8_t** pbuf, int& num_bytes, int& bits_unused, int num_bits, unsigned long value)->bool
{
	int bytes = (num_bits / 8) + (((num_bits % 8) > 0) ? 1 : 0);
	if ((*pbuf = (uint8_t *)calloc(bytes, sizeof(uint8_t))) == NULL)
		return(false);
	num_bytes = bytes;
	bits_unused = bytes * 8 - num_bits;
	ul2octString(*pbuf, bytes, value << bits_unused);
	return(true);
};

/// convert uint32_t vehicle ID to Temporary ID
auto vehId2temporaryId = [](uint8_t** pbuf, int& size, unsigned long value)->bool
{ // Temporary ID has 4 bytes
	if ((*pbuf = (uint8_t *)calloc(4, sizeof(uint8_t))) == NULL)
		return(false);
	size = 4;
	ul2octString(*pbuf, 4, value);
	return(true);
};

size_t AsnJ2735Lib::encode_mapdata_payload(const MapData_element_t& mapDataIn, uint8_t* buf, size_t size)
{
	std::string allocate_level{"MapData"};
	MapData_t* pMapData = (MapData_t *)calloc(1, sizeof(MapData_t));
	if (pMapData == NULL)
	{
		std::cerr << "encode_mapdata_payload: failed allocate " << allocate_level << std::endl;
		return(0);
	}
	// MapData:
	// -- Required objects ------------------------------------ //
	//	msgIssueRevision
	// -- OPTIONAL objects ------------ including/excluding  -- //
	//	MinuteOfTheYear                     EXCL
	//	LayerType                           INCL
	//	LayerID                             EXCL
	//	IntersectionGeometryList            INCL
	//	RoadSegmentList                     EXCL
	//	DataParameters                      EXCL
	//	RestrictionClassList                EXCL
	//	RegionalExtension                   EXCL
	// -------------------------------------------------------- //

	// msgIssueRevision
	pMapData->msgIssueRevision	= mapDataIn.mapVersion;
	// LayerType
	if ((pMapData->layerType = (LayerType_t *)calloc(1, sizeof(LayerType_t))) == NULL)
	{
		std::cerr << "encode_mapdata_payload: intersectionId=" << mapDataIn.id;
		std::cerr << ", failed allocate " << allocate_level << ".LayerType" << std::endl;
		free(pMapData);
		return(0);
	}
	*(pMapData->layerType) = LayerType_intersectionData;

	// IntersectionGeometryList - one intersection per MapData
	allocate_level += ".IntersectionGeometryList";
	pMapData->intersections = (IntersectionGeometryList_t *)calloc(1, sizeof(IntersectionGeometryList_t));
	if (pMapData->intersections != NULL) // one IntersectionGeometry per distinct speed limit
		pMapData->intersections->list.array = (IntersectionGeometry_t **)calloc(mapDataIn.speeds.size(), sizeof(IntersectionGeometry_t *));
	if ((pMapData->intersections == NULL) || (pMapData->intersections->list.array == NULL))
	{
		std::cerr << "encode_mapdata_payload: intersectionId=" << mapDataIn.id;
		std::cerr << ", failed allocate " << allocate_level << std::endl;
		if (pMapData->intersections != NULL)
			free(pMapData->intersections);
		free(pMapData->layerType);
		free(pMapData);
		return(0);
	}
	pMapData->intersections->list.size = static_cast<int>(mapDataIn.speeds.size());

	// loop through distinct speed limits
	allocate_level += ".IntersectionGeometry";
	bool has_error = false;  // for earlier return
	int& geoListCnt = pMapData->intersections->list.count;
	for (const auto& speed_limit : mapDataIn.speeds)
	{	// get indexes of approaches and number of lanes having the targeted speed limit
		std::vector<uint8_t> approachIndex;
		size_t num_lanes = 0;
		for (auto it = mapDataIn.mpApproaches.begin(); it != mapDataIn.mpApproaches.end(); ++it)
		{
			if ((it->speed_limit == speed_limit) && (!it->mpLanes.empty()))
			{
				approachIndex.push_back((uint8_t)(it - mapDataIn.mpApproaches.begin()));
				num_lanes += it->mpLanes.size();
			}
		}
		// get the reference lane width for this speed group
		uint16_t refLaneWidth = mapDataIn.mpApproaches[approachIndex[0]].mpLanes[0].width;
		// allocate IntersectionGeometry - one per speed group
		if ((pMapData->intersections->list.array[geoListCnt] = (IntersectionGeometry_t *)calloc(1, sizeof(IntersectionGeometry_t))) == NULL)
		{
			std::cerr << "encode_mapdata_payload: intersectionId=" << mapDataIn.id;
			std::cerr << ", failed allocate " << allocate_level << std::endl;
			has_error = true;
			break;
		}
		IntersectionGeometry_t* pIntersectionGeometry = pMapData->intersections->list.array[geoListCnt];
		// IntersectionGeometry:
		// -- Required objects ------------------------------------ //
		//	IntersectionReferenceID
		//	MsgCount
		//	Position3D
		//	LaneList
		// -- OPTIONAL objects ------------ including/excluding  -- //
		//	DescriptiveName                     EXCL
		//	LaneWidth                           INCL
		//	SpeedLimitList                      INCL
		//	PreemptPriorityList                 EXCL
		//	RegionalExtension                   EXCL
		// -------------------------------------------------------- //

		// IntersectionReferenceID
		pIntersectionGeometry->id.id = mapDataIn.id;
		// MsgCount
		pIntersectionGeometry->revision = geoListCnt++;
		// Position3D
		pIntersectionGeometry->refPoint.lat = mapDataIn.geoRef.latitude;
		pIntersectionGeometry->refPoint.Long = mapDataIn.geoRef.longitude;
		if (mapDataIn.attributes.test(0))
		{ // include elevation data
			if ((pIntersectionGeometry->refPoint.elevation = (Elevation_t *)calloc(1, sizeof(Elevation_t))) == NULL)
			{
				std::cerr << "encode_mapdata_payload: intersectionId=" << mapDataIn.id;
				std::cerr << ", failed allocate " << allocate_level << ".Position3D.Elevation" << std::endl;
				has_error = true;
				break;
			}
			*(pIntersectionGeometry->refPoint.elevation) = mapDataIn.geoRef.elevation;
		}
		// LaneWidth
		if ((pIntersectionGeometry->laneWidth = (LaneWidth_t *)calloc(1, sizeof(LaneWidth_t))) == NULL)
		{
			std::cerr << "encode_mapdata_payload: intersectionId=" << mapDataIn.id;
			std::cerr << ", failed allocate " << allocate_level << ".LaneWidth" << std::endl;
			has_error = true;
			break;
		}
		*(pIntersectionGeometry->laneWidth) = refLaneWidth;
		// SpeedLimitList
		if ((speed_limit > 0) && (speed_limit < MsgEnum::unknown_speed))
		{ // 0 = crosswalk, MsgEnum::unknown_speed = speed limit not available on vehicular lanes
			pIntersectionGeometry->speedLimits = (SpeedLimitList_t *)calloc(1, sizeof(SpeedLimitList_t));
			if (pIntersectionGeometry->speedLimits != NULL)
				pIntersectionGeometry->speedLimits->list.array = (RegulatorySpeedLimit_t **)calloc(1, sizeof(RegulatorySpeedLimit_t *));
			if ((pIntersectionGeometry->speedLimits == NULL) || (pIntersectionGeometry->speedLimits->list.array == NULL))
			{
				std::cerr << "encode_mapdata_payload: intersectionId=" << mapDataIn.id;
				std::cerr << ", failed allocate " << allocate_level << ".SpeedLimitList" << std::endl;
				has_error = true;
				break;
			}
			pIntersectionGeometry->speedLimits->list.size = 1;
			if ((pIntersectionGeometry->speedLimits->list.array[0] = (RegulatorySpeedLimit_t *)calloc(1, sizeof(RegulatorySpeedLimit_t))) == NULL)
			{
				std::cerr << "encode_mapdata_payload: intersectionId=" << mapDataIn.id;
				std::cerr << ", failed allocate " << allocate_level << ".SpeedLimitList.RegulatorySpeedLimit" << std::endl;
				has_error = true;
				break;
			}
			pIntersectionGeometry->speedLimits->list.count = 1;
			pIntersectionGeometry->speedLimits->list.array[0]->type  = SpeedLimitType_vehicleMaxSpeed;
			pIntersectionGeometry->speedLimits->list.array[0]->speed = speed_limit;
		}

		// LaneList
		std::string branch_level{".LaneList"};
		if ((pIntersectionGeometry->laneSet.list.array = (GenericLane_t **)calloc(num_lanes, sizeof(GenericLane_t *))) == NULL)
		{
			std::cerr << "encode_mapdata_payload: intersectionId=" << mapDataIn.id;
			std::cerr << ", failed allocate " << allocate_level << branch_level << std::endl;
			has_error = true;
			break;
		}
		pIntersectionGeometry->laneSet.list.size = static_cast<int>(num_lanes);
		branch_level += ".GenericLane";
		int& laneListCnt = pIntersectionGeometry->laneSet.list.count;
		// loop through approaches / lanes
		for (const auto& i_approach :approachIndex)
		{
			const auto& approachStruct = mapDataIn.mpApproaches[i_approach];
			for (const auto& laneStruct : approachStruct.mpLanes)
			{
				if ((pIntersectionGeometry->laneSet.list.array[laneListCnt] = (GenericLane_t *)calloc(1, sizeof(GenericLane_t))) == NULL)
				{
					std::cerr << "encode_mapdata_payload: intersectionId=" << mapDataIn.id;
					std::cerr << ", failed allocate " << allocate_level;
					std::cerr	<< branch_level << std::endl;
					has_error = true;
					break;
				}
				GenericLane_t* pGenericLane = pIntersectionGeometry->laneSet.list.array[laneListCnt++];
				// GenericLane:
				// -- Required objects ------------------------------------ //
				//	LaneID
				//	LaneAttributes
				//	NodeListXY
				// -- OPTIONAL objects ------------ including/excluding  -- //
				//	DescriptiveName                     EXCL
				//	ApproachID (inbound/outbound)       INCL
				//	AllowedManeuvers                    INCL
				//	ConnectsToList                      INCL
				//	OverlayLaneList                     EXCL
				//	RegionalExtension                   EXCL
				// -------------------------------------------------------- //

				// LaneID
				pGenericLane->laneID = laneStruct.id;
				// LaneAttributes::LaneDirection - 2 bits BIT STRING
				std::bitset<2> direction_attributes;
				switch(approachStruct.type)
				{
				case MsgEnum::approachType::inbound:
					direction_attributes.set(0);
					break;
				case MsgEnum::approachType::outbound:
					direction_attributes.set(1);
					break;
				case MsgEnum::approachType::crosswalk:
					direction_attributes.set();
					break;
				}
				auto& laneDirection = pGenericLane->laneAttributes.directionalUse;
				if (!ul2bitString(&laneDirection.buf, laneDirection.size, laneDirection.bits_unused, 2, direction_attributes.to_ulong()))
				{
					std::cerr << "encode_mapdata_payload: intersectionId=" << mapDataIn.id;
					std::cerr << ", failed allocate " << allocate_level << branch_level << ".LaneAttributes.LaneDirection" << std::endl;
					has_error = true;
					break;
				}
				// LaneAttributes::LaneSharing - 10 bits BIT STRING - 'not shared' and 'not overlapping'
				auto& laneSharing = pGenericLane->laneAttributes.sharedWith;
				if (!ul2bitString(&laneSharing.buf, laneSharing.size, laneSharing.bits_unused, 10, 0))
				{
					std::cerr << "encode_mapdata_payload: intersectionId=" << mapDataIn.id;
					std::cerr << ", failed allocate " << allocate_level << branch_level << ".LaneAttributes.LaneSharing" << std::endl;
					has_error = true;
					break;
				}
				// LaneAttributes::LaneTypeAttributes
				auto& laneTypeAttributes = pGenericLane->laneAttributes.laneType;
				if (approachStruct.type == MsgEnum::approachType::crosswalk)
				{ // Crosswalk - 16 bits BIT STRING
					laneTypeAttributes.present = LaneTypeAttributes_PR_crosswalk;
					auto& crosswalk = laneTypeAttributes.choice.crosswalk;
					if (!ul2bitString(&crosswalk.buf, crosswalk.size, crosswalk.bits_unused, 16, laneStruct.attributes.to_ulong()))
					{
						std::cerr << "encode_mapdata_payload: intersectionId=" << mapDataIn.id;
						std::cerr << ", failed allocate " << allocate_level << branch_level << ".LaneAttributes.LaneTypeAttributes.Crosswalk" << std::endl;
						has_error = true;
						break;
					}
				}
				else
				{ // Vehicle - 8 bits BIT STRING
					laneTypeAttributes.present = LaneTypeAttributes_PR_vehicle;
					auto& vehicle = laneTypeAttributes.choice.vehicle;
					if (!ul2bitString(&vehicle.buf, vehicle.size, vehicle.bits_unused, 8, (laneStruct.attributes.to_ulong() & 0xFF)))
					{
						std::cerr << "encode_mapdata_payload: intersectionId=" << mapDataIn.id;
						std::cerr << ", failed allocate " << allocate_level << branch_level << ".LaneAttributes.LaneTypeAttributes.Vehicle" << std::endl;
						has_error = true;
						break;
					}
				}
				// ApproachID
				switch(approachStruct.type)
				{
				case MsgEnum::approachType::outbound:
					if ((pGenericLane->egressApproach = (ApproachID_t *)calloc(1, sizeof(ApproachID_t))) == NULL)
						has_error = true;
					else
						*(pGenericLane->egressApproach) = approachStruct.id;
					break;
				case MsgEnum::approachType::inbound:
				case MsgEnum::approachType::crosswalk:
					if ((pGenericLane->ingressApproach = (ApproachID_t *)calloc(1, sizeof(ApproachID_t))) == NULL)
						has_error = true;
					else
						*(pGenericLane->ingressApproach) = approachStruct.id;
					break;
				}
				if (has_error)
				{
					std::cerr << "encode_mapdata_payload: intersectionId=" << mapDataIn.id;
					std::cerr << ", failed allocate " << allocate_level << branch_level << ".ApproachID" << std::endl;
					break;
				}
				// AllowedManeuvers - 12 bits BIT STRING
				if (approachStruct.type != MsgEnum::approachType::crosswalk)
				{
					if (((pGenericLane->maneuvers = (AllowedManeuvers_t *)calloc(1, sizeof(AllowedManeuvers_t))) == NULL)
						|| !(ul2bitString(&pGenericLane->maneuvers->buf, pGenericLane->maneuvers->size,
							pGenericLane->maneuvers->bits_unused, 12, (laneStruct.attributes.to_ulong() >> 8))))
					{
						std::cerr << "encode_mapdata_payload: intersectionId=" << mapDataIn.id;
						std::cerr << ", failed allocate " << allocate_level << branch_level << ".AllowedManeuvers" << std::endl;
						has_error = true;
						break;
					}
				}

				// ConnectsToList
				if (!laneStruct.mpConnectTo.empty())
				{
					pGenericLane->connectsTo = (ConnectsToList_t *)calloc(1, sizeof(ConnectsToList_t));
					if (pGenericLane->connectsTo != NULL)
						pGenericLane->connectsTo->list.array = (Connection_t **)calloc(laneStruct.mpConnectTo.size(), sizeof(Connection_t *));
					if ((pGenericLane->connectsTo == NULL) || (pGenericLane->connectsTo->list.array == NULL))
					{
						std::cerr << "encode_mapdata_payload: intersectionId=" << mapDataIn.id;
						std::cerr << ", failed allocate " << allocate_level << branch_level << ".ConnectsToList" << std::endl;
						has_error = true;
						break;
					}
					pGenericLane->connectsTo->list.size = static_cast<int>(laneStruct.mpConnectTo.size());
					int& connListCnt = pGenericLane->connectsTo->list.count;
					for (const auto& connStruct : laneStruct.mpConnectTo)
					{
						if ((pGenericLane->connectsTo->list.array[connListCnt] = (Connection_t *)calloc(1, sizeof(Connection_t))) == NULL)
						{
							std::cerr << "encode_mapdata_payload: intersectionId=" << mapDataIn.id;
							std::cerr << ", failed allocate " << allocate_level << branch_level << ".ConnectsToList.Connection" << std::endl;
							has_error = true;
							break;
						}
						Connection_t* pConnection = pGenericLane->connectsTo->list.array[connListCnt++];
						// Connection::ConnectingLane
						auto& connLane = pConnection->connectingLane;
						connLane.lane = connStruct.laneId;
						if (connStruct.laneManeuver != MsgEnum::maneuverType::unavailable)
						{ // ConnectingLane::AllowedManeuvers - 12 bits BIT STRING
							std::bitset<12> connecting_maneuvers;
							switch(connStruct.laneManeuver)
							{
							case MsgEnum::maneuverType::uTurn:
								connecting_maneuvers.set(3);
								break;
							case MsgEnum::maneuverType::leftTurn:
								connecting_maneuvers.set(1);
								break;
							case MsgEnum::maneuverType::rightTurn:
								connecting_maneuvers.set(2);
								break;
							case MsgEnum::maneuverType::straightAhead:
							case MsgEnum::maneuverType::straight:
								connecting_maneuvers.set(0);
								break;
							default:
								break;
							}
							if (((connLane.maneuver = (AllowedManeuvers_t *)calloc(1, sizeof(AllowedManeuvers_t))) == NULL)
								|| !(ul2bitString(&connLane.maneuver->buf, connLane.maneuver->size,
									connLane.maneuver->bits_unused, 12, connecting_maneuvers.to_ulong())))
							{
								std::cerr << "encode_mapdata_payload: intersectionId=" << mapDataIn.id;
								std::cerr << ", failed allocate " << allocate_level << branch_level;
								std::cerr << ".ConnectsToList.Connection.connectingLane.AllowedManeuvers" << std::endl;
								has_error = true;
								break;
							}
						}
						// Connection::IntersectionReferenceID
						if (connStruct.intersectionId != mapDataIn.id)
						{
							if ((pConnection->remoteIntersection = (IntersectionReferenceID_t *)calloc(1, sizeof(IntersectionReferenceID_t))) == NULL)
							{
								std::cerr << "encode_mapdata_payload: intersectionId=" << mapDataIn.id;;
								std::cerr << ", failed allocate " << allocate_level << branch_level;
								std::cerr << ".ConnectsToList.Connection.IntersectionReferenceID" << std::endl;
								has_error = true;
								break;
							}
							pConnection->remoteIntersection->id = connStruct.intersectionId;
						}
						// Connection::signalGroup
						if (laneStruct.controlPhase != 0)
						{
							if ((pConnection->signalGroup = (SignalGroupID_t *)calloc(1, sizeof(SignalGroupID_t))) == NULL)
							{
								std::cerr << "encode_mapdata_payload: intersectionId=" << mapDataIn.id;
								std::cerr << ", failed allocate " << allocate_level << branch_level << ".ConnectsToList.Connection.signalGroup" << std::endl;
								has_error = true;
								break;
							}
							*(pConnection->signalGroup) = laneStruct.controlPhase;
						}
					} // end of loop through connectsTo
					if (has_error)
						break;
				}

				// NodeListXY
				pGenericLane->nodeList.present = NodeListXY_PR_nodes; // NodeSetXY
				auto& nodeSet = pGenericLane->nodeList.choice.nodes;
				if ((nodeSet.list.array = (NodeXY_t **)calloc(laneStruct.mpNodes.size(), sizeof(NodeXY_t *))) == NULL)
				{
					std::cerr << "encode_mapdata_payload: intersectionId=" << mapDataIn.id;
					std::cerr << ", failed allocate " << allocate_level << branch_level << ".NodeListXY" << std::endl;
					has_error = true;
					break;
				}
				nodeSet.list.size = static_cast<int>(laneStruct.mpNodes.size());
				// NodeSetXY::NodeXY
				int& nodeListCnt = nodeSet.list.count;
				for (auto it = laneStruct.mpNodes.cbegin(); it != laneStruct.mpNodes.cend(); ++it)
				{
					const auto& offset_x = it->offset_x;
					const auto& offset_y = it->offset_y;
					uint32_t offset_dist = static_cast<uint32_t>(std::sqrt(offset_x * offset_x + offset_y * offset_y));
					if ((nodeSet.list.array[nodeListCnt] = (NodeXY_t *)calloc(1, sizeof(NodeXY_t))) == NULL)
					{
						std::cerr << "encode_mapdata_payload: intersectionId=" << mapDataIn.id;
						std::cerr << ", failed allocate " << allocate_level << branch_level	<< ".NodeListXY.NodeXY" << std::endl;
						has_error = true;
						break;
					}
					NodeXY_t* pNode = nodeSet.list.array[nodeListCnt++];
					// NodeXY::NodeOffsetPointXY
					if (offset_dist <= 511)
					{
						pNode->delta.present = NodeOffsetPointXY_PR_node_XY1;
						pNode->delta.choice.node_XY1.x = offset_x;
						pNode->delta.choice.node_XY1.y = offset_y;
					}
					else if (offset_dist <= 1023)
					{
						pNode->delta.present = NodeOffsetPointXY_PR_node_XY2;
						pNode->delta.choice.node_XY2.x = offset_x;
						pNode->delta.choice.node_XY2.y = offset_y;
					}
					else if (offset_dist <= 2047)
					{
						pNode->delta.present = NodeOffsetPointXY_PR_node_XY3;
						pNode->delta.choice.node_XY3.x = offset_x;
						pNode->delta.choice.node_XY3.y = offset_y;
					}
					else if (offset_dist <= 4096)
					{
						pNode->delta.present = NodeOffsetPointXY_PR_node_XY4;
						pNode->delta.choice.node_XY4.x = offset_x;
						pNode->delta.choice.node_XY4.y = offset_y;
					}
					else if (offset_dist <= 8191)
					{
						pNode->delta.present = NodeOffsetPointXY_PR_node_XY5;
						pNode->delta.choice.node_XY5.x = offset_x;
						pNode->delta.choice.node_XY5.y = offset_y;
					}
					else
					{
						pNode->delta.present = NodeOffsetPointXY_PR_node_XY6;
						pNode->delta.choice.node_XY6.x = offset_x;
						pNode->delta.choice.node_XY6.y = offset_y;
					}
					// NodeXY::NodeAttributeSetXY - lane width adjustment w.r.t. refLaneWidth
					if ((laneStruct.width != refLaneWidth) && (it == laneStruct.mpNodes.cbegin()))
					{
						pNode->attributes = (NodeAttributeSetXY_t *)calloc(1, sizeof(NodeAttributeSetXY_t));
						if (pNode->attributes != NULL)
							pNode->attributes->dWidth = (Offset_B10_t *)calloc(1, sizeof(Offset_B10_t));
						if ((pNode->attributes == NULL) || (pNode->attributes->dWidth == NULL))
						{
							std::cerr << "encode_mapdata_payload: intersectionId=" << mapDataIn.id;
							std::cerr << ", failed allocate " << allocate_level << branch_level << ".NodeListXY.NodeXY.NodeAttributeSetXY" << std::endl;
							has_error = true;
							break;
						}
						*(pNode->attributes->dWidth) = laneStruct.width - refLaneWidth;
					}
				}
				if (has_error)
					break;
			}
			if (has_error)
				break;
		}
		if (has_error)
			break;
	}
	if (has_error)
	{
		ASN_STRUCT_FREE(asn_DEF_MapData, pMapData);
		return(0);
	}
	// encode MapData
	asn_enc_rval_t rval = uper_encode_to_buffer(&asn_DEF_MapData, pMapData, buf, size);
	ASN_STRUCT_FREE(asn_DEF_MapData, pMapData);
	return(numbits2numbytes(rval.encoded));
}

size_t AsnJ2735Lib::decode_mapdata_payload(const uint8_t* buf, size_t size, MapData_element_t& mapDataOut)
{
	MapData_t* pMapData = NULL;
	asn_dec_rval_t rval = uper_decode(0, &asn_DEF_MapData,(void **)&pMapData, buf, size, 0, 0);
	if (rval.code != RC_OK)
	{
		std::cerr << "decode_mapdata_payload: failed UPER decoding" << std::endl;
		ASN_STRUCT_FREE(asn_DEF_MapData, pMapData);
		return(0);
	}
	if ((pMapData == NULL) || (pMapData->intersections == NULL) || (pMapData->intersections->list.count == 0))
	{
		std::cerr << "decode_mapdata_payload: empty IntersectionGeometryList" << std::endl;
		ASN_STRUCT_FREE(asn_DEF_MapData, pMapData);
		return(0);
	}
	mapDataOut.reset();
	mapDataOut.id = static_cast<uint16_t>(pMapData->intersections->list.array[0]->id.id);
	mapDataOut.mapVersion = static_cast<uint8_t>(pMapData->msgIssueRevision);
	mapDataOut.mpApproaches.resize(12);  // 8 vehicular traffic approaches plus 4 crosswalks
	mapDataOut.attributes.set(1);        // Geometric data is included
	const auto& position3D = pMapData->intersections->list.array[0]->refPoint;
	mapDataOut.geoRef.latitude = static_cast<int32_t>(position3D.lat);
	mapDataOut.geoRef.longitude = static_cast<int32_t>(position3D.Long);
	if ((position3D.elevation != NULL) && (*(position3D.elevation) != (Elevation_t)MsgEnum::unknown_elevation))
	{ // Elevation data is included
		mapDataOut.attributes.set(0);
		mapDataOut.geoRef.elevation = static_cast<int32_t>(*(position3D.elevation));
	}
	else
		mapDataOut.geoRef.elevation = 0;
	mapDataOut.mapPayload.assign(buf, buf + size);
	// loop through speed groups
	bool has_error = false;
	for (int i = 0; i < pMapData->intersections->list.count; i++)
	{
		const IntersectionGeometry_t* pIntersectionGeometry = pMapData->intersections->list.array[i];
		if (static_cast<uint16_t>(pIntersectionGeometry->id.id) != mapDataOut.id)
		{
			std::cerr << "decode_mapdata_payload: intersectionId=" << mapDataOut.id;
			std::cerr << ", containing multiples IntersectionReferenceID" << std::endl;
			has_error = true;
			break;
		}
		if (pIntersectionGeometry->laneWidth == NULL)
		{
			std::cerr << "decode_mapdata_payload: intersectionId=" << mapDataOut.id;
			std::cerr << ", missing LaneWidth" << std::endl;
			has_error = true;
			break;
		}
		uint16_t refLaneWidth = static_cast<uint16_t>(*(pIntersectionGeometry->laneWidth));
		if (pIntersectionGeometry->laneSet.list.count == 0)
		{
			std::cerr << "decode_mapdata_payload: intersectionId=" << mapDataOut.id;
			std::cerr << ", missing LaneList" << std::endl;
			has_error = true;
			break;
		}
		// speed limit for this speed group
		const auto& firstLaneDirection = pIntersectionGeometry->laneSet.list.array[0]->laneAttributes.directionalUse;
		uint8_t  laneDirectionalUse = static_cast<uint8_t>(bitString2ul(firstLaneDirection.buf, firstLaneDirection.size, firstLaneDirection.bits_unused));
		if ((laneDirectionalUse < 0x01) || (laneDirectionalUse > 0x03))
		{
			std::cerr << "decode_mapdata_payload: intersectionId=" << mapDataOut.id;
			std::cerr << ", unknown laneDirectionalUse=" << static_cast<unsigned int>(laneDirectionalUse);
			std::cerr << ", bitString size=" << firstLaneDirection.size;
			std::cerr << ", bits_unused=" << firstLaneDirection.bits_unused << std::endl;
			has_error = true;
			break;
		}
		uint16_t speed_limit = (laneDirectionalUse == 0x03) ? 0 : MsgEnum::unknown_speed;
		// check whether or not the speed limit is set for vehicular traffic lanes
		if ((laneDirectionalUse != 0x03) && (pIntersectionGeometry->speedLimits != NULL))
		{ // loop through speedLimits to find SpeedLimitType_vehicleMaxSpeed entry
			for (int j = 0; j < pIntersectionGeometry->speedLimits->list.count; j++)
			{
				const RegulatorySpeedLimit_t* pRegulatorySpeedLimit = pIntersectionGeometry->speedLimits->list.array[j];
				if ((pRegulatorySpeedLimit->type == SpeedLimitType_vehicleMaxSpeed)
					&& (pRegulatorySpeedLimit->speed != (Velocity_t)MsgEnum::unknown_speed))
				{ // speed limit data is included
					mapDataOut.attributes.set(2);
					speed_limit = static_cast<uint16_t>(pRegulatorySpeedLimit->speed);
					break;
				}
			}
		}
		// loop through lanes
		for (int j = 0; j < pIntersectionGeometry->laneSet.list.count; j++)
		{
			const GenericLane_t* pGenericLane = pIntersectionGeometry->laneSet.list.array[j];
			const auto& laneDirection = pGenericLane->laneAttributes.directionalUse;
			uint8_t directionalUse = static_cast<uint8_t>(bitString2ul(laneDirection.buf, laneDirection.size, laneDirection.bits_unused));
			if ((directionalUse < 0x01) || (directionalUse > 0x03))
			{
				std::cerr << "decode_mapdata_payload: intersectionId=" << mapDataOut.id;
				std::cerr << ", laneId=" << pGenericLane->laneID;
				std::cerr << ", unknown directionalUse=" << static_cast<unsigned int>(directionalUse);
				std::cerr << ", bitString size=" << laneDirection.size;
				std::cerr << ", bits_unused=" << laneDirection.bits_unused << std::endl;
				has_error = true;
				break;
			}
			if ((pGenericLane->ingressApproach == NULL) && (pGenericLane->egressApproach == NULL))
			{
				std::cerr << "decode_mapdata_payload: intersectionId=" << mapDataOut.id;
				std::cerr << ", laneId=" << pGenericLane->laneID << ", missing ApproachID" << std::endl;
				has_error = true;
				break;
			}
			if ((directionalUse != 0x02)  // not an outbound traffic lane
				&& ((pGenericLane->connectsTo == NULL) || (pGenericLane->connectsTo->list.count == 0)
					|| (pGenericLane->connectsTo->list.array[0]->signalGroup == NULL)))
			{ // required to provide signal phase that controls the movement
				std::cerr << "decode_mapdata_payload: intersectionId=" << mapDataOut.id;
				std::cerr << ", laneId=" << pGenericLane->laneID << ", missing ConnectsTo" << std::endl;
				has_error = true;
				break;
			}
			if ((directionalUse != 0x03) && (pGenericLane->maneuvers == NULL))
			{ // not crosswalk
				std::cerr << "decode_mapdata_payload: intersectionId=" << mapDataOut.id;
				std::cerr << ", laneId=" << pGenericLane->laneID << ", missing AllowedManeuvers" << std::endl;
				has_error = true;
				break;
			}
			if ((pGenericLane->nodeList.present != NodeListXY_PR_nodes)
				|| (pGenericLane->nodeList.choice.nodes.list.array == NULL)
				|| (pGenericLane->nodeList.choice.nodes.list.count < 2)
				|| (pGenericLane->nodeList.choice.nodes.list.count > 63))
			{
				std::cerr << "decode_mapdata_payload: intersectionId=" << mapDataOut.id;
				std::cerr << ", laneId=" << pGenericLane->laneID << ", missing NodeListXY" << std::endl;
				has_error = true;
				break;
			}

			uint8_t approachId = static_cast<uint8_t>((pGenericLane->ingressApproach != NULL) ?
				(*(pGenericLane->ingressApproach)) : (*(pGenericLane->egressApproach)));
			if ((approachId > 0) && (approachId <= 12) && (mapDataOut.mpApproaches[approachId-1].id != approachId))
			{ // assign ApproachStruct variables
				mapDataOut.mpApproaches[approachId-1].id = approachId;
				mapDataOut.mpApproaches[approachId-1].speed_limit = speed_limit;
				if (std::find(mapDataOut.speeds.begin(), mapDataOut.speeds.end(),	speed_limit) == mapDataOut.speeds.end())
					mapDataOut.speeds.push_back(speed_limit);
				switch(directionalUse)
				{
				case 0x01:
					mapDataOut.mpApproaches[approachId-1].type = MsgEnum::approachType::inbound;
					break;
				case 0x02:
					mapDataOut.mpApproaches[approachId-1].type = MsgEnum::approachType::outbound;
					break;
				case 0x03:
					mapDataOut.mpApproaches[approachId-1].type = MsgEnum::approachType::crosswalk;
					break;
				}
			}

			// assign LaneStruct variables
			lane_element_t laneStruct;
			laneStruct.id = static_cast<uint8_t>(pGenericLane->laneID);
			laneStruct.type = (directionalUse == 0x03) ? (MsgEnum::laneType::crosswalk) : (MsgEnum::laneType::traffic);
			const auto& laneType = pGenericLane->laneAttributes.laneType;
			unsigned long laneTypeAttrib = (laneType.present == LaneTypeAttributes_PR_crosswalk)
				? bitString2ul(laneType.choice.crosswalk.buf, laneType.choice.crosswalk.size, laneType.choice.crosswalk.bits_unused)
				: bitString2ul(laneType.choice.vehicle.buf, laneType.choice.vehicle.size, laneType.choice.vehicle.bits_unused);
			unsigned long allowedManeuvers = (directionalUse == 0x03) ? 0
				: bitString2ul(pGenericLane->maneuvers->buf, pGenericLane->maneuvers->size, pGenericLane->maneuvers->bits_unused);
			laneStruct.attributes = std::bitset<20>(allowedManeuvers << 8 | laneTypeAttrib);
			laneStruct.width = ((pGenericLane->nodeList.choice.nodes.list.array[0]->attributes != NULL)
				&& (pGenericLane->nodeList.choice.nodes.list.array[0]->attributes->dWidth != NULL))
				? (uint16_t)(refLaneWidth + *(pGenericLane->nodeList.choice.nodes.list.array[0]->attributes->dWidth))
				: refLaneWidth;
			laneStruct.controlPhase = ((pGenericLane->connectsTo == NULL) || (pGenericLane->connectsTo->list.count == 0)
				|| (pGenericLane->connectsTo->list.array[0]->signalGroup == NULL)) ? 0
				: static_cast<uint8_t>(*(pGenericLane->connectsTo->list.array[0]->signalGroup));
			// loop through connecting lanes
			if (pGenericLane->connectsTo != NULL)
			{
				laneStruct.mpConnectTo.resize(pGenericLane->connectsTo->list.count);
				for (int k = 0; k < pGenericLane->connectsTo->list.count; k++)
				{ // assign ConnectStruct variables
					const Connection_t* pConnection = pGenericLane->connectsTo->list.array[k];
					const auto& connectingLane = pConnection->connectingLane;
					laneStruct.mpConnectTo[k].intersectionId = (pConnection->remoteIntersection == NULL) ?
						mapDataOut.id : static_cast<uint16_t>(pConnection->remoteIntersection->id);
					laneStruct.mpConnectTo[k].laneId = static_cast<uint8_t>(connectingLane.lane);
					unsigned long connecting_maneuvers = (connectingLane.maneuver == NULL) ? 0 :
						bitString2ul(connectingLane.maneuver->buf, connectingLane.maneuver->size, connectingLane.maneuver->bits_unused);
					switch(connecting_maneuvers)
					{
					case 0x00:
						laneStruct.mpConnectTo[k].laneManeuver = MsgEnum::maneuverType::unavailable;
						break;
					case 0x01:
						laneStruct.mpConnectTo[k].laneManeuver = (directionalUse == 0x01) ?
							(MsgEnum::maneuverType::straightAhead) : (MsgEnum::maneuverType::straight);
						break;
					case 0x02:
						laneStruct.mpConnectTo[k].laneManeuver = MsgEnum::maneuverType::leftTurn;
						break;
					case 0x04:
						laneStruct.mpConnectTo[k].laneManeuver = MsgEnum::maneuverType::rightTurn;
						break;
					case 0x08:
						laneStruct.mpConnectTo[k].laneManeuver = MsgEnum::maneuverType::uTurn;
						break;
					}
				}
			}
			// loop through lane nodes
			laneStruct.mpNodes.resize(pGenericLane->nodeList.choice.nodes.list.count);
			int nodeCnt = 0;
			for (int k = 0; k < pGenericLane->nodeList.choice.nodes.list.count; k++)
			{
				const NodeXY_t* pNode = pGenericLane->nodeList.choice.nodes.list.array[k];
				int32_t offset_x, offset_y;
				switch(pNode->delta.present)
				{
				case NodeOffsetPointXY_PR_node_XY1:
					offset_x = static_cast<int32_t>(pNode->delta.choice.node_XY1.x);
					offset_y = static_cast<int32_t>(pNode->delta.choice.node_XY1.y);
					break;
				case NodeOffsetPointXY_PR_node_XY2:
					offset_x = static_cast<int32_t>(pNode->delta.choice.node_XY2.x);
					offset_y = static_cast<int32_t>(pNode->delta.choice.node_XY2.y);
					break;
				case NodeOffsetPointXY_PR_node_XY3:
					offset_x = static_cast<int32_t>(pNode->delta.choice.node_XY3.x);
					offset_y = static_cast<int32_t>(pNode->delta.choice.node_XY3.y);
					break;
				case NodeOffsetPointXY_PR_node_XY4:
					offset_x = static_cast<int32_t>(pNode->delta.choice.node_XY4.x);
					offset_y = static_cast<int32_t>(pNode->delta.choice.node_XY4.y);
					break;
				case NodeOffsetPointXY_PR_node_XY5:
					offset_x = static_cast<int32_t>(pNode->delta.choice.node_XY5.x);
					offset_y = static_cast<int32_t>(pNode->delta.choice.node_XY5.y);
					break;
				case NodeOffsetPointXY_PR_node_XY6:
					offset_x = static_cast<int32_t>(pNode->delta.choice.node_XY6.x);
					offset_y = static_cast<int32_t>(pNode->delta.choice.node_XY6.y);
					break;
				default:
					continue;
					break;
				}
				laneStruct.mpNodes[nodeCnt].offset_x = offset_x;
				laneStruct.mpNodes[nodeCnt].offset_y = offset_y;
				nodeCnt++;
			}
			if (nodeCnt != pGenericLane->nodeList.choice.nodes.list.count)
				laneStruct.mpNodes.resize(nodeCnt);
			mapDataOut.mpApproaches[approachId-1].mpLanes.push_back(laneStruct);
		}
		if (has_error)
			break;
	}
	ASN_STRUCT_FREE(asn_DEF_MapData, pMapData);
	return((has_error) ? (0) : (numbits2numbytes(rval.consumed)));
}

size_t AsnJ2735Lib::encode_spat_payload(const SPAT_element_t& spatIn, uint8_t* buf, size_t size)
{ // get array of signalGroupID for permitted vehicular and pedestrian phases
	std::vector<int> signalGroupArray;
	for (uint8_t i = 0; i < 8; i++)
	{ // vehicular phases
		if (spatIn.permittedPhases.test(i))
			signalGroupArray.push_back(i);
	}
	for (uint8_t i = 0; i < 8; i++)
	{ // pedestrian phases
		if (spatIn.permittedPedPhases.test(i))
			signalGroupArray.push_back(i + 8);
	}
	if (signalGroupArray.empty())
		return(0);   // nothing to encode

	std::string allocate_level{"SPAT"};
	SPAT_t* pSPAT = (SPAT_t *)calloc(1, sizeof(SPAT_t));
	if (pSPAT == NULL)
	{
		std::cerr << "encode_spat_payload: failed allocate " << allocate_level << std::endl;
		return(0);
	}
	// SPAT:
	// -- Required objects ------------------------------------ //
	//	IntersectionStateList
	// -- OPTIONAL objects ------------ including/excluding  -- //
	//	MinuteOfTheYear                     EXCL
	//	DescriptiveName                     EXCL
	//	RegionalExtension                   EXCL
	// -------------------------------------------------------- //

	// IntersectionStateList - one intersection per SPAT
	allocate_level += ".IntersectionStateList";
	if ((pSPAT->intersections.list.array = (IntersectionState_t **)calloc(1, sizeof(IntersectionState_t *))) == NULL)
	{
		std::cerr << "encode_spat_payload: failed allocate " << allocate_level << std::endl;
		free(pSPAT);
		return(0);
	}
	pSPAT->intersections.list.size  = 1;
	allocate_level += ".IntersectionState";
	if ((pSPAT->intersections.list.array[0] = (IntersectionState_t *)calloc(1, sizeof(IntersectionState_t))) == NULL)
	{
		std::cerr << "encode_spat_payload: failed allocate " << allocate_level << std::endl;
		free(pSPAT->intersections.list.array);
		free(pSPAT);
		return(0);
	}
	pSPAT->intersections.list.count = 1;
	IntersectionState_t* pIntsectionState = pSPAT->intersections.list.array[0];
	// IntersectionStateList::IntersectionState
	// -- Required objects ------------------------------------ //
	//	IntersectionReferenceID
	//	MsgCount
	//	IntersectionStatusObject
	//	MovementList
	// -- OPTIONAL objects ------------ including/excluding  -- //
	//	DescriptiveName                     EXCL
	//	MinuteOfTheYear                     OPTIONAL
	//	DSecond                             OPTIONAL
	//	EnabledLaneList                     EXCL
	//	ManeuverAssistList                  EXCL
	//	RegionalExtension                   EXCL
	// -------------------------------------------------------- //

	// IntersectionReferenceID
	pIntsectionState->id.id = spatIn.id;
	// MsgCount
	pIntsectionState->revision = spatIn.msgCnt;
	// IntersectionStatusObject - 16 bits BIT STRING
	if (!ul2bitString(&pIntsectionState->status.buf, pIntsectionState->status.size,
		pIntsectionState->status.bits_unused, 16, spatIn.status.to_ulong()))
	{
		std::cerr << "encode_spat_payload: failed allocate " << allocate_level;
		std::cerr << ".IntersectionStatusObject" << std::endl;
		ASN_STRUCT_FREE(asn_DEF_SPAT, pSPAT);
		return(0);
	}
	// TimeStamp
	if (spatIn.timeStampMinute < MsgEnum::invalid_timeStampMinute)
	{
		if ((pIntsectionState->moy = (MinuteOfTheYear_t *)calloc(1, sizeof(MinuteOfTheYear_t))) == NULL)
		{
			std::cerr << "encode_spat_payload: failed allocate " << allocate_level;
			std::cerr	<< ".MinuteOfTheYear" << std::endl;
			ASN_STRUCT_FREE(asn_DEF_SPAT, pSPAT);
			return(0);
		}
		*(pIntsectionState->moy) = spatIn.timeStampMinute;
	}
	if (spatIn.timeStampSec < 0xFFFF)
	{
		if ((pIntsectionState->timeStamp = (DSecond_t *)calloc(1, sizeof(DSecond_t))) == NULL)
		{
			std::cerr << "encode_spat_payload: failed allocate " << allocate_level;
			std::cerr	<< ".timeStamp" << std::endl;
			ASN_STRUCT_FREE(asn_DEF_SPAT, pSPAT);
			return(0);
		}
		*(pIntsectionState->timeStamp) = spatIn.timeStampSec;
	}

	// MovementList
	allocate_level += ".MovementList"; // one MovementState per vehicular/pedestrian signal group
	if ((pIntsectionState->states.list.array =
		(MovementState_t **)calloc(signalGroupArray.size(), sizeof(MovementState_t *))) == NULL)
	{
		std::cerr << "encode_spat_payload: failed allocate " << allocate_level << std::endl;
		ASN_STRUCT_FREE(asn_DEF_SPAT, pSPAT);
		return(0);
	}
	pIntsectionState->states.list.size  = static_cast<int>(signalGroupArray.size());

	// loop through permitted movements
	allocate_level += ".MovementState";
	bool has_error = false;  // for earlier return
	int& stateListCnt = pIntsectionState->states.list.count;
	for (const auto& signal_group : signalGroupArray)
	{
		const PhaseState_element_t& phaseState = (signal_group < 8) ?
			spatIn.phaseState[signal_group] : spatIn.pedPhaseState[signal_group - 8];
		// allocate MovementState object
		if ((pIntsectionState->states.list.array[stateListCnt] = (MovementState_t *)calloc(1, sizeof(MovementState_t))) == NULL)
		{
			std::cerr << "encode_spat_payload: failed allocate " << allocate_level << std::endl;
			has_error = true;
			break;
		}
		MovementState_t* pMovementState = pIntsectionState->states.list.array[stateListCnt++];
		// MovementState
		// -- Required objects ------------------------------------ //
		//	SignalGroupID
		//	MovementEventList
		// -- OPTIONAL objects ------------ including/excluding  -- //
		//	DescriptiveName                     EXCL
		//	ManeuverAssistList                  EXCL
		//	RegionalExtension                   EXCL
		// -------------------------------------------------------- //

		// SignalGroupID
		pMovementState->signalGroup = signal_group + 1;
		// MovementEventList - one MovementEvent per movement
		std::string branch_level(".MovementEventList");
		if ((pMovementState->state_time_speed.list.array =
			(MovementEvent_t **)calloc(1, sizeof(MovementEvent_t *))) == NULL)
		{
			std::cerr << "encode_spat_payload: failed allocate " << allocate_level;
			std::cerr	<< branch_level << std::endl;
			has_error = true;
			break;
		}
		pMovementState->state_time_speed.list.size  = 1;
		branch_level += ".MovementEvent";
		if ((pMovementState->state_time_speed.list.array[0] =
			(MovementEvent_t *)calloc(1, sizeof(MovementEvent_t))) == NULL)
		{
			std::cerr << "encode_spat_payloadL: failed allocate " << allocate_level;
			std::cerr << branch_level << std::endl;
			has_error = true;
			break;
		}
		pMovementState->state_time_speed.list.count = 1;
		MovementEvent_t* pMovementEvent = pMovementState->state_time_speed.list.array[0];
		// MovementEvent
		// -- Required objects ------------------------------------ //
		//	MovementPhaseState
		// -- OPTIONAL objects ------------ including/excluding  -- //
		//	TimeChangeDetails                   OPTIONAL
		//	AdvisorySpeedList                   EXCL
		//	RegionalExtension                   EXCL
		// -------------------------------------------------------- //

		// MovementPhaseState
		pMovementEvent->eventState = static_cast<MovementPhaseState_t>(phaseState.currState);
		branch_level += ".TimeChangeDetails";
		// TimeChangeDetails
		// -- Required objects ------------------------------------ //
		// 	minEndTime
		// -- OPTIONAL objects ------------ including/excluding  -- //
		//	startTime                           OPTIONAL
		//	maxEndTime                          OPTIONAL
		//	likelyTime                          EXCL
		//	confidence (on likelyTime)          EXCL
		//	nextTime                            EXCL
		// -------------------------------------------------------- //
		if (phaseState.minEndTime < MsgEnum::unknown_timeDetail)
		{
			if ((pMovementEvent->timing = (TimeChangeDetails *)calloc(1, sizeof(TimeChangeDetails))) == NULL)
			{
				std::cerr << "encode_spat_payloadL: failed allocate " << allocate_level;
				std::cerr	<< branch_level << std::endl;
				has_error = true;
				break;
			}
			// minEndTime
			pMovementEvent->timing->minEndTime = phaseState.minEndTime;
			// startTime
			if (phaseState.startTime < MsgEnum::unknown_timeDetail)
			{
				if ((pMovementEvent->timing->startTime = (TimeMark_t *)calloc(1, sizeof(TimeMark_t))) == NULL)
				{
					std::cerr << "encode_spat_payloadL: failed allocate " << allocate_level;
					std::cerr	<< branch_level	<< ".startTime" << std::endl;
					has_error = true;
					break;
				}
				*(pMovementEvent->timing->startTime) = phaseState.startTime;
			}
			// maxEndTime
			if (phaseState.maxEndTime < MsgEnum::unknown_timeDetail)
			{
				if ((pMovementEvent->timing->maxEndTime = (TimeMark_t *)calloc(1, sizeof(TimeMark_t))) == NULL)
				{
					std::cerr << "encode_spat_payloadL: failed allocate " << allocate_level;
					std::cerr	<< branch_level	<< ".maxEndTime" << std::endl;
					has_error = true;
					break;
				}
				*(pMovementEvent->timing->maxEndTime) = phaseState.maxEndTime;
			}
		}
	}
	if (has_error)
	{
		ASN_STRUCT_FREE(asn_DEF_SPAT, pSPAT);
		return(0);
	}
	// encode SPAT
	asn_enc_rval_t rval = uper_encode_to_buffer(&asn_DEF_SPAT, pSPAT, buf, size);
	ASN_STRUCT_FREE(asn_DEF_SPAT, pSPAT);
	return(numbits2numbytes(rval.encoded));
}

size_t AsnJ2735Lib::decode_spat_payload(const uint8_t* buf, size_t size, SPAT_element_t& spatOut)
{
	SPAT_t* pSPAT = NULL;
	asn_dec_rval_t rval = uper_decode(0, &asn_DEF_SPAT,(void **)&pSPAT, buf, size, 0, 0);
	if (rval.code != RC_OK)
	{
		std::cerr << "decode_spat_payload: failed UPER decoding" << std::endl;
		ASN_STRUCT_FREE(asn_DEF_SPAT, pSPAT);
		return(0);
	}
	if ((pSPAT == NULL) || (pSPAT->intersections.list.count == 0))
	{
		std::cerr << "decode_spat_payload: empty IntersectionStateList" << std::endl;
		ASN_STRUCT_FREE(asn_DEF_SPAT, pSPAT);
		return(0);
	}
	spatOut.reset();
	const IntersectionState_t* pIntsectionState = pSPAT->intersections.list.array[0];
	spatOut.id = static_cast<uint16_t>(pIntsectionState->id.id);
	spatOut.msgCnt = static_cast<uint8_t>(pIntsectionState->revision);
	if (pIntsectionState->moy != NULL)
		spatOut.timeStampMinute = static_cast<uint32_t>(*(pIntsectionState->moy));
	if (pIntsectionState->timeStamp != NULL)
		spatOut.timeStampSec = static_cast<uint16_t>(*(pIntsectionState->timeStamp));
	spatOut.status = std::bitset<16>(bitString2ul(pIntsectionState->status.buf, pIntsectionState->status.size, pIntsectionState->status.bits_unused));
	if (pIntsectionState->states.list.count == 0)
	{
		std::cerr << "decode_spat_payload: empty MovementList" << std::endl;
		ASN_STRUCT_FREE(asn_DEF_SPAT, pSPAT);
		return(0);
	}
	auto& permittedPhases = spatOut.permittedPhases;
	auto& permittedPedPhases = spatOut.permittedPedPhases;
	bool has_error = false;
	for (int i = 0; i < pIntsectionState->states.list.count; i++)
	{
		const MovementState_t* pMovementState = pIntsectionState->states.list.array[i];
		if ((pMovementState->signalGroup < 1) || (pMovementState->signalGroup > 2 * 8))
		{
			std::cerr << "decode_spat_payload: invalid SignalGroupID" << std::endl;
			has_error = true;
			break;
		}
		if (pMovementState->state_time_speed.list.count == 0)
			continue;
		const MovementEvent_t* pMovementEvent = pMovementState->state_time_speed.list.array[0];
		int j = static_cast<int>((pMovementState->signalGroup - 1) % 8);
		PhaseState_element_t& phaseState = (pMovementState->signalGroup > 8) ?
			spatOut.pedPhaseState[j] : spatOut.phaseState[j];
		if (pMovementState->signalGroup > 8)
			permittedPedPhases.set(j);
		else
			permittedPhases.set(j);
		phaseState.currState = static_cast<MsgEnum::phaseState>(pMovementEvent->eventState);
		if (pMovementEvent->timing != NULL)
		{
			phaseState.minEndTime = static_cast<uint16_t>(pMovementEvent->timing->minEndTime);
			if (pMovementEvent->timing->startTime != NULL)
				phaseState.startTime = static_cast<uint16_t>(*(pMovementEvent->timing->startTime));
			if (pMovementEvent->timing->maxEndTime != NULL)
				phaseState.maxEndTime = static_cast<uint16_t>(*(pMovementEvent->timing->maxEndTime));
		}
	}
	ASN_STRUCT_FREE(asn_DEF_SPAT, pSPAT);
	return((has_error) ? (0) : (numbits2numbytes(rval.consumed)));
}

size_t AsnJ2735Lib::encode_srm_payload(const SRM_element_t& srmIn, uint8_t* buf, size_t size)
{
	if ((srmIn.inApprochId == 0) && (srmIn.inLaneId == 0))
	{
		std::cerr << "encode_srm_payload: either entry approachID or laneID shall be specified" << std::endl;
		return(0);
	}

	std::string allocate_level{"SRM"};
	SignalRequestMessage_t* pSRM = (SignalRequestMessage_t *)calloc(1, sizeof(SignalRequestMessage_t));
	if (pSRM == NULL)
	{
		std::cerr << "encode_srm_payload: failed allocate " << allocate_level << std::endl;
		return(0);
	}
	// SRM:
	// -- Required objects ------------------------------------ //
	//	DSecond
	//	RequestorDescription
	// -- OPTIONAL objects ------------ including/excluding  -- //
	//	MinuteOfTheYear                     OPTIONAL
	//	MsgCount                            OPTIONAL
	//	SignalRequestList                   INCL
	//	RegionalExtension                   EXCL
	// -------------------------------------------------------- //

	// DSecond
	pSRM->second = srmIn.timeStampSec;
	// SRM::MinuteOfTheYear
	if (srmIn.timeStampMinute < MsgEnum::invalid_timeStampMinute)
	{
		if ((pSRM->timeStamp = (MinuteOfTheYear_t *)calloc(1, sizeof(MinuteOfTheYear_t))) == NULL)
		{
			std::cerr << "encode_srm_payload: failed allocate " << allocate_level;
			std::cerr << ".MinuteOfTheYear" << std::endl;
			free(pSRM);
			return(0);
		}
		*(pSRM->timeStamp) = srmIn.timeStampMinute;
	}
	// MsgCount
	if (srmIn.msgCnt < 0xFF)
	{
		if ((pSRM->sequenceNumber = (MsgCount_t *)calloc(1, sizeof(MsgCount_t))) == NULL)
		{
			std::cerr << "encode_srm_payload: failed allocate " << allocate_level;
			std::cerr	<< ".MsgCount" << std::endl;
			if (pSRM->timeStamp != NULL)
				free(pSRM->timeStamp);
			free(pSRM);
			return(0);
		}
		*(pSRM->sequenceNumber) = srmIn.msgCnt;
	}
	// SignalRequestList - request for one intersection
	std::string branch_level(".SignalRequestList");
	pSRM->requests = (SignalRequestList_t *)calloc(1, sizeof(SignalRequestList_t));
	if (pSRM->requests != NULL)
		pSRM->requests->list.array = (SignalRequestPackage_t **)calloc(1, sizeof(SignalRequestPackage_t *));
	if ((pSRM->requests == NULL) || (pSRM->requests->list.array == NULL))
	{
		std::cerr << "encode_srm_payload: failed allocate " << allocate_level << branch_level << std::endl;
		if (pSRM->requests != NULL)
			free(pSRM->requests);
		if (pSRM->timeStamp != NULL)
			free(pSRM->timeStamp);
		if (pSRM->sequenceNumber != NULL)
			free(pSRM->sequenceNumber);
		free(pSRM);
		return(0);
	}
	pSRM->requests->list.size = 1;
	branch_level += ".SignalRequestPackage";
	if ((pSRM->requests->list.array[0] = (SignalRequestPackage_t *)calloc(1, sizeof(SignalRequestPackage_t))) == NULL)
	{
		std::cerr << "encode_srm_payload: failed allocate " << allocate_level << branch_level << std::endl;
		ASN_STRUCT_FREE(asn_DEF_SignalRequestMessage, pSRM);
		return(0);
	}
	pSRM->requests->list.count = 1;
	SignalRequestPackage_t* pSignalRequestPackage = pSRM->requests->list.array[0];
	// SignalRequestPackage
	// -- Required objects ------------------------------------ //
	//	SignalRequest
	// -- OPTIONAL objects ------------ including/excluding  -- //
	//	minute (ETA.minutes)                OPTIONAL
	//	second (ETA.milliseconds)           OPTIONAL
	//	duration                            OPTIONAL
	//	RegionalExtension                   EXCL
	// -------------------------------------------------------- //

	// ETA
	if (srmIn.ETAminute < MsgEnum::invalid_timeStampMinute)
	{
		if ((pSignalRequestPackage->minute = (MinuteOfTheYear_t *)calloc(1, sizeof(MinuteOfTheYear_t))) == NULL)
		{
			std::cerr << "encode_srm_payload: failed allocate " << allocate_level << branch_level;
			std::cerr << ".ETAminute" << std::endl;
			ASN_STRUCT_FREE(asn_DEF_SignalRequestMessage, pSRM);
			return(0);
		}
		*(pSignalRequestPackage->minute) = srmIn.ETAminute;
	}
	if (srmIn.ETAsec < 0xFFFF)
	{
		if ((pSignalRequestPackage->second = (DSecond_t *)calloc(1, sizeof(DSecond_t))) == NULL)
		{
			std::cerr << "encode_srm_payload: failed allocate " << allocate_level << branch_level;
			std::cerr	<< ".ETAsec" << std::endl;
			ASN_STRUCT_FREE(asn_DEF_SignalRequestMessage, pSRM);
			return(0);
		}
		*(pSignalRequestPackage->second) = srmIn.ETAsec;
	}
	// duration
	if (srmIn.duration < 0xFFFF)
	{
		if ((pSignalRequestPackage->duration = (DSecond_t *)calloc(1, sizeof(DSecond_t))) == NULL)
		{
			std::cerr << "encode_srm_payload: failed allocate " << allocate_level << branch_level;
			std::cerr	<< ".duration" << std::endl;
			ASN_STRUCT_FREE(asn_DEF_SignalRequestMessage, pSRM);
			return(0);
		}
		*(pSignalRequestPackage->duration) = srmIn.duration;
	}
	auto& signalRequest = pSignalRequestPackage->request;
	// SignalRequest
	// -- Required objects ------------------------------------ //
	//	IntersectionReferenceID
	//	RequestID
	//	PriorityRequestType
	//	inBoundLane
	// -- OPTIONAL objects ------------ including/excluding  -- //
	//	outBoundLane                        OPTIONAL
	//	RegionalExtension                   EXCL
	// -------------------------------------------------------- //

	// IntersectionReferenceID
	signalRequest.id.id = srmIn.intId;
	// RequestID
	signalRequest.requestID = srmIn.reqId;
	// PriorityRequestType
	signalRequest.requestType = static_cast<PriorityRequestType_t>(srmIn.reqType);
	// inBoundLane
	if (srmIn.inLaneId == 0)
	{
		signalRequest.inBoundLane.present = IntersectionAccessPoint_PR_approach;
		signalRequest.inBoundLane.choice.approach = srmIn.inApprochId;
	}
	else
	{
		signalRequest.inBoundLane.present = IntersectionAccessPoint_PR_lane;
		signalRequest.inBoundLane.choice.lane = srmIn.inLaneId;
	}
	// outBoundLane
	if (!((srmIn.outApproachId == 0) && (srmIn.outLaneId == 0)))
	{
		if ((signalRequest.outBoundLane = (IntersectionAccessPoint_t *)calloc(1, sizeof(IntersectionAccessPoint_t))) == NULL)
		{
			std::cerr << "encode_srm_payload: failed allocate " << allocate_level << branch_level;
			std::cerr << ".SignalRequest.outBoundLane" << std::endl;
			ASN_STRUCT_FREE(asn_DEF_SignalRequestMessage, pSRM);
			return(0);
		}
		if (srmIn.outLaneId == 0)
		{
			signalRequest.outBoundLane->present = IntersectionAccessPoint_PR_approach;
			signalRequest.outBoundLane->choice.approach = srmIn.outApproachId;
		}
		else
		{
			signalRequest.outBoundLane->present = IntersectionAccessPoint_PR_lane;
			signalRequest.outBoundLane->choice.lane = srmIn.outLaneId;
		}
	}

	// RequestorDescription
	allocate_level += ".RequestorDescription";
	auto& requestor = pSRM->requestor;
	// RequestorDescription
	// -- Required objects ------------------------------------ //
	//	VehicleID
	// -- OPTIONAL objects ------------ including/excluding  -- //
	//	RequestorType                       INCL
	//	RequestorPositionVector             INCL
	//	DescriptiveName.name                EXCL
	//	DescriptiveName.routeName           EXCL
	//	TransitVehicleStatus                EXCL
	//	TransitVehicleOccupancy             EXCL
	//	DeltaTime.transitSchedule           EXCL
	//	RegionalExtension                   EXCL
	// -------------------------------------------------------- //

	// VehicleID - TemporaryID
	requestor.id.present = VehicleID_PR_entityID;
	if (!vehId2temporaryId(&requestor.id.choice.entityID.buf, requestor.id.choice.entityID.size, srmIn.vehId))
	{
		std::cerr << "encode_srm_payload: failed allocate " << allocate_level;
		std::cerr << ".VehicleID" << std::endl;
		ASN_STRUCT_FREE(asn_DEF_SignalRequestMessage, pSRM);
		return(0);
	}
	// RequestorType
	if ((requestor.type = (RequestorType_t *)calloc(1, sizeof(RequestorType_t))) == NULL)
	{
		std::cerr << "encode_srm_payload: failed allocate " << allocate_level;
		std::cerr	<< ".RequestorType" << std::endl;
		ASN_STRUCT_FREE(asn_DEF_SignalRequestMessage, pSRM);
		return(0);
	}
	// RequestorType
	// -- Required objects ------------------------------------ //
	//	BasicVehicleRole
	// -- OPTIONAL objects ------------ including/excluding  -- //
	//	RequestSubRole                      EXCL
	//	RequestImportanceLevel              EXCL
	//	Iso3833VehicleType                  EXCL
	//	VehicleType                         INCL
	//	RegionalExtension                   EXCL
	// -------------------------------------------------------- //

	// BasicVehicleRole
	requestor.type->role = static_cast<BasicVehicleRole_t>(srmIn.vehRole);
	// VehicleType
	if ((requestor.type->hpmsType = (VehicleType_t *)calloc(1, sizeof(VehicleType_t))) == NULL)
	{
		std::cerr << "encode_srm_payload: failed allocate " << allocate_level;
		std::cerr	<< ".RequestorType.VehicleType" << std::endl;
		ASN_STRUCT_FREE(asn_DEF_SignalRequestMessage, pSRM);
		return(0);
	}
	*(requestor.type->hpmsType) = static_cast<VehicleType_t>(srmIn.vehType);

	// RequestorPositionVector
	if ((requestor.position = (RequestorPositionVector_t *)calloc(1, sizeof(RequestorPositionVector_t))) == NULL)
	{
		std::cerr << "encode_srm_payload: failed allocate " << allocate_level;
		std::cerr	<< ".RequestorPositionVector" << std::endl;
		ASN_STRUCT_FREE(asn_DEF_SignalRequestMessage, pSRM);
		return(0);
	}
	// RequestorPositionVector
	// -- Required objects ------------------------------------ //
	//	Position3D
	// -- OPTIONAL objects ------------ including/excluding  -- //
	//	heading                             INCL
	//	speed                               INCL
	// -------------------------------------------------------- //

	// Position3D
	requestor.position->position.lat  = srmIn.latitude;
	requestor.position->position.Long = srmIn.longitude;
	if (srmIn.elevation > MsgEnum::unknown_elevation)
	{
		if ((requestor.position->position.elevation = (Elevation_t *)calloc(1, sizeof(Elevation_t))) == NULL)
		{
			std::cerr << "encode_srm_payload: failed allocate " << allocate_level;
			std::cerr	<< ".RequestorPositionVector.Position3D.elevation" << std::endl;
			ASN_STRUCT_FREE(asn_DEF_SignalRequestMessage, pSRM);
			return(0);
		}
		*(requestor.position->position.elevation) = srmIn.elevation;
	}
	// heading
	if ((requestor.position->heading = (Angle_t *)calloc(1, sizeof(Angle_t))) == NULL)
	{
		std::cerr << "encode_srm_payload: failed allocate " << allocate_level;
		std::cerr	<< ".RequestorPositionVector.heading" << std::endl;
		ASN_STRUCT_FREE(asn_DEF_SignalRequestMessage, pSRM);
		return(0);
	}
	*(requestor.position->heading) = srmIn.heading;
	// speed
	if ((requestor.position->speed = (TransmissionAndSpeed_t *)calloc(1, sizeof(TransmissionAndSpeed_t))) == NULL)
	{
		std::cerr << "encode_srm_payload: failed allocate " << allocate_level;
		std::cerr << ".RequestorPositionVector.speed" << std::endl;
		ASN_STRUCT_FREE(asn_DEF_SignalRequestMessage, pSRM);
		return(0);
	}
	requestor.position->speed->transmisson = static_cast<TransmissionState_t>(srmIn.transState);
	requestor.position->speed->speed = srmIn.speed;
	// encode SRM
	asn_enc_rval_t rval = uper_encode_to_buffer(&asn_DEF_SignalRequestMessage, pSRM, buf, size);
	ASN_STRUCT_FREE(asn_DEF_SignalRequestMessage, pSRM);
	return(numbits2numbytes(rval.encoded));
}

size_t AsnJ2735Lib::decode_srm_payload(const uint8_t* buf, size_t size, SRM_element_t& srmOut)
{
	SignalRequestMessage_t* pSRM = NULL;
	asn_dec_rval_t rval = uper_decode(0, &asn_DEF_SignalRequestMessage, (void **)&pSRM, buf, size, 0, 0);
	if (rval.code != RC_OK)
	{
		std::cerr << "decode_srm_payload: failed UPER decoding" << std::endl;
		ASN_STRUCT_FREE(asn_DEF_SignalRequestMessage, pSRM);
		return(0);
	}
	if ((pSRM == NULL) || (pSRM->requests == NULL) || (pSRM->requests->list.count == 0))
	{
		std::cerr << "decode_srm_payload: missing SignalRequestList" << std::endl;
		ASN_STRUCT_FREE(asn_DEF_SignalRequestMessage, pSRM);
		return(0);
	}
	srmOut.reset();
	srmOut.timeStampSec = static_cast<uint16_t>(pSRM->second);
	if (pSRM->timeStamp != NULL)
		srmOut.timeStampMinute = static_cast<uint32_t>(*(pSRM->timeStamp));
	if (pSRM->sequenceNumber != NULL)
		srmOut.msgCnt = static_cast<uint8_t>(*(pSRM->sequenceNumber));
	const SignalRequestPackage_t* pSignalRequestPackage = pSRM->requests->list.array[0];
	if (pSignalRequestPackage->minute != NULL)
		srmOut.ETAminute = static_cast<uint32_t>(*(pSignalRequestPackage->minute));
	if (pSignalRequestPackage->second != NULL)
		srmOut.ETAsec = static_cast<uint16_t>(*(pSignalRequestPackage->second));
	if (pSignalRequestPackage->duration != NULL)
		srmOut.duration = static_cast<uint16_t>(*(pSignalRequestPackage->duration));
	const SignalRequest_t& signalRequest = pSignalRequestPackage->request;
	srmOut.intId = static_cast<uint16_t>(signalRequest.id.id);
	srmOut.reqId = static_cast<uint8_t>(signalRequest.requestID);
	srmOut.reqType = static_cast<MsgEnum::requestType>(signalRequest.requestType);
	if (signalRequest.inBoundLane.present == IntersectionAccessPoint_PR_approach)
		srmOut.inApprochId = static_cast<uint8_t>(signalRequest.inBoundLane.choice.approach);
	else if (signalRequest.inBoundLane.present == IntersectionAccessPoint_PR_lane)
		srmOut.inLaneId = static_cast<uint8_t>(signalRequest.inBoundLane.choice.lane);
	if (signalRequest.outBoundLane != NULL)
	{
		if (signalRequest.outBoundLane->present == IntersectionAccessPoint_PR_approach)
			srmOut.outApproachId = static_cast<uint8_t>(signalRequest.outBoundLane->choice.approach);
		else if (signalRequest.outBoundLane->present == IntersectionAccessPoint_PR_lane)
			srmOut.outLaneId = static_cast<uint8_t>(signalRequest.outBoundLane->choice.lane);
	}
	const RequestorDescription_t& requestor = pSRM->requestor;
	if (requestor.id.present != VehicleID_PR_entityID)
	{
		std::cerr << "decode_srm_payload: missing Temporary ID" << std::endl;
		ASN_STRUCT_FREE(asn_DEF_SignalRequestMessage, pSRM);
		return(0);
	}
	srmOut.vehId = static_cast<uint32_t>(octString2ul(requestor.id.choice.entityID.buf, requestor.id.choice.entityID.size));
	if (requestor.type == NULL)
	{
		std::cerr << "decode_srm_payload: missing RequestorType" << std::endl;
		ASN_STRUCT_FREE(asn_DEF_SignalRequestMessage, pSRM);
		return(0);
	}
	srmOut.vehRole = static_cast<MsgEnum::basicRole>(requestor.type->role);
	if (requestor.type->hpmsType == NULL)
	{
		std::cerr << "decode_srm_payload: missing VehicleType" << std::endl;
		ASN_STRUCT_FREE(asn_DEF_SignalRequestMessage, pSRM);
		return(0);
	}
	srmOut.vehType = static_cast<MsgEnum::vehicleType>(*(requestor.type->hpmsType));
	if (requestor.position == NULL)
	{
		std::cerr << "decode_srm_payload: missing RequestorPositionVector" << std::endl;
		ASN_STRUCT_FREE(asn_DEF_SignalRequestMessage, pSRM);
		return(0);
	}
	srmOut.latitude = static_cast<int32_t>(requestor.position->position.lat);
	srmOut.longitude = static_cast<int32_t>(requestor.position->position.Long);
	if (requestor.position->position.elevation != NULL)
		srmOut.elevation =static_cast<int32_t>(*(requestor.position->position.elevation));
	if (requestor.position->heading == NULL)
	{
		std::cerr << "decode_srm_payload: missing heading" << std::endl;
		ASN_STRUCT_FREE(asn_DEF_SignalRequestMessage, pSRM);
		return(0);
	}
	srmOut.heading = static_cast<uint16_t>(*(requestor.position->heading));
	if (requestor.position->speed == NULL)
	{
		std::cerr << "decode_srm_payload: missing TransmissionAndSpeed" << std::endl;
		ASN_STRUCT_FREE(asn_DEF_SignalRequestMessage, pSRM);
		return(0);
	}
	srmOut.transState = static_cast<MsgEnum::transGear>(requestor.position->speed->transmisson);
	srmOut.speed = static_cast<uint16_t>(requestor.position->speed->speed);
	ASN_STRUCT_FREE(asn_DEF_SignalRequestMessage, pSRM);
	return(numbits2numbytes(rval.consumed));
}

size_t AsnJ2735Lib::encode_ssm_payload(const SSM_element_t& ssmIn, uint8_t* buf, size_t size)
{
	if (ssmIn.mpSignalRequetStatus.empty())
		return(0);  // nothing to encode

	std::string allocate_level{"SSM"};
	SignalStatusMessage_t* pSSM = (SignalStatusMessage_t *)calloc(1, sizeof(SignalStatusMessage_t));
	if (pSSM == NULL)
	{
		std::cerr << "encode_ssm_payload: failed allocate " << allocate_level << std::endl;
		return(0);
	}
	// SSM:
	// -- Required objects ------------------------------------ //
	//	DSecond
	//	SignalStatusList
	// -- OPTIONAL objects ------------ including/excluding  -- //
	//	MinuteOfTheYear                     OPTIONAL
	//	MsgCount                            OPTIONAL
	//	RegionalExtension                   EXCL
	// -------------------------------------------------------- //

	// DSecond
	pSSM->second = ssmIn.timeStampSec;
	// SSM::MinuteOfTheYear
	if (ssmIn.timeStampMinute < MsgEnum::invalid_timeStampMinute)
	{
		if ((pSSM->timeStamp = (MinuteOfTheYear_t *)calloc(1, sizeof(MinuteOfTheYear_t))) == NULL)
		{
			std::cerr << "encode_ssm_payload: failed allocate " << allocate_level;
			std::cerr	<< ".MinuteOfTheYear" << std::endl;
			free(pSSM);
			return(0);
		}
		*(pSSM->timeStamp) = ssmIn.timeStampMinute;
	}
	// MsgCount
	if (ssmIn.msgCnt < 0xFF)
	{
		if ((pSSM->sequenceNumber = (MsgCount_t *)calloc(1, sizeof(MsgCount_t))) == NULL)
		{
			std::cerr << "encode_ssm_payload: failed allocate " << allocate_level;
			std::cerr	<< ".MsgCount" << std::endl;
			if (pSSM->timeStamp != NULL)
				free(pSSM->timeStamp);
			free(pSSM);
			return(0);
		}
		*(pSSM->sequenceNumber) = ssmIn.msgCnt;
	}
	// SignalStatusList - one intersection per SSM
	allocate_level += ".SignalStatusList";
	if ((pSSM->status.list.array = (SignalStatus_t **)calloc(1, sizeof(SignalStatus_t *))) == NULL)
	{
		std::cerr << "encode_ssm_payload: failed allocate " << allocate_level << std::endl;
		if (pSSM->sequenceNumber != NULL)
			free(pSSM->sequenceNumber);
		if (pSSM->timeStamp != NULL)
			free(pSSM->timeStamp);
		free(pSSM);
		return(0);
	}
	pSSM->status.list.size  = 1;
	allocate_level += ".SignalStatus";
	if ((pSSM->status.list.array[0] = (SignalStatus_t *)calloc(1, sizeof(SignalStatus_t))) == NULL)
	{
		std::cerr << "encode_ssm_payload: failed allocate " << allocate_level << std::endl;
		free(pSSM->status.list.array);
		if (pSSM->sequenceNumber != NULL)
			free(pSSM->sequenceNumber);
		if (pSSM->timeStamp != NULL)
			free(pSSM->timeStamp);
		free(pSSM);
		return(0);
	}
	pSSM->status.list.count = 1;
	SignalStatus_t* pSignalStatus = pSSM->status.list.array[0];
	// SignalStatus
	// -- Required objects ------------------------------------ //
	//	MsgCount
	//	IntersectionReferenceID
	//	SignalStatusPackageList
	// -- OPTIONAL objects ------------ including/excluding  -- //
	//	RegionalExtension                   EXCL
	// -------------------------------------------------------- //

	// MsgCount
	pSignalStatus->sequenceNumber = ssmIn.updateCnt;
	// IntersectionReferenceID
	pSignalStatus->id.id = ssmIn.id;
	// SignalStatusPackageList
	allocate_level += ".SignalStatusPackageList";
	if ((pSignalStatus->sigStatus.list.array = (SignalStatusPackage_t **)calloc(ssmIn.mpSignalRequetStatus.size(),
		sizeof(SignalStatusPackage_t *))) == NULL)
	{
		std::cerr << "encode_ssm_payload: failed allocate " << allocate_level << std::endl;
		free(pSSM->status.list.array[0]);
		free(pSSM->status.list.array);
		if (pSSM->sequenceNumber != NULL)
			free(pSSM->sequenceNumber);
		if (pSSM->timeStamp != NULL)
			free(pSSM->timeStamp);
		free(pSSM);
		return(0);
	}
	pSignalStatus->sigStatus.list.size = static_cast<int>(ssmIn.mpSignalRequetStatus.size());

	// loop through the list of priority/preemption requests
	allocate_level += ".SignalStatusPackage";
	bool has_error = false;  // for earlier return
	int& statusListCnt = pSignalStatus->sigStatus.list.count;
	for (const auto& signalRequetStatus : ssmIn.mpSignalRequetStatus)
	{
		if ((pSignalStatus->sigStatus.list.array[statusListCnt] =
			(SignalStatusPackage_t *)calloc(1, sizeof(SignalStatusPackage_t))) = NULL)
		{
			std::cerr << "encode_ssm_payload: failed allocate " << allocate_level << std::endl;
			has_error = true;
			break;
		}
		SignalStatusPackage_t* pSignalStatusPackage = pSignalStatus->sigStatus.list.array[statusListCnt++];
		// SignalStatusPackage
		// -- Required objects ------------------------------------ //
		//	inboundOn
		//	PrioritizationResponseStatus
		// -- OPTIONAL objects ------------ including/excluding  -- //
		//	SignalRequesterInfo                 INCL
		//	outboundOn                          OPTIONAL
		//  ETA.minute                          OPTIONAL
		//  ETA.second                          OPTIONAL
		//	duration                            OPTIONAL
		//	RegionalExtension                   EXCL
		// -------------------------------------------------------- //

		// PrioritizationResponseStatus
		pSignalStatusPackage->status = static_cast<PrioritizationResponseStatus_t>(signalRequetStatus.status);
		// inboundOn
		if ((signalRequetStatus.inApprochId == 0) && (signalRequetStatus.inLaneId == 0))
		{
			std::cerr << "encode_ssm_payload: either entry approach or lane needs to be specified" << std::endl;
			has_error = true;
			break;
		}
		if (signalRequetStatus.inLaneId == 0)
		{
			pSignalStatusPackage->inboundOn.present = IntersectionAccessPoint_PR_approach;
			pSignalStatusPackage->inboundOn.choice.approach = signalRequetStatus.inApprochId;
		}
		else
		{
			pSignalStatusPackage->inboundOn.present = IntersectionAccessPoint_PR_lane;
			pSignalStatusPackage->inboundOn.choice.lane = signalRequetStatus.inLaneId;
		}
		// outboundOn
		if (!((signalRequetStatus.outApproachId == 0) && (signalRequetStatus.outLaneId == 0)))
		{
			if ((pSignalStatusPackage->outboundOn =
				(IntersectionAccessPoint_t *)calloc(1, sizeof(IntersectionAccessPoint_t))) == NULL)
			{
				std::cerr << "encode_ssm_payload: failed allocate " << allocate_level;
				std::cerr << ".outboundOn" << std::endl;
				has_error = true;
				break;
			}
			if (signalRequetStatus.outLaneId == 0)
			{
				pSignalStatusPackage->outboundOn->present = IntersectionAccessPoint_PR_approach;
				pSignalStatusPackage->outboundOn->choice.approach = signalRequetStatus.outApproachId;
			}
			else
			{
				pSignalStatusPackage->outboundOn->present = IntersectionAccessPoint_PR_lane;
				pSignalStatusPackage->outboundOn->choice.lane = signalRequetStatus.outLaneId;
			}
		}
		// ETA
		if (signalRequetStatus.ETAminute < MsgEnum::invalid_timeStampMinute)
		{
			if ((pSignalStatusPackage->minute =	(MinuteOfTheYear_t *)calloc(1, sizeof(MinuteOfTheYear_t))) == NULL)
			{
				std::cerr << "encode_ssm_payload: failed allocate " << allocate_level;
				std::cerr	<< ".ETAminute" << std::endl;
				has_error = true;
				break;
			}
			*(pSignalStatusPackage->minute) = signalRequetStatus.ETAminute;
		}
		if (signalRequetStatus.ETAsec < 0xFFFF)
		{
			if ((pSignalStatusPackage->second = (DSecond_t *)calloc(1, sizeof(DSecond_t))) == NULL)
			{
				std::cerr << "encode_ssm_payload: failed allocate " << allocate_level;
				std::cerr	<< ".ETAsec" << std::endl;
				has_error = true;
				break;
			}
			*(pSignalStatusPackage->second) = signalRequetStatus.ETAsec;
		}
		// duration
		if (signalRequetStatus.duration < 0xFFFF)
		{
			if ((pSignalStatusPackage->duration = (DSecond_t *)calloc(1, sizeof(DSecond_t))) == NULL)
			{
				std::cerr << "encode_ssm_payload: failed allocate " << allocate_level;
				std::cerr	<< ".duration" << std::endl;
				has_error = true;
				break;
			}
			*(pSignalStatusPackage->duration) = signalRequetStatus.duration;
		}
		// SignalRequesterInfo
		if ((pSignalStatusPackage->requester = (SignalRequesterInfo_t *)calloc(1, sizeof(SignalRequesterInfo_t))) == NULL)
		{
			std::cerr << "encode_ssm_payload: failed allocate " << allocate_level;
			std::cerr	<< ".SignalRequesterInfo" << std::endl;
			has_error = true;
			break;
		}
		// SignalRequesterInfo
		// -- Required objects ------------------------------------ //
		//	VehicleID
		//	RequestID
		//	MsgCount
		// -- OPTIONAL objects ------------ including/excluding  -- //
		//	BasicVehicleRole                    OPTIONAL
		//	RequestorType                       EXCL
		// -------------------------------------------------------- //

		// VehicleID - TemporaryID
		pSignalStatusPackage->requester->id.present = VehicleID_PR_entityID;
		if (!vehId2temporaryId(&pSignalStatusPackage->requester->id.choice.entityID.buf,
			pSignalStatusPackage->requester->id.choice.entityID.size, signalRequetStatus.vehId))
		{
			std::cerr << "encode_ssm_payload: failed allocate " << allocate_level;
			std::cerr	<< ".SignalRequesterInfo.VehicleID" << std::endl;
			has_error = true;
			break;
		}
		// RequestID
		pSignalStatusPackage->requester->request = signalRequetStatus.reqId;
		// MsgCount
		pSignalStatusPackage->requester->sequenceNumber = signalRequetStatus.sequenceNumber;
		// BasicVehicleRole
		if (signalRequetStatus.vehRole != MsgEnum::basicRole::unavailable)
		{
			if ((pSignalStatusPackage->requester->role = (BasicVehicleRole_t *)calloc(1, sizeof(BasicVehicleRole_t))) == NULL)
			{
				std::cerr << "encode_ssm_payload: failed allocate " << allocate_level;
				std::cerr	<< ".SignalRequesterInfo.BasicVehicleRole" << std::endl;
				has_error = true;
				break;
			}
			*(pSignalStatusPackage->requester->role) = static_cast<BasicVehicleRole_t>(signalRequetStatus.vehRole);
		}
	}
	if (has_error)
	{
		ASN_STRUCT_FREE(asn_DEF_SignalStatusMessage, pSSM);
		return(0);
	}
	// encode SSM
	asn_enc_rval_t rval = uper_encode_to_buffer(&asn_DEF_SignalStatusMessage, pSSM, buf, size);
	ASN_STRUCT_FREE(asn_DEF_SignalStatusMessage, pSSM);
	return(numbits2numbytes(rval.encoded));
}

size_t AsnJ2735Lib::decode_ssm_payload(const uint8_t* buf, size_t size, SSM_element_t& ssmOut)
{
	SignalStatusMessage_t* pSSM	= NULL;
	asn_dec_rval_t rval = uper_decode(0, &asn_DEF_SignalStatusMessage,(void **)&pSSM, buf, size, 0, 0);
	if (rval.code != RC_OK)
	{
		std::cerr << "decode_ssm_payload: failed UPER decoding" << std::endl;
		ASN_STRUCT_FREE(asn_DEF_SignalStatusMessage, pSSM);
		return(0);
	}
	if ((pSSM == NULL) || (pSSM->status.list.count == 0))
	{
		std::cerr << "decode_ssm_payload: empty SignalStatusList" << std::endl;
		ASN_STRUCT_FREE(asn_DEF_SignalStatusMessage, pSSM);
		return(0);
	}
	ssmOut.reset();
	ssmOut.timeStampSec = static_cast<uint16_t>(pSSM->second);
	if (pSSM->timeStamp != NULL)
		ssmOut.timeStampMinute = static_cast<uint32_t>(*(pSSM->timeStamp));
	if (pSSM->sequenceNumber != NULL)
		ssmOut.msgCnt = static_cast<uint8_t>(*(pSSM->sequenceNumber));
	const SignalStatus_t* pSignalStatus = pSSM->status.list.array[0];
	ssmOut.updateCnt = static_cast<uint8_t>(pSignalStatus->sequenceNumber);
	ssmOut.id = static_cast<uint16_t>(pSignalStatus->id.id);
	if (pSignalStatus->sigStatus.list.count == 0)
	{
		std::cerr << "decode_ssm_payload: empty SignalStatusPackageList" << std::endl;
		ASN_STRUCT_FREE(asn_DEF_SignalStatusMessage, pSSM);
		return(0);
	}
	ssmOut.mpSignalRequetStatus.resize(pSignalStatus->sigStatus.list.count);
	bool has_error = false;
	for (int i = 0; i < pSignalStatus->sigStatus.list.count; i++)
	{
		const SignalStatusPackage_t* pSignalStatusPackage = pSignalStatus->sigStatus.list.array[i];
		SignalRequetStatus_t& signalRequetStatus = ssmOut.mpSignalRequetStatus[i];
		signalRequetStatus.reset();
		signalRequetStatus.status = static_cast<MsgEnum::requestStatus>(pSignalStatusPackage->status);
		if (pSignalStatusPackage->inboundOn.present == IntersectionAccessPoint_PR_approach)
			signalRequetStatus.inApprochId = static_cast<uint8_t>(pSignalStatusPackage->inboundOn.choice.approach);
		else if (pSignalStatusPackage->inboundOn.present == IntersectionAccessPoint_PR_lane)
			signalRequetStatus.inLaneId = static_cast<uint8_t>(pSignalStatusPackage->inboundOn.choice.lane);
		if (pSignalStatusPackage->outboundOn != NULL)
		{
			if (pSignalStatusPackage->outboundOn->present == IntersectionAccessPoint_PR_approach)
				signalRequetStatus.outApproachId = static_cast<uint8_t>(pSignalStatusPackage->outboundOn->choice.approach);
			else if (pSignalStatusPackage->outboundOn->present == IntersectionAccessPoint_PR_lane)
				signalRequetStatus.outLaneId = static_cast<uint8_t>(pSignalStatusPackage->outboundOn->choice.lane);
		}
		if (pSignalStatusPackage->minute != NULL)
			signalRequetStatus.ETAminute = static_cast<uint32_t>(*(pSignalStatusPackage->minute));
		if (pSignalStatusPackage->second != NULL)
			signalRequetStatus.ETAsec = static_cast<uint16_t>(*(pSignalStatusPackage->second));
		if (pSignalStatusPackage->duration != NULL)
			signalRequetStatus.duration = static_cast<uint16_t>(*(pSignalStatusPackage->duration));
		if (pSignalStatusPackage->requester == NULL)
		{
			std::cerr << "decode_ssm_payload: missing SignalRequesterInfo" << std::endl;
			has_error = true;
			break;
		}
		if (pSignalStatusPackage->requester->id.present != VehicleID_PR_entityID)
		{
			std::cerr << "decode_ssm_payload: missing Temporary ID" << std::endl;
			has_error = true;
			break;
		}
		signalRequetStatus.vehId = static_cast<uint32_t>(octString2ul(pSignalStatusPackage->requester->id.choice.entityID.buf,
			pSignalStatusPackage->requester->id.choice.entityID.size));
		signalRequetStatus.reqId = static_cast<uint8_t>(pSignalStatusPackage->requester->request);
		signalRequetStatus.sequenceNumber = static_cast<uint8_t>(pSignalStatusPackage->requester->sequenceNumber);
		if (pSignalStatusPackage->requester->role == NULL)
		{
			std::cerr << "decode_ssm_payload: missing BasicVehicleRole" << std::endl;
			has_error = true;
			break;
		}
		signalRequetStatus.vehRole = static_cast<MsgEnum::basicRole>(*(pSignalStatusPackage->requester->role));
	}
	ASN_STRUCT_FREE(asn_DEF_SignalStatusMessage, pSSM);
	return((has_error) ? (0) : (numbits2numbytes(rval.consumed)));
}

size_t AsnJ2735Lib::encode_bsm_payload(const BSM_element_t& bsmIn, uint8_t* buf, size_t size)
{
	std::string allocate_level{"BSM"};
	BasicSafetyMessage_t* pBSM = (BasicSafetyMessage_t *)calloc(1, sizeof(BasicSafetyMessage_t));
	if (pBSM == NULL)
	{
		std::cerr << "encode_bsm_payload: failed allocate " << allocate_level << std::endl;
		return(0);
	}
	// BSM:
	// -- Required objects ------------------------------------ //
	//	BSMcoreData
	// -- OPTIONAL objects ------------ including/excluding  -- //
	//	partII                              EXCL
	//	RegionalExtension                   EXCL
	// -------------------------------------------------------- //
	allocate_level += ".BSMcoreData";
	auto& coreData = pBSM->coreData;
	coreData.msgCnt = bsmIn.msgCnt;
	// BSMcoreData::TemporaryID
	if (!vehId2temporaryId(&coreData.id.buf, coreData.id.size, bsmIn.id))
	{
		std::cerr << "encode_bsm_payload: failed allocate " << allocate_level;
		std::cerr << ".TemporaryID" << std::endl;
		free(pBSM);
		return(0);
	}
	coreData.secMark = bsmIn.timeStampSec;
	coreData.lat = bsmIn.latitude;
	coreData.Long = bsmIn.longitude;
	coreData.elev = bsmIn.elevation;
	coreData.accuracy.semiMajor = bsmIn.semiMajor;
	coreData.accuracy.semiMinor = bsmIn.semiMinor;
	coreData.accuracy.orientation = bsmIn.orientation;
	coreData.transmission = static_cast<TransmissionState_t>(bsmIn.transState);
	coreData.speed = bsmIn.speed;
	coreData.heading = bsmIn.heading;
	coreData.angle = bsmIn.steeringAngle;
	coreData.accelSet.Long = bsmIn.accelLon;
	coreData.accelSet.lat = bsmIn.accelLat;
	coreData.accelSet.vert = bsmIn.accelVert;
	coreData.accelSet.yaw = bsmIn.yawRate;
	coreData.size.width = bsmIn.vehWidth;
	coreData.size.length = bsmIn.vehLen;
	allocate_level += ".BrakeSystemStatus";
	auto& brakeSystemStatus = coreData.brakes;
	auto& wheelBrakes = brakeSystemStatus.wheelBrakes;  // SIZE (5) BIT STRING
	if (!ul2bitString(&wheelBrakes.buf, wheelBrakes.size, wheelBrakes.bits_unused, 5, bsmIn.brakeAppliedStatus.to_ulong()))
	{
		std::cerr << "encode_bsm_payload: failed allocate " << allocate_level;
		std::cerr	<< ".BrakeAppliedStatus" << std::endl;
		ASN_STRUCT_FREE(asn_DEF_BasicSafetyMessage, pBSM);
		return(0);
	}
	brakeSystemStatus.traction = static_cast<TractionControlStatus_t>(bsmIn.tractionControlStatus);
	brakeSystemStatus.abs = static_cast<AntiLockBrakeStatus_t>(bsmIn.absStatus);
	brakeSystemStatus.scs = static_cast<StabilityControlStatus_t>(bsmIn.stabilityControlStatus);
	brakeSystemStatus.brakeBoost = static_cast<BrakeBoostApplied_t>(bsmIn.brakeBoostApplied);
	brakeSystemStatus.auxBrakes = static_cast<AuxiliaryBrakeStatus_t>(bsmIn.auxiliaryBrakeStatus);
	// encode BSM
	asn_enc_rval_t rval = uper_encode_to_buffer(&asn_DEF_BasicSafetyMessage, pBSM, buf, size);
	ASN_STRUCT_FREE(asn_DEF_BasicSafetyMessage, pBSM);
	return(numbits2numbytes(rval.encoded));
}

size_t AsnJ2735Lib::decode_bsm_payload(const uint8_t* buf, size_t size, BSM_element_t& bsmOut)
{
	BasicSafetyMessage_t* pBSM = NULL;
	asn_dec_rval_t rval = uper_decode(0, &asn_DEF_BasicSafetyMessage,(void **)&pBSM, buf, size, 0, 0);
	if (rval.code != RC_OK)
	{
		std::cerr << "decode_bsm_payload: failed UPER decoding" << std::endl;
		ASN_STRUCT_FREE(asn_DEF_BasicSafetyMessage, pBSM);
		return(0);
	}
	const auto& coreData = pBSM->coreData;
	bsmOut.msgCnt = static_cast<uint8_t>(coreData.msgCnt);
	bsmOut.id = static_cast<uint32_t>(octString2ul(coreData.id.buf, coreData.id.size));
	bsmOut.timeStampSec = static_cast<uint16_t>(coreData.secMark);
	bsmOut.latitude = static_cast<int32_t>(coreData.lat);
	bsmOut.longitude = static_cast<int32_t>(coreData.Long);
	bsmOut.elevation = static_cast<int32_t>(coreData.elev);
	bsmOut.semiMajor = static_cast<uint8_t>(coreData.accuracy.semiMajor);
	bsmOut.semiMinor = static_cast<uint8_t>(coreData.accuracy.semiMinor);
	bsmOut.orientation = static_cast<uint16_t>(coreData.accuracy.orientation);
	bsmOut.speed = static_cast<uint16_t>(coreData.speed);
	bsmOut.heading = static_cast<uint16_t>(coreData.heading);
	bsmOut.steeringAngle = static_cast<int8_t>(coreData.angle);
	bsmOut.accelLon = static_cast<int16_t>(coreData.accelSet.Long);
	bsmOut.accelLat = static_cast<int16_t>(coreData.accelSet.lat);
	bsmOut.accelVert = static_cast<int8_t>(coreData.accelSet.vert);
	bsmOut.yawRate = static_cast<int16_t>(coreData.accelSet.yaw);
	bsmOut.vehWidth = static_cast<uint16_t>(coreData.size.width);
	bsmOut.vehLen = static_cast<uint16_t>(coreData.size.length);
	bsmOut.transState	= static_cast<MsgEnum::transGear>(coreData.transmission);
	const auto& brakeSystemStatus = coreData.brakes;
	const auto& wheelBrakes = brakeSystemStatus.wheelBrakes;
	bsmOut.brakeAppliedStatus = std::bitset<5>(bitString2ul(wheelBrakes.buf, wheelBrakes.size, wheelBrakes.bits_unused));
	bsmOut.tractionControlStatus = static_cast<MsgEnum::engageStatus>(brakeSystemStatus.traction);
	bsmOut.absStatus = static_cast<MsgEnum::engageStatus>(brakeSystemStatus.abs);
	bsmOut.stabilityControlStatus = static_cast<MsgEnum::engageStatus>(brakeSystemStatus.scs);
	bsmOut.brakeBoostApplied = static_cast<MsgEnum::engageStatus>(brakeSystemStatus.brakeBoost);
	bsmOut.auxiliaryBrakeStatus = static_cast<MsgEnum::engageStatus>(brakeSystemStatus.auxBrakes);
	ASN_STRUCT_FREE(asn_DEF_BasicSafetyMessage, pBSM);
	return(numbits2numbytes(rval.consumed));
}
