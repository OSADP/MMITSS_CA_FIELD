//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************
#ifndef _GEO_UTILS_H
#define _GEO_UTILS_H

#include <cstdlib>
#include <cmath>
#include <vector>
#include <stdint.h>   // c++11 <cstdint>

#include "msgenum.h"

namespace GeoUtils
{ 
  struct polygon_enum_t
  {
    enum polygonType { COLINEAR, CONCAVE, CONVEX };
  };
  
  struct vehicleInMap_enum_t
  {
    enum locationType {NOT_IN_MAP, INSIDE_INTERSECTION_BOX, ON_APPROACH, AT_INTERSECTION_BOX, ON_EGRESS};
    enum LaneLocationType {UNKNOWN, APPROACHING, INSIDE, LEAVING};
  };

  struct geoPoint_t
  {
    double latitude;    // in degree
    double longitude;   // in degree
    double elevation;   // in meters
    geoPoint_t& operator=(const GeoUtils::geoPoint_t& p)
    {
      latitude = p.latitude;
      longitude = p.longitude;
      elevation = p.elevation;
      return *this;
    };
    void reset(void)
    {
      latitude = 0.0;
      longitude = 0.0;
      elevation = 0.0;
    };
  };

  struct geoRefPoint_t
  {
    int32_t latitude;   // in 1/10th microdegree
    int32_t longitude;  // in 1/10th microdegree
    int32_t elevation;  // in centimetres
    bool operator==(const GeoUtils::geoRefPoint_t& p) const
    {
      // node elevation is the same as intersection ref elevation
      return ((latitude == p.latitude) && (longitude == p.longitude));
    };
  };
  
  struct point3D_t
  {
    double x; // in meters
    double y; // in meters
    double z; // in meters
  };
  
  struct transMatrix_t
  {
    double dSinLat;
    double dCosLat;
    double dSinLong;
    double dCosLong;
  };
  
  struct enuCoord_t
  {
    GeoUtils::point3D_t     pointECEF;        // ENU origin reference in ECEF
    GeoUtils::transMatrix_t transMatrix;
  };
  
  struct point2D_t
  {
    int32_t x;  // in centimetre
    int32_t y;  // in centimetre  
    bool operator==(const GeoUtils::point2D_t& p) const
    {
      return ((x == p.x) && (y == p.y));
    };
    bool operator<(const GeoUtils::point2D_t& p) const 
    {
      return (x < p.x || (x == p.x && y < p.y));
    };
    point2D_t& operator=(const GeoUtils::point2D_t& p)
    {
      x = p.x;
      y = p.y;
      return *this;
    };
    uint32_t length() const
    {
      return (static_cast<uint32_t>(sqrt(x  * x + y * y)));
    };  
    uint32_t distance2pt(const GeoUtils::point2D_t& p) const
    {
      return (static_cast<uint32_t>(sqrt((x - p.x) * (x - p.x) + (y - p.y) * (y - p.y))));
    };  
    uint16_t direction2pt(const GeoUtils::point2D_t& p) const;
  };
  
  struct vector2D_t
  {
    int32_t X;
    int32_t Y;
    bool operator==(const GeoUtils::vector2D_t& P) const
    {
      return ((X == P.X) && (Y == P.Y));
    };
    vector2D_t& operator=(const GeoUtils::vector2D_t& P)
    {
      X = P.X;
      Y = P.Y;
      return *this;
    };
    void set(const GeoUtils::point2D_t& startPoint, const GeoUtils::point2D_t& endPoint)
    {
      X = endPoint.x - startPoint.x;
      Y = endPoint.y - startPoint.y;
    };
  };
  
  struct motion_t
  {
    double speed;   // in m/s
    double heading; // in degree, 0..360
    motion_t& operator=(const GeoUtils::motion_t& p)
    {
      speed = p.speed;
      heading = p.heading;
      return *this;
    };        
    void reset(void)
    {
      speed = 0.0;
      heading = 0.0;
    };
  };
    
  struct projection_t
  {
    double t;       // along the line (unit-less)
    double d;       // distance away from the line, positive means on the right-side of travel 
    double length;  // length of the line segment
    projection_t& operator=(const GeoUtils::projection_t& p)
    {
      t = p.t;
      d = p.d;
      length = p.length;
      return *this;
    };    
    void reset(void)
    {
      t = 0.0;
      d = 0.0;
      length = 0.0;
    }
  };
  
  struct laneProjection_t
  {
    uint8_t nodeIndex;
    projection_t proj2segment;
    laneProjection_t& operator=(const GeoUtils::laneProjection_t& p)
    {
      nodeIndex = p.nodeIndex;
      proj2segment = p.proj2segment;
      return *this;
    };    
    void reset(void)
    {
      nodeIndex = 0;
      proj2segment.reset();
    }
  };
    
  struct intersectionTracking_t
  {
    GeoUtils::vehicleInMap_enum_t::locationType vehicleIntersectionStatus;
    uint8_t   intersectionIndex;  // meaningful when vehicleIntersectionStatus != NOT_IN_MAP
    uint8_t   approachIndex;      // meaningful when vehicleIntersectionStatus != NOT_IN_MAP & INSIDE_INTERSECTION_BOX
    uint8_t   laneIndex;          // meaningful when vehicleIntersectionStatus != NOT_IN_MAP & INSIDE_INTERSECTION_BOX
    intersectionTracking_t& operator=(const GeoUtils::intersectionTracking_t& p)
    {
      vehicleIntersectionStatus = p.vehicleIntersectionStatus;
      intersectionIndex = p.intersectionIndex;
      approachIndex = p.approachIndex;
      laneIndex = p.laneIndex;
      return *this;
    };    
    bool operator==(const GeoUtils::intersectionTracking_t& p) const
    {
      return ((vehicleIntersectionStatus == p.vehicleIntersectionStatus) 
        && (intersectionIndex == p.intersectionIndex)
        && (approachIndex == p.approachIndex)
        && (laneIndex == p.laneIndex));
    };    
    void reset(void)
    {
      vehicleIntersectionStatus = GeoUtils::vehicleInMap_enum_t::NOT_IN_MAP;
      intersectionIndex = 0;
      approachIndex = 0;
      laneIndex = 0;
    };
  };
  
  struct laneTracking_t
  {
    GeoUtils::vehicleInMap_enum_t::LaneLocationType vehicleLaneStatus;
    laneProjection_t laneProj;
  };
  
  struct vehicleTracking_t
  {
    intersectionTracking_t      intsectionTrackingState;
    GeoUtils::laneProjection_t  laneProj;
    vehicleTracking_t& operator=(const GeoUtils::vehicleTracking_t& p)
    {
      intsectionTrackingState = p.intsectionTrackingState;
      laneProj = p.laneProj;
      return *this;
    };        
    void reset(void)
    {
      intsectionTrackingState.reset();
      laneProj.reset();
    };
  };
  
  struct dist2go_t
  {
    double distLong;    // in meter, dist2stopbar when approaching (positive) or inside intersection box (negative); 
                        // dist2entrance when leaving the intersection (positive)
    double distLat;     // in meter, distance away from lane center, positive/negative value means on the right/left side of travel
    dist2go_t& operator=(const GeoUtils::dist2go_t& p)
    {
      distLong = p.distLong;
      distLat = p.distLat;
      return *this;
    };                
    void reset(void)
    {
      distLong = 0.0;
      distLat = 0.0;
    };
  };
  
  struct connectTo_t
  {
    uint32_t  intersectionId;
    uint8_t   laneId;
    MsgEnum::map_enum_t::maneuverType laneManeuver; 
    connectTo_t& operator=(const GeoUtils::connectTo_t& p)
    {
      intersectionId = p.intersectionId;
      laneId = p.laneId;
      laneManeuver = p.laneManeuver;
      return *this;
    };                
    void reset(void)
    {
      intersectionId = 0;
      laneId = 0;
      laneManeuver = MsgEnum::map_enum_t::UNKNOWN;
    }
  };
  
  struct locationAware_t
  {
    uint32_t    intersectionId;
    uint8_t     laneId;
    uint8_t     controlPhase;
    GeoUtils::dist2go_t   dist2go;
    std::vector<GeoUtils::connectTo_t> connect2go;
    locationAware_t& operator=(const GeoUtils::locationAware_t& p)
    {
      intersectionId = p.intersectionId;
      laneId = p.laneId;
      controlPhase = p.controlPhase;
      dist2go = p.dist2go;
      connect2go = p.connect2go;
      return *this;
    };            
    void reset(void)
    {
      intersectionId = 0;
      laneId = 0;
      controlPhase = 0;
      dist2go.reset();
      connect2go.clear();
    }; 
  };
  
  struct signalAware_t
  {
    MsgEnum::phasestate_enum_t::vehicular currState;
    uint16_t  timeToChange;         // In units of deciseconds
    MsgEnum::phasestate_enum_t::confidentce stateConfidence;
    MsgEnum::phasestate_enum_t::vehicular nextState;
    uint16_t  clearanceIntv;        // yellow interval, in deciseconds
    MsgEnum::phasestate_enum_t::confidentce yellStateConfidence;
    signalAware_t& operator=(const GeoUtils::signalAware_t& p)
    {
      currState = p.currState;
      timeToChange = p.timeToChange;
      stateConfidence = p.stateConfidence;
      nextState = p.nextState;
      clearanceIntv = p.clearanceIntv;
      yellStateConfidence = p.yellStateConfidence;
      return *this;
    };                
    void reset(void)
    {
      currState = MsgEnum::phasestate_enum_t::UNKNOWN;
      timeToChange = 0;
      stateConfidence = MsgEnum::phasestate_enum_t::UNKNOWN_ESTIMATE;
      nextState = MsgEnum::phasestate_enum_t::UNKNOWN;
      clearanceIntv = 0;
      yellStateConfidence = MsgEnum::phasestate_enum_t::UNKNOWN_ESTIMATE;
    };
  };
    
  struct connectedVehicle_t
  {
    uint32_t  id;
    long long ts;     // milliseconds since epoch begin (00:00:00 Jan 1, 1970 UTC)
    GeoUtils::geoPoint_t  geoPoint;
    GeoUtils::motion_t    motionState;
    bool      isVehicleInMap;
    GeoUtils::vehicleTracking_t vehicleTrackingState;
    GeoUtils::locationAware_t   vehicleLocationAware;
    GeoUtils::signalAware_t     vehicleSignalAware;
    void reset(void)
    {
      id = 0;
      ts = 0;
      geoPoint.reset();
      motionState.reset();
      isVehicleInMap = false;
      vehicleTrackingState.reset();
      vehicleLocationAware.reset();
      vehicleSignalAware.reset();
    };
  };

  // methods
  void geoPoint2geoRefPoint(const GeoUtils::geoPoint_t& geoPoint, GeoUtils::geoRefPoint_t& geoRef);
  void geoRefPoint2geoPoint(const GeoUtils::geoRefPoint_t & geoRef, GeoUtils::geoPoint_t& geoPoint);
  void setEnuCoord(const GeoUtils::geoPoint_t& geoPoint, GeoUtils::enuCoord_t& enuCoord);
  void lla2ecef(const GeoUtils::geoPoint_t& geoPoint, GeoUtils::point3D_t& ptECEF);
  void ecef2enu(const GeoUtils::enuCoord_t& enuCoord, const GeoUtils::point3D_t& ptECEF, GeoUtils::point3D_t& ptENU);
  void lla2enu(const GeoUtils::enuCoord_t& enuCoord, const GeoUtils::geoPoint_t& geoPoint, GeoUtils::point3D_t& ptENU);
  void lla2enu(const GeoUtils::enuCoord_t& enuCoord, const GeoUtils::geoPoint_t& geoPoint, GeoUtils::point2D_t& ptENU);
  void lla2enu(const GeoUtils::enuCoord_t& enuCoord, const GeoUtils::geoRefPoint_t& geoRef, GeoUtils::point2D_t& ptENU);
  void enu2ecef(const GeoUtils::enuCoord_t& enuCoord, const GeoUtils::point3D_t& ptENU, GeoUtils::point3D_t& ptECEF);
  void enu2ecef(const GeoUtils::enuCoord_t& enuCoord, const GeoUtils::point2D_t& ptENU, GeoUtils::point3D_t& ptECEF);
  void ecef2lla(const GeoUtils::point3D_t& ptECEF, GeoUtils::geoPoint_t& geoPoint);
  void enu2lla(const GeoUtils::enuCoord_t& enuCoord, const GeoUtils::point3D_t& ptENU, GeoUtils::geoPoint_t& geoPoint);
  void enu2lla(const GeoUtils::enuCoord_t& enuCoord, const GeoUtils::point2D_t& ptENU, GeoUtils::geoPoint_t& geoPoint);
  double distlla2lla(const GeoUtils::geoPoint_t& p1,const GeoUtils::geoPoint_t& p2);
  long dotProduct(const GeoUtils::vector2D_t& v1, const GeoUtils::vector2D_t& v2);
  long crossProduct(const GeoUtils::vector2D_t& v1, const GeoUtils::vector2D_t& v2);
  long cross(const GeoUtils::point2D_t& O, const GeoUtils::point2D_t& A, const GeoUtils::point2D_t& B);
  void projectPt2Line(const GeoUtils::point2D_t& startPoint, const GeoUtils::point2D_t& endPoint, const GeoUtils::point2D_t& pt, GeoUtils::projection_t& proj2line);
  GeoUtils::polygon_enum_t::polygonType convexcave(const std::vector<point2D_t>& p, const size_t size);
  bool isPointInsidePolygon(const std::vector<point2D_t>& pPolygon, const size_t size, const GeoUtils::point2D_t& waypoint);  
  int isLeft(const GeoUtils::point2D_t& p0,const GeoUtils::point2D_t& p1,const GeoUtils::point2D_t& p2);
  std::vector<GeoUtils::point2D_t> convexHull2D(const std::vector<GeoUtils::point2D_t>& P,const size_t size);
  std::vector<GeoUtils::point2D_t> convexHullAndrew(std::vector<GeoUtils::point2D_t> P);  
};

#endif
