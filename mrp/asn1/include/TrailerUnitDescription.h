//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************
/*
 * Generated by asn1c-0.9.28 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "../j2735_asn/J2735_201601_ASN_mmitss.asn"
 * 	`asn1c -fcompound-names -gen-PER`
 */

#ifndef	_TrailerUnitDescription_H_
#define	_TrailerUnitDescription_H_


#include <asn_application.h>

/* Including external dependencies */
#include "IsDolly.h"
#include "VehicleWidth.h"
#include "VehicleLength.h"
#include "VehicleHeight.h"
#include "TrailerMass.h"
#include "PivotPointDescription.h"
#include "Offset-B12.h"
#include "Node-XY-24b.h"
#include "VertOffset-B07.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct BumperHeights;
struct PivotPointDescription;
struct TrailerHistoryPointList;

/* TrailerUnitDescription */
typedef struct TrailerUnitDescription {
	IsDolly_t	 isDolly;
	VehicleWidth_t	 width;
	VehicleLength_t	 length;
	VehicleHeight_t	*height	/* OPTIONAL */;
	TrailerMass_t	*mass	/* OPTIONAL */;
	struct BumperHeights	*bumperHeights	/* OPTIONAL */;
	VehicleHeight_t	*centerOfGravity	/* OPTIONAL */;
	PivotPointDescription_t	 frontPivot;
	struct PivotPointDescription	*rearPivot	/* OPTIONAL */;
	Offset_B12_t	*rearWheelOffset	/* OPTIONAL */;
	Node_XY_24b_t	 positionOffset;
	VertOffset_B07_t	*elevationOffset	/* OPTIONAL */;
	struct TrailerHistoryPointList	*crumbData	/* OPTIONAL */;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} TrailerUnitDescription_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_TrailerUnitDescription;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "BumperHeights.h"
#include "PivotPointDescription.h"
#include "TrailerHistoryPointList.h"

#endif	/* _TrailerUnitDescription_H_ */
#include <asn_internal.h>