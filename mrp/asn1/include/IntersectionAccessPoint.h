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

#ifndef	_IntersectionAccessPoint_H_
#define	_IntersectionAccessPoint_H_


#include <asn_application.h>

/* Including external dependencies */
#include "LaneID.h"
#include "ApproachID.h"
#include "LaneConnectionID.h"
#include <constr_CHOICE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum IntersectionAccessPoint_PR {
	IntersectionAccessPoint_PR_NOTHING,	/* No components present */
	IntersectionAccessPoint_PR_lane,
	IntersectionAccessPoint_PR_approach,
	IntersectionAccessPoint_PR_connection,
	/* Extensions may appear below */
	
} IntersectionAccessPoint_PR;

/* IntersectionAccessPoint */
typedef struct IntersectionAccessPoint {
	IntersectionAccessPoint_PR present;
	union IntersectionAccessPoint_u {
		LaneID_t	 lane;
		ApproachID_t	 approach;
		LaneConnectionID_t	 connection;
		/*
		 * This type is extensible,
		 * possible extensions are below.
		 */
	} choice;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} IntersectionAccessPoint_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_IntersectionAccessPoint;

#ifdef __cplusplus
}
#endif

#endif	/* _IntersectionAccessPoint_H_ */
#include <asn_internal.h>
