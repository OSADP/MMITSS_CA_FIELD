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

#ifndef	_RoadSignID_H_
#define	_RoadSignID_H_


#include <asn_application.h>

/* Including external dependencies */
#include "Position3D.h"
#include "HeadingSlice.h"
#include "MUTCDCode.h"
#include "MsgCRC.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* RoadSignID */
typedef struct RoadSignID {
	Position3D_t	 position;
	HeadingSlice_t	 viewAngle;
	MUTCDCode_t	*mutcdCode	/* OPTIONAL */;
	MsgCRC_t	*crc	/* OPTIONAL */;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} RoadSignID_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_RoadSignID;

#ifdef __cplusplus
}
#endif

#endif	/* _RoadSignID_H_ */
#include <asn_internal.h>
