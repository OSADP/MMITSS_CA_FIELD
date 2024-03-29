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

#ifndef	_Node_XY_26b_H_
#define	_Node_XY_26b_H_


#include <asn_application.h>

/* Including external dependencies */
#include "Offset-B13.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Node-XY-26b */
typedef struct Node_XY_26b {
	Offset_B13_t	 x;
	Offset_B13_t	 y;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} Node_XY_26b_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_Node_XY_26b;

#ifdef __cplusplus
}
#endif

#endif	/* _Node_XY_26b_H_ */
#include <asn_internal.h>
