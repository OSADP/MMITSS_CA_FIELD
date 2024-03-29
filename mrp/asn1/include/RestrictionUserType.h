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

#ifndef	_RestrictionUserType_H_
#define	_RestrictionUserType_H_


#include <asn_application.h>

/* Including external dependencies */
#include "RestrictionAppliesTo.h"
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>
#include <constr_CHOICE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum RestrictionUserType_PR {
	RestrictionUserType_PR_NOTHING,	/* No components present */
	RestrictionUserType_PR_basicType,
	RestrictionUserType_PR_regional,
	/* Extensions may appear below */
	
} RestrictionUserType_PR;

/* Forward declarations */
struct RegionalExtension;

/* RestrictionUserType */
typedef struct RestrictionUserType {
	RestrictionUserType_PR present;
	union RestrictionUserType_u {
		RestrictionAppliesTo_t	 basicType;
		struct RestrictionUserType__regional {
			A_SEQUENCE_OF(struct RegionalExtension) list;
			
			/* Context for parsing across buffer boundaries */
			asn_struct_ctx_t _asn_ctx;
		} regional;
		/*
		 * This type is extensible,
		 * possible extensions are below.
		 */
	} choice;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} RestrictionUserType_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_RestrictionUserType;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "RegionalExtension.h"

#endif	/* _RestrictionUserType_H_ */
#include <asn_internal.h>
