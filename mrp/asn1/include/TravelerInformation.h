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

#ifndef	_TravelerInformation_H_
#define	_TravelerInformation_H_


#include <asn_application.h>

/* Including external dependencies */
#include "MsgCount.h"
#include "MinuteOfTheYear.h"
#include "UniqueMSGID.h"
#include "URL-Base.h"
#include "TravelerDataFrameList.h"
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct RegionalExtension;

/* TravelerInformation */
typedef struct TravelerInformation {
	MsgCount_t	 msgCnt;
	MinuteOfTheYear_t	*timeStamp	/* OPTIONAL */;
	UniqueMSGID_t	*packetID	/* OPTIONAL */;
	URL_Base_t	*urlB	/* OPTIONAL */;
	TravelerDataFrameList_t	 dataFrames;
	struct TravelerInformation__regional {
		A_SEQUENCE_OF(struct RegionalExtension) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *regional;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} TravelerInformation_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_TravelerInformation;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "RegionalExtension.h"

#endif	/* _TravelerInformation_H_ */
#include <asn_internal.h>
