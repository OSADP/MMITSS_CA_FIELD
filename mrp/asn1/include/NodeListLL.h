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

#ifndef	_NodeListLL_H_
#define	_NodeListLL_H_


#include <asn_application.h>

/* Including external dependencies */
#include "NodeSetLL.h"
#include <constr_CHOICE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum NodeListLL_PR {
	NodeListLL_PR_NOTHING,	/* No components present */
	NodeListLL_PR_nodes,
	/* Extensions may appear below */
	
} NodeListLL_PR;

/* NodeListLL */
typedef struct NodeListLL {
	NodeListLL_PR present;
	union NodeListLL_u {
		NodeSetLL_t	 nodes;
		/*
		 * This type is extensible,
		 * possible extensions are below.
		 */
	} choice;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} NodeListLL_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_NodeListLL;

#ifdef __cplusplus
}
#endif

#endif	/* _NodeListLL_H_ */
#include <asn_internal.h>
