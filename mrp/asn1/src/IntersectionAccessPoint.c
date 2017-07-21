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

#include "IntersectionAccessPoint.h"

static asn_per_constraints_t asn_PER_type_IntersectionAccessPoint_constr_1 GCC_NOTUSED = {
	{ APC_CONSTRAINED | APC_EXTENSIBLE,  2,  2,  0l,  2l }	/* (0..2,...) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_TYPE_member_t asn_MBR_IntersectionAccessPoint_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct IntersectionAccessPoint, choice.lane),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_LaneID,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"lane"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct IntersectionAccessPoint, choice.approach),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_ApproachID,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"approach"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct IntersectionAccessPoint, choice.connection),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_LaneConnectionID,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"connection"
		},
};
static const asn_TYPE_tag2member_t asn_MAP_IntersectionAccessPoint_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* lane */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* approach */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 } /* connection */
};
static asn_CHOICE_specifics_t asn_SPC_IntersectionAccessPoint_specs_1 = {
	sizeof(struct IntersectionAccessPoint),
	offsetof(struct IntersectionAccessPoint, _asn_ctx),
	offsetof(struct IntersectionAccessPoint, present),
	sizeof(((struct IntersectionAccessPoint *)0)->present),
	asn_MAP_IntersectionAccessPoint_tag2el_1,
	3,	/* Count of tags in the map */
	0,
	3	/* Extensions start */
};
asn_TYPE_descriptor_t asn_DEF_IntersectionAccessPoint = {
	"IntersectionAccessPoint",
	"IntersectionAccessPoint",
	CHOICE_free,
	CHOICE_print,
	CHOICE_constraint,
	CHOICE_decode_ber,
	CHOICE_encode_der,
	CHOICE_decode_xer,
	CHOICE_encode_xer,
	CHOICE_decode_uper,
	CHOICE_encode_uper,
	CHOICE_decode_aper,
	CHOICE_encode_aper,
	CHOICE_outmost_tag,
	0,	/* No effective tags (pointer) */
	0,	/* No effective tags (count) */
	0,	/* No tags (pointer) */
	0,	/* No tags (count) */
	&asn_PER_type_IntersectionAccessPoint_constr_1,
	asn_MBR_IntersectionAccessPoint_1,
	3,	/* Elements count */
	&asn_SPC_IntersectionAccessPoint_specs_1	/* Additional specs */
};
