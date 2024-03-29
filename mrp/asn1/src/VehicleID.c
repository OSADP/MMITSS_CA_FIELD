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

#include "VehicleID.h"

static asn_per_constraints_t asn_PER_type_VehicleID_constr_1 GCC_NOTUSED = {
	{ APC_CONSTRAINED,	 1,  1,  0l,  1l }	/* (0..1) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_TYPE_member_t asn_MBR_VehicleID_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct VehicleID, choice.entityID),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_TemporaryID,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"entityID"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct VehicleID, choice.stationID),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_StationID,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"stationID"
		},
};
static const asn_TYPE_tag2member_t asn_MAP_VehicleID_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* entityID */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* stationID */
};
static asn_CHOICE_specifics_t asn_SPC_VehicleID_specs_1 = {
	sizeof(struct VehicleID),
	offsetof(struct VehicleID, _asn_ctx),
	offsetof(struct VehicleID, present),
	sizeof(((struct VehicleID *)0)->present),
	asn_MAP_VehicleID_tag2el_1,
	2,	/* Count of tags in the map */
	0,
	-1	/* Extensions start */
};
asn_TYPE_descriptor_t asn_DEF_VehicleID = {
	"VehicleID",
	"VehicleID",
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
	&asn_PER_type_VehicleID_constr_1,
	asn_MBR_VehicleID_1,
	2,	/* Elements count */
	&asn_SPC_VehicleID_specs_1	/* Additional specs */
};

