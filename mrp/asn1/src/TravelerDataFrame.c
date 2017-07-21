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

#include "TravelerDataFrame.h"

static int
memb_regions_constraint_1(asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	size_t size;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	/* Determine the number of elements */
	size = _A_CSEQUENCE_FROM_VOID(sptr)->count;
	
	if((size >= 1LL && size <= 16LL)) {
		/* Perform validation of the inner elements */
		return td->check_constraints(td, sptr, ctfailcb, app_key);
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static asn_per_constraints_t asn_PER_type_msgId_constr_4 GCC_NOTUSED = {
	{ APC_CONSTRAINED,	 1,  1,  0l,  1l }	/* (0..1) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_per_constraints_t asn_PER_type_regions_constr_12 GCC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED,	 4,  4,  1l,  16l }	/* (SIZE(1..16)) */,
	0, 0	/* No PER value map */
};
static asn_per_constraints_t asn_PER_type_content_constr_16 GCC_NOTUSED = {
	{ APC_CONSTRAINED,	 3,  3,  0l,  4l }	/* (0..4) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_per_constraints_t asn_PER_memb_regions_constr_12 GCC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED,	 4,  4,  1l,  16l }	/* (SIZE(1..16)) */,
	0, 0	/* No PER value map */
};
static asn_TYPE_member_t asn_MBR_msgId_4[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct TravelerDataFrame__msgId, choice.furtherInfoID),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_FurtherInfoID,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"furtherInfoID"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct TravelerDataFrame__msgId, choice.roadSignID),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_RoadSignID,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"roadSignID"
		},
};
static const asn_TYPE_tag2member_t asn_MAP_msgId_tag2el_4[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* furtherInfoID */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* roadSignID */
};
static asn_CHOICE_specifics_t asn_SPC_msgId_specs_4 = {
	sizeof(struct TravelerDataFrame__msgId),
	offsetof(struct TravelerDataFrame__msgId, _asn_ctx),
	offsetof(struct TravelerDataFrame__msgId, present),
	sizeof(((struct TravelerDataFrame__msgId *)0)->present),
	asn_MAP_msgId_tag2el_4,
	2,	/* Count of tags in the map */
	0,
	-1	/* Extensions start */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_msgId_4 = {
	"msgId",
	"msgId",
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
	&asn_PER_type_msgId_constr_4,
	asn_MBR_msgId_4,
	2,	/* Elements count */
	&asn_SPC_msgId_specs_4	/* Additional specs */
};

static asn_TYPE_member_t asn_MBR_regions_12[] = {
	{ ATF_POINTER, 0, 0,
		(ASN_TAG_CLASS_UNIVERSAL | (16 << 2)),
		0,
		&asn_DEF_GeographicalPath,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		""
		},
};
static const ber_tlv_tag_t asn_DEF_regions_tags_12[] = {
	(ASN_TAG_CLASS_CONTEXT | (8 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static asn_SET_OF_specifics_t asn_SPC_regions_specs_12 = {
	sizeof(struct TravelerDataFrame__regions),
	offsetof(struct TravelerDataFrame__regions, _asn_ctx),
	0,	/* XER encoding is XMLDelimitedItemList */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_regions_12 = {
	"regions",
	"regions",
	SEQUENCE_OF_free,
	SEQUENCE_OF_print,
	SEQUENCE_OF_constraint,
	SEQUENCE_OF_decode_ber,
	SEQUENCE_OF_encode_der,
	SEQUENCE_OF_decode_xer,
	SEQUENCE_OF_encode_xer,
	SEQUENCE_OF_decode_uper,
	SEQUENCE_OF_encode_uper,
	SEQUENCE_OF_decode_aper,
	SEQUENCE_OF_encode_aper,
	0,	/* Use generic outmost tag fetcher */
	asn_DEF_regions_tags_12,
	sizeof(asn_DEF_regions_tags_12)
		/sizeof(asn_DEF_regions_tags_12[0]) - 1, /* 1 */
	asn_DEF_regions_tags_12,	/* Same as above */
	sizeof(asn_DEF_regions_tags_12)
		/sizeof(asn_DEF_regions_tags_12[0]), /* 2 */
	&asn_PER_type_regions_constr_12,
	asn_MBR_regions_12,
	1,	/* Single element */
	&asn_SPC_regions_specs_12	/* Additional specs */
};

static asn_TYPE_member_t asn_MBR_content_16[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct TravelerDataFrame__content, choice.advisory),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_ITIScodesAndText,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"advisory"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct TravelerDataFrame__content, choice.workZone),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_WorkZone,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"workZone"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct TravelerDataFrame__content, choice.genericSign),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_GenericSignage,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"genericSign"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct TravelerDataFrame__content, choice.speedLimit),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_SpeedLimit,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"speedLimit"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct TravelerDataFrame__content, choice.exitService),
		(ASN_TAG_CLASS_CONTEXT | (4 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_ExitService,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"exitService"
		},
};
static const asn_TYPE_tag2member_t asn_MAP_content_tag2el_16[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* advisory */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* workZone */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* genericSign */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 }, /* speedLimit */
    { (ASN_TAG_CLASS_CONTEXT | (4 << 2)), 4, 0, 0 } /* exitService */
};
static asn_CHOICE_specifics_t asn_SPC_content_specs_16 = {
	sizeof(struct TravelerDataFrame__content),
	offsetof(struct TravelerDataFrame__content, _asn_ctx),
	offsetof(struct TravelerDataFrame__content, present),
	sizeof(((struct TravelerDataFrame__content *)0)->present),
	asn_MAP_content_tag2el_16,
	5,	/* Count of tags in the map */
	0,
	-1	/* Extensions start */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_content_16 = {
	"content",
	"content",
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
	&asn_PER_type_content_constr_16,
	asn_MBR_content_16,
	5,	/* Elements count */
	&asn_SPC_content_specs_16	/* Additional specs */
};

static asn_TYPE_member_t asn_MBR_TravelerDataFrame_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct TravelerDataFrame, sspTimRights),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_SSPindex,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"sspTimRights"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct TravelerDataFrame, frameType),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_TravelerInfoType,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"frameType"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct TravelerDataFrame, msgId),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		+1,	/* EXPLICIT tag at current level */
		&asn_DEF_msgId_4,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"msgId"
		},
	{ ATF_POINTER, 1, offsetof(struct TravelerDataFrame, startYear),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_DYear,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"startYear"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct TravelerDataFrame, startTime),
		(ASN_TAG_CLASS_CONTEXT | (4 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_MinuteOfTheYear,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"startTime"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct TravelerDataFrame, duratonTime),
		(ASN_TAG_CLASS_CONTEXT | (5 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_MinutesDuration,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"duratonTime"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct TravelerDataFrame, priority),
		(ASN_TAG_CLASS_CONTEXT | (6 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_SignPrority,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"priority"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct TravelerDataFrame, sspLocationRights),
		(ASN_TAG_CLASS_CONTEXT | (7 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_SSPindex,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"sspLocationRights"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct TravelerDataFrame, regions),
		(ASN_TAG_CLASS_CONTEXT | (8 << 2)),
		0,
		&asn_DEF_regions_12,
		memb_regions_constraint_1,
		&asn_PER_memb_regions_constr_12,
		0,
		"regions"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct TravelerDataFrame, sspMsgRights1),
		(ASN_TAG_CLASS_CONTEXT | (9 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_SSPindex,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"sspMsgRights1"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct TravelerDataFrame, sspMsgRights2),
		(ASN_TAG_CLASS_CONTEXT | (10 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_SSPindex,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"sspMsgRights2"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct TravelerDataFrame, content),
		(ASN_TAG_CLASS_CONTEXT | (11 << 2)),
		+1,	/* EXPLICIT tag at current level */
		&asn_DEF_content_16,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"content"
		},
	{ ATF_POINTER, 1, offsetof(struct TravelerDataFrame, url),
		(ASN_TAG_CLASS_CONTEXT | (12 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_URL_Short,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"url"
		},
};
static const int asn_MAP_TravelerDataFrame_oms_1[] = { 3, 12 };
static const ber_tlv_tag_t asn_DEF_TravelerDataFrame_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_TravelerDataFrame_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* sspTimRights */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* frameType */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* msgId */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 }, /* startYear */
    { (ASN_TAG_CLASS_CONTEXT | (4 << 2)), 4, 0, 0 }, /* startTime */
    { (ASN_TAG_CLASS_CONTEXT | (5 << 2)), 5, 0, 0 }, /* duratonTime */
    { (ASN_TAG_CLASS_CONTEXT | (6 << 2)), 6, 0, 0 }, /* priority */
    { (ASN_TAG_CLASS_CONTEXT | (7 << 2)), 7, 0, 0 }, /* sspLocationRights */
    { (ASN_TAG_CLASS_CONTEXT | (8 << 2)), 8, 0, 0 }, /* regions */
    { (ASN_TAG_CLASS_CONTEXT | (9 << 2)), 9, 0, 0 }, /* sspMsgRights1 */
    { (ASN_TAG_CLASS_CONTEXT | (10 << 2)), 10, 0, 0 }, /* sspMsgRights2 */
    { (ASN_TAG_CLASS_CONTEXT | (11 << 2)), 11, 0, 0 }, /* content */
    { (ASN_TAG_CLASS_CONTEXT | (12 << 2)), 12, 0, 0 } /* url */
};
static asn_SEQUENCE_specifics_t asn_SPC_TravelerDataFrame_specs_1 = {
	sizeof(struct TravelerDataFrame),
	offsetof(struct TravelerDataFrame, _asn_ctx),
	asn_MAP_TravelerDataFrame_tag2el_1,
	13,	/* Count of tags in the map */
	asn_MAP_TravelerDataFrame_oms_1,	/* Optional members */
	2, 0,	/* Root/Additions */
	12,	/* Start extensions */
	14	/* Stop extensions */
};
asn_TYPE_descriptor_t asn_DEF_TravelerDataFrame = {
	"TravelerDataFrame",
	"TravelerDataFrame",
	SEQUENCE_free,
	SEQUENCE_print,
	SEQUENCE_constraint,
	SEQUENCE_decode_ber,
	SEQUENCE_encode_der,
	SEQUENCE_decode_xer,
	SEQUENCE_encode_xer,
	SEQUENCE_decode_uper,
	SEQUENCE_encode_uper,
	SEQUENCE_decode_aper,
	SEQUENCE_encode_aper,
	0,	/* Use generic outmost tag fetcher */
	asn_DEF_TravelerDataFrame_tags_1,
	sizeof(asn_DEF_TravelerDataFrame_tags_1)
		/sizeof(asn_DEF_TravelerDataFrame_tags_1[0]), /* 1 */
	asn_DEF_TravelerDataFrame_tags_1,	/* Same as above */
	sizeof(asn_DEF_TravelerDataFrame_tags_1)
		/sizeof(asn_DEF_TravelerDataFrame_tags_1[0]), /* 1 */
	0,	/* No PER visible constraints */
	asn_MBR_TravelerDataFrame_1,
	13,	/* Elements count */
	&asn_SPC_TravelerDataFrame_specs_1	/* Additional specs */
};
