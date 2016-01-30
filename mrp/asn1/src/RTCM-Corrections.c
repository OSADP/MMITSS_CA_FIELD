/*
 * Generated by asn1c-0.9.22 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "module.asn1"
 * 	`asn1c -S/skeletons`
 */

#include <asn_internal.h>

#include "RTCM-Corrections.h"

static int
memb_rtcmSets_constraint_1(asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	size_t size;
	
	if(!sptr) {
		_ASN_CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	/* Determine the number of elements */
	size = _A_CSEQUENCE_FROM_VOID(sptr)->count;
	
	if((size >= 1 && size <= 5)) {
		/* Perform validation of the inner elements */
		return td->check_constraints(td, sptr, ctfailcb, app_key);
	} else {
		_ASN_CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static asn_TYPE_member_t asn_MBR_rtcmSets_7[] = {
	{ ATF_POINTER, 0, 0,
		(ASN_TAG_CLASS_UNIVERSAL | (16 << 2)),
		0,
		&asn_DEF_RTCMmsg,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		""
		},
};
static ber_tlv_tag_t asn_DEF_rtcmSets_tags_7[] = {
	(ASN_TAG_CLASS_CONTEXT | (5 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static asn_SET_OF_specifics_t asn_SPC_rtcmSets_specs_7 = {
	sizeof(struct RTCM_Corrections__rtcmSets),
	offsetof(struct RTCM_Corrections__rtcmSets, _asn_ctx),
	0,	/* XER encoding is XMLDelimitedItemList */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_rtcmSets_7 = {
	"rtcmSets",
	"rtcmSets",
	SEQUENCE_OF_free,
	SEQUENCE_OF_print,
	SEQUENCE_OF_constraint,
	SEQUENCE_OF_decode_ber,
	SEQUENCE_OF_encode_der,
	SEQUENCE_OF_decode_xer,
	SEQUENCE_OF_encode_xer,
	0, 0,	/* No PER support, use "-gen-PER" to enable */
	0,	/* Use generic outmost tag fetcher */
	asn_DEF_rtcmSets_tags_7,
	sizeof(asn_DEF_rtcmSets_tags_7)
		/sizeof(asn_DEF_rtcmSets_tags_7[0]) - 1, /* 1 */
	asn_DEF_rtcmSets_tags_7,	/* Same as above */
	sizeof(asn_DEF_rtcmSets_tags_7)
		/sizeof(asn_DEF_rtcmSets_tags_7[0]), /* 2 */
	0,	/* No PER visible constraints */
	asn_MBR_rtcmSets_7,
	1,	/* Single element */
	&asn_SPC_rtcmSets_specs_7	/* Additional specs */
};

static asn_TYPE_member_t asn_MBR_RTCM_Corrections_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct RTCM_Corrections, msgID),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_DSRCmsgID,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"msgID"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct RTCM_Corrections, msgCnt),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_MsgCount,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"msgCnt"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct RTCM_Corrections, rev),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_RTCM_Revision,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"rev"
		},
	{ ATF_POINTER, 1, offsetof(struct RTCM_Corrections, anchorPoint),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_FullPositionVector,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"anchorPoint"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct RTCM_Corrections, rtcmHeader),
		(ASN_TAG_CLASS_CONTEXT | (4 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_RTCMHeader,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"rtcmHeader"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct RTCM_Corrections, rtcmSets),
		(ASN_TAG_CLASS_CONTEXT | (5 << 2)),
		0,
		&asn_DEF_rtcmSets_7,
		memb_rtcmSets_constraint_1,
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"rtcmSets"
		},
};
static ber_tlv_tag_t asn_DEF_RTCM_Corrections_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static asn_TYPE_tag2member_t asn_MAP_RTCM_Corrections_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* msgID at 322 */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* msgCnt at 323 */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* rev at 324 */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 }, /* anchorPoint at 328 */
    { (ASN_TAG_CLASS_CONTEXT | (4 << 2)), 4, 0, 0 }, /* rtcmHeader at 332 */
    { (ASN_TAG_CLASS_CONTEXT | (5 << 2)), 5, 0, 0 } /* rtcmSets at 338 */
};
static asn_SEQUENCE_specifics_t asn_SPC_RTCM_Corrections_specs_1 = {
	sizeof(struct RTCM_Corrections),
	offsetof(struct RTCM_Corrections, _asn_ctx),
	asn_MAP_RTCM_Corrections_tag2el_1,
	6,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	5,	/* Start extensions */
	7	/* Stop extensions */
};
asn_TYPE_descriptor_t asn_DEF_RTCM_Corrections = {
	"RTCM-Corrections",
	"RTCM-Corrections",
	SEQUENCE_free,
	SEQUENCE_print,
	SEQUENCE_constraint,
	SEQUENCE_decode_ber,
	SEQUENCE_encode_der,
	SEQUENCE_decode_xer,
	SEQUENCE_encode_xer,
	0, 0,	/* No PER support, use "-gen-PER" to enable */
	0,	/* Use generic outmost tag fetcher */
	asn_DEF_RTCM_Corrections_tags_1,
	sizeof(asn_DEF_RTCM_Corrections_tags_1)
		/sizeof(asn_DEF_RTCM_Corrections_tags_1[0]), /* 1 */
	asn_DEF_RTCM_Corrections_tags_1,	/* Same as above */
	sizeof(asn_DEF_RTCM_Corrections_tags_1)
		/sizeof(asn_DEF_RTCM_Corrections_tags_1[0]), /* 1 */
	0,	/* No PER visible constraints */
	asn_MBR_RTCM_Corrections_1,
	6,	/* Elements count */
	&asn_SPC_RTCM_Corrections_specs_1	/* Additional specs */
};
