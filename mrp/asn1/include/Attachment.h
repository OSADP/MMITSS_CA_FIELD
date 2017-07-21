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

#ifndef	_Attachment_H_
#define	_Attachment_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum Attachment {
	Attachment_unavailable	= 0,
	Attachment_stroller	= 1,
	Attachment_bicycleTrailer	= 2,
	Attachment_cart	= 3,
	Attachment_wheelchair	= 4,
	Attachment_otherWalkAssistAttachments	= 5,
	Attachment_pet	= 6
	/*
	 * Enumeration is extensible
	 */
} e_Attachment;

/* Attachment */
typedef long	 Attachment_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_Attachment;
asn_struct_free_f Attachment_free;
asn_struct_print_f Attachment_print;
asn_constr_check_f Attachment_constraint;
ber_type_decoder_f Attachment_decode_ber;
der_type_encoder_f Attachment_encode_der;
xer_type_decoder_f Attachment_decode_xer;
xer_type_encoder_f Attachment_encode_xer;
per_type_decoder_f Attachment_decode_uper;
per_type_encoder_f Attachment_encode_uper;
per_type_decoder_f Attachment_decode_aper;
per_type_encoder_f Attachment_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _Attachment_H_ */
#include <asn_internal.h>