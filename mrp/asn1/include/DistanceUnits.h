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

#ifndef	_DistanceUnits_H_
#define	_DistanceUnits_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum DistanceUnits {
	DistanceUnits_centimeter	= 0,
	DistanceUnits_cm2_5	= 1,
	DistanceUnits_decimeter	= 2,
	DistanceUnits_meter	= 3,
	DistanceUnits_kilometer	= 4,
	DistanceUnits_foot	= 5,
	DistanceUnits_yard	= 6,
	DistanceUnits_mile	= 7
} e_DistanceUnits;

/* DistanceUnits */
typedef long	 DistanceUnits_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_DistanceUnits;
asn_struct_free_f DistanceUnits_free;
asn_struct_print_f DistanceUnits_print;
asn_constr_check_f DistanceUnits_constraint;
ber_type_decoder_f DistanceUnits_decode_ber;
der_type_encoder_f DistanceUnits_encode_der;
xer_type_decoder_f DistanceUnits_decode_xer;
xer_type_encoder_f DistanceUnits_encode_xer;
per_type_decoder_f DistanceUnits_decode_uper;
per_type_encoder_f DistanceUnits_encode_uper;
per_type_decoder_f DistanceUnits_decode_aper;
per_type_encoder_f DistanceUnits_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _DistanceUnits_H_ */
#include <asn_internal.h>
