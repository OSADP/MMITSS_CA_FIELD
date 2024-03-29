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

#ifndef	_SpeedConfidence_H_
#define	_SpeedConfidence_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum SpeedConfidence {
	SpeedConfidence_unavailable	= 0,
	SpeedConfidence_prec100ms	= 1,
	SpeedConfidence_prec10ms	= 2,
	SpeedConfidence_prec5ms	= 3,
	SpeedConfidence_prec1ms	= 4,
	SpeedConfidence_prec0_1ms	= 5,
	SpeedConfidence_prec0_05ms	= 6,
	SpeedConfidence_prec0_01ms	= 7
} e_SpeedConfidence;

/* SpeedConfidence */
typedef long	 SpeedConfidence_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_SpeedConfidence;
asn_struct_free_f SpeedConfidence_free;
asn_struct_print_f SpeedConfidence_print;
asn_constr_check_f SpeedConfidence_constraint;
ber_type_decoder_f SpeedConfidence_decode_ber;
der_type_encoder_f SpeedConfidence_encode_der;
xer_type_decoder_f SpeedConfidence_decode_xer;
xer_type_encoder_f SpeedConfidence_encode_xer;
per_type_decoder_f SpeedConfidence_decode_uper;
per_type_encoder_f SpeedConfidence_encode_uper;
per_type_decoder_f SpeedConfidence_decode_aper;
per_type_encoder_f SpeedConfidence_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _SpeedConfidence_H_ */
#include <asn_internal.h>
