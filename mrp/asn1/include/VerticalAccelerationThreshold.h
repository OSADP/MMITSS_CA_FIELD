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

#ifndef	_VerticalAccelerationThreshold_H_
#define	_VerticalAccelerationThreshold_H_


#include <asn_application.h>

/* Including external dependencies */
#include <BIT_STRING.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum VerticalAccelerationThreshold {
	VerticalAccelerationThreshold_notEquipped	= 0,
	VerticalAccelerationThreshold_leftFront	= 1,
	VerticalAccelerationThreshold_leftRear	= 2,
	VerticalAccelerationThreshold_rightFront	= 3,
	VerticalAccelerationThreshold_rightRear	= 4
} e_VerticalAccelerationThreshold;

/* VerticalAccelerationThreshold */
typedef BIT_STRING_t	 VerticalAccelerationThreshold_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_VerticalAccelerationThreshold;
asn_struct_free_f VerticalAccelerationThreshold_free;
asn_struct_print_f VerticalAccelerationThreshold_print;
asn_constr_check_f VerticalAccelerationThreshold_constraint;
ber_type_decoder_f VerticalAccelerationThreshold_decode_ber;
der_type_encoder_f VerticalAccelerationThreshold_encode_der;
xer_type_decoder_f VerticalAccelerationThreshold_decode_xer;
xer_type_encoder_f VerticalAccelerationThreshold_encode_xer;
per_type_decoder_f VerticalAccelerationThreshold_decode_uper;
per_type_encoder_f VerticalAccelerationThreshold_encode_uper;
per_type_decoder_f VerticalAccelerationThreshold_decode_aper;
per_type_encoder_f VerticalAccelerationThreshold_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _VerticalAccelerationThreshold_H_ */
#include <asn_internal.h>
