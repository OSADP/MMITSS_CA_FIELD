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

#ifndef	_SpeedProfileMeasurement_H_
#define	_SpeedProfileMeasurement_H_


#include <asn_application.h>

/* Including external dependencies */
#include "GrossSpeed.h"

#ifdef __cplusplus
extern "C" {
#endif

/* SpeedProfileMeasurement */
typedef GrossSpeed_t	 SpeedProfileMeasurement_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_SpeedProfileMeasurement;
asn_struct_free_f SpeedProfileMeasurement_free;
asn_struct_print_f SpeedProfileMeasurement_print;
asn_constr_check_f SpeedProfileMeasurement_constraint;
ber_type_decoder_f SpeedProfileMeasurement_decode_ber;
der_type_encoder_f SpeedProfileMeasurement_encode_der;
xer_type_decoder_f SpeedProfileMeasurement_decode_xer;
xer_type_encoder_f SpeedProfileMeasurement_encode_xer;
per_type_decoder_f SpeedProfileMeasurement_decode_uper;
per_type_encoder_f SpeedProfileMeasurement_encode_uper;
per_type_decoder_f SpeedProfileMeasurement_decode_aper;
per_type_encoder_f SpeedProfileMeasurement_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _SpeedProfileMeasurement_H_ */
#include <asn_internal.h>