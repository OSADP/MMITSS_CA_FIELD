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

#ifndef	_WeatherReport_H_
#define	_WeatherReport_H_


#include <asn_application.h>

/* Including external dependencies */
#include "EssPrecipYesNo.h"
#include "EssPrecipRate.h"
#include "EssPrecipSituation.h"
#include "EssSolarRadiation.h"
#include "EssMobileFriction.h"
#include "CoefficientOfFriction.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* WeatherReport */
typedef struct WeatherReport {
	EssPrecipYesNo_t	 isRaining;
	EssPrecipRate_t	*rainRate	/* OPTIONAL */;
	EssPrecipSituation_t	*precipSituation	/* OPTIONAL */;
	EssSolarRadiation_t	*solarRadiation	/* OPTIONAL */;
	EssMobileFriction_t	*friction	/* OPTIONAL */;
	CoefficientOfFriction_t	*roadFriction	/* OPTIONAL */;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} WeatherReport_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_WeatherReport;

#ifdef __cplusplus
}
#endif

#endif	/* _WeatherReport_H_ */
#include <asn_internal.h>
