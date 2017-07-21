//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************
#ifndef _ASN_J2735_LIB_H
#define _ASN_J2735_LIB_H

#include <cstddef>
#include <cstdint>

#include "dsrcBSM.h"
#include "dsrcSPAT.h"
#include "dsrcSRM.h"
#include "dsrcSSM.h"
#include "dsrcMapData.h"

namespace AsnJ2735Lib
{
	// UPER encoding functions
	size_t encode_mapdata_payload(const MapData_element_t& mapDataIn, uint8_t* buf, size_t size);
	size_t encode_spat_payload(const SPAT_element_t& spatIn, uint8_t* buf, size_t size);
	size_t encode_srm_payload(const SRM_element_t& srmIn, uint8_t* buf, size_t size);
	size_t encode_ssm_payload(const SSM_element_t& ssmIn, uint8_t* buf, size_t size);
	size_t encode_bsm_payload(const BSM_element_t& bsmIn, uint8_t* buf, size_t size);

	// UPER decoding functions
	size_t decode_mapdata_payload(const uint8_t* buf, size_t size, MapData_element_t& mapDataOut);
	size_t decode_spat_payload(const uint8_t* buf, size_t size, SPAT_element_t& spatOut);
	size_t decode_srm_payload(const uint8_t* buf, size_t size, SRM_element_t& srmOut);
	size_t decode_ssm_payload(const uint8_t* buf, size_t size, SSM_element_t& ssmOut);
	size_t decode_bsm_payload(const uint8_t* buf, size_t size, BSM_element_t& bsmOut);
};

#endif
