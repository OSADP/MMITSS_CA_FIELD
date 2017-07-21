//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************
#ifndef _MSGTRANS_H
#define _MSGTRANS_H

#include "wmeUtils.h"
#include "libwme.h"

struct registration_t
{
	int  handler_index;
	bool isConfirmed;
	wmeUtils::appRole   registered_role;
	wmeUtils::direction transm_direction;
	struct savariwme_reg_req registered_req;
};

void init_wme_req(struct savariwme_reg_req& wme_req);
void deinit_wme(void);
void savari_user_confirm(void* ctx, int confirm);
void savari_provider_confirm(void* ctx, int confirm);
void savari_wsm_indication(void* ctx, struct savariwme_rx_indication* rxind);

#endif
