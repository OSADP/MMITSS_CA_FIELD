//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************
#ifndef _MMITSSWMERX_H
#define _MMITSSWMERX_H

#include <string>
#include <cstring>
#include <stdint.h>	// c++11 <cstdint>
#include <csetjmp>
#include <csignal>
#include <vector>
#include <map>
#include <sys/socket.h>
#include <netinet/in.h>

#include "libwme.h"	// savari_wme_handler_t, savariwme_cbs, savariwme_reg_req, savariwme_tx_req

/// interrupt signal
#define ERROR -1
static int sig_list[] = 
{
	/// list of signals for interruption, handled by sig_hand ()
	SIGABRT,
	SIGINT,			
	SIGQUIT,		
	SIGTERM,		
	SIGKILL,
	SIGSEGV,
	ERROR
};
static jmp_buf exit_env;
static void sig_hand( int code )
{
	longjmp( exit_env, code );
};
static void sig_ign(int *sigList, void sig_hand(int sig))
{
  int i = 0;
  while (sigList[i] != ERROR)
	{
    signal(sigList[i], sig_hand);
    i++;
  };
};

struct savariWmeHandler_t
{
	bool isRegistered;
	struct savariwme_reg_req wmereq;
};

void savari_rx(int sock_t, void *eloop_data, void *user_data);
void savari_user_confirm(void *ctx, int conf_result_indication);
void savari_provider_confirm(void *ctx, int conf_result_indication);
void savari_wsm_indication(void *ctx, struct savariwme_rx_indication *rxind);
void close_all(void);

#endif
