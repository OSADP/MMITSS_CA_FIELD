#ifndef _MMITSSWMETX_H
#define _MMITSSWMETX_H

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
	struct savariwme_tx_req wmetx;
};

void wsm_tx(int sock_t, void *eloop_data, void *user_data);
void close_all(void);

#endif
