//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************
#ifndef _CATCHRUNTIMESIGNAL_H
#define _CATCHRUNTIMESIGNAL_H

#include <csetjmp>
#include <csignal>

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

#endif
