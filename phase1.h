/*
 * These are the definitions for phase1 of the project (the kernel).
 */

#ifndef _PHASE1_H
#define _PHASE1_H

#include <usloss.h>

/*
 * Maximum number of processes. 
 */

#define MAXPROC      50

/*
 * Maximum length of a process name
 */

#define MAXNAME      50

/*
 * Maximum length of string argument passed to a newly created process
 */

#define MAXARG       100

/*
 * Maximum number of syscalls.
 */

#define MAXSYSCALLS  50


/* 
 * These functions must be provided by Phase 1.
 */

extern void phase1_init(void);
extern int  spork(char *name, int(*func)(void *), void *arg,
                  int stacksize, int priority);
extern int  join(int *status);
extern void quit(int status) __attribute__((__noreturn__));
extern void zap(int pid);
extern void blockMe(void);
extern int  unblockProc(int pid);

extern void dispatcher(void);

extern int  currentTime(void);

extern int  getpid(void);
extern void dumpProcesses(void);


/*
 * These functions are called *BY* Phase 1 code, and are implemented in
 * Phase 5.  If we are testing code before Phase 5 is written, then the
 * testcase must provide a NOP implementation of each.
 */

extern USLOSS_PTE *phase5_mmu_pageTable_alloc(int pid);
extern void        phase5_mmu_pageTable_free (int pid, USLOSS_PTE*);



/* these functions are also called by the phase 1 code, from inside
 * init_main().  They are called first; after they return, init()
 * enters an infinite loop, just join()ing with children forever.
 *
 * In early phases, these are provided (as NOPs) by the testcase.
 */
extern void phase2_start_service_processes(void);
extern void phase3_start_service_processes(void);
extern void phase4_start_service_processes(void);
extern void phase5_start_service_processes(void);

/* this function is called by the init process, after the service
 * processes are running, to start whatever processes the testcase
 * wants to run.  This may call spork() many times, and
 * block as long as you want.  When it returns, Halt() will be
 * called by the Phase 1 code (nonzero means error).
 */
extern int testcase_main(void);



#endif /* _PHASE1_H */
