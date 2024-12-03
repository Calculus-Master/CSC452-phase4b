
#ifndef _PHASE3_KERNEL_INTERFACES_H
#define _PHASE3_KERNEL_INTERFACES_H

// these functions are available for use in later Phases
int kernSemCreate(int value, int *semaphore);
int kernSemP     (int semaphore);
int kernSemV     (int semaphore);

#endif /* _PHASE3_KERNEL_INTERFACES_H */

