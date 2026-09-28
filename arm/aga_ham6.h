#ifndef AGA_PISTORM_FRF_HAM6_H
#define AGA_PISTORM_FRF_HAM6_H

#include <stdint.h>

/* Physical RGB output only. V0.15.2 uses a runtime-geometry lock-free CPU3->CPU1 A/B mailbox plus optional physical HAM page flipping:
 * CPU3 snapshots completed software-AGA RGB frames into Fast RAM; CPU1 owns
 * HAM conversion, dirty detection and all physical Chip-RAM publication. */
void aga_ham6_set_descriptor(uint32_t addr);
void aga_ham6_defer_start(int on);          /* program-scoped handover: descriptor may exist while physical HAM stays off */
uint32_t aga_ham6_descriptor(void);
int  aga_ham6_begin(void);
int  aga_ham6_active(void);
void aga_ham6_present(const uint32_t *frame, int native_active);
void aga_ham6_cpu1_service(void);
void aga_ham6_cpu1_main(void);
void aga_ham6_end(void);

#endif
