/*
 * Performance counter — RISC-V mcycle CSR, 480MHz HPM5300
 * Standalone utility. Include only from main.c, NOT from mahony.h.
 *   1 cycle = 2.083 ns,  1 μs = 480 cycles
 */
#ifndef PERF_COUNTER_H
#define PERF_COUNTER_H
#include <stdint.h>
#include "hpm_csr_drv.h"
#ifdef __cplusplus
extern "C" {
#endif

#define PERF_CYCLE_TO_US(c)  ((float)(c) / 480.0f)

static inline uint64_t perf_now(void) { return hpm_csr_get_core_cycle(); }
static inline void perf_init(void)    { hpm_csr_enable_access_to_csr_cycle(); }

typedef struct { uint64_t min, max, sum, count, overflow; } perf_stat_t;

static inline void perf_stat_reset(perf_stat_t *s)
{ s->min=UINT64_MAX; s->max=0; s->sum=0; s->count=0; s->overflow=0; }

static inline void perf_stat_record(perf_stat_t *s, uint64_t t)
{ if(t>100000000ULL){s->overflow++;return;} if(t<s->min)s->min=t; if(t>s->max)s->max=t; s->sum+=t; s->count++; }

#ifdef __cplusplus
}
#endif
#endif
