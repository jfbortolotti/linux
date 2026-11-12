/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __ASM_SPIN_TABLE_H
#define __ASM_SPIN_TABLE_H

#include <linux/types.h>

/*
 * EXPERIMENTAL (apple-s2idle): CPU hotplug for spin-table CPUs, for platforms whose
 * firmware can restart a powered-off core (Apple: the core comes back through its RVBAR,
 * locked to m1n1, which jumps to the entry handed over in cpu_die).
 *
 * @cpu_die:  runs on the dying CPU, IRQs masked. Must power the core off and never return;
 *            when the core restarts it must branch to @entry (physical address of the
 *            holding pen) with the MMU off.
 * @cpu_kill: runs on another CPU after cpu_die, returns 0 if the core is off.
 * @cpu_boot: power the core back on (called after the pen release is written).
 */
struct spin_table_hotplug_ops {
	void (*cpu_die)(unsigned int cpu, phys_addr_t entry);
	int (*cpu_kill)(unsigned int cpu);
	int (*cpu_boot)(unsigned int cpu);
};

void smp_spin_table_set_hotplug_ops(const struct spin_table_hotplug_ops *ops);

#endif
