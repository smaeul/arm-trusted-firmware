/*
 * Copyright (c) 2017-2018, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <arch_helpers.h>
#include <assert.h>
#include <css_scpi.h>
#include <debug.h>
#include <gicv2.h>
#include <mmio.h>
#include <platform.h>
#include <platform_def.h>
#include <psci.h>
#include <sunxi_cpucfg.h>
#include <sunxi_def.h>
#include <sunxi_mmap.h>
#include <sunxi_private.h>

#define CPU_PWR_LVL			MPIDR_AFFLVL0
#define CLUSTER_PWR_LVL			MPIDR_AFFLVL1
#define SYSTEM_PWR_LVL			MPIDR_AFFLVL2

#define CPU_PWR_STATE(state) \
	((state)->pwr_domain_state[CPU_PWR_LVL])
#define CLUSTER_PWR_STATE(state) \
	((state)->pwr_domain_state[CLUSTER_PWR_LVL])
#define SYSTEM_PWR_STATE(state) \
	((state)->pwr_domain_state[SYSTEM_PWR_LVL])

#define mpidr_is_valid(mpidr) (plat_core_pos_by_mpidr(mpidr) >= 0)

static void sunxi_cpu_standby(plat_local_state_t cpu_state)
{
	unsigned int scr = read_scr_el3();

	assert(is_local_state_retn(cpu_state));

	write_scr_el3(scr | SCR_IRQ_BIT);
	wfi();
	write_scr_el3(scr);
}

static int sunxi_pwr_domain_on(u_register_t mpidr)
{
	if (mpidr_is_valid(mpidr) == 0)
		return PSCI_E_INTERN_FAIL;

	scpi_set_css_power_state(mpidr,
				 scpi_power_on,
				 scpi_power_on,
	                         scpi_power_on);

	return PSCI_E_SUCCESS;
}

static void sunxi_pwr_domain_off_common(const psci_power_state_t *target_state)
{
	if (is_local_state_off(CPU_PWR_STATE(target_state)))
		gicv2_cpuif_disable();

	scpi_set_css_power_state(read_mpidr(),
				 CPU_PWR_STATE(target_state),
	                         CLUSTER_PWR_STATE(target_state),
				 SYSTEM_PWR_STATE(target_state));
}

static void sunxi_pwr_domain_off(const psci_power_state_t *target_state)
{
	assert(is_local_state_off(CPU_PWR_STATE(target_state)));

	sunxi_pwr_domain_off_common(target_state);
}

static void sunxi_pwr_domain_suspend(const psci_power_state_t *target_state)
{
	sunxi_pwr_domain_off_common(target_state);
}

static void sunxi_pwr_domain_on_finish_common(const psci_power_state_t
								*target_state)
{
	if (is_local_state_off(SYSTEM_PWR_STATE(target_state)))
		gicv2_distif_init();
	if (is_local_state_off(CPU_PWR_STATE(target_state))) {
		gicv2_pcpu_distif_init();
		gicv2_cpuif_enable();
	}
}

static void sunxi_pwr_domain_on_finish(const psci_power_state_t *target_state)
{
	sunxi_pwr_domain_on_finish_common(target_state);
}

static void sunxi_pwr_domain_suspend_finish(const psci_power_state_t *target_state)
{
	sunxi_pwr_domain_on_finish_common(target_state);
}

static void __dead2 sunxi_system_off(void)
{
	gicv2_cpuif_disable();

	/* Send the power down request to the SCP */
	if (scpi_sys_power_state(scpi_system_shutdown) != SCP_OK)
		ERROR("System Off: SCP error.\n");

	wfi();
	panic();
}

static void __dead2 sunxi_system_reset(void)
{
	gicv2_cpuif_disable();

	/* Send the system reset request to the SCP */
	if (scpi_sys_power_state(scpi_system_reset) != SCP_OK)
		ERROR("System Reset: SCP error.\n");

	wfi();
	panic();
}

static int sunxi_validate_power_state(unsigned int power_state,
				       psci_power_state_t *req_state)
{
	unsigned int power_level = psci_get_pstate_pwrlvl(power_state);
	unsigned int type = psci_get_pstate_type(power_state);

	assert(req_state);

	if (power_level > PLAT_MAX_PWR_LVL)
		return PSCI_E_INVALID_PARAMS;

	if (type == PSTATE_TYPE_STANDBY) {
		/* Only one retention power state is supported. */
		if (psci_get_pstate_id(power_state) > 0)
			return PSCI_E_INVALID_PARAMS;
		/* The SoC cannot be suspended without losing state */
		if (power_level == SYSTEM_PWR_LVL)
			return PSCI_E_INVALID_PARAMS;
		for (int i = 0; i <= power_level; ++i)
			req_state->pwr_domain_state[i] = PLAT_MAX_RET_STATE;
	} else {
		/* Only one off power state is supported. */
		if (psci_get_pstate_id(power_state) > 0)
			return PSCI_E_INVALID_PARAMS;
		for (int i = 0; i <= power_level; ++i)
			req_state->pwr_domain_state[i] = PLAT_MAX_OFF_STATE;
	}
	/* Higher power domain levels should all remain running */
	for (int i = power_level + 1; i <= PLAT_MAX_PWR_LVL; ++i)
		req_state->pwr_domain_state[i] = PSCI_LOCAL_STATE_RUN;

	return PSCI_E_SUCCESS;
}

static int sunxi_validate_ns_entrypoint(uintptr_t ns_entrypoint)
{
	/* The non-secure entry point must be in DRAM */
	if (ns_entrypoint >= SUNXI_DRAM_BASE)
		return PSCI_E_SUCCESS;

	return PSCI_E_INVALID_ADDRESS;
}

static void sunxi_get_sys_suspend_power_state(psci_power_state_t *req_state)
{
	assert(req_state);

	for (int i = 0; i <= PLAT_MAX_PWR_LVL; ++i)
		req_state->pwr_domain_state[i] = PLAT_MAX_OFF_STATE;
}

static int sunxi_get_node_hw_state(u_register_t mpidr,
				    unsigned int power_level)
{
	unsigned int cluster_state, cpu_state;

	/* SoC power level (always on if PSCI works). */
	if (power_level == SYSTEM_PWR_LVL)
		return HW_ON;
	if (scpi_get_css_power_state(mpidr, &cpu_state, &cluster_state))
		return PSCI_E_NOT_SUPPORTED;
	/* Cluster power level (full power state available). */
	if (power_level == CLUSTER_PWR_LVL) {
		if (is_local_state_run(cluster_state))
			return HW_ON;
		if (is_local_state_retn(cluster_state))
			return HW_STANDBY;
		return HW_OFF;
	}
	/* CPU power level (one bit boolean for on or off). */
	return cpu_state & BIT(MPIDR_AFFLVL0_VAL(mpidr)) ? HW_ON : HW_OFF;
}

static plat_psci_ops_t sunxi_psci_ops = {
	.cpu_standby			= sunxi_cpu_standby,
	.pwr_domain_on			= sunxi_pwr_domain_on,
	.pwr_domain_off			= sunxi_pwr_domain_off,
	.pwr_domain_suspend		= sunxi_pwr_domain_suspend,
	.pwr_domain_on_finish		= sunxi_pwr_domain_on_finish,
	.pwr_domain_suspend_finish	= sunxi_pwr_domain_suspend_finish,
	.system_off			= sunxi_system_off,
	.system_reset			= sunxi_system_reset,
	.validate_power_state		= sunxi_validate_power_state,
	.validate_ns_entrypoint		= sunxi_validate_ns_entrypoint,
	.get_sys_suspend_power_state	= sunxi_get_sys_suspend_power_state,
	.get_node_hw_state		= sunxi_get_node_hw_state,
};

int plat_setup_psci_ops(uintptr_t sec_entrypoint,
			const plat_psci_ops_t **psci_ops)
{
	assert(psci_ops);

	/* Program all CPU entry points. */
	for (int cpu = 0; cpu < PLATFORM_CORE_COUNT; cpu += 1) {
		mmio_write_32(SUNXI_CPUCFG_RVBAR_LO_REG(cpu),
			      sec_entrypoint & 0xffffffff);
		mmio_write_32(SUNXI_CPUCFG_RVBAR_HI_REG(cpu),
			      sec_entrypoint >> 32);
	}

	/* Check for a valid reset vector, and boot the SCP. */
	if (mmio_read_32(SUNXI_SRAM_A2_BASE - 0x4000 + 0x100) != 0) {
		NOTICE("ARISC firmware found, deasserting reset\n");
		mmio_setbits_32(SUNXI_R_CPUCFG_BASE, BIT(0));
		if (scpi_wait_ready())
			ERROR("SCP firmware is not responding\n");
	} else {
		ERROR("ARISC firmware not found, PSCI will not function!\n");
	}

	*psci_ops = &sunxi_psci_ops;

	return 0;
}
