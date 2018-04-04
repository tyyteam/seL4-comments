/*
 * Copyright 2018, Data61
 * Commonwealth Scientific and Industrial Research Organisation (CSIRO)
 * ABN 41 687 119 230.
 *
 * This software may be distributed and modified according to the terms of
 * the GNU General Public License version 2. Note that NO WARRANTY is provided.
 * See "LICENSE_GPLv2.txt" for details.
 *
 * @TAG(DATA61_GPL)
 */

#ifndef __ARCH_ARMV_VCPU_H_
#define __ARCH_ARMV_VCPU_H_

#include <config.h>

#ifdef CONFIG_ARM_HYPERVISOR_SUPPORT

#include <arch/object/vcpu.h>

/* Note that the HCR_DC for ARMv8 disables S1 translation if enabled */
/* Trap WFI/WFE/SMC and override CPSR.AIF */
#define HCR_COMMON ( HCR_TWI | HCR_TWE | HCR_VM | HCR_RW | HCR_AMO | HCR_IMO | HCR_FMO )

/* Allow native tasks to run at EL0, but restrict access */
#define HCR_NATIVE ( HCR_COMMON | HCR_TGE | HCR_TVM | HCR_TTLB | HCR_DC \
                   | HCR_TAC | HCR_SWIO |  HCR_TSC | HCR_IMO | HCR_FMO | HCR_AMO)
#define HCR_VCPU   ( HCR_COMMON)

#define SCTLR_EL1_UCI   BIT(26)     /* Enable EL0 access to DC CVAU, DC CIVAC, DC CVAC,
                                       and IC IVAU in AArch64 state   */
#define SCTLR_EL1_C     BIT(2)      /* Enable data and unified caches */
#define SCTLR_EL1_I     BIT(12)     /* Enable instruction cache       */
/* Disable MMU, SP alignment check, and alignment check */
/* A57 default value */
#define SCTLR_EL1_NATIVE   (0x34d58820 | SCTLR_EL1_C | SCTLR_EL1_I | SCTLR_EL1_UCI)
#define SCTLR_EL1_VM       0x34d58820
#define SCTLR_DEFAULT      SCTLR_EL1_NATIVE

/* for EL1 SCTLR */
static inline word_t
getSCTLR(void)
{
    return readSystemControlRegister();
}

static inline void
setSCTLR(word_t sctlr)
{
    writeSystemControlRegister(sctlr);
}

static inline word_t
readTTBR0(void)
{
    word_t reg;
    MRS("ttbr0_el1", reg);
    return reg;
}

static inline void
writeTTBR0(word_t reg)
{
    MSR("ttbr0_el1", reg);
}

static inline word_t
readTTBR1(void)
{
    word_t reg;
    MRS("ttbr1_el1", reg);
    return reg;
}

static inline void
writeTTBR1(word_t reg)
{
    MSR("ttbr1_el1", reg);
}

static inline word_t
readTCR(void)
{
    word_t reg;
    MRS("tcr_el1", reg);
    return reg;
}

static inline void
writeTCR(word_t reg)
{
    MSR("tcr_el1", reg);
}

static inline word_t
readMAIR(void)
{
    word_t reg;
    MRS("mair_el1", reg);
    return reg;
}

static inline void
writeMAIR(word_t reg)
{
    MSR("mair_el1", reg);
}

static inline word_t
readAMAIR(void)
{
    word_t reg;
    MRS("amair_el1", reg);
    return reg;
}

static inline void
writeAMAIR(word_t reg)
{
    MSR("amair_el1", reg);
}

static inline word_t
readCIDR(void)
{
    uint32_t reg;
    MRS("contextidr_el1", reg);
    return (word_t)reg;
}

static inline void
writeCIDR(word_t reg)
{
    MSR("contextidr_el1", (uint32_t)reg);
}

static inline word_t
readACTLR(void)
{
    word_t reg;
    MRS("actlr_el1", reg);
    return reg;
}

static inline void
writeACTLR(word_t reg)
{
    MSR("actlr_el1", reg);
}

static inline word_t
readAFSR0(void)
{
    uint32_t reg;
    MRS("afsr0_el1", reg);
    return (word_t)reg;
}

static inline void
writeAFSR0(word_t reg)
{
    MSR("afsr0_el1", (uint32_t)reg);
}

static inline word_t
readAFSR1(void)
{
    uint32_t reg;
    MRS("afsr1_el1", reg);
    return (word_t)reg;
}

static inline void
writeAFSR1(word_t reg)
{
    MSR("afsr1_el1", (uint32_t)reg);
}

static inline word_t
readESR(void)
{
    uint32_t reg;
    MRS("esr_el1", reg);
    return (word_t)reg;
}

static inline void
writeESR(word_t reg)
{
    MSR("esr_el1", (uint32_t)reg);
}

static inline word_t
readFAR(void)
{
    word_t reg;
    MRS("far_el1", reg);
    return reg;
}

static inline void
writeFAR(word_t reg)
{
    MSR("far_el1", (uint32_t)reg);
}

/* ISR is read-only */
static inline word_t
readISR(void)
{
    uint32_t reg;
    MRS("isr_el1", reg);
    return (word_t)reg;
}

static inline word_t
readVBAR(void)
{
    word_t reg;
    MRS("vbar_el1", reg);
    return reg;
}

static inline void
writeVBAR(word_t reg)
{
    MSR("vbar_el1", reg);
}

static inline word_t
readTPIDR_EL0(void)
{
    word_t reg;
    MRS("TPIDR_EL0", reg);
    return reg;
}

static inline void
writeTPIDR_EL0(word_t reg)
{
    MSR("TPIDR_EL0", reg);
}

static inline word_t
readTPIDR_EL1(void)
{
    word_t reg;
    MRS("TPIDR_EL1", reg);
    return reg;
}

static inline void
writeTPIDR_EL1(word_t reg)
{
    MSR("TPIDR_EL1", reg);
}

static inline word_t
readTPIDRRO_EL0(void)
{
    word_t reg;
    MRS("TPIDRRO_EL0", reg);
    return reg;
}

static inline void
writeTPIDRRO_EL0(word_t reg)
{
    MSR("TPIDRRO_EL0", reg);
}

static inline word_t
readSP_EL1(void)
{
    word_t reg;
    MRS("sp_el1", reg);
    return reg;
}

static inline void
writeSP_EL1(word_t reg)
{
    MSR("sp_el1", reg);
}

static inline word_t
readELR_EL1(void)
{
    word_t reg;
    MRS("elr_el1", reg);
    return reg;
}

static inline void
writeELR_EL1(word_t reg)
{
    MRS("elr_el1", reg);
}

static inline word_t
readSPSR_EL1(void)
{
    word_t reg;
    MRS("spsr_el1", reg);
    return reg;
}

static inline void
writeSPSR_EL1(word_t reg)
{
    MSR("spsr_el1", reg);
}

static inline word_t
readCPACR_EL1(void)
{
    word_t reg;
    MRS("cpacr_el1", reg);
    return reg;
}

static inline void
writeCPACR_EL1(word_t reg)
{
    MSR("cpacr_el1", reg);
}

static inline word_t
readCNTV_TVAL_EL0(void)
{
    word_t reg;
    MRS("cntv_tval_el0", reg);
    return reg;
}

static inline void
writeCNTV_TVAL_EL0(word_t reg)
{
    MSR("cntv_tval_el0", reg);
}

static inline word_t
readCNTV_CTL_EL0(void)
{
    word_t reg;
    MRS("cntv_ctl_el0", reg);
    return reg;
}

static inline void
writeCNTV_CTL_EL0(word_t reg)
{
    MSR("cntv_ctl_el0", reg);
}

#endif /* End of CONFIG_ARM_HYPERVISOR_SUPPORT */

#endif

