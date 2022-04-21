/*
 * Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)
 * Copyright 2015, 2016 Hesham Almatary <heshamelmatary@gmail.com>
 * Copyright 2021, HENSOLDT Cyber
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include <config.h>
#include <assert.h>
#include <kernel/boot.h>
#include <machine/io.h>
#include <model/statedata.h>
#include <object/interrupt.h>
#include <arch/machine.h>
#include <arch/kernel/boot.h>
#include <arch/kernel/vspace.h>
#include <arch/benchmark.h>
#include <linker.h>
#include <plat/machine/hardware.h>
#include <machine.h>

#ifdef ENABLE_SMP_SUPPORT
BOOT_BSS static volatile word_t node_boot_lock;
#endif

/*CY 保留区域，rv就是3块：kernel， dtb， user image */
BOOT_BSS static region_t res_reg[NUM_RESERVED_REGIONS];

BOOT_CODE cap_t create_mapped_it_frame_cap(cap_t pd_cap, pptr_t pptr, vptr_t vptr, asid_t asid, bool_t
                                           use_large, bool_t executable)
{
    cap_t cap;
    vm_page_size_t frame_size;

    if (use_large) {
        frame_size = RISCV_Mega_Page;
    } else {
        frame_size = RISCV_4K_Page;
    }

    cap = cap_frame_cap_new(
              asid,                            /* capFMappedASID       */
              pptr,                            /* capFBasePtr          */
              frame_size,                      /* capFSize             */
              wordFromVMRights(VMReadWrite),   /* capFVMRights         */
              0,                               /* capFIsDevice         */
              vptr                             /* capFMappedAddress    */
          );

    map_it_frame_cap(pd_cap, cap);
    return cap;
}

/*CY 初始化可用的物理内存区域 */
BOOT_CODE static bool_t arch_init_freemem(region_t ui_reg,
                                          p_region_t dtb_p_reg,
                                          v_region_t it_v_reg,
                                          word_t extra_bi_size_bits)
{
    /* Reserve the kernel image region. This may look a bit awkward, as the
     * symbols are a reference in the kernel image window, but all allocations
     * are done in terms of the main kernel window, so we do some translation.
     */
    /*CY 保留区域第一块是kernel */
    res_reg[0].start = (pptr_t)paddr_to_pptr(kpptr_to_paddr((void *)KERNEL_ELF_BASE)); /*CY 将Kernel ELF那段的虚拟地址转换为PSpace那段所对应的Kernel ELF的虚拟地址，下同。意思就是要访问物理内存一律通过PSpace */
    res_reg[0].end = (pptr_t)paddr_to_pptr(kpptr_to_paddr((void *)ki_end));

    int index = 1;

    /* add the dtb region, if it is not empty */
    if (dtb_p_reg.start) {
        if (index >= ARRAY_SIZE(res_reg)) {
            printf("ERROR: no slot to add DTB to reserved regions\n");
            return false;
        }
        res_reg[index] = paddr_to_pptr_reg(dtb_p_reg);
        index += 1;
    }

    /* reserve the user image region */
    if (index >= ARRAY_SIZE(res_reg)) {
        printf("ERROR: no slot to add user image to reserved regions\n");
        return false;
    }
    res_reg[index] = ui_reg;
    index += 1;

    /* avail_p_regs comes from the auto-generated code */
    /*CY 去掉保留区域用掉的物理内存，剩余可用的物理内存 */
    return init_freemem(ARRAY_SIZE(avail_p_regs), avail_p_regs,
                        index, res_reg, /*CY 这里index也就是保留区域的个数 */
                        it_v_reg, extra_bi_size_bits);
}

BOOT_CODE static void init_irqs(cap_t root_cnode_cap)
{
    irq_t i;

    for (i = 0; i <= maxIRQ; i++) {//QT 设置每个irq号的状态为不活跃，maxIRQ=1，除了非法irq号
        if (i != irqInvalid) {
            /* IRQ 0 is irqInvalid */
            setIRQState(IRQInactive, i);//设置时间中断是不活跃状态
        }
    }
    setIRQState(IRQTimer, KERNEL_TIMER_IRQ);//单独分配IRQTimer状态
#ifdef ENABLE_SMP_SUPPORT
    setIRQState(IRQIPI, irq_remote_call_ipi);
    setIRQState(IRQIPI, irq_reschedule_ipi);
#endif
    /* provide the IRQ control cap */
    write_slot(SLOT_PTR(pptr_of_cap(root_cnode_cap), seL4_CapIRQControl), cap_irq_control_cap_new());
}

/* ASM symbol for the CPU initialisation trap. */
/*CY 内陷处理函数定义在src/arch/riscv/traps.S */
extern char trap_entry[1];

/* This and only this function initialises the CPU. It does NOT initialise any kernel state. */

#ifdef CONFIG_HAVE_FPU
BOOT_CODE static void init_fpu(void)
{
    set_fs_clean();
    write_fcsr(0);
    disableFpu();
}
#endif

BOOT_CODE static void init_cpu(void)
{
    /*CY 激活内核虚拟空间，实际上就是设置页表寄存器*/
    activate_kernel_vspace();
    /* Write trap entry address to stvec *//*QT 把内陷函数入口地址写入stvec寄存器*/
    /*CY stvec寄存器中存放的是内陷处理函数入口地址 */
    write_stvec((word_t)trap_entry);
    /*CY 初始化中断请求控制器 */
    initLocalIRQController();
    /*QT 中断分为本地中断和全局中断。在本地中断里设置软中断和定时器中断，全局中断也可以理解为外设中断*/
#ifndef CONFIG_KERNEL_MCS
    initTimer();
#endif

    /* disable FPU access */
    set_fs_off();

#ifdef CONFIG_HAVE_FPU
    init_fpu();
#endif
}

/* This and only this function initialises the platform. It does NOT initialise any kernel state. */

BOOT_CODE static void init_plat(void)
{
    initIRQController();
}


#ifdef ENABLE_SMP_SUPPORT
BOOT_CODE static bool_t try_init_kernel_secondary_core(word_t hart_id, word_t core_id)
{
    while (!node_boot_lock);

    fence_r_rw();

    init_cpu();
    NODE_LOCK_SYS;

    ksNumCPUs++;
    init_core_state(SchedulerAction_ResumeCurrentThread);
    ifence_local();
    return true;
}

BOOT_CODE static void release_secondary_cores(void)
{
    node_boot_lock = 1;
    fence_w_r();

    while (ksNumCPUs != CONFIG_MAX_NUM_NODES) {
        __atomic_signal_fence(__ATOMIC_ACQ_REL);
    }
}

#endif
/* Main kernel initialisation function. */

static BOOT_CODE bool_t try_init_kernel(
    paddr_t ui_p_reg_start,
    paddr_t ui_p_reg_end,
    sword_t pv_offset,
    vptr_t  v_entry,
    paddr_t dtb_phys_addr,
    word_t  dtb_size
)
{
    cap_t root_cnode_cap;
    cap_t it_pd_cap;
    cap_t it_ap_cap;
    cap_t ipcbuf_cap; 
    /*CY 内核启动代码的物理位置 x86_64

    kpptr_to_paddr是将内核代码对应的虚拟地址转化为实际物理地址，
    这里是 KERNEL_ELF_BASE - KERNEL_ELF_BASE_OFFSET，
    其中KERNEL_ELF_BASE_OFFSET（0xffffffff80000000） = KERNEL_ELF_BASE(0xffffffff80100000) - KERNEL_ELF_PADDR_BASE（0x00100000），
    KERNEL_ELF_BASE(0xffffffff80100000) = PPTR_TOP（0xffffffff80000000）+ KERNEL_ELF_PADDR_BASE（0x00100000）,
    KERNEL_ELF_BASE_OFFSET也就是PPTR_TOP
    */
    /**qtliu rv
     * KERNEL_ELF_BASE_OFFSET(0xFFFFFFFF0000_0000)=KERNEL_ELF_BASE(0xFFFFFFFF8400_0000)-KERNEL_ELF_PADDR_BASE(0x8400_0000)
     * kpptr_to_paddr((void *)KERNEL_ELF_BASE)=KERNEL_ELF_BASE(0xFFFFFFFF8400_0000) - KERNEL_ELF_BASE_OFFSET(0xFFFFFFFF0000_0000)=0x8400_0000
     * kpptr_to_paddr(ki_boot_end)=ki_boot_end(lds文件中的内核镜像结束位置)-KERNEL_ELF_BASE_OFFSET(0xFFFFFFFF0000_0000)=?
     * */
    p_region_t boot_mem_reuse_p_reg = ((p_region_t) {
        kpptr_to_paddr((void *)KERNEL_ELF_BASE), kpptr_to_paddr(ki_boot_end)
    });
    /*CY 内核启动代码的虚拟位置

    物理地址 + PPTR_BASE_OFFSET 
    PPTR_BASE_OFFSET = PPTR_BASE（0xffffff8000000000） - PADDR_BASE（0x00000000）
    */
    region_t boot_mem_reuse_reg = paddr_to_pptr_reg(boot_mem_reuse_p_reg);
    /*CY user image的虚拟位置 */
    region_t ui_reg = paddr_to_pptr_reg((p_region_t) {
        ui_p_reg_start, ui_p_reg_end
    });
    word_t extra_bi_size = 0;
    pptr_t extra_bi_offset = 0;
    vptr_t extra_bi_frame_vptr;
    vptr_t bi_frame_vptr;
    vptr_t ipcbuf_vptr;
    create_frames_of_region_ret_t create_frames_ret;
    create_frames_of_region_ret_t extra_bi_ret;

    /* convert from physical addresses to userland vptrs */
    v_region_t ui_v_reg = {
        .start = ui_p_reg_start - pv_offset,
        .end   = ui_p_reg_end   - pv_offset
    };

    /*CY ipcbuffer跟在user image后面 */
    ipcbuf_vptr = ui_v_reg.end;
    /* bootinfo跟在ipcbuffer后面，ipcbuffer占一个页 */
    bi_frame_vptr = ipcbuf_vptr + BIT(PAGE_BITS);
    /* bootinfo的extra部分跟在bootinfo后面，bootinfo占一个页 */
    extra_bi_frame_vptr = bi_frame_vptr + BIT(BI_FRAME_SIZE_BITS);

    /*CY 将内核代码映射到实际的物理区域 */
    map_kernel_window();

    /* initialise the CPU */
    init_cpu();

    printf("Bootstrapping kernel\n");

    /* initialize the platform */
    init_plat();

    /* If a DTB was provided, pass the data on as extra bootinfo */
    p_region_t dtb_p_reg = P_REG_EMPTY;
    if (dtb_size > 0) {
        paddr_t dtb_phys_end = dtb_phys_addr + dtb_size;
        if (dtb_phys_end < dtb_phys_addr) {
            /* An integer overflow happened in DTB end address calculation, the
             * location or size passed seems invalid.
             */
            printf("ERROR: DTB location at %"SEL4_PRIx_word
                   " len %"SEL4_PRIu_word" invalid\n",
                   dtb_phys_addr, dtb_size);
            return false;
        }
        /* If the DTB is located in physical memory that is not mapped in the
         * kernel window we cannot access it.
         */
        if (dtb_phys_end >= PADDR_TOP) {
            printf("ERROR: DTB at [%"SEL4_PRIx_word"..%"SEL4_PRIx_word"] "
                   "exceeds PADDR_TOP (%"SEL4_PRIx_word")\n",
                   dtb_phys_addr, dtb_phys_end, PADDR_TOP);
            return false;
        }
        /* DTB seems valid and accessible, pass it on in bootinfo. */
        extra_bi_size += sizeof(seL4_BootInfoHeader) + dtb_size;
        /* Remember the memory region it uses. */
        dtb_p_reg = (p_region_t) {
            .start = dtb_phys_addr,
            .end   = dtb_phys_end
        };
    }

    /* The region of the initial thread is the user image + ipcbuf + boot info + extra */
    word_t extra_bi_size_bits = calculate_extra_bi_size_bits(extra_bi_size);  /*CY extra_bi_size来自于DTB，不然这个值就是0，这里暂时先不管 */
    /*CY initial thread的虚拟空间 */
    v_region_t it_v_reg = {
        .start = ui_v_reg.start,
        .end   = extra_bi_frame_vptr + BIT(extra_bi_size_bits)
    };
    /*CY 检查一下initial thread是否超出用户虚拟空间 */
    if (it_v_reg.end >= USER_TOP) {
        /* Variable arguments for printf() require well defined integer types
         * to work properly. Unfortunately, the definition of USER_TOP differs
         * between platforms (int, long), so we have to cast here to play safe.
         */
        printf("ERROR: userland image virt [%"SEL4_PRIx_word"..%"SEL4_PRIx_word"]"
               "exceeds USER_TOP (%"SEL4_PRIx_word")\n",
               it_v_reg.start, it_v_reg.end, (word_t)USER_TOP);
        return false;
    }

    /* make the free memory available to alloc_region() */
    /*CY 初始化可用的物理内存区域 */
    if (!arch_init_freemem(ui_reg, dtb_p_reg, it_v_reg, extra_bi_size_bits)) {
        printf("ERROR: free memory management initialization failed\n");
        return false;
    }

    /* create the root cnode */
    root_cnode_cap = create_root_cnode();
    if (cap_get_capType(root_cnode_cap) == cap_null_cap) {
        printf("ERROR: root c-node creation failed\n");
        return false;
    }

    /* create the cap for managing thread domains */
    create_domain_cap(root_cnode_cap);

    /* initialise the IRQ states and provide the IRQ control cap */
    /*QT 可能与中断有关*/
    init_irqs(root_cnode_cap);

    /* create the bootinfo frame */
    populate_bi_frame(0, CONFIG_MAX_NUM_NODES, ipcbuf_vptr, extra_bi_size);

    /* put DTB in the bootinfo block, if present. */
    seL4_BootInfoHeader header;
    if (dtb_size > 0) {
        header.id = SEL4_BOOTINFO_HEADER_FDT;
        header.len = sizeof(header) + dtb_size;
        *(seL4_BootInfoHeader *)(rootserver.extra_bi + extra_bi_offset) = header;
        extra_bi_offset += sizeof(header);
        memcpy((void *)(rootserver.extra_bi + extra_bi_offset),
               paddr_to_pptr(dtb_phys_addr),
               dtb_size);
        extra_bi_offset += dtb_size;
    }

    if (extra_bi_size > extra_bi_offset) {
        /* provide a chunk for any leftover padding in the extended boot info */
        header.id = SEL4_BOOTINFO_HEADER_PADDING;
        header.len = (extra_bi_size - extra_bi_offset);
        *(seL4_BootInfoHeader *)(rootserver.extra_bi + extra_bi_offset) = header;
    }

    /* Construct an initial address space with enough virtual addresses
     * to cover the user image + ipc buffer and bootinfo frames */
    it_pd_cap = create_it_address_space(root_cnode_cap, it_v_reg);
    if (cap_get_capType(it_pd_cap) == cap_null_cap) {
        printf("ERROR: address space creation for initial thread failed\n");
        return false;
    }

    /* Create and map bootinfo frame cap */
    create_bi_frame_cap(
        root_cnode_cap,
        it_pd_cap,
        bi_frame_vptr
    );

    /* create and map extra bootinfo region */
    if (extra_bi_size > 0) {
        region_t extra_bi_region = {
            .start = rootserver.extra_bi,
            .end = rootserver.extra_bi + extra_bi_size
        };
        extra_bi_ret =
            create_frames_of_region(
                root_cnode_cap,
                it_pd_cap,
                extra_bi_region,
                true,
                pptr_to_paddr((void *)extra_bi_region.start) - extra_bi_frame_vptr
            );
        if (!extra_bi_ret.success) {
            printf("ERROR: mapping extra boot info to initial thread failed\n");
            return false;
        }
        ndks_boot.bi_frame->extraBIPages = extra_bi_ret.region;
    }

#ifdef CONFIG_KERNEL_MCS
    init_sched_control(root_cnode_cap, CONFIG_MAX_NUM_NODES);
#endif

    /* create the initial thread's IPC buffer */
    ipcbuf_cap = create_ipcbuf_frame_cap(root_cnode_cap, it_pd_cap, ipcbuf_vptr);
    if (cap_get_capType(ipcbuf_cap) == cap_null_cap) {
        printf("ERROR: could not create IPC buffer for initial thread\n");
        return false;
    }

    /* create all userland image frames */
    create_frames_ret =
        create_frames_of_region(
            root_cnode_cap,
            it_pd_cap,
            ui_reg,
            true,
            pv_offset
        );
    if (!create_frames_ret.success) {
        printf("ERROR: could not create all userland image frames\n");
        return false;
    }
    ndks_boot.bi_frame->userImageFrames = create_frames_ret.region;

    /* create the initial thread's ASID pool */
    it_ap_cap = create_it_asid_pool(root_cnode_cap);
    if (cap_get_capType(it_ap_cap) == cap_null_cap) {
        printf("ERROR: could not create ASID pool for initial thread\n");
        return false;
    }
    write_it_asid_pool(it_ap_cap, it_pd_cap);

#ifdef CONFIG_KERNEL_MCS
    NODE_STATE(ksCurTime) = getCurrentTime();
#endif

    /* create the idle thread */
    if (!create_idle_thread()) {
        printf("ERROR: could not create idle thread\n");
        return false;
    }

    /* create the initial thread */
    tcb_t *initial = create_initial_thread(
                         root_cnode_cap,
                         it_pd_cap,
                         v_entry,
                         bi_frame_vptr,
                         ipcbuf_vptr,
                         ipcbuf_cap
                     );

    if (initial == NULL) {
        printf("ERROR: could not create initial thread\n");
        return false;
    }

    init_core_state(initial);

    /* convert the remaining free memory into UT objects and provide the caps */
    if (!create_untypeds(
            root_cnode_cap,
            boot_mem_reuse_reg)) {
        printf("ERROR: could not create untypteds for kernel image boot memory\n");
        return false;
    }

    /* no shared-frame caps (RISC-V has no multikernel support) */
    ndks_boot.bi_frame->sharedFrames = S_REG_EMPTY;

    /* finalise the bootinfo frame */
    bi_finalise();

    ksNumCPUs = 1;

    SMP_COND_STATEMENT(clh_lock_init());
    SMP_COND_STATEMENT(release_secondary_cores());

    /* All cores are up now, so there can be concurrency. The kernel booting is
     * supposed to be finished before the secondary cores are released, all the
     * primary has to do now is schedule the initial thread. Currently there is
     * nothing that touches any global data structures, nevertheless we grab the
     * BKL here to play safe. It is released when the kernel is left. */
    NODE_LOCK_SYS;

    printf("Booting all finished, dropped to user space\n");
    return true;
}
  /*qtliu 这段注释复制自head.S 
   *    Call bootstrapping implemented in C with parameters:
   *    a0/x10: user image physical start address
   *    a1/x11: user image physical end address
   *    a2/x12: physical/virtual offset
   *    a3/x13: user image virtual entry address
   *    a4/x14: DTB physical address (0 if there is none)
   *    a5/x15: DTB size (0 if there is none)
   * 
   *    ifdef SMP
   *    a6/x16: hart ID (SMP only, unused on non-SMP)
   *    a7/x17: core ID (SMP only, unused on non-SMP)
   *    endif 
   */
BOOT_CODE VISIBLE void init_kernel(
    paddr_t ui_p_reg_start,
    paddr_t ui_p_reg_end,
    sword_t pv_offset,
    vptr_t  v_entry,
    paddr_t dtb_addr_p,
    uint32_t dtb_size
#ifdef ENABLE_SMP_SUPPORT
    ,
    word_t hart_id,
    word_t core_id
#endif
)
{
    bool_t result;

#ifdef ENABLE_SMP_SUPPORT
    add_hart_to_core_map(hart_id, core_id);
    if (core_id == 0) {
        result = try_init_kernel(ui_p_reg_start,
                                 ui_p_reg_end,
                                 pv_offset,
                                 v_entry,
                                 dtb_addr_p,
                                 dtb_size);
    } else {
        result = try_init_kernel_secondary_core(hart_id, core_id);
    }
#else
    result = try_init_kernel(ui_p_reg_start,
                             ui_p_reg_end,
                             pv_offset,
                             v_entry,
                             dtb_addr_p,
                             dtb_size);
#endif
    if (!result) {
        fail("ERROR: kernel init failed");
        UNREACHABLE();
    }

#ifdef CONFIG_KERNEL_MCS
    NODE_STATE(ksCurTime) = getCurrentTime();
    NODE_STATE(ksConsumed) = 0;
#endif

    schedule();
    activateThread();
}
