#include <sys/cdefs.h>
#include <sys/types.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/boot.h>
#include <sys/cons.h>
#include <sys/mutex.h>
#include <sys/kdb.h>
#include <vm/vm.h>
#include <vm/vm_param.h>
#include <vm/vm_object.h>
#include <vm/vm_page.h>
#include <vm/vm_phys.h>
#include <machine/bus.h>
#include <machine/hwfunc.h>
#include <machine/md_var.h>
#include <machine/cpuinfo.h>
#include <machine/clock.h>
#include <machine/cache.h>
#include <dev/uart/uart.h>
#include <dev/uart/uart_bus.h>
#include <dev/uart/uart_cpu.h>
#include "opt_platform.h"
#include <dev/fdt/fdt_common.h>
#include <dev/ofw/openfirm.h>
#include "bcm63xx.h"
#include "bcm63xx_reg.h"

static int* edata, *end;
static char boot1_env[(1024*4)];

static void mips_init(void) {
	struct mem_region mr[FDT_MEM_REGIONS];
	uint64_t v = (0);
	int mr_count = (0), i = (0);
	if(fdt_get_mem_regions(mr, &mr_count, &v) != 0)
		panic("mips_init: failed to get memory info from fdt");
	realmem = btoc(v);
	for(i = 0; i < PHYS_AVAIL_ENTRIES; i++)
		phys_avail[i] = (0);
	physmem = realmem;
	phys_avail[0] = MIPS_KSEG0_TO_PHYS(kernel_kseg0_end);
	phys_avail[1] = ctob(realmem);
	dump_avail[0] = phys_avail[0];
	dump_avail[1] = (phys_avail[1]-dump_avail[0]);
	init_param1();
	init_param2(physmem);
	mips_cpu_init();
	pmap_bootstrap();
	mips_proc0_init();
	mutex_init();
	kdb_init();
}

void platform_start(
	__register_t a0,
	__register_t a1,
	__register_t a2,
	__register_t a3
) {
	vm_offset_t kern_end = (0);
	uint64_t cpu_freq = (0);
	uint32_t r = (0);
	kern_end = (vm_offset_t)(&end);
	memset(&edata, 0, (kern_end-((vm_offset_t)(&edata))));
	bootverbose = (1);
	init_static_kenv(boot1_env, sizeof(boot1_env));
	mips_postboot_fixup();
	mips_pcpu0_init();
	if(!OF_install(OFW_FDT, 0))
		for(;;);
	if(OF_init(&fdt_static_dtb) != 0)
		for(;;);
	cpu_freq = bcm63xx_get_cpu_freq();
	mips_timer_early_init(cpu_freq);
	cninit();
	mips_init();
	mips_timer_init_params(cpu_freq, 0);
	__asm__("mfc0 %0, $22" : "=r" (r));
	printf("bcm63xx: config0 = 0x%x\n", r);
	// PERF->blkEnables |= USBH_CLK_EN;
	*(volatile uint32_t*)(0xb0000004) |= (1<<8);
	*(volatile uint32_t*)(0xb0000010) &= (~(1<<5));
	DELAY(20000);
	*(volatile uint32_t*)(0xb0000010) |= (1<<5);
	DELAY(20000);
	// USBH->SwapControl = EHCI_ENDIAN_SWAP | OHCI_ENDIAN_SWAP;
	*(volatile uint32_t*)(0xb000271c) |= (1<<4|1<<1);
	// .utmictl1_dev_set = USBH_UC1_DEV_MODE_SEL,
	*(volatile uint32_t*)(0xb0002700+0x10) |= (1<<0);
	// USBH->Setup |= USBH_IOC;
	*(volatile uint32_t*)(0xb0002728) |= (1<<4);

	// PERF->blkEnables |= PCIE_CLK_EN;
	*(volatile uint32_t*)(0xb0000004) |= (1<<10);
	// MISC->miscSerdesCtrl |= (SERDES_PCIE_ENABLE|SERDES_PCIE_EXD_ENABLE);
	*(volatile uint32_t*)(0xb0001800) |= ((1<<15)|1);
	// PERF->softResetB &= ~(SOFT_RST_PCIE|SOFT_RST_PCIE_EXT|SOFT_RST_PCIE_CORE);
	*(volatile uint32_t*)(0xb0000000+0x10) &= (~((1<<9)|(1<<8)|(1<<7)));
	// PERF->softResetB &= ~SOFT_RST_PCIE_HARD;
	*(volatile uint32_t*)(0xb0000000+0x10) &= (~(1<<10));
	DELAY(100);
	// PERF->softResetB |= SOFT_RST_PCIE_HARD;
	*(volatile uint32_t*)(0xb0000000+0x10) |= (1<<10);
	DELAY(100);
	// PERF->softResetB |= (SOFT_RST_PCIE|SOFT_RST_PCIE_CORE);
	*(volatile uint32_t*)(0xb0000000+0x10) |= ((1<<8)|(1<<7));
	DELAY(100);
	*(volatile uint32_t*)(0xb0000000+0x10) |= (1<<9);
	DELAY(200);

	// RAC (readahead cache): RAC_D=1, FLH=1
	*(volatile uint32_t*)(0xff400000) = ((1<<1));
}

void platform_cpu_init(void) {}

void platform_reset(void) {
	bcm63xx_reset();
}

#if 0
//#if defined(EARLY_PRINTF)
static void bcm63xx_early_putc(int c) {
	for(; (!(*((volatile uint32_t*)(BCM6328_REG_UART_BASE+0x10))&(1<<5))););
	(*(volatile uint32_t*)(BCM6328_REG_UART_BASE+0x14)) = (c);
	for(; (!(*((volatile uint32_t*)(BCM6328_REG_UART_BASE+0x10))&(1<<5))););
}
early_putc_t* early_putc = &bcm63xx_early_putc;
#endif
