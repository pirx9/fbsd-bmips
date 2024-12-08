#include <sys/types.h>
#include <sys/systm.h>
#include <dev/fdt/fdt_common.h>
#include <dev/ofw/openfirm.h>
#include "bcm63xx.h"
#include "bcm63xx_reg.h"

uint64_t bcm63xx_get_cpu_freq() {
	phandle_t root_node = (0);
	uint32_t v = (0), 
			 pll_fcvo = (0);
	if((root_node = OF_finddevice("/")) == -1)
		panic("bcm63xx_get_cpu_freq: failed to get root node");
	if(ofw_bus_node_is_compatible(root_node, "brcm,bcm6328")) {
		v = *(volatile uint32_t*)(REG_BCM6328_MISC_BASE+0x240);
		pll_fcvo = (v&(0x1f<<7));
		switch(pll_fcvo) {
			case 0x12:
			case 0x14:
			case 0x19:
				return (160000000);
			case 0x1c:
				return (192000000);
			case 0x13:
			case 0x15:
				return (200000000);
			case 0x1a:
				return (384000000);
			case 0x16:
				return (400000000);
			default:
				return (320000000);
		}
	}
	panic("bcm63xx_get_cpu_freq: failed to determine freq");
}

void bcm63xx_reset() {
	phandle_t root_node = (0);
	if((root_node = OF_finddevice("/")) == -1)
		panic("bcm63xx_reset: failed to get root node");
	printf("bcm63xx_reset: resetting board!\n\n");
	if(ofw_bus_node_is_compatible(root_node, "brcm,bcm6328"))
		*((volatile uint32_t*)(0xb0000068)) = (0x1);
	for(;;);
}
