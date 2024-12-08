#include <sys/cdefs.h>
#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/systm.h>
#include <sys/proc.h>
#include <machine/bus.h>
#include <machine/intr.h>
#include <dev/fdt/fdt_common.h>
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include "pic_if.h"

#define PIC_CHIP_IRQ_STATUS (0xc)
#define PIC_EXTRA_CHIP_IRQ_STATUS (0x8)
#define PIC_CHIP_IRQ_MASK (0x4)
#define PIC_EXTRA_CHIP_IRQ_MASK (0x0)

struct bcmpic_isrc_entry {
	struct intr_irqsrc isrc;
	uint32_t irq;
};

struct bcmpic_softc {
	struct resource* sc_res[2];
	struct bcmpic_isrc_entry sc_irqs[64];
	void* sc_cookie;
	uint32_t sc_irq_count;
	device_t sc_dev;
};

static struct resource_spec bcmpic_spec[] = {
	{SYS_RES_MEMORY, 0, RF_ACTIVE},
	{SYS_RES_IRQ, 0, RF_ACTIVE},
	{-1, 0}
};

static void pic_disable_irq(struct bcmpic_softc* sc, uint32_t irq) {
	uint32_t reg = (0);
	if(irq >= 32) {
		irq -= 32;
		reg = bus_read_4(sc->sc_res[0], PIC_EXTRA_CHIP_IRQ_MASK);
		reg &= (~(1<<irq));
		bus_write_4(sc->sc_res[0], PIC_EXTRA_CHIP_IRQ_MASK, reg);
	} else {
		reg = bus_read_4(sc->sc_res[0], PIC_CHIP_IRQ_MASK);
		reg &= (~(1<<irq));
		bus_write_4(sc->sc_res[0], PIC_CHIP_IRQ_MASK, reg);
	}
}

static void pic_enable_irq(struct bcmpic_softc* sc, uint32_t irq) {
	uint32_t reg = (0);
	if(irq >= 32) {
		irq -= 32;
		reg = bus_read_4(sc->sc_res[0], PIC_EXTRA_CHIP_IRQ_MASK);
		reg |= (1<<irq);
		bus_write_4(sc->sc_res[0], PIC_EXTRA_CHIP_IRQ_MASK, reg);
	} else {
		reg = bus_read_4(sc->sc_res[0], PIC_CHIP_IRQ_MASK);
		reg |= (1<<irq);
		bus_write_4(sc->sc_res[0], PIC_CHIP_IRQ_MASK, reg);
	}
}

static int bcmpic_intr(void* arg) {
	struct bcmpic_softc* sc = arg;
	uint32_t extra_status = (0),
			 status = (0),
			 i = (0);
	curthread->td_intr_nesting_level--;
	status = bus_read_4(sc->sc_res[0], PIC_CHIP_IRQ_STATUS);
	extra_status = bus_read_4(sc->sc_res[0], PIC_EXTRA_CHIP_IRQ_STATUS);
	status &= bus_read_4(sc->sc_res[0], PIC_CHIP_IRQ_MASK);
	extra_status &= bus_read_4(sc->sc_res[0], PIC_EXTRA_CHIP_IRQ_MASK);
	for(; ((i = fls(status)) != 0);) {
		i--;
		status &= (~(1<<i));
		if(intr_isrc_dispatch(&sc->sc_irqs[i].isrc,
							  curthread->td_intr_frame) != 0) {
			device_printf(sc->sc_dev, "stray irq %u detected\n", i);
			pic_disable_irq(sc, i);
			continue;
		}
	}
	for(; ((i = fls(extra_status)) != 0);) {
		i--;
		extra_status &= (~(1<<i));
		if(intr_isrc_dispatch(&sc->sc_irqs[(32+i)].isrc,
							  curthread->td_intr_frame) != 0) {
			device_printf(sc->sc_dev, "stray irq %u detected\n", (32+i));
			pic_disable_irq(sc, (32+i));
			continue;
		}
	}
	curthread->td_intr_nesting_level++;
	return (FILTER_HANDLED);
}

static int bcmpic_probe(device_t dev) {
	if(!ofw_bus_status_okay(dev) ||
			!ofw_bus_is_compatible(dev, "brcm,bcm63xx-pic"))
		return (ENXIO);
	device_set_desc(dev, "Broadcom BCM63xx ChipIRQ");
	return (0);
}

static int bcmpic_attach(device_t dev) {
	struct bcmpic_softc* sc = NULL;
	uint32_t i = (0);
	intptr_t xref = (0);
	int ret = (0);
	const char* dev_name = NULL;
	sc = device_get_softc(dev);
	xref = OF_xref_from_node(ofw_bus_get_node(dev));
	dev_name = device_get_nameunit(dev);
	if(bus_alloc_resources(dev, bcmpic_spec, sc->sc_res) != 0) {
		device_printf(dev, "could not alloc resources\n");
		return (ENXIO);
	}
	sc->sc_dev = dev;
	sc->sc_irq_count = (64);
	for(i = 0; i < sc->sc_irq_count; i++) {
		if(i <= 8)
			sc->sc_irqs[i].irq = (i+8);
		else
			sc->sc_irqs[i].irq = i;
		ret = intr_isrc_register(
				  &sc->sc_irqs[i].isrc,
				  dev,
				  0,
				  "%s",
				  dev_name
			  );
		if(ret != 0)
			panic("bcmpic_attach: int_isrc_register failed");
	}
	bus_write_4(sc->sc_res[0], PIC_CHIP_IRQ_MASK, 0);
	bus_write_4(sc->sc_res[0], PIC_EXTRA_CHIP_IRQ_MASK, 0);
	if(!intr_pic_register(dev, xref)) {
		device_printf(dev, "failed to register pic\n");
		return (ENXIO);
	}
	if(bus_setup_intr(dev, sc->sc_res[1], INTR_TYPE_MISC,
					  bcmpic_intr, NULL, sc, &sc->sc_cookie) != 0) {
		intr_pic_deregister(dev, xref);
		bus_release_resources(dev, bcmpic_spec, sc->sc_res);
		return (ENXIO);
	}
	return (0);
}

static void bcmpic_enable_intr(device_t dev, struct intr_irqsrc* isrc) {
	pic_enable_irq(device_get_softc(dev), ((struct bcmpic_isrc_entry*)(isrc))->irq);
}

static void bcmpic_disable_intr(device_t dev, struct intr_irqsrc* isrc) {
	pic_disable_irq(device_get_softc(dev), ((struct bcmpic_isrc_entry*)(isrc))->irq);
}

static int bcmpic_map_intr(device_t dev, struct intr_map_data* data,
						   struct intr_irqsrc** isrc) {
	struct bcmpic_softc* sc = NULL;
	struct intr_map_data_fdt* fdata = NULL;
	uint32_t i = (0);
	sc = device_get_softc(dev);
	fdata = (struct intr_map_data_fdt*)(data);
	if(!data || (data->type != INTR_MAP_DATA_FDT)
			|| (fdata->ncells != 1))
		return (EINVAL);
	for(i = 0; i < sc->sc_irq_count; i++) {
		if(sc->sc_irqs[i].irq == fdata->cells[0]) {
			*isrc = &sc->sc_irqs[i].isrc;
			return (0);
		}
	}
	return (EINVAL);
}

static void bcmpic_post_ithread(device_t dev, struct intr_irqsrc* isrc) {
	pic_enable_irq(device_get_softc(dev), ((struct bcmpic_isrc_entry*)(isrc))->irq);
}

static void bcmpic_pre_ithread(device_t dev, struct intr_irqsrc* isrc) {
	pic_disable_irq(device_get_softc(dev), ((struct bcmpic_isrc_entry*)(isrc))->irq);
}

static device_method_t bcmpic_fdt_methods[] = {
	DEVMETHOD(device_probe, bcmpic_probe),
	DEVMETHOD(device_attach, bcmpic_attach),
	DEVMETHOD(pic_enable_intr, bcmpic_enable_intr),
	DEVMETHOD(pic_disable_intr, bcmpic_disable_intr),
	DEVMETHOD(pic_map_intr, bcmpic_map_intr),
	DEVMETHOD(pic_post_ithread, bcmpic_post_ithread),
	DEVMETHOD(pic_pre_ithread, bcmpic_pre_ithread),
	DEVMETHOD_END
};

static devclass_t bcmpic_devclass;

static driver_t bcmpic_driver = {
	"bcmpic",
	bcmpic_fdt_methods,
	sizeof(struct bcmpic_softc)
};

EARLY_DRIVER_MODULE(bcmpic, simplebus, bcmpic_driver, bcmpic_devclass, 0, 0,
					BUS_PASS_INTERRUPT + BUS_PASS_ORDER_MIDDLE);
