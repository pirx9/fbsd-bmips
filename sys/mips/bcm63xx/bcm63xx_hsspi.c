#include <sys/types.h>
#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/endian.h>
#include <sys/bus.h>
#include <sys/mutex.h>
#include <sys/module.h>
#include <machine/bus.h>
#include <dev/fdt/fdt_common.h>
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/spibus/spi.h>
#include <dev/spibus/spibusvar.h>
#include "spibus_if.h"
#include "bcm63xx_reg.h"

#define HSSPI_PLL_FREQ (133333333)
#define HSSPI_MAX_SYNC_FREQ (30000000)
#define HSSPI_FIFO_LEN (512)

struct bcm63xx_hsspi_softc {
	device_t sc_dev;
	struct resource* sc_mem_res;
	struct mtx sc_mtx;
	uint32_t sc_cs_pols;
};

static struct ofw_compat_data compat_data[] = {
	{"brcm,bcm6328-hsspi", 1},
	{NULL, 0}
};

static uint32_t bcm63xx_hsspi_xfer(
	struct bcm63xx_hsspi_softc* sc,
	uint8_t* txbuf,
	uint8_t* rxbuf,
	uint32_t buf_len,
	uint32_t cs
) {
	uint32_t i = (0), d = (0),
			 v = (0), o = (0),
			 t = (0);
	for(i = buf_len; (i > 0);) {
		d = min(HSSPI_FIFO_LEN, i);
		bus_write_2(sc->sc_mem_res, REG_HSSPI_PINGPONG_0_FIFO_0,
					htobe16((1<<13)|(d&0x3ff)));
		bus_write_region_1(sc->sc_mem_res,
						   (REG_HSSPI_PINGPONG_0_FIFO_0+2), &txbuf[o], d);
		v = (1);
		v |= ((cs<<8)|(!cs<<12));
		bus_write_4(sc->sc_mem_res, REG_HSSPI_PINGPONG_0_COMMAND, v);
		v = (0);
		for(t = 0; t < 1000; t++) {
			v = bus_read_4(sc->sc_mem_res, REG_HSSPI_PINGPONG_0_STATUS);
			if(!(v&REG_HSSPI_PINGPONG_0_STATUS_SOURCE_BUSY))
				break;
		}
		if(v&REG_HSSPI_PINGPONG_0_STATUS_SOURCE_BUSY)
			return (0);
		bus_read_region_1(sc->sc_mem_res,
						  REG_HSSPI_PINGPONG_0_FIFO_0, &rxbuf[o], d);
		i -= d, o += d;
	}
	return (1);
}

static int bcm63xx_hsspi_probe(device_t dev) {
	if(!ofw_bus_status_okay(dev))
		return (ENXIO);
	if(!ofw_bus_search_compatible(dev, compat_data)->ocd_data)
		return (ENXIO);
	device_set_desc(dev, "Broadcom BCM63xx high-speed SPI");
	return (0);
}

static int bcm63xx_hsspi_attach(device_t dev) {
	struct bcm63xx_hsspi_softc* sc = device_get_softc(dev);
	int rid = (0);
	mtx_init(&sc->sc_mtx, device_get_nameunit(dev), NULL, MTX_DEF);
	sc->sc_dev = dev;
	if(!(sc->sc_mem_res = bus_alloc_resource_any(
							  dev, SYS_RES_MEMORY, &rid, RF_ACTIVE)))
		return (ENXIO);
	sc->sc_cs_pols = (bus_read_4(sc->sc_mem_res,
								 REG_HSSPI_GLOBALCNTRL_CONTROL)&0xff);
	bus_write_4(sc->sc_mem_res,
				REG_HSSPI_GLOBALCNTRL_INTERRUPT_MASK, 0);
	bus_write_4(sc->sc_mem_res,
				REG_HSSPI_GLOBALCNTRL_INTERRUPT_STATUS, 0xff001f1f);
	bus_write_4(sc->sc_mem_res,
				REG_HSSPI_GLOBALCNTRL_CONTROL,
				(REG_HSSPI_GLOBALCNTRL_CONTROL_GATE_CLK_WHEN_SSOFF|
				 REG_HSSPI_GLOBALCNTRL_CONTROL_MOSI_STATE_WHEN_IDLE|
				 REG_HSSPI_GLOBALCNTRL_CONTROL_SPI_CLK_CLKEN));
	device_add_child(dev, "spibus", -1);
	return (bus_generic_attach(dev));
}

static int bcm63xx_hsspi_detach(device_t dev) {
	struct bcm63xx_hsspi_softc* sc = device_get_softc(dev);
	if(sc->sc_mem_res)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->sc_mem_res);
	mtx_destroy(&sc->sc_mtx);
	return (0);
}

static int bcm63xx_hsspi_transfer(device_t dev, device_t child,
								  struct spi_command* cmd) {
	struct bcm63xx_hsspi_softc* sc = device_get_softc(dev);
	uint32_t dev_freq = (0),
			 cs = (0),
			 clock = (0),
			 v = (0),
			 i = (0);

	spibus_get_clock(child, &dev_freq);
	spibus_get_cs(child, &cs);

	if(cs > 8) {
		device_printf(dev, "clock select > 8");
		return (EINVAL);
	}

	clock = (((HSSPI_PLL_FREQ+dev_freq)-1)/dev_freq);
	clock = (((2048+clock)-1)/clock);

	mtx_lock(&sc->sc_mtx);

	v = (REG_HSSPI_PROFILE_0_CLK_CTRL_ACCUM_RST_ON_LOOP|clock);
	bus_write_4(sc->sc_mem_res, REG_HSSPI_PROFILE_0_CLK_CTRL, v);

	v = (REG_HSSPI_PROFILE_0_SIGNAL_CTRL_LAUNCH_MSB_FIRST|
		 REG_HSSPI_PROFILE_0_SIGNAL_CTRL_LATCH_MSB_FIRST|
		 REG_HSSPI_PROFILE_0_SIGNAL_CTRL_LATCH_RISING|
		 4);
	if(dev_freq > HSSPI_MAX_SYNC_FREQ)
		v |= REG_HSSPI_PROFILE_0_SIGNAL_CTRL_ASYNC_INPUT_PATH_EN;
	bus_write_4(sc->sc_mem_res, REG_HSSPI_PROFILE_0_SIGNAL_CTRL, v);

	// enable CS
	v = bus_read_4(sc->sc_mem_res, REG_HSSPI_GLOBALCNTRL_CONTROL);
	if(sc->sc_cs_pols&(1<<cs))
		v &= (~(1<<cs));
	else
		v |= (1<<cs);
	if(sc->sc_cs_pols&(1<<!cs))
		v &= (~(1<<!cs));
	else
		v |= (1<<!cs);
	bus_write_4(sc->sc_mem_res, REG_HSSPI_GLOBALCNTRL_CONTROL, v);

	v = (REG_HSSPI_PROFILE_0_MODE_CTRL_MULTIDATA_WR_SIZE_DUAL|
		 REG_HSSPI_PROFILE_0_MODE_CTRL_MULTIDATA_RD_SIZE_DUAL|
		 0xff);
	bus_write_4(sc->sc_mem_res, REG_HSSPI_PROFILE_0_MODE_CTRL, v);

	bcm63xx_hsspi_xfer(sc, cmd->tx_cmd,
					   cmd->rx_cmd, cmd->rx_cmd_sz, cs);
	bcm63xx_hsspi_xfer(sc, cmd->tx_data,
					   cmd->rx_data, cmd->rx_data_sz, cs);

	// disable CS
	v = bus_read_4(sc->sc_mem_res, REG_HSSPI_GLOBALCNTRL_CONTROL);
	v &= (~0xff);
	v |= sc->sc_cs_pols;
	bus_write_4(sc->sc_mem_res, REG_HSSPI_GLOBALCNTRL_CONTROL, v);

	mtx_unlock(&sc->sc_mtx);

	return (0);
}

static phandle_t bcm63xx_hsspi_get_node(device_t bus, device_t dev) {
	return (ofw_bus_get_node(bus));
}

static device_method_t bcm63xx_hsspi_methods[] = {
	DEVMETHOD(device_probe, bcm63xx_hsspi_probe),
	DEVMETHOD(device_attach, bcm63xx_hsspi_attach),
	DEVMETHOD(device_detach, bcm63xx_hsspi_detach),
	DEVMETHOD(spibus_transfer, bcm63xx_hsspi_transfer),
	DEVMETHOD(ofw_bus_get_node, bcm63xx_hsspi_get_node),
	DEVMETHOD_END
};

static driver_t bcm63xx_hsspi_driver = {
	"hsspi",
	bcm63xx_hsspi_methods,
	sizeof(struct bcm63xx_hsspi_softc)
};

static devclass_t bcm63xx_hsspi_devclass;

DRIVER_MODULE(hsspi, simplebus, bcm63xx_hsspi_driver, bcm63xx_hsspi_devclass, 0, 0);
DRIVER_MODULE(ofw_spibus, hsspi, ofw_spibus_driver, ofw_spibus_devclass, 0, 0);
MODULE_DEPEND(hsspi, ofw_spibus, 1, 1, 1);
OFWBUS_PNP_INFO(compat_data);
