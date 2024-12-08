#include <sys/cdefs.h>
#include "opt_platform.h"
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kobj.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/sysctl.h>
#include <machine/bus.h>
#include <dev/fdt/fdt_common.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/uart/uart.h>
#include <dev/uart/uart_cpu.h>
#include <dev/uart/uart_cpu_fdt.h>
#include <dev/uart/uart_bus.h>
#include "uart_if.h"
#include "bcm63xx_reg.h"

static int bcm63xx_uart_probe(struct uart_bas* bas) {
	return (0);
}

static void bcm63xx_uart_init(
	struct uart_bas* bas,
	int baudrate,
	int databits,
	int stopbits,
	int parity
) {
	uint32_t v = (0);
	v = (((bas->rclk/baudrate)>>4)>>1);
	if(!(v&0x1)) v--;
	uart_setreg(bas, REG_UART_BAUDWORD, v);
	uart_barrier(bas);
	// UART_MISC_CTL: TxFifoThreshold=8, RxFifoThreshold=8
	uart_setreg(bas, REG_UART_MISC_CTL, ((0x8<<12)|(0x8<<8)));
	uart_barrier(bas);
	uart_setreg(bas, REG_UART_EXT_CTL, 0);
	uart_barrier(bas);
	v = (REG_UART_CONTROL_RXEN|
			REG_UART_CONTROL_TXEN|
			REG_UART_CONTROL_BRGEN|
			(stopbits==1?REG_UART_CONTROL_STOPBITS_1:0));
	switch(databits) {
		case 8:
			v |= (0x3<<12);
			break;
		case 7:
			v |= (0x2<<12);
			break;
		case 6:
			v |= (0x1<<12);
			break;
		case 5:
			break;
		default:
			return;
	}
	if(parity) {
		v |= (REG_UART_CONTROL_RXPARITYEVEN|
				REG_UART_CONTROL_RXPARITYEN);
		v |= (REG_UART_CONTROL_TXPARITYEVEN|
				REG_UART_CONTROL_TXPARITYEN);
	}
	uart_setreg(bas, REG_UART_CONTROL, v);
	uart_barrier(bas);
}

static void bcm63xx_uart_term(struct uart_bas* bas) {}

static void bcm63xx_uart_putc(struct uart_bas* bas, int c) {
	for(; !(uart_getreg(bas, REG_UART_INTSTATUSMASK)&
				REG_UART_INTSTATUSMASK_TXFIFOEMT););
	uart_setreg(bas, REG_UART_UARTFIFO, c);
	uart_barrier(bas);
}

static int bcm63xx_uart_rxready(struct uart_bas* bas) {
	return (uart_getreg(bas, REG_UART_INTSTATUSMASK)&
			REG_UART_INTSTATUSMASK_RXFIFONE);
}

static int bcm63xx_uart_getc(struct uart_bas* bas, struct mtx* m) {
	uint32_t c = (0);
	uart_lock(m);
	for(; !(uart_getreg(bas, REG_UART_INTSTATUSMASK)&
				REG_UART_INTSTATUSMASK_RXFIFONE);) {
		uart_unlock(m);
		DELAY(10);
		uart_lock(m);
	}
	c = uart_getreg(bas, REG_UART_UARTFIFO);
	uart_unlock(m);
	return (c);
}

static int bcm63xx_uart_bus_attach(struct uart_softc* sc) {
	struct uart_bas* bas = &sc->sc_bas;
	sc->sc_rxfifosz = (16);
	sc->sc_txfifosz = (16);
	uart_setreg(bas, REG_UART_INTSTATUSMASK, 
			REG_UART_INTSTATUSMASK_RXFIFONEMSK);
	uart_barrier(bas);
	return (0);
}

static int bcm63xx_uart_bus_detach(struct uart_softc* sc) {
	return (0);
}

static int bcm63xx_uart_bus_flush(struct uart_softc* sc, int res) {
	struct uart_bas* bas = &sc->sc_bas;
	uint32_t v = (0);
	uart_lock(sc->sc_hwmtx);
	v = uart_getreg(bas, REG_UART_CONTROL);
	if(res&UART_FLUSH_RECEIVER)
		v |= REG_UART_CONTROL_RSTRXFIFOS;
	if(res&UART_FLUSH_TRANSMITTER)
		v |= REG_UART_CONTROL_RSTTXFIFOS;
	uart_setreg(bas, REG_UART_CONTROL, v);
	uart_barrier(bas);
	uart_unlock(sc->sc_hwmtx);
	return (0);
}

static int bcm63xx_uart_bus_getsig(struct uart_softc* sc) {
	return (0);
}

static int bcm63xx_uart_bus_ioctl(struct uart_softc* sc, int req, intptr_t data) {
	struct uart_bas* bas = &sc->sc_bas;
	uint32_t baud = (0), div = (0);
	int ret = (0);
	switch(req) {
	case UART_IOCTL_BREAK:
	case UART_IOCTL_IFLOW:
	case UART_IOCTL_OFLOW:
		break;
	case UART_IOCTL_BAUD:
		div = uart_getreg(bas, REG_UART_BAUDWORD);
		baud = (bas->rclk/(div*16));
		*(intptr_t*)(data) = (baud);
		break;
	default:
		ret = (EINVAL);
		break;
	}
	return (ret);
}

static int bcm63xx_uart_bus_ipend(struct uart_softc* sc) {
	uint32_t status = (0),
			 a = (0);
	int ret = (0);
	uart_lock(sc->sc_hwmtx);
	status = uart_getreg(&sc->sc_bas, REG_UART_INTSTATUSMASK);
	uart_barrier(&sc->sc_bas);
	if(status&REG_UART_INTSTATUSMASK_TXFIFOTHOLD) {
		if(sc->sc_txbusy)
			ret |= SER_INT_TXIDLE;
		a = uart_getreg(&sc->sc_bas, REG_UART_INTSTATUSMASK);
		uart_barrier(&sc->sc_bas);
		a &= (~REG_UART_INTSTATUSMASK_TXFIFOTHOLDMSK);
		uart_setreg(&sc->sc_bas, REG_UART_INTSTATUSMASK, a);
		uart_barrier(&sc->sc_bas);
	}
	if(status&REG_UART_INTSTATUSMASK_RXFIFONE)
		ret |= SER_INT_RXREADY;
	uart_unlock(sc->sc_hwmtx);
	return (ret);
}

static int bcm63xx_uart_bus_param(
	struct uart_softc* sc,
	int baudrate,
	int databits,
	int stopbits,
	int parity
) {
	uart_lock(sc->sc_hwmtx);
	bcm63xx_uart_init(&sc->sc_bas, baudrate, databits, stopbits, parity);
	uart_unlock(sc->sc_hwmtx);
	return (0);
}

static int bcm63xx_uart_bus_probe(struct uart_softc* sc) {
	device_set_desc(sc->sc_dev, "Broadcom BCM63xx UART");
	return (0);
}

static int bcm63xx_uart_bus_receive(struct uart_softc* sc) {
	struct uart_bas* bas = &sc->sc_bas;
	uint32_t c = (0);
	uart_lock(sc->sc_hwmtx);
	for(; (uart_getreg(bas, REG_UART_INTSTATUSMASK)&
				REG_UART_INTSTATUSMASK_RXFIFONE);) {
		c = uart_getreg(bas, REG_UART_UARTFIFO);
		uart_barrier(bas);
		uart_rx_put(sc, c);
	}
	uart_unlock(sc->sc_hwmtx);
	return (0);
}

static int bcm63xx_uart_bus_setsig(struct uart_softc* sc, int sig) {
	return (0);
}

static int bcm63xx_uart_bus_transmit(struct uart_softc* sc) {
	struct uart_bas* bas = &sc->sc_bas;
	uint32_t reg = (0),
			 i = (0);
	uart_lock(sc->sc_hwmtx);
	reg = uart_getreg(bas, REG_UART_INTSTATUSMASK);
	reg &= (~REG_UART_INTSTATUSMASK_TXFIFOTHOLDMSK);
	uart_setreg(bas, REG_UART_INTSTATUSMASK, reg);
	for(; !(uart_getreg(bas, REG_UART_INTSTATUSMASK)&
				REG_UART_INTSTATUSMASK_TXFIFOEMT););
	reg = uart_getreg(bas, REG_UART_INTSTATUSMASK);
	reg |= (REG_UART_INTSTATUSMASK_TXFIFOTHOLDMSK);
	uart_setreg(bas, REG_UART_INTSTATUSMASK, reg);
	for(i = 0; i < sc->sc_txdatasz; i++) {
		uart_setreg(bas, REG_UART_UARTFIFO, sc->sc_txbuf[i]);
		uart_barrier(bas);
	}
	sc->sc_txbusy = (1);
	for(; !(uart_getreg(bas, REG_UART_INTSTATUSMASK)&
				REG_UART_INTSTATUSMASK_TXFIFOEMT););
	uart_unlock(sc->sc_hwmtx);
	return (0);
}

static void bcm63xx_uart_bus_grab(struct uart_softc* sc) {
	struct uart_bas* bas = &sc->sc_bas;
	uint32_t v = (0);
	uart_lock(sc->sc_hwmtx);
	v = uart_getreg(bas, REG_UART_INTSTATUSMASK);
	v &= (~(REG_UART_INTSTATUSMASK_TXFIFOTHOLDMSK|
				REG_UART_INTSTATUSMASK_RXFIFONEMSK));
	uart_setreg(bas, REG_UART_INTSTATUSMASK, v);
	uart_barrier(bas);
	uart_unlock(sc->sc_hwmtx);
}

static void bcm63xx_uart_bus_ungrab(struct uart_softc* sc) {
	struct uart_bas* bas = &sc->sc_bas;
	uint32_t v = (0);
	uart_lock(sc->sc_hwmtx);
	v = uart_getreg(bas, REG_UART_INTSTATUSMASK);
	v |= (REG_UART_INTSTATUSMASK_TXFIFOTHOLDMSK|
			REG_UART_INTSTATUSMASK_RXFIFONEMSK);
	uart_setreg(bas, REG_UART_INTSTATUSMASK, v);
	uart_barrier(bas);
	uart_unlock(sc->sc_hwmtx);
}

static kobj_method_t bcm63xx_uart_methods[] = {
	KOBJMETHOD(uart_attach, bcm63xx_uart_bus_attach),
	KOBJMETHOD(uart_detach, bcm63xx_uart_bus_detach),
	KOBJMETHOD(uart_flush, bcm63xx_uart_bus_flush),
	KOBJMETHOD(uart_getsig, bcm63xx_uart_bus_getsig),
	KOBJMETHOD(uart_ioctl, bcm63xx_uart_bus_ioctl),
	KOBJMETHOD(uart_ipend, bcm63xx_uart_bus_ipend),
	KOBJMETHOD(uart_param, bcm63xx_uart_bus_param),
	KOBJMETHOD(uart_probe, bcm63xx_uart_bus_probe),
	KOBJMETHOD(uart_receive, bcm63xx_uart_bus_receive),
	KOBJMETHOD(uart_setsig, bcm63xx_uart_bus_setsig),
	KOBJMETHOD(uart_transmit, bcm63xx_uart_bus_transmit),
	KOBJMETHOD(uart_grab, bcm63xx_uart_bus_grab),
	KOBJMETHOD(uart_ungrab, bcm63xx_uart_bus_ungrab),
	KOBJMETHOD_END
};

static struct uart_ops bcm63xx_uart_ops = {
	.probe = bcm63xx_uart_probe,
	.init = bcm63xx_uart_init,
	.term = bcm63xx_uart_term,
	.putc = bcm63xx_uart_putc,
	.rxready = bcm63xx_uart_rxready,
	.getc = bcm63xx_uart_getc
};

static struct uart_class bcm63xx_uart_class = {
	"bcm63xx_uart",
	bcm63xx_uart_methods,
	0,
	.uc_ops = &bcm63xx_uart_ops,
	// 0x10000100-0x10000114
	.uc_range = 0x18,
	.uc_rclk = 50000000,
	.uc_riowidth = 4
};

static struct ofw_compat_data compat_data[] = {
	{"brcm,bcm6345-uart", (uintptr_t)(&bcm63xx_uart_class)},
	{NULL, 0}
};

UART_FDT_CLASS_AND_DEVICE(compat_data);
