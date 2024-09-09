#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/mfd/wbec.h>
#include <linux/kthread.h>
#include <linux/mutex.h>

#define DRIVER_NAME "wbec-uart"

#define printk(...)
#define dev_info(...)
#define snprintf(...)

#define WBEC_UART_PORT_COUNT			2

#define WBEC_REGMAP_READ_BIT			BIT(15)

#define WBEC_UART_REGMAP_BUFFER_SIZE		64

struct wbec_uart_regmap_address {
	u16 ctrl;
	u16 tx_start;
	u16 exchange;
};

static const struct wbec_uart_regmap_address wbec_uart_regmap_address[WBEC_UART_PORT_COUNT] = {
	[0] = {
		.ctrl = 0x100,
		.tx_start = 0x120,
		.exchange = 0x180,
	},
	[1] = {
		.ctrl = 0x108,
		.tx_start = 0x121,
		.exchange = 0x1A1,
	}
};

struct wbec_uart_one_port {
	struct uart_port port;
	struct wbec_uart *wbec_uart;
	struct completion tx_complete;
};

struct wbec_uart {
	struct device *dev;
	struct spi_device *spi;
	struct regmap *regmap;

	struct wbec_uart_one_port ports[WBEC_UART_PORT_COUNT];
};

struct wbec_regmap_header {
	u16 address : 15;
	u16 read_bit : 1;
	u16 pad[WBEC_REGMAP_PAD_WORDS_COUNT];
} __packed;

struct uart_rx {
    u8 read_bytes_count;
    u8 ready_for_tx : 1;
    u8 tx_completed : 1;
    u8 read_bytes[WBEC_UART_REGMAP_BUFFER_SIZE];
} __packed;

struct uart_tx {
    u8 bytes_to_send_count;
    u8 reserved;
    u8 bytes_to_send[WBEC_UART_REGMAP_BUFFER_SIZE];
} __packed;

struct uart_ctrl {
    /* offset 0x00 */
    uint16_t enable : 1;
    uint16_t ctrl_applyed : 1;
    uint16_t res1 : 14;
    /* offset 0x01 */
    uint16_t baud_x100;
    /* offset 0x02 */
    uint16_t parity : 2;
    uint16_t stop_bits : 2;
	uint16_t res2 : 12;
} __packed;

union uart_ctrl_regs {
	struct uart_ctrl ctrl;
	u16 buf[3];
};

union uart_tx_regs {
	struct {
		struct wbec_regmap_header header;
		struct uart_tx tx;
	};
	u16 buf[33 + 6];
};

union uart_rx_regs {
	struct {
		struct wbec_regmap_header header;
		struct uart_rx rx;
	};
	u16 buf[33 + 6];
};

union uart_exchange_regs_tx {
	struct {
		struct wbec_regmap_header header;
		struct uart_tx tx[WBEC_UART_PORT_COUNT];
	};
	u16 buf[(sizeof(struct uart_tx) * WBEC_UART_PORT_COUNT + sizeof(struct wbec_regmap_header)) / 2];
};

union uart_exchange_regs_rx {
	struct {
		struct wbec_regmap_header header;
		struct uart_rx rx[WBEC_UART_PORT_COUNT];
	};
	u16 buf[(sizeof(struct uart_rx) * WBEC_UART_PORT_COUNT + sizeof(struct wbec_regmap_header)) / 2];
};

static struct uart_driver wbec_uart_driver = {
	.owner = THIS_MODULE,
	.driver_name = DRIVER_NAME,
	.dev_name = "ttyWBE",
	.nr = WBEC_UART_PORT_COUNT,
};

static void swap_bytes(u16 *buf, int len)
{
	int i;
	for (i = 0; i < len; i++) {
		buf[i] = __fswab16(buf[i]);
	}
}

static void wbec_spi_exchange_sync(struct wbec_uart *wbec_uart)
{
	union uart_exchange_regs_rx rx;
	union uart_exchange_regs_tx tx = {};
	struct spi_message msg;
	struct spi_transfer transfer = {};
	int ret, port_i;
	char str[256];
	u8 bytes_sent_in_xfer[2] = {0, 0};

	// prepare tx data
	for (port_i = 0; port_i < WBEC_UART_PORT_COUNT; port_i++) {
		struct wbec_uart_one_port *wbec_one_port = &wbec_uart->ports[port_i];
		struct uart_port *port = &wbec_one_port->port;
		struct circ_buf *xmit = &port->state->xmit;

		printk(KERN_INFO "process port_i=%d\n", port_i);

		if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
			printk(KERN_INFO "empty or stopped\n");
			tx.tx[port_i].bytes_to_send_count = 0;
		} else {
			int i;
			unsigned int to_send = uart_circ_chars_pending(xmit);
			printk(KERN_INFO "to_send linux: %d; head=%d, tail=%d\n", to_send, xmit->head, xmit->tail);
			to_send = min(to_send, WBEC_UART_REGMAP_BUFFER_SIZE);
			printk(KERN_INFO "to_send spi: %d\n", to_send);

			tx.tx[port_i].bytes_to_send_count = to_send;
			bytes_sent_in_xfer[port_i] = to_send;

			/* Convert to linear buffer */
			for (i = 0; i < to_send; ++i) {
				u8 c = xmit->buf[(xmit->tail + i) & (UART_XMIT_SIZE - 1)];
				tx.tx[port_i].bytes_to_send[i] = c;
			}

			if (to_send)
				reinit_completion(&wbec_one_port->tx_complete);
		}
	}

	tx.header.address = wbec_uart_regmap_address[0].exchange;

	transfer.tx_buf = tx.buf;
	transfer.rx_buf = rx.buf;
	transfer.len = sizeof(tx.header) + sizeof(tx.tx[0]) * WBEC_UART_PORT_COUNT;

	spi_message_init_with_transfers(&msg, &transfer, 1);

	swap_bytes(tx.buf, transfer.len / 2);

	// transfer
	ret = spi_sync(wbec_uart->spi, &msg);
	if (ret) {
		dev_err(wbec_uart->dev, "spi_sync failed: %d\n", ret);
		return;
	}

	swap_bytes(rx.buf, transfer.len / 2);


	for (port_i = 0; port_i < WBEC_UART_PORT_COUNT; port_i++) {
		struct wbec_uart_one_port *wbec_one_port = &wbec_uart->ports[port_i];
		struct uart_port *port = &wbec_one_port->port;
		struct circ_buf *xmit = &port->state->xmit;
		u8 bytes_sent = bytes_sent_in_xfer[port_i];

		snprintf(str, ARRAY_SIZE(str), "received_bytes: %d: ", rx.rx[port_i].read_bytes_count);
		if (rx.rx[port_i].read_bytes_count > WBEC_UART_REGMAP_BUFFER_SIZE) {
			dev_err(wbec_uart->dev, "received_bytes_count > WBEC_UART_REGMAP_BUFFER_SIZE\n");
			rx.rx[port_i].read_bytes_count = WBEC_UART_REGMAP_BUFFER_SIZE;
		}
		if (rx.rx[port_i].read_bytes_count > 0) {
			int i;
			for (i = 0; i < rx.rx[port_i].read_bytes_count; i++) {
				u8 c = rx.rx[port_i].read_bytes[i];
				// snprintf(str, ARRAY_SIZE(str), "%s[%.2X]", str, c);
				port->icount.rx++;
				uart_insert_char(port, 0, 0, c, TTY_NORMAL);
			}
			tty_flip_buffer_push(&port->state->port);
		}

		if (rx.rx[port_i].ready_for_tx) {

			printk(KERN_INFO "bytes_sent=%d; tail_was=%d\n", bytes_sent, xmit->tail);

			if (bytes_sent > 0) {
				uart_xmit_advance(port, bytes_sent);
			}

			printk(KERN_INFO "new_tail=%d\n", xmit->tail);

			if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
				uart_write_wakeup(port);
		}

		if (rx.rx[port_i].tx_completed && (bytes_sent == 0)) {
			complete(&wbec_one_port->tx_complete);
		}
	}
}


static unsigned int wbec_uart_tx_empty(struct uart_port *port)
{
	// struct wbec_uart *wbec_uart = container_of(port,
	// 				      struct wbec_uart,
	// 				      port);
	printk(KERN_INFO "%s called\n", __func__);

	return TIOCSER_TEMT;
}

static void wbec_uart_start_tx(struct uart_port *port)
{
	struct wbec_uart_one_port *wbec_one_port = container_of(port,
					      struct wbec_uart_one_port,
					      port);

	printk(KERN_INFO "%s called\n", __func__);

	reinit_completion(&wbec_one_port->tx_complete);

	regmap_write_async(wbec_one_port->wbec_uart->regmap,
			   wbec_uart_regmap_address[port->line].tx_start,
			   0x1);
}

static void wbec_uart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	printk(KERN_INFO "%s called\n", __func__);
}

static unsigned int wbec_uart_get_mctrl(struct uart_port *port)
{
	printk(KERN_INFO "%s called\n", __func__);
	return 0;
}

static void wbec_uart_stop_tx(struct uart_port *port)
{
	// struct wbec_uart *wbec_uart = container_of(port,
	// 				      struct wbec_uart,
	// 				      port);
	printk(KERN_INFO "%s called\n", __func__);
}

static void wbec_uart_stop_rx(struct uart_port *port)
{
	// struct wbec_uart *wbec_uart = container_of(port,
	// 				      struct wbec_uart,
	// 				      port);
	printk(KERN_INFO "%s called\n", __func__);
}

static void wbec_uart_break_ctl(struct uart_port *port, int break_state)
{
	printk(KERN_INFO "%s called\n", __func__);
}

static int wbec_uart_startup(struct uart_port *port)
{
	struct wbec_uart_one_port *wbec_one_port = container_of(port,
					      struct wbec_uart_one_port,
					      port);

	struct regmap *regmap = wbec_one_port->wbec_uart->regmap;
	int ret, val;
	u16 ctrl_reg = wbec_uart_regmap_address[port->line].ctrl;

	printk(KERN_INFO "%s called\n", __func__);

	/* set enable=1; applyed=0 */
	regmap_write(regmap, ctrl_reg, 0x1);

	/* wait for applyed==1 */
	ret = regmap_read_poll_timeout(regmap, ctrl_reg, val,
				       (val & 0x0002),
				       100, 1000000);

	if (ret == 0) {
		printk(KERN_INFO "UART port %d is ready\n", port->line);
	} else {
		printk(KERN_INFO "UART port %d is not ready\n", port->line);
	}

	complete(&wbec_one_port->tx_complete);

	return ret;
}

static void wbec_uart_shutdown(struct uart_port *port)
{
	struct wbec_uart_one_port *wbec_one_port = container_of(port,
					      struct wbec_uart_one_port,
					      port);

	struct regmap *regmap = wbec_one_port->wbec_uart->regmap;

	int ret, val;
	u16 ctrl_reg = wbec_uart_regmap_address[port->line].ctrl;

	printk(KERN_INFO "%s called\n", __func__);


	wait_for_completion_timeout(&wbec_one_port->tx_complete, msecs_to_jiffies(10000));

	/* set enable=0; applyed=0 */
	regmap_write(regmap, ctrl_reg, 0x0);

	/* wait for applyed==1 */
	ret = regmap_read_poll_timeout(regmap, ctrl_reg, val,
				       (val & 0x0002),
				       100, 1000000);
}

static void wbec_uart_set_termios(struct uart_port *port, struct ktermios *new,
				       const struct ktermios *old)
{
	struct wbec_uart_one_port *wbec_one_port = container_of(port,
					      struct wbec_uart_one_port,
					      port);
	struct regmap *regmap = wbec_one_port->wbec_uart->regmap;
	int ret, val, baud;
	u16 ctrl_reg = wbec_uart_regmap_address[port->line].ctrl;
	union uart_ctrl_regs ctrl_regs;

	printk(KERN_INFO "%s called\n", __func__);

	regmap_bulk_read(regmap, ctrl_reg, ctrl_regs.buf, ARRAY_SIZE(ctrl_regs.buf));

	ctrl_regs.ctrl.ctrl_applyed = 0;

	/* Parity */
	if (new->c_cflag & PARENB) {
		if (new->c_cflag & PARODD)
			ctrl_regs.ctrl.parity = 0x2; /* Odd */
		else
			ctrl_regs.ctrl.parity = 0x1; /* Even */
	} else {
		ctrl_regs.ctrl.parity = 0x0; /* None */
	}

	/* Stop bits */
	if (new->c_cflag & CSTOPB)
		ctrl_regs.ctrl.stop_bits = 2; /* 2 */
	else
		ctrl_regs.ctrl.stop_bits = 0; /* 1 */

	/* Baud rate */
	baud = uart_get_baud_rate(port, new, old, 1200, 115200);
	printk(KERN_INFO "baud=%d\n", baud);
	ctrl_regs.ctrl.baud_x100 = baud / 100;

	regmap_bulk_write(regmap, ctrl_reg, ctrl_regs.buf, ARRAY_SIZE(ctrl_regs.buf));

	/* wait for applyed==1 */
	ret = regmap_read_poll_timeout(regmap, ctrl_reg, val,
				       (val & 0x0002),
				       100, 1000000);

	uart_update_timeout(port, new->c_cflag, baud);
}

static void wbec_uart_config_port(struct uart_port *port, int flags)
{
	printk(KERN_INFO "%s called\n", __func__);
	if (flags & UART_CONFIG_TYPE)
		port->type = 123;
}

static int wbec_uart_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	printk(KERN_INFO "%s called\n", __func__);
	return 0;
}

static void wbec_uart_pm(struct uart_port *port, unsigned int state, unsigned int oldstate)
{
	printk(KERN_INFO "%s called\n", __func__);
}

static int wbec_uart_request_port(struct uart_port *port)
{
	printk(KERN_INFO "%s called\n", __func__);
	return 0;
}

static void wbec_uart_null_void(struct uart_port *port)
{
	printk(KERN_INFO "%s called\n", __func__);
}

static const char * wbec_uart_type(struct uart_port *port)
{
	printk(KERN_INFO "%s called\n", __func__);
	return 0;
}

static irqreturn_t wbec_uart_irq(int irq, void *dev_id)
{
	struct wbec_uart *wbec_uart = dev_id;

	printk(KERN_INFO "%s called\n", __func__);

	wbec_spi_exchange_sync(wbec_uart);

	return IRQ_HANDLED;
}

static const struct uart_ops wbec_uart_ops = {
	.tx_empty	= wbec_uart_tx_empty,
	.set_mctrl	= wbec_uart_set_mctrl,
	.get_mctrl	= wbec_uart_get_mctrl,
	.stop_tx	= wbec_uart_stop_tx,
	.start_tx	= wbec_uart_start_tx,
	.stop_rx	= wbec_uart_stop_rx,
	.break_ctl	= wbec_uart_break_ctl,
	.startup	= wbec_uart_startup,
	.shutdown	= wbec_uart_shutdown,
	.set_termios	= wbec_uart_set_termios,
	.type		= wbec_uart_type,
	.request_port	= wbec_uart_request_port,
	.release_port	= wbec_uart_null_void,
	.config_port	= wbec_uart_config_port,
	.verify_port	= wbec_uart_verify_port,
	.pm		= wbec_uart_pm,
};

static int wbec_uart_config_rs485(struct uart_port *,
				  struct ktermios *termios,
				  struct serial_rs485 *rs485)
{
	printk(KERN_INFO "%s called\n", __func__);
	// port->rs485 = *rs485;

	return 0;
}

static int wbec_uart_probe(struct platform_device *pdev)
{
	struct wbec *wbec = dev_get_drvdata(pdev->dev.parent);
	struct wbec_uart *wbec_uart;
	int ret, irq, i;
	int wbec_id;

	dev_info(&pdev->dev, "%s called\n", __func__);

	wbec_uart = devm_kzalloc(&pdev->dev, sizeof(struct wbec_uart),
				GFP_KERNEL);
	if (!wbec_uart)
		return -ENOMEM;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "Failed to get IRQ: %d\n", irq);
		return irq;
	}

	wbec_uart->dev = &pdev->dev;
	wbec_uart->spi = wbec->spi;
	wbec_uart->regmap = wbec->regmap;

	platform_set_drvdata(pdev, wbec_uart);

	ret = regmap_read(wbec_uart->regmap, 0x00, &wbec_id);
	if (ret) {
		dev_err(&pdev->dev, "Failed to read WBE ID: %d\n", ret);
		return ret;
	}
	dev_info(&pdev->dev, "wbec_id 0xB0: %.2X\n", wbec_id);

	// Register the UART driver
	ret = uart_register_driver(&wbec_uart_driver);
	if (ret) {
		pr_err("Failed to register UART driver\n");
		return ret;
	}

	for (i = 0; i < wbec_uart_driver.nr; i++) {
		wbec_uart->ports[i].wbec_uart = wbec_uart;

		init_completion(&wbec_uart->ports[i].tx_complete);

		// Register the UART port
		// Initialize the UART port
		wbec_uart->ports[i].port.ops = &wbec_uart_ops;
		wbec_uart->ports[i].port.dev = &pdev->dev;
		wbec_uart->ports[i].port.type = PORT_GENERIC;
		wbec_uart->ports[i].port.irq = irq;
		wbec_uart->ports[i].port.iotype = UPIO_PORT;
		wbec_uart->ports[i].port.flags	= UPF_FIXED_TYPE | UPF_LOW_LATENCY;
		// wbec_uart->port.flags = UPF_BOOT_AUTOCONF;
		wbec_uart->ports[i].port.rs485_config = wbec_uart_config_rs485;
		wbec_uart->ports[i].port.uartclk = 115200;
		wbec_uart->ports[i].port.fifosize = 64;
		wbec_uart->ports[i].port.line = i;

		ret = uart_add_one_port(&wbec_uart_driver, &wbec_uart->ports[i].port);
		if (ret) {
			pr_err("Failed to register UART port\n");
			return ret;
		}
	}

	dev_info(&pdev->dev, "IRQ: %d\n", irq);

	ret = devm_request_threaded_irq(wbec_uart->dev, irq, NULL, wbec_uart_irq,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					dev_name(wbec_uart->dev), wbec_uart);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request IRQ: %d\n", ret);
		return ret;
	}

	dev_info(&pdev->dev, "WBE UART driver loaded\n");

	return 0;
}

static int wbec_uart_remove(struct platform_device *pdev)
{
	struct wbec_uart *wbec_uart = platform_get_drvdata(pdev);
	int i;

	printk(KERN_INFO "%s called\n", __func__);

	// Unregister the UART port
	for (i = 0; i < wbec_uart_driver.nr; i++) {
		struct wbec_uart_one_port *wbec_one_port = &wbec_uart->ports[i];
		uart_remove_one_port(&wbec_uart_driver, &wbec_one_port->port);
	}

	// Unregister the UART driver
	uart_unregister_driver(&wbec_uart_driver);

	return 0;
}

static const struct of_device_id wbec_uart_of_match[] = {
	{ .compatible = "wirenboard,wbec-uart" },
	{}
};
MODULE_DEVICE_TABLE(of, wbec_uart_of_match);

static struct platform_driver wbec_uart_platform_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = wbec_uart_of_match,
	},
	.probe = wbec_uart_probe,
	.remove = wbec_uart_remove,
};

module_platform_driver(wbec_uart_platform_driver);

MODULE_ALIAS("platform:wbec-uart");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("WBE UART Driver");
