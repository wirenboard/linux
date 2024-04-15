#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/regmap.h>
#include <linux/serial.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/kthread.h>

#define DRIVER_NAME "wbec-uart"

#define printk(...)
#define dev_info(...)
#define snprintf(...)

static const struct regmap_config wbec_regmap_config = {
	.reg_bits = 16,
	.read_flag_mask = BIT(7),
	.val_bits = 16,
	.max_register = 0xFF,
};

struct wbec_uart {
	struct device *dev;
	struct uart_port port;
	struct regmap *regmap;
	struct delayed_work tx_poll;
	struct delayed_work rx_work;
};

union uart_tx {
	struct {
		u16 bytes_to_send_count;
		u8 bytes_to_send[64];
	};
	u16 buf[33];
};

union uart_rx {
	struct {
		u16 read_bytes_count;
		u8 read_bytes[64];
	};
	u16 buf[33];
};

static struct uart_driver wbec_uart_driver = {
	.owner = THIS_MODULE,
	.driver_name = DRIVER_NAME,
	.dev_name = "ttyWBE",
	.nr = 1,
};

void wbec_uart_tx_poll_wq(struct work_struct *work)
{
	unsigned int reg, i, to_send, c;
	int ret;
	int bytes_sent;
	struct wbec_uart *wbec_uart =
		container_of(work, struct wbec_uart, tx_poll.work);
	struct uart_port *port = &wbec_uart->port;
	struct circ_buf *xmit = &wbec_uart->port.state->xmit;
	union uart_tx tx;
	char str[256] = "";

	// printk(KERN_INFO "%s called\n", __func__);

	to_send = uart_circ_chars_pending(xmit);
	to_send = min(to_send, ARRAY_SIZE(tx.bytes_to_send));

	tx.bytes_to_send_count = to_send;

	if (to_send) {
		snprintf(str, ARRAY_SIZE(str), "to_send: %d; ", to_send);

		/* Convert to linear buffer */
		for (i = 0; i < to_send; ++i) {
			c = xmit->buf[(xmit->tail + i) & (UART_XMIT_SIZE - 1)];
			tx.bytes_to_send[i] = c;
			snprintf(str, ARRAY_SIZE(str), "%s[%.2X]", str, c);
		}
	}
	regmap_bulk_write(wbec_uart->regmap, 0x31, tx.buf, 1 + (to_send + 1) / 2);

	usleep_range(50, 150);

	regmap_read(wbec_uart->regmap, 0x30, &bytes_sent);

	// Bytes actually sent
	snprintf(str, ARRAY_SIZE(str), "%s; actually sent: %d", str, bytes_sent);
	port->icount.tx += bytes_sent;
	xmit->tail = (xmit->tail + bytes_sent) & (UART_XMIT_SIZE - 1);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	if (!uart_circ_empty(xmit)) {
		usleep_range(500, 1000);
		schedule_delayed_work(&wbec_uart->tx_poll, 0);
	}
	printk(KERN_INFO "%s\n", str);
}

static unsigned int wbec_uart_tx_empty(struct uart_port *port)
{
	struct wbec_uart *wbec_uart = container_of(port,
					      struct wbec_uart,
					      port);
	printk(KERN_INFO "%s called\n", __func__);

	return TIOCSER_TEMT;
}

static void wbec_uart_start_tx(struct uart_port *port)
{
	unsigned int txlen, to_send, i, c;

	struct wbec_uart *wbec_uart = container_of(port,
					      struct wbec_uart,
					      port);

	schedule_delayed_work(&wbec_uart->tx_poll, 0);
	printk(KERN_INFO "%s called\n", __func__);
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
	struct wbec_uart *wbec_uart = container_of(port,
					      struct wbec_uart,
					      port);
	printk(KERN_INFO "%s called\n", __func__);
	cancel_delayed_work_sync(&wbec_uart->tx_poll);
}

static void wbec_uart_stop_rx(struct uart_port *port)
{
	struct wbec_uart *wbec_uart = container_of(port,
					      struct wbec_uart,
					      port);
	printk(KERN_INFO "%s called\n", __func__);
	cancel_delayed_work_sync(&wbec_uart->rx_work);
}

static void wbec_uart_break_ctl(struct uart_port *port, int break_state)
{
	printk(KERN_INFO "%s called\n", __func__);
}

static int wbec_uart_startup(struct uart_port *port)
{
	struct wbec_uart *wbec_uart = container_of(port,
					      struct wbec_uart,
					      port);
	int rx_buf_size;
	printk(KERN_INFO "%s called\n", __func__);

	// flush the RX buffer
	regmap_read(wbec_uart->regmap, 0x60, &rx_buf_size);

	return 0;
}

static void wbec_uart_shutdown(struct uart_port *port)
{
	struct wbec_uart *wbec_uart = container_of(port,
					      struct wbec_uart,
					      port);
	printk(KERN_INFO "%s called\n", __func__);
	cancel_delayed_work_sync(&wbec_uart->tx_poll);
	cancel_delayed_work_sync(&wbec_uart->rx_work);
}

static void wbec_uart_set_termios(struct uart_port *port, struct ktermios *termios, struct ktermios *old)
{
	printk(KERN_INFO "%s called\n", __func__);



	uart_update_timeout(port, termios->c_cflag, 115200);
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

static void wbec_uart_rx_work_wq(struct work_struct *work)
{
	struct wbec_uart *wbec_uart =
		container_of(work, struct wbec_uart, rx_work.work);
	union uart_rx rx;
	int i;
	u8 c;
	char str[256] = "";

	regmap_bulk_read(wbec_uart->regmap, 0x60, rx.buf, ARRAY_SIZE(rx.buf));

	snprintf(str, ARRAY_SIZE(str), "received_bytes: %d: ", rx.read_bytes_count);
	if (rx.read_bytes_count > 0) {
		for (i = 0; i < rx.read_bytes_count; i++) {
			c = rx.read_bytes[i];
			snprintf(str, ARRAY_SIZE(str), "%s[%.2X]", str, c);
			wbec_uart->port.icount.rx++;
			uart_insert_char(&wbec_uart->port, 0, 0, c, TTY_NORMAL);
		}
		tty_flip_buffer_push(&wbec_uart->port.state->port);
	}
	printk(KERN_INFO "%s\n", str);
}

static irqreturn_t wbec_uart_irq(int irq, void *dev_id)
{
	struct wbec_uart *wbec_uart = dev_id;

	printk(KERN_INFO "%s called\n", __func__);
	schedule_delayed_work(&wbec_uart->rx_work, 0);

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

static int wbec_uart_config_rs485(struct uart_port *port,
				  struct serial_rs485 *rs485)
{
	printk(KERN_INFO "%s called\n", __func__);
	port->rs485 = *rs485;

	return 0;
}

static int wbec_uart_probe(struct spi_device *spi)
{
	int ret, wbec_id;
	struct wbec_uart *wbec_uart;

	printk(KERN_INFO "%s called\n", __func__);

	wbec_uart = devm_kzalloc(&spi->dev, sizeof(struct wbec_uart),
				GFP_KERNEL);

	wbec_uart->dev = &spi->dev;

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	spi_setup(spi);

	spi_set_drvdata(spi, wbec_uart);

	wbec_uart->regmap = devm_regmap_init_spi(spi, &wbec_regmap_config);
	if (IS_ERR(wbec_uart->regmap)) {
		dev_err(&spi->dev, "regmap initialization failed\n");
		return PTR_ERR(wbec_uart->regmap);
	}

	ret = regmap_read(wbec_uart->regmap, 0, &wbec_id);
	if (ret < 0) {
		dev_err(&spi->dev, "failed to read the wbec id at 0x0\n");
		return ret;
	}
	ret = regmap_read(wbec_uart->regmap, 0xB0, &wbec_id);
	if (ret < 0) {
		dev_err(&spi->dev, "failed to read the wbec id at 0xB0\n");
		return ret;
	}
	dev_info(&spi->dev, "wbec_id 0xB0: %.2X\n", wbec_id);

	// Register the UART driver
	ret = uart_register_driver(&wbec_uart_driver);
	if (ret) {
		pr_err("Failed to register UART driver\n");
		return ret;
	}

	INIT_DELAYED_WORK(&wbec_uart->tx_poll, wbec_uart_tx_poll_wq);
	INIT_DELAYED_WORK(&wbec_uart->rx_work, wbec_uart_rx_work_wq);

	dev_info(&spi->dev, "IRQ: %d\n", spi->irq);

	ret = devm_request_irq(wbec_uart->dev, spi->irq, wbec_uart_irq,
					IRQF_TRIGGER_RISING,
					dev_name(wbec_uart->dev), wbec_uart);
	if (ret) {
		dev_err(&spi->dev, "Failed to request IRQ: %d\n", ret);
		return ret;
	}


	// Register the UART port
	// Initialize the UART port
	wbec_uart->port.ops = &wbec_uart_ops;
	wbec_uart->port.dev = &spi->dev;
	wbec_uart->port.type = 123;
	wbec_uart->port.irq = spi->irq;
	wbec_uart->port.iotype = UPIO_PORT;
	wbec_uart->port.flags	= UPF_FIXED_TYPE | UPF_LOW_LATENCY;
	// wbec_uart->port.flags = UPF_BOOT_AUTOCONF;
	wbec_uart->port.rs485_config = wbec_uart_config_rs485;

	wbec_uart->port.uartclk = 115200;
	wbec_uart->port.fifosize = 64;
	wbec_uart->port.line = 0;

	ret = uart_add_one_port(&wbec_uart_driver, &wbec_uart->port);
	if (ret) {
		pr_err("Failed to register UART port\n");
		return ret;
	}

	printk(KERN_INFO "WBE UART driver loaded\n");

	return 0;
}

static int wbec_uart_remove(struct spi_device *spi)
{
	printk(KERN_INFO "%s called\n", __func__);

	struct wbec_uart *wbec_uart = dev_get_drvdata(&spi->dev);

	cancel_delayed_work_sync(&wbec_uart->tx_poll);
	cancel_delayed_work_sync(&wbec_uart->rx_work);

	// Unregister the UART port
	uart_remove_one_port(&wbec_uart_driver, &wbec_uart->port);

	// Unregister the UART driver
	uart_unregister_driver(&wbec_uart_driver);

	return 0;
}

static const struct of_device_id wbec_uart_of_match[] = {
	{ .compatible = "wirenboard,wbec-uart" },
	{}
};
MODULE_DEVICE_TABLE(of, wbec_uart_of_match);

static struct spi_driver wbec_uart_spi_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = wbec_uart_of_match,
	},
	.probe = wbec_uart_probe,
	.remove = wbec_uart_remove,
};

module_spi_driver(wbec_uart_spi_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("WBE UART Driver");
