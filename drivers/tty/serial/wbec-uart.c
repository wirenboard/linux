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

#define DRIVER_NAME "wbec-uart"

static const struct regmap_config wbec_regmap_config = {
	.reg_bits = 16,
	.read_flag_mask = BIT(7),
	.val_bits = 16,
	.max_register = 0xFF,
};

struct wbec_uart {
	// static struct spi_device *spi_dev;
	struct device *dev;
	struct uart_port port;
	struct regmap *regmap;
	// struct kthread_worker		kworker;
	// struct task_struct		*kworker_task;
	struct delayed_work rx_poll;
	struct delayed_work tx_poll;
	bool ignore_first_zero_byte;
};

static struct uart_driver wbec_uart_driver = {
	.owner = THIS_MODULE,
	.driver_name = DRIVER_NAME,
	.dev_name = "ttyWBE",
	.nr = 1,
};

static int wbec_uart_open(struct tty_struct *tty, struct file *filp)
{
	return 0;
}

static void wbec_uart_close(struct tty_struct *tty, struct file *filp)
{
}

static unsigned int wbec_uart_tx_empty(struct uart_port *port)
{
	struct wbec_uart *wbec_uart = container_of(port,
					      struct wbec_uart,
					      port);
	printk(KERN_INFO "%s called\n", __func__);
	cancel_delayed_work_sync(&wbec_uart->tx_poll);

	return TIOCSER_TEMT;
}

static void wbec_uart_start_tx(struct uart_port *port)
{
	unsigned int txlen, to_send, i, c;

	struct wbec_uart *wbec_uart = container_of(port,
					      struct wbec_uart,
					      port);


	printk(KERN_INFO "%s called\n", __func__);
	wbec_uart->ignore_first_zero_byte = true;

	// struct circ_buf *xmit = &port->state->xmit;
	// to_send = uart_circ_chars_pending(xmit);
	// printk(KERN_INFO "to_send: %d\n", to_send);
	// if (to_send) {
	// 	printk(KERN_INFO "xmit->head: %d\n", xmit->head);
	// 	printk(KERN_INFO "xmit->tail: %d\n", xmit->tail);

	// 	/* Convert to linear buffer */
	// 	for (i = 0; i < to_send; ++i) {
	// 		c = xmit->buf[xmit->tail];
	// 		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
	// 		printk(KERN_INFO "c: %.2X\n", c);
	// 	}
	// }
	schedule_delayed_work(&wbec_uart->tx_poll, 0);
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
	cancel_delayed_work_sync(&wbec_uart->rx_poll);
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
	printk(KERN_INFO "%s called\n", __func__);
	schedule_delayed_work(&wbec_uart->rx_poll, msecs_to_jiffies(20));
	wbec_uart->ignore_first_zero_byte = false;
	return 0;
}

static void wbec_uart_shutdown(struct uart_port *port)
{
	printk(KERN_INFO "%s called\n", __func__);
}

static void wbec_uart_set_termios(struct uart_port *port, struct ktermios *termios, struct ktermios *old)
{
	printk(KERN_INFO "%s called\n", __func__);
}

static void wbec_uart_config_port(struct uart_port *port, int flags)
{
	printk(KERN_INFO "%s called\n", __func__);
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

void wbec_uart_rx_poll_wq(struct work_struct *work)
{
	unsigned int reg, i, ret;
	struct wbec_uart *wbec_uart =
		container_of(work, struct wbec_uart, rx_poll.work);
	u16 buf[33];
	u16 received_bytes;

	// printk(KERN_INFO "%s called\n", __func__);
	regmap_bulk_read(wbec_uart->regmap, 0x61, buf, ARRAY_SIZE(buf));

	received_bytes = buf[0];


	if (received_bytes > 32) {
		printk(KERN_INFO "received_bytes > 32: %d\n", received_bytes);
	} else if (received_bytes > 0) {
		printk(KERN_INFO "received_bytes: %d\n", received_bytes);
		for (i = 0; i < received_bytes; i++) {
			printk(KERN_INFO "buf[%d]: %.2X\n", i + 1, buf[i + 1]);
			// uart_insert_char(&wbec_uart->port, 0, 0, 'A', TTY_NORMAL);
			if (wbec_uart->ignore_first_zero_byte && i == 0 && buf[i + 1] == 0) {
				wbec_uart->ignore_first_zero_byte = false;
				dev_info(wbec_uart->dev, "ignore first zero byte\n");
				continue;
			}
			wbec_uart->port.icount.rx++;
			uart_insert_char(&wbec_uart->port, 0, 0, buf[i + 1], TTY_NORMAL);
		}
		tty_flip_buffer_push(&wbec_uart->port.state->port);
	}

	// clear the RX buffer
	ret = regmap_write(wbec_uart->regmap, 0x60, received_bytes);
	if (ret < 0) {
		dev_err(wbec_uart->dev, "failed to write the RX buffer at 0x60\n");
	}

	// Schedule the next work
	schedule_delayed_work(&wbec_uart->rx_poll, msecs_to_jiffies(2));
}

void wbec_uart_tx_poll_wq(struct work_struct *work)
{
	unsigned int reg, i, to_send, c;
	int ret;
	int wbec_tx_size;
	u16 buf[34];
	struct wbec_uart *wbec_uart =
		container_of(work, struct wbec_uart, tx_poll.work);
	struct uart_port *port = &wbec_uart->port;
	struct circ_buf *xmit = &wbec_uart->port.state->xmit;

	printk(KERN_INFO "%s called\n", __func__);

	// Read available TX buffer size
	ret = regmap_read(wbec_uart->regmap, 0x30, &wbec_tx_size);
	if (ret < 0) {
		dev_err(wbec_uart->dev, "failed to read the wbec_tx_size at 0x100\n");
	}

	dev_info(wbec_uart->dev, "wbec_tx_size: %d\n", wbec_tx_size);

	to_send = uart_circ_chars_pending(xmit);
	dev_info(wbec_uart->dev, "uart circ chars pending: %d\n", to_send);

	// Select the minimum of the two
	to_send = min(to_send, wbec_tx_size);
	to_send = min(to_send, 16);

	printk(KERN_INFO "minimum to_send: %d\n", to_send);

	if (to_send) {
		printk(KERN_INFO "xmit->head: %d\n", xmit->head);
		printk(KERN_INFO "xmit->tail: %d\n", xmit->tail);

		/* Add data to send */
		port->icount.tx += to_send;

		buf[0] = 0;
		buf[1] = to_send;
		/* Convert to linear buffer */
		for (i = 0; i < to_send; ++i) {
			c = xmit->buf[xmit->tail];
			xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
			buf[2 + i] = c;
			printk(KERN_INFO "char to send: %c / %.2X\n", c, c);
		}

		regmap_bulk_write(wbec_uart->regmap, 0x30,
				buf, ARRAY_SIZE(buf));

		if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
			uart_write_wakeup(port);

		// Check if the TX buffer is empty
		to_send = uart_circ_chars_pending(xmit);
		if (to_send) {
			dev_info(wbec_uart->dev, "tx buffer not empty, schedule next tx_poll %d bytes\n", to_send);
			schedule_delayed_work(&wbec_uart->tx_poll, msecs_to_jiffies(2));
		} else {
			dev_info(wbec_uart->dev, "tx buffer empty\n");
		}
	}

	// Read the UART data
	// regmap_read(wbec_uart->regmap, 0x00, &reg);
	// reg = 'A';
	// printk(KERN_INFO "reg: %.2X\n", reg);

	// // put read data to tty buffer
	// if (uart_circ_chars_pending(xmit) < UART_XMIT_SIZE) {
	// 	xmit->buf[xmit->head] = reg;
	// 	xmit->head = (xmit->head + 1) & (UART_XMIT_SIZE - 1);
	// }

	// uart_insert_char(&wbec_uart->port, 0, 0, 'A', TTY_NORMAL);
	// tty_flip_buffer_push(&wbec_uart->port.state->port);


	// Schedule the next work
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


	// Save the SPI device
	// spi_dev = spi;


	// Register the UART port
	// Initialize the UART port
	wbec_uart->port.ops = &wbec_uart_ops;
	wbec_uart->port.dev = &spi->dev;
	wbec_uart->port.type = 123;
	wbec_uart->port.irq = 0;
	wbec_uart->port.iotype = UPIO_PORT;
	wbec_uart->port.flags	= UPF_FIXED_TYPE | UPF_LOW_LATENCY;
	// wbec_uart->port.flags = UPF_BOOT_AUTOCONF;

	wbec_uart->port.uartclk = 115200;
	wbec_uart->port.fifosize = 64;
	wbec_uart->port.line = 0;

	ret = uart_add_one_port(&wbec_uart_driver, &wbec_uart->port);
	if (ret) {
		pr_err("Failed to register UART port\n");
		return ret;
	}

	INIT_DELAYED_WORK(&wbec_uart->rx_poll, wbec_uart_rx_poll_wq);
	INIT_DELAYED_WORK(&wbec_uart->tx_poll, wbec_uart_tx_poll_wq);

	printk(KERN_INFO "WBE UART driver loaded\n");

	return 0;
}

static int wbec_uart_remove(struct spi_device *spi)
{
	printk(KERN_INFO "%s called\n", __func__);

	struct wbec_uart *wbec_uart = dev_get_drvdata(&spi->dev);

	cancel_delayed_work_sync(&wbec_uart->rx_poll);
	cancel_delayed_work_sync(&wbec_uart->tx_poll);

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
