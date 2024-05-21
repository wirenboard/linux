#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/delay.h>


#define DRIVER_NAME "wbec-uart"

#define printk(...)
#define dev_info(...)
#define snprintf(...)

#define WBEC_REGMAP_PAD_WORDS_COUNT   		5
#define WBEC_REGMAP_READ_BIT				BIT(15)


static void u16_word_to_spi_buf(u16 word, u8 *buf, int pos_in_words)
{
	buf[pos_in_words * 2 + 1] = word & 0xFF;
	buf[pos_in_words * 2] = (word >> 8) & 0xFF;
}

static int wbec_read_regs_sync(struct spi_device *spi, u16 addr, u16 *buf, int len_words)
{
	const int transfer_len_words = 1 + WBEC_REGMAP_PAD_WORDS_COUNT + len_words;
	const int transfer_len_bytes = transfer_len_words * sizeof(u16);
	u8 *tx_buf = kmalloc(transfer_len_bytes, GFP_KERNEL);
	u8 *rx_buf = kmalloc(transfer_len_bytes, GFP_KERNEL);
	struct spi_transfer xfer = {
		.tx_buf = tx_buf,
		.rx_buf = rx_buf,
		.len = transfer_len_bytes,
	};
	int ret, i;

	if (!tx_buf || !rx_buf) {
		ret = -ENOMEM;
		goto out;
	}

	u16_word_to_spi_buf(addr | WBEC_REGMAP_READ_BIT, tx_buf, 0);
	for (i = 1; i < transfer_len_words; i++) {
		u16_word_to_spi_buf(i, tx_buf, i);
	}

	ret = spi_sync_transfer(spi, &xfer, 1);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to read registers: %d\n", ret);
		goto out;
	}

	for (i = 0; i < len_words; i++) {
		buf[i] = rx_buf[(1 + WBEC_REGMAP_PAD_WORDS_COUNT) * 2 + i * 2 + 1] |
			(rx_buf[(1 + WBEC_REGMAP_PAD_WORDS_COUNT) * 2 + i * 2] << 8);
	}

out:
	kfree(tx_buf);
	kfree(rx_buf);
	return ret;
}

static int wbec_write_regs_sync(struct spi_device *spi, u16 addr, const u16 *buf, int len_words)
{
	const int transfer_len_words = 1 + WBEC_REGMAP_PAD_WORDS_COUNT + len_words;
	const int transfer_len_bytes = transfer_len_words * sizeof(u16);
	u8 *tx_buf = kmalloc(transfer_len_bytes, GFP_KERNEL);
	u8 *rx_buf = kmalloc(transfer_len_bytes, GFP_KERNEL);
	struct spi_transfer xfer = {
		.tx_buf = tx_buf,
		.rx_buf = rx_buf,
		.len = transfer_len_bytes,
	};
	int ret, i;

	if (!tx_buf || !rx_buf) {
		ret = -ENOMEM;
		goto out;
	}

	u16_word_to_spi_buf(addr, tx_buf, 0);
	for (i = 0; i < transfer_len_words; i++) {
		u16_word_to_spi_buf(buf[i], tx_buf, WBEC_REGMAP_PAD_WORDS_COUNT + 1 + i);
	}

	ret = spi_sync_transfer(spi, &xfer, 1);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to read registers: %d\n", ret);
		goto out;
	}

out:
	kfree(tx_buf);
	kfree(rx_buf);
	return ret;
}

struct wbec_uart {
	struct device *dev;
	struct spi_device *spi;
	struct uart_port port;
	struct completion spi_transfer_complete;

	bool tx_in_progress;
};

union uart_tx {
	struct {
		u8 bytes_to_send_count;
		u8 reserved;
		u8 bytes_to_send[64];
	};
	u16 buf[33];
};

union uart_tx_start {
	struct {
		u16 bytes_to_send_count;
		u8 bytes_to_send[64];
	};
	u16 buf[33];
};

union uart_rx {
	struct {
		u8 read_bytes_count;
		u8 ready_for_tx;
		u8 read_bytes[64];
	};
	u16 buf[33];
};

union uart_ctrl {
	struct {
		u16 reset : 1;
		u16 want_to_tx : 1;
	};
	u16 buf[1];
};

enum wbec_spi_cmd {
	WBEC_SPI_CMD_CTRL,
	WBEC_SPI_CMD_EXCHANGE,
	WBEC_SPI_CMD_TX_START,
};

#define WBEC_SPI_BUF_SIZE    			(WBEC_REGMAP_PAD_WORDS_COUNT * 2 + sizeof(union uart_rx) + 2)

struct wbec_data_exchange {
	enum wbec_spi_cmd cmd;

	struct spi_message msg;
	struct spi_transfer transfer;

	u8 tx_buf[256];
	u8 rx_buf[256];

	struct wbec_uart *priv;
};

static struct uart_driver wbec_uart_driver = {
	.owner = THIS_MODULE,
	.driver_name = DRIVER_NAME,
	.dev_name = "ttyWBE",
	.nr = 1,
};

static int wbec_data_exchange_start_async(struct wbec_uart *wbec_uart);

static void wbec_cmd_data_exchange(struct wbec_uart *wbec_uart, const u8 *tx_buf, const u8 *rx_buf, int len_bytes)
{
	struct uart_port *port = &wbec_uart->port;
	struct circ_buf *xmit = &port->state->xmit;
	int i;
	u8 c;
	char str[256] = "";
	const int len_words = len_bytes / 2 - 1 - WBEC_REGMAP_PAD_WORDS_COUNT;
	union uart_rx rx;
	union uart_tx tx;

	printk(KERN_INFO "%s called\n", __func__);

	// get data from rx

	for (i = 0; i < len_words; i++) {
		rx.buf[i] = rx_buf[(1 + WBEC_REGMAP_PAD_WORDS_COUNT) * 2 + i * 2 + 1] |
			(rx_buf[(1 + WBEC_REGMAP_PAD_WORDS_COUNT) * 2 + i * 2] << 8);
		tx.buf[i] = tx_buf[(1 + WBEC_REGMAP_PAD_WORDS_COUNT) * 2 + i * 2 + 1] |
			(tx_buf[(1 + WBEC_REGMAP_PAD_WORDS_COUNT) * 2 + i * 2] << 8);
	}

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

	if (rx.ready_for_tx) {
		u8 bytes_sent = tx.bytes_to_send_count;

		printk(KERN_INFO "bytes_sent=%d; tail_was=%d\n", bytes_sent, xmit->tail);


		if (bytes_sent > 0) {
			port->icount.tx += bytes_sent;
			xmit->tail = (xmit->tail + bytes_sent) & (UART_XMIT_SIZE - 1);
		} else {
			if (wbec_uart->tx_in_progress) {
				wbec_uart->tx_in_progress = false;
			}
		}

		printk(KERN_INFO "new_tail=%d\n", xmit->tail);

		if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
			uart_write_wakeup(port);
	}

	printk(KERN_INFO "%s\n", str);
}

static void wbec_spi_transfer_complete(void *context)
{
	struct wbec_data_exchange *wbec_data_exchange = context;
	struct wbec_uart *wbec_uart = wbec_data_exchange->priv;

	switch (wbec_data_exchange->cmd) {
	case WBEC_SPI_CMD_CTRL:
		break;

	case WBEC_SPI_CMD_EXCHANGE:
		wbec_cmd_data_exchange(wbec_uart, wbec_data_exchange->tx_buf, wbec_data_exchange->rx_buf, wbec_data_exchange->transfer.len);
		break;
	};
	printk(KERN_INFO "%s called, cmd = %d\n", __func__, wbec_data_exchange->cmd);

	complete(&wbec_uart->spi_transfer_complete);
	kfree(wbec_data_exchange);
}


static int wbec_spi_transfer_start(struct wbec_uart *wbec_uart, enum wbec_spi_cmd cmd, u16 addr, const u16 *vals, int len_words)
{
	const int transfer_len_words = 1 + WBEC_REGMAP_PAD_WORDS_COUNT + len_words;
	const int transfer_len_bytes = transfer_len_words * sizeof(u16);
	int ret, i;
	struct spi_device *spi = wbec_uart->spi;
	struct wbec_data_exchange *wbec_data_exchange;
	struct spi_transfer *xfer;
	u8 *tx_buf;
	u8 *rx_buf;

	reinit_completion(&wbec_uart->spi_transfer_complete);

	printk(KERN_INFO "%s called, cmd = %d\n", __func__, cmd);

	wbec_data_exchange = kmalloc(sizeof(*wbec_data_exchange), GFP_ATOMIC);
	if (!wbec_data_exchange) {
		return -ENOMEM;
	}
	memset(wbec_data_exchange, 0, sizeof(*wbec_data_exchange));

	xfer = &wbec_data_exchange->transfer;
	tx_buf = wbec_data_exchange->tx_buf;
	rx_buf = wbec_data_exchange->rx_buf;

	wbec_data_exchange->priv = wbec_uart;
	wbec_data_exchange->cmd = cmd;
	xfer->tx_buf = tx_buf;
	xfer->rx_buf = rx_buf;
	xfer->len = transfer_len_bytes;

	u16_word_to_spi_buf(addr, tx_buf, 0);
	for (i = 1; i < (1 + WBEC_REGMAP_PAD_WORDS_COUNT); i++) {
		u16_word_to_spi_buf(i, tx_buf, i);
	}

	for (i = 0; i < len_words; i++) {
		u16_word_to_spi_buf(vals[i], tx_buf, WBEC_REGMAP_PAD_WORDS_COUNT + 1 + i);
	}

	spi_message_init_with_transfers(&wbec_data_exchange->msg, xfer, 1);
	wbec_data_exchange->msg.complete = wbec_spi_transfer_complete;
	wbec_data_exchange->msg.context = wbec_data_exchange;

	ret = spi_async(spi, &wbec_data_exchange->msg);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to write registers: %d\n", ret);
	}
	return ret;
}

static int wbec_data_exchange_start_async(struct wbec_uart *wbec_uart)
{
	struct uart_port *port = &wbec_uart->port;
	struct circ_buf *xmit = &port->state->xmit;
	union uart_tx tx = {};

	printk(KERN_INFO "%s called\n", __func__);


	// get data for tx
	if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
		tx.bytes_to_send_count = 0;
	} else {
		int i;
		unsigned int to_send = uart_circ_chars_pending(xmit);
		printk(KERN_INFO "to_send linux: %d; head=%d, tail=%d\n", to_send, xmit->head, xmit->tail);
		to_send = min(to_send, ARRAY_SIZE(tx.bytes_to_send));
		printk(KERN_INFO "to_send spi: %d\n", to_send);

		tx.bytes_to_send_count = to_send;

		/* Convert to linear buffer */
		for (i = 0; i < to_send; ++i) {
			tx.bytes_to_send[i] = xmit->buf[(xmit->tail + i) & (UART_XMIT_SIZE - 1)];
		}
	}


	return wbec_spi_transfer_start(wbec_uart, WBEC_SPI_CMD_EXCHANGE, 0x100, tx.buf, sizeof(tx) / 2);
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
	struct wbec_uart *wbec_uart = container_of(port,
					      struct wbec_uart,
					      port);

	printk(KERN_INFO "%s called\n", __func__);

	if (!wbec_uart->tx_in_progress) {
		union uart_tx_start tx_start;
		struct uart_port *port = &wbec_uart->port;
		struct circ_buf *xmit = &port->state->xmit;
		int i;
		unsigned int to_send = uart_circ_chars_pending(xmit);

		wbec_uart->tx_in_progress = true;

		printk(KERN_INFO "to_send linux: %d; head=%d, tail=%d\n", to_send, xmit->head, xmit->tail);
		to_send = min(to_send, ARRAY_SIZE(tx_start.bytes_to_send));
		printk(KERN_INFO "to_send spi: %d\n", to_send);

		tx_start.bytes_to_send_count = to_send;

		/* Convert to linear buffer */
		for (i = 0; i < to_send; ++i) {
			tx_start.bytes_to_send[i] = xmit->buf[xmit->tail];
			xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		}

		wbec_spi_transfer_start(wbec_uart, WBEC_SPI_CMD_TX_START, 0x190, tx_start.buf, 1 + (to_send + 1) / 2);
	}
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
}

static void wbec_uart_stop_rx(struct uart_port *port)
{
	struct wbec_uart *wbec_uart = container_of(port,
					      struct wbec_uart,
					      port);
	printk(KERN_INFO "%s called\n", __func__);
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
	union uart_ctrl uart_ctrl = {};

	printk(KERN_INFO "%s called\n", __func__);

	uart_ctrl.reset = 1;

	wbec_uart->tx_in_progress = false;

	wbec_write_regs_sync(wbec_uart->spi, 0x1E0, uart_ctrl.buf, sizeof(uart_ctrl) / 2);
	mdelay(1);
	return 0;
}

static void wbec_uart_shutdown(struct uart_port *port)
{
	struct wbec_uart *wbec_uart = container_of(port,
					      struct wbec_uart,
					      port);
	printk(KERN_INFO "%s called\n", __func__);

	wait_for_completion(&wbec_uart->spi_transfer_complete);
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

static irqreturn_t wbec_uart_irq(int irq, void *dev_id)
{
	struct wbec_uart *wbec_uart = dev_id;

	printk(KERN_INFO "%s called\n", __func__);

	wbec_data_exchange_start_async(wbec_uart);

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
	int ret;
	u16 wbec_id;
	struct wbec_uart *wbec_uart;

	printk(KERN_INFO "%s called\n", __func__);

	wbec_uart = devm_kzalloc(&spi->dev, sizeof(struct wbec_uart),
				GFP_KERNEL);

	wbec_uart->dev = &spi->dev;

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	spi_setup(spi);

	spi_set_drvdata(spi, wbec_uart);

	wbec_uart->spi = spi;

	ret = wbec_read_regs_sync(spi, 0x00, &wbec_id, 1);
	dev_info(&spi->dev, "wbec_id 0xB0: %.2X\n", wbec_id);

	// Register the UART driver
	ret = uart_register_driver(&wbec_uart_driver);
	if (ret) {
		pr_err("Failed to register UART driver\n");
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

	init_completion(&wbec_uart->spi_transfer_complete);

	ret = uart_add_one_port(&wbec_uart_driver, &wbec_uart->port);
	if (ret) {
		pr_err("Failed to register UART port\n");
		return ret;
	}

	dev_info(&spi->dev, "IRQ: %d\n", spi->irq);

	ret = devm_request_irq(wbec_uart->dev, spi->irq, wbec_uart_irq,
					IRQF_TRIGGER_RISING,
					dev_name(wbec_uart->dev), wbec_uart);
	if (ret) {
		dev_err(&spi->dev, "Failed to request IRQ: %d\n", ret);
		return ret;
	}

	printk(KERN_INFO "WBE UART driver loaded\n");

	return 0;
}

static int wbec_uart_remove(struct spi_device *spi)
{
	struct wbec_uart *wbec_uart = dev_get_drvdata(&spi->dev);

	printk(KERN_INFO "%s called\n", __func__);

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
