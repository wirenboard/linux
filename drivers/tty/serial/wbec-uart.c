#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/serial.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>


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
	struct delayed_work tx_poll;
	struct delayed_work rx_work;

	struct dentry *wbec_uart_dir;

	u8 rx_buf_size_stat[400000];
	int rx_buf_size_stat_idx;

	u8 tx_buf_size_stat[400000];
	int tx_buf_size_stat_idx;
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
	u16 bytes_sent;
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
	wbec_write_regs_sync(wbec_uart->spi, 0x31, tx.buf, 1 + (to_send + 1) / 2);

	usleep_range(50, 150);

	wbec_read_regs_sync(wbec_uart->spi, 0x30, &bytes_sent, 1);

	// Bytes actually sent
	snprintf(str, ARRAY_SIZE(str), "%s; actually sent: %d", str, bytes_sent);
	port->icount.tx += bytes_sent;
	xmit->tail = (xmit->tail + bytes_sent) & (UART_XMIT_SIZE - 1);

	if (wbec_uart->tx_buf_size_stat_idx < ARRAY_SIZE(wbec_uart->tx_buf_size_stat)) {
		wbec_uart->tx_buf_size_stat[wbec_uart->tx_buf_size_stat_idx++] = bytes_sent;
	}

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
	u16 rx_buf_size;
	printk(KERN_INFO "%s called\n", __func__);

	wbec_uart->rx_buf_size_stat_idx = 0;
	wbec_uart->tx_buf_size_stat_idx = 0;

	enable_irq(wbec_uart->port.irq);
	// flush the RX buffer
	wbec_read_regs_sync(wbec_uart->spi, 0x60, &rx_buf_size, 1);

	return 0;
}

static void wbec_uart_shutdown(struct uart_port *port)
{
	struct wbec_uart *wbec_uart = container_of(port,
					      struct wbec_uart,
					      port);
	disable_irq(wbec_uart->port.irq);
	cancel_delayed_work_sync(&wbec_uart->tx_poll);
	cancel_delayed_work_sync(&wbec_uart->rx_work);
	printk(KERN_INFO "%s called\n", __func__);
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

	wbec_read_regs_sync(wbec_uart->spi, 0x60, rx.buf, ARRAY_SIZE(rx.buf));

	if (wbec_uart->rx_buf_size_stat_idx < ARRAY_SIZE(wbec_uart->rx_buf_size_stat)) {
		wbec_uart->rx_buf_size_stat[wbec_uart->rx_buf_size_stat_idx++] = rx.read_bytes_count;
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


static ssize_t wbec_uart_rx_stat_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	struct wbec_uart *wbec_uart = file->private_data;
	ssize_t ret;

	if (*ppos >= wbec_uart->rx_buf_size_stat_idx)
		return 0;

	count = min(count, wbec_uart->rx_buf_size_stat_idx - *ppos);

	ret = copy_to_user(buf, &wbec_uart->rx_buf_size_stat[*ppos], count);
	if (ret)
		return -EFAULT;

	*ppos += count;

	return count;
}

static const struct file_operations wbec_uart_rx_stat_fops = {
	.owner = THIS_MODULE,
	.read = wbec_uart_rx_stat_read,
	.open = simple_open,
};

static ssize_t wbec_uart_tx_stat_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	struct wbec_uart *wbec_uart = file->private_data;
	ssize_t ret;

	if (*ppos >= wbec_uart->tx_buf_size_stat_idx)
		return 0;

	count = min(count, wbec_uart->tx_buf_size_stat_idx - *ppos);

	ret = copy_to_user(buf, &wbec_uart->tx_buf_size_stat[*ppos], count);
	if (ret)
		return -EFAULT;

	*ppos += count;

	return count;
}

static const struct file_operations wbec_uart_tx_stat_fops = {
	.owner = THIS_MODULE,
	.read = wbec_uart_tx_stat_read,
	.open = simple_open,
};


static int wbec_uart_probe(struct spi_device *spi)
{
	int ret;
	u16 wbec_id;
	struct wbec_uart *wbec_uart;
	u16 buf[33];

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
	disable_irq(spi->irq);


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

	// init file operations
	wbec_uart->rx_buf_size_stat_idx = 0;
	wbec_uart->tx_buf_size_stat_idx = 0;
	wbec_uart->wbec_uart_dir = debugfs_create_dir("wbec_uart", NULL);
	debugfs_create_file("rx_buf_size_stat", 0444, wbec_uart->wbec_uart_dir, wbec_uart, &wbec_uart_rx_stat_fops);
	debugfs_create_file("tx_buf_size_stat", 0444, wbec_uart->wbec_uart_dir, wbec_uart, &wbec_uart_tx_stat_fops);

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

	debugfs_remove_recursive(wbec_uart->wbec_uart_dir);

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
