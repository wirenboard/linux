/**
 * drivers/serial/sc16is7x2.c
 *
 * Copyright (C) 2009 Manuel Stahl <manuel.stahl@iis.fraunhofer.de>
 * Copyright (C) 2013 Evgeny Boger <boger@contactless.ru>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * The SC16IS7x2 device is a SPI driven dual UART with GPIOs.
 *
 * The driver exports two uarts and a gpiochip interface.
 */

//~ #define DEBUG

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/serial_reg.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/tty_flip.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/freezer.h>
#include <linux/platform_data/sc16is7x2.h>
#include <linux/of_device.h>
#include <linux/console.h>


#define MAX_SC16IS7X2		8
#define FIFO_SIZE		64

#define DRIVER_NAME		"sc16is7x2"
#define TYPE_NAME		"SC16IS7x2"


#define REG_READ	0x80
#define REG_WRITE	0x00

/* Special registers */
#define REG_SPR	    0x07	/* Test character register */
#define REG_TXLVL	0x08	/* Transmitter FIFO Level register */
#define REG_RXLVL	0x09	/* Receiver FIFO Level register */
#define REG_IOD		0x0A	/* IO Direction register */
#define REG_IOS		0x0B	/* IO State register */
#define REG_IOI		0x0C	/* IO Interrupt Enable register */
#define REG_IOC		0x0E	/* IO Control register */
#define REG_EFCR	0x0F	/* Extra Features Control Register */


#define IOC_SRESET	0x08    /* Software reset */
#define IOC_GPIO30	0x04    /* GPIO 3:0 unset: as IO, set: as modem pins */
#define IOC_GPIO74	0x02    /* GPIO 7:4 unset: as IO, set: as modem pins */
#define IOC_IOLATCH	0x01    /* Unset: input unlatched, set: input latched */

#define EFCR_RTSINVER 0x20  /* Invert not-RTS signal in RS-485 mode */
#define EFCR_RTSCON   0x10  /* Auto RS-485 mode: Enable the transmitter to control the RTS pin. */


struct sc16is7x2_chip;

/*
 * Some registers must be read back to modify.
 * To save time we cache them here in memory.
 */
struct sc16is7x2_channel {
	struct spi_transfer	t;
	struct spi_message	m;
	u8		test_buf[4];

	struct sc16is7x2_chip	*chip;	/* back link */
	struct uart_port	uart;

	/* Workqueue that does all the magic */
	struct workqueue_struct *workqueue;
	struct work_struct	work;

	u16		quot;		/* baud rate divisor */
	u8		iir;		/* state of IIR register */
	u8		lsr;		/* state of LSR register */
	u8		msr;		/* state of MSR register */
	u8		ier;		/* cache for IER register */
	u8		fcr;		/* cache for FCR register */
	u8		lcr;		/* cache for LCR register */
	u8		mcr;		/* cache for MCR register */
	u8		efr;		/* cache for EFR register */
	u8      efcr;       /* cache for EFCR register */

#ifdef DEBUG
	bool		handle_irq;
#endif
	bool		handle_baud;	/* baud rate needs update */
	bool		handle_regs;	/* other regs need update */
	u8		buf[FIFO_SIZE+1]; /* fifo transfer buffer */

	bool use_modem_pins_by_default;
	bool console_enabled;
};

struct sc16is7x2_chip {
	spinlock_t		spi_lock;

	struct spi_device *spi;
	struct sc16is7x2_channel channel[2];

	unsigned int	uartclk;
	/* uart line number of the first channel */
	unsigned	uart_base;
	/* number assigned to the first GPIO */
	unsigned	gpio_base;

	char		*gpio_label;
	/* list of GPIO names (array length = SC16IS7X2_NR_GPIOS) */
	const char	*const *gpio_names;


#ifdef CONFIG_GPIOLIB
	struct gpio_chip gpio;
	struct mutex	io_lock;	/* lock for GPIO functions */
	u8		io_dir;		/* cache for IODir register */
	u8		io_state;	/* cache for IOState register */
	u8		io_gpio;	/* PIN is GPIO */
	u8		io_control;	/* cache for IOControl register */
#endif
};

static struct sc16is7x2_channel *sc16is7x2_channels[MAX_SC16IS7X2];


/////////////// test spi wrappers
static void sc16is7x2_spi_complete(void *arg)
{
	complete(arg);
}

static ssize_t
sc16is7x2_spi_sync(struct sc16is7x2_chip *ts, struct spi_message *message)
{
	DECLARE_COMPLETION_ONSTACK(done);
	int status;

	message->complete = sc16is7x2_spi_complete;
	message->context = &done;

	spin_lock_irq(&ts->spi_lock);
	if (ts->spi == NULL)
		status = -ESHUTDOWN;
	else
		status = spi_async(ts->spi, message);
	spin_unlock_irq(&ts->spi_lock);

	if (status == 0) {
		wait_for_completion(&done);
		status = message->status;
		if (status == 0)
			status = message->actual_length;
	}
	return status;
}

static inline ssize_t
sc16is7x2_spi_sync_write(struct sc16is7x2_chip *ts, const void *buf, size_t len)
{
	struct spi_transfer	t = {
			.tx_buf		= buf,
			.len		= len,
		};
	struct spi_message	m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return sc16is7x2_spi_sync(ts, &m);
}


//////////// end of test spi wrappers





/* ******************************** SPI ********************************* */

static inline u8 write_cmd(u8 reg, u8 ch)
{
	return REG_WRITE | (reg & 0xf) << 3 | (ch & 0x1) << 1;
}

static inline u8 read_cmd(u8 reg, u8 ch)
{
	return REG_READ  | (reg & 0xf) << 3 | (ch & 0x1) << 1;
}

/*
 * sc16is7x2_write - Write a new register content (sync)
 * @reg: Register offset
 * @ch:  Channel (0 or 1)
 */
static int sc16is7x2_write(struct sc16is7x2_chip *ts, u8 reg, u8 ch, u8 val)
{
	u8 out[2];
	dev_dbg(&ts->spi->dev, " %s (%i) reg %x cmd hex %x%x \n", __func__, ch, reg, write_cmd(reg, ch), val);

	out[0] = write_cmd(reg, ch);
	out[1] = val;
	return spi_write(ts->spi, out, sizeof(out));
	//~ return sc16is7x2_spi_sync_write(ts, out, sizeof(out));

}

/**
 * sc16is7x2_read - Read back register content
 * @reg: Register offset
 * @ch:  Channel (0 or 1)
 *
 * Returns positive 8 bit value from the device if successful or a
 * negative value on error
 */
static int sc16is7x2_read(struct sc16is7x2_chip *ts, unsigned reg, unsigned ch)
{
	dev_dbg(&ts->spi->dev, " %s (%i) reg %x cmd %x \n", __func__, ch, reg, read_cmd(reg, ch));
	return spi_w8r8(ts->spi, read_cmd(reg, ch));
}

/* ******************************** IRQ ********************************* */

static int sc16is7x2_handle_rx(struct sc16is7x2_chip *ts, unsigned ch)
{
	struct sc16is7x2_channel *chan = &ts->channel[ch];
	struct uart_port *uart = &chan->uart;
	struct tty_port *tty = &uart->state->port;
	struct spi_message message;
	struct spi_transfer t[2];
	unsigned long flags;
	u8 lsr = chan->lsr;
	int rxlvl;

	struct timespec ts_start;
	getnstimeofday(&ts_start);
	//~ printk("sc16is7x2_handle_rx at %lu s, %lu ns\n", ts_start.tv_sec, ts_start.tv_nsec);

	rxlvl = sc16is7x2_read(ts, REG_RXLVL, ch);

	if (rxlvl <= 0) {
		return 0;
	} else if (rxlvl >= FIFO_SIZE) {
		/* Ensure sanity of RX level */
		rxlvl = FIFO_SIZE;

		dev_err(&ts->spi->dev, " %s (%i) overrun!\n", __func__, ch);

	}


	memset(t, 0, sizeof t);
	chan->buf[0] = read_cmd(UART_RX, ch);
	t[0].len = 1;
	t[0].tx_buf = &chan->buf[0];
	//~ t[0].rx_buf = 0;
	t[1].len = rxlvl;
	t[1].rx_buf = &chan->buf[1];
	//~ t[1].tx_buf = 0;

	spi_message_init(&message);
	spi_message_add_tail(&t[0], &message);
	spi_message_add_tail(&t[1], &message);


	if (spi_sync(ts->spi, &message)) {
		dev_err(&ts->spi->dev, " SPI transfer RX handling failed\n");
		return rxlvl;
	}
	chan->buf[rxlvl + 1] = '\0';
	//~ dev_dbg(&ts->spi->dev, "%s\n", &chan->buf[1]);

	spin_lock_irqsave(&uart->lock, flags);

	if (unlikely(lsr & UART_LSR_BRK_ERROR_BITS)) {
		/*
		 * For statistics only
		 */
		if (lsr & UART_LSR_BI) {
			lsr &= ~(UART_LSR_FE | UART_LSR_PE);
			chan->uart.icount.brk++;
			/*
			 * We do the SysRQ and SAK checking
			 * here because otherwise the break
			 * may get masked by ignore_status_mask
			 * or read_status_mask.
			 */
			if (uart_handle_break(&chan->uart))
				goto ignore_char;
		} else if (lsr & UART_LSR_PE)
			chan->uart.icount.parity++;
		else if (lsr & UART_LSR_FE)
			chan->uart.icount.frame++;
		if (lsr & UART_LSR_OE)
			chan->uart.icount.overrun++;
	}

	/* Insert received data */
	tty_insert_flip_string(tty, &chan->buf[1], rxlvl);
	/* Update RX counter */
	uart->icount.rx += rxlvl;



ignore_char:
	spin_unlock_irqrestore(&uart->lock, flags);

	/* Push the received data to receivers */
	if (rxlvl)
		tty_flip_buffer_push(tty);
	return rxlvl;
	//~ dev_err(&ts->spi->dev, " %s (%i) %d bytes\n", __func__, ch, rxlvl);

}

static void sc16is7x2_handle_tx(struct sc16is7x2_chip *ts, unsigned ch)
{
	struct sc16is7x2_channel *chan = &ts->channel[ch];
	struct uart_port *uart = &chan->uart;
	struct circ_buf *xmit = &uart->state->xmit;
	unsigned long flags;
	unsigned i, len;
	int txlvl;

	if (chan->uart.x_char && chan->lsr & UART_LSR_THRE) {
		dev_dbg(&ts->spi->dev, " tx: x-char\n");
		sc16is7x2_write(ts, UART_TX, ch, uart->x_char);
		uart->icount.tx++;
		uart->x_char = 0;
		return;
	}
	if (uart_circ_empty(xmit) || uart_tx_stopped(&chan->uart))
		/* No data to send or TX is stopped */
		return;

	txlvl = sc16is7x2_read(ts, REG_TXLVL, ch);
	if (txlvl <= 0) {
		dev_info(&ts->spi->dev, " %s (%i) fifo full\n", __func__, ch);
		return;
	}

	txlvl = min(txlvl, 30);

	/* number of bytes to transfer to the fifo */

	dev_dbg(&ts->spi->dev, " %s (%i) %d txlvl\n", __func__, ch, txlvl);

	len = min(txlvl, (int)uart_circ_chars_pending(xmit));

	dev_dbg(&ts->spi->dev, " %s (%i) %d bytes\n", __func__, ch, len);

	spin_lock_irqsave(&uart->lock, flags);
	for (i = 1; i <= len ; i++) {
		chan->buf[i] = xmit->buf[xmit->tail];
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
	}
	uart->icount.tx += len;
	spin_unlock_irqrestore(&uart->lock, flags);

	chan->buf[0] = write_cmd(UART_TX, ch);
	if (spi_write(ts->spi, chan->buf, len + 1))
	//~ if (sc16is7x2_spi_sync_write(ts, chan->buf, len + 1))
		dev_err(&ts->spi->dev, " SPI transfer TX handling failed\n");

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(uart);
}

static void sc16is7x2_handle_baud(struct sc16is7x2_chip *ts, unsigned ch)
{
	struct sc16is7x2_channel *chan = &ts->channel[ch];

	if (!chan->handle_baud)
		return;

	dev_dbg(&ts->spi->dev, "%s\n", __func__);

	sc16is7x2_write(ts, UART_IER, ch, 0);
	sc16is7x2_write(ts, UART_LCR, ch, UART_LCR_DLAB); /* access DLL&DLM */
	sc16is7x2_write(ts, UART_DLL, ch, chan->quot & 0xff);
	sc16is7x2_write(ts, UART_DLM, ch, chan->quot >> 8);
	sc16is7x2_write(ts, UART_LCR, ch, chan->lcr);     /* reset DLAB */

	chan->handle_baud = false;
}

static void sc16is7x2_handle_regs(struct sc16is7x2_chip *ts, unsigned ch)
{
	struct sc16is7x2_channel *chan = &ts->channel[ch];

	if (!chan->handle_regs)
		return;

	dev_dbg(&ts->spi->dev, "%s\n", __func__);

        sc16is7x2_write(ts, UART_LCR, ch, 0xBF);  /* access EFR */
	sc16is7x2_write(ts, UART_EFR, ch, chan->efr);
	sc16is7x2_write(ts, UART_LCR, ch, chan->lcr);
	sc16is7x2_write(ts, UART_FCR, ch, chan->fcr);
	sc16is7x2_write(ts, UART_MCR, ch, chan->mcr);
	sc16is7x2_write(ts, UART_IER, ch, chan->ier);


	//~ unsigned char test_char_r;
	//~ unsigned char test_char_w = 'H';
//~
	//~ sc16is7x2_write(ts, REG_SPR, ch, test_char_w);
	//~ test_char_r = sc16is7x2_read(ts, REG_SPR, ch);
//~
	//~ dev_info(&ts->spi->dev, "test char in=%d, out=%d\n", test_char_w, test_char_r);

	chan->handle_regs = false;
}

static void sc16is7x2_read_status(struct sc16is7x2_chip *ts, unsigned ch)
{
	struct sc16is7x2_channel *chan = &(ts->channel[ch]);
/*	struct spi_message m;
	struct spi_transfer t;
	u8 *buf = chan->buf; */
	u8 ier;

#ifdef DEBUG

	//~ ier = sc16is7x2_read(ts, UART_IER, ch);
#endif
	ier = sc16is7x2_read(ts, UART_IER, ch);


	chan->iir = sc16is7x2_read(ts, UART_IIR, ch);
	chan->msr = sc16is7x2_read(ts, UART_MSR, ch);
	chan->lsr = sc16is7x2_read(ts, UART_LSR, ch);

	//~ dev_err(&ts->spi->dev, " %s ier=0x%02x iir=0x%02x msr=0x%02x lsr=0x%02x\n",
			//~ __func__, ier, chan->iir, chan->msr, chan->lsr);

/*
	buf[0] = read_cmd(UART_IER, ch);
	buf[1] = read_cmd(UART_IIR, ch);
	buf[2] = read_cmd(UART_MSR, ch);
	buf[3] = read_cmd(UART_LSR, ch);

	t.tx_buf = buf;
	t.rx_buf = &buf[16];
	t.len = 5;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	spi_sync(ts->spi, &m); */

}

static void sc16is7x2_handle_channel(struct work_struct *w)
{
	struct sc16is7x2_channel *chan =
			container_of(w, struct sc16is7x2_channel, work);
	struct sc16is7x2_chip *ts = chan->chip;
	unsigned ch = (chan == ts->channel) ? 0 : 1;

	int rxlvl = 0;
	unsigned num_read = 0;

	struct timespec ts_start;
	getnstimeofday(&ts_start);
	//~ printk("call queue work at %lu s, %lu ns\n", ts_start.tv_sec, ts_start.tv_nsec);



#ifdef DEBUG
	dev_dbg(&ts->spi->dev, "%s (%i) %s\n", __func__, ch,
			chan->handle_irq ? "irq" : "");
	chan->handle_irq = false;
#endif
	//~ dev_err(&ts->spi->dev, "%s (%i)\n", __func__, ch);


	//~ chan->ier &= ~UART_IER_RLSI;
	//~ chan->ier = 0;
//~
	//~ sc16is7x2_write(ts, UART_IER, ch, 0);
	//~ sc16is7x2_write(ts, UART_IER, 0, 0);
	//~ sc16is7x2_write(ts, UART_IER, 1, 0);


	//~ disable_irq(ts->spi->irq);
	do {
		sc16is7x2_handle_baud(ts, ch);
		sc16is7x2_handle_regs(ts, ch);


		do {
			rxlvl = sc16is7x2_handle_rx(ts, ch);
			++num_read;
		} while ((rxlvl > 16) && (num_read < 10));


		sc16is7x2_handle_tx(ts, ch);

		sc16is7x2_read_status(ts, ch); // read all registers at once

		//~ sc16is7x2_handle_rx(ts, ch);
		//~ sc16is7x2_handle_rx(ts, ch);
		//~ sc16is7x2_handle_rx(ts, ch);
		//~ sc16is7x2_handle_rx(ts, ch);



	} while (!(chan->iir & UART_IIR_NO_INT));

	//~ enable_irq(ts->spi->irq);

	//~ chan->ier |= UART_IER_RLSI;
	//~ sc16is7x2_write(ts, UART_IER, ch, chan->ier);

	//~ dev_err(&ts->spi->dev, "%s finished\n", __func__);
}


static void sc16is7x2_test_spi_complete(void *ctx)
{
	printk("test spi transfer completed\n");
}

/* Trigger work thread*/
static void sc16is7x2_dowork(struct sc16is7x2_channel *chan)
{
	if (!freezing(current)) {

		//~ struct timespec ts_start;
		//~ getnstimeofday(&ts_start);
		//~ printk("request queue work at %lu s, %lu ns\n", ts_start.tv_sec, ts_start.tv_nsec);




		chan->t.tx_buf = chan->test_buf;
		chan->test_buf[0] = 0xa8;
		chan->test_buf[1] = 0x00;

		chan->t.len = 2;

		spi_message_init(&chan->m);
		spi_message_add_tail(&chan->t, &chan->m);
		chan->m.complete = &sc16is7x2_test_spi_complete;


   	    //~ spi_async(chan->chip->spi, &chan->m);








		queue_work(chan->workqueue, &chan->work);
	}
}

static irqreturn_t sc16is7x2_irq(int irq, void *data)
{
	struct sc16is7x2_channel *chan = data;

#ifdef DEBUG
	/* Indicate irq */
	chan->handle_irq = true;
#endif

	/* Trigger work thread */
	sc16is7x2_dowork(chan);

	return IRQ_HANDLED;
}

/* ******************************** UART ********************************* */

#define to_sc16is7x2_channel(port) \
		container_of(port, struct sc16is7x2_channel, uart)


static unsigned int sc16is7x2_tx_empty(struct uart_port *port)
{
	struct sc16is7x2_channel *chan = to_sc16is7x2_channel(port);
	struct sc16is7x2_chip *ts = chan->chip;
	unsigned lsr;

	dev_dbg(&ts->spi->dev, "%s = %s\n", __func__,
			chan->lsr & UART_LSR_TEMT ? "yes" : "no");

	lsr = chan->lsr;
	return lsr & UART_LSR_TEMT ? TIOCSER_TEMT : 0;
}

static unsigned int sc16is7x2_get_mctrl(struct uart_port *port)
{
	struct sc16is7x2_channel *chan = to_sc16is7x2_channel(port);
	struct sc16is7x2_chip *ts = chan->chip;
	unsigned int status;
	unsigned int ret;

	dev_dbg(&ts->spi->dev, "%s (0x%02x)\n", __func__, chan->msr);

	status = chan->msr;

	ret = 0;
	if (status & UART_MSR_DCD)
		ret |= TIOCM_CAR;
	if (status & UART_MSR_RI)
		ret |= TIOCM_RNG;
	if (status & UART_MSR_DSR)
		ret |= TIOCM_DSR;
	if (status & UART_MSR_CTS)
		ret |= TIOCM_CTS;
	return ret;
}

static void sc16is7x2_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct sc16is7x2_channel *chan = to_sc16is7x2_channel(port);
	struct sc16is7x2_chip *ts = chan->chip;

	dev_dbg(&ts->spi->dev, "%s (0x%02x)\n", __func__, mctrl);

	/* TODO: set DCD and DSR
	 * CTS/RTS is handled automatically
	 */
}

static void sc16is7x2_stop_tx(struct uart_port *port)
{
	/* Do nothing */
}

static void sc16is7x2_start_tx(struct uart_port *port)
{
	struct sc16is7x2_channel *chan = to_sc16is7x2_channel(port);
	struct sc16is7x2_chip *ts = chan->chip;

	dev_dbg(&ts->spi->dev, "%s\n", __func__);

	/* Trigger work thread for sending data */
	sc16is7x2_dowork(chan);
}

static void sc16is7x2_stop_rx(struct uart_port *port)
{
	struct sc16is7x2_channel *chan = to_sc16is7x2_channel(port);
	struct sc16is7x2_chip *ts = chan->chip;

	dev_dbg(&ts->spi->dev, "%s\n", __func__);

	chan->ier &= ~UART_IER_RLSI;
	chan->uart.read_status_mask &= ~UART_LSR_DR;
	chan->handle_regs = true;
	/* Trigger work thread for doing the actual configuration change */
	sc16is7x2_dowork(chan);
}

static void sc16is7x2_enable_ms(struct uart_port *port)
{
	struct sc16is7x2_channel *chan = to_sc16is7x2_channel(port);
	struct sc16is7x2_chip *ts = chan->chip;

	dev_dbg(&ts->spi->dev, "%s\n", __func__);

	chan->ier |= UART_IER_MSI;
	chan->handle_regs = true;
	/* Trigger work thread for doing the actual configuration change */
	sc16is7x2_dowork(chan);
}

static void sc16is7x2_break_ctl(struct uart_port *port, int break_state)
{
	/* We don't support break control yet, do nothing */
}

static int sc16is7x2_startup(struct uart_port *port)
{
	struct sc16is7x2_channel *chan = to_sc16is7x2_channel(port);
	struct sc16is7x2_chip *ts = chan->chip;
	unsigned ch = (&ts->channel[1] == chan) ? 1 : 0;
	unsigned long flags;

	dev_info(&ts->spi->dev, "\n%s (%d)\n", __func__, port->line);

	if (chan->workqueue) {
		dev_err(&ts->spi->dev, "\n%s (%d) duplicate startup, do nothing\n", __func__, port->line);
		return 0;
	}



	{
		dev_dbg(&ts->spi->dev, "before handle");
//~ //~
		sc16is7x2_write(ts, UART_LCR, ch, 0xBF);  /* access EFR */
		sc16is7x2_write(ts, UART_EFR, ch, 0x10);
//~ //~
//~ //~
		unsigned char test_char_r;
		unsigned char test_char_w = 'H';
//~ //~
		sc16is7x2_write(ts, REG_SPR, ch, test_char_w);
		test_char_r = sc16is7x2_read(ts, REG_SPR, ch);
//~ //~
		dev_info(&ts->spi->dev, "test char in=%d, out=%d\n", test_char_w, test_char_r);
//~ //~
//~ //~
		dev_dbg(&ts->spi->dev, "after handle");
	}




	/* Clear the interrupt registers. */
	sc16is7x2_write(ts, UART_IER, ch, 0);
	sc16is7x2_read_status(ts, ch);


	/* Initialize work queue */
	chan->workqueue = create_freezable_workqueue("sc16is7x2");
	if (!chan->workqueue) {
		dev_err(&ts->spi->dev, "Workqueue creation failed\n");
		return -EBUSY;
	}
	INIT_WORK(&chan->work, sc16is7x2_handle_channel);

	/* Setup IRQ. Actually we have a low active IRQ, but we want
	 * one shot behaviour */
	if (request_irq(ts->spi->irq, sc16is7x2_irq,
			IRQF_TRIGGER_FALLING | IRQF_SHARED,
			"sc16is7x2", chan)) {
		dev_err(&ts->spi->dev, "IRQ request failed\n");
		destroy_workqueue(chan->workqueue);
		chan->workqueue = NULL;
		return -EBUSY;
	}


	spin_lock_irqsave(&chan->uart.lock, flags);
	chan->lcr = UART_LCR_WLEN8;
	chan->mcr = 0;
	chan->fcr = 0;
	chan->ier = UART_IER_RLSI | UART_IER_RDI | UART_IER_THRI;


	chan->efcr = EFCR_RTSCON | EFCR_RTSINVER; // Enable RS-485. FIXME: user generic interface

	spin_unlock_irqrestore(&chan->uart.lock, flags);

	sc16is7x2_write(ts, UART_FCR, ch, UART_FCR_ENABLE_FIFO |
		       UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);
	sc16is7x2_write(ts, UART_FCR, ch, chan->fcr);
	/* Now, initialize the UART */
	sc16is7x2_write(ts, UART_LCR, ch, chan->lcr);
	sc16is7x2_write(ts, UART_MCR, ch, chan->mcr);
	sc16is7x2_write(ts, UART_IER, ch, chan->ier);

	sc16is7x2_write(ts, REG_EFCR, ch, chan->efcr);  // RS-485. FIXME: user generic interface


	return 0;
}

static void sc16is7x2_shutdown(struct uart_port *port)
{
	struct sc16is7x2_channel *chan = to_sc16is7x2_channel(port);
	struct sc16is7x2_chip *ts = chan->chip;
	unsigned long flags;
	unsigned ch = port->line & 0x01;

	dev_info(&ts->spi->dev, "%s\n", __func__);

	if (chan->console_enabled) {
		// no need to actually shutdown port if console is enabled on this port
		return;
	}


	BUG_ON(!chan);
	BUG_ON(!ts);

	/* Free the interrupt */
	free_irq(ts->spi->irq, chan);

	if (chan->workqueue) {
		/* Flush and destroy work queue */
		flush_workqueue(chan->workqueue);
		destroy_workqueue(chan->workqueue);
		chan->workqueue = NULL;
	}

	/* Suspend HW */
	spin_lock_irqsave(&chan->uart.lock, flags);
	chan->ier = UART_IERX_SLEEP;
	spin_unlock_irqrestore(&chan->uart.lock, flags);
	sc16is7x2_write(ts, UART_IER, ch, chan->ier);
}

static void
sc16is7x2_set_termios(struct uart_port *port, struct ktermios *termios,
		       struct ktermios *old)
{
	struct sc16is7x2_channel *chan = to_sc16is7x2_channel(port);
	struct sc16is7x2_chip *ts = chan->chip;
	unsigned long flags;
	unsigned int baud;
	u8 lcr, fcr = 0;

	/* Ask the core to calculate the divisor for us. */
	baud = uart_get_baud_rate(port, termios, old,
				  port->uartclk / 16 / 0xffff,
				  port->uartclk / 16);
	chan->quot = uart_get_divisor(port, baud);
	chan->handle_baud = true;

	dev_info(&ts->spi->dev, "%s (baud %u)\n", __func__, baud);

	/* set word length */
	switch (termios->c_cflag & CSIZE) {
	case CS5:
		lcr = UART_LCR_WLEN5;
		break;
	case CS6:
		lcr = UART_LCR_WLEN6;
		break;
	case CS7:
		lcr = UART_LCR_WLEN7;
		break;
	default:
	case CS8:
		lcr = UART_LCR_WLEN8;
		break;
	}

	if (termios->c_cflag & CSTOPB)
		lcr |= UART_LCR_STOP;
	if (termios->c_cflag & PARENB)
		lcr |= UART_LCR_PARITY;
	if (!(termios->c_cflag & PARODD))
		lcr |= UART_LCR_EPAR;
#ifdef CMSPAR
	if (termios->c_cflag & CMSPAR)
		lcr |= UART_LCR_SPAR;
#endif

	fcr = UART_FCR_ENABLE_FIFO;
	/* configure the fifo */
	if (baud < 2400)
		fcr |= UART_FCR_TRIGGER_1;
	else
		fcr |= UART_FCR_R_TRIG_01 | UART_FCR_T_TRIG_10;

	chan->efr = UART_EFR_ECB;
	chan->mcr |= UART_MCR_RTS;
	if (termios->c_cflag & CRTSCTS)
		chan->efr |= UART_EFR_CTS | UART_EFR_RTS;

	/*
	 * Ok, we're now changing the port state.  Do it with
	 * interrupts disabled.
	 */
	spin_lock_irqsave(&chan->uart.lock, flags);

	/* we are sending char from a workqueue so enable */
	chan->uart.state->port.low_latency = 1;

	/* Update the per-port timeout. */
	uart_update_timeout(port, termios->c_cflag, baud);

	chan->uart.read_status_mask = UART_LSR_OE | UART_LSR_THRE | UART_LSR_DR;
	if (termios->c_iflag & INPCK)
		chan->uart.read_status_mask |= UART_LSR_FE | UART_LSR_PE;
	if (termios->c_iflag & (BRKINT | PARMRK))
		chan->uart.read_status_mask |= UART_LSR_BI;

	/* Characters to ignore */
	chan->uart.ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		chan->uart.ignore_status_mask |= UART_LSR_PE | UART_LSR_FE;
	if (termios->c_iflag & IGNBRK) {
		chan->uart.ignore_status_mask |= UART_LSR_BI;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			chan->uart.ignore_status_mask |= UART_LSR_OE;
	}

	/* ignore all characters if CREAD is not set */
	if ((termios->c_cflag & CREAD) == 0)
		chan->uart.ignore_status_mask |= UART_LSR_DR;

	/* CTS flow control flag and modem status interrupts */
	chan->ier &= ~UART_IER_MSI;
	if (UART_ENABLE_MS(&chan->uart, termios->c_cflag))
		chan->ier |= UART_IER_MSI;

	chan->lcr = lcr;	/* Save LCR */
	chan->fcr = fcr;	/* Save FCR */
	chan->handle_regs = true;

	spin_unlock_irqrestore(&chan->uart.lock, flags);

	/* Trigger work thread for doing the actual configuration change */
	sc16is7x2_dowork(chan);


}

static const char * sc16is7x2_type(struct uart_port *port)
{
	pr_debug("%s\n", __func__);
	return TYPE_NAME;
}

static void sc16is7x2_release_port(struct uart_port *port)
{
	pr_debug("%s\n", __func__);
}

static int sc16is7x2_request_port(struct uart_port *port)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static void sc16is7x2_config_port(struct uart_port *port, int flags)
{
	struct sc16is7x2_channel *chan = to_sc16is7x2_channel(port);
	struct sc16is7x2_chip *ts = chan->chip;

	dev_dbg(&ts->spi->dev, "%s\n", __func__);
	if (flags & UART_CONFIG_TYPE)
		chan->uart.type = PORT_SC16IS7X2;
}

static int
sc16is7x2_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	if (ser->type == PORT_UNKNOWN || ser->type == PORT_SC16IS7X2)
		return 0;

	return -EINVAL;
}

static struct uart_ops sc16is7x2_uart_ops = {
	.tx_empty	= sc16is7x2_tx_empty,
	.set_mctrl	= sc16is7x2_set_mctrl,
	.get_mctrl	= sc16is7x2_get_mctrl,
	.stop_tx        = sc16is7x2_stop_tx,
	.start_tx	= sc16is7x2_start_tx,
	.stop_rx	= sc16is7x2_stop_rx,
	.enable_ms      = sc16is7x2_enable_ms,
	.break_ctl      = sc16is7x2_break_ctl,
	.startup	= sc16is7x2_startup,
	.shutdown	= sc16is7x2_shutdown,
	.set_termios	= sc16is7x2_set_termios,
	.type		= sc16is7x2_type,
	.release_port   = sc16is7x2_release_port,
	.request_port   = sc16is7x2_request_port,
	.config_port	= sc16is7x2_config_port,
	.verify_port	= sc16is7x2_verify_port,
};


/* ******************************** GPIO ********************************* */

#ifdef CONFIG_GPIOLIB

static int sc16is7x2_gpio_request(struct gpio_chip *gpio, unsigned offset)
{
	struct sc16is7x2_chip *ts =
			container_of(gpio, struct sc16is7x2_chip, gpio);
	int control = (offset < 4) ? IOC_GPIO30 : IOC_GPIO74;
	int ret = 0;

	BUG_ON(offset > 8);
	dev_dbg(&ts->spi->dev, "%s: offset = %d\n", __func__, offset);

	mutex_lock(&ts->io_lock);

	/* GPIO 0:3 and 4:7 can only be controlled as block */
	ts->io_gpio |= BIT(offset);
	if (ts->io_control & control) {
		dev_dbg(&ts->spi->dev, "activate GPIOs %s\n",
				(offset < 4) ? "0-3" : "4-7");
		ts->io_control &= ~control;
		ret = sc16is7x2_write(ts, REG_IOC, 0, ts->io_control);
	}

	mutex_unlock(&ts->io_lock);

	return ret;
}

static void sc16is7x2_gpio_free(struct gpio_chip *gpio, unsigned offset)
{
	struct sc16is7x2_chip *ts =
			container_of(gpio, struct sc16is7x2_chip, gpio);
	int control = (offset < 4) ? IOC_GPIO30 : IOC_GPIO74;
	int mask = (offset < 4) ? 0x0f : 0xf0;

	BUG_ON(offset > 8);

	mutex_lock(&ts->io_lock);

	/* GPIO 0:3 and 4:7 can only be controlled as block */
	ts->io_gpio &= ~BIT(offset);
	dev_dbg(&ts->spi->dev, "%s: io_gpio = 0x%02X\n", __func__, ts->io_gpio);
	if (!(ts->io_control & control) && !(ts->io_gpio & mask)) {
		if (ts->channel[offset < 4 ? 0 : 1].use_modem_pins_by_default) {

			dev_dbg(&ts->spi->dev, "deactivate GPIOs %s\n",
					(offset < 4) ? "0-3" : "4-7");
			ts->io_control |= control;
			sc16is7x2_write(ts, REG_IOC, 0, ts->io_control);
		}
	}

	mutex_unlock(&ts->io_lock);
}

static int sc16is7x2_direction_input(struct gpio_chip *gpio, unsigned offset)
{
	struct sc16is7x2_chip *ts =
			container_of(gpio, struct sc16is7x2_chip, gpio);
	unsigned io_dir;

	BUG_ON(offset > 8);

	mutex_lock(&ts->io_lock);

	ts->io_dir &= ~BIT(offset);
	io_dir = ts->io_dir;

	mutex_unlock(&ts->io_lock);

	return sc16is7x2_write(ts, REG_IOD, 0, io_dir);
}

static int sc16is7x2_direction_output(struct gpio_chip *gpio, unsigned offset,
				    int value)
{
	struct sc16is7x2_chip *ts =
			container_of(gpio, struct sc16is7x2_chip, gpio);

	BUG_ON(offset > 8);

	mutex_lock(&ts->io_lock);

	if (value)
		ts->io_state |= BIT(offset);
	else
		ts->io_state &= ~BIT(offset);

	ts->io_dir |= BIT(offset);

	mutex_unlock(&ts->io_lock);

	sc16is7x2_write(ts, REG_IOS, 0, ts->io_state);
	return sc16is7x2_write(ts, REG_IOD, 0, ts->io_dir);
}

static int sc16is7x2_get(struct gpio_chip *gpio, unsigned offset)
{
	struct sc16is7x2_chip *ts =
			container_of(gpio, struct sc16is7x2_chip, gpio);
	int level = -EINVAL;

	BUG_ON(offset > 8);

	mutex_lock(&ts->io_lock);

	if (ts->io_dir & BIT(offset)) {
		/* Output: return cached level */
		level = (ts->io_state >> offset) & 0x01;
	} else {
		/* Input: read out all pins */
		level = sc16is7x2_read(ts, REG_IOS, 0);
		if (level >= 0) {
			ts->io_state = level;
			level = (ts->io_state >> offset) & 0x01;
		}
	}

	mutex_unlock(&ts->io_lock);

	return level;
}

static void sc16is7x2_set(struct gpio_chip *gpio, unsigned offset, int value)
{
	struct sc16is7x2_chip *ts =
			container_of(gpio, struct sc16is7x2_chip, gpio);
	unsigned io_state;

	BUG_ON(offset > 8);

	mutex_lock(&ts->io_lock);

	if (value)
		ts->io_state |= BIT(offset);
	else
		ts->io_state &= ~BIT(offset);
	io_state = ts->io_state;

	mutex_unlock(&ts->io_lock);

	sc16is7x2_write(ts, REG_IOS, 0, io_state);
}

#endif /* CONFIG_GPIOLIB */

/* ******************************** INIT ********************************* */

static struct uart_driver sc16is7x2_uart_driver;

static int sc16is7x2_register_gpio(struct sc16is7x2_chip *ts)
{
#ifdef CONFIG_GPIOLIB
	ts->gpio.label = (ts->gpio_label) ? ts->gpio_label : DRIVER_NAME;
	ts->gpio.request	= sc16is7x2_gpio_request;
	ts->gpio.free		= sc16is7x2_gpio_free;
	ts->gpio.get		= sc16is7x2_get;
	ts->gpio.set		= sc16is7x2_set;
	ts->gpio.direction_input = sc16is7x2_direction_input;
	ts->gpio.direction_output = sc16is7x2_direction_output;

	ts->gpio.base = ts->gpio_base;
	ts->gpio.names = ts->gpio_names;
	ts->gpio.ngpio = SC16IS7X2_NR_GPIOS;
	ts->gpio.can_sleep = 1;
	ts->gpio.dev = &ts->spi->dev;
	ts->gpio.owner = THIS_MODULE;

	mutex_init(&ts->io_lock);

	ts->io_gpio = 0;
	ts->io_state = 0;
	ts->io_dir = 0;

	sc16is7x2_write(ts, REG_IOI, 0, 0); /* no support for irqs yet */
	sc16is7x2_write(ts, REG_IOS, 0, ts->io_state);
	sc16is7x2_write(ts, REG_IOD, 0, ts->io_dir);

	return gpiochip_add(&ts->gpio);
#else
	return 0;
#endif
}

static int sc16is7x2_register_uart_port(struct sc16is7x2_chip *ts, unsigned ch)
{
	struct sc16is7x2_channel *chan = &(ts->channel[ch]);
	struct uart_port *uart = &chan->uart;
	int ret;

	/* Disable irqs and go to sleep */
	sc16is7x2_write(ts, UART_IER, ch, UART_IERX_SLEEP);

	chan->chip = ts;

	printk(KERN_ERR "irq: %d\n" , ts->spi->irq);
	uart->irq = ts->spi->irq;
	uart->uartclk = ts->uartclk;
	uart->fifosize = FIFO_SIZE;
	uart->ops = &sc16is7x2_uart_ops;
	uart->flags = UPF_SKIP_TEST | UPF_BOOT_AUTOCONF;
	uart->line = ts->uart_base + ch;
	uart->type = PORT_SC16IS7X2;
	uart->dev = &ts->spi->dev;


	sc16is7x2_channels[uart->line] = chan;

	ret = uart_add_one_port(&sc16is7x2_uart_driver, uart);
	if (!ret) {
		if (uart->cons) {
			register_console(uart->cons);
		}
	} else {
		sc16is7x2_channels[uart->line] = NULL;
	}


	return ret;
}


static int sc16is7x2_remove_one_port(struct sc16is7x2_chip *ts, unsigned ch)
{
	struct sc16is7x2_channel *chan = &(ts->channel[ch]);
	struct uart_port *uart = &chan->uart;
	int line =  uart->line;
	int ret;

	ret = uart_remove_one_port(&sc16is7x2_uart_driver, &ts->channel[ch].uart);
	if (!ret) {
		sc16is7x2_channels[line] = NULL;
	}

	return ret;
}




#ifdef CONFIG_OF
/*
 * This function returns 1 iff pdev isn't a device instatiated by dt, 0 iff it
 * could successfully get all information from dt or a negative errno.
 */
static int sc16is7x2_probe_dt(struct sc16is7x2_chip *ts,
		struct spi_device *spi)
{
	struct device_node *np = spi->dev.of_node;
	//~ const struct of_device_id *of_id =
			//~ of_match_device(imx_uart_dt_ids, &pdev->dev);
	//~ int ret;
	const __be32 *iprop;
	int prop_len;

	if (!np)
		/* no device tree device */
		return 1;



	iprop = of_get_property(np, "uartclk", &prop_len);
	if (!iprop || prop_len < sizeof(*iprop)) {
		dev_err(&spi->dev, "Missing uartclck property in devicetree\n");
		return -EINVAL;
	}

	ts->uartclk = be32_to_cpup(iprop);


	iprop = of_get_property(np, "gpio-base", NULL);
	if (!iprop || prop_len < sizeof(*iprop)) {
		dev_err(&spi->dev, "Missing gpio-base property in devicetree\n");
		return -EINVAL;
	}
	ts->gpio_base = be32_to_cpup(iprop);

	dev_err(&spi->dev, "gpio-base=%d\n",ts->gpio_base);
	dev_err(&spi->dev, "uartclk=%d\n",ts->uartclk);





	iprop = of_get_property(np, "uart-base", NULL);
	if (!iprop || prop_len < sizeof(*iprop)) {
		dev_err(&spi->dev, "Missing uart-base property in devicetree\n");
		return -EINVAL;
	}
	ts->uart_base = be32_to_cpup(iprop);


	dev_err(&spi->dev, "uart_base=%d\n",ts->uart_base);

	if (of_property_read_bool(np, "disable-modem-pins-on-startup-a"))
		ts->channel[0].use_modem_pins_by_default = false;

	if (of_property_read_bool(np, "disable-modem-pins-on-startup-b"))
		ts->channel[1].use_modem_pins_by_default = false;


	return 0;
}
#else
static inline int sc16is7x2_probe_dt(struct sc16is7x2_chip *ts,
		struct spi_device *spi)
{
	return 1;
}
#endif

static void sc16is7x2_probe_pdata(struct sc16is7x2_chip *ts,
		struct spi_device *spi)
{
	struct sc16is7x2_platform_data *pdata = dev_get_platdata(&spi->dev);

	if (!pdata)
		return;

	ts->uart_base = pdata->uart_base;
	ts->gpio_base = pdata->gpio_base;
	ts->uartclk = pdata->uartclk;
	ts->gpio_label  = pdata->label;
	ts->gpio_names  = pdata->names;
}



static int sc16is7x2_probe(struct spi_device *spi)
{
	struct sc16is7x2_chip *ts;
	//~ struct sc16is7x2_platform_data fake_pdata;

	int ret;
	unsigned char ch;

	/* Only even uart base numbers are supported */
	dev_info(&spi->dev, "probe start\n");

	ts = kzalloc(sizeof(struct sc16is7x2_chip), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	spi_set_drvdata(spi, ts);
	ts->spi = spi;

	for (ch = 0; ch < 2; ++ch) {
		ts->channel[ch].use_modem_pins_by_default = true;
		ts->channel[ch].console_enabled = false;
	}

	ret = sc16is7x2_probe_dt(ts, spi);
	if (ret > 0)
		sc16is7x2_probe_pdata(ts, spi);
	else if (ret < 0)
		return ret;




	if (!ts->gpio_base) {
		dev_err(&spi->dev, "incorrect gpio_base\n");
		return -EINVAL;
	}

	if (ts->uart_base & 1) {
		dev_err(&spi->dev, "incorrect uart_base\n");
		return -EINVAL;
	}





	/* Reset the chip */
	sc16is7x2_write(ts, REG_IOC, 0, IOC_SRESET);

	/* disable all GPIOs, enable on request */
	ts->io_control = 0;
	if (ts->channel[0].use_modem_pins_by_default)
		ts->io_control |= IOC_GPIO30;

	if (ts->channel[1].use_modem_pins_by_default)
		ts->io_control |= IOC_GPIO74;

	sc16is7x2_write(ts, REG_IOC, 0, ts->io_control);


	ret = sc16is7x2_register_uart_port(ts, 0);
	if (ret)
		goto exit_destroy;

	ret = sc16is7x2_register_uart_port(ts, 1);
	if (ret)
		goto exit_uart0;

	ret = sc16is7x2_register_gpio(ts);
	if (ret)
		goto exit_uart1;




	dev_info(&spi->dev, DRIVER_NAME " at CS%d (irq %d), 2 UARTs, 8 GPIOs\n"
			"    ttyNSC%d, ttyNSC%d, gpiochip%d\n",
			spi->chip_select, spi->irq,
			ts->uart_base, ts->uart_base + 1,
			ts->gpio_base);


//~
	//~ struct timespec ts_start,ts_end,test_of_time;
//~
	//~ int i, n;
	//~ n = 50000;
//~
	//~ getnstimeofday(&ts_start);
	//~ for (i = 0; i < n; ++i) {
		//~ gpio_set_value(17, 0);
		//~ gpio_set_value(17, 1);
	//~ }
    //~ getnstimeofday(&ts_end);
    //~ test_of_time = timespec_sub(ts_end,ts_start);
    //~ printk("2x%d gpio toggle took %lu ns\n", n, test_of_time.tv_nsec);







	return 0;

exit_uart1:
	sc16is7x2_remove_one_port(ts, 1);

exit_uart0:
	sc16is7x2_remove_one_port(ts, 0);

exit_destroy:
	dev_set_drvdata(&spi->dev, NULL);
	kfree(ts);
	return ret;
}

static int sc16is7x2_remove(struct spi_device *spi)
{
	struct sc16is7x2_chip *ts = spi_get_drvdata(spi);
	int ret;

	if (ts == NULL)
		return -ENODEV;

	ret = sc16is7x2_remove_one_port(ts, 0);
	if (ret)
		return ret;

	ret = sc16is7x2_remove_one_port(ts, 1);
	if (ret)
		return ret;

#ifdef CONFIG_GPIOLIB
	ret = gpiochip_remove(&ts->gpio);
	if (ret)
		return ret;
#endif

	kfree(ts);

	return 0;
}





static inline int __uart_put_char(struct uart_port *port,
				struct circ_buf *circ, unsigned char c)
{
	unsigned long flags;
	int ret = 0;

	if (!circ->buf)
		return 0;

	spin_lock_irqsave(&port->lock, flags);
	if (uart_circ_chars_free(circ) != 0) {
		circ->buf[circ->head] = c;
		circ->head = (circ->head + 1) & (UART_XMIT_SIZE - 1);
		ret = 1;
	}
	spin_unlock_irqrestore(&port->lock, flags);
	return ret;
}

static int uart_put_char(struct tty_struct *tty, unsigned char ch)
{
	struct uart_state *state = tty->driver_data;

	return __uart_put_char(state->uart_port, &state->xmit, ch);
}

static void sc16is7x2_console_putchar(struct uart_port *port, int character)
{
	struct sc16is7x2_channel *chan =  to_sc16is7x2_channel(port);
	unsigned ch = (chan == chan->chip->channel) ? 0 : 1;

	sc16is7x2_write(chan->chip, UART_TX, ch, character);



	//~ struct tty_port *tty_port = &ch->uart.state->port;
	//~ struct tty_struct *tty = tty_port_tty_get(tty_port);
	//~ struct uart_state *state = tty->driver_data;
	//~ printk("test state=%d\n", state);
	//~ return __uart_put_char(state->uart_port, &state->xmit, ch);




}

static void
sc16is7x2_console_write(struct console *co, const char *s, unsigned int count)
{

	struct sc16is7x2_channel *ch = sc16is7x2_channels[co->index];

	uart_console_write(&ch->uart, s, count, sc16is7x2_console_putchar);

}
static int __init sc16is7x2_console_setup(struct console *co, char *options)
{
	//~ printk("sc16is7x2_console_setup co=%d options=%s\n", co, options);
	printk("sc16is7x2_console_setup\n");
	struct sc16is7x2_channel *ch;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';
	int ret;
	/*
	 * Check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	 */

	if (co->index >= MAX_SC16IS7X2)
		co->index = 0;

	ch = sc16is7x2_channels[co->index];
	if (!ch)
		return -ENODEV;


	if (ch->console_enabled) {
		return 0;
	}
	ch->console_enabled = true;

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	printk("setup ch->uart=%d, co=%d\n", &ch->uart, co);

	sc16is7x2_startup(&ch->uart);




	return uart_set_options(&ch->uart, co, baud, parity, bits, flow);
	return 0;
}




static struct uart_driver sc16is7x2_uart_driver;
#ifndef MODULE
static struct console sc16is7x2_console = {
	.name		= "ttyNSC",
	.write		= sc16is7x2_console_write,
	.device		= uart_console_device,
	.setup		= sc16is7x2_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &sc16is7x2_uart_driver,
};

#define SC16IS7x2_CONSOLE	(&sc16is7x2_console)
#endif

static struct uart_driver sc16is7x2_uart_driver = {
	.owner          = THIS_MODULE,
	.driver_name    = DRIVER_NAME,
	.dev_name       = "ttyNSC",
	.nr             = MAX_SC16IS7X2,
#ifndef MODULE
	.cons			= SC16IS7x2_CONSOLE,
#endif
};





static struct of_device_id spi_sc16is7x2_dt_ids[] = {
	{ .compatible = "fsl,spi-sc16is7x2", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, spi_sc16is7x2_dt_ids);

/* Spi driver data */
static struct spi_driver sc16is7x2_spi_driver = {
	.driver = {
		.name		= DRIVER_NAME,
		.bus		= &spi_bus_type,
		.owner		= THIS_MODULE,
		.of_match_table = spi_sc16is7x2_dt_ids,
	},
	.probe		= sc16is7x2_probe,
	.remove		= sc16is7x2_remove,
};


/* Driver init function */
static int __init sc16is7x2_init(void)
{
	int ret = uart_register_driver(&sc16is7x2_uart_driver);
	if (ret)
		return ret;

	return spi_register_driver(&sc16is7x2_spi_driver);
}

/* Driver exit function */
static void __exit sc16is7x2_exit(void)
{
	spi_unregister_driver(&sc16is7x2_spi_driver);
	uart_unregister_driver(&sc16is7x2_uart_driver);
}

/* register after spi postcore initcall and before
 * subsys initcalls that may rely on these GPIOs
 */

subsys_initcall(sc16is7x2_init);
module_exit(sc16is7x2_exit);

MODULE_AUTHOR("Manuel Stahl");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("SC16IS7x2 SPI based UART chip");
MODULE_ALIAS("spi:" DRIVER_NAME);
