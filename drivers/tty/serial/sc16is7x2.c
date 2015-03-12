/**
 * drivers/serial/sc16is7x2.c
 *
 * Copyright (C) 2009 Manuel Stahl <manuel.stahl@iis.fraunhofer.de>
 * Copyright (C) 2013 Evgeny Boger <boger@contactless.ru>
 * Copyright (C) 2015 Plyaskin Stepan <strelok@e-kirov.ru>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * The SC16IS7x2 device is a SPI driven dual UART with GPIOs.
 *
 * The driver exports two uarts and a gpiochip interface.
 */

//#define DEBUG

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
#include <linux/delay.h>
#include <linux/completion.h>
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

/* SC16IS7XX register definitions */
#define SC16IS7XX_RHR_REGADDR		(0x00) /* RX FIFO */
#define SC16IS7XX_THR_REGADDR		(0x00) /* TX FIFO */
#define SC16IS7XX_IER_REGADDR		(0x01) /* Interrupt enable */
#define SC16IS7XX_IIR_REGADDR		(0x02) /* Interrupt Identification */
#define SC16IS7XX_FCR_REGADDR		(0x02) /* FIFO control */
#define SC16IS7XX_LCR_REGADDR		(0x03) /* Line Control */
#define SC16IS7XX_MCR_REGADDR		(0x04) /* Modem Control */
#define SC16IS7XX_LSR_REGADDR		(0x05) /* Line Status */
#define SC16IS7XX_MSR_REGADDR		(0x06) /* Modem Status */
#define SC16IS7XX_SPR_REGADDR		(0x07) /* Scratch Pad */
#define SC16IS7XX_TXLVL_REGADDR		(0x08) /* TX FIFO level */
#define SC16IS7XX_RXLVL_REGADDR		(0x09) /* RX FIFO level */
#define SC16IS7XX_IODIR_REGADDR		(0x0a) /* I/O Direction
						* - only on 75x/76x
						*/
#define SC16IS7XX_IOSTATE_REGADDR		(0x0b) /* I/O State
						* - only on 75x/76x
						*/
#define SC16IS7XX_IOINTENA_REGADDR		(0x0c) /* I/O Interrupt Enable
						* - only on 75x/76x
						*/
#define SC16IS7XX_IOCONTROL_REGADDR		(0x0e) /* I/O Control
						* - only on 75x/76x
						*/
#define SC16IS7XX_EFCR_REGADDR		(0x0f) /* Extra Features Control */

/* TCR/TLR Register set: Only if ((MCR[2] == 1) && (EFR[4] == 1)) */
#define SC16IS7XX_TCR_REGADDR		(0x06) /* Transmit control */
#define SC16IS7XX_TLR_REGADDR		(0x07) /* Trigger level */

/* Special Register set: Only if ((LCR[7] == 1) && (LCR != 0xBF)) */
#define SC16IS7XX_DLL_REGADDR		(0x00) /* Divisor Latch Low */
#define SC16IS7XX_DLH_REGADDR		(0x01) /* Divisor Latch High */

/* Enhanced Register set: Only if (LCR == 0xBF) */
#define SC16IS7XX_EFR_REGADDR		(0x02) /* Enhanced Features */
#define SC16IS7XX_XON1_REGADDR		(0x04) /* Xon1 word */
#define SC16IS7XX_XON2_REGADDR		(0x05) /* Xon2 word */
#define SC16IS7XX_XOFF1_REGADDR		(0x06) /* Xoff1 word */
#define SC16IS7XX_XOFF2_REGADDR		(0x07) /* Xoff2 word */

/* Number of channel register */
#define SC16IS7XX_RHR_REG		(0) /* RX FIFO */
#define SC16IS7XX_THR_REG		(1) /* TX FIFO */
#define SC16IS7XX_IER_REG		(2) /* Interrupt enable */
#define SC16IS7XX_IIR_REG		(3) /* Interrupt Identification */
#define SC16IS7XX_FCR_REG		(4) /* FIFO control */
#define SC16IS7XX_LCR_REG		(5) /* Line Control */
#define SC16IS7XX_MCR_REG		(6) /* Modem Control */
#define SC16IS7XX_LSR_REG		(7) /* Line Status */
#define SC16IS7XX_MSR_REG		(8) /* Modem Status */
#define SC16IS7XX_SPR_REG		(9) /* Scratch Pad */
#define SC16IS7XX_TXLVL_REG		(10) /* TX FIFO level */
#define SC16IS7XX_RXLVL_REG		(11) /* RX FIFO level */
#define SC16IS7XX_EFCR_REG		(12) /* Extra Features Control */
#define SC16IS7XX_TCR_REG		(13) /* Transmit control */
#define SC16IS7XX_TLR_REG		(14) /* Trigger level */
#define SC16IS7XX_DLL_REG		(15) /* Divisor Latch Low */
#define SC16IS7XX_DLH_REG		(16) /* Divisor Latch High */
#define SC16IS7XX_EFR_REG		(17) /* Enhanced Features */
#define SC16IS7XX_XON1_REG		(18) /* Xon1 word */
#define SC16IS7XX_XON2_REG		(19) /* Xon2 word */
#define SC16IS7XX_XOFF1_REG		(20) /* Xoff1 word */
#define SC16IS7XX_XOFF2_REG		(21) /* Xoff2 word */

#define SC16IS7XX_CHANNEL_REG_NUM	(22) /* Count of chanell registers */


/* Number of chip register */
#define SC16IS7XX_IODIR_REG		(0) /* I/O Direction */
#define SC16IS7XX_IOSTATE_REG		(1) /* I/O State */
#define SC16IS7XX_IOINTENA_REG		(2) /* I/O Interrupt Enable */
#define SC16IS7XX_IOCONTROL_REG		(3) /* I/O Control
						* - only on 75x/76x
						*/
#define SC16IS7XX_CHIP_REG_NUM	(4) /* Count of chanell registers */

/* IER register bits */
#define SC16IS7XX_IER_RDI_BIT		(1 << 0) /* Enable RX data interrupt */
#define SC16IS7XX_IER_THRI_BIT		(1 << 1) /* Enable TX holding register
						  * interrupt */
#define SC16IS7XX_IER_RLSI_BIT		(1 << 2) /* Enable RX line status
						  * interrupt */
#define SC16IS7XX_IER_MSI_BIT		(1 << 3) /* Enable Modem status
						  * interrupt */

/* IER register bits - write only if (EFR[4] == 1) */
#define SC16IS7XX_IER_SLEEP_BIT		(1 << 4) /* Enable Sleep mode */
#define SC16IS7XX_IER_XOFFI_BIT		(1 << 5) /* Enable Xoff interrupt */
#define SC16IS7XX_IER_RTSI_BIT		(1 << 6) /* Enable nRTS interrupt */
#define SC16IS7XX_IER_CTSI_BIT		(1 << 7) /* Enable nCTS interrupt */

/* FCR register bits */
#define SC16IS7XX_FCR_FIFO_BIT		(1 << 0) /* Enable FIFO */
#define SC16IS7XX_FCR_RXRESET_BIT	(1 << 1) /* Reset RX FIFO */
#define SC16IS7XX_FCR_TXRESET_BIT	(1 << 2) /* Reset TX FIFO */
#define SC16IS7XX_FCR_RXLVLL_BIT	(1 << 6) /* RX Trigger level LSB */
#define SC16IS7XX_FCR_RXLVLH_BIT	(1 << 7) /* RX Trigger level MSB */

/* FCR register bits - write only if (EFR[4] == 1) */
#define SC16IS7XX_FCR_TXLVLL_BIT	(1 << 4) /* TX Trigger level LSB */
#define SC16IS7XX_FCR_TXLVLH_BIT	(1 << 5) /* TX Trigger level MSB */

/* IIR register bits */
#define SC16IS7XX_IIR_NO_INT_BIT	(1 << 0) /* No interrupts pending */
#define SC16IS7XX_IIR_ID_MASK		0x3f     /* Mask for the interrupt ID */
#define SC16IS7XX_IIR_THRI_SRC		0x02     /* TX holding register empty */
#define SC16IS7XX_IIR_RDI_SRC		0x04     /* RX data interrupt */
#define SC16IS7XX_IIR_RLSE_SRC		0x06     /* RX line status error */
#define SC16IS7XX_IIR_RTOI_SRC		0x0c     /* RX time-out interrupt */
#define SC16IS7XX_IIR_MSI_SRC		0x00     /* Modem status interrupt
						  * - only on 75x/76x
						  */
#define SC16IS7XX_IIR_INPIN_SRC		0x30     /* Input pin change of state
						  * - only on 75x/76x
						  */
#define SC16IS7XX_IIR_XOFFI_SRC		0x10     /* Received Xoff */
#define SC16IS7XX_IIR_CTSRTS_SRC	0x20     /* nCTS,nRTS change of state
						  * from active (LOW)
						  * to inactive (HIGH)
						  */
/* LCR register bits */
#define SC16IS7XX_LCR_LENGTH0_BIT	(1 << 0) /* Word length bit 0 */
#define SC16IS7XX_LCR_LENGTH1_BIT	(1 << 1) /* Word length bit 1
						  *
						  * Word length bits table:
						  * 00 -> 5 bit words
						  * 01 -> 6 bit words
						  * 10 -> 7 bit words
						  * 11 -> 8 bit words
						  */
#define SC16IS7XX_LCR_STOPLEN_BIT	(1 << 2) /* STOP length bit
						  *
						  * STOP length bit table:
						  * 0 -> 1 stop bit
						  * 1 -> 1-1.5 stop bits if
						  *      word length is 5,
						  *      2 stop bits otherwise
						  */
#define SC16IS7XX_LCR_PARITY_BIT	(1 << 3) /* Parity bit enable */
#define SC16IS7XX_LCR_EVENPARITY_BIT	(1 << 4) /* Even parity bit enable */
#define SC16IS7XX_LCR_FORCEPARITY_BIT	(1 << 5) /* 9-bit multidrop parity */
#define SC16IS7XX_LCR_TXBREAK_BIT	(1 << 6) /* TX break enable */
#define SC16IS7XX_LCR_DLAB_BIT		(1 << 7) /* Divisor Latch enable */
#define SC16IS7XX_LCR_WORD_LEN_5	(0x00)
#define SC16IS7XX_LCR_WORD_LEN_6	(0x01)
#define SC16IS7XX_LCR_WORD_LEN_7	(0x02)
#define SC16IS7XX_LCR_WORD_LEN_8	(0x03)
#define SC16IS7XX_LCR_CONF_MODE_A	SC16IS7XX_LCR_DLAB_BIT /* Special
								* reg set */
#define SC16IS7XX_LCR_CONF_MODE_B	0xBF                   /* Enhanced
								* reg set */

/* MCR register bits */
#define SC16IS7XX_MCR_DTR_BIT		(1 << 0) /* DTR complement
						  * - only on 75x/76x
						  */
#define SC16IS7XX_MCR_RTS_BIT		(1 << 1) /* RTS complement */
#define SC16IS7XX_MCR_TCRTLR_BIT	(1 << 2) /* TCR/TLR register enable */
#define SC16IS7XX_MCR_LOOP_BIT		(1 << 4) /* Enable loopback test mode */
#define SC16IS7XX_MCR_XONANY_BIT	(1 << 5) /* Enable Xon Any
						  * - write enabled
						  * if (EFR[4] == 1)
						  */
#define SC16IS7XX_MCR_IRDA_BIT		(1 << 6) /* Enable IrDA mode
						  * - write enabled
						  * if (EFR[4] == 1)
						  */
#define SC16IS7XX_MCR_CLKSEL_BIT	(1 << 7) /* Divide clock by 4
						  * - write enabled
						  * if (EFR[4] == 1)
						  */

/* LSR register bits */
#define SC16IS7XX_LSR_DR_BIT		(1 << 0) /* Receiver data ready */
#define SC16IS7XX_LSR_OE_BIT		(1 << 1) /* Overrun Error */
#define SC16IS7XX_LSR_PE_BIT		(1 << 2) /* Parity Error */
#define SC16IS7XX_LSR_FE_BIT		(1 << 3) /* Frame Error */
#define SC16IS7XX_LSR_BI_BIT		(1 << 4) /* Break Interrupt */
#define SC16IS7XX_LSR_BRK_ERROR_MASK	0x1E     /* BI, FE, PE, OE bits */
#define SC16IS7XX_LSR_THRE_BIT		(1 << 5) /* TX holding register empty */
#define SC16IS7XX_LSR_TEMT_BIT		(1 << 6) /* Transmitter empty */
#define SC16IS7XX_LSR_FIFOE_BIT		(1 << 7) /* Fifo Error */

/* MSR register bits */
#define SC16IS7XX_MSR_DCTS_BIT		(1 << 0) /* Delta CTS Clear To Send */
#define SC16IS7XX_MSR_DDSR_BIT		(1 << 1) /* Delta DSR Data Set Ready
						  * or (IO4)
						  * - only on 75x/76x
						  */
#define SC16IS7XX_MSR_DRI_BIT		(1 << 2) /* Delta RI Ring Indicator
						  * or (IO7)
						  * - only on 75x/76x
						  */
#define SC16IS7XX_MSR_DCD_BIT		(1 << 3) /* Delta CD Carrier Detect
						  * or (IO6)
						  * - only on 75x/76x
						  */
#define SC16IS7XX_MSR_CTS_BIT		(1 << 0) /* CTS */
#define SC16IS7XX_MSR_DSR_BIT		(1 << 1) /* DSR (IO4)
						  * - only on 75x/76x
						  */
#define SC16IS7XX_MSR_RI_BIT		(1 << 2) /* RI (IO7)
						  * - only on 75x/76x
						  */
#define SC16IS7XX_MSR_CD_BIT		(1 << 3) /* CD (IO6)
						  * - only on 75x/76x
						  */
#define SC16IS7XX_MSR_DELTA_MASK	0x0F     /* Any of the delta bits! */

/*
 * TCR register bits
 * TCR trigger levels are available from 0 to 60 characters with a granularity
 * of four.
 * The programmer must program the TCR such that TCR[3:0] > TCR[7:4]. There is
 * no built-in hardware check to make sure this condition is met. Also, the TCR
 * must be programmed with this condition before auto RTS or software flow
 * control is enabled to avoid spurious operation of the device.
 */
#define SC16IS7XX_TCR_RX_HALT(words)	((((words) / 4) & 0x0f) << 0)
#define SC16IS7XX_TCR_RX_RESUME(words)	((((words) / 4) & 0x0f) << 4)

/*
 * TLR register bits
 * If TLR[3:0] or TLR[7:4] are logical 0, the selectable trigger levels via the
 * FIFO Control Register (FCR) are used for the transmit and receive FIFO
 * trigger levels. Trigger levels from 4 characters to 60 characters are
 * available with a granularity of four.
 *
 * When the trigger level setting in TLR is zero, the SC16IS740/750/760 uses the
 * trigger level setting defined in FCR. If TLR has non-zero trigger level value
 * the trigger level defined in FCR is discarded. This applies to both transmit
 * FIFO and receive FIFO trigger level setting.
 *
 * When TLR is used for RX trigger level control, FCR[7:6] should be left at the
 * default state, that is, '00'.
 */
#define SC16IS7XX_TLR_TX_TRIGGER(words)	((((words) / 4) & 0x0f) << 0)
#define SC16IS7XX_TLR_RX_TRIGGER(words)	((((words) / 4) & 0x0f) << 4)

/* IOControl register bits (Only 750/760) */
#define SC16IS7XX_IOCONTROL_LATCH_BIT	(1 << 0) /* Enable input latching */
#define SC16IS7XX_IOCONTROL_GPIO74_BIT	(1 << 1) /* Enable GPIO[7:4] */
#define SC16IS7XX_IOCONTROL_GPIO30_BIT	(1 << 2) /* Enable GPIO[3:0] */
#define SC16IS7XX_IOCONTROL_SRESET_BIT	(1 << 3) /* Software Reset */

/* EFCR register bits */
#define SC16IS7XX_EFCR_9BIT_MODE_BIT	(1 << 0) /* Enable 9-bit or Multidrop
						  * mode (RS485) */
#define SC16IS7XX_EFCR_RXDISABLE_BIT	(1 << 1) /* Disable receiver */
#define SC16IS7XX_EFCR_TXDISABLE_BIT	(1 << 2) /* Disable transmitter */
#define SC16IS7XX_EFCR_AUTO_RS485_BIT	(1 << 4) /* Auto RS485 RTS direction */
#define SC16IS7XX_EFCR_RTS_INVERT_BIT	(1 << 5) /* RTS output inversion */
#define SC16IS7XX_EFCR_IRDA_MODE_BIT	(1 << 7) /* IrDA mode
						  * 0 = rate upto 115.2 kbit/s
						  *   - Only 750/760
						  * 1 = rate upto 1.152 Mbit/s
						  *   - Only 760
						  */

/* EFR register bits */
#define SC16IS7XX_EFR_AUTORTS_BIT	(1 << 6) /* Auto RTS flow ctrl enable */
#define SC16IS7XX_EFR_AUTOCTS_BIT	(1 << 7) /* Auto CTS flow ctrl enable */
#define SC16IS7XX_EFR_XOFF2_DETECT_BIT	(1 << 5) /* Enable Xoff2 detection */
#define SC16IS7XX_EFR_ENABLE_BIT	(1 << 4) /* Enable enhanced functions
						  * and writing to IER[7:4],
						  * FCR[5:4], MCR[7:5]
						  */
#define SC16IS7XX_EFR_SWFLOW3_BIT	(1 << 3) /* SWFLOW bit 3 */
#define SC16IS7XX_EFR_SWFLOW2_BIT	(1 << 2) /* SWFLOW bit 2
						  *
						  * SWFLOW bits 3 & 2 table:
						  * 00 -> no transmitter flow
						  *       control
						  * 01 -> transmitter generates
						  *       XON2 and XOFF2
						  * 10 -> transmitter generates
						  *       XON1 and XOFF1
						  * 11 -> transmitter generates
						  *       XON1, XON2, XOFF1 and
						  *       XOFF2
						  */
#define SC16IS7XX_EFR_SWFLOW1_BIT	(1 << 1) /* SWFLOW bit 2 */
#define SC16IS7XX_EFR_SWFLOW0_BIT	(1 << 0) /* SWFLOW bit 3
						  *
						  * SWFLOW bits 3 & 2 table:
						  * 00 -> no received flow
						  *       control
						  * 01 -> receiver compares
						  *       XON2 and XOFF2
						  * 10 -> receiver compares
						  *       XON1 and XOFF1
						  * 11 -> receiver compares
						  *       XON1, XON2, XOFF1 and
						  *       XOFF2
						  */

/* Misc definitions */
#define SC16IS7XX_FIFO_SIZE		(64)
#define SC16IS7XX_REG_SHIFT		2

struct sc16is7x2_chip;

/**
 * only for reading struct
 */
struct sc16is7x2_read_message {
	/* spi structs */
	struct spi_message	m;
	struct spi_transfer	t[2];
	u8		readcmd;		// tx_buf
	u8		data;			// rx_buf
	struct sc16is7x2_reg *reg;
	void (*complete) (void*, struct sc16is7x2_read_message*);
	void  *context;
	struct sc16is7x2_chip *chip;
};

/**
 * only for writing struct
 */
struct sc16is7x2_write_message {
	/* spi structs */
	struct spi_message	m;
	struct spi_transfer	t;
	u8		data[2];		// tx_buf
	bool	completion;		// "barrier"
	struct sc16is7x2_reg *reg;
	void (*complete) (void*, struct sc16is7x2_write_message*);
	void  *context;
	struct sc16is7x2_chip *chip;
};

#define SC16IS7X2_READ_MESSAGE_QUEUE_SIZE	(16)
struct sc16is7x2_read_message_queue {
	struct sc16is7x2_read_message queue[SC16IS7X2_READ_MESSAGE_QUEUE_SIZE];
	bool	completion;		// "barrier"
	struct completion	work;
	struct sc16is7x2_read_message *wait_msg;
	u8		begin;
	u8		end;
};

#define SC16IS7X2_WRITE_MESSAGE_QUEUE_SIZE	(16)
struct sc16is7x2_write_message_queue  {
	struct sc16is7x2_write_message queue[SC16IS7X2_WRITE_MESSAGE_QUEUE_SIZE];
	bool	completion;		// "barrier"
	struct completion	work;
	struct sc16is7x2_write_message *wait_msg;
	u8		begin;
	u8		end;
};

/**
 * struct sc16is7x2_reg
 * for writing
 *
 */
struct sc16is7x2_reg {
	u8 readcmd;
	u8 writecmd;
	u8 val;
	char name[15];
};

struct sc16is7x2_channel {
	struct sc16is7x2_chip	*chip;	/* back link */
	struct uart_port	uart;

	u8		rx_buf[FIFO_SIZE];

	u8		tx_buf[(FIFO_SIZE>>1)+1];

	/* channel initiallited and works */
	bool	started;
	/* true if IIR reading async rught now */
	bool	iir_reading;
	/* true if rx buffer reading async rught now */
	bool	rx_reading;
	/* true if tx buffer sending async rught now */
	bool	tx_writing;
	/* uart have unhandled chars */
	bool	tx_buffer_wait;

	u16		quot;		/* baud rate divisor */

	/* registers */
	struct sc16is7x2_reg regs[SC16IS7XX_CHANNEL_REG_NUM];

	bool use_modem_pins_by_default;
	bool console_enabled;
};

struct sc16is7x2_chip {
	spinlock_t		spi_lock;
	spinlock_t		reg_lock;

	struct spi_device *spi;
	struct sc16is7x2_channel channel[2];

	struct sc16is7x2_read_message_queue		rq;
	struct sc16is7x2_write_message_queue	wq;

	unsigned int	uartclk;
	/* uart line number of the first channel */
	unsigned	uart_base;
	/* number assigned to the first GPIO */
	unsigned	gpio_base;
	/* number assigned to the first GPIO */
	unsigned		irq_base;

	char		*gpio_label;
	/* list of GPIO names (array length = SC16IS7X2_NR_GPIOS) */
	const char	*const *gpio_names;

	struct sc16is7x2_reg regs[SC16IS7XX_CHIP_REG_NUM];

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

/* ***************************** SPI ASYNC ****************************** */

static inline struct sc16is7x2_write_message*
sc16is7x2_get_write_message(struct sc16is7x2_chip *ts){
	return &ts->wq.queue[ts->wq.end];
}

static inline struct sc16is7x2_read_message*
sc16is7x2_get_read_message(struct sc16is7x2_chip *ts){
	return &ts->rq.queue[ts->rq.end];
}

static inline void
sc16is7x2_reg_set_val(struct sc16is7x2_reg *reg, u8 val)
{
	reg->val = val;
}

static inline void
sc16is7x2_reg_set_bits(struct sc16is7x2_reg *reg, u8 mask, u8 val)
{
	reg->val &= ~mask;
	reg->val |= val;
}

static inline u8
sc16is7x2_reg_get_val(struct sc16is7x2_reg *reg)
{
	return reg->val;
}

static inline struct sc16is7x2_reg*
sc16is7x2_get_reg(struct sc16is7x2_channel *chan, u8 reg)
{
	return &chan->regs[reg];
}

/*
static inline u8
sc16is7x2_reg_get_val_from_buf(struct sc16is7x2_reg *reg)
{
	return reg->buf[1];
}
 */

static void sc16is7x2_read_message_complete(void *data)
{
	struct sc16is7x2_read_message *rm = data;
	struct sc16is7x2_reg *reg = rm->reg;
	struct sc16is7x2_chip *ts = rm->chip;
	unsigned ch = (reg->writecmd & 0x2) == 0 ? 0 : 1;
	//struct sc16is7x2_channel *chan = &ts->channel[ch];
	dev_dbg(&ts->spi->dev, "%s %s", __func__, reg->name);

	if(*rm->complete){
		rm->complete(rm->context, rm);
	}
/*
	if(ts->rq.wait_msg == rm){
		dev_dbg(&ts->spi->dev, "%s ch%d: %s done", __func__, ch, reg->name);
		ts->rq.wait_msg = NULL;
		complete(&ts->rq.work);
	}
*/
	ts->rq.begin++;
	if(ts->rq.begin >= SC16IS7X2_READ_MESSAGE_QUEUE_SIZE)
		ts->rq.begin -= SC16IS7X2_READ_MESSAGE_QUEUE_SIZE;
#if 0
	dev_dbg(&ts->spi->dev, "%s ch%d: b %d; e %d", __func__,
			ch, ts->rq.begin, ts->rq.end);
#endif
}

static void sc16is7x2_write_message_complete(void *data)
{
	struct sc16is7x2_write_message *wm = data;
	struct sc16is7x2_reg *reg = wm->reg;
	struct sc16is7x2_chip *ts = wm->chip;
	unsigned ch = (reg->writecmd & 0x2) == 0 ? 0 : 1;
#if 0
	struct sc16is7x2_channel *chan = &ts->channel[ch];
#endif
	dev_dbg(&ts->spi->dev, "%s %s", __func__, reg->name);

	if(*wm->complete){
		wm->complete(wm->context, wm);
}
/*
	if(ts->wq.wait_msg == wm){
		dev_dbg(&ts->spi->dev, "%s ch%d: %s done", __func__, ch, reg->name);
		ts->wq.wait_msg = NULL;
		complete_all(&ts->wq.work);
	}
*/
	ts->wq.begin++;
	if(ts->wq.begin >= SC16IS7X2_WRITE_MESSAGE_QUEUE_SIZE)
		ts->wq.begin -= SC16IS7X2_WRITE_MESSAGE_QUEUE_SIZE;

#if 0
	dev_dbg(&ts->spi->dev, "%s ch%d: b %d; e %d", __func__,
			ch, ts->wq.begin, ts->wq.end);
#endif
}

static inline void
sc16is7x2_write_message_set_tx(struct sc16is7x2_write_message *wm, u8 *buf, u8 len)
{
	wm->t.tx_buf = buf;
	wm->t.len = len;
}

static inline void
sc16is7x2_read_message_set_rx(struct sc16is7x2_read_message *rm, u8 *buf, u8 len)
{
	rm->t[1].rx_buf = buf;
	rm->t[1].len = len;
}

static inline int
sc16is7x2_write_message_run(struct sc16is7x2_chip *ts)
{
	char queue_len;
	unsigned ret;
	struct sc16is7x2_write_message *wm = sc16is7x2_get_write_message(ts);
	struct sc16is7x2_write_message_queue *wq = &ts->wq;
	ret = spi_async(ts->spi, &wm->m);

	wq->end++;
	if(wq->end >= SC16IS7X2_WRITE_MESSAGE_QUEUE_SIZE)
		wq->end -= SC16IS7X2_WRITE_MESSAGE_QUEUE_SIZE;

	if(wq->end < wq->begin){
		queue_len = wq->end + SC16IS7X2_WRITE_MESSAGE_QUEUE_SIZE;
		queue_len -= wq->begin;
	} else
		queue_len = wq->end - wq->begin;

//#if 0
	dev_dbg(&ts->spi->dev, "%s: b %d; e %d", __func__,
			ts->wq.begin, ts->wq.end);
//#endif

	if(queue_len >= (SC16IS7X2_WRITE_MESSAGE_QUEUE_SIZE >> 1)){
		if(!wq->completion && !in_irq()){
			//wq->wait_msg = wm;
			wq->completion = true;

			//init_completion(&wq->work);
			dev_dbg(&ts->spi->dev, "%s: queue %d - entering", __func__,
				queue_len);
			schedule();
			//wait_for_completion(&wq->work);
			dev_dbg(&ts->spi->dev, "%s: complete", __func__);


			wq->wait_msg = NULL;
			wq->completion = false;
		}
}

	return ret;
}

static inline int
sc16is7x2_read_message_run(struct sc16is7x2_chip *ts)
{
	char queue_len;
	unsigned ret;
	struct sc16is7x2_read_message *rm = sc16is7x2_get_read_message(ts);
	struct sc16is7x2_read_message_queue *rq = &ts->rq;
	ret = spi_async(ts->spi, &rm->m);

	rq->end++;
	if(rq->end >= SC16IS7X2_READ_MESSAGE_QUEUE_SIZE)
		rq->end -= SC16IS7X2_READ_MESSAGE_QUEUE_SIZE;

	if(rq->end < rq->begin){
		queue_len = rq->end + SC16IS7X2_READ_MESSAGE_QUEUE_SIZE;
		queue_len -= rq->begin;
	} else
		queue_len = rq->end - rq->begin;

//#if 10
	dev_dbg(&ts->spi->dev, "%s: b %d; e %d", __func__,
			ts->rq.begin, ts->rq.end);
//#endif

	if(queue_len >= (SC16IS7X2_READ_MESSAGE_QUEUE_SIZE >> 1)){
		if(!rq->completion && !in_irq()){
			//rq->wait_msg = rm;
			rq->completion = true;

			//init_completion(&rq->work);
			dev_dbg(&ts->spi->dev, "%s: queue %d - entering", __func__,
				queue_len);

			schedule();
			//wait_for_completion(&rq->work);
			dev_dbg(&ts->spi->dev, "%s: complete", __func__);

			rq->completion = false;
			rq->wait_msg = NULL;
		}
	}
	return ret;
}

static void
sc16is7x2_read_reg(struct sc16is7x2_chip *ts,
							struct sc16is7x2_reg *reg,
							void (*complete)(void*, struct sc16is7x2_read_message*),
							void *context)
{
	unsigned ch = (reg->writecmd & 0x2) == 0 ? 0 : 1;
	struct sc16is7x2_read_message *rm = sc16is7x2_get_read_message(ts);
	dev_dbg(&ts->spi->dev, "%s ch%d: %s", __func__, ch, reg->name);

	rm->t[1].rx_buf = &rm->data;
	rm->t[1].len = 1;
	rm->readcmd = reg->readcmd;
	rm->reg = reg;
	rm->complete = complete;
	rm->context = context;

	sc16is7x2_read_message_run(ts);
}

static void
sc16is7x2_read_message_rx(struct sc16is7x2_chip *ts,
							struct sc16is7x2_reg *reg,
							u8 *buf, u8 len,
							void (*complete)(void*, struct sc16is7x2_read_message*),
							void *context)
{
	unsigned ch = (reg->writecmd & 0x2) == 0 ? 0 : 1;
	struct sc16is7x2_read_message *rm = sc16is7x2_get_read_message(ts);
	dev_dbg(&ts->spi->dev, "%s ch%d: %s", __func__, ch, reg->name);

	rm->t[1].rx_buf = buf;
	rm->t[1].len = len;
	rm->readcmd = reg->readcmd;
	rm->reg = reg;
	rm->complete = complete;
	rm->context = context;

	sc16is7x2_read_message_run(ts);
}

static void
sc16is7x2_write_reg(struct sc16is7x2_chip *ts,
					struct sc16is7x2_reg *reg, u8 val,
					void (*complete)(void*, struct sc16is7x2_write_message*),
					void *context)
{
	unsigned ch = (reg->writecmd & 0x2) == 0 ? 0 : 1;
	struct sc16is7x2_write_message *wm = sc16is7x2_get_write_message(ts);
	dev_dbg(&ts->spi->dev, "%s ch%d: %s = 0x%02X", __func__, ch, reg->name, val);
	
	wm->t.tx_buf = wm->data;
	wm->t.len = 2;
	wm->data[0] = reg->writecmd;
	wm->data[1] = val;
	wm->reg = reg;
	wm->complete = complete;
	wm->context = context;

	sc16is7x2_write_message_run(ts);
}

/**
 * sc16is7x2_write_reg_tx_async - special function for writing.
 *
 */
static void
sc16is7x2_write_message_tx(struct sc16is7x2_chip *ts,
					struct sc16is7x2_reg *reg,
					u8 *buf, u8 len,
					void (*complete)(void*, struct sc16is7x2_write_message*),
					void *context)
{
	unsigned ch = (reg->writecmd & 0x2) == 0 ? 0 : 1;
	struct sc16is7x2_write_message *wm = sc16is7x2_get_write_message(ts);
	dev_dbg(&ts->spi->dev, "%s ch%d: %s", __func__, ch, reg->name);

	buf[0] = reg->writecmd;
	sc16is7x2_write_message_set_tx(wm, buf, len+1);
	wm->reg = reg;
	wm->complete = complete;
	wm->context = context;

	sc16is7x2_write_message_run(ts);
}

/* helpers for channel registers */
static inline void
sc16is7x2_write_val(struct sc16is7x2_channel *chan, u8 reg, u8 val)
{
	sc16is7x2_write_reg(chan->chip, &chan->regs[reg], val, NULL, NULL);
	}

/*
static inline void
sc16is7x2_write_val_complete(struct sc16is7x2_channel *chan, u8 reg, u8 val)
{
	sc16is7x2_write_reg(&chan->regs[reg], val, sc16is7x2_complete_work, chan);
}
*/

static inline void
sc16is7x2_write(struct sc16is7x2_channel *chan, u8 reg)
{
	sc16is7x2_write_reg(chan->chip, &chan->regs[reg],
			chan->regs[reg].val, NULL, NULL);
}

static inline void
sc16is7x2_set(struct sc16is7x2_channel *chan, u8 reg, u8 val)
{
	sc16is7x2_reg_set_val(&chan->regs[reg], val);
}

static inline void
sc16is7x2_set_bits(struct sc16is7x2_channel *chan, u8 reg, u8 mask, u8 val)
{
	sc16is7x2_reg_set_bits(&chan->regs[reg], mask, val);
}

static inline void
sc16is7x2_set_and_write(struct sc16is7x2_channel *chan, u8 reg, u8 val)
{
	sc16is7x2_set(chan, reg, val);
	sc16is7x2_write(chan, reg);
	}

static inline void
sc16is7x2_set_bits_and_write(struct sc16is7x2_channel *chan, u8 reg,
															u8 mask, u8 val)
{
	sc16is7x2_set_bits(chan, reg, mask, val);
	sc16is7x2_write(chan, reg);
}

static inline void
sc16is7x2_read(struct sc16is7x2_channel *chan, u8 reg,
					void (*complete) (void*, struct sc16is7x2_read_message*),
					void *context)
{
	sc16is7x2_read_reg(chan->chip, &chan->regs[reg], complete, context);
}

/*
static inline void
sc16is7x2_read_ro(struct sc16is7x2_channel *chan, u8 reg,
					void (*complete) (void*, struct sc16is7x2_reg*),
					void *context)
{
	sc16is7x2_read_reg_ro_async(&chan->regs[reg], complete, context);
}
*/

/* helpers for chip registers */
static inline void
sc16is7x2_write_chipreg_val(struct sc16is7x2_chip *ts, u8 reg, u8 val)
{
	sc16is7x2_write_reg(ts, &ts->regs[reg], val, NULL, NULL);
	}
/*
static inline void
sc16is7x2_write_chipreg_val_complete(struct sc16is7x2_chip *ts, u8 reg, u8 val)
{
	sc16is7x2_write_reg(&ts->regs[reg], val, sc16is7x2_complete_work, ts);
	}
*/

static inline void
sc16is7x2_write_chipreg(struct sc16is7x2_chip *ts, u8 reg)
{
	sc16is7x2_write_reg(ts, &ts->regs[reg], ts->regs[reg].val, NULL, NULL);
}

static inline void
sc16is7x2_set_chipreg(struct sc16is7x2_chip *ts, u8 reg, u8 val)
{
	sc16is7x2_reg_set_val(&ts->regs[reg], val);
	}

static inline void
sc16is7x2_set_bits_chipreg(struct sc16is7x2_chip *ts, u8 reg, u8 mask, u8 val)
{
	sc16is7x2_reg_set_bits(&ts->regs[reg], mask, val);
}

static inline void
sc16is7x2_set_and_write_chipreg(struct sc16is7x2_chip *ts, u8 reg, u8 val)
{
	sc16is7x2_set_chipreg(ts, reg, val);
	sc16is7x2_write_chipreg(ts, reg);
}

static inline void
sc16is7x2_set_bits_and_write_chipreg(struct sc16is7x2_chip *ts, u8 reg,
															u8 mask, u8 val)
{
	sc16is7x2_set_bits_chipreg(ts, reg, mask, val);
	sc16is7x2_write_chipreg(ts, reg);
}

static void sc16is7x2_power(struct sc16is7x2_channel *chan, int on)
{
	sc16is7x2_set_bits_and_write(chan, SC16IS7XX_IER_REG,
			      SC16IS7XX_IER_SLEEP_BIT,
			      on ? 0 : SC16IS7XX_IER_SLEEP_BIT);
	}


static void
sc16is7x2_read_rx(struct sc16is7x2_channel *chan, u8 rxlvl);

static void
sc16is7x2_read_irq_complete(void *data, struct sc16is7x2_read_message *rm);

static void sc16is7x2_write_tx(struct sc16is7x2_channel *chan);

static void
sc16is7x2_read_status_lsr(void *data, struct sc16is7x2_read_message *rm)
{
	struct sc16is7x2_channel *chan = data;
	struct sc16is7x2_chip *ts = chan->chip;
	unsigned ch = (chan == ts->channel) ? 0 : 1;
	struct sc16is7x2_reg *reg = rm->reg;
	u8 lsr = rm->data;
	sc16is7x2_reg_set_val(reg, lsr);
	dev_dbg(&ts->spi->dev, "%s ch%d: %02x", __func__, ch, lsr);
	}

static void
sc16is7x2_read_status_msr(void *data, struct sc16is7x2_read_message *rm)
{
	struct sc16is7x2_channel *chan = data;
	struct sc16is7x2_chip *ts = chan->chip;
	unsigned ch = (chan == ts->channel) ? 0 : 1;
	struct sc16is7x2_reg *reg = rm->reg;
	u8 msr = rm->data;
	sc16is7x2_reg_set_val(reg, msr);
	dev_dbg(&ts->spi->dev, "%s ch%d: %02x", __func__, ch, msr);
}

static void 
sc16is7x2_read_status_ier(void *data, struct sc16is7x2_read_message *rm)
{
	struct sc16is7x2_channel *chan = data;
	struct sc16is7x2_chip *ts = chan->chip;
	unsigned ch = (chan == ts->channel) ? 0 : 1;
	struct sc16is7x2_reg *reg = rm->reg;
	u8 ier = rm->data;
	sc16is7x2_reg_set_val(reg, ier);
	dev_dbg(&ts->spi->dev, "%s ch%d: %02x", __func__, ch, ier);
}

static void
sc16is7x2_read_status_spr(void *data, struct sc16is7x2_read_message *rm)
{
	struct sc16is7x2_channel *chan = data;
	struct sc16is7x2_chip *ts = chan->chip;
	unsigned ch = (chan == ts->channel) ? 0 : 1;
	struct sc16is7x2_reg *reg = rm->reg;
	u8 spr = rm->data;
	sc16is7x2_reg_set_val(reg, spr);
	dev_dbg(&ts->spi->dev, "%s ch%d: %02x", __func__, ch, spr);
}

static void
sc16is7x2_read_rxlvl_complete(void *data, struct sc16is7x2_read_message *rm)
{
	struct sc16is7x2_channel *chan = data;
	struct sc16is7x2_chip *ts = chan->chip;
	unsigned ch = (chan == ts->channel) ? 0 : 1;
	struct sc16is7x2_reg *reg = rm->reg;
	u8 ier = sc16is7x2_reg_get_val(&chan->regs[SC16IS7XX_IER_REG]);
	u8 rxlvl = rm->data;
	sc16is7x2_reg_set_val(reg, rxlvl);

	dev_dbg(&ts->spi->dev, "%s ch%d: %d", __func__, ch, rxlvl);

	if(rxlvl > 0){
		chan->rx_reading = true;
		if(ier & SC16IS7XX_IER_RDI_BIT){
			/*	that meaning the begin of reading
				turn off RDI and RLSI interrupt
				for improvement speed: less IRQ	*/
			sc16is7x2_set_bits_and_write(chan, SC16IS7XX_IER_REG,
						SC16IS7XX_IER_RDI_BIT | SC16IS7XX_IER_RLSI_BIT, 0);
		}
		sc16is7x2_read_rx(chan, rxlvl);
	} else {
		/* end of reading */
		if(!(ier & SC16IS7XX_IER_RDI_BIT)){
			/* turn on RHI and RLSI  interrupt */
			sc16is7x2_set_bits_and_write(chan, SC16IS7XX_IER_REG,
					SC16IS7XX_IER_RDI_BIT | SC16IS7XX_IER_RLSI_BIT,
					SC16IS7XX_IER_RDI_BIT | SC16IS7XX_IER_RLSI_BIT);
		}
		chan->rx_reading = false;
	}
}

static void sc16is7x2_read_rxlvl(struct sc16is7x2_channel *chan)
{
	struct sc16is7x2_chip *ts = chan->chip;
	unsigned ch = (chan == ts->channel) ? 0 : 1;

	if(!chan->rx_reading){
		chan->rx_reading = true;

		sc16is7x2_read(chan, SC16IS7XX_RXLVL_REG,
									sc16is7x2_read_rxlvl_complete, chan);

	} else {
		dev_dbg(&ts->spi->dev, "%s ch%d: already", __func__, ch);
	}
}

static void
sc16is7x2_read_txlvl_complete(void *data, struct sc16is7x2_read_message *rm)
{
	struct sc16is7x2_channel *chan = data;
	struct sc16is7x2_chip *ts = chan->chip;
	unsigned ch = (chan == ts->channel) ? 0 : 1;
	struct sc16is7x2_reg *reg = rm->reg;
	u8 ier = sc16is7x2_reg_get_val(&chan->regs[SC16IS7XX_IER_REG]);
	u8 txlvl = rm->data;
	sc16is7x2_reg_set_val(reg, txlvl);

	dev_dbg(&ts->spi->dev, "%s ch%d: %d", __func__, ch, txlvl);

	if(chan->tx_buffer_wait){
		if(txlvl >= (FIFO_SIZE >> 1)){
			sc16is7x2_write_tx(chan);
		} else {
			if(!(ier & SC16IS7XX_IER_THRI_BIT)){
				sc16is7x2_set_bits_and_write(chan, SC16IS7XX_IER_REG,
						SC16IS7XX_IER_THRI_BIT, SC16IS7XX_IER_THRI_BIT);
			}
		}
	} else {
		chan->tx_writing = false;
		if(ier & UART_IER_THRI){
			sc16is7x2_set_bits_and_write(chan, SC16IS7XX_IER_REG,
					SC16IS7XX_IER_THRI_BIT, 0);
		}
	}

	dev_dbg(&ts->spi->dev, "%s ch%d: END", __func__, ch);
}

static void sc16is7x2_read_txlvl(struct sc16is7x2_channel *chan)
{
	if(!chan->tx_writing){
		chan->tx_writing = true;
		sc16is7x2_read(chan, SC16IS7XX_TXLVL_REG,
									sc16is7x2_read_txlvl_complete, chan);
	}
}

/**
 * sc16is7x2_read_status - read RXLVL, IIR.
 *
 */
static void sc16is7x2_read_status(struct sc16is7x2_channel *chan)
{
	struct sc16is7x2_chip *ts = chan->chip;
	unsigned ch = (chan == ts->channel) ? 0 : 1;

	dev_dbg(&ts->spi->dev, "%s ch%d", __func__, ch);

	sc16is7x2_read_rxlvl(chan);
	sc16is7x2_read(chan, SC16IS7XX_IIR_REG, sc16is7x2_read_irq_complete, chan);
}

/**
 * sc16is7x2_write_tx_complete - end of writing
 *
 */
static void sc16is7x2_write_tx_complete(void *data, struct sc16is7x2_write_message *wm){
	struct sc16is7x2_channel *chan = data;
	struct sc16is7x2_chip *ts = chan->chip;
	unsigned ch = (chan == ts->channel) ? 0 : 1;

	dev_dbg(&ts->spi->dev, "%s ch%d", __func__, ch);
}

static void sc16is7x2_write_tx(struct sc16is7x2_channel *chan){
	struct sc16is7x2_chip *ts = chan->chip;
	unsigned ch = (chan == ts->channel) ? 0 : 1;
	struct uart_port *uart = &chan->uart;
	struct circ_buf *xmit = &uart->state->xmit;

	u8 ier = sc16is7x2_reg_get_val(&chan->regs[SC16IS7XX_IER_REG]);
	u8 lsr = sc16is7x2_reg_get_val(&chan->regs[SC16IS7XX_LSR_REG]);
	//unsigned long flags;
	unsigned i, len;
	int chars_pending;

	dev_dbg(&ts->spi->dev, "%s ch%d", __func__, ch);


	if (chan->uart.x_char && lsr & UART_LSR_THRE) {
		dev_dbg(&ts->spi->dev, " tx: x-char\n");
		//sc16is7x2_write(ts, UART_TX, ch, uart->x_char);
		uart->icount.tx++;
		uart->x_char = 0;
		return;
	}

	if (uart_circ_empty(xmit) || uart_tx_stopped(&chan->uart)){
		/* No data to send or TX is stopped */
		dev_dbg(&ts->spi->dev, "%s ch%d: No data to send or TX is stopped", __func__, ch);
		chan->tx_writing = false;
		chan->tx_buffer_wait = false;
		if(ier & UART_IER_THRI){
			/* turn off THRI */
			sc16is7x2_set_bits_and_write(chan, SC16IS7XX_IER_REG,
					SC16IS7XX_IER_THRI_BIT, 0);
		}
		return;
	}

	chan->tx_writing = true;

	/* number of bytes to transfer to the fifo */
	chars_pending = (int)uart_circ_chars_pending(xmit);
	len = min((FIFO_SIZE>>1), chars_pending);
	chan->tx_buffer_wait = len < chars_pending;
		
	dev_dbg(&ts->spi->dev, "%s ch%i: %d bytes will be sent\n",
			__func__, ch, len);

	/* fill buffer to send */
	//spin_lock_irqsave(&uart->lock, flags);
	for (i = 1; i <= len ; i++) {
		chan->tx_buf[i] = xmit->buf[xmit->tail];
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
	}
	uart->icount.tx += len;
	//spin_unlock_irqrestore(&uart->lock, flags);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(uart);

	/* send data */
	sc16is7x2_write_message_tx(ts, &chan->regs[SC16IS7XX_THR_REG],
			chan->tx_buf, len, sc16is7x2_write_tx_complete, chan);

	/* get TXLVL after sending */
	sc16is7x2_read(chan, SC16IS7XX_TXLVL_REG,
									sc16is7x2_read_txlvl_complete, chan);
}

/**
* sc16is7x2_read_rx_complete - handle readed data
*
* @data - current channel
*/
static void 
sc16is7x2_read_rx_complete(void *data, struct sc16is7x2_read_message *rm){
	struct sc16is7x2_channel *chan = data;
	struct sc16is7x2_chip *ts = chan->chip;
	unsigned ch = (chan == ts->channel) ? 0 : 1;
	struct uart_port *uart = &chan->uart;
	struct tty_port *tty = &uart->state->port;
	unsigned long flags;
	u8 rxlvl = rm->t[1].len;	// prev rxlvl val.

	dev_dbg(&ts->spi->dev, "%s ch%d: readed %d bytes", __func__, ch, rxlvl);

	spin_lock_irqsave(&uart->lock, flags);
	/* Insert received data */
	tty_insert_flip_string(tty, (char *)chan->rx_buf, rxlvl);
	/* Update RX counter */
	uart->icount.rx += rxlvl;
	spin_unlock_irqrestore(&uart->lock, flags);

	tty_flip_buffer_push(tty);
}

/**
* sc16is7x2_read_rx - reading data from FIFO
*
* @data - current channel
*/
static void sc16is7x2_read_rx(struct sc16is7x2_channel *chan, u8 rxlvl)
{
	struct sc16is7x2_chip *ts = chan->chip;
	//unsigned ch = (chan == ts->channel) ? 0 : 1;

	sc16is7x2_read_message_rx(ts, sc16is7x2_get_reg(chan, SC16IS7XX_RHR_REG),
			chan->rx_buf, rxlvl,
			sc16is7x2_read_rx_complete, chan);

	sc16is7x2_read(chan, SC16IS7XX_RXLVL_REG,
								sc16is7x2_read_rxlvl_complete, chan);
}

static void 
sc16is7x2_read_irq_complete(void *data, struct sc16is7x2_read_message *rm)
{
	struct sc16is7x2_channel *chan = data;
	struct sc16is7x2_reg *reg = rm->reg;
	u8 iir = rm->data;

#ifdef DEBUG
	struct sc16is7x2_chip *ts = chan->chip;
	unsigned ch = (chan == ts->channel) ? 0 : 1;
#endif

	sc16is7x2_reg_set_val(reg, iir);
	chan->iir_reading = false;

#ifdef DEBUG
	#define print_iir_if(iir)	case iir: \
			dev_dbg(&ts->spi->dev, "%s: ch%d: " #iir, __func__, ch); \
			break; 
	
	switch(iir & SC16IS7XX_IIR_ID_MASK){
		print_iir_if(UART_IIR_NO_INT);
		print_iir_if(UART_IIR_MSI);
		print_iir_if(UART_IIR_THRI);
		print_iir_if(UART_IIR_RDI);
		print_iir_if(UART_IIR_RLSI);
		print_iir_if(UART_IIR_BUSY);
		print_iir_if(UART_IIR_RX_TIMEOUT);
		print_iir_if(UART_IIR_XOFF);
		print_iir_if(UART_IIR_CTS_RTS_DSR);
		default:
			dev_dbg(&ts->spi->dev, "%s: ch%d: UNKNOWN", __func__, ch);
			break;
	}
#endif

	switch(iir & SC16IS7XX_IIR_ID_MASK){
		case UART_IIR_THRI:
		/*
			sc16is7x2_write_tx(chan);
		*/
			sc16is7x2_read(chan, SC16IS7XX_TXLVL_REG,
					sc16is7x2_read_txlvl_complete, chan);

			break;
		case UART_IIR_MSI:
			sc16is7x2_read(chan, SC16IS7XX_MSR_REG,
					sc16is7x2_read_status_msr, chan);
			break;
		case UART_IIR_RLSI:
			sc16is7x2_read(chan, SC16IS7XX_LSR_REG,
								sc16is7x2_read_status_lsr, chan);
			break;
		default: ;
	}

}

/**
* sc16is7x2_irq - handle irq
* 
*/
static irqreturn_t sc16is7x2_irq(int irq, void *data){

	struct sc16is7x2_chip *ts = data;
	u8 i;

	for(i = 0; i < 2; ++i){	
		//if(ts->channel[i].started){
			if(ts->channel[i].iir_reading){
				dev_dbg(&ts->spi->dev, "%s ch%d: iir already reading", __func__, i);
			} else {
				ts->channel[i].iir_reading = true;
				sc16is7x2_read_status(&ts->channel[i]);
			}
		//}
	}

	dev_dbg(&ts->spi->dev, "%s: IRQ_HANDLED", __func__);
	return IRQ_HANDLED;
}

/* ******************************** UART ********************************* */

#define to_sc16is7x2_channel(port) \
		container_of(port, struct sc16is7x2_channel, uart)


static unsigned int sc16is7x2_tx_empty(struct uart_port *port)
{
	struct sc16is7x2_channel *chan = to_sc16is7x2_channel(port);
	struct sc16is7x2_chip *ts = chan->chip;
	unsigned ch = (chan == ts->channel) ? 0 : 1;

	u8 txlvl = sc16is7x2_reg_get_val(&chan->regs[SC16IS7XX_TXLVL_REG]);
	bool empty = txlvl == SC16IS7XX_FIFO_SIZE;

	if(!empty){
		sc16is7x2_read_txlvl(chan);
	}

	dev_dbg(&ts->spi->dev, "%s ch%d: txlvl = %d; empty = %s",
			__func__, ch, txlvl, empty ? "yes" : "no");

	return empty ? TIOCSER_TEMT : 0;
}

static unsigned int sc16is7x2_get_mctrl(struct uart_port *port)
{
	struct sc16is7x2_channel *chan = to_sc16is7x2_channel(port);
	struct sc16is7x2_chip *ts = chan->chip;
	dev_dbg(&ts->spi->dev, __func__);

	/* DCD and DSR are not wired and CTS/RTS is handled automatically
	 * so just indicate DSR and CAR asserted
	 */
	return TIOCM_DSR | TIOCM_CAR;
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
	struct sc16is7x2_channel *chan = to_sc16is7x2_channel(port);
	struct sc16is7x2_chip *ts = chan->chip;
	unsigned ch = (&ts->channel[1] == chan) ? 1 : 0;

	dev_dbg(&ts->spi->dev, "%s ch%d", __func__, ch);
}

static void sc16is7x2_start_tx(struct uart_port *port)
{
	struct sc16is7x2_channel *chan = to_sc16is7x2_channel(port);
	struct sc16is7x2_chip *ts = chan->chip;
	unsigned ch = (&ts->channel[1] == chan) ? 1 : 0;

	dev_dbg(&ts->spi->dev, "%s ch%d", __func__, ch);

	chan->tx_buffer_wait = true;
	sc16is7x2_read_txlvl(chan);
}

static void sc16is7x2_stop_rx(struct uart_port *port)
{
	struct sc16is7x2_channel *chan = to_sc16is7x2_channel(port);
	struct sc16is7x2_chip *ts = chan->chip;
	unsigned ch = (&ts->channel[1] == chan) ? 1 : 0;

	dev_dbg(&ts->spi->dev, "%s ch%d", __func__, ch);

/*
	chan->uart.read_status_mask &= ~SC16IS7XX_LSR_DR_BIT;
	sc16is7x2_set_bits_and_write(chan, SC16IS7XX_IER_REG,
			      SC16IS7XX_LSR_DR_BIT,
			      0);
*/
}

static void sc16is7x2_break_ctl(struct uart_port *port, int break_state)
{
	struct sc16is7x2_channel *chan = to_sc16is7x2_channel(port);
	struct sc16is7x2_chip *ts = chan->chip;
	unsigned ch = (&ts->channel[1] == chan) ? 1 : 0;

	dev_dbg(&ts->spi->dev, "%s ch%d = %08X", __func__, ch, break_state);

	sc16is7x2_set_bits_and_write(chan, SC16IS7XX_LCR_REG,
			      SC16IS7XX_LCR_TXBREAK_BIT,
			      break_state ? SC16IS7XX_LCR_TXBREAK_BIT : 0);
}

/*
static int sc16is7xx_config_rs485(struct uart_port *port,
				   struct serial_rs485 *rs485)
{
	struct sc16is7x2_channel *chan = to_sc16is7x2_channel(port);
	struct sc16is7x2_chip *ts = chan->chip;
	unsigned ch = (&ts->channel[1] == chan) ? 1 : 0;

	dev_dbg(&ts->spi->dev, "%s ch%d", __func__, ch);

	if (port->rs485.flags & SER_RS485_ENABLED)
		sc16is7x2_set_bits_and_write(chan, SC16IS7XX_EFCR_REG,
				      SC16IS7XX_EFCR_AUTO_RS485_BIT,
				      SC16IS7XX_EFCR_AUTO_RS485_BIT);
	else
		sc16is7x2_set_bits_and_write(chan, SC16IS7XX_EFCR_REG,
				      SC16IS7XX_EFCR_AUTO_RS485_BIT,
				      0);
	port->rs485 = *rs485;

	return 0;
}
*/


static int sc16is7x2_startup(struct uart_port *port)
{
	struct sc16is7x2_channel *chan = to_sc16is7x2_channel(port);
	struct sc16is7x2_chip *ts = chan->chip;
	unsigned ch = (&ts->channel[1] == chan) ? 1 : 0;
	//unsigned long flags;
	//u8 val;

	dev_dbg(&ts->spi->dev, "%s ch%d", __func__, ch);

	if (chan->started) {
		dev_err(&ts->spi->dev, "\n%s (%d) duplicate startup, do nothing\n", __func__, port->line);
		return 0;
	}

	sc16is7x2_power(chan, 1);

	sc16is7x2_set_and_write(&ts->channel[ch], SC16IS7XX_FCR_REG,
			SC16IS7XX_FCR_RXRESET_BIT |
			SC16IS7XX_FCR_TXRESET_BIT |
			SC16IS7XX_FCR_FIFO_BIT);

	sc16is7x2_write_val(chan, SC16IS7XX_SPR_REG, 0x5E);
	sc16is7x2_read(chan, SC16IS7XX_SPR_REG, sc16is7x2_read_status_spr, chan);
/*
	sc16is7x2_write_val(chan, SC16IS7XX_FCR_REG,
			SC16IS7XX_FCR_RXRESET_BIT |
			SC16IS7XX_FCR_TXRESET_BIT |
			SC16IS7XX_FCR_FIFO_BIT);
*/
	/* access EFR [async] */
	sc16is7x2_write_val(chan, SC16IS7XX_LCR_REG,
			SC16IS7XX_LCR_CONF_MODE_B);

	/* Enable enhanced functions */
	sc16is7x2_set_and_write(chan, SC16IS7XX_EFR_REG,
			SC16IS7XX_EFR_ENABLE_BIT);

	//spin_lock_irqsave(&chan->uart.lock, flags);
#if 0
	/* Enable TCR/TLR */
	sc16is7x2_set_bits_and_write(chan, SC16IS7XX_MCR_REG,
			SC16IS7XX_MCR_TCRTLR_BIT,
			SC16IS7XX_MCR_TCRTLR_BIT);

	/* Configure flow control levels */
	/* Flow control halt level 48, resume level 24 */
	val = SC16IS7XX_TCR_RX_RESUME(24) | SC16IS7XX_TCR_RX_HALT(48);
	sc16is7x2_set_bits_and_write(chan, SC16IS7XX_TCR_REG, val, val);

	/* Now, initialize the UART */
	/* termios will do it
	sc16is7x2_set_bits_and_write(chan, SC16IS7XX_LCR_REG,
			SC16IS7XX_LCR_WORD_LEN_8,
			SC16IS7XX_LCR_WORD_LEN_8);
	*/
	/* Enable the Rx and Tx FIFO  and autoRS485*/
	sc16is7x2_set_bits_and_write(chan, SC16IS7XX_EFCR_REG,
			SC16IS7XX_EFCR_RXDISABLE_BIT |
			SC16IS7XX_EFCR_TXDISABLE_BIT |
			SC16IS7XX_EFCR_AUTO_RS485_BIT,
			SC16IS7XX_EFCR_AUTO_RS485_BIT);

	/* Enable RX interrupts */
	sc16is7x2_set_bits_and_write(chan, SC16IS7XX_IER_REG,
			SC16IS7XX_IER_RDI_BIT,
			SC16IS7XX_IER_RDI_BIT);

	sc16is7x2_set_and_write(chan, SC16IS7XX_FCR_REG, SC16IS7XX_FCR_FIFO_BIT);

	//spin_unlock_irqrestore(&chan->uart.lock, flags);

#endif
	/* Now, initialize the UART */
	sc16is7x2_set_and_write(chan, SC16IS7XX_LCR_REG, SC16IS7XX_LCR_WORD_LEN_8);
	sc16is7x2_set_and_write(chan, SC16IS7XX_IER_REG,
			SC16IS7XX_IER_RDI_BIT | SC16IS7XX_IER_RLSI_BIT);
	sc16is7x2_set_and_write(chan, SC16IS7XX_FCR_REG,
			SC16IS7XX_FCR_FIFO_BIT |
			SC16IS7XX_FCR_TXLVLL_BIT |
			SC16IS7XX_FCR_TXLVLH_BIT);
	sc16is7x2_set_and_write(chan, SC16IS7XX_MCR_REG, 0);
	sc16is7x2_set_and_write(chan, SC16IS7XX_EFCR_REG,
			SC16IS7XX_EFCR_AUTO_RS485_BIT | SC16IS7XX_EFCR_RTS_INVERT_BIT);


	sc16is7x2_read(chan, SC16IS7XX_IER_REG, sc16is7x2_read_status_ier, chan);
	sc16is7x2_read(chan, SC16IS7XX_MSR_REG, sc16is7x2_read_status_msr, chan);
	sc16is7x2_read(chan, SC16IS7XX_LSR_REG, sc16is7x2_read_status_lsr, chan);

	chan->started = true;

	return 0;
}

static void sc16is7x2_shutdown(struct uart_port *port)
{
	struct sc16is7x2_channel *chan = to_sc16is7x2_channel(port);
	struct sc16is7x2_chip *ts = chan->chip;
	//unsigned long flags;
	unsigned ch = port->line & 0x01;

	dev_info(&ts->spi->dev, "%s ch%d\n", __func__, ch);

	if (chan->console_enabled) {
		// no need to actually shutdown port if console is enabled on this port
		return;
	}


	BUG_ON(!chan);
	BUG_ON(!ts);

	sc16is7x2_write_val(&ts->channel[ch], SC16IS7XX_FCR_REG,
			SC16IS7XX_FCR_RXRESET_BIT |
			SC16IS7XX_FCR_TXRESET_BIT |
			SC16IS7XX_FCR_FIFO_BIT);
	/* Disable all interrupts and go to sleep */
	//sc16is7x2_set_and_write(chan, SC16IS7XX_IER_REG, SC16IS7XX_IER_SLEEP_BIT);
	sc16is7x2_set_and_write(chan, SC16IS7XX_IER_REG, 0);
	/* Disable TX/RX */
	sc16is7x2_set_bits_and_write(chan, SC16IS7XX_EFCR_REG,
			SC16IS7XX_EFCR_RXDISABLE_BIT |
			SC16IS7XX_EFCR_TXDISABLE_BIT,
			SC16IS7XX_EFCR_RXDISABLE_BIT |
			SC16IS7XX_EFCR_TXDISABLE_BIT);

	chan->started = false;
}

static void
sc16is7x2_set_termios(struct uart_port *port,
						struct ktermios *termios,
			   struct ktermios *old)
{
	struct sc16is7x2_channel *chan = to_sc16is7x2_channel(port);
	struct sc16is7x2_chip *ts = chan->chip;
	unsigned ch = (&ts->channel[1] == chan) ? 1 : 0;

	unsigned int lcr, flow = 0;
	int baud;
	u8 prescaler = 0;
	unsigned long clk = port->uartclk, div = 0;

	dev_dbg(&ts->spi->dev, "%s ch%d : ktermios : c_iflag %08X, c_oflag %08X,"
		" c_cflag %08X, c_lflag %08X, c_line %02X, ", __func__, ch,
		termios->c_iflag, termios->c_oflag,
		termios->c_cflag, termios->c_lflag,
		termios->c_line);

	/* Get baud rate generator configuration */
	baud = uart_get_baud_rate(port, termios, old,
				port->uartclk / 16 / 4 / 0xffff,
				  port->uartclk / 16);

	/* Setup baudrate generator */
	div = clk / 16 / baud;

	if (div > 0xffff) {
		prescaler = SC16IS7XX_MCR_CLKSEL_BIT;
		div /= 4;
	}

	sc16is7x2_write_val(chan, SC16IS7XX_LCR_REG, SC16IS7XX_LCR_CONF_MODE_A);
	/* Open the LCR divisors for configuration [async] */

	/* Write the new divisor */
	sc16is7x2_set_and_write(chan, SC16IS7XX_DLH_REG, div / 256);
	sc16is7x2_set_and_write(chan, SC16IS7XX_DLL_REG, div % 256);

	baud = DIV_ROUND_CLOSEST(clk / 16, div);

	/* debug info */
	dev_info(&ts->spi->dev, "%s ch%d : baud %u", __func__, ch, baud);

	/* Mask termios capabilities we don't support */
	termios->c_cflag &= ~CMSPAR;

	/* Word size */
	switch (termios->c_cflag & CSIZE) {
	case CS5:
		lcr = SC16IS7XX_LCR_WORD_LEN_5;
		break;
	case CS6:
		lcr = SC16IS7XX_LCR_WORD_LEN_6;
		break;
	case CS7:
		lcr = SC16IS7XX_LCR_WORD_LEN_7;
		break;
	case CS8:
		lcr = SC16IS7XX_LCR_WORD_LEN_8;
		break;
	default:
		lcr = SC16IS7XX_LCR_WORD_LEN_8;
		termios->c_cflag &= ~CSIZE;
		termios->c_cflag |= CS8;
		break;
	}

	/* Parity */
	if (termios->c_cflag & PARENB) {
		lcr |= SC16IS7XX_LCR_PARITY_BIT;
	if (!(termios->c_cflag & PARODD))
			lcr |= SC16IS7XX_LCR_EVENPARITY_BIT;
	}

	/* Stop bits */
	if (termios->c_cflag & CSTOPB)
		lcr |= SC16IS7XX_LCR_STOPLEN_BIT; /* 2 stops */

	/* Set read status mask */
	port->read_status_mask = SC16IS7XX_LSR_OE_BIT;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= SC16IS7XX_LSR_PE_BIT |
					  SC16IS7XX_LSR_FE_BIT;
	if (termios->c_iflag & (BRKINT | PARMRK))
		port->read_status_mask |= SC16IS7XX_LSR_BI_BIT;

	/* Set status ignore mask */
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNBRK)
		port->ignore_status_mask |= SC16IS7XX_LSR_BI_BIT;
	if (!(termios->c_cflag & CREAD))
		port->ignore_status_mask |= SC16IS7XX_LSR_BRK_ERROR_MASK;

	/* Open the LCR divisors for configuration  */
	sc16is7x2_write_val(chan, SC16IS7XX_LCR_REG, SC16IS7XX_LCR_CONF_MODE_B);

	/* Enable enhanced features */
	sc16is7x2_write_val(chan, SC16IS7XX_EFR_REG, SC16IS7XX_EFR_ENABLE_BIT);

	sc16is7x2_set_bits_and_write(chan, SC16IS7XX_MCR_REG,
			SC16IS7XX_MCR_CLKSEL_BIT,
			prescaler);

	/* Configure flow control */
	sc16is7x2_set_and_write(chan, SC16IS7XX_XON1_REG, termios->c_cc[VSTART]);
	sc16is7x2_set_and_write(chan, SC16IS7XX_XOFF1_REG, termios->c_cc[VSTOP]);
	if (termios->c_cflag & CRTSCTS)
		flow |= SC16IS7XX_EFR_AUTOCTS_BIT | SC16IS7XX_EFR_AUTORTS_BIT;
	if (termios->c_iflag & IXON)
		flow |= SC16IS7XX_EFR_SWFLOW3_BIT;
	if (termios->c_iflag & IXOFF)
		flow |= SC16IS7XX_EFR_SWFLOW1_BIT;

	sc16is7x2_set_and_write(chan, SC16IS7XX_EFR_REG, flow);


	/* Update LCR register */
	sc16is7x2_set(chan, SC16IS7XX_LCR_REG, lcr);
	sc16is7x2_write_val(chan, SC16IS7XX_LCR_REG,
			lcr);

	/* Update timeout according to new baud rate */
	uart_update_timeout(port, termios->c_cflag, baud);
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
/*
static void sc16is7x2_pm(struct uart_port *port, unsigned int state,
			 unsigned int oldstate)
{
	struct sc16is7x2_channel *chan = to_sc16is7x2_channel(port);
	sc16is7x2_power(chan, (state == UART_PM_STATE_ON) ? 1 : 0);
}
*/

static struct uart_ops sc16is7x2_uart_ops = {
	.tx_empty	= sc16is7x2_tx_empty,
	.set_mctrl	= sc16is7x2_set_mctrl,
	.get_mctrl	= sc16is7x2_get_mctrl,
	.stop_tx        = sc16is7x2_stop_tx,
	.start_tx	= sc16is7x2_start_tx,
	.stop_rx	= sc16is7x2_stop_rx,
//	.enable_ms      = sc16is7x2_enable_ms,
	.break_ctl      = sc16is7x2_break_ctl,
	.startup	= sc16is7x2_startup,
	.shutdown	= sc16is7x2_shutdown,
	.set_termios	= sc16is7x2_set_termios,
	.type		= sc16is7x2_type,
	.release_port   = sc16is7x2_release_port,
	.request_port   = sc16is7x2_request_port,
	.config_port	= sc16is7x2_config_port,
	.verify_port	= sc16is7x2_verify_port,
	//.pm		= sc16is7x2_pm,
};


/* ******************************** GPIO ********************************* */

#ifdef CONFIG_GPIOLIB

static int sc16is7x2_gpio_request(struct gpio_chip *gpio, unsigned offset)
{
#if 0
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
#endif
	return 0;
}

static void sc16is7x2_gpio_free(struct gpio_chip *gpio, unsigned offset)
{
#if 0
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
#endif
}

static int sc16is7x2_direction_input(struct gpio_chip *gpio, unsigned offset)
{
#if 10
	struct sc16is7x2_chip *ts =
			container_of(gpio, struct sc16is7x2_chip, gpio);

	unsigned io_state = BIT(offset);

	BUG_ON(offset > 8);

	sc16is7x2_set_bits_and_write_chipreg(ts, SC16IS7XX_IODIR_REG,
			io_state, 0);
#endif
	return 0;
}

static int sc16is7x2_direction_output(struct gpio_chip *gpio, unsigned offset,
					int value)
{
#if 10
	struct sc16is7x2_chip *ts =
			container_of(gpio, struct sc16is7x2_chip, gpio);

	unsigned io_state = BIT(offset);

	BUG_ON(offset > 8);

	sc16is7x2_set_bits_and_write_chipreg(ts, SC16IS7XX_IOSTATE_REG,
			io_state, value ? io_state : 0);

	sc16is7x2_set_bits_and_write_chipreg(ts, SC16IS7XX_IODIR_REG,
			io_state, io_state);
#endif
	return 0;
}

static int sc16is7x2_gpio_get(struct gpio_chip *gpio, unsigned offset)
{
#if 10
	struct sc16is7x2_chip *ts =
			container_of(gpio, struct sc16is7x2_chip, gpio);
	int level;

	BUG_ON(offset > 8);

	level = spi_w8r8(ts->spi, ts->regs[SC16IS7XX_IOSTATE_REG].readcmd);
	if(level < 0)
		return level;

	level = (level >> offset) & 0x01;

	return level;
#endif
	return 0;
}

static void sc16is7x2_gpio_set(struct gpio_chip *gpio, unsigned offset, int value)
{
#if 10
	struct sc16is7x2_chip *ts =
			container_of(gpio, struct sc16is7x2_chip, gpio);
	unsigned io_state = BIT(offset);

	BUG_ON(offset > 8);

	sc16is7x2_set_bits_and_write_chipreg(ts, SC16IS7XX_IOSTATE_REG,
			io_state, value ? io_state : 0);

#endif
}

#endif /* CONFIG_GPIOLIB */

/* ******************************** INIT ********************************* */
static struct uart_driver sc16is7x2_uart_driver;

static int sc16is7x2_register_uart_port(struct sc16is7x2_chip *ts, unsigned ch)
{
	struct sc16is7x2_channel *chan = &(ts->channel[ch]);
	struct uart_port *uart = &chan->uart;
	int ret;


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

static struct uart_driver sc16is7x2_uart_driver;

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
 * This function returns 1 if pdev isn't a device instatiated by dt, 0 if it
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

/*
	iprop = of_get_property(np, "irq-base", NULL);
	if (!iprop || prop_len < sizeof(*iprop)) {
		dev_err(&spi->dev, "Missing irq-base property in devicetree\n");
		return -EINVAL;
	}
	ts->irq_base = be32_to_cpup(iprop);
*/
	dev_err(&spi->dev, "gpio-base=%d\n",ts->gpio_base);
	//dev_err(&spi->dev, "irq-base  = %d\n",ts->irq_base);
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
	//ts->irq_base  = pdata->irq_base;
	ts->uartclk = pdata->uartclk;
	ts->gpio_label  = pdata->label;
	ts->gpio_names  = pdata->names;
}


/* Helpers */
static inline u8 sc16is7x2_make_writecmd(u8 reg, u8 ch)
{
	return REG_WRITE | (reg & 0xf) << 3 | (ch & 0x1) << 1;
}

static inline u8 sc16is7x2_make_readcmd(u8 reg, u8 ch)
{
	return REG_READ  | (reg & 0xf) << 3 | (ch & 0x1) << 1;
}

static inline void sc16is7x2_init_reg(struct sc16is7x2_reg *r,
									struct sc16is7x2_chip *chip,
									u8 ch, u8 addr, char *name)
{
	u8 i = 0;
	r->readcmd = sc16is7x2_make_readcmd(addr, ch);
	r->writecmd = sc16is7x2_make_writecmd(addr, ch);

	while(name[i]){
		r->name[i] = name[i];
		i++;
	}
	r->name[i] = '[';
	i++;
	r->name[i] = addr < 10 ? addr + '0' : addr + 'A' - 10;
	i++;
	r->name[i] = ']';
	i++;
	r->name[i] = 0;

}

/* initializing registers helpers */
#define SC16IS7X2_INIT_NAMEDREG(ch, name) 									\
	sc16is7x2_init_reg(&ts->channel[ch].regs[SC16IS7XX ## name ## REG],		\
							ts, ch, SC16IS7XX ## name ## REGADDR, #name)

#define SC16IS7X2_INIT_CHIP_NAMEDREG(name) 									\
	sc16is7x2_init_reg(&ts->regs[SC16IS7XX ## name ## REG],		\
							ts, 0, SC16IS7XX ## name ## REGADDR, #name)


static int sc16is7x2_probe(struct spi_device *spi)
{
	struct sc16is7x2_chip *ts;
	struct sc16is7x2_read_message *r_msg;
	struct sc16is7x2_write_message *w_msg;

	int ret;
	unsigned char ch, i;

	/* Only even uart base numbers are supported */
	dev_info(&spi->dev, "probe start\n");

	ts = kzalloc(sizeof(struct sc16is7x2_chip), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	spi_set_drvdata(spi, ts);
	ts->spi = spi;

	for (ch = 0; ch < 2; ++ch) {
		ts->channel[ch].use_modem_pins_by_default = false; //TODO: true
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
/*
	if (!ts->irq_base) {
		dev_err(&spi->dev, "incorrect irq_base\n");
		return -EINVAL;
	}
*/
	if (ts->uart_base & 1) {
		dev_err(&spi->dev, "incorrect uart_base\n");
		return -EINVAL;
	}

	/* Init reg structures... */
	/* ...for chip */
	SC16IS7X2_INIT_CHIP_NAMEDREG(_IODIR_);
	SC16IS7X2_INIT_CHIP_NAMEDREG(_IOSTATE_);
	SC16IS7X2_INIT_CHIP_NAMEDREG(_IOINTENA_);
	SC16IS7X2_INIT_CHIP_NAMEDREG(_IOCONTROL_);
	/* ...for each channel */
	for (ch = 0; ch < 2; ++ch) {
		SC16IS7X2_INIT_NAMEDREG(ch, _RHR_);
		SC16IS7X2_INIT_NAMEDREG(ch, _THR_);
		SC16IS7X2_INIT_NAMEDREG(ch, _IER_);
		SC16IS7X2_INIT_NAMEDREG(ch, _IIR_);
		SC16IS7X2_INIT_NAMEDREG(ch, _FCR_);
		SC16IS7X2_INIT_NAMEDREG(ch, _LCR_);
		SC16IS7X2_INIT_NAMEDREG(ch, _MCR_);
		SC16IS7X2_INIT_NAMEDREG(ch, _LSR_);
		SC16IS7X2_INIT_NAMEDREG(ch, _MSR_);
		SC16IS7X2_INIT_NAMEDREG(ch, _SPR_);
		SC16IS7X2_INIT_NAMEDREG(ch, _TXLVL_);
		SC16IS7X2_INIT_NAMEDREG(ch, _RXLVL_);
		SC16IS7X2_INIT_NAMEDREG(ch, _EFCR_);
		SC16IS7X2_INIT_NAMEDREG(ch, _TCR_);
		SC16IS7X2_INIT_NAMEDREG(ch, _TLR_);
		SC16IS7X2_INIT_NAMEDREG(ch, _DLL_);
		SC16IS7X2_INIT_NAMEDREG(ch, _DLH_);
		SC16IS7X2_INIT_NAMEDREG(ch, _EFR_);
		SC16IS7X2_INIT_NAMEDREG(ch, _XON1_);
		SC16IS7X2_INIT_NAMEDREG(ch, _XON2_);
		SC16IS7X2_INIT_NAMEDREG(ch, _XOFF1_);
		SC16IS7X2_INIT_NAMEDREG(ch, _XOFF2_);
	}

	for(i = 0; i < SC16IS7X2_READ_MESSAGE_QUEUE_SIZE; i++){
		r_msg = &ts->rq.queue[i];
		spi_message_init_with_transfers(&r_msg->m, r_msg->t, 2);
		r_msg->m.complete = sc16is7x2_read_message_complete;
		r_msg->m.context = r_msg;
		r_msg->t[0].tx_buf = &r_msg->readcmd;
		r_msg->t[0].len = 1;
		//r_msg->t[1].rx_buf = &r_msg->data;
		//r_msg->t[1].len = 1;
		r_msg->chip = ts;
	}

	for(i = 0; i < SC16IS7X2_WRITE_MESSAGE_QUEUE_SIZE; i++){
		w_msg = &ts->wq.queue[i];
		spi_message_init_with_transfers(&w_msg->m, &w_msg->t, 1);
		w_msg->m.complete = sc16is7x2_write_message_complete;
		w_msg->m.context = w_msg;
		//w_msg->t[0].tx_buf = &w_msg->data;
		//w_msg->t[0].len = 2;
		w_msg->chip = ts;
	}

	/* Reset the chip */
	sc16is7x2_write_chipreg_val(ts,
			SC16IS7XX_IOCONTROL_REG,
			SC16IS7XX_IOCONTROL_SRESET_BIT);


	/* disable all GPIOs, enable on request */
	ts->io_control = 0;
	if (ts->channel[0].use_modem_pins_by_default)
		ts->io_control |= SC16IS7XX_IOCONTROL_GPIO30_BIT;

	if (ts->channel[1].use_modem_pins_by_default)
		ts->io_control |= SC16IS7XX_IOCONTROL_GPIO74_BIT;

	sc16is7x2_set_and_write_chipreg(ts, SC16IS7XX_IOCONTROL_REG,
								ts->io_control);


	ts->channel[0].chip = ts;
	ts->channel[1].chip = ts;

	/* fill uart data  */
	for (ch = 0; ch < 2; ++ch) {
		ts->channel[ch].uart.irq = ts->spi->irq;
		ts->channel[ch].uart.uartclk = ts->uartclk;
		ts->channel[ch].uart.fifosize = FIFO_SIZE;
		ts->channel[ch].uart.ops = &sc16is7x2_uart_ops;
		ts->channel[ch].uart.flags = UPF_SKIP_TEST | UPF_BOOT_AUTOCONF;
		ts->channel[ch].uart.line = ts->uart_base + ch;
		ts->channel[ch].uart.type = PORT_SC16IS7X2;
		ts->channel[ch].uart.dev = &ts->spi->dev;
	}
/*
	dev_info(&spi->dev, DRIVER_NAME " at CS%d (irq %d), 2 UARTs, 8 GPIOs\n"
			"    ttyNSC%d, ttyNSC%d, gpiochip%d\n",
			spi->chip_select, spi->irq,
			ts->uart_base, ts->uart_base + 1,
			ts->gpio_base);

	return 0;
	 */
	/* add ports */
	ret = sc16is7x2_register_uart_port(ts, 0);
//	ret = uart_add_one_port(&sc16is7x2_uart_driver, &ts->channel[0].uart);

	if (ret)
		goto exit_destroy;

//	ret = uart_add_one_port(&sc16is7x2_uart_driver, &ts->channel[1].uart);
	ret = sc16is7x2_register_uart_port(ts, 1);
	if (ret)
		goto exit_uart0;

	/* init gpio */
#ifdef CONFIG_GPIOLIB
	ts->gpio.label = (ts->gpio_label) ? ts->gpio_label : DRIVER_NAME;
	//ts->gpio.request	= sc16is7x2_gpio_request;
	//ts->gpio.free		= sc16is7x2_gpio_free;
	ts->gpio.get		= sc16is7x2_gpio_get;
	ts->gpio.set		= sc16is7x2_gpio_set;
	ts->gpio.direction_input = sc16is7x2_direction_input;
	ts->gpio.direction_output = sc16is7x2_direction_output;

	ts->gpio.base = ts->gpio_base;
	ts->gpio.names = ts->gpio_names;
	ts->gpio.ngpio = SC16IS7X2_NR_GPIOS;
	ts->gpio.can_sleep = 0;
	ts->gpio.dev = &ts->spi->dev;
	ts->gpio.owner = THIS_MODULE;

	mutex_init(&ts->io_lock);

	ts->io_gpio = 0;
	ts->io_state = 0;
	ts->io_dir = 0;

	sc16is7x2_set_and_write_chipreg(ts, SC16IS7XX_IOINTENA_REG, 0);
	sc16is7x2_set_and_write_chipreg(ts, SC16IS7XX_IOSTATE_REG, 0);
	sc16is7x2_set_and_write_chipreg(ts, SC16IS7XX_IODIR_REG, 0);

	/* prepare channels for fast startup */
	for(ch = 0; ch < 2; ch++){
		sc16is7x2_set_and_write(&ts->channel[ch], SC16IS7XX_IER_REG, 0);
		/*
		sc16is7x2_set_and_write(&ts->channel[ch], SC16IS7XX_FCR_REG,
				SC16IS7XX_FCR_RXRESET_BIT |
				SC16IS7XX_FCR_TXRESET_BIT |
				SC16IS7XX_FCR_FIFO_BIT);
		sc16is7x2_write_val(&ts->channel[ch], SC16IS7XX_LCR_REG,
				SC16IS7XX_LCR_CONF_MODE_B);
		sc16is7x2_set_and_write(&ts->channel[ch], SC16IS7XX_EFCR_REG,
				SC16IS7XX_EFCR_RXDISABLE_BIT |
				SC16IS7XX_EFCR_TXDISABLE_BIT);
		*/
	}

	ret =  gpiochip_add(&ts->gpio);
	if (ret)
		goto exit_uart1;
#endif


	ret = request_irq(spi->irq, sc16is7x2_irq,
			IRQF_TRIGGER_FALLING | IRQF_SHARED,
			"sc16is7x2", ts);
	if (ret)
		goto exit_gpio;

	dev_info(&spi->dev, DRIVER_NAME " at CS%d (irq %d), 2 UARTs, 8 GPIOs\n"
			"    ttyNSC%d, ttyNSC%d, gpiochip%d\n",
			spi->chip_select, spi->irq,
			ts->uart_base, ts->uart_base + 1,
			ts->gpio_base);

	return 0;

exit_gpio:
#ifdef CONFIG_GPIOLIB
	gpiochip_remove(&ts->gpio);
#endif

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

	/* Free the interrupt */
	free_irq(spi->irq, ts);

	ret = sc16is7x2_remove_one_port(ts, 0);
	if (ret)
		return ret;

	ret = sc16is7x2_remove_one_port(ts, 1);
	if (ret)
		return ret;

#ifdef CONFIG_GPIOLIB
	gpiochip_remove(&ts->gpio);
#endif

	kfree(ts);

	return 0;
}






static void sc16is7x2_console_putchar(struct uart_port *port, int character)
{
	struct sc16is7x2_channel *chan =  to_sc16is7x2_channel(port);
	unsigned ch = (chan == chan->chip->channel) ? 0 : 1;
    sc16is7x2_set_and_write(chan, UART_TX, character);


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
	struct sc16is7x2_channel *ch;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	printk("sc16is7x2_console_setup\n");

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

	//~ printk("setup ch->uart=%d, co=%d\n", &ch->uart, co);

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
