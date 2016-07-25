/*
 * flexcan.c - FLEXCAN CAN controller driver
 *
 * Copyright (c) 2005-2006 Varma Electronics Oy
 * Copyright (c) 2009 Sascha Hauer, Pengutronix
 * Copyright (c) 2010 Marc Kleine-Budde, Pengutronix
 *
 * Based on code originally by Andrey Volkov <avolkov@varma-el.com>
 *
 * LICENCE:
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define USE_STRIM_FLEXCAN

#include <linux/netdevice.h>
#ifndef USE_STRIM_FLEXCAN
#include <linux/can.h>
#include <linux/can/dev.h>
#endif
#include <linux/can/error.h>
#ifndef USE_STRIM_FLEXCAN
#include <linux/can/led.h>
#endif
#include <linux/can/platform/flexcan.h>
#include <linux/clk.h>
#include <linux/delay.h>
#ifndef USE_STRIM_FLEXCAN
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#endif
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regmap.h>

#ifdef USE_STRIM_FLEXCAN
#include <linux/types.h>
#include <linux/fs.h>
#ifdef USE_FLEXCAN_PROC
#include <linux/proc_fs.h>
#endif
#include <linux/uaccess.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/kdev_t.h>
#include <linux/version.h>
#include <asm/io.h>
#include <linux/string.h>
#include <linux/wait.h>
#include <linux/fcntl.h>
#include <linux/poll.h>
#include <linux/init.h>

#include "chr_flexcan.h"

#define FLEXCAN_DRV_NAME           "flexcan"
#define FLEXCAN_DRV_VER            "1.3.56"
#else
#define DRV_NAME			"flexcan"
#endif
/* 8 for RX fifo and 2 error handling */
#define FLEXCAN_NAPI_WEIGHT		(8 + 2)

/* FLEXCAN module configuration register (CANMCR) bits */
#define FLEXCAN_MCR_MDIS		BIT(31)
#define FLEXCAN_MCR_FRZ			BIT(30)
#define FLEXCAN_MCR_FEN			BIT(29)
#define FLEXCAN_MCR_HALT		BIT(28)
#define FLEXCAN_MCR_NOT_RDY		BIT(27)
#define FLEXCAN_MCR_WAK_MSK		BIT(26)
#define FLEXCAN_MCR_SOFTRST		BIT(25)
#define FLEXCAN_MCR_FRZ_ACK		BIT(24)
#define FLEXCAN_MCR_SUPV		BIT(23)
#define FLEXCAN_MCR_SLF_WAK		BIT(22)
#define FLEXCAN_MCR_WRN_EN		BIT(21)
#define FLEXCAN_MCR_LPM_ACK		BIT(20)
#define FLEXCAN_MCR_WAK_SRC		BIT(19)
#define FLEXCAN_MCR_DOZE		BIT(18)
#define FLEXCAN_MCR_SRX_DIS		BIT(17)
#define FLEXCAN_MCR_BCC			BIT(16)
#define FLEXCAN_MCR_LPRIO_EN		BIT(13)
#define FLEXCAN_MCR_AEN			BIT(12)
#define FLEXCAN_MCR_MAXMB(x)		((x) & 0x1f)
#define FLEXCAN_MCR_IDAM_A		(0 << 8)
#define FLEXCAN_MCR_IDAM_B		(1 << 8)
#define FLEXCAN_MCR_IDAM_C		(2 << 8)
#define FLEXCAN_MCR_IDAM_D		(3 << 8)

/* FLEXCAN control register (CANCTRL) bits */
#define FLEXCAN_CTRL_PRESDIV(x)		(((x) & 0xff) << 24)
#define FLEXCAN_CTRL_RJW(x)		(((x) & 0x03) << 22)
#define FLEXCAN_CTRL_PSEG1(x)		(((x) & 0x07) << 19)
#define FLEXCAN_CTRL_PSEG2(x)		(((x) & 0x07) << 16)
#define FLEXCAN_CTRL_BOFF_MSK		BIT(15)
#define FLEXCAN_CTRL_ERR_MSK		BIT(14)
#define FLEXCAN_CTRL_CLK_SRC		BIT(13)
#define FLEXCAN_CTRL_LPB		BIT(12)
#define FLEXCAN_CTRL_TWRN_MSK		BIT(11)
#define FLEXCAN_CTRL_RWRN_MSK		BIT(10)
#define FLEXCAN_CTRL_SMP		BIT(7)
#define FLEXCAN_CTRL_BOFF_REC		BIT(6)
#define FLEXCAN_CTRL_TSYN		BIT(5)
#define FLEXCAN_CTRL_LBUF		BIT(4)
#define FLEXCAN_CTRL_LOM		BIT(3)
#define FLEXCAN_CTRL_PROPSEG(x)		((x) & 0x07)
#define FLEXCAN_CTRL_ERR_BUS		(FLEXCAN_CTRL_ERR_MSK)
#define FLEXCAN_CTRL_ERR_STATE \
	(FLEXCAN_CTRL_TWRN_MSK | FLEXCAN_CTRL_RWRN_MSK | \
	 FLEXCAN_CTRL_BOFF_MSK)
#define FLEXCAN_CTRL_ERR_ALL \
	(FLEXCAN_CTRL_ERR_BUS | FLEXCAN_CTRL_ERR_STATE)

/* FLEXCAN error and status register (ESR) bits */
#define FLEXCAN_ESR_TWRN_INT		BIT(17)
#define FLEXCAN_ESR_RWRN_INT		BIT(16)
#define FLEXCAN_ESR_BIT1_ERR		BIT(15)
#define FLEXCAN_ESR_BIT0_ERR		BIT(14)
#define FLEXCAN_ESR_ACK_ERR		BIT(13)
#define FLEXCAN_ESR_CRC_ERR		BIT(12)
#define FLEXCAN_ESR_FRM_ERR		BIT(11)
#define FLEXCAN_ESR_STF_ERR		BIT(10)
#define FLEXCAN_ESR_TX_WRN		BIT(9)
#define FLEXCAN_ESR_RX_WRN		BIT(8)
#define FLEXCAN_ESR_IDLE		BIT(7)
#define FLEXCAN_ESR_TXRX		BIT(6)
#define FLEXCAN_EST_FLT_CONF_SHIFT	(4)
#define FLEXCAN_ESR_FLT_CONF_MASK	(0x3 << FLEXCAN_EST_FLT_CONF_SHIFT)
#define FLEXCAN_ESR_FLT_CONF_ACTIVE	(0x0 << FLEXCAN_EST_FLT_CONF_SHIFT)
#define FLEXCAN_ESR_FLT_CONF_PASSIVE	(0x1 << FLEXCAN_EST_FLT_CONF_SHIFT)
#define FLEXCAN_ESR_BOFF_INT		BIT(2)
#define FLEXCAN_ESR_ERR_INT		BIT(1)
#define FLEXCAN_ESR_WAK_INT		BIT(0)
#define FLEXCAN_ESR_ERR_BUS \
	(FLEXCAN_ESR_BIT1_ERR | FLEXCAN_ESR_BIT0_ERR | \
	 FLEXCAN_ESR_ACK_ERR | FLEXCAN_ESR_CRC_ERR | \
	 FLEXCAN_ESR_FRM_ERR | FLEXCAN_ESR_STF_ERR)
#define FLEXCAN_ESR_ERR_STATE \
	(FLEXCAN_ESR_TWRN_INT | FLEXCAN_ESR_RWRN_INT | FLEXCAN_ESR_BOFF_INT)
#define FLEXCAN_ESR_ERR_ALL \
	(FLEXCAN_ESR_ERR_BUS | FLEXCAN_ESR_ERR_STATE)
#define FLEXCAN_ESR_ALL_INT \
	(FLEXCAN_ESR_TWRN_INT | FLEXCAN_ESR_RWRN_INT | \
	 FLEXCAN_ESR_BOFF_INT | FLEXCAN_ESR_ERR_INT | \
	 FLEXCAN_ESR_WAK_INT)

/* FLEXCAN interrupt flag register (IFLAG) bits */
#define FLEXCAN_RESERVED_BUF_ID		8
#define FLEXCAN_TX_BUF_ID		13
#define FLEXCAN_IFLAG_BUF(x)		BIT(x)
#define FLEXCAN_IFLAG_RX_FIFO_OVERFLOW	BIT(7)
#define FLEXCAN_IFLAG_RX_FIFO_WARN	BIT(6)
#define FLEXCAN_IFLAG_RX_FIFO_AVAILABLE	BIT(5)
#define FLEXCAN_IFLAG_DEFAULT \
	(FLEXCAN_IFLAG_RX_FIFO_OVERFLOW | FLEXCAN_IFLAG_RX_FIFO_AVAILABLE | \
	 FLEXCAN_IFLAG_BUF(FLEXCAN_TX_BUF_ID))

/* FLEXCAN message buffers */
#define FLEXCAN_MB_CNT_CODE(x)		(((x) & 0xf) << 24)
#define FLEXCAN_MB_CNT_SRR		BIT(22)
#define FLEXCAN_MB_CNT_IDE		BIT(21)
#define FLEXCAN_MB_CNT_RTR		BIT(20)
#define FLEXCAN_MB_CNT_LENGTH(x)	(((x) & 0xf) << 16)
#define FLEXCAN_MB_CNT_TIMESTAMP(x)	((x) & 0xffff)

#define FLEXCAN_MB_CODE_MASK		(0xf0ffffff)

/*
 * FLEXCAN hardware feature flags
 *
 * Below is some version info we got:
 *    SOC   Version   IP-Version  Glitch-  [TR]WRN_INT
 *                                Filter?   connected?
 *   MX25  FlexCAN2  03.00.00.00     no         no
 *   MX28  FlexCAN2  03.00.04.00    yes        yes
 *   MX35  FlexCAN2  03.00.00.00     no         no
 *   MX53  FlexCAN2  03.00.00.00    yes         no
 *   MX6s  FlexCAN3  10.00.12.00    yes        yes
 *
 * Some SOCs do not have the RX_WARN & TX_WARN interrupt line connected.
 */
#define FLEXCAN_HAS_V10_FEATURES	BIT(1) /* For core version >= 10 */
#define FLEXCAN_HAS_BROKEN_ERR_STATE	BIT(2) /* [TR]WRN_INT not connected */
#define FLEXCAN_HAS_ERR005829		BIT(3) /* have errata ERR005829 */

#ifdef USE_STRIM_FLEXCAN

#define FLEXCAN_GPIO_0             (106)
#define FLEXCAN_GPIO_1             (80)

 /* Flexcan precalc bitrate settings */
#define FLEXCAN_BTRT_1000_SP866    (0x01290005)
#define FLEXCAN_BTRT_1000_SP800    (0x012a0004)
#define FLEXCAN_BTRT_1000_SP733    (0x01230004)
#define FLEXCAN_BTRT_1000_SP700    (0x02120002)

#define FLEXCAN_BTRT_1000          (0x01290005) /* Canconfig calculated value */
#define FLEXCAN_BTRT_500           (0x023b0006) /* Canconfig calculated value */
#define FLEXCAN_BTRT_250           (0x053b0006) /* Canconfig calculated value */
#define FLEXCAN_BTRT_125           (0x0b3b0006) /* Canconfig calculated value */
#define FLEXCAN_BTRT_100           (0x0e3b0006) /* Canconfig calculated value */
#define FLEXCAN_BTRT_DFLT          FLEXCAN_BTRT_1000

#define FLEXCAN_BTRT_MASK          (0xFFFF0007)

#define FLEXCAN_AUTOSET_WORK       BIT(0)   /* Идет настройка скорости */
#define FLEXCAN_AUTOSET_END        BIT(1)   /* Настройка скорости завершена */
#define FLEXCAN_AUTOSET_ERR        BIT(2)   /* Ошибка при текущей скорости */
#define FLEXCAN_AUTOSET_OK         BIT(3)   /* Текущая скорость верна */

#define FLEXCAN_DEV_FIRST          (0)
#define FLEXCAN_DEV_COUNT          (2)

#define FLEXCAN_BUF_RECV_CAP       (262144) //131072
#define FLEXCAN_BUF_SEND_CAP       (128)

#define FLEXCAN_BUF_RECV_MASK      ((&f_brecv[dev_num])->capacity - 1)
#define FLEXCAN_BUF_SEND_MASK      ((&f_bsend[dev_num])->capacity - 1)

#define FLEXCAN_IRQ_BASE           (142)

#define init_MUTEX(LOCKNAME)       sema_init(LOCKNAME,1);

#endif

/* Structure of the hardware registers */
struct flexcan_regs {
	u32 mcr;		/* 0x00 */
	u32 ctrl;		/* 0x04 */
	u32 timer;		/* 0x08 */
	u32 _reserved1;		/* 0x0c */
	u32 rxgmask;		/* 0x10 */
	u32 rx14mask;		/* 0x14 */
	u32 rx15mask;		/* 0x18 */
	u32 ecr;		/* 0x1c */
	u32 esr;		/* 0x20 */
	u32 imask2;		/* 0x24 */
	u32 imask1;		/* 0x28 */
	u32 iflag2;		/* 0x2c */
	u32 iflag1;		/* 0x30 */
	u32 crl2;		/* 0x34 */
	u32 esr2;		/* 0x38 */
	u32 imeur;		/* 0x3c */
	u32 lrfr;		/* 0x40 */
	u32 crcr;		/* 0x44 */
	u32 rxfgmask;		/* 0x48 */
	u32 rxfir;		/* 0x4c */
	u32 _reserved3[12];
	struct flexcan_mb cantxfg[64];
};

struct flexcan_devtype_data {
	u32 features;	/* hardware controller features */
};

#ifdef USE_STRIM_FLEXCAN
typedef struct {
    unsigned int n_push, n_pop, n_save_pop;
} flexcan_nbuf_t;

struct flexcan_frame_mb {
    struct flexcan_mb mb;
    struct timeval time;
};

struct flexcan_c_device {
/* Имя устройства. Это же имя отдается файлу устройства в /dev и /proc */
    char name[IFNAMSIZ];

/* Адрес начала блока памяти устройства. Сохраняется при регистрации устройства.
 * Используется для чтения регистров устройства с помощью struct flexcan_regs */
    void __iomem *base;

/* Содержит major и minor номер зарегистрированного устройства.
 * Номера извлекаются макросами MAJOR(dev_t devt) и MINOR(dev_t devt).
 * Номер формируется при регистрации автоматически и через MKDEV(major, minor) */
    dev_t devt;

/* Флаг открытия файла устройства. Защита от множественного доступа к файлу
 * (а еще для запрета удаления драйвера при открытом файле. Не реализовано) */
    int nreaders, nwriters;

/* Значение регистра CTRL для настройки скорости передачи и режима работы.
 * При инициализации устанавливается значение по умолчанию FLEXCAN_BTRT_DFLT
 * Изменяется через IOCTL запрос */
    u32 reg_ctrl_bittiming;

/* Значение по умолчанию регистра CTRL для будущих использований.
 * Создается в функции chip_start */
    u32 reg_ctrl_default;

    u8 autoset_bittiming_flags; /* Флаги процесса автонастройки скорости */
    u32 reg_ctrl_autoset;       /* Сохраненное значение регистра CTRL при настрйоке скорости */

    u8 irq_num;                 /* Номер вектора прерывания IRQ для устройства */
    struct cdev chrdev;        /* Структура символьного утсройства */
    struct device dev;         /* Структура утсройства */

/* Указатель на структуру встроенного в контроллер устройства flexcan */
    struct flexcan_platform_data *pdata;

#ifdef USE_FLEXCAN_PROC
/* Структура с данными файла устройства в файловой системе /proc  */
    struct proc_dir_entry *dev_proc_file;
#endif

    struct clk *clk_ipg;
    struct clk *clk_per;

/* Структура с данными о настройках таймингов устройства */
    struct can_bittiming *bittiming;

/* Содержит различные счетчики и переменные состояния устройства */
    struct flexcan_stats *stats;

    struct semaphore sem;               /* Семафор */
    wait_queue_head_t inq, outq;         /* Очереди ожидания данных */
    struct fasync_struct *async_queue;  /* Структура для вызовов poll и select */

/* Номер GPIO порта для моргания */
    int gpio_led;

    // int (*do_set_bittiming)(__u8 *dev_num, struct can_bittiming *bittiming);
    int (*do_set_mode)(__u8 *dev_num, enum can_mode mode);                  // вернуть когда буду убирать netdevice
    // int (*do_get_state)(__u8 *dev_num, enum can_state *state);
    int (*do_get_berr_counter)(__u8 *dev_num, struct flexcan_stats *bec);
};


struct flexcan_c_brecv {
    /* Размер буфера для сохранения принятых сообщений */
    u32 capacity;
    /* Номера ячеек для записи и чтения буффера для сохранения */
    flexcan_nbuf_t nbuf;
    /* Циклический буффер для сохранения принятых сообщений */
    struct flexcan_frame_mb *buf;
};


struct flexcan_c_bsend {
    /* Размер буфера для сохранения сообщений на отправку */
    u32 capacity;
    /* Номера ячеек для записи и чтения буффера сообщений на отправку  */
    flexcan_nbuf_t nbuf;
    /* Циклический буффер для сохранения сообщений на отправку */
    struct can_frame *buf;
};


struct flexcan_c_driver {
    char name[PLATFORM_NAME_SIZE];
    u8 is_init;                 /* флаг инициализации char группы*/
    dev_t devt;               /* major номер группы */

    struct class *f_class;
    const struct flexcan_devtype_data *devtype_data;
#ifdef USE_FLEXCAN_PROC
    struct proc_dir_entry *dev_proc_dir;
#endif
};
#else
struct flexcan_stop_mode {
	struct regmap *gpr;
	u8 req_gpr;
	u8 req_bit;
	u8 ack_gpr;
	u8 ack_bit;
};
struct flexcan_priv {
	struct can_priv can;
	struct net_device *dev;
	struct napi_struct napi;

	void __iomem *base;
	u32 reg_esr;
	u32 reg_ctrl_default;

	struct clk *clk_ipg;
	struct clk *clk_per;
	struct flexcan_platform_data *pdata;
	const struct flexcan_devtype_data *devtype_data;
	struct flexcan_stop_mode stm;
	int stby_gpio;
	enum of_gpio_flags stby_gpio_flags;
	int id;
};
#endif


static struct flexcan_devtype_data fsl_p1010_devtype_data = {
	.features = FLEXCAN_HAS_BROKEN_ERR_STATE,
};
static struct flexcan_devtype_data fsl_imx28_devtype_data;
static struct flexcan_devtype_data fsl_imx6q_devtype_data = {
	.features = FLEXCAN_HAS_V10_FEATURES | FLEXCAN_HAS_ERR005829,
};

static const struct can_bittiming_const flexcan_bittiming_const = {
#ifdef USE_STRIM_FLEXCAN
    .name = FLEXCAN_DRV_NAME,
#else
	.name = DRV_NAME,
#endif
	.tseg1_min = 4,
	.tseg1_max = 16,
	.tseg2_min = 2,
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_min = 1,
	.brp_max = 256,
	.brp_inc = 1,
};

#ifdef USE_STRIM_FLEXCAN
/* Глобальные структуры */
static struct flexcan_c_driver *f_drv;
static struct flexcan_c_device *f_chrdev;

static struct flexcan_c_bsend *f_bsend;
static struct flexcan_c_brecv *f_brecv;

static const flexcan_gpio_num[2] = {FLEXCAN_GPIO_0, FLEXCAN_GPIO_1};


static unsigned int flexcan_start_transmit(const u8 dev_num);
static void flexcan_set_bittiming(const u8 dev_num, const u32 reg_ctrl);
static void flexcan_chip_stop(const u8 dev_num);
static int flexcan_chip_start(const u8 dev_num);
static int flexcan_autoset_baudrate(const u8 dev_num);
#endif

/*
 * Abstract off the read/write for arm versus ppc.
 */
#if defined(__BIG_ENDIAN)
static inline u32 flexcan_read(void __iomem *addr)
{
	return in_be32(addr);
}

static inline void flexcan_write(u32 val, void __iomem *addr)
{
	out_be32(addr, val);
}
#else
static inline u32 flexcan_read(void __iomem *addr)
{
	return readl(addr);
}

static inline void flexcan_write(u32 val, void __iomem *addr)
{
	writel(val, addr);
}
#endif


#ifdef USE_STRIM_FLEXCAN
static inline int flexcan_c_is_init(void)
{
    if(IS_ERR_OR_NULL(f_drv)) {
        return -ENOMEM;
    }
    else if(!f_drv->is_init) {
        return 0;
    }

    return 1;
}


static inline unsigned int flexcan_c_rbuf_is_empty(const u8 dev_num)
{
    struct flexcan_c_brecv *f_buf = &f_brecv[dev_num];
    flexcan_nbuf_t *f_nbuf = &(f_buf->nbuf);

    return (f_nbuf->n_push == f_nbuf->n_pop);
}


static inline unsigned int flexcan_c_rbuf_is_full(const u8 dev_num)
{
    struct flexcan_c_brecv *f_buf = &f_brecv[dev_num];
    flexcan_nbuf_t *f_nbuf = &(f_buf->nbuf);

    unsigned int n_next_first = (f_nbuf->n_push + 1) & FLEXCAN_BUF_RECV_MASK;
    return (n_next_first == f_nbuf->n_pop);
}


static inline unsigned int flexcan_c_rbuf_push(const u8 dev_num, const struct flexcan_mb *s_mb, const struct timeval *s_tv)
{
    struct flexcan_c_brecv *f_buf = &f_brecv[dev_num];
    flexcan_nbuf_t *f_nbuf = &(f_buf->nbuf);
    struct flexcan_frame_mb *buf;

    if(flexcan_c_rbuf_is_full(dev_num)) {
        return 1;
    }

    buf = (struct flexcan_frame_mb *) (f_buf->buf + f_nbuf->n_push);
    memcpy(&buf->mb, s_mb, sizeof(struct flexcan_mb));
    memcpy(&buf->time, s_tv, sizeof(struct timeval));
    f_nbuf->n_push = (f_nbuf->n_push + 1) & FLEXCAN_BUF_RECV_MASK;

    return 0;
}


static inline unsigned int flexcan_c_rbuf_pop(const u8 dev_num, struct flexcan_frame_mb *s_frame)
{
    struct flexcan_c_brecv *f_buf = &f_brecv[dev_num];
    flexcan_nbuf_t *f_nbuf = &(f_buf->nbuf);

    if(flexcan_c_rbuf_is_empty(dev_num))  {
        return 1;
    }

    *(struct flexcan_frame_mb *) s_frame = *(struct flexcan_frame_mb *) (f_buf->buf + f_nbuf->n_pop);
    f_nbuf->n_pop = (f_nbuf->n_pop + 1) & FLEXCAN_BUF_RECV_MASK;
    f_nbuf->n_save_pop = f_nbuf->n_pop;

    return 0;
}


static inline unsigned int flexcan_c_rbuf_free_space(const u8 dev_num)
{
    struct flexcan_c_brecv *f_buf = &f_brecv[dev_num];
    flexcan_nbuf_t *f_nbuf = &(f_buf->nbuf);

    unsigned int buf_free_space = (f_nbuf->n_pop - f_nbuf->n_push - 1) & FLEXCAN_BUF_RECV_MASK;
    return buf_free_space;
}


static inline unsigned int flexcan_c_rbuf_get_data_size(const u8 dev_num)
{
    struct flexcan_c_brecv *f_buf = &f_brecv[dev_num];
    flexcan_nbuf_t *f_nbuf = &(f_buf->nbuf);

    // unsigned int buf_data_size = (f_buf->n_push - f_buf->n_pop) & FLEXCAN_BUF_RECV_MASK;
    unsigned int buf_data_size = (f_nbuf->n_push - f_nbuf->n_save_pop) & FLEXCAN_BUF_RECV_MASK;
    return buf_data_size;
}


static inline unsigned int flexcan_c_rbuf_end(const u8 dev_num)
{
    struct flexcan_c_brecv *f_buf = &f_brecv[dev_num];
    flexcan_nbuf_t *f_nbuf = &(f_buf->nbuf);

    f_nbuf->n_pop = f_nbuf->n_push;
    return 0;
}


static inline unsigned int flexcan_c_rbuf_begin(const u8 dev_num)
{
    struct flexcan_c_brecv *f_buf = &f_brecv[dev_num];
    flexcan_nbuf_t *f_nbuf = &(f_buf->nbuf);

    f_nbuf->n_pop = f_nbuf->n_save_pop;
    return 0;
}


static inline unsigned int flexcan_c_sbuf_is_empty(const u8 dev_num)
{
    struct flexcan_c_bsend *f_buf = &f_bsend[dev_num];
    flexcan_nbuf_t *f_nbuf = &(f_buf->nbuf);

    return (f_nbuf->n_push == f_nbuf->n_pop);
}


static inline unsigned int flexcan_c_sbuf_is_full(const u8 dev_num)
{
    struct flexcan_c_bsend *f_buf = &f_bsend[dev_num];
    flexcan_nbuf_t *f_nbuf = &(f_buf->nbuf);

    unsigned int n_next_first = (f_nbuf->n_push + 1) & FLEXCAN_BUF_SEND_MASK;
    return (n_next_first == f_nbuf->n_pop);
}


static inline unsigned int flexcan_c_sbuf_push(const u8 dev_num, const struct can_frame *s_cf)
{
    struct flexcan_c_bsend *f_buf = &f_bsend[dev_num];
    flexcan_nbuf_t *f_nbuf = &(f_buf->nbuf);
    struct can_frame *buf;

    if(flexcan_c_sbuf_is_full(dev_num))   {
        return 1;
    }

    buf = (struct can_frame *) (f_buf->buf + f_nbuf->n_push);
    memcpy(buf, s_cf, sizeof(struct can_frame));
    f_nbuf->n_push = (f_nbuf->n_push + 1) & FLEXCAN_BUF_SEND_MASK;

    return 0;
}


static inline unsigned int flexcan_c_sbuf_pop(const u8 dev_num, struct can_frame *s_frame)
{
    struct flexcan_c_bsend *f_buf = &f_bsend[dev_num];
    flexcan_nbuf_t *f_nbuf = &(f_buf->nbuf);

    if(flexcan_c_sbuf_is_empty(dev_num))  {
        return 1;
    }

    *(struct can_frame *) s_frame = *(struct can_frame *) (f_buf->buf + f_nbuf->n_pop);
    f_nbuf->n_pop = (f_nbuf->n_pop + 1) & FLEXCAN_BUF_SEND_MASK;

    return 0;
}


static inline unsigned int flexcan_c_sbuf_free_space(const u8 dev_num)
{
    struct flexcan_c_bsend *f_buf = &f_bsend[dev_num];
    flexcan_nbuf_t *f_nbuf = &(f_buf->nbuf);

    unsigned int buf_free_space = (f_nbuf->n_pop - f_nbuf->n_push - 1) & FLEXCAN_BUF_SEND_MASK;
    return buf_free_space;
}


static inline unsigned int flexcan_c_sbuf_data_size(const u8 dev_num)
{
    struct flexcan_c_bsend *f_buf = &f_bsend[dev_num];
    flexcan_nbuf_t *f_nbuf = &(f_buf->nbuf);

    unsigned int buf_data_size = (f_nbuf->n_push - f_nbuf->n_pop) & FLEXCAN_BUF_SEND_MASK;
    return buf_data_size;
}



static void flexcan_c_decode_frame(struct can_frame *cf, struct flexcan_mb *mb)
{
    if (mb->can_ctrl & FLEXCAN_MB_CNT_IDE)  {
        cf->can_id = ((mb->can_id >> 0) & CAN_EFF_MASK) | CAN_EFF_FLAG;
    }
    else    {
        cf->can_id = (mb->can_id >> 18) & CAN_SFF_MASK;
    }

    if (mb->can_ctrl & FLEXCAN_MB_CNT_RTR)  {
        cf->can_id |= CAN_RTR_FLAG;
    }
    cf->can_dlc = ((mb->can_ctrl >> 16) & 0xf);

    if(cf->can_dlc == 0)    {
        *(__be32 *)(cf->data + 0) = 0x00000000;
        *(__be32 *)(cf->data + 4) = 0x00000000;
    }
    else if(cf->can_dlc <= 4)   {
        *(__be32 *)(cf->data + 0) = cpu_to_be32(flexcan_read(&mb->data[0]));
        *(__be32 *)(cf->data + 4) = 0x00000000;
    }
    else    {
        *(__be32 *)(cf->data + 0) = cpu_to_be32(flexcan_read(&mb->data[0]));
        *(__be32 *)(cf->data + 4) = cpu_to_be32(flexcan_read(&mb->data[1]));
    }
}


static int flexcan_c_file_fasync(int fd, struct file *filp, int mode)
{
    struct inode *inode = filp->f_dentry->d_inode;
    const u8 dev_num = iminor(inode);
//    struct flexcan_c_device *f_cdev = &f_chrdev[dev_num];

    return fasync_helper(fd, filp, mode, &((&f_chrdev[dev_num])->async_queue));
}


static int flexcan_c_file_open(struct inode *i, struct file *filp)
{
    const u8 dev_num = iminor(i);
    struct flexcan_c_device *f_cdev = &f_chrdev[dev_num];

    if(down_interruptible(&f_cdev->sem)) {
        return -ERESTARTSYS;
    }
    if(filp->f_mode & FMODE_READ) {
        f_cdev->nreaders++;
    }
    if (filp->f_mode & FMODE_WRITE) {
        f_cdev->nwriters++;
    }
    up(&f_cdev->sem);

    dev_dbg(&f_cdev->dev, "chardev file open success\n");

    try_module_get(THIS_MODULE);
    // return nonseekable_open(i, filp);    // если отключим llseek
    return 0;
}


static int flexcan_c_file_release(struct inode *i, struct file *filp)
{
    const u8 dev_num = iminor(i);
    struct flexcan_c_device *f_cdev = &f_chrdev[dev_num];

//    printk("%s.%d: %s\n", f_drv->name, dev_num, __func__);

    flexcan_c_file_fasync(-1, filp, 0);

    down(&f_cdev->sem);
    if (filp->f_mode & FMODE_READ) {
        f_cdev->nreaders--;
    }
    if (filp->f_mode & FMODE_WRITE) {
        f_cdev->nwriters--;
    }
    up(&f_cdev->sem);

    module_put(THIS_MODULE);

    return 0;
}


static loff_t flexcan_c_file_llseek(struct file *filp, loff_t off, int whence)
{
    loff_t newpos = 0;
    struct inode *inode = filp->f_path.dentry->d_inode;
    const u8 dev_num = iminor(inode);
//    struct flexcan_c_device *f_cdev = &f_chrdev[dev_num];

//    printk("%s.%d: %s\n", f_drv->name, dev_num, __func__);

    switch(whence) {
        case 0: /* SEEK_SET */
            flexcan_c_rbuf_begin(dev_num);
            newpos = 0; /* Всегда в начало */
            break;
        case 1: /* SEEK_CUR */
            newpos = flexcan_c_rbuf_get_data_size(dev_num) * CAN_DATA_MSG_LENGTH;
            break;
        case 2: /* SEEK_END */
            flexcan_c_rbuf_end(dev_num);
            newpos = flexcan_c_rbuf_get_data_size(dev_num) * CAN_DATA_MSG_LENGTH;
            break;
        default: /* не может произойти */
            return -EINVAL;
    }

    if(newpos < 0) {
        return -EINVAL;
    }

    filp->f_pos = newpos;
    return newpos;
}


static unsigned int flexcan_c_file_poll(struct file *filp, poll_table *wait)
{

    unsigned int mask = 0;
    struct inode *inode = filp->f_path.dentry->d_inode;
    const u8 dev_num = iminor(inode);
    struct flexcan_c_device *f_cdev = &f_chrdev[dev_num];
    unsigned int read_size = 0;

//    printk("%s.%d: %s\n", f_drv->name, dev_num, __func__);

    down(&f_cdev->sem);

    poll_wait(filp, &f_cdev->inq,  wait);
    poll_wait(filp, &f_cdev->outq, wait);

    read_size = flexcan_c_rbuf_get_data_size(dev_num);
    if(read_size) {
        mask |= POLLIN | POLLRDNORM;    /* readable */
    }
    if(flexcan_c_sbuf_is_full(dev_num) == 0) {
        mask |= POLLOUT | POLLWRNORM;   /* writable */
    }

    up(&f_cdev->sem);

    return mask;
}


#define IS_EMPTY (flexcan_c_rbuf_is_empty(dev_num))


static ssize_t flexcan_c_file_read(struct file *filp, char __user *buf, size_t length_read, loff_t *off)
{
    struct can_frame cf_decoded;
    struct flexcan_frame_mb read_frame;
    char data_msg_buf[CAN_DATA_MSG_LENGTH + 1];

    struct inode *inode = filp->f_path.dentry->d_inode;
    const u8 dev_num = iminor(inode);
    struct flexcan_c_device *f_cdev = &f_chrdev[dev_num];

    int ret = 0, is_empty = 0;
    u64 msg_data = 0;
    ssize_t total_length = 0;

//    printk("%s.%d: %s\n", f_drv->name, dev_num, __func__);

    if(length_read < CAN_DATA_MSG_LENGTH) { /* Проверяем размер буффера пользователя */
        dev_dbg(&f_cdev->dev, "chardev read return EFAULT\n");
        return -EINVAL;
    }
    if(down_interruptible(&f_cdev->sem)) {
        return -ERESTARTSYS;
    }

    is_empty = IS_EMPTY;
    while(is_empty) { /* Проверяем пустой ли буффер */
        up(&f_cdev->sem); /* release the lock */
        if (filp->f_flags & O_NONBLOCK) {   /* если вызов не блокирующий, вываливаемся */
            dev_dbg(&f_cdev->dev, "chardev read return EAGAIN\n");
            printk("%s.%d: %s (filp->f_flags & O_NONBLOCK). return -EAGAIN\n", f_drv->name, dev_num, __func__);
            return -EAGAIN;
        }
        if (wait_event_interruptible(f_cdev->inq, !IS_EMPTY)) {
            printk("%s.%d: %s wait_event_interruptible. return -ERESTARTSYS\n", f_drv->name, dev_num, __func__);
//            printk("%s.%d: %s flexcan_c_rbuf_is_empty is %d, flexcan_c_rbuf_get_data_size is %d\n", f_drv->name, dev_num, __func__, flexcan_c_rbuf_is_empty(dev_num), flexcan_c_rbuf_get_data_size(dev_num));
            return -ERESTARTSYS; /* signal: tell the fs layer to handle it */
        }
        if (down_interruptible(&f_cdev->sem)) {
            printk("%s.%d: %s down_interruptible 2. return -ERESTARTSYS\n", f_drv->name, dev_num, __func__);
            return -ERESTARTSYS;
        }
        is_empty = IS_EMPTY;
    }

    do {
        if(flexcan_c_rbuf_pop(dev_num, &read_frame)) {
            dev_dbg(&f_cdev->dev, "chardev read err\n");
            break;
        }

        msg_data = 0;
        flexcan_c_decode_frame(&cf_decoded, &read_frame.mb);
        memcpy((void*) &msg_data, (void*) cf_decoded.data, cf_decoded.can_dlc);

        ret = snprintf(data_msg_buf, (CAN_DATA_MSG_LENGTH + 1), "$CAN,%d,%08x,%05x,%08x,%d,%016llx\n", dev_num, (unsigned int) read_frame.time.tv_sec,
            (unsigned int) read_frame.time.tv_usec, cf_decoded.can_id, cf_decoded.can_dlc, msg_data);
        if(ret != CAN_DATA_MSG_LENGTH) {
            dev_dbg(&f_cdev->dev, "chardev read error size: need[%d], ret[%d]\n", CAN_DATA_MSG_LENGTH, ret);
            continue;
        }

        ret = copy_to_user((void *) buf, &data_msg_buf, CAN_DATA_MSG_LENGTH);
        if(ret) {
            dev_dbg(&f_cdev->dev, "chardev read can't copy = %d bytes\n", ret);
            buf += (CAN_DATA_MSG_LENGTH - ret);
            total_length += (CAN_DATA_MSG_LENGTH - ret);
            break;
        }
        else {
            buf += CAN_DATA_MSG_LENGTH;
            total_length += CAN_DATA_MSG_LENGTH;
        }
    } while(((total_length + CAN_DATA_MSG_LENGTH) <= length_read) && (!IS_EMPTY));

    up (&f_cdev->sem);
    wake_up_interruptible(&f_cdev->outq);

    return total_length;
}


static ssize_t flexcan_c_file_write(struct file *filp, const char __user *buf, size_t length, loff_t *off)
{
    struct can_frame cf;
    struct can_frame* user_buf_ptr = (struct can_frame *) buf;

    struct inode *inode = filp->f_path.dentry->d_inode;
    const u8 dev_num = iminor(inode);
    struct flexcan_c_device *f_cdev = &f_chrdev[dev_num];

    int ret = 0, to_read = 0, was_read = 0;

    if(length < sizeof(struct can_frame)) { /* Проверяем размер сообщения */
        dev_dbg(&f_cdev->dev, "chardev write return -EFAULT\n");
        return -EINVAL;
    }
    to_read = (((int) length) / sizeof(struct can_frame));

    if(down_interruptible(&f_cdev->sem)) {
        return -ERESTARTSYS;
    }

    while(flexcan_c_sbuf_is_full(dev_num)) {  /* Проверяем наличие данных в буфере */
        flexcan_start_transmit(dev_num);

        DEFINE_WAIT(wait);
        up(&f_cdev->sem);
        if (filp->f_flags & O_NONBLOCK) {
            return -EAGAIN;
        }
        prepare_to_wait(&f_cdev->outq, &wait, TASK_INTERRUPTIBLE);
        finish_wait(&f_cdev->outq, &wait);
        if (signal_pending(current)) {
            return -ERESTARTSYS; /* signal: tell the fs layer to handle it */
        }
        if (down_interruptible(&f_cdev->sem)) {
            return -ERESTARTSYS;
        }
    }

    do {    /* Получение данных */
        ret = copy_from_user(&cf, user_buf_ptr++, sizeof(struct can_frame));
        if(ret) {
            dev_dbg(&f_cdev->dev, "chardev write can't copy = %d bytes\n", ret);
            // printk("%s.%d: %s copy_from_user can't copy = %d bytes\n", f_drv->name, (int) dev_num, __func__, ret);
        }
        else {
            dev_dbg(&f_cdev->dev, "chardev write frame: 0x%08x, %d, 0x%llx\n", cf.can_id, cf.can_dlc, (long long unsigned int) cf.data[0]);
            if(cf.can_dlc <= 8) {
                if(flexcan_c_sbuf_push(dev_num, &cf)) {
                    dev_dbg(&f_cdev->dev, "chardev write buffer drop frame\n");
                    // printk("%s.%d: %s buffer drop frame\n", f_drv->name, (int) dev_num, __func__);
                    break;
                }
            flexcan_start_transmit(dev_num);
            }
            else {
                dev_dbg(&f_cdev->dev, "chardev write BAD frame\n");
                // printk("%s.%d: %s BAD frame\n", f_drv->name, (int) dev_num, __func__);
            }
            was_read++;
            to_read--;
        }
    } while((to_read > 0) && (ret == 0));

    if(flexcan_c_sbuf_data_size(dev_num)) {
        flexcan_start_transmit(dev_num);
    }

    up(&f_cdev->sem);
    wake_up_interruptible(&f_cdev->inq);  /* blocked in read() and select() */
    if(f_cdev->async_queue) {    /* and signal asynchronous readers, explained late in chapter 5 */
        kill_fasync(&(f_cdev->async_queue), SIGIO, POLL_IN);
    }

    return (was_read * sizeof(struct can_frame));
}


static long flexcan_c_file_ioctl(struct file *filp, unsigned int ioctl_num, unsigned long ioctl_param)
{
    struct inode *inode = filp->f_path.dentry->d_inode;
    const u8 dev_num = iminor(inode);
    struct flexcan_c_device *f_cdev = &f_chrdev[dev_num];

    static u32 set_reg_ctrl = 0;
    u32 read_settings = 0, set_mode = 0, set_bitrate = 0;

    dev_dbg(&f_cdev->dev, "IOCTL request\n");
    switch (ioctl_num) {
        case FLEXCAN_IOCTL_READ:
            put_user(read_settings, (int *) ioctl_param);
            break;
        case FLEXCAN_IOCTL_WRITE:
            get_user(read_settings, (int *) ioctl_param);

            set_mode = read_settings & SET_CAN_MODE_MASK;
            set_bitrate = read_settings & SET_CAN_BITRATE_MASK;

            if(set_bitrate || set_mode) {
                switch (set_bitrate) {
                    case SET_CAN_BITRATE_1000:
                        set_reg_ctrl = FLEXCAN_BTRT_1000;
                        printk("%s.%d: %s set bitrate 1000000\n", f_drv->name, dev_num, __func__);
                        break;
                    case SET_CAN_BITRATE_500:
                        set_reg_ctrl = FLEXCAN_BTRT_500;
                        printk("%s.%d: %s set bitrate 500000\n", f_drv->name, dev_num, __func__);
                        break;
                    case SET_CAN_BITRATE_250:
                        set_reg_ctrl = FLEXCAN_BTRT_250;
                        printk("%s.%d: %s set bitrate 250000\n", f_drv->name, dev_num, __func__);
                        break;
                    case SET_CAN_BITRATE_125:
                        set_reg_ctrl = FLEXCAN_BTRT_125;
                        printk("%s.%d: %s set bitrate 125000\n", f_drv->name, dev_num, __func__);
                        break;
                    case SET_CAN_BITRATE_100:
                        set_reg_ctrl = FLEXCAN_BTRT_100;
                        printk("%s.%d: %s set bitrate 100000\n", f_drv->name, dev_num, __func__);
                        break;
                }
                if (set_mode & SET_CAN_MODE_LOOPBACK) {
                    set_reg_ctrl |= FLEXCAN_CTRL_LPB;
                }
                if (set_mode & SET_CAN_MODE_LISTENONLY) {
                    set_reg_ctrl |= FLEXCAN_CTRL_LOM;
                }
                if (set_mode & SET_CAN_MODE_3_SAMPLES) {
                    set_reg_ctrl |= FLEXCAN_CTRL_SMP;
                }

                if(f_cdev->reg_ctrl_bittiming != set_reg_ctrl) {

                    f_cdev->reg_ctrl_bittiming = set_reg_ctrl;

                    up (&f_cdev->sem);   // возможно с семафором тут лишнее
                    if (down_interruptible(&f_cdev->sem)) {
                        return -ERESTARTSYS;
                    }
                    flexcan_chip_stop(dev_num);
                    if(flexcan_chip_start(dev_num)) {
                        dev_dbg(&f_cdev->dev, "error when try to start the module\n");
                    }
                    up (&f_cdev->sem);
                }
            }
            // тут вызвать функцию которая задаст новые настройки для конкретного can интерфейса
            // возможно нужно самому написать эту функцию, переписав flexcan_set_bittiming() или не нужно, еще не проверял.
            break;
        case FLEXCAN_IOCTL_WR_RD:
            return -EINVAL; // будет не поддерживаемая инструкция пока не придумаю зачем она надо
            break;
        default:
            return -EINVAL;
            break;
    }
    return 0;
}


static struct file_operations flexcan_c_fops =
{
    .owner = THIS_MODULE,
    .llseek = flexcan_c_file_llseek,
    .read = flexcan_c_file_read,
    .write = flexcan_c_file_write,
    .poll = flexcan_c_file_poll,
    .open = flexcan_c_file_open,
    .release = flexcan_c_file_release,
    #if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
        .ioctl = flexcan_c_file_ioctl,
    #else
        .unlocked_ioctl = flexcan_c_file_ioctl,
    #endif
    .fasync = flexcan_c_file_fasync,
};


#ifdef USE_FLEXCAN_PROC
#define NAME_DIR "flexcan"
#define LEN_MSG (3072)
#define MAX_DIGITS (10)

static char *get_rw_buf(const u8 dev_num, int *length)
{
    struct flexcan_stats *stats = f_chrdev[dev_num]->stats;
    static char buf_msg[LEN_MSG + 1];
    char *buf_ptr = buf_msg;
    int buf_length = 0;

    memset(buf_msg, 0, (LEN_MSG + 1));

    snprintf(buf_ptr, (30 + 1), "Data communication statistic:\n");
    buf_ptr += 30;
    buf_length += 30;
    snprintf(buf_ptr, (22 + MAX_DIGITS + 1), "\tReceived frames   = %d\n", (int) stats->rx_frames); // количество принятых фреймов
    buf_ptr += (22 + MAX_DIGITS);
    buf_length += (22 + MAX_DIGITS);
    snprintf(buf_ptr, (22 + MAX_DIGITS + 1), "\tTransmited frames = %d\n", (int) stats->tx_frames); // количество отправленных фреймов
    buf_ptr += (22 + MAX_DIGITS);
    buf_length += (22 + MAX_DIGITS);
    snprintf(buf_ptr, (22 + MAX_DIGITS + 1), "\tReceived bytes    = %d\n", (int) stats->rx_bytes);  // количество принятых байт данных
    buf_ptr += (22 + MAX_DIGITS);
    buf_length += (22 + MAX_DIGITS);
    snprintf(buf_ptr, (22 + MAX_DIGITS + 1), "\tTransmited bytes  = %d\n", (int) stats->tx_bytes);  // количество отправленных байт данных
    buf_ptr += (22 + MAX_DIGITS);
    buf_length += (22 + MAX_DIGITS);

    snprintf(buf_ptr, (23 + 1), "\nInterrupts statistic:\n");
    buf_ptr += 23;
    buf_length += 23;
    snprintf(buf_ptr, (21 + MAX_DIGITS + 1), "\tWakeUp           = %d\n", (int) stats->int_wak);    // количество прерываний пробуждения
    buf_ptr += (21 + MAX_DIGITS);
    buf_length += (21 + MAX_DIGITS);
    snprintf(buf_ptr, (21 + MAX_DIGITS + 1), "\tState change     = %d\n", (int) stats->int_state);  // количество прерываний проверки состояния
    buf_ptr += (21 + MAX_DIGITS);
    buf_length += (21 + MAX_DIGITS);
    snprintf(buf_ptr, (21 + MAX_DIGITS + 1), "\tRead frame       = %d\n", (int) stats->int_rx_frame);   // количество прерываний чтения фреймов
    buf_ptr += (21 + MAX_DIGITS);
    buf_length += (21 + MAX_DIGITS);
    snprintf(buf_ptr, (21 + MAX_DIGITS + 1), "\tTransmit frame   = %d\n", (int) stats->int_tx_frame);   // количество прерываний отправки фреймов
    buf_ptr += (21 + MAX_DIGITS);
    buf_length += (21 + MAX_DIGITS);
    snprintf(buf_ptr, (21 + MAX_DIGITS + 1), "\tTotal interrupts = %d\n", (int) stats->int_num);    // количество прерываний
    buf_ptr += (21 + MAX_DIGITS);
    buf_length += (21 + MAX_DIGITS);

    snprintf(buf_ptr, (19 + 1), "\nErrors statistic:\n");
    buf_ptr += 19;
    buf_length += 19;
    snprintf(buf_ptr, (13 + MAX_DIGITS + 1), "\tTransmit = %d\n", (int) stats->err_tx); // количество ошибок отправки фреймов
    buf_ptr += (13 + MAX_DIGITS);
    buf_length += (13 + MAX_DIGITS);
    snprintf(buf_ptr, (13 + MAX_DIGITS + 1), "\tReceive  = %d\n", (int) stats->err_rx); // количество ошибок принятия фреймов
    buf_ptr += (13 + MAX_DIGITS);
    buf_length += (13 + MAX_DIGITS);
    snprintf(buf_ptr, (13 + MAX_DIGITS + 1), "\tOverflow = %d\n", (int) stats->err_over);   // количество переполнений буффера
    buf_ptr += (13 + MAX_DIGITS);
    buf_length += (13 + MAX_DIGITS);
    snprintf(buf_ptr, (13 + MAX_DIGITS + 1), "\tWarning  = %d\n", (int) stats->err_warn);   // количество заполнений буффера
    buf_ptr += (13 + MAX_DIGITS);
    buf_length += (13 + MAX_DIGITS);
    snprintf(buf_ptr, (13 + MAX_DIGITS + 1), "\tFrame    = %d\n", (int) stats->err_frame);  // количество ошибочных фреймов
    buf_ptr += (13 + MAX_DIGITS);
    buf_length += (13 + MAX_DIGITS);
    snprintf(buf_ptr, (13 + MAX_DIGITS + 1), "\tDroped   = %d\n", (int) stats->err_drop);   // количество потерянных при чтении фреймов
    buf_ptr += (13 + MAX_DIGITS);
    buf_length += (13 + MAX_DIGITS);
    snprintf(buf_ptr, (13 + MAX_DIGITS + 1), "\tLength   = %d\n", (int) stats->err_length); // количество ошибок длины фреймов
    buf_ptr += (13 + MAX_DIGITS);
    buf_length += (13 + MAX_DIGITS);
    snprintf(buf_ptr, (13 + MAX_DIGITS + 1), "\tFIFO     = %d\n", (int) stats->err_fifo);   // количество ошибок fifo буффера
    buf_ptr += (13 + MAX_DIGITS);
    buf_length += (13 + MAX_DIGITS);

    snprintf(buf_ptr, (18 + 1), "\nRegisters state:\n");
    buf_ptr += 18;
    buf_length += 18;
    snprintf(buf_ptr, (11 + MAX_DIGITS + 1), "\tESR  = %#08x\n", (int) stats->reg_esr); // состояние регистра esr
    buf_ptr += (11 + MAX_DIGITS);
    buf_length += (11 + MAX_DIGITS);
    snprintf(buf_ptr, (11 + MAX_DIGITS + 1), "\tMCR  = %#08x\n", (int) stats->reg_mcr); // состояние регистра mcr
    buf_ptr += (11 + MAX_DIGITS);
    buf_length += (11 + MAX_DIGITS);
    snprintf(buf_ptr, (11 + MAX_DIGITS + 1), "\tCTRL = %#08x\n", (int) stats->reg_ctrl);    // состояние регистра ctrl
    buf_ptr += (11 + MAX_DIGITS);
    buf_length += (11 + MAX_DIGITS);

    snprintf(buf_ptr, (13 + MAX_DIGITS + 1), "\nFrequency = %d\n", (int) stats->freq);  // частота модуля
    buf_ptr += (13 + MAX_DIGITS);
    buf_length += (13 + MAX_DIGITS);

    *length = buf_length;

    return buf_msg;
}


static ssize_t flexcan_proc_read(struct file *filp, char *buf, size_t count, loff_t *ppos) // чтение из /proc/*my_dir*/*my_node* :
{
    struct inode *inode = filp->f_path.dentry->d_inode;
    const u8 dev_num = iminor(inode);
    struct flexcan_c_device *f_dev = f_cdev[dev_num];
    int buf_length = 0, ret = 0;

    char *buf_msg = get_rw_buf(dev_num, &buf_length);

    if(*ppos >= buf_length)     {
        dev_dbg(&f_dev->dev, "%s.%d: %s eof\n", f_drv->name, dev_num, __func__);
        return 0;
    }

    ret = copy_to_user((void*)buf, buf_msg + *ppos, buf_length);
    *ppos += (buf_length - ret);

    return (buf_length - ret);
}


static const struct file_operations flexcan_proc_fops = {
   .owner = THIS_MODULE,
   .read  = flexcan_proc_read,
};


static int flexcan_proc_dir_init(const u8 dev_num)
{
    struct flexcan_c_device *f_dev = f_cdev[dev_num];
    int ret;

    f_drv->dev_proc_dir = create_proc_entry(NAME_DIR, S_IFDIR | S_IRWXUGO, NULL);
    if(NULL == f_drv->dev_proc_dir) {
        ret = -ENOENT;
        dev_dbg(&f_dev->dev, "flexcan_proc_dir_init can't create directory /proc/%s\n", NAME_DIR);
        goto err_dir;
    }

    f_drv->dev_proc_dir->uid = 0;
    f_drv->dev_proc_dir->gid = 0;

    return 0;

 err_dir:
    return ret;
}


static int flexcan_proc_file_init(const u8 dev_num)
{
    struct flexcan_c_device *f_dev = f_cdev[dev_num];
    int ret;

    f_dev->dev_proc_file = create_proc_entry(f_dev->name, (S_IFREG | S_IRUGO), f_drv->dev_proc_dir);
    if(NULL == f_dev->dev_proc_file) {
        ret = -ENOENT;
        dev_dbg(&f_dev->dev, "flexcan_proc_file_init can't create node /proc/%s/%s\n", NAME_DIR, f_dev->name);
        goto err_file;
    }

    f_dev->dev_proc_file->uid = 0;
    f_dev->dev_proc_file->gid = 0;
    f_dev->dev_proc_file->proc_fops = &flexcan_proc_fops;

    return 0;

 err_file:
    return ret;
}


static void flexcan_proc_file_exit(const u8 dev_num)
{
    remove_proc_entry(f_cdev[dev_num]->name, f_drv->dev_proc_dir);
}


static void flexcan_proc_dir_exit(void)
{
    remove_proc_entry(NAME_DIR, NULL);
}
#endif // USE_FLEXCAN_PROC
#endif // USE_STRIM_FLEXCAN


#ifndef USE_STRIM_FLEXCAN
static inline void flexcan_enter_stop_mode(struct flexcan_priv *priv)
{
	/* enable stop request */
	if (priv->devtype_data->features & FLEXCAN_HAS_V10_FEATURES)
		regmap_update_bits(priv->stm.gpr, priv->stm.req_gpr,
			1 << priv->stm.req_bit, 1 << priv->stm.req_bit);
}

static inline void flexcan_exit_stop_mode(struct flexcan_priv *priv)
{
	/* remove stop request */
	if (priv->devtype_data->features & FLEXCAN_HAS_V10_FEATURES)
		regmap_update_bits(priv->stm.gpr, priv->stm.req_gpr,
			1 << priv->stm.req_bit, 0);
}
#endif // USE_STRIM_FLEXCAN


/*
 * Swtich transceiver on or off
 */
static void flexcan_transceiver_switch(const struct flexcan_platform_data *pdata, int on)
{
    if (pdata && pdata->transceiver_switch) {
        pdata->transceiver_switch(on);
    }
}


static inline int flexcan_has_and_handle_berr(const struct flexcan_stats *stats, u32 reg_esr)
{
    return (stats->ctrlmode & CAN_CTRLMODE_BERR_REPORTING) && (reg_esr & FLEXCAN_ESR_ERR_BUS);
}


static inline void flexcan_chip_enable(void __iomem *base)
{
    struct flexcan_regs __iomem *regs = base;
    u32 reg;

    reg = flexcan_read(&regs->mcr);
    reg &= ~FLEXCAN_MCR_MDIS;
    flexcan_write(reg, &regs->mcr);

    udelay(10);
}


static inline void flexcan_chip_disable(void __iomem *base)
{
    struct flexcan_regs __iomem *regs = base;
    u32 reg;

    reg = flexcan_read(&regs->mcr);
    reg |= FLEXCAN_MCR_MDIS;
    flexcan_write(reg, &regs->mcr);
}


static int flexcan_get_berr_counter(u8 *dev_num, struct can_berr_counter *bec)
{
    struct flexcan_c_device *f_cdev = &f_chrdev[*dev_num];
    struct flexcan_regs __iomem *regs = f_cdev->base;
    u32 reg = flexcan_read(&regs->ecr);

    bec->txerr = (reg >> 0) & 0xff;
    bec->rxerr = (reg >> 8) & 0xff;

    return 0;
}


#ifdef USE_STRIM_FLEXCAN
static unsigned int flexcan_start_transmit(const u8 dev_num)
{
    struct can_frame cf;
    u32 can_id, ctrl, data;

    struct flexcan_c_device *f_cdev = &f_chrdev[dev_num];
    struct flexcan_regs __iomem *regs = f_cdev->base;
    struct flexcan_stats *stats = f_cdev->stats;

    /* На будущее проверка статуса буфера в данный момент */
    // u8 mb_code = ((flexcan_read(&regs->cantxfg[FLEXCAN_TX_BUF_ID].can_ctrl) >> 24) & 0x0F);

    if(flexcan_c_sbuf_pop(dev_num, &cf) != 0) {
        dev_dbg(&f_cdev->dev, "buffer free\n");
        return 0;
    }

    dev_dbg(&f_cdev->dev, "send messages\n");
    ctrl = FLEXCAN_MB_CNT_CODE(0xc) | (cf.can_dlc << 16);
    if (cf.can_id & CAN_EFF_FLAG) {
        can_id = cf.can_id & CAN_EFF_MASK;
        ctrl |= FLEXCAN_MB_CNT_IDE | FLEXCAN_MB_CNT_SRR;
    }
    else {
        can_id = (cf.can_id & CAN_SFF_MASK) << 18;
    }

    if (cf.can_id & CAN_RTR_FLAG) {
        ctrl |= FLEXCAN_MB_CNT_RTR;
    }

    if (cf.can_dlc > 0) {
        data = be32_to_cpup((__be32 *)&cf.data[0]);
        flexcan_write(data, &regs->cantxfg[FLEXCAN_TX_BUF_ID].data[0]);
    }
    if (cf.can_dlc > 3) {
        data = be32_to_cpup((__be32 *)&cf.data[4]);
        flexcan_write(data, &regs->cantxfg[FLEXCAN_TX_BUF_ID].data[1]);
    }

    flexcan_write(can_id, &regs->cantxfg[FLEXCAN_TX_BUF_ID].can_id);
    flexcan_write(ctrl, &regs->cantxfg[FLEXCAN_TX_BUF_ID].can_ctrl);
    /* tx_packets is incremented in flexcan_irq */
    stats->tx_bytes += cf.can_dlc;

    return 0;
}
#else
static int flexcan_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	const struct flexcan_priv *priv = netdev_priv(dev);
	struct flexcan_regs __iomem *regs = priv->base;
	struct can_frame *cf = (struct can_frame *)skb->data;
	u32 can_id;
	u32 ctrl = FLEXCAN_MB_CNT_CODE(0xc) | (cf->can_dlc << 16);

	if (can_dropped_invalid_skb(dev, skb))
		return NETDEV_TX_OK;

	netif_stop_queue(dev);

	if (cf->can_id & CAN_EFF_FLAG) {
		can_id = cf->can_id & CAN_EFF_MASK;
		ctrl |= FLEXCAN_MB_CNT_IDE | FLEXCAN_MB_CNT_SRR;
	} else {
		can_id = (cf->can_id & CAN_SFF_MASK) << 18;
	}

	if (cf->can_id & CAN_RTR_FLAG)
		ctrl |= FLEXCAN_MB_CNT_RTR;

	if (cf->can_dlc > 0) {
		u32 data = be32_to_cpup((__be32 *)&cf->data[0]);
		flexcan_write(data, &regs->cantxfg[FLEXCAN_TX_BUF_ID].data[0]);
	}
	if (cf->can_dlc > 3) {
		u32 data = be32_to_cpup((__be32 *)&cf->data[4]);
		flexcan_write(data, &regs->cantxfg[FLEXCAN_TX_BUF_ID].data[1]);
	}

	can_put_echo_skb(skb, dev, 0);

	flexcan_write(can_id, &regs->cantxfg[FLEXCAN_TX_BUF_ID].can_id);
	flexcan_write(ctrl, &regs->cantxfg[FLEXCAN_TX_BUF_ID].can_ctrl);

	if (priv->devtype_data->features & FLEXCAN_HAS_ERR005829) {
		flexcan_write(0x0, &regs->cantxfg[FLEXCAN_RESERVED_BUF_ID].can_ctrl);
		flexcan_write(0x0, &regs->cantxfg[FLEXCAN_RESERVED_BUF_ID].can_ctrl);
	}

	return NETDEV_TX_OK;
}
#endif


static void do_bus_err(struct can_frame *cf, u32 reg_esr, const u8 dev_num)
{
    struct flexcan_c_device *f_cdev = &f_chrdev[dev_num];
    struct flexcan_stats *stats = f_cdev->stats;
    int rx_errors = 0, tx_errors = 0;

    cf->can_id |= CAN_ERR_PROT | CAN_ERR_BUSERROR;
    if (reg_esr & FLEXCAN_ESR_BIT1_ERR) {
        dev_dbg(&f_cdev->dev, "BIT1_ERR irq\n");
        cf->data[2] |= CAN_ERR_PROT_BIT1;
        tx_errors = 1;
    }
    if (reg_esr & FLEXCAN_ESR_BIT0_ERR) {
        dev_dbg(&f_cdev->dev, "BIT0_ERR irq\n");
        cf->data[2] |= CAN_ERR_PROT_BIT0;
        tx_errors = 1;
    }
    if (reg_esr & FLEXCAN_ESR_ACK_ERR) {
        dev_dbg(&f_cdev->dev, "ACK_ERR irq\n");
        cf->can_id |= CAN_ERR_ACK;
        cf->data[3] |= CAN_ERR_PROT_LOC_ACK;
        tx_errors = 1;
    }
    if (reg_esr & FLEXCAN_ESR_CRC_ERR) {
        dev_dbg(&f_cdev->dev, "CRC_ERR irq\n");
        cf->data[2] |= CAN_ERR_PROT_BIT;
        cf->data[3] |= CAN_ERR_PROT_LOC_CRC_SEQ;
        rx_errors = 1;
    }
    if (reg_esr & FLEXCAN_ESR_FRM_ERR) {
        dev_dbg(&f_cdev->dev, "FRM_ERR irq\n");
        cf->data[2] |= CAN_ERR_PROT_FORM;
        rx_errors = 1;
    }
    if (reg_esr & FLEXCAN_ESR_STF_ERR) {
        dev_dbg(&f_cdev->dev, "STF_ERR irq\n");
        cf->data[2] |= CAN_ERR_PROT_STUFF;
        rx_errors = 1;
    }

    stats->dev_stats.bus_error++;
    if (rx_errors)  {
        stats->err_rx++;
    }
    if (tx_errors)  {
        stats->err_tx++;
    }
}


static void do_state(struct can_frame *cf, const u8 dev_num, enum can_state new_state)
{
    struct flexcan_c_device *f_cdev = &f_chrdev[dev_num];
    struct flexcan_stats *stats = f_cdev->stats;
    struct can_berr_counter bec;

    dev_dbg(&f_cdev->dev, "get berr counter\n");
    flexcan_get_berr_counter(&dev_num, &bec);

    switch (stats->state) {
        case CAN_STATE_ERROR_ACTIVE:
            /*
             * from: ERROR_ACTIVE
             * to  : ERROR_WARNING, ERROR_PASSIVE, BUS_OFF
             * =>  : there was a warning int
             */
            if (new_state >= CAN_STATE_ERROR_WARNING &&
                new_state <= CAN_STATE_BUS_OFF) {
                dev_dbg(&f_cdev->dev, "Error Warning IRQ\n");
                stats->dev_stats.error_warning++;

                cf->can_id |= CAN_ERR_CRTL;
                cf->data[1] = (bec.txerr > bec.rxerr) ? CAN_ERR_CRTL_TX_WARNING : CAN_ERR_CRTL_RX_WARNING;
            }
        case CAN_STATE_ERROR_WARNING:   /* fallthrough */
            /*
             * from: ERROR_ACTIVE, ERROR_WARNING
             * to  : ERROR_PASSIVE, BUS_OFF
             * =>  : error passive int
             */
            if (new_state >= CAN_STATE_ERROR_PASSIVE &&
                new_state <= CAN_STATE_BUS_OFF) {
                dev_dbg(&f_cdev->dev, "Error Passive IRQ\n");
                stats->dev_stats.error_passive++;

                cf->can_id |= CAN_ERR_CRTL;
                cf->data[1] = (bec.txerr > bec.rxerr) ? CAN_ERR_CRTL_TX_PASSIVE : CAN_ERR_CRTL_RX_PASSIVE;
            }
            break;
        case CAN_STATE_BUS_OFF:
            dev_dbg(&f_cdev->dev, "BUG! hardware recovered automatically from BUS_OFF\n");
            break;
        default:
            break;
    }

    /* process state changes depending on the new state */
    switch (new_state) {
        case CAN_STATE_ERROR_ACTIVE:
            dev_dbg(&f_cdev->dev, "Error Active\n");
            cf->can_id |= CAN_ERR_PROT;
            cf->data[2] = CAN_ERR_PROT_ACTIVE;
            break;
        case CAN_STATE_BUS_OFF:
            cf->can_id |= CAN_ERR_BUSOFF;
            // can_bus_off(dev);
            stats->dev_stats.bus_off++;
            break;
        default:
            break;
    }
}


static irqreturn_t flexcan_irq(int irq, void *dev_id)
{
    const u8 dev_num = (irq - FLEXCAN_IRQ_BASE);
    struct flexcan_c_device *f_cdev = &f_chrdev[dev_num];
    struct flexcan_stats *stats = f_cdev->stats;
    struct flexcan_regs __iomem *regs = f_cdev->base;
    struct flexcan_mb __iomem *mb = &regs->cantxfg[0];
    struct timeval time;
    u32 reg_iflag1, reg_esr;
    u32 read_frames = 0;

    // flexcan_transceiver_switch(f_cdev->pdata, 1);
    reg_iflag1 = flexcan_read(&regs->iflag1);
    reg_esr = flexcan_read(&regs->esr);
    stats->int_num++;   /* Считаем прерывания */

    if(reg_esr & FLEXCAN_ESR_WAK_INT) {
//        if(f_drv->devtype_data->features & FLEXCAN_HAS_V10_FEATURES) {
//            mxc_iomux_set_gpr_register(13, 28, 1, 0);
//        }
        flexcan_write(FLEXCAN_ESR_WAK_INT, &regs->esr);
    }

    if (reg_iflag1 & FLEXCAN_IFLAG_DEFAULT) {
        if(reg_iflag1 & FLEXCAN_IFLAG_RX_FIFO_OVERFLOW) {
            stats->err_over++;
            stats->err_drop++;
        }
        else if(reg_iflag1 & FLEXCAN_IFLAG_RX_FIFO_WARN) {
            stats->err_warn++;
        }
        stats->int_rx_frame++;

        gpio_set_value(f_cdev->gpio_led, 1);

        read_frames = 0;
        while((read_frames < 10) && (reg_iflag1 & FLEXCAN_IFLAG_RX_FIFO_AVAILABLE)) {
            do_gettimeofday(&time);
            stats->rx_bytes += ((mb->can_ctrl >> 16) & 0xf);
            if(flexcan_c_rbuf_push(dev_num, mb, &time)) { /* буффер заполнен */
                stats->err_drop++;
                stats->err_fifo++;
            }
            else {  /* все хорошо */
                stats->rx_frames++;
            }

            if(reg_iflag1 & FLEXCAN_IFLAG_RX_FIFO_OVERFLOW) {
                flexcan_write((FLEXCAN_IFLAG_DEFAULT), &regs->iflag1);
            }
            else if(reg_iflag1 & FLEXCAN_IFLAG_RX_FIFO_WARN) {
                flexcan_write((FLEXCAN_IFLAG_RX_FIFO_WARN | FLEXCAN_IFLAG_RX_FIFO_AVAILABLE), &regs->iflag1);
            }
            else if(reg_iflag1 & FLEXCAN_IFLAG_RX_FIFO_AVAILABLE) {
                flexcan_write(FLEXCAN_IFLAG_RX_FIFO_AVAILABLE, &regs->iflag1);
            }

            reg_iflag1 = flexcan_read(&regs->iflag1);
            flexcan_read(&regs->timer);
            read_frames++;
        }
    }

    if(f_cdev->autoset_bittiming_flags & FLEXCAN_AUTOSET_WORK) {
        if(reg_esr & FLEXCAN_ESR_BIT0_ERR) {
            f_cdev->autoset_bittiming_flags |= FLEXCAN_AUTOSET_ERR;
        }
        else {
            f_cdev->autoset_bittiming_flags |= FLEXCAN_AUTOSET_OK;
        }

        flexcan_autoset_baudrate(dev_num);
    }

    if((reg_esr & FLEXCAN_ESR_ERR_STATE) || (flexcan_has_and_handle_berr(stats, reg_esr))) {
        stats->int_state++;

        stats->reg_esr = reg_esr & FLEXCAN_ESR_ERR_BUS;
        flexcan_write(reg_esr, &regs->esr);
        flexcan_write(f_cdev->reg_ctrl_default, &regs->ctrl);
    }

    if(reg_iflag1 & (1 << FLEXCAN_TX_BUF_ID)) { /* transmission complete interrupt */
        stats->int_tx_frame++;  /* tx_bytes is incremented in flexcan_start_xmit */
        stats->tx_frames++;
        flexcan_write((1 << FLEXCAN_TX_BUF_ID), &regs->iflag1);
    }

    if(read_frames != 0) {  /* Будим читателей */
        wake_up_interruptible(&f_cdev->inq); /* blocked in read() and select() */
        if(f_cdev->async_queue) {    /* and signal asynchronous readers, explained late in chapter 5 */
            kill_fasync(&(f_cdev->async_queue), SIGIO, POLL_IN);
        }
    }
    gpio_set_value(f_cdev->gpio_led, 0);

    return IRQ_HANDLED;
}


static void flexcan_set_bittiming(const u8 dev_num, const u32 reg_ctrl)
{
    u32 reg;

    struct flexcan_c_device *f_cdev = &f_chrdev[dev_num];
    struct flexcan_stats *stats = f_cdev->stats;
    struct flexcan_regs __iomem *regs = f_cdev->base;

    reg = flexcan_read(&regs->ctrl);
    reg &= ~(FLEXCAN_BTRT_MASK |
         FLEXCAN_CTRL_LPB |
         FLEXCAN_CTRL_SMP |
         FLEXCAN_CTRL_LOM);
    reg |= reg_ctrl;

    dev_dbg(&f_cdev->dev, "writing ctrl = %#08x\n", reg);
    flexcan_write(reg, &regs->ctrl);
    stats->reg_mcr = flexcan_read(&regs->mcr);
    stats->reg_ctrl = flexcan_read(&regs->ctrl);

    /* print chip status */
    dev_dbg(&f_cdev->dev, "mcr = %#08x ctrl = %#08x\n", flexcan_read(&regs->mcr), flexcan_read(&regs->ctrl));
}


#ifdef USE_STRIM_FLEXCAN
static int flexcan_chip_start(const u8 dev_num)
{
    struct flexcan_c_device *f_cdev = &f_chrdev[dev_num];
    struct flexcan_stats *stats = f_cdev->stats;
    struct flexcan_regs __iomem *regs = f_cdev->base;
    u32 reg_mcr, reg_ctrl;
    int i, err;

    flexcan_chip_enable(f_cdev->base); /* enable module */

    flexcan_write(FLEXCAN_MCR_SOFTRST, &regs->mcr);    /* soft reset */
    udelay(10);

    reg_mcr = flexcan_read(&regs->mcr);
    if (reg_mcr & FLEXCAN_MCR_SOFTRST) {
        dev_dbg(&f_cdev->dev, "failed to softreset can module, mcr = %#08x\n", reg_mcr);
        err = -ENODEV;
        goto out;
    }

    flexcan_set_bittiming(dev_num, f_cdev->reg_ctrl_bittiming);

    /* MCR
     * enable freeze | enable fifo | halt now  | only supervisor access  |
     * enable warning int |  choose format C  | enable self wakeup
     */
    reg_mcr = flexcan_read(&regs->mcr);
    reg_mcr |= FLEXCAN_MCR_FRZ | FLEXCAN_MCR_FEN | FLEXCAN_MCR_HALT | FLEXCAN_MCR_SUPV |
        FLEXCAN_MCR_WRN_EN | FLEXCAN_MCR_IDAM_C | FLEXCAN_MCR_WAK_MSK | FLEXCAN_MCR_SLF_WAK;

    dev_dbg(&f_cdev->dev, "writing mcr = %#08x\n", reg_mcr);
    flexcan_write(reg_mcr, &regs->mcr);
    stats->reg_mcr = flexcan_read(&regs->mcr);
    /* CTRL
     * disable timer sync feature | disable auto busoff recovery | transmit lowest buffer first
     * enable tx and rx warning interrupt | enable bus off interrupt (== FLEXCAN_CTRL_ERR_STATE)
     * _note_: we enable the "error interrupt" (FLEXCAN_CTRL_ERR_MSK), too. Otherwise we don't get any
     * Otherwise we don't get any warning or bus passive interrupts.
     */
    reg_ctrl = flexcan_read(&regs->ctrl);
    f_cdev->reg_ctrl_default = reg_ctrl;    // Save for latest use
    reg_ctrl &= ~FLEXCAN_CTRL_TSYN;
    // reg_ctrl |= /*FLEXCAN_CTRL_BOFF_REC | */ FLEXCAN_CTRL_LBUF |/*| FLEXCAN_CTRL_ERR_STATE | */FLEXCAN_CTRL_ERR_MSK ;
    reg_ctrl |= FLEXCAN_CTRL_BOFF_REC | FLEXCAN_CTRL_LBUF;

    f_cdev->reg_ctrl_bittiming = reg_ctrl; /* save for later use */

    dev_dbg(&f_cdev->dev, "writing ctrl = %#08x\n", reg_ctrl);
    flexcan_write(reg_ctrl, &regs->ctrl);
    stats->reg_ctrl = flexcan_read(&regs->ctrl);

    for (i = 0; i < ARRAY_SIZE(regs->cantxfg); i++) {
        flexcan_write(0, &regs->cantxfg[i].can_ctrl);
        flexcan_write(0, &regs->cantxfg[i].can_id);
        flexcan_write(0, &regs->cantxfg[i].data[0]);
        flexcan_write(0, &regs->cantxfg[i].data[1]);
        /* put MB into rx queue */
        flexcan_write(FLEXCAN_MB_CNT_CODE(0x4), &regs->cantxfg[i].can_ctrl);
    }
    /* acceptance mask/acceptance code (accept everything) */
    flexcan_write(0x0, &regs->rxgmask);
    flexcan_write(0x0, &regs->rx14mask);
    flexcan_write(0x0, &regs->rx15mask);

    if(f_drv->devtype_data->features & FLEXCAN_HAS_V10_FEATURES) {
        flexcan_write(0x0, &regs->rxfgmask);
    }
    // flexcan_transceiver_switch(priv, 1);                     //заменить priv  на pdata из структуры вызывающего устройства

    /* synchronize with the can bus */
    reg_mcr = flexcan_read(&regs->mcr);
    reg_mcr &= ~FLEXCAN_MCR_HALT;
    flexcan_write(reg_mcr, &regs->mcr);
    stats->reg_mcr = flexcan_read(&regs->mcr);
    stats->state = CAN_STATE_ERROR_ACTIVE;

    flexcan_write(FLEXCAN_IFLAG_DEFAULT, &regs->imask1);   /* enable FIFO interrupts */

    /* print chip status */
    dev_dbg(&f_cdev->dev, "reading mcr = %#08x ctrl = %#08x\n", flexcan_read(&regs->mcr), flexcan_read(&regs->ctrl));
#ifdef USE_FLEXCAN_PROC
    flexcan_proc_file_init(dev_num);
#endif

    return 0;

 out:
    dev_dbg(&f_cdev->dev, "chip disable\n");
    flexcan_chip_disable(f_cdev->base);
    return err;
}
#else
/*
 * flexcan_chip_start
 *
 * this functions is entered with clocks enabled
 *
 */
static int flexcan_chip_start(struct net_device *dev)
{
	struct flexcan_priv *priv = netdev_priv(dev);
	struct flexcan_regs __iomem *regs = priv->base;
	int err;
	u32 reg_mcr, reg_ctrl;

	/* enable module */
	flexcan_chip_enable(priv);

	/* soft reset */
	flexcan_write(FLEXCAN_MCR_SOFTRST, &regs->mcr);
	udelay(10);

	reg_mcr = flexcan_read(&regs->mcr);
	if (reg_mcr & FLEXCAN_MCR_SOFTRST) {
		netdev_err(dev, "Failed to softreset can module (mcr=0x%08x)\n",
			   reg_mcr);
		err = -ENODEV;
		goto out;
	}

	flexcan_set_bittiming(dev);

	/*
	 * MCR
	 *
	 * enable freeze
	 * enable fifo
	 * halt now
	 * only supervisor access
	 * enable warning int
	 * choose format C
	 * disable local echo
	 * enable self wakeup
	 */
	reg_mcr = flexcan_read(&regs->mcr);
	reg_mcr &= ~FLEXCAN_MCR_MAXMB(0xff);
	reg_mcr |= FLEXCAN_MCR_FRZ | FLEXCAN_MCR_FEN | FLEXCAN_MCR_HALT |
		FLEXCAN_MCR_SUPV | FLEXCAN_MCR_WRN_EN |
		FLEXCAN_MCR_IDAM_C | FLEXCAN_MCR_SRX_DIS |
		FLEXCAN_MCR_WAK_MSK | FLEXCAN_MCR_SLF_WAK |
		FLEXCAN_MCR_MAXMB(FLEXCAN_TX_BUF_ID);
	netdev_dbg(dev, "%s: writing mcr=0x%08x", __func__, reg_mcr);
	flexcan_write(reg_mcr, &regs->mcr);

	/*
	 * CTRL
	 *
	 * disable timer sync feature
	 *
	 * disable auto busoff recovery
	 * transmit lowest buffer first
	 *
	 * enable tx and rx warning interrupt
	 * enable bus off interrupt
	 * (== FLEXCAN_CTRL_ERR_STATE)
	 */
	reg_ctrl = flexcan_read(&regs->ctrl);
	reg_ctrl &= ~FLEXCAN_CTRL_TSYN;
	reg_ctrl |= FLEXCAN_CTRL_BOFF_REC | FLEXCAN_CTRL_LBUF |
		FLEXCAN_CTRL_ERR_STATE;
	/*
	 * enable the "error interrupt" (FLEXCAN_CTRL_ERR_MSK),
	 * on most Flexcan cores, too. Otherwise we don't get
	 * any error warning or passive interrupts.
	 */
	if (priv->devtype_data->features & FLEXCAN_HAS_BROKEN_ERR_STATE ||
	    priv->can.ctrlmode & CAN_CTRLMODE_BERR_REPORTING)
		reg_ctrl |= FLEXCAN_CTRL_ERR_MSK;

	/* save for later use */
	priv->reg_ctrl_default = reg_ctrl;
	netdev_dbg(dev, "%s: writing ctrl=0x%08x", __func__, reg_ctrl);
	flexcan_write(reg_ctrl, &regs->ctrl);

	/* Abort any pending TX, mark Mailbox as INACTIVE */
	flexcan_write(FLEXCAN_MB_CNT_CODE(0x4),
		      &regs->cantxfg[FLEXCAN_TX_BUF_ID].can_ctrl);

	/* acceptance mask/acceptance code (accept everything) */
	flexcan_write(0x0, &regs->rxgmask);
	flexcan_write(0x0, &regs->rx14mask);
	flexcan_write(0x0, &regs->rx15mask);

	if (priv->devtype_data->features & FLEXCAN_HAS_V10_FEATURES)
		flexcan_write(0x0, &regs->rxfgmask);

	flexcan_transceiver_switch(priv, 1);

	/* synchronize with the can bus */
	reg_mcr = flexcan_read(&regs->mcr);
	reg_mcr &= ~FLEXCAN_MCR_HALT;
	flexcan_write(reg_mcr, &regs->mcr);

	priv->can.state = CAN_STATE_ERROR_ACTIVE;

	/* enable FIFO interrupts */
	flexcan_write(FLEXCAN_IFLAG_DEFAULT, &regs->imask1);

	/* print chip status */
	netdev_dbg(dev, "%s: reading mcr=0x%08x ctrl=0x%08x\n", __func__,
		   flexcan_read(&regs->mcr), flexcan_read(&regs->ctrl));

	return 0;

 out:
	flexcan_chip_disable(priv);
	return err;
}
#endif


static void flexcan_chip_stop(const u8 dev_num)
{
    struct flexcan_c_device *f_cdev = &f_chrdev[dev_num];
    struct flexcan_stats *stats = f_cdev->stats;
    struct flexcan_regs __iomem *regs = f_cdev->base;
    u32 reg;

#ifdef USE_FLEXCAN_PROC
    flexcan_proc_file_exit(dev_num);
#endif

    flexcan_write(0, &regs->imask1);   /* Disable all interrupts */

    /* Disable + halt module */
    reg = flexcan_read(&regs->mcr);
    reg |= FLEXCAN_MCR_MDIS | FLEXCAN_MCR_HALT;
    flexcan_write(reg, &regs->mcr);
    stats->reg_mcr = flexcan_read(&regs->mcr);

    // flexcan_transceiver_switch(priv, 0); //заменить priv  на pdata из структуры вызывающего устройства
    stats->state = CAN_STATE_STOPPED;
}


static int flexcan_open(const u8 dev_num)
{
    int err;
    struct flexcan_c_device *f_cdev = &f_chrdev[dev_num];
    unsigned int irq = f_cdev->irq_num;

    clk_prepare_enable(f_cdev->clk_ipg);
    clk_prepare_enable(f_cdev->clk_per);

    err = request_irq(irq, flexcan_irq, IRQF_SHARED, f_cdev->name, f_cdev->pdata);
    if(err) {
        dev_dbg(&f_cdev->dev, "request irq %d failed with err = %d\n", irq, err);
        goto out;
    }

    flexcan_autoset_baudrate(dev_num);

//    err = flexcan_chip_start(dev_num);  /* start chip and queuing */
//    if(err) {
//        goto out;
//    }

    // TODO посмотреть что это за красота такая и, может, попробовать применить
//    can_led_event(dev, CAN_LED_EVENT_OPEN);

    return 0;

 out:
    clk_disable_unprepare(f_cdev->clk_per);
    clk_disable_unprepare(f_cdev->clk_ipg);

    return err;
}


static int flexcan_close(const u8 dev_num)
{
    struct flexcan_c_device *f_cdev = &f_chrdev[dev_num];
    unsigned int irq = f_cdev->irq_num;

    flexcan_chip_stop(dev_num);

    free_irq(irq, f_cdev->pdata);

    clk_disable_unprepare(f_cdev->clk_per);
    clk_disable_unprepare(f_cdev->clk_ipg);

//    can_led_event(dev, CAN_LED_EVENT_STOP);

    return 0;
}


static int flexcan_set_mode(__u8 *dev_num, enum can_mode mode)
{
    int err = 0;

    switch (mode) {
    case CAN_MODE_START:
        err = flexcan_chip_start(*dev_num);
        if(err) {
            return err;
        }
        break;
    default:
        return -EOPNOTSUPP;
    }

    return 0;
}


#ifndef USE_STRIM_FLEXCAN
static int register_flexcandev(struct net_device *dev)
{
	struct flexcan_priv *priv = netdev_priv(dev);
	struct flexcan_regs __iomem *regs = priv->base;
	u32 reg, err;

	clk_prepare_enable(priv->clk_ipg);
	clk_prepare_enable(priv->clk_per);

	/* select "bus clock", chip must be disabled */
	flexcan_chip_disable(priv);
	reg = flexcan_read(&regs->ctrl);
	reg |= FLEXCAN_CTRL_CLK_SRC;
	flexcan_write(reg, &regs->ctrl);

	flexcan_chip_enable(priv);

	/* set freeze, halt and activate FIFO, restrict register access */
	reg = flexcan_read(&regs->mcr);
	reg |= FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT |
		FLEXCAN_MCR_FEN | FLEXCAN_MCR_SUPV;
	flexcan_write(reg, &regs->mcr);

	/*
	 * Currently we only support newer versions of this core
	 * featuring a RX FIFO. Older cores found on some Coldfire
	 * derivates are not yet supported.
	 */
	reg = flexcan_read(&regs->mcr);
	if (!(reg & FLEXCAN_MCR_FEN)) {
		netdev_err(dev, "Could not enable RX FIFO, unsupported core\n");
		err = -ENODEV;
		goto out;
	}

	err = register_candev(dev);

 out:
	/* disable core and turn off clocks */
	flexcan_chip_disable(priv);
	clk_disable_unprepare(priv->clk_per);
	clk_disable_unprepare(priv->clk_ipg);

	return err;
}

static void unregister_flexcandev(struct net_device *dev)
{
	unregister_candev(dev);
}
#endif


#ifndef USE_STRIM_FLEXCAN
static int flexcan_of_parse_stop_mode(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct device_node *np = pdev->dev.of_node;
	struct device_node *node;
	struct flexcan_priv *priv;
	phandle phandle;
	u32 out_val[5];
	int ret;

	if (!np)
		return -EINVAL;
	/*
	 * stop mode property format is:
	 * <&gpr req_gpr req_bit ack_gpr ack_bit>.
	 */
	ret = of_property_read_u32_array(np, "stop-mode", out_val, 5);
	if (ret) {
		dev_dbg(&pdev->dev, "no stop-mode property\n");
		return ret;
	}
	phandle = *out_val;

	node = of_find_node_by_phandle(phandle);
	if (!node) {
		dev_dbg(&pdev->dev, "could not find gpr node by phandle\n");
		return PTR_ERR(node);
	}

	priv = netdev_priv(dev);
	priv->stm.gpr = syscon_node_to_regmap(node);
	if (IS_ERR_OR_NULL(priv->stm.gpr)) {
		dev_dbg(&pdev->dev, "could not find gpr regmap\n");
		return PTR_ERR(priv->stm.gpr);
	}

	of_node_put(node);

	priv->stm.req_gpr = out_val[1];
	priv->stm.req_bit = out_val[2];
	priv->stm.ack_gpr = out_val[3];
	priv->stm.ack_bit = out_val[4];

	dev_dbg(&pdev->dev, "gpr %s req_gpr 0x%x req_bit %u ack_gpr 0x%x ack_bit %u\n",
			node->full_name, priv->stm.req_gpr,
			priv->stm.req_bit, priv->stm.ack_gpr,
			priv->stm.ack_bit);
	return 0;
}
#endif


static const struct of_device_id flexcan_of_match[] = {
	{ .compatible = "fsl,imx6q-flexcan", .data = &fsl_imx6q_devtype_data, },
	{ .compatible = "fsl,imx28-flexcan", .data = &fsl_imx28_devtype_data, },
	{ .compatible = "fsl,p1010-flexcan", .data = &fsl_p1010_devtype_data, },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, flexcan_of_match);

static const struct platform_device_id flexcan_id_table[] = {
	{ .name = "flexcan", .driver_data = (kernel_ulong_t)&fsl_p1010_devtype_data, },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(platform, flexcan_id_table);


/* Размещает в памяти буферы сохранения входящих данных для size устройств */
static inline void *flexcan_c_allocate(int size, int capacity)
{
    if(!size || !capacity) {
        return ZERO_SIZE_PTR;
    }

    return kmalloc(size * capacity, GFP_KERNEL);
}


static int flexcan_c_alloc_base(void)
{
    int err;

    f_drv = flexcan_c_allocate(sizeof(struct flexcan_c_driver), 1);
    if (IS_ERR_OR_NULL(f_drv)) {
        err = -ENOMEM;
        goto failed_kmalloc_driver;
    }
    memset(f_drv, 0, sizeof(struct flexcan_c_driver));

    f_chrdev = flexcan_c_allocate(sizeof(struct flexcan_c_device), FLEXCAN_DEV_COUNT);
    if (IS_ERR_OR_NULL(f_chrdev)) {
        err = -ENOMEM;
        goto failed_kmalloc_cdev;
    }
    memset(f_chrdev, 0, sizeof(struct flexcan_c_device) * FLEXCAN_DEV_COUNT);

    f_brecv = flexcan_c_allocate(sizeof(struct flexcan_c_brecv), FLEXCAN_DEV_COUNT);
    if (IS_ERR_OR_NULL(f_brecv)) {
        err = -ENOMEM;
        goto failed_kmalloc_fbrecv;
    }
    memset(f_brecv, 0, sizeof(struct flexcan_c_brecv) * FLEXCAN_DEV_COUNT);

    f_bsend = flexcan_c_allocate(sizeof(struct flexcan_c_bsend), FLEXCAN_DEV_COUNT);
    if (IS_ERR_OR_NULL(f_bsend)) {
        err = -ENOMEM;
        goto failed_kmalloc_fbsend;
    }
    memset(f_bsend, 0, sizeof(struct flexcan_c_bsend) * FLEXCAN_DEV_COUNT);

    f_drv->is_init = 0;

    return 0;
 failed_kmalloc_fbsend:
     kfree(f_brecv);
 failed_kmalloc_fbrecv:
     kfree(f_chrdev);
 failed_kmalloc_cdev:
     kfree(f_drv);
 failed_kmalloc_driver:
    return err;
}


static inline void flexcan_c_free_base(void)
{
    kfree(f_bsend);
    kfree(f_brecv);
    kfree(f_chrdev);
    kfree(f_drv);
}


static int flexcan_c_alloc_device(u8 dev_num)
{
    int err;
    struct flexcan_c_brecv *rbuf = &f_brecv[dev_num];
    struct flexcan_c_bsend *sbuf = &f_bsend[dev_num];
    struct flexcan_c_device *f_cdev = &f_chrdev[dev_num];

    f_cdev->bittiming = flexcan_c_allocate(sizeof(struct can_bittiming), 1);
    if(IS_ERR_OR_NULL(f_cdev->bittiming)) {
        err = -ENOMEM;
        goto failed_kmalloc_bittiming;
    }
    memset(f_cdev->bittiming, 0, sizeof(struct can_bittiming));

    f_cdev->stats = flexcan_c_allocate(sizeof(struct flexcan_stats), 1);
    if(IS_ERR_OR_NULL(f_cdev->stats)) {
        err = -ENOMEM;
        goto failed_kmalloc_stats;
    }
    memset(f_cdev->stats, 0, sizeof(struct flexcan_stats));

    rbuf->buf = flexcan_c_allocate(sizeof(struct flexcan_frame_mb), FLEXCAN_BUF_RECV_CAP);
    if(IS_ERR_OR_NULL(rbuf->buf)) {
        err = -ENOMEM;
        goto failed_kmalloc_brecv;
    }
    memset(rbuf->buf, 0, sizeof(struct flexcan_frame_mb) * FLEXCAN_BUF_RECV_CAP);

    sbuf->buf = flexcan_c_allocate(sizeof(struct can_frame), FLEXCAN_BUF_SEND_CAP);
    if(IS_ERR_OR_NULL(sbuf->buf)) {
        err = -ENOMEM;
        goto failed_kmalloc_bsend;
    }
    memset(sbuf->buf, 0, sizeof(struct can_frame) * FLEXCAN_BUF_SEND_CAP);

    rbuf->capacity = FLEXCAN_BUF_RECV_CAP;
    rbuf->nbuf.n_pop = 0;
    rbuf->nbuf.n_push = 0;
    rbuf->nbuf.n_save_pop = 0;

    sbuf->capacity = FLEXCAN_BUF_SEND_CAP;
    sbuf->nbuf.n_pop = 0;
    sbuf->nbuf.n_push = 0;
    sbuf->nbuf.n_save_pop = 0;

    return 0;

 failed_kmalloc_queue:
     kfree(sbuf->buf);
 failed_kmalloc_bsend:
     kfree(rbuf->buf);
 failed_kmalloc_brecv:
     kfree(f_cdev->stats);
 failed_kmalloc_stats:
     kfree(f_cdev->bittiming);
 failed_kmalloc_bittiming:
     return err;
}


static inline void flexcan_c_free_device(u8 dev_num)
{
    kfree((&f_bsend[dev_num])->buf);
    kfree((&f_brecv[dev_num])->buf);;
    kfree((&f_chrdev[dev_num])->stats);
    kfree((&f_chrdev[dev_num])->bittiming);
}


static inline u8 flexcan_c_calc_num(struct platform_device *pdev)
{
    if((pdev->id < 0) && (IS_ERR_OR_NULL(f_drv))) {
        return 0;
    }
    else if((pdev->id < 0) && (f_drv)) {
        return 1;
    }

    return -1;
}


static inline void flexcan_c_cdev_clear_stats(struct flexcan_stats *stats)
{
    if(IS_ERR_OR_NULL(stats)) {
        return;
    }

    stats->tx_frames = 0;
    stats->rx_frames = 0;
    stats->rx_bytes = 0;
    stats->tx_bytes = 0;

    stats->int_wak = 0;
    stats->int_state = 0;
    stats->int_rx_frame = 0;
    stats->int_tx_frame = 0;
    stats->int_num = 0;

    stats->err_tx = 0;
    stats->err_rx = 0;
    stats->err_over = 0;
    stats->err_warn = 0;
    stats->err_frame = 0;
    stats->err_drop = 0;
    stats->err_length = 0;
    stats->err_fifo = 0;

    stats->reg_esr = 0;
    stats->reg_mcr = 0;
    stats->reg_ctrl = 0;
}


static inline void flexcan_c_cdev_clear(struct flexcan_c_device *f_cdev)
{
    flexcan_c_cdev_clear_stats(f_cdev->stats);
    f_cdev->reg_ctrl_bittiming = FLEXCAN_BTRT_DFLT | FLEXCAN_CTRL_LOM;

    init_waitqueue_head(&(f_cdev->inq));
    init_waitqueue_head(&(f_cdev->outq));

    init_MUTEX(&f_cdev->sem);
}


static int flexcan_autoset_baudrate(const u8 dev_num)
{
    struct flexcan_c_device *f_cdev = &f_chrdev[dev_num];
    struct flexcan_stats *stats = f_cdev->stats;
    struct flexcan_regs __iomem *regs = f_cdev->base;
    u32 reg_mcr, reg_ctrl;
    int i, err;

    if(((f_cdev->autoset_bittiming_flags & FLEXCAN_AUTOSET_END) == 0) && ((f_cdev->autoset_bittiming_flags & FLEXCAN_AUTOSET_WORK) == 0)) {
        // Сбрасываем флаги (на всякий случай)
        f_cdev->autoset_bittiming_flags = 0;

        // Сохраняем регистр ctrl для востановления (если что)
        f_cdev->reg_ctrl_autoset = flexcan_read(&regs->ctrl);
        // устанавливаем настройки по умолчанию для регистра скорости
        f_cdev->reg_ctrl_bittiming = FLEXCAN_BTRT_DFLT | FLEXCAN_CTRL_LOM;

        // Запускаем автонастройку
        f_cdev->autoset_bittiming_flags |= FLEXCAN_AUTOSET_WORK;

        up (&f_cdev->sem);   // возможно с семафором тут лишнее
        if (down_interruptible(&f_cdev->sem)) {
            return -ERESTARTSYS;
        }
        flexcan_chip_stop(dev_num);
        if(flexcan_chip_start(dev_num)) {
            dev_dbg(&f_cdev->dev, "error when try to start the module\n");
        }
        up (&f_cdev->sem);
    }


    if(((f_cdev->autoset_bittiming_flags & FLEXCAN_AUTOSET_END) == 0) && ((f_cdev->autoset_bittiming_flags & FLEXCAN_AUTOSET_WORK) == 1)) {
        if((f_cdev->autoset_bittiming_flags & FLEXCAN_AUTOSET_ERR) == 1) {
            // Ошибки на линии, меняем скорость

            reg_ctrl = flexcan_read(&regs->ctrl);
            if((reg_ctrl & FLEXCAN_BTRT_MASK) == FLEXCAN_BTRT_1000) {
                reg_ctrl &= ~FLEXCAN_BTRT_MASK; // Сбрасываем настройки скорости
                reg_ctrl |= FLEXCAN_BTRT_500;   // Устанавливаем новую скорость
                f_cdev->reg_ctrl_bittiming |= reg_ctrl;
            }
            else if((reg_ctrl & FLEXCAN_BTRT_MASK) == FLEXCAN_BTRT_500) {
                reg_ctrl &= ~FLEXCAN_BTRT_MASK; // Сбрасываем настройки скорости
                reg_ctrl |= FLEXCAN_BTRT_250;   // Устанавливаем новую скорость
                f_cdev->reg_ctrl_bittiming |= reg_ctrl;
            }
            else if((reg_ctrl & FLEXCAN_BTRT_MASK) == FLEXCAN_BTRT_250) {
                reg_ctrl &= ~FLEXCAN_BTRT_MASK; // Сбрасываем настройки скорости
                reg_ctrl |= FLEXCAN_BTRT_125;   // Устанавливаем новую скорость
                f_cdev->reg_ctrl_bittiming |= reg_ctrl;
            }
            else if((reg_ctrl & FLEXCAN_BTRT_MASK) == FLEXCAN_BTRT_125) {
                reg_ctrl &= ~FLEXCAN_BTRT_MASK; // Сбрасываем настройки скорости
                reg_ctrl |= FLEXCAN_BTRT_100;   // Устанавливаем новую скорость
                f_cdev->reg_ctrl_bittiming |= reg_ctrl;
            }
            else if((reg_ctrl & FLEXCAN_BTRT_MASK) == FLEXCAN_BTRT_100) {
                reg_ctrl &= ~FLEXCAN_BTRT_MASK; // Сбрасываем настройки скорости
                reg_ctrl |= FLEXCAN_BTRT_1000;   // Устанавливаем новую скорость
                f_cdev->reg_ctrl_bittiming |= reg_ctrl;
            }
            else {
                dev_dbg(&f_cdev->dev, "unknown module baudrate\n");
            }

            f_cdev->autoset_bittiming_flags &= ~FLEXCAN_AUTOSET_ERR;

            up (&f_cdev->sem);   // возможно с семафором тут лишнее
            if (down_interruptible(&f_cdev->sem)) {
                return -ERESTARTSYS;
            }
            flexcan_chip_stop(dev_num);
            if(flexcan_chip_start(dev_num)) {
                dev_dbg(&f_cdev->dev, "error when try to start the module\n");
            }
            up (&f_cdev->sem);
        }
        else if((f_cdev->autoset_bittiming_flags & FLEXCAN_AUTOSET_OK) == 1) {
            // Скорость выбрана верно, останавливаем настройку
            f_cdev->autoset_bittiming_flags |= FLEXCAN_AUTOSET_END;
            f_cdev->autoset_bittiming_flags &= ~FLEXCAN_AUTOSET_WORK;
        }
    }

    return 0;
}


static int flexcan_probe(struct platform_device *pdev)
{
    const struct of_device_id *of_id;
    const struct flexcan_devtype_data *devtype_data;

    struct device *temp_dev;
    struct flexcan_c_device *f_cdev;

    struct resource *mem;
    struct clk *clk_ipg = NULL, *clk_per = NULL;
    resource_size_t mem_size;

    void __iomem *base;
    int err, irq, major;
    u8 dev_num = 0;
    u32 clock_freq = 0;

    printk("%s driver ver %s by Strim-tech\n", FLEXCAN_DRV_NAME, FLEXCAN_DRV_VER);

    if(pdev->dev.of_node) {
        of_property_read_u32(pdev->dev.of_node, "clock-frequency", &clock_freq);
    }

    if (!clock_freq) {
        clk_ipg = devm_clk_get(&pdev->dev, "ipg");
        if (IS_ERR_OR_NULL(clk_ipg)) {
            dev_err(&pdev->dev, "no ipg clock defined\n");
            err = PTR_ERR(clk_ipg);
            goto failed_clock;
        }

        clk_per = devm_clk_get(&pdev->dev, "per");
        if (IS_ERR_OR_NULL(clk_per)) {
            dev_err(&pdev->dev, "no per clock defined\n");
            err = PTR_ERR(clk_per);
            goto failed_clock;
        }

        clock_freq = clk_get_rate(clk_per);
    }

    mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    irq = platform_get_irq(pdev, 0);
    if (IS_ERR_OR_NULL(mem) || irq <= 0) {
        err = -ENODEV;
        goto failed_get;
    }

    dev_num = irq - FLEXCAN_IRQ_BASE;

    mem_size = resource_size(mem);
    if (!request_mem_region(mem->start, mem_size, pdev->name)) {
        err = -EBUSY;
        goto failed_get;
    }

    base = ioremap(mem->start, mem_size);
    if (IS_ERR_OR_NULL(base)) {
        err = -ENOMEM;
        goto failed_map;
    }

    if(IS_ERR_OR_NULL(f_drv)) {
        err = flexcan_c_alloc_base();
        if(err) {
            goto failed_kmalloc;
        }
    }

    err = flexcan_c_alloc_device(dev_num);
    if(err) {
        goto failed_kmalloc_dev;
    }

    of_id = of_match_device(flexcan_of_match, &pdev->dev);
    if(of_id) {
        devtype_data = of_id->data;
    }
    else if(pdev->id_entry->driver_data) {
        devtype_data = (struct flexcan_devtype_data *) pdev->id_entry->driver_data;
    }
    else{
        err = -ENODEV;
        goto failed_devtype;
    }

    gpio_request(flexcan_gpio_num[dev_num], "sysfs");
    gpio_direction_output(flexcan_gpio_num[dev_num], false);
    gpio_export(flexcan_gpio_num[dev_num], false);

    f_drv->devtype_data = devtype_data;
    strcpy(f_drv->name, FLEXCAN_DRV_NAME);

    f_cdev = &f_chrdev[dev_num];
    flexcan_c_cdev_clear(f_cdev);
    snprintf(f_cdev->name, IFNAMSIZ, "%s%d", "can", dev_num);

    f_cdev->irq_num = irq;
    f_cdev->stats->freq = clock_freq;
    f_cdev->stats->bittiming_const = &flexcan_bittiming_const;
    f_cdev->stats->ctrlmode_supported = CAN_CTRLMODE_LOOPBACK | CAN_CTRLMODE_LISTENONLY | CAN_CTRLMODE_3_SAMPLES | CAN_CTRLMODE_BERR_REPORTING;
    f_cdev->do_set_mode = flexcan_set_mode;
    f_cdev->do_get_berr_counter = flexcan_get_berr_counter;
    f_cdev->base = base;
    f_cdev->dev = pdev->dev;
    f_cdev->clk_ipg = clk_ipg;
    f_cdev->clk_per = clk_per;
    f_cdev->pdata = pdev->dev.platform_data;
    f_cdev->gpio_led = flexcan_gpio_num[dev_num];

    if(flexcan_c_is_init() == 0) {
        err = alloc_chrdev_region(&f_drv->devt, FLEXCAN_DEV_FIRST, FLEXCAN_DEV_COUNT, f_drv->name);
        if (err < 0) {
            dev_dbg(&f_cdev->dev, "alloc chrdev region failed\n");
            goto failed_reg_chrdev;
        }
    }

    major = MAJOR(f_drv->devt);
    f_cdev->devt = MKDEV(major, dev_num);
    dev_dbg(&f_cdev->dev, "major = %d, dev_num = %d\n",major, dev_num);

    cdev_init(&f_cdev->chrdev, &flexcan_c_fops);
    f_cdev->chrdev.owner = THIS_MODULE;
    f_cdev->chrdev.ops = &flexcan_c_fops;

    err = cdev_add(&f_cdev->chrdev, f_cdev->devt, 1);
    if(err) {
        dev_dbg(&f_cdev->dev, "cdev add error\n");
        goto failed_cdev_add;
    }

    if(flexcan_c_is_init() == 0) {
        f_drv->f_class = class_create(THIS_MODULE, f_drv->name);
        if(IS_ERR_OR_NULL(f_drv->f_class))  {
            dev_dbg(&f_cdev->dev, "f_class create error\n");
            goto failed_class_create;
        }
#ifdef USE_FLEXCAN_PROC
        flexcan_proc_dir_init(dev_num);
#endif
    }

    temp_dev = device_create(f_drv->f_class, &pdev->dev, f_cdev->devt, NULL, f_cdev->name);
    if (IS_ERR_OR_NULL(temp_dev))    {
        err = -1;
        dev_dbg(&f_cdev->dev, "device create error\n");
        goto failed_device_create;
    }

    if(flexcan_c_is_init() == 0) {
        f_drv->is_init = 1;
    }

    flexcan_open(dev_num);

    return 0;

 failed_device_create:
    if(flexcan_c_is_init() == 0) {
        class_destroy(f_drv->f_class);
    }
 failed_class_create:
 failed_cdev_add:
    if(flexcan_c_is_init() == 0) {
        unregister_chrdev_region(f_cdev->devt, FLEXCAN_DEV_COUNT);
    }
 failed_devtype:
 failed_reg_chrdev:
    gpio_unexport(flexcan_gpio_num[dev_num]);
    gpio_free(flexcan_gpio_num[dev_num]);
    iounmap(base);
    flexcan_c_free_device(dev_num);
 failed_kmalloc_dev:
    if(flexcan_c_is_init() == 0) {
        flexcan_c_free_base();
    }
 failed_kmalloc:
 failed_map:
    release_mem_region(mem->start, mem_size);
 failed_get:
    clk_put(clk_ipg);
    clk_put(clk_per);
 failed_clock:
    return err;
}


static int flexcan_remove(struct platform_device *pdev)
{
    struct resource *mem;
    struct flexcan_c_device *f_cdev;
    u8 dev_num = flexcan_c_calc_num(pdev);

    if(flexcan_c_is_init() == 0) {
        dev_num = 0;
    }
    f_cdev = &f_chrdev[dev_num];

    gpio_unexport(f_cdev->gpio_led);
    gpio_free(f_cdev->gpio_led);
//    printk("%s driver %s func: dev_num %d start\n", FLEXCAN_DRV_NAME, __FUNCTION__, dev_num);
//    printk("%s driver %s func: &f_chrdev[%d] 0x%x\n", FLEXCAN_DRV_NAME, __FUNCTION__, dev_num,  &f_chrdev[dev_num]);
//    printk("%s driver %s func: f_cdev 0x%x\n", FLEXCAN_DRV_NAME, __FUNCTION__, f_cdev);
//    printk("%s driver %s func: f_drv 0x%x\n", FLEXCAN_DRV_NAME, __FUNCTION__, f_drv);
//    printk("%s driver %s func: f_drv->f_class 0x%x\n", FLEXCAN_DRV_NAME, __FUNCTION__, f_drv->f_class);
//    printk("%s driver %s func: flexcan_c_is_init() = %d\n", FLEXCAN_DRV_NAME, __FUNCTION__, flexcan_c_is_init());

    flexcan_close(dev_num);

#ifdef USE_FLEXCAN_PROC
    flexcan_proc_file_exit(dev_num);
#endif

    device_destroy(f_drv->f_class, f_cdev->devt);
    if(flexcan_c_is_init() == 0) {
#ifdef USE_FLEXCAN_PROC
        flexcan_proc_dir_exit();
#endif
        class_destroy(f_drv->f_class);
    }

    cdev_del(&f_cdev->chrdev);
    if(flexcan_c_is_init() == 0) {
        unregister_chrdev_region(f_cdev->devt, FLEXCAN_DEV_COUNT);
        // flexcan_proc_dir_exit(); /* нужно перенести, но куда?..... */
    }

    platform_set_drvdata(pdev, NULL);
    iounmap(f_cdev->base);

    mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    release_mem_region(mem->start, resource_size(mem));

    clk_put(f_cdev->clk_ipg);
    clk_put(f_cdev->clk_per);

    flexcan_c_free_device(dev_num);

    if(flexcan_c_is_init() == 0) {
        flexcan_c_free_base();
    }
    else if(flexcan_c_is_init() == 1)  {
        f_drv->is_init = 0;
    }

    printk("%s driver %s func: dev_num %d end\n", FLEXCAN_DRV_NAME, __FUNCTION__, dev_num);

    return 0;
}


#ifdef CONFIG_PM
#ifdef USE_STRIM_FLEXCAN
static int flexcan_suspend(struct platform_device *pdev, pm_message_t state)
{
    int err;
    u8 dev_num = flexcan_c_calc_num(pdev);
    struct flexcan_c_device *f_cdev = &f_chrdev[dev_num];

    f_cdev->stats->state = CAN_STATE_SLEEPING;
//    if(f_drv->version >= FLEXCAN_VER_10_0_12) { /* enable stop request for wakeup */
//        mxc_iomux_set_gpr_register(13, 28, 1, 1);
//    }

    err = enable_irq_wake(f_cdev->irq_num);
    if(err) {
        return err;
    }

    return 0;
}
#else
static int flexcan_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct flexcan_priv *priv = netdev_priv(dev);

	if (netif_running(dev)) {
		netif_stop_queue(dev);
		netif_device_detach(dev);
		/*
		* if wakeup is enabled, enter stop mode
		* else enter disabled mode.
		*/
		if (device_may_wakeup(&pdev->dev)) {
			enable_irq_wake(dev->irq);
			flexcan_enter_stop_mode(priv);
		} else {
			flexcan_chip_disable(priv);
		}
	} else {
		flexcan_chip_disable(priv);
	}
	priv->can.state = CAN_STATE_SLEEPING;

	return 0;
}
#endif


#ifdef USE_STRIM_FLEXCAN
static int flexcan_resume(struct platform_device *pdev)
{
    int err;
    u8 dev_num = flexcan_c_calc_num(pdev);
    struct flexcan_c_device *f_cdev = &f_chrdev[dev_num];

    err = disable_irq_wake(f_cdev->irq_num);
    if(err) {
        return err;
    }

    f_cdev->stats->state = CAN_STATE_ERROR_ACTIVE;

    return 0;
}
#else
static int flexcan_resume(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct flexcan_priv *priv = netdev_priv(dev);

	priv->can.state = CAN_STATE_ERROR_ACTIVE;
	if (netif_running(dev)) {
		netif_device_attach(dev);
		netif_start_queue(dev);

		if (device_may_wakeup(&pdev->dev)) {
			disable_irq_wake(dev->irq);
			flexcan_exit_stop_mode(priv);
		} else {
			flexcan_chip_enable(priv);
		}
	} else {
		flexcan_chip_enable(priv);
	}

	return 0;
}
#endif

#else
#define flexcan_suspend NULL
#define flexcan_resume NULL
#endif

static struct platform_driver flexcan_driver = {
	.driver = {
#ifdef USE_STRIM_FLEXCAN
	    .name = FLEXCAN_DRV_NAME,
#else
	    .name = DRV_NAME,
#endif
		.owner = THIS_MODULE,
		.of_match_table = flexcan_of_match,
	},
	.probe = flexcan_probe,
	.remove = flexcan_remove,
	.suspend = flexcan_suspend,
	.resume = flexcan_resume,
	.id_table = flexcan_id_table,
};

module_platform_driver(flexcan_driver);

MODULE_AUTHOR("Sascha Hauer <kernel@pengutronix.de>, "
	      "Marc Kleine-Budde <kernel@pengutronix.de>, "
          "Nikita Divakov <n.divakov@strim-tech.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("CAN port driver with file operations and circle buffer for flexcan based chip");
