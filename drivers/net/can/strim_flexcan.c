/**
 * @file flexcan.c
 * @brief FLEXCAN CAN controller driver
 * @details This file is driver for work with flexcan controller
 * @version 1.1.25
 * @date 04.04.2015
 * @author Mikita Dzivakou
 * @copyright (c) 2005-2006 Varma Electronics Oy
 * @copyright (c) 2009 Sascha Hauer, Pengutronix
 * @copyright (c) 2010 Marc Kleine-Budde, Pengutronix
 * @copyright (c) 2015 Mikita Dzivakou, Strim-tech
 * @verbatim
   LICENCE:
   This program is free software; you can redistribute it and/or
   modify it under the terms of the GNU General Public License as
   published by the Free Software Foundation version 2.
   
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
 * @endverbatim
 */

// #include <linux/netdevice.h>
#include <linux/can/error.h>
#include <linux/can/platform/flexcan.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/time.h>

#include <linux/types.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/kdev_t.h>
#include <linux/version.h>
#include <asm/io.h>
#include <linux/string.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/fcntl.h>
#include <linux/poll.h>

#include <mach/clock.h>
#include <mach/hardware.h>

#include "chr_flexcan.h"

#ifdef CONFIG_ARCH_MXC
	#include <mach/iomux-v3.h>
#endif

/* FLEXCAN module configuration register (CANMCR) bits */
#define FLEXCAN_MCR_MDIS				BIT(31)
#define FLEXCAN_MCR_FRZ					BIT(30)
#define FLEXCAN_MCR_FEN					BIT(29)
#define FLEXCAN_MCR_HALT				BIT(28)
#define FLEXCAN_MCR_NOT_RDY				BIT(27)
#define FLEXCAN_MCR_WAK_MSK				BIT(26)
#define FLEXCAN_MCR_SOFTRST				BIT(25)
#define FLEXCAN_MCR_FRZ_ACK				BIT(24)
#define FLEXCAN_MCR_SUPV				BIT(23)
#define FLEXCAN_MCR_SLF_WAK				BIT(22)
#define FLEXCAN_MCR_WRN_EN				BIT(21)
#define FLEXCAN_MCR_LPM_ACK				BIT(20)
#define FLEXCAN_MCR_WAK_SRC				BIT(19)
#define FLEXCAN_MCR_DOZE				BIT(18)
#define FLEXCAN_MCR_SRX_DIS				BIT(17)
#define FLEXCAN_MCR_BCC					BIT(16)
#define FLEXCAN_MCR_LPRIO_EN			BIT(13)
#define FLEXCAN_MCR_AEN					BIT(12)
#define FLEXCAN_MCR_MAXMB(x)			((x) & 0xf)
#define FLEXCAN_MCR_IDAM_A				(0 << 8)
#define FLEXCAN_MCR_IDAM_B				(1 << 8)
#define FLEXCAN_MCR_IDAM_C				(2 << 8)
#define FLEXCAN_MCR_IDAM_D				(3 << 8)

/* FLEXCAN control register (CANCTRL) bits */
#define FLEXCAN_CTRL_PRESDIV(x)			(((x) & 0xff) << 24)
#define FLEXCAN_CTRL_RJW(x)				(((x) & 0x03) << 22)
#define FLEXCAN_CTRL_PSEG1(x)			(((x) & 0x07) << 19)
#define FLEXCAN_CTRL_PSEG2(x)			(((x) & 0x07) << 16)
#define FLEXCAN_CTRL_BOFF_MSK			BIT(15)
#define FLEXCAN_CTRL_ERR_MSK			BIT(14)
#define FLEXCAN_CTRL_CLK_SRC			BIT(13)
#define FLEXCAN_CTRL_LPB				BIT(12)
#define FLEXCAN_CTRL_TWRN_MSK			BIT(11)
#define FLEXCAN_CTRL_RWRN_MSK			BIT(10)
#define FLEXCAN_CTRL_SMP				BIT(7)
#define FLEXCAN_CTRL_BOFF_REC			BIT(6)
#define FLEXCAN_CTRL_TSYN				BIT(5)
#define FLEXCAN_CTRL_LBUF				BIT(4)
#define FLEXCAN_CTRL_LOM				BIT(3)
#define FLEXCAN_CTRL_PROPSEG(x)			((x) & 0x07)

#define FLEXCAN_CTRL_ERR_BUS			(FLEXCAN_CTRL_ERR_MSK)
#define FLEXCAN_CTRL_ERR_STATE 			(FLEXCAN_CTRL_TWRN_MSK | FLEXCAN_CTRL_RWRN_MSK | FLEXCAN_CTRL_BOFF_MSK)
#define FLEXCAN_CTRL_ERR_ALL 			(FLEXCAN_CTRL_ERR_BUS | FLEXCAN_CTRL_ERR_STATE)

/* FLEXCAN error and status register (ESR) bits */
#define FLEXCAN_ESR_TWRN_INT           BIT(17)
#define FLEXCAN_ESR_RWRN_INT           BIT(16)
#define FLEXCAN_ESR_BIT1_ERR           BIT(15)
#define FLEXCAN_ESR_BIT0_ERR           BIT(14)
#define FLEXCAN_ESR_ACK_ERR            BIT(13)
#define FLEXCAN_ESR_CRC_ERR            BIT(12)
#define FLEXCAN_ESR_FRM_ERR            BIT(11)
#define FLEXCAN_ESR_STF_ERR            BIT(10)
#define FLEXCAN_ESR_TX_WRN             BIT(9)
#define FLEXCAN_ESR_RX_WRN             BIT(8)
#define FLEXCAN_ESR_IDLE               BIT(7)
#define FLEXCAN_ESR_TXRX               BIT(6)
#define FLEXCAN_EST_FLT_CONF_SHIFT     (4)
#define FLEXCAN_ESR_FLT_CONF_MASK      (0x3 << FLEXCAN_EST_FLT_CONF_SHIFT)
#define FLEXCAN_ESR_FLT_CONF_ACTIVE    (0x0 << FLEXCAN_EST_FLT_CONF_SHIFT)
#define FLEXCAN_ESR_FLT_CONF_PASSIVE   (0x1 << FLEXCAN_EST_FLT_CONF_SHIFT)
#define FLEXCAN_ESR_BOFF_INT           BIT(2)
#define FLEXCAN_ESR_ERR_INT            BIT(1)
#define FLEXCAN_ESR_WAK_INT            BIT(0)

#define FLEXCAN_ESR_ERR_BUS 			(FLEXCAN_ESR_BIT1_ERR | FLEXCAN_ESR_BIT0_ERR | FLEXCAN_ESR_ACK_ERR | \
										 FLEXCAN_ESR_CRC_ERR | FLEXCAN_ESR_FRM_ERR | FLEXCAN_ESR_STF_ERR)
#define FLEXCAN_ESR_ERR_STATE 			(FLEXCAN_ESR_TWRN_INT | FLEXCAN_ESR_RWRN_INT | FLEXCAN_ESR_BOFF_INT)
#define FLEXCAN_ESR_ERR_ALL 			(FLEXCAN_ESR_ERR_BUS | FLEXCAN_ESR_ERR_STATE)

/* FLEXCAN interrupt flag register (IFLAG) bits */
#define FLEXCAN_TX_BUF_ID                  8
#define FLEXCAN_IFLAG_BUF(x)               BIT(x)
#define FLEXCAN_IFLAG_RX_FIFO_OVERFLOW     BIT(7)
#define FLEXCAN_IFLAG_RX_FIFO_WARN	        BIT(6)
#define FLEXCAN_IFLAG_RX_FIFO_AVAILABLE    BIT(5)

#define FLEXCAN_IFLAG_DEFAULT          (FLEXCAN_IFLAG_RX_FIFO_OVERFLOW | FLEXCAN_IFLAG_RX_FIFO_AVAILABLE | FLEXCAN_IFLAG_BUF(FLEXCAN_TX_BUF_ID))

/* FLEXCAN message buffers */
#define FLEXCAN_MB_CNT_CODE(x)         (((x) & 0xf) << 24)
#define FLEXCAN_MB_CNT_SRR             BIT(22)
#define FLEXCAN_MB_CNT_IDE             BIT(21)
#define FLEXCAN_MB_CNT_RTR             BIT(20)
#define FLEXCAN_MB_CNT_LENGTH(x)       (((x) & 0xf) << 16)
#define FLEXCAN_MB_CNT_TIMESTAMP(x)    ((x) & 0xffff)

#define FLEXCAN_MB_CODE_MASK           (0xf0ffffff)


#define FLEXCAN_BTRT_1000_SP866    (0x01290005)
#define FLEXCAN_BTRT_1000_SP800    (0x012a0004)
#define FLEXCAN_BTRT_1000_SP733    (0x01230004)
#define FLEXCAN_BTRT_1000_SP700    (0x02120002)

#define FLEXCAN_BTRT_1000          (0x01290005)
#define FLEXCAN_BTRT_500           (0x023b0006)		/* Предварительно, настройка рассчитана через canconfig */
#define FLEXCAN_BTRT_250           (0x053b0006)		/* Предварительно, настройка рассчитана через canconfig */
#define FLEXCAN_BTRT_125           (0x0b3b0006)		/* Предварительно, настройка рассчитана через canconfig */
#define FLEXCAN_BTRT_100           (0x0e3b0006)		/* Предварительно, настройка рассчитана через canconfig */
#define FLEXCAN_BTRT_DFLT          FLEXCAN_BTRT_1000

#define FLEXCAN_BTRT_MASK          (0xFFFF0007)

#define FLEXCAN_DRV_NAME           "flexcan"
#define FLEXCAN_DRV_VER            "1.2.68"

#define FLEXCAN_DEV_FIRST          (0)
#define FLEXCAN_DEV_COUNT          (2)

#define FLEXCAN_BUF_RECV_CAP       (262144)
#define FLEXCAN_BUF_SEND_CAP       (64)

#define FLEXCAN_BUF_RECV_MASK      (fimx6d.f_dev[dev_num].buf_recv_size - 1)
#define FLEXCAN_BUF_SEND_MASK      (fimx6d.f_dev[dev_num].buf_send_size - 1)

#define FLEXCAN_IRQ_BASE           (142)

#define init_MUTEX(LOCKNAME) sema_init(LOCKNAME,1);

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
	u32 _reserved2[2];
	u32 crcr;		/* 0x44 */
	u32 rxfgmask;		/* 0x48 */
	u32 rxfir;		/* 0x4c */
	u32 _reserved3[12];
	struct flexcan_mb cantxfg[64];
};

enum flexcan_ip_version {
	FLEXCAN_VER_3_0_0,
	FLEXCAN_VER_3_0_4,
	FLEXCAN_VER_10_0_12,
};

static struct can_bittiming_const flexcan_bittiming_const = {
	.name = FLEXCAN_DRV_NAME,
	.tseg1_min = 4,
	.tseg1_max = 16,
	.tseg2_min = 2,
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_min = 1,
	.brp_max = 256,
	.brp_inc = 1,
};

typedef struct {
	unsigned int n_push, n_pop, n_save_pop;
} circle_buf_t;

struct flexcan_frame_mb {
	struct flexcan_mb mb;
	struct timeval time;
};

struct flexcan_chardev {

/* Имя устройства.
 * Это же имя отдается файлу устройства в /dev и /proc */
	char name[IFNAMSIZ];

/* Содержит major и minor номер зарегистрированного устройства
 * Номера извлекаются макросами MAJOR(dev_t devt) и MINOR(dev_t devt)
 * Номер формируется при регистрации устройства автоматически
 * Так же формируется макросом MKDEV(major, minor) */
	dev_t f_devt;

/* Адрес начала блока памяти устройства
 * Сохраняется при регистрации устройства
 * Используется для чтения регистров устройства с помощью struct flexcan_regs */
	void __iomem *f_base;

/* Номер вектора прерывания IRQ для устройства */
	u8 irq_num;

/* Флаг открытия файла устройства
 * Нужен для предотвращения множественного доступа к файлу
 * (а еще для запрета удаления драйвера при открытом файле, но не реализовано) */
 	int nreaders, nwriters;

/* Значение регистра CTRL для настройки скорости передачи и режима работы
 * При инициализации устанавливается значение по умолчанию FLEXCAN_BTRT_DFLT
 * Изменяется через IOCTL запрос */
	u32 reg_ctrl_bittiming;

/* Значение по умолчанию регистра CTRL для будущих использований
 * Создается в функции chip_start */
	u32 reg_ctrl_default;

/* Структура символьного утсройства */
	struct cdev f_cdev;	

/* Структура утсройства */
	struct device dev;	

/* Указатель на структуру встроенного в контроллер устройства flexcan */
	struct flexcan_platform_data *pdata;

/* Структура с данными файла устройства в файловой системе /proc  */
	struct proc_dir_entry *dev_proc_file;

/* Структура с данными по тактированию устройства */
	struct clk *clk;

/* Структура с данными о настройках таймингов устройства */
	struct can_bittiming bittiming;

/* Структура с данными о состоянии и работе устройства
 * Содержит различные счетчики и переменные состояния устройства */
	struct flexcan_stats stats;

/* Размер буфера для сохранения принятых сообщений */
	u32 buf_recv_size;

/* Номера ячеек для записи и чтения циклического буффера для сохранения */
	circle_buf_t frame_buf_recv;

/* Циклический буффер для сохранения принятых сообщений */
	struct flexcan_frame_mb buf_recv[FLEXCAN_BUF_RECV_CAP];

/* Размер буфера для сохранения сообщений на отправку */
	u32 buf_send_size;

/* Номера ячеек для записи и чтения циклического буффера сообщений на отправку  */
	circle_buf_t frame_buf_send;

/* Циклический буффер для сохранения сообщений на отправку */
	struct can_frame buf_send[FLEXCAN_BUF_SEND_CAP];

/* Структура для вызовов poll и select */
	struct fasync_struct *async_queue;

/* Очереди ожидания данных */
	wait_queue_head_t inq, outq;

/* Семафор */
	struct semaphore sem;

	// int (*do_set_bittiming)(__u8 *dev_num, struct can_bittiming *bittiming);
	int (*do_set_mode)(__u8 *dev_num, enum can_mode mode);					// вернуть когда буду убирать netdevice
	// int (*do_get_state)(__u8 *dev_num, enum can_state *state);
	int (*do_get_berr_counter)(__u8 *dev_num, struct flexcan_stats *bec);
};

struct flexcan_device	{
	char name[PLATFORM_NAME_SIZE];						
	u8 is_init;					// флаг инициализации
	dev_t f_devt;				// major номер группы

	enum flexcan_ip_version version;

	struct proc_dir_entry *dev_proc_dir;
	struct class *f_class;	
	struct flexcan_chardev f_dev[FLEXCAN_DEV_COUNT];
};

static struct flexcan_device fimx6d;

static unsigned int flexcan_start_transmit(const u8 dev_num);
static void flexcan_set_bittiming(const u8 dev_num, const u32 reg_ctrl);
static void flexcan_chip_stop(const u8 dev_num);
static int flexcan_chip_start(const u8 dev_num);

// ##########################################################################################################################################
// ##########################################################################################################################################

static inline unsigned int flexcan_recv_buf_is_empty(const u8 dev_num)
{
	struct flexcan_chardev* f_dev = &fimx6d.f_dev[dev_num];
	return (f_dev->frame_buf_recv.n_push == f_dev->frame_buf_recv.n_pop);
}


static inline unsigned int flexcan_recv_buf_is_full(const u8 dev_num)
{
	struct flexcan_chardev* f_dev = &fimx6d.f_dev[dev_num];
	unsigned int n_next_first = (f_dev->frame_buf_recv.n_push + 1) & FLEXCAN_BUF_RECV_MASK;
	return (n_next_first == f_dev->frame_buf_recv.n_pop);
}


static inline unsigned int flexcan_recv_buf_push(const u8 dev_num, const struct flexcan_mb *s_mb, const struct timeval *s_tv)
{
	struct flexcan_chardev* f_dev = &fimx6d.f_dev[dev_num];
	circle_buf_t *f_buf = &f_dev->frame_buf_recv;

	if(flexcan_recv_buf_is_full(dev_num)) {
		return 1; 
	}

	struct flexcan_frame_mb *buf = (struct flexcan_frame_mb *) (f_dev->buf_recv + f_buf->n_push);
	memcpy(&buf->mb, s_mb, sizeof(struct flexcan_mb));
	memcpy(&buf->time, s_tv, sizeof(struct timeval));
	f_buf->n_push = (f_buf->n_push + 1) & FLEXCAN_BUF_RECV_MASK;

	return 0;
}


static inline unsigned int flexcan_recv_buf_pop(const u8 dev_num, struct flexcan_frame_mb *s_frame)
{
	struct flexcan_chardev* f_dev = &fimx6d.f_dev[dev_num];
	circle_buf_t *f_buf = &f_dev->frame_buf_recv;

	if(flexcan_recv_buf_is_empty(dev_num))	{
		return 1;
	}

	*(struct flexcan_frame_mb *) s_frame = *(struct flexcan_frame_mb *) (f_dev->buf_recv + f_buf->n_pop);
	f_buf->n_pop = (f_buf->n_pop + 1) & FLEXCAN_BUF_RECV_MASK;
	f_buf->n_save_pop = f_buf->n_pop;
	
	return 0;
}


static inline unsigned int flexcan_recv_buf_get_free_space(const u8 dev_num)
{
	circle_buf_t *f_buf = &fimx6d.f_dev[dev_num].frame_buf_recv;
	unsigned int buf_free_space = (f_buf->n_pop - f_buf->n_push - 1) & FLEXCAN_BUF_RECV_MASK;
	return buf_free_space;
}


static inline unsigned int flexcan_recv_buf_get_data_size(const u8 dev_num)
{
	circle_buf_t *f_buf = &fimx6d.f_dev[dev_num].frame_buf_recv;
	// unsigned int buf_data_size = (f_buf->n_push - f_buf->n_pop) & FLEXCAN_BUF_RECV_MASK;
	unsigned int buf_data_size = (f_buf->n_push - f_buf->n_save_pop) & FLEXCAN_BUF_RECV_MASK;
	return buf_data_size;
}


static inline unsigned int flexcan_recv_buf_end(const u8 dev_num)
{
	circle_buf_t *f_buf = &fimx6d.f_dev[dev_num].frame_buf_recv;
	f_buf->n_pop = f_buf->n_push;
	return 0;
}


static inline unsigned int flexcan_recv_buf_begin(const u8 dev_num)
{
	circle_buf_t *f_buf = &fimx6d.f_dev[dev_num].frame_buf_recv;
	f_buf->n_pop = f_buf->n_save_pop;
	return 0;
}

// ##########################################################################################################################################
// ##########################################################################################################################################

static inline unsigned int flexcan_send_buf_is_empty(const u8 dev_num)
{
	struct flexcan_chardev* f_dev = &fimx6d.f_dev[dev_num];
	return (f_dev->frame_buf_send.n_push == f_dev->frame_buf_send.n_pop);
}


static inline unsigned int flexcan_send_buf_is_full(const u8 dev_num)
{
	struct flexcan_chardev* f_dev = &fimx6d.f_dev[dev_num];
	unsigned int n_next_first = (f_dev->frame_buf_send.n_push + 1) & FLEXCAN_BUF_SEND_MASK;
	return (n_next_first == f_dev->frame_buf_send.n_pop);
}


static inline unsigned int flexcan_send_buf_push(const u8 dev_num, const struct can_frame *s_cf)
{
	struct flexcan_chardev* f_dev = &fimx6d.f_dev[dev_num];
	circle_buf_t *f_buf = &f_dev->frame_buf_send;
	unsigned int n_next_first = (f_buf->n_push + 1) & FLEXCAN_BUF_SEND_MASK;

	if(flexcan_send_buf_is_full(dev_num)) 	{
		return 1;
	}

	struct can_frame *buf = (struct can_frame *) (f_dev->buf_send + f_buf->n_push);
	memcpy(buf, s_cf, sizeof(struct can_frame));
	f_buf->n_push = n_next_first;

	return 0;
}


static inline unsigned int flexcan_send_buf_pop(const u8 dev_num, struct can_frame *s_frame)
{
	struct flexcan_chardev* f_dev = &fimx6d.f_dev[dev_num];
	circle_buf_t *f_buf = &f_dev->frame_buf_send;

	if(flexcan_send_buf_is_empty(dev_num))	{
		return 1;
	}

	*(struct can_frame *) s_frame = *(struct can_frame *) (f_dev->buf_send + f_buf->n_pop);
	f_buf->n_pop = (f_buf->n_pop + 1) & FLEXCAN_BUF_SEND_MASK;

	return 0;
}


static inline unsigned int flexcan_send_buf_free_space(const u8 dev_num)
{
	circle_buf_t *f_buf = &fimx6d.f_dev[dev_num].frame_buf_send;
	unsigned int buf_free_space = (f_buf->n_pop - f_buf->n_push - 1) & FLEXCAN_BUF_SEND_MASK;
	return buf_free_space;
}


static inline unsigned int flexcan_send_buf_data_size(const u8 dev_num)
{
	circle_buf_t *f_buf = &fimx6d.f_dev[dev_num].frame_buf_send;
	unsigned int buf_data_size = (f_buf->n_push - f_buf->n_pop) & FLEXCAN_BUF_SEND_MASK;
	return buf_data_size;
}

// ##########################################################################################################################################
// ##########################################################################################################################################

static void flexcan_decode_frame(struct can_frame *cf, const struct flexcan_mb *mb)
{
	if (mb->can_ctrl & FLEXCAN_MB_CNT_IDE)	{
		cf->can_id = ((mb->can_id >> 0) & CAN_EFF_MASK) | CAN_EFF_FLAG;
	}
	else 	{
		cf->can_id = (mb->can_id >> 18) & CAN_SFF_MASK;
	}

	if (mb->can_ctrl & FLEXCAN_MB_CNT_RTR)	{
		cf->can_id |= CAN_RTR_FLAG;
	}
	cf->can_dlc = ((mb->can_ctrl >> 16) & 0xf);

	if(cf->can_dlc == 0)	{
		*(__be32 *)(cf->data + 0) = 0x00000000;
		*(__be32 *)(cf->data + 4) = 0x00000000;
	}
	else if(cf->can_dlc <= 4)	{
		*(__be32 *)(cf->data + 0) = cpu_to_be32(readl(&mb->data[0]));
		*(__be32 *)(cf->data + 4) = 0x00000000;		
	}
	else 	{
		*(__be32 *)(cf->data + 0) = cpu_to_be32(readl(&mb->data[0]));
		*(__be32 *)(cf->data + 4) = cpu_to_be32(readl(&mb->data[1]));
	}
}

// ##########################################################################################################################################
// ##########################################################################################################################################

static int flexcan_char_fasync(int fd, struct file *filp, int mode)
{
	struct inode *inode = filp->f_path.dentry->d_inode;
	const u8 dev_num = iminor(inode);
	// printk("%s.%d: %s\n", fimx6d.name, (int) dev_num, __func__);
	return fasync_helper(fd, filp, mode, &fimx6d.f_dev[dev_num].async_queue);
}


static int flexcan_char_open(struct inode *i, struct file *filp)
{
	const u8 dev_num = iminor(i);
	struct flexcan_chardev* f_dev = &fimx6d.f_dev[dev_num];

	// printk("%s.%d: %s\n", fimx6d.name, (int) dev_num, __func__);
	if(down_interruptible(&f_dev->sem)) {
		return -ERESTARTSYS;
	}
	if(filp->f_mode & FMODE_READ) {
		f_dev->nreaders++;
	}
	if (filp->f_mode & FMODE_WRITE) {
		f_dev->nwriters++;
	}
	up(&f_dev->sem);

	dev_dbg(&f_dev->dev, "flexcan_char_open file open success\n");
	
	try_module_get(THIS_MODULE);
	// return nonseekable_open(i, filp);	// если отключим llseek
	return 0;
}


static int flexcan_char_release(struct inode *i, struct file *filp)
{
	const u8 dev_num = iminor(i);
	struct flexcan_chardev* f_dev = &fimx6d.f_dev[dev_num];

	// printk("%s.%d: %s\n", fimx6d.name, (int) dev_num, __func__);
	flexcan_char_fasync(-1, filp, 0);
	down(&f_dev->sem);
	if (filp->f_mode & FMODE_READ) {
		f_dev->nreaders--;
	}
	if (filp->f_mode & FMODE_WRITE) {
		f_dev->nwriters--;
	}
	up(&f_dev->sem);

	module_put(THIS_MODULE);
    return 0;
}


static loff_t flexcan_char_llseek(struct file *filp, loff_t off, int whence)
{
	struct inode *inode = filp->f_path.dentry->d_inode;
	const u8 dev_num = iminor(inode);
	struct flexcan_chardev* f_dev = &fimx6d.f_dev[dev_num];
    loff_t newpos = 0;

    // printk("%s.%d: %s\n", fimx6d.name, (int) dev_num, __func__);
    switch(whence) {
		case 0: /* SEEK_SET */
			// printk("%s.%d: %s SEEK_SET filp->f_pos = %d, off = %d\n", fimx6d.name, (int) dev_num, __func__, filp->f_pos, off);
			flexcan_recv_buf_begin(dev_num);
			newpos = 0;	/* Всегда в начало */
			break;
		case 1: /* SEEK_CUR */
			// printk("%s.%d: %s SEEK_CUR filp->f_pos = %d, off = %d\n", fimx6d.name, (int) dev_num, __func__, filp->f_pos, off);
			newpos = flexcan_recv_buf_get_data_size(dev_num) * CAN_DATA_MSG_LENGTH;
			break;
		case 2: /* SEEK_END */
			// printk("%s.%d: %s SEEK_END filp->f_pos = %d, off = %d\n", fimx6d.name, (int) dev_num, __func__, filp->f_pos, off);
			flexcan_recv_buf_end(dev_num);
			newpos = flexcan_recv_buf_get_data_size(dev_num) * CAN_DATA_MSG_LENGTH;
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


static unsigned int flexcan_char_poll(struct file *filp, poll_table *wait)
{
	struct inode *inode = filp->f_path.dentry->d_inode;
	const u8 dev_num = iminor(inode);
	struct flexcan_chardev* f_dev = &fimx6d.f_dev[dev_num];
	unsigned int mask = 0;

	// printk("%s.%d: %s\n", fimx6d.name, (int) dev_num, __func__);
	down(&f_dev->sem);
	poll_wait(filp, &f_dev->inq,  wait);
	poll_wait(filp, &f_dev->outq, wait);
	if(flexcan_recv_buf_get_data_size(dev_num)) {
		mask |= POLLIN | POLLRDNORM;	/* readable */
	}
	if(flexcan_send_buf_is_full(dev_num) == 0) {
		mask |= POLLOUT | POLLWRNORM;	/* writable */
	}
	up(&f_dev->sem);

	return mask;
}

// ##########################################################################################################################################
// ##########################################################################################################################################

static ssize_t flexcan_char_read(struct file *filp, char __user *buf, size_t length_read, loff_t *off)
{
	struct inode *inode = filp->f_path.dentry->d_inode;
	const u8 dev_num = iminor(inode);
	struct flexcan_chardev* f_dev = &fimx6d.f_dev[dev_num];
	struct flexcan_frame_mb read_frame;
	struct can_frame cf_decoded;
	ssize_t total_length = 0;
	char data_msg_buf[CAN_DATA_MSG_LENGTH + 1];
	u64 msg_data = 0;
	int ret = 0;

	// printk("%s.%d: %s\n", fimx6d.name, (int) dev_num, __func__);
	if(length_read < CAN_DATA_MSG_LENGTH) {	/* Проверяем размер буффера пользователя */
		dev_dbg(&f_dev->dev, "flexcan_char_read return EFAULT\n");
		return -EINVAL;
	}
	if (down_interruptible(&f_dev->sem)) {
		return -ERESTARTSYS;	
	}

	while(flexcan_recv_buf_is_empty(dev_num)) {	/* Проверяем пустой ли буффер */
		up(&f_dev->sem); /* release the lock */
		if (filp->f_flags & O_NONBLOCK) {	/* если вызов не блокирующий, вываливаемся */
			dev_dbg(&f_dev->dev, "flexcan_char_read return EAGAIN\n");
			return -EAGAIN;
		}
		if (wait_event_interruptible(f_dev->inq, !flexcan_recv_buf_is_empty(dev_num))) {
			return -ERESTARTSYS; /* signal: tell the fs layer to handle it */
		}
		if (down_interruptible(&f_dev->sem)) {
			return -ERESTARTSYS;
		}
	}

	do {
		if(flexcan_recv_buf_pop(dev_num, &read_frame)) {
			dev_dbg(&f_dev->dev, "flexcan_char_read read err\n");
			break;
		}
		
		msg_data = 0;
		flexcan_decode_frame(&cf_decoded, &read_frame.mb);
		memcpy((void*) &msg_data, (void*) cf_decoded.data, cf_decoded.can_dlc);

		ret = snprintf(data_msg_buf, (CAN_DATA_MSG_LENGTH + 1), "$CAN,%d,%08x,%05x,%08x,%d,%016llx\n", dev_num, (unsigned int) read_frame.time.tv_sec, 
			(unsigned int) read_frame.time.tv_usec, cf_decoded.can_id, cf_decoded.can_dlc, msg_data);
		if(ret != CAN_DATA_MSG_LENGTH) {
			dev_dbg(&f_dev->dev, "flexcan_char_read error size: need[%d], ret[%d]\n", CAN_DATA_MSG_LENGTH, ret);
			// printk("%s.%d: %s error size: need[%d], ret[%d]\n", fimx6d.name, dev_num, __func__, msg_length, ret);
			continue;
		}

		// printk("%s.%d: %s copy_to_user %s\n", fimx6d.name, dev_num, __func__, data_msg_buf);
		ret = copy_to_user((void *) buf, &data_msg_buf, CAN_DATA_MSG_LENGTH);	
		if(ret)	{
			dev_dbg(&f_dev->dev, "flexcan_char_read can't copy = %d bytes\n", ret);
			// printk("flexcan_char_read can't copy = %d bytes\n", ret);
			buf += (CAN_DATA_MSG_LENGTH - ret);
			total_length += (CAN_DATA_MSG_LENGTH - ret);
			break;
		}
		else {
			buf += CAN_DATA_MSG_LENGTH;
			total_length += CAN_DATA_MSG_LENGTH;
		}
	} while(((total_length + CAN_DATA_MSG_LENGTH) <= length_read) && (flexcan_recv_buf_is_empty(dev_num) == 0));

	up (&f_dev->sem);
	wake_up_interruptible(&f_dev->outq);

	return total_length;
}


static ssize_t flexcan_char_write(struct file *filp, const char __user *buf, size_t length, loff_t *off)
{
	struct inode *inode = filp->f_path.dentry->d_inode;
	const u8 dev_num = iminor(inode);
	struct flexcan_chardev* f_dev = &fimx6d.f_dev[dev_num];
	struct can_frame* user_buf_ptr = (struct can_frame *) buf;
	struct can_frame cf;
	int ret = 0, to_read = 0, was_read = 0;;

	// printk("%s.%d: %s\n", fimx6d.name, (int) dev_num, __func__);
	if(length < sizeof(struct can_frame)) { /* Проверяем размер сообщения */
		dev_dbg(&f_dev->dev, "flexcan_char_write return -EFAULT\n");
		return -EINVAL;
	}
	to_read = (((int) length) / sizeof(struct can_frame));

	if(down_interruptible(&f_dev->sem)) {
		return -ERESTARTSYS;
	}

	while(flexcan_send_buf_is_full(dev_num)) {	/* Проверяем наличие данных в буфере */
		flexcan_start_transmit(dev_num);

		DEFINE_WAIT(wait);
		up(&f_dev->sem);
		if (filp->f_flags & O_NONBLOCK) {
			return -EAGAIN;
		}
		prepare_to_wait(&f_dev->outq, &wait, TASK_INTERRUPTIBLE);
		finish_wait(&f_dev->outq, &wait);
		if (signal_pending(current)) {
			return -ERESTARTSYS; /* signal: tell the fs layer to handle it */
		}
		if (down_interruptible(&f_dev->sem)) {
			return -ERESTARTSYS;
		}
	}

	do {	/* Получение данных */
		ret = copy_from_user(&cf, user_buf_ptr++, sizeof(struct can_frame));
		if(ret) {
			dev_dbg(&f_dev->dev, "flexcan_char_write can't copy = %d bytes\n", ret);
			// printk("%s.%d: %s copy_from_user can't copy = %d bytes\n", fimx6d.name, (int) dev_num, __func__, ret);
		}
		else {
			dev_dbg(&f_dev->dev, "flexcan_char_write frame: 0x%08x, %d, 0x%llx\n", cf.can_id, cf.can_dlc, (long long unsigned int) cf.data[0]);
			if(cf.can_dlc <= 8) {
				if(flexcan_send_buf_push(dev_num, &cf)) {
					dev_dbg(&f_dev->dev, "flexcan_char_write buffer drop frame\n");
					// printk("%s.%d: %s buffer drop frame\n", fimx6d.name, (int) dev_num, __func__);
					break;
				}
			flexcan_start_transmit(dev_num);
			}
			else {
				dev_dbg(&f_dev->dev, "flexcan_char_write BAD frame\n");
				// printk("%s.%d: %s BAD frame\n", fimx6d.name, (int) dev_num, __func__);
			}
			was_read++;
			to_read--;
		}
	} while((to_read > 0) && (ret == 0));

	if(flexcan_send_buf_data_size(dev_num)) {
		flexcan_start_transmit(dev_num);
	}

	up(&f_dev->sem);
	wake_up_interruptible(&f_dev->inq);  /* blocked in read() and select() */
	if(f_dev->async_queue) {	/* and signal asynchronous readers, explained late in chapter 5 */
		kill_fasync(&f_dev->async_queue, SIGIO, POLL_IN);
	}

    return (was_read * sizeof(struct can_frame));
}


static long flexcan_char_ioctl(struct file *filp, unsigned int ioctl_num, unsigned long ioctl_param)
{	
	struct inode *inode = filp->f_path.dentry->d_inode;
	const u8 dev_num = iminor(inode);
	struct flexcan_chardev* f_dev = &fimx6d.f_dev[dev_num];
	u32 read_settings = 0, set_mode = 0, set_bitrate = 0;
	static u32 set_reg_ctrl = 0;

	dev_dbg(&f_dev->dev, "flexcan_char_ioctl IOCTL request\n");
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
						printk("%s.%d: %s set bitrate 1000000\n", fimx6d.name, dev_num, __func__);
						break;
					case SET_CAN_BITRATE_500:
						set_reg_ctrl = FLEXCAN_BTRT_500;
						printk("%s.%d: %s set bitrate 500000\n", fimx6d.name, dev_num, __func__);
						break;
					case SET_CAN_BITRATE_250:
						set_reg_ctrl = FLEXCAN_BTRT_250;
						printk("%s.%d: %s set bitrate 250000\n", fimx6d.name, dev_num, __func__);
						break;
					case SET_CAN_BITRATE_125:
						set_reg_ctrl = FLEXCAN_BTRT_125;
						printk("%s.%d: %s set bitrate 125000\n", fimx6d.name, dev_num, __func__);
						break;
					case SET_CAN_BITRATE_100:
						set_reg_ctrl = FLEXCAN_BTRT_100;
						printk("%s.%d: %s set bitrate 100000\n", fimx6d.name, dev_num, __func__);
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

				if(f_dev->reg_ctrl_bittiming != set_reg_ctrl) {
					// printk("%s.%d: %s need restart module\n", fimx6d.name, dev_num, __func__);
					f_dev->reg_ctrl_bittiming = set_reg_ctrl;

					up (&f_dev->sem);	// возможно с семафором тут лишнее
					if (down_interruptible(&f_dev->sem)) {
						return -ERESTARTSYS;
					}
					flexcan_chip_stop(dev_num);
					if(flexcan_chip_start(dev_num)) {
						dev_dbg(&f_dev->dev, "flexcan_char_ioctl error when try to start the module\n");
					}
					up (&f_dev->sem);
				}
			}
			// тут вызвать функцию которая задаст новые настройки для конкретного can интерфейса
			// возможно нужно самому написать эту функцию, переписав flexcan_set_bittiming() или не нужно, еще не проверял.
			break;
		case FLEXCAN_IOCTL_WR_RD:
			return -EINVAL;	// будет не поддерживаемая инструкция пока не придумаю зачем она надо
			break;
		default:
			return -EINVAL;
			break;
	}
	return 0;
}

// ##########################################################################################################################################
// ##########################################################################################################################################

static struct file_operations flexcan_fops =
{
    .owner = THIS_MODULE,
    .llseek = flexcan_char_llseek,
    .read = flexcan_char_read,
    .write = flexcan_char_write,
    .poll = flexcan_char_poll,
    .open = flexcan_char_open,
    .release = flexcan_char_release,
	#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
    	.ioctl = flexcan_char_ioctl,
	#else
    	.unlocked_ioctl = flexcan_char_ioctl,
	#endif
    .fasync = flexcan_char_fasync,
};

// ##########################################################################################################################################
// ##########################################################################################################################################

static inline void flexcan_f_dev_init(struct flexcan_chardev *f_dev)
{
	f_dev->stats.rx_frames = 0;
	f_dev->stats.tx_frames = 0;
	f_dev->stats.rx_bytes = 0;
	f_dev->stats.tx_bytes = 0;	

	f_dev->stats.int_wak = 0;
	f_dev->stats.int_state = 0;
	f_dev->stats.int_rx_frame = 0;
	f_dev->stats.int_tx_frame = 0;
	f_dev->stats.int_num = 0;

	f_dev->stats.err_tx = 0;
	f_dev->stats.err_rx = 0;
	f_dev->stats.err_over = 0;
	f_dev->stats.err_warn = 0;
	f_dev->stats.err_frame = 0;	
	f_dev->stats.err_drop = 0;
	f_dev->stats.err_length = 0;
	f_dev->stats.err_fifo = 0;

	f_dev->stats.reg_esr = 0;	
	f_dev->stats.reg_mcr = 0;	
	f_dev->stats.reg_ctrl = 0;

	f_dev->buf_recv_size = FLEXCAN_BUF_RECV_CAP;
	f_dev->frame_buf_recv.n_push = 0;
	f_dev->frame_buf_recv.n_save_pop = 0;
	f_dev->frame_buf_recv.n_pop = 0;

	f_dev->buf_send_size = FLEXCAN_BUF_SEND_CAP;
	f_dev->frame_buf_send.n_push = 0;
	f_dev->frame_buf_send.n_save_pop = 0;
	f_dev->frame_buf_send.n_pop = 0;

	f_dev->reg_ctrl_bittiming = FLEXCAN_BTRT_DFLT;

	init_waitqueue_head(&(f_dev->inq));
	init_waitqueue_head(&(f_dev->outq));
	init_MUTEX(&f_dev->sem);
}

// ##########################################################################################################################################
// ##########################################################################################################################################

#define NAME_DIR "flexcan"
#define LEN_MSG (3072)
#define MAX_DIGITS (10)

static char *get_rw_buf(const u8 dev_num, int *length) 
{
	struct flexcan_stats *stats = &fimx6d.f_dev[dev_num].stats;
	static char buf_msg[LEN_MSG + 1];
	char *buf_ptr = buf_msg;
	int buf_length = 0;

	memset(buf_msg, 0, (LEN_MSG + 1));

	snprintf(buf_ptr, (30 + 1), "Data communication statistic:\n");
	buf_ptr += 30;
	buf_length += 30;
	snprintf(buf_ptr, (22 + MAX_DIGITS + 1), "\tReceived frames   = %d\n", (int) stats->rx_frames);	// количество принятых фреймов
	buf_ptr += (22 + MAX_DIGITS);
	buf_length += (22 + MAX_DIGITS);
	snprintf(buf_ptr, (22 + MAX_DIGITS + 1), "\tTransmited frames = %d\n", (int) stats->tx_frames);	// количество отправленных фреймов
	buf_ptr += (22 + MAX_DIGITS);
	buf_length += (22 + MAX_DIGITS);
	snprintf(buf_ptr, (22 + MAX_DIGITS + 1), "\tReceived bytes    = %d\n", (int) stats->rx_bytes);	// количество принятых байт данных
	buf_ptr += (22 + MAX_DIGITS);
	buf_length += (22 + MAX_DIGITS);
	snprintf(buf_ptr, (22 + MAX_DIGITS + 1), "\tTransmited bytes  = %d\n", (int) stats->tx_bytes);	// количество отправленных байт данных
	buf_ptr += (22 + MAX_DIGITS);
	buf_length += (22 + MAX_DIGITS);

	snprintf(buf_ptr, (23 + 1), "\nInterrupts statistic:\n");
	buf_ptr += 23;
	buf_length += 23;	
	snprintf(buf_ptr, (21 + MAX_DIGITS + 1), "\tWakeUp           = %d\n", (int) stats->int_wak);	// количество прерываний пробуждения
	buf_ptr += (21 + MAX_DIGITS);
	buf_length += (21 + MAX_DIGITS);
	snprintf(buf_ptr, (21 + MAX_DIGITS + 1), "\tState change     = %d\n", (int) stats->int_state);	// количество прерываний проверки состояния
	buf_ptr += (21 + MAX_DIGITS);
	buf_length += (21 + MAX_DIGITS);
	snprintf(buf_ptr, (21 + MAX_DIGITS + 1), "\tRead frame       = %d\n", (int) stats->int_rx_frame);	// количество прерываний чтения фреймов
	buf_ptr += (21 + MAX_DIGITS);
	buf_length += (21 + MAX_DIGITS);
	snprintf(buf_ptr, (21 + MAX_DIGITS + 1), "\tTransmit frame   = %d\n", (int) stats->int_tx_frame);	// количество прерываний отправки фреймов
	buf_ptr += (21 + MAX_DIGITS);
	buf_length += (21 + MAX_DIGITS);
	snprintf(buf_ptr, (21 + MAX_DIGITS + 1), "\tTotal interrupts = %d\n", (int) stats->int_num);	// количество прерываний
	buf_ptr += (21 + MAX_DIGITS);
	buf_length += (21 + MAX_DIGITS);

	snprintf(buf_ptr, (19 + 1), "\nErrors statistic:\n");
	buf_ptr += 19;
	buf_length += 19;
	snprintf(buf_ptr, (13 + MAX_DIGITS + 1), "\tTransmit = %d\n", (int) stats->err_tx);	// количество ошибок отправки фреймов
	buf_ptr += (13 + MAX_DIGITS);
	buf_length += (13 + MAX_DIGITS);
	snprintf(buf_ptr, (13 + MAX_DIGITS + 1), "\tReceive  = %d\n", (int) stats->err_rx);	// количество ошибок принятия фреймов
	buf_ptr += (13 + MAX_DIGITS);
	buf_length += (13 + MAX_DIGITS);
	snprintf(buf_ptr, (13 + MAX_DIGITS + 1), "\tOverflow = %d\n", (int) stats->err_over);	// количество переполнений буффера
	buf_ptr += (13 + MAX_DIGITS);
	buf_length += (13 + MAX_DIGITS);
	snprintf(buf_ptr, (13 + MAX_DIGITS + 1), "\tWarning  = %d\n", (int) stats->err_warn);	// количество заполнений буффера
	buf_ptr += (13 + MAX_DIGITS);
	buf_length += (13 + MAX_DIGITS);
	snprintf(buf_ptr, (13 + MAX_DIGITS + 1), "\tFrame    = %d\n", (int) stats->err_frame);	// количество ошибочных фреймов
	buf_ptr += (13 + MAX_DIGITS);
	buf_length += (13 + MAX_DIGITS);
	snprintf(buf_ptr, (13 + MAX_DIGITS + 1), "\tDroped   = %d\n", (int) stats->err_drop);	// количество потерянных при чтении фреймов
	buf_ptr += (13 + MAX_DIGITS);
	buf_length += (13 + MAX_DIGITS);
	snprintf(buf_ptr, (13 + MAX_DIGITS + 1), "\tLength   = %d\n", (int) stats->err_length);	// количество ошибок длины фреймов
	buf_ptr += (13 + MAX_DIGITS);
	buf_length += (13 + MAX_DIGITS);
	snprintf(buf_ptr, (13 + MAX_DIGITS + 1), "\tFIFO     = %d\n", (int) stats->err_fifo);	// количество ошибок fifo буффера
	buf_ptr += (13 + MAX_DIGITS);
	buf_length += (13 + MAX_DIGITS);

	snprintf(buf_ptr, (18 + 1), "\nRegisters state:\n");
	buf_ptr += 18;
	buf_length += 18;
	snprintf(buf_ptr, (11 + MAX_DIGITS + 1), "\tESR  = %#08x\n", (int) stats->reg_esr);	// состояние регистра esr
	buf_ptr += (11 + MAX_DIGITS);
	buf_length += (11 + MAX_DIGITS);
	snprintf(buf_ptr, (11 + MAX_DIGITS + 1), "\tMCR  = %#08x\n", (int) stats->reg_mcr);	// состояние регистра mcr
	buf_ptr += (11 + MAX_DIGITS);
	buf_length += (11 + MAX_DIGITS);
	snprintf(buf_ptr, (11 + MAX_DIGITS + 1), "\tCTRL = %#08x\n", (int) stats->reg_ctrl);	// состояние регистра ctrl
	buf_ptr += (11 + MAX_DIGITS);
	buf_length += (11 + MAX_DIGITS);

	snprintf(buf_ptr, (13 + MAX_DIGITS + 1), "\nFrequency = %d\n", (int) stats->freq);	// частота модуля
	buf_ptr += (13 + MAX_DIGITS);
	buf_length += (13 + MAX_DIGITS);

	*length = buf_length;

	return buf_msg;
}

// ##########################################################################################################################################
// ##########################################################################################################################################

static ssize_t flexcan_proc_read(struct file *filp, char *buf, size_t count, loff_t *ppos) // чтение из /proc/*my_dir*/*my_node* :
{
	struct inode *inode = filp->f_path.dentry->d_inode;
	const u8 dev_num = iminor(inode);
	struct flexcan_chardev* f_dev = &fimx6d.f_dev[dev_num];
	int buf_length = 0, ret = 0;

	char *buf_msg = get_rw_buf(dev_num, &buf_length);

	if(*ppos >= buf_length) 	{
		dev_dbg(&f_dev->dev, "%s.%d: %s eof\n", fimx6d.name, dev_num, __func__);
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
	struct flexcan_chardev* f_dev = &fimx6d.f_dev[dev_num];
	int ret;

	fimx6d.dev_proc_dir = create_proc_entry(NAME_DIR, S_IFDIR | S_IRWXUGO, NULL);
	if(NULL == fimx6d.dev_proc_dir) {
		ret = -ENOENT;
		dev_dbg(&f_dev->dev, "flexcan_proc_dir_init can't create directory /proc/%s\n", NAME_DIR);
		goto err_dir;
	}

	fimx6d.dev_proc_dir->uid = 0;
	fimx6d.dev_proc_dir->gid = 0;

	return 0;

 err_dir:
	return ret;
}


static int flexcan_proc_file_init(const u8 dev_num) 
{
	struct flexcan_chardev* f_dev = &fimx6d.f_dev[dev_num];
	int ret;

	f_dev->dev_proc_file = create_proc_entry(f_dev->name, (S_IFREG | S_IRUGO), fimx6d.dev_proc_dir);
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
	remove_proc_entry(fimx6d.f_dev[dev_num].name, fimx6d.dev_proc_dir);
}


static void flexcan_proc_dir_exit(void) 
{
	remove_proc_entry(NAME_DIR, NULL);
}

// ##########################################################################################################################################
// ##########################################################################################################################################

static void flexcan_transceiver_switch(const struct flexcan_platform_data *pdata, int on)
{
	if (pdata && pdata->transceiver_switch) {
		pdata->transceiver_switch(on);
	}
}

static void flexcan_gpio_switch(const struct flexcan_platform_data *pdata, int on)
{
//	if (pdata && pdata->gpio_switch) {
//		pdata->gpio_switch(on);
//	}
}

static inline int flexcan_has_and_handle_berr(const struct flexcan_stats *stats, u32 reg_esr)
{
	return (stats->ctrlmode & CAN_CTRLMODE_BERR_REPORTING) && (reg_esr & FLEXCAN_ESR_ERR_BUS);
}

static inline void flexcan_chip_enable(void __iomem *base)
{
	struct flexcan_regs __iomem *regs = base;
	u32 reg;

	reg = readl(&regs->mcr);
	reg &= ~FLEXCAN_MCR_MDIS;
	writel(reg, &regs->mcr);

	udelay(10);
}

static inline void flexcan_chip_disable(void __iomem *base)
{
	struct flexcan_regs __iomem *regs = base;
	u32 reg;

	reg = readl(&regs->mcr);
	reg |= FLEXCAN_MCR_MDIS;
	writel(reg, &regs->mcr);
}

static int flexcan_get_berr_counter(__u8 dev_num, struct can_berr_counter *bec)
{
	struct flexcan_regs __iomem *regs = fimx6d.f_dev[dev_num].f_base;
	u32 reg = readl(&regs->ecr);

	bec->txerr = (reg >> 0) & 0xff;
	bec->rxerr = (reg >> 8) & 0xff;

	return 0;
}

static unsigned int flexcan_start_transmit(const u8 dev_num)
{
	struct flexcan_chardev* f_dev = &fimx6d.f_dev[dev_num];
	struct flexcan_regs __iomem *regs = f_dev->f_base;
	struct flexcan_stats *stats = &f_dev->stats;
	struct can_frame cf;
	u32 can_id, ctrl, data;

	/* На будущее проверка статуса буфера в данный момент */
	// u8 mb_code = ((readl(&regs->cantxfg[FLEXCAN_TX_BUF_ID].can_ctrl) >> 24) & 0x0F);

	if(flexcan_send_buf_pop(dev_num, &cf) != 0) {
		dev_dbg(&f_dev->dev, "flexcan_start_transmit buffer free\n");
		return 0;
	}

	dev_dbg(&f_dev->dev, "flexcan_start_transmit send messages\n");
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
		writel(data, &regs->cantxfg[FLEXCAN_TX_BUF_ID].data[0]);
	}	
	if (cf.can_dlc > 3) {
		data = be32_to_cpup((__be32 *)&cf.data[4]);
		writel(data, &regs->cantxfg[FLEXCAN_TX_BUF_ID].data[1]);
	}

	writel(can_id, &regs->cantxfg[FLEXCAN_TX_BUF_ID].can_id);
	writel(ctrl, &regs->cantxfg[FLEXCAN_TX_BUF_ID].can_ctrl);
	/* tx_packets is incremented in flexcan_irq */
	stats->tx_bytes += cf.can_dlc;

	return 0;
}

static void do_bus_err(struct can_frame *cf, u32 reg_esr, const u8 dev_num)
{
	struct flexcan_chardev* f_dev = &fimx6d.f_dev[dev_num];
	struct flexcan_stats *stats = &f_dev->stats;
	int rx_errors = 0, tx_errors = 0;

	cf->can_id |= CAN_ERR_PROT | CAN_ERR_BUSERROR;
	if (reg_esr & FLEXCAN_ESR_BIT1_ERR) {
		dev_dbg(&f_dev->dev, "do_bus_err BIT1_ERR irq\n");
		cf->data[2] |= CAN_ERR_PROT_BIT1;
		tx_errors = 1;
	}
	if (reg_esr & FLEXCAN_ESR_BIT0_ERR) {
		dev_dbg(&f_dev->dev, "do_bus_err BIT0_ERR irq\n");
		cf->data[2] |= CAN_ERR_PROT_BIT0;
		tx_errors = 1;
	}
	if (reg_esr & FLEXCAN_ESR_ACK_ERR) {
		dev_dbg(&f_dev->dev, "do_bus_err ACK_ERR irq\n");
		cf->can_id |= CAN_ERR_ACK;
		cf->data[3] |= CAN_ERR_PROT_LOC_ACK;
		tx_errors = 1;
	}
	if (reg_esr & FLEXCAN_ESR_CRC_ERR) {
		dev_dbg(&f_dev->dev, "do_bus_err CRC_ERR irq\n");
		cf->data[2] |= CAN_ERR_PROT_BIT;
		cf->data[3] |= CAN_ERR_PROT_LOC_CRC_SEQ;
		rx_errors = 1;
	}
	if (reg_esr & FLEXCAN_ESR_FRM_ERR) {
		dev_dbg(&f_dev->dev, "do_bus_err FRM_ERR irq\n");
		cf->data[2] |= CAN_ERR_PROT_FORM;
		rx_errors = 1;
	}
	if (reg_esr & FLEXCAN_ESR_STF_ERR) {
		dev_dbg(&f_dev->dev, "do_bus_err STF_ERR irq\n");
		cf->data[2] |= CAN_ERR_PROT_STUFF;
		rx_errors = 1;
	}

	stats->dev_stats.bus_error++;
	if (rx_errors)	{
		stats->err_rx++;
	}
	if (tx_errors)	{
		stats->err_tx++;
	}
}

static void do_state(struct can_frame *cf, const u8 dev_num, enum can_state new_state)
{
	struct flexcan_chardev* f_dev = &fimx6d.f_dev[dev_num];
	struct flexcan_stats *stats = &f_dev->stats;
	struct can_berr_counter bec;

	dev_dbg(&f_dev->dev, "do_state get berr counter\n");
	flexcan_get_berr_counter(dev_num, &bec);

	switch (stats->state) {
		case CAN_STATE_ERROR_ACTIVE:
			/*
			 * from: ERROR_ACTIVE
			 * to  : ERROR_WARNING, ERROR_PASSIVE, BUS_OFF
			 * =>  : there was a warning int
			 */
			if (new_state >= CAN_STATE_ERROR_WARNING &&
			    new_state <= CAN_STATE_BUS_OFF) {
				dev_dbg(&f_dev->dev, "do_state Error Warning IRQ\n");
				stats->dev_stats.error_warning++;

				cf->can_id |= CAN_ERR_CRTL;
				cf->data[1] = (bec.txerr > bec.rxerr) ? CAN_ERR_CRTL_TX_WARNING : CAN_ERR_CRTL_RX_WARNING;
			}
		case CAN_STATE_ERROR_WARNING:	/* fallthrough */
			/*
			 * from: ERROR_ACTIVE, ERROR_WARNING
			 * to  : ERROR_PASSIVE, BUS_OFF
			 * =>  : error passive int
			 */
			if (new_state >= CAN_STATE_ERROR_PASSIVE &&
			    new_state <= CAN_STATE_BUS_OFF) {
				dev_dbg(&f_dev->dev, "do_state Error Passive IRQ\n");
				stats->dev_stats.error_passive++;

				cf->can_id |= CAN_ERR_CRTL;
				cf->data[1] = (bec.txerr > bec.rxerr) ? CAN_ERR_CRTL_TX_PASSIVE : CAN_ERR_CRTL_RX_PASSIVE;
			}
			break;
		case CAN_STATE_BUS_OFF:
			dev_dbg(&f_dev->dev, "do_state BUG! hardware recovered automatically from BUS_OFF\n");
			break;
		default:
			break;
	}

	/* process state changes depending on the new state */
	switch (new_state) {
		case CAN_STATE_ERROR_ACTIVE:
			dev_dbg(&f_dev->dev, "do_state Error Active\n");
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

// ##########################################################################################################################################
// ##########################################################################################################################################

static irqreturn_t flexcan_irq(int irq, void *dev_id)
{
	const u8 dev_num = (irq - FLEXCAN_IRQ_BASE);
	struct flexcan_chardev* f_dev = &fimx6d.f_dev[dev_num];
	struct flexcan_stats *stats = &f_dev->stats;
	struct flexcan_regs __iomem *regs = f_dev->f_base;
	struct flexcan_mb __iomem *mb = &regs->cantxfg[0];
	struct timeval time;
	u32 reg_iflag1, reg_esr;
	u32 read_frames = 0;

	// flexcan_transceiver_switch(f_dev->pdata, 1);
	reg_iflag1 = readl(&regs->iflag1);
	reg_esr = readl(&regs->esr);
	stats->int_num++;	/* Считаем прерывания */

	if(reg_esr & FLEXCAN_ESR_WAK_INT) {
		if (fimx6d.version >= FLEXCAN_VER_10_0_12) {
			mxc_iomux_set_gpr_register(13, 28, 1, 0);
		}
		writel(FLEXCAN_ESR_WAK_INT, &regs->esr);
	}

	if (reg_iflag1 & FLEXCAN_IFLAG_DEFAULT)	{
		if(reg_iflag1 & FLEXCAN_IFLAG_RX_FIFO_OVERFLOW) {
			stats->err_over++;
			stats->err_drop++;
		}
		else if(reg_iflag1 & FLEXCAN_IFLAG_RX_FIFO_WARN) {
			stats->err_warn++;
		}
		stats->int_rx_frame++;

		read_frames = 0;
		while((read_frames < 10) && (reg_iflag1 & FLEXCAN_IFLAG_RX_FIFO_AVAILABLE)) {	
			do_gettimeofday(&time);
			stats->rx_bytes += ((mb->can_ctrl >> 16) & 0xf);
			if(flexcan_recv_buf_push(dev_num, mb, &time)) { /* буффер заполнен */
				stats->err_drop++;
				stats->err_fifo++;
			}
			else {	/* все хорошо */
				stats->rx_frames++;
			}

			if(reg_iflag1 & FLEXCAN_IFLAG_RX_FIFO_OVERFLOW) {
				writel((FLEXCAN_IFLAG_DEFAULT), &regs->iflag1);
			}
			else if(reg_iflag1 & FLEXCAN_IFLAG_RX_FIFO_WARN) {
				writel((FLEXCAN_IFLAG_RX_FIFO_WARN | FLEXCAN_IFLAG_RX_FIFO_AVAILABLE), &regs->iflag1);
			}
			else if(reg_iflag1 & FLEXCAN_IFLAG_RX_FIFO_AVAILABLE) {
				writel(FLEXCAN_IFLAG_RX_FIFO_AVAILABLE, &regs->iflag1);
			}

			reg_iflag1 = readl(&regs->iflag1);
			readl(&regs->timer);
			read_frames++;
		}
	}

	if((reg_esr & FLEXCAN_ESR_ERR_STATE) || (flexcan_has_and_handle_berr(stats, reg_esr))) {
		stats->int_state++;

		stats->reg_esr = reg_esr & FLEXCAN_ESR_ERR_BUS;
		writel(reg_esr, &regs->esr);
		writel(f_dev->reg_ctrl_default, &regs->ctrl);			
	}

	if(reg_iflag1 & (1 << FLEXCAN_TX_BUF_ID)) {	/* transmission complete interrupt */
		stats->int_tx_frame++;	/* tx_bytes is incremented in flexcan_start_xmit */
		stats->tx_frames++;
		writel((1 << FLEXCAN_TX_BUF_ID), &regs->iflag1);
	}

	if(read_frames != 0) {	/* Будим читателей */
		wake_up_interruptible(&f_dev->inq); /* blocked in read() and select() */
		if(f_dev->async_queue) {	/* and signal asynchronous readers, explained late in chapter 5 */
			kill_fasync(&f_dev->async_queue, SIGIO, POLL_IN);
		}
	}

	return IRQ_HANDLED;
}

// ##########################################################################################################################################
// ##########################################################################################################################################

static void flexcan_set_bittiming(const u8 dev_num, const u32 reg_ctrl)
{
	struct flexcan_chardev* f_dev = &fimx6d.f_dev[dev_num];
	struct flexcan_stats *stats = &f_dev->stats;
	struct flexcan_regs __iomem *regs = f_dev->f_base;
	u32 reg;

	reg = readl(&regs->ctrl);
	reg &= ~(FLEXCAN_BTRT_MASK | FLEXCAN_CTRL_LPB | FLEXCAN_CTRL_SMP | FLEXCAN_CTRL_LOM);
	reg |= reg_ctrl;

	dev_dbg(&f_dev->dev, "flexcan_set_bittiming writing ctrl = %#08x\n", reg);
	writel(reg, &regs->ctrl);
	stats->reg_mcr = readl(&regs->mcr);
	stats->reg_ctrl = readl(&regs->ctrl);

	/* print chip status */
	dev_dbg(&f_dev->dev, "flexcan_set_bittiming mcr = %#08x ctrl = %#08x\n", readl(&regs->mcr), readl(&regs->ctrl));
}


static int flexcan_chip_start(const u8 dev_num)
{
	struct flexcan_chardev* f_dev = &fimx6d.f_dev[dev_num];
	struct flexcan_stats *stats = &f_dev->stats;
	struct flexcan_regs __iomem *regs = f_dev->f_base;
	u32 reg_mcr, reg_ctrl;
	int i, err;

	flexcan_chip_enable(f_dev->f_base);	/* enable module */

	writel(FLEXCAN_MCR_SOFTRST, &regs->mcr);	/* soft reset */
	udelay(10);

	reg_mcr = readl(&regs->mcr);
	if (reg_mcr & FLEXCAN_MCR_SOFTRST) {
		dev_dbg(&f_dev->dev, "flexcan_chip_start failed to softreset can module, mcr = %#08x\n", reg_mcr);
		err = -ENODEV;
		goto out;
	}

	flexcan_set_bittiming(dev_num, f_dev->reg_ctrl_bittiming);

	/* MCR
	 * enable freeze | enable fifo | halt now  | only supervisor access  | 
	 * enable warning int |  choose format C  | enable self wakeup
	 */
	reg_mcr = readl(&regs->mcr);
	reg_mcr |= FLEXCAN_MCR_FRZ | FLEXCAN_MCR_FEN | FLEXCAN_MCR_HALT | FLEXCAN_MCR_SUPV | 
		FLEXCAN_MCR_WRN_EN | FLEXCAN_MCR_IDAM_C | FLEXCAN_MCR_WAK_MSK | FLEXCAN_MCR_SLF_WAK;
	
	dev_dbg(&f_dev->dev, "flexcan_chip_start writing mcr = %#08x\n", reg_mcr);
	writel(reg_mcr, &regs->mcr);
	stats->reg_mcr = readl(&regs->mcr);
	/* CTRL
	 * disable timer sync feature | disable auto busoff recovery | transmit lowest buffer first
	 * enable tx and rx warning interrupt | enable bus off interrupt (== FLEXCAN_CTRL_ERR_STATE)
	 * _note_: we enable the "error interrupt" (FLEXCAN_CTRL_ERR_MSK), too. Otherwise we don't get any
	 * Otherwise we don't get any warning or bus passive interrupts.
	 */
	reg_ctrl = readl(&regs->ctrl);
	reg_ctrl &= ~FLEXCAN_CTRL_TSYN;
	// reg_ctrl |= /*FLEXCAN_CTRL_BOFF_REC | */ FLEXCAN_CTRL_LBUF |/*| FLEXCAN_CTRL_ERR_STATE | */FLEXCAN_CTRL_ERR_MSK ;
	reg_ctrl |= FLEXCAN_CTRL_BOFF_REC | FLEXCAN_CTRL_LBUF;

	f_dev->reg_ctrl_default = reg_ctrl;	/* save for later use */

	dev_dbg(&f_dev->dev, "flexcan_chip_start writing ctrl = %#08x\n", reg_ctrl);
	writel(reg_ctrl, &regs->ctrl);
	stats->reg_ctrl = readl(&regs->ctrl);

	for (i = 0; i < ARRAY_SIZE(regs->cantxfg); i++) {
		writel(0, &regs->cantxfg[i].can_ctrl);
		writel(0, &regs->cantxfg[i].can_id);
		writel(0, &regs->cantxfg[i].data[0]);
		writel(0, &regs->cantxfg[i].data[1]);
		/* put MB into rx queue */
		writel(FLEXCAN_MB_CNT_CODE(0x4), &regs->cantxfg[i].can_ctrl);
	}
	/* acceptance mask/acceptance code (accept everything) */
	writel(0x0, &regs->rxgmask);
	writel(0x0, &regs->rx14mask);
	writel(0x0, &regs->rx15mask);

	if(fimx6d.version >= FLEXCAN_VER_10_0_12) {	/* clear rx fifo global mask */
		writel(0x0, &regs->rxfgmask);
	}
	// flexcan_transceiver_switch(priv, 1);						//заменить priv  на pdata из структуры вызывающего устройства

	/* synchronize with the can bus */
	reg_mcr = readl(&regs->mcr);
	reg_mcr &= ~FLEXCAN_MCR_HALT;
	writel(reg_mcr, &regs->mcr);
	stats->reg_mcr = readl(&regs->mcr);
	stats->state = CAN_STATE_ERROR_ACTIVE;

	writel(FLEXCAN_IFLAG_DEFAULT, &regs->imask1);	/* enable FIFO interrupts */

	/* print chip status */
	dev_dbg(&f_dev->dev, "flexcan_chip_start reading mcr = %#08x ctrl = %#08x\n", readl(&regs->mcr), readl(&regs->ctrl));
	flexcan_proc_file_init(dev_num);

	return 0;

 out:
 	dev_dbg(&f_dev->dev, "flexcan_chip_start chip disable\n");
	flexcan_chip_disable(f_dev->f_base);
	return err;
}


static void flexcan_chip_stop(const u8 dev_num)
{
	struct flexcan_chardev* f_dev = &fimx6d.f_dev[dev_num];
	struct flexcan_stats *stats = &f_dev->stats;
	struct flexcan_regs __iomem *regs = f_dev->f_base;
	u32 reg;
	
	regs = f_dev->f_base;
	flexcan_proc_file_exit(dev_num);

	writel(0, &regs->imask1);	/* Disable all interrupts */

	/* Disable + halt module */
	reg = readl(&regs->mcr);
	reg |= FLEXCAN_MCR_MDIS | FLEXCAN_MCR_HALT;
	writel(reg, &regs->mcr);
	stats->reg_mcr = readl(&regs->mcr);

	// flexcan_transceiver_switch(priv, 0);	//заменить priv  на pdata из структуры вызывающего устройства
	stats->state = CAN_STATE_STOPPED;
}


static int flexcan_net_open(const u8 dev_num)
{
	struct flexcan_chardev* f_dev = &fimx6d.f_dev[dev_num];
	unsigned int irq = f_dev->irq_num;
	int err;

	clk_enable(f_dev->clk);

	err = request_irq(irq, flexcan_irq, IRQF_SHARED, f_dev->name, f_dev->pdata);
	if(err)	{
		dev_dbg(&f_dev->dev, "flexcan_net_open request irq %d failed with err = %d\n", irq, err);
		goto out;
	}

	err = flexcan_chip_start(dev_num);	/* start chip and queuing */
	if(err)	{
		goto out;
	}

	return 0;

 out:
 	dev_dbg(&f_dev->dev, "flexcan_net_open clk disable\n");
	clk_disable(f_dev->clk);

	return err;
}


static int flexcan_net_close(const u8 dev_num)
{
	struct flexcan_chardev* f_dev = &fimx6d.f_dev[dev_num];
	unsigned int irq = f_dev->irq_num;

	flexcan_chip_stop(dev_num);

	free_irq(irq, f_dev->pdata);

	clk_disable(f_dev->clk);

	return 0;
}


static int flexcan_set_mode(__u8 dev_num, enum can_mode mode)
{
	int err = 0;

	switch (mode) {
	case CAN_MODE_START:
		err = flexcan_chip_start(dev_num);
		if (err)	{
			return err;
		}
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}


static int __devinit register_flexcandev(const u8 dev_num)
{
	struct flexcan_chardev* f_dev = &fimx6d.f_dev[dev_num];
	struct flexcan_regs __iomem *regs = f_dev->f_base;
	u32 reg, err;

	clk_enable(f_dev->clk);

	/* select "bus clock", chip must be disabled */
	flexcan_chip_disable(f_dev->f_base);
	reg = readl(&regs->ctrl);
	reg |= FLEXCAN_CTRL_CLK_SRC;
	writel(reg, &regs->ctrl);

	flexcan_chip_enable(f_dev->f_base);
	/* set freeze, halt and activate FIFO, restrict register access */
	reg = readl(&regs->mcr);
	reg |= FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT | FLEXCAN_MCR_FEN | FLEXCAN_MCR_SUPV;
	writel(reg, &regs->mcr);

	/* Currently we only support newer versions of this core
	 * featuring a RX FIFO. Older cores found on some Coldfire
	 * derivates are not yet supported.
	 */
	reg = readl(&regs->mcr);
	if (!(reg & FLEXCAN_MCR_FEN)) {
		dev_dbg(&f_dev->dev, "register_flexcandev could not enable RX FIFO, unsupported core\n");
		err = -ENODEV;
		goto out;
	}

 out:
	/* disable core and turn off clocks */
 	flexcan_chip_disable(f_dev->f_base);
	clk_disable(f_dev->clk);

	return err;
}


static struct platform_device_id flexcan_devtype[] = {
	{
		.name = "imx25-flexcan",
		.driver_data = FLEXCAN_VER_3_0_0,
	}, {
		.name = "imx28-flexcan",
		.driver_data = FLEXCAN_VER_3_0_4,
	}, {
		.name = "imx35-flexcan",
		.driver_data = FLEXCAN_VER_3_0_0,
	}, {
		.name = "imx53-flexcan",
		.driver_data = FLEXCAN_VER_3_0_0,
	}, {
		.name = "imx6q-flexcan",
		.driver_data = FLEXCAN_VER_10_0_12,
	},
};


static int __devinit flexcan_probe(struct platform_device *pdev)
{
	const u8 dev_num = pdev->id;
	struct flexcan_chardev* f_dev = &fimx6d.f_dev[dev_num];
	char name0[IFNAMSIZ] = "can0", name1[IFNAMSIZ] = "can1";
	resource_size_t mem_size;
	struct resource *mem;
	struct clk *clk;
	void __iomem *base;
	int err, irq, major;

	printk("%s driver ver %s by Strim-tech\n", FLEXCAN_DRV_NAME, FLEXCAN_DRV_VER);

	/* pdev->id 	- ID устройства (0 или 1)
	 * dev->irq 	- IRQ number
	 * dev->name 	- Имя устройства (can0 или can1)
	 * pdev->id_entry->name - Имя родителя (imx6d-flexcan)
	 */

	clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "no clock defined\n");	
		err = PTR_ERR(clk);
		goto failed_clock;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	if (!mem || irq <= 0) {
		err = -ENODEV;
		goto failed_get;
	}

	mem_size = resource_size(mem);
	if (!request_mem_region(mem->start, mem_size, pdev->name)) {
		err = -EBUSY;
		goto failed_get;
	}

	base = ioremap(mem->start, mem_size);
	if (!base) {
		err = -ENOMEM;
		goto failed_map;
	}

	flexcan_f_dev_init(f_dev);
	f_dev->irq_num = irq;
	f_dev->stats.freq = clk_get_rate(clk);
	f_dev->stats.bittiming_const = &flexcan_bittiming_const;
	f_dev->do_set_mode = flexcan_set_mode;
	f_dev->do_get_berr_counter = flexcan_get_berr_counter;
	f_dev->stats.ctrlmode_supported = CAN_CTRLMODE_LOOPBACK | CAN_CTRLMODE_LISTENONLY | CAN_CTRLMODE_3_SAMPLES | CAN_CTRLMODE_BERR_REPORTING;
	f_dev->f_base = base;
	f_dev->dev = pdev->dev;
	f_dev->clk = clk;
	f_dev->pdata = pdev->dev.platform_data;
	fimx6d.version = pdev->id_entry->driver_data;
	memcpy(fimx6d.name, pdev->id_entry->name, sizeof(fimx6d.name));
	
    if(dev_num == 0)	{
		memcpy(f_dev->name, name0, sizeof(f_dev->name));
    }
    else if(dev_num == 1) 	{
    	memcpy(f_dev->name, name1, sizeof(f_dev->name));
    }

	if(!fimx6d.is_init)	{
		err = alloc_chrdev_region(&fimx6d.f_devt, FLEXCAN_DEV_FIRST, FLEXCAN_DEV_COUNT, fimx6d.name);
		if (err < 0) {
			err = -1;
			dev_dbg(&f_dev->dev, "flexcan_probe alloc chrdev region failed\n");
			goto err_reg_chrdev;
		} 
	}
	major = MAJOR(fimx6d.f_devt);
	dev_dbg(&f_dev->dev, "flexcan_probe major = %d, dev_num = %d\n",major, dev_num);	

	f_dev->f_devt = MKDEV(major, dev_num);  
			
	cdev_init(&f_dev->f_cdev, &flexcan_fops);
	f_dev->f_cdev.owner = THIS_MODULE;
	f_dev->f_cdev.ops = &flexcan_fops;

	err = cdev_add(&f_dev->f_cdev, f_dev->f_devt, 1);
	if(err)	{
		err = -1;
		dev_dbg(&f_dev->dev, "flexcan_probe c_dev add error\n");
		goto err_cdev_add;
	}

	if(!fimx6d.is_init)	{
	    if ((fimx6d.f_class = class_create(THIS_MODULE, fimx6d.name)) == NULL)	{
			err = -1;
			dev_dbg(&f_dev->dev, "flexcan_probe class create error\n");
			goto err_class_create;
		}
		flexcan_proc_dir_init(dev_num);
	}

	if (device_create(fimx6d.f_class, &pdev->dev, f_dev->f_devt, NULL, f_dev->name) == NULL)	{
		err = -1;
		dev_dbg(&f_dev->dev, "flexcan_probe device create error\n");
		goto err_device_create;
	}

	if(!fimx6d.is_init)	{
    	fimx6d.is_init = 1;
    }

    flexcan_net_open(dev_num);

	return 0;

 err_device_create:
 	if(!fimx6d.is_init)	{
		class_destroy(fimx6d.f_class);
	}
 err_class_create:
 err_cdev_add:	
 	if(!fimx6d.is_init)	{
 		unregister_chrdev_region(f_dev->f_devt, FLEXCAN_DEV_COUNT);
 	}
 err_reg_chrdev:
	iounmap(base);
 failed_map:
	release_mem_region(mem->start, mem_size);
 failed_get:
	clk_put(clk);
 failed_clock:
	return err;
}


static int __devexit flexcan_remove(struct platform_device *pdev)
{
	struct resource *mem;
	const u8 dev_num = pdev->id;
	struct flexcan_chardev* f_dev = &fimx6d.f_dev[dev_num];

	flexcan_net_close(dev_num);

	// flexcan_proc_file_exit(dev_num);
	device_destroy(fimx6d.f_class, f_dev->f_devt);
	if(!fimx6d.is_init)	{
		flexcan_proc_dir_exit();
		class_destroy(fimx6d.f_class);
	}

	cdev_del(&f_dev->f_cdev);
	if(!fimx6d.is_init)	{
		unregister_chrdev_region(f_dev->f_devt, FLEXCAN_DEV_COUNT);
		// flexcan_proc_dir_exit();	/* нужно перенести, но куда?..... */
	}

	if(fimx6d.is_init)	{
    	fimx6d.is_init = 0;
    }

	platform_set_drvdata(pdev, NULL);
	iounmap(f_dev->f_base);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(mem->start, resource_size(mem));

	clk_put(f_dev->clk);

	return 0;
}


#ifdef CONFIG_PM
static int flexcan_suspend(struct platform_device *pdev, pm_message_t state)
{
	const u8 dev_num = pdev->id;
	struct flexcan_chardev* f_dev = &fimx6d.f_dev[dev_num];
	int ret;

	f_dev->stats.state = CAN_STATE_SLEEPING;
	if(fimx6d.version >= FLEXCAN_VER_10_0_12) {	/* enable stop request for wakeup */
		mxc_iomux_set_gpr_register(13, 28, 1, 1);
	}

	ret = irq_set_irq_wake(f_dev->irq_num, 1);
	if(ret)	{
		return ret;
	}

	return 0;
}


static int flexcan_resume(struct platform_device *pdev)
{
	const u8 dev_num = pdev->id;
	struct flexcan_chardev* f_dev = &fimx6d.f_dev[dev_num];
	int ret;

	ret = irq_set_irq_wake(f_dev->irq_num, 0);
	if(ret)	{
		return ret;
	}
	f_dev->stats.state = CAN_STATE_ERROR_ACTIVE;

	return 0;
}
#else
#define flexcan_suspend NULL
#define flexcan_resume NULL
#endif


static struct platform_driver flexcan_driver = {
	.driver.name = 	FLEXCAN_DRV_NAME,
//	.probe = 		flexcan_probe,
	.id_table = 	flexcan_devtype,
//	.remove = 		__devexit_p(flexcan_remove),
	.suspend = 		flexcan_suspend,
	.resume = 		flexcan_resume,
};


static int __init flexcan_init(void)
{
	return platform_driver_register(&flexcan_driver);
}


static void __exit flexcan_exit(void)
{
	platform_driver_unregister(&flexcan_driver);
}

module_init(flexcan_init);
module_exit(flexcan_exit);

MODULE_AUTHOR("Sascha Hauer <kernel@pengutronix.de>, "
	      "Marc Kleine-Budde <kernel@pengutronix.de>, "
	      "Mikita Dzivakou <grommerin@gmail.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("CAN port driver (with file operations) for flexcan based chip");
