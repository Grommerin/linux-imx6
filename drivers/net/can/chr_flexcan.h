#ifndef FLEXCAN_H
#define FLEXCAN_H

#include <linux/ioctl.h>
#include <linux/types.h>

#include <linux/can/netlink.h>
#include <linux/can/error.h>

#define SET_DRIVER_CLEAR            (0)             /* For init setting variable */
#define SET_DRIVER_DEBUG_MASK       0x000000f0      /* For check set debug in message */
#define SET_DRIVER_DEBUG_ON         (0xC << 4)      /* Turn on debug messages in griver */
#define SET_DRIVER_DEBUG_OFF        (0x0 << 4)      /* Turn off debug messages in griver */
#define SET_DRIVER_DEBUG_L0         (0x8 << 4)      /* Set level of debug messages as 0, few reports */
#define SET_DRIVER_DEBUG_L1         (0x1 << 4)      /* Set level of debug messages as 1, average reports */
#define SET_DRIVER_DEBUG_L2         (0x2 << 4)      /* Set level of debug messages as 2, many reports */
#define SET_DRIVER_DEBUG_L3         (0x3 << 4)      /* Set level of debug messages as 3, all reports */

#define SET_CAN_BITRATE_MASK        0x00000f00      /* For check set bitrate in message */
#define SET_CAN_BITRATE_1000        (0x1 << 8)      /* Set bitrate 1000 */
#define SET_CAN_BITRATE_500         (0x2 << 8)      /* Set bitrate 500 */
#define SET_CAN_BITRATE_250         (0x3 << 8)      /* Set bitrate 250 */
#define SET_CAN_BITRATE_125         (0x4 << 8)      /* Set bitrate 125 */
#define SET_CAN_BITRATE_100         (0x5 << 8)      /* Set bitrate 100 */

#define SET_CAN_MODE_MASK           0x0000f000      /* For check set control mode in message */
#define SET_CAN_MODE_LOOPBACK       (0x1 << 12)     /* Set bitrate 1000 */
#define SET_CAN_MODE_LISTENONLY     (0x2 << 12)     /* Set bitrate 500 */
#define SET_CAN_MODE_3_SAMPLES      (0x3 << 12)     /* Set bitrate 250 */
#define SET_CAN_MODE_ONE_SHOT       (0x4 << 12)     /* Set bitrate 125 */
#define SET_CAN_MODE_BERR_REPORTING (0x5 << 12)     /* Set bitrate 100 */

/* controller area network (CAN) kernel definitions */

/* fixed length of message with CAN data */
#define CAN_DATA_MSG_LENGTH         (50)

/* first char in CAN message */
#define CAN_DATA_MSG_CHAR_START     '$'

/*
 * struct can_message - can message format structure
 *
 * $CAN,0,55f5a4d3,4f537,8000F00F,8,0807060504030201\n
 *
 * @mess_head:	head os message 
 * @comma_%d:   separators 
 * @dev_num:    device number 
 * @time_sec:   message time (sec) 
 * @time_usec:  message time (usec) 
 * @can_id:     can ID 
 * @can_dlc:    data length 
 * @can_data:   can data 
 * @mess_end:   end of message 
 */
struct can_message {
	char mess_head[4];	/* "$CAN" */
	char comma_0[1];	/* "," */
	char dev_num[1];	/* %d */
	char comma_1[1];	/* "," */
	char time_sec[8];	/* %08x */
	char comma_2[1];	/* "," */
	char time_usec[5];	/* %05x */
	char comma_3[1];	/* "," */
	char can_id[8]; 	/* %08x */
	char comma_4[1];	/* "," */
	char can_dlc[1];	/* %d */
	char comma_5[1];	/* "," */
	char can_data[16];	/* %016llx */
	char mess_end[1];	/* "\n" */
};



/**
 * Controller Area Network Identifier structure
 *
 * bit 0-28: CAN identifier (11/29 bit)
 * bit 29: error frame flag (0 = data frame, 1 = error frame)
 * bit 30: remote transmission request flag (1 = rtr frame)
 * bit 31: frame format flag (0 = standard 11 bit, 1 = extended 29 bit)
 */
#define CANID_T_
typedef __u32 canid_t;

/* special address description flags for the CAN_ID */
#define CAN_EFF_FLAG 0x80000000U /* EFF/SFF is set in the MSB */
#define CAN_RTR_FLAG 0x40000000U /* remote transmission request */
#define CAN_ERR_FLAG 0x20000000U /* error frame */

/* valid bits in CAN ID for frame formats */
#define CAN_SFF_MASK 0x000007FFU /* standard frame format (SFF) */
#define CAN_EFF_MASK 0x1FFFFFFFU /* extended frame format (EFF) */
#define CAN_ERR_MASK 0x1FFFFFFFU /* omit EFF, RTR, ERR flags */

/**
 * struct can_frame - basic CAN frame structure
 * @can_id:  the CAN ID of the frame and CAN_*_FLAG flags, see above.
 * @can_dlc: the data length field of the CAN frame
 * @data:    the CAN frame payload.
 */
struct can_frame {
	canid_t can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
	__u8    can_dlc; /* data length code: 0 .. 8 */
	__u8    data[8] __attribute__((aligned(8)));
};

/**
 * struct can_filter - CAN ID based filter in can_register().
 *
 * @can_id:   relevant bits of CAN ID which are not masked out.
 * @can_mask: CAN mask (see description)
 *
 * Description:
 * A filter matches, when
 *          <received_can_id> & mask == can_id & mask
 *
 * The filter can be inverted (CAN_INV_FILTER bit set in can_id) or it can
 * filter for error frames (CAN_ERR_FLAG bit set in mask).
 */
struct can_filter {
	canid_t can_id;
	canid_t can_mask;
};

enum can_mode {
	CAN_MODE_STOP = 0,
	CAN_MODE_START,
	CAN_MODE_SLEEP
};

struct flexcan_mb {
	__u32 can_ctrl;
	__u32 can_id;
	__u32 data[2];
};

struct flexcan_frame_cf {
	struct can_frame cf;
	struct timeval time;
};

struct flexcan_stats {
	__u32 rx_frames;		// количество принятых фреймов
	__u32 tx_frames;		// количество отправленных фреймов
	/* возможно стоит предусмотреть увеличение счетчиков байт данных т.к. может переполниться за 8 часов */
	__u32 rx_bytes;			// количество принятых байт данных
	__u32 tx_bytes;			// количество отправленных байт данных

	__u32 int_wak;			// количество прерываний пробуждения
	__u32 int_state;		// количество прерываний проверки состояния
	__u32 int_rx_frame;		// количество прерываний чтения фреймов
	__u32 int_tx_frame;		// количество прерываний отправки фреймов
	__u32 int_num;			// количество прерываний

	__u32 err_tx;			// количество ошибок отправки фреймов
	__u32 err_rx;			// количество ошибок принятия фреймов
	__u32 err_over;			// количество переполнений буффера
	__u32 err_warn;			// количество заполнений буффера
	__u32 err_frame;		// количество ошибочных фреймов
	__u32 err_drop;			// количество потерянных при чтении фреймов
	__u32 err_length;		// количество ошибок длины фреймов
	__u32 err_fifo;			// количество ошибок fifo буффера

	__u32 reg_esr;			// состояние регистра esr
	__u32 reg_mcr;			// состояние регистра mcr
	__u32 reg_ctrl;			// состояние регистра ctrl

	__u32 freq;				// частота модуля

	__u32 ctrlmode;
	__u32 ctrlmode_supported;

	struct can_device_stats dev_stats;

	enum can_state state;
	struct can_bittiming_const *bittiming_const;
};

/* ioctl commands magic numder */
#define FLEXCAN_IOC_MAGIC		0x81

/* supported ioctl commands */
//#define FLEXCAN_IOCTL_CMD 	_IO(FLEXCAN_IOC_MAGIC, 0x00)
#define FLEXCAN_IOCTL_READ 		_IOR(FLEXCAN_IOC_MAGIC, 0x00, int *)
#define FLEXCAN_IOCTL_WRITE 	_IOW(FLEXCAN_IOC_MAGIC, 0x01, int *)
#define FLEXCAN_IOCTL_WR_RD		_IOWR(FLEXCAN_IOC_MAGIC, 0x02, int *)

#endif
