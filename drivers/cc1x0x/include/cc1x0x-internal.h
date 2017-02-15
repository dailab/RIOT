/*
 * Copyright (C) 2015 Kaspar Schleiser <kaspar@schleiser.de>
 * Copyright (C) 2014 Freie Universität Berlin
 * Copyright (C) 2013 INRIA
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_cc1x0x
 * @{
 *
 * @file
 * @brief       Data structures and variables for the cc1x0x driver interface
 *
 * @author      Oliver Hahm <oliver.hahm@inria.fr>
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 */

#ifndef CC1X0X_INTERNAL_H
#define CC1X0X_INTERNAL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CC1X0X_RXBUF_SIZE           (2)
#define CC1X0X_MAX_DATA_LENGTH      (58+64)

#define CC1X0X_HEADER_LENGTH        (3)     /**< Header covers SRC, DST and
                                                 FLAGS */
#define CC1X0X_BROADCAST_ADDRESS    (0x00)  /**< CC1X0X broadcast address */

#define MIN_UID                     (0x01)  /**< Minimum UID of a node is
                                                 1 */
#define MAX_UID                     (0xFF)  /**< Maximum UID of a node is
                                                 255 */

#define MIN_CHANNR                  (0)     /**< Minimum channel number */
#define MAX_CHANNR                  (24)    /**< Maximum channel number */

#define CC1X0X_PACKET_LENGTH        (0xFF)  /**< max packet length = 255b */
#define CC1X0X_SYNC_WORD_TX_TIME    (90000) /**< loop count (max. timeout ~15ms)
                                                 to wait for sync word to be
                                                 transmitted (GDO2 from low to
                                                 high) */

#define RESET_WAIT_TIME             (610)   /**< Reset wait time (in reset
                                                 procedure) */
#define IDLE_TO_RX_TIME             (122)   /**< Time chip needs to go to RX */
#define CS_SO_WAIT_TIME             (488)   /**< Time to wait for SO to go low
                                                 after CS */
#define CC1X0X_GDO1_LOW_RETRY       (100)   /**< Max. retries for SO to go low
                                                 after CS */
#ifndef CC1X0X_DEFAULT_CHANNEL
#define CC1X0X_DEFAULT_CHANNEL      (0)     /**< The default channel number */
#endif
#define CC1X0X_MIN_CHANNR           (0)     /**< lowest possible channel number */
#define CC1X0X_MAX_CHANNR           (0)     /**< highest possible channel number */

/**
 * @name    State values for state machine
 * @{
 */
enum {
    RADIO_UNKNOWN,
    RADIO_IDLE,
    RADIO_TX_BUSY,
    RADIO_RX,
    RADIO_RX_BUSY,
    RADIO_PWD,
};
/** @} */

/**
 * @brief array holding cc1x0x register values
 */
extern char cc1x0x_conf[];

/**
 * @brief   CC1X0X layer 0 protocol
 *
 * <pre>
---------------------------------------------------
|        |         |         |       |            |
| Length | Address | PhySrc  | Flags |    Data    |
|        |         |         |       |            |
---------------------------------------------------
  1 byte   1 byte    1 byte   1 byte   <= 251 bytes

Flags:
        Bit | Meaning
        --------------------
        7:4 | -
        3:1 | Protocol
          0 | Identification
</pre>
Notes:
\li length & address are given by CC1X0X
\li Identification is increased is used to scan duplicates. It must be increased
    for each new packet and kept for packet retransmissions.
 */
typedef struct __attribute__((packed))
{
    uint8_t length;                         /**< Length of the packet (without length byte) */
    uint8_t address;                        /**< Destination address */
    uint8_t phy_src;                        /**< Source address (physical source) */
    uint8_t flags;                          /**< Flags */
    uint8_t data[CC1X0X_MAX_DATA_LENGTH];   /**< Data (high layer protocol) */
} cc1x0x_pkt_t;

/**
 * @brief struct holding cc1x0x packet + metadata
 */
typedef struct {
    uint8_t rssi;                           /**< RSSI value */
    uint8_t lqi;                            /**< link quality indicator */
    uint8_t pos;                            /**< I have no clue. */
    cc1x0x_pkt_t packet;                    /**< whole packet */
} cc1x0x_pkt_buf_t;

/**
 * @brief enum for holding cc1x0x radio on/off state */
enum cc1x0x_radio_mode {
    RADIO_MODE_GET  = -1,                   /**< leave mode unchanged */
    RADIO_MODE_OFF  = 0,                    /**< turn radio off */
    RADIO_MODE_ON   = 1                     /**< turn radio on */
};

/**
 * @brief   CC110x register configuration
 */
typedef struct {
    uint8_t _IOCFG2;      /**< GDO2 output pin configuration */
    uint8_t _IOCFG1;      /**< GDO1 output pin configuration */
    uint8_t _IOCFG0;      /**< GDO0 output pin configuration */
    uint8_t _FIFOTHR;     /**< RX FIFO and TX FIFO thresholds */
    uint8_t _SYNC1;       /**< Sync word, high byte */
    uint8_t _SYNC0;       /**< Sync word, low byte */
    uint8_t _PKTLEN;      /**< Packet length */
    uint8_t _PKTCTRL1;    /**< Packet automation control */
    uint8_t _PKTCTRL0;    /**< Packet automation control */
    uint8_t _ADDR;        /**< Device address */
    uint8_t _CHANNR;      /**< Channel number */
    uint8_t _FSCTRL1;     /**< Frequency synthesizer control */
    uint8_t _FSCTRL0;     /**< Frequency synthesizer control */
    uint8_t _FREQ2;       /**< Frequency control word, high byte */
    uint8_t _FREQ1;       /**< Frequency control word, middle byte */
    uint8_t _FREQ0;       /**< Frequency control word, low byte */
    uint8_t _MDMCFG4;     /**< Modem configuration */
    uint8_t _MDMCFG3;     /**< Modem configuration */
    uint8_t _MDMCFG2;     /**< Modem configuration */
    uint8_t _MDMCFG1;     /**< Modem configuration */
    uint8_t _MDMCFG0;     /**< Modem configuration */
    uint8_t _DEVIATN;     /**< Modem deviation setting */
    uint8_t _MCSM2;       /**< Main Radio Control State Machine configuration */
    uint8_t _MCSM1;       /**< Main Radio Control State Machine configuration */
    uint8_t _MCSM0;       /**< Main Radio Control State Machine configuration */
    uint8_t _FOCCFG;      /**< Frequency Offset Compensation configuration */
    uint8_t _BSCFG;       /**< Bit Synchronization configuration */
    uint8_t _AGCCTRL2;    /**< AGC control */
    uint8_t _AGCCTRL1;    /**< AGC control */
    uint8_t _AGCCTRL0;    /**< AGC control */
    uint8_t _WOREVT1;     /**< High byte Event 0 timeout */
    uint8_t _WOREVT0;     /**< Low byte Event 0 timeout */
    uint8_t _WORCTRL;     /**< Wake On Radio control */
    uint8_t _FREND1;      /**< Front end RX configuration */
    uint8_t _FREND0;      /**< Front end TX configuration */
    uint8_t _FSCAL3;      /**< Frequency synthesizer calibration */
    uint8_t _FSCAL2;      /**< Frequency synthesizer calibration */
    uint8_t _FSCAL1;      /**< Frequency synthesizer calibration */
    uint8_t _FSCAL0;      /**< Frequency synthesizer calibration */
} cc1x0x_reg_t;

/**
 * @brief   CC110x radio configuration
 */
typedef struct {
    cc1x0x_reg_t reg_cfg;       /**< CC1X0X register configuration */
    uint8_t pa_power;           /**< Output power setting */
} cc1x0x_cfg_t;

/**
 * @brief   Radio Control Flags
 */
typedef struct {
    uint8_t  _RSSI;             /**< The RSSI value of last received packet */
    uint8_t  _LQI;              /**< The LQI value of the last received packet */
} cc1x0x_flags_t;

/**
 * @brief   Statistic interface for debugging
 */
typedef struct cc1x0x_statistic {
    uint32_t    packets_in;             /**< total nr of packets received */
    uint32_t    packets_in_crc_fail;    /**< dropped because of invalid crc */
    uint32_t    packets_in_while_tx;    /**< receive while tx */
    uint32_t    raw_packets_out;        /**< packets sent */
} cc1x0x_statistic_t;

#ifdef __cplusplus
}
#endif

/** @} */
#endif /* CC1X0X_INTERNAL_H */
