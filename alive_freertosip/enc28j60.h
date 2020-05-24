#ifndef __ENC28J60_H__
#define __ENC28J60_H__

#include <stdint.h>

#include <FreeRTOS_IP.h>

typedef enum
{
   ENC_OK,   // code indicating success (no error)
   ENC_FAIL  // generic code indicating failure
} enc_err_t;

enc_err_t enc28j60_init(const uint8_t mac_addr[ipMAC_ADDRESS_LENGTH_BYTES],
                        UBaseType_t int_work_task_prio,
                        uint16_t eth_max_frame_size);

int enc28j60_get_phy_link_status(void);

enc_err_t enc28j60_send_packet(const uint8_t *buf, size_t len);

#endif
