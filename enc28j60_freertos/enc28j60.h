#ifndef __ENC28J60_H__
#define __ENC28J60_H__

#include <stdint.h>
#include <stddef.h>

#define ENC28J60_ETH_MAX_FRAME_SIZE  1518

void enc28j60_init(void);

void enc28j60_test_send_packet(const uint8_t *buf, size_t len);

void enc28j60_test_recv_packet(uint8_t *buf, size_t len, size_t *actual);

#endif
