#ifndef __ENC28J60_H__
#define __ENC28J60_H__

#include <stdint.h>
#include <stddef.h>

int enc28j60_init(void);

void enc28j60_test_send_packet(const uint8_t *buf, size_t len);

#endif
