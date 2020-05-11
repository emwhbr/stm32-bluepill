#ifndef __IP_H__
#define __IP_H__

#include <stdint.h>

/////////////////////////////////////////////////////////////

// Ethernet
#define ETH_ADDR_LEN  6
typedef struct
{
   uint8_t  daddr[ETH_ADDR_LEN];
   uint8_t  saddr[ETH_ADDR_LEN];
   uint16_t type;
} __attribute__ ((__packed__)) ETH_HEADER;

#define ETH_TYPE_IP  0x0800

/////////////////////////////////////////////////////////////

// IP
typedef struct
{
#if 1 // little-endian
   uint8_t  ihl : 4;
   uint8_t  version : 4;
#else
   uint8_t  version : 4;
   uint8_t  ihl : 4;
#endif
   uint8_t  tos;
   uint16_t len;
   uint16_t id;
   uint16_t flags : 3;
   uint16_t frag_offset : 13;
   uint8_t  ttl;
   uint8_t  proto;
   uint16_t chksum;
   uint32_t saddr;
   uint32_t daddr;
} __attribute__((packed)) IPV4_HEADER;

#define IP_PROTO_ICMP   1
#define IP_PROTO_TCP    6
#define IP_PROTO_UDP   17

typedef uint32_t IPV4_ADDR;
#define TO_IPV4_ADDR(a, b, c, d) (((uint32_t) (a) << 24) | ((b) << 16) | ((c) << 8) | (d))

/////////////////////////////////////////////////////////////

// UDP
typedef struct
{
   uint16_t sport;
   uint16_t dport;
   uint16_t len;
   uint16_t chksum;
} __attribute__((packed)) UDP_HEADER;

typedef struct
{
   UDP_HEADER udp_h;
   uint32_t   udp_data;
} __attribute__((packed)) UDP_DGRAM;

/////////////////////////////////////////////////////////////

// Putting it all together
typedef struct
{
   ETH_HEADER  eth_h;
   IPV4_HEADER ipv4_h;
   UDP_DGRAM   udp_dgram;
} __attribute__((packed)) ETH_PACKET;

/////////////////////////////////////////////////////////////

// Misc
#define htons(n) (((((uint16_t)(n) & 0x00ff)) << 8) | \
                   (((uint16_t)(n) & 0xff00) >> 8))

#define ntohs(n) htons(n)

#define htonl(n) (((((uint32_t)(n) & 0x000000ff)) << 24) | \
                  ((((uint32_t)(n) & 0x0000ff00)) << 8) | \
                  ((((uint32_t)(n) & 0x00ff0000)) >> 8) | \
                  ((((uint32_t)(n) & 0xff000000)) >> 24))

#define ntohl(n) htonl(n)

#endif
