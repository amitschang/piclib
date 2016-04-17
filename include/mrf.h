#include <xc.h>
#include "mrf24j40.h"

#define MRF_SADDR_LEN 2
#define MRF_LADDR_LEN 8
#define MRF_SADDR MRF_SADDR_LEN
#define MRF_LADDR MRF_LADDR_LEN
#define MRF_PANID_LEN 2
#define MRF_MAX_DLEN  125 // max payload 128 - 3 (min header)
#define MRF_SEC 0b00001000
#define MRF_ACK 0b00100000
#define MRF_FLIP_ADDR 0x01
#define MRF_DATA_TYPE 1
#define MRF_INT_RISING 1
#define MRF_INT_FALLING 0
#define MRF_INT_TXN  0b00000001
#define MRF_INT_TXG1 0b00000010
#define MRF_INT_TXG2 0b00000100
#define MRF_INT_RX   0b00001000
#define MRF_INT_SEC  0b00010000
#define MRF_INT_HSYM 0b00100000
#define MRF_INT_WAKE 0b01000000
#define MRF_INT_SLP  0b10000000
#define MRF_INT_ALL  0xFF;
#define MRF_CHANNEL_11 0x03
#define MRF_CHANNEL_12 0x13
#define MRF_CHANNEL_13 0x23
#define MRF_CHANNEL_14 0x33
#define MRF_CHANNEL_15 0x43
#define MRF_CHANNEL_16 0x53
#define MRF_CHANNEL_17 0x63
#define MRF_CHANNEL_18 0x73
#define MRF_CHANNEL_19 0x83
#define MRF_CHANNEL_20 0x93
#define MRF_CHANNEL_21 0xA3
#define MRF_CHANNEL_22 0xB3
#define MRF_CHANNEL_23 0xC3
#define MRF_CHANNEL_24 0xD3
#define MRF_CHANNEL_25 0xE3
#define MRF_CHANNEL_26 0xF3
/*
 interrupt flags
*/
#define MRF_STAT_SLEEP 0x80
#define MRF_STAT_WAKE  0x40
#define MRF_STAT_SEC   0x10
#define MRF_STAT_RX    0x08
#define MRF_STAT_TXN   0x01

#define mrf_set_bit(addr, bit) mrf_write( addr, mrf_read(addr) | 1<<bit )
#define mrf_clear_bit(addr, bit) mrf_write( addr, mrf_read(addr) & ~(1<<bit) )
#define mrf_test_bit(addr, bit) (mrf_read(addr) & 1<<bit)>>bit
#define mrf_normal_mode() mrf_write( MRF_RXMCR, mrf_read(MRF_RXMCR) & 0xFC )
#define mrf_promisc_mode() mrf_write( MRF_RXMCR, (mrf_read(MRF_RXMCR) & 0xFC) | 0x01 )
#define mrf_error_mode() mrf_write( MRF_RXMCR, (mrf_read(MRF_RXMCR) & 0xFC) | 0x02 )
#define mrf_stat(stat) MRF_INTF->stat & stat



typedef struct {
  char stat;
  char txstat;
  char seq;
  char *intf_saddr;
  char *intf_laddr;
  char *intf_panid;
  volatile unsigned char *mrf_cs;
  char mrf_port;
} mrf_intf;

typedef struct {
  char srclen;
  char srcaddr[MRF_LADDR_LEN];
  char srcpan[MRF_PANID_LEN];
  char dstlen;
  char dstaddr[MRF_LADDR_LEN];
  char dstpan[MRF_PANID_LEN];
} mrf_addr;

typedef struct {
  mrf_addr addr;
  char length;
  char type;
  char seq;
  char lqi;
  char data[MRF_MAX_DLEN];
} mrf_packet;

void mrf_spi_init(void);
void mrf_select(mrf_intf *intf);
void mrf_init_intf (mrf_intf *intf, volatile char *reg, char bitoffset);
void mrf_write (short addr, char data);
char mrf_read (short addr);
void mrf_send (mrf_addr *addr, char dlen, char *data, char flags);
char mrf_recv (mrf_packet *packet);
void mrf_init(char channel);
void mrf_set_saddr (char *addr);
void mrf_set_laddr (char *addr);
void mrf_set_panid (char *addr);
char mrf_get_lqi (void);
void mrf_release_buffer (void);
char mrf_check (void);
void mrf_isleep(void);
void mrf_iwake(void);
void mrf_rf_reset(void);
void mrf_enable_interrupt (char intedge, char types);
