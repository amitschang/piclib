// specifically for enc28j60 interfacing
#include <xc.h>
#include "spi.h"
#include "enc28j60.h"

#define ENC_FILTER_AND    0x01
#define ENC_FILTER_OR     0x00
#define ENC_FILTER_UCAST  0b10000000
#define ENC_FILTER_BCAST  0b00000001
#define ENC_FILTER_MCAST  0b00000010
#define ENC_FILTER_CRC    0b00100000
#define ENC_FILTER_PATERN 0b00010000
#define ENC_FILTER_MAGIC  0b00001000
#define ENC_FILTER_HASH   0b00000100

typedef struct {
  char src_hwaddr[6];
  char dst_hwaddr[6];
  char type[2];
  short length;
  short next_packet_loc;
  short cur_packet_depth;
  short tx_buff_start;
  char curbank;
  char *mac_addr;
  volatile unsigned char *enc_cs;
  char enc_port;
} enc_intf;

void __enc_cs_low(void);
void __enc_cs_high(void);
void enc_spi_init (void);
void enc_init_intf (enc_intf *intf, volatile char *reg, char bitoffset);
void enc_select_intf(enc_intf *intf);
void enc_write_conreg (short reg, char data);
char enc_read_ethreg (short reg);
char enc_read_macreg (short reg);
short enc_init_rx (void);
short enc_read_rx (char *out, short num);
void enc_flush_rx(void);
char enc_count_rx (void);
void enc_write_tx (char *data, short num);
void enc_init_tx (char *dest, char *type);
char enc_tx (char block);
void enc_init_buffers (short start, short size);
void enc_setmacaddr (char mac[]);
void enc_init_mac (char mac[]);
void enc_enable_recv (void);
void enc_init (char mac[], short rbuffsize);
void enc_set_filters (char andor, char filters);
void enc_softreset (void);
