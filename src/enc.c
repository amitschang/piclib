// specifically for enc28j60 interfacing
#include <xc.h>
#include "spi.h"
#include "enc28j60.h"

#define RCR 0b00000000 // read control reg
#define RBM 0b00111010 // read buffer mem
#define WCR 0b01000000 // write control reg
#define WBM 0b01111010 // write buffer mem
#define BFS 0b10000000 // bit field set
#define BFC 0b10100000 // bit field clear
#define SRC 0b11111111 // system reset
#define ADDR_MASK 0b00011111
#define ENC_FILTER_AND    0x01
#define ENC_FILTER_OR     0x00
#define ENC_FILTER_UCAST  0b10000000
#define ENC_FILTER_BCAST  0b00000001
#define ENC_FILTER_MCAST  0b00000010
#define ENC_FILTER_CRC    0b00100000
#define ENC_FILTER_PATERN 0b00010000
#define ENC_FILTER_MAGIC  0b00001000
#define ENC_FILTER_HASH   0b00000100

/*
 this structure type describes a single interface (i.e. an ENC84J60
 chip) connected to our SPI bus. Any number are allowed, each
 referenced by an invocation of this struct. Only one is active at a
 time, which can be selected with enc_select_intf (not necessary with
 only one
*/
typedef struct {
  /*
    The rest are internal state machine variables for this particular
    interface
  */
  short next_packet_loc;
  short cur_packet_depth;
  short tx_buff_start;
  char curbank;
  char *mac_addr;
  /*
   Where this interface can be found physically, Chip Select line
  */
  volatile unsigned char *enc_cs;
  char enc_port;
  // end
} enc_intf;

typedef struct {
  char src_hwaddr[6];
  char dst_hwaddr[6];
  char type[2];
  short length;
} enc_packet;


static enc_intf *CUR_INTF;
static volatile unsigned char *CUR_CS;
static char CUR_CS_PORT;

void enc_spi_init (void){
  spi_init(SPI_MASTER_FOSC_4, SPI_CLOCK_IDLE_LOW, SPI_TX_ACTIVE_IDLE);
}

/*
 Helper functions that do the slave select for SPI transactions
*/
void __enc_cs_low (void){
  *CUR_CS = *CUR_CS & ~(0x01<<CUR_CS_PORT);
}

void __enc_cs_high (void){
  *CUR_CS = *CUR_CS | (0x01<<CUR_CS_PORT);
}

void enc_select_intf (enc_intf *intf){
  CUR_INTF = intf;
  CUR_CS = CUR_INTF->enc_cs;
  CUR_CS_PORT = CUR_INTF->enc_port;
  __enc_cs_high();
}

void enc_init_intf (enc_intf *intf, volatile char *reg, char bitoffset){
  /*
   Initialize an interface. Takes a reference to the interface
   structure to be initialized and the LATCH and bit offset of the
   chip select pin for it
  */
  intf->enc_cs = reg;
  intf->enc_port = bitoffset;
  intf->curbank = 0;
  enc_select_intf( intf );
}

/*
 Register reading and writing functions
*/
static void enc_banksel (short reg){
  /*
   Select the bank of the register specified. From the include file
   the register is written at the 8th bit. The ECON1 register on the
   ENC chip hols the bank select reg
  */
  __enc_cs_low();
  spi_byte( RCR | ECON1 );
  char econ = spi_byte( 0x00 );
  __enc_cs_high();
  /*
   The bank is the least significant two bits of ECON1, so preserve
   the other bits and write only the bank
  */
  econ = (econ & 0b11111100) | (char)(reg>>8);
  __enc_cs_low();
  spi_byte( WCR | ECON1 );
  spi_byte( econ );
  __enc_cs_high();
  /*
   Set the current bank to avoid duplicating this communication when
   we already have it selected
  */
  CUR_INTF->curbank = (char)(reg>>8);
}

void enc_write_conreg (short reg, char data){
  /*
   Write data to a control register, selecting the appropriate bank if
   necessary
  */
  char cmd = WCR | (ADDR_MASK & reg);
  if (CUR_INTF->curbank != (char)(reg >> 8) )
    enc_banksel( reg );
  __enc_cs_low();
  spi_byte(cmd);
  spi_byte(data);
  __enc_cs_high();
}

char enc_read_ethreg (short reg){
  /*
   Read the value of an eth register, which is one byte in
   length. These are the registers named E*
  */
  char cmd = RCR | (ADDR_MASK & reg);
  if (CUR_INTF->curbank != (char)(reg >> 8) )
    enc_banksel( reg );
  __enc_cs_low();
  spi_byte( cmd );
  char data = spi_byte( 0x00 );
  __enc_cs_high();
  return data;
}

char enc_read_macreg (short reg){
  /*
   Read a mac register, which is one byte in length. The mac registers
   are those named M*
  */
  char cmd = RCR | (ADDR_MASK & reg);
  if (CUR_INTF->curbank != (char)(reg >> 8) )
    enc_banksel( reg );
  __enc_cs_low();
  spi_byte( cmd );
  spi_byte( 0x00 );
  char data = spi_byte( 0x00 );
  __enc_cs_high();
  return data;
}

/*
 Functions for reading from the recieve buffer
*/

void readrecvbuffer (char *out, short num){
  /*
   This function does the actual reading of the buffer, but does not
   check if the number of bytes is appropriate
  */
  short i=0;
  __enc_cs_low();
  spi_byte( RBM );
  while (i < num){
    out[i] = spi_byte( 0x00 );
    i++;
  }
  __enc_cs_high();
}

void enc_enable_interrupt (char types){
  /* first clear the flags */
  enc_write_conreg(EIR, 0x00);
  /* now enable all types interrupts and global enable */
  enc_write_conreg(EIE, 0x80 | types);
}

char enc_count_rx (void){
  return enc_read_ethreg( EPKTCNT );
}

short enc_init_rx (enc_packet *pckt){
  unsigned int status;
  short len;
  if (0==enc_count_rx())
    return -1;
  readrecvbuffer( (char *)&(CUR_INTF->next_packet_loc), 2 );
  readrecvbuffer( (char *)&len, 2);
  readrecvbuffer( (char *)&status, 2 );
  readrecvbuffer( pckt->dst_hwaddr, 6);
  readrecvbuffer( pckt->src_hwaddr, 6);
  readrecvbuffer( pckt->type, 2);
  len -= 18;
  pckt->length = len;
  CUR_INTF->cur_packet_depth = len;
  return len;
}

short enc_read_rx (char *out, short num){
  /*
   only allow to read up to the next packet like this, then must read
   the new header in
  */
  if (num > CUR_INTF->cur_packet_depth)
    num = CUR_INTF->cur_packet_depth;
  if (num == 0) return 0;
  readrecvbuffer (out, num);
  CUR_INTF->cur_packet_depth -= num;
  return num;
}

void enc_flush_rx(void){
  /*
   Flush the recieve buffer to advance to the next packet pending and
   allow space to be used by incoming packets. This function must be
   used after completely reading any packet, and cannot be used until
   the packet headers have been read in

   Move the read data pointer to the next packet position
  */
  enc_write_conreg( ERDPTL, (char)(CUR_INTF->next_packet_loc) );
  enc_write_conreg( ERDPTH, (char)((CUR_INTF->next_packet_loc)>>8) );
  /*
   Also move the buffer write position pointer so that new data can
   take its place in the buffer
  */
  enc_write_conreg( ERXRDPTL, (char)(CUR_INTF->next_packet_loc) );
  enc_write_conreg( ERXRDPTH, (char)((CUR_INTF->next_packet_loc)>>8) );
  /*
   Decrement the pending packet counter
  */
  enc_write_conreg( ECON2, enc_read_ethreg( ECON2 ) | 0b01000000 );
}

/*
 Transmission functions
*/

void enc_write_tx (char *data, short num){
  short i=0;
  __enc_cs_low();
  spi_byte( WBM );
  while (i < num){
    spi_byte( data[i] );
    i++;
  }
  __enc_cs_high();
}

void enc_init_tx_from (char *src, char *dest, char *type){
  /*
   Initialize a transmission. The transmission buffer works a single
   packet at a time, starting from the beginning each time

   First check to make sure a transmission is not pending, don't want
   to overwrite data.
  */
  while ( 0 != (enc_read_ethreg( ECON1 ) & 0b00001000) ){
    NOP();
  }
  /*
   Start writing at beginning of transmist space
  */
  enc_write_conreg( EWRPTL, (char)(CUR_INTF->tx_buff_start) );
  enc_write_conreg( EWRPTH, (char)(CUR_INTF->tx_buff_start>>8) );
  /*
   Set the packet transmit start pointers to the same value
  */
  enc_write_conreg( ETXSTL, (char)(CUR_INTF->tx_buff_start) );
  enc_write_conreg( ETXSTH, (char)(CUR_INTF->tx_buff_start>>8) );
  /*
   write in packet headers, including the per packet control byte,
   which in this case is no overrides
  */
  enc_write_tx( 0x00, 1 ); // per packet control byte
  enc_write_tx( dest, 6 );
  enc_write_tx( src, 6 );
  enc_write_tx( type, 2 );
}

void enc_init_tx (char *dest, char *type){
  enc_init_tx_from(CUR_INTF->mac_addr, dest, type);
}

void enc_init_tx_bcast (char *type){
  char mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  enc_init_tx( mac, type );
}

char enc_tx (char block){
  /*
   Transmit the packet. The block argument, if 1, blocks until the
   transmission is complete, and returns the status of transmission

  */
  short wptr;
  wptr = enc_read_ethreg( EWRPTL );
  wptr = wptr | (enc_read_ethreg( EWRPTH )<<8);
  wptr--; // due to autoinc
  enc_write_conreg( ETXNDL, (char)wptr );
  enc_write_conreg( ETXNDH, (char)(wptr>>8) );
  // Start transmission setting ECON1.TXRTS
  char econ1 = enc_read_ethreg( ECON1 );
  enc_write_conreg( ECON1, econ1 | 0b00001000 );
  if ( block ){
    while ( 0 != (enc_read_ethreg( ECON1 ) & 0b00001000) ){
      NOP();
    }
    // if we are blocking, return the transmission status
    return (enc_read_ethreg(ESTAT)>>1) & 0x01;
  }
  // otherwise just return 1
  return 1;
}

void writephyreg (char reg, short data){
  enc_write_conreg( MIREGADR, reg );
  enc_write_conreg( MIWRL, (char)data );
  enc_write_conreg( MIWRH, (char)(data>>8));
  // __delay_us(11); // to allow it to write
}

short readphyreg (char reg){
  short res;
  enc_write_conreg( MIREGADR, reg );
  enc_write_conreg( MICMD, 0x01 );
  NOP();
  while ( (enc_read_macreg( MISTAT ) & 0x01) != 0 ) continue;
  enc_write_conreg( MICMD, 0x00 );
  res = enc_read_macreg( MIRDL );
  res = res << 8;
  res = res + enc_read_macreg( MIRDH );
  return res;
}

void enc_init_buffers (short start, short size){
  /*
   initialize the transmit and receive buffers. It takes the start and
   size of the recv buffer to create as arguments. The transmit buffer
   is the remaining
  */
  short end = start+size;
  /*
   Write low and high bytes of the start pointer
  */
  enc_write_conreg( ERXSTL, (char)start );
  enc_write_conreg( ERXSTH, (char)(start>>8) );
  /*
   Set the recv buffer write pointer to the start of buffer
  */
  enc_write_conreg( ERDPTL, 0 );
  enc_write_conreg( ERDPTH, 0 );
  /*
   Write low and high bytes of the end pointer
  */
  enc_write_conreg( ERXNDL, (char)end );
  enc_write_conreg( ERXNDH, (char)(end>>8) );
  /*
   start of tx buffer is just after end of recv
  */
  CUR_INTF->tx_buff_start = end+2;
}

void enc_setmacaddr (char mac[]){
  enc_write_conreg( MAADR1, mac[0] );
  enc_write_conreg( MAADR2, mac[1] );
  enc_write_conreg( MAADR3, mac[2] );
  enc_write_conreg( MAADR4, mac[3] );
  enc_write_conreg( MAADR5, mac[4] );
  enc_write_conreg( MAADR6, mac[5] );
}

void enc_init_mac_hdpx (char mac[]){
  // enable mac recv
  enc_write_conreg( MACON1, 0b00000001 );
  /*
    Half duplex mode, do zero padding, no proprietary header and do
    append a crc to transmission. Ensure the PHY register matches
    duplex
  */
  enc_write_conreg( MACON3, 0b01110110 );
  writephyreg(0x00, 0x0000); // PHCON1
  // loopback disable (most of the time dont want this)
  writephyreg(0x10, 0x0100); // PHCON2
  // back to back interpacket gap
  // for half duplex, recommended 12h (9.6 us)
  enc_write_conreg( MABBIPG, 0x12 );
  // recommended non-back to back values for half duplex
  enc_write_conreg( MAIPGL, 0x12 );
  enc_write_conreg( MAIPGH, 0x0C );
  enc_setmacaddr(mac);
  CUR_INTF->mac_addr = mac;
}

void enc_init_mac_fdpx (char mac[]){
  // enable mac recv, set TXPAUS and RXPAUS, no PASSALL
  enc_write_conreg( MACON1, 0b00001101 );
  /*
    full duplex mode, do zero padding, no proprietary header and do
    append a crc to transmission. Ensure the PHY register matches
    duplex setting
  */
  enc_write_conreg( MACON3, 0b01110111 );
  writephyreg(0x00, 0x0100); // PHCON1
  // back to back interpacket gap
  // for fill duplex, recommended 15h (9.6 us)
  enc_write_conreg( MABBIPG, 0x15 );
  // recommended non-back to back values
  enc_write_conreg( MAIPGL, 0x12 );
  enc_setmacaddr(mac);
  CUR_INTF->mac_addr = mac;
}

void enc_enable_recv (void){
  char econ = enc_read_ethreg( ECON1 );
  enc_write_conreg( ECON1, econ | 0b00000100 );
}

void enc_init_hdpx (char mac[], short rbuffsize){
  enc_init_mac_hdpx(mac);
  enc_init_buffers( 0, rbuffsize );
  enc_enable_recv();
}

void enc_init (char mac[], short rbuffsize){
  enc_init_mac_fdpx(mac);
  enc_init_buffers( 0, rbuffsize );
  enc_enable_recv();
}

void enc_set_filters (char andor, char filters){
  char erxfcon = (andor << 6) | filters;
  enc_write_conreg(ERXFCON, erxfcon);
}

void enc_softreset (void){
  __enc_cs_low();
  spi_byte( 0xFF );
  __enc_cs_high();
}
