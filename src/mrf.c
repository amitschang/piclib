#include <xc.h>
#include "spi.h"
#include "mrf24j40.h"

#define MRF_SADDR_LEN 2
#define MRF_LADDR_LEN 8
#define MRF_SADDR MRF_SADDR_LEN
#define MRF_LADDR MRF_LADDR_LEN
#define MRF_PANID 0
#define MRF_PANID_LEN 2
#define MRF_MAX_DLEN  125 // max payload 128 - 3 (min header)
#define MRF_SEC 0b00001000
#define MRF_ACK 0b00100000
#define MRF_FLIP_ADDR 0x01

#define mrf_set_bit(addr, bit) mrf_write( addr, mrf_read(addr) | 1<<bit )
#define mrf_clear_bit(addr, bit) mrf_write( addr, mrf_read(addr) & ~(1<<bit) )

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

static volatile unsigned char *MRF_CS;
static char MRF_PORT;
static mrf_intf *MRF_INTF;

void mrf_spi_init(void){
  spi_init(SPI_MASTER_FOSC_4, SPI_CLOCK_IDLE_LOW, SPI_TX_ACTIVE_IDLE);
}
/*
 Helper functions that do the slave select for SPI transactions
*/
static void __mrf_cs_low (void){
  *MRF_CS = *MRF_CS & ~(0x01<<MRF_PORT);
}

static void __mrf_cs_high (void){
  *MRF_CS = *MRF_CS | (0x01<<MRF_PORT);
}

/*
 Select the active mrf chip. Accepts the register corresponding to
 controller IO pin that is connected to the MRF chip select line and
 the bit offset of the pin itself. Must be called before
 initialization and tristating the pin is the controllers
 responsibility
 */
void mrf_select(mrf_intf *intf){
  MRF_CS = intf->mrf_cs;
  MRF_PORT = intf->mrf_port;
  MRF_INTF = intf;
  __mrf_cs_high();
}

void mrf_init_intf (mrf_intf *intf, volatile char *reg, char bitoffset){
  intf->mrf_cs = reg;
  intf->mrf_port = bitoffset;
  intf->seq = PORTA;
  mrf_select( intf );
}

/*
 These are the low level read and write functions for short and long
 addresses. Usually an application will use the higher level read and
 write (and send/recv for packets) wich determine the type based on
 regiter
*/
static void __mrf_saddr_write (char addr, char data){
  __mrf_cs_low();
  spi_byte( 0x01 | addr<<1);
  spi_byte( data );
  __mrf_cs_high();
}

static char __mrf_saddr_read (char addr){
  __mrf_cs_low();
  spi_byte( addr<<1);
  char data = spi_byte( 0x00 );
  __mrf_cs_high();
  return data;
}

static void __mrf_laddr_write (short addr, char data){
  __mrf_cs_low();
  spi_byte( 0x80 | (char)(addr>>3));
  spi_byte( 0x10 | (char)(addr<<5));
  spi_byte( data );
  __mrf_cs_high();
}

char __mrf_laddr_read (short addr){
  __mrf_cs_low();
  spi_byte( 0x80 | (char)(addr>>3));
  spi_byte( (char)(addr<<5));
  char data = spi_byte( 0x00 );
  __mrf_cs_high();
  return data;
}

/*
 High level register read and write functions
*/
void mrf_write (short addr, char data){
  if (addr>>9 == 1)
    __mrf_laddr_write( addr, data );
  else
    __mrf_saddr_write( (char)addr, data );
}

char mrf_read (short addr){
  if (addr>>9 == 1)
    return __mrf_laddr_read( addr );
  else
    return __mrf_saddr_read( (char)addr );
}

void mrf_init (char channel){
  /*
   These steps are straight out of the manual. I do not know what all
   do nor what is necessary, but seem to work as non-beacon enabled
   network
  */
  mrf_write(MRF_SOFTRST, 0x07);
  // RFCON2 has PLLEN bit which must be enabled for transceiving
  mrf_write(MRF_RFCON2, 0x80);
  mrf_write(MRF_BBREG2, 0x80);
  mrf_write(MRF_CCAEDTH, 0x60);
  // reset the state machine
  mrf_write(MRF_RFCON0, channel); // channel 11
  mrf_write(MRF_RFCTL, 0x04);
  mrf_write(MRF_RFCTL, 0x00);
  /*
    The following settings are from the recommendations in the
    datasheet, but its not clear any are necessary for normal
    operation

   * mrf_write(MRF_RFCON6, 0x90);
   * mrf_write(MRF_RFCON7, 0x80);
   * mrf_write(MRF_RFCON8, 0x10);
   * mrf_write(MRF_SLPCON1, 0x21);
   * mrf_write(MRF_PACON2, 0x98);
   * mrf_write(MRF_TXSTBL, 0x95);
   * mrf_write(MRF_RFCON1, 0x01);
   */
}

void mrf_enable_interrupt (char intedge, char types){
  mrf_write(MRF_INTCON, ~types);
  if (intedge){
    mrf_set_bit(MRF_SLPCON0, 1);
  }
  else {
    mrf_clear_bit(MRF_SLPCON0, 1);
  }
}

void mrf_set_saddr (char *addr){
  char i;
  MRF_INTF->intf_saddr = addr;
  for (i=0;i<MRF_SADDR_LEN;i++)
    mrf_write(MRF_SADRL+i, addr[i]);
}

void mrf_set_laddr (char *addr){
  char i;
  MRF_INTF->intf_laddr = addr;
  for (i=0;i<MRF_LADDR_LEN;i++)
    mrf_write(MRF_EADR0+i, addr[i]);
}

void mrf_set_panid (char *addr){
  char i;
  MRF_INTF->intf_panid = addr;
  for (i=0;i<MRF_PANID_LEN;i++)
    mrf_write(MRF_PANIDL+i, addr[i]);
}

char mrf_check (void){
  /*
   Keep state of all flags by oring with the stored flags. Certain
   operations such as reading recieved packets clear these states. The
   tx flag is more passive, so we clear it first.
  */
  MRF_INTF->stat &= 0xFE;
  MRF_INTF->stat |= mrf_read( MRF_INTSTAT );
  if (MRF_INTF->stat & 0x01){
    MRF_INTF->txstat = mrf_read( MRF_TXSTAT );
    if (MRF_INTF->txstat & 0x01){
      if (MRF_INTF->txstat & 0x20){
	MRF_INTF->txstat = -1;
      }
      else {
	MRF_INTF->txstat = (MRF_INTF->txstat >> 6);
      }
    }
    else {
      MRF_INTF->txstat = 0;
    }
  }
  if (MRF_INTF->stat & 0x08){
    return 1;
  }
  else {
    return 0;
  }
}

/*
 Send data frame
*/
void mrf_send (mrf_addr *addr, char dlen, char *data, char flags){
  int i;
  char hlen = 3;
  /*
   frame control field, start with data type frame and version >2003
  */
  short fcon = 0x1001;
  fcon = fcon | (MRF_SEC & flags);
  fcon = fcon | (MRF_ACK & flags);
  /*
   addressing modes. Set pointers to address structure, this allows us
   to enable flipping of the address information (e.g. In replying
   this would be a very common operation, especially when we have
   unicast filter set)
  */
  char _srclen, _dstlen;
  char *_srcpan, *_dstpan, *_srcaddr, *_dstaddr;
  if (MRF_FLIP_ADDR & flags){
    _srclen = addr->dstlen;
    _srcpan = addr->dstpan;
    _srcaddr = addr->dstaddr;
    _dstlen = addr->srclen;
    _dstpan = addr->srcpan;
    _dstaddr = addr->srcaddr;
  }
  else {
    _srclen = addr->srclen;
    _srcpan = addr->srcpan;
    _srcaddr = addr->srcaddr;
    _dstlen = addr->dstlen;
    _dstpan = addr->dstpan;
    _dstaddr = addr->dstaddr;
  }
  /*
   Now actually set the address information in the transmit buffer
  */
  if (_dstlen != 0){
    __mrf_laddr_write(0x002+hlen, _dstpan[0]);
    __mrf_laddr_write(0x002+hlen+1, _dstpan[1]);
    hlen+=2;
    for (i=0;i<_dstlen;i++){
      __mrf_laddr_write(0x002+hlen+i, _dstaddr[i]);
    }
    hlen+=_dstlen;
    if (_dstlen == MRF_SADDR_LEN){
      fcon = fcon | 0x0800;
    }
    else {
      fcon = fcon | 0x0C00;
    }
  }

  if (_srclen != 0){
    __mrf_laddr_write(0x002+hlen, _srcpan[0]);
    __mrf_laddr_write(0x002+hlen+1, _srcpan[1]);
    hlen+=2;
    for (i=0;i<_srclen;i++){
      __mrf_laddr_write(0x002+hlen+i, _srcaddr[i]);
    }
    hlen+=_srclen;
    if (_srclen == MRF_SADDR_LEN){
      fcon = fcon | 0x8000;
    }
    else {
      fcon = fcon | 0xC000;
    }
  }
  /*
   now we can write the header length and frame control
  */
  __mrf_laddr_write(0x000, hlen);
  __mrf_laddr_write(0x002, fcon);
  __mrf_laddr_write(0x003, fcon>>8);
  __mrf_laddr_write(0x004, MRF_INTF->seq++); //seq
  /*
   write dlen and data
  */
  __mrf_laddr_write(0x001, dlen+hlen);
  for (i=0; i<dlen; i++){
      __mrf_laddr_write(0x002+hlen+i, data[i]);
  }
  /*
   Set appropriate ack and sec flags
  */
  if (MRF_ACK & flags)
    mrf_write(MRF_TXNCON, mrf_read(MRF_TXNCON) | 0b00000100);

  /*
   now transmit by setting TXNCON 0x1B
  */
  mrf_write(0x1B, mrf_read(0x1B) | 0x01);
}

char mrf_recv (mrf_packet *packet){
  /*
   prevent any overwriting of received packets
  */
  mrf_set_bit(MRF_BBREG1, 2);
  /*
   Find length of entire packet plus FCS
  */
  char i;
  short idx = 0x300;
  char len = __mrf_laddr_read( idx++ );

  /*
   read in the frame control and header
  */
  short fcon = __mrf_laddr_read( idx++ );
  fcon = fcon | __mrf_laddr_read( idx++ )<<8;

  packet->type = (char)(fcon & 0x03);
  packet->seq = __mrf_laddr_read( idx++ ); //store the seq number

  /*
    Get the dst address. For modes with any address specified, must
    have a pan id along with it. There are two address types, short
    and long
   */
  if ((fcon & 0x0C00) != 0x0000){
    (packet->addr).dstpan[0] = __mrf_laddr_read( idx++ );
    (packet->addr).dstpan[1] = __mrf_laddr_read( idx++ );
    if ((fcon & 0x0C00) == 0x0C00){
      for (i=0;i<MRF_LADDR_LEN;i++){
	(packet->addr).dstaddr[i] = __mrf_laddr_read( idx++ );
      }
      (packet->addr).dstlen = MRF_LADDR_LEN;
    }
    else if ((fcon & 0x0C00) == 0x0800){
      for (i=0;i<MRF_SADDR_LEN;i++){
	(packet->addr).dstaddr[i] = __mrf_laddr_read( idx++ );
      }
      (packet->addr).dstlen = MRF_SADDR_LEN;
    }
  }
  else {
    (packet->addr).dstlen = 0;
  }
  // src as dest
  if ((fcon & 0xC000) != 0x0000){
    (packet->addr).srcpan[0] = __mrf_laddr_read( idx++ );
    (packet->addr).srcpan[1] = __mrf_laddr_read( idx++ );
    if ((fcon & 0xC000) == 0xC000){
      for (i=0;i<MRF_LADDR_LEN;i++){
	(packet->addr).srcaddr[i] = __mrf_laddr_read( idx++ );
      }
      (packet->addr).srclen = MRF_LADDR_LEN;
    }
    else if ((fcon & 0xC000) == 0x8000){
      for (i=0;i<MRF_SADDR_LEN;i++){
	(packet->addr).srcaddr[i] = __mrf_laddr_read( idx++ );
      }
      (packet->addr).srclen = MRF_SADDR_LEN;
    }
  }
  else {
    (packet->addr).srclen = 0;
  }
  /*
   read the data until length minus 2, which includes FCS. First store
   the length of the data component (which is total length minus the
   header minus 2
  */
  packet->length = len - (idx-0x300) - 1;

  for (i=0;i<packet->length;i++)
    packet->data[i] = __mrf_laddr_read( idx++ );

  /*
   Store the Link quality indicator. 0 means very poor link quality
   and 1 means very high.
  */
  idx+=2;
  packet->lqi = __mrf_laddr_read( idx++ );

  // restore the receiver to allow packets
  mrf_clear_bit(MRF_BBREG1, 2);

  MRF_INTF->stat &= 0xF7;

  return packet->length;
}

void mrf_isleep (void){
  /*
   Instantly put the MRF to sleep
  */
  mrf_write(MRF_WAKECON,0x80);
  mrf_write(MRF_SOFTRST, 0x04);
  mrf_write(MRF_SLPACK, 0x80);
}

void mrf_iwake (void){
  /*
   Wake the module up from instant sleep
  */

  mrf_write(MRF_WAKECON,0xC0);
  mrf_write(MRF_WAKECON,0x00);
}

void mrf_rf_reset(void){
  mrf_write(MRF_RFCTL, 0x04);
  mrf_write(MRF_RFCTL, 0x00);
}
