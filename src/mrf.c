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
void mrf_send (char src_alen, char dst_alen, char *dst_addr, char *dst_pid,
	       char ack, char sec, char dlen, char *data){
  int i;
  char hlen = 3;
  /* 
   frame control field, start with data type frame and version >2003
  */
  short fcon = 0x1001;
  fcon = fcon | sec<<3;
  fcon = fcon | ack<<5;
  /* 
   addressing modes
  */
  if (dst_alen != 0){
    __mrf_laddr_write(0x002+hlen, dst_pid[0]);
    __mrf_laddr_write(0x002+hlen+1, dst_pid[1]);
    hlen+=2;
  }
  if (dst_alen == MRF_SADDR_LEN){
    for (i=0;i<MRF_SADDR_LEN;i++){
      __mrf_laddr_write(0x002+hlen+i, dst_addr[i]);
    }
    fcon = fcon | 0x0800;
    hlen+=2;
  }
  else if (dst_alen == MRF_LADDR_LEN){
    for (i=0;i<MRF_LADDR_LEN;i++){
      __mrf_laddr_write(0x002+hlen+i, dst_addr[i]);
    }
    fcon = fcon | 0x0C00;
    hlen+=8;
  }
  if (src_alen != 0){
    __mrf_laddr_write(0x002+hlen, MRF_INTF->intf_panid[0]);
    __mrf_laddr_write(0x002+hlen+1, MRF_INTF->intf_panid[1]);
    hlen+=2;
  }
  if (src_alen == MRF_SADDR_LEN){
    for (i=0;i<MRF_SADDR_LEN;i++){
      __mrf_laddr_write(0x002+hlen+i, MRF_INTF->intf_saddr[i]);
    }
    fcon = fcon | 0x8000;
    hlen+=2;
  }
  else if (src_alen == MRF_LADDR_LEN){
    for (i=0;i<MRF_LADDR_LEN;i++){
      __mrf_laddr_write(0x002+hlen+i, MRF_INTF->intf_laddr[i]);
    }
    fcon = fcon | 0xC000;
    hlen+=8;
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
  if (ack)
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
    packet->dstpan[0] = __mrf_laddr_read( idx++ );
    packet->dstpan[1] = __mrf_laddr_read( idx++ );
    if ((fcon & 0x0C00) == 0x0C00){
      for (i=0;i<MRF_LADDR_LEN;i++)
	packet->dstaddr[i] = __mrf_laddr_read( idx++ );
      packet->dstlen = MRF_LADDR_LEN;
    }
    else if ((fcon & 0x0C00) == 0x0800){
      for (i=0;i<MRF_SADDR_LEN;i++)
	packet->dstaddr[i] = __mrf_laddr_read( idx++ );
      packet->dstlen = MRF_SADDR_LEN;
    }
  }
  else {
    packet->dstlen = 0;
  }
  // src as dest
  if ((fcon & 0xC000) != 0x0000){
    packet->srcpan[0] = __mrf_laddr_read( idx++ );
    packet->srcpan[1] = __mrf_laddr_read( idx++ );
    if ((fcon & 0xC000) == 0xC000){
      for (i=0;i<MRF_LADDR_LEN;i++)
	packet->srcaddr[i] = __mrf_laddr_read( idx++ );
      packet->srclen = MRF_LADDR_LEN;
    }
    else if ((fcon & 0xC000) == 0x8000){
      for (i=0;i<MRF_SADDR_LEN;i++)
	packet->srcaddr[i] = __mrf_laddr_read( idx++ );
      packet->srclen = MRF_SADDR_LEN;
    }
  }
  else {
    packet->srclen = 0;
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
  int i=100000;
  mrf_write(MRF_WAKECON,0x40);
  while (i-- > 0){
    NOP();
  }
  mrf_write(MRF_WAKECON,0x00);
  mrf_write(MRF_RFCTL, 0x04);
  mrf_write(MRF_RFCTL, 0x00);
}
