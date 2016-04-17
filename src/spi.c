#include <xc.h>
#include "spiports.h"

#define SPI_MASTER_FOSC_4  0
#define SPI_MASTER_FOSC_16 1
#define SPI_MASTER_FOSC_64 2
#define SPI_MASTER_TMR2    3
#define SPI_SLAVE_SS       4
#define SPI_SLAVE_NO_SS    5
//
// clock settings
//
#define SPI_CLOCK_IDLE_LOW  0
#define SPI_CLOCK_IDLE_HIGH 1
#define SPI_TX_IDLE_ACTIVE  0
#define SPI_TX_ACTIVE_IDLE  1

/*
 Index of active spi interface
*/

volatile unsigned char *ACTIVE_SSPBUF  = &SSPBUF;
volatile unsigned char *ACTIVE_SSPSTAT = &SSPSTAT;
volatile unsigned char *ACTIVE_SSPCON1 = &SSPCON1;

/*
 Interface functions
*/

void spi_select (unsigned char iface){
#ifdef SSP2BUF
  switch ( iface ){
  case 1:
    ACTIVE_SSPBUF  = &SSPBUF;
    ACTIVE_SSPSTAT = &SSPSTAT;
    ACTIVE_SSPCON1 = &SSPCON1;
    break;
  case 2:
    ACTIVE_SSPBUF  = &SSP2BUF;
    ACTIVE_SSPSTAT = &SSP2STAT;
    ACTIVE_SSPCON1 = &SSP2CON1;
    break;
  }
#else
}
#endif

void spi_enable (void){
  *ACTIVE_SSPCON1 = *ACTIVE_SSPCON1 | 0b00100000;
}

void spi_disable (void){
  *ACTIVE_SSPCON1 = *ACTIVE_SSPCON1 & 0b11011111;
}

void spi_init (unsigned char mode,
	       unsigned char cpol,
	       unsigned char cedge){
  //
  // Ensure tristates are right
  //
  TRISSDO = 0; // data output
  TRISSDI = 1; // data int
#ifdef ANSELSDI
  ANSELSDI = 0;
#endif
#ifdef ANSELSCK
    ANSELSCK = 0;// clock, oddly enough, this seems to be nec. even on master
#endif
  if ( mode == SPI_SLAVE_SS ||
       mode == SPI_SLAVE_NO_SS ){
    TRISSCK = 1; // slave gets clock from master
    TRISSS = 1;  // slave select function is input
#ifdef ANSELSS
    ANSELSS = 0; // digital input for slave select and
#endif
  }
  else {
    TRISSCK = 0;
  }
  //
  // setup SPI mode
  //
  char sspmode = mode | cpol<<4;
  *ACTIVE_SSPCON1 = (*ACTIVE_SSPCON1 & 0b11100000) | sspmode;
  *ACTIVE_SSPSTAT = (*ACTIVE_SSPSTAT & 0b10111111) | cedge<<6;
  //
  // enable the SPI module
  //
  spi_enable();
}

unsigned char spi_byte (unsigned char data){
  //
  // Load the data to the active SSP buffer to begin the transfer
  //
  *ACTIVE_SSPBUF = data;
  //
  // Now wait for the incoming data to finish loading
  //
  while ( (*ACTIVE_SSPSTAT & 0x01) == 0 )
    continue;
  //
  // return the byte read
  //
  return *ACTIVE_SSPBUF;
}

unsigned char spi_read(){
  return *ACTIVE_SSPBUF;
}

void spi_write(unsigned char data){
  *ACTIVE_SSPBUF = data;
}


