#define SPI_MASTER_FOSC_4  0
#define SPI_MASTER_FOSC_16 1
#define SPI_MASTER_FOSC_64 2
#define SPI_MASTER_TMR2    3
#define SPI_SLAVE_SS       4
#define SPI_SLAVE_NO_SS    5
// clock settings
#define SPI_CLOCK_IDLE_LOW  0
#define SPI_CLOCK_IDLE_HIGH 1
#define SPI_TX_IDLE_ACTIVE  0
#define SPI_TX_ACTIVE_IDLE  1

/*
 The following defines work for pic16f1826/27. Need to figure out
 how the chip is passed in xc.h
*/

#define TRISSDO TRISB2
#define TRISSDI TRISB1
#define TRISSCK TRISB4
#define TRISSS  TRISA5

/*
 Interface functions
*/

void spi_select (unsigned char iface);
void spi_enable (void);
void spi_enable (void);
void spi_init (unsigned char mode,
	       unsigned char cpol,
	       unsigned char cedge);
unsigned char spi_byte (unsigned char data);
unsigned char spi_read();
void spi_write(unsigned char data);
