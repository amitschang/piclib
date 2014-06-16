/* 
 Ports used for SPI functions in various chips
*/

#if defined(_16F1826) || defined(_16F1827)
#define TRISSDO TRISB2
#define TRISSDI TRISB1
#define TRISSCK TRISB4
#define TRISSS  TRISB5
#define ANSELSS ANSB5
#define ANSELSCK ANSB4
#define ANSELSDI ANSB1
#elif defined(_16F1936) || defined(_16F1518)
#define TRISSDO TRISC5
#define TRISSDI TRISC4
#define TRISSCK TRISC3
#define TRISSS  TRISA5
#define ANSELSS ANSA5
#elif defined(_16F1825)
#define TRISSDO TRISC2
#define TRISSDI TRISC1
#define TRISSCK TRISC0
#define TRISSS  TRISC3
#elif defined(_16F1455)
#define TRISSDO TRISC2
#define TRISSDI TRISC1
#define TRISSCK TRISC0
#define TRISSS  TRISC3
#define ANSELSS ANSC3
#define ANSELSCK ANSC0
#define ANSELSDI ANSC1
#endif
