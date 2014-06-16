/*
 ports for CCP modules in select microchip chips
*/

#if defined(_16F1936)
#define TRISCCP1 TRISC2
#define TRISCCP2 TRISC1
#endif
#if defined(_16F1826)
#define TRISCCP1 TRISB3
#define TMR2ONLY
#endif
