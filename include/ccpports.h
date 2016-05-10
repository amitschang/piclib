/*
 ports for CCP modules in select microchip chips.

 TMR2ONLY applies if the standard PWM functionality is fixed to
 timer2. Some chips have extended modules that allow more timers as
 well as standard, but these macros apply to standard.
*/

#if defined(_16F1936)
#define TRISCCP1 TRISC2
#define TRISCCP2 TRISC1
#endif
#if defined(_16F1826)
#define TRISCCP1 TRISB3
#define TMR2ONLY
#endif
#if defined(_16F1709)
#define TMR2ONLY
#define TRISCCP1 TRISC5
#define TRISCCP2 TRISC3
#endif
#if defined(_16F1705)
#define TMR2ONLY
#define TRISCCP1 TRISC5
#define TRISCCP2 TRISC3
#endif
