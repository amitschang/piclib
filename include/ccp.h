#include "ccpports.h"
/*
 Functions and macros for managing CCP modules on microchip PIC
 chips
*/

/*
 macros for the standard PWM operation
*/
#define CCP1 1
#define CCP2 2
#define CCP3 3
#define CCP4 4
#define CCP5 5
#define CCP6 6
#define CCP7 7
#define CCP8 8
#define CCP9 9
#define CCP10 10
#define TIMER2 2
#define TIMER4 4
#define TIMER6 6
#define TMR2SEL 0
#define TMR4SEL 1
#define TMR6SEL 2
#define TMR_PRESCALE_1  0
#define TMR_PRESCALE_4  1
#define TMR_PRESCALE_16 2
#define TMR_PRESCALE_64 3

#define _pwm_duty(ccp, duty) \
  CCPR ## ccp ## L = duty >> 2; \
  CCP ## ccp ## CON &= 0b11001111 | (duty << 4)
#define pwm_duty(ccp, duty) \
  _pwm_duty(ccp, duty)
#define _pwm_init_1(timer) \
  CCPTMRS0 &= 0b11111100 | TMR ## timer ## SEL
#define _pwm_init_2(timer) \
  CCPTMRS0 &= 0b11110011 | TMR ## timer ## SEL << 2
#define _pwm_init_3(timer) \
  CCPTMRS0 &= 0b11001111 | TMR ## timer ## SEL << 4
#define _pwm_init_4(timer) \
  CCPTMRS0 &= 0b00111111 | TMR ## timer ## SEL << 6
#define _pwm_init_5(timer) \
  CCPTMRS1 &= 0b11111100 | TMR ## timer ## SEL
#define _pwm_init_6(timer) \
  CCPTMRS1 &= 0b11110011 | TMR ## timer ## SEL << 2
#define _pwm_init_7(timer) \
  CCPTMRS1 &= 0b11001111 | TMR ## timer ## SEL << 4
#define _pwm_init_8(timer) \
  CCPTMRS1 &= 0b00111111 | TMR ## timer ## SEL << 6
#define _pwm_init_9(timer) \
  CCPTMRS2 &= 0b11111100 | TMR ## timer ## SEL
#define _pwm_init_10(timer) \
  CCPTMRS2 &= 0b11110011 | TMR ## timer ## SEL << 2
#ifdef TMR2ONLY
#define _pwm_init(ccp, timer, prescale, period) \
  PR ## timer = period; \
  CCP ## ccp ## CON = 0b00001100; \
  T ## timer ## CON = prescale; \
  TMR ## timer ## ON = 1; \
  TRISCCP ## ccp = 0
#else
#define _pwm_init(ccp, timer, prescale, period) \
  PR ## timer = period; \
  CCP ## ccp ## CON = 0b00001100; \
  _pwm_init_ ## ccp(timer); \
  T ## timer ## CON = prescale; \
  TMR ## timer ## ON = 1; \
  TRISCCP ## ccp = 0
#endif
#define pwm_init(ccp, timer, prescale, period) \
  _pwm_init(ccp, timer, prescale, period)
#define _pwm_disable(ccp) \
  CCP ## ccp ## CON = 0
#define pwm_disable(ccp) \
  _pwm_disable(ccp)
