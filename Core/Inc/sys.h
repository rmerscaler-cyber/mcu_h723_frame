#ifndef _SYS_H
#define _SYS_H
#include "stm32h7xx.h"

/* [H7] 位带操作不可用(H7 GPIO不在0x40000000位带区), 改用BSRR/IDR直接操作 */
#define PHout_SET(n)    (GPIOH->BSRR = (1U << (n)))
#define PHout_CLR(n)    (GPIOH->BSRR = (1U << ((n) + 16U)))
#define PHin(n)         ((GPIOH->IDR >> (n)) & 1U)

#endif
