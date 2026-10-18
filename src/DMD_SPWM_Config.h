#pragma once
/*--------------------------------------------------------------------------------------
 DMD_SPWM_Config.h - compile-time options for the STM_PWM library
 (STM32duino / Arduino_Core_STM32, STM32F1 and STM32F4 only)
--------------------------------------------------------------------------------------*/

#if !defined(ARDUINO_ARCH_STM32)
#error "STM_PWM requires the STM32duino core (Arduino_Core_STM32)"
#endif
#if !defined(STM32F1xx) && !defined(STM32F4xx)
#error "STM_PWM supports only STM32F1 and STM32F4 MCUs"
#endif

/* ---------------------------------------------------------------------------------
   Timers
   Default:      MAIN timer = TIM4, OE (GCLK) timer = TIM3 -> OE pin: PA6 PA7 PB0 PB1
   Alternative:  MAIN timer = TIM3, OE (GCLK) timer = TIM4 -> OE pin: PB6 PB7 PB8 PB9
   Uncomment (or define in build flags) to use the alternative set.
   DCLK timer (FM6363 / DP3264 family only):
      STM32F1: TIM2 + DMA1 channel 1     STM32F4: TIM1 + DMA2 stream 6
   Do not use these timers for Servo/tone/analogWrite in your sketch.
----------------------------------------------------------------------------------- */
//#define DMD_ALTERNATIVE_TIMERS

/* ---------------------------------------------------------------------------------
   Data clock shape
   CLK_AFTER_DATA (default): data is set, CLK rises one write later.
   DMD_DUAL_CLK_PULSE: hold CLK high for two writes (needed on fast F4 MCUs).
----------------------------------------------------------------------------------- */
#ifndef DMD_DUAL_CLK_PULSE
#if defined(STM32F4xx)
#define DMD_DUAL_CLK_PULSE 1
#else
#define DMD_DUAL_CLK_PULSE 0
#endif
#endif

/* Priority of the row-switching interrupt (0 = highest). */
#ifndef DMD_IRQ_PRIORITY
#define DMD_IRQ_PRIORITY 0
#endif
