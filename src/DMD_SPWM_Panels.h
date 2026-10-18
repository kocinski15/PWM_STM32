#pragma once
/*--------------------------------------------------------------------------------------
 DMD_SPWM_Panels.h - panel descriptions

   #define NAME  MUX_pins, Panel_width, Panel_height, Scan, Pattern

   MUX_pins : number of A,B,C.. lines. 33 = shift-register (595 type) row driver
              driven by 3 pins (A = CLK, B = LAT, C = SDI)
   Scan     : number of multiplexed rows (1/Scan)
   Pattern  : 0 - plain (scan = height/2, or multiplexed with 8-pixel pixbase)
              1, 2, 3 - outdoor "plain" layouts (see DMD_SPWM.cpp, panel_index())
--------------------------------------------------------------------------------------*/

#define RGB32x16plainS8    3, 32, 16, 8, 0
#define RGB40x20plainS10   4, 40, 20, 10, 0
#define RGB32x32plainS16   4, 32, 32, 16, 0
#define RGB64x32plainS16   4, 64, 32, 16, 0
#define RGB80x40plainS20   5, 80, 40, 20, 0
#define RGB64x64plainS32   5, 64, 64, 32, 0
#define RGB128x64plainS32  5, 128, 64, 32, 0

#define RGB32x16plainS4         2, 32, 16, 4, 1
#define RGB32x16plainS4_pt2     2, 32, 16, 4, 2
#define RGB32x16plainS4_DIRECT  4, 32, 16, 4, 1
#define RGB32x16plainS2         1, 32, 16, 2, 1
#define RGB32x16plainS2_DIRECT  2, 32, 16, 2, 1
#define RGB32x32_S8_maxmurugan  3, 32, 32, 8, 3
#define RGB64x32_S8_OKSingra    3, 64, 32, 8, 3

// 64x32 1/8 FM6373/TC7559 panel with shift-register row driver
#define RGB64x32s8_6373         33, 64, 32, 8, 3

#define DMD_MUX_SHIFTREG_CODE 33

// Color depth parameters of the old library are accepted and ignored
// (the framebuffer is always RGB565, sent to the chip as 12..14-bit greyscale).
#define COLOR_1BITS         1
#define COLOR_4BITS         4
#define COLOR_4BITS_Packed  3
