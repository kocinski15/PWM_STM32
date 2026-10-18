# STM_PWM — HUB75 S-PWM panel library for STM32duino

A slimmed-down port of [DMD_STM32](https://github.com/board707/DMD_STM32) (fork: [kocinski15/PWM_STM32](https://github.com/kocinski15/PWM_STM32)) that does one thing: it drives HUB75 RGB panels built with **S-PWM driver chips** from **STM32F1 / STM32F4** boards on the **STM32duino core** ([Arduino_Core_STM32](https://github.com/stm32duino/Arduino_Core_STM32)).

Changes from DMD_STM32:

* Only S-PWM chips are left. Classic shift-register panels (DMD_RGB, FM6126a, monochrome, parallel) are gone.
* Adafruit_GFX is gone too. In its place there is an RGB565 framebuffer, `setPixel()`, a handful of primitives, and the simple DMD bitmap fonts (`src/fonts`).
* Only STM32F1 and STM32F4 are supported. The libmaple (Roger Clark) and RP2040 code has been removed.
* The timers, DMA and interrupt are written with STM32duino LL drivers and registers. HardwareTimer is not used.
* Color is better: each RGB565 channel is gamma-corrected to the chip's 12–14-bit greyscale. The old library used 4 bits per channel.
* `setBrightness()` works now. It scales the greyscale values.

## Supported chips

| Architecture | Chips | Class |
|---|---|---|
| GCLK only | FM6353, ICN2153 | `DMD_RGB_FM6353` / `DMD_RGB_ICN2153` |
| GCLK + DCLK | FM6363 | `DMD_RGB_FM6363` |
| Decoupled (DCLK PWM, OE row sync) | DP3264, ICN2055, FM6373, SM16380SH, ICND1065 | `DMD_RGB_DP3264`, `DMD_RGB_ICN2055`, `DMD_RGB_FM6373`, `DMD_RGB_SM16380SH`, `DMD_RGB_ICN1065` |

Panel types are in `src/DMD_SPWM_Panels.h` (plain patterns 0–3, binary/direct or 595 shift-register row mux).

## Quick start

```cpp
#include <DMD_SPWM.h>
#include "fonts/SystemFont5x7.h"

uint8_t mux_list[] = { PB6, PB5, PB4, PB3, PB8 };          // A B C D E  (one port)
uint8_t rgbpins[]  = { PA6, PA0, PA1, PA2, PA3, PA4, PA5 };  // CLK R0 G0 B0 R1 G1 B1 (one port)
DMD_RGB_FM6373<RGB128x64plainS32> dmd(mux_list, PB0 /*OE*/, PB7 /*LAT*/, rgbpins, 1, 1);

void setup() {
  dmd.init();
  dmd.selectFont(System5x7);
  dmd.fillScreen(0);
  dmd.drawString(0, 0, "Hello", dmd.Color888(255, 128, 0));
  dmd.setPixel(10, 20, dmd.Color888(0, 0, 255));
  dmd.update();            // send the framebuffer to the panel
}
void loop() {}
```

The panel keeps the image in its own SRAM and refreshes itself. Call `update()` (or the old name `swapBuffers()`) once after each change.

### API

* Framebuffer: `setPixel`, `getPixel`, `fillScreen`, `clearScreen`, `fillRect`, `drawRect`, `drawFastHLine`, `drawFastVLine`, `drawLine`, `drawBitmap` (RGB565), `drawMonoBitmap` (1-bit), `getBuffer`, `setRotation`, `setConnectScheme(CONNECT_ZIGZAG)`
* Text: `selectFont(font, spacing)`, `drawChar`, `drawString(x, y, str, fg, bg = DMD_TRANSPARENT)`, `stringWidth`, `charWidth`, `fontHeight`
* Color: `Color888`, `Color444`, `Color333`, `ColorHSV`, `setColorOrder(DMD_Color_order::BRG)`
* Output: `update()`, `setBrightness(0..255)`, `shiftColorBrightnessDown(n)`
* Chip registers: `setConfigRegisters(r, count, g, b)`, `resetConfigRegisters()`, `applyConfigRegisters()`
* Call `configure_multiplexer(DMD_MUX_TYPE_SHIFTREG)` and `setColorOrder()` **before** `init()`.

## Pins and hardware resources

| Signal | Rule |
|---|---|
| CLK, R0, G0, B0, R1, G1, B1 | any pins, all on **one GPIO port** |
| A, B, C, D, E | any pins, all on **one GPIO port** (595 mux: A = CLK, B = LAT, C = SDI) |
| LAT | any pin |
| OE | a TIM3 channel: PA6 PA7 PB0 PB1 (or TIM4: PB6–PB9 with `DMD_ALTERNATIVE_TIMERS`) |

* Timers: TIM4 (row timer) and TIM3 (GCLK on OE). With `DMD_ALTERNATIVE_TIMERS` defined in `DMD_SPWM_Config.h`, the two swap roles.
* FM6363 and the decoupled family also use a DCLK timer + DMA: TIM2 + DMA1 channel 1 on F1, TIM1 + DMA2 stream 6 on F4.
* The library copies the vector table to RAM (512 bytes) and installs its own handler for the row timer, so the core's HardwareTimer code never handles that timer. Don't use these timers for Servo, tone or analogWrite.
* On F1, JTAG is turned off so PA15, PB3 and PB4 can be used as normal pins. SWD keeps working.
* RAM: the framebuffer takes width × height × 2 bytes. That is 16 KB for 128×64, so use an F4 for big panels. A BluePill (20 KB) fits 64×32.

Timing values come from the original library. They were tuned for 72 MHz F103 and 84–100 MHz F401/F411 BlackPill boards.

## Register Config Finder

`examples/spwm_register_finder` steps through the register profiles in `src/register_profiles/<chip>/<chip>_scan_N.h`. Each profile is shown on the panel with its number (`REG n`). Once you find one that works, set `FINDER_OVERRIDE` to its number. In your own sketch you can load it like this:

```cpp
#include <DMD_SPWM_RegProfiles.h>
#define DMD_SPWM_REGISTER_TEST_PROFILE_CHIP DMD_SPWM_REGISTER_TEST_PROFILE_CHIP_FM6373
static const DMD_SPWM_FM6373_Profile profiles[] = {
#include "register_profiles/fm6373/fm6373_scan_32.h"
};
...
dmd.init();
dmdApplyRegProfile(dmd, *dmdFindRegProfile(profiles, 42));
```

## Migrating from DMD_STM32

* Change `#include "DMD_SPWM_Driver.h"` to `#include <DMD_SPWM.h>`. The class names stay the same, and the color-depth template argument is accepted but ignored.
* Replace GFX fonts and `drawMarquee`/`stepMarquee` with DMD fonts and `drawString()` at a moving x position, then `update()`. See `examples/spwm_basic`.
* `setTextColor()` is gone. Colors are passed straight to `drawString()`.

## License

GPL v3, the same as the original DMD_STM32 by Dmitry Dmitriev (board707).
