/*--------------------------------------------------------------------------------------
 STM_PWM - basic demo for HUB75 panels with S-PWM driver chips
 STM32duino core, STM32F1 (BluePill) / STM32F4 (BlackPill F401/F411)

 The panel keeps the picture in its own SRAM: draw into the framebuffer,
 then call dmd.update() once per changed frame.
--------------------------------------------------------------------------------------*/
#include <DMD_SPWM.h>
#include "fonts/SystemFont5x7.h"
#include "fonts/Arial_black_16.h"

#define DISPLAYS_ACROSS 1
#define DISPLAYS_DOWN   1

// Mux pins A,B,C,D,E: all on one port.
// RGB pins and CLK: all on one port (any pins of that port).
// OE pin: a TIM3 channel (PA6 PA7 PB0 PB1), or TIM4 (PB6..PB9) with DMD_ALTERNATIVE_TIMERS.
uint8_t mux_list[] = { PB6, PB5, PB4, PB3, PB8 };
#define DMD_PIN_nOE  PB0
#define DMD_PIN_SCLK PB7     // LAT

#if defined(STM32F1xx)
uint8_t rgbpins[] = { PA15, PA0, PA1, PA2, PA3, PA4, PA5 };   // CLK, R0, G0, B0, R1, G1, B1
// 64x32 keeps the framebuffer at 4 KB (STM32F103C8 has 20 KB RAM)
DMD_RGB_FM6373<RGB64x32plainS16> dmd(mux_list, DMD_PIN_nOE, DMD_PIN_SCLK, rgbpins, DISPLAYS_ACROSS, DISPLAYS_DOWN);
#else
uint8_t rgbpins[] = { PA6, PA0, PA1, PA2, PA3, PA4, PA5 };    // CLK, R0, G0, B0, R1, G1, B1
DMD_RGB_FM6373<RGB128x64plainS32> dmd(mux_list, DMD_PIN_nOE, DMD_PIN_SCLK, rgbpins, DISPLAYS_ACROSS, DISPLAYS_DOWN);
#endif

// Other chips (same constructor):
//   DMD_RGB_FM6353 (=ICN2153), DMD_RGB_FM6363, DMD_RGB_DP3264,
//   DMD_RGB_ICN2055, DMD_RGB_SM16380SH, DMD_RGB_ICN1065
// Panel types: see DMD_SPWM_Panels.h

void showBars()
{
	int16_t w = dmd.width(), h = dmd.height();
	int16_t band = h / 4;
	for (int16_t x = 0; x < w; x++) {
		uint8_t v = (uint32_t)x * 255 / (w - 1);
		dmd.drawFastVLine(x, 0, band, dmd.Color888(v, 0, 0));
		dmd.drawFastVLine(x, band, band, dmd.Color888(0, v, 0));
		dmd.drawFastVLine(x, 2 * band, band, dmd.Color888(0, 0, v));
		dmd.drawFastVLine(x, 3 * band, h - 3 * band, dmd.Color888(v, v, v));
	}
	dmd.update();
}

void setup()
{
	Serial.begin(115200);

	// options that must be set before init():
	//dmd.configure_multiplexer(DMD_MUX_TYPE_SHIFTREG);  // 595-type row driver
	//dmd.setColorOrder(DMD_Color_order::BRG);

	if (!dmd.init()) {
		Serial.println("dmd.init() failed: OE pin has no OE timer channel, or RGB/CLK pins are not on one port");
		while (1) {}
	}
	showBars();
	delay(3000);
}

void loop()
{
	static const char msg[] = "Hello STM32duino!";
	uint16_t colors[] = { dmd.Color888(255, 0, 0), dmd.Color888(0, 255, 0),
		dmd.Color888(0, 0, 255), dmd.Color888(255, 200, 0) };

	// 1. scrolling text
	dmd.selectFont(Arial_Black_16, 1);
	int16_t tw = dmd.stringWidth(msg);
	int16_t y = (dmd.height() - dmd.fontHeight()) / 2;
	for (uint8_t pass = 0; pass < 4; pass++) {
		for (int16_t x = dmd.width(); x > -tw; x--) {
			dmd.fillScreen(0);
			dmd.drawString(x, y, msg, colors[pass]);
			dmd.update();
			delay(20);
		}
	}

	// 2. small font, rainbow, brightness fade
	dmd.fillScreen(0);
	dmd.selectFont(System5x7, 1);
	for (int16_t x = 0; x < dmd.width(); x++)
		dmd.drawFastVLine(x, dmd.height() - 8, 8, dmd.ColorHSV(x * 360 / dmd.width(), 255, 255));
	dmd.drawString(1, 1, "SPWM", dmd.Color888(255, 255, 255));
	dmd.drawRect(0, 0, dmd.width(), dmd.height() - 8, dmd.Color888(0, 80, 255));
	for (int b = 255; b >= 10; b -= 5) { dmd.setBrightness(b); dmd.update(); delay(15); }
	for (int b = 10; b <= 255; b += 5) { dmd.setBrightness(b); dmd.update(); delay(15); }
	dmd.setBrightness(255);
	delay(1500);

	// 3. gradient bars
	showBars();
	delay(3000);
}
