#pragma once
/*--------------------------------------------------------------------------------------
 STM_PWM - HUB75 LED panel library for S-PWM driver chips
 STM32duino (Arduino_Core_STM32) port for STM32F1 / STM32F4

 Based on DMD_STM32 by Dmitry Dmitriev (board707), https://github.com/board707/DMD_STM32
 Reduced to S-PWM driver chips only, Adafruit_GFX removed: RGB565 framebuffer,
 setPixel() and simple DMD bitmap fonts.

 Supported chips (three control architectures):
   * GCLK-only     : FM6353, ICN2153
   * GCLK + DCLK   : FM6363
   * Decoupled     : DP3264, ICN2055, FM6373, SM16380SH, ICND1065

 This program is free software: you can redistribute it and/or modify it under the terms
 of the version 3 GNU General Public License as published by the Free Software Foundation.
--------------------------------------------------------------------------------------*/
#include <Arduino.h>
#include "DMD_SPWM_Config.h"
#include "DMD_SPWM_Panels.h"

#define DMD_TRANSPARENT 0x10000UL      // "no background" value for text/bitmaps

// Panel connection variants
#define CONNECT_NORMAL 0
#define CONNECT_ZIGZAG 2

enum class DMD_Color_order : uint8_t { RGB = 0, RBG, BRG, BGR, GRB, GBR };
enum MUX_TYPE { DMD_MUX_TYPE138 = 0, DMD_MUX_TYPE_SHIFTREG = 1 };

struct DMD_PanelCfg {
	uint8_t mux_cnt;    // A,B,C... line count, 33 = shift register mux
	uint8_t width;      // single panel width
	uint8_t height;     // single panel height
	uint8_t scan;       // number of multiplexed rows
	uint8_t pattern;    // pixel layout, see DMD_SPWM_Panels.h
};

/*--------------------------------------------------------------------------------------*/
// Row multiplexer: binary/direct decoder (A,B,C,..) or 595-type shift register
/*--------------------------------------------------------------------------------------*/
class DMD_Mux {
public:
	DMD_Mux(const uint8_t* pins, uint8_t pin_cnt, uint8_t n_rows, MUX_TYPE type);
	~DMD_Mux();
	void begin();
	void set(uint8_t row);
	MUX_TYPE type;
private:
	uint8_t pins[8];
	uint8_t cnt;
	uint8_t n_rows;
	uint8_t last_row = 0xFF;
	uint32_t* masks = nullptr;
	volatile uint32_t* bsrr = nullptr;
	uint32_t clk_m = 0, lat_m = 0, sdi_m = 0;
};

/*--------------------------------------------------------------------------------------*/
// Base class: framebuffer, drawing, text, HUB75 protocol and timers.
// Chip classes below only set timing values and default config registers.
/*--------------------------------------------------------------------------------------*/
class DMD_SPWM {
public:
	DMD_SPWM(const DMD_PanelCfg& panel, uint8_t* mux_list, uint8_t pin_nOE, uint8_t pin_SCLK,
		uint8_t* rgbpins, uint8_t panelsWide = 1, uint8_t panelsHigh = 1);
	virtual ~DMD_SPWM();

	// Configure pins, timers and chip. Returns false if the OE pin has no
	// channel on the OE timer or pins are not on the required common ports.
	// The argument is ignored (kept for compatibility).
	bool init(uint16_t unused = 0);

	// ---------- framebuffer (RGB565) ----------
	int16_t width() const { return _width; }
	int16_t height() const { return _height; }
	void setPixel(int16_t x, int16_t y, uint16_t color);
	void drawPixel(int16_t x, int16_t y, uint16_t color) { setPixel(x, y, color); }
	uint16_t getPixel(int16_t x, int16_t y);
	void fillScreen(uint16_t color);
	void clearScreen(bool = true) { fillScreen(0); }
	void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
	void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
	void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) { fillRect(x, y, w, 1, color); }
	void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) { fillRect(x, y, 1, h, color); }
	void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
	// RGB565 image, w*h words, row-major
	void drawBitmap(int16_t x, int16_t y, const uint16_t* bitmap, int16_t w, int16_t h);
	// 1-bit image, row-major, MSB first, each row padded to a byte
	void drawMonoBitmap(int16_t x, int16_t y, const uint8_t* bitmap, int16_t w, int16_t h,
		uint16_t fg, uint32_t bg = DMD_TRANSPARENT);
	uint16_t* getBuffer() { return fb; }
	// Framebuffer word index of pixel (x,y) as used by setPixel(), -1 if outside.
	// Same mapping as extras/pc/spwm_fb.py
	int32_t fbIndex(int16_t x, int16_t y);
	uint32_t fbWords() const { return (uint32_t)WIDTH * HEIGHT; }

	void setRotation(uint8_t r) { rotation = r & 3; _width = (rotation & 1) ? HEIGHT : WIDTH; _height = (rotation & 1) ? WIDTH : HEIGHT; }
	uint8_t getRotation() const { return rotation; }
	void setConnectScheme(uint8_t sch) { connectScheme = sch; }

	// ---------- text (DMD bitmap fonts, see src/fonts) ----------
	void selectFont(const uint8_t* font, uint8_t charSpacing = 1);
	int drawChar(int16_t x, int16_t y, unsigned char c, uint16_t fg, uint32_t bg = DMD_TRANSPARENT);
	// returns the x coordinate after the last char
	int drawString(int16_t x, int16_t y, const char* s, uint16_t fg, uint32_t bg = DMD_TRANSPARENT);
	int charWidth(unsigned char c);
	uint16_t stringWidth(const char* s);
	uint8_t fontHeight() const;

	// ---------- colors ----------
	static uint16_t Color888(uint8_t r, uint8_t g, uint8_t b) {
		return ((uint16_t)(r & 0xF8) << 8) | ((uint16_t)(g & 0xFC) << 3) | (b >> 3);
	}
	static uint16_t Color444(uint8_t r, uint8_t g, uint8_t b) { return Color888(r * 17, g * 17, b * 17); }
	static uint16_t Color333(uint8_t r, uint8_t g, uint8_t b) { return Color888(r * 36, g * 36, b * 36); }
	static uint16_t ColorHSV(int hue, uint8_t sat, uint8_t val);
	void setColorOrder(DMD_Color_order order);

	// ---------- output ----------
	// Send framebuffer (and config registers) to the panel. The panel keeps the
	// image in its own SRAM, so call it only when the picture has changed.
	void update();
	void swapBuffers(bool copy = true) { (void)copy; update(); }   // compatibility
	// 0..255, scales greyscale values (applied on next update())
	void setBrightness(uint8_t level);
	// Move color data down by <shift> bits of greyscale (old brightness method).
	// Returns the shift actually applied.
	uint8_t shiftColorBrightnessDown(uint8_t shift);
	// Call before init() (or init() again) to select a 595-type row driver
	void configure_multiplexer(MUX_TYPE mux);

	// ---------- chip config registers ----------
	// Replace the built-in register set. Words are in datasheet / catalog order.
	// g and b may be nullptr to send <r> to all three color lanes.
	bool setConfigRegisters(const uint16_t* r, uint8_t count, const uint16_t* g = nullptr, const uint16_t* b = nullptr);
	void resetConfigRegisters();               // back to the built-in set
	// Number of update() cycles needed to load the whole register set
	uint8_t configLoadCycles() const { return (cfg_mode == CFG_ALL) ? 1 : cfg_cnt; }
	// Load the whole register set now (runs update() configLoadCycles() times)
	void applyConfigRegisters(uint16_t delay_ms = 30);

	void isr();     // row switching interrupt (internal use)

protected:
	enum Family : uint8_t { FAM_GCLK = 0, FAM_DCLK, FAM_DECOUPLED };
	enum CfgMode : uint8_t { CFG_ALL = 0, CFG_ONE_FRAMED, CFG_ONE_PLAIN };

	// --- set by chip classes in setup_chip() ---
	virtual void setup_chip() = 0;
	// map a register transmission slot to the stored (catalog) index
	virtual uint8_t cfg_slot(uint8_t slot) { return slot; }
	void set_builtin_regs(const uint16_t* regs, uint8_t count);
	Family family = FAM_GCLK;
	CfgMode cfg_mode = CFG_ALL;
	uint8_t gclk_bits = 14;
	uint32_t main_reload = 0, main_cc1 = 0, main_cc2 = 0;
	uint32_t oe_reload = 0, oe_cc = 0;
	uint32_t clk_period = 8;

	// panel geometry
	const DMD_PanelCfg panel;
	const uint8_t panelsWide, panelsHigh;
	const int16_t WIDTH, HEIGHT;
	int16_t _width, _height;
	uint8_t rotation = 0;
	uint8_t connectScheme = CONNECT_NORMAL;
	const uint8_t nRows;
	const uint8_t pol_displ;
	const uint8_t multiplex;
	const uint16_t x_len;     // pixels per scan line in the driver chain
	uint16_t* fb = nullptr;   // [row][chain position][upper/lower]
	int32_t panel_index(int16_t x, int16_t y);

	// pins
	uint8_t pin_oe, pin_lat, pin_clk;
	uint8_t rgb_pins[6];
	uint8_t mux_pins[8];
	uint8_t mux_cnt;
	DMD_Mux* mux = nullptr;
	MUX_TYPE mux_type;
	DMD_Color_order color_order = DMD_Color_order::RGB;

	volatile uint32_t* datasetreg = nullptr;   // BSRR of RGB/CLK port
	volatile uint32_t* latsetreg = nullptr;
	volatile uint32_t* oesetreg = nullptr;
	uint32_t clkmask = 0, clk_clrmask = 0, rgbmask_all = 0, latmask = 0, oemask = 0;
	uint32_t lane_mask[3] = { 0, 0, 0 };       // physical R, G, B (upper|lower)
	uint32_t expand[64];
	uint32_t dclk_strobe[2] = { 0, 0 };
	uint8_t oe_channel = 0;

	// greyscale
	uint8_t brightness = 255;
	uint8_t dimming_factor = 0;
	uint16_t lut6[64];
	uint16_t lut5[32];
	void build_lut();
	void build_expand();

	// config registers
	uint16_t* cfg[3] = { nullptr, nullptr, nullptr };
	uint8_t cfg_cnt = 0;
	uint8_t cfg_next = 0;
	uint16_t* builtin = nullptr;
	uint8_t builtin_cnt = 0;

	// scan state
	volatile uint8_t row = 0;
	volatile bool oe_scan_flag = false;
	volatile bool oe_scan_res = false;
	bool initialized = false;

	// protocol
	void send_cfg_word(uint16_t r, uint16_t g, uint16_t b, uint16_t latches);
	void send_allRGB(uint16_t data, uint16_t latches) { send_cfg_word(data, data, data, latches); }
	void send_cfg_slot(uint8_t slot, uint16_t latches);
	void send_latches(uint16_t latches);
	void send_clocks(uint16_t clocks);
	void send_vsync();
	void load_config_regs();
	void send_pair(const uint16_t* p, bool latch, bool oe_pulse);
	void data_transfer();
	void data_transfer_decoupled();

	// timers
	bool hw_init();
	void stop_GCLK();
	void start_GCLK();
	void start_DCLK();
	void dclk_dma(bool on);
	void oe_mode(bool af);
	volatile uint32_t* oe_cfg_reg = nullptr;
	uint32_t oe_cfg_mask = 0, oe_cfg_af = 0, oe_cfg_out = 0;

	// font
	const uint8_t* font = nullptr;
	uint8_t interCharSpace = 1;
	void transform_XY(int16_t& x, int16_t& y);
};

/*--------------------------------------------------------------------------------------*/
// Chip classes
/*--------------------------------------------------------------------------------------*/
#define DMD_SPWM_CHIP_CLASS(NAME)                                                         \
class NAME : public DMD_SPWM {                                                            \
public:                                                                                   \
	NAME(const DMD_PanelCfg& p, uint8_t* mux_list, uint8_t pin_nOE, uint8_t pin_SCLK,    \
		uint8_t* rgbpins, uint8_t panelsWide = 1, uint8_t panelsHigh = 1)                  \
		: DMD_SPWM(p, mux_list, pin_nOE, pin_SCLK, rgbpins, panelsWide, panelsHigh) {}    \
protected:                                                                                \
	void setup_chip() override;

DMD_SPWM_CHIP_CLASS(DMD_SPWM_FM6353) };      // also ICN2153
DMD_SPWM_CHIP_CLASS(DMD_SPWM_FM6363)
	uint8_t cfg_slot(uint8_t slot) override { return (uint8_t)((slot + cfg_cnt - 1) % cfg_cnt); }
};
DMD_SPWM_CHIP_CLASS(DMD_SPWM_DP3264) };
DMD_SPWM_CHIP_CLASS(DMD_SPWM_ICN2055) };
DMD_SPWM_CHIP_CLASS(DMD_SPWM_FM6373) };
DMD_SPWM_CHIP_CLASS(DMD_SPWM_SM16380SH) };
DMD_SPWM_CHIP_CLASS(DMD_SPWM_ICN1065) };
typedef DMD_SPWM_FM6353 DMD_SPWM_ICN2153;

/*--------------------------------------------------------------------------------------*/
// Template front-ends with the familiar DMD_STM32 syntax:
//   DMD_RGB_FM6373<RGB128x64plainS32> dmd(mux_list, OE, LAT, rgbpins, 1, 1);
// An old color-depth argument (COLOR_4BITS...) and d_buf flag are accepted and ignored.
/*--------------------------------------------------------------------------------------*/
#define DMD_SPWM_TEMPLATE(TNAME, BASE)                                                    \
template <int MUX_CNT, int P_W, int P_H, int SCAN, int PATTERN, int... Ignored>           \
class TNAME : public BASE {                                                               \
public:                                                                                   \
	TNAME(uint8_t* mux_list, uint8_t pin_nOE, uint8_t pin_SCLK, uint8_t* rgbpins,        \
		uint8_t panelsWide = 1, uint8_t panelsHigh = 1, bool d_buf = false)               \
		: BASE(DMD_PanelCfg{ MUX_CNT, P_W, P_H, SCAN, PATTERN }, mux_list, pin_nOE,       \
			pin_SCLK, rgbpins, panelsWide, panelsHigh) { (void)d_buf; }                    \
};

DMD_SPWM_TEMPLATE(DMD_RGB_FM6353, DMD_SPWM_FM6353)
DMD_SPWM_TEMPLATE(DMD_RGB_ICN2153, DMD_SPWM_FM6353)
DMD_SPWM_TEMPLATE(DMD_RGB_FM6363, DMD_SPWM_FM6363)
DMD_SPWM_TEMPLATE(DMD_RGB_DP3264, DMD_SPWM_DP3264)
DMD_SPWM_TEMPLATE(DMD_RGB_ICN2055, DMD_SPWM_ICN2055)
DMD_SPWM_TEMPLATE(DMD_RGB_FM6373, DMD_SPWM_FM6373)
DMD_SPWM_TEMPLATE(DMD_RGB_SM16380SH, DMD_SPWM_SM16380SH)
DMD_SPWM_TEMPLATE(DMD_RGB_ICN1065, DMD_SPWM_ICN1065)
