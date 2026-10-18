/*--------------------------------------------------------------------------------------
 DMD_SPWM.cpp - framebuffer, drawing, fonts and S-PWM protocol

 Based on DMD_STM32 (c) Dmitry Dmitriev (board707), GPL v3
--------------------------------------------------------------------------------------*/
#include "DMD_SPWM.h"

// gamma 2.2, 6-bit input -> 16-bit output
static const uint16_t gamma16[64] = {
	    0,     7,    33,    81,   152,   249,   371,   521,
	  699,   906,  1143,  1409,  1707,  2035,  2396,  2788,
	 3214,  3672,  4164,  4690,  5250,  5845,  6475,  7140,
	 7841,  8578,  9351, 10161, 11007, 11890, 12811, 13770,
	14766, 15800, 16872, 17983, 19133, 20322, 21550, 22817,
	24124, 25471, 26858, 28285, 29752, 31260, 32809, 34398,
	36029, 37701, 39415, 41170, 42967, 44805, 46686, 48610,
	50575, 52583, 54634, 56728, 58865, 61045, 63268, 65535
};

// DMD font header layout
#define FONT_LENGTH        0
#define FONT_FIXED_WIDTH   2
#define FONT_HEIGHT        3
#define FONT_FIRST_CHAR    4
#define FONT_CHAR_COUNT    5
#define FONT_WIDTH_TABLE   6

static const uint8_t MIN_GCLK = 8;

/*======================================================================================*/
// DMD_Mux
/*======================================================================================*/
static inline __attribute__((always_inline)) void mux_delay(volatile uint32_t n) { while (n--) __asm volatile("nop"); }

DMD_Mux::DMD_Mux(const uint8_t* p, uint8_t pin_cnt, uint8_t rows, MUX_TYPE t)
	: type(t), cnt(pin_cnt > 8 ? 8 : pin_cnt), n_rows(rows)
{
	memcpy(pins, p, cnt);
}

DMD_Mux::~DMD_Mux() { free(masks); }

void DMD_Mux::begin()
{
	for (uint8_t i = 0; i < cnt; i++) {
		pinMode(pins[i], OUTPUT);
		digitalWrite(pins[i], LOW);
	}
	bsrr = &digitalPinToPort(pins[0])->BSRR;
	last_row = 0xFF;
	if (type == DMD_MUX_TYPE_SHIFTREG) {
		clk_m = digitalPinToBitMask(pins[0]);
		lat_m = digitalPinToBitMask(pins[1]);
		sdi_m = digitalPinToBitMask(pins[2]);
		return;
	}
	free(masks);
	masks = (uint32_t*)malloc((n_rows + 1) * sizeof(uint32_t));
	for (uint8_t i = 0; i < n_rows; i++) {
		masks[i] = 0;
		if (cnt == n_rows) {                       // DIRECT mux: selected line LOW
			for (uint8_t j = 0; j < n_rows; j++) {
				uint32_t m = digitalPinToBitMask(pins[j]);
				masks[i] |= (i == j) ? (m << 16) : m;
			}
		}
		else {                                     // BINARY mux
			for (uint8_t j = 0; (1 << j) < n_rows && j < cnt; j++) {
				uint32_t m = digitalPinToBitMask(pins[j]);
				masks[i] |= (i & (1 << j)) ? m : (m << 16);
			}
		}
	}
	masks[n_rows] = masks[0];
}

void DMD_Mux::set(uint8_t r)
{
	if (type != DMD_MUX_TYPE_SHIFTREG) {
		*bsrr = masks[r];
		return;
	}
	// 595-type row driver: shift a single "1" for row 0, "0" for the next rows
	if (r == last_row) return;
	*bsrr = lat_m;
	*bsrr = (r == 0) ? sdi_m : (sdi_m << 16);
	mux_delay(1);
	*bsrr = clk_m;
	mux_delay(1);
	*bsrr = clk_m << 16;
	*bsrr = lat_m << 16;
	last_row = r;
}

/*======================================================================================*/
// DMD_SPWM - construction / init
/*======================================================================================*/
DMD_SPWM::DMD_SPWM(const DMD_PanelCfg& p, uint8_t* mux_list, uint8_t pin_nOE, uint8_t pin_SCLK,
	uint8_t* rgbpins, uint8_t pW, uint8_t pH)
	: panel(p), panelsWide(pW), panelsHigh(pH),
	WIDTH(pW * p.width), HEIGHT(pH * p.height), _width(pW * p.width), _height(pH * p.height),
	nRows(p.scan), pol_displ(p.height / 2), multiplex((p.height / 2) / p.scan),
	x_len(pW * p.width * ((p.height / 2) / p.scan) * pH),
	pin_oe(pin_nOE), pin_lat(pin_SCLK)
{
	pin_clk = rgbpins[0];
	memcpy(rgb_pins, rgbpins + 1, 6);
	if (p.mux_cnt == DMD_MUX_SHIFTREG_CODE) { mux_cnt = 3; mux_type = DMD_MUX_TYPE_SHIFTREG; }
	else { mux_cnt = p.mux_cnt > 8 ? 8 : p.mux_cnt; mux_type = DMD_MUX_TYPE138; }
	memcpy(mux_pins, mux_list, mux_cnt);
	fb = (uint16_t*)calloc((size_t)WIDTH * HEIGHT, sizeof(uint16_t));
}

DMD_SPWM::~DMD_SPWM()
{
	free(fb);
	free(cfg[0]);
	free(builtin);
	delete mux;
}

void DMD_SPWM::configure_multiplexer(MUX_TYPE m)
{
	mux_type = m;
	if (m == DMD_MUX_TYPE_SHIFTREG && mux_cnt > 3) mux_cnt = 3;
	if (initialized) {
		delete mux;
		mux = new DMD_Mux(mux_pins, mux_cnt, nRows, mux_type);
		mux->begin();
	}
}

bool DMD_SPWM::init(uint16_t unused)
{
	(void)unused;
	if (!fb) return false;
	initialized = false;
	setup_chip();
	if (cfg_cnt == 0) resetConfigRegisters();

	delete mux;
	mux = new DMD_Mux(mux_pins, mux_cnt, nRows, mux_type);
	mux->begin();

	if (!hw_init()) return false;
	build_lut();
	fillScreen(0);
	row = 0;
	oe_scan_flag = false;
	initialized = true;
	// one-register-per-frame chips need the whole set loaded at start
	applyConfigRegisters(family == FAM_DECOUPLED ? 30 : 1);
	return true;
}

/*======================================================================================*/
// config registers
/*======================================================================================*/
void DMD_SPWM::set_builtin_regs(const uint16_t* regs, uint8_t count)
{
	free(builtin);
	builtin = (uint16_t*)malloc(count * sizeof(uint16_t));
	memcpy(builtin, regs, count * sizeof(uint16_t));
	builtin_cnt = count;
}

bool DMD_SPWM::setConfigRegisters(const uint16_t* r, uint8_t count, const uint16_t* g, const uint16_t* b)
{
	if (!r || !count) return false;
	uint16_t* buf = (uint16_t*)malloc(3u * count * sizeof(uint16_t));
	if (!buf) return false;
	memcpy(buf, r, count * 2);
	memcpy(buf + count, g ? g : r, count * 2);
	memcpy(buf + 2 * count, b ? b : r, count * 2);
	free(cfg[0]);
	cfg[0] = buf; cfg[1] = buf + count; cfg[2] = buf + 2 * count;
	cfg_cnt = count;
	cfg_next = 0;
	return true;
}

void DMD_SPWM::resetConfigRegisters()
{
	if (builtin) setConfigRegisters(builtin, builtin_cnt);
}

void DMD_SPWM::applyConfigRegisters(uint16_t delay_ms)
{
	cfg_next = 0;
	uint8_t n = configLoadCycles();
	for (uint8_t i = 0; i < n; i++) {
		update();
		if (delay_ms) delay(delay_ms);
	}
}

/*======================================================================================*/
// colors / greyscale
/*======================================================================================*/
void DMD_SPWM::setColorOrder(DMD_Color_order order)
{
	color_order = order;
	if (initialized) build_expand();
}

// expand[] : 6-bit lane index (R0 G0 B0 R1 G1 B1 of the *image*) -> port bits
void DMD_SPWM::build_expand()
{
	// which image component goes to physical pin R, G, B
	static const uint8_t src_for_pin[6][3] = {
		{0, 1, 2}, {0, 2, 1}, {2, 0, 1}, {2, 1, 0}, {1, 0, 2}, {1, 2, 0} };
	uint8_t pin_for_src[3];
	const uint8_t* s = src_for_pin[(uint8_t)color_order];
	for (uint8_t p = 0; p < 3; p++) pin_for_src[s[p]] = p;

	uint32_t m[6];
	for (uint8_t i = 0; i < 6; i++)
		m[i] = digitalPinToBitMask(rgb_pins[pin_for_src[i % 3] + 3 * (i / 3)]);
	for (uint8_t i = 0; i < 64; i++) {
		uint32_t v = 0;
		for (uint8_t b = 0; b < 6; b++) if (i & (1 << b)) v |= m[b];
#if DMD_CLK_WITH_DATA
		v |= clkmask;              // CLK rises together with the data
#endif
		expand[i] = v;
	}
}

void DMD_SPWM::build_lut()
{
	uint8_t msb = gclk_bits - dimming_factor;
	for (uint8_t i = 0; i < 64; i++) {
		uint32_t v = ((uint32_t)gamma16[i] * brightness) / 255;
		v >>= (16 - msb);
		if (i && !v && brightness) v = 1;
		lut6[i] = (uint16_t)v;
	}
	for (uint8_t i = 0; i < 32; i++) lut5[i] = lut6[(i << 1) | (i >> 4)];
}

void DMD_SPWM::setBrightness(uint8_t level)
{
	brightness = level;
	build_lut();
}

uint8_t DMD_SPWM::shiftColorBrightnessDown(uint8_t shift)
{
	uint8_t max_shift = gclk_bits - MIN_GCLK;
	dimming_factor = (shift > max_shift) ? max_shift : shift;
	build_lut();
	return dimming_factor;
}

uint16_t DMD_SPWM::ColorHSV(int hue, uint8_t sat, uint8_t val)
{
	const uint16_t hue_range = 360, hue_sect = hue_range / 6;
	uint16_t r, g, b, base;
	if (sat == 0) { r = g = b = val; }
	else {
		base = ((255 - sat) * val) >> 8;
		hue = hue % hue_range;
		if (hue < 0) hue += hue_range;
		uint16_t rise = (((val - base) * (hue % hue_sect)) / hue_sect) + base;
		uint16_t fall = (((val - base) * (hue_sect - (hue % hue_sect))) / hue_sect) + base;
		switch (hue / hue_sect) {
		case 0:  r = val;  g = rise; b = base; break;
		case 1:  r = fall; g = val;  b = base; break;
		case 2:  r = base; g = val;  b = rise; break;
		case 3:  r = base; g = fall; b = val;  break;
		case 4:  r = rise; g = base; b = val;  break;
		default: r = val;  g = base; b = fall; break;
		}
	}
	return Color888(r, g, b);
}

/*======================================================================================*/
// framebuffer
/*======================================================================================*/
void DMD_SPWM::transform_XY(int16_t& x, int16_t& y)
{
	int16_t t;
	switch (rotation) {
	case 1: t = x; x = WIDTH - 1 - y; y = t; break;
	case 2: x = WIDTH - 1 - x; y = HEIGHT - 1 - y; break;
	case 3: t = x; x = y; y = HEIGHT - 1 - t; break;
	}
	if (connectScheme == CONNECT_ZIGZAG) {
		int16_t prow = y / panel.height;
		if (prow & 1) {
			x = (WIDTH - 1) - x;
			y = prow * panel.height + (panel.height - 1) - (y % panel.height);
		}
	}
}

// Physical (unrotated) x,y -> framebuffer index.
// Buffer order = order of data in the driver chain:
//   [scan row][chain position][0 = upper half (R0G0B0), 1 = lower half (R1G1B1)]
int32_t DMD_SPWM::panel_index(int16_t x, int16_t y)
{
	const uint8_t PW = panel.width, PH = panel.height;
	uint8_t pol_y = y % pol_displ;
	uint32_t base;
	if (panel.pattern == 0 && multiplex == 1) {
		base = (uint32_t)pol_y * WIDTH * panelsHigh + (y / PH) * WIDTH + x;
	}
	else {
		x += (y / PH) * WIDTH;
		switch (panel.pattern) {
		case 1:
			base = (uint32_t)(pol_y / multiplex) * x_len + (x / PW) * multiplex * PW + (pol_y % multiplex) * PW + x % PW;
			break;
		case 2:
			base = (uint32_t)(pol_y % nRows) * x_len + (x / PW) * multiplex * PW + (pol_y / nRows) * PW + x % PW;
			break;
		case 3:
			base = (uint32_t)(pol_y % nRows) * x_len + (x / PW) * multiplex * PW + x % PW;
			if (pol_y < nRows) base += PW;
			break;
		default:
			base = (uint32_t)(pol_y % nRows) * x_len + (x / 8) * multiplex * 8 + (pol_y / nRows) * 8 + x % 8;
			break;
		}
	}
	return (int32_t)(base * 2 + (((y % PH) >= pol_displ) ? 1 : 0));
}

void DMD_SPWM::setPixel(int16_t x, int16_t y, uint16_t color)
{
	if (x < 0 || y < 0 || x >= _width || y >= _height) return;
	transform_XY(x, y);
	fb[panel_index(x, y)] = color;
}

int32_t DMD_SPWM::fbIndex(int16_t x, int16_t y)
{
	if (x < 0 || y < 0 || x >= _width || y >= _height) return -1;
	transform_XY(x, y);
	return panel_index(x, y);
}

uint16_t DMD_SPWM::getPixel(int16_t x, int16_t y)
{
	if (x < 0 || y < 0 || x >= _width || y >= _height) return 0;
	transform_XY(x, y);
	return fb[panel_index(x, y)];
}

void DMD_SPWM::fillScreen(uint16_t color)
{
	uint32_t n = (uint32_t)WIDTH * HEIGHT;
	for (uint32_t i = 0; i < n; i++) fb[i] = color;
}

void DMD_SPWM::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
	if (w < 0) { x += w + 1; w = -w; }
	if (h < 0) { y += h + 1; h = -h; }
	int16_t x1 = x + w, y1 = y + h;
	if (x < 0) x = 0;
	if (y < 0) y = 0;
	if (x1 > _width) x1 = _width;
	if (y1 > _height) y1 = _height;
	for (int16_t j = y; j < y1; j++)
		for (int16_t i = x; i < x1; i++) {
			int16_t tx = i, ty = j;
			transform_XY(tx, ty);
			fb[panel_index(tx, ty)] = color;
		}
}

void DMD_SPWM::drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
	fillRect(x, y, w, 1, color);
	fillRect(x, y + h - 1, w, 1, color);
	fillRect(x, y, 1, h, color);
	fillRect(x + w - 1, y, 1, h, color);
}

void DMD_SPWM::drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
	int16_t dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
	int16_t dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
	int16_t err = dx + dy;
	for (;;) {
		setPixel(x0, y0, color);
		if (x0 == x1 && y0 == y1) break;
		int16_t e2 = 2 * err;
		if (e2 >= dy) { err += dy; x0 += sx; }
		if (e2 <= dx) { err += dx; y0 += sy; }
	}
}

void DMD_SPWM::drawBitmap(int16_t x, int16_t y, const uint16_t* bmp, int16_t w, int16_t h)
{
	for (int16_t j = 0; j < h; j++)
		for (int16_t i = 0; i < w; i++)
			setPixel(x + i, y + j, bmp[j * w + i]);
}

void DMD_SPWM::drawMonoBitmap(int16_t x, int16_t y, const uint8_t* bmp, int16_t w, int16_t h,
	uint16_t fg, uint32_t bg)
{
	int16_t bw = (w + 7) / 8;
	for (int16_t j = 0; j < h; j++)
		for (int16_t i = 0; i < w; i++) {
			if (bmp[j * bw + i / 8] & (0x80 >> (i & 7))) setPixel(x + i, y + j, fg);
			else if (bg != DMD_TRANSPARENT) setPixel(x + i, y + j, (uint16_t)bg);
		}
}

/*======================================================================================*/
// fonts (DMD format: see src/fonts/SystemFont5x7.h header comment)
/*======================================================================================*/
void DMD_SPWM::selectFont(const uint8_t* f, uint8_t spacing)
{
	font = f;
	interCharSpace = spacing;
}

uint8_t DMD_SPWM::fontHeight() const { return font ? font[FONT_HEIGHT] : 0; }

int DMD_SPWM::charWidth(unsigned char c)
{
	if (!font) return 0;
	uint8_t first = font[FONT_FIRST_CHAR], count = font[FONT_CHAR_COUNT];
	if (c < first || c >= first + count) return 0;
	if (font[FONT_LENGTH] == 0 && font[FONT_LENGTH + 1] == 0) return font[FONT_FIXED_WIDTH];
	uint8_t w = font[FONT_WIDTH_TABLE + (c - first)];
	if (c == ' ' && w == 0) {
		if ('n' >= first && 'n' < first + count) w = font[FONT_WIDTH_TABLE + ('n' - first)];
		else w = 5;
	}
	return w;
}

int DMD_SPWM::drawChar(int16_t x, int16_t y, unsigned char c, uint16_t fg, uint32_t bg)
{
	if (!font) return 0;
	uint8_t first = font[FONT_FIRST_CHAR], count = font[FONT_CHAR_COUNT];
	if (c < first || c >= first + count) return 0;
	uint8_t height = font[FONT_HEIGHT];
	uint8_t width = charWidth(c);
	if (x >= _width || y >= _height || x + width < 0 || y + height < 0) return width;

	bool mono = (font[FONT_LENGTH] == 0 && font[FONT_LENGTH + 1] == 0);
	if (c == ' ' && !mono) {
		if (bg != DMD_TRANSPARENT) fillRect(x, y, width, height, (uint16_t)bg);
		return width;
	}
	uint8_t bytes = (height + 7) / 8;
	uint8_t idx = c - first;
	uint32_t index = 0;
	if (mono) index = (uint32_t)idx * bytes * width + FONT_WIDTH_TABLE;
	else {
		for (uint8_t i = 0; i < idx; i++) index += font[FONT_WIDTH_TABLE + i];
		index = index * bytes + count + FONT_WIDTH_TABLE;
	}
	for (uint8_t j = 0; j < width; j++) {
		for (uint8_t i = 0; i < bytes; i++) {
			uint8_t data = font[index + j + i * width];
			int offset = (i == bytes - 1 && bytes > 1) ? (height - 8) : (i * 8);
			for (uint8_t k = 0; k < 8; k++) {
				int r = offset + k;
				if (r < i * 8 || r >= height) continue;
				if (data & (1 << k)) setPixel(x + j, y + r, fg);
				else if (bg != DMD_TRANSPARENT) setPixel(x + j, y + r, (uint16_t)bg);
			}
		}
	}
	return width;
}

int DMD_SPWM::drawString(int16_t x, int16_t y, const char* s, uint16_t fg, uint32_t bg)
{
	if (!font || !s) return x;
	uint8_t h = font[FONT_HEIGHT];
	bool first = true;
	while (*s) {
		int w = charWidth((unsigned char)*s);
		if (w > 0) {
			if (!first && interCharSpace) {
				if (bg != DMD_TRANSPARENT) fillRect(x, y, interCharSpace, h, (uint16_t)bg);
				x += interCharSpace;
			}
			drawChar(x, y, (unsigned char)*s, fg, bg);
			x += w;
			first = false;
		}
		if (x >= _width) break;
		s++;
	}
	return x;
}

uint16_t DMD_SPWM::stringWidth(const char* s)
{
	uint16_t w = 0;
	bool first = true;
	while (s && *s) {
		int cw = charWidth((unsigned char)*s++);
		if (cw > 0) { w += cw + (first ? 0 : interCharSpace); first = false; }
	}
	return w;
}

/*======================================================================================*/
// HUB75 S-PWM protocol
/*======================================================================================*/

// Register word to all driver chips of a line: 16 bits MSB first, repeated along the
// chain, LAT held high during the last <latches> clocks. Separate data per color lane.
void DMD_SPWM::send_cfg_word(uint16_t r, uint16_t g, uint16_t b, uint16_t latches)
{
	volatile uint32_t* ds = datasetreg;
	const uint32_t rgb_off = rgbmask_all << 16;
	for (uint16_t i = 0; i < x_len; i++) {
		const uint16_t m = 0x8000 >> (i & 15);
		uint32_t bits = 0;
		if (r & m) bits |= lane_mask[0];
		if (g & m) bits |= lane_mask[1];
		if (b & m) bits |= lane_mask[2];
		if (i == x_len - latches) *latsetreg = latmask;
		*ds = rgb_off | bits;          // BSRR: set bits win over reset bits
		*ds = clkmask;
		*ds = clkmask << 16;
	}
	*latsetreg = latmask << 16;
	*ds = rgb_off;
}

void DMD_SPWM::send_cfg_slot(uint8_t slot, uint16_t latches)
{
	uint8_t i = cfg_slot(slot);
	send_cfg_word(cfg[0][i], cfg[1][i], cfg[2][i], latches);
}

void DMD_SPWM::send_latches(uint16_t latches)
{
	*datasetreg = clk_clrmask;
	*latsetreg = latmask;
	while (latches--) {
		*datasetreg = clkmask;
		*datasetreg = clkmask << 16;
	}
	*latsetreg = latmask << 16;
}

void DMD_SPWM::send_clocks(uint16_t clocks)
{
	*datasetreg = clk_clrmask;
	while (clocks--) {
		*datasetreg = clkmask;
		*datasetreg = clkmask << 16;
	}
}

// VSYNC = 3 clocks LAT pulse
void DMD_SPWM::send_vsync()
{
	*datasetreg = clk_clrmask;
	*datasetreg = clkmask;
	send_latches(3);
}

void DMD_SPWM::load_config_regs()
{
	if (!cfg_cnt) return;
	switch (cfg_mode) {
	case CFG_ALL:                                  // FM6353 / FM6363: whole set every frame
		send_latches(14);                          // pre-active command
		send_latches(12);                          // enable all outputs
		send_vsync();
		for (uint8_t r = 0; r < cfg_cnt; r++) {
			delayMicroseconds(1);
			send_latches(14);
			send_cfg_slot(r, r * 2 + 2);
		}
		return;
	case CFG_ONE_FRAMED:                           // ICN2055 / FM6373 / SM16380SH / ICND1065
		send_vsync();
		send_clocks(8);
		send_latches(11);
		send_clocks(8);
		send_latches(14);
		send_clocks(8);
		send_allRGB(0x00aa, 5);
		send_allRGB(0x01aa, 5);
		send_cfg_slot(cfg_next, 5);
		send_allRGB(0x0055, 5);
		send_allRGB(0x0155, 5);
		send_clocks(8);
		break;
	case CFG_ONE_PLAIN:                            // DP3264
		send_vsync();
		send_clocks(16);
		send_latches(14);
		send_clocks(8);
		send_cfg_slot(cfg_next, 5);
		send_clocks(8);
		break;
	}
	if (++cfg_next >= cfg_cnt) cfg_next = 0;
}

#if DMD_CLK_WITH_DATA
// expand[] already holds the CLK bit: one write sets data + CLK, one clears both
#if DMD_DUAL_CLK_PULSE
#define PEW(v) { *ds = (v); *ds = (v); *ds = clr; }
#else
#define PEW(v) { *ds = (v); *ds = clr; }
#endif
#else
#if DMD_DUAL_CLK_PULSE
#define PEW(v) { *ds = (v); *ds = ck; *ds = ck; *ds = clr; }
#else
#define PEW(v) { *ds = (v); *ds = ck; *ds = clr; }
#endif
#endif

/*--------------------------------------------------------------------------------------
 Transpose eight bits of the six lane values into eight port patterns.

 The panel wants one bit of every lane per clock, but the greyscale values hold one
 lane each. Doing that bit by bit cost about 16 instructions per clock; this is
 Hacker's Delight transpose8rS32 and costs about 3. It is called twice per pixel
 pair, for greyscale bits 15..8 and 7..0.

 transpose8rS32 puts row i at output bit (7-i), so the lanes are packed in reverse
 row order. The eight results are then bytes of x (first) and y (second), MSB first.
--------------------------------------------------------------------------------------*/
__attribute__((optimize("O2"), always_inline)) static inline void dmd_transpose8(
	uint32_t w01, uint32_t w23, uint32_t w45, uint8_t sh, uint32_t* px, uint32_t* py)
{
	const uint8_t sh2 = sh + 16;
	uint32_t x = (((w45 >> sh2) & 0xFF) << 8) | ((w45 >> sh) & 0xFF);
	uint32_t y = (((w23 >> sh2) & 0xFF) << 24) | (((w23 >> sh) & 0xFF) << 16)
		| (((w01 >> sh2) & 0xFF) << 8) | ((w01 >> sh) & 0xFF);
	uint32_t t;
	t = (x ^ (x >> 7)) & 0x00AA00AA; x = x ^ t ^ (t << 7);
	t = (y ^ (y >> 7)) & 0x00AA00AA; y = y ^ t ^ (t << 7);
	t = (x ^ (x >> 14)) & 0x0000CCCC; x = x ^ t ^ (t << 14);
	t = (y ^ (y >> 14)) & 0x0000CCCC; y = y ^ t ^ (t << 14);
	t = (x & 0xF0F0F0F0) | ((y >> 4) & 0x0F0F0F0F);
	*py = ((x << 4) & 0xF0F0F0F0) | (y & 0x0F0F0F0F);
	*px = t;
}

// One 16-bit greyscale word for 6 lanes (upper and lower RGB pixel), MSB first.
// LAT is held high on the last bit if <latch>; OE pulses on the last 4 bits if <oe_pulse>.
__attribute__((optimize("O2"), always_inline)) inline void DMD_SPWM::send_pair(const uint16_t* p, bool latch, bool oe_pulse)
{
	volatile uint32_t* const ds = datasetreg;
#if !DMD_CLK_WITH_DATA
	const uint32_t ck = clkmask;
#endif
	const uint32_t clr = clk_clrmask;
	const uint32_t* const ex = expand;
	const uint16_t c0 = p[0], c1 = p[1];
	// two lanes per word: low half = lane n, high half = lane n+1
	const uint32_t w01 = lut5[c0 >> 11] | ((uint32_t)lut6[(c0 >> 5) & 63] << 16);
	const uint32_t w23 = lut5[c0 & 31] | ((uint32_t)lut5[c1 >> 11] << 16);
	const uint32_t w45 = lut6[(c1 >> 5) & 63] | ((uint32_t)lut5[c1 & 31] << 16);
	uint32_t x, y, x2, y2;

	// transpose both halves up front, so the lane words are dead during the emit
	dmd_transpose8(w01, w23, w45, 8, &x, &y);    // greyscale bits 15..8
	dmd_transpose8(w01, w23, w45, 0, &x2, &y2);  // greyscale bits 7..0

	PEW(ex[(x >> 24) & 0x3F]) PEW(ex[(x >> 16) & 0x3F]) PEW(ex[(x >> 8) & 0x3F]) PEW(ex[x & 0x3F])
	PEW(ex[(y >> 24) & 0x3F]) PEW(ex[(y >> 16) & 0x3F]) PEW(ex[(y >> 8) & 0x3F]) PEW(ex[y & 0x3F])

	PEW(ex[(x2 >> 24) & 0x3F]) PEW(ex[(x2 >> 16) & 0x3F]) PEW(ex[(x2 >> 8) & 0x3F]) PEW(ex[x2 & 0x3F])
	if (oe_pulse) *oesetreg = oemask;
	PEW(ex[(y2 >> 24) & 0x3F]) PEW(ex[(y2 >> 16) & 0x3F]) PEW(ex[(y2 >> 8) & 0x3F])
	if (latch) *latsetreg = latmask;
	PEW(ex[y2 & 0x3F])
	if (latch) *latsetreg = latmask << 16;
	if (oe_pulse) *oesetreg = oemask << 16;
}

// FM6353 / FM6363: plain greyscale upload (the panel keeps displaying meanwhile)
__attribute__((optimize("O2"))) void DMD_SPWM::data_transfer()
{
	const uint8_t num_sect = x_len / 16;
	for (uint8_t y = 0; y < nRows; y++) {
		const uint16_t* rowp = fb + (uint32_t)y * x_len * 2;
		for (uint8_t x = 0; x < 16; x++) {
			for (uint8_t sect = 0; sect < num_sect; sect++)
				send_pair(rowp + (sect * 16 + x) * 2, sect == num_sect - 1, false);
		}
	}
}

// DP3264 family: CLK is also the PWM clock, so the rows must keep switching during
// the upload: an OE pulse every 128 clocks and a row switch at 112 clocks.
__attribute__((optimize("O2"))) void DMD_SPWM::data_transfer_decoupled()
{
	const uint8_t num_sect = x_len / 16;
	oe_mode(false);
	// 12-clock OE pulse starts the data group
	*oesetreg = oemask;
	send_clocks(12);
	*oesetreg = oemask << 16;

	uint16_t clk_count = 16;
	for (uint8_t y = 0; y < nRows; y++) {
		const uint16_t* rowp = fb + (uint32_t)y * x_len * 2;
		for (uint8_t x = 0; x < 16; x++) {
			for (uint8_t sect = 0; sect < num_sect; sect++) {
				if (clk_count == 112) {
					mux->set(row);
					if (++row >= nRows) row = 0;
				}
				bool pulse = (clk_count == 128);
				send_pair(rowp + (sect * 16 + x) * 2, sect == num_sect - 1, pulse);
				if (pulse) clk_count = 0;
				clk_count += 16;
			}
		}
	}
}

/*--------------------------------------------------------------------------------------*/
// Upload config registers + framebuffer
/*--------------------------------------------------------------------------------------*/
void DMD_SPWM::update()
{
	if (!initialized) return;
	stop_GCLK();
	*datasetreg = clk_clrmask;
	delayMicroseconds(2);
	load_config_regs();
	start_GCLK();
	if (family == FAM_DECOUPLED) data_transfer_decoupled();
	else data_transfer();
	if (family != FAM_GCLK) start_DCLK();
}
