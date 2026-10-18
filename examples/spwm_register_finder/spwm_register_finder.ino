/*--------------------------------------------------------------------------------------
 STM_PWM - S-PWM Register Config Finder

 Cycles through register profiles from the catalog (src/register_profiles) and shows
 a test picture with the profile number "REG n".

  1. Select the chip below (FINDER_CHIP_xxx) and the matching driver class.
  2. Uncomment one or more scan headers of that chip in the profile list.
  3. Upload. Short press of the button = next profile, the number is shown on the panel
     and printed to Serial. Without a button (FINDER_BUTTON_PIN commented out) the
     profiles advance automatically every FINDER_AUTO_MS.
  4. When a profile looks good, set FINDER_OVERRIDE to its REG number: the sketch then
     applies only that profile at startup (use the same code in your own sketch).
--------------------------------------------------------------------------------------*/
#include <DMD_SPWM.h>
#include <DMD_SPWM_RegProfiles.h>
#include "fonts/SystemFont5x7.h"

// ---------- chip / profile selection ----------
#define FINDER_CHIP_FM6373          // FM6353 FM6363 FM6373 ICND1065L ICND2055 SM16380SH DP3264
#define FINDER_OVERRIDE   0         // 0 = interactive finder, N = apply REG N only
#define FINDER_RGB_LANES  true      // send separate R/G/B register words
#define FINDER_BUTTON_PIN PB10      // active low; must not be a panel pin
#define FINDER_AUTO_MS    4000

#if defined(FINDER_CHIP_FM6353)
#define DMD_SPWM_REGISTER_TEST_PROFILE_CHIP DMD_SPWM_REGISTER_TEST_PROFILE_CHIP_FM6353
typedef DMD_SPWM_FM6353_Profile Profile;
static const Profile profiles[] = {
//#include "register_profiles/fm6353/fm6353_scan_16.h"
#include "register_profiles/fm6353/fm6353_scan_32.h"
};
#define FINDER_DRIVER DMD_RGB_FM6353
#elif defined(FINDER_CHIP_FM6363)
#define DMD_SPWM_REGISTER_TEST_PROFILE_CHIP DMD_SPWM_REGISTER_TEST_PROFILE_CHIP_FM6363
typedef DMD_SPWM_FM6363_Profile Profile;
static const Profile profiles[] = {
#include "register_profiles/fm6363/fm6363_scan_32.h"
};
#define FINDER_DRIVER DMD_RGB_FM6363
#elif defined(FINDER_CHIP_FM6373)
#define DMD_SPWM_REGISTER_TEST_PROFILE_CHIP DMD_SPWM_REGISTER_TEST_PROFILE_CHIP_FM6373
typedef DMD_SPWM_FM6373_Profile Profile;
static const Profile profiles[] = {
//#include "register_profiles/fm6373/fm6373_scan_16.h"
#include "register_profiles/fm6373/fm6373_scan_32.h"
};
#define FINDER_DRIVER DMD_RGB_FM6373
#elif defined(FINDER_CHIP_ICND1065L)
#define DMD_SPWM_REGISTER_TEST_PROFILE_CHIP DMD_SPWM_REGISTER_TEST_PROFILE_CHIP_ICND1065L
typedef DMD_SPWM_ICND1065L_Profile Profile;
static const Profile profiles[] = {
#include "register_profiles/icnd1065l/icnd1065l_scan_32.h"
};
#define FINDER_DRIVER DMD_RGB_ICN1065
#elif defined(FINDER_CHIP_ICND2055)
#define DMD_SPWM_REGISTER_TEST_PROFILE_CHIP DMD_SPWM_REGISTER_TEST_PROFILE_CHIP_ICND2055
typedef DMD_SPWM_ICND2055_Profile Profile;
static const Profile profiles[] = {
#include "register_profiles/icnd2055/icnd2055_scan_32.h"
};
#define FINDER_DRIVER DMD_RGB_ICN2055
#elif defined(FINDER_CHIP_SM16380SH)
#define DMD_SPWM_REGISTER_TEST_PROFILE_CHIP DMD_SPWM_REGISTER_TEST_PROFILE_CHIP_SM16380SH
typedef DMD_SPWM_SM16380SH_Profile Profile;
static const Profile profiles[] = {
#include "register_profiles/sm16380sh/sm16380sh_scan_32.h"
};
#define FINDER_DRIVER DMD_RGB_SM16380SH
#elif defined(FINDER_CHIP_DP3264)
#define DMD_SPWM_REGISTER_TEST_PROFILE_CHIP DMD_SPWM_REGISTER_TEST_PROFILE_CHIP_DP3264
typedef DMD_SPWM_DP3264_Profile Profile;
static const Profile profiles[] = {
#include "register_profiles/dp3264/dp3264_scan_32.h"
};
#define FINDER_DRIVER DMD_RGB_DP3264
#endif

static const uint16_t profile_cnt = sizeof(profiles) / sizeof(profiles[0]);

// ---------- panel ----------
uint8_t mux_list[] = { PB6, PB5, PB4, PB3, PB8 };
#define DMD_PIN_nOE  PB0
#define DMD_PIN_SCLK PB7
#if defined(STM32F1xx)
uint8_t rgbpins[] = { PA15, PA0, PA1, PA2, PA3, PA4, PA5 };   // CLK, R0, G0, B0, R1, G1, B1
#else
uint8_t rgbpins[] = { PA6, PA7, PA1, PA2, PA3, PA4, PA5 };    // R0 on PA7 keeps PA0 (KEY) free
#endif
// STM32F103C8 (20 KB RAM): use a 64x32 panel type, a 128x64 framebuffer needs 16 KB
#define FINDER_PANEL RGB128x64plainS32
FINDER_DRIVER<FINDER_PANEL> dmd(mux_list, DMD_PIN_nOE, DMD_PIN_SCLK, rgbpins, 1, 1);

// ---------- test picture ----------
void drawTestPicture(uint16_t reg)
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
	dmd.drawRect(0, 0, w, h, dmd.Color888(255, 255, 255));        // edges / alignment
	char label[16];
	snprintf(label, sizeof(label), "REG %u", reg);
	dmd.selectFont(System5x7, 1);
	dmd.fillRect(1, 1, dmd.stringWidth(label) + 2, dmd.fontHeight() + 2, 0);
	dmd.drawString(2, 2, label, dmd.Color888(255, 255, 255));
	dmd.update();
}

void applyProfile(uint16_t i)
{
	dmdApplyRegProfile(dmd, profiles[i], FINDER_RGB_LANES);
	drawTestPicture(profiles[i].catalog_index);
	Serial.print("REG ");
	Serial.print(profiles[i].catalog_index);
	Serial.print("  (");
	Serial.print(i + 1);
	Serial.print('/');
	Serial.print(profile_cnt);
	Serial.println(')');
}

void setup()
{
	Serial.begin(115200);
	if (!dmd.init()) {
		Serial.println("dmd.init() failed");
		while (1) {}
	}
#if FINDER_OVERRIDE > 0
	const Profile* p = dmdFindRegProfile(profiles, FINDER_OVERRIDE);
	if (p) dmdApplyRegProfile(dmd, *p, FINDER_RGB_LANES);
	else Serial.println("REG not found - include its scan header");
	drawTestPicture(FINDER_OVERRIDE);
#else
#ifdef FINDER_BUTTON_PIN
	pinMode(FINDER_BUTTON_PIN, INPUT_PULLUP);
#endif
	applyProfile(0);
#endif
}

void loop()
{
#if FINDER_OVERRIDE == 0
	static uint16_t current = 0;
	static uint32_t last = millis();
	bool next = false;
#ifdef FINDER_BUTTON_PIN
	if (digitalRead(FINDER_BUTTON_PIN) == LOW) {
		delay(30);
		while (digitalRead(FINDER_BUTTON_PIN) == LOW) {}
		next = true;
	}
#else
	next = (millis() - last) > FINDER_AUTO_MS;
#endif
	if (next) {
		last = millis();
		current = (current + 1) % profile_cnt;
		applyProfile(current);
	}
#endif
}
