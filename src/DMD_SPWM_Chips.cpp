/*--------------------------------------------------------------------------------------
 DMD_SPWM_Chips.cpp - per-chip timing values and default config registers
 Values taken from DMD_STM32 v1.2.18 (board707). Timer values are in MAIN/OE timer
 ticks (STM32F1: 72 MHz, STM32F401: 84 MHz, STM32F411: 100 MHz).

  MAIN_TIM period = row period. CH1 (cc1) gates GCLK packets, CH2 (cc2) switches rows.
  OE_TIM period   = GCLK period (OE pin).
--------------------------------------------------------------------------------------*/
#include "DMD_SPWM.h"

/*--------------------------------------------------------------------------------------*/
// GCLK-only family: GCLK_NUM pulses per row + ADD_NUM*8 ticks for row switching
/*--------------------------------------------------------------------------------------*/
static void gclk_timing(uint32_t gclk_num, uint32_t period, uint32_t add_num,
	uint32_t& reload, uint32_t& cc1, uint32_t& cc2, uint32_t& oe_reload, uint32_t& oe_cc)
{
	reload = gclk_num * period + 8 * add_num - 1;
	cc1 = gclk_num * period;
	cc2 = gclk_num * period + 2;
	oe_reload = period - 1;
	oe_cc = period / 2;
}

// Decoupled family: one 4-CLK OE pulse per row, row period = ADD_NUM CLK periods
static void decoupled_timing(uint32_t period, uint32_t add_num, uint32_t cc2_back,
	uint32_t& reload, uint32_t& cc1, uint32_t& cc2, uint32_t& oe_reload, uint32_t& oe_cc)
{
	reload = period + period * add_num + 4;
	cc1 = 12 * period;                       // stop OE timer after one 4x CLK pulse
	cc2 = period + period * (add_num - cc2_back);
	oe_reload = 12 * period - 1;
	oe_cc = 4 * period;
}

#if defined(STM32F1xx)
#define F1_F4(a, b) (a)
#else
#define F1_F4(a, b) (b)
#endif

/*--------------------------------------------------------------------------------------*/
// FM6353 (ICN2153 compatible)
/*--------------------------------------------------------------------------------------*/
void DMD_SPWM_FM6353::setup_chip()
{
	family = FAM_GCLK;
	cfg_mode = CFG_ALL;
	gclk_bits = 14;
	gclk_timing(138, 8, F1_F4(40, 22), main_reload, main_cc1, main_cc2, oe_reload, oe_cc);
	uint16_t regs[] = { 0x0008, 0x1f70, 0x6707, 0x40f7, 0x0040 };
	regs[1] = ((nRows - 1) << 8) | (regs[1] & 0xFF);
	set_builtin_regs(regs, sizeof(regs) / 2);
}

/*--------------------------------------------------------------------------------------*/
// FM6363 - registers are stored in catalog order and transmitted last word first
/*--------------------------------------------------------------------------------------*/
void DMD_SPWM_FM6363::setup_chip()
{
	family = FAM_DCLK;
	cfg_mode = CFG_ALL;
	gclk_bits = 13;
	clk_period = 8;
	gclk_timing(74, 8, 32, main_reload, main_cc1, main_cc2, oe_reload, oe_cc);
	// wire order was { 0x7e08, 0x0fb0, 0xe6fc, 0x60b6, 0x5a70 }
	uint16_t regs[] = { 0x0fb0, 0xe6fc, 0x60b6, 0x5a70, 0x7e08 };
	regs[0] = ((nRows - 1) << 8) | (regs[0] & 0xFF);
	set_builtin_regs(regs, sizeof(regs) / 2);
}

/*--------------------------------------------------------------------------------------*/
// DP3264
/*--------------------------------------------------------------------------------------*/
void DMD_SPWM_DP3264::setup_chip()
{
	family = FAM_DECOUPLED;
	cfg_mode = CFG_ONE_PLAIN;
	gclk_bits = 13;
	clk_period = F1_F4(12, 8);
	decoupled_timing(clk_period, F1_F4(235, 232), 40, main_reload, main_cc1, main_cc2, oe_reload, oe_cc);
	uint16_t regs[] = { 0x020f, 0x033f, 0x043f, 0x0504, 0x0642, 0x0700, 0x08BF,
		0x0960, 0x0ABE, 0x0B8B, 0x0C88, 0x0D12, 0x1100 };
	regs[0] = 0x0200 | (nRows - 1);
	set_builtin_regs(regs, sizeof(regs) / 2);
}

/*--------------------------------------------------------------------------------------*/
// ICN2055
/*--------------------------------------------------------------------------------------*/
void DMD_SPWM_ICN2055::setup_chip()
{
	family = FAM_DECOUPLED;
	cfg_mode = CFG_ONE_FRAMED;
	gclk_bits = 13;
	clk_period = F1_F4(12, 8);
	decoupled_timing(clk_period, 228, 35, main_reload, main_cc1, main_cc2, oe_reload, oe_cc);
	uint16_t regs[] = {
		0x021f, 0x033f, 0x0400, 0x0507, 0x0603, 0x0720, 0x0820, 0x0908, 0x0a08, 0x0b00,
		0x0c08, 0x0d01, 0x0e04, 0x0f01, 0x1082, 0x1121, 0x1201, 0x17f0, 0x181f, 0x1950,
		0x1a1f, 0x1b10, 0x1ccf, 0x1d0a, 0x1e4c, 0x1f20, 0x2008, 0x2101, 0x221c };
	regs[0] = 0x0200 | (nRows - 1);
	set_builtin_regs(regs, sizeof(regs) / 2);
}

/*--------------------------------------------------------------------------------------*/
// FM6373
/*--------------------------------------------------------------------------------------*/
void DMD_SPWM_FM6373::setup_chip()
{
	family = FAM_DECOUPLED;
	cfg_mode = CFG_ONE_FRAMED;
	gclk_bits = 14;
	clk_period = F1_F4(12, 8);
	decoupled_timing(clk_period, 228, 35, main_reload, main_cc1, main_cc2, oe_reload, oe_cc);
	uint16_t regs[] = {
		0x021f, 0x033f, 0x0402, 0x0507, 0x0603, 0x0720, 0x0820, 0x0900, 0x0a00, 0x0b00,
		0x0c01, 0x0d01, 0x0e04, 0x0f01, 0x10c2, 0x1121, 0x1201, 0x17f0, 0x181f, 0x1900,
		0x1a1f, 0x1b10, 0x1cc1, 0x1d0a, 0x1e42, 0x1f04, 0x2008, 0x2101, 0x221c };
	regs[0] = 0x0200 | (nRows - 1);
	set_builtin_regs(regs, sizeof(regs) / 2);
}

/*--------------------------------------------------------------------------------------*/
// SM16380SH
/*--------------------------------------------------------------------------------------*/
void DMD_SPWM_SM16380SH::setup_chip()
{
	family = FAM_DECOUPLED;
	cfg_mode = CFG_ONE_FRAMED;
	gclk_bits = 13;
	clk_period = F1_F4(12, 8);
	decoupled_timing(clk_period, 232, 35, main_reload, main_cc1, main_cc2, oe_reload, oe_cc);
	uint16_t regs[] = {
		0x021f, 0x0300, 0x0400, 0x0500, 0x0600, 0x0750, 0x0800, 0x0900, 0x0a02, 0x0b0c,
		0x0c08, 0x0d00, 0x0e05, 0x0f00, 0x1000, 0x1100, 0x1200, 0x1300, 0x1414, 0x1500,
		0x1630, 0x1700, 0x1801, 0x1904, 0x1a03, 0x1b14, 0x1c12, 0x1d00, 0x1e00, 0x1f0c };
	regs[0] = 0x0200 | (nRows - 1);
	set_builtin_regs(regs, sizeof(regs) / 2);
}

/*--------------------------------------------------------------------------------------*/
// ICND1065
/*--------------------------------------------------------------------------------------*/
void DMD_SPWM_ICN1065::setup_chip()
{
	family = FAM_DECOUPLED;
	cfg_mode = CFG_ONE_FRAMED;
	gclk_bits = 12;
	clk_period = F1_F4(12, 8);
	decoupled_timing(clk_period, 228, 35, main_reload, main_cc1, main_cc2, oe_reload, oe_cc);
	uint16_t regs[] = {
		0x00aa, 0x01aa, 0x022a, 0x0335, 0x0412, 0x0500, 0x0601, 0x0720, 0x0c18, 0x0d01, 0x0e86, 0x0f01,
		0x1040, 0x1127, 0x1200, 0x1300, 0x1400, 0x1500, 0x1600, 0x1800, 0x1906, 0x1c60, 0x1dca, 0x1e73,
		0x1f00, 0x2000, 0x2100, 0x2200, 0x2300, 0x2400, 0x2500, 0x2600, 0x2700, 0x7000, 0x7100, 0x7200,
		0x7300, 0x74A0 };
	regs[2] = 0x0200 | (nRows - 1);   // scan register is at index 2
	set_builtin_regs(regs, sizeof(regs) / 2);
}
