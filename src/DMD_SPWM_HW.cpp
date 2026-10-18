/*--------------------------------------------------------------------------------------
 DMD_SPWM_HW.cpp - STM32duino low level part: GPIO, timers, DMA, interrupt
 STM32F1 / STM32F4 only. Uses LL drivers / registers directly, no HardwareTimer.

 Timer scheme (same as DMD_STM32 for libmaple):
  * OE_TIM generates GCLK packets in PWM1 mode on the OE pin. Its counting is gated by
    MAIN_TIM OC1REF (master/slave gated mode), so every packet has an exact number
    of GCLK pulses.
  * MAIN_TIM CC2 compare interrupt switches the rows.
  * FM6363 / DP3264 family: CLK_TIM CC3 fires a circular DMA that writes CLK high/low
    words to the GPIO BSRR register -> continuous DCLK on any pin of the RGB port.
--------------------------------------------------------------------------------------*/
#include "DMD_SPWM.h"
#include "pinmap.h"
#include "PeripheralPins.h"
#include "stm32yyxx_ll_tim.h"
#include "stm32yyxx_ll_dma.h"
#include "stm32yyxx_ll_bus.h"

#if defined(DMD_ALTERNATIVE_TIMERS)
#define MAIN_TIM      TIM3
#define OE_TIM        TIM4
#define MAIN_IRQn     TIM3_IRQn
#define OE_TRIGGER    LL_TIM_TS_ITR2      // TIM4 ITR2 = TIM3 TRGO
#else
#define MAIN_TIM      TIM4
#define OE_TIM        TIM3
#define MAIN_IRQn     TIM4_IRQn
#define OE_TRIGGER    LL_TIM_TS_ITR3      // TIM3 ITR3 = TIM4 TRGO
#endif

#if defined(STM32F1xx)
#define CLK_TIM       TIM2                // TIM2_CH3 -> DMA1 channel 1
#define CLK_TIM_APB2  0
#else
#define CLK_TIM       TIM1                // TIM1_CH3 -> DMA2 stream 6 channel 6
#define CLK_TIM_APB2  1
#endif

static DMD_SPWM* g_dmd = nullptr;

/*--------------------------------------------------------------------------------------*/
// Interrupt: the core's HardwareTimer owns TIMx_IRQHandler, so the vector table is
// copied to RAM once and the MAIN timer vector is pointed to our own handler.
/*--------------------------------------------------------------------------------------*/
#define DMD_VECTOR_CNT 128
static uint32_t dmd_ram_vectors[DMD_VECTOR_CNT] __attribute__((aligned(512)));

static void dmd_main_irq(void)
{
	if (MAIN_TIM->SR & TIM_SR_CC2IF) {
		MAIN_TIM->SR = ~TIM_SR_CC2IF;
		if (g_dmd) g_dmd->isr();
	}
}

static void dmd_install_irq(IRQn_Type irq, void (*handler)(void))
{
	if (SCB->VTOR != (uint32_t)dmd_ram_vectors) {
		uint32_t primask = __get_PRIMASK();
		__disable_irq();
		memcpy(dmd_ram_vectors, (const void*)SCB->VTOR, sizeof(dmd_ram_vectors));
		SCB->VTOR = (uint32_t)dmd_ram_vectors;
		__DSB(); __ISB();
		if (!primask) __enable_irq();
	}
	dmd_ram_vectors[16 + (int)irq] = (uint32_t)handler;
	NVIC_SetPriority(irq, DMD_IRQ_PRIORITY);
	NVIC_ClearPendingIRQ(irq);
	NVIC_EnableIRQ(irq);
}

/*--------------------------------------------------------------------------------------*/
// helpers
/*--------------------------------------------------------------------------------------*/
static void tim_reset_enable(TIM_TypeDef* t)
{
	if (t == TIM1) { __HAL_RCC_TIM1_CLK_ENABLE(); __HAL_RCC_TIM1_FORCE_RESET(); __HAL_RCC_TIM1_RELEASE_RESET(); }
	else if (t == TIM2) { __HAL_RCC_TIM2_CLK_ENABLE(); __HAL_RCC_TIM2_FORCE_RESET(); __HAL_RCC_TIM2_RELEASE_RESET(); }
	else if (t == TIM3) { __HAL_RCC_TIM3_CLK_ENABLE(); __HAL_RCC_TIM3_FORCE_RESET(); __HAL_RCC_TIM3_RELEASE_RESET(); }
	else if (t == TIM4) { __HAL_RCC_TIM4_CLK_ENABLE(); __HAL_RCC_TIM4_FORCE_RESET(); __HAL_RCC_TIM4_RELEASE_RESET(); }
}

static uint32_t timer_clock(bool apb2)
{
	uint32_t f = apb2 ? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq();
	uint32_t pre = apb2 ? (RCC->CFGR & RCC_CFGR_PPRE2) : (RCC->CFGR & RCC_CFGR_PPRE1);
	bool divided = apb2 ? (pre != RCC_CFGR_PPRE2_DIV1) : (pre != RCC_CFGR_PPRE1_DIV1);
	return divided ? 2 * f : f;
}

static uint32_t ll_channel(uint8_t ch)
{
	switch (ch) {
	case 2: return LL_TIM_CHANNEL_CH2;
	case 3: return LL_TIM_CHANNEL_CH3;
	case 4: return LL_TIM_CHANNEL_CH4;
	default: return LL_TIM_CHANNEL_CH1;
	}
}

static void oe_set_compare(uint8_t ch, uint32_t v)
{
	switch (ch) {
	case 1: LL_TIM_OC_SetCompareCH1(OE_TIM, v); break;
	case 2: LL_TIM_OC_SetCompareCH2(OE_TIM, v); break;
	case 3: LL_TIM_OC_SetCompareCH3(OE_TIM, v); break;
	case 4: LL_TIM_OC_SetCompareCH4(OE_TIM, v); break;
	}
}

static GPIO_TypeDef* pin_port(uint32_t pin) { return get_GPIO_Port(STM_PORT(digitalPinToPinName(pin))); }
static uint32_t pin_num(uint32_t pin) { return STM_PIN(digitalPinToPinName(pin)); }

// fastest output slew rate (pin must already be an output)
static void pin_speed_max(uint32_t pin)
{
	GPIO_TypeDef* port = pin_port(pin);
	uint32_t n = pin_num(pin);
#if defined(STM32F1xx)
	volatile uint32_t* cr = (n < 8) ? &port->CRL : &port->CRH;
	*cr |= 3u << ((n & 7) * 4);               // MODE = 11 (50 MHz)
#else
	port->OSPEEDR |= 3u << (n * 2);            // very high speed
#endif
}

static void pin_fast_output(uint32_t pin, uint32_t level)
{
	digitalWrite(pin, level);
	pinMode(pin, OUTPUT);
	digitalWrite(pin, level);
	pin_speed_max(pin);
}

// switch OE pin between timer output (af=true) and plain GPIO output
void DMD_SPWM::oe_mode(bool af)
{
	*oe_cfg_reg = (*oe_cfg_reg & ~oe_cfg_mask) | (af ? oe_cfg_af : oe_cfg_out);
}

/*--------------------------------------------------------------------------------------*/
// GPIO + timers + DMA setup
/*--------------------------------------------------------------------------------------*/
bool DMD_SPWM::hw_init()
{
	// ---- RGB + CLK : common port ----
	GPIO_TypeDef* data_port = pin_port(pin_clk);
	for (uint8_t i = 0; i < 6; i++) if (pin_port(rgb_pins[i]) != data_port) return false;
	for (uint8_t i = 0; i < 6; i++) pin_fast_output(rgb_pins[i], LOW);
	pin_fast_output(pin_clk, LOW);
	pin_fast_output(pin_lat, LOW);

	datasetreg = &data_port->BSRR;
	latsetreg = &pin_port(pin_lat)->BSRR;
	clkmask = digitalPinToBitMask(pin_clk);
	latmask = digitalPinToBitMask(pin_lat);
	rgbmask_all = 0;
	for (uint8_t i = 0; i < 6; i++) rgbmask_all |= digitalPinToBitMask(rgb_pins[i]);
	for (uint8_t i = 0; i < 3; i++)
		lane_mask[i] = digitalPinToBitMask(rgb_pins[i]) | digitalPinToBitMask(rgb_pins[i + 3]);
	clk_clrmask = (rgbmask_all | clkmask) << 16;
	dclk_strobe[0] = clkmask;
	dclk_strobe[1] = clkmask << 16;
	build_expand();

	// ---- OE pin : channel of OE_TIM ----
	PinName oe_pn = digitalPinToPinName(pin_oe);
	PinName tim_pn = NC;
	oe_channel = 0;
	for (const PinMap* m = PinMap_TIM; m->pin != NC; m++) {
		if ((PinName)(m->pin & PNUM_MASK) == (PinName)(oe_pn & PNUM_MASK) &&
			m->peripheral == OE_TIM && !STM_PIN_INVERTED(m->function)) {
			tim_pn = m->pin;
			oe_channel = STM_PIN_CHANNEL(m->function);
			break;
		}
	}
	if (oe_channel == 0) return false;

	pin_fast_output(pin_oe, HIGH);
	oesetreg = &pin_port(pin_oe)->BSRR;
	oemask = digitalPinToBitMask(pin_oe);
	pinmap_pinout(tim_pn, PinMap_TIM);         // AF (and AFIO remap on F1)
	{
		GPIO_TypeDef* port = pin_port(pin_oe);
		uint32_t n = pin_num(pin_oe);
#if defined(STM32F1xx)
		oe_cfg_reg = (n < 8) ? &port->CRL : &port->CRH;
		uint32_t sh = (n & 7) * 4;
		oe_cfg_mask = 0xFu << sh;
		oe_cfg_af = 0xBu << sh;    // CNF=10 AF push-pull, MODE=11
		oe_cfg_out = 0x3u << sh;   // CNF=00 GP push-pull, MODE=11
#else
		port->OSPEEDR |= 3u << (n * 2);
		oe_cfg_reg = &port->MODER;
		oe_cfg_mask = 3u << (n * 2);
		oe_cfg_af = 2u << (n * 2);
		oe_cfg_out = 1u << (n * 2);
#endif
		oe_mode(true);
	}

	// ---- MAIN timer (master) ----
	tim_reset_enable(MAIN_TIM);
	LL_TIM_DisableCounter(MAIN_TIM);
	LL_TIM_SetPrescaler(MAIN_TIM, 0);
	LL_TIM_SetAutoReload(MAIN_TIM, main_reload);
	LL_TIM_EnableMasterSlaveMode(MAIN_TIM);
	LL_TIM_SetTriggerOutput(MAIN_TIM, LL_TIM_TRGO_OC1REF);
	LL_TIM_OC_SetMode(MAIN_TIM, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);   // CH1 gates OE timer
	LL_TIM_OC_SetCompareCH1(MAIN_TIM, main_cc1);
	LL_TIM_OC_SetMode(MAIN_TIM, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_FROZEN); // CH2 row switch IRQ
	LL_TIM_OC_SetCompareCH2(MAIN_TIM, main_cc2);
	LL_TIM_ClearFlag_CC2(MAIN_TIM);
	LL_TIM_EnableIT_CC2(MAIN_TIM);
	g_dmd = this;
	dmd_install_irq(MAIN_IRQn, dmd_main_irq);

	// ---- OE timer (slave, gated) ----
	tim_reset_enable(OE_TIM);
	LL_TIM_DisableCounter(OE_TIM);
	LL_TIM_SetPrescaler(OE_TIM, 0);
	LL_TIM_SetTriggerInput(OE_TIM, OE_TRIGGER);
	LL_TIM_SetSlaveMode(OE_TIM, LL_TIM_SLAVEMODE_GATED);
	uint32_t ch = ll_channel(oe_channel);
	LL_TIM_OC_SetMode(OE_TIM, ch, LL_TIM_OCMODE_PWM1);
	LL_TIM_SetAutoReload(OE_TIM, oe_reload);
	LL_TIM_OC_SetPolarity(OE_TIM, ch, LL_TIM_OCPOLARITY_LOW);   // GCLK active low
	oe_set_compare(oe_channel, oe_cc);
	LL_TIM_CC_EnableChannel(OE_TIM, ch);

	// ---- DCLK timer + DMA ----
	if (family != FAM_GCLK) {
		// keep the DCLK/MAIN tick ratio independent of the bus clocks
		uint32_t f_main = timer_clock(false);
		uint32_t f_clk = timer_clock(CLK_TIM_APB2);
		uint32_t period = (clk_period * f_clk) / f_main;
		if (period < 2) period = 2;

		tim_reset_enable(CLK_TIM);
		LL_TIM_DisableCounter(CLK_TIM);
		LL_TIM_SetPrescaler(CLK_TIM, 0);
		LL_TIM_SetAutoReload(CLK_TIM, period - 1);
		LL_TIM_OC_SetCompareCH3(CLK_TIM, period / 2);
		LL_TIM_EnableDMAReq_CC3(CLK_TIM);

#if defined(STM32F1xx)
		__HAL_RCC_DMA1_CLK_ENABLE();
		LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
		LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_1,
			LL_DMA_DIRECTION_MEMORY_TO_PERIPH | LL_DMA_PRIORITY_VERYHIGH | LL_DMA_MODE_CIRCULAR |
			LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT |
			LL_DMA_PDATAALIGN_WORD | LL_DMA_MDATAALIGN_WORD);
		LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)datasetreg);
		LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)dclk_strobe);
#else
		__HAL_RCC_DMA2_CLK_ENABLE();
		LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_6);
		while (LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_6)) {}
		LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_6, LL_DMA_CHANNEL_6);
		LL_DMA_ConfigTransfer(DMA2, LL_DMA_STREAM_6,
			LL_DMA_DIRECTION_MEMORY_TO_PERIPH | LL_DMA_PRIORITY_VERYHIGH | LL_DMA_MODE_CIRCULAR |
			LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT |
			LL_DMA_PDATAALIGN_WORD | LL_DMA_MDATAALIGN_WORD);
		LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_6);
		LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_6, (uint32_t)datasetreg);
		LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_6, (uint32_t)dclk_strobe);
#endif
		dclk_dma(false);
	}
#if defined(STM32F1xx)
	// free PA15/PB3/PB4 (JTAG) for panel use, keep SWD. Must follow pinmap_pinout(),
	// because AFIO remaps rewrite the (write-only) SWJ_CFG bits.
	__HAL_RCC_AFIO_CLK_ENABLE();
	__HAL_AFIO_REMAP_SWJ_NOJTAG();
#endif
	return true;
}

/*--------------------------------------------------------------------------------------*/
// DCLK DMA on/off. Re-arms the transfer counter each time it is enabled.
/*--------------------------------------------------------------------------------------*/
void DMD_SPWM::dclk_dma(bool on)
{
#if defined(STM32F1xx)
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
	if (on) {
		LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 2);
		LL_DMA_ClearFlag_GI1(DMA1);
		LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
	}
#else
	LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_6);
	while (LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_6)) {}
	if (on) {
		LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_6, 2);
		LL_DMA_ClearFlag_TC6(DMA2); LL_DMA_ClearFlag_HT6(DMA2); LL_DMA_ClearFlag_TE6(DMA2);
		LL_DMA_ClearFlag_DME6(DMA2); LL_DMA_ClearFlag_FE6(DMA2);
		LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_6);
	}
#endif
}

/*--------------------------------------------------------------------------------------*/
// Row switching interrupt (MAIN_TIM CC2)
// When a new frame is waiting (oe_scan_flag == false) GCLK is stopped at row 0.
/*--------------------------------------------------------------------------------------*/
void DMD_SPWM::isr()
{
	mux->set(row);
	OE_TIM->CNT = 0;              // re-sync OE timer with MAIN timer
	if (row == 0 && !oe_scan_flag) {
		MAIN_TIM->CCR1 = 0;       // OC1REF stays low -> GCLK generator off
		oe_scan_res = false;
		return;
	}
	if (++row >= nRows) row = 0;
}

/*--------------------------------------------------------------------------------------*/
void DMD_SPWM::stop_GCLK()
{
	if (oe_scan_flag) {
		noInterrupts();
		oe_scan_flag = false;
		oe_scan_res = true;
		interrupts();
		uint32_t t0 = millis();
		while (oe_scan_res && (millis() - t0) < 50) {}
		if (family == FAM_GCLK) delayMicroseconds(5);
	}
	LL_TIM_DisableCounter(MAIN_TIM);
	if (family != FAM_GCLK) {
		LL_TIM_DisableCounter(CLK_TIM);
		dclk_dma(false);
		*datasetreg = clk_clrmask;
	}
	// switch to row 0
	row = 0;
	mux->set(row);
	row++;
	delayMicroseconds(10);
	oe_scan_flag = true;
}

/*--------------------------------------------------------------------------------------*/
void DMD_SPWM::start_GCLK()
{
	LL_TIM_DisableCounter(OE_TIM);
	if (family == FAM_DECOUPLED) {
		LL_TIM_GenerateEvent_UPDATE(MAIN_TIM);
		LL_TIM_GenerateEvent_UPDATE(OE_TIM);
		// counter just after CC1: first loop starts right after the 12-CLK OE pulse
		LL_TIM_SetCounter(MAIN_TIM, main_cc1 + 2);
		LL_TIM_OC_SetCompareCH1(MAIN_TIM, main_cc1);
		return;   // timers are started in start_DCLK() after data transfer
	}
	LL_TIM_OC_SetCompareCH1(MAIN_TIM, main_cc1);
	LL_TIM_SetCounter(MAIN_TIM, 0);
	LL_TIM_SetCounter(OE_TIM, 0);
	LL_TIM_EnableCounter(OE_TIM);
	LL_TIM_EnableCounter(MAIN_TIM);
}

/*--------------------------------------------------------------------------------------*/
void DMD_SPWM::start_DCLK()
{
	dclk_dma(true);
	LL_TIM_SetCounter(CLK_TIM, 0);
	if (family == FAM_DECOUPLED) {
		oe_mode(true);
		LL_TIM_EnableCounter(OE_TIM);
		LL_TIM_EnableCounter(MAIN_TIM);
	}
	LL_TIM_EnableCounter(CLK_TIM);
}
