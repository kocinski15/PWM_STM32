/*--------------------------------------------------------------------------------------
 DMD_SPWM_Clock.cpp - optional system clock from an external 8 MHz crystal (HSE)

 Enabled only when DMD_HSE_CRYSTAL_8MHZ is defined (PlatformIO build_flags, or
 Arduino IDE build_opt.h). It replaces the core's WEAK SystemClock_Config():
   STM32F411xx : HSE 8 MHz -> PLL -> 100 MHz (APB1 50, APB2 100, timers 100 MHz)
   STM32F401xx : HSE 8 MHz -> PLL ->  84 MHz (APB1 42, APB2 84, timers 84 MHz, USB 48)
   STM32F405/407/415/417 : HSE 8 MHz -> 168 MHz (APB1 42, APB2 84, USB 48)
   STM32F103xx : HSE 8 MHz -> PLL x9 -> 72 MHz (APB1 36, APB2 72, USB 48)
 Uses the crystal oscillator (RCC_HSE_ON), not the bypass mode of Nucleo boards
 (which take an 8 MHz clock from the ST-Link MCO).
 If the crystal does not start, Error_Handler() is called (MCU halts).

 PlatformIO: also set "lib_archive = no", otherwise the linker can keep the core's
 weak version instead of this one.
--------------------------------------------------------------------------------------*/
#if defined(DMD_HSE_CRYSTAL_8MHZ)
#include <Arduino.h>

#if defined(HSE_VALUE) && (HSE_VALUE != 8000000U)
#error "DMD_HSE_CRYSTAL_8MHZ: HSE_VALUE must be 8000000"
#endif

extern "C" void SystemClock_Config(void)
{
	RCC_OscInitTypeDef osc = {};
	RCC_ClkInitTypeDef clk = {};
	uint32_t latency;

	osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	osc.HSEState = RCC_HSE_ON;                 // crystal on OSC_IN / OSC_OUT
	osc.PLL.PLLState = RCC_PLL_ON;
	osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	clk.AHBCLKDivider = RCC_SYSCLK_DIV1;

#if defined(STM32F411xE) || defined(STM32F411xC)
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	osc.PLL.PLLM = 4;                          // 8 / 4 = 2 MHz
	osc.PLL.PLLN = 100;                        // 200 MHz VCO
	osc.PLL.PLLP = RCC_PLLP_DIV2;              // 100 MHz
	osc.PLL.PLLQ = 4;                          // 50 MHz (USB not exact at 100 MHz)
	clk.APB1CLKDivider = RCC_HCLK_DIV2;
	clk.APB2CLKDivider = RCC_HCLK_DIV1;
	latency = FLASH_LATENCY_3;
#elif defined(STM32F401xE) || defined(STM32F401xC)
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
	osc.PLL.PLLM = 4;                          // 2 MHz
	osc.PLL.PLLN = 168;                        // 336 MHz VCO
	osc.PLL.PLLP = RCC_PLLP_DIV4;              // 84 MHz
	osc.PLL.PLLQ = 7;                          // 48 MHz USB
	clk.APB1CLKDivider = RCC_HCLK_DIV2;
	clk.APB2CLKDivider = RCC_HCLK_DIV1;
	latency = FLASH_LATENCY_2;
#elif defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F415xx) || defined(STM32F417xx)
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	osc.PLL.PLLM = 8;                          // 1 MHz
	osc.PLL.PLLN = 336;                        // 336 MHz VCO
	osc.PLL.PLLP = RCC_PLLP_DIV2;              // 168 MHz
	osc.PLL.PLLQ = 7;                          // 48 MHz USB
	clk.APB1CLKDivider = RCC_HCLK_DIV4;
	clk.APB2CLKDivider = RCC_HCLK_DIV2;
	latency = FLASH_LATENCY_5;
#elif defined(STM32F1xx)
	osc.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	osc.PLL.PLLMUL = RCC_PLL_MUL9;             // 72 MHz
	clk.APB1CLKDivider = RCC_HCLK_DIV2;
	clk.APB2CLKDivider = RCC_HCLK_DIV1;
	latency = FLASH_LATENCY_2;
#else
#error "DMD_HSE_CRYSTAL_8MHZ: no clock setup for this MCU, remove the define and use the variant clock"
#endif

	if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();
	if (HAL_RCC_ClockConfig(&clk, latency) != HAL_OK) Error_Handler();

#if defined(STM32F1xx)
	RCC_PeriphCLKInitTypeDef pclk = {};
	pclk.PeriphClockSelection = RCC_PERIPHCLK_USB | RCC_PERIPHCLK_ADC;
	pclk.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
	pclk.AdcClockSelection = RCC_ADCPCLK2_DIV6;
	HAL_RCCEx_PeriphCLKConfig(&pclk);
#endif
}
#endif // DMD_HSE_CRYSTAL_8MHZ
