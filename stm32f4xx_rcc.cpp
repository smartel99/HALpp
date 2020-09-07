/**
 * @file        stm32f405zgtx.h
 * @author      Samuel Martel
 * @p           https://www.github.com/smartel99
 * @date        2020/09/06
 * @brief       This file provides firmware functions to manage the following
 *             functionalities of the Reset and Clock Control (RCC) peripheral:
 *           + Initialization and de-initialization functions
 *           + Peripheral Control functions
 *              
 ******************************************************************************
==============================================================================
                     ##### RCC specific features #####
 ==============================================================================
   [..]
     After reset the device is running from Internal High Speed oscillator
     (HSI 16MHz) with Flash 0 wait state, Flash prefetch buffer, D-Cache
     and I-Cache are disabled, and all peripherals are off except internal
     SRAM, Flash and JTAG.
     (+) There is no prescaler on High speed (AHB) and Low speed (APB) busses;
         all peripherals mapped on these busses are running at HSI speed.
     (+) The clock for all peripherals is switched off, except the SRAM and FLASH.
     (+) All GPIOs are in input floating state, except the JTAG pins which
         are assigned to be used for debug purpose.
   [..]
     Once the device started from reset, the user application has to:
     (+) Configure the clock source to be used to drive the System clock
         (if the application needs higher frequency/performance)
     (+) Configure the System clock frequency and Flash settings
     (+) Configure the AHB and APB busses prescalers
     (+) Enable the clock for the peripheral(s) to be used
     (+) Configure the clock source(s) for peripherals which clocks are not
         derived from the System clock (I2S, RTC, ADC, USB OTG FS/SDIO/RNG)
                     ##### RCC Limitations #####
 ==============================================================================
   [..]
     A delay between an RCC peripheral clock enable and the effective peripheral
     enabling should be taken into account in order to manage the peripheral read/write
     from/to registers.
     (+) This delay depends on the peripheral mapping.
     (+) If peripheral is mapped on AHB: the delay is 2 AHB clock cycle
         after the clock enable bit is set on the hardware register
     (+) If peripheral is mapped on APB: the delay is 2 APB clock cycle
         after the clock enable bit is set on the hardware register
   [..]
     Implemented Workaround:
     (+) For AHB & APB peripherals, a dummy read to the peripheral register has been
         inserted in each __HAL_RCC_PPP_CLK_ENABLE() macro.
 @endverbatim
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/* Includes -----------------------------------------------------------------*/
#include "stm32f4xx_core.h"

/**
 * @addtogroup STM32F4xx_HAL_Driver
 * @{
 */
namespace HAL
{
    /** @defgroup RCC RCC
     * @brief RCC HAL module Driver
     * @{
     */
    namespace RstClkCtrl
    {
#if defined(HAL_RCC_MODULE_ENABLED)

        /* Private typedef ----------------------------------------------------------*/
        /* Private defines ----------------------------------------------------------*/
        /** @addtogroup RCC_Private_Constants
         * @{
         */
        /* Private macro ------------------------------------------------------------*/
#define __MCO1_CLK_ENABLE() GpioAClkEnable()
        constexpr auto MCO1_GPIO_PORT = GPIOA;
        constexpr auto MCO1_PIN = GPIO_PIN_8;

#define __MCO2_CLK_ENABLE() GpioCClkEnable()
        constexpr auto MCO2_GPIO_PORT = GPIOC;
        constexpr auto MCO2_PIN = GPIO_PIN_9;
        /** @}
         * RCC_Private_Constants
         */
        /* Private variables ---------------------------------------------------------*/
        /** @defgroup RCC_Private_Variables RCC Private Variables
         * @{
         */
        /**
         * @}
         */
        /* Private function prototypes -----------------------------------------------*/
        /* Private functions ---------------------------------------------------------*/

        /** @defgroup RCC_Exported_Functions RCC Exported Functions
         *  @{
         */

        /** @defgroup RCC_Exported_Functions_Group1 Initialization and de-initialization functions
         *  @brief    Initialization and Configuration functions
         *
        @verbatim
        ===============================================================================
                ##### Initialization and de-initialization functions #####
        ===============================================================================
            [..]
            This section provides functions allowing to configure the internal/external oscillators
            (HSE, HSI, LSE, LSI, PLL, CSS and MCO) and the System busses clocks (SYSCLK, AHB, APB1
            and APB2).

            [..] Internal/external clock and PLL configuration
                (#) HSI (high-speed internal), 16 MHz factory-trimmed RC used directly or through
                    the PLL as System clock source.

                (#) LSI (low-speed internal), 32 KHz low consumption RC used as IWDG and/or RTC
                    clock source.

                (#) HSE (high-speed external), 4 to 26 MHz crystal oscillator used directly or
                    through the PLL as System clock source. Can be used also as RTC clock source.

                (#) LSE (low-speed external), 32 KHz oscillator used as RTC clock source.

                (#) PLL (clocked by HSI or HSE), featuring two different output clocks:
                (++) The first output is used to generate the high speed system clock (up to 168 MHz)
                (++) The second output is used to generate the clock for the USB OTG FS (48 MHz),
                        the random analog generator (<=48 MHz) and the SDIO (<= 48 MHz).

                (#) CSS (Clock security system), once enable using the macro __HAL_RCC_CSS_ENABLE()
                    and if a HSE clock failure occurs(HSE used directly or through PLL as System
                    clock source), the System clocks automatically switched to HSI and an interrupt
                    is generated if enabled. The interrupt is linked to the Cortex-M4 NMI
                    (Non-Maskable Interrupt) exception vector.

                (#) MCO1 (microcontroller clock output), used to output HSI, LSE, HSE or PLL
                    clock (through a configurable prescaler) on PA8 pin.

                (#) MCO2 (microcontroller clock output), used to output HSE, PLL, SYSCLK or PLLI2S
                    clock (through a configurable prescaler) on PC9 pin.

            [..] System, AHB and APB busses clocks configuration
                (#) Several clock sources can be used to drive the System clock (SYSCLK): HSI,
                    HSE and PLL.
                    The AHB clock (HCLK) is derived from System clock through configurable
                    prescaler and used to clock the CPU, memory and peripherals mapped
                    on AHB bus (DMA, GPIO...). APB1 (PCLK1) and APB2 (PCLK2) clocks are derived
                    from AHB clock through configurable prescalers and used to clock
                    the peripherals mapped on these busses. You can use
                    "HAL_RCC_GetSysClockFreq()" function to retrieve the frequencies of these clocks.

                (#) For the STM32F405xx/07xx and STM32F415xx/17xx devices, the maximum
                    frequency of the SYSCLK and HCLK is 168 MHz, PCLK2 84 MHz and PCLK1 42 MHz.
                    Depending on the device voltage range, the maximum frequency should
                    be adapted accordingly (refer to the product datasheets for more details).

                (#) For the STM32F42xxx, STM32F43xxx, STM32F446xx, STM32F469xx and STM32F479xx devices,
                    the maximum frequency of the SYSCLK and HCLK is 180 MHz, PCLK2 90 MHz and PCLK1 45 MHz.
                    Depending on the device voltage range, the maximum frequency should
                    be adapted accordingly (refer to the product datasheets for more details).

                (#) For the STM32F401xx, the maximum frequency of the SYSCLK and HCLK is 84 MHz,
                    PCLK2 84 MHz and PCLK1 42 MHz.
                    Depending on the device voltage range, the maximum frequency should
                    be adapted accordingly (refer to the product datasheets for more details).

                (#) For the STM32F41xxx, the maximum frequency of the SYSCLK and HCLK is 100 MHz,
                    PCLK2 100 MHz and PCLK1 50 MHz.
                    Depending on the device voltage range, the maximum frequency should
                    be adapted accordingly (refer to the product datasheets for more details).

        @endverbatim
        * @{
        */

        /**
         * @brief  Resets the RCC clock configuration to the default reset state.
         * @note   The default reset state of the clock configuration is given below:
         *            - HSI ON and used as system clock source
         *            - HSE and PLL OFF
         *            - AHB, APB1 and APB2 prescaler set to 1.
         *            - CSS, MCO1 and MCO2 OFF
         *            - All interrupts disabled
         * @note   This function doesn't modify the configuration of the
         *            - Peripheral clocks
         *            - LSI, LSE and RTC clocks
         * @retval HAL status
         */
        Status RstClkCtrl::DeInit()
        {
            // Get the Start Tick.
            uint32_t tickStart = HAL::STM::Get()->GetTick();

            // Set HSION bit to the reset value.
            SET_BIT(RCC->CR, RCC_CR_HSION);

            // Wait until HSI is ready.
            while (READ_BIT(RCC->CR, RCC_CR_HSIRDY) == 0)
            {
                if ((HAL::STM::Get()->GetTick() - tickStart) > HSI_TIMEOUT_VALUE)
                {
                    return Status::Timeout;
                }
            }

            // Set HSITRIM[4:0] bits to the reset value.
            SET_BIT(RCC->CR, RCC_CR_HSITRIM_4);

            // Get Start Tick.
            tickStart = HAL::STM::Get()->GetTick();

            // Reset CFGR register.
            CLEAR_REG(RCC->CFGR);

            // Wait until clock switch is ready.
            while (READ_BIT(RCC->CFGR, RCC_CFGR_SWS) != 0)
            {
                if ((HAL::STM::Get()->GetTick() - tickStart) > CLOWCKSWITCH_TIMEOUT_VALUE)
                {
                    return Status::Timeout;
                }
            }

            // Get start tick.
            tickStart = HAL::STM::Get()->GetTick();

            // Clear HSEON, HSEBYP and CSSON bits.
            CLEAR_BIT(RCC->CR, RCC_CR_HSEON | RCC_CR_HSEBYP | RCC_CR_CSSON);

            // Wait till HSE is disabled.
            while (READ_BIT(RCC->CR, RCC_CR_HSERDY) != 0)
            {
                if ((HAL::STM::Get()->GetTick() - tickStart) > HSE_TIMEOUT_VALUE)
                {
                    return Status::Timeout;
                }
            }

            // Get Start Tick.
            tickStart = HAL::STM::Get()->GetTick();

            // Clear PLLON bit.
            CLEAR_BIT(RCC->CR, RCC_CR_PLLON);

            // Wait till PLL is disabled.
            while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) != 0)
            {
                if ((HAL::STM::Get()->GetTick() - tickStart) > PLL_TIMEOUT_VALUE)
                {
                    return Status::Timeout;
                }
            }

#if defined(RCC_PLLI2S_SUPPORT)
            // Get Start Tick.
            tickStart = HAL::STM::Get()->GetTick();

            // Reset PLLI2SON bit.
            CLEAR_BIT(RCC->CR, RCC_CR_PLLI2SON);

            // Wait until PLLI2S is disabled.
            while (READ_BIT(RCC->CR, RCC_CR_PLLI2SRDY) != 0)
            {
                if ((HAL::STM::Get()->GetTick() - tickStart) > PLLI2S_TIMEOUT_VALUE)
                {
                    return Status::Timeout;
                }
            }
#endif // RCC_PLLI2S_SUPPORT

#if defined(RCC_PLLSAI_SUPPORT)
            // Get Start Tick.
            tickStart = HAL::STM::Get()->GetTick();

            // Reset PLLSAI bit.
            CLEAR_BIT(RCC->CR, RCC_CR_PLLSAION);

            // Wait until PLLSAI is disabled.
            while (READ_BIT(RCC->CR, RCC_CR_PLLSAIRDY) != 0)
            {
                if ((HAL::STM::Get()->GetTick() - tickStart) > PLLSAI_TIMEOUT_VALUE)
                {
                    return Status::Timeout;
                }
            }
#endif // RCC_PLLSAI_SUPPORT

            // Once PLL, PLLI2S and PLLSAI are OFF, reset PLLCFGR register to default value.
#if defined(STM32F412Cx) || defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || defined(STM32F413xx) || \
    defined(STM32F423xx) || defined(STM32F446xx) || defined(STM32F469xx) || defined(STM32F479xx)
            RCC->PLLCFGR = RCC_PLLCFGR_PLLM_4 | RCC_PLLCFGR_PLLN_6 | RCC_PLLCFGR_PLLN_7 | RCC_PLLCFGR_PLLQ_2 | RCC_PLLCFGR_PLLR_1;
#elif defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx)
            RCC->PLLCFGR = RCC_PLLCFGR_PLLR_0 | RCC_PLLCFGR_PLLR_1 | RCC_PLLCFGR_PLLR_2 | RCC_PLLCFGR_PLLM_4 | RCC_PLLCFGR_PLLN_6 | RCC_PLLCFGR_PLLN_7 | RCC_PLLCFGR_PLLQ_0 | RCC_PLLCFGR_PLLQ_1 | RCC_PLLCFGR_PLLQ_2 | RCC_PLLCFGR_PLLQ_3;
#else
            RCC->PLLCFGR = RCC_PLLCFGR_PLLM_4 | RCC_PLLCFGR_PLLN_6 | RCC_PLLCFGR_PLLN_7 | RCC_PLLCFGR_PLLQ_2;
#endif // STM32F412Cx || STM32F412Rx || STM32F412Vx || STM32F412Zx || STM32F413xx || STM32F423xx || STM32F446xx || STM32F469xx || STM32F479xx

            // Reset PLLI2SCFGR register to default value.
#if defined(STM32F412Cx) || defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || defined(STM32F413xx) || \
    defined(STM32F423xx) || defined(STM32F446xx)
            RCC->PLLI2SCFGR = RCC_PLLI2SCFGR_PLLI2SM_4 | RCC_PLLI2SCFGR_PLLI2SN_6 | RCC_PLLI2SCFGR_PLLI2SN_7 | RCC_PLLI2SCFGR_PLLI2SQ_2 | RCC_PLLI2SCFGR_PLLI2SR_1;
#elif defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F417xx)
            RCC->PLLI2SCFGR = RCC_PLLI2SCFGR_PLLI2SN_6 | RCC_PLLI2SCFGR_PLLI2SN_7 | RCC_PLLI2SCFGR_PLLI2SR_1;
#elif defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F479xx)
            RCC->PLLI2SCFGR = RCC_PLLI2SCFGR_PLLI2SN_6 | RCC_PLLI2SCFGR_PLLI2SN_7 | RCC_PLLI2SCFGR_PLLI2SQ_2 | RCC_PLLI2SCFGR_PLLI2SR_1;
#elif defined(STM32F411xE)
            RCC->PLLI2SCFGR = RCC_PLLI2SCFGR_PLLI2SM_4 | RCC_PLLI2SCFGR_PLLI2SN_6 | RCC_PLLI2SCFGR_PLLI2SN_7 | RCC_PLLI2SCFGR_PLLI2SR_1;
#endif // STM32F412Cx || STM32F412Rx || STM32F412Vx || STM32F412Zx || STM32F413xx || STM32F423xx || STM32F446xx

            // Reset PLLSAICFGR register.
#if defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F479xx)
            RCC->PLLSAICFGR = RCC_PLLSAICFGR_PLLSAIN_6 | RCC_PLLSAICFGR_PLLSAIN_7 | RCC_PLLSAICFGR_PLLSAIQ_2 | RCC_PLLSAICFGR_PLLSAIR_1;
#elif defined(STM32F446xx)
            RCC->PLLSAICFGR = RCC_PLLSAICFGR_PLLSAIM_4 | RCC_PLLSAICFGR_PLLSAIN_6 | RCC_PLLSAICFGR_PLLSAIN_7 | RCC_PLLSAICFGR_PLLSAIQ_2;
#endif // STM32F427xx || STM32F429xx || STM32F437xx || STM32F439xx || STM32F469xx || STM32F479xx

            // Disable all interrupts.
            CLEAR_BIT(RCC->CIR, RCC_CIR_LSIRDYIE | RCC_CIR_LSERDYIE | RCC_CIR_HSIRDYIE | RCC_CIR_HSERDYIE | RCC_CIR_PLLRDYIE);

#if defined(RCC_CIR_PLLI2SRDYIE)
            CLEAR_BIT(RCC->CIR, RCC_CIR_PLLI2SRDYIE);
#endif // RCC_CIR_PLLI2SRDYIE

#if defined(RCC_CIR_PLLSAIRDYIE)
            CLEAR_BIT(RCC->CIR, RCC_CIR_PLLSAIRDYIE);
#endif // RCC_CIR_PLLSAIRDYIE

            // Clear all interrupt flags.
            SET_BIT(RCC->CIR, RCC_CIR_LSIRDYC | RCC_CIR_LSERDYC | RCC_CIR_HSIRDYC | RCC_CIR_HSERDYC | RCC_CIR_PLLRDYC | RCC_CIR_CSSC);

#if defined(RCC_CIR_PLLI2SRDYC)
            SET_BIT(RCC->CIR, RCC_CIR_PLLI2SRDYC);
#endif // RCC_CIR_PLLI2SRDYC

#if defined(RCC_CIR_PLLSAIRDYC)
            SET_BIT(RCC->CIR, RCC_CIR_PLLSAIRDYC);
#endif // RCC_CIR_PLLSAIRDYC

            // Clear LSION bit.
            CLEAR_BIT(RCC->CSR, RCC_CSR_LSION);

            // Reset all CSR flags.
            SET_BIT(RCC->CSR, RCC_CSR_RMVF);

            // Update the SystemCoreClock global variable.
            m_systemCoreClock = HSI_VALUE;

            // Adapt Systick interrupt period.
            if (STM::Get()->InitTick(uwTickPrio) != Status::Ok)
            {
                return Status::Error;
            }
            else
            {
                return Status::Ok;
            }
        }

        Status RstClkCtrl::OscConfig(const OscInit &config);
        Status RstClkCtrl::ClockConfig(const ClkInit &config, uint32_t fLatency);

        void RstClkCtrl::ConfigMco(McoIndex mco, uint32_t src, McoXClockPrescaller div);
        void RstClkCtrl::EnableCss();
        void RstClkCtrl::DisableCss();

        uint32_t RstClkCtrl::GetSysClockFreq() const;
        uint32_t RstClkCtrl::GetHclkFreq() const;
        uint32_t RstClkCtrl::GetPclk1Freq() const;
        uint32_t RstClkCtrl::GetPclk2Freq() const;
        const OscInit &RstClkCtrl::GetOscConfig() const { return m_oscConfig; }
        const ClkInit &RstClkCtrl::GetClockConfig() const { return m_clkConfig; };

        // CSS NMI IRQ Handler.
        void RstClkCtrl::NmiIrqHandler();

        // User Callbacks in non blocking mode (IT mode).
        void RstClkCtrl::CssCallback();
#endif
    } // namespace RstClkCtrl
    /** @}
 * RCC
 */
} // namespace HAL
  /** @}
 * STM32F4xx_HAL_Driver
 */