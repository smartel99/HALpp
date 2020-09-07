/**
 * @file        stm32f405zgtx.h
 * @author      Samuel Martel
 * @p           https://www.github.com/smartel99
 * @date        2020/09/06
 * @brief       Header file of RCC STM32 module.
 *              
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
#ifndef __STM32F405ZGTX_RCC_H__
#define __STM32F405ZGTX_RCC_H__
#include "stm32f405zgtx_def.h"
#include "stm32f405xx.h"

/** @addtogroup STM32
 * @{
 */
namespace HAL
{
    /** @defgroup RCC RCC
     * @{
     */
    namespace RstCtrlClk
    {
        /**
         * @defgroup Exported_Constants Exported Constants
         * @{
         */

        /** @defgroup OscillatorType Oscillator Type
         * @{
         */
        enum class OscillatorType
        {
            None = 0x00,
            HSE = 0x01,
            HSi = 0x02,
            LSE = 0x04,
            LSI = 0x08
        };
        /** @} */

        /** @defgroup HseConfig HSE Config
         * @{
         */
        enum class HseConfig
        {
            Off = 0x00,
            On = RCC_CR_HSEON,
            Bypass = ((uint32_t)(RCC_CR_HSEBYP | RCC_CR_HSEON))
        };
        /** @} */

        /** @defgroup LseConfig LSE Config
         * @{
         */
        enum class LseConfig
        {
            Off = 0x00,
            On = RCC_BDCR_LSEON,
            Bypass = ((uint32_t)(RCC_BDCR_LSEBYP | RCC_BDCR_LSEON))
        };
        /** @} */

        /** @defgroup HsiConfig HSI Config
         * @{
         */
        enum class HsiConfig
        {
            Off = 0x00,
            On = 0x01,
        };

        /**
         * @brief Default HSI calibration trimming value.
         */
        constexpr uint32_t HsiCalibrationDefault = 0x10U;
        /** @} */

        /** @defgroup LsiConfig LSI Config
         * @{
         */
        enum class LsiConfig
        {
            Off = 0x00,
            On = 0x01
        };
        /** @} */

        /** @defgroup PllConfig PLL Config
         * @{
         */
        enum class PllConfig
        {
            None = 0x00,
            Off = 0x01,
            On = 0x02,
        };
        /** @} */

        /** @defgroup PllPClockDivider PLLP Clock Divider
         * @{
         */
        enum class PllPClockDivider
        {
            Div2 = 0x00000002U,
            Div4 = 0x00000004U,
            Div6 = 0x00000006U,
            Div8 = 0x00000008U
        };
        /** @} */

        /** @defgroup PllClockSource PLL Clock Source
         * @{
         */
        enum class PllClockSource
        {
            HSI = RCC_PLLCFGR_PLLSRC_HSI,
            HSE = RCC_PLLCFGR_PLLSRC_HSE,
        };
        /** @} */

        /** @defgroup SystemClockType System Clock Type
         * @{
         */
        enum class SystemClockType
        {
            SYSCLK = 0x00000001U,
            HCLK = 0x00000002U,
            PCLK1 = 0x00000004U,
            PCLK2 = 0x00000008U
        };
        /** @} */

        /** @defgroup SystemClkSource System Clock Source
         * @note     The RCC_SYSCLKSOURCE_PLLRCLK parameter is available only for
         *           STM32F446xx devices.
         * @{
         */
        enum class SystemClkSource
        {
            HSI = RCC_CFGR_SW_HSI,
            HSE = RCC_CFGR_SW_HSE,
            PLLCLK = RCC_CFGR_SW_PLL,
            PLLRCLK = ((uint32_t)(RCC_CFGR_SW_0 | RCC_CFGR_SW_1))
        };
        /** @} */

        /** @defgroup SystemClkSourceStatus System Clock Source Status
         * @note     The RCC_SYSCLKSOURCE_STATUS_PLLRCLK parameter is available only for
         *           STM32F446xx devices.
         * @{
         */
        enum class SystemClkSourceStatus
        {
            HSI = RCC_CFGR_SWS_HSI,                                  /*!< HSI used as system clock */
            HSE = RCC_CFGR_SWS_HSE,                                  /*!< HSE used as system clock */
            PLLCLK = RCC_CFGR_SWS_PLL,                               /*!< PLL used as system clock */
            PLLRCLK = ((uint32_t)(RCC_CFGR_SWS_0 | RCC_CFGR_SWS_1)), /*!< PLLR used as system clock */
        };
        /** @} */

        /** @defgroup SysclkDivider SYSCLK Clock Divider
         * @{
         */
        enum class SysclkDivider
        {
            Div1 = RCC_CFGR_HPRE_DIV1,
            Div2 = RCC_CFGR_HPRE_DIV2,
            Div4 = RCC_CFGR_HPRE_DIV4,
            Div8 = RCC_CFGR_HPRE_DIV8,
            Div16 = RCC_CFGR_HPRE_DIV16,
            Div64 = RCC_CFGR_HPRE_DIV64,
            Div128 = RCC_CFGR_HPRE_DIV128,
            Div256 = RCC_CFGR_HPRE_DIV256,
            Div512 = RCC_CFGR_HPRE_DIV512
        };
        /** @} */

        /** @defgroup HclkDivider HCLK Clock Divider
         * @{
         */
        enum class HclkDivider
        {
            Div1 = RCC_CFGR_PPRE1_DIV1,
            Div2 = RCC_CFGR_PPRE1_DIV2,
            Div4 = RCC_CFGR_PPRE1_DIV4,
            Div8 = RCC_CFGR_PPRE1_DIV8,
            Div16 = RCC_CFGR_PPRE1_DIV16
        };
        /** @} */

        /** @defgroup RtcClockSource RTC Clock Source
         * @{
         */
        enum class RtcClockSource
        {
            NO_CLK = 0x00000000U,
            LSE = 0x00000100U,
            LSI = 0x00000200U,
            HSE_DIVX = 0x00000300U,
            HSE_DIV2 = 0x00020300U,
            HSE_DIV3 = 0x00030300U,
            HSE_DIV4 = 0x00040300U,
            HSE_DIV5 = 0x00050300U,
            HSE_DIV6 = 0x00060300U,
            HSE_DIV7 = 0x00070300U,
            HSE_DIV8 = 0x00080300U,
            HSE_DIV9 = 0x00090300U,
            HSE_DIV10 = 0x000A0300U,
            HSE_DIV11 = 0x000B0300U,
            HSE_DIV12 = 0x000C0300U,
            HSE_DIV13 = 0x000D0300U,
            HSE_DIV14 = 0x000E0300U,
            HSE_DIV15 = 0x000F0300U,
            HSE_DIV16 = 0x00100300U,
            HSE_DIV17 = 0x00110300U,
            HSE_DIV18 = 0x00120300U,
            HSE_DIV19 = 0x00130300U,
            HSE_DIV20 = 0x00140300U,
            HSE_DIV21 = 0x00150300U,
            HSE_DIV22 = 0x00160300U,
            HSE_DIV23 = 0x00170300U,
            HSE_DIV24 = 0x00180300U,
            HSE_DIV25 = 0x00190300U,
            HSE_DIV26 = 0x001A0300U,
            HSE_DIV27 = 0x001B0300U,
            HSE_DIV28 = 0x001C0300U,
            HSE_DIV29 = 0x001D0300U,
            HSE_DIV30 = 0x001E0300U,
            HSE_DIV31 = 0x001F0300U,
        };
        /** @} */

        /** @defgroup McoIndex MCO Index
         * @{
         */
        enum class McoIndex
        {
            MCO1 = 0x00000000U,
            MCO2 = 0x00000001U,
        };
        /** @} */

        /** @defgroup Mco1ClockSource MCO1 Clock Source
         * @{
         */
        enum class Mco1ClockSource
        {
            HSI = 0x000000000U,
            LSE = RCC_CFGR_MCO1_0,
            HSE = RCC_CFGR_MCO1_1,
            PLLCLK = RCC_CFGR_MCO1
        };
        /** @} */

        /** @defgroup Mco2ClockSource MCO2 Clock Source
         * @{
         */
        enum class Mco2ClockSource
        {
            SYSCLK = 0x00000000U,
            PLLI2SCLK = RCC_CFGR_MCO2_0,
            HSE = RCC_CFGR_MCO2_1,
            PLLCLK = RCC_CFGR_MCO2
        };
        /** @} */

        /** @defgroup McoXClockPrescaller MCOx Clock Prescaller
         * @{
         */
        enum class McoXClockPrescaller
        {
            Div1 = 0x00000000U,
            Div2 = RCC_CFGR_MCO1PRE_2,
            Div3 = ((uint32_t)RCC_CFGR_MCO1PRE_0 | RCC_CFGR_MCO1PRE_2),
            Div4 = ((uint32_t)RCC_CFGR_MCO1PRE_1 | RCC_CFGR_MCO1PRE_2),
        };
        /** @} */

        /** @defgroup Interrupt Interrupts
         * @{
         */
        enum class Interrupt
        {
            LSIRdy = 0x01,
            LSERdy = 0x02,
            HSIRdy = 0x04,
            HSERdy = 0x08,
            PLLRdy = 0x10,
            PLLI2SRdy = 0x20,
            CSS = 0x80
        };
        /** @} */

        /** @defgroup   Flag Flags
         *              Elements values convention: 0XXYYYYYb
         *                  - YYYYY : Flag position in the register
         *                  - XX : Register index
         *                      - 01 : CR register
         *                      - 10 : BDCR register
         *                      - 11 : CSR register
         * @{
         */
        enum class Flags
        {
            HSIRdy = 0x21,
            HSERdy = 0x31,
            PLLRdy = 0x39,
            PLLI2SRdy = 0x3B,
            LSERdy = 0x41,
            LSIRdy = 0x61,
            BORRST = 0x79,
            PINRST = 0x7A,
            PORRST = 0x7B,
            SFTRST = 0x7C,
            IWDGRST = 0x7D,
            WWDGRST = 0x7E,
            LPWRRST = 0x7F
        };

        /** @} 
         * Flag
        */

        /** @} 
         * Exported_Constants
        */

        /*****************************************************************************/
        /* Exported Macros                                                           */
        /*****************************************************************************/
        /** @defgroup ExportedMacros RCC Exported Macros
         * @{
         */

        /** @defgroup RCC_AHB1_Clock_Enable_Disable AHB1 Peripheral Clock Enable Disable
         * @brief   Enable or disable the AHB1 peripheral clock.
         * @note    After reset, the peripheral clock (used for registers read/write access)
         *          is disabled and the application software has to enable this clock before
         *          using it.
         * @{
         */
        constexpr void GpioAClkEnable()
        {
            __IO uint32_t tmpreg = 0x00;
            SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);
            // Delay after an RCC peripheral clock enabling.
            tmpreg = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);
        }

        constexpr void GpioBClkEnable()
        {
            __IO uint32_t tmpreg = 0x00;
            SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN);
            // Delay after an RCC peripheral clock enabling.
            tmpreg = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN);
        }

        constexpr void GpioCClkEnable()
        {
            __IO uint32_t tmpreg = 0x00;
            SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN);
            // Delay after an RCC peripheral clock enabling.
            tmpreg = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN);
        }

        constexpr void GpioHClkEnable()
        {
            __IO uint32_t tmpreg = 0x00;
            SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOHEN);
            // Delay after an RCC peripheral clock enabling.
            tmpreg = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOHEN);
        }

        constexpr void Dma1ClkEnable()
        {
            __IO uint32_t tmpreg = 0x00;
            SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA1EN);
            // Delay after an RCC peripheral clock enabling.
            tmpreg = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA1EN);
        }

        constexpr void Dma2ClkEnable()
        {
            __IO uint32_t tmpreg = 0x00;
            SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA2EN);
            // Delay after an RCC peripheral clock enabling.
            tmpreg = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA2EN);
        }

        constexpr void GpioAClkDisable() { RCC->AHB1ENR &= ~(RCC_AHB1ENR_GPIOAEN); }
        constexpr void GpioBClkDisable() { RCC->AHB1ENR &= ~(RCC_AHB1ENR_GPIOBEN); }
        constexpr void GpioCClkDisable() { RCC->AHB1ENR &= ~(RCC_AHB1ENR_GPIOCEN); }
        constexpr void GpioHClkDisable() { RCC->AHB1ENR &= ~(RCC_AHB1ENR_GPIOHEN); }
        constexpr void Dma1ClkDisable() { RCC->AHB1ENR &= ~(RCC_AHB1ENR_DMA1EN); }
        constexpr void Dma2ClkDisable() { RCC->AHB1ENR &= ~(RCC_AHB1ENR_DMA2EN); }
        /** @}
         * RCC_AHB1_Clock_Enable_Disable
         */

        /** @defgroup RCC_AHB1_Peripheral_Clock_Enable_Disable_Status AHB1 Peripheral Clock Enable Disable Status
         * @brief  Get the enable or disable status of the AHB1 peripheral clock.
         * @note   After reset, the peripheral clock (used for registers read/write access)
         *         is disabled and the application software has to enable this clock before
         *         using it.
         * @{
         */
        constexpr bool GpioAIsClkEnabled() { return ((RCC->AHB1ENR & (RCC_AHB1ENR_GPIOAEN)) != 0); }
        constexpr bool GpioBIsClkEnabled() { return ((RCC->AHB1ENR & (RCC_AHB1ENR_GPIOBEN)) != 0); }
        constexpr bool GpioCIsClkEnabled() { return ((RCC->AHB1ENR & (RCC_AHB1ENR_GPIOCEN)) != 0); }
        constexpr bool GpioHIsClkEnabled() { return ((RCC->AHB1ENR & (RCC_AHB1ENR_GPIOHEN)) != 0); }
        constexpr bool Dma1IsClkEnabled() { return ((RCC->AHB1ENR & (RCC_AHB1ENR_DMA1EN)) != 0); }
        constexpr bool Dma2IsClkEnabled() { return ((RCC->AHB1ENR & (RCC_AHB1ENR_DMA2EN)) != 0); }

        constexpr bool GpioAIsClkDisabled() { return ((RCC->AHB1ENR & (RCC_AHB1ENR_GPIOAEN)) == 0); }
        constexpr bool GpioBIsClkDisabled() { return ((RCC->AHB1ENR & (RCC_AHB1ENR_GPIOBEN)) == 0); }
        constexpr bool GpioCIsClkDisabled() { return ((RCC->AHB1ENR & (RCC_AHB1ENR_GPIOCEN)) == 0); }
        constexpr bool GpioHIsClkDisabled() { return ((RCC->AHB1ENR & (RCC_AHB1ENR_GPIOHEN)) == 0); }
        constexpr bool Dma1IsClkDisabled() { return ((RCC->AHB1ENR & (RCC_AHB1ENR_DMA1EN)) == 0); }
        constexpr bool Dma2IsClkDisabled() { return ((RCC->AHB1ENR & (RCC_AHB1ENR_DMA2EN)) == 0); }
        /** @}
         * RCC_AHB1_Peripheral_Clock_Enable_Disable_Status
         */

        /** @defgroup RCC_APB1_Clock_Enable_Disable APB1 Peripheral Clock Enable Disable
         * @brief  Enable or disable the Low Speed APB (APB1) peripheral clock.
         * @note   After reset, the peripheral clock (used for registers read/write access)
         *         is disabled and the application software has to enable this clock before
         *         using it.
         * @{
         */
        constexpr void Tim5ClkEnable()
        {
            __IO uint32_t tmpreg = 0x00U;
            SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM5EN);
            /* Delay after an RCC peripheral clock enabling */
            tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM5EN);
        }

        constexpr void WwdgClkEnable()
        {
            __IO uint32_t tmpreg = 0x00U;
            SET_BIT(RCC->APB1ENR, RCC_APB1ENR_WWDGEN);
            /* Delay after an RCC peripheral clock enabling */
            tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_WWDGEN);
        }

        constexpr void Spi2ClkEnable()
        {
            __IO uint32_t tmpreg = 0x00U;
            SET_BIT(RCC->APB1ENR, RCC_APB1ENR_SPI2EN);
            /* Delay after an RCC peripheral clock enabling */
            tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_SPI2EN);
        }

        constexpr void Usart2ClkEnable()
        {
            __IO uint32_t tmpreg = 0x00U;
            SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN);
            /* Delay after an RCC peripheral clock enabling */
            tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN);
        }

        constexpr void I2c1ClkEnable()
        {
            __IO uint32_t tmpreg = 0x00U;
            SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C1EN);
            /* Delay after an RCC peripheral clock enabling */
            tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C1EN);
        }

        constexpr void I2c2ClkEnable()
        {
            __IO uint32_t tmpreg = 0x00U;
            SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C2EN);
            /* Delay after an RCC peripheral clock enabling */
            tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C2EN);
        }

        constexpr void PwrClkEnable()
        {
            __IO uint32_t tmpreg = 0x00U;
            SET_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);
            /* Delay after an RCC peripheral clock enabling */
            tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);
        }

        constexpr void Tim5ClkDisable() { (RCC->APB1ENR &= ~(RCC_APB1ENR_TIM5EN)); }
        constexpr void WwdgClkDisable() { (RCC->APB1ENR &= ~(RCC_APB1ENR_WWDGEN)); }
        constexpr void Spi2ClkDisable() { (RCC->APB1ENR &= ~(RCC_APB1ENR_SPI2EN)); }
        constexpr void Usart2ClkDisable() { (RCC->APB1ENR &= ~(RCC_APB1ENR_USART2EN)); }
        constexpr void I2c1ClkDisable() { (RCC->APB1ENR &= ~(RCC_APB1ENR_I2C1EN)); }
        constexpr void I2c2ClkDisable() { (RCC->APB1ENR &= ~(RCC_APB1ENR_I2C2EN)); }
        constexpr void PwrClkDisable() { (RCC->APB1ENR &= ~(RCC_APB1ENR_PWREN)); }
        /** @}
         * RCC_APB1_Clock_Enable_Disable
         */

        /** @defgroup RCC_APB1_Peripheral_Clock_Enable_Disable_Status APB1 Peripheral Clock Enable Disable Status
         * @brief  Get the enable or disable status of the APB1 peripheral clock.
         * @note   After reset, the peripheral clock (used for registers read/write access)
         *         is disabled and the application software has to enable this clock before
         *         using it.
         * @{
         */
        constexpr bool Tim5IsClkEnabled() { return ((RCC->APB1ENR & (RCC_APB1ENR_TIM5EN)) != 0); }
        constexpr bool WwdgIsClkEnabled() { return ((RCC->APB1ENR & (RCC_APB1ENR_WWDGEN)) != 0); }
        constexpr bool Spi2IsClkEnabled() { return ((RCC->APB1ENR & (RCC_APB1ENR_SPI2EN)) != 0); }
        constexpr bool Usart2IsClkEnabled() { return ((RCC->APB1ENR & (RCC_APB1ENR_USART2EN)) != 0); }
        constexpr bool I2c1IsClkEnabled() { return ((RCC->APB1ENR & (RCC_APB1ENR_I2C1EN)) != 0); }
        constexpr bool I2c2IsClkEnabled() { return ((RCC->APB1ENR & (RCC_APB1ENR_I2C2EN)) != 0); }
        constexpr bool PwrIsClkEnabled() { return ((RCC->APB1ENR & (RCC_APB1ENR_PWREN)) != 0); }

        constexpr bool Tim5IsClkDisabled() { return ((RCC->APB1ENR & (RCC_APB1ENR_TIM5EN)) == 0); }
        constexpr bool WwdgIsClkDisabled() { return ((RCC->APB1ENR & (RCC_APB1ENR_WWDGEN)) == 0); }
        constexpr bool Spi2IsClkDisabled() { return ((RCC->APB1ENR & (RCC_APB1ENR_SPI2EN)) == 0); }
        constexpr bool Usart2IsClkDisabled() { return ((RCC->APB1ENR & (RCC_APB1ENR_USART2EN)) == 0); }
        constexpr bool I2c1IsClkDisabled() { return ((RCC->APB1ENR & (RCC_APB1ENR_I2C1EN)) == 0); }
        constexpr bool I2c2IsClkDisabled() { return ((RCC->APB1ENR & (RCC_APB1ENR_I2C2EN)) == 0); }
        constexpr bool PwrIsClkDisabled() { return ((RCC->APB1ENR & (RCC_APB1ENR_PWREN)) == 0); }
        /** @}
         * RCC_APB1_Peripheral_Clock_Enable_Disable_Status
         */

        /** @defgroup RCC_APB2_Clock_Enable_Disable APB2 Peripheral Clock Enable Disable
         * @brief  Enable or disable the High Speed APB (APB2) peripheral clock.
         * @note   After reset, the peripheral clock (used for registers read/write access)
         *         is disabled and the application software has to enable this clock before
         *         using it.
         * @{
         */
        constexpr void Tim1ClkEnable()
        {
            __IO uint32_t tmpreg = 0x00U;
            SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM1EN);
            /* Delay after an RCC peripheral clock enabling */
            tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM1EN);
        }

        constexpr void Usart1ClkEnable()
        {
            __IO uint32_t tmpreg = 0x00U;
            SET_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN);
            /* Delay after an RCC peripheral clock enabling */
            tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN);
        }

        constexpr void Usart6ClkEnable()
        {
            __IO uint32_t tmpreg = 0x00U;
            SET_BIT(RCC->APB2ENR, RCC_APB2ENR_USART6EN);
            /* Delay after an RCC peripheral clock enabling */
            tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_USART6EN);
        }

        constexpr void Adc1ClkEnable()
        {
            __IO uint32_t tmpreg = 0x00U;
            SET_BIT(RCC->APB2ENR, RCC_APB2ENR_ADC1EN);
            /* Delay after an RCC peripheral clock enabling */
            tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_ADC1EN);
        }

        constexpr void Spi1ClkEnable()
        {
            __IO uint32_t tmpreg = 0x00U;
            SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI1EN);
            /* Delay after an RCC peripheral clock enabling */
            tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI1EN);
        }

        constexpr void SyscfgClkEnable()
        {
            __IO uint32_t tmpreg = 0x00U;
            SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);
            /* Delay after an RCC peripheral clock enabling */
            tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);
        }

        constexpr void Tim9ClkEnable()
        {
            __IO uint32_t tmpreg = 0x00U;
            SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM9EN);
            /* Delay after an RCC peripheral clock enabling */
            tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM9EN);
        }

        constexpr void Tim11ClkEnable()
        {
            __IO uint32_t tmpreg = 0x00U;
            SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM11EN);
            /* Delay after an RCC peripheral clock enabling */
            tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM11EN);
        }

        constexpr void Tim1ClkDisable() { (RCC->APB2ENR &= ~(RCC_APB2ENR_TIM1EN)); }
        constexpr void Usart1ClkDisable() { (RCC->APB2ENR &= ~(RCC_APB2ENR_USART1EN)); }
        constexpr void Usart6ClkDisable() { (RCC->APB2ENR &= ~(RCC_APB2ENR_USART6EN)); }
        constexpr void Adc1ClkDisable() { (RCC->APB2ENR &= ~(RCC_APB2ENR_ADC1EN)); }
        constexpr void Spi1ClkDisable() { (RCC->APB2ENR &= ~(RCC_APB2ENR_SPI1EN)); }
        constexpr void SyscfgClkDisable() { (RCC->APB2ENR &= ~(RCC_APB2ENR_SYSCFGEN)); }
        constexpr void Tim9ClkDisable() { (RCC->APB2ENR &= ~(RCC_APB2ENR_TIM9EN)); }
        constexpr void Tim11ClkDisable() { (RCC->APB2ENR &= ~(RCC_APB2ENR_TIM11EN)); }
        /** @}
         * RCC_APB2_Clock_Enable_Disable
         */

        /** @defgroup RCC_APB2_Peripheral_Clock_Enable_Disable_Status APB2 Peripheral Clock Enable Disable Status
         * @brief  Get the enable or disable status of the APB2 peripheral clock.
         * @note   After reset, the peripheral clock (used for registers read/write access)
         *         is disabled and the application software has to enable this clock before
         *         using it.
         * @{
         */
        constexpr bool Tim1IsClkEnabled() { return ((RCC->APB2ENR & (RCC_APB2ENR_TIM1EN)) != 0); }
        constexpr bool Usart1IsClkEnabled() { return ((RCC->APB2ENR & (RCC_APB2ENR_USART1EN)) != 0); }
        constexpr bool Usart6IsClkEnabled() { return ((RCC->APB2ENR & (RCC_APB2ENR_USART6EN)) != 0); }
        constexpr bool Adc1IsClkEnabled() { return ((RCC->APB2ENR & (RCC_APB2ENR_ADC1EN)) != 0); }
        constexpr bool Spi1IsClkEnabled() { return ((RCC->APB2ENR & (RCC_APB2ENR_SPI1EN)) != 0); }
        constexpr bool SyscfgIsClkEnabled() { return ((RCC->APB2ENR & (RCC_APB2ENR_SYSCFGEN)) != 0); }
        constexpr bool Tim9IsClkEnabled() { return ((RCC->APB2ENR & (RCC_APB2ENR_TIM9EN)) != 0); }
        constexpr bool Tim11IsClkEnabled() { return ((RCC->APB2ENR & (RCC_APB2ENR_TIM11EN)) != 0); }

        constexpr bool Tim1IsClkDisabled() { return ((RCC->APB2ENR & (RCC_APB2ENR_TIM1EN)) == 0); }
        constexpr bool Usart1IsClkDisabled() { return ((RCC->APB2ENR & (RCC_APB2ENR_USART1EN)) == 0); }
        constexpr bool Usart6IsClkDisabled() { return ((RCC->APB2ENR & (RCC_APB2ENR_USART6EN)) == 0); }
        constexpr bool Adc1IsClkDisabled() { return ((RCC->APB2ENR & (RCC_APB2ENR_ADC1EN)) == 0); }
        constexpr bool Spi1IsClkDisabled() { return ((RCC->APB2ENR & (RCC_APB2ENR_SPI1EN)) == 0); }
        constexpr bool SyscfgIsClkDisabled() { return ((RCC->APB2ENR & (RCC_APB2ENR_SYSCFGEN)) == 0); }
        constexpr bool Tim9IsClkDisabled() { return ((RCC->APB2ENR & (RCC_APB2ENR_TIM9EN)) == 0); }
        constexpr bool Tim11IsClkDisabled() { return ((RCC->APB2ENR & (RCC_APB2ENR_TIM11EN)) == 0); }
        /** @}
         * RCC_APB2_Peripheral_Clock_Enable_Disable_Status
         */

        /** @defgroup RCC_AHB1_Force_Release_Reset AHB1 Force Release Reset
         * @brief  Force or release AHB1 peripheral reset.
         * @{
         */
        constexpr void Ahb1ForceReset() { (RCC->AHB1RSTR = 0xFFFFFFFFU); }
        constexpr void GpioAForceReset() { (RCC->AHB1RSTR |= (RCC_AHB1RSTR_GPIOARST)); }
        constexpr void GpioBForceReset() { (RCC->AHB1RSTR |= (RCC_AHB1RSTR_GPIOBRST)); }
        constexpr void GpioCForceReset() { (RCC->AHB1RSTR |= (RCC_AHB1RSTR_GPIOCRST)); }
        constexpr void GpioHForceReset() { (RCC->AHB1RSTR |= (RCC_AHB1RSTR_GPIOHRST)); }
        constexpr void Dma1ForceReset() { (RCC->AHB1RSTR |= (RCC_AHB1RSTR_DMA1RST)); }
        constexpr void Dma2ForceReset() { (RCC->AHB1RSTR |= (RCC_AHB1RSTR_DMA2RST)); }

        constexpr void Ahb1ReleaseReset() { (RCC->AHB1RSTR = 0x00U); }
        constexpr void GpioAReleaseReset() { (RCC->AHB1RSTR &= ~(RCC_AHB1RSTR_GPIOARST)); }
        constexpr void GpioBReleaseReset() { (RCC->AHB1RSTR &= ~(RCC_AHB1RSTR_GPIOBRST)); }
        constexpr void GpioCReleaseReset() { (RCC->AHB1RSTR &= ~(RCC_AHB1RSTR_GPIOCRST)); }
        constexpr void GpioHReleaseReset() { (RCC->AHB1RSTR &= ~(RCC_AHB1RSTR_GPIOHRST)); }
        constexpr void Dma1ReleaseReset() { (RCC->AHB1RSTR &= ~(RCC_AHB1RSTR_DMA1RST)); }
        constexpr void Dma2ReleaseReset() { (RCC->AHB1RSTR &= ~(RCC_AHB1RSTR_DMA2RST)); }
        /** @}
         * RCC_AHB1_Force_Release_Reset
         */

        /** @defgroup RCC_APB1_Force_Release_Reset APB1 Force Release Reset
         * @brief  Force or release APB1 peripheral reset.
         * @{
         */
        constexpr void Apb1ForceReset() { (RCC->APB1RSTR = 0xFFFFFFFFU); }
        constexpr void Tim5ForceReset() { (RCC->APB1RSTR |= (RCC_APB1RSTR_TIM5RST)); }
        constexpr void WwdgForceReset() { (RCC->APB1RSTR |= (RCC_APB1RSTR_WWDGRST)); }
        constexpr void Spi1ForceReset() { (RCC->APB1RSTR |= (RCC_APB1RSTR_SPI2RST)); }
        constexpr void Usart2ForceReset() { (RCC->APB1RSTR |= (RCC_APB1RSTR_USART2RST)); }
        constexpr void I2c1ForceReset() { (RCC->APB1RSTR |= (RCC_APB1RSTR_I2C1RST)); }
        constexpr void I2c2ForceReset() { (RCC->APB1RSTR |= (RCC_APB1RSTR_I2C2RST)); }
        constexpr void PwrForceReset() { (RCC->APB1RSTR |= (RCC_APB1RSTR_PWRRST)); }

        constexpr void Apb1ReleaseReset() { (RCC->APB1RSTR = 0x00U); }
        constexpr void Tim5ReleaseReset() { (RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM5RST)); }
        constexpr void WwdgReleaseReset() { (RCC->APB1RSTR &= ~(RCC_APB1RSTR_WWDGRST)); }
        constexpr void Spi1ReleaseReset() { (RCC->APB1RSTR &= ~(RCC_APB1RSTR_SPI2RST)); }
        constexpr void Usart2ReleaseReset() { (RCC->APB1RSTR &= ~(RCC_APB1RSTR_USART2RST)); }
        constexpr void I2c1ReleaseReset() { (RCC->APB1RSTR &= ~(RCC_APB1RSTR_I2C1RST)); }
        constexpr void I2c2ReleaseReset() { (RCC->APB1RSTR &= ~(RCC_APB1RSTR_I2C2RST)); }
        constexpr void PwrReleaseReset() { (RCC->APB1RSTR &= ~(RCC_APB1RSTR_PWRRST)); }
        /** @}
         * RCC_APB1_Force_Release_Reset
         */

        /** @defgroup RCC_APB2_Force_Release_Reset APB2 Force Release Reset
         * @brief  Force or release APB2 peripheral reset.
         * @{
         */
        constexpr void Apb2ForceReset() { (RCC->APB2RSTR = 0xFFFFFFFFU); }
        constexpr void Tim1ForceReset() { (RCC->APB2RSTR |= (RCC_APB2RSTR_TIM1RST)); }
        constexpr void Usart1ForceReset() { (RCC->APB2RSTR |= (RCC_APB2RSTR_USART1RST)); }
        constexpr void Usart6ForceReset() { (RCC->APB2RSTR |= (RCC_APB2RSTR_USART6RST)); }
        constexpr void AdcForceReset() { (RCC->APB2RSTR |= (RCC_APB2RSTR_ADCRST)); }
        constexpr void Spi1ForceReset() { (RCC->APB2RSTR |= (RCC_APB2RSTR_SPI1RST)); }
        constexpr void SyscfgForceReset() { (RCC->APB2RSTR |= (RCC_APB2RSTR_SYSCFGRST)); }
        constexpr void Tim9ForceReset() { (RCC->APB2RSTR |= (RCC_APB2RSTR_TIM9RST)); }
        constexpr void Tim11ForceReset() { (RCC->APB2RSTR |= (RCC_APB2RSTR_TIM11RST)); }

        constexpr void Apb2ReleaseReset() { (RCC->APB2RSTR = 0x00U); }
        constexpr void Tim1ReleaseReset() { (RCC->APB2RSTR &= ~(RCC_APB2RSTR_TIM1RST)); }
        constexpr void Usart1ReleaseReset() { (RCC->APB2RSTR &= ~(RCC_APB2RSTR_USART1RST)); }
        constexpr void Usart6ReleaseReset() { (RCC->APB2RSTR &= ~(RCC_APB2RSTR_USART6RST)); }
        constexpr void AdcReleaseReset() { (RCC->APB2RSTR &= ~(RCC_APB2RSTR_ADCRST)); }
        constexpr void Spi1ReleaseReset() { (RCC->APB2RSTR &= ~(RCC_APB2RSTR_SPI1RST)); }
        constexpr void SyscfgReleaseReset() { (RCC->APB2RSTR &= ~(RCC_APB2RSTR_SYSCFGRST)); }
        constexpr void Tim9ReleaseReset() { (RCC->APB2RSTR &= ~(RCC_APB2RSTR_TIM9RST)); }
        constexpr void Tim11ReleaseReset() { (RCC->APB2RSTR &= ~(RCC_APB2RSTR_TIM11RST)); }
        /** @}
         * RCC_APB2_Force_Release_Reset
         */

        /** @defgroup RCC_AHB1_LowPower_Enable_Disable AHB1 Peripheral Low Power Enable Disable
         * @brief  Enable or disable the AHB1 peripheral clock during Low Power (Sleep) mode.
         * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
         *         power consumption.
         * @note   After wake-up from SLEEP mode, the peripheral clock is enabled again.
         * @note   By default, all peripheral clocks are enabled during SLEEP mode.
         * @{
         */
        constexpr void GpioAClkSleepEnable() { (RCC->AHB1LPENR |= (RCC_AHB1LPENR_GPIOALPEN)); }
        constexpr void GpioBClkSleepEnable() { (RCC->AHB1LPENR |= (RCC_AHB1LPENR_GPIOBLPEN)); }
        constexpr void GpioCClkSleepEnable() { (RCC->AHB1LPENR |= (RCC_AHB1LPENR_GPIOCLPEN)); }
        constexpr void GpioHClkSleepEnable() { (RCC->AHB1LPENR |= (RCC_AHB1LPENR_GPIOHLPEN)); }
        constexpr void Dma1ClkSleepEnable() { (RCC->AHB1LPENR |= (RCC_AHB1LPENR_DMA1LPEN)); }
        constexpr void Dma2ClkSleepEnable() { (RCC->AHB1LPENR |= (RCC_AHB1LPENR_DMA2LPEN)); }

        constexpr void GpioAClkSleepDisable() { (RCC->AHB1LPENR &= ~(RCC_AHB1LPENR_GPIOALPEN)); }
        constexpr void GpioBClkSleepDisable() { (RCC->AHB1LPENR &= ~(RCC_AHB1LPENR_GPIOBLPEN)); }
        constexpr void GpioCClkSleepDisable() { (RCC->AHB1LPENR &= ~(RCC_AHB1LPENR_GPIOCLPEN)); }
        constexpr void GpioHClkSleepDisable() { (RCC->AHB1LPENR &= ~(RCC_AHB1LPENR_GPIOHLPEN)); }
        constexpr void Dma1ClkSleepDisable() { (RCC->AHB1LPENR &= ~(RCC_AHB1LPENR_DMA1LPEN)); }
        constexpr void Dma2ClkSleepDisable() { (RCC->AHB1LPENR &= ~(RCC_AHB1LPENR_DMA2LPEN)); }
        /** @}
         * RCC_AHB1_LowPower_Enable_Disable
         */

        /** @defgroup RCC_APB1_LowPower_Enable_Disable APB1 Peripheral Low Power Enable Disable
         * @brief  Enable or disable the APB1 peripheral clock during Low Power (Sleep) mode.
         * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
         *         power consumption.
         * @note   After wake-up from SLEEP mode, the peripheral clock is enabled again.
         * @note   By default, all peripheral clocks are enabled during SLEEP mode.
         * @{
         */
        constexpr void Tim5ClkSleepEnable() { (RCC->APB1LPENR |= (RCC_APB1LPENR_TIM5LPEN)); }
        constexpr void WwdgClkSleepEnable() { (RCC->APB1LPENR |= (RCC_APB1LPENR_WWDGLPEN)); }
        constexpr void Spi2ClkSleepEnable() { (RCC->APB1LPENR |= (RCC_APB1LPENR_SPI2LPEN)); }
        constexpr void Usart2ClkSleepEnable() { (RCC->APB1LPENR |= (RCC_APB1LPENR_USART2LPEN)); }
        constexpr void I2c1ClkSleepEnable() { (RCC->APB1LPENR |= (RCC_APB1LPENR_I2C1LPEN)); }
        constexpr void I2c2ClkSleepEnable() { (RCC->APB1LPENR |= (RCC_APB1LPENR_I2C2LPEN)); }
        constexpr void PwrClkSleepEnable() { (RCC->APB1LPENR |= (RCC_APB1LPENR_PWRLPEN)); }

        constexpr void Tim5ClkSleepDisable() { (RCC->APB1LPENR &= ~(RCC_APB1LPENR_TIM5LPEN)); }
        constexpr void WwdgClkSleepDisable() { (RCC->APB1LPENR &= ~(RCC_APB1LPENR_WWDGLPEN)); }
        constexpr void Spi2ClkSleepDisable() { (RCC->APB1LPENR &= ~(RCC_APB1LPENR_SPI2LPEN)); }
        constexpr void Usart2ClkSleepDisable() { (RCC->APB1LPENR &= ~(RCC_APB1LPENR_USART2LPEN)); }
        constexpr void I2c1ClkSleepDisable() { (RCC->APB1LPENR &= ~(RCC_APB1LPENR_I2C1LPEN)); }
        constexpr void I2c2ClkSleepDisable() { (RCC->APB1LPENR &= ~(RCC_APB1LPENR_I2C2LPEN)); }
        constexpr void PwrClkSleepDisable() { (RCC->APB1LPENR &= ~(RCC_APB1LPENR_PWRLPEN)); }
        /** @}
         * RCC_APB1_LowPower_Enable_Disable
         */

        /** @defgroup RCC_APB2_LowPower_Enable_Disable APB2 Peripheral Low Power Enable Disable
         * @brief  Enable or disable the APB2 peripheral clock during Low Power (Sleep) mode.
         * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
         *         power consumption.
         * @note   After wake-up from SLEEP mode, the peripheral clock is enabled again.
         * @note   By default, all peripheral clocks are enabled during SLEEP mode.
         * @{
         */
        constexpr void Tim1ClkSleepEnable() { (RCC->APB2LPENR |= (RCC_APB2LPENR_TIM1LPEN)); }
        constexpr void Usart1ClkSleepEnable() { (RCC->APB2LPENR |= (RCC_APB2LPENR_USART1LPEN)); }
        constexpr void Usart6ClkSleepEnable() { (RCC->APB2LPENR |= (RCC_APB2LPENR_USART6LPEN)); }
        constexpr void Adc1ClkSleepEnable() { (RCC->APB2LPENR |= (RCC_APB2LPENR_ADC1LPEN)); }
        constexpr void Spi1ClkSleepEnable() { (RCC->APB2LPENR |= (RCC_APB2LPENR_SPI1LPEN)); }
        constexpr void SyscfgClkSleepEnable() { (RCC->APB2LPENR |= (RCC_APB2LPENR_SYSCFGLPEN)); }
        constexpr void Tim9ClkSleepEnable() { (RCC->APB2LPENR |= (RCC_APB2LPENR_TIM9LPEN)); }
        constexpr void Tim11ClkSleepEnable() { (RCC->APB2LPENR |= (RCC_APB2LPENR_TIM11LPEN)); }

        constexpr void Tim1ClkSleepDisable() { (RCC->APB2LPENR &= ~(RCC_APB2LPENR_TIM1LPEN)); }
        constexpr void Usart1ClkSleepDisable() { (RCC->APB2LPENR &= ~(RCC_APB2LPENR_USART1LPEN)); }
        constexpr void Usart6ClkSleepDisable() { (RCC->APB2LPENR &= ~(RCC_APB2LPENR_USART6LPEN)); }
        constexpr void Adc1ClkSleepDisable() { (RCC->APB2LPENR &= ~(RCC_APB2LPENR_ADC1LPEN)); }
        constexpr void Spi1ClkSleepDisable() { (RCC->APB2LPENR &= ~(RCC_APB2LPENR_SPI1LPEN)); }
        constexpr void SyscfgClkSleepDisable() { (RCC->APB2LPENR &= ~(RCC_APB2LPENR_SYSCFGLPEN)); }
        constexpr void Tim9ClkSleepDisable() { (RCC->APB2LPENR &= ~(RCC_APB2LPENR_TIM9LPEN)); }
        constexpr void Tim11ClkSleepDisable() { (RCC->APB2LPENR &= ~(RCC_APB2LPENR_TIM11LPEN)); }
        /** @}
         * RCC_APB2_LowPower_Enable_Disable
         */

        /** @defgroup RCC_HSI_Configuration HSI Configuration
         * @{
         */

        /** @brief  Macros to enable or disable the Internal High Speed oscillator (HSI).
         * @note   The HSI is stopped by hardware when entering STOP and STANDBY modes.
         *         It is used (enabled by hardware) as system clock source after startup
         *         from Reset, wake-up from STOP and STANDBY mode, or in case of failure
         *         of the HSE used directly or indirectly as system clock (if the Clock
         *         Security System CSS is enabled).
         * @note   HSI can not be stopped if it is used as system clock source. In this case,
         *         you have to select another source of the system clock then stop the HSI.
         * @note   After enabling the HSI, the application software should wait on HSIRDY
         *         flag to be set indicating that HSI clock is stable and can be used as
         *         system clock source.
         *         This parameter can be: ENABLE or DISABLE.
         * @note   When the HSI is stopped, HSIRDY flag goes low after 6 HSI oscillator
         *         clock cycles.
         */
        constexpr void HsiEnable() { *(__IO uint32_t *)RCC_CR_HSION_BB = ~0U; }
        constexpr void HsiDisable() { *(__IO uint32_t *)RCC_CR_HSION_BB = 0U; }

        /** @brief  Macro to adjust the Internal High Speed oscillator (HSI) calibration value.
         * @note   The calibration is used to compensate for the variations in voltage
         *         and temperature that influence the frequency of the internal HSI RC.
         * @param  __HSICalibrationValue__ specifies the calibration trimming value.
         *         (default is RCC_HSICALIBRATION_DEFAULT).
         *         This parameter must be a number between 0 and 0x1F.
         */
        template <typename T = uint32_t>
        constexpr void HsiCalibrationValueAdjust(T value) { (MODIFY_REG(RCC->CR, RCC_CR_HSITRIM, (uint32_t)(value) << RCC_CR_HSITRIM_Pos)); }
        /** @}
         * RCC_HSI_Configuration
         */

        /** @defgroup RCC_LSI_Configuration LSI Configuration
         * @{
         */

        /** @brief  Macros to enable or disable the Internal Low Speed oscillator (LSI).
         * @note   After enabling the LSI, the application software should wait on
         *         LSIRDY flag to be set indicating that LSI clock is stable and can
         *         be used to clock the IWDG and/or the RTC.
         * @note   LSI can not be disabled if the IWDG is running.
         * @note   When the LSI is stopped, LSIRDY flag goes low after 6 LSI oscillator
         *         clock cycles.
         */
        constexpr void LsiEnable() { *(__IO uint32_t *)RCC_CSR_LSION_BB = ~0U; }
        constexpr void LsiDisable() { *(__IO uint32_t *)RCC_CSR_LSION_BB = 0U; }
        /** @}
         * RCC_LSI_Configuration
         */

        /** @defgroup RCC_HSE_Configuration HSE Configuration
         * @{
         */

        /**
         * @brief  Macro to configure the External High Speed oscillator (HSE).
         * @note   Transition HSE Bypass to HSE On and HSE On to HSE Bypass are not supported by this macro.
         *         User should request a transition to HSE Off first and then HSE On or HSE Bypass.
         * @note   After enabling the HSE (RCC_HSE_ON or RCC_HSE_Bypass), the application
         *         software should wait on HSERDY flag to be set indicating that HSE clock
         *         is stable and can be used to clock the PLL and/or system clock.
         * @note   HSE state can not be changed if it is used directly or through the
         *         PLL as system clock. In this case, you have to select another source
         *         of the system clock then change the HSE state (ex. disable it).
         * @note   The HSE is stopped by hardware when entering STOP and STANDBY modes.
         * @note   This function reset the CSSON bit, so if the clock security system(CSS)
         *         was previously enabled you have to enable it again after calling this
         *         function.
         * @param  __STATE__ specifies the new state of the HSE.
         *         This parameter can be one of the following values:
         *            @arg RCC_HSE_OFF: turn OFF the HSE oscillator, HSERDY flag goes low after
         *                              6 HSE oscillator clock cycles.
         *            @arg RCC_HSE_ON: turn ON the HSE oscillator.
         *            @arg RCC_HSE_BYPASS: HSE oscillator bypassed with external clock.
         */
        constexpr void ConfigHse(HseConfig state)
        {
            if (state == HseConfig::On)
            {
                SET_BIT(RCC->CR, RCC_CR_HSEON);
            }
            else if (state == HseConfig::Bypass)
            {
                SET_BIT(RCC->CR, RCC_CR_HSEBYP);
                SET_BIT(RCC->CR, RCC_CR_HSEON);
            }
            else
            {
                CLEAR_BIT(RCC->CR, RCC_CR_HSEON);
                CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP);
            }
        }
        /** @}
         * RCC_HSE_Configuration
         */

        /** @defgroup RCC_LSE_Configuration LSE Configuration
         * @{
         */

        /**
         * @brief  Macro to configure the External Low Speed oscillator (LSE).
         * @note   Transition LSE Bypass to LSE On and LSE On to LSE Bypass are not supported by this macro.
         *         User should request a transition to LSE Off first and then LSE On or LSE Bypass.
         * @note   As the LSE is in the Backup domain and write access is denied to
         *         this domain after reset, you have to enable write access using
         *         HAL_PWR_EnableBkUpAccess() function before to configure the LSE
         *         (to be done once after reset).
         * @note   After enabling the LSE (RCC_LSE_ON or RCC_LSE_BYPASS), the application
         *         software should wait on LSERDY flag to be set indicating that LSE clock
         *         is stable and can be used to clock the RTC.
         * @param  __STATE__ specifies the new state of the LSE.
         *         This parameter can be one of the following values:
         *            @arg RCC_LSE_OFF: turn OFF the LSE oscillator, LSERDY flag goes low after
         *                              6 LSE oscillator clock cycles.
         *            @arg RCC_LSE_ON: turn ON the LSE oscillator.
         *            @arg RCC_LSE_BYPASS: LSE oscillator bypassed with external clock.
         */
        constexpr void ConfigLse(LseConfig state)
        {
            if (state == LseConfig::On)
            {
                SET_BIT(RCC->BDCR, RCC_BDCR_LSEON);
            }
            else if (state == LseConfig::Bypass)
            {
                SET_BIT(RCC->BDCR, RCC_BDCR_LSEBYP);
                SET_BIT(RCC->BDCR, RCC_BDCR_LSEON);
            }
            else
            {
                CLEAR_BIT(RCC->BDCR, RCC_BDCR_LSEON);
                CLEAR_BIT(RCC->BDCR, RCC_BDCR_LSEBYP);
            }
        }
        /** @}
         * RCC_LSE_Configuration
         */

        /** @defgroup RCC_Internal_RTC_Clock_Configuration RTC Clock Configuration
         * @{
         */

        /** @brief  Macros to enable or disable the RTC clock.
         * @note   These macros must be used only after the RTC clock source was selected.
         */
        constexpr void RtcEnable() { *(__IO uint32_t *)RCC_BDCR_RTCEN_BB = ~0U; }
        constexpr void RtcDisable() { *(__IO uint32_t *)RCC_BDCR_RTCEN_BB = 0; }

        /** @brief  Macros to configure the RTC clock (RTCCLK).
         * @note   As the RTC clock configuration bits are in the Backup domain and write
         *         access is denied to this domain after reset, you have to enable write
         *         access using the Power Backup Access macro before to configure
         *         the RTC clock source (to be done once after reset).
         * @note   Once the RTC clock is configured it can't be changed unless the
         *         Backup domain is reset using __HAL_RCC_BackupReset_RELEASE() macro, or by
         *         a Power On Reset (POR).
         * @param  __RTCCLKSource__ specifies the RTC clock source.
         *         This parameter can be one of the following values:
                     @arg @ref RCC_RTCCLKSOURCE_NO_CLK: No clock selected as RTC clock.
        *            @arg @ref RCC_RTCCLKSOURCE_LSE: LSE selected as RTC clock.
        *            @arg @ref RCC_RTCCLKSOURCE_LSI: LSI selected as RTC clock.
        *            @arg @ref RCC_RTCCLKSOURCE_HSE_DIVX: HSE clock divided by x selected
        *                                                 as RTC clock, where x:[2,31]
        * @note   If the LSE or LSI is used as RTC clock source, the RTC continues to
        *         work in STOP and STANDBY modes, and can be used as wake-up source.
        *         However, when the HSE clock is used as RTC clock source, the RTC
        *         cannot be used in STOP and STANDBY modes.
        * @note   The maximum input clock frequency for RTC is 1MHz (when using HSE as
        *         RTC clock source).
        */
        constexpr void SetRtcClkPrescaler(RtcClockSource clkSrc)
        {
            (((uint32_t)clkSrc & RCC_BDCR_RTCSEL) == RCC_BDCR_RTCSEL) ? MODIFY_REG(RCC->CFGR, RCC_CFGR_RTCPRE, (((uint32_t)(clkSrc)) & 0xFFFFCFFU))
                                                                      : CLEAR_BIT(RCC->CFGR, RCC_CFGR_RTCPRE);
        }
        constexpr void ConfigRtc(RtcClockSource clkSrc)
        {
            SetRtcClkPrescaler(clkSrc);
            RCC->BDCR |= ((uint32_t)clkSrc & 0x00000FFFU);
        }

        /** @brief Macro to get the RTC clock source.
         * @retval The clock source can be one of the following values:
         *            @arg @ref RCC_RTCCLKSOURCE_NO_CLK No clock selected as RTC clock
         *            @arg @ref RCC_RTCCLKSOURCE_LSE LSE selected as RTC clock
         *            @arg @ref RCC_RTCCLKSOURCE_LSI LSI selected as RTC clock
         *            @arg @ref RCC_RTCCLKSOURCE_HSE_DIVX HSE divided by X selected as RTC clock (X can be retrieved thanks to @ref __HAL_RCC_GET_RTC_HSE_PRESCALER()
         */
        constexpr RtcClockSource GetRtcClkSource() { return (RtcClockSource)READ_BIT(RCC->BDCR, RCC_BDCR_RTCSEL); }

        /**
         * @brief   Get the RTC and HSE clock divider (RTCPRE).
         * @retval Returned value can be one of the following values:
         *            @arg @ref RCC_RTCCLKSOURCE_HSE_DIVX: HSE clock divided by x selected
         *                                                 as RTC clock, where x:[2,31]
         */
        constexpr uint32_t GetRtcHsePrescaler() { return READ_BIT(RCC->CFGR, RCC_CFGR_RTCPRE) | RCC_BDCR_RTCSEL; }

        /** @brief  Macros to force or release the Backup domain reset.
         * @note   This function resets the RTC peripheral (including the backup registers)
         *         and the RTC clock source selection in RCC_CSR register.
         * @note   The BKPSRAM is not affected by this reset.
         */
        constexpr void BackupResetForce() { (*(__IO uint32_t *)RCC_BDCR_BDRST_BB = ~0U); }
        constexpr void BackupResetRelease() { (*(__IO uint32_t *)RCC_BDCR_BDRST_BB = 0U); }
        /** @}
         * RCC_Internal_RTC_Clock_Configuration
         */

        /** @defgroup RCC_PLL_Configuration PLL Configuration
         * @{
         */

        /** @brief  Macros to enable or disable the main PLL.
         * @note   After enabling the main PLL, the application software should wait on
         *         PLLRDY flag to be set indicating that PLL clock is stable and can
         *         be used as system clock source.
         * @note   The main PLL can not be disabled if it is used as system clock source
         * @note   The main PLL is disabled by hardware when entering STOP and STANDBY modes.
         */
        constexpr void PllEnable() { (*(__IO uint32_t *)RCC_CR_PLLON_BB = ~0U); }
        constexpr void PllDisable() { (*(__IO uint32_t *)RCC_CR_PLLON_BB = 0U); }

        /** @brief  Macro to configure the PLL clock source.
         * @note   This function must be used only when the main PLL is disabled.
         * @param  __PLLSOURCE__ specifies the PLL entry clock source.
         *         This parameter can be one of the following values:
         *            @arg RCC_PLLSOURCE_HSI: HSI oscillator clock selected as PLL clock entry
         *            @arg RCC_PLLSOURCE_HSE: HSE oscillator clock selected as PLL clock entry
         *
         */
        constexpr void ConfigPllSource(PllClockSource src) { MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC, (uint32_t)src); }

        /** @brief  Macro to configure the PLL multiplication factor.
         * @note   This function must be used only when the main PLL is disabled.
         * @param  __PLLM__ specifies the division factor for PLL VCO input clock
         *         This parameter must be a number between Min_Data = 2 and Max_Data = 63.
         * @note   You have to set the PLLM parameter correctly to ensure that the VCO input
         *         frequency ranges from 1 to 2 MHz. It is recommended to select a frequency
         *         of 2 MHz to limit PLL jitter.
         *
         */
        constexpr void ConfigPllM(uint32_t m) { MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLM, m); }
        /**
         * @}
         * RCC_PLL_Configuration
         */

        /** @defgroup RCC_Get_Clock_source Get Clock source
         * @{
         */
        /**
         * @brief Macro to configure the system clock source.
         * @param __RCC_SYSCLKSOURCE__ specifies the system clock source.
         * This parameter can be one of the following values:
         *              - RCC_SYSCLKSOURCE_HSI: HSI oscillator is used as system clock source.
         *              - RCC_SYSCLKSOURCE_HSE: HSE oscillator is used as system clock source.
         *              - RCC_SYSCLKSOURCE_PLLCLK: PLL output is used as system clock source.
         *              - RCC_SYSCLKSOURCE_PLLRCLK: PLLR output is used as system clock source. This
         *                parameter is available only for STM32F446xx devices.
         */
        constexpr void ConfigSysclk(SystemClkSource src) { MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, (uint32_t)src); }

        /** @brief  Macro to get the clock source used as system clock.
         * @retval The clock source used as system clock. The returned value can be one
         *         of the following:
         *              - RCC_SYSCLKSOURCE_STATUS_HSI: HSI used as system clock.
         *              - RCC_SYSCLKSOURCE_STATUS_HSE: HSE used as system clock.
         *              - RCC_SYSCLKSOURCE_STATUS_PLLCLK: PLL used as system clock.
         *              - RCC_SYSCLKSOURCE_STATUS_PLLRCLK: PLLR used as system clock. This parameter
         *                is available only for STM32F446xx devices.
         */
        constexpr SystemClkSource GetSysclkSrc() { return (SystemClkSource)(RCC->CFGR & RCC_CFGR_SWS); }

        /** @brief  Macro to get the oscillator used as PLL clock source.
         * @retval The oscillator used as PLL clock source. The returned value can be one
         *         of the following:
         *              - RCC_PLLSOURCE_HSI: HSI oscillator is used as PLL clock source.
         *              - RCC_PLLSOURCE_HSE: HSE oscillator is used as PLL clock source.
         */
        constexpr PllClockSource GetPllClkSrc() { return (PllClockSource)(RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC); }
        /** @}
         * RCC_Get_Clock_source
         */

        /** @defgroup RCCEx_MCOx_Clock_Config RCC Extended MCOx Clock Config
         * @{
         */

        /** @brief  Macro to configure the MCO1 clock.
         * @param  __MCOCLKSOURCE__ specifies the MCO clock source.
         *          This parameter can be one of the following values:
         *            @arg RCC_MCO1SOURCE_HSI: HSI clock selected as MCO1 source
         *            @arg RCC_MCO1SOURCE_LSE: LSE clock selected as MCO1 source
         *            @arg RCC_MCO1SOURCE_HSE: HSE clock selected as MCO1 source
         *            @arg RCC_MCO1SOURCE_PLLCLK: main PLL clock selected as MCO1 source
         * @param  __MCODIV__ specifies the MCO clock prescaler.
         *          This parameter can be one of the following values:
         *            @arg RCC_MCODIV_1: no division applied to MCOx clock
         *            @arg RCC_MCODIV_2: division by 2 applied to MCOx clock
         *            @arg RCC_MCODIV_3: division by 3 applied to MCOx clock
         *            @arg RCC_MCODIV_4: division by 4 applied to MCOx clock
         *            @arg RCC_MCODIV_5: division by 5 applied to MCOx clock
         */
        constexpr void ConfigMco1(Mco1ClockSource src, McoXClockPrescaller div)
        {
            MODIFY_REG(RCC->CFGR, (RCC_CFGR_MCO1 | RCC_CFGR_MCO1PRE), ((uint32_t)src | (uint32_t)div));
        }

        /** @brief  Macro to configure the MCO2 clock.
         * @param  __MCOCLKSOURCE__ specifies the MCO clock source.
         *          This parameter can be one of the following values:
         *            @arg RCC_MCO2SOURCE_SYSCLK: System clock (SYSCLK) selected as MCO2 source
         *            @arg RCC_MCO2SOURCE_PLLI2SCLK: PLLI2S clock selected as MCO2 source, available for all STM32F4 devices except STM32F410xx
         *            @arg RCC_MCO2SOURCE_I2SCLK: I2SCLK clock selected as MCO2 source, available only for STM32F410Rx devices
         *            @arg RCC_MCO2SOURCE_HSE: HSE clock selected as MCO2 source
         *            @arg RCC_MCO2SOURCE_PLLCLK: main PLL clock selected as MCO2 source
         * @param  __MCODIV__ specifies the MCO clock prescaler.
         *          This parameter can be one of the following values:
         *            @arg RCC_MCODIV_1: no division applied to MCOx clock
         *            @arg RCC_MCODIV_2: division by 2 applied to MCOx clock
         *            @arg RCC_MCODIV_3: division by 3 applied to MCOx clock
         *            @arg RCC_MCODIV_4: division by 4 applied to MCOx clock
         *            @arg RCC_MCODIV_5: division by 5 applied to MCOx clock
         * @note  For STM32F410Rx devices, to output I2SCLK clock on MCO2, you should have
         *        at least one of the SPI clocks enabled (SPI1, SPI2 or SPI5).
         */
        constexpr void ConfigMco2(Mco2ClockSource src, McoXClockPrescaller div)
        {
            MODIFY_REG(RCC->CFGR, (RCC_CFGR_MCO2 | RCC_CFGR_MCO2PRE), ((uint32_t)src | ((uint32_t)div << 3))));
        }
        /** @}
         * RCCEx_MCOx_Clock_Config
         */

        /** @defgroup RCC_Flags_Interrupts_Management Flags Interrupts Management
         * @brief macros to manage the specified RCC Flags and interrupts.
         * @{
         */

        /** @brief  Enable RCC interrupt (Perform Byte access to RCC_CIR[14:8] bits to enable
         *         the selected interrupts).
         * @param  __INTERRUPT__ specifies the RCC interrupt sources to be enabled.
         *         This parameter can be any combination of the following values:
         *            @arg RCC_IT_LSIRDY: LSI ready interrupt.
         *            @arg RCC_IT_LSERDY: LSE ready interrupt.
         *            @arg RCC_IT_HSIRDY: HSI ready interrupt.
         *            @arg RCC_IT_HSERDY: HSE ready interrupt.
         *            @arg RCC_IT_PLLRDY: Main PLL ready interrupt.
         *            @arg RCC_IT_PLLI2SRDY: PLLI2S ready interrupt.
         */
        constexpr void EnableIt(Interrupt it) { (*(__IO uint8_t *)RCC_CIR_BYTE1_ADDRESS |= (uint8_t)it); }

        /** @brief Disable RCC interrupt (Perform Byte access to RCC_CIR[14:8] bits to disable
         *        the selected interrupts).
         * @param  __INTERRUPT__ specifies the RCC interrupt sources to be disabled.
         *         This parameter can be any combination of the following values:
         *            @arg RCC_IT_LSIRDY: LSI ready interrupt.
         *            @arg RCC_IT_LSERDY: LSE ready interrupt.
         *            @arg RCC_IT_HSIRDY: HSI ready interrupt.
         *            @arg RCC_IT_HSERDY: HSE ready interrupt.
         *            @arg RCC_IT_PLLRDY: Main PLL ready interrupt.
         *            @arg RCC_IT_PLLI2SRDY: PLLI2S ready interrupt.
         */
        constexpr void DisableIt(Interrupt it) { (*(__IO uint8_t *)RCC_CIR_BYTE1_ADDRESS &= ~((uint8_t)it)); }

        /** @brief  Clear the RCC's interrupt pending bits (Perform Byte access to RCC_CIR[23:16]
         *         bits to clear the selected interrupt pending bits.
         * @param  __INTERRUPT__ specifies the interrupt pending bit to clear.
         *         This parameter can be any combination of the following values:
         *            @arg RCC_IT_LSIRDY: LSI ready interrupt.
         *            @arg RCC_IT_LSERDY: LSE ready interrupt.
         *            @arg RCC_IT_HSIRDY: HSI ready interrupt.
         *            @arg RCC_IT_HSERDY: HSE ready interrupt.
         *            @arg RCC_IT_PLLRDY: Main PLL ready interrupt.
         *            @arg RCC_IT_PLLI2SRDY: PLLI2S ready interrupt.
         *            @arg RCC_IT_CSS: Clock Security System interrupt
         */
        constexpr void ClearIt(Interrupt it) { (*(__IO uint8_t *)RCC_CIR_BYTE2_ADDRESS = (uint8_t)it); }

        /** @brief  Check the RCC's interrupt has occurred or not.
         * @param  __INTERRUPT__ specifies the RCC interrupt source to check.
         *         This parameter can be one of the following values:
         *            @arg RCC_IT_LSIRDY: LSI ready interrupt.
         *            @arg RCC_IT_LSERDY: LSE ready interrupt.
         *            @arg RCC_IT_HSIRDY: HSI ready interrupt.
         *            @arg RCC_IT_HSERDY: HSE ready interrupt.
         *            @arg RCC_IT_PLLRDY: Main PLL ready interrupt.
         *            @arg RCC_IT_PLLI2SRDY: PLLI2S ready interrupt.
         *            @arg RCC_IT_CSS: Clock Security System interrupt
         * @retval The new state of __INTERRUPT__ (TRUE or FALSE).
         */
        constexpr bool GetIt(Interrupt it) { return (RCC->CIR & (uint32_t)it) == (uint32_t)it; }

        /** @brief Set RMVF bit to clear the reset flags: RCC_FLAG_PINRST, RCC_FLAG_PORRST,
         *        RCC_FLAG_SFTRST, RCC_FLAG_IWDGRST, RCC_FLAG_WWDGRST and RCC_FLAG_LPWRRST.
         */
        constexpr void ResetFlags() { RCC->CSR |= RCC_CSR_RMVFl }

        /** @brief  Check RCC flag is set or not.
         * @param  __FLAG__ specifies the flag to check.
         *         This parameter can be one of the following values:
         *            @arg RCC_FLAG_HSIRDY: HSI oscillator clock ready.
         *            @arg RCC_FLAG_HSERDY: HSE oscillator clock ready.
         *            @arg RCC_FLAG_PLLRDY: Main PLL clock ready.
         *            @arg RCC_FLAG_PLLI2SRDY: PLLI2S clock ready.
         *            @arg RCC_FLAG_LSERDY: LSE oscillator clock ready.
         *            @arg RCC_FLAG_LSIRDY: LSI oscillator clock ready.
         *            @arg RCC_FLAG_BORRST: POR/PDR or BOR reset.
         *            @arg RCC_FLAG_PINRST: Pin reset.
         *            @arg RCC_FLAG_PORRST: POR/PDR reset.
         *            @arg RCC_FLAG_SFTRST: Software reset.
         *            @arg RCC_FLAG_IWDGRST: Independent Watchdog reset.
         *            @arg RCC_FLAG_WWDGRST: Window Watchdog reset.
         *            @arg RCC_FLAG_LPWRRST: Low Power reset.
         * @retval The new state of __FLAG__ (TRUE or FALSE).
         */
        constexpr uint8_t FlagMask = 0x1FU;
        constexpr bool GetFlag(Flags flag) { return ((((((((uint8_t)flag) >> 5U) == 1U) ? RCC->CR : (((((uint8_t)flag) >> 5U) == 2U) ? RCC->BDCR : (((((uint8_t)flag) >> 5U) == 3U) ? RCC->CSR : RCC->CIR))) & (1U << (((uint8_t)flag) & RCC_FLAG_MASK))) != 0U) ? 1U : 0U); }
        /** @}
         * RCC_Flags_Interrupts_Management
         */

        /** @}
         * ExportedMacros
         */

        /** @defgroup ExportedTypes RCC Exported Types
         * @{
         */

        /**
         * @brief RCC PLL configuration structure definition.
         * 
         */
        struct PllInit
        {
            /**
             * @brief The new state of the PLL.
             * 
             */
            PllConfig pllState;
            /**
             * @brief PLL Entry clock source.
             * 
             */
            PllClockSource pllSource;
            /**
             * @brief   Division factor for PLL VCO input clock.
             *          This parameter must be a number between 0 and 63.
             * 
             */
            uint32_t pllM;
            /**
             * @brief   Multiplication factor for PLL VCO output clock.
             *          This parameter must be a number between 50 and 432,
             *          except for STM32F411xE devices where the minimum is 192.
             * 
             */
            uint32_t pllN;
            /**
             * @brief   Division factor for main system clock (SYSCLK).
             * 
             */
            PllPClockDivider pllP;
            /**
             * @brief   Division factor for OTG FS, SDIO and RNG clocks.
             *          This parameter must be a number between 2 and 15.
             * 
             */
            uint32_t pllQ;
        };

        struct OscInit
        {
            uint32_t oscillatorType;
            uint32_t hseState;
            uint32_t lseState;
            uint32_t hsiState;
            uint32_t hsiCalivrationValue;
            uint32_t lsiState;
            PllInit pll;
        };

        struct ClkInit
        {
            SystemClockType clkType;
            SystemClkSource sysclkSrc;
            SysclkDivider ahbClkDivider;
            HclkDivider apb1ClkDivider;
            HclkDivider apb2ClkDivider;
        }

        /** @} */

        /*****************************************************************************/
        /* Exported Functions                                                        */
        /*****************************************************************************/
        /** @addtogroup RCC_Exported_Functions
        * @{
        */
        class RstCtrlClk
        {
        public:
            RstCtrlClk() = default;
            ~RstCtrlClk() = default;

            Status DeInit();
            Status OscConfig(const OscInit &config);
            Status ClockConfig(const ClkInit &config, uint32_t fLatency);

            void ConfigMco(McoIndex mco, uint32_t src, McoXClockPrescaller div);
            void EnableCss();
            void DisableCss();

            uint32_t GetSysClockFreq() const { return m_systemCoreClock; }
            uint32_t GetHclkFreq() const;
            uint32_t GetPclk1Freq() const;
            uint32_t GetPclk2Freq() const;
            const OscInit &GetOscConfig() const { return m_oscConfig; }
            const ClkInit &GetClockConfig() const { return m_clkConfig; };

            // CSS NMI IRQ Handler.
            void NmiIrqHandler();

            // User Callbacks in non blocking mode (IT mode).
            void CssCallback();

        private:
            OscInit m_oscConfig;
            ClkInit m_clkConfig;
            uint32_t m_systemCoreClock = 0;

            constexpr uint32_t OFFSET = (RCC_BASE - PERIPH_BASE);
            // --- CR Register ---
            // Alias word address of HSION bit.
            constexpr uint32_t CR_OFFSET = (OFFSET + 0x00U);
            constexpr uint32_t HSION_BIT_NUMBER = 0x00U;
            constexpr uint32_t CR_HSION_BB = (PERIPH_BB_BASE + (CR_OFFSET * 32U) + (HSION_BIT_NUMBER * 4U));
            // Alias word address of CSSON bit.
            constexpr uint32_t CSSON_BIT_NUMBER = 0x13U;
            constexpr uint32_t CR_CSSON_BB = (PERIPH_BB_BASE + (CR_OFFSET * 32U) + (CSSON_BIT_NUMBER * 4U));
            // Alias word address of PLLON bit.
            constexpr uint32_t PLLON_BIT_NUMBER = 0x18U;
            constexpr uint32_t CR_PLLON_BB = (PERIPH_BB_BASE + (CR_OFFSET * 32U) + (PLLON_BIT_NUMBER * 4U));

            // --- BDCR Register ---
            // Alias word address for RTCEN bit.
            constexpr uint32_t BDCR_OFFSET = OFFSET + 0x70U;
            constexpr uint32_t RTCEN_BIT_NUMBER = 0x0FU;
            constexpr uint32_t BDCR_RTCEN_BB = (PERIPH_BB_BASE + (BDCR_OFFSET * 32U) + (RTCEN_BIT_NUMBER * 4U));
            // Alias word address of BDRST bit.
            constexpr uint32_t BDRST_BIT_NUMBER = 0x10U;
            constexpr uint32_t BDCR_BDRST_BB = (PERIPH_BB_BASE + (BDCR_OFFSET * 32U) + (BDRST_BIT_NUMBER * 4U));

            // --- CSR Register ---
            // Alias word address of LSION bit.
            constexpr uint32_t CSR_OFFSET = OFFSET + 0x74U;
            constexpr uint32_t LSION_BIT_NUMBER = 0x00U;
            constexpr uint32_t CSR_LSION_BB = (PERIPH_BB_BASE + (CSR_OFFSET * 32U) + (LSION_BIT_NUMBER * 4U));

            // CR register byte 3 (bits[23:16]) base address.
            constexpr uint32_t CR_BYTE2_ADDRESS = 0x40023802U;

            // CIR register byte 2 (bits[15:8]) base address.
            constexpr uint32_t CIR_BYTE1_ADDRESS = ((uint32_t)(RCC_BASE + 0x0CU + 0x01U));

            // CIR register byte 3 (bits[23:16]) base address.
            constexpr uint32_t CIR_BYTE2_ADDRESS = ((uint32_t)(RCC_BASE + 0x0CU + 0x02U));

            // BDCR register base address.
            constexpr uint32_t BDCR_BYTE0_ADDRESS = (PERIPH_BASE + BDCR_OFFSET);

            constexpr uint32_t DBP_TIMEOUT_VALUE = 2U;
            constexpr uint32_t LSE_TIMEOUT_VALUE = LSE_STARTUP_TIMEOUT;
            constexpr uint32_t HSE_TIMEOUT_VALUE = HSE_STARTUP_TIMEOUT;
            constexpr uint32_t HSI_TIMEOUT_VALUE = 2U;            // 2 ms.
            constexpr uint32_t LSI_TIMEOUT_VALUE = 2U;            // 2 ms.
            constexpr uint32_t CLOCKSWITCH_TIMEOUT_VALUE = 5000U; // 5 s.

            /*****************************************************************/
            /* Private Macros                                                */
            /*****************************************************************/
            /** @defgroup RCC_Private_Macros RCC Private Macros
             * @{
             */

            /** @defgroup RCC_IS_RCC_Definitions RCC Private macros to check input parameters
             * @{
             */
            constexpr bool IS_RCC_OSCILLATOR_TYPE(auto osc) { return (osc <= 15U); }
            constexpr bool IS_RCC_HSE(HseConfig cfg)
            {
                return (((cfg) == HseConfig::Off) || ((cfg) == HseConfig::On) ||
                        ((cfg) == HseConfig::Bypass));
            }
            constexpr bool IS_RCC_LSE(LseConfig cfg)
            {
                return ((cfg == LseConfig::On) || (cfg == LseConfig::Bypass) || (cfg == LseConfig::Off));
            }

            constexpr bool IS_RCC_HSI(HsiConfig cfg)
            {
                return ((cfg == HsiConfig::On) || (cfg == HsiConfig::Off));
            }

            constexpr bool IS_RCC_LSI(LsiConfig cfg)
            {
                return ((cfg == LsiConfig::On) || (cfg == LsiConfig::Off));
            }

            constexpr bool IS_RCC_PLL(PllConfig cfg)
            {
                return ((cfg == PllConfig::None) || (cfg == PllConfig::Off) || (cfg == PllConfig::On));
            }

            constexpr bool IS_RCC_PLLSOURCE(PllClockSource cfg)
            {
                return ((cfg == PllClockSource::HSI) || (cfg == PllClockSource::HSE) || (cfg == PllClockSource::PLLCLK) || (cfg == PllClockSource::PLLRCLK));
            }

            constexpr bool IS_RCC_RTCCLKSOURCE(RtcClockSource cfg)
            {
                return ((cfg == RtcClockSource::LSE) || (cfg == RtcClockSource::LSI) || (cfg == RtcClockSource::HSE_DIVX) || (cfg == RtcClockSource::HSE_DIV2) || (cfg == RtcClockSource::HSE_DIV3) || (cfg == RtcClockSource::HSE_DIV4) || (cfg == RtcClockSource::HSE_DIV5) || (cfg == RtcClockSource::HSE_DIV6) || (cfg == RtcClockSource::HSE_DIV7) || (cfg == RtcClockSource::HSE_DIV8) || (cfg == RtcClockSource::HSE_DIV9) || (cfg == RtcClockSource::HSE_DIV10) || (cfg == RtcClockSource::HSE_DIV11) || (cfg == RtcClockSource::HSE_DIV12) || (cfg == RtcClockSource::HSE_DIV13) || (cfg == RtcClockSource::HSE_DIV14) || (cfg == RtcClockSource::HSE_DIV15) || (cfg == RtcClockSource::HSE_DIV16) || (cfg == RtcClockSource::HSE_DIV17) || (cfg == RtcClockSource::HSE_DIV18) || (cfg == RtcClockSource::HSE_DIV19) || (cfg == RtcClockSource::HSE_DIV20) || (cfg == RtcClockSource::HSE_DIV21) || (cfg == RtcClockSource::HSE_DIV22) || (cfg == RtcClockSource::HSE_DIV23) || (cfg == RtcClockSource::HSE_DIV24) || (cfg == RtcClockSource::HSE_DIV25) || (cfg == RtcClockSource::HSE_DIV26) || (cfg == RtcClockSource::HSE_DIV27) || (cfg == RtcClockSource::HSE_DIV28) || (cfg == RtcClockSource::HSE_DIV29) || (cfg == RtcClockSource::HSE_DIV30) || (cfg == RtcClockSource::HSE_DIV31));
            }

            constexpr bool IS_RCC_PLLM_VALUE(uint32_t val) { return val <= 63U; }

            constexpr bool IS_RCC_PLLP_VALUE(uint32_t val) { return ((val == 2U) || (val == 4U) || (val == 6U) || (val == 8U)); }

            constexpr bool IS_RCC_PLLQ_VALUE(uint32_t val) { return ((2U <= val) && (val <= 15U)); }

            constexpr bool IS_RCC_HCLK(SysclkDivider cfg)
            {
                return ((cfg == SysclkDivider::Div1) || (cfg == SysclkDivider::Div2) || (cfg == SysclkDivider::Div4) || (cfg == SysclkDivider::Div8) || (cfg == SysclkDivider::Div16) || (cfg == SysclkDivider::Div64) || (cfg == SysclkDivider::Div128) || (cfg == SysclkDivider::Div256) || (cfg == SysclkDivider::Div512));
            }

            constexpr bool IS_RCC_CLOCKTYPE(uint32_t cfg) { return ((1U <= cfg) && (cfg <= 15U)); }

            constexpr bool IS_RCC_PCLK(HclkDivider cfg)
            {
                return ((cfg == HclkDivider::Div1) || (cfg == HclkDivider::Div2) || (cfg == HclkDivider::Div4) || (cfg == HclkDivider::Div8) || (cfg == HclkDivider::Div16));
            }

            constexpr bool IS_RCC_MCO(McoIndex mco) { return ((mco == McoIndex::MCO1) || (mco == McoIndex::MCO2)); }

            constexpr bool IS_RCC_MCO1SOURCE(Mco1ClockSource src)
            {
                return ((src == Mco1ClockSource::HSI) || (src == Mco1ClockSource::LSE) || (src == Mco1ClockSource::HSE) || (src == Mco1ClockSource::PLLCLK));
            }

            constexpr bool IS_RCC_MCODIV(McoXClockPrescaller cfg)
            {
                return ((cfg == McoXClockPrescaller::Div1) || (cfg == McoXClockPrescaller::Div2) || (cfg == McoXClockPrescaller::Div3) || (cfg == McoXClockPrescaller::Div4) || (cfg == McoXClockPrescaller::Div5));
            }

            constexpr bool IS_RCC_CALIBRATION_VALUE(uint32_t val) { return (val <= 0x1FU); }
        }
        /** @}
         * RCC_Exported_Functions
         */
    } // namespace RstCtrlClk
    /** @} */
} // namespace HAL
/** @} */
#endif