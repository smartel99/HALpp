/**
 * @file        stm32f4xx_core.h
 * @author      Samuel Martel
 * @p           https://www.github.com/smartel99
 * @date        2020/09/06
 * @brief       Header containing the declaration of the STM32F405ZGTx class.
 */
/*************************************************************************************************/
#ifndef __stm32f4xx_core_H
#define __stm32f4xx_core_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_conf.h"

#include "stm32f4xx_def.h"

#include <cstdint>

/** @addtogroup STM32F4xx_HAL_Driver
 * @{
 */
namespace HAL
{
    /** @addtogroup HAL
     * @{
     */

    /* Exported types ------------------------------------------------------------*/
    /* Exported constants --------------------------------------------------------*/

    /** @defgroup HAL_Exported_Constants HAL Exported Constants
     * @{
     */

    /** @defgroup HAL_TICK_FREQ Tick Frequency
     * @{
     */
    enum class TickFreq
    {
        Hz10 = 100U,
        Hz100 = 10U,
        Hz1k = 1U,
        Default = Hz1k
    };
    /** @}
     * HAL_TICK_FREQ
     */

    /** @}
     * HAL_Exported_Constants
     */

    /* Exported macro ------------------------------------------------------------*/
    /** @defgroup HAL_Exported_Macros HAL Exported Macros
     * @{
     */

    /** @brief  Freeze/Unfreeze Peripherals in Debug mode 
     */
    constexpr void DBGMCU_FREEZE_TIM2() { (DBGMCU->APB1FZ |= (DBGMCU_APB1_FZ_DBG_TIM2_STOP)); }
    constexpr void DBGMCU_FREEZE_TIM3() { (DBGMCU->APB1FZ |= (DBGMCU_APB1_FZ_DBG_TIM3_STOP)); }
    constexpr void DBGMCU_FREEZE_TIM4() { (DBGMCU->APB1FZ |= (DBGMCU_APB1_FZ_DBG_TIM4_STOP)); }
    constexpr void DBGMCU_FREEZE_TIM5() { (DBGMCU->APB1FZ |= (DBGMCU_APB1_FZ_DBG_TIM5_STOP)); }
    constexpr void DBGMCU_FREEZE_TIM6() { (DBGMCU->APB1FZ |= (DBGMCU_APB1_FZ_DBG_TIM6_STOP)); }
    constexpr void DBGMCU_FREEZE_TIM7() { (DBGMCU->APB1FZ |= (DBGMCU_APB1_FZ_DBG_TIM7_STOP)); }
    constexpr void DBGMCU_FREEZE_TIM12() { (DBGMCU->APB1FZ |= (DBGMCU_APB1_FZ_DBG_TIM12_STOP)); }
    constexpr void DBGMCU_FREEZE_TIM13() { (DBGMCU->APB1FZ |= (DBGMCU_APB1_FZ_DBG_TIM13_STOP)); }
    constexpr void DBGMCU_FREEZE_TIM14() { (DBGMCU->APB1FZ |= (DBGMCU_APB1_FZ_DBG_TIM14_STOP)); }
    constexpr void DBGMCU_FREEZE_RTC() { (DBGMCU->APB1FZ |= (DBGMCU_APB1_FZ_DBG_RTC_STOP)); }
    constexpr void DBGMCU_FREEZE_WWDG() { (DBGMCU->APB1FZ |= (DBGMCU_APB1_FZ_DBG_WWDG_STOP)); }
    constexpr void DBGMCU_FREEZE_IWDG() { (DBGMCU->APB1FZ |= (DBGMCU_APB1_FZ_DBG_IWDG_STOP)); }
    constexpr void DBGMCU_FREEZE_I2C1_TIMEOUT() { (DBGMCU->APB1FZ |= (DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT)); }
    constexpr void DBGMCU_FREEZE_I2C2_TIMEOUT() { (DBGMCU->APB1FZ |= (DBGMCU_APB1_FZ_DBG_I2C2_SMBUS_TIMEOUT)); }
    constexpr void DBGMCU_FREEZE_I2C3_TIMEOUT() { (DBGMCU->APB1FZ |= (DBGMCU_APB1_FZ_DBG_I2C3_SMBUS_TIMEOUT)); }
    constexpr void DBGMCU_FREEZE_CAN1() { (DBGMCU->APB1FZ |= (DBGMCU_APB1_FZ_DBG_CAN1_STOP)); }
    constexpr void DBGMCU_FREEZE_CAN2() { (DBGMCU->APB1FZ |= (DBGMCU_APB1_FZ_DBG_CAN2_STOP)); }
    constexpr void DBGMCU_FREEZE_TIM1() { (DBGMCU->APB2FZ |= (DBGMCU_APB2_FZ_DBG_TIM1_STOP)); }
    constexpr void DBGMCU_FREEZE_TIM8() { (DBGMCU->APB2FZ |= (DBGMCU_APB2_FZ_DBG_TIM8_STOP)); }
    constexpr void DBGMCU_FREEZE_TIM9() { (DBGMCU->APB2FZ |= (DBGMCU_APB2_FZ_DBG_TIM9_STOP)); }
    constexpr void DBGMCU_FREEZE_TIM10() { (DBGMCU->APB2FZ |= (DBGMCU_APB2_FZ_DBG_TIM10_STOP)); }
    constexpr void DBGMCU_FREEZE_TIM11() { (DBGMCU->APB2FZ |= (DBGMCU_APB2_FZ_DBG_TIM11_STOP)); }

    constexpr void DBGMCU_UNFREEZE_TIM2() { (DBGMCU->APB1FZ &= ~(DBGMCU_APB1_FZ_DBG_TIM2_STOP)); }
    constexpr void DBGMCU_UNFREEZE_TIM3() { (DBGMCU->APB1FZ &= ~(DBGMCU_APB1_FZ_DBG_TIM3_STOP)); }
    constexpr void DBGMCU_UNFREEZE_TIM4() { (DBGMCU->APB1FZ &= ~(DBGMCU_APB1_FZ_DBG_TIM4_STOP)); }
    constexpr void DBGMCU_UNFREEZE_TIM5() { (DBGMCU->APB1FZ &= ~(DBGMCU_APB1_FZ_DBG_TIM5_STOP)); }
    constexpr void DBGMCU_UNFREEZE_TIM6() { (DBGMCU->APB1FZ &= ~(DBGMCU_APB1_FZ_DBG_TIM6_STOP)); }
    constexpr void DBGMCU_UNFREEZE_TIM7() { (DBGMCU->APB1FZ &= ~(DBGMCU_APB1_FZ_DBG_TIM7_STOP)); }
    constexpr void DBGMCU_UNFREEZE_TIM12() { (DBGMCU->APB1FZ &= ~(DBGMCU_APB1_FZ_DBG_TIM12_STOP)); }
    constexpr void DBGMCU_UNFREEZE_TIM13() { (DBGMCU->APB1FZ &= ~(DBGMCU_APB1_FZ_DBG_TIM13_STOP)); }
    constexpr void DBGMCU_UNFREEZE_TIM14() { (DBGMCU->APB1FZ &= ~(DBGMCU_APB1_FZ_DBG_TIM14_STOP)); }
    constexpr void DBGMCU_UNFREEZE_RTC() { (DBGMCU->APB1FZ &= ~(DBGMCU_APB1_FZ_DBG_RTC_STOP)); }
    constexpr void DBGMCU_UNFREEZE_WWDG() { (DBGMCU->APB1FZ &= ~(DBGMCU_APB1_FZ_DBG_WWDG_STOP)); }
    constexpr void DBGMCU_UNFREEZE_IWDG() { (DBGMCU->APB1FZ &= ~(DBGMCU_APB1_FZ_DBG_IWDG_STOP)); }
    constexpr void DBGMCU_UNFREEZE_I2C1_TIMEOUT() { (DBGMCU->APB1FZ &= ~(DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT)); }
    constexpr void DBGMCU_UNFREEZE_I2C2_TIMEOUT() { (DBGMCU->APB1FZ &= ~(DBGMCU_APB1_FZ_DBG_I2C2_SMBUS_TIMEOUT)); }
    constexpr void DBGMCU_UNFREEZE_I2C3_TIMEOUT() { (DBGMCU->APB1FZ &= ~(DBGMCU_APB1_FZ_DBG_I2C3_SMBUS_TIMEOUT)); }
    constexpr void DBGMCU_UNFREEZE_CAN1() { (DBGMCU->APB1FZ &= ~(DBGMCU_APB1_FZ_DBG_CAN1_STOP)); }
    constexpr void DBGMCU_UNFREEZE_CAN2() { (DBGMCU->APB1FZ &= ~(DBGMCU_APB1_FZ_DBG_CAN2_STOP)); }
    constexpr void DBGMCU_UNFREEZE_TIM1() { (DBGMCU->APB2FZ &= ~(DBGMCU_APB2_FZ_DBG_TIM1_STOP)); }
    constexpr void DBGMCU_UNFREEZE_TIM8() { (DBGMCU->APB2FZ &= ~(DBGMCU_APB2_FZ_DBG_TIM8_STOP)); }
    constexpr void DBGMCU_UNFREEZE_TIM9() { (DBGMCU->APB2FZ &= ~(DBGMCU_APB2_FZ_DBG_TIM9_STOP)); }
    constexpr void DBGMCU_UNFREEZE_TIM10() { (DBGMCU->APB2FZ &= ~(DBGMCU_APB2_FZ_DBG_TIM10_STOP)); }
    constexpr void DBGMCU_UNFREEZE_TIM11() { (DBGMCU->APB2FZ &= ~(DBGMCU_APB2_FZ_DBG_TIM11_STOP)); }

    /**
     * @brief Main Flash memory mapped at 0x00000000
     * 
     */
    constexpr void SYSCFG_REMAPMEMORY_FLASH() { (SYSCFG->MEMRMP &= ~(SYSCFG_MEMRMP_MEM_MODE)); }

    /**
     * @brief System Flash memory mapped at 0x00000000
     * 
     */
    constexpr void SYSCFG_REMAPMEMORY_SYSTEMFLASH()
    {
        SYSCFG->MEMRMP &= ~(SYSCFG_MEMRMP_MEM_MODE);
        SYSCFG->MEMRMP |= SYSCFG_MEMRMP_MEM_MODE_0;
    }

    /**
     * @brief Embedded SRAM mapped at 0x00000000
     * 
     */
    constexpr void SYSCFG_REMAPMEMORY_SRAM()
    {
        SYSCFG->MEMRMP &= ~(SYSCFG_MEMRMP_MEM_MODE);
        SYSCFG->MEMRMP |= (SYSCFG_MEMRMP_MEM_MODE_0 | SYSCFG_MEMRMP_MEM_MODE_1);
    }

#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F417xx)
    /** @brief  FSMC Bank1 (NOR/PSRAM 1 and 2) mapped at 0x00000000
     */
    constexpr void SYSCFG_REMAPMEMORY_FSMC()
    {
        SYSCFG->MEMRMP &= ~(SYSCFG_MEMRMP_MEM_MODE);
        SYSCFG->MEMRMP |= (SYSCFG_MEMRMP_MEM_MODE_1);
    }
#endif /* STM32F405xx || STM32F415xx || STM32F407xx || STM32F417xx */

#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || \
    defined(STM32F469xx) || defined(STM32F479xx)
    /** @brief  FMC Bank1 (NOR/PSRAM 1 and 2) mapped at 0x00000000
     */
    constexpr void SYSCFG_REMAPMEMORY_FMC()
    {
        SYSCFG->MEMRMP &= ~(SYSCFG_MEMRMP_MEM_MODE);
        SYSCFG->MEMRMP |= (SYSCFG_MEMRMP_MEM_MODE_1);
    }

    /** @brief  FMC/SDRAM Bank 1 and 2 mapped at 0x00000000
     */
    constexpr void SYSCFG_REMAPMEMORY_FMC_SDRAM()
    {
        SYSCFG->MEMRMP &= ~(SYSCFG_MEMRMP_MEM_MODE);
        SYSCFG->MEMRMP |= (SYSCFG_MEMRMP_MEM_MODE_2);
    }
#endif /* STM32F427xx || STM32F437xx || STM32F429xx || STM32F439xx || STM32F469xx || STM32F479xx */

#if defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx) || defined(STM32F413xx) || defined(STM32F423xx)
    /** @defgroup Cortex_Lockup_Enable Cortex Lockup Enable
     * @{
      */
    /** @brief  SYSCFG Break Lockup lock
     *         Enables and locks the connection of Cortex-M4 LOCKUP (Hardfault) output to TIM1/8 input
     * @note   The selected configuration is locked and can be unlocked by system reset
     */
    constexpr void SYSCFG_BREAK_PVD_LOCK()
    {
        SYSCFG->CFGR2 &= ~(SYSCFG_CFGR2_PVD_LOCK);
        SYSCFG->CFGR2 |= SYSCFG_CFGR2_PVD_LOCK;
    }
    /**
     * @}
     */

    /** @defgroup PVD_Lock_Enable PVD Lock
     * @{
     */
    /** @brief  SYSCFG Break PVD lock
     *         Enables and locks the PVD connection with Timer1/8 Break Input, , as well as the PVDE and PLS[2:0] in the PWR_CR register
     * @note   The selected configuration is locked and can be unlocked by system reset
     */
    constexpr void SYSCFG_BREAK_LOCKUP_LOCK()
    {
        SYSCFG->CFGR2 &= ~(SYSCFG_CFGR2_LOCKUP_LOCK);
        SYSCFG->CFGR2 |= SYSCFG_CFGR2_LOCKUP_LOCK;
    }
    /**
     * @}
     */
#endif /* STM32F410Tx || STM32F410Cx || STM32F410Rx || STM32F413xx || STM32F423xx */
    /**
     * @}
     */

    /** @defgroup HAL_Private_Macros HAL Private Macros
     * @{
     */

    constexpr bool IsTickFreq(TickFreq freq) { return ((freq == TickFreq::Hz10) || (freq == TickFreq::Hz100) || (freq == TickFreq::Hz1k) || (freq == TickFreq::Default)); }
    /** @} */
    /** @}
     * HAL_Exported_Macros
     */
    class STM
    {
    public:
        STM() = default;
        ~STM() = default;

        // Initialization and Configuration Functions.
        Status Init();
        Status DeInit();
        void MspInit();
        void MspDeInit();
        Status InitTick(uint32_t tickPriority);

        // Peripheral Control functions.
        void IncTick();
        void Delay(uint32_t delay);
        uint32_t GetTick() const { return m_tick; }
        uint32_t GetTickPrio() const { return m_tickPrio; }
        Status SetTickFreq(TickFreq freq);
        TickFreq GetTickFreq() const { return m_tickFreq; }
        void SuspendTick();
        void ResumeTick();
        uint32_t GetHalVersion();
        uint32_t GetREVID();
        uint32_t GetDEVID();
        void DBGMCU_EnableDbgSleepMode();
        void DBGMCU_DisableDbgSleepMode();
        void DBGMCU_EnableDbgStopMode();
        void DBGMCU_DisableDbgStopMode();
        void DBGMCU_EnableDbgStandbyMode();
        void DBGMCU_DisableDbgStandbyMode();
        void EnableCompensationCell();
        void DisableCompensationCell();
        uint32_t GetUIDw0();
        uint32_t GetUIDw1();
        uint32_t GetUIDw2();
#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || \
    defined(STM32F469xx) || defined(STM32F479xx)
        void EnableMemorySwappingBank();
        void DisableMemorySwappingBank();
#endif /* STM32F427xx || STM32F437xx || STM32F429xx || STM32F439xx || STM32F469xx || STM32F479xx */

        static STM *Get()
        {
            return s_instance;
        }

    private:
        static STM *s_instance;

        volatile uint32_t m_tick = 0;
        uint32_t m_tickPrio = TICK_INT_PRIORITY;
        TickFreq m_tickFreq = TickFreq::Default;

        RstCtrlClk::RstCtrlClk m_rcc;

        constexpr uint8_t HAL_VERSION_MAIN = 0x01U; //!< [31:24] main version.
        constexpr uint8_t HAL_VERSION_SUB1 = 0x07U; //!< [23:16] sub1 version.
        constexpr uint8_t HAL_VERSION_SUB2 = 0x08U; //!< [15:8] sub2 version.
        constexpr uint8_t HAL_VERSION_RC = 0x00U;   //!< [7:0] release candidate.
        constexpr uint32_t HAL_VERSION = (((uint32_t)HAL_VERSION_MAIN << 24U) | ((uint32_t)HAL_VERSION_SUB1 << 16U) | ((uint32_t)HAL_VERSION_SUB2 << 8U) | ((uint32_t)HAL_VERSION_RC));

        constexpr uint32_t IDCODE_DEVID_MASK = 0x00000FFFU;

        // RCC Registers bit address in the alias region.
        constexpr uint32_t SYSCFG_OFFSET = (SYSCFG_BASE - PERIPH_BASE);
        // MEMRMP Register.
        // Alias word address of UFB_MODE bit.
        constexpr uint32_t MEMRMP_OFFSET = SYSCFG_OFFSET;
        constexpr uint32_t UFB_MODE_BIT_NUMBER = SYSCFG_MEMRMP_MEM_MODE_Pos;
        constexpr uint32_t UFB_MODE_BB = (uint32_t)(PERIPH_BB_BASE + (MEMRMP_OFFSET * 32U) + (UFB_MODE_BIT_NUMBER * 4U));

        // CMPCR Register.
        // Alias word address of CMP_PD bit.
        constexpr uint32_t CMPCR_OFFSET = SYSCFG_OFFSET + 0x20U;
        constexpr uint32_t CMP_PD_BIT_NUMBER = SYSCFG_CMPCR_CMP_PD_Pos;
        constexpr uint32_t CMPCR_CMP_PD_BB = (uint32_t)(PERIPH_BB_BASE + (CMPCR_OFFSET * 32U) + (CMP_PD_BIT_NUMBER * 4U));

        // MCHDLYCR Register.
        // Alias word address of BSCKSEL bit.
        constexpr uint32_t MCHDLYCR_OFFSET = SYSCFG_OFFSET + 0x30U;
        constexpr uint32_t BSCKSEL_BIT_NUMBER = SYSCFG_MCHDLYCR_BSCKSEL_Pos;
        constexpr uint32_t MCHDLYCR_BSCKSEL_BB = (uint32_t)(PERIPH_BB_BASE + (MCHDLYCR_OFFSET * 32U) + (BSCKSEL_BIT_NUMBER * 4U));
    };
} // namespace HAL

/** @}
 * HAL
 */

/** @}
 * STM32F4xx_HAL_Driver
 */

#endif
