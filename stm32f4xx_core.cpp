/**
 * @file        stm32f4xx_core.cpp
 * @author      Samuel Martel
 * @p           https://www.github.com/smartel99
 * @date        2020/09/06
 * @brief       File containing the definition of the STM class.
 */
/*************************************************************************************************/
#include "stm32f4xx_core.h"

namespace HAL
{
    STM *STM::s_instance = new STM();
    /** @defgroup HAL_Exported_Functions HAL Exported Functions
     * @{
     */

    /** @defgroup HAL_Exported_Functions_Group1 Initialization and de-initialization Functions 
     *  @brief    Initialization and de-initialization functions
     *
    @verbatim    
    ===============================================================================
                ##### Initialization and Configuration functions #####
    ===============================================================================
        [..]  This section provides functions allowing to:
        (+) Initializes the Flash interface the NVIC allocation and initial clock 
            configuration. It initializes the systick also when timeout is needed 
            and the backup domain when enabled.
        (+) De-Initializes common part of the HAL.
        (+) Configure the time base source to have 1ms time base with a dedicated 
            Tick interrupt priority. 
            (++) SysTick timer is used by default as source of time base, but user
                can eventually implement his proper time base source (a general purpose 
                timer for example or other time source), keeping in mind that Time base 
                duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
                handled in milliseconds basis.
            (++) Time base configuration function (HAL_InitTick ()) is called automatically 
                at the beginning of the program after reset by HAL_Init() or at any time 
                when clock is configured, by HAL_RCC_ClockConfig(). 
            (++) Source of time base is configured  to generate interrupts at regular 
                time intervals. Care must be taken if HAL_Delay() is called from a 
                peripheral ISR process, the Tick interrupt line must have higher priority 
                (numerically lower) than the peripheral interrupt. Otherwise the caller 
                ISR process will be blocked. 
        (++) functions affecting time base configurations are declared as __weak  
                to make  override possible  in case of other  implementations in user file.
    @endverbatim
    * @{
    */

    /**
     * @brief  This function is used to initialize the HAL Library; it must be the first 
     *         instruction to be executed in the main program (before to call any other
     *         HAL function), it performs the following:
     *           Configure the Flash prefetch, instruction and Data caches.
     *           Configures the SysTick to generate an interrupt each 1 millisecond,
     *           which is clocked by the HSI (at this stage, the clock is not yet
     *           configured and thus the system is running from the internal HSI at 16 MHz).
     *           Set NVIC Group Priority to 4.
     *           Calls the HAL_MspInit() callback function defined in user file 
     *           "stm32f4xx_hal_msp.c" to do the global low level hardware initialization 
     *            
     * @note   SysTick is used as time base for the HAL_Delay() function, the application
     *         need to ensure that the SysTick time base is always set to 1 millisecond
     *         to have correct HAL operation.
     * @retval HAL status
     */
    Status STM::Init()
    {
        // Configure Flash prefetch, Instruction cache, Data cache.
#if (INSTRUCTION_CHACHE_ENABLE != 0U)
        FLASH_INSTRUCTION_CACHE_ENABLE();
#endif

#if (DATA_CACHE_ENABLE != 0U)
        FLASH_DATA_CACHE_ENABLE();
#endif

#if (PREFETCH_ENABLE != 0U)
        FLASH_PREFETCH_BUFFER_ENABLE();
#endif

        // Setup Interrupt Group Priority.
        // m_nvic.SetPriorityGrouping(NVIC::PriorityGroup::Group4);

        // Use systick as time base source and configure 1ms tick (default clock after Reset is HSI).
        InitTick(TICK_INT_PRIORITY);

        // Init the low level hardware.
        MspInit();

        return Status::Ok;
    }

    /**
     * @brief  This function de-Initializes common part of the HAL and stops the systick.
     *         This function is optional.   
     * @retval HAL status
     */
    Status STM::DeInit()
    {
        // Reset all periphehrals.
        HAL::RstCtrlClk::Apb1ForceReset();
        HAL::RstCtrlClk::Apb1ReleaseReset();

        HAL::RstCtrlClk::Apb2ForceReset();
        HAL::RstCtrlClk::Apb2ReleaseReset();

        HAL::RstCtrlClk::Ahb1ForceReset();
        HAL::RstCtrlClk::Ahb1ReleaseReset();

        HAL::RstCtrlClk::Ahb2ForceReset();
        HAL::RstCtrlClk::Ahb2ReleaseReset();

        HAL::RstCtrlClk::Ahb3ForceReset();
        HAL::RstCtrlClk::Ahb3ReleaseReset();

        // De-init the low level hardware.
        MspDeInit();

        return Status::Ok;
    }

    /**
     * @brief  Initialize the MSP.
     * @retval None
     */
    void STM::MspInit()
    {
        HAL::RstCtrlClk::SyscfgClkEnable();
        HAL::RstCtrlClk::PwrClkEnable();

        m_nvic.SetPriorityGrouping(HAL::NVIC::PriorityGroup::Group3);

        // System Interrupt Init.
        // DebugMonitor_IRQn interrupt configuration.
        m_nvic.SetPriority(HAL::NVIC::Irq::DebugMonitor, 5, 1);
    }

    /**
     * @brief  DeInitializes the MSP.
     * @retval None
     */
    void STM::MspDeInit() {}

    /**
     * @brief This function configures the source of the time base.
     *        The time source is configured  to have 1ms time base with a dedicated 
     *        Tick interrupt priority.
     * @note This function is called  automatically at the beginning of program after
     *       reset by Init() or at any time when clock is reconfigured  by HAL::RstCtrlClk::ConfigClock().
     * @note In the default implementation, SysTick timer is the source of time base. 
     *       It is used to generate interrupts at regular time intervals. 
     *       Care must be taken if STM::Delay() is called from a peripheral ISR process, 
     *       The SysTick interrupt must have higher priority (numerically lower)
     *       than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
     * @param TickPriority Tick interrupt priority.
     * @retval HAL status
     */
    Status STM::InitTick(uint32_t tickPriority)
    {
        // Configure the SysTick to increment once every 1ms.
        if (m_cortex.SystickConfig(m_rcc.GetSysClockFreq() / (1000U / m_tickFreq)) > 0U)
        {
            return Status::Error;
        }

        // Configure the SysTick IRQ priority.
        if (tickPriority < (1UL << __NVIC_PRIO_BITS))
        {
            m_nvic.SetPriority(HAL::NVIC::Irq::SysTick, tickPriority, 0U);
            m_tickPrio = tickPriority;
        }
        else
        {
            return Status::Error;
        }

        return Status::Ok;
    }
    /** @}
     * HAL_Exported_Functions_Group1
     */

    /** @defgroup HAL_Exported_Functions_Group2 HAL Control functions 
     *  @brief    HAL Control functions
     *
    @verbatim
    ===============================================================================
                        ##### HAL Control functions #####
    ===============================================================================
        [..]  This section provides functions allowing to:
        (+) Provide a tick value in millisecond
        (+) Provide a blocking delay in millisecond
        (+) Suspend the time base source interrupt
        (+) Resume the time base source interrupt
        (+) Get the HAL API driver version
        (+) Get the device identifier
        (+) Get the device revision identifier
        (+) Enable/Disable Debug module during SLEEP mode
        (+) Enable/Disable Debug module during STOP mode
        (+) Enable/Disable Debug module during STANDBY mode

    @endverbatim
    * @{
    */

    /**
     * @brief This function is called to increment  a global variable "uwTick"
     *        used as application time base.
     * @note In the default implementation, this variable is incremented each 1ms
     *       in SysTick ISR.
     * @retval None
     */
    void STM::IncTick() { m_tick += m_tickFreq; }

    /**
     * @brief Set new tick Freq.
     * @retval Status
     */
    Status STM::SetTickFreq(TickFreq freq)
    {
        assert_param(IsTickFreq(freq));

        Status status = Status::Ok;

        if (m_tickFreq != freq)
        {
            // Backup m_tickFreq.
            TickFreq backup = m_tickFreq;

            // Update m_tickFreq which is used by InitTick().
            m_tickFreq = freq;

            // Apply the new tick frequency.
            status = InitTick(m_tickPrio);
            if (status != Status::Ok)
            {
                // Restore previous tick frequency.
                m_tickFreq = backup;
            }
        }

        return status;
    }

    /**
     * @brief This function provides minimum delay (in milliseconds) based 
     *        on variable incremented.
     * @note In the default implementation , SysTick timer is the source of time base.
     *       It is used to generate interrupts at regular time intervals where uwTick
     *       is incremented.
     * @param Delay specifies the delay time length, in milliseconds.
     * @retval None
     */
    void STM::Delay(uint32_t delay)
    {
        uint32_t start = GetTick();

        // Add a freq to guarantee minimum wait.
        if (delay < HAL::MAX_DELAY)
        {
            delay += (uint32_t)m_tickFreq;
        }

        while ((m_tick - start) < delay)
        {
        }
    }

    /**
     * @brief Suspend Tick increment.
     * @note In the default implementation , SysTick timer is the source of time base. It is
     *       used to generate interrupts at regular time intervals. Once HAL_SuspendTick()
     *       is called, the SysTick interrupt will be disabled and so Tick increment 
     *       is suspended.
     * @retval None
     */
    void STM::SuspendTick()
    {
        // Disable SysTick Interrupt.
        SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
    }

    /**
     * @brief Resume Tick increment.
     * @note In the default implementation , SysTick timer is the source of time base. It is
     *       used to generate interrupts at regular time intervals. Once HAL_ResumeTick()
     *       is called, the SysTick interrupt will be enabled and so Tick increment 
     *       is resumed.
     * @retval None
     */
    void STM::ResumeTick()
    {
        // Enable SysTick Interrupt.
        SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
    }

    /**
     * @brief  Returns the HAL revision
     * @retval version : 0xXYZR (8bits for each decimal, R for RC)
     */
    uint32_t STM::GetHalVersion() { return HAL_VERSION; }

    /**
     * @brief  Returns the device revision identifier.
     * @retval Device revision identifier
     */
    uint32_t STM::GetREVID() { return (DBGMCU->IDCODE) >> 16U; }

    /**
     * @brief  Returns the device identifier.
     * @retval Device identifier
     */
    uint32_t STM::GetDEVID() { return ((DBGMCU->IDCODE) & DBGMCU_IDCODE_DEV_ID_Msk); }

    /**
     * @brief  Enable the Debug Module during SLEEP mode
     * @retval None
     */
    void STM::DBGMCU_EnableDbgSleepMode() { SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_SLEEP); }

    /**
     * @brief  Disable the Debug Module during SLEEP mode
     * @retval None
     */
    void STM::DBGMCU_DisableDbgSleepMode() { CLEAR_BIT(DBGMCU->CR, DBGMCU_CR_DBG_SLEEP); }

    /**
     * @brief  Enable the Debug Module during STOP mode
     * @retval None
     */
    void STM::DBGMCU_EnableDbgStopMode() { SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_STOP); }

    /**
     * @brief  Disable the Debug Module during STOP mode
     * @retval None
     */
    void STM::DBGMCU_DisableDbgStopMode() { CLEAR_BIT(DBGMCU->CR, DBGMCU_CR_DBG_STOP); }

    /**
     * @brief  Enable the Debug Module during STANDBY mode
     * @retval None
     */
    void STM::DBGMCU_EnableDbgStandbyMode() { SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_STANDBY); }

    /**
     * @brief  Disable the Debug Module during STANDBY mode
     * @retval None
     */
    void STM::DBGMCU_DisableDbgStandbyMode() { CLEAR_BIT(DBGMCU->CR, DBGMCU_CR_DBG_STANDBY); }

    /**
     * @brief  Enables the I/O Compensation Cell.
     * @note   The I/O compensation cell can be used only when the device supply
     *         voltage ranges from 2.4 to 3.6 V.  
     * @retval None
     */
    void STM::EnableCompensationCell() { *(__IO uint32_t *)CMPCR_CMP_PD_BB = (uint32_t)ENABLE; }

    /**
     * @brief  Power-down the I/O Compensation Cell.
     * @note   The I/O compensation cell can be used only when the device supply
     *         voltage ranges from 2.4 to 3.6 V.  
     * @retval None
     */
    void STM::DisableCompensationCell() { *(__IO uint32_t *)CMPCR_CMP_PD_BB = (uint32_t)DISABLE; }

    /**
     * @brief  Returns first word of the unique device identifier (UID based on 96 bits)
     * @retval Device identifier
     */
    uint32_t STM::GetUIDw0() { return (READ_REG(*((uint32_t *)UID_BASE))); }

    /**
     * @brief  Returns second word of the unique device identifier (UID based on 96 bits)
     * @retval Device identifier
     */
    uint32_t STM::GetUIDw1() { return (READ_REG(*((uint32_t *)(UID_BASE + 4U)))); }

    /**
     * @brief  Returns third word of the unique device identifier (UID based on 96 bits)
     * @retval Device identifier
     */
    uint32_t STM::GetUIDw2() { return (READ_REG(*((uint32_t *)(UID_BASE + 8U)))); }

#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || \
    defined(STM32F469xx) || defined(STM32F479xx)
    /**
     * @brief  Enables the Internal FLASH Bank Swapping.
     *   
     * @note   This function can be used only for STM32F42xxx/43xxx/469xx/479xx devices. 
     *
     * @note   Flash Bank2 mapped at 0x08000000 (and aliased @0x00000000) 
     *         and Flash Bank1 mapped at 0x08100000 (and aliased at 0x00100000)   
     *
     * @retval None
     */
    void STM::EnableMemorySwappingBank()
    {
        *(__IO uint32_t *)UFB_MODE_BB = (uint32_t)ENABLE;
    }

    /**
     * @brief  Disables the Internal FLASH Bank Swapping.
     *   
     * @note   This function can be used only for STM32F42xxx/43xxx/469xx/479xx devices. 
     *
     * @note   The default state : Flash Bank1 mapped at 0x08000000 (and aliased @0x00000000) 
     *         and Flash Bank2 mapped at 0x08100000 (and aliased at 0x00100000) 
     *           
     * @retval None
     */
    void STM::DisableMemorySwappingBank()
    {
        *(__IO uint32_t *)UFB_MODE_BB = (uint32_t)DISABLE;
    }
#endif /* STM32F427xx || STM32F437xx || STM32F429xx || STM32F439xx || STM32F469xx || STM32F479xx */
    /**
     * @}
     */

    /**
     * @}
     */

    /**
     * @}
     */

    /**
     * @}
     */
} // namespace HAL
