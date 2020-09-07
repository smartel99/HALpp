/**
 * @file        stm32f405zgtx.h
 * @author      Samuel Martel
 * @p           https://www.github.com/smartel99
 * @date        2020/09/06
 * @brief       This file contains stm32f405zgtx common defines, enumeration, 
 *              macros and structures definitions. 
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
#ifndef __STM32F405ZGTX_DEF_H__
#define __STM32F405ZGTX_DEF_H__

#include <cstddef>

namespace HAL
{
    /**
     * @enum    Status
     * @brief   STM32 Status structures definition
     */
    enum class Status
    {
        Ok = 0,
        Error = 1,
        Busy = 2,
        Timeout = 3,
    };

    /**
     * @brief STM32 Lock structures definition.
     * 
     */
    enum class Lock
    {
        Unlocked = 0,
        Locked = 1,
    };

/* Exported Macros ----------------------------------------------------------*/
/**
 * @brief To Avoid GCC/G++ warnings
 * 
 */
#define UNUSED(X) (void)X

    constexpr size_t MAX_DELAY = 0xFFFFFFFFU;

    template <typename R, typename B>
    constexpr bool IsBitSet(R reg, B bit) { return ((reg & bit) == bit); }
    template <typename R, typename B>
    constexpr bool IsBitClr(R reg, B bit) { return ((reg & bit) == 0); }

#define __STM32_LINKDMA(__HANDLE__, __PPP_DMA_FIELD__, __DMA_HANDLE__) \
    do                                                                 \
    {                                                                  \
        (__HANDLE__)->__PPP_DMA_FIELD__ = &(__DMA_HANDLE__);           \
        (__DMA_HANDLE__).Parent = (__HANDLE__);                        \
    } while (0U)

#define __STM32_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->State = 0U)

#define __STM32_LOCK(__HANDLE__)                     \
    do                                               \
    {                                                \
        if ((__HANDLE__)->Lock == STM::Lock::Locked) \
        {                                            \
            return STM::Status::Busy;                \
        }                                            \
        else                                         \
        {                                            \
            (__HANDLE__)->Lock = STM::Lock::Locked;  \
        }                                            \
    } while (0U)

#define __STM32_UNLOCK(__HANDLE__)                \
    do                                            \
    {                                             \
        (__HANDLE__)->Lock = STM::Lock::Unlocked; \
    } while (0U)

#if defined(__GNUC__) && !defined(__CC_ARM) /* GNU Compiler */
#ifndef __weak
#define __weak __attribute__((weak))
#endif /* __weak */
#ifndef __packed
#define __packed __attribute__((__packed__))
#endif /* __packed */
#endif /* __GNUC__ */

/* Macro to get variable aligned on 4-bytes, for __ICCARM__ the directive "#pragma data_alignment=4" must be used instead */
#if defined(__GNUC__) && !defined(__CC_ARM) /* GNU Compiler */
#ifndef __ALIGN_END
#define __ALIGN_END __attribute__((aligned(4)))
#endif /* __ALIGN_END */
#ifndef __ALIGN_BEGIN
#define __ALIGN_BEGIN
#endif /* __ALIGN_BEGIN */
#else
#ifndef __ALIGN_END
#define __ALIGN_END
#endif /* __ALIGN_END */
#ifndef __ALIGN_BEGIN
#if defined(__CC_ARM) /* ARM Compiler */
#define __ALIGN_BEGIN __align(4)
#elif defined(__ICCARM__) /* IAR Compiler */
#define __ALIGN_BEGIN
#endif /* __CC_ARM */
#endif /* __ALIGN_BEGIN */
#endif /* __GNUC__ */

/** 
  * @brief  __RAM_FUNC definition
  */
#if defined(__CC_ARM)
/* ARM Compiler
   ------------
   RAM functions are defined using the toolchain options. 
   Functions that are executed in RAM should reside in a separate source module.
   Using the 'Options for File' dialog you can simply change the 'Code / Const' 
   area of a module to a memory space in physical RAM.
   Available memory areas are declared in the 'Target' tab of the 'Options for Target'
   dialog. 
*/
#define __RAM_FUNC

#elif defined(__ICCARM__)
/* ICCARM Compiler
   ---------------
   RAM functions are defined using a specific toolchain keyword "__ramfunc". 
*/
#define __RAM_FUNC __ramfunc

#elif defined(__GNUC__)
/* GNU Compiler
   ------------
  RAM functions are defined using a specific toolchain attribute 
   "__attribute__((section(".RamFunc")))".
*/
#define __RAM_FUNC __attribute__((section(".RamFunc")))

#endif

/** 
  * @brief  __NOINLINE definition
  */
#if defined(__CC_ARM) || defined(__GNUC__)
/* ARM & GNUCompiler 
   ---------------- 
*/
#define __NOINLINE __attribute__((noinline))

#elif defined(__ICCARM__)
/* ICCARM Compiler
   ---------------
*/
#define __NOINLINE _Pragma("optimize = no_inline")

} // namespace HAL
#endif
#endif
