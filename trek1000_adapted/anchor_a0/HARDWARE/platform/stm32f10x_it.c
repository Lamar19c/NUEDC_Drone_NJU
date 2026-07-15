/**
 * stm32f10x_it.c — TREK1000 移植版 (适配 Schematic2 网表引脚)
 *
 * 基于 TREK1000 Station stm32f10x_it.c，修改 ISR 映射以适配:
 *   RSTn: PA0→PB4 (EXTI_Line4 替代 EXTI_Line0)
 *   IRQ:  PB5→PB0 (EXTI_Line0 替代 EXTI_Line5)
 *
 * 核心变更:
 *   EXTI0_IRQHandler  原先处理 process_dwRSTn_irq() → 现处理 process_deca_irq()
 *   EXTI4_IRQHandler  新增 → 处理 process_dwRSTn_irq()
 *   EXTI9_5_IRQHandler 原先处理 process_deca_irq() → 保留但不再用于 DW1000 IRQ
 *
 * 替换方法:
 *   将本文件覆盖到 TREK1000 源码的 HARDWARE/platform/stm32f10x_it.c
 *   (Station: test_rx11/HARDWARE/platform/; Tag: tag/HARDWARE/platform/)
 *
 *   ⚠️ 本文件基于 STATION 版本。TAG 版本内容基本相同，差异仅在 USB_SUPPORT 配置。
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "port.h"

#ifdef USB_SUPPORT

#include "usb_core.h"
#include "usbd_core.h"

extern USB_OTG_CORE_HANDLE           USB_OTG_dev;
extern uint32_t USBD_OTG_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);

#ifdef USB_OTG_HS_DEDICATED_EP1_ENABLED
extern uint32_t USBD_OTG_EP1IN_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);
extern uint32_t USBD_OTG_EP1OUT_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);
#endif

#endif
__IO unsigned long time32_incr;

/* Cortex-M3 Processor Exceptions Handlers */

void NMI_Handler(void) { }

void pop_registers_from_fault_stack(unsigned int * hardfault_args)
{
    unsigned int stacked_r0  = ((unsigned long) hardfault_args[0]);
    unsigned int stacked_r1  = ((unsigned long) hardfault_args[1]);
    unsigned int stacked_r2  = ((unsigned long) hardfault_args[2]);
    unsigned int stacked_r3  = ((unsigned long) hardfault_args[3]);
    unsigned int stacked_r12 = ((unsigned long) hardfault_args[4]);
    unsigned int stacked_lr  = ((unsigned long) hardfault_args[5]);
    unsigned int stacked_pc  = ((unsigned long) hardfault_args[6]);
    unsigned int stacked_psr = ((unsigned long) hardfault_args[7]);
    for( ;; ) { }
    {
        unsigned long u = stacked_r0 + stacked_r1 + stacked_r2 + stacked_r3
                        + stacked_r12 + stacked_lr + stacked_pc + stacked_psr;
        if (u == 0) return ;
    }
}

void HardFault_Handler(void)
{
    { __asm volatile (  " tst lr, #4 \n"
                        " ite eq \n"
                        " mrseq r0, msp \n"
                        " mrsne r0, psp \n"
                        " ldr r1, [r0, #24] \n"
                        " ldr r2, handler2_address_const \n"
                        " bx r2 \n"
                        " handler2_address_const: .word pop_registers_from_fault_stack \n" );
    }
    while (1) { }
}

void MemManage_Handler(void)  { while (1) { } }
void BusFault_Handler(void)   { while (1) { } }
void UsageFault_Handler(void) { while (1) { } }
void SVC_Handler(void)        { }
void DebugMon_Handler(void)   { }
void PendSV_Handler(void)     { }

void RTC_IRQHandler(void) { }

void SysTick_Handler(void)
{
    time32_incr++;
#ifdef FILESYSTEM_ENABLE
    fsd_service();
#endif
}

void EXTI15_10_IRQHandler(void)
{
    button_callback();
    EXTI_ClearITPendingBit(EXTI_Line13);
}

/******************************************************************************/
/*            DW1000 中断 — Schematic2 适配版                                */
/*                                                                            */
/*  原映射:                                                                   */
/*    EXTI0_IRQHandler  → process_dwRSTn_irq()  (RSTn=PA0, EXTI_Line0)       */
/*    EXTI9_5_IRQHandler → process_deca_irq()    (IRQ=PB5,  EXTI_Line5)      */
/*                                                                            */
/*  Schematic2 适配:                                                          */
/*    EXTI4_IRQHandler  → process_dwRSTn_irq()  (RSTn=PB4, EXTI_Line4) ←新增 */
/*    EXTI0_IRQHandler  → process_deca_irq()    (IRQ=PB0,  EXTI_Line0) ←改动 */
/******************************************************************************/

/* ── DW1000 IRQ (DECAIRQ): PB0, EXTI_Line0 (原为 EXTI9_5) ── */
void EXTI0_IRQHandler(void)
{
    process_deca_irq();
    EXTI_ClearITPendingBit(DECAIRQ_EXTI);       /* EXTI_Line0 */
}

/* ── DW1000 RSTn IRQ (DECARSTIRQ): PB4, EXTI_Line4 (原为 EXTI0) ── */
void EXTI4_IRQHandler(void)
{
    process_dwRSTn_irq();
    EXTI_ClearITPendingBit(DECARSTIRQ_EXTI);    /* EXTI_Line4 */
}

/* ── 保留: EXTI3 IRQ (TAG 按键等)——视具体 PCB 使用 ── */
void EXTI3_IRQHandler(void)
{
    process_deca_irq();
    EXTI_ClearITPendingBit(EXTI_Line3);
}

/* ── 保留: EXTI9_5 — 不再用于 DW1000, 如其他外设使用请保留 ── */
void EXTI9_5_IRQHandler(void)
{
    process_deca_irq();
    EXTI_ClearITPendingBit(DECAIRQ_EXTI);
}

#ifdef USB_SUPPORT
#ifdef USE_USB_OTG_FS
void OTG_FS_WKUP_IRQHandler(void) { /* ... */ EXTI_ClearITPendingBit(EXTI_Line18); }
#endif
#ifdef USE_USB_OTG_HS
void OTG_HS_WKUP_IRQHandler(void) { /* ... */ EXTI_ClearITPendingBit(EXTI_Line20); }
#endif
#ifdef USE_USB_OTG_HS
void OTG_HS_IRQHandler(void)
#else
void OTG_FS_IRQHandler(void)
#endif
{ USBD_OTG_ISR_Handler (&USB_OTG_dev); }
#ifdef USB_OTG_HS_DEDICATED_EP1_ENABLED
void OTG_HS_EP1_IN_IRQHandler(void)  { USBD_OTG_EP1IN_ISR_Handler (&USB_OTG_dev); }
void OTG_HS_EP1_OUT_IRQHandler(void) { USBD_OTG_EP1OUT_ISR_Handler (&USB_OTG_dev); }
#endif
#endif
