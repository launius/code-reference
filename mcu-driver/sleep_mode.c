/*
 * The example code for deep-sleep mode of MCU GD32F3x0
 *
 * Copyright (c) 2025 Yunjae Lim <launius@gmail.com>
 *
 * The Deep-sleep mode is based on the SLEEPDEEP mode of the Cortex®-M4.
 * Any interrupt or wakeup event from EXTI lines can wake up the
 * system from the deep-sleep mode including the 16 external lines.
 *
 * 1. EXTI sources selection register 1 (SYSCFG_EXTISS1)

	15          12   11           8   7            4   3            0
	-------------------------------------------------------------------
	EXTI7_SS [3:0] | EXTI6_SS [3:0] | EXTI5_SS [3:0] | EXTI4_SS [3:0] |
	-------------------------------------------------------------------

	Bits	Fields			Descriptions
	7:4		EXTI5_SS[3:0]	EXTI 5 sources selection
							X000: PA5 pin
							X001: PB5 pin
							X010: PC5 pin
							X011: Reserved
							X100: Reserved
							X101: PF5 pin
							X110: Reserved
							X111: Reserved
 *
 * 2. Interrupt vector table

	Interrupt number    Vector number    Peripheral interrupt description
	---------------------------------------------------------------------
	IRQ 5               21               EXTI line0-1 interrupts
	IRQ 6               22               EXTI line2-3 interrupts
	IRQ 7               23               EXTI line4-15 interrupts
 *
 * 3. EXTI source
 
	EXTI Line    Source
	Number
	----------------------------------
	5            PA5 / PB5 / PC5 / PF5
 *
 * Reference: https://www.gigadevice.com.cn/Public/Uploads/uploadfile/files/20240407/GD32F3x0_User_Manual_Rev2.9.pdf
 */

#include "gd32f3x0.h"	// including gd32f3x0_syscfg.h and gd32f3x0_exti.h

#define EXTI_IRQ_PREPRIORITY									6U
#define EXTI_IRQ_SUBPRIORITY									1U

struct sleep_mode_config
{
    uint8_t exti_port;
    uint8_t exti_pin;
    exti_line_enum linex;
} smc = {EXTI_SOURCE_GPIOF, EXTI_SOURCE_PIN5, EXTI_5};

void sleep_mode_init()
{
    syscfg_exti_line_config(smc.exti_port, smc.exti_pin);

    /* Configure EXTI Line 5 */
    exti_init(smc.linex, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_interrupt_flag_clear(smc.linex);
    nvic_irq_enable(EXTI4_15_IRQn, EXTI_IRQ_PREPRIORITY, EXTI_IRQ_SUBPRIORITY);
}

void EXTI4_15_IRQHandler(void)
{
    if(exti_interrupt_flag_get(smc.linex)) {
        /* clear the EXTI line 5 interrupt flag */
        exti_interrupt_flag_clear(smc.linex);
    }
}

void sleep_mode_enable(void)
{
    exti_interrupt_flag_clear(smc.linex);

	/// do something before entering deep-sleep mode.
	// e.g. disable peripherals, save state, etc.
	// digital_io_deinit();

    /* Enter to sleep mode */
    __disable_irq();
    pmu_to_deepsleepmode(PMU_LDO_LOWPOWER, WFI_CMD);
    __enable_irq();

	// do something after waking up from deep-sleep mode
	// e.g. re-initialize peripherals, restore state, etc.
	// digital_io_init();
}
