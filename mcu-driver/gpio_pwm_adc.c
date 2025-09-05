/*
 * Copyright (c) 2025 Yunjae Lim <launius@gmail.com>
 *
 * GD32F350G8(QFN28 package) MCU reference code with
 *   LED outputs using PWM mode,
 *   GPIO inputs using hysterisis for encoders/buttons and
 *   ADC inputs using polling or interrupt
 *
 * Pin configurations:
 *
 * PF0: GPIO input, power switch, LED R on/off
 * PF1: GPIO input, encoder button, LED G on/blink
 * PA6/PA7: GPIO input, encoder rotation, LED B brightness
 *
 * PA4: GPIO output, LED R, driven by timer13 using PWM
 * PB6: GPIO output, LED G, driven by timer15 using PWM
 * PB7: GPIO output, LED B, driven by timer16 using PWM
 * PA2: GPIO output, LED W, driven by timer14 using PWM
 *
 * PA0: ADC input, battery voltage
 * PA1: ADC input, USB-C CC voltage
 *
 * Reference:
 *   https://www.gigadevice.com.cn/Public/Uploads/uploadfile/files/20241008/GD32F350xxDatasheetRev3.1.pdf
 *   https://www.gigadevice.com.cn/Public/Uploads/uploadfile/files/20240407/GD32F3x0_User_Manual_Rev2.9.pdf
 */

#include "gd32f3x0.h"

volatile uint32_t tick = 0U;

volatile uint8_t led_mode = 0;	// 0: ON, 1: Blinking
static uint32_t led_r = 0;		// default off
static uint32_t led_g = 0;
static uint32_t led_b = 0;
static uint32_t led_w = 0;
static uint32_t delta = 2;

enum led_color {
	LED_RED,
	LED_GREEN,
	LED_BLUE,
	LED_WHITE,
	LED_BLACK,
};

void SysTick_Handler(void)
{
	tick++;
}

void systick_config(void)
{
	// setup systick timer for 1000Hz interrupts
	SysTick_Config(SystemCoreClock / 1000);

	// configure the systick handler priority
	NVIC_SetPriority(SysTick_IRQn, 0x00U);
}

uint32_t systick_millis(void)
{
	return tick;
}

void systick_delay(uint32_t ms)
{
	uint32_t start = tick;
	while ((tick - start) < ms);
}

static void _pwm_rcu_config(void)
{
	rcu_periph_clock_enable(RCU_GPIOA);
	rcu_periph_clock_enable(RCU_GPIOB);
	rcu_periph_clock_enable(RCU_GPIOF);

	rcu_periph_clock_enable(RCU_TIMER13);
	rcu_periph_clock_enable(RCU_TIMER14);
	rcu_periph_clock_enable(RCU_TIMER15);
	rcu_periph_clock_enable(RCU_TIMER16);
}

static void _pwm_gpio_config(void)
{
	/*
	 * GD32F350xx pin alternate functions in the datasheet.
	
		Pin
		Name	| AF0		| AF1		| AF2		| AF3		| AF4

		PA2		| TIMER14_CH0 |			| TIMER1_CH2 |			|
		PA4		|			|			|			|			| TIMER13_CH0
		PB6		|			|			| TIMER15_CH0_ON |		|
		PB7		|			|			| TIMER16_CH0_ON |		|
	 */

	/* configure PA2 as PWM output */
	gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_2);
	gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);
	gpio_af_set(GPIOA, GPIO_AF_0, GPIO_PIN_2);

	/* configure PA4 as PWM output */
	gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_4);
	gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_4);
	gpio_af_set(GPIOA, GPIO_AF_4, GPIO_PIN_4);

	/* configure PB6/PB7 as PWM output */
	gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_6 | GPIO_PIN_7);
	gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6 | GPIO_PIN_7);
	gpio_af_set(GPIOB, GPIO_AF_2, GPIO_PIN_6 | GPIO_PIN_7);
}

static void _pwm_timer_config(void)
{
	/*
	 * Timers info from the datasheet
	 *
	 * 1. GD32F350Gx QFN28 pin definitions:

		Pin Name | Functions description
		PA2		 | TIMER14_CH0
		PA4		 | TIMER13_CH0
		PB6		 | TIMER15_CH0_ON
		PB7		 | TIMER16_CH0_ON
	 *
	 * 2. GD32F350xx clock tree:
	 
		TIMER1,2,5,13
			if(APB1 prescaler=1)
				= AHB
			else
				= AHB / (APB1 prescaler /2)

		TIMER0,14,15,16
			if(APB2 prescaler=1)
				= AHB
			else
				= AHB / (APB2 prescaler /2)
	 *
	 * Need to check AHB, APB1, APB2 bus clock settings
	 * in system_gd32f3x0.c. For example,
	 
		static void system_clock_96m_irc8m(void)
		{
			// AHB = SYSCLK
			RCU_CFG0 |= RCU_AHB_CKSYS_DIV1;
			// APB2 = AHB/2
			RCU_CFG0 |= RCU_APB2_CKAHB_DIV2;
			// APB1 = AHB/2
			RCU_CFG0 |= RCU_APB1_CKAHB_DIV2;
	 */

	/* initialize TIMER13-16 */
	timer_parameter_struct timer_initpara;
	timer_oc_parameter_struct timer_ocintpara;

	timer_deinit(TIMER13);
	timer_deinit(TIMER14);
	timer_deinit(TIMER15);
	timer_deinit(TIMER16);

	/* TIMER14-16 configuration */
	timer_initpara.prescaler		 = 107;		// 108MHz / (107+1) = 1MHz
	timer_initpara.alignedmode		 = TIMER_COUNTER_EDGE;
	timer_initpara.counterdirection	 = TIMER_COUNTER_UP;
	timer_initpara.period			 = 255;		// 8-bit PWM
	timer_initpara.clockdivision	 = TIMER_CKDIV_DIV1;
	timer_initpara.repetitioncounter = 0;

	timer_init(TIMER14, &timer_initpara);
	timer_init(TIMER15, &timer_initpara);
	timer_init(TIMER16, &timer_initpara);

	timer_initpara.prescaler = 1;	// different prescale for TIMER13 if APB1 uses different clock
	timer_init(TIMER13,&timer_initpara);

	/* CH0 configuration in PWM mode */
	timer_ocintpara.outputstate	 = TIMER_CCX_ENABLE;
	timer_ocintpara.outputnstate = TIMER_CCXN_DISABLE;
	timer_ocintpara.ocpolarity	 = TIMER_OC_POLARITY_LOW;
	timer_ocintpara.ocnpolarity	 = TIMER_OCN_POLARITY_LOW;
	timer_ocintpara.ocidlestate	 = TIMER_OC_IDLE_STATE_HIGH;
	timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_HIGH;
	timer_channel_output_config(TIMER13, TIMER_CH_0, &timer_ocintpara);
	timer_channel_output_config(TIMER14, TIMER_CH_0, &timer_ocintpara);

	timer_ocintpara.outputstate	 = TIMER_CCX_DISABLE;
	timer_ocintpara.outputnstate = TIMER_CCXN_ENABLE;
	timer_ocintpara.ocpolarity	 = TIMER_OC_POLARITY_LOW;
	timer_ocintpara.ocnpolarity	 = TIMER_OCN_POLARITY_LOW;
	timer_ocintpara.ocidlestate	 = TIMER_OC_IDLE_STATE_HIGH;
	timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_HIGH;
	timer_channel_output_config(TIMER15, TIMER_CH_0, &timer_ocintpara);
	timer_channel_output_config(TIMER16, TIMER_CH_0, &timer_ocintpara);

	/* PWM0 mode configure channel0 in PWM0 mode */
	timer_channel_output_mode_config(TIMER13, TIMER_CH_0, TIMER_OC_MODE_PWM0);
	timer_channel_output_mode_config(TIMER14, TIMER_CH_0, TIMER_OC_MODE_PWM0);
	timer_channel_output_mode_config(TIMER15, TIMER_CH_0, TIMER_OC_MODE_PWM0);
	timer_channel_output_mode_config(TIMER16, TIMER_CH_0, TIMER_OC_MODE_PWM0);

	timer_channel_output_pulse_value_config(TIMER13, TIMER_CH_0, 128);	// 50% duty
	timer_channel_output_pulse_value_config(TIMER14, TIMER_CH_0, 128);
	timer_channel_output_pulse_value_config(TIMER15, TIMER_CH_0, 255);	// 100% duty
	timer_channel_output_pulse_value_config(TIMER16, TIMER_CH_0, 255);

	timer_channel_output_shadow_config(TIMER13,TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);
	timer_channel_output_shadow_config(TIMER14,TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);
	timer_channel_output_shadow_config(TIMER15,TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);
	timer_channel_output_shadow_config(TIMER16,TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);

	/* enable TIMER primary output */
	timer_primary_output_config(TIMER13, ENABLE);
	timer_primary_output_config(TIMER14, ENABLE);
	timer_primary_output_config(TIMER15, ENABLE);
	timer_primary_output_config(TIMER16, ENABLE);

	timer_auto_reload_shadow_enable(TIMER13);
	timer_auto_reload_shadow_enable(TIMER14);
	timer_auto_reload_shadow_enable(TIMER15);
	timer_auto_reload_shadow_enable(TIMER16);
	
	/* enable TIMER13-16 counter */
	timer_enable(TIMER13);
	timer_enable(TIMER14);
	timer_enable(TIMER15);
	timer_enable(TIMER16);
}

void led_init(void)
{
	_pwm_rcu_config();
	
	_pwm_gpio_config();
	
	_pwm_timer_config();
}

void led_update(void)
{
	static volatile uint8_t blink_state = 0;
	static uint32_t blink_tick = 0;
	uint32_t now = systick_millis();

	if (led_mode) {
		if (now - blink_tick > 300) {
			blink_state ^= 1;
			blink_tick = now;
		}

		timer_channel_output_pulse_value_config(TIMER15, TIMER_CH_0, blink_state ? 0 : led_g);
	}
	else
		timer_channel_output_pulse_value_config(TIMER15, TIMER_CH_0, led_g);

	timer_channel_output_pulse_value_config(TIMER13, TIMER_CH_0, led_r);
	timer_channel_output_pulse_value_config(TIMER16, TIMER_CH_0, led_b);
	timer_channel_output_pulse_value_config(TIMER14, TIMER_CH_0, led_w);
}

void led_set_color(int32_t color, uint32_t brightness)
{
	switch (color) {
		case LED_RED:
			led_r = brightness;
			break;
		case LED_GREEN:
			led_g = brightness;
			break;
		case LED_BLUE:
			led_b = brightness;
			break;
		case LED_WHITE:
			led_w = brightness;
			break;
		case LED_BLACK:
		default:
			led_r = 0;	  led_g = 0;	led_b = 0;	  led_w = 0;
			break;
	}
}

void led_increase_brightness(int32_t color)
{
	switch (color) {
		case LED_RED:
			led_r += delta;
			if (led_r > 255)		led_r = 255;
			break;
		case LED_GREEN:
			led_g += delta;
			if (led_g > 255)		led_g = 255;
			break;
		case LED_BLUE:
			led_b += delta;
			if (led_b > 255)		led_b = 255;
			break;
		case LED_WHITE:
			led_w += delta;
			if (led_w > 255)		led_w = 255;
			break;
		default:
}

void led_decrease_brightness(int32_t color)
{
	switch (color) {
		case LED_RED:
			if (led_r > delta)		led_r -= delta;
			else					led_r = 0;
			break;
		case LED_GREEN:
			if (led_g > delta)		led_g -= delta;
			else					led_g = 0;
			break;
		case LED_BLUE:
			if (led_b > delta)		led_b -= delta;
			else					led_b = 0;
			break;
		case LED_WHITE:
			if (led_w > delta)		led_w -= delta;
			else					led_w = 0;
			break;
		default:
}

void button_init(void)
{
	rcu_periph_clock_enable(RCU_GPIOF);
	
	// TODO: confirm pull_up_down
	gpio_mode_set(GPIOF, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_0 | GPIO_PIN_1);
}

#define HYSTERESIS_COUNT 5
void button_poll(void)
{
	static uint8_t last_pf0 = 0;
	static uint8_t last_pf1 = 0;
	static uint8_t hysterisis_pf0 = 0;
	static uint8_t hysterisis_pf1 = 0;
	static uint8_t led_on = 0;

	uint8_t pf0 = gpio_input_bit_get(GPIOF, GPIO_PIN_0);
	uint8_t pf1 = gpio_input_bit_get(GPIOF, GPIO_PIN_1);

	if (last_pf0 == RESET && pf0 == SET) {
		if (hysterisis_pf0 < HYSTERESIS_COUNT)
			hysterisis_pf0++;
	}
	else {
		hysterisis_pf0 = 0;
		last_pf0 = pf0;
	}
	
	if (hysterisis_pf0 == HYSTERESIS_COUNT) {
		if (led_on == 0) {
			led_set_color(LED_RED, 255);
			led_on = 1;
		}
		else {
			led_set_color(LED_RED, 0);
			led_on = 0;
		}

		last_pf0 = pf0;
	}
	
	if (last_pf1 == RESET && pf1 == SET) {
		if (hysterisis_pf1 < HYSTERESIS_COUNT)
			hysterisis_pf1++;
	}
	else {
		hysterisis_pf1 = 0;
		last_pf1 = pf1;
	}

	if (hysterisis_pf1 == HYSTERESIS_COUNT) {
		led_mode ^= 1;
		last_pf1 = pf1;
	}
}

void encoder_init(void)
{
	rcu_periph_clock_enable(RCU_GPIOA);
	gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_6 | GPIO_PIN_7);
}

void encoder_poll(void)
{
	uint8_t a = gpio_input_bit_get(GPIOA, GPIO_PIN_6);
	uint8_t b = gpio_input_bit_get(GPIOA, GPIO_PIN_7);
	uint8_t state = (a << 1) | b;
	static uint8_t last_state = 0;

	if (state != last_state) {
		if ((last_state == 0b00 && state == 0b01) || (last_state == 0b01 && state == 0b11) ||
			(last_state == 0b11 && state == 0b10) || (last_state == 0b10 && state == 0b00))
			led_increase_brightness(LED_BLUE);
		else
			led_decrease_brightness(LED_BLUE);

		last_state = state;
	}
}

#define ADC_INTERRUPT_MODE		1
__IO uint16_t adc_value[2];

void adc_config(void)
{
	/* enable GPIOA clock */
	rcu_periph_clock_enable(RCU_GPIOA);
	/* enable ADC clock */
	rcu_periph_clock_enable(RCU_ADC);
	/* config ADC clock */
	rcu_adc_clock_config(RCU_ADCCK_APB2_DIV6);

	/* config GPIO PA0/PA1 as analog mode */
	gpio_mode_set(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_PIN_0 | GPIO_PIN_1);

	/* ADC data alignment config */
	adc_data_alignment_config(ADC_DATAALIGN_RIGHT);
	/* ADC channel length config */
	adc_channel_length_config(ADC_REGULAR_CHANNEL, 1U);

	/* ADC trigger config */
	adc_external_trigger_source_config(ADC_REGULAR_CHANNEL, ADC_EXTTRIG_REGULAR_NONE);
	/* ADC external trigger config */
	adc_external_trigger_config(ADC_REGULAR_CHANNEL, ENABLE);

	/* enable ADC interface */
	adc_enable();

	systick_delay(10);

	/* ADC calibration and reset calibration */
	adc_calibration_enable();

#if ADC_INTERRUPT_MODE
	/* enable ADC EOC interrupt */
	adc_interrupt_enable(ADC_INT_EOC);

	/* enable ADC interrupt in NVIC */
	NVIC_SetPriority(ADC_CMP_IRQn, 10);
	NVIC_EnableIRQ(ADC_CMP_IRQn); 

	/* ADC software trigger enable */
	adc_software_trigger_enable(ADC_REGULAR_CHANNEL);
#endif
}

void ADC_CMP_IRQHandler(void)
{
	static uint8_t adc_channel = 0;

	if (adc_interrupt_flag_get(ADC_INT_FLAG_EOC)) {
		adc_value[adc_channel] = adc_regular_data_read();
		/* clear interrupt flag */
		adc_interrupt_flag_clear(ADC_INT_FLAG_EOC);

		adc_regular_channel_config(0U, adc_channel, ADC_SAMPLETIME_55POINT5);

		/* ADC software trigger enable */
		adc_software_trigger_enable(ADC_REGULAR_CHANNEL);
	}
	else if (adc_interrupt_flag_get(ADC_INT_FLAG_EOIC)) {
		adc_interrupt_flag_clear(ADC_INT_FLAG_EOIC);
	}
}

uint16_t adc_channel_sample(uint8_t channel)
{
	/* ADC regular channel config */
	adc_regular_channel_config(0U, channel, ADC_SAMPLETIME_55POINT5);
	/* ADC software trigger enable */
	adc_software_trigger_enable(ADC_REGULAR_CHANNEL);

	/* wait the end of conversion flag */
	while(!adc_flag_get(ADC_FLAG_EOC));
	/* clear the end of conversion flag */
	adc_flag_clear(ADC_FLAG_EOC);
	/* return regular channel sample value */
	return (adc_regular_data_read());
}

int main(void)
{
	systick_config();

	adc_config();
	
	button_init();

	encoder_init();

	led_init();

	while (1) {
#if !ADC_INTERRUPT_MODE
		adc_value[0] = adc_channel_sample(ADC_CHANNEL_0);
		adc_value[1] = adc_channel_sample(ADC_CHANNEL_1);
#endif

		button_poll();

		encoder_poll();
		
		led_update();
	}
}
