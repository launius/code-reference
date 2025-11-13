/*
 * Copyright (c) 2025 Yunjae Lim <launius@gmail.com>
 *
 * GD32F3x0 MCU code snippet for I/O expander, TI TCA9535 compatible,
 * which is connected via I2C, controls LEDs and MOSFETs
 *
 * 1. I2C Interface Definition
 *
	Byte               |7 MSB|  6  |  5  |  4  |  3  |  2  |  1  |0 LSB|
	--------------------------------------------------------------------
	I2C target address |  L  |  H  |  L  |  L  | A2  | A1  | A0  | R/W |
	P0x I/O data bus   | P07 | P06 | P05 | P04 | P03 | P02 | P01 | P00 |
	P1x I/O data bus   | P17 | P16 | P15 | P14 | P13 | P12 | P11 | P10 |
	--------------------------------------------------------------------
 *
 * 2. TCA9535 Address Reference (if A2 = A1 = A0 = GND, 0x20)
 *
	| Target Address            |R/W|
	| 0 | 1 | 0 | 0 |A2 |A1 |A0 |   |
	|Fixed          |Programmable|

	------------------------------------
	|A2 |A1 |A0 | I2C Bus Target Address
	------------------------------------
	| L | L | L | 0x20
	| L | L | H | 0x21
	| L | H | L | 0x22
	| H | L | L | 0x23
	...
 *
 * 3. Control Register Bits
 *
	---------------------------------
	| 0 | 0 | 0 | 0 | 0 |B2 |B1 |B0 |
	---------------------------------

	--------------------------------------------------
	|B2 |B1 |B0 | Command Byte | Register
	--------------------------------------------------
	...
	| 0 | 1 | 0 | 0x02         | Output Port 0
	| 0 | 1 | 1 | 0x03         | Output Port 1
	...
	| 1 | 1 | 0 | 0x06         | Configuration Port 0
	| 1 | 1 | 1 | 0x07         | Configuration Port 1
	--------------------------------------------------
 *
 * 4. Registers 6 and 7 (Configuration Registers):
 *  configure the directions of the I/O pins (1: input, 0: output)
 *
	--------------------------------------------------------
	Bit    |C0.7 |C0.6 |C0.5 |C0.4 |C0.3 |C0.2 |C0.1 |C0.0 |
	Default|  1  |  1  |  1  |  1  |  1  |  1  |  1  |  1  |
	Bit    |C1.7 |C1.6 |C1.5 |C1.4 |C1.3 |C1.2 |C1.1 |C1.0 |
	Default|  1  |  1  |  1  |  1  |  1  |  1  |  1  |  1  |
	--------------------------------------------------------
 *
 * 5. Registers 2 and 3 (Output Port Registers):
 *  outgoing logic levels of the pins defined as outputs by the Configuration register
 *
	--------------------------------------------------------
	Bit    |O0.7 |O0.6 |O0.5 |O0.4 |O0.3 |O0.2 |O0.1 |O0.0 |
	Default|  1  |  1  |  1  |  1  |  1  |  1  |  1  |  1  |
	Bit    |O1.7 |O1.6 |O1.5 |O1.4 |O1.3 |O1.2 |O1.1 |O1.0 |
	Default|  1  |  1  |  1  |  1  |  1  |  1  |  1  |  1  |
	--------------------------------------------------------
 * 
 * Reference: TCA9535 datasheet
 */

#define I2C_IOEXP_SCL_PIN		GPIO_PIN_9
#define I2C_IOEXP_SCL_PORT		GPIOA
#define I2C_IOEXP_SDA_PIN		GPIO_PIN_10
#define I2C_IOEXP_SDA_PORT		GPIOA

#define I2C0_SLAVE_ADDRESS7		0x20

void i2c_ioexp_init(void)
{
	rcu_periph_clock_enable(RCU_GPIOA);
	rcu_periph_clock_enable(RCU_I2C0);

	gpio_af_set(I2C_IOEXP_SCL_PORT, GPIO_AF_4, I2C_IOEXP_SCL_PIN);
	gpio_af_set(I2C_IOEXP_SDA_PORT, GPIO_AF_4, I2C_IOEXP_SDA_PIN);

	gpio_mode_set(I2C_IOEXP_SCL_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, I2C_IOEXP_SCL_PIN);
	gpio_output_options_set(I2C_IOEXP_SCL_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, I2C_IOEXP_SCL_PIN);
	gpio_mode_set(I2C_IOEXP_SDA_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, I2C_IOEXP_SDA_PIN);
	gpio_output_options_set(I2C_IOEXP_SDA_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, I2C_IOEXP_SDA_PIN);

	i2c_clock_config(I2C0, 100000, I2C_DTCY_2);
	i2c_enable(I2C0);

	// TODO: check if ack is needed
	i2c_ack_config(I2C0, I2C_ACK_ENABLE);
}

void i2c_ioexp_write(uint8_t reg, uint8_t data)
{
	/* wait until I2C bus is idle */
	while(i2c_flag_get(I2C0, I2C_FLAG_I2CBSY));
	/* send a start condition to I2C bus */
	i2c_start_on_bus(I2C0);
	/* wait until SBSEND bit is set */
	while(!i2c_flag_get(I2C0, I2C_FLAG_SBSEND));
	/* send slave address to I2C bus */
	i2c_master_addressing(I2C0, (I2C0_SLAVE_ADDRESS7 << 1), I2C_TRANSMITTER);
	/* wait until ADDSEND bit is set */
	while(!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND));
	/* clear ADDSEND bit */
	i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);
	/* wait until the transmit data buffer is empty */
	while(!i2c_flag_get(I2C0, I2C_FLAG_TBE));

	/* data transmission */
	i2c_data_transmit(I2C0, reg);
	/* wait until the TBE bit is set */
	while(!i2c_flag_get(I2C0, I2C_FLAG_TBE));
	/* data transmission */
	i2c_data_transmit(I2C0, data);
	/* wait until the TBE bit is set */
	while(!i2c_flag_get(I2C0, I2C_FLAG_TBE));

	/* send a stop condition to I2C bus */
	i2c_stop_on_bus(I2C0);
	while(I2C_CTL0(I2C0) & I2C_CTL0_STOP);
}

/*
 * TCA9535 I/O Expander Pin Mapping and Configuration
 * --------------------------------------------------
 *
 * - LEDs require an Active Low signal (set pin to 0 to turn ON).
 * - MOSFETs require an Active High signal (set pin to 1 to turn ON).
 *
 * P0 Port Configuration (All LEDs - Output, Active Low):
	P00: LED_PAD_1
	P01: LED_PAD_2
	P02: LED_PAD_3
	P03: LED_HPF_1
	P04: LED_HPF_2
	P05: LED_HPF_3
	P06: LED_POL_1
	P07: LED_POL_2
 *
 * P1 Port Configuration (Mixed - Output):
	P10: LED_POL_3 (Active Low)
	P11: MOS_HPF_1 (Active High)
	P12: MOS_HPF_3 (Active High)
	P13: MOS_PAD_1 (Active High)
	P14: MOS_PAD_3 (Active High)
	P15: MOS_POL_1 (Active High)
	P16: MOS_POL_3 (Active High)
	P17: X (Not Used)
 */

#define LED_PAD_1   (1 << 0)
#define LED_PAD_2   (1 << 1)
#define LED_PAD_3   (1 << 2)
#define LED_HPF_1   (1 << 3)
#define LED_HPF_2   (1 << 4)
#define LED_HPF_3   (1 << 5)
#define LED_POL_1   (1 << 6)
#define LED_POL_2   (1 << 7)

#define LED_POL_3   (1 << 8)
#define MOS_HPF_1   (1 << 9)
#define MOS_HPF_3   (1 << 10)
#define MOS_PAD_1   (1 << 11)
#define MOS_PAD_3   (1 << 12)
#define MOS_POL_1   (1 << 13)
#define MOS_POL_3   (1 << 14)

#define PAD_BITMASK     (LED_PAD_1 | LED_PAD_2 | LED_PAD_3 | MOS_PAD_1 | MOS_PAD_3)
#define HPF_BITMASK     (LED_HPF_1 | LED_HPF_2 | LED_HPF_3 | MOS_HPF_1 | MOS_HPF_3)
#define POL_BITMASK     (LED_POL_1 | LED_POL_2 | LED_POL_3 | MOS_POL_1 | MOS_POL_3)

typedef enum {
	BTN_PAD,
	BTN_HPF,
	BTN_POL,
} button_t;

typedef enum {
	PAD_1,
	PAD_2,
	PAD_3,
	PAD_MAX
} pad_state_t;

typedef enum {
	HPF_1,
	HPF_2,
	HPF_3,
	HPF_MAX
} hpf_state_t;

typedef enum {
	POL_1,
	POL_2,
	POL_3,
	POL_MAX
} pol_state_t;

pad_state_t pad_state = PAD_1;
hpf_state_t hpf_state = HPF_1;
pol_state_t pol_state = POL_1;

uint16_t io_state = 0x0000;     // bit mask for LED and MOSFET states

void led_mosfet_init(void)
{
	i2c_ioexp_init();

	i2c_ioexp_write(0x06, 0x00);    // config P07-P00 as output
	i2c_ioexp_write(0x07, 0x00);    // config P17-P10 as output

	// LED output levels are inverted. on: 0x00, off: 0xff
	I2C_IOExp_Write(0x02, 0xFF);    // set LED P07-P00 high
	I2C_IOExp_Write(0x03, 0x01);    // set MOSFET P17-P11 low, LED P10 low
	
	// TODO: restore states from NVM
}

void led_mosfet_set_state(button_t btn, int32_t state)
{
	switch (btn) {
		case BTN_PAD:
			pad_state = (pad_state_t)state;
			break;
		case BTN_HPF:
			hpf_state = (hpf_state_t)state;
			break;
		case BTN_POL:
			pol_state = (pol_state_t)state;
			break;
		default:
			break;
	}

	update_led_mosfet();
}

static void handle_button(int32_t btn)
{
	// TODO: do someting based on the state
	if (btn == BTN_PAD) {
		pad_state = (pad_state_t)((pad_state + 1) % PAD_MAX);
	}
	else if (btn == BTN_HPF) {
		hpf_state = (hpf_state_t)((hpf_state + 1) % HPF_MAX);
	}
	else if (btn == BTN_POL) {
		pol_state = (pol_state_t)((pol_state + 1) % POL_MAX);
	}

	update_led_mosfet();
}

static void update_led_mosfet(void)
{
	switch (pad_state) {
		case PAD_1:
			io_state = (uint16_t)((io_state & (uint16_t)~PAD_BITMASK) | LED_PAD_1 | MOS_PAD_1);
			break;
		case PAD_2:
			io_state = (uint16_t)((io_state & (uint16_t)~PAD_BITMASK) | LED_PAD_2);
			break;
		case PAD_3:
			io_state = (uint16_t)((io_state & (uint16_t)~PAD_BITMASK) | LED_PAD_3 | MOS_PAD_3);
			break;
		default:
			break;
	}

	switch (hpf_state) {
		case HPF_1:
			io_state = (uint16_t)((io_state & (uint16_t)~HPF_BITMASK) | LED_HPF_1 | MOS_HPF_1);
			break;
		case HPF_2:
			io_state = (uint16_t)((io_state & (uint16_t)~HPF_BITMASK) | LED_HPF_2);
			break;
		case HPF_3:
			io_state = (uint16_t)((io_state & (uint16_t)~HPF_BITMASK) | LED_HPF_3 | MOS_HPF_3);
			break;
		default:
			break;
	}

	switch (pol_state) {
		case POL_1:
			io_state = (uint16_t)((io_state & (uint16_t)~POL_BITMASK) | LED_POL_1 | MOS_POL_1);
			break;
		case POL_2:
			io_state = (uint16_t)((io_state & (uint16_t)~POL_BITMASK) | LED_POL_2);
			break;
		case POL_3:
			io_state = (uint16_t)((io_state & (uint16_t)~POL_BITMASK) | LED_POL_3 | MOS_POL_3);
			break;
		default:
			break;
	}

	set_led_mosfet_io();
}

static void set_led_mosfet_io(void)
{
	// LED logic is inverted
	uint8_t port0 = ~(io_state & 0xFF);
	uint8_t port1 = ((~((io_state >> 8) & 0x01)) | ((io_state >> 8) & 0xFE));

	__IO uint32_t delay = 100;

	// to prevent flickering, first turn off all outputs
	I2C_IOExp_Write(0x02, 0xFF);    // set P07-P00 high
	I2C_IOExp_Write(0x03, 0x01);    // set P17-P11 low, P10 high

	while (delay--) __NOP();

	i2c_ioexp_write(0x02, port0);
	i2c_ioexp_write(0x03, port1);
	
	// TODO: save io_state in NVM
}

/*
 * GD32F3x0 GPIO Configuration: Button Inputs
 *
 * PA2/PA3/PA4 are configured as input for push buttons.
 *
 * Note: For sleep mode functionality, these pins must be connected 
 * to their corresponding External Interrupt (EXTI) lines, and the 
 * EXTI interrupt must be enabled and configured to wake the system.
 *
 * 1. EXTI sources selection register 0 (SYSCFG_EXTISS0)

	15          12   11           8   7            4   3            0
	-------------------------------------------------------------------
	EXTI3_SS [3:0] | EXTI2_SS [3:0] | EXTI1_SS [3:0] | EXTI0_SS [3:0] |
	-------------------------------------------------------------------

	Bits	Fields			Descriptions
	15:12	EXTI3_SS[3:0]	EXTI 3 sources selection
							X000: PA3 pin
							X001: PB3 pin
							X010: PC3 pin
							...

	11:8	EXTI2_SS[3:0]	EXTI 2 sources selection
							X000: PA2 pin
							...
 *
 * 2. Interrupt vector table

	Interrupt number    Vector number    Peripheral interrupt description
	---------------------------------------------------------------------
	IRQ 5               21               EXTI line0-1 interrupts
	IRQ 6               22               EXTI line2-3 interrupts
	IRQ 7               23               EXTI line4-15 interrupts
 *
 * 3. EXTI source
 
	EXTI Line Number    Source
	-----------------------------------------
	2                   PA2 / PB2 / PC2 / PD2
	3                   PA3 / PB3 / PC3
	4                   PA4 / PB4 / PC4 / PF4
 *
 * Reference: https://www.gigadevice.com.cn/Public/Uploads/uploadfile/files/20240407/GD32F3x0_User_Manual_Rev2.9.pdf
 */

#define PIN_BTN_POL         GPIO_PIN_2
#define PIN_BTN_HPF         GPIO_PIN_3
#define PIN_BTN_PAD         GPIO_PIN_4

void button_init(void)
{
	rcu_periph_clock_enable(RCU_GPIOA);

	gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, PIN_BTN_POL);
	gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, PIN_BTN_HPF);
	gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, PIN_BTN_PAD);
}

void button_poll(void)
{
	static int last_pol = SET;
	static int last_hpf = SET;
	static int last_pad = SET;
	static int debounce_pol = 0;
	static int debounce_hpf = 0;
	static int debounce_pad = 0;

	int pol = gpio_input_bit_get(GPIOA, PIN_BTN_POL);
	int hpf = gpio_input_bit_get(GPIOA, PIN_BTN_HPF);
	int pad = gpio_input_bit_get(GPIOA, PIN_BTN_PAD);

	if (last_pol == SET && pol == RESET) {
		if (debounce_pol < 5)
			debounce_pol++;
		else {
			handle_button(BTN_POL);
//			debounce_pol = 0;
			last_pol = pol;
		}
	}
	else {
		debounce_pol = 0;
		last_pol = pol;
	}

	if (last_hpf == SET && hpf == RESET) {
		if (debounce_hpf < 5)
			debounce_hpf++;
		else {
			handle_button(BTN_HPF);
			last_hpf = hpf;
		}
	}
	else {
		last_hpf = hpf;
	}

	if (last_pad == SET && pad == RESET) {
		if (debounce_pad < 5)
			debounce_pad++;
		else {
			handle_button(BTN_PAD);
			last_pad = pad;
		}
	}
	else {
		last_pad = pad;
	}
}

void button_intrrupt_init(void)
{
	rcu_periph_clock_enable(RCU_GPIOA);
	rcu_periph_clock_enable(RCU_CFGCMP);

	gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, PIN_BTN_POL | PIN_BTN_HPF | PIN_BTN_PAD);

	/* enable and set key EXTI interrupt to the lowest priority */
	nvic_irq_enable(EXTI2_3_IRQn, 2U, 0U);
	nvic_irq_enable(EXTI4_15_IRQn, 2U, 0U);

	/* connect key EXTI line to key GPIO pin */
	syscfg_exti_line_config(EXTI_SOURCE_GPIOA, EXTI_SOURCE_PIN2);
	syscfg_exti_line_config(EXTI_SOURCE_GPIOA, EXTI_SOURCE_PIN3);
	syscfg_exti_line_config(EXTI_SOURCE_GPIOA, EXTI_SOURCE_PIN4);

	/* configure key EXTI line */
	exti_init(EXTI_2, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
	exti_init(EXTI_3, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
	exti_init(EXTI_4, EXTI_INTERRUPT, EXTI_TRIG_FALLING);

	exti_interrupt_flag_clear(EXTI_2);
	exti_interrupt_flag_clear(EXTI_3);
	exti_interrupt_flag_clear(EXTI_4);
}

void EXTI2_3_IRQHandler(void)
{
	if (exti_interrupt_flag_get(EXTI_2) != RESET) {
		handle_button(BTN_POL);
		exti_interrupt_flag_clear(EXTI_2);
	}

	if (exti_interrupt_flag_get(EXTI_3) != RESET) {
		handle_button(BTN_HPF);
		exti_interrupt_flag_clear(EXTI_3);
	}
}

void EXTI4_15_IRQHandler(void)
{
	if (exti_interrupt_flag_get(EXTI_4) != RESET) {
		handle_button(BTN_PAD);
		exti_interrupt_flag_clear(EXTI_4);
	}
}

/*
 * Run Low Power Mode Example:
 *
 * Demonstrates how to initialize peripherals and enter low-power modes
 * using specific driver APIs.
 */
void run_low_power_mode(void)
{
	/* Initialize required peripherals (LEDs and MOSFETs) to a safe default state. */
	led_mosfet_init();

	/* Initialize the button interrupts (EXTI) to act as wake-up sources.
	   This function has to configure the EXTI lines used by button input pins. */
	button_intrrupt_init();

	while (1) {
		/* Sleep Mode:
			In this mode, only CPU clock is off.
			WFI (Wait For Interrupt) instruction is used to enter the mode. */
		// pmu_to_sleepmode(WFI_CMD); 
		
		/* Deep-Sleep Mode:
			All clocks in the 1.2V. Domain are off. Disable IRC8M, IRC28M, IRC48M, HXTAL and PLL.
			Specify PMU_LDO_LOWPOWER mode to reduce regulation current,
			and use the WFI instruction to enter the mode. */
		pmu_to_deepsleepmode(PMU_LDO_LOWPOWER, WFI_CMD);
	}
}
