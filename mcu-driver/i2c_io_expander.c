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
 * TCA9535 I/O expander pins
	P00: LED_PAD_1  P01: LED_PAD_2  P02: LED_PAD_3
	P03: LED_HPF_1  P04: LED_HPF_2  P05: LED_HPF_3
	P06: LED_POL_1  P07: LED_POL_2

	P10: LED_POL_3  P11: MOS_HPF_1  P12: MOS_HPF_3
	P13: MOS_PAD_1  P14: MOS_PAD_3  P15: MOS_POL_1
	P16: MOS_POL_3  P17: X
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

	// initialize output levels
	i2c_ioexp_write(0x02, 0xff);
	i2c_ioexp_write(0x03, 0xff);
}

void led_mosfet_set_state(button_t btn, int32_t state) {
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
	uint8_t port0 = ~(io_state & 0xFF);         // P00-P07
	uint8_t port1 = ~((io_state >> 8) & 0xFF);  // P10-P17

	i2c_ioexp_write(0x02, port0);
	i2c_ioexp_write(0x03, port1);
}

/* GD32F3x0 pins for button inputs */
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
