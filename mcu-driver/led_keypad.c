/*
 * (c) 2023 Yunjae Lim <launius@gmail.com>
 *
 * Microcontroller Driver for LED control and Keypad/Rotary inputs
 * using SPI trasmit and ADC channel inputs
 *
 */

#include "gd32f3x0.h"
#include "string.h"
#include "scheduler.h"

#define ARRAYSIZE                (16+63*8+16)

__IO uint32_t adc_value[7];

const char Compiler_Date[] = __DATE__;
const char Compiler_Time[] = __TIME__;

const uint8_t RED[] =          {0x98,0xC6,0xFF,0xFF,0x00,0x00,0x00,0x00};
const uint8_t RED_DIMMED[] =   {0x84,0x21,0x0F,0xFF,0x00,0x00,0x00,0x00};
const uint8_t GREEN[] =        {0x98,0xC6,0x00,0x00,0xFF,0xFF,0x00,0x00};
const uint8_t GREEN_DIMMED[] = {0x84,0x21,0x00,0x00,0x0F,0xFF,0x00,0x00};
const uint8_t BLUE[] =         {0x98,0xC6,0x00,0x00,0x00,0x00,0xFF,0xFF};
const uint8_t BLUE_DIMMED[] =  {0x84,0x21,0x00,0x00,0x00,0x00,0x0F,0xFF};
const uint8_t WHITE[] =        {0x98,0xC6,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
const uint8_t WHITE_DIMMED[] = {0x84,0x21,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF};
const uint8_t TEAL[] =         {0x98,0xC6,0x00,0x00,0x80,0x00,0x80,0x00};
const uint8_t PLUM[] =         {0x98,0xC6,0x4B,0x00,0x00,0x00,0x82,0x00};
const uint8_t BLACK[] =        {0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

uint8_t spi0_send_array[ARRAYSIZE] = \
{0,0,0,0,0,0,0,0,\
0,0,0,0,0,0,0,0,\
0x98,0xC6,0xFF,0xFF,0x00,0x00,0x00,0x00,\
0x98,0xC6,0x00,0x00,0xFF,0xFF,0x00,0x00,\
0x98,0xC6,0x00,0x00,0x00,0x00,0xFF,0xFF,\
0x98,0xC6,0xFF,0xFF,0x00,0x00,0x00,0x00,\
0x98,0xC6,0x00,0x00,0xFF,0xFF,0x00,0x00,\
0x98,0xC6,0x00,0x00,0x00,0x00,0xFF,0xFF,\
0x98,0xC6,0xFF,0xFF,0x00,0x00,0x00,0x00,\
0x98,0xC6,0x00,0x00,0xFF,0xFF,0x00,0x00,\
0x98,0xC6,0x00,0x00,0x00,0x00,0xFF,0xFF,\
0x98,0xC6,0xFF,0xFF,0x00,0x00,0x00,0x00,\
0x98,0xC6,0x00,0x00,0xFF,0xFF,0x00,0x00,\
0x98,0xC6,0x00,0x00,0x00,0x00,0xFF,0xFF,\
0x98,0xC6,0xFF,0xFF,0x00,0x00,0x00,0x00,\
0x84,0x21,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,\
0x84,0x21,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,\
0x84,0x21,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,\
0x84,0x21,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,\
0x84,0x21,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,\
0x84,0x21,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,\
0x84,0x21,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,\
0x84,0x21,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,\
0x84,0x21,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,\
0x84,0x21,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,\
0x84,0x21,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,\
0x84,0x21,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,\
0x84,0x21,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,\
0x84,0x21,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,\
0x84,0x21,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,\
0x84,0x21,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,\
0x84,0x21,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,\
0x84,0x21,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,\
0x84,0x21,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,\
0x84,0x21,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,\
0x84,0x21,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,\
0x84,0x21,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,\
0x84,0x21,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,\
0x84,0x21,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,\
0x84,0x21,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,\
0x84,0x21,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,\
0x84,0x21,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,\
0x84,0x21,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,\
0x84,0x21,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,\
0x84,0x21,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,\
0x84,0x21,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,\
0x84,0x21,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,\
0x84,0x21,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,\
0x84,0x21,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,\
0x84,0x21,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,\
0x84,0x21,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,\
0x84,0x21,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,\
0x84,0x21,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,\
0x84,0x21,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,\
0x84,0x21,0x00,0x00,0xFF,0xFF,0x00,0x00,\
0x84,0x21,0x00,0x00,0x00,0x00,0xFF,0xFF,\
0x84,0x21,0xFF,0xFF,0x00,0x00,0x00,0x00,\
0x84,0x21,0x00,0x00,0xFF,0xFF,0x00,0x00,\
0x84,0x21,0x00,0x00,0x00,0x00,0xFF,0xFF,\
0x84,0x21,0xFF,0xFF,0x00,0x00,0x00,0x00,\
0x84,0x21,0x00,0x00,0xFF,0xFF,0x00,0x00,\
0x84,0x21,0x00,0x00,0x00,0x00,0xFF,0xFF,\
0x84,0x21,0x00,0x00,0x00,0x00,0xFF,0xFF,\
0x84,0x21,0xFF,0xFF,0x00,0x00,0x00,0x00,\
0x84,0x21,0x00,0x00,0xFF,0xFF,0x00,0x00,\
0,0,0,0,0,0,0,0,\
0,0,0,0,0,0,0,0\
};

#define I2C0_SLAVE_ADDRESS7 0xA2

uint8_t spi0_receive_array[ARRAYSIZE];

uint32_t send_n = 0, receive_n = 0;

uint32_t crc_value1 = 0, crc_value2 = 0;

enum led_number
{
    LED_R1C1 = 0,   LED_R1C2,   LED_R1C3,   LED_R1C4,   LED_R1C5,   LED_R1C6,   LED_R1C7,
    LED_R1C8,       LED_R2C8,   LED_R3C8,   LED_R4C8,
    LED_R1C9,       LED_R4C9,

    LED79_2 = 13,   LED78_2,    LED77_2,    LED76_2,    LED75_2,    LED74_2,
    LED73_2,        LED72_2,    LED71_2,    LED70_2,    LED69_2,    LED68_2,
    LED67_2,        LED66_2,    LED65_2,    LED64_2,    LED63_2,    LED62_2,

    LED2_3 = 31,    LED2_4,    LED2_5,

    LED79_1 = 34,   LED78_1,    LED77_1,    LED76_1,    LED75_1,    LED74_1,
    LED73_1,        LED72_1,    LED71_1,    LED70_1,    LED69_1,    LED68_1,
    LED67_1,        LED66_1,    LED65_1,    LED64_1,    LED63_1,    LED62_1,

    LED_R3C7 = 52,  LED_R3C6,   LED_R3C5,   LED_R3C4,   LED_R3C3,   LED_R3C2,   LED_R3C1,
    LED_R5C1,

    LED2_1 = 60,    LED2_2,

    LED_MAX
};

static uint8_t led_enc1 = LED62_1;
static uint8_t led_enc2 = LED62_2;

void clock_enable(void);

void spi_config(void);
void spi_trasmit(void);

void i2c_config(void);

void adc_config(void);
uint16_t adc_channel_sample(uint8_t channel);
void adc_value_set_callback(uint8_t num, uint32_t value);

void set_led(uint8_t led_number, uint8_t high_low_brightness);
void set_led_encoder(uint8_t enc_number, int8_t direction);
void set_led_color(uint8_t led_number, const uint8_t *color);
void set_led_default(void);

void clock_enable(void)
{
    /* enable the led clock */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
}

int led_keypad_init(void)
{
    clock_enable();

    systick_config();

    spi_config();

    /* configure I2C */
    i2c_config();

    while(send_n < ARRAYSIZE) {
        while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_TBE)) {}
        spi_i2s_data_transmit(SPI0, spi0_send_array[send_n++]);
    }
    send_n = 0;

    adc_config();

    return 0;
}

int led_keypad_test(void)
{
    bool R3C1 = FALSE;
    bool R3C2 = FALSE;
    bool R3C3 = FALSE;
    bool R5C1 = FALSE;
    
    bool ENC1_SW = FALSE;
    bool ENC2_SW = FALSE;

    while(1) {
        adc_value[0] = adc_channel_sample(ADC_CHANNEL_0);
        adc_value[1] = adc_channel_sample(ADC_CHANNEL_1);
        adc_value[2] = adc_channel_sample(ADC_CHANNEL_2);
        adc_value[3] = adc_channel_sample(ADC_CHANNEL_3);
        adc_value[4] = adc_channel_sample(ADC_CHANNEL_4);
        adc_value[5] = adc_channel_sample(ADC_CHANNEL_5);
        adc_value[6] = adc_channel_sample(ADC_CHANNEL_6);

        set_led_default();

        /* insert 100 ms delay */
        systick_delay(100);

        // ADC Channel 0
        if      ((adc_value[0] > 2700-50)&&(adc_value[0] < 2700+50))  {}
        else if ((adc_value[0] > 3300-50)&&(adc_value[0] < 3300+50))  { set_led_color(LED_R1C1, WHITE); }
        else if ((adc_value[0] > 3700-50)&&(adc_value[0] < 3700+50))  { R3C1 = !R3C1; }
        else if ((adc_value[0] > 3900-50)&&(adc_value[0] < 3900+50))  { R5C1 = !R5C1; }
        else                                                          {}

        // ADC Channel 1
        if      ((adc_value[1] > 2700-50)&&(adc_value[1] < 2700+50))  { ENC1_SW = !ENC1_SW; }
        else if ((adc_value[1] > 3300-50)&&(adc_value[1] < 3300+50))  { ENC2_SW = !ENC2_SW; }
        else if ((adc_value[1] > 3700-50)&&(adc_value[1] < 3700+50))  { set_led_color(LED_R4C9, WHITE); }
        else if ((adc_value[1] > 3900-50)&&(adc_value[1] < 3900+50))  { set_led_color(LED_R1C9, WHITE); }
        else                                                          {}

        // ADC Channel 2
        if      ((adc_value[2] > 2700-50)&&(adc_value[2] < 2700+50))  { R3C2 = !R3C2; }
        else if ((adc_value[2] > 3300-50)&&(adc_value[2] < 3300+50))  { R3C3 = !R3C3; }
        else if ((adc_value[2] > 3700-50)&&(adc_value[2] < 3700+50))  { set_led_color(LED_R1C3, WHITE); }
        else if ((adc_value[2] > 3900-50)&&(adc_value[2] < 3900+50))  { set_led_color(LED_R1C2, WHITE); }
        else                                                          {}

        // ADC Channel 3
        if      ((adc_value[3] > 2700-50)&&(adc_value[3] < 2700+50))  { set_led_color(LED_R3C5, WHITE); }
        else if ((adc_value[3] > 3300-50)&&(adc_value[3] < 3300+50))  { set_led_color(LED_R3C4, WHITE); }
        else if ((adc_value[3] > 3700-50)&&(adc_value[3] < 3700+50))  { set_led_color(LED_R1C5, WHITE); }
        else if ((adc_value[3] > 3900-50)&&(adc_value[3] < 3900+50))  { set_led_color(LED_R1C4, WHITE); }
        else                                                          {}

        // ADC Channel 4
        if      ((adc_value[4] > 2700-50)&&(adc_value[4] < 2700+50))  { set_led_color(LED_R3C6, WHITE); }
        else if ((adc_value[4] > 3300-50)&&(adc_value[4] < 3300+50))  { set_led_color(LED_R3C7, WHITE); }
        else if ((adc_value[4] > 3700-50)&&(adc_value[4] < 3700+50))  { set_led_color(LED_R1C6, WHITE); }
        else if ((adc_value[4] > 3900-50)&&(adc_value[4] < 3900+50))  { set_led_color(LED_R1C7, WHITE); }
        else                                                          {}

        // ADC Channel 5
        if      ((adc_value[5] > 2700-50)&&(adc_value[5] < 2700+50))  { set_led_color(LED_R1C8, WHITE); }
        else if ((adc_value[5] > 3300-50)&&(adc_value[5] < 3300+50))  { set_led_color(LED_R2C8, WHITE); }
        else if ((adc_value[5] > 3700-50)&&(adc_value[5] < 3700+50))  { set_led_color(LED_R3C8, WHITE); }
        else if ((adc_value[5] > 3900-50)&&(adc_value[5] < 3900+50))  { set_led_color(LED_R4C8, WHITE); }
        else                                                          {}

        // ADC Channel 0
        if(R3C1)    memcpy(&(spi0_send_array[16+58*8]), BLUE, sizeof(BLUE));
        else        memcpy(&(spi0_send_array[16+58*8]), RED, sizeof(RED));
        if(R5C1)    memcpy(&(spi0_send_array[16+59*8]), RED_DIMMED, sizeof(RED_DIMMED));
        else        memcpy(&(spi0_send_array[16+59*8]), RED, sizeof(RED));

        // ADC Channel 1
        if(ENC1_SW) memcpy(&(spi0_send_array[16+8*LED2_3]), WHITE, sizeof(WHITE));
        else        memcpy(&(spi0_send_array[16+8*LED2_3]), WHITE_DIMMED, sizeof(WHITE_DIMMED));
        if(ENC2_SW) memcpy(&(spi0_send_array[16+8*LED2_5]), WHITE, sizeof(WHITE));
        else        memcpy(&(spi0_send_array[16+8*LED2_5]), WHITE_DIMMED, sizeof(WHITE_DIMMED));

        // ADC Channel 2
        if(R3C2)    memcpy(&(spi0_send_array[16+57*8]), BLUE_DIMMED, sizeof(BLUE_DIMMED));
        else        memcpy(&(spi0_send_array[16+57*8]), BLUE, sizeof(BLUE));
        if(R3C3)    memcpy(&(spi0_send_array[16+56*8]), RED_DIMMED, sizeof(RED_DIMMED));
        else        memcpy(&(spi0_send_array[16+56*8]), RED, sizeof(RED));

        spi_trasmit();
    }
}

void spi_config(void)
{
    spi_parameter_struct spi_init_struct;

    rcu_periph_clock_enable(RCU_SPI0);

    /* configure SPI0 GPIO: SCK/PB3, MISO/PB4, MOSI/PB5 */
    gpio_af_set(GPIOB, GPIO_AF_0, GPIO_PIN_3 | GPIO_PIN_5);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_3 | GPIO_PIN_5);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_3 | GPIO_PIN_5);

    /* deinitialize SPI and the parameters */
    spi_i2s_deinit(SPI0);
    spi_struct_para_init(&spi_init_struct);

    /* configure SPI0 parameter */
    spi_init_struct.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.device_mode          = SPI_MASTER;
    spi_init_struct.frame_size           = SPI_FRAMESIZE_8BIT;
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_HIGH_PH_2EDGE;
    spi_init_struct.nss                  = SPI_NSS_SOFT;
    spi_init_struct.prescale             = SPI_PSC_32;
    spi_init_struct.endian               = SPI_ENDIAN_MSB;

    spi_init(SPI0, &spi_init_struct);

    /* enable SPI */
    spi_enable(SPI0);
}

void spi_trasmit(void)
{
    systick_delay(100);

    while(send_n < ARRAYSIZE)
    {
        while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_TBE)) {}
        spi_i2s_data_transmit(SPI0, spi0_send_array[send_n++]);
    }
    send_n = 0;
}

void adc_config(void)
{
    /* enable ADC clock */
    rcu_periph_clock_enable(RCU_ADC);
    /* config ADC clock */
    rcu_adc_clock_config(RCU_ADCCK_APB2_DIV6);

    /* config the GPIO as analog mode */
    gpio_mode_set(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6);

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

    systick_delay(1U);

    /* ADC calibration and reset calibration */
    adc_calibration_enable();
}

uint16_t adc_channel_sample(uint8_t channel)
{
    /* ADC regular channel config */
    adc_regular_channel_config(0U, channel, ADC_SAMPLETIME_7POINT5);
    /* ADC software trigger enable */
    adc_software_trigger_enable(ADC_REGULAR_CHANNEL);

    /* wait the end of conversion flag */
    while(!adc_flag_get(ADC_FLAG_EOC));
    /* clear the end of conversion flag */
    adc_flag_clear(ADC_FLAG_EOC);
    /* return regular channel sample value */
    return (adc_regular_data_read());
}

void adc_value_set_callback(uint8_t num, uint32_t value)
{
    adc_value[num] = value;
}

void i2c_config(void)
{
    /* enable I2C0 clock */
    rcu_periph_clock_enable(RCU_I2C0);

    /* connect PB6 to I2C0_SCL */
    gpio_af_set(GPIOB, GPIO_AF_1, GPIO_PIN_6);
    /* connect PB7 to I2C0_SDA */
    gpio_af_set(GPIOB, GPIO_AF_1, GPIO_PIN_7);
    /* configure GPIO pins of I2C0 */
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_6);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_6);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_7);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_7);

    /* configure I2C clock */
    i2c_clock_config(I2C0, 100000, I2C_DTCY_2);
    /* configure I2C address */
    i2c_mode_addr_config(I2C0, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, I2C0_SLAVE_ADDRESS7);
    /* enable I2C0 */
    i2c_enable(I2C0);
    /* enable acknowledge */
    i2c_ack_config(I2C0, I2C_ACK_ENABLE);
}

void set_led(uint8_t led_number, uint8_t high_low_brightness)
{
    if (high_low_brightness == 0) {
        spi0_send_array[16+8*led_number+2] = 0x00;
        spi0_send_array[16+8*led_number+2+1] = 0x00;
        spi0_send_array[16+8*led_number+2+2] = 0x00;
        spi0_send_array[16+8*led_number+2+3] = 0x00;
        spi0_send_array[16+8*led_number+2+4] = 0x00;
        spi0_send_array[16+8*led_number+2+5] = 0x00;
    }
    else {
        spi0_send_array[16+8*led_number+2] = 0xFF;
        spi0_send_array[16+8*led_number+2+1] = 0xFF;
        spi0_send_array[16+8*led_number+2+2] = 0xFF;
        spi0_send_array[16+8*led_number+2+3] = 0xFF;
        spi0_send_array[16+8*led_number+2+4] = 0x00;
        spi0_send_array[16+8*led_number+2+5] = 0x00;
    }
}

void set_led_encoder(uint8_t enc_number, int8_t direction)
{
    if (enc_number == 1 && direction == 1) {
        memcpy(&(spi0_send_array[16+8*led_enc1]), WHITE, sizeof(WHITE));

        if (led_enc1 > LED79_1)
            led_enc1--;
    }
    else if (enc_number == 1 && direction == -1) {
        //TODO: ignored first ccw operation

        memcpy(&(spi0_send_array[16+8*led_enc1]), WHITE_DIMMED, sizeof(WHITE_DIMMED));

        if (led_enc1 < LED62_1)
            led_enc1++;
    }
    else if (enc_number == 2 && direction == 1) {
        memcpy(&(spi0_send_array[16+8*led_enc2]), WHITE, sizeof(WHITE));

        if (led_enc2 > LED79_2)
            led_enc2--;
    }
    else if (enc_number == 2 && direction == -1) {
        memcpy(&(spi0_send_array[16+8*led_enc2]), WHITE_DIMMED, sizeof(WHITE_DIMMED));

        if (led_enc2 < LED62_2)
            led_enc2++;
    }
}

void set_led_color(uint8_t led_number, const uint8_t *color)
{
    uint32_t i;
    for (i = 0 ; i < 6 ; i++)
        spi0_send_array[16+8*led_number+2+i] = color[i];
}

void set_led_default(void)
{
    uint32_t i;
    for (i = 0 ; i < LED79_2 ; i++)
        memcpy(&(spi0_send_array[16+8*i]), WHITE_DIMMED, sizeof(WHITE_DIMMED));

    for (i = LED_R3C7 ; i < LED_MAX ; i++)
        memcpy(&(spi0_send_array[16+8*i]), WHITE_DIMMED, sizeof(WHITE_DIMMED));
}
