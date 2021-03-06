#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dac.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/spi.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "uart.h"
#include "mcp492x.h"

#define M1_ENC (4096-512)
#define M2_ENC (4096-512)
#define M3_ENC (4096-512)

volatile uint32_t system_millis;

void sys_tick_handler(void)
{
	system_millis++;
}

void msleep(uint32_t delay)
{
	uint32_t wake = system_millis + delay;
	while (wake > system_millis);
}

static void systick_setup(void)
{
	systick_set_reload(4800);
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	systick_counter_enable();
	systick_interrupt_enable();
}

static void clock_setup(void)
{
	rcc_clock_setup_in_hsi_out_48mhz();
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_USART1);
	rcc_periph_clock_enable(RCC_SPI1);
}

uint8_t channel_array[] = { 2, 2, ADC_CHANNEL_TEMP};
static void adc_setup(void)
{
	rcc_periph_clock_enable(RCC_ADC);
	rcc_periph_clock_enable(RCC_GPIOA);

	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO2);
	//gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);

	adc_power_off(ADC1);
	adc_set_clk_source(ADC1, ADC_CLKSOURCE_ADC);
	adc_calibrate(ADC1);
	adc_set_operation_mode(ADC1, ADC_MODE_SCAN);
	adc_disable_external_trigger_regular(ADC1);
	adc_set_right_aligned(ADC1);
	adc_enable_temperature_sensor();
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPTIME_071DOT5);
	adc_set_regular_sequence(ADC1, 1, channel_array);
	adc_set_resolution(ADC1, ADC_RESOLUTION_12BIT);
	adc_disable_analog_watchdog(ADC1);
	adc_power_on(ADC1);

	/* Wait for ADC starting up. */
	int i;
	for (i = 0; i < 800000; i++) {    /* Wait a bit. */
		__asm__("nop");
	}

}

static void dac_setup(){
	rcc_periph_clock_enable(RCC_DAC);
	rcc_periph_clock_enable(RCC_GPIOA);

	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO4);
	gpio_set_af(GPIOA, GPIO_AF1, GPIO4);
	dac_disable(CHANNEL_1);
	dac_disable_waveform_generation(CHANNEL_1);
	dac_enable(CHANNEL_1);
	dac_set_trigger_source(DAC_CR_TSEL1_SW);
}

void set_dac(uint16_t data){
	dac_load_data_buffer_single(data, RIGHT12, CHANNEL_1);
	dac_software_trigger(CHANNEL_1);
}

static void gpio_setup(void)
{
	/* Setup GPIO pin GPIO13/14/15 on GPIO port C for LEDs. */
	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO13 | GPIO14 | GPIO15);

#ifdef MCU1
	/* Setup GPIO pin GPIO10/11 on GPIO port A for Buttons. */
	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO10 | GPIO11);
#elif MCU2
	/* Setup GPIO pin GPIO2/3 on GPIO port B for Buttons. */
	gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO2 | GPIO3);
#endif

	/* Setup GPIO pins for USART1 transmit. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);

	/* Setup USART1 TX & RX pin as alternate function. */
	gpio_set_af(GPIOA, GPIO_AF0, GPIO2 | GPIO3 );

	/*Setup SPI1 pins */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO5 | GPIO6 | GPIO7);
	gpio_set_af(GPIOA, GPIO_AF0, GPIO5 | GPIO6 | GPIO7);

	/*init B10=LDAC, B11=DACCS1, B12=DACCS2 B13=DACCS3 */
	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LDAC | DACCS1 | DACCS2 | DACCS3  );
	gpio_set(GPIOB, DACCS1 | DACCS2 | DACCS3);
	gpio_clear(GPIOB, LDAC );
	//gpio_set(GPIOB, LDAC );
}

static void spi_setup(){
	spi_init_master(SPI1, SPI_CR1_BR_FPCLK_DIV_64, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
			SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_MSBFIRST);
	spi_set_clock_phase_0(SPI1);
	spi_set_clock_polarity_0(SPI1);
	spi_set_data_size(SPI1, SPI_CR2_DS_16BIT);
	spi_fifo_reception_threshold_16bit(SPI1);
	spi_enable_software_slave_management(SPI1);
	spi_set_unidirectional_mode(SPI1);
	spi_set_full_duplex_mode(SPI1);
	//spi_disable_ss_output(SPI1);
	spi_set_nss_high(SPI1);
	spi_enable(SPI1);
}

static void tim_setup(void)
{
	//TIM1_CH1 = A8, TIM1_CH2 = A9 AF2
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO8 | GPIO9);
	gpio_set_af(GPIOA, GPIO_AF2, GPIO8 | GPIO9 );

	//TIM2_CH1 = A0, TIM2_CH2 = A1 AF2
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO0 | GPIO1);
	gpio_set_af(GPIOA, GPIO_AF2, GPIO0 | GPIO1 );

	//TIM3_CH1 = B4, TIM2_CH2 = B5 AF1
	rcc_periph_clock_enable(RCC_GPIOB);
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO4 | GPIO5);
	gpio_set_af(GPIOB, GPIO_AF1, GPIO4 | GPIO5 );


	rcc_periph_clock_enable(RCC_TIM1);
	timer_set_period(TIM1, M1_ENC);
	timer_slave_set_mode(TIM1, 0x3); //encoder
	timer_ic_set_input(TIM1, TIM_IC1, TIM_IC_IN_TI1);
	timer_ic_set_input(TIM1, TIM_IC2, TIM_IC_IN_TI2);
	timer_enable_counter(TIM1);

	rcc_periph_clock_enable(RCC_TIM2);
	timer_set_period(TIM2, M2_ENC);
	timer_slave_set_mode(TIM2, 0x3); //encoder
	timer_ic_set_input(TIM2, TIM_IC1, TIM_IC_IN_TI1);
	timer_ic_set_input(TIM2, TIM_IC2, TIM_IC_IN_TI2);
	timer_enable_counter(TIM2);

	rcc_periph_clock_enable(RCC_TIM3);
	timer_set_period(TIM3, M3_ENC);
	timer_slave_set_mode(TIM3, 0x3); //encoder
	timer_ic_set_input(TIM3, TIM_IC1, TIM_IC_IN_TI1);
	timer_ic_set_input(TIM3, TIM_IC2, TIM_IC_IN_TI2);
	timer_enable_counter(TIM3);

}

uint16_t current_to_dac(float curr){
	if(curr < 0) curr = 0;
	if(curr > 10) curr = 10;
	float v = 4.75 - 0.4747 * curr;
	return (uint16_t)((v / 5.0f) * 4096.0f);
}

void set_current(int chip, float curr){
	dac_write(chip, CH_I, current_to_dac(curr));
}

uint16_t vout_to_dac(float v){
	/*
	 * V+ = 5v * 6.8k/(10k + 6.8k) = 2.024
	 * (Vout - V+)/47k = (V+ - Vdac)/10k
	 * VOut = 5.7*V+ - 4.7Vdac
	 * VDac = (5.7*V+ - VOut)/4.7
	 * 5.7*V+ = 11.54
	 */
	if(v < -11.96) v = -11.96;
	if(v > 11.54) v = 11.54;
	float dac = (11.54 - v)/4.7;
	return (uint16_t)((dac/5.0f)*4095.0f);
}

void set_v(int chip, float v){
	dac_write(chip, CH_V, vout_to_dac(v));
}

float angle_to_radian(float angle){
	return angle/180.0*3.14;
}

float radian_to_angle(float rad){
	return rad/3.14*180.0;
}

#pragma pack(push, 1)
typedef struct {
	uint16_t sync1;
	uint16_t sync2;

	uint16_t tim1;
	uint16_t tim2;
	uint16_t tim3;
} mcu_msg_t;

typedef struct {
	uint16_t sync;

	float dac1;
	float dac2;
	float dac3;

	float curr1;
	float curr2;
	float curr3;

	uint16_t stop;
} zenom_msg_t;

typedef struct{
	uint16_t sync;

	uint16_t enc1;
	uint16_t enc2;
	uint16_t enc3;
} enc_msg_t;
#pragma pack(pop)

enc_msg_t enc_msg = {0};
zenom_msg_t zenom_msg = {0};

int main(void)
{
	mcu_msg_t mcu_msg;
	mcu_msg.sync1 = 0xABCD;
	mcu_msg.sync2 = 0xEFAA;
	enc_msg.sync = 0xACEF;
	zenom_msg.sync = 0xABCD;
	zenom_msg.stop = 1;

	clock_setup();
	systick_setup();
	gpio_setup();
	usart_setup(460800*2);
	//adc_setup();
	//dac_setup();
	tim_setup();
	spi_setup();

	//linear amplifier initial current and voltage setup
	dac_write(CHIP1, CH_V, vout_to_dac(0));
	dac_write(CHIP2, CH_V, vout_to_dac(0));
	dac_write(CHIP3, CH_V, vout_to_dac(0));
	set_current(CHIP1, 0);
	set_current(CHIP2, 0);
	set_current(CHIP3, 0);

	uint8_t buff[64] = {0};
	uint32_t last_mcu_msg_time = 0;

	while(1)
	{
		if(zenom_msg.stop == 0){

			mcu_msg.tim1 = timer_get_counter(TIM1);
			mcu_msg.tim2 = timer_get_counter(TIM2);
			mcu_msg.tim3 = timer_get_counter(TIM3);

			_write(1, (char *)&mcu_msg, sizeof(mcu_msg_t));
			gpio_toggle(GPIOC, GPIO14);
		}

		last_mcu_msg_time = system_millis;

		
		if(zenom_msg.stop == 1){

			set_v(CHIP1, 0);
			set_v(CHIP2, 0);
			set_v(CHIP3, 0);

			set_current(CHIP1, 0);
			set_current(CHIP2, 0);
			set_current(CHIP3, 0);

			gpio_clear(GPIOC, GPIO15);
		}

		while(uart_rx_available()){
			uint8_t sync1 = 0, sync2 = 0;
			sync1 = uart_read_ch();
			sync2 = uart_read_ch();
			if(sync1 == 0xCD && sync2 == 0xAB){
				while(uart_rx_available() < sizeof(zenom_msg_t)-2);
				buff[0] = sync1;
				buff[1] = sync2;
				uart_read(&buff[2], sizeof(zenom_msg_t)-2);
				memcpy(&zenom_msg, buff, sizeof(zenom_msg_t)); 
				gpio_toggle(GPIOC, GPIO13);

				if(zenom_msg.stop == 0){
					set_v(CHIP1, zenom_msg.dac1);
					set_v(CHIP2, zenom_msg.dac2);
					set_v(CHIP3, zenom_msg.dac3);

					set_current(CHIP1, zenom_msg.curr1);
					set_current(CHIP2, zenom_msg.curr2);
					set_current(CHIP3, zenom_msg.curr3);
				}
			}
			if(sync1 == 0xEF && sync2 == 0xAC){
				while(uart_rx_available() < sizeof(enc_msg_t)-2);
				buff[0] = sync1;
				buff[1] = sync2;
				uart_read(&buff[2], sizeof(enc_msg_t)-2);
				memcpy(&enc_msg, buff, sizeof(enc_msg_t)); 

				timer_set_period(TIM1, enc_msg.enc1);
				timer_set_period(TIM2, enc_msg.enc2);
				timer_set_period(TIM3, enc_msg.enc3);
			}
		}


		if(last_mcu_msg_time + 10 - system_millis > 0)
			msleep(last_mcu_msg_time + 10 - system_millis);
	}
}

