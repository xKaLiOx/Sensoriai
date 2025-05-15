/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * NOTE: If you are looking into an implementation of button events with
 * debouncing, check out `input` subsystem and `samples/subsys/input/input_dump`
 * example instead.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>
#include <inttypes.h>
#include <zephyr/irq.h>

#include <nrfx_timer.h>

// for calculating
#define p1_coefficient 1.8718
#define p2_coefficient 12.3130
#define p3_coefficient 17.6750

#define SLEEP_TIME_MS 1
#define REFRESH_RATE 1000 // every 1s
#define TIMER_FREQ_HZ 16000000
#define BUFFER_SIZE 5 

uint16_t Impulse_counter = 0;
volatile float radiation = 0;
volatile float radiation_circular[BUFFER_SIZE] = {0}; //Ciklinis buferis kai reiksmes mazai skiriasi, kai atsiranda didelis pokytis tarp imciu, rodyt momentini
volatile uint8_t buffer_cnt = 0;


void timer21_int_callback(nrf_timer_event_t event_type, void *p_context);
static bool timer21_init(void);
void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
float Impulses_to_uRoentgenPerSecond(uint16_t impulse_count);
float uRoentgenPerSecondTouSievertsPerHour(float radiation);

struct moving_average
{
	uint8_t counter;
	float sum;
	float buffer[BUFFER_SIZE];
} MA_FILTER;

/*
 * Get button configuration from the devicetree sw0 alias. This is mandatory.
 */
#define SW0_NODE DT_ALIAS(sw0)

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0});
static const struct gpio_dt_spec buzzer = GPIO_DT_SPEC_GET(DT_N_NODELABEL_my_buzzer_external, buzzer_gpios);

static struct gpio_callback button_cb_data;
static const nrfx_timer_t my_timer = NRFX_TIMER_INSTANCE(21);
static struct gpio_dt_spec led = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios, {0});

int main(void)
{
	for(uint8_t x = 0; x<BUFFER_SIZE;x++) // Clear buffer
	{
		MA_FILTER.buffer[x] = 0;
	}
	int ret;


	if (!gpio_is_ready_dt(&button))
	{
		printk("Error: button device %s is not ready\n",
			   button.port->name);
		return 0;
	}
	if (!device_is_ready(buzzer.port))
	{
		printk("Buzzer GPIO device not ready\n");
		return 0;
	}
	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0)
	{
		printk("Error configuring button pin\n");
		return 0;
	}
	ret = gpio_pin_configure_dt(&buzzer, GPIO_OUTPUT_INACTIVE);
	if (ret != 0)
	{
		printk("Error configuring buzzer pin\n");
		return 0;
	}
	if (ret != 0)
	{
		printk("Error %d: failed to configure %s pin %d\n",
			   ret, button.port->name, button.pin);
		return 0;
	}

	ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0)
	{
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			   ret, button.port->name, button.pin);
		return 0;
	}

	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);
	printk("Set up button at %s pin %d\n", button.port->name, button.pin);

	if (led.port && !gpio_is_ready_dt(&led))
	{
		printk("Error %d: LED device %s is not ready; ignoring it\n",
			   ret, led.port->name);
		led.port = NULL;
	}
	if (led.port)
	{
		ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT);
		if (ret != 0)
		{
			printk("Error %d: failed to configure LED device %s pin %d\n",
				   ret, led.port->name, led.pin);
			led.port = NULL;
		}
		else
		{
			printk("Set up LED at %s pin %d\n", led.port->name, led.pin);
		}
	}
	//Start buzzer for 1s
	gpio_pin_set_dt(&buzzer, 1);
	k_msleep(1000);
	gpio_pin_set_dt(&buzzer, 0);

	timer21_init();

	if (led.port)
	{
		while (1)
		{
		}
	}
	return 0;
}

void button_pressed(const struct device *dev, struct gpio_callback *cb,
					uint32_t pins)
{
	Impulse_counter++;
	printk("Button pressed, current counter value: %d \r\n", Impulse_counter);
	gpio_pin_toggle(led.port, led.pin);
}

void timer21_int_callback(nrf_timer_event_t event_type, void *p_context)
{
	switch (event_type)
	{
	case NRF_TIMER_EVENT_COMPARE0:
 		radiation = uRoentgenPerSecondTouSievertsPerHour(Impulses_to_uRoentgenPerSecond(Impulse_counter));
		MA_FILTER.sum += radiation;
		MA_FILTER.buffer[MA_FILTER.counter] = radiation;
		MA_FILTER.counter++;
		MA_FILTER.counter %=BUFFER_SIZE;
		if(radiation > 200) // jei daugiau, rodomi impulsai, kitu atveju rodomas slenkancio vidurkio atsakymas
		{
			for(uint8_t x = 0; x<BUFFER_SIZE;x++)
			{
				MA_FILTER.buffer[x] = radiation;
			}
			MA_FILTER.sum = BUFFER_SIZE*radiation;
		}
		static uint32_t timestamp;
		timestamp = k_cycle_get_32();
		radiation = MA_FILTER.sum/BUFFER_SIZE;
		int int_part = (int)radiation;
		int frac_part = (int)((radiation - int_part) * 10.0);
		printk("tim %d callback, radiation = %d.%1d at %u\r\n",buffer_cnt, int_part, frac_part, timestamp);
		Impulse_counter = 0;
		MA_FILTER.sum -= MA_FILTER.buffer[MA_FILTER.counter];//slankusis, priekyje esancia verte atimti, taciau po to, kai atlikti skaiciavimai, nes tada paimsim tik 4
		
		if(radiation > 500)
		{
			gpio_pin_set_dt(&buzzer,1); // kaukia software buzzer
		}
		else gpio_pin_set_dt(&buzzer,0);
		break;

	default:
		break;
	}
}


bool timer21_init(void)
{
	uint32_t time_ticks;
	nrfx_err_t err;
	nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG(1000000);
	timer_cfg.mode = NRF_TIMER_MODE_TIMER;
	timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
	timer_cfg.interrupt_priority = 2;

	err = nrfx_timer_init(&my_timer, &timer_cfg, timer21_int_callback);
	if (err == NRFX_ERROR_ALREADY_INITIALIZED)
	{
		printk("timer21 already initialized, continuing.\n");
	}
	else if (err != NRFX_SUCCESS)
	{
		printk("Error initializing timer21: %x\n", err);
		return false; // Don't continue if timer didn't initialize
	}

	IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_TIMER21),
				2, // Priority (same as your timer config)
				nrfx_timer_21_irq_handler,
				NULL,
				0);
	irq_enable(NRFX_IRQ_NUMBER_GET(NRF_TIMER21));

	time_ticks = nrfx_timer_ms_to_ticks(&my_timer, 1000);
	nrfx_timer_extended_compare(
		&my_timer,
		NRF_TIMER_CC_CHANNEL0,
		time_ticks,
		NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
		true);
	nrfx_timer_enable(&my_timer);
	return true;
}



float Impulses_to_uRoentgenPerSecond(uint16_t impulse_count) // grafikas scaled
{
	uint16_t xmean = 1248;
	double stdev = 610.8, X_NORM, radiation;
	X_NORM = ((impulse_count - xmean) * 1.0) / stdev;
	radiation = p1_coefficient * X_NORM * X_NORM + p2_coefficient * X_NORM + p3_coefficient;
	return radiation;
}
float uRoentgenPerSecondTouSievertsPerHour(float radiation)
{
	return radiation * 32;
}