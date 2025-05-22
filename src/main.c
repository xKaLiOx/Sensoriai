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

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <soc.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>
#include <inttypes.h>
#include <zephyr/irq.h>
#include <nrfx_timer.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <bluetooth/services/nsms.h>
#include <bluetooth/services/lbs.h>

#include <zephyr/settings/settings.h>

#include "main.h"
// for calculating

K_THREAD_STACK_DEFINE(my_thread_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(my_workqueue_stack, STACK_SIZE);

/*
 * Get button configuration from the devicetree sw0 alias. This is mandatory.
 */
#define DEVICE_NAME             CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN         (sizeof(DEVICE_NAME) - 1)
#define SW0_NODE DT_ALIAS(sw0)

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0});
static const struct gpio_dt_spec buzzer = GPIO_DT_SPEC_GET(DT_N_NODELABEL_my_buzzer_external, buzzer_gpios);
static struct gpio_callback button_cb_data;

static struct gpio_dt_spec led = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios, {0});
static struct gpio_dt_spec led_BT_conn_status = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led3), gpios, {0}); // BT run LED
static const nrfx_timer_t my_timer = NRFX_TIMER_INSTANCE(21);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_LBS_VAL),
};

static struct bt_lbs_cb lbs_callbacks = {
	.led_cb = app_led_cb,
};


BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected        = connected,
	.disconnected     = disconnected,
#ifdef CONFIG_BT_LBS_SECURITY_ENABLED
	.security_changed = security_changed,
#endif
};

//static struct bt_conn_auth_cb conn_auth_callbacks;
//static struct bt_conn_auth_info_cb conn_auth_info_callbacks;

int main(void)
{
	k_work_queue_start(&my_workqueue_data,my_workqueue_stack,K_THREAD_STACK_SIZEOF(my_workqueue_stack),6,NULL);
	k_work_init(&my_work, work_function);
	printk("Workqueue started\n");

	k_sem_init(&semaphore_update_timer,0,1);
	k_thread_create(&my_thread_data, my_thread_stack,
                    K_THREAD_STACK_SIZEOF(my_thread_stack),
                    calculation_thread,
                    NULL, NULL, NULL,
                    THREAD_PRIORITY, 0, K_NO_WAIT);
					
	for(uint8_t x = 0; x<BUFFER_SIZE;x++) // Clear buffer
	{
		MA_FILTER.buffer[x] = 0;
	}
	int ret;

	ret = initialise_gpio_timer_button();
	if(ret != 1)
	{
		printk("ERROR");
		while(1)
		{
		};
	}
	
			if (led_BT_conn_status.port && !gpio_is_ready_dt(&led_BT_conn_status))
	{
		printk("Error %d: led_BT_conn_status device %s is not ready; ignoring it\n",
			   ret, led_BT_conn_status.port->name);
		led_BT_conn_status.port = NULL;
	}
		if (led_BT_conn_status.port)
	{
		ret = gpio_pin_configure_dt(&led_BT_conn_status, GPIO_OUTPUT);
		if (ret != 0)
		{
			printk("Error %d: failed to configure LED device %s pin %d\n",
				   ret, led_BT_conn_status.port->name, led_BT_conn_status.pin);
			led_BT_conn_status.port = NULL;
		}
		else
		{
			printk("Set up LED at %s pin %d\n", led_BT_conn_status.port->name, led_BT_conn_status.pin);
		}
	}
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

	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);
	printk("Set up button at %s pin %d\n", button.port->name, button.pin);

	//Start buzzer for 1s
	gpio_pin_set_dt(&buzzer, 1);
	k_msleep(1000);
	gpio_pin_set_dt(&buzzer, 0);

	timer21_init();

	int err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}
	printk("Bluetooth initialized\n");

/* 		if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	} */
		err = bt_lbs_init(&lbs_callbacks);
	if (err) {
		printk("Failed to init LBS (err:%d)\n", err);
		return 0;
	}

		err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad),
			      sd, ARRAY_SIZE(sd));
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return 0;
	}

	printk("Advertising successfully started\n");

	if (led.port)
	{
		while (1)
		{
			k_sleep(K_MSEC(1000)); //rtos, kad superloop neblokuotu giju
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
		MA_FILTER.impulse_sum += Impulse_counter;
		MA_FILTER.buffer[MA_FILTER.counter] = Impulse_counter;
		MA_FILTER.counter++;
		MA_FILTER.counter %=BUFFER_SIZE;
		if(Impulse_counter > IMPULSE_THRESHOLD_HIGH) // jei daugiau nei 10 per sekunde impulsu, rodomi momentiniai skaiciavimai, kitu atveju rodomas slenkancio vidurkio atsakymas
		{
			for(uint8_t x = 0; x<BUFFER_SIZE;x++)
			{
				MA_FILTER.buffer[x] = Impulse_counter;
			}
			MA_FILTER.impulse_sum = BUFFER_SIZE*Impulse_counter;
		}
		k_work_submit_to_queue(&my_workqueue_data,&my_work);
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



float Impulses_to_uRoentgenPer30Second(uint16_t impulse_count) // grafikas scaled
{
	uint16_t xmean = 1248;
	double stdev = 610.8, X_NORM, radiation;
	X_NORM = ((impulse_count - xmean) * 1.0) / stdev;
	radiation = p1_coefficient * X_NORM * X_NORM * X_NORM + p2_coefficient * X_NORM*X_NORM + p3_coefficient*X_NORM +p4_coefficient;
	return radiation;
	//buffer 30s, tai gauname mikrorentgena per 30s
}
float uRoentgenPer30SecondTouSievertsPerHour(float radiation)
{
	return radiation*2*60/115;
}

void calculation_thread(void *arg1, void *arg2, void *arg3)
{
    printk("Calculation thread started\r\n");
    while (1) {
		k_sem_take(&semaphore_update_timer, K_FOREVER);//LAUKIAMA semaforo is taimerio, po to atliekami skaiciavimai siame


		static uint32_t timestamp;
		timestamp = k_cycle_get_32();
		radiation = uRoentgenPer30SecondTouSievertsPerHour(Impulses_to_uRoentgenPer30Second(MA_FILTER.impulse_sum)); // per 30 sekundziu gauta radiacija
		if(radiation < 1)// konvertuojame i  nanosievertus
		{
			Indication_value = NANO;
			radiation = radiation*1000;
		}
		else
		{
			Indication_value = MICRO;
		}
		int int_part = (int)radiation;
		int frac_part = (int)((radiation - int_part) * (float)10.0);
		if(Indication_value == MICRO)
		{
			printk("tim callback, radiation(MICRO) = %d.%1d at %u\r\n", int_part, frac_part, timestamp);
		}
		else
		{
			printk("tim callback, radiation(NANO) = %d.%1d at %u\r\n", int_part, frac_part, timestamp);
		}

		Impulse_counter = 0;
		MA_FILTER.impulse_sum -= MA_FILTER.buffer[MA_FILTER.counter];//slankusis, priekyje esancia verte atimti, taciau po to, kai atlikti skaiciavimai, nes tada paimsim tik 4
		
	    if(Impulse_counter > 10) // kai per sekunde daugiau nei 10 impulsu
		{
			gpio_pin_set_dt(&buzzer,1); // kaukia software buzzer
		}
		else gpio_pin_set_dt(&buzzer,0);
    }
}

void work_function(struct k_work *work)
{
	k_sem_give(&semaphore_update_timer);//WORKQUEUE gija duoda semafora, jog kita gija galetu atlikti skaiciavimus, daroma sitaip, nes per ISR negalima iskviesti kitos gijos (0 prioritetas)
}

uint8_t initialise_gpio_timer_button()
{
	uint8_t ret = 1;
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
	return 1;
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed, err 0x%02x %s\n", err, bt_hci_err_to_str(err));
		return;
	}

	printk("Connected\n");

	gpio_pin_set(led_BT_conn_status.port,led_BT_conn_status.pin,GPIO_ACTIVE_LOW);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected, reason 0x%02x %s\n", reason, bt_hci_err_to_str(reason));

	gpio_pin_set(led_BT_conn_status.port,led_BT_conn_status.pin,GPIO_ACTIVE_HIGH);
}

static void app_led_cb(bool val)
{
	printk("SMTH happening\n");
}