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
#include <stdlib.h>
#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>
#include <inttypes.h>
#include <zephyr/irq.h>
#include <nrfx_timer.h>
#include <nrfx_log.h>

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

BT_NSMS_DEF(nsms_radiation, "Radiation", false, "Unknown", BUF_SIZE);
BT_NSMS_DEF(nsms_radiation_unit, "Units", false, "nSv/h", 10);

#define BT_UUID_RAD_SVC_VAL BT_UUID_128_ENCODE(0xd49e121c, 0x3893, 0x11f0, 0xa345, 0x325096b39f47) // custom UUID for SVC
#define BT_UUID_RAD_VAL BT_UUID_128_ENCODE(0xd49e121d, 0x3893, 0x11f0, 0xa345, 0x325096b39f47)	   // custom UUID for radiation value
#define BT_UUID_RAD_IDX BT_UUID_128_ENCODE(0xd49e121e, 0x3893, 0x11f0, 0xa345, 0x325096b39f47)	   // custom UUID for radiation index

#define BT_UUID_RAD_SVC BT_UUID_DECLARE_128(BT_UUID_RAD_SVC_VAL)
#define BT_UUID_RAD BT_UUID_DECLARE_128(BT_UUID_RAD_VAL)
#define BT_UUID_IDX BT_UUID_DECLARE_128(BT_UUID_RAD_IDX)

/*
 * Get button configuration from the devicetree sw0 alias. This is mandatory.
 */
#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
#define SW0_NODE DT_ALIAS(sw0)

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.le_param_req = bt_param_requirements,
	.le_param_updated = bt_param_updates

};
static struct bt_conn_auth_cb conn_auth_callbacks = {
	.passkey_display = NULL,
	.passkey_confirm = NULL,
	.cancel = auth_cancel,
	.pairing_confirm = NULL,
};
static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed};

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0});
static const struct gpio_dt_spec buzzer = GPIO_DT_SPEC_GET(DT_N_NODELABEL_external_user, buzzer_gpios);
static const struct gpio_dt_spec impulse_counting = GPIO_DT_SPEC_GET(DT_N_NODELABEL_external_user, impulse_counter_gpios);
static struct gpio_callback button_cb_data0;
static struct gpio_callback button_cb_data1;


static struct gpio_dt_spec led = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios, {0});
static struct gpio_dt_spec led_BT_conn_status = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led3), gpios, {0}); // BT run LED
static const nrfx_timer_t my_timer = NRFX_TIMER_INSTANCE(21);

static struct bt_update_payload bt_payload= {0};;
static struct k_work bt_update_work;
struct bt_conn_info info;
struct bt_conn *my_conn=NULL;

BT_GATT_SERVICE_DEFINE(my_radiation_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_RAD_SVC),
					   BT_GATT_CHARACTERISTIC(BT_UUID_RAD,
											  BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
											  BT_GATT_PERM_READ | BT_GATT_PERM_NONE, bt_read_radiation_value, NULL,
											  &bt_payload.radiation),
					   BT_GATT_CCC(myRadiation_value_ccc_cfg_changed,
								   BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
					   BT_GATT_CHARACTERISTIC(BT_UUID_IDX,
											  BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
											  BT_GATT_PERM_READ | BT_GATT_PERM_NONE, bt_read_radiation_format, NULL,
											  &bt_payload.indication),
					   BT_GATT_CCC(myRadiation_format_ccc_cfg_changed,
								   BT_GATT_PERM_READ | BT_GATT_PERM_WRITE));

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_RAD_SVC_VAL),
};

/* static const struct bt_data ad1[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd1[] = {
	BT_DATA_BYTES(0),
}; */

int main(void)
{
	printk("hello\n");
	k_work_queue_start(&my_workqueue_data, my_workqueue_stack, K_THREAD_STACK_SIZEOF(my_workqueue_stack), 6, NULL);
	k_work_init(&my_work, work_function);
	k_work_init(&bt_update_work, bt_update_handler);
	printk("Workqueue started\n");

	k_sem_init(&semaphore_update_timer, 0, 1);
	k_thread_create(&my_thread_data, my_thread_stack,
					K_THREAD_STACK_SIZEOF(my_thread_stack),
					calculation_thread,
					NULL, NULL, NULL,
					THREAD_PRIORITY, 0, K_NO_WAIT);

	for (uint8_t x = 0; x < BUFFER_SIZE; x++) // Clear buffer
	{
		MA_FILTER.buffer[x] = 0;
	}
	int ret;

	ret = initialise_gpio_timer_button();
	if (ret != 1)
	{
		printk("ERROR");
		while (1)
		{
		};
	}

	gpio_init_callback(&button_cb_data0, button_pressed, BIT(button.pin));
	ret = gpio_add_callback(button.port, &button_cb_data0);
	if(!ret)
	{
		printk("Set up button interrupt at %s pin %d\n", button.port->name, button.pin);
	}
	gpio_init_callback(&button_cb_data1, button_pressed, BIT(impulse_counting.pin));
	ret = gpio_add_callback(impulse_counting.port, &button_cb_data1);
	if(!ret)
	{
		printk("Set up counter interrupt at %s pin %d\n", impulse_counting.port->name, impulse_counting.pin);
	}

	// Start buzzer for 1s
	gpio_pin_set_dt(&buzzer, 1);
	k_msleep(1000);
	gpio_pin_set_dt(&buzzer, 0);
	k_msleep(1000);
	gpio_pin_set_dt(&buzzer, 1);
	k_msleep(1000);
	gpio_pin_set_dt(&buzzer, 0);


	int err = bt_enable(NULL);
	if (err != 0)
	{
		printk("Bluetooth enable failed (err %d)\n", err);
	}

	k_msleep(10);
	
/* 	err = settings_load(); //AAAHHHH THIS ERROR WAS PAIN
	if (err)
	{
		printk("Settings load failed (%d)\n", err);
		return -1;
	} */

	if (bt_ready() != 0)
	{
		printk("Bluetooth configuration failed\n");
	}

	timer21_init();


	while (1)
	{
		k_sleep(K_SECONDS(1));
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
		MA_FILTER.counter %= BUFFER_SIZE;
		if (Impulse_counter > IMPULSE_THRESHOLD_HIGH) // jei daugiau nei 10 per sekunde impulsu, rodomi momentiniai skaiciavimai, kitu atveju rodomas slenkancio vidurkio atsakymas
		{
			for (uint8_t x = 0; x < BUFFER_SIZE; x++)
			{
				MA_FILTER.buffer[x] = Impulse_counter;
			}
			MA_FILTER.impulse_sum = BUFFER_SIZE * Impulse_counter;
		}
		k_work_submit_to_queue(&my_workqueue_data, &my_work);
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
	radiation = p1_coefficient * X_NORM * X_NORM * X_NORM + p2_coefficient * X_NORM * X_NORM + p3_coefficient * X_NORM + p4_coefficient;
	return radiation;
	// buffer 30s, tai gauname mikrorentgena per 30s
}
float uRoentgenPer30SecondTouSievertsPerHour(float radiation)
{
	return radiation * 2 * 60 / 115;
}

void calculation_thread(void *arg1, void *arg2, void *arg3)
{
	printk("Calculation thread started\r\n");
	while (1)
	{
		k_sem_take(&semaphore_update_timer, K_FOREVER); // LAUKIAMA semaforo is taimerio, po to atliekami skaiciavimai siame

		static uint32_t timestamp;
		timestamp = k_cycle_get_32();
		radiation = uRoentgenPer30SecondTouSievertsPerHour(Impulses_to_uRoentgenPer30Second(MA_FILTER.impulse_sum)); // per 30 sekundziu gauta radiacija
		if (radiation < 1)																							 // konvertuojame i  nanosievertus
		{
			Indication_value = NANO;
			radiation = radiation * 1000;
		}
		else
		{
			Indication_value = MICRO;
		}
		int int_part = (int)radiation;
		int frac_part = (int)((radiation - int_part) * (float)10.0);
		if (Indication_value == MICRO)
		{
			printk("tim callback, radiation(MICRO) = %d.%1d at %u\r\n", int_part, frac_part, timestamp);
		}
		else
		{
			printk("tim callback, radiation(NANO) = %d.%1d at %u\r\n", int_part, frac_part, timestamp);
		}
		bt_payload.radiation = radiation;
		bt_payload.indication = Indication_value;
		k_work_submit(&bt_update_work);
		if (Impulse_counter > 10) // kai per sekunde daugiau nei 10 impulsu
		{
			gpio_pin_set_dt(&buzzer, 1); // kaukia software buzzer
		}
		else
			gpio_pin_set_dt(&buzzer, 0);
		Impulse_counter = 0;
		MA_FILTER.impulse_sum -= MA_FILTER.buffer[MA_FILTER.counter]; // slankusis, priekyje esancia verte atimti, taciau po to, kai atlikti skaiciavimai, nes tada paimsim tik 4
	}
}

void work_function(struct k_work *work)
{
	k_sem_give(&semaphore_update_timer); // WORKQUEUE gija duoda semafora, jog kita gija galetu atlikti skaiciavimus, daroma sitaip, nes per ISR negalima iskviesti kitos gijos (0 prioritetas)
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
	if (!device_is_ready(buzzer.port))// all the same port
	{
		printk("Buzzer GPIO device not ready\n");
		return 0;
	}
	ret = gpio_pin_configure_dt(&impulse_counting, GPIO_INPUT);
	if (ret != 0)
	{
		printk("Error configuring counting pin\n");
		return 0;
	}
	ret = gpio_pin_configure_dt(&buzzer, GPIO_OUTPUT_INACTIVE);
	if (ret != 0)
	{
		printk("Error configuring buzzer pin\n");
		return 0;
	}
	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0)
	{
		printk("Error %d: failed to configure button as input on %s pin %d",
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
	ret = gpio_pin_interrupt_configure_dt(&impulse_counting, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0)
	{
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			   ret, button.port->name, button.pin);
		return 0;
	}
	if (led_BT_conn_status.port && !gpio_is_ready_dt(&led_BT_conn_status))
	{
		printk("Error %d: led_BT_conn_status device %s is not ready; ignoring it\n",
			   ret, led_BT_conn_status.port->name);
		led_BT_conn_status.port = NULL;
		return 0;
	}
	if (led_BT_conn_status.port)
	{
		ret = gpio_pin_configure_dt(&led_BT_conn_status, GPIO_OUTPUT);
		if (ret != 0)
		{
			printk("Error %d: failed to configure LED device %s pin %d\n",
				   ret, led_BT_conn_status.port->name, led_BT_conn_status.pin);
			led_BT_conn_status.port = NULL;
			return 0;
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
		return 0;
	}
	if (led.port)
	{
		ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT);
		if (ret != 0)
		{
			printk("Error %d: failed to configure LED device %s pin %d\n",
				   ret, led.port->name, led.pin);
			led.port = NULL;
			return 0;
		}
		else
		{
			printk("Set up LED at %s pin %d\n", led.port->name, led.pin);
		}
	}
	return 1;
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err)
	{
		printk("Connection failed, err 0x%02x %s\n", err, bt_hci_err_to_str(err));
		return;
	}
	printk("Connected\n");
	
    if (my_conn) {
        bt_conn_unref(my_conn);
    }
    my_conn = bt_conn_ref(conn);
	
	err = bt_conn_get_info(conn, &info);
	if (err)
	{
		LOG_ERR("bt_conn_get_info() returned %d", err);
		return;
	}

	gpio_pin_set(led_BT_conn_status.port, led_BT_conn_status.pin, GPIO_ACTIVE_LOW);
	BT_connected = true;
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected, reason 0x%02x %s\n", reason, bt_hci_err_to_str(reason));

	gpio_pin_set(led_BT_conn_status.port, led_BT_conn_status.pin, GPIO_ACTIVE_HIGH);
	BT_connected = false;
	bt_le_adv_stop();
	bt_conn_unref(my_conn);
	my_conn = NULL;
}

static void bt_update_handler(struct k_work *work)
{
	send_value_bt(bt_payload.radiation, bt_payload.indication);
}
bool bt_param_requirements(struct bt_conn *conn, struct bt_le_conn_param *param)
{
	return true;
}
void bt_param_updates(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout)
{
	printk("Param updates: interval:%d, latency:%d, timeout:%d\n", interval, latency, timeout);
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing cancelled\n");
}

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing completed\n");
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (reason == BT_SECURITY_ERR_AUTH_REQUIREMENT)
	{
		bt_unpair(BT_ID_DEFAULT, BT_ADDR_LE_ANY);
		if (bt_ready() != 0)
		{
			printk("Pairing failed, of AUTH requirements");
		}
	}
	printk("Pairing failed, reason %d\n", reason);
}

static bool send_value_bt(float value, enum indicator_value indication)
{
	if (!BT_connected | !my_conn)
	{
		return false;
	}
	char msg[20];
	sprintf(msg, "%1.1f", (double)value);
	int err = 0;
	if (notif_enabled_val)
	{
		if (bt_gatt_is_subscribed(my_conn, &my_radiation_svc.attrs[RAD_CHAR_ATTR_INDEX], BT_GATT_CCC_NOTIFY))
		{
			err = bt_gatt_notify(my_conn, &my_radiation_svc.attrs[RAD_CHAR_ATTR_INDEX], msg, strlen(msg));
			if (err)
			{
				printk("NOTIFY ERROR for format \n");
			}
		}
	}
	k_sleep(K_MSEC(10));
	bt_nsms_set_status(&nsms_radiation, msg);
	k_sleep(K_MSEC(10));
	sprintf(msg, "%s", indicator_name[indication]);
	if (notif_enabled_format)
	{
		if (bt_gatt_is_subscribed(my_conn, &my_radiation_svc.attrs[IDX_CHAR_ATTR_INDEX], BT_GATT_CCC_NOTIFY))
		{
		err = bt_gatt_notify(my_conn, &my_radiation_svc.attrs[IDX_CHAR_ATTR_INDEX], msg, strlen(msg));
		if(err)
		{
			printk("NOTIFY ERROR for format \n");
		}
		}
	}
	k_sleep(K_MSEC(10));
	bt_nsms_set_status(&nsms_radiation_unit, msg);
	return true;
}

static int bt_ready(void)
{
	int err;
	err = bt_conn_auth_cb_register(&conn_auth_callbacks);
	if (err != 0)
	{
		printk("Authentication cb register error (err %d)\n", err);
		return -1;
	}
	err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
	if (err != 0)
	{
		printk("Authentication cb register info error (err %d)\n", err);
		return -1;
	}
	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad),
						  sd, ARRAY_SIZE(sd));
	/* err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, NULL, 0, NULL, 0); */
	if (err != 0)
	{
		printk("Advertising failed to start (err %d)\n", err);
		return -1;
	}
	printk("Bluetooth initialized\n");
	return 0;
}

static ssize_t bt_read_radiation_value(struct bt_conn *conn,
									   const struct bt_gatt_attr *attr,
									   void *buf, uint16_t len,
									   uint16_t offset)
{
	float *value = attr->user_data;
	char str_val[10];
	snprintf(str_val, sizeof(str_val), "%.1f", *value);
	return bt_gatt_attr_read(conn, attr, buf, len, offset, str_val, strlen(str_val));
}
static ssize_t bt_read_radiation_format(struct bt_conn *conn,
										const struct bt_gatt_attr *attr,
										void *buf, uint16_t len,
										uint16_t offset)
{
	enum indicator_value VALUE = *(enum indicator_value *)attr->user_data;
	char buffer[10];
	if (VALUE == NANO)
	{
		sprintf(buffer, "nSv/h");
	}
	else
	{
		sprintf(buffer, "uSv/h");
	}
	return bt_gatt_attr_read(conn, attr, buf, len, offset, buffer, strlen(buffer));
}
static void myRadiation_value_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	notif_enabled_val = (value == BT_GATT_CCC_NOTIFY);
	printk("Radiation notification for value %s\n", notif_enabled_val ? "enabled" : "disabled");
}
static void myRadiation_format_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	notif_enabled_format = (value == BT_GATT_CCC_NOTIFY);
	printk("Radiation notifications for format %s\n", notif_enabled_format ? "enabled" : "disabled");
}