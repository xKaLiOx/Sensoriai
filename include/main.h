//Pagrindines konstantos ir funkciju deklaravimas kad neuztersti main.c

#define p1_coefficient 0.155197599188893
#define p2_coefficient 1.988361091448442
#define p3_coefficient 12.051176069965416
#define p4_coefficient 17.646137314648263 //17.6221373 offset

#define SLEEP_TIME_MS 1
#define REFRESH_RATE 1000 // every 1s
#define TIMER_FREQ_HZ 16000000
#define BUFFER_SIZE 30
#define IMPULSE_THRESHOLD_HIGH 10

#define STACK_SIZE 4096
#define THREAD_PRIORITY 6

#define BUF_SIZE 64 //BT message size

#define RAD_CHAR_ATTR_INDEX 2
#define IDX_CHAR_ATTR_INDEX 5

struct moving_average
{
	uint8_t counter;
	uint16_t impulse_sum;
	uint16_t buffer[BUFFER_SIZE];
} MA_FILTER;
enum indicator_value {NANO, MICRO} Indication_value = NANO;
char indicator_name[2][10] = {"nSv/h", "uSv/h"};

struct k_thread my_thread_data= {0};
struct k_work_q my_workqueue_data= {0};
struct k_work my_work= {0};

struct bt_update_payload {
	float radiation;
	enum indicator_value indication;
};
static struct k_work bt_update_work = {0};

volatile uint16_t Impulse_counter = 0;
volatile float radiation = 0;
volatile bool BT_connected = false;
volatile bool notif_enabled_val = false;    // for radiation value
volatile bool notif_enabled_format = false; // for radiation format

void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
float Impulses_to_uRoentgenPer30Second(uint16_t impulse_count);
float uRoentgenPer30SecondTouSievertsPerHour(float radiation);

void timer21_int_callback(nrf_timer_event_t event_type, void *p_context);
static bool timer21_init(void);
void calculation_thread(void *arg1, void *arg2, void *arg3);
void work_function(struct k_work *work);
uint8_t initialise_gpio_timer_button();
struct k_sem semaphore_update_timer;

static void connected(struct bt_conn *conn, uint8_t err);
static void disconnected(struct bt_conn *conn, uint8_t err);
static void bt_update_handler(struct k_work *work);
void bt_param_updates (struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout);
bool bt_param_requirements (struct bt_conn *conn, struct bt_le_conn_param *param);
static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason);
static void pairing_complete(struct bt_conn *conn, bool bonded);
static void auth_cancel(struct bt_conn *conn);
static bool send_value_bt(float value, enum indicator_value indication);
static int bt_ready(void);
static ssize_t bt_read_radiation_value(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr,
                              void *buf, uint16_t len,
                              uint16_t offset);
static ssize_t bt_read_radiation_format(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr,
                              void *buf, uint16_t len,
                              uint16_t offset);
static void myRadiation_value_ccc_cfg_changed(const struct bt_gatt_attr *attr,uint16_t value);
static void myRadiation_format_ccc_cfg_changed(const struct bt_gatt_attr *attr,uint16_t value);

