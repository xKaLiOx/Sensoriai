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

#define STACK_SIZE 2048
#define THREAD_PRIORITY 5

#define BUF_SIZE 64 //BT message size

struct moving_average
{
	uint8_t counter;
	uint16_t impulse_sum;
	uint16_t buffer[BUFFER_SIZE];
} MA_FILTER;
enum indicator_value {NANO, MICRO} Indication_value = NANO;
char indicator_name[2][10] = {"nSv/h", "uSv/h"};

struct k_thread my_thread_data;
struct k_work_q my_workqueue_data;
struct k_work my_work;

struct bt_update_payload {
	float radiation;
	enum indicator_value indication;
};
static struct k_work bt_update_work;

volatile uint16_t Impulse_counter = 0;
volatile float radiation = 0;
volatile bool BT_connected;

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

