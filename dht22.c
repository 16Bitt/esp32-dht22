#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "rom/ets_sys.h"
#include "driver/gpio.h"

#define TAG                     CONFIG_DHT22_TAG
#define DHT22_GPIO              CONFIG_DHT22_GPIO
#define HOST_DIAL_LO_DURATION   CONFIG_DHT22_HOST_DIAL_LO_DURATION
#define HOST_DIAL_HI_DURATION   CONFIG_DHT22_HOST_DIAL_HI_DURATION
#define QUEUE_READ_MAX_TICKS    CONFIG_DHT22_QUEUE_READ_TIMEOUT
#define SENSOR_READY_DURATION   CONFIG_DHT22_SENSOR_READY_DURATION
#define BIT_START_DURATION      CONFIG_DHT22_SENSOR_BIT_START_DURATION
#define BIT_MAX_DURATION        CONFIG_DHT22_SENSOR_BIT_MAX_DURATION
#define BIT_POSITIVE_THRESHOLD  CONFIG_DHT22_BIT_POSITIVE_THRESHOLD

/*
 * 82 is the number of edge transitions that the sensor should write to the
 * GPIO. Ideally the ESP can keep up with the queue, but in practice we need
 * enough space to store all 82 intervals -- decrease at your own risk.
 */
#define ISR_QUEUE_SIZE 82

typedef struct {
  uint8_t checksum : 8;
  uint8_t temp_lo : 8;
  uint8_t temp_hi : 8;
  uint8_t humidity_lo : 8;
  uint8_t humidity_hi : 8;
  unsigned int ignored : 24;
} dht22_datapoint;

typedef union {
  uint64_t bitstream;
  dht22_datapoint data;
} dht22_bitstream;

static QueueHandle_t dht22_event_queue = NULL;
static bool dht22_isr_disabled = true;
static int64_t dht22_last_transition_at;

static void dht22_dial(void);
static void IRAM_ATTR dht22_gpio_isr(void *data);
static void dht22_update_sensor_data(dht22_datapoint *data, float *humidity, float *temp, bool *valid_checksum);
static float dht22_bits_to_float(uint8_t hi, uint8_t lo);
static int32_t dht22_read_timing(int32_t max_threshold);
static bool dht22_read_failed(int32_t bit_no, int32_t result);
static void dht22_gpio_set_write(void);

void dht22_initialize(void) {
  dht22_event_queue = xQueueCreate(ISR_QUEUE_SIZE, sizeof(int32_t));

  ESP_ERROR_CHECK(gpio_set_intr_type(DHT22_GPIO, GPIO_INTR_ANYEDGE));
  ESP_ERROR_CHECK(gpio_pullup_en(DHT22_GPIO));
  ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1));
  ESP_ERROR_CHECK(gpio_isr_handler_add(DHT22_GPIO, dht22_gpio_isr, NULL));
  ESP_ERROR_CHECK(gpio_intr_enable(DHT22_GPIO));
}

void dht22_delay(void) {
  vTaskDelay((CONFIG_DHT22_DELAY * 1000) / portTICK_PERIOD_MS);
}

int dht22_read(float *humidity, float *temp, bool *valid_checksum) {
  int32_t duration;
  dht22_bitstream stream;

  dht22_dial();

  /* Sensor pulls high for ~80us */
  duration = dht22_read_timing(SENSOR_READY_DURATION);
  if (dht22_read_failed(0, duration)) {
    return -1;
  }

  /* Sensor pulls low for ~80us */
  duration = dht22_read_timing(SENSOR_READY_DURATION);
  if (dht22_read_failed(1, duration)) {
    return -1;
  }

  for (int i = 0; i < 40; i++) {
    /*
     * Sensor pulls signal high for ~50us to signal start of bit
     */
    duration = dht22_read_timing(BIT_START_DURATION);
    if (dht22_read_failed(2 + i * 2, duration)) {
      return -1;
    }

    /*
     * Sensor pulls signal high for up to ~70us to indicate 1 or 0
     */
    duration = dht22_read_timing(BIT_MAX_DURATION);
    if (dht22_read_failed(3 + i * 2, duration)) {
      return -1;
    }

    if (duration > BIT_POSITIVE_THRESHOLD) {
      stream.bitstream = (stream.bitstream << 1) | 1;
    } else {
      stream.bitstream = stream.bitstream << 1;
    }
  }

  dht22_gpio_set_write();
  dht22_update_sensor_data(&stream.data, humidity, temp, valid_checksum);

  return 0;
}

/*
 * Process GPIO interrupts on any edge for the DHT22. Once an interrupt is
 * received, the time between the last edge transition and the current ISR is
 * pushed onto a queue for processing later.
 */
static void IRAM_ATTR dht22_gpio_isr(void *data) {
  /*
   * Toggling interrupts on and off through the normal gpio_intr_* functions is
   * too slow for this use case, so a simple alleviates the overhead of
   * changing ISR/GPIO settings.
   */
  if (dht22_isr_disabled) {
    return;
  }

  /*
   * NOTE: i64 values used for times, but the individual timing intervals
   * should always be small enough to fit in 32 bits, so we can save a bit of
   * space with i32s for the queue.
   */
  int64_t now = esp_timer_get_time();
  int32_t isr_time = now - dht22_last_transition_at;
  dht22_last_transition_at = now;

  xQueueSendFromISR(dht22_event_queue, &isr_time, NULL);
}

/*
 * Disable interrupts and set GPIO mode to write
 */
static void dht22_gpio_set_write(void) {
  dht22_isr_disabled = true;
  gpio_set_direction(DHT22_GPIO, GPIO_MODE_OUTPUT);
}

/*
 * Signal transaction start with DHT22.
 *
 * Steps:
 * - ESP pulls signal low for ~2000us
 * - ESP pulls signal high for ~30us
 * - Switch GPIO control over to the sensor
 * - Enable ISR
 *
 * After this step is complete, the DHT22 queue should start filling with the
 * transition timings for each signal.
 */
static void dht22_dial(void) {
  ESP_LOGI(TAG, "start dial");

  dht22_gpio_set_write();

  /* Clear the queue to capture a new stream of timings */
  xQueueReset(dht22_event_queue);

  /* Host sets signal low for 1-3ms */
  gpio_set_level(DHT22_GPIO, 0);
  ets_delay_us(HOST_DIAL_LO_DURATION);

  /* Host sets signal high for 20-40us */
  gpio_set_level(DHT22_GPIO, 1);
  ets_delay_us(HOST_DIAL_HI_DURATION);

  /*
   * Reset timestamp of last bit. This will be used to capture the initial
   * timing of the sensor response in the ISR.
   */
  dht22_last_transition_at = esp_timer_get_time();

  /*
   * Prepare for read from sensor. Enable interrupts on GPIO, the sensor should
   * start the data handshake in ~80us.
   */
  gpio_set_direction(DHT22_GPIO, GPIO_MODE_INPUT);
  dht22_isr_disabled = false;
}

/*
 * Read a timing interval from the interval queue.
 *
 * Returns 0 when queue is not filled within QUEUE_READ_MAX_TICKS.
 * Returns negative number of microseconds that were missed when data exceeds
 *  max_threshold
 */
static int32_t dht22_read_timing(int32_t max_threshold) {
  int32_t time;

  if (xQueueReceive(dht22_event_queue, &time, QUEUE_READ_MAX_TICKS)) {
    if (time > max_threshold) {
      return -1 * (time - max_threshold);
    }

    return time;
  }

  return 0;
}

/*
 * Check timing response from last read. If data is invalid, log, disable DHT22
 * interrupts, and return true.
 */
static bool dht22_read_failed(int32_t bit_no, int32_t result) {
  if (result > 0) {
    return false;
  }

  dht22_gpio_set_write();
  if (result == 0) {
    ESP_LOGE(TAG, "failed to read sensor transition %d: no response", bit_no);
  } else {
    ESP_LOGE(TAG, "failed to read sensor transition %d: exceeded timeout by %d", bit_no, result * -1);
  }

  return true;
}

/*
 * Translate two bytes read from DHT22 into a float
 */
static float dht22_bits_to_float(uint8_t hi, uint8_t lo) {
  return ((hi << 8) | lo) / 10.0;
}

/*
 * Update static sensor data, transforming the bitstream into values we can
 * work with in C.
 */
static void dht22_update_sensor_data(dht22_datapoint *data, float *humidity, float *temp, bool *valid_checksum) {
  *temp = dht22_bits_to_float(data->temp_hi, data->temp_lo);
  *humidity = dht22_bits_to_float(data->humidity_hi, data->humidity_lo);

  uint8_t calc_checksum = data->temp_hi + data->temp_lo + data->humidity_hi + data->humidity_lo;
  *valid_checksum = calc_checksum == data->checksum;

  if (*valid_checksum) {
    ESP_LOGI(TAG, "read T=%f H=%f", *temp, *humidity);
  } else {
    ESP_LOGE(TAG, "checksum failed!");
  }
}

