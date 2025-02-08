/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include <esp_timer.h>
#include <time.h>
#include <math.h>
#include "esp_log.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "hal/spi_types.h"
#include "portmacro.h"
#include "xtensa/core-macros.h"
#include "driver/i2c_master.h"
#include "driver/i2c_slave.h"
#include "driver/i2c.h"
#include "soc/rtc_periph.h"

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_SLAVE_NUM  I2C_NUM_1

#define I2C_MASTER_SDA_IO (CONFIG_EXAMPLE_MASTER_SDA)
#define I2C_MASTER_SCL_IO (CONFIG_EXAMPLE_MASTER_SCL)
#define I2C_SLAVE_SDA_IO  (CONFIG_EXAMPLE_SLAVE_SDA)
#define I2C_SLAVE_SCL_IO  (CONFIG_EXAMPLE_SLAVE_SCL)

#define I2C_MASTER_FREQ_HZ 100000
#define I2C_SLAVE_ADDR      0x28
#define I2C_SLAVE_BUF_SIZE  1024

/**
 * This is an example which echos any data it receives on configured UART back to the sender,
 * with hardware flow control turned off. It does not use UART driver event queue.
 *
 * - Port: configured UART
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: off
 * - Pin assignment: see defines below (See Kconfig)
 */

#define OUT_TAG "RESULT_I2C"


#define TASK_STACK_SIZE    (CONFIG_EXAMPLE_TASK_STACK_SIZE)

#define TOTAL_COUNT (CONFIG_EXAMPLE_TOTAL_COUNT)
#define DEADLINE_MS (CONFIG_EXAMPLE_DEADLINE)


#define BUF_SIZE (1024)
#define DATA_SIZE (CONFIG_EXAMPLE_DATA_SIZE) // Size of the data to transmit and receive


SemaphoreHandle_t sync_semaphore;

i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t dev_handle;

QueueHandle_t s_receive_queue;

uint32_t tx_time_list[TOTAL_COUNT];
uint32_t rx_time_list[TOTAL_COUNT];


static const char *TAG = "example";

typedef struct {
    char *json_buffer;
    QueueHandle_t event_queue;
    uint8_t* command_data;
    i2c_slave_dev_handle_t handle;
} i2c_slave_github_context_t;

typedef enum {
    I2C_SLAVE_EVT_RX,
    I2C_SLAVE_EVT_TX
} i2c_slave_event_t;

static bool i2c_slave_request_cb(i2c_slave_dev_handle_t i2c_slave, const i2c_slave_request_event_data_t *evt_data, void *arg)
{
    ESP_LOGI("I2C", "REQUEST CALLBACK INVOKED");
    i2c_slave_github_context_t *context = (i2c_slave_github_context_t *)arg;
    i2c_slave_event_t evt = I2C_SLAVE_EVT_TX;
    BaseType_t xTaskWoken = 0;
    xQueueSendFromISR(context->event_queue, &evt, &xTaskWoken);
    return xTaskWoken;
}


void populate_data_rand(uint8_t* data, uint32_t size){
  for(uint32_t i=0; i<size; i++){
    data[i] = rand() % (1 << 8);
  }
}

void i2c_transmit(uint8_t* data, uint32_t size){
  esp_err_t res = i2c_master_transmit(dev_handle, data, size, DEADLINE_MS * 2 / portTICK_PERIOD_MS);
  //esp_err_t res = i2c_master_write_to_device(I2C_MASTER_NUM, I2C_SLAVE_ADDR, data, size, 1000 / portTICK_PERIOD_MS);
  if(res != ESP_OK){
    ESP_LOGE("I2C", "Error transmitting from master");
    ESP_LOGE("I2C", "Error description: %s", esp_err_to_name(res));
  }
}

void tx_task(void *arg) {
    uint8_t* out_data = (uint8_t*) heap_caps_malloc(DATA_SIZE * sizeof(uint8_t), MALLOC_CAP_DMA);
    if (out_data == NULL) {
      ESP_LOGE("SPI", "Failed to allocate memory for out_data");
      // Handle allocation failure
    }

    for(unsigned int i=0; i< TOTAL_COUNT; i++) {
        populate_data_rand(out_data, DATA_SIZE);
        //ESP_LOGD("UART", "After populate");
        int64_t start_time = esp_timer_get_time();
        //ESP_LOGI("I2C", "Pre transmit");
        i2c_transmit(out_data, DATA_SIZE);
        //ESP_LOGI("I2C", "Post transmit");
        //ESP_LOGD("UART", "Transmit::Time taken: lld microseconds");
        int64_t end_time = esp_timer_get_time();
        //ESP_LOGD("UART", "Transmit::Time taken: lld microseconds");
        int64_t duration = end_time - start_time;
        //ESP_LOGD("UART", "Transmit::Time taken: lld microseconds");
        tx_time_list[i] = duration;
        ESP_LOGD("UART", "Transmit::Time taken: %lld microseconds", duration);
//        vTaskDelay(DEADLINE_MS / portTICK_PERIOD_MS); // in SPI only the master must have a delay, introducing a delay on the slave side causes sync errors in SPI.
        //ESP_LOGD("UART", "After delay");
    }

    ESP_LOGI("SPI", "TX DONE");

    xSemaphoreGive(sync_semaphore);

    vTaskDelete(NULL);
}

static bool i2c_slave_receive_cb(i2c_slave_dev_handle_t i2c_slave, const i2c_slave_rx_done_event_data_t *evt_data, void *arg)
{
    i2c_slave_github_context_t *context = (i2c_slave_github_context_t *)arg;
    i2c_slave_event_t evt = I2C_SLAVE_EVT_RX;
    BaseType_t xTaskWoken = 0;
    // Command only contains one byte, so just save one bytes here.
    context->command_data = evt_data->buffer;
    //xQueueSendFromISR(context->event_queue, &evt, &xTaskWoken);
    return 0;
}

static void rx_task(void *arg)
{
    i2c_slave_github_context_t *context = (i2c_slave_github_context_t *)arg;
    i2c_slave_dev_handle_t handle = (i2c_slave_dev_handle_t)context->handle;

    uint32_t i =0;

    while (true) {
        i2c_slave_event_t evt;
        int64_t start_time = esp_timer_get_time();
        if (xQueueReceive(context->event_queue, &evt, 10) == pdTRUE) {
            if (evt == I2C_SLAVE_EVT_RX) {
              uint8_t* buff = context->command_data;
              int64_t end_time = esp_timer_get_time();
              int64_t duration = end_time - start_time;
              tx_time_list[i++] = duration;
              ESP_LOGD("UART", "Receive::Time taken: %lld microseconds", duration);
            }
        }
    }
    vTaskDelete(NULL);
}

void i2c_master_init() {
  i2c_master_bus_config_t i2c_mst_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_MASTER_NUM,
    .scl_io_num = I2C_MASTER_SCL_IO,
    .sda_io_num = I2C_MASTER_SDA_IO,
    .glitch_ignore_cnt = 7,
  };

  ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

  i2c_device_config_t dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = I2C_SLAVE_ADDR,
    .scl_speed_hz = I2C_MASTER_FREQ_HZ,
  };

  ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
}

void slave_init(i2c_slave_github_context_t* context){
    i2c_slave_config_t i2c_slv_config = {
        .i2c_port = I2C_SLAVE_NUM,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .scl_io_num = I2C_SLAVE_SCL_IO,
        .sda_io_num = I2C_SLAVE_SDA_IO,
        .slave_addr = I2C_SLAVE_ADDR,
        .send_buf_depth = 100,
        .receive_buf_depth = 100,
    };

    ESP_ERROR_CHECK(i2c_new_slave_device(&i2c_slv_config, &(context->handle)));
    i2c_slave_event_callbacks_t cbs = {
        .on_receive = i2c_slave_receive_cb,
        .on_request = i2c_slave_request_cb,
    };
    ESP_ERROR_CHECK(i2c_slave_register_event_callbacks(context->handle, &cbs, context));
}


uint32_t calc_average(uint32_t list[]){
  uint32_t count = TOTAL_COUNT;
  uint64_t sum = 0;
  for (register unsigned int i=4; i < count; i++){
    sum += (uint64_t) list[i];
  }
  double sumd = (double) sum;
  return sumd / (count -4);
}

double calc_std(uint32_t list[], double avg){
  const uint32_t count = TOTAL_COUNT;
  double std_sum = 0;
  for (register unsigned int i=4; i < count; i++){
    double f = (((double) list[i]) - avg);
    std_sum += f*f;
  }
  std_sum /= count-5;
  return sqrt(std_sum);
}

uint32_t calc_wcet(uint32_t list[]){
  const uint32_t count = TOTAL_COUNT;
  uint32_t max = 0;
  for (register unsigned int i=4; i < count; i++){
    register unsigned long int temp = list[i];
    if(temp > max) max = temp;
  }
  return max;
}

uint32_t calc_dlm(uint32_t list[]){
  const uint32_t count = TOTAL_COUNT;
  const uint32_t dlt = DEADLINE_MS * 1000;
  uint32_t n = 0;
  for (register unsigned int i=4; i < count; i++){
    if(list[i] > dlt) n++;
  }
  return n;
}

void app_main(void)
{
    static i2c_slave_github_context_t context = {0};

    i2c_master_init();
    slave_init(&context);

    context.event_queue = xQueueCreate(16, sizeof(i2c_slave_event_t));
    if (!context.event_queue) {
        ESP_LOGE(TAG, "Creating queue failed");
        return;
    }

    //xTaskCreate(i2c_slave_task, "i2c_slave_task", 1024 * 4, &context, 10, NULL);

    sync_semaphore = xSemaphoreCreateBinary();

    xTaskCreate(tx_task, "tx_task", TASK_STACK_SIZE, &context, 1, NULL);
    xTaskCreate(rx_task, "rx_task", TASK_STACK_SIZE, &context, 1, NULL);

    // Wait for both tasks to complete
    xSemaphoreTake(sync_semaphore, portMAX_DELAY);
    xSemaphoreTake(sync_semaphore, portMAX_DELAY);

    ESP_LOGI("UART", "BOTH TASKS ENDED, calculating metrics");


    uint32_t rx_avg = calc_average(rx_time_list);
    double rx_std = calc_std(rx_time_list, rx_avg);
    uint32_t rx_wcet = calc_wcet(rx_time_list);
    uint32_t rx_dlm = calc_dlm(rx_time_list);

    uint32_t tx_avg = calc_average(tx_time_list);
    double tx_std = calc_std(tx_time_list, tx_avg);
    uint32_t tx_wcet = calc_wcet(tx_time_list);
    uint32_t tx_dlm = calc_dlm(tx_time_list);

    ESP_LOGI(OUT_TAG, "======== Average Time ========");
    ESP_LOGI(OUT_TAG, "Average read time: %lu us", rx_avg);
    ESP_LOGI(OUT_TAG, "Average write time: %lu us", tx_avg);
    ESP_LOGI(OUT_TAG, "Standard deviation of read times: %0.04lf us", rx_std);
    ESP_LOGI(OUT_TAG, "Standard deviation of write times: %0.04lf us", tx_std);
    ESP_LOGI(OUT_TAG, "============ WCET ============");
    ESP_LOGI(OUT_TAG, "Read WCET: %lu us", rx_wcet);
    ESP_LOGI(OUT_TAG, "Write WCET: %lu us", tx_wcet);
    ESP_LOGI(OUT_TAG, "========== Deadlines =========");
    ESP_LOGI(OUT_TAG, "Read Deadlines missed (%d ms): %lu", DEADLINE_MS, rx_dlm);
    ESP_LOGI(OUT_TAG, "Write Deadlines missed (%d ms): %lu", DEADLINE_MS, tx_dlm);
    ESP_LOGI(OUT_TAG, "=========== FINISH ===========");
}
