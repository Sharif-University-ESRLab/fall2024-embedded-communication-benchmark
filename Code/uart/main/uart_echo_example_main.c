#include <stdio.h>
#include <time.h>
#include <math.h>
#include <esp_timer.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "portmacro.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "xtensa/core-macros.h"
#include "rtc_wdt.h"
#include "hal/wdt_hal.h"

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

#define OUT_TAG "RESULT_UART"

#define INF_TXD (CONFIG_EXAMPLE_UART_TXD_INF)
#define INF_RXD (CONFIG_EXAMPLE_UART_RXD_INF)
#define DEV_TXD (CONFIG_EXAMPLE_UART_TXD_DEV)
#define DEV_RXD (CONFIG_EXAMPLE_UART_RXD_DEV)

#define INF_PORT_NUM      (CONFIG_EXAMPLE_UART_PORT_NUM_INF)
#define DEV_PORT_NUM      (CONFIG_EXAMPLE_UART_PORT_NUM_DEV)

#define UART_BAUD_RATE     (CONFIG_EXAMPLE_UART_BAUD_RATE)
#define TASK_STACK_SIZE    (CONFIG_EXAMPLE_TASK_STACK_SIZE)

#define TOTAL_COUNT (CONFIG_EXAMPLE_TOTAL_COUNT)
#define INTERNAL (CONFIG_EXAMPLE_INTERNAL_SIMUL)
#define DEADLINE_MS (CONFIG_EXAMPLE_DEADLINE)


#define BUF_SIZE (1024)
#define DATA_SIZE (1024) // Size of the data to transmit and receive

// Task handles
TaskHandle_t tx_task_handle = NULL;
TaskHandle_t rx_task_handle = NULL;

SemaphoreHandle_t sync_semaphore;

uint32_t tx_time_list[TOTAL_COUNT];
uint32_t rx_time_list[TOTAL_COUNT];


void uart_init(uart_port_t uart_num, int tx_pin, int rx_pin) {
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        //.use_ref_tick = false,
    };
    ESP_ERROR_CHECK(uart_driver_install(uart_num, BUF_SIZE, BUF_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

void populate_data_rand(uint8_t* data, uint32_t size){
  for(uint32_t i=0; i<size; i++){
    data[i] = rand() % (1 << 8);
  }
}

void uart_transmit(uart_port_t uart_num, const uint8_t *data, size_t size) {
    uart_write_bytes(uart_num, (const char *)data, size);
}

uint8_t uart_receive(uart_port_t uart_num, uint8_t *data, size_t size) {
    int len = uart_read_bytes(uart_num, data, size, (2*DEADLINE_MS) / portTICK_PERIOD_MS);
    //if (len > 0) {
    //    ESP_LOGI("UART", "Received %d bytes", len);
    //}
    return len != DATA_SIZE;
}

void tx_task(void *arg) {
    uint8_t out_data[DATA_SIZE+1]; // Data to Transmit
    uart_port_t tx_uart = INF_PORT_NUM;
    uart_init(tx_uart, INF_TXD, INF_RXD);

    for(register unsigned int i=0; i< TOTAL_COUNT; i++) {
        populate_data_rand(out_data, DATA_SIZE+1);
        uart_transmit(tx_uart, out_data, 1); // signal start
        int64_t start_time = esp_timer_get_time();
        uart_transmit(tx_uart, out_data+1, DATA_SIZE);
        int64_t end_time = esp_timer_get_time();
        int64_t duration = end_time - start_time;
        tx_time_list[i] = duration;
        ESP_LOGD("UART", "Transmit::Time taken: %lld microseconds", duration);
        vTaskDelay(DEADLINE_MS / portTICK_PERIOD_MS);
    }

    xSemaphoreGive(sync_semaphore);

    vTaskDelete(NULL);
}

void rx_task(void *arg) {
    uint8_t in_data[DATA_SIZE] = {0};  // Buffer to receive data
    uart_port_t rx_uart = DEV_PORT_NUM;
    uart_init(rx_uart, DEV_TXD, DEV_RXD);
    uint8_t dummy;

    for(register unsigned int i =0; i< TOTAL_COUNT; i++) {
        uart_receive(rx_uart, &dummy, 1); // receive signal
        int64_t start_time = esp_timer_get_time();
        if(uart_receive(rx_uart, in_data, DATA_SIZE)){
          ESP_LOGW("UART", "INCOMPLETE DATA RECEIPT");
        }
        int64_t end_time = esp_timer_get_time();
        int64_t duration = end_time - start_time;
        rx_time_list[i] = duration;
        ESP_LOGD("UART", "Receive::Time taken: %lld microseconds", duration);
        vTaskDelay(DEADLINE_MS / portTICK_PERIOD_MS);
    }

    xSemaphoreGive(sync_semaphore);

    vTaskDelete(NULL);
}

uint32_t calc_average(uint32_t list[]){
  uint32_t count = TOTAL_COUNT;
  uint64_t sum = 0;
  for (register unsigned int i=4; i < count; i++){
    sum += (uint64_t) list[i];
  }
  double sumd = (double) sum;
  return sumd / (count-4);
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

void app_main(void) {

    sync_semaphore = xSemaphoreCreateBinary();

    xTaskCreate(tx_task, "tx_task", TASK_STACK_SIZE, NULL, 1, &tx_task_handle);
    xTaskCreate(rx_task, "rx_task", TASK_STACK_SIZE, NULL, 1, &rx_task_handle);

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

