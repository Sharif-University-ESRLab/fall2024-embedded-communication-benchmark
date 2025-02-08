/* UART Echo Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
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

static const char *TAG = "uart_test";

#define BUF_SIZE (1024)
#define DATA_SIZE (4)
#define BENCHMARK_NAME "UART"

#define MIN(x,y) (x)>(y) ? (y) : (x)

#define TIMEOUT_CONTEXT_SWITCH 5

const uint32_t CONVERT_COEFF = 1000;

static void setup_uart(int port_num, int rx, int tx, int baud){
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = baud,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(port_num, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(port_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(port_num, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

void populate_data_rand(uint8_t* data, uint32_t size){
  for(uint32_t i=0; i<size; i++){
    data[i] = rand() % (1 << 8);
  }
}

static void write_from_inf(void* arg){
  uint8_t* out_data = (uint8_t*)arg;
  uint8_t* temp_data = (uint8_t*) malloc(BUF_SIZE);
  int len;
  uint32_t total = 0;
  while(1){
    ESP_LOGI(TAG, "starting write inf");
    len = uart_write_bytes(INF_PORT_NUM, out_data, DATA_SIZE);
    ESP_LOGI(TAG, "ended write inf");
    vTaskDelay( pdMS_TO_TICKS(TIMEOUT_CONTEXT_SWITCH) );
    if(len < 0){
      ESP_LOGE(TAG, "Failure writing to INFINITE uart");
    }
    total += len;
    if (total == DATA_SIZE) break;
    if(len == 0){
      ESP_LOGE(TAG, "Write Unexpected EOF at %lu. Expecting %d", total, DATA_SIZE);
      break;
    }
  }
  ESP_LOGI(TAG, "%s\n", out_data );
  while(1){
    ESP_LOGI(TAG, "starting read inf");
    len = uart_read_bytes(INF_PORT_NUM, temp_data, MIN((BUF_SIZE - 1), DATA_SIZE), 2000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "ended read inf");
    vTaskDelay( pdMS_TO_TICKS(TIMEOUT_CONTEXT_SWITCH) );
    ESP_LOGI(TAG, "%s", temp_data);
    if(len < 0){
      ESP_LOGE(TAG, "Failure reading from DEVICE uart");
    }
    total += len;
    if (total >= DATA_SIZE) break;
    if(len == 0){
      ESP_LOGE(TAG, "Unexpected EOF at %lu. Expecting %d", total, DATA_SIZE);
      break;
    }
  }
  vTaskDelete(NULL);
}

static void benchmark_task(void* arg){
  #if INTERNAL
  uint8_t* in_data = (uint8_t*) malloc(DATA_SIZE);
  #endif

  uint8_t* temp_data = (uint8_t*) malloc(BUF_SIZE);
  uint8_t* out_data = (uint8_t*) malloc(DATA_SIZE);

  uint32_t wcet_read_elapsed = 0;
  uint32_t wcet_write_elapsed = 0;
  
  uint32_t total_read_elapsed = 0;
  uint32_t total_write_elapsed = 0;
  uint32_t deadline_miss_count = 0;

  uint32_t count;
  int len;
  uint32_t total_data_read = 0;

  uint32_t start_read;
  uint32_t end_read;
  uint32_t start_write;
  uint32_t end_write;

  uint32_t read_elapsed;
  uint32_t write_elapsed;

  uint32_t total_elapsed_ms;

  uint32_t avg_read_elapsed;
  uint32_t avg_write_elapsed;

  char name[13];

  srand(time(NULL));
  #if INTERNAL
  setup_uart(INF_PORT_NUM, INF_RXD, INF_TXD, UART_BAUD_RATE);
  #endif
  setup_uart(DEV_PORT_NUM, DEV_RXD, DEV_TXD, UART_BAUD_RATE);

  for(count=1; count <= TOTAL_COUNT; count++){
    ESP_LOGI(TAG, "count %lu", count);
    #if INTERNAL
    populate_data_rand(in_data, DATA_SIZE);
    #endif
    populate_data_rand(out_data, DATA_SIZE);

    sprintf(name, "write_inf%03ld", count);

    #if INTERNAL
      xTaskCreate(write_from_inf, name, TASK_STACK_SIZE, out_data, 10, NULL);
      //vTaskDelay(2 / portTICK_PERIOD_MS);
    #endif
    start_read = esp_timer_get_time();
    while(1){
      ESP_LOGI(TAG, "starting read dev");
      len = uart_read_bytes(DEV_PORT_NUM, temp_data, MIN((BUF_SIZE - 1), DATA_SIZE), 2000 / portTICK_PERIOD_MS);
      ESP_LOGI(TAG, "ended read dev");
      vTaskDelay( pdMS_TO_TICKS(TIMEOUT_CONTEXT_SWITCH) );
      ESP_LOGI(TAG, "%s", temp_data);
      if(len < 0){
        ESP_LOGE(TAG, "Failure reading from DEVICE uart");
      }
      total_data_read += len;
      if (total_data_read >= DATA_SIZE) break;
      if(len == 0){
        ESP_LOGE(TAG, "Unexpected EOF at %lu. Expecting %d", total_data_read, DATA_SIZE);
        break;
      }
    }
    end_read = esp_timer_get_time();
    read_elapsed = end_read - start_read;
    if(read_elapsed > wcet_read_elapsed) wcet_read_elapsed = read_elapsed;
    total_read_elapsed += read_elapsed;

    len = 0;
    total_data_read = 0;
    start_write = esp_timer_get_time();
  
    while(1){
      ESP_LOGI(TAG, "starting write dev");
      len = uart_write_bytes(DEV_PORT_NUM, out_data, DATA_SIZE);
      ESP_LOGI(TAG, "ended write dev");
      vTaskDelay( pdMS_TO_TICKS(TIMEOUT_CONTEXT_SWITCH) );
      if(len < 0){
        ESP_LOGE(TAG, "Failure writing to DEVICE uart");
      }
      total_data_read += len;
      if (total_data_read >= DATA_SIZE) break;
      if(len == 0){
        ESP_LOGE(TAG, "DEVICE Unexpected EOF at %lu. Expecting %d", total_data_read, DATA_SIZE);
        break;
      }
    }
    end_write = esp_timer_get_time();
    write_elapsed = end_write - start_write;
    if(write_elapsed > wcet_write_elapsed) wcet_write_elapsed = write_elapsed;
    total_write_elapsed += write_elapsed;

    total_elapsed_ms = (read_elapsed + write_elapsed) / CONVERT_COEFF;
    if(total_elapsed_ms >= DEADLINE_MS){
      deadline_miss_count ++;
    }
  }
  printf("BENCHMARKREPORT\n");
  printf("==========BENCHMARK REPORT===========\n");
  printf("Benchmark name: " BENCHMARK_NAME "\n");
  printf("Read time (WCET) %lu ticks\n", wcet_read_elapsed);
  printf("Write time (WCET) %lu ticks\n", wcet_read_elapsed);
  printf("Read time (WCET) %f ms\n", (float) wcet_read_elapsed / CONVERT_COEFF);
  printf("Write time (WCET) %f ms\n", (float) wcet_read_elapsed / CONVERT_COEFF);
  printf("\n");
  avg_read_elapsed = total_read_elapsed / TOTAL_COUNT;
  avg_write_elapsed = total_write_elapsed / TOTAL_COUNT;
  printf("Read time (AVG) %lu ticks\n", avg_read_elapsed);
  printf("Write time (AVG) %lu ticks\n", avg_read_elapsed);
  printf("Read time (AVG) %f ms\n", (float) avg_read_elapsed / CONVERT_COEFF);
  printf("Write time (AVG) %f ms\n", (float) avg_read_elapsed / CONVERT_COEFF);
  printf("\n");
  printf("Missed deadlines: %lu\n", deadline_miss_count);
  printf("====================================\n");
  fflush(stdout);
  vTaskDelete(NULL);
}

static void dog_feeder(void* arg){
  while(1){
    rtc_wdt_feed();
    vTaskDelay(5);
  }
}

void app_main(void)
{
  wdt_hal_context_t rtc_wdt_ctx = RWDT_HAL_CONTEXT_DEFAULT();
  wdt_hal_write_protect_disable(&rtc_wdt_ctx);
  wdt_hal_disable(&rtc_wdt_ctx);
  wdt_hal_write_protect_enable(&rtc_wdt_ctx);
  //xTaskCreate(dog_feeder, "yummy", TASK_STACK_SIZE, NULL, 10, NULL);
  xTaskCreate(benchmark_task, "uart_benchmark_task", TASK_STACK_SIZE, NULL, 10, NULL);
}
