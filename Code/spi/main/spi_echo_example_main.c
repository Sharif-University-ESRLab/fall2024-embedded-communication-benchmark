#include <stdio.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <esp_timer.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "hal/spi_types.h"
#include "portmacro.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "xtensa/core-macros.h"
#include "rtc_wdt.h"
#include "hal/wdt_hal.h"
#include "driver/spi_master.h"
#include "driver/spi_slave.h"
#include "soc/rtc_periph.h"


// Pins in use
#define MASTER_GPIO_MOSI (CONFIG_EXAMPLE_MASTER_MOSI)
#define MASTER_GPIO_MISO (CONFIG_EXAMPLE_MASTER_MISO)
#define MASTER_GPIO_SCLK (CONFIG_EXAMPLE_MASTER_CLK)
#define MASTER_GPIO_CS (CONFIG_EXAMPLE_MASTER_CS)

#define SLAVE_GPIO_MOSI (CONFIG_EXAMPLE_SLAVE_MOSI)
#define SLAVE_GPIO_MISO (CONFIG_EXAMPLE_SLAVE_MISO)
#define SLAVE_GPIO_SCLK (CONFIG_EXAMPLE_SLAVE_CLK)
#define SLAVE_GPIO_CS (CONFIG_EXAMPLE_SLAVE_CS)

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

#define TASK_STACK_SIZE    (CONFIG_EXAMPLE_TASK_STACK_SIZE)

#define TOTAL_COUNT (CONFIG_EXAMPLE_TOTAL_COUNT)
#define INTERNAL (CONFIG_EXAMPLE_INTERNAL_SIMUL)
#define DEADLINE_MS (CONFIG_EXAMPLE_DEADLINE)


#define BUF_SIZE (1024)
#define DATA_SIZE (15) // Size of the data to transmit and receive

spi_device_handle_t handle;

SemaphoreHandle_t sync_semaphore;

uint32_t tx_time_list[TOTAL_COUNT];
uint32_t rx_time_list[TOTAL_COUNT];

void configure_spi_master_device(int mosi, int miso, int clk, int cs){

  // Configuration for the SPI bus
  spi_bus_config_t buscfg = {
    .mosi_io_num = mosi,
    .miso_io_num = miso,
    .sclk_io_num = clk,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1};

  // Configuration for the SPI device on the other side of the bus
  spi_device_interface_config_t devcfg = {
    .command_bits = 0,
    .address_bits = 0,
    .dummy_bits = 0,
    .clock_speed_hz = 10000000,
    //.duty_cycle_pos = 128, // 50% duty cycle
    .mode = 0,
    .spics_io_num = cs,
    //.cs_ena_posttrans = 3, // Keep the CS low 3 cycles after transaction
    .queue_size = 1
  };


  spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
  spi_bus_add_device(HSPI_HOST, &devcfg, &handle);
}

void configure_spi_slave_device(int mosi, int miso, int clk, int cs){
  //Configuration for the SPI bus
  spi_bus_config_t buscfg={
    .mosi_io_num=mosi,
    .miso_io_num=miso,
    .sclk_io_num=clk,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
  };

  //Configuration for the SPI slave interface
  spi_slave_interface_config_t slvcfg={
    .mode=0,
    .spics_io_num=cs,
    .queue_size=3,
    .flags=0,
  };

  //Initialize SPI slave interface
  //TODO: change the VSPI shit with KCONFIG
  spi_slave_initialize(VSPI_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
}

void spi_transmit(int host, uint8_t* buff, uint32_t size){
  char dummy;
  spi_transaction_t t;
  memset(&t, 0, sizeof(t));
  t.length = (size+1) * 8;
  t.tx_buffer = buff;
  t.rx_buffer = &dummy;
  spi_device_transmit(handle, &t);
}


uint8_t spi_receive(int host, uint8_t* buff, uint32_t size){
  char dummy = 'p';
  spi_slave_transaction_t t;
  memset(&t, 0, sizeof(t));
  t.length=(size+1)*8;
  t.rx_buffer=buff;
  t.tx_buffer=&dummy;
  esp_err_t res = spi_slave_transmit(host, &t, 2*DEADLINE_MS);
  if(res != ESP_OK){
    ESP_LOGE("SPI", "Spi Slave receive error");
  }
  return (t.trans_len / 8) != size + 1;
}


void populate_data_rand(uint8_t* data, uint32_t size){
  for(uint32_t i=0; i<size; i++){
    data[i] = rand() % (1 << 8);
  }
}


void tx_task(void *arg) {
    //uint8_t* out_data = (uint8_t*) malloc(DATA_SIZE * sizeof(uint8_t)); // Data to Transmit
    //uint8_t out_data[DATA_SIZE];
    
    uint32_t heap_size = heap_caps_get_free_size(MALLOC_CAP_DMA);
    ESP_LOGI("SPI", "heap size is: %lu", heap_size);

    uint8_t* out_data = (uint8_t*) heap_caps_malloc(DATA_SIZE * sizeof(uint8_t), MALLOC_CAP_DMA);
    if (out_data == NULL) {
      ESP_LOGE("SPI", "Failed to allocate memory for out_data");
      // Handle allocation failure
    }
    vTaskDelay(DEADLINE_MS / (2*portTICK_PERIOD_MS));

    for(register unsigned int i=0; i< TOTAL_COUNT; i++) {
        populate_data_rand(out_data, DATA_SIZE+1);
        int64_t start_time = esp_timer_get_time();
        spi_transmit(HSPI_HOST, out_data, DATA_SIZE);
        int64_t end_time = esp_timer_get_time();
        int64_t duration = end_time - start_time;
        tx_time_list[i] = duration;
        ESP_LOGD("UART", "Transmit::Time taken: %lld microseconds", duration);
        vTaskDelay(DEADLINE_MS / portTICK_PERIOD_MS); // in SPI only the master must have a delay, introducing a delay on the slave side causes sync errors in SPI.
    }

    ESP_LOGI("SPI", "TX DONE");

    xSemaphoreGive(sync_semaphore);

    vTaskDelete(NULL);
}

void rx_task(void *arg) {
    //uint8_t* in_data = (uint8_t*)malloc(DATA_SIZE * sizeof(uint8_t));  // Buffer to receive data
    //uint8_t in_data[DATA_SIZE];

    uint8_t* in_data = (uint8_t*) heap_caps_malloc(DATA_SIZE * sizeof(uint8_t), MALLOC_CAP_DMA);
    if (in_data == NULL) {
      ESP_LOGE("SPI", "Failed to allocate memory for in_data");
      // Handle allocation failure
    }

    for(register unsigned int i =0; i< TOTAL_COUNT; i++) {
        int64_t start_time = esp_timer_get_time();
        if(spi_receive(VSPI_HOST, in_data, DATA_SIZE)){
          ESP_LOGW("UART", "INCOMPLETE DATA RECEIPT");
        }
        int64_t end_time = esp_timer_get_time();
        int64_t duration = end_time - start_time;
        rx_time_list[i] = duration;
        ESP_LOGD("UART", "Receive::Time taken: %lld microseconds", duration);
    }

    ESP_LOGI("SPI", "RX DONE");

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

void app_main(void) {

    srand(time(NULL));

    configure_spi_master_device(MASTER_GPIO_MOSI, MASTER_GPIO_MISO, MASTER_GPIO_SCLK, MASTER_GPIO_CS);
    configure_spi_slave_device(SLAVE_GPIO_MOSI, SLAVE_GPIO_MISO, SLAVE_GPIO_SCLK, SLAVE_GPIO_CS);

    vTaskDelay(2*DEADLINE_MS / portTICK_PERIOD_MS);

    sync_semaphore = xSemaphoreCreateBinary();

    xTaskCreate(tx_task, "tx_task", TASK_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(rx_task, "rx_task", TASK_STACK_SIZE, NULL, 1, NULL);

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

