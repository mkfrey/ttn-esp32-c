/*******************************************************************************
 * 
 * ttn-esp32 - The Things Network device library for ESP-IDF / SX127x
 * 
 * Copyright (c) 2018-2019 Manuel Bleichenbacher
 * 
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 *
 * Hardware abstraction layer to run LMIC on a ESP32 using ESP-IDF.
 *******************************************************************************/

#ifndef _hal_esp32_h_
#define _hal_esp32_h_

#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_timer.h>


typedef enum {
    CHECK_IO,
    WAIT_FOR_ANY_EVENT,
    WAIT_FOR_TIMER
} WaitKind;

typedef struct  {
    spi_host_device_t spi_host;
    gpio_num_t io_num_nss;
    gpio_num_t io_num_rx_tx;
    gpio_num_t io_num_rst;
    gpio_num_t io_num_dio0;
    gpio_num_t io_num_dio1;
    int8_t rssi_cal;
    spi_device_handle_t spi_handle;
    spi_transaction_t spi_transaction;
    SemaphoreHandle_t mutex;
    esp_timer_handle_t timer;
    int64_t next_alarm;
    volatile bool run_background_task;
} ttn_hal_esp;

void ttn_hal_esp_configure_pins(ttn_hal_esp* ttn_hal, spi_host_device_t spi_host, uint8_t nss, uint8_t rxtx, uint8_t rst, uint8_t dio0, uint8_t dio1);
void ttn_hal_esp_init(ttn_hal_esp* ttn_hal);
void ttn_hal_esp_start_lmic_task(ttn_hal_esp* ttn_hal);
void ttn_hal_esp_stop_lmic_task(ttn_hal_esp* ttn_hal);

void ttn_hal_esp_wake_up();
void ttn_hal_esp_init_critical_section(ttn_hal_esp* ttn_hal);
void ttn_hal_esp_enter_critical_section(ttn_hal_esp* ttn_hal);
void ttn_hal_esp_leave_critical_section(ttn_hal_esp* ttn_hal);

void ttn_hal_esp_spi_write(ttn_hal_esp* ttn_hal, uint8_t cmd, const uint8_t *buf, size_t len);
void ttn_hal_esp_spi_read(ttn_hal_esp* ttn_hal, uint8_t cmd, uint8_t *buf, size_t len);
uint8_t ttn_hal_esp_check_timer(ttn_hal_esp* ttn_hal, uint32_t osTime);
void ttn_hal_esp_sleep(ttn_hal_esp* ttn_hal);

uint32_t ttn_hal_esp_wait_until(ttn_hal_esp* ttn_hal, uint32_t osTime);


/** Private mebers. */

/*
class HAL_ESP32
{
public:
    HAL_ESP32();

    void configurePins(spi_host_device_t spi_host, uint8_t nss, uint8_t rxtx, uint8_t rst, uint8_t dio0, uint8_t dio1);
    void init();
    void startLMICTask();
    void stopLMICTask();
    
    void wakeUp();
    void initCriticalSection();
    void enterCriticalSection();
    void leaveCriticalSection();

    void spiWrite(uint8_t cmd, const uint8_t *buf, size_t len);
    void spiRead(uint8_t cmd, uint8_t *buf, size_t len);
    uint8_t checkTimer(uint32_t osTime);
    void sleep();
    
    uint32_t waitUntil(uint32_t osTime);

    spi_host_device_t spiHost;
    gpio_num_t pinNSS;
    gpio_num_t pinRxTx;
    gpio_num_t pinRst;
    gpio_num_t pinDIO0;
    gpio_num_t pinDIO1;
    int8_t rssiCal;

private:
    static void lmicBackgroundTask(void* pvParameter);
    static void dioIrqHandler(void* arg);
    static void timerCallback(void *arg);
    static int64_t osTimeToEspTime(int64_t espNow, uint32_t osTime);

    void ioInit();
    void spiInit();
    void timerInit();

    void setNextAlarm(int64_t time);
    void armTimer(int64_t espNow);
    void disarmTimer();
    bool wait(WaitKind waitKind);

    static TaskHandle_t lmicTask;
    static uint32_t dioInterruptTime;
    static uint8_t dioNum;

    spi_device_handle_t spiHandle;
    spi_transaction_t spiTransaction;
    SemaphoreHandle_t mutex;
    esp_timer_handle_t timer;
    int64_t nextAlarm;
    volatile bool runBackgroundTask;
};*/

extern ttn_hal_esp ttn_hal;


#endif // _hal_esp32_h_