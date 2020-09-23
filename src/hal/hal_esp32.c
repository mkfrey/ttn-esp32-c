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

#include "../lmic/lmic.h"
#include "../hal/hal_esp32.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/timer.h"
#include "esp_log.h"

#define LMIC_UNUSED_PIN 0xff

#define NOTIFY_BIT_DIO 1
#define NOTIFY_BIT_TIMER 2
#define NOTIFY_BIT_WAKEUP 4
#define NOTIFY_BIT_STOP 8

static const char* const TAG = "ttn_hal";

// -----------------------------------------------------------------------------
// Global and static variables with initialization

TaskHandle_t ttn_hal_esp_lmic_task_handle = NULL;
uint32_t ttn_hal_esp_dio_interrupt_time = 0;
uint8_t ttn_hal_esp_dio_num = 0;
ttn_hal_esp ttn_hal = {
    .rssi_cal = 10,
    .next_alarm = 0
};


// -----------------------------------------------------------------------------
// "Private" function declarations

static void ttn_hal_esp_lmic_background_task(void* pvParameter);
static void ttn_hal_esp_dio_irq_handler(void* arg);
static void ttn_hal_esp_timer_callback(void *arg);
static int64_t ttn_hal_esp_os_time_to_esp_time(int64_t espNow, uint32_t osTime);

void ttn_hal_esp_io_init(ttn_hal_esp* ttn_hal);
void ttn_hal_esp_spi_init(ttn_hal_esp* ttn_hal);
void ttn_hal_esp_timer_init(ttn_hal_esp* ttn_hal);

void ttn_hal_esp_set_next_alarm(ttn_hal_esp* ttn_hal, int64_t time);
void ttn_hal_esp_arm_timer(ttn_hal_esp* ttn_hal, int64_t espNow);
void ttn_hal_esp_disarm_timer(ttn_hal_esp* ttn_hal);
bool ttn_hal_esp_wait(ttn_hal_esp* ttn_hal, WaitKind waitKind);


// -----------------------------------------------------------------------------
// I/O

void ttn_hal_esp_configure_pins(ttn_hal_esp* ttn_hal, spi_host_device_t spi_host, uint8_t nss, uint8_t rxtx, uint8_t rst, uint8_t dio0, uint8_t dio1)
{
    ttn_hal->spi_host = spi_host;
    ttn_hal->io_num_nss = (gpio_num_t)nss;
    ttn_hal->io_num_rx_tx = (gpio_num_t)rxtx;
    ttn_hal->io_num_rst = (gpio_num_t)rst;
    ttn_hal->io_num_dio0 = (gpio_num_t)dio0;
    ttn_hal->io_num_dio1 = (gpio_num_t)dio1;

    // Until the background process has been started, use the current task
    // for supporting calls like `hal_waitUntil()`.
    ttn_hal_esp_lmic_task_handle = xTaskGetCurrentTaskHandle();
}


void IRAM_ATTR ttn_hal_esp_dio_irq_handler(void *arg)
{
    ttn_hal_esp_dio_interrupt_time = hal_ticks();
    ttn_hal_esp_dio_num = (u1_t)(long)arg;
    BaseType_t higherPrioTaskWoken = pdFALSE;
    xTaskNotifyFromISR(ttn_hal_esp_lmic_task_handle, NOTIFY_BIT_DIO, eSetBits, &higherPrioTaskWoken);
    if (higherPrioTaskWoken)
        portYIELD_FROM_ISR();
}

void ttn_hal_esp_io_init(ttn_hal_esp* ttn_hal)
{
    // pinNSS and pinDIO0 and pinDIO1 are required
    ASSERT(ttn_hal->io_num_nss != LMIC_UNUSED_PIN);
    ASSERT(ttn_hal->io_num_dio0 != LMIC_UNUSED_PIN);
    ASSERT(ttn_hal->io_num_dio1 != LMIC_UNUSED_PIN);

    gpio_pad_select_gpio(ttn_hal->io_num_nss);
    gpio_set_level(ttn_hal->io_num_nss, 0);
    gpio_set_direction(ttn_hal->io_num_nss, GPIO_MODE_OUTPUT);

    if (ttn_hal->io_num_rx_tx != LMIC_UNUSED_PIN)
    {
        gpio_pad_select_gpio(ttn_hal->io_num_rx_tx);
        gpio_set_level(ttn_hal->io_num_rx_tx, 0);
        gpio_set_direction(ttn_hal->io_num_rx_tx, GPIO_MODE_OUTPUT);
    }

    if (ttn_hal->io_num_rst != LMIC_UNUSED_PIN)
    {
        gpio_pad_select_gpio(ttn_hal->io_num_rst);
        gpio_set_level(ttn_hal->io_num_rst, 0);
        gpio_set_direction(ttn_hal->io_num_rst, GPIO_MODE_OUTPUT);
    }

    // DIO pins with interrupt handlers
    gpio_pad_select_gpio(ttn_hal->io_num_dio0);
    gpio_set_direction(ttn_hal->io_num_dio0, GPIO_MODE_INPUT);
    gpio_set_intr_type(ttn_hal->io_num_dio0, GPIO_INTR_POSEDGE);

    gpio_pad_select_gpio(ttn_hal->io_num_dio1);
    gpio_set_direction(ttn_hal->io_num_dio1, GPIO_MODE_INPUT);
    gpio_set_intr_type(ttn_hal->io_num_dio1, GPIO_INTR_POSEDGE);

    ESP_LOGI(TAG, "IO initialized");
}

void hal_pin_rxtx(u1_t val)
{
    if (ttn_hal.io_num_rx_tx == LMIC_UNUSED_PIN)
        return;

    gpio_set_level(ttn_hal.io_num_rx_tx, val);
}

void hal_pin_rst(u1_t val)
{
    if (ttn_hal.io_num_rst == LMIC_UNUSED_PIN)
        return;

    if (val == 0 || val == 1)
    {
        // drive pin
        gpio_set_level(ttn_hal.io_num_rst, val);
        gpio_set_direction(ttn_hal.io_num_rst, GPIO_MODE_OUTPUT);
    }
    else
    {
#if defined(CONFIG_TTN_RESET_STATES_ASSERTED)
        // drive up the pin because the hardware is nonstandard
        gpio_set_level(ttn_hal.io_num_rst, 1);
        gpio_set_direction(ttn_hal.io_num_rst, GPIO_MODE_OUTPUT);
#else
        // keep pin floating
        gpio_set_level(ttn_hal.io_num_rst, val);
        gpio_set_direction(ttn_hal.io_num_rst, GPIO_MODE_INPUT);
#endif
    }
}

s1_t hal_getRssiCal (void)
{
    return ttn_hal.rssi_cal;
}

ostime_t hal_setModuleActive (bit_t val)
{
    return 0;
}

bit_t hal_queryUsingTcxo(void)
{
    return false;
}

uint8_t hal_getTxPowerPolicy(u1_t inputPolicy, s1_t requestedPower, u4_t frequency)
{
    return LMICHAL_radio_tx_power_policy_paboost;
}



// -----------------------------------------------------------------------------
// SPI

void ttn_hal_esp_spi_init(ttn_hal_esp* ttn_hal)
{
    // init device
    spi_device_interface_config_t spiConfig = {
        .mode = 1,
        .clock_speed_hz = CONFIG_TTN_SPI_FREQ,
        .command_bits = 0,
        .address_bits = 8,
        .spics_io_num = ttn_hal->io_num_nss,
        .queue_size = 1,
        .cs_ena_posttrans = 2
    };

    esp_err_t ret = spi_bus_add_device(ttn_hal->spi_host, &spiConfig, &ttn_hal->spi_handle);
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "SPI initialized");
}

void hal_spi_write(u1_t cmd, const u1_t *buf, size_t len)
{
    ttn_hal_esp_spi_write(&ttn_hal, cmd, buf, len);
}

void ttn_hal_esp_spi_write(ttn_hal_esp* ttn_hal, uint8_t cmd, const uint8_t *buf, size_t len)
{
    ttn_hal->spi_transaction = (spi_transaction_t){
        .addr = cmd,
        .length = 8 * len,
        .tx_buffer = buf
    };
    esp_err_t err = spi_device_transmit(ttn_hal->spi_handle, &ttn_hal->spi_transaction);
    ESP_ERROR_CHECK(err);
}

void hal_spi_read(u1_t cmd, u1_t *buf, size_t len)
{
    ttn_hal_esp_spi_read(&ttn_hal, cmd, buf, len);
}

void ttn_hal_esp_spi_read(ttn_hal_esp* ttn_hal, uint8_t cmd, uint8_t *buf, size_t len)
{
    memset(buf, 0, len);
    ttn_hal->spi_transaction = (spi_transaction_t){
        .addr = cmd,
        .length = 8 * len,
        .rxlength = 8 * len,
        .tx_buffer = buf,
        .rx_buffer = buf
    };
    esp_err_t err = spi_device_transmit(ttn_hal->spi_handle, &ttn_hal->spi_transaction);
    ESP_ERROR_CHECK(err);
}


// -----------------------------------------------------------------------------
// TIME

/*
 * LIMIC uses a 32 bit time system (ostime_t) counting ticks. In this
 * implementation each tick is 16µs. It will wrap arounnd every 19 hours.
 * 
 * The ESP32 has a 64 bit timer counting microseconds. It will wrap around
 * every 584,000 years. So we don't need to bother.
 * 
 * Based on this timer, future callbacks can be scheduled. This is used to
 * schedule the next LMIC job.
 */

// Convert LMIC tick time (ostime_t) to ESP absolute time.
// `osTime` is assumed to be somewhere between one hour in the past and
// 18 hours into the future. 
static int64_t ttn_hal_esp_os_time_to_esp_time(int64_t espNow, uint32_t osTime)
{
    int64_t espTime;
    uint32_t osNow = (uint32_t)(espNow >> 4);

    // unsigned difference:
    // 0x00000000 - 0xefffffff: future (0 to about 18 hours)
    // 0xf0000000 - 0xffffffff: past (about 1 to 0 hours)
    uint32_t osDiff = osTime - osNow;
    if (osDiff < 0xf0000000)
    {
        espTime = espNow + (((int64_t)osDiff) << 4);
    }
    else
    {
        // one's complement instead of two's complement:
        // off by 1 µs and ignored
        osDiff = ~osDiff;
        espTime = espNow - (((int64_t)osDiff) << 4);
    }

    return espTime;
}

void ttn_hal_esp_timer_init(ttn_hal_esp* ttn_hal)
{
    esp_timer_create_args_t timerConfig = {
        .callback = &ttn_hal_esp_timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "lmic_job"
    };
    esp_err_t err = esp_timer_create(&timerConfig, &ttn_hal->timer);
    ESP_ERROR_CHECK(err);

    ESP_LOGI(TAG, "Timer initialized");
}

void ttn_hal_esp_set_next_alarm(ttn_hal_esp* ttn_hal, int64_t time)
{
    ttn_hal->next_alarm = time;
}

void ttn_hal_esp_arm_timer(ttn_hal_esp* ttn_hal, int64_t espNow)
{
    if (ttn_hal->next_alarm == 0)
        return;
    int64_t timeout = ttn_hal->next_alarm - esp_timer_get_time();
    if (timeout < 0)
        timeout = 10;
    esp_timer_start_once(ttn_hal->timer, timeout);
}

void ttn_hal_esp_disarm_timer(ttn_hal_esp* ttn_hal)
{
    esp_timer_stop(ttn_hal->timer);
}

static void ttn_hal_esp_timer_callback(void *arg)
{
    xTaskNotify(ttn_hal_esp_lmic_task_handle, NOTIFY_BIT_TIMER, eSetBits);
}

// Wait for the next external event. Either:
// - scheduled timer due to scheduled job or waiting for a given time
// - wake up event from the client code
// - I/O interrupt (DIO0 or DIO1 pin)
bool ttn_hal_esp_wait(ttn_hal_esp* ttn_hal, WaitKind waitKind)
{
    TickType_t ticksToWait = waitKind == CHECK_IO ? 0 : portMAX_DELAY;
    while (true)
    {
        uint32_t bits = ulTaskNotifyTake(pdTRUE, ticksToWait);
        if (bits == 0)
            return false;

        if ((bits & NOTIFY_BIT_STOP) != 0)
            return false;

        if ((bits & NOTIFY_BIT_WAKEUP) != 0)
        {
            if (waitKind != WAIT_FOR_TIMER)
            {
                ttn_hal_esp_disarm_timer(ttn_hal);
                return true;
            }
        }
        else if ((bits & NOTIFY_BIT_TIMER) != 0)
        {
            ttn_hal_esp_disarm_timer(ttn_hal);
            ttn_hal_esp_set_next_alarm(ttn_hal, 0);
            if (waitKind != CHECK_IO)
                return true;
        }
        else // IO interrupt
        {
            if (waitKind != WAIT_FOR_TIMER)
                ttn_hal_esp_disarm_timer(ttn_hal);
            ttn_hal_esp_enter_critical_section(ttn_hal);
            radio_irq_handler_v2(ttn_hal_esp_dio_num, ttn_hal_esp_dio_interrupt_time);
            ttn_hal_esp_leave_critical_section(ttn_hal);
            if (waitKind != WAIT_FOR_TIMER)
                return true;
        }
    }
}

// Gets current time in LMIC ticks
u4_t hal_ticks()
{
    // LMIC tick unit: 16µs
    // esp_timer unit: 1µs
    return (u4_t)(esp_timer_get_time() >> 4);
}

// Wait until the specified time.
// Called if the LMIC code needs to wait for a precise time.
// All other events are ignored and will be served later.
u4_t hal_waitUntil(u4_t time)
{
    return ttn_hal_esp_wait_until(&ttn_hal, time);
}

uint32_t ttn_hal_esp_wait_until(ttn_hal_esp* ttn_hal, uint32_t osTime)
{
    int64_t espNow = esp_timer_get_time();
    int64_t espTime = ttn_hal_esp_os_time_to_esp_time(espNow, osTime);
    ttn_hal_esp_set_next_alarm(ttn_hal, espTime);
    ttn_hal_esp_arm_timer(ttn_hal, espNow);
    ttn_hal_esp_wait(ttn_hal, WAIT_FOR_TIMER);

    u4_t osNow = hal_ticks();
    u4_t diff = osNow - osTime;
    return diff < 0x80000000U ? diff : 0;
}

// Called by client code to wake up LMIC to do something,
// e.g. send a submitted messages.

void ttn_hal_esp_wake_up()
{
    xTaskNotify(ttn_hal_esp_lmic_task_handle, NOTIFY_BIT_WAKEUP, eSetBits);
}

// Check if the specified time has been reached or almost reached.
// Otherwise, save it as alarm time.
// LMIC calls this function with the scheduled time of the next job
// in the queue. If the job is not due yet, LMIC will go to sleep.
u1_t hal_checkTimer(uint32_t time)
{
    return ttn_hal_esp_check_timer(&ttn_hal, time);
}

uint8_t ttn_hal_esp_check_timer(ttn_hal_esp* ttn_hal, uint32_t osTime)
{
    int64_t espNow = esp_timer_get_time();
    int64_t espTime = ttn_hal_esp_os_time_to_esp_time(espNow, osTime);
    int64_t diff = espTime - espNow;
    if (diff < 100)
        return 1; // timer has expired or will expire very soon

    ttn_hal_esp_set_next_alarm(ttn_hal, espTime);
    return 0;
}

// Go to sleep until next event.
// Called when LMIC is not busy and not job is due to be executed.
void hal_sleep()
{
    ttn_hal_esp_sleep(&ttn_hal);
}

void ttn_hal_esp_sleep(ttn_hal_esp* ttn_hal)
{
    if (ttn_hal_esp_wait(ttn_hal, CHECK_IO))
        return;

    ttn_hal_esp_arm_timer(ttn_hal, esp_timer_get_time());
    ttn_hal_esp_wait(ttn_hal, WAIT_FOR_ANY_EVENT);
}


// -----------------------------------------------------------------------------
// IRQ

void hal_disableIRQs()
{
    // nothing to do as interrupt handlers post message to queue
    // and don't access any shared data structures
}

void hal_enableIRQs()
{
    // nothing to do as interrupt handlers post message to queue
    // and don't access any shared data structures
}

void hal_processPendingIRQs()
{
    // nothing to do as interrupt handlers post message to queue
    // and don't access any shared data structures
}


// -----------------------------------------------------------------------------
// Synchronization between application code and background task

void ttn_hal_esp_init_critical_section(ttn_hal_esp* ttn_hal)
{
    ttn_hal->mutex = xSemaphoreCreateRecursiveMutex();
}

void ttn_hal_esp_enter_critical_section(ttn_hal_esp* ttn_hal)
{
    xSemaphoreTakeRecursive(ttn_hal->mutex, portMAX_DELAY);
}

void ttn_hal_esp_leave_critical_section(ttn_hal_esp* ttn_hal)
{
    xSemaphoreGiveRecursive(ttn_hal->mutex);
}

// -----------------------------------------------------------------------------

static void ttn_hal_esp_lmic_background_task(void* pvParameter)
{
    ttn_hal_esp* instance = (ttn_hal_esp*)pvParameter;
    while (instance->run_background_task)
        os_runloop_once();
    vTaskDelete(NULL);
}

void hal_init_ex(const void *pContext)
{
    ttn_hal_esp_init(&ttn_hal);
}

void ttn_hal_esp_init(ttn_hal_esp* ttn_hal)
{
    // configure radio I/O and interrupt handler
    ttn_hal_esp_io_init(ttn_hal);
    // configure radio SPI
    ttn_hal_esp_spi_init(ttn_hal);
    // configure timer and alarm callback
    ttn_hal_esp_timer_init(ttn_hal);
}


void ttn_hal_esp_start_lmic_task(ttn_hal_esp* ttn_hal)
{
    ttn_hal->run_background_task = true;
    xTaskCreate(ttn_hal_esp_lmic_background_task, "ttn_lmic", 1024 * 4, ttn_hal, CONFIG_TTN_BG_TASK_PRIO, &ttn_hal_esp_lmic_task_handle);

    // enable interrupts
    gpio_isr_handler_add(ttn_hal->io_num_dio0, ttn_hal_esp_dio_irq_handler, (void *)0);
    gpio_isr_handler_add(ttn_hal->io_num_dio1, ttn_hal_esp_dio_irq_handler, (void *)1);
}

void ttn_hal_esp_stop_lmic_task(ttn_hal_esp* ttn_hal)
{
    ttn_hal->run_background_task = false;
    gpio_isr_handler_remove(ttn_hal->io_num_dio0);
    gpio_isr_handler_remove(ttn_hal->io_num_dio1);
    ttn_hal_esp_disarm_timer(ttn_hal);
    xTaskNotify(ttn_hal_esp_lmic_task_handle, NOTIFY_BIT_STOP, eSetBits);
}


// -----------------------------------------------------------------------------
// Fatal failure

static hal_failure_handler_t* custom_hal_failure_handler = NULL;

void hal_set_failure_handler(const hal_failure_handler_t* const handler)
{
    custom_hal_failure_handler = handler;
}

void hal_failed(const char *file, u2_t line)
{
    if (custom_hal_failure_handler != NULL)
        (*custom_hal_failure_handler)(file, line);

    ESP_LOGE(TAG, "LMIC failed and stopped: %s:%d", file, line);

    // go to sleep forever
    while (true)
    {
        vTaskDelay(portMAX_DELAY);
    }
}
