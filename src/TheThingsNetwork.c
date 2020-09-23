/*******************************************************************************
 * 
 * ttn-esp32 - The Things Network device library for ESP-IDF / SX127x
 * 
 * Copyright (c) 2018-2019 Manuel Bleichenbacher
 * 
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 *
 * High-level API for ttn-esp32.
 *******************************************************************************/

#include "freertos/FreeRTOS.h"
#include "esp_event.h"
#include "esp_log.h"
#include "hal/hal_esp32.h"
#include "lmic/lmic.h"
#include "TheThingsNetwork.h"
#include "TTNProvisioning.h"
#include "TTNLogging.h"


/**
 * @brief Reason the user code is waiting
 */
typedef enum
{
    eWaitingNone,
    eWaitingForJoin,
    eWaitingForTransmission
} TTNWaitingReason;

/**
 * @brief Event type
 */
typedef enum {
    eEvtNone,
    eEvtJoinCompleted,
    eEvtJoinFailed,
    eEvtMessageReceived,
    eEvtTransmissionCompleted,
    eEvtTransmissionFailed
} TTNEvent;

/**
 * @brief Event message sent from LMIC task to waiting client task
 */
typedef struct {
    TTNEvent event;
    uint8_t port;
    const uint8_t* message;
    size_t messageSize;
} TTNLmicEvent;

TTNLmicEvent ttn_lmic_event_default = {
    .event = eEvtNone
};

static const char *TAG = "ttn";

static QueueHandle_t lmicEventQueue = NULL;
static TTNWaitingReason waitingReason = eWaitingNone;
static ttn_provisioning provisioning;

#if LMIC_ENABLE_event_logging
static ttn_log* logging;
#endif
static TTNRFSettings lastRfSettings[4];
static TTNRxTxWindow currentWindow;

static void eventCallback(void* userData, ev_t event);
static void messageReceivedCallback(void *userData, uint8_t port, const uint8_t *message, size_t messageSize);
static void messageTransmittedCallback(void *userData, int success);
static void saveRFSettings(TTNRFSettings* rfSettings);
static void clearRFSettings(TTNRFSettings* rfSettings);


TTNMessageCallback messageCallback = NULL;

bool ttn_join_core();

void ttn_init() {
#if defined(TTN_IS_DISABLED)
    ESP_LOGE(TAG, "TTN is disabled. Configure a frequency plan using 'make menuconfig'");
    ASSERT(0);
#endif

  ASSERT(ttn_hal.mutex == NULL);
  ttn_hal_esp_init_critical_section(&ttn_hal);
}

void ttn_configure_pins(spi_host_device_t spi_host, uint8_t nss, uint8_t rxtx, uint8_t rst, uint8_t dio0, uint8_t dio1)
{
    ttn_hal_esp_configure_pins(&ttn_hal, spi_host, nss, rxtx, rst, dio0, dio1);

#if LMIC_ENABLE_event_logging
    logging = &ttn_log_instance;
    ttn_log_init(logging);
#endif

    ttn_provisioning_init(&provisioning);

    LMIC_registerEventCb(eventCallback, NULL);
    LMIC_registerRxMessageCb(messageReceivedCallback, NULL);

    os_init_ex(NULL);
    ttn_reset();

    lmicEventQueue = xQueueCreate(4, sizeof(TTNLmicEvent));
    ASSERT(lmicEventQueue != NULL);
    ttn_hal_esp_start_lmic_task(&ttn_hal);
}

void ttn_reset()
{
    ttn_hal_esp_enter_critical_section(&ttn_hal);
    LMIC_reset();
    LMIC_setClockError(MAX_CLOCK_ERROR * 4 / 100);
    waitingReason = eWaitingNone;
    ttn_hal_esp_leave_critical_section(&ttn_hal);
}

void ttn_shutdown()
{
    ttn_hal_esp_enter_critical_section(&ttn_hal);
    LMIC_shutdown();
    ttn_hal_esp_stop_lmic_task(&ttn_hal);
    waitingReason = eWaitingNone;
    ttn_hal_esp_leave_critical_section(&ttn_hal);
}

void ttn_startup()
{
    ttn_hal_esp_enter_critical_section(&ttn_hal);
    LMIC_reset();
    ttn_hal_esp_start_lmic_task(&ttn_hal);
    ttn_hal_esp_leave_critical_section(&ttn_hal);
}

bool ttn_provision(const char *devEui, const char *appEui, const char *appKey)
{
    if (!ttn_provisioning_decode_keys(&provisioning, devEui, appEui, appKey))
        return false;

    return ttn_provisioning_save_keys();
}

bool ttn_provision_with_mac(const char *appEui, const char *appKey)
{
    if (!ttn_provisioning_from_mac(&provisioning, appEui, appKey))
        return false;

    return ttn_provisioning_save_keys();
}


void ttn_start_provisioning_task()
{
#if defined(TTN_HAS_AT_COMMANDS)
    ttn_provisioning_start_task(&provisioning);
#else
    ESP_LOGE(TAG, "AT commands are disabled. Change the configuration using 'make menuconfig'");
    ASSERT(0);
    esp_restart();
#endif
}

void ttn_wait_for_provisioning()
{
#if defined(TTN_HAS_AT_COMMANDS)
    if (ttn_is_provisioned())
    {
        ESP_LOGI(TAG, "Device is already provisioned");
        return;
    }

    while (!ttn_provisioning_have_keys(&provisioning))
        vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "Device successfully provisioned");
#else
    ESP_LOGE(TAG, "AT commands are disabled. Change the configuration using 'make menuconfig'");
    ASSERT(0);
    esp_restart();
#endif
}

bool ttn_join_with_params(const char *devEui, const char *appEui, const char *appKey)
{
    if (!ttn_provisioning_decode_keys(&provisioning, devEui, appEui, appKey))
        return false;

    return ttn_join_core();
}

bool ttn_join()
{
    if (!ttn_provisioning_have_keys(&provisioning))
    {
        if (!ttn_provisioning_restore_keys(&provisioning, false))
            return false;
    }

    return ttn_join_core();
}

bool ttn_join_core()
{
    if (!ttn_provisioning_have_keys(&provisioning))
    {
        ESP_LOGW(TAG, "Device EUI, App EUI and/or App key have not been provided");
        return false;
    }

    ttn_hal_esp_enter_critical_section(&ttn_hal);
    xQueueReset(lmicEventQueue);
    waitingReason = eWaitingForJoin;
    LMIC_startJoining();
    ttn_hal_esp_wake_up();
    ttn_hal_esp_leave_critical_section(&ttn_hal);

    TTNLmicEvent event = ttn_lmic_event_default;
    xQueueReceive(lmicEventQueue, &event, portMAX_DELAY);
    return event.event == eEvtJoinCompleted;
}

TTNResponseCode ttn_transmit_message(const uint8_t *payload, size_t length, port_t port, bool confirm)
{
    ttn_hal_esp_enter_critical_section(&ttn_hal);
    if (waitingReason != eWaitingNone || (LMIC.opmode & OP_TXRXPEND) != 0)
    {
        ttn_hal_esp_leave_critical_section(&ttn_hal);
        return kTTNErrorTransmissionFailed;
    }

    waitingReason = eWaitingForTransmission;
    LMIC.client.txMessageCb = messageTransmittedCallback;
    LMIC.client.txMessageUserData = NULL;
    LMIC_setTxData2(port, (xref2u1_t)payload, length, confirm);
    ttn_hal_esp_wake_up();
    ttn_hal_esp_leave_critical_section(&ttn_hal);

    while (true)
    {
        TTNLmicEvent result = ttn_lmic_event_default;
        xQueueReceive(lmicEventQueue, &result, portMAX_DELAY);

        switch (result.event)
        {
            case eEvtMessageReceived:
                if (messageCallback != NULL)
                    messageCallback(result.message, result.messageSize, result.port);
                break;

            case eEvtTransmissionCompleted:
                return kTTNSuccessfulTransmission;

            case eEvtTransmissionFailed:
                return kTTNErrorTransmissionFailed;

            default:
                ASSERT(0);
        }
    }
}

void ttn_on_message(TTNMessageCallback callback)
{
    messageCallback = callback;
}


bool ttn_is_provisioned()
{
    if (ttn_provisioning_have_keys(&provisioning))
        return true;

    ttn_provisioning_restore_keys(&provisioning, true);

    return ttn_provisioning_have_keys(&provisioning);
}

void ttn_set_rssi_cal(int8_t rssiCal)
{
    ttn_hal.rssi_cal = rssiCal;
}

bool ttn_adr_enabled()
{
    return LMIC.adrEnabled != 0;
}

void ttn_set_adr_enabled(bool enabled)
{
    LMIC_setAdrMode(enabled);
}

TTNRFSettings ttn_get_rf_settings(TTNRxTxWindow window)
{
    int index = (int)(window) & 0x03;
    return lastRfSettings[index];
}

TTNRFSettings ttn_tx_settings()
{
    return lastRfSettings[(int)(kTTNTxWindow)];
}

TTNRFSettings ttn_rx1_settings()
{
    return lastRfSettings[(int)(kTTNRx1Window)];
}

TTNRFSettings ttn_rx2_settings()
{
    return lastRfSettings[(int)(kTTNRx2Window)];
}

TTNRxTxWindow ttn_rx_tx_window()
{
    return currentWindow;
}

int ttn_rssi()
{
    return LMIC.rssi;
}


// --- Callbacks ---

#if CONFIG_LOG_DEFAULT_LEVEL >= 3 || LMIC_ENABLE_event_logging
const char *eventNames[] = { LMIC_EVENT_NAME_TABLE__INIT };
#endif


// Called by LMIC when an LMIC event (join, join failed, reset etc.) occurs
void eventCallback(void* userData, ev_t event)
{
    // update monitoring information
    switch(event)
    {
        case EV_TXSTART:
            currentWindow = kTTNTxWindow;
            saveRFSettings(&lastRfSettings[(int)(kTTNTxWindow)]);
            clearRFSettings(&lastRfSettings[(int)(kTTNRx1Window)]);
            clearRFSettings(&lastRfSettings[(int)(kTTNRx2Window)]);
            break;

        case EV_RXSTART:
            if (currentWindow != kTTNRx1Window)
            {
                currentWindow = kTTNRx1Window;
                saveRFSettings(&lastRfSettings[(int)(kTTNRx1Window)]);
            }
            else
            {
                currentWindow = kTTNRx2Window;
                saveRFSettings(&lastRfSettings[(int)(kTTNRx2Window)]);
            }
            break;

        default:
            currentWindow = kTTNIdleWindow;
            break;
    };

#if LMIC_ENABLE_event_logging
    ttn_log_event(logging, event, eventNames[event], 0);
#elif CONFIG_LOG_DEFAULT_LEVEL >= 3
    ESP_LOGI(TAG, "event %s", eventNames[event]);
#endif

    TTNEvent ttnEvent = eEvtNone;

    if (waitingReason == eWaitingForJoin)
    {
        if (event == EV_JOINED)
        {
            ttnEvent = eEvtJoinCompleted;
        }
        else if (event == EV_REJOIN_FAILED || event == EV_RESET)
        {
            ttnEvent = eEvtJoinFailed;
        }
    }

    if (ttnEvent == eEvtNone)
        return;

    TTNLmicEvent result = ttn_lmic_event_default;
    result.event = ttnEvent;
    waitingReason = eWaitingNone;
    xQueueSend(lmicEventQueue, &result, pdMS_TO_TICKS(100));
}

// Called by LMIC when a message has been received
void messageReceivedCallback(void *userData, uint8_t port, const uint8_t *message, size_t nMessage)
{
    TTNLmicEvent result = ttn_lmic_event_default;
    result.event = eEvtMessageReceived;
    result.port = port;
    result.message = message;
    result.messageSize = nMessage;
    xQueueSend(lmicEventQueue, &result, pdMS_TO_TICKS(100));
}

// Called by LMIC when a message has been transmitted (or the transmission failed)
void messageTransmittedCallback(void *userData, int success)
{
    waitingReason = eWaitingNone;
    TTNLmicEvent result = ttn_lmic_event_default;
    result.event = (success ? eEvtTransmissionCompleted : eEvtTransmissionFailed);
    xQueueSend(lmicEventQueue, &result, pdMS_TO_TICKS(100));
}


// --- Helpers


void saveRFSettings(TTNRFSettings* rfSettings)
{
    rfSettings->spreadingFactor = (TTNSpreadingFactor)(getSf(LMIC.rps) + 1);
    rfSettings->bandwidth = (TTNBandwidth)(getBw(LMIC.rps) + 1);
    rfSettings->frequency = LMIC.freq;
}

void clearRFSettings(TTNRFSettings* rfSettings)
{
    memset(rfSettings, 0, sizeof(*rfSettings));
}
