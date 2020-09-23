/*******************************************************************************
 * 
 * ttn-esp32 - The Things Network device library for ESP-IDF / SX127x
 * 
 * Copyright (c) 2018-2019 Manuel Bleichenbacher
 * 
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 *
 * Circular buffer for detailed logging without affecting LMIC timing.
 *******************************************************************************/

#ifndef _ttnlogging_h_
#define _ttnlogging_h_


#if LMIC_ENABLE_event_logging

#include <freertos/FreeRTOS.h>
#include <freertos/ringbuf.h>


/**
 * @brief Logging class.
 * 
 * Logs internal information from LMIC in an asynchrnous fashion in order
 * not to distrub the sensitive LORA timing.
 * 
 * A ring buffer and a separate logging task is ued. The LMIC core records
 * relevant values from the current LORA settings and writes them to a ring
 * buffer. The logging tasks receives the message and the values, formats
 * them and outputs them via the regular ESP-IDF logging mechanism.
 * 
 * In order to activate the detailed logging, set the macro
 * `LMIC_ENABLE_event_logging` to 1.
 * 
 * This class is not to be used directly.
 */

typedef struct {
    RingbufHandle_t ring_buffer;
} ttn_log;

void ttn_log_init(ttn_log* ttn_log);
void ttn_log_event(ttn_log* ttn_log, int event, const char* message, uint32_t datum);

static void ttn_log_task(void* param);
static void ttn_log_fatal(const char* file, uint16_t line);

extern ttn_log ttn_log_instance;

#endif

#endif
