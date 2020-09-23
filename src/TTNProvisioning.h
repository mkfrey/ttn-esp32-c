/*******************************************************************************
 * 
 * ttn-esp32 - The Things Network device library for ESP-IDF / SX127x
 * 
 * Copyright (c) 2018-2019 Manuel Bleichenbacher
 * 
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 *
 * Task listening on a UART port for provisioning commands.
 *******************************************************************************/

#ifndef _ttnprovisioning_h_
#define _ttnprovisioning_h_

#include "lmic/oslmic.h"
#include "nvs_flash.h"

typedef struct {
    bool have_keys;
#if defined(TTN_HAS_AT_COMMANDS)
    QueueHandle_t uart_queue;
    char* line_buf;
    int line_length;
    uint8_t last_line_end_char;
    bool quit_task;
#endif
} ttn_provisioning;

void ttn_provisioning_init(ttn_provisioning* provisioning);
bool ttn_provisioning_have_keys(ttn_provisioning* provisioning);
bool ttn_provisioning_decode_keys(ttn_provisioning* provisioning, const char *dev_eui, const char *app_eui, const char *app_key);
bool ttn_provisioning_from_mac(ttn_provisioning* provisioning, const char *app_eui, const char *app_key);
bool ttn_provisioning_save_keys();
bool ttn_provisioning_restore_keys(ttn_provisioning* provisioning, bool silent);

#if defined(TTN_HAS_AT_COMMANDS)
    void ttn_provisioning_start_task(ttn_provisioning* provisioning);
#endif

#endif
