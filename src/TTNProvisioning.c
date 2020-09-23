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

#include "freertos/FreeRTOS.h"
#include "driver/uart.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "TTNProvisioning.h"
#include "lmic/lmic.h"
#include "hal/hal_esp32.h"

#if defined(TTN_HAS_AT_COMMANDS)
const uart_port_t UART_NUM = (uart_port_t) CONFIG_TTN_PROVISION_UART_NUM;
const int MAX_LINE_LENGTH = 128;
#endif

static const char* const TAG = "ttn_prov";
static const char* const NVS_FLASH_PARTITION = "ttn";
static const char* const NVS_FLASH_KEY_DEV_EUI = "devEui";
static const char* const NVS_FLASH_KEY_APP_EUI = "appEui";
static const char* const NVS_FLASH_KEY_APP_KEY = "appKey";

static uint8_t global_dev_eui[8];
static uint8_t global_app_eui[8];
static uint8_t global_app_key[16];

bool ttn_provisioning_decode(ttn_provisioning* provisioning, bool incl_dev_eui, const char *dev_eui, const char *app_eui, const char *app_key);
bool ttn_provisioning_read_nvs_value(nvs_handle handle, const char* key, uint8_t* data, size_t expected_length, bool silent);
bool ttn_provisioning_write_nvs_value(nvs_handle handle, const char* key, const uint8_t* data, size_t len);

#if defined(TTN_HAS_AT_COMMANDS)
void ttn_provisioning_provisioning_task(ttn_provisioning* provisioning);
void ttn_provisioning_add_line_data(ttn_provisioning* provisioning, int numBytes);
void ttn_provisioning_detect_line_end(ttn_provisioning* provisioning, int start_at);
void ttn_provisioning_process_line(ttn_provisioning* provisioning);
void ttn_provisioning_task_caller(void* pvParameter);
#endif


#if defined(TTN_CONFIG_UART)
void ttn_provisioning_config_uart();
#endif

static bool ttn_helper_hex_str_to_bin(const char *hex, uint8_t *buf, int len);
static int ttn_helper_hex_tuple_to_byte(const char *hex);
static int ttn_helper_hex_digit_to_val(char ch);
static void ttn_helper_bin_to_hex_str(const uint8_t* buf, int len, char* hex);
static char ttn_helper_val_to_hex_digit(int val);
static void ttn_helper_swap_bytes(uint8_t* buf, int len);
static bool ttn_helper_is_all_zeros(const uint8_t* buf, int len);

// --- LMIC callbacks

// This EUI must be in little-endian format, so least-significant-byte first.
// When copying an EUI from ttnctl output, this means to reverse the bytes.
// For TTN issued EUIs the last bytes should be 0xD5, 0xB3, 0x70.
// The order is swapped in provisioning_decode_keys().
void os_getArtEui (u1_t* buf)
{
    memcpy(buf, global_app_eui, 8);
}

// This should also be in little endian format, see above.
void os_getDevEui (u1_t* buf)
{
    memcpy(buf, global_dev_eui, 8);
}

// This key should be in big endian format (or, since it is not really a number
// but a block of memory, endianness does not really apply). In practice, a key
// taken from ttnctl can be copied as-is.
void os_getDevKey (u1_t* buf)
{
    memcpy(buf, global_app_key, 16);
}

// --- Constructor
void ttn_provisioning_init(ttn_provisioning* provisioning) {
    provisioning->have_keys = false;

#if defined(TTN_HAS_AT_COMMANDS)
    provisioning->uart_queue = NULL;
    provisioning->line_buf = NULL;
    provisioning->line_length = 0;
    provisioning->last_line_end_char = 0;
    provisioning->quit_task = false;
#endif
}

// --- Provisioning task

#if defined(TTN_HAS_AT_COMMANDS)

void ttn_provisioning_start_task(ttn_provisioning* provisioning)
{
#if defined(TTN_CONFIG_UART)
    ttn_provisioning_config_uart(provisioning);
#endif

    esp_err_t err = uart_driver_install(UART_NUM, 2048, 2048, 20, &provisioning->uart_queue, 0);
    ESP_ERROR_CHECK(err);

    xTaskCreate(ttn_provisioning_task_caller, "ttn_provision", 2048, provisioning, 1, NULL);
}

void ttn_provisioning_task_caller(void* pvParameter)
{
    ttn_provisioning* provisioning = (ttn_provisioning*)pvParameter;
    ttn_provisioning_provisioning_task(provisioning);
    vTaskDelete(NULL);
}

void ttn_provisioning_provisioning_task(ttn_provisioning* provisioning)
{
    provisioning->line_buf = (char*)malloc(MAX_LINE_LENGTH + 1);
    provisioning->line_length = 0;

    uart_event_t event;

    ESP_LOGI(TAG, "Provisioning task started");

    while (!provisioning->quit_task)
    {
        if (!xQueueReceive(provisioning->uart_queue, &event, portMAX_DELAY))
            continue;

        switch (event.type)
        {
            case UART_DATA:
                ttn_provisioning_add_line_data(provisioning, event.size);
                break;

            case UART_FIFO_OVF:
            case UART_BUFFER_FULL:
                uart_flush_input(UART_NUM);
                xQueueReset(provisioning->uart_queue);
                break;

            default:
                break;
        }
    }

    free(provisioning->line_buf);
    uart_driver_delete(UART_NUM);
}

void ttn_provisioning_add_line_data(ttn_provisioning* provisioning, int numBytes)
{
    int n;
top:
    n = numBytes;
    if (provisioning->line_length + n > MAX_LINE_LENGTH)
        n = MAX_LINE_LENGTH - provisioning->line_length;

    uart_read_bytes(UART_NUM, (uint8_t*)provisioning->line_buf + provisioning->line_length, n, portMAX_DELAY);
    int start_at = provisioning->line_length;
    provisioning->line_length += n;

    ttn_provisioning_detect_line_end(provisioning, start_at);

    if (n < numBytes)
    {
        numBytes -= n;
        goto top;
    }
}

void ttn_provisioning_detect_line_end(ttn_provisioning* provisioning, int start_at)
{
top:
    for (int p = start_at; p < provisioning->line_length; p++)
    {
        char ch = provisioning->line_buf[p];
        if (ch == 0x0d || ch == 0x0a)
        {
            if (p > 0)
                uart_write_bytes(UART_NUM, provisioning->line_buf + start_at, provisioning->line_length - start_at - 1);
            if (p > 0 || ch == 0x0d || provisioning->last_line_end_char == 0x0a)
                uart_write_bytes(UART_NUM, "\r\n", 2);

            provisioning->line_buf[p] = 0;
            provisioning->last_line_end_char = ch;

            if (p > 0)
                ttn_provisioning_process_line(provisioning);

            memcpy(provisioning->line_buf, provisioning->line_buf + p + 1, provisioning->line_length - p - 1);
            provisioning->line_length -= p + 1;
            start_at = 0;
            goto top;
        }
    }

    if (provisioning->line_length > 0)
        uart_write_bytes(UART_NUM, provisioning->line_buf + start_at, provisioning->line_length - start_at);

    if (provisioning->line_length == MAX_LINE_LENGTH)
        provisioning->line_length = 0; // Line too long; flush it
}

void ttn_provisioning_process_line(ttn_provisioning* provisioning)
{
    bool is_ok = true;
    bool reset_needed = false;
    char* line_buf = provisioning->line_buf;
    // Expected format:
    // AT+PROV?
    // AT+PROV=hex16-hex16-hex32
    // AT+PROVM=hex16-hex32
    // AT+MAC?
    // AT+HWEUI?

    if (strcmp(line_buf, "AT+PROV?") == 0)
    {
        uint8_t binbuf[8];
        char hexbuf[16];

        memcpy(binbuf, global_dev_eui, 8);
        ttn_helper_swap_bytes(binbuf, 8);
        ttn_helper_bin_to_hex_str(binbuf, 8, hexbuf);
        uart_write_bytes(UART_NUM, hexbuf, 16);
        uart_write_bytes(UART_NUM, "-", 1);

        memcpy(binbuf, global_app_eui, 8);
        ttn_helper_swap_bytes(binbuf, 8);
        ttn_helper_bin_to_hex_str(binbuf, 8, hexbuf);
        uart_write_bytes(UART_NUM, hexbuf, 16);

        uart_write_bytes(UART_NUM, "-00000000000000000000000000000000\r\n", 35);
    }
    else if (strncmp(line_buf, "AT+PROV=", 8) == 0)
    {
        is_ok  = strlen(line_buf) == 74 && line_buf[24] == '-' && line_buf[41] == '-';
        if (is_ok)
        {
            line_buf[24] = 0;
            line_buf[41] = 0;
            is_ok = ttn_provisioning_decode_keys(provisioning, line_buf + 8, line_buf + 25, line_buf + 42);
            reset_needed = is_ok;
        }
    }
    else if (strncmp(line_buf, "AT+PROVM=", 8) == 0)
    {
        is_ok = strlen(line_buf) == 58 && line_buf[25] == '-';
        if (is_ok)
        {
            line_buf[25] = 0;
            is_ok = ttn_provisioning_from_mac(provisioning, line_buf + 9, line_buf + 26);
            reset_needed = is_ok;
        }
    }
    else if (strcmp(line_buf, "AT+MAC?") == 0)
    {
        uint8_t mac[6];
        char hexbuf[12];

        esp_err_t err = esp_efuse_mac_get_default(mac);
        ESP_ERROR_CHECK(err);

        ttn_helper_bin_to_hex_str(mac, 6, hexbuf);
        for (int i = 0; i < 12; i += 2) {
            if (i > 0)
                uart_write_bytes(UART_NUM, ":", 1);
            uart_write_bytes(UART_NUM, hexbuf + i, 2);
        }
        uart_write_bytes(UART_NUM, "\r\n", 2);
    }
    else if (strcmp(line_buf, "AT+HWEUI?") == 0)
    {
        uint8_t mac[6];
        char hexbuf[12];

        esp_err_t err = esp_efuse_mac_get_default(mac);
        ESP_ERROR_CHECK(err);

        ttn_helper_bin_to_hex_str(mac, 6, hexbuf);
        for (int i = 0; i < 12; i += 2) {
            uart_write_bytes(UART_NUM, hexbuf + i, 2);
            if (i == 4)
              uart_write_bytes(UART_NUM, "FFFE", 4);
        }
        uart_write_bytes(UART_NUM, "\r\n", 2);
    }
    else if (strcmp(line_buf, "AT+PROVQ") == 0)
    {
        provisioning->quit_task = true;
    }
    else if (strcmp(line_buf, "AT") != 0)
    {
        is_ok = false;
    }

    if (reset_needed)
    {
        ttn_hal_esp_enter_critical_section(&ttn_hal);
        LMIC_reset();
        ttn_hal_esp_leave_critical_section(&ttn_hal);
        LMIC.client.eventCb(LMIC.client.eventUserData, EV_RESET);
    }

    uart_write_bytes(UART_NUM, is_ok ? "OK\r\n" : "ERROR\r\n", is_ok ? 4 : 7);
}

#endif


#if defined(TTN_CONFIG_UART)

void ttn_provisioning_config_uart()
{
    esp_err_t err;

    uart_config_t uart_config = {
        .baud_rate = CONFIG_TTN_PROVISION_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .use_ref_tick = false
    };
    err = uart_param_config(UART_NUM, &uart_config);
    ESP_ERROR_CHECK(err);

    err = uart_set_pin(UART_NUM, CONFIG_TTN_PROVISION_UART_TX_GPIO, CONFIG_TTN_PROVISION_UART_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    ESP_ERROR_CHECK(err);
}

#endif


// --- Key handling

bool ttn_provisioning_have_keys(ttn_provisioning* provisioning)
{
    return provisioning->have_keys;
}

bool ttn_provisioning_decode_keys(ttn_provisioning* provisioning, const char *dev_eui, const char *app_eui, const char *app_key)
{
    return ttn_provisioning_decode(provisioning, true, dev_eui, app_eui, app_key);
}

bool ttn_provisioning_from_mac(ttn_provisioning* provisioning, const char *app_eui, const char *app_key)
{
    uint8_t mac[6];
    esp_err_t err = esp_efuse_mac_get_default(mac);
    ESP_ERROR_CHECK(err);

    global_dev_eui[7] = mac[0];
    global_dev_eui[6] = mac[1];
    global_dev_eui[5] = mac[2];
    global_dev_eui[4] = 0xff;
    global_dev_eui[3] = 0xfe;
    global_dev_eui[2] = mac[3];
    global_dev_eui[1] = mac[4];
    global_dev_eui[0] = mac[5];

    return ttn_provisioning_decode(provisioning, false, NULL, app_eui, app_key);
}

bool ttn_provisioning_decode(ttn_provisioning* provisioning, bool incl_dev_eui, const char *dev_eui, const char *app_eui, const char *app_key)
{
    uint8_t buf_dev_eui[8];
    uint8_t buf_app_eui[8];
    uint8_t buf_app_key[16];

    if (incl_dev_eui && (strlen(dev_eui) != 16 || !ttn_helper_hex_str_to_bin(dev_eui, buf_dev_eui, 8)))
    {
        ESP_LOGW(TAG, "Invalid device EUI: %s", dev_eui);
        return false;
    }

    if (incl_dev_eui)
        ttn_helper_swap_bytes(buf_dev_eui, 8);

    if (strlen(app_eui) != 16 || !ttn_helper_hex_str_to_bin(app_eui, buf_app_eui, 8))
    {
        ESP_LOGW(TAG, "Invalid application EUI: %s", app_eui);
        return false;
    }

    ttn_helper_swap_bytes(buf_app_eui, 8);

    if (strlen(app_key) != 32 || !ttn_helper_hex_str_to_bin(app_key, buf_app_key, 16))
    {
        ESP_LOGW(TAG, "Invalid application key: %s", app_key);
        return false;
    }

    if (incl_dev_eui)
        memcpy(global_dev_eui, buf_dev_eui, sizeof(global_dev_eui));
    memcpy(global_app_eui, buf_app_eui, sizeof(global_app_eui));
    memcpy(global_app_key, buf_app_key, sizeof(global_app_key));

    provisioning->have_keys = !ttn_helper_is_all_zeros(global_dev_eui, sizeof(global_dev_eui))
        && !ttn_helper_is_all_zeros(global_app_eui, sizeof(global_app_eui))
        && !ttn_helper_is_all_zeros(global_app_key, sizeof(global_app_key));

    if (!ttn_provisioning_save_keys())
        return false;

    return true;
}


// --- Non-volatile storage

bool ttn_provisioning_save_keys()
{
    bool result = false;

    nvs_handle handle = 0;
    esp_err_t res = nvs_open(NVS_FLASH_PARTITION, NVS_READWRITE, &handle);
    if (res == ESP_ERR_NVS_NOT_INITIALIZED)
    {
        ESP_LOGW(TAG, "NVS storage is not initialized. Call 'nvs_flash_init()' first.");
        goto done;
    }
    ESP_ERROR_CHECK(res);
    if (res != ESP_OK)
        goto done;

    if (!ttn_provisioning_write_nvs_value(handle, NVS_FLASH_KEY_DEV_EUI, global_dev_eui, sizeof(global_dev_eui)))
        goto done;

    if (!ttn_provisioning_write_nvs_value(handle, NVS_FLASH_KEY_APP_EUI, global_app_eui, sizeof(global_app_eui)))
        goto done;

    if (!ttn_provisioning_write_nvs_value(handle, NVS_FLASH_KEY_APP_KEY, global_app_key, sizeof(global_app_key)))
        goto done;

    res = nvs_commit(handle);
    ESP_ERROR_CHECK(res);

    result = true;
    ESP_LOGI(TAG, "Dev and app EUI and app key saved in NVS storage");

done:
    nvs_close(handle);
    return result;
}

bool ttn_provisioning_restore_keys(ttn_provisioning* provisioning, bool silent)
{
    uint8_t buf_dev_eui[8];
    uint8_t buf_app_eui[8];
    uint8_t buf_app_key[16];

    nvs_handle handle = 0;
    esp_err_t res = nvs_open(NVS_FLASH_PARTITION, NVS_READONLY, &handle);
    if (res == ESP_ERR_NVS_NOT_FOUND)
        return false; // partition does not exist yet
    if (res == ESP_ERR_NVS_NOT_INITIALIZED)
    {
        ESP_LOGW(TAG, "NVS storage is not initialized. Call 'nvs_flash_init()' first.");
        goto done;
    }
    ESP_ERROR_CHECK(res);
    if (res != ESP_OK)
        goto done;

    if (!ttn_provisioning_read_nvs_value(handle, NVS_FLASH_KEY_DEV_EUI, buf_dev_eui, sizeof(global_dev_eui), silent))
        goto done;

    if (!ttn_provisioning_read_nvs_value(handle, NVS_FLASH_KEY_APP_EUI, buf_app_eui, sizeof(global_app_eui), silent))
        goto done;

    if (!ttn_provisioning_read_nvs_value(handle, NVS_FLASH_KEY_APP_KEY, buf_app_key, sizeof(global_app_key), silent))
        goto done;

    memcpy(global_dev_eui, buf_dev_eui, sizeof(global_dev_eui));
    memcpy(global_app_eui, buf_app_eui, sizeof(global_app_eui));
    memcpy(global_app_key, buf_app_key, sizeof(global_app_key));

    provisioning->have_keys = !ttn_helper_is_all_zeros(global_dev_eui, sizeof(global_dev_eui))
        && !ttn_helper_is_all_zeros(global_app_eui, sizeof(global_app_eui))
        && !ttn_helper_is_all_zeros(global_app_key, sizeof(global_app_key));

    if (provisioning->have_keys)
    {
       ESP_LOGI(TAG, "Dev and app EUI and app key have been restored from NVS storage");
    }
    else
    {
        ESP_LOGW(TAG, "Dev and app EUI and app key are invalid (zeroes only)");
    }

done:
    nvs_close(handle);
    return true;
}

bool ttn_provisioning_read_nvs_value(nvs_handle handle, const char* key, uint8_t* data, size_t expected_length, bool silent)
{
    size_t size = expected_length;
    esp_err_t res = nvs_get_blob(handle, key, data, &size);
    if (res == ESP_OK && size == expected_length)
        return true;

    if (res == ESP_OK && size != expected_length)
    {
        if (!silent)
            ESP_LOGW(TAG, "Invalid size of NVS data for %s", key);
        return false;
    }

    if (res == ESP_ERR_NVS_NOT_FOUND)
    {
        if (!silent)
            ESP_LOGW(TAG, "No NVS data found for %s", key);
        return false;
    }

    ESP_ERROR_CHECK(res);
    return false;
}

bool ttn_provisioning_write_nvs_value(nvs_handle handle, const char* key, const uint8_t* data, size_t len)
{
    uint8_t buf[16];
    if (ttn_provisioning_read_nvs_value(handle, key, buf, len, true) && memcmp(buf, data, len) == 0)
        return true; // unchanged

    esp_err_t res = nvs_set_blob(handle, key, data, len);
    ESP_ERROR_CHECK(res);

    return res == ESP_OK;
}


// --- Helper functions ---

static bool ttn_helper_hex_str_to_bin(const char *hex, uint8_t *buf, int len)
{
    const char* ptr = hex;
    for (int i = 0; i < len; i++)
    {
        int val = ttn_helper_hex_tuple_to_byte(ptr);
        if (val < 0)
            return false;
        buf[i] = val;
        ptr += 2;
    }
    return true;
}

static int ttn_helper_hex_tuple_to_byte(const char *hex)
{
    int nibble1 = ttn_helper_hex_digit_to_val(hex[0]);
    if (nibble1 < 0)
        return -1;
    int nibble2 = ttn_helper_hex_digit_to_val(hex[1]);
    if (nibble2 < 0)
        return -1;
    return (nibble1 << 4) | nibble2;
}

static int ttn_helper_hex_digit_to_val(char ch)
{
    if (ch >= '0' && ch <= '9')
        return ch - '0';
    if (ch >= 'A' && ch <= 'F')
        return ch + 10 - 'A';
    if (ch >= 'a' && ch <= 'f')
        return ch + 10 - 'a';
    return -1;
}

static void ttn_helper_bin_to_hex_str(const uint8_t* buf, int len, char* hex)
{
    for (int i = 0; i < len; i++)
    {
        uint8_t b = buf[i];
        *hex = ttn_helper_val_to_hex_digit((b & 0xf0) >> 4);
        hex++;
        *hex = ttn_helper_val_to_hex_digit(b & 0x0f);
        hex++;
    }
}

static char ttn_helper_val_to_hex_digit(int val)
{
    return "0123456789ABCDEF"[val];
}

static void ttn_helper_swap_bytes(uint8_t* buf, int len)
{
    uint8_t* p1 = buf;
    uint8_t* p2 = buf + len - 1;
    while (p1 < p2)
    {
        uint8_t t = *p1;
        *p1 = *p2;
        *p2 = t;
        p1++;
        p2--;
    }
}

static bool ttn_helper_is_all_zeros(const uint8_t* buf, int len)
{
    for (int i = 0; i < len; i++)
        if (buf[i] != 0)
            return false;
    return true;
}