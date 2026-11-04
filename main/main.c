#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"

static const char *TAG = "RS485";

#define RS485_UART   UART_NUM_1
#define RS485_TX_PIN 6
#define RS485_RX_PIN 7
#define BUF_SIZE     256

#define SLAVE_ID  1
#define NUM_REGS  128

static uint16_t holding_regs[NUM_REGS];

static uint16_t crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
        }
    }
    return crc;
}

static void send_frame(uint8_t *frame, size_t len)
{
    uint16_t crc = crc16(frame, len);
    frame[len] = crc & 0xFF; // CRC is little-endian
    frame[len + 1] = crc >> 8;
    uart_write_bytes(RS485_UART, frame, len + 2);
}

static void send_exception(uint8_t func, uint8_t code)
{
    uint8_t resp[5] = { SLAVE_ID, func | 0x80, code };
    send_frame(resp, 3);
}

static void handle_frame(const uint8_t *f, int len)
{
    if (len < 8 || crc16(f, len - 2) != (uint16_t)(f[len - 2] | (f[len - 1] << 8))) {
        ESP_LOGW(TAG, "Bad CRC/short frame (%d bytes)", len);
        return;
    }
    if (f[0] != SLAVE_ID && f[0] != 0) {
        return; // addressed to another slave
    }
    bool reply = f[0] != 0; // no reply to broadcast
    uint8_t func = f[1];
    uint16_t addr = (f[2] << 8) | f[3];

    if (func == 0x06) {
        uint16_t value = (f[4] << 8) | f[5];
        if (addr >= NUM_REGS) {
            if (reply) send_exception(func, 0x02);
            return;
        }
        holding_regs[addr] = value;
        ESP_LOGI(TAG, "Write single: reg %u = %u", addr, value);
        if (reply) {
            uint8_t resp[8];
            memcpy(resp, f, 6); // response echoes the request
            send_frame(resp, 6);
        }
    } else if (func == 0x10) {
        uint16_t qty = (f[4] << 8) | f[5];
        uint8_t bytes = f[6];
        if (qty < 1 || qty > 123 || bytes != qty * 2 || len != 9 + bytes) {
            if (reply) send_exception(func, 0x03);
            return;
        }
        if (addr + qty > NUM_REGS) {
            if (reply) send_exception(func, 0x02);
            return;
        }
        for (int i = 0; i < qty; i++) {
            holding_regs[addr + i] = (f[7 + 2 * i] << 8) | f[8 + 2 * i];
            ESP_LOGI(TAG, "Write multiple: reg %u = %u", addr + i, holding_regs[addr + i]);
        }
        if (reply) {
            uint8_t resp[8];
            memcpy(resp, f, 6);
            send_frame(resp, 6);
        }
    } else if (reply) {
        send_exception(func, 0x01); // illegal function
    }
}

static void rs485_rx_task(void *arg)
{
    uint8_t frame[BUF_SIZE];
    while (1) {
        int len = uart_read_bytes(RS485_UART, frame, BUF_SIZE, pdMS_TO_TICKS(100));
        if (len <= 0) {
            continue;
        }
        // Keep reading until the line is idle: that gap marks the end of a frame
        int more;
        while (len < BUF_SIZE &&
               (more = uart_read_bytes(RS485_UART, frame + len, BUF_SIZE - len, pdMS_TO_TICKS(20))) > 0) {
            len += more;
        }
        ESP_LOG_BUFFER_HEX(TAG, frame, len);
        handle_frame(frame, len);
    }
}

void app_main(void)
{
    const uart_config_t cfg = {
        .baud_rate = 19200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(RS485_UART, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(RS485_UART, &cfg));
    // MAX13487 handles DE/RE automatically, so no direction pin is needed
    ESP_ERROR_CHECK(uart_set_pin(RS485_UART, RS485_TX_PIN, RS485_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    xTaskCreate(rs485_rx_task, "rs485_rx", 4096, NULL, 10, NULL);

    ESP_LOGI(TAG, "Modbus RTU slave ready, id=%d", SLAVE_ID);
}
