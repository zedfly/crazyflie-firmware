/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2021 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * @file esp_rom_bootloader.c
 * Driver for communicating with the ESP32 ROM bootloader
 *  
 */

#define DEBUG_MODULE "ESPROMBL"

#include <string.h>

#include "FreeRTOS.h"
#include "debug.h"
#include "deck.h"
#include "esp_rom_bootloader.h"
#include "esp_slip.h"
#include "queue.h"
#include "task.h"
#include "uart2.h"
#include <math.h>

static esp_uart_send_packet sender_pckt;
static esp_uart_receive_packet receiver_pckt;

void espblInit()
{
    pinMode(DECK_GPIO_IO1, OUTPUT);
    digitalWrite(DECK_GPIO_IO1, LOW);
    pinMode(DECK_GPIO_IO4, OUTPUT);
    digitalWrite(DECK_GPIO_IO4, LOW);
    vTaskDelay(10);
    digitalWrite(DECK_GPIO_IO4, HIGH);
    pinMode(DECK_GPIO_IO4, INPUT_PULLUP);
    vTaskDelay(100);
    digitalWrite(DECK_GPIO_IO1, HIGH);
    pinMode(DECK_GPIO_IO1, INPUT_PULLUP);
}

bool espblSync(uint8_t *send_buffer)
{
    sender_pckt.command = SYNC;
    sender_pckt.data_size = 0x24;
    send_buffer[9 + 0] = 0x07;
    send_buffer[9 + 1] = 0x07;
    send_buffer[9 + 2] = 0x12;
    send_buffer[9 + 3] = 0x20;
    for (int i = 0; i < 32; i++)
    {
        send_buffer[9 + 4 + i] = 0x55;
    }

    bool sync = false;
    for (int i = 0; i < 10 && !sync; i++) // maximum 10 sync attempts
    {
        sync = espblExchange(send_buffer, &receiver_pckt, &sender_pckt, uart2SendDataDmaBlocking, uart2GetDataWithTimeout, 100);
    }

    // ESP32 responds multiple times upon succesful SYNC. Wait until all responses are received, so they can be cleared before next transmission.
    vTaskDelay(M2T(100));

    return sync;
}

bool spiAttach(uint8_t *send_buffer)
{
    sender_pckt.command = SPI_ATTACH;
    sender_pckt.data_size = 0x4;
    send_buffer[9 + 0] = 0x00;
    send_buffer[9 + 1] = 0x00;
    send_buffer[9 + 2] = 0x00;
    send_buffer[9 + 3] = 0x00;
    send_buffer[9 + 4] = 0x00;
    send_buffer[9 + 5] = 0x00;
    send_buffer[9 + 6] = 0x00;
    send_buffer[9 + 7] = 0x00;

    return espblExchange(send_buffer, &receiver_pckt, &sender_pckt, uart2SendDataDmaBlocking, uart2GetDataWithTimeout, 100);
}

bool espblFlashBegin(uint8_t *send_buffer, uint32_t number_of_data_packets, uint32_t firmware_size, uint32_t flash_offset)
{
    sender_pckt.command = FLASH_BEGIN;
    sender_pckt.data_size = 0x10;
    send_buffer[9 + 0] = (uint8_t)((firmware_size >> 0) & 0x000000FF);
    send_buffer[9 + 1] = (uint8_t)((firmware_size >> 8) & 0x000000FF);
    send_buffer[9 + 2] = (uint8_t)((firmware_size >> 16) & 0x000000FF);
    send_buffer[9 + 3] = (uint8_t)((firmware_size >> 24) & 0x000000FF);

    send_buffer[9 + 4] = (uint8_t)((number_of_data_packets >> 0) & 0x000000FF);
    send_buffer[9 + 5] = (uint8_t)((number_of_data_packets >> 8) & 0x000000FF);
    send_buffer[9 + 6] = (uint8_t)((number_of_data_packets >> 16) & 0x000000FF);
    send_buffer[9 + 7] = (uint8_t)((number_of_data_packets >> 24) & 0x000000FF);

    send_buffer[9 + 8] = (uint8_t)((ESP_MTU >> 0) & 0x000000FF);
    send_buffer[9 + 9] = (uint8_t)((ESP_MTU >> 8) & 0x000000FF);
    send_buffer[9 + 10] = (uint8_t)((ESP_MTU >> 16) & 0x000000FF);
    send_buffer[9 + 11] = (uint8_t)((ESP_MTU >> 24) & 0x000000FF);
    send_buffer[9 + 12] = (uint8_t)((flash_offset >> 0) & 0x000000FF);
    send_buffer[9 + 13] = (uint8_t)((flash_offset >> 8) & 0x000000FF);
    send_buffer[9 + 14] = (uint8_t)((flash_offset >> 16) & 0x000000FF);
    send_buffer[9 + 15] = (uint8_t)((flash_offset >> 24) & 0x000000FF);

    return espblExchange(send_buffer, &receiver_pckt, &sender_pckt, uart2SendDataDmaBlocking, uart2GetDataWithTimeout, 10000);
}

bool espblFlashData(uint8_t *send_buffer, uint32_t flash_data_size, uint32_t sequence_number)
{
    sender_pckt.command = FLASH_DATA;
    sender_pckt.data_size = ESP_MTU + 16; // set data size to the data size including the additional header

    send_buffer[9 + 0] = (uint8_t)((ESP_MTU >> 0) & 0x000000FF);
    send_buffer[9 + 1] = (uint8_t)((ESP_MTU >> 8) & 0x000000FF);
    send_buffer[9 + 2] = (uint8_t)((ESP_MTU >> 16) & 0x000000FF);
    send_buffer[9 + 3] = (uint8_t)((ESP_MTU >> 24) & 0x000000FF);

    send_buffer[9 + 4] = (uint8_t)((sequence_number >> 0) & 0x000000FF);
    send_buffer[9 + 5] = (uint8_t)((sequence_number >> 8) & 0x000000FF);
    send_buffer[9 + 6] = (uint8_t)((sequence_number >> 16) & 0x000000FF);
    send_buffer[9 + 7] = (uint8_t)((sequence_number >> 24) & 0x000000FF);

    send_buffer[9 + 8] = 0x00;
    send_buffer[9 + 9] = 0x00;
    send_buffer[9 + 10] = 0x00;
    send_buffer[9 + 11] = 0x00;
    send_buffer[9 + 12] = 0x00;
    send_buffer[9 + 13] = 0x00;
    send_buffer[9 + 14] = 0x00;
    send_buffer[9 + 15] = 0x00;

    if (flash_data_size < ESP_MTU)
    {
        // pad the data with 0xFF
        memset(&send_buffer[9 + 16 + flash_data_size], 0xFF, ESP_MTU - flash_data_size);
    }

    return espblExchange(send_buffer, &receiver_pckt, &sender_pckt, uart2SendDataDmaBlocking, uart2GetDataWithTimeout, 100);
}




