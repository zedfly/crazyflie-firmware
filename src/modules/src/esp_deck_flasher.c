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
 * @file esp_deck_flasher.c
 * Handles flashing of binaries on the ESP32
 *  
 */

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#define DEBUG_MODULE "ESPFL"
#include "debug.h"

#include "deck_core.h"
#include "esp_deck_flasher.h"
#include "esp_rom_bootloader.h"
#include "mem.h"

#include "FreeRTOS.h"
#include "task.h"

#include "aideck.h"
#include "deck.h"
#include "uart2.h"

static bool inBootloaderMode = true;
static bool hasStarted = false;

bool espDeckFlasherCheckVersionAndBoot()
{
    hasStarted = true;
    return true;
}

static uint32_t sequence_number;
static uint32_t number_of_data_packets;
static uint8_t send_buffer[ESP_MTU + 10 + 16];
static uint8_t overshoot = 0;
static uint32_t send_buffer_idx;

bool espDeckFlasherWrite(const uint32_t memAddr, const uint8_t writeLen, const uint8_t *buffer)
{
    if (memAddr == 0)
    {
        uart2Init(115200);
        espblInit();
        if (!espblSync(&send_buffer[0]))
        {
            DEBUG_PRINT("Sync failed\n");
            return false;
        }
        if (!spiAttach(&send_buffer[0]))
        {
            DEBUG_PRINT("SPI attach failed\n");
            return false;
        }

        number_of_data_packets = (((ESP_BITSTREAM_SIZE - 1) / ESP_MTU) + ((ESP_BITSTREAM_SIZE / ESP_MTU) < 0 ? 0 : 1)) >> 0;
        DEBUG_PRINT("Will send %lu data packets\n", number_of_data_packets);

        if (!espblFlashBegin(&send_buffer[0], number_of_data_packets, ESP_BITSTREAM_SIZE, ESP_FW_ADDRESS)) // placeholder erase size
        {
            DEBUG_PRINT("Failed to start flashing\n");
            return 0;
        }
            sequence_number = 0;
        send_buffer_idx = 0;
    }

    // assemble buffer until full
    if (send_buffer_idx + writeLen >= ESP_MTU)
    {
        overshoot = send_buffer_idx + writeLen - ESP_MTU;
        memcpy(&send_buffer[9 + 16 + send_buffer_idx], buffer, writeLen - overshoot);
        send_buffer_idx += writeLen - overshoot;
    }
    else
    {
        memcpy(&send_buffer[9 + 16 + send_buffer_idx], buffer, writeLen);
        send_buffer_idx += writeLen;
    }
    DEBUG_PRINT("send_buffer_idx: %lu\n", send_buffer_idx);

    // send buffer if full
    if (send_buffer_idx == ESP_MTU || ((sequence_number == number_of_data_packets - 1) && (send_buffer_idx == ESP_BITSTREAM_SIZE % ESP_MTU)))
    {
        if (!espblFlashData(&send_buffer[0], send_buffer_idx, sequence_number))
        {
            DEBUG_PRINT("Flash write failed\n");
            return false;
        }
        else
        {
            DEBUG_PRINT("Flash write successful\n");
        }

        // put overshoot into send buffer for next send & update send_buffer_idx
        if (overshoot)
        {
            DEBUG_PRINT("Overshoot: %d\n", overshoot);
            memcpy(&send_buffer[9 + 16 + 0], &buffer[writeLen - overshoot], overshoot);
            send_buffer_idx = overshoot;
            overshoot = 0;
        }
        else
        {
            send_buffer_idx = 0;
        }

        // increment sequence number
        sequence_number++;
        DEBUG_PRINT("Sequence number %lu\n", sequence_number);

        // if very last radio packet triggered overshoot, send padded carry buffer
        if (((sequence_number == number_of_data_packets - 1) && (send_buffer_idx == ESP_BITSTREAM_SIZE % ESP_MTU)))
        {
            DEBUG_PRINT("Last radio packet triggered overshoot of %lu bytes\n", send_buffer_idx);

            if (!espblFlashData(&send_buffer[0], send_buffer_idx, sequence_number))
            {
                DEBUG_PRINT("Flash write failed\n");
                return false;
            }
            else
            {
                DEBUG_PRINT("Flash write successful\n");
            }
        }
    }

    return true;
}

uint8_t espDeckFlasherPropertiesQuery()
{
    uint8_t result = 0;

    if (hasStarted)
    {
        result |= DECK_MEMORY_MASK_STARTED;
    }

    if (inBootloaderMode)
    {
        result |= DECK_MEMORY_MASK_BOOT_LOADER_ACTIVE | DECK_MEMORY_MASK_UPGRADE_REQUIRED;
    }

    return result;
}
