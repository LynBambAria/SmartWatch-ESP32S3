/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void ble_app_main(void);
void ble_send_heart_rate(uint8_t heart_rate);
void ble_send_steps(uint32_t steps);

/* Attributes State Machine */
/* 
 * UUID 说明（给Flutter APP参考）:
 * Service UUID: 00FF (16-bit)
 * - Heart Rate Characteristic: FF01, supports READ + WRITE + NOTIFY
 * - Steps Characteristic:    FF02, supports READ + WRITE + NOTIFY (预留)
 * - Command Characteristic:   FF03, supports READ + WRITE
 */
enum
{
    IDX_SVC,
    IDX_CHAR_HEART_RATE,      // 心率特征值
    IDX_CHAR_VAL_HEART_RATE,  // 心率值
    IDX_CHAR_CFG_HEART_RATE,  // CCC描述符（用于notify）

    IDX_CHAR_STEPS,           // 步数特征值（预留）
    IDX_CHAR_VAL_STEPS,       // 步数值

    IDX_CHAR_COMMAND,         // 命令特征值（预留，APP->手表）
    IDX_CHAR_VAL_COMMAND,     // 命令值

    HRS_IDX_NB,
};
