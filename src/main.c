/**
 * @file   main.c
 * @brief  Application main.
 *
 * @author Cerevo Inc.
 */

/*
Copyright 2015 Cerevo Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "TZ01_system.h"
#include "TZ01_console.h"

#include "utils.h"

int BLE_tz1em_init(void);
int BLE_init(uint8_t id);
void BLE_term(void);
void BLE_main(void);

uint8_t msg[80];

int main(void)
{
    int ret;
    /* Initialize */
    ret = BLE_tz1em_init();
    TZ01_system_init();
    TZ01_console_init();
    
    sprintf(msg, "BLE_tz1em_init(): %d\r\n", ret);
    TZ01_console_puts(msg);
    
    ret = BLE_init(0);
    sprintf(msg, "BLE_init(): %d\r\n", ret);
    TZ01_console_puts(msg);
    if (ret != 0) {
        goto systerm;
    }
    

    TZ01_console_puts("Cerevo BlueNinja\r\n");
    TZ01_console_puts("BLE Central DEMO\r\n");
    
    for (;;) {
        if (TZ01_system_run() == RUNEVT_POWOFF) {
            /* Power off operation detected */
            break;
        }
        BLE_main();
    }
    
systerm:
    TZ01_system_term();
    TZ01_console_puts("Program terminated.\r\n");
    return 0;
}
