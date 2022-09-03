#ifndef DWM1001_UART_PICO_H
#define DWM1001_UART_PICO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "dwm_api.h"

typedef struct
{
    uart_inst_t * uart;
    dwm1001_ctx_t uwb_device;
} DWM1001_Device;

void dwm1001_init(DWM1001_Device * handle, uart_inst_t * uartPort, uint tx_pin, uint rx_pin);
bool dwm1001_check_communication(DWM1001_Device * handle);
bool dwm1001_check_location_ready(DWM1001_Device * handle);
bool dwm1001_get_location(DWM1001_Device * handle, dwm_loc_data_t* loc);
void dwm1001_set_updateRate_deciSecond(DWM1001_Device * handle, uint16_t normal, uint16_t stationary);

#ifdef __cplusplus
}
#endif

#endif