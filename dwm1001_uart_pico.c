#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "include/dwm_api.h"
#include "include/dwm1001_tlv.h"
#include "include/dwm1001_uart_pico.h"

#define DWM1001_UART_BAUDRATE 115200
#define DWM1001_UART_DATA_BITS 8
#define DWM1001_UART_STOP_BITS 1

#define READ_WAIT_MICROSECONDS 100000

#define DWM1001_SUPPORTED_FW_VERSION_MAJOR 1
#define DWM1001_SUPPORTED_FW_VERSION_MINOR 3
#define DWM1001_SUPPORTED_FW_VERSION_PATCH 0
#define DWM1001_SUPPORTED_FW_VERSION_VARIANT 1

int32_t platform_write(void * handle, const uint8_t * buf, uint8_t len)
{
    DWM1001_Device * device = (DWM1001_Device *) handle;

    for(int i = 0; i < len; i++) uart_putc(device->uart, *(buf + i));

    return 0;
}

int32_t platform_read(void * handle, uint8_t * buf, uint8_t * actualLen, uint8_t expectedLen)
{
    DWM1001_Device * device = (DWM1001_Device *) handle;

    for(int i = 0; i < expectedLen; i++)
    {
        if(!uart_is_readable_within_us(device->uart, READ_WAIT_MICROSECONDS))
            break;
        *(buf + i) = uart_getc(device->uart);
        *actualLen = i + 1;
    }

    return 0;
}

void dwm1001_init(DWM1001_Device * handle, uart_inst_t * uartPort, uint tx_pin, uint rx_pin)
{
    handle->uart = uartPort;

    gpio_set_function(tx_pin, GPIO_FUNC_UART);
    gpio_set_function(rx_pin, GPIO_FUNC_UART);

    uart_init(uartPort, DWM1001_UART_BAUDRATE);
    uart_set_format(uartPort, DWM1001_UART_DATA_BITS, DWM1001_UART_STOP_BITS, UART_PARITY_NONE);

    handle->uwb_device.readData = platform_read;
    handle->uwb_device.writeData = platform_write;
    handle->uwb_device.handle = handle;
}

bool dwm1001_check_communication(DWM1001_Device * handle)
{
    dwm_ver_t version;
    if(dwm_ver_get(&(handle->uwb_device), &version) != DWM_OK)
        return 0;
    
    if(version.fw.var != DWM1001_SUPPORTED_FW_VERSION_VARIANT)
        return 0;
    if(version.fw.patch != DWM1001_SUPPORTED_FW_VERSION_PATCH)
        return 0;
    if(version.fw.min != DWM1001_SUPPORTED_FW_VERSION_MINOR)
        return 0;
    if(version.fw.maj != DWM1001_SUPPORTED_FW_VERSION_MAJOR)
        return 0;

    return 1;
}

bool dwm1001_check_location_ready(DWM1001_Device * handle)
{
    dwm_status_t status;

    dwm_status_get(&(handle->uwb_device), &status);

    return status.loc_data;
}

bool dwm1001_get_location(DWM1001_Device * handle, dwm_loc_data_t* loc)
{
    if(!dwm1001_check_location_ready(handle))
        return 0;

    dwm_loc_get(&(handle->uwb_device), loc);

    return 1;
}

void dwm1001_set_updateRate_deciSecond(DWM1001_Device * handle, uint16_t normal, uint16_t stationary)
{
    uint16_t curNorm, curStnry;

    dwm_upd_rate_get(&(handle->uwb_device), &curNorm, &curStnry);

    if(curNorm != normal || curStnry != stationary)
        dwm_upd_rate_set(&(handle->uwb_device), normal, stationary);
}
