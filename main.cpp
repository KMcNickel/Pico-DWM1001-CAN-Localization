#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/timer.h"
#include "include/mcp2515.h"
#include "include/filters.hpp"
#include "include/dwm1001_uart_pico.h"

#define VERSION_NUMBER "1.1 build 2"

#define CAN_BAUD_RATE 500000    //500 kbaud
#define CAN_NODE_ID 0x5
#define CAN_LOC_X_ADDRESS (CAN_NODE_ID << 5) | 0x1
#define CAN_LOC_Y_ADDRESS (CAN_NODE_ID << 5) | 0x2
#define CAN_LOC_Z_ADDRESS (CAN_NODE_ID << 5) | 0x3
#define CAN_LOC_T_ADDRESS (CAN_NODE_ID << 5) | 0x4

#define US_TO_MS_INT(us) (us / 1000)
#define CONVERT_POSITION(p) ((p / 100) * 100.0)    //Truncate to 10cm then bring back to mm (as a float)

//CAN Interface
#define CAN_SCK_PIN 2
#define CAN_TX_PIN 3
#define CAN_RX_PIN 4
#define CAN_CS_PIN 5
//UWB Interface
#define UWB_TX_PIN 8
#define UWB_RX_PIN 9
//LEDs
#define GREEN_LED_PIN 16
#define RED_LED_PIN 17

#define STDIO_UART_PERIPHERAL uart0
#define UWB_UART_PERIPHERAL uart1
#define CANBUS_SPI_PERIPHERAL spi0

DWM1001_Device dwm1001;
repeating_timer_t canTimer;
#define POSITION_FILTER_ALPHA 0.9
LowPassFilter xPosFilter(POSITION_FILTER_ALPHA);
LowPassFilter yPosFilter(POSITION_FILTER_ALPHA);
LowPassFilter zPosFilter(POSITION_FILTER_ALPHA);

dwm_pos_t position;
dwm_loc_data_t location;
float fPosition[4];
absolute_time_t lastLocationTime;

MCP2515 mcp2515(CANBUS_SPI_PERIPHERAL, CAN_CS_PIN, CAN_TX_PIN, CAN_RX_PIN, CAN_SCK_PIN, CAN_BAUD_RATE);

bool canTimerCallback(repeating_timer_t * timer)
{
    can_frame frame;
    int32_t deltaTime = US_TO_MS_INT(absolute_time_diff_us(lastLocationTime, get_absolute_time()));

    frame.can_id = CAN_LOC_X_ADDRESS;
    frame.can_dlc = 4;
    memcpy(frame.data, &fPosition[0], 4);
    mcp2515.sendMessage(&frame);
    frame.can_id = CAN_LOC_Y_ADDRESS;
    frame.can_dlc = 4;
    memcpy(frame.data, &fPosition[1], 4);
    mcp2515.sendMessage(&frame);
    frame.can_id = CAN_LOC_Z_ADDRESS;
    frame.can_dlc = 4;
    memcpy(frame.data, &fPosition[2], 4);
    mcp2515.sendMessage(&frame);
    frame.can_id = CAN_LOC_T_ADDRESS;
    frame.can_dlc = 4;
    memcpy(frame.data, &deltaTime, 4);
    mcp2515.sendMessage(&frame);

    return true;
}

void startupStdio()
{
    stdio_init_all();

    for(int i = 0; i < 10; i++)
    {
        printf(".");
        sleep_ms(500);
    }

    printf("\r\n\r\n\r\nVersion: %s\r\n", VERSION_NUMBER);
}

void startupUWB()
{
    printf("Setting up DWM1001...\n");

    dwm1001_init(&dwm1001, UWB_UART_PERIPHERAL, UWB_TX_PIN, UWB_RX_PIN);
    if(!dwm1001_check_communication(&dwm1001))
    {
        printf("Failed to communicate with the DWM1001\n");
        while(1) sleep_ms(1);
    }

    printf("DWM1001 setup complete\n");
}

void startupCANBus()
{
    printf("Setting up MCP2515...\n");

    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS, MCP_16MHZ);
    mcp2515.setNormalMode();

    printf("MCP2515 setup complete\n");
}

void startupAlarms()
{
    alarm_pool_init_default();
    add_repeating_timer_ms(100, canTimerCallback, NULL, &canTimer);
}

void setupGPIO()
{
    gpio_init(GREEN_LED_PIN);
    gpio_init(RED_LED_PIN);
    gpio_set_dir(GREEN_LED_PIN, GPIO_OUT);
    gpio_set_dir(RED_LED_PIN, GPIO_OUT);
}

void peripheralStartup()
{
    setupGPIO();
    gpio_put(RED_LED_PIN, true);

    startupStdio();

    printf("Setting up Pins and Peripherals...\n");

    startupCANBus();
    startupUWB();

    printf("Pins and Peripherals setup complete\n");

    gpio_put(RED_LED_PIN, false);
    gpio_put(GREEN_LED_PIN, true);
}

int main ()
{
    peripheralStartup();

    location.p_pos = &position;

    while(!dwm1001_get_location(&dwm1001, &location));
    xPosFilter.setInitialValue(CONVERT_POSITION(position.x));
    yPosFilter.setInitialValue(CONVERT_POSITION(position.y));
    zPosFilter.setInitialValue(CONVERT_POSITION(position.z));
    
    startupAlarms();

    while(true)
    {
        if(dwm1001_get_location(&dwm1001, &location))
        {
            lastLocationTime = get_absolute_time();

            fPosition[0] = xPosFilter.addNewData(CONVERT_POSITION(position.x));
            fPosition[1] = yPosFilter.addNewData(CONVERT_POSITION(position.y));
            fPosition[2] = zPosFilter.addNewData(CONVERT_POSITION(position.z));
            printf("Filtered: %.2f, %.2f, %.2f, Raw: %.2f, %.2f, %.2f, Quality: %d%%, Time: %d\r\n",
                    fPosition[0], fPosition[1], fPosition[2],
                    CONVERT_POSITION(position.x), CONVERT_POSITION(position.y), CONVERT_POSITION(position.z),
                    position.qf, to_ms_since_boot(lastLocationTime));

        }
    }
}