#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/pll.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"
#include "include/mcp2515.h"
#include "include/filters.hpp"
#include "include/dwm1001_uart_pico.h"

#define VER_MAJOR       1
#define VER_MINOR       2
#define VER_REVISION    0
#define VER_BUILD       1

#define CAN_BAUD_RATE 500000    //500 kbaud
#define CAN_NODE_ID 0x5
//NOTE: There is a max of 32 messages per Node ID
#define CAN_VER_ADDRESS         (CAN_NODE_ID << 5) | 0x0
#define CAN_LOC_X_ADDRESS       (CAN_NODE_ID << 5) | 0x10
#define CAN_LOC_Y_ADDRESS       (CAN_NODE_ID << 5) | 0x11
#define CAN_LOC_Z_ADDRESS       (CAN_NODE_ID << 5) | 0x12
#define CAN_QUALITY_ADDRESS     (CAN_NODE_ID << 5) | 0x13
#define CAN_DELTA_TIME_ADDRESS  (CAN_NODE_ID << 5) | 0x14

#define US_TO_MS_INT(us) (us / 1000)

//CAN Interface
#define CAN_SCK_PIN 2
#define CAN_TX_PIN 3
#define CAN_RX_PIN 4
#define CAN_CS_PIN 5
//UWB Interface
#define UWB_TX_PIN  8
#define UWB_RX_PIN  9
#define UWB_INT_PIN 10
#define UWB_RST_PIN 11
//LEDs
#define GREEN_LED_PIN 16
#define RED_LED_PIN 17
#define LED_PWM_SLICE 0 //(PinNum >> 1) & 7 ... Can also use: uint pwm_gpio_to_slice_num(pinNum)
#define LED_PWM_COUNTER_MIN 0
#define LED_PWM_RED_ON_LVL  80
#define LED_PWM_GRN_ON_LVL  40
#define LED_PWM_COUNTER_MAX 100
#define GREEN_LED_PWM_CHAN PWM_CHAN_A
#define RED_LED_PWM_CHAN   PWM_CHAN_B

#define STDIO_UART_PERIPHERAL uart0
#define UWB_UART_PERIPHERAL uart1
#define CANBUS_SPI_PERIPHERAL spi0

#define UWB_UPDATE_RATE_DECISECOND 1

DWM1001_Device dwm1001;

dwm_pos_t position;
dwm_loc_data_t location;
float outputPosition[3];
int32_t lastPosition[3];
absolute_time_t lastLocationTime;
absolute_time_t currentLocationTime;
int32_t timeDifference;
can_frame frame;
uint8_t frameID = 0;

MCP2515 mcp2515(CANBUS_SPI_PERIPHERAL, CAN_CS_PIN, CAN_TX_PIN, CAN_RX_PIN, CAN_SCK_PIN, CAN_BAUD_RATE);

void startupStdio()
{
    stdio_init_all();

    for(int i = 0; i < 10; i++)
    {
        printf(".");
        sleep_ms(500);
        
    }

    printf("\r\n\r\n\r\nVersion: %d.%d.%d build %d\r\n", VER_MAJOR, VER_MINOR, VER_REVISION, VER_BUILD);
}

void startupUWB()
{
    printf("Setting up DWM1001...\n");

    gpio_set_dir(UWB_RST_PIN, GPIO_OUT);
    gpio_put(UWB_INT_PIN, 1);
    sleep_ms(10);
    gpio_put(UWB_INT_PIN, 0);
    sleep_ms(50);
    gpio_put(UWB_INT_PIN, 1);
    sleep_ms(100);

    dwm1001_init(&dwm1001, UWB_UART_PERIPHERAL, UWB_TX_PIN, UWB_RX_PIN);
    if(!dwm1001_check_communication(&dwm1001))
    {
        printf("Failed to communicate with the DWM1001\n");
        while(1) sleep_ms(1);
    }

    dwm1001_set_updateRate_deciSecond(&dwm1001, UWB_UPDATE_RATE_DECISECOND, UWB_UPDATE_RATE_DECISECOND);

    printf("DWM1001 setup complete\n");
}

void startupCANBus()
{
    printf("Setting up MCP2515...\n");

    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS, MCP_16MHZ);
    mcp2515.setNormalMode();

    can_frame frame;

    frame.can_id = CAN_VER_ADDRESS;
    frame.can_dlc = 4;
    frame.data[3] = VER_MAJOR;
    frame.data[2] = VER_MINOR;
    frame.data[1] = VER_REVISION;
    frame.data[0] = VER_BUILD;
    MCP2515::ERROR err = mcp2515.sendMessage(&frame);
    if(err != MCP2515::ERROR_OK) printf("MCP Error: %d\r\n", err);

    printf("MCP2515 setup complete\n");
}

void setRedLED(bool on)
{
    pwm_set_chan_level(LED_PWM_SLICE, RED_LED_PWM_CHAN, on ? LED_PWM_RED_ON_LVL : LED_PWM_COUNTER_MIN);
}

void setGreenLED(bool on)
{
    pwm_set_chan_level(LED_PWM_SLICE, GREEN_LED_PWM_CHAN, on ? LED_PWM_GRN_ON_LVL : LED_PWM_COUNTER_MIN);
}

void setupGPIO()
{
    gpio_set_function(GREEN_LED_PIN, GPIO_FUNC_PWM);
    gpio_set_function(RED_LED_PIN, GPIO_FUNC_PWM);
    pwm_set_wrap(LED_PWM_SLICE, LED_PWM_COUNTER_MAX);
    setRedLED(false);
    setGreenLED(false);
    pwm_set_enabled(LED_PWM_SLICE, true);

}

void peripheralStartup()
{
    setupGPIO();
    setRedLED(true);

    startupStdio();

    printf("Setting up Pins and Peripherals...\n");

    startupCANBus();
    startupUWB();

    printf("Pins and Peripherals setup complete\n");

    setRedLED(false);
    setGreenLED(true);
}

void convertPosition(float * outputPosition, int32_t inputPosition)
{
    //If we are initialized, just move the position over
    if(*outputPosition == 0 || (inputPosition % 100 > 20 && inputPosition % 100 < 80))
        *outputPosition = (inputPosition / 100) / 10.0;

}

int main ()
{
    peripheralStartup();

    location.p_pos = &position;

    while(true)
    {
        if(dwm1001_get_location(&dwm1001, &location))
        {
            currentLocationTime = get_absolute_time();
            convertPosition(&outputPosition[0], position.x);
            convertPosition(&outputPosition[1], position.y);
            convertPosition(&outputPosition[2], position.z);
            timeDifference = US_TO_MS_INT(absolute_time_diff_us(lastLocationTime, currentLocationTime));

            printf("ID: %3d, Quality: %3d%%, Delta Time: %10d, Position: Raw: %+10d, %+10d, %+10d, \"Filtered\": %+8.2f, %+8.2f, %+8.2f\r\n",
                    frameID, position.qf, timeDifference,
                    position.x, position.y, position.z,
                    outputPosition[0], outputPosition[1], outputPosition[2]);

            //Set up the CAN frame
            frame.can_dlc = 5;  //Most things are 32 bit (float or integer)
            //Add the frame ID so the other side can match all of the messages together
            frame.data[0] = frameID;

            frame.can_id = CAN_LOC_X_ADDRESS;
            memcpy(frame.data + 1, &outputPosition[0], sizeof(float));
            mcp2515.sendMessage(&frame);

            frame.can_id = CAN_LOC_Y_ADDRESS;
            memcpy(frame.data + 1, &outputPosition[1], sizeof(float));
            mcp2515.sendMessage(&frame);

            frame.can_id = CAN_LOC_Z_ADDRESS;
            memcpy(frame.data + 1, &outputPosition[2], sizeof(float));
            mcp2515.sendMessage(&frame);

            frame.can_id = CAN_DELTA_TIME_ADDRESS;
            memcpy(frame.data + 1, &timeDifference, sizeof(int32_t));
            mcp2515.sendMessage(&frame);

            frame.can_id = CAN_QUALITY_ADDRESS;
            frame.can_dlc = 2;      //Uses an 8-bit integer, so 1 byte
            memcpy(frame.data + 1, &position.qf, sizeof(uint8_t));
            mcp2515.sendMessage(&frame);

            lastLocationTime = currentLocationTime;
            frameID++;
        }
    }
}