// Copyright (c) 2015, XMOS Ltd, All rights reserved

#include <platform.h>
#include <xs1.h>
#include <stdio.h>
#include <string.h>
#include <timer.h>
#include "i2c.h"
#include "xud_cdc.h"

/* App specific defines */
#define MENU_MAX_CHARS  30
#define MENU_LIST       11
#define DEBOUNCE_TIME   (XS1_TIMER_HZ/50)
#define BUTTON_PRESSED  0x00

char welcomeMessage3[MENU_MAX_CHARS] = {"Welcome, to your death"};
char welcomeMessage2[2][MENU_MAX_CHARS] = {
        {"\n\r-------------------------\r\n"},
        {"Welcome, to your death"},
};

#define ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))

void showWelcome(client interface usb_cdc_interface cdc)
{
    unsigned length;
    for(int i = 0; i < 2; i++) {
        length = strlen(welcomeMessage3);
        cdc.write(welcomeMessage3, length);
    }
}

/* Application task */
void app_virtual_com_extended(client interface usb_cdc_interface cdc)
{
    unsigned int length, led_id;
//    int x = 0;
//    int y = 0;
//    int z = 0;
    char value, tmp_string[50];
    unsigned int button_1_valid, button_2_valid;
    timer tmr;
    unsigned int timer_val;

    showWelcome(cdc);

    button_1_valid = button_2_valid = 1;

    while(1)
    {

        /* Check if user has input any character */
        if(cdc.available_bytes())
        {
            value = cdc.get_char();

            printf("read %c\n",value);
            /* Do the chosen operation */
            if(value == '1') {
//                length = strlen(echo_mode_str[0]);
//                cdc.write(echo_mode_str[0], length);
//                length = strlen(echo_mode_str[1]);
//                cdc.write(echo_mode_str[1], length);
//
//                while(value != 0x1A) { /* 0x1A = Ctrl + Z */
//                    value = cdc.get_char();
//                    cdc.put_char(value);
//                }
//                length = strlen(echo_mode_str[2]);
//                cdc.write(echo_mode_str[2], length);
            }
            else if((value >= '2') && (value <= '5')) {
//                /* Find out which LED to toggle */
//                led_id = (value - 0x30) - 2;    // 0x30 used to convert the ascii to number
//                toggle_led(led_id);
            }
//            else if(value == '6') {
//                char status_data = 0;
//                i2c_regop_res_t result;
//
//                // Wait for valid accelerometer data
//                do {
//                  status_data = i2c.read_reg(FXOS8700EQ_I2C_ADDR, FXOS8700EQ_DR_STATUS, result);
//                } while (!status_data & 0x08);
//
//                // Read x and y axis values
//                x = read_acceleration(i2c, FXOS8700EQ_OUT_X_MSB);
//                y = read_acceleration(i2c, FXOS8700EQ_OUT_Y_MSB);
//                z = read_acceleration(i2c, FXOS8700EQ_OUT_Z_MSB);
//
//                length = sprintf(tmp_string, "Accelerometer: x[%d] y[%d] z[%d]\r\n", x, y, z);
//                cdc.write(tmp_string, length);
//            }
            else if(value == '7') {
                /* Read 32-bit timer value */
                tmr :> timer_val;
                length = sprintf(tmp_string, "Timer ticks: %u\r\n", timer_val);
                cdc.write(tmp_string, length);
            }
            else {
                showWelcome(cdc);
            }
        }
    } /* end of while(1) */
}
