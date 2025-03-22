/**
 * -------------------------------------------- COPYRIGHT NOTICE ---------------------------------------------------
 * Copyright (C) 2023 Octobotics Tech Pvt. Ltd. All Rights Reserved.
 * Do not remove this copyright notice.
 * Do not use, reuse, copy, merge, publish, sub-license, sell, distribute or modify this code - except without explicit,
 * written permission from Octobotics Tech Pvt. Ltd.
 * Contact connect@octobotics.tech for full license information.
 * Author : Vaibhav
 * -------------------------------------------- COPYRIGHT NOTICE ---------------------------------------------------
 *
 *
 * @file main.cpp
 * @author Vaibhav
 * @brief
 * @date 2023-02-17
 *
 *
 */

#include "rmd/x10_api_base.h"
#include "rmd/x10_api.h"

#include <iostream>
#include <unistd.h>
#include <cstdio>
#include <stdio.h>

void api_base();
void api();
void print_pid(uint8_t *read_pid_arr);
void continuous_command_test();

int main()
{

    std::cout << "####################################\n";
    std::cout << "I am here in main.cpp!\n";
    std::cout << "####################################\n";

    // api_base();
    api();
    // continuous_command_test();

    return 0;
}

void api_base()
{
    int8_t x_error = 0;

    X10ApiBase *xobj;
    xobj = new X10ApiBase;
    xobj->rmdX10_init();
    uint8_t read_pid_arr[6] = {0};
    int16_t motor_stat1[4] = {0};
    int16_t motor_stat2[4] = {0};
    int16_t motor_stat3[4] = {0};

    printf("serial_binary_send: \n");
    x_error = xobj->Motor_read_pid(0x01, read_pid_arr);
    printf("Motor_read_pid->x_error: %d \n", x_error);
    print_pid(read_pid_arr);

    // read acceleration
    int32_t accel = xobj->Motor_read_accel(0x1);
    printf("accel: %d \n", accel);
    printf("---------------------------\n");

    // read multi_turn_encoder_pos
    int32_t multi_turn_encoder_pos = xobj->multi_turn_encoder_pos(0x1);
    printf("multi_turn_encoder_pos: %d \n", multi_turn_encoder_pos);
    printf("---------------------------\n");

    // for (int i = 0; i < 500; i++)
    {
        x_error = xobj->speedControl(0x1, 3000);
        printf("speedControl->x_error: %d \n", x_error);

        x_error = xobj->Motor_state1(0x1, motor_stat1);
        printf("Motor_state1->x_error: %d \n", x_error);

        x_error = xobj->Motor_state2(0x1, motor_stat2);
        printf("Motor_state2->x_error: %d \n", x_error);

        x_error = xobj->Motor_state3(0x1, motor_stat3);
        printf("Motor_state1->x_error: %d \n", x_error);

        printf("motor_stat1: %4d\t %4d\t %0.2f\t %4d \n", motor_stat1[0], motor_stat1[1], (float)motor_stat1[2] / 10.00, motor_stat1[3]);  // motor_stat1
        printf("motor_stat2: %4d\t %0.4f\t %4d\t %4d \n", motor_stat2[0], (float)motor_stat2[1] / 100.00, motor_stat2[2], motor_stat2[3]); // motor_stat2
        printf("motor_stat3: %4d\t %4d\t %4d\t %4d \n", motor_stat3[0], motor_stat3[1], motor_stat3[2], motor_stat3[3]);                   // motor_stat3

        printf("---------------------------\n");
        usleep(20000);
    }
    x_error = xobj->speedControl(0x1, 00);
    printf("speedControl->x_error: %d \n", x_error);

    x_error = xobj->Motor_shut_down(0x1);
    printf("speedControl->x_error: %d \n", x_error);

    // sleep(1);
    xobj->rmdX10_shut_down();
}

void api()
{
    uint8_t read_pid_arr[6] = {0};
    X10ApiSerial *xobj;
    xobj = new X10ApiSerial();

    xobj->rmdX10_init();
    while (1)
    {
        // read pid
        xobj->Motor_read_pid(0x2, read_pid_arr);
        print_pid(read_pid_arr);
        sleep(1);
    }
#if 0

    // read acceleration
    int32_t accel = xobj->Motor_read_accel(0x1);
    printf("accel: %d \n", accel);
    printf("---------------------------\n");

    // read multi_turn_encoder_pos
    int32_t multi_turn_encoder_pos = xobj->multi_turn_encoder_pos(0x1);
    printf("multi_turn_encoder_pos: %d \n", multi_turn_encoder_pos);
    printf("---------------------------\n");

    // read motor_state
    int16_t motor_stat1[4] = {0};
    int16_t motor_stat2[4] = {0};
    int16_t motor_stat3[4] = {0};

    // control speed
    for (int i = 0; i < 50; i++)
    {
        xobj->speedControl(0x1, 3000);
        // sleep(2);
        xobj->Motor_state1(0x1, motor_stat1);
        xobj->Motor_state2(0x1, motor_stat2);
        xobj->Motor_state3(0x1, motor_stat3);
        printf("Motor state1: %4d\t %4d\t %0.2f\t %4d \n", motor_stat1[0], motor_stat1[1], (float)motor_stat1[2] / 10.0, motor_stat1[3]);   // motor_stat1
        printf("Motor state2: %4d\t %0.4f\t %4d\t %4d \n", motor_stat2[0], (float)motor_stat2[1] / 100.00, motor_stat2[2], motor_stat2[3]); // motor_stat2
        printf("Motor state3: %4d\t %4d\t %4d\t %4d \n", motor_stat3[0], motor_stat3[1], motor_stat3[2], motor_stat3[3]);                   // motor_stat3
        printf("-------------------------------------------------------\n");
        usleep(20000);
    }
    sleep(1);
    // xobj->Motor_stop(0x1);
    // xobj->speedControl(0x1, 3000);
    // sleep(3);
    // xobj->Motor_stop(0x1);
    while (1)
    {
        xobj->Motor_shut_down(0x1);
        xobj->Motor_state1(0x1, motor_stat1);
        xobj->Motor_state2(0x1, motor_stat2);
        xobj->Motor_state3(0x1, motor_stat3);
        printf("Motor state1: %4d\t %4d\t %0.2f\t %4d \n", motor_stat1[0], motor_stat1[1], (float)motor_stat1[2] / 10.0, motor_stat1[3]);   // motor_stat1
        printf("Motor state2: %4d\t %0.4f\t %4d\t %4d \n", motor_stat2[0], (float)motor_stat2[1] / 100.00, motor_stat2[2], motor_stat2[3]); // motor_stat2
        printf("Motor state3: %4d\t %4d\t %4d\t %4d \n", motor_stat3[0], motor_stat3[1], motor_stat3[2], motor_stat3[3]);                   // motor_stat3
        printf("-------------------------------------------------------\n");
        usleep(20000);
    }
#endif

    xobj->rmdX10_shut_down();
}

void print_pid(uint8_t *read_pid_arr)
{
    printf("currKP: %d \n", read_pid_arr[0]);
    printf("currKI: %d \n", read_pid_arr[1]);
    printf("spdKP : %d \n", read_pid_arr[2]);
    printf("spdKI : %d \n", read_pid_arr[3]);
    printf("posKP : %d \n", read_pid_arr[4]);
    printf("posKI : %d \n", read_pid_arr[5]);
    printf("---------------------------\n");
}

void continuous_command_test()
{
    X10ApiSerial *xobj;
    xobj = new X10ApiSerial();
    xobj->rmdX10_init();
    int8_t ret = 0;
    uint8_t mode;

    int16_t motor_stat1[4] = {0};
    int16_t motor_stat2[4] = {0};
    int16_t motor_stat3[4] = {0};
    ret = xobj->Motor_mode(0x1, mode);
    printf("Start---> ret: %d mode: %d \n", ret, mode);

    // for (int i = 0; i < 50; i++)
    {
        xobj->speedControl(0x1, 3000);
        // for (int j = 1; j < 5; j++)
        {
            ret = xobj->Motor_state1(1, motor_stat1);
            printf("ID: %d ret:%d  %4d\t %4d\t %0.2f\t %4d \n", 1, ret, motor_stat1[0], motor_stat1[1], (float)motor_stat1[2] / 10.0, motor_stat1[3]); // motor_stat1

            printf("-------------------------------------------------------\n");
            usleep(200);
            sleep(1);
        }
        usleep(20000);
    }
    ret = xobj->Motor_mode(0x1, mode);
    printf("In motion--> ret: %d mode: %d \n", ret, mode);

    sleep(1);
    xobj->Motor_stop(0x1);
    ret = xobj->Motor_mode(0x1, mode);
    printf("Stop_motor: ret: %d mode: %d \n", ret, mode);

    xobj->Motor_shut_down(0x1);
    ret = xobj->Motor_mode(0x1, mode);
    printf("Motor_shut_down: ret: %d mode: %d \n", ret, mode);

    xobj->Motor_reset(0x1);
    ret = xobj->Motor_mode(0x1, mode);
    printf("Motor_reset: ret: %d mode: %d \n", ret, mode);

    xobj->rmdX10_shut_down();
}