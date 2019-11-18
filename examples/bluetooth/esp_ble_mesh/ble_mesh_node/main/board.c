/* board.c - Board-specific hooks */

/*
 * Copyright (c) 2017 Intel Corporation
 * Additional Copyright (c) 2018 Espressif Systems (Shanghai) PTE LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "board.h"
#include "esp_ble_mesh_provisioning_api.h"

#define TAG "BOARD"

#define INTR_FLAG_DEFAULT 0

static xQueueHandle s_evt_queue;

struct _led_state led_state[3] = {
    { LED_OFF, LED_OFF, LED_R, "red"   },
    { LED_OFF, LED_OFF, LED_G, "green" },
    { LED_OFF, LED_OFF, LED_B, "blue"  },
};

uint8_t button_state = 0;
TickType_t button_debounce_ms = 20;
TickType_t lastTickTime = 0;


void (*onoff_pub_func_global)(uint8_t);

void board_output_number(esp_ble_mesh_output_action_t action, uint32_t number)
{
    ESP_LOGI(TAG, "Board output number %d", number);
}

void board_prov_complete(void)
{
    board_led_operation(LED_B, LED_OFF);
}

void board_led_operation(uint8_t pin, uint8_t onoff)
{
    for (int i = 0; i < 3; i++) {
        if (led_state[i].pin != pin) {
            continue;
        }
        if (onoff == led_state[i].previous) {
            ESP_LOGW(TAG, "led %s is already %s",
                     led_state[i].name, (onoff ? "on" : "off"));
            return;
        }
        gpio_set_level(pin, onoff);
        led_state[i].previous = onoff;
        return;
    }

    ESP_LOGE(TAG, "LED is not found!");
}

static void board_led_init(void)
{
    for (int i = 0; i < 3; i++) {
        gpio_pad_select_gpio(led_state[i].pin);
        gpio_set_direction(led_state[i].pin, GPIO_MODE_OUTPUT);
        gpio_set_level(led_state[i].pin, LED_OFF);
        led_state[i].previous = LED_OFF;
    }
}

static void IRAM_ATTR switch_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    TickType_t actualTime = xTaskGetTickCountFromISR();
    if ((actualTime -  button_debounce_ms) > lastTickTime)
    {
    	xQueueSendFromISR(s_evt_queue, &gpio_num, NULL);
    	lastTickTime = actualTime;
    }
}

static void switch_key_init(uint32_t key)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = 1 << key;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);

    gpio_set_intr_type(key, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(key, switch_isr_handler, (void *)key);
}

static void switch_task_entry(void *arg)
{
    uint32_t io_num;
    while (1) {
        if (xQueueReceive(s_evt_queue, &io_num, portMAX_DELAY) == pdTRUE) {
			button_state = !button_state;
            ESP_LOGI(TAG, "GPIO[%d] intr, val: %d", io_num, button_state);
            onoff_pub_func_global(button_state);
        }
    }
}

static void switch_init(gpio_num_t gpio_num)
{
    s_evt_queue = xQueueCreate(3, sizeof(uint32_t));
    if (!s_evt_queue) {
        return;
    }

    BaseType_t ret = xTaskCreate(switch_task_entry, "switch", 4096, NULL, 4, NULL);
    if (ret == pdFAIL) {
        goto fail;
    }

    switch_key_init(gpio_num);
    return;

fail:
    vQueueDelete(s_evt_queue);
}

void board_init(void (*onoff_pub_func)(uint8_t))
{
    board_led_init();
    switch_init(GPIO_NUM_18);
    onoff_pub_func_global = onoff_pub_func;
}
