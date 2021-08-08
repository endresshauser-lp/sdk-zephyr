/**
 * Copyright (c) 2021 grandcentrix GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __ZEPHYR_DRIVERS_WIFI_SILABS_WFX200_INTERNAL_H_
#define __ZEPHYR_DRIVERS_WIFI_SILABS_WFX200_INTERNAL_H_

#include <stdint.h>
#include <stdbool.h>

#include <device.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>
#include <net/ethernet.h>
#include <net/wifi_mgmt.h>

#include <sl_wfx_constants.h>

enum wfx200_event {
	WFX200_CONNECT_EVENT = 0,
	WFX200_CONNECT_FAILED_EVENT,
	WFX200_DISCONNECT_EVENT,
	WFX200_AP_START_EVENT,
	WFX200_AP_START_FAILED_EVENT,
	WFX200_AP_STOP_EVENT,
};

struct wfx200_queue_event {
	enum wfx200_event ev;
	union {

	};
};

struct wfx200_gpio {
	const struct device *dev;
	uint8_t pin;
};

enum wfx200_state {
	WFX200_STATE_IDLE = 0,
	WFX200_STATE_INITIALIZED,
	WFX200_STATE_INTERFACE_INITIALIZED,
	WFX200_STATE_AP_MODE,
	WFX200_STATE_STA_MODE,
};

struct wfx200_dev {
	struct net_if *iface;
	const struct device *dev;

	const struct device *spi;
	struct spi_config spi_cfg;
	struct spi_cs_control cs_ctrl;

	size_t firmware_pos;

	struct wfx200_gpio interrupt;
	struct wfx200_gpio reset;
	struct wfx200_gpio wakeup;
	struct wfx200_gpio hif_sel;

	struct k_sem wakeup_sem;
	struct k_sem event_sem;

	struct k_mutex bus_mutex;
	struct k_mutex event_mutex;

	uint8_t waited_event_id;

	struct gpio_callback int_cb;

	scan_result_cb_t scan_cb;

	sl_wfx_context_t sl_context;

	enum wfx200_state state;

	struct k_work_q incoming_work_q;
	struct k_work incoming_work;

	struct k_queue event_queue;
	struct k_thread event_thread;

	K_KERNEL_STACK_MEMBER(wfx200_stack_area, CONFIG_WIFI_WFX200_STACK_SIZE);
	K_KERNEL_STACK_MEMBER(wfx200_event_stack_area, CONFIG_WIFI_WFX200_STACK_SIZE);

	struct k_heap heap;
	char heap_buffer[CONFIG_WIFI_WFX200_HEAP_SIZE];
};

struct wfx200_gpio_config {
	const char *port;
	uint8_t pin;
	gpio_dt_flags_t flags;
};

#define WFX200_INIT_GPIO_CONFIG(_inst, _name) {		   \
		.port = DT_INST_GPIO_LABEL(_inst, _name),  \
		.pin = DT_INST_GPIO_PIN(_inst, _name),	   \
		.flags = DT_INST_GPIO_FLAGS(_inst, _name), \
}

struct wfx200_config {
	struct wfx200_gpio_config interrupt;
	struct wfx200_gpio_config reset;
	struct wfx200_gpio_config wakeup;
	struct wfx200_gpio_config hif_select;
	struct wfx200_gpio_config spi_cs;
	const char *spi_port;
	uint32_t spi_freq;
	uint8_t spi_slave;
};

void wfx200_enable_interrupt(struct wfx200_dev *context);
void wfx200_disable_interrupt(struct wfx200_dev *context);

void wfx200_event_thread(void *p1, void *p2, void *p3);

#endif /* __ZEPHYR_SILABS_WFX200_INTERNAL_H_ */
