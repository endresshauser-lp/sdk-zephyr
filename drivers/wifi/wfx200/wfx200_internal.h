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
#include <sl_wfx_constants.h>

struct wfx200_gpio {
	const struct device *dev;
	unsigned int pin;
};

struct wfx200_dev {
	const struct device *spi;
	struct spi_config spi_cfg;
	struct spi_cs_control cs_ctrl;

	size_t firmware_pos;

	struct wfx200_gpio interrupt;
	struct wfx200_gpio reset;
	struct wfx200_gpio wakeup;

	struct k_sem wakeup_sem;
	struct k_sem event_sem;

	struct k_mutex bus_mutex;
	struct k_mutex event_mutex;

	uint8_t waited_event_id;

	struct gpio_callback int_cb;

	struct k_work_q work_q;
	struct k_work work;

	struct net_if *iface;
	// uint8_t mac_address[6];

	sl_wfx_context_t sl_context;

	bool iface_initialized : 1;
};

extern struct wfx200_dev wfx200;

#endif /* __ZEPHYR_SILABS_WFX200_INTERNAL_H_ */
