/* SiLabs WFX200 WiFi Module with SPI
 * or SDIO Interface
 *
 * Copyright (c) 2021 grandcentrix GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT silabs_wfx200

#define LOG_MODULE_NAME wifi_wfx200
#define LOG_LEVEL CONFIG_WIFI_LOG_LEVEL

#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <zephyr.h>
#include <device.h>
#include <string.h>
#include <errno.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>
#include <net/net_pkt.h>
#include <net/net_if.h>
#include <net/ethernet.h>

#include "wfx200_internal.h"

#include <sl_wfx.h>
#include <firmware/sl_wfx_registers.h>
#include <firmware/sl_wfx_cmd_api.h>

K_THREAD_STACK_DEFINE(wfx200_stack_area, CONFIG_WIFI_WFX200_STACK_SIZE);

static void wfx200_iface_init(struct net_if *iface)
{
	LOG_DBG("Interface init");
	wfx200.iface = iface;
	net_if_set_link_addr(iface, wfx200.sl_context.mac_addr_0.octet,
			     sizeof(wfx200.sl_context.mac_addr_0.octet),
			     NET_LINK_ETHERNET);
	ethernet_init(iface);
	wfx200.iface_initialized = true;
	net_if_flag_set(iface, NET_IF_NO_AUTO_START);
	if (IS_ENABLED(CONFIG_WIFI_WFX200_AUTOCONNECT)) {
		sl_wfx_send_scan_command(WFM_SCAN_MODE_PASSIVE,
					 NULL,
					 0,
					 NULL,
					 0,
					 NULL,
					 0,
					 NULL);
	}
}

int wfx200_send(const struct device *dev, struct net_pkt *pkt)
{
	size_t len = net_pkt_get_len(pkt), frame_len;
	sl_wfx_send_frame_req_t *tx_buffer;
	sl_status_t result;

	ARG_UNUSED(dev);

	frame_len = SL_WFX_ROUND_UP(len, 2);
	result = sl_wfx_allocate_command_buffer((sl_wfx_generic_message_t **)&tx_buffer,
						SL_WFX_SEND_FRAME_REQ_ID,
						SL_WFX_TX_FRAME_BUFFER,
						frame_len + sizeof(sl_wfx_send_frame_req_t));
	if (result != SL_STATUS_OK) {
		return -ENOMEM;
	}
	if (net_pkt_read(pkt, tx_buffer->body.packet_data, len)) {
		return -EIO;
	}
	/* TODO: Handle AP */
	result = sl_wfx_send_ethernet_frame(tx_buffer, frame_len, SL_WFX_STA_INTERFACE, WFM_PRIORITY_BE0);
	sl_wfx_free_command_buffer((sl_wfx_generic_message_t *)tx_buffer,
				   SL_WFX_SEND_FRAME_REQ_ID,
				   SL_WFX_TX_FRAME_BUFFER);
	if (result == SL_STATUS_OK) {
		return 0;
	}
	return -EIO;
}

static const struct ethernet_api api_funcs = {
	.iface_api.init = wfx200_iface_init,
	.send = wfx200_send,
};

void wfx200_interrupt_handler(const struct device *dev, struct gpio_callback *cb,
			      uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	if (IS_ENABLED(CONFIG_WIFI_WFX200_BUS_SPI)) {
		k_sem_give(&wfx200.wakeup_sem);
		k_work_submit_to_queue(&wfx200.work_q, &wfx200.work);
	}
}

void wfx200_handle_incoming(struct k_work *item)
{
	uint16_t control_register = 0;

	ARG_UNUSED(item);

	do {
		/* TODO: Disable interrupt? */
		if (sl_wfx_receive_frame(&control_register) != SL_STATUS_OK) {
			break;
		}
	} while ((control_register & SL_WFX_CONT_NEXT_LEN_MASK) != 0);
}

static int wfx200_init(const struct device *dev)
{
	int res;

	LOG_INF("Initializing WFX200 Driver, using FMAC Driver Version %s", FMAC_DRIVER_VERSION_STRING);
	k_sem_init(&wfx200.wakeup_sem, 0, 1);
	k_sem_init(&wfx200.event_sem, 0, K_SEM_MAX_LIMIT);
	k_mutex_init(&wfx200.bus_mutex);
	k_mutex_init(&wfx200.event_mutex);
	wfx200.reset.dev = device_get_binding(DT_INST_GPIO_LABEL(0, reset_gpios));
	if (!wfx200.reset.dev) {
		LOG_ERR("Failed to initialize GPIO driver: %s",
			DT_INST_GPIO_LABEL(0, reset_gpios));
		return -ENODEV;
	}
	wfx200.reset.pin = DT_INST_GPIO_PIN(0, reset_gpios);
	if (gpio_pin_configure(wfx200.reset.dev, wfx200.reset.pin,
			       DT_INST_GPIO_FLAGS(0, reset_gpios) | GPIO_OUTPUT_INACTIVE) < 0) {
		LOG_ERR("Failed to configure %s pin %d", DT_INST_GPIO_LABEL(0, reset_gpios),
			wfx200.reset.pin);
		return -ENODEV;
	}
	wfx200.wakeup.dev = device_get_binding(DT_INST_GPIO_LABEL(0, wake_gpios));
	if (!wfx200.wakeup.dev) {
		LOG_ERR("Failed to initialize GPIO driver: %s",
			DT_INST_GPIO_LABEL(0, wake_gpios));
		return -ENODEV;
	}
	wfx200.wakeup.pin = DT_INST_GPIO_PIN(0, wake_gpios);
	if (gpio_pin_configure(wfx200.wakeup.dev, wfx200.wakeup.pin,
			       DT_INST_GPIO_FLAGS(0, wakeup_gpios) | GPIO_OUTPUT_INACTIVE) < 0) {
		LOG_ERR("Failed to configure %s pin %d", DT_INST_GPIO_LABEL(0, wake_gpios),
			wfx200.wakeup.pin);
		return -ENODEV;
	}
	wfx200.interrupt.dev = device_get_binding(DT_INST_GPIO_LABEL(0, int_gpios));
	if (!wfx200.interrupt.dev) {
		LOG_ERR("Failed to initialize GPIO driver: %s",
			DT_INST_GPIO_LABEL(0, int_gpios));
		return -ENODEV;
	}
	wfx200.interrupt.pin = DT_INST_GPIO_PIN(0, int_gpios);
	if (gpio_pin_configure(wfx200.interrupt.dev, wfx200.interrupt.pin,
			       DT_INST_GPIO_FLAGS(0, int_gpios) | GPIO_INPUT) < 0) {
		LOG_ERR("Failed to configure %s pin %d", DT_INST_GPIO_LABEL(0, int_gpios),
			wfx200.interrupt.pin);
		return -ENODEV;
	}
	if (gpio_pin_interrupt_configure(wfx200.interrupt.dev, wfx200.interrupt.pin,
					 GPIO_INT_EDGE_TO_ACTIVE) < 0) {
		LOG_ERR("Failed to configure interrupt on %s pin %d", DT_INST_GPIO_LABEL(0, int_gpios),
			wfx200.interrupt.pin);
		return -ENODEV;
	}
	gpio_init_callback(&wfx200.int_cb, wfx200_interrupt_handler, BIT(wfx200.interrupt.pin));
	gpio_add_callback(wfx200.interrupt.dev, &wfx200.int_cb);
	if (IS_ENABLED(CONFIG_WIFI_WFX200_BUS_SPI)) {
		/* setup SPI device */
		wfx200.spi = device_get_binding(DT_INST_BUS_LABEL(0));
		if (!wfx200.spi) {
			LOG_ERR("Unable to get SPI device binding");
			return -ENODEV;
		}
		wfx200.spi_cfg.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER;
		wfx200.spi_cfg.frequency = DT_INST_PROP(0, spi_max_frequency);
		wfx200.spi_cfg.slave = DT_INST_REG_ADDR(0);
		if (DT_INST_SPI_DEV_HAS_CS_GPIOS(0)) {
			wfx200.cs_ctrl.gpio_dev = device_get_binding(
				DT_INST_SPI_DEV_CS_GPIOS_LABEL(0));
			if (!wfx200.cs_ctrl.gpio_dev) {
				LOG_ERR("Unable to get GPIO SPI CS device");
				return -ENODEV;
			}
			wfx200.cs_ctrl.gpio_pin = DT_INST_SPI_DEV_CS_GPIOS_PIN(0);
			wfx200.cs_ctrl.gpio_dt_flags = DT_INST_SPI_DEV_CS_GPIOS_FLAGS(0);
			wfx200.cs_ctrl.delay = 0U;
			wfx200.spi_cfg.cs = &wfx200.cs_ctrl;
			LOG_DBG("SPI GPIO CS configured on %s:%u",
				DT_INST_SPI_DEV_CS_GPIOS_LABEL(0),
				DT_INST_SPI_DEV_CS_GPIOS_PIN(0));
		}
	}
	k_work_queue_start(&wfx200.work_q, wfx200_stack_area, K_THREAD_STACK_SIZEOF(wfx200_stack_area),
			   CONFIG_WIFI_WFX200_PRIORITY, &(const struct k_work_queue_config){
		.name = "wfx200",
	});
	k_work_init(&wfx200.work, wfx200_handle_incoming);
	res = sl_wfx_init(&wfx200.sl_context);
	if (res != SL_STATUS_OK) {
		switch (res) {
		case SL_STATUS_WIFI_INVALID_KEY:
			LOG_ERR("Failed to init WF200: Firmware keyset invalid");
			break;
		case SL_STATUS_WIFI_FIRMWARE_DOWNLOAD_TIMEOUT:
			LOG_ERR("Failed to init WF200: Firmware download timeout");
			break;
		case SL_STATUS_TIMEOUT:
			LOG_ERR("Failed to init WF200: Poll for value timeout");
			break;
		case SL_STATUS_FAIL:
			LOG_ERR("Failed to init WF200: Error");
			break;
		default:
			LOG_ERR("Failed to init WF200: Unknown error");
		}
		return -ENODEV;
	}
	LOG_INF("WFX200 driver initialized");
	LOG_INF("FW version %d.%d.%d",
		wfx200.sl_context.firmware_major,
		wfx200.sl_context.firmware_minor,
		wfx200.sl_context.firmware_build);
	return 0;
}

ETH_NET_DEVICE_DT_INST_DEFINE(0,
			      wfx200_init, device_pm_control_nop,
			      &wfx200, NULL,
			      CONFIG_ETH_INIT_PRIORITY, &api_funcs, NET_ETH_MTU);
