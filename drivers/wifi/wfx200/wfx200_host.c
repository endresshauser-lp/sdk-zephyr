/**
 * Copyright (c) 2021 grandcentrix GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_MODULE_NAME wifi_wfx200_host
#define LOG_LEVEL CONFIG_WIFI_LOG_LEVEL

#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <stdarg.h>
#include <stdint.h>

#include <zephyr.h>
#include <kernel.h>
#include <device.h>
#include <sys/util.h>
#include <drivers/spi.h>
#include <drivers/gpio.h>

#include "wfx200_internal.h"

#include <sl_wfx.h>
#include <sl_status.h>
#include <sl_wfx_bus.h>
#include <sl_wfx_host_api.h>
#include <sl_wfx_general_api.h>
#include <sl_wfx_wf200_C0.h>

#define WFX200_HOST_NOT_IMPLEMENTED() do {		      \
		LOG_ERR("%s: not implemented yet", __func__); \
		return SL_STATUS_FAIL;			      \
} while (0);

struct wfx200_dev wfx200 = { 0 };

static const char *const pds_table_brd8023a[] = {
	"{a:{a:4,b:0}}",
	"{b:{a:{a:4,b:0,c:0,d:0,e:A},b:{a:4,b:0,c:0,d:0,e:B},c:{a:4,b:0,c:0,d:0,e:C},d:{a:4,b:0,c:0,d:0,e:D},e:{a:4,b:0,c:0,d:0,e:E},f:{a:4,b:0,c:0,d:0,e:F},g:{a:4,b:0,c:0,d:0,e:G},h:{a:4,b:0,c:0,d:0,e:H},i:{a:4,b:0,c:0,d:0,e:I},j:{a:4,b:0,c:0,d:0,e:J},k:{a:4,b:0,c:0,d:0,e:K},l:{a:4,b:0,c:0,d:1,e:L},m:{a:4,b:0,c:0,d:1,e:M}}}",
	"{c:{a:{a:4},b:{a:6},c:{a:6,c:0},d:{a:6},e:{a:6},f:{a:6}}}",
	"{e:{b:0,c:1}}",
	"{h:{e:0,a:50,b:0,d:0,c:[{a:1,b:[0,0,0,0,0,0]},{a:2,b:[0,0,0,0,0,0]},{a:[3,9],b:[0,0,0,0,0,0]},{a:A,b:[0,0,0,0,0,0]},{a:B,b:[0,0,0,0,0,0]},{a:[C,D],b:[0,0,0,0,0,0]},{a:E,b:[0,0,0,0,0,0]}]}}",
	"{j:{a:0,b:0}}",
};

/* WFX host callbacks */
void sl_wfx_connect_callback(sl_wfx_connect_ind_t *connect);
void sl_wfx_disconnect_callback(sl_wfx_disconnect_ind_t *disconnect);
void sl_wfx_start_ap_callback(sl_wfx_start_ap_ind_t *start_ap);
void sl_wfx_stop_ap_callback(sl_wfx_stop_ap_ind_t *stop_ap);
void sl_wfx_host_received_frame_callback(sl_wfx_received_ind_t *rx_buffer);
void sl_wfx_scan_result_callback(sl_wfx_scan_result_ind_t *scan_result);
void sl_wfx_scan_complete_callback(sl_wfx_scan_complete_ind_t *scan_complete);
void sl_wfx_generic_status_callback(sl_wfx_generic_ind_t *frame);
void sl_wfx_ap_client_connected_callback(sl_wfx_ap_client_connected_ind_t *ap_client_connected);
void sl_wfx_ap_client_rejected_callback(sl_wfx_ap_client_rejected_ind_t *ap_client_rejected);
void sl_wfx_ap_client_disconnected_callback(sl_wfx_ap_client_disconnected_ind_t *ap_client_disconnected);
void sl_wfx_ext_auth_callback(sl_wfx_ext_auth_ind_t *ext_auth_indication);

/**
 * Functions for initializing the host, firmware uploading and pds
 */
sl_status_t sl_wfx_host_init(void)
{
	/* Initialization happens in wfx200_init, nothing to do here */
	return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_get_firmware_data(const uint8_t **data, uint32_t data_size)
{
	if (data == NULL || wfx200.firmware_pos >= sizeof(sl_wfx_firmware)) {
		return SL_STATUS_FAIL;
	}
	*data = sl_wfx_firmware + wfx200.firmware_pos;
	wfx200.firmware_pos += data_size;
	return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_get_firmware_size(uint32_t *firmware_size)
{
	if (firmware_size == NULL) {
		return SL_STATUS_FAIL;
	}
	*firmware_size = sizeof(sl_wfx_firmware);
	return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_get_pds_data(const char **pds_data, uint16_t index)
{
	/*
	 * TODO: Implement proper and not just pass the default
	 */
	if (pds_data == NULL) {
		return SL_STATUS_FAIL;
	}
	*pds_data = pds_table_brd8023a[index];
	return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_get_pds_size(uint16_t *pds_size)
{
	if (pds_size == NULL) {
		return SL_STATUS_FAIL;
	}
	*pds_size = ARRAY_SIZE(pds_table_brd8023a);
	return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_deinit(void)
{
	/*
	 * TODO
	 */
	return SL_STATUS_OK;
}

/**
 * Functions for GPIO Access
 */
sl_status_t sl_wfx_host_reset_chip(void)
{
	LOG_DBG("resetting chip");
	gpio_pin_set(wfx200.reset.dev, wfx200.reset.pin, 1);
	/* arbitrary reset pulse length */
	k_sleep(K_MSEC(50));
	gpio_pin_set(wfx200.reset.dev, wfx200.reset.pin, 0);
	/* The WFM200 Wi-Fi Expansion Board has a built in reset delay of 1 ms.
	 * Wait here 2 ms to ensure the reset has been released
	 */
	k_sleep(K_MSEC(2));
	return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_set_wake_up_pin(uint8_t state)
{
	if (state > 0) {
		gpio_pin_set(wfx200.wakeup.dev, wfx200.wakeup.pin, 1);
	} else {
		gpio_pin_set(wfx200.wakeup.dev, wfx200.wakeup.pin, 0);
	}
	return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_wait_for_wake_up(void)
{
	k_sem_take(&wfx200.wakeup_sem, K_NO_WAIT);
	/* Time taken from sample code */
	k_sem_take(&wfx200.wakeup_sem, K_MSEC(3));
	return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_hold_in_reset(void)
{
	LOG_DBG("holding chip in reset");
	gpio_pin_set(wfx200.reset.dev, wfx200.reset.pin, 1);
	return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_sleep_grant(sl_wfx_host_bus_transfer_type_t type,
				    sl_wfx_register_address_t address,
				    uint32_t length)
{
	ARG_UNUSED(type);
	ARG_UNUSED(address);
	ARG_UNUSED(length);
	return SL_STATUS_WIFI_SLEEP_GRANTED;
}

/**
 * Transmit message to the WFx chip
 */
sl_status_t sl_wfx_host_transmit_frame(void *frame, uint32_t frame_len)
{
	return sl_wfx_data_write(frame, frame_len);
}

/**
 * Functions to handle confirmations and indications
 */
sl_status_t sl_wfx_host_setup_waited_event(uint8_t event_id)
{
	k_mutex_lock(&wfx200.event_mutex, K_FOREVER);
	/* Resetting the semaphore will unblock all current waiting tasks.
	 * The event, they are waiting for, will never arrive.*/
	if (wfx200.waited_event_id != 0 && wfx200.waited_event_id != event_id) {
		LOG_WRN("Still waiting for event 0x%02x while setting up waiter for 0x%02x",
			wfx200.waited_event_id, event_id);
		k_sem_reset(&wfx200.event_sem);
	}
	wfx200.waited_event_id = event_id;
	k_mutex_unlock(&wfx200.event_mutex);
	return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_wait_for_confirmation(uint8_t confirmation_id,
					      uint32_t timeout_ms,
					      void **event_payload_out)
{
	k_mutex_lock(&wfx200.event_mutex, K_FOREVER);
	if (wfx200.waited_event_id != confirmation_id) {
		if (wfx200.waited_event_id == 0) {
			LOG_DBG("Confirmation waiter wasn't set up, now waiting for event 0x%02x",
				confirmation_id);
		} else {
			LOG_WRN("Confirmation waiter set up to wait for event 0x%02x but waiting for 0x%02x",
				wfx200.waited_event_id, confirmation_id);
		}
		k_sem_reset(&wfx200.event_sem);
		wfx200.waited_event_id = confirmation_id;
	}
	k_mutex_unlock(&wfx200.event_mutex);

	if (k_sem_take(&wfx200.event_sem, K_MSEC(timeout_ms)) == -EAGAIN) {
		LOG_WRN("Didn't receive confirmation for event 0x%02x", confirmation_id);
		return SL_STATUS_TIMEOUT;
	}
	k_mutex_lock(&wfx200.event_mutex, K_FOREVER);
	wfx200.waited_event_id = 0;
	if (event_payload_out != NULL) {
		*event_payload_out = wfx200.sl_context.event_payload_buffer;
	}
	k_mutex_unlock(&wfx200.event_mutex);
	return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_wait(uint32_t wait_ms)
{
	k_sleep(K_MSEC(wait_ms));
	return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_post_event(sl_wfx_generic_message_t *event_payload)
{
	union {
		sl_wfx_received_ind_t *ethernet_frame;
		sl_wfx_exception_ind_t *firmware_exception;
		sl_wfx_error_ind_t *firmware_error;
	} u;

	switch (event_payload->header.id) {
	/******** INDICATION ********/
	case SL_WFX_CONNECT_IND_ID:
		sl_wfx_connect_callback((sl_wfx_connect_ind_t *) event_payload);
		break;
	case SL_WFX_DISCONNECT_IND_ID:
		sl_wfx_disconnect_callback((sl_wfx_disconnect_ind_t *) event_payload);
		break;
	case SL_WFX_START_AP_IND_ID:
		sl_wfx_start_ap_callback((sl_wfx_start_ap_ind_t *) event_payload);
		break;
	case SL_WFX_STOP_AP_IND_ID:
		sl_wfx_stop_ap_callback((sl_wfx_stop_ap_ind_t *) event_payload);
		break;
	case SL_WFX_RECEIVED_IND_ID:
		u.ethernet_frame = (sl_wfx_received_ind_t *) event_payload;
		if (u.ethernet_frame->body.frame_type == 0) {
			sl_wfx_host_received_frame_callback(u.ethernet_frame);
		}
		break;
	case SL_WFX_SCAN_RESULT_IND_ID:
		sl_wfx_scan_result_callback((sl_wfx_scan_result_ind_t *) event_payload);
		break;
	case SL_WFX_SCAN_COMPLETE_IND_ID:
		sl_wfx_scan_complete_callback((sl_wfx_scan_complete_ind_t *) event_payload);
		break;
	case SL_WFX_AP_CLIENT_CONNECTED_IND_ID:
		sl_wfx_ap_client_connected_callback((sl_wfx_ap_client_connected_ind_t *) event_payload);
		break;
	case SL_WFX_AP_CLIENT_REJECTED_IND_ID:
		sl_wfx_ap_client_rejected_callback((sl_wfx_ap_client_rejected_ind_t *) event_payload);
		break;
	case SL_WFX_AP_CLIENT_DISCONNECTED_IND_ID:
		sl_wfx_ap_client_disconnected_callback(
			(sl_wfx_ap_client_disconnected_ind_t *) event_payload);
		break;
	case SL_WFX_EXT_AUTH_IND_ID:
		sl_wfx_ext_auth_callback((sl_wfx_ext_auth_ind_t *) event_payload);
		break;
	case SL_WFX_GENERIC_IND_ID:
		break;
	case SL_WFX_EXCEPTION_IND_ID:
		u.firmware_exception = (sl_wfx_exception_ind_t *)event_payload;
		LOG_ERR("Firmware exception %u", u.firmware_exception->body.reason);
		LOG_DBG("Header: id(%hhx) info(%hhx)", u.firmware_exception->header.id,
			u.firmware_exception->header.info);
		LOG_HEXDUMP_DBG(u.firmware_exception->body.data,
				u.firmware_exception->header.length - offsetof(sl_wfx_exception_ind_t, body.data),
				"Body data");
		break;
	case SL_WFX_ERROR_IND_ID:
		u.firmware_error = (sl_wfx_error_ind_t *)event_payload;
		LOG_ERR("Firmware error %u", u.firmware_error->body.type);
		LOG_DBG("Header: id(%hhx) info(%hhx)", u.firmware_error->header.id,
			u.firmware_error->header.info);
		LOG_HEXDUMP_DBG(u.firmware_error->body.data,
				u.firmware_error->header.length - offsetof(sl_wfx_error_ind_t, body.data),
				"Body data");
		break;
	}

	k_mutex_lock(&wfx200.event_mutex, K_FOREVER);
	if (wfx200.waited_event_id == event_payload->header.id) {
		if (event_payload->header.length < sizeof(wfx200.sl_context.event_payload_buffer)) {
			/* Post the event into the "queue".
			 * Not using a queue here. The SiLabs examples use the queue similar to a
			 * counting semaphore and are not passing the "queue" elements via the "queue"
			 * but instead with a single buffer. So we can only communicate a single
			 * element at the same time. The sample code also only adds elements to the "queue",
			 * which it has been configured to explicit waiting for. A queue is therefore
			 * not needed.
			 * */
			memcpy(wfx200.sl_context.event_payload_buffer,
			       (void *) event_payload,
			       event_payload->header.length);
			k_sem_give(&wfx200.event_sem);
		}
	}
	k_mutex_unlock(&wfx200.event_mutex);

	return SL_STATUS_OK;
}

/* WFX host callbacks */
void sl_wfx_connect_callback(sl_wfx_connect_ind_t *connect)
{
	switch (connect->body.status) {
	case WFM_STATUS_SUCCESS:
		LOG_INF("Connected");
		sl_wfx_context->state |= SL_WFX_STA_INTERFACE_CONNECTED;
		if (wfx200.iface_initialized) {
			net_if_up(wfx200.iface);
		}
		break;
	case WFM_STATUS_NO_MATCHING_AP:
		LOG_ERR("Connection failed, access point not found");
		break;
	case WFM_STATUS_CONNECTION_ABORTED:
		LOG_ERR("Connection aborted");
		break;
	case WFM_STATUS_CONNECTION_TIMEOUT:
		LOG_ERR("Connection timeout");
		break;
	case WFM_STATUS_CONNECTION_REJECTED_BY_AP:
		LOG_ERR("Connection rejected by the access point");
		break;
	case WFM_STATUS_CONNECTION_AUTH_FAILURE:
		LOG_ERR("Connection authentication failure");
		break;
	default:
		LOG_ERR("Connection attempt error");
	}
}

struct sl_wfx_scan_result_ind_body_s scanres;
bool found = false;

void connect_to_selected_wifi()
{
	char ssid[33];
	sl_wfx_security_mode_t mode;

	if (found) {
		memcpy(ssid,
		       scanres.ssid_def.ssid,
		       MIN(sizeof(ssid), scanres.ssid_def.ssid_length));
		ssid[scanres.ssid_def.ssid_length] = 0;

		if (scanres.security_mode.wpa2) {
			mode = WFM_SECURITY_MODE_WPA2_PSK;
		} else if (scanres.security_mode.wpa) {
			mode = WFM_SECURITY_MODE_WPA2_WPA1_PSK;
		} else if (scanres.security_mode.wep) {
			mode = WFM_SECURITY_MODE_WEP;
		} else if (!scanres.security_mode.psk && !scanres.security_mode.eap) {
			mode = WFM_SECURITY_MODE_OPEN;
		} else {
			LOG_ERR("Not supported security mode for %s", log_strdup(ssid));
			return;
		}
		LOG_INF("Connecting to %s on channel %d", log_strdup(ssid), scanres.channel);
		sl_wfx_send_join_command(scanres.ssid_def.ssid,
					 scanres.ssid_def.ssid_length,
					 NULL,
					 scanres.channel,
					 mode,
					 0,
					 0,
					 (const uint8_t *)CONFIG_WIFI_WFX200_PSK,
					 strlen(CONFIG_WIFI_WFX200_PSK),
					 NULL,
					 0
					 );
	}
}

void sl_wfx_disconnect_callback(sl_wfx_disconnect_ind_t *disconnect)
{
	ARG_UNUSED(disconnect);
	if (wfx200.iface_initialized) {
		net_if_down(wfx200.iface);
		sl_wfx_context->state &= ~SL_WFX_STA_INTERFACE_CONNECTED;
		LOG_INF("WiFi disconnected");
		if (IS_ENABLED(CONFIG_WIFI_WFX200_AUTOCONNECT)) {
			connect_to_selected_wifi();
		}
	}
}

void sl_wfx_start_ap_callback(sl_wfx_start_ap_ind_t *start_ap)
{
	ARG_UNUSED(start_ap);
	LOG_DBG("%s", __func__);
}

void sl_wfx_stop_ap_callback(sl_wfx_stop_ap_ind_t *stop_ap)
{
	ARG_UNUSED(stop_ap);
	LOG_DBG("%s", __func__);
}

void sl_wfx_host_received_frame_callback(sl_wfx_received_ind_t *rx_buffer)
{
	struct net_pkt *pkt;
	int res;

	if (wfx200.iface == NULL) {
		LOG_ERR("network interface unavailable");
		return;
	}
	pkt = net_pkt_rx_alloc_with_buffer(wfx200.iface, rx_buffer->body.frame_length,
					   AF_UNSPEC, 0, K_NO_WAIT);
	if (!pkt) {
		LOG_ERR("Failed to get net buffer");
		return;
	}
	if (net_pkt_write(pkt, ((uint8_t *)rx_buffer->body.frame) + rx_buffer->body.frame_padding,
			  rx_buffer->body.frame_length) < 0) {
		LOG_ERR("Failed to write pkt");
		goto pkt_unref;
	}
	if ((res = net_recv_data(wfx200.iface, pkt)) < 0) {
		LOG_ERR("Failed to push received data %d", res);
		goto pkt_unref;
	}
	return;
pkt_unref:
	net_pkt_unref(pkt);
	return;
}

void sl_wfx_scan_result_callback(sl_wfx_scan_result_ind_t *scan_result)
{
	char ssid[33];
	struct sl_wfx_scan_result_ind_body_s *body = &scan_result->body;

	memcpy(ssid,
	       body->ssid_def.ssid,
	       MIN(sizeof(ssid), body->ssid_def.ssid_length));
	ssid[body->ssid_def.ssid_length] = 0;

	LOG_INF("%-32s\t%02x:%02x:%02x:%02x:%02x:%02x\t%d\t%s", log_strdup(ssid),
		body->mac[0],
		body->mac[1],
		body->mac[2],
		body->mac[3],
		body->mac[4],
		body->mac[5],
		body->channel,
		((body->security_mode.eap)?"EAP":((body->security_mode.psk)?"PSK":"None"))
		);

	if (IS_ENABLED(CONFIG_WIFI_WFX200_AUTOCONNECT) &&
	    strcmp(ssid, CONFIG_WIFI_WFX200_SSID) == 0) {
		found = true;
		memcpy(&scanres, body, sizeof(scanres));
	}
}

void sl_wfx_scan_complete_callback(sl_wfx_scan_complete_ind_t *scan_complete)
{
	ARG_UNUSED(scan_complete);
	if (IS_ENABLED(CONFIG_WIFI_WFX200_AUTOCONNECT)) {
		connect_to_selected_wifi();
	}
}

void sl_wfx_generic_status_callback(sl_wfx_generic_ind_t *frame)
{
	ARG_UNUSED(frame);
	LOG_DBG("%s", __func__);
}

void sl_wfx_ap_client_connected_callback(sl_wfx_ap_client_connected_ind_t *ap_client_connected)
{
	ARG_UNUSED(ap_client_connected);
	LOG_DBG("%s", __func__);
}

void sl_wfx_ap_client_rejected_callback(sl_wfx_ap_client_rejected_ind_t *ap_client_rejected)
{
	ARG_UNUSED(ap_client_rejected);
	LOG_DBG("%s", __func__);
}

void sl_wfx_ap_client_disconnected_callback(sl_wfx_ap_client_disconnected_ind_t *ap_client_disconnected)
{
	ARG_UNUSED(ap_client_disconnected);
	LOG_DBG("%s", __func__);
}

void sl_wfx_ext_auth_callback(sl_wfx_ext_auth_ind_t *ext_auth_indication)
{
	ARG_UNUSED(ext_auth_indication);
	LOG_DBG("%s", __func__);
}

/**
 * Functions to allocate Memory
 */
#if CONFIG_HEAP_MEM_POOL_SIZE < 2048
#error "WFX200: please allow for at least 2k kernel memory heap"
#endif

sl_status_t sl_wfx_host_allocate_buffer(void **buffer,
					sl_wfx_buffer_type_t type,
					uint32_t buffer_size)
{
	ARG_UNUSED(type);
	if (buffer == NULL) {
		return SL_STATUS_FAIL;
	}
	*buffer = k_malloc(buffer_size);
	if (*buffer == NULL) {
		LOG_ERR("Failed to allocate %d bytes", buffer_size);
		return SL_STATUS_ALLOCATION_FAILED;
	}
	return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_free_buffer(void *buffer, sl_wfx_buffer_type_t type)
{
	ARG_UNUSED(type);
	if (buffer != NULL) {
		k_free(buffer);
	}
	return SL_STATUS_OK;
}

/**
 * Bus access
 */
sl_status_t sl_wfx_host_lock(void)
{
	/* Time taken from silabs sample code */
	if (k_mutex_lock(&wfx200.bus_mutex, K_MSEC(500)) == -EAGAIN) {
		LOG_DBG("Wi-Fi driver mutex timeout");
		return SL_STATUS_TIMEOUT;
	}
	return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_unlock(void)
{
	k_mutex_unlock(&wfx200.bus_mutex);
	return SL_STATUS_OK;
}

/* WF200 host bus API */
sl_status_t sl_wfx_host_init_bus(void)
{
	/* Initialization happens in wfx200_init, nothing to do here */
	return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_deinit_bus(void)
{
	/* Nothing to do here */
	return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_enable_platform_interrupt(void)
{
	/* TODO: implement properly */
	return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_disable_platform_interrupt(void)

{
	/* TODO: implement properly */
	return SL_STATUS_OK;
}

/* WF200 host SPI bus API */
sl_status_t sl_wfx_host_spi_cs_assert(void)
{
	/* Not needed, cs will be handled by spi driver */
	return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_spi_cs_deassert(void)
{
	/* Not needed, cs will be handled by spi driver */
	return SL_STATUS_OK;
}

/* Despite its name cs will be handled by spi hardware */
sl_status_t sl_wfx_host_spi_transfer_no_cs_assert(sl_wfx_host_bus_transfer_type_t type,
						  uint8_t *header,
						  uint16_t header_length,
						  uint8_t *buffer,
						  uint16_t buffer_length)
{
	int err;
	const struct spi_buf_set tx = {
		.buffers = (const struct spi_buf[]){ {
							     .buf = header,
							     .len = header_length
						     }, {
							     .buf = buffer,
							     .len = buffer_length
						     }, },
		.count = (type == SL_WFX_BUS_WRITE)?2:1,
	};
	const struct spi_buf_set rx = {
		.buffers = (const struct spi_buf[]){ {
							     .buf = NULL,
							     .len = header_length
						     }, {
							     .buf = buffer,
							     .len = buffer_length
						     }, },
		.count = 2,
	};

	if (header == NULL || buffer == NULL) {
		return SL_STATUS_FAIL;
	}
	if (type == SL_WFX_BUS_WRITE) {
		if ((err = spi_write(wfx200.spi, &wfx200.spi_cfg, &tx)) < 0) {
			LOG_ERR("spi_write fail: %d", err);
			return SL_STATUS_FAIL;
		}
	} else {
		if ((err = spi_transceive(wfx200.spi, &wfx200.spi_cfg, &tx, &rx)) < 0) {
			LOG_ERR("spi_transceive fail: %d", err);
			return SL_STATUS_FAIL;
		}
	}
	return SL_STATUS_OK;
}

/* WF200 host SDIO bus API */
#ifdef WIFI_WFX200_BUS_SDIO
#error "WFX200: SDIO is currently not supported!"
#endif /* WIFI_WFX200_BUS_SDIO */

sl_status_t sl_wfx_host_sdio_transfer_cmd52(sl_wfx_host_bus_transfer_type_t type,
					    uint8_t function,
					    uint32_t address,
					    uint8_t *buffer)
{
	ARG_UNUSED(type);
	ARG_UNUSED(function);
	ARG_UNUSED(address);
	ARG_UNUSED(buffer);
	WFX200_HOST_NOT_IMPLEMENTED();
}

sl_status_t sl_wfx_host_sdio_transfer_cmd53(sl_wfx_host_bus_transfer_type_t type,
					    uint8_t function,
					    uint32_t address,
					    uint8_t *buffer,
					    uint16_t buffer_length)
{
	ARG_UNUSED(type);
	ARG_UNUSED(function);
	ARG_UNUSED(address);
	ARG_UNUSED(buffer);
	ARG_UNUSED(buffer_length);
	WFX200_HOST_NOT_IMPLEMENTED();
}

sl_status_t sl_wfx_host_sdio_enable_high_speed_mode(void)
{
	WFX200_HOST_NOT_IMPLEMENTED();
}

void sl_wfx_host_log(const char *string, ...)
{
	va_list args;

	va_start(args, string);
	log_printk(string, args);
	va_end(args);
}

/**
 * Functions to support SecureLink
 */
sl_status_t sl_wfx_host_get_secure_link_mac_key(uint8_t *sl_mac_key)
{
	ARG_UNUSED(sl_mac_key);
	WFX200_HOST_NOT_IMPLEMENTED();
}

sl_status_t sl_wfx_host_compute_pub_key(sl_wfx_securelink_exchange_pub_keys_req_body_t *request,
					const uint8_t *sl_mac_key)
{
	ARG_UNUSED(request);
	ARG_UNUSED(sl_mac_key);
	WFX200_HOST_NOT_IMPLEMENTED();
}

sl_status_t sl_wfx_host_verify_pub_key(sl_wfx_securelink_exchange_pub_keys_ind_t *response_packet,
				       const uint8_t *sl_mac_key,
				       uint8_t *sl_host_pub_key)
{
	ARG_UNUSED(response_packet);
	ARG_UNUSED(sl_mac_key);
	ARG_UNUSED(sl_host_pub_key);
	WFX200_HOST_NOT_IMPLEMENTED();
}

sl_status_t sl_wfx_host_free_crypto_context(void)
{
	WFX200_HOST_NOT_IMPLEMENTED();
}

sl_status_t sl_wfx_host_decode_secure_link_data(uint8_t *buffer,
						uint32_t length,
						uint8_t *session_key)
{
	ARG_UNUSED(buffer);
	ARG_UNUSED(length);
	ARG_UNUSED(session_key);
	WFX200_HOST_NOT_IMPLEMENTED();
}

sl_status_t sl_wfx_host_encode_secure_link_data(sl_wfx_generic_message_t *buffer,
						uint32_t data_length,
						uint8_t *session_key,
						uint8_t *nonce)
{
	ARG_UNUSED(buffer);
	ARG_UNUSED(data_length);
	ARG_UNUSED(session_key);
	ARG_UNUSED(nonce);
	WFX200_HOST_NOT_IMPLEMENTED();
}

sl_status_t sl_wfx_host_schedule_secure_link_renegotiation(void)
{
	WFX200_HOST_NOT_IMPLEMENTED();
}
