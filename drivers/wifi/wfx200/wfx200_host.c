/**
 * Copyright (c) 2021 grandcentrix GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_MODULE_NAME wifi_wfx200_host
#define LOG_LEVEL CONFIG_WIFI_LOG_LEVEL

#include <sl_wfx.h>
#undef BIT

#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <stdarg.h>
#include <stdint.h>

#include <zephyr.h>
#include <device.h>
#include <sys/util.h>
#include <drivers/spi.h>
#include <drivers/gpio.h>

#include "wfx200_internal.h"

#ifdef CONFIG_WIFI_WFX200_FIRMWARE_SOURCE_USE_FLASH
#include <sl_wfx_wf200_C0.h>
#endif

#define WFX200_HOST_NOT_IMPLEMENTED() do {		      \
		LOG_ERR("%s: not implemented yet", __func__); \
		return SL_STATUS_FAIL;			      \
} while (0);

extern struct wfx200_dev wfx200_0;

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
	OnFirmwareInit();

	/* Initialization happens in wfx200_init, nothing to do here */
	return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_get_firmware_data(const uint8_t **data, uint32_t data_size)
{
	if (data == NULL || wfx200_0.firmware_pos >= wfx200_0.firmware_size) {
		return SL_STATUS_FAIL;
    }

#ifdef CONFIG_WIFI_WFX200_FIRMWARE_SOURCE_USE_EXTERNAL_MEMORY
	if (!OnFirmwareChunkRequested(data, data_size)) {
		return SL_STATUS_FAIL;
	}
#elif CONFIG_WIFI_WFX200_FIRMWARE_SOURCE_USE_FLASH
	*data = sl_wfx_firmware + wfx200_0.firmware_pos;
#endif

	wfx200_0.firmware_pos += data_size;
	
	return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_get_firmware_size(uint32_t *firmware_size)
{
	if (firmware_size == NULL) {
        return SL_STATUS_FAIL;
    }
#ifdef CONFIG_WIFI_WFX200_FIRMWARE_SOURCE_USE_EXTERNAL_MEMORY
	if (!OnFirmwareSizeRequested(firmware_size)) {
        return SL_STATUS_FAIL;
	}
#elif CONFIG_WIFI_WFX200_FIRMWARE_SOURCE_USE_FLASH
    *firmware_size = sizeof(sl_wfx_firmware);
#endif
    
	wfx200_0.firmware_size = *firmware_size;

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
	if (wfx200_0.hif_sel.dev != NULL) {
		if (IS_ENABLED(CONFIG_WIFI_WFX200_BUS_SPI)) {
			LOG_DBG("selecting SPI interface");
			gpio_pin_set(wfx200_0.hif_sel.dev, wfx200_0.hif_sel.pin, 0);
		} else if (IS_ENABLED(CONFIG_WIFI_WFX200_BUS_SDIO)) {
			LOG_DBG("selecting SDIO interface");
			gpio_pin_set(wfx200_0.hif_sel.dev, wfx200_0.hif_sel.pin, 1);
		}
	}
	gpio_pin_set(wfx200_0.reset.dev, wfx200_0.reset.pin, 1);
	/* arbitrary reset pulse length */
	k_sleep(K_MSEC(50));
	gpio_pin_set(wfx200_0.reset.dev, wfx200_0.reset.pin, 0);
	/* The WFM200 Wi-Fi Expansion Board has a built in reset delay of 1 ms.
	 * Wait here 2 ms to ensure the reset has been released
	 */
	k_sleep(K_MSEC(2));
	return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_set_wake_up_pin(uint8_t state)
{
	if (wfx200_0.wakeup.dev != NULL) {
		if (state > 0) {
			gpio_pin_set(wfx200_0.wakeup.dev, wfx200_0.wakeup.pin, 1);
		} else {
			gpio_pin_set(wfx200_0.wakeup.dev, wfx200_0.wakeup.pin, 0);
		}
	} else {
		LOG_WRN("Set wake up pin requested, but wake up pin not configured");
	}
	return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_wait_for_wake_up(void)
{
	int result;

	if (IS_ENABLED(CONFIG_WIFI_WFX200_SLEEP)) {
		k_sem_take(&wfx200_0.wakeup_sem, K_NO_WAIT);
		/* Time taken from sample code */
		result = k_sem_take(&wfx200_0.wakeup_sem, K_MSEC(3));
		if (result == -EAGAIN) {
			LOG_DBG("Wake up timed out");
			return SL_STATUS_TIMEOUT;
		}
	}
	return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_hold_in_reset(void)
{
	LOG_DBG("holding chip in reset");
	gpio_pin_set(wfx200_0.reset.dev, wfx200_0.reset.pin, 1);
	return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_sleep_grant(sl_wfx_host_bus_transfer_type_t type,
				    sl_wfx_register_address_t address,
				    uint32_t length)
{
	ARG_UNUSED(type);
	ARG_UNUSED(address);
	ARG_UNUSED(length);
	/* TODO: we could give a answer depending on the host devices state */
	if (IS_ENABLED(CONFIG_WIFI_WFX200_SLEEP)) {
		return SL_STATUS_WIFI_SLEEP_GRANTED;
	}
	return SL_STATUS_WIFI_SLEEP_NOT_GRANTED;
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
	k_mutex_lock(&wfx200_0.event_mutex, K_FOREVER);
	/* Resetting the semaphore will unblock all current waiting tasks.
	 * The event, they are waiting for, will never arrive.*/
	if (wfx200_0.waited_event_id != 0 && wfx200_0.waited_event_id != event_id) {
		LOG_WRN("Still waiting for event 0x%02x while setting up waiter for 0x%02x",
			wfx200_0.waited_event_id, event_id);
		k_sem_reset(&wfx200_0.event_sem);
	}
	wfx200_0.waited_event_id = event_id;
	k_mutex_unlock(&wfx200_0.event_mutex);
	return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_wait_for_confirmation(uint8_t confirmation_id,
					      uint32_t timeout_ms,
					      void **event_payload_out)
{
	k_mutex_lock(&wfx200_0.event_mutex, K_FOREVER);
	if (wfx200_0.waited_event_id != confirmation_id) {
		if (wfx200_0.waited_event_id == 0) {
			LOG_DBG("Confirmation waiter wasn't set up, now waiting for event 0x%02x",
				confirmation_id);
		} else {
			LOG_WRN("Confirmation waiter set up to wait for event 0x%02x but waiting for 0x%02x",
				wfx200_0.waited_event_id, confirmation_id);
		}
		k_sem_reset(&wfx200_0.event_sem);
		wfx200_0.waited_event_id = confirmation_id;
	}
	k_mutex_unlock(&wfx200_0.event_mutex);

	if (k_sem_take(&wfx200_0.event_sem, K_MSEC(timeout_ms)) == -EAGAIN) {
		LOG_WRN("Didn't receive confirmation for event 0x%02x", confirmation_id);
		return SL_STATUS_TIMEOUT;
	}
	k_mutex_lock(&wfx200_0.event_mutex, K_FOREVER);
	wfx200_0.waited_event_id = 0;
	k_mutex_unlock(&wfx200_0.event_mutex);
	if (event_payload_out != NULL) {
		*event_payload_out = wfx200_0.sl_context.event_payload_buffer;
	}
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

	k_mutex_lock(&wfx200_0.event_mutex, K_FOREVER);
	if (wfx200_0.waited_event_id == event_payload->header.id) {
		if (event_payload->header.length < sizeof(wfx200_0.sl_context.event_payload_buffer)) {
			/* Post the event into the "queue".
			 * Not using a queue here. The SiLabs examples use the queue similar to a
			 * counting semaphore and are not passing the "queue" elements via the "queue"
			 * but instead with a single buffer. So we can only communicate a single
			 * element at the same time. The sample code also only adds elements to the "queue",
			 * which it has been configured to explicit waiting for. A queue is therefore
			 * not needed.
			 * */
			memcpy(wfx200_0.sl_context.event_payload_buffer,
			       (void *) event_payload,
			       event_payload->header.length);
			k_sem_give(&wfx200_0.event_sem);
		}
	}
	k_mutex_unlock(&wfx200_0.event_mutex);

	return SL_STATUS_OK;
}

/* WFX host callbacks */
void sl_wfx_connect_callback(sl_wfx_connect_ind_t *connect)
{
	struct wfx200_queue_event *event = k_heap_alloc(&wfx200_0.heap,
							sizeof(struct wfx200_queue_event), K_NO_WAIT);

	if (event == NULL) {
		/* TODO */
		return;
	}

	switch (connect->body.status) {
	case WFM_STATUS_SUCCESS:
		LOG_INF("Connected");
		wfx200_0.sl_context.state |= SL_WFX_STA_INTERFACE_CONNECTED;
		event->ev = WFX200_CONNECT_EVENT;
		k_queue_append(&wfx200_0.event_queue, event);
		return;
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
	event->ev = WFX200_CONNECT_FAILED_EVENT;
	k_queue_append(&wfx200_0.event_queue, event);
}

void sl_wfx_disconnect_callback(sl_wfx_disconnect_ind_t *disconnect)
{
	ARG_UNUSED(disconnect);
	struct wfx200_queue_event *event = k_heap_alloc(&wfx200_0.heap,
							sizeof(struct wfx200_queue_event), K_NO_WAIT);

	if (event == NULL) {
		/* TODO */
		return;
	}

	wfx200_0.sl_context.state &= ~SL_WFX_STA_INTERFACE_CONNECTED;
	event->ev = WFX200_DISCONNECT_EVENT;
	k_queue_append(&wfx200_0.event_queue, event);

	LOG_INF("WiFi disconnected");
}

void sl_wfx_start_ap_callback(sl_wfx_start_ap_ind_t *start_ap)
{
	struct wfx200_queue_event *event = k_heap_alloc(&wfx200_0.heap,
							sizeof(struct wfx200_queue_event), K_NO_WAIT);

	if (event == NULL) {
		/* TODO */
		return;
	}

	if (start_ap->body.status == 0) {
		LOG_INF("AP started");
		wfx200_0.sl_context.state |= SL_WFX_AP_INTERFACE_UP;
		event->ev = WFX200_AP_START_EVENT;
	} else {
		LOG_ERR("AP start failed(%d)", start_ap->body.status);
		event->ev = WFX200_AP_START_FAILED_EVENT;
	}
	k_queue_append(&wfx200_0.event_queue, event);
}

void sl_wfx_stop_ap_callback(sl_wfx_stop_ap_ind_t *stop_ap)
{
	struct wfx200_queue_event *event = k_heap_alloc(&wfx200_0.heap,
							sizeof(struct wfx200_queue_event), K_NO_WAIT);

	ARG_UNUSED(stop_ap);

	if (event == NULL) {
		/* TODO */
		return;
	}

	LOG_DBG("AP Stopped");
	wfx200_0.sl_context.state &= ~SL_WFX_AP_INTERFACE_UP;
	event->ev = WFX200_AP_STOP_EVENT;
	k_queue_append(&wfx200_0.event_queue, event);
}

void sl_wfx_host_received_frame_callback(sl_wfx_received_ind_t *rx_buffer)
{
	struct net_pkt *pkt;
	int res;

	if (wfx200_0.iface == NULL) {
		LOG_ERR("Network interface unavailable");
		return;
	}
	switch (wfx200_0.state) {
	case WFX200_STATE_STA_MODE:
		if ((rx_buffer->header.info & SL_WFX_MSG_INFO_INTERFACE_MASK) ==
		    (SL_WFX_SOFTAP_INTERFACE << SL_WFX_MSG_INFO_INTERFACE_OFFSET)) {
			LOG_WRN("Got ethernet packet from softap interface in sta mode. Dropping packet...");
			return;
		}
		break;
	case WFX200_STATE_AP_MODE:
		if ((rx_buffer->header.info & SL_WFX_MSG_INFO_INTERFACE_MASK) ==
		    (SL_WFX_STA_INTERFACE << SL_WFX_MSG_INFO_INTERFACE_OFFSET)) {
			LOG_WRN("Got ethernet packet from sta interface in ap mode. Dropping packet...");
			return;
		}
	default:
		LOG_ERR("Network interface not initialized");
	}

	pkt = net_pkt_rx_alloc_with_buffer(wfx200_0.iface, rx_buffer->body.frame_length,
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
	if ((res = net_recv_data(wfx200_0.iface, pkt)) < 0) {
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
	struct sl_wfx_scan_result_ind_body_s *body = &scan_result->body;
	int rssi = (body->rcpi / 2) - 110;
	struct wifi_scan_result wifi_scan_result = { 0 };

	if (wfx200_0.state < WFX200_STATE_INTERFACE_INITIALIZED ||
	    wfx200_0.scan_cb == NULL) {
		return;
	}
	memcpy(wifi_scan_result.ssid, body->ssid_def.ssid,
	       MIN(sizeof(wifi_scan_result.ssid), body->ssid_def.ssid_length));
	wifi_scan_result.channel = body->channel;
	wifi_scan_result.rssi = rssi;
	wifi_scan_result.security = body->security_mode.psk ?
				    WIFI_SECURITY_TYPE_PSK :
				    WIFI_SECURITY_TYPE_NONE;
	wfx200_0.scan_cb(wfx200_0.iface, 0, &wifi_scan_result);
}

void sl_wfx_scan_complete_callback(sl_wfx_scan_complete_ind_t *scan_complete)
{
	ARG_UNUSED(scan_complete);

	if (wfx200_0.state < WFX200_STATE_INTERFACE_INITIALIZED) {
		return;
	}
	if (scan_complete->body.status == 0) {
		LOG_DBG("Scan complete");
		if (wfx200_0.scan_cb != NULL) {
			wfx200_0.scan_cb(wfx200_0.iface, 0, NULL);
		}
	} else {
		LOG_WRN("Scan failed(%d)", scan_complete->body.status);
		if (wfx200_0.scan_cb != NULL) {
			wfx200_0.scan_cb(wfx200_0.iface, 1, NULL);
		}
	}
	wfx200_0.scan_cb = NULL;
}

void sl_wfx_generic_status_callback(sl_wfx_generic_ind_t *frame)
{
	ARG_UNUSED(frame);
	LOG_DBG("%s", __func__);
}

void sl_wfx_ap_client_connected_callback(sl_wfx_ap_client_connected_ind_t *ap_client_connected)
{
	struct sl_wfx_ap_client_connected_ind_body_s *body = &ap_client_connected->body;

	LOG_INF("Client %02x:%02x:%02x:%02x:%02x:%02x connected to AP",
		body->mac[0],
		body->mac[1],
		body->mac[2],
		body->mac[3],
		body->mac[4],
		body->mac[5]);
}

void sl_wfx_ap_client_rejected_callback(sl_wfx_ap_client_rejected_ind_t *ap_client_rejected)
{
	ARG_UNUSED(ap_client_rejected);
	LOG_DBG("%s", __func__);
}

void sl_wfx_ap_client_disconnected_callback(sl_wfx_ap_client_disconnected_ind_t *ap_client_disconnected)
{
	struct sl_wfx_ap_client_disconnected_ind_body_s *body = &ap_client_disconnected->body;

	LOG_INF("Client %02x:%02x:%02x:%02x:%02x:%02x disconnected from AP",
		body->mac[0],
		body->mac[1],
		body->mac[2],
		body->mac[3],
		body->mac[4],
		body->mac[5]);
}

void sl_wfx_ext_auth_callback(sl_wfx_ext_auth_ind_t *ext_auth_indication)
{
	ARG_UNUSED(ext_auth_indication);
	LOG_DBG("%s", __func__);
}

sl_status_t sl_wfx_host_allocate_buffer(void **buffer,
					sl_wfx_buffer_type_t type,
					uint32_t buffer_size)
{
	ARG_UNUSED(type);

	if (buffer == NULL) {
		return SL_STATUS_FAIL;
	}
	*buffer = k_heap_alloc(&wfx200_0.heap, buffer_size, K_NO_WAIT);
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
		k_heap_free(&wfx200_0.heap, buffer);
	}
	return SL_STATUS_OK;
}

/**
 * Bus access
 */
sl_status_t sl_wfx_host_lock(void)
{
	/* Time taken from silabs sample code */
	if (k_mutex_lock(&wfx200_0.bus_mutex, K_MSEC(500)) == -EAGAIN) {
		LOG_DBG("Wi-Fi driver mutex timeout");
		return SL_STATUS_TIMEOUT;
	}
	return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_unlock(void)
{
	k_mutex_unlock(&wfx200_0.bus_mutex);
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
	wfx200_enable_interrupt(&wfx200_0);
	return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_disable_platform_interrupt(void)

{
	wfx200_disable_interrupt(&wfx200_0);
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
		if ((err = spi_write(wfx200_0.spi, &wfx200_0.spi_cfg, &tx)) < 0) {
			LOG_ERR("spi_write fail: %d", err);
			return SL_STATUS_FAIL;
		}
	} else {
		if ((err = spi_transceive(wfx200_0.spi, &wfx200_0.spi_cfg, &tx, &rx)) < 0) {
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
	if (IS_ENABLED(WIFI_WFX200_VERBOSE_DEBUG)) {
		va_list args;

		va_start(args, string);
		log_printk(string, args);
		va_end(args);
	}
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
