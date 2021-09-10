/*
 * Copyright (c) 2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Public API for network link address
 */

#ifndef ZEPHYR_INCLUDE_NET_NET_LINKADDR_H_
#define ZEPHYR_INCLUDE_NET_NET_LINKADDR_H_

#include <zephyr/types.h>
#include <stdbool.h>
#include <errno.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Network link address library
 * @defgroup net_linkaddr Network Link Address Library
 * @ingroup networking
 * @{
 */

/** Maximum length of the link address */
#ifdef CONFIG_NET_L2_IEEE802154
#define NET_LINK_ADDR_MAX_LENGTH 8
#else
#ifdef CONFIG_NET_L2_PPP
#define NET_LINK_ADDR_MAX_LENGTH 8
#else
#define NET_LINK_ADDR_MAX_LENGTH 6
#endif
#endif

/**
 * Type of the link address. This indicates the network technology that this
 * address is used in. Note that in order to save space we store the value
 * into a uint8_t variable, so please do not introduce any values > 255 in
 * this enum.
 */
enum net_link_type {
	/** Unknown link address type. */
	NET_LINK_UNKNOWN = 0,
	/** IEEE 802.15.4 link address. */
	NET_LINK_IEEE802154,
	/** Bluetooth IPSP link address. */
	NET_LINK_BLUETOOTH,
	/** Ethernet link address. */
	NET_LINK_ETHERNET,
	/** Dummy link address. Used in testing apps and loopback support. */
	NET_LINK_DUMMY,
	/** CANBUS link address. */
	NET_LINK_CANBUS_RAW,
	/** 6loCAN link address. */
	NET_LINK_CANBUS,
} __packed;

/**
 *  @brief Hardware link address structure
 *
 *  Used to hold the link address information
 */
struct net_linkaddr {
	/** What kind of address is this for */
	uint8_t type;

	/** The real length of the ll address. */
	uint8_t len;

	/** The array of bytes representing the address */
	uint8_t addr[NET_LINK_ADDR_MAX_LENGTH];
};

/**
 *  @def net_linkaddr_copy
 *  @brief Copy Link address
 *
 *  @param dest Destination Link address.
 *  @param src Source Link address.
 *
 *  @return Destination address.
 */
#define net_linkaddr_copy(dest, src) \
	UNALIGNED_PUT(UNALIGNED_GET(src), dest)

const struct net_linkaddr *net_linkaddr_eth_unspecified_address(void);

const struct net_linkaddr *net_linkaddr_eth_broadcast_address(void);

/**
 * @brief Compare two link layer addresses.
 *
 * @param lladdr1 Pointer to a link layer address
 * @param lladdr2 Pointer to a link layer address
 *
 * @return True if the addresses are the same, false otherwise.
 */
static inline bool net_linkaddr_cmp(struct net_linkaddr *lladdr1,
				    struct net_linkaddr *lladdr2)
{
	if (!lladdr1 || !lladdr2) {
		return false;
	}

	if (lladdr1->len != lladdr2->len) {
		return false;
	}

	if (lladdr1->type != lladdr2->type) {
		return false;
	}

	return !memcmp(lladdr1->addr, lladdr2->addr, lladdr1->len);
}

/**
 *
 * @brief Set the member data of a link layer address storage structure.
 *
 * @param lladdr_store The link address storage structure to change.
 * @param new_addr Array of bytes containing the link address.
 * @param new_len Length of the link address array.
 * This value should always be <= NET_LINK_ADDR_MAX_LENGTH.
 */
static inline int net_linkaddr_set(struct net_linkaddr *lladdr,
				   const uint8_t *new_addr, uint8_t new_len,
				   enum net_link_type new_type)
{
	if (!lladdr || !new_addr) {
		return -EINVAL;
	}

	if (new_len > NET_LINK_ADDR_MAX_LENGTH) {
		return -EMSGSIZE;
	}

	lladdr->type = new_type;
	lladdr->len = new_len;
	memcpy(lladdr->addr, new_addr, new_len);

	return 0;
}

static inline int net_linkaddr_eth_set(struct net_linkaddr *linkaddr, const uint8_t *addr)
{
	return net_linkaddr_set(linkaddr, addr, 6, NET_LINK_ETHERNET);
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_NET_NET_LINKADDR_H_ */
