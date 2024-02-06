/** @file
 * @brief DNS cache
 *
 * An cache holding dns records for faster dns resolving.
 */

/*
 * Copyright (c) 2024 Endress+Hauser AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_NET_DNS_CACHE_H_
#define ZEPHYR_INCLUDE_NET_DNS_CACHE_H_

#include <stdint.h>
#include <zephyr/net/dns_resolve.h>

#define DNS_CACHE_QUERY_MAX_SIZE 255

struct dns_cache_entry {
	char query[DNS_CACHE_QUERY_MAX_SIZE];
	struct dns_addrinfo data;
	uint32_t ttl;
	int64_t uptime;
};

struct dns_cache {
	struct dns_cache_entry *entries;
	uint8_t size;
};

/**
 * @brief Statically define and initialize a DNS queue.
 *
 * The cache can be accessed outside the module where it is defined using:
 *
 * @code extern struct dns_cache <name>; @endcode
 *
 * @param name Name of the cache.
 */
#define DNS_CACHE_DEFINE(name, cache_size)                                                         \
	static struct dns_cache_entry name##_entries[cache_size];                                  \
	static struct dns_cache name = {                                                           \
		.entries = name##_entries,                                                         \
		.size = cache_size,                                                                \
	};

/**
 * @brief Flushes the dns cache removing all its entries.
 *
 * @param cache Cache to be flushed
 * @retval 0 on success
 * @retval On error, a negative value is returned.
 */
int dns_cache_flush(struct dns_cache *cache);

/**
 * @brief Adds a new entry to the dns cache removing the oldest one if no free
 * space is available.
 *
 * @param cache Cache where the entry should be added.
 * @param query Query which should be persisted in the cache.
 * @param addrinfo Addrinfo resulting from the query which will be returned
 * upon cache hit.
 * @param ttl Time to live for the entry in seconds. This usually represents
 * the TTL of the RR.
 * @retval 0 on success
 * @retval On error, a negative value is returned.
 */
int dns_cache_add(struct dns_cache *cache, char const *query, struct dns_addrinfo const *addrinfo,
		  uint32_t ttl);

/**
 * @brief Tries to find the specified query entry within the cache.
 *
 * @param cache Cache where the entry should be searched.
 * @param query Query which should be searched for.
 * @param addrinfo Addrinfo which will be written if the query was found.
 * @retval 0 on success
 * @retval On error or cache miss, a negative value is returned.
 * -1 means cache miss.
 */
int dns_cache_find(struct dns_cache const *cache, const char *query, struct dns_addrinfo *addrinfo);

#endif /* ZEPHYR_INCLUDE_NET_DNS_CACHE_H_ */