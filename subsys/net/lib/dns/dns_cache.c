/*
 * Copyright (c) 2024 Endress+Hauser AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/net/dns_resolve.h>
#include "dns_cache.h"

LOG_MODULE_REGISTER(net_dns_cache, CONFIG_LOG_DEFAULT_LEVEL);

static void dns_cache_clean(struct dns_cache const *cache);

int dns_cache_flush(struct dns_cache *cache)
{
	for (size_t i = 0; i < cache->size; i++) {
		cache->entries[i].uptime = 0;
	}

	return 0;
}

int dns_cache_add(struct dns_cache *cache, char const *query, struct dns_addrinfo const *addrinfo,
		  uint32_t ttl)
{
	LOG_DBG("ADD \"%s\" with TTL %u", query, ttl);
	int64_t oldest_entry_uptime = INT64_MAX;
	size_t index_to_replace = 0;
	bool found_empty_slot = false;

	if (cache == NULL || query == NULL || addrinfo == NULL || ttl == 0) {
		return -EINVAL;
	}
	if (strlen(query) >= DNS_CACHE_QUERY_MAX_SIZE) {
        LOG_WRN("Query string to big to be processed %u >=" DNS_CACHE_QUERY_MAX_SIZE, strlen(query));
		return -EINVAL;
	}

	dns_cache_clean(cache);

	for (size_t i = 0; i < cache->size; i++) {
		if (cache->entries[i].uptime > 0 &&
		    memcmp(cache->entries[i].query, query, strlen(query)) == 0) {
			index_to_replace = i;
			break;
		}
		if (!found_empty_slot) {
			if (cache->entries[i].uptime <= 0) {
				index_to_replace = i;
				found_empty_slot = true;
			} else if (cache->entries[i].uptime < oldest_entry_uptime) {
				index_to_replace = i;
				oldest_entry_uptime = cache->entries[i].uptime;
			}
		}
	}

	memset(cache->entries[index_to_replace].query, 0, DNS_CACHE_QUERY_MAX_SIZE);
	memcpy(cache->entries[index_to_replace].query, query, strlen(query));
	memcpy(&cache->entries[index_to_replace].data, addrinfo, sizeof(*addrinfo));
	cache->entries[index_to_replace].ttl = ttl;
	cache->entries[index_to_replace].uptime = k_uptime_get();

	return 0;
}

int dns_cache_find(struct dns_cache const *cache, const char *query, struct dns_addrinfo *addrinfo)
{
	LOG_DBG("FIND \"%s\"", query);
	if (cache == NULL || query == NULL || addrinfo == NULL) {
		return -EINVAL;
	}
	if (strlen(query) >= DNS_CACHE_QUERY_MAX_SIZE) {
        LOG_WRN("Query string to big to be processed %u >=" DNS_CACHE_QUERY_MAX_SIZE, strlen(query));
		return -EINVAL;
	}

	dns_cache_clean(cache);

	for (size_t i = 0; i < cache->size; i++) {
		if (cache->entries[i].uptime == 0) {
			continue;
		}
		if (strlen(cache->entries[i].query) != strlen(query)) {
			continue;
		}
		if (memcmp(cache->entries[i].query, query, strlen(query)) != 0) {
			continue;
		}
		memcpy(addrinfo, &cache->entries[i].data, sizeof(*addrinfo));
		LOG_DBG("FOUND \"%s\"", query);
		return 0;
	}

	LOG_DBG("COULD NOT FIND \"%s\"", query);
	return -1;
}

static void dns_cache_clean(struct dns_cache const *cache)
{
	for (size_t i = 0; i < cache->size; i++) {
		if (cache->entries[i].uptime == 0) {
			continue;
		}
		int64_t reftime = cache->entries[i].uptime;
		int64_t delta_seconds = k_uptime_delta(&reftime) / 1000;

		if (delta_seconds >= cache->entries[i].ttl) {
			LOG_DBG("REMOVE \"%s\"", cache->entries[i].query);
			cache->entries[i].uptime = 0;
		}
	}
}
