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
	k_mutex_lock(cache->mut, K_FOREVER);
	for (size_t i = 0; i < cache->size; i++) {
		cache->entries[i].creation_uptime = 0;
	}
	k_mutex_unlock(cache->mut);

	return 0;
}

int dns_cache_add(struct dns_cache *cache, char const *query, struct dns_addrinfo const *addrinfo,
		  uint32_t ttl)
{
	int ret = -1;
	int64_t oldest_entry_uptime = INT64_MAX;
	size_t index_to_replace = 0;

	if (cache == NULL || query == NULL || addrinfo == NULL || ttl == 0) {
		return -EINVAL;
	}

	if (strlen(query) >= DNS_MAX_NAME_LEN) {
		LOG_WRN("Query string to big to be processed %u >= DNS_MAX_NAME_LEN",
			strlen(query));
		return -EINVAL;
	}

	k_mutex_lock(cache->mut, K_FOREVER);

	LOG_ERR("ADD \"%s\" with TTL %u", query, ttl);

	dns_cache_clean(cache);

	for (size_t i = 0; i < cache->size; i++) {
		if (cache->entries[i].creation_uptime <= 0) {
			index_to_replace = i;
			break;
		} else if (cache->entries[i].creation_uptime < oldest_entry_uptime) {
			index_to_replace = i;
			oldest_entry_uptime = cache->entries[i].creation_uptime;
		}
	}

	strncpy(cache->entries[index_to_replace].query, query, DNS_MAX_NAME_LEN - 1);
	cache->entries[index_to_replace].data = *addrinfo;
	cache->entries[index_to_replace].ttl = ttl;
	cache->entries[index_to_replace].creation_uptime = k_uptime_get();

	k_mutex_unlock(cache->mut);

	return 0;
}

int dns_cache_remove(struct dns_cache *cache, char const *query)
{
	LOG_ERR("REMOVE all entries with query \"%s\"", query);
	if (strlen(query) >= DNS_MAX_NAME_LEN) {
		LOG_WRN("Query string to big to be processed %u >= DNS_MAX_NAME_LEN",
			strlen(query));
		return -EINVAL;
	}

	k_mutex_lock(cache->mut, K_FOREVER);

	dns_cache_clean(cache);

	for (size_t i = 0; i < cache->size; i++) {
		if (cache->entries[i].creation_uptime > 0 &&
		    strcmp(cache->entries[i].query, query) == 0) {
			cache->entries[i].creation_uptime = 0;
		}
	}

	k_mutex_unlock(cache->mut);

	return 0;
}

int dns_cache_find(struct dns_cache const *cache, const char *query, struct dns_addrinfo *addrinfo,
		   size_t addrinfo_array_len)
{
	size_t found = 0;
	LOG_ERR("FIND \"%s\"", query);
	if (cache == NULL || query == NULL || addrinfo == NULL || addrinfo_array_len <= 0) {
		return -EINVAL;
	}
	if (strlen(query) >= DNS_MAX_NAME_LEN) {
		LOG_WRN("Query string to big to be processed %u >= DNS_MAX_NAME_LEN",
			strlen(query));
		return -EINVAL;
	}

	k_mutex_lock(cache->mut, K_FOREVER);

	dns_cache_clean(cache);

	for (size_t i = 0; i < cache->size; i++) {
		if (cache->entries[i].creation_uptime == 0) {
			continue;
		}
		if (strcmp(cache->entries[i].query, query) != 0) {
			continue;
		}
		if (found >= addrinfo_array_len) {
			LOG_WRN("FOUND \"%s\" but not enough space in provided buffer.", query);
			found++;
		} else {
			addrinfo[found] = cache->entries[i].data;
			found++;
			LOG_ERR("FOUND \"%s\"", query);
		}
	}

	k_mutex_unlock(cache->mut);

	if (found > addrinfo_array_len) {
		return -ENOSR;
	}

	if (found == 0) {
		LOG_ERR("COULD NOT FIND \"%s\"", query);
	}
	return found;
}

/* Needs to be called when lock is already aquired */
static void dns_cache_clean(struct dns_cache const *cache)
{
	for (size_t i = 0; i < cache->size; i++) {
		if (cache->entries[i].creation_uptime == 0) {
			continue;
		}
		int64_t reftime = cache->entries[i].creation_uptime;
		int64_t delta_seconds = k_uptime_delta(&reftime) / 1000;

		if (delta_seconds >= cache->entries[i].ttl) {
			LOG_ERR("REMOVE \"%s\"", cache->entries[i].query);
			cache->entries[i].creation_uptime = 0;
		}
	}
}
