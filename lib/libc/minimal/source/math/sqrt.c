/*
 * Copyright (c) 2019 Vestas Wind Systems A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>

// borrowed from sqrtf of minimal libc, but with double instead of float, otherwise precision is low
// current minimal libc sqrt implementation is not correct (i.e. low number of iterations)
// https://github.com/zephyrproject-rtos/zephyr/blob/zephyr-v3.3.0/lib/libc/minimal/source/math/sqrt.c
// https://github.com/zephyrproject-rtos/zephyr/issues/55962
#define MINDIFF 2.25e-308
double sqrt(double square)
{
	double root, last, diff;

	root = square / 3.0;

	if (square <= 0) {
		return 0;
	}

	do {
		last = root;
		root = (root + square / root) / 2.0;
		diff = root - last;
	} while (diff > MINDIFF || diff < -MINDIFF);

	return root;
}