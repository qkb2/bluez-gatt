#pragma once

#include <stdbool.h>
#include <stdint.h>

/* call at startup */
bool ble_client_start(void);

void ble_client_stop(void);

bool ble_is_connected(void);

/* sensor getters (thread-safe) */
bool ble_get_temperature(float *out_celsius);
bool ble_get_pressure(float *out_hpa);
bool ble_get_humidity(float *out_rh);
