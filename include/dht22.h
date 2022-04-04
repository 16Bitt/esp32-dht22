#pragma once

/*
 * DHT22 interface
 */

/*
 * Configure GPIO ISRs and sensor data queue for DHT22
 */
void dht22_initialize(void);

/*
 * Delay task in order to give sensor time to come online
 */
void dht22_delay(void);

/*
 * Read from dht22, updating humidity, temp, and valid_checksum on success. On
 * failure, returns -1. Note that an invalid checksum will not return -1;
 * callers are expected to check the valid_checksum value.
 */
int dht22_read(float *humidity, float *temp, bool *valid_checksum);
