# ESP32 DHT22 Interface

A simple library for interfacing DHT22 sensors via GPIO. The DHT22 is a simple sensor chip that can
read temperature and humidity. All communication occurs over a single data line.

## Configuration

All configuration is available through the standard `idf.py menuconfig`, through the DHT22
component.

## Example

```c
#include "dht22.h"

void app_main() {
  /* Configure GPIO and interrupts for DHT22 */
  dht22_initialize();

  int result = -1;
  float temp, humidity;
  bool checksum_passed;

  while (result == -1) {
    /* Delay task a safe duration to poll chip */
    dht22_delay(); 

    /* Attempt to read sensor data */
    result = dht22_read(&humidity, &temp, &valid_checksum);
  }

  if (valid_checksum) {
    ESP_LOGI(TAG, "Temp: %f Humidity: %f", temp, humidity);
  } else {
    ESP_LOGI(TAG, "Invalid checksum reading sensor data");
  }
}
```

## What's different about this implementation?

After reading through some other implementations and writing my own, I noticed that all
implementations effectively busy-loop waiting for readings from the sensor. The DHT22 hardware
interface can be split into three phases:

1. The host (ESP32) pulls the signal low and then high
2. The DHT22 pulls the signal low and then high
3. The DHT22 pulls the signal low and then high fourty times, each indicating a bit in the data
   stream

While step 1 requires the host to write to the data line with exact timings, all of the data
_received_ from the sensor could be processed in the background through interrupts. As long as we
can time the transition between each edge from the signal, it's easy to handle without burning too
much CPU.

Here's the updated process:

1. The host (ESP32) pulls the signal low and then high
2. The host enables interrupts on any edge from the sensor data line
3. The duration of each edge is pushed into a queue through an interrupt service routine
4. When CPU time is available, the ESP32 can process the data stream from the queue.

Whether this is much more efficient/practical than bit-banging the data line is unclear to me, but
it was a fun project nonetheless!
