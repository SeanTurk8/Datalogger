This project logs temperature, humidity, and distance sensor data using an STM32 microcontroller. Data is collected in real-time from a DHT11 and ultrasonic sensor, stored in SPI flash memory (SST25VF080B) using AAI (Auto Address Increment) mode, and then read back and printed over UART in a human-readable format.

Power the system – Connect your microcontroller and power it through USB or an external source.
Start logging – The sensors will automatically start recording data every 1 second (or whatever interval is set in the code).
Data is saved – Sensor values (temperature, humidity, and distance) are written in 8-byte blocks to flash memory. Each new reading is saved to a new location so no data is overwritten.
Wait for completion – After logging for the set duration (e.g. 30 or 60 seconds), data collection will stop.
Read the data – The system will automatically read back the saved data from flash memory and display it over UART. You can view this using any serial terminal (e.g., PuTTY, Arduino Serial Monitor, etc.).
