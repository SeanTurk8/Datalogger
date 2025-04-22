## How It Works (Step-by-Step)

This STM32 project logs temperature, humidity, and distance sensor data using an STM32F4 microcontroller. It reads values in real time from a DHT11 sensor and an HC-SR04 ultrasonic sensor. The data is stored into SPI flash memory (SST25VF080B) using **AAI (Auto Address Increment)** mode, and can be read back and printed over UART in a human-readable format.

### 1. Power the System  
Connect your STM32 board via USB or an external power source. The code starts automatically after power-on.

### 2. Start Logging  
The logging loop runs in `main.c`. Sensor readings are taken at a regular interval (default is every 1 second).  
You can adjust this by modifying:
```
#define LOGGING_INTERVAL 1000  // Delay between logs in milliseconds
```

### 3. Save Data to Flash  
Each complete reading is written to SPI flash in an **8-byte block** using AAI mode:
- 2 bytes: timestamp (from wakeup timer or counter)
- 2 bytes: temperature (integer and decimal parts)
- 2 bytes: humidity (integer and decimal parts)
- 2 bytes: ultrasonic echo time (used to calculate distance)

All values are stored in separate memory locations to prevent overwrite.

### 4. Wait for Logging to Finish  
Logging continues until the configured duration is reached. The default is 60 seconds.  
You can change this by editing:
```
#define TOTAL_LOG_DURATION 60000  // in milliseconds
```

### 5. Read Back and Display  
Once logging is done, the system reads data back from flash memory and prints it over UART in this format:
```
Address: 0x000000 TEMP: 22.00 RH: 45.00 DIST: 100.23
Address: 0x000010 TEMP: 22.00 RH: 45.00 DIST: 100.23
...
```

Each row shows the address and decoded sensor values (temperature, humidity, and distance).

## How to Customize

### Change Logging Interval
In `main.c`, modify the delay between each sensor read:
```
#define LOGGING_INTERVAL 1000  // ms between each reading
```

### Change Total Logging Duration
To adjust how long data is collected:
```
#define TOTAL_LOG_DURATION 60000  // Total runtime in milliseconds
```

### Adjust SPI Flash Storage
Each log entry uses 8 bytes. To log more or fewer entries, modify how many times the logging loop runs, or expand available memory regions. The flash chip used (SST25VF080B) supports 8Mbit total size.

### View Output
Use any UART serial monitor (9600 baud) to see output:
- PuTTY  
- Arduino Serial Monitor  
- Tera Term  
- RealTerm  

Now the STM32 logs sensor data into SPI flash and displays it live once logging finishes.
