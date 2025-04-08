/* Includes */
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "SPI.h"

/* Private variables */
SPI_HandleTypeDef hspi2;   // SPI2 handle used for communication with the SST25VF080B flash memory chip
TIM_HandleTypeDef htim1;   // Timer 1: used for input capture (ultrasonic echo timing)
TIM_HandleTypeDef htim10;  // Timer 10: used for microsecond delay (DHT11, ultrasonic trigger)
UART_HandleTypeDef huart2; // UART2: used for serial communication

/* Sensor Defines */
#define DHT11_GPIO_PORT GPIOB  	  // GPIO port for DHT11 data pin
#define DHT11_PIN GPIO_PIN_5   	  // DHT11 data pin
#define US_TRIG_PORT GPIOA    	  // GPIO port for ultrasonic TRIG pin
#define US_TRIG_PIN GPIO_PIN_0 	  // Ultrasonic TRIG pin
#define FLASH_CS_PORT GPIOB		  // GPIO port connected to the Chip Select (CS) pin of the SPI flash
#define FLASH_CS_PIN  GPIO_PIN_12 // GPIO pin used as Chip Select (CS) for SPI flash communication

/* Ultrasonic Sensor Variables */
uint32_t echo_start = 0;        // timestamp when echo signal rises
uint32_t echo_end = 0;			// timestamp when echo signal falls
uint32_t echo_duration = 0;		// time between rising and falling edge
uint8_t echo_ready = 0;			// flag: 0 = waiting, 1 = got start, 2 = measurement done
float measured_distance = 0.0;	// calculated distance in cm
uint8_t logging_interval = 1;	// Time between each sensor log (in seconds)
uint8_t duration = 60;			// Total duration of data logging (in seconds)
uint16_t time = 0;				// Elapsed time used for timestamping each log entry

/* DHT11 Variables */
uint8_t hum_int = 0, hum_dec = 0, temp_int = 0, temp_dec = 0; // humidity integer, humidity decimal, temperature integer, temperature decimal
uint8_t checksum = 0;										  // data checksum
uint8_t dht_response = 0;									  // sensor response flag

/* UART Buffer */
char uart_message[100]; // UART transmit buffer
char uart_message2[100]; // UART transmit buffer

/* Private function prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM10_Init(void);
static void MX_SPI2_Init(void);

/* Function Prototypes */
void Trigger_Ultrasonic(void);								 // Sends a 10us pulse to the ultrasonic TRIG pin
void Delay_us(uint16_t time);								 // Microsecond delay using TIM10
void Start_DHT11(void);										 // Begins communication with the DHT11 sensor
uint8_t DHT11_Check_Response(void);							 // Checks if DHT11 sends a response after start signal
uint8_t DHT11_Read_Byte(void);								 // Reads 1 byte (8 bits) from DHT11 sensor
void Set_Pin_Output(GPIO_TypeDef *port, uint16_t pin);  	 // Changes a GPIO pin to output mode
void Set_Pin_Input(GPIO_TypeDef *port, uint16_t pin);		 // Changes a GPIO pin to input mode
void Read_DHT11(void);										 // Reads full temp and humidity data from DHT11
void ProcessSensorData(uint8_t *read_buf, int max_interval); // Formats and prints sensor data from flash over UART

/* TIM Callback */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	// Check if this interrupt is from TIM1 and channel 3 (ultrasonic echo input)
    if (htim->Instance == TIM1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
    {
        HAL_GPIO_TogglePin(GPIOA, LD2_Pin);  // Toggle LED (PA5) for visual debug pulse

        if (echo_ready == 0)
        {
        	// First edge (rising) detected — echo signal just started
            echo_start = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
            echo_ready = 1; // Flag that we’re waiting for the falling edge

            // Change polarity to capture falling edge next
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
        }
        else
        {
        	// Second edge (falling) detected — echo signal ended
            echo_end = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
            __HAL_TIM_SET_COUNTER(htim, 0); // Reset timer count

            // Calculate duration between rising and falling edge
            if (echo_end > echo_start)
                echo_duration = echo_end - echo_start;
            else
                echo_duration = (0xFFFF - echo_start) + echo_end; // 0xFFFF = maximum value of a 16-bit number (65535)

            // Convert duration to distance in cm
            measured_distance = (echo_duration * 0.0343f) / 2.0f; // 0.0343 = speed of sound in cm/us - pulse travels to the object and back ( / 2)

            echo_ready = 2;  // Mark that measurement is complete

            // Reset polarity to capture rising edge for next cycle
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);

            // Stop the input capture interrupt until next trigger
            HAL_TIM_IC_Stop_IT(&htim1, TIM_CHANNEL_3);
        }
    }
}

/* Microsecond Delay Using TIM10 */
void Delay_us(uint16_t time) {
    __HAL_TIM_SET_COUNTER(&htim10, 0);				// reset TIM10 counter to 0
    while (__HAL_TIM_GET_COUNTER(&htim10) < time);  // wait until counter reaches target time
}

/* Trigger Ultrasonic */
void Trigger_Ultrasonic(void)
{
    echo_ready = 0; // reset state flag before starting new measurement

    // configure TIM1 input capture to detect rising edge first
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);

    // clear any pending capture interrupt flag
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC3);

    // enable input capture interrupt on channel 3
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);

    // send 10 microsecond trigger pulse to ultrasonic sensor
    HAL_GPIO_WritePin(US_TRIG_PORT, US_TRIG_PIN, GPIO_PIN_SET);
    Delay_us(10);
    HAL_GPIO_WritePin(US_TRIG_PORT, US_TRIG_PIN, GPIO_PIN_RESET);
}

/* DHT11 Interface - Set GPIO pin as output (MCU drives the line)
   Needed because DHT11 uses a single-wire bidirectional protocol
   MCU pulls the line LOW to initiate communication */
void Set_Pin_Output(GPIO_TypeDef *port, uint16_t pin) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(port, &GPIO_InitStruct);
}

/* DHT11 Interface - Set GPIO pin as input (MCU listens to sensor)
   After sending the start signal, MCU switches to input to read response and data */
void Set_Pin_Input(GPIO_TypeDef *port, uint16_t pin) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(port, &GPIO_InitStruct);
}

/* Sends the start signal to the DHT11 sensor to initiate communication */
void Start_DHT11(void) {
    Set_Pin_Output(DHT11_GPIO_PORT, DHT11_PIN);						// Set DHT11 pin to output mode so MCU can control it
    HAL_GPIO_WritePin(DHT11_GPIO_PORT, DHT11_PIN, GPIO_PIN_RESET);  // Pull pin LOW to send start signal to DHT11
    Delay_us(18000);												// Wait at least 18ms
    HAL_GPIO_WritePin(DHT11_GPIO_PORT, DHT11_PIN, GPIO_PIN_SET);	// Pull pin HIGH to end start signal
    Delay_us(20);													// Wait 20us before switching to input (DHT11 will respond)
    Set_Pin_Input(DHT11_GPIO_PORT, DHT11_PIN);						// Set pin to input mode so MCU can listen to DHT11 response
}

/* Checks for a response from the DHT11 sensor after sending the start signal */
uint8_t DHT11_Check_Response(void) {
    uint8_t resp = 0;												// variable to hold response status (0 = no response, 1 = OK)
    Delay_us(40);													// wait 40us for sensor's response to begin
    if (!HAL_GPIO_ReadPin(DHT11_GPIO_PORT, DHT11_PIN)) {			// sensor should pull line LOW
        Delay_us(80);												// wait 80us for sensor to pull line HIGH again
        if (HAL_GPIO_ReadPin(DHT11_GPIO_PORT, DHT11_PIN)) {			// if line is HIGH, response is valid
        	resp = 1;
        }
    }
    while (HAL_GPIO_ReadPin(DHT11_GPIO_PORT, DHT11_PIN));			// wait while the line stays HIGH (end of response)
    return resp;													// return response status: 1 = sensor responded correctly
}

/* Reads one byte (8 bits) from the DHT11 sensor */
uint8_t DHT11_Read_Byte(void) {
    uint8_t data = 0;
    for (int i = 0; i < 8; i++) {

    	// Wait for the pin to go HIGH (start of bit)
        while (!HAL_GPIO_ReadPin(DHT11_GPIO_PORT, DHT11_PIN));
        Delay_us(40);

        // If the pin is still HIGH after 40us, it's a 1; otherwise, it's a 0
        if (HAL_GPIO_ReadPin(DHT11_GPIO_PORT, DHT11_PIN))
            data |= (1 << (7 - i)); // Set the corresponding bit (MSB (most significant bit) to LSB)

        // Wait for the pin to go LOW again (end of bit)
        while (HAL_GPIO_ReadPin(DHT11_GPIO_PORT, DHT11_PIN));
    }
    return data;
}

/* Reads temperature and humidity data from the DHT11 sensor */
void Read_DHT11(void) {
    Start_DHT11();							// Send start signal to the sensor
    dht_response = DHT11_Check_Response();  // Check if sensor responded correctly
    hum_int = DHT11_Read_Byte();			// Read humidity integer part
    hum_dec = DHT11_Read_Byte();			// Read humidity decimal part
    temp_int = DHT11_Read_Byte();			// Read temperature integer part
    temp_dec = DHT11_Read_Byte();			// Read temperature decimal part
    checksum = DHT11_Read_Byte();			// Read checksum for validation (It should equal the sum of the previous 4 bytes)
}

/* Processes and displays sensor data read from flash memory */
void ProcessSensorData(uint8_t *read_buf, int max_interval)
{
    char uart_message2[100];													 // Buffer to hold formatted UART output
    int len;																	 // Holds length of formatted string

    for (int i = 0; i < max_interval; i++) {									 // Loop through each 8-byte entry in the read buffer
        uint8_t *entry = &read_buf[i * 8];										 // Point to the current 8-byte entry
        uint16_t time = logging_interval * i;  									 // Recalculate timestamp (in seconds) based on logging interval

        // Extract temperature and humidity values from buffer
        uint8_t temp_int = entry[2];
        uint8_t temp_dec = entry[3];
        uint8_t hum_int  = entry[4];
        uint8_t hum_dec  = entry[5];

        uint16_t echo_duration = (entry[6] << 8) | entry[7];					 // Reconstruct 16-bit echo duration from two bytes
        float distance_cm = (echo_duration * 0.0343f) / 2.0f;					 // Convert echo duration to distance in centimeters

        // Skip entries that are clearly invalid (default flash values or failed readings)
        if (temp_int == 0xFF && hum_int == 0xFF && echo_duration == 0xFFFF) {
            continue;
        }

        // Format all data
        len = snprintf(uart_message2, sizeof(uart_message2),
            "TIME: %us | TEMP: %d.%d | RH: %d.%d | DIST: %.2fcm\r\n",
            time, temp_int, temp_dec, hum_int, hum_dec, distance_cm);

        HAL_UART_Transmit(&huart2, (uint8_t *)uart_message2, len, HAL_MAX_DELAY); // Send formatted string over UART
    }
}

#define FLASH_MAX_ADDRESS 0xFFFFF  // Full 1MB for SST25VF080B

int main(void)
{
   HAL_Init();
   SystemClock_Config();
   MX_GPIO_Init();
   MX_USART2_UART_Init();
   MX_TIM1_Init();
   MX_TIM10_Init();
   MX_SPI2_Init();

   HAL_TIM_Base_Start(&htim10);							  // Start TIM10 to enable microsecond delay functionality

   char uart_buf[64];								      // UART transmission buffer to hold debug or sensor data strings
   int uart_buf_len;									  // Stores the length of the formatted UART message

   uint32_t flash_address = 0x000000;  					  // Start writing to SPI flash at address 0x000000
   int max_interval = (duration / logging_interval) + 1;  // Total number of logs based on time and interval
   int i;												  // Loop counter for sensor logging iterations

   for (i = 0; i < max_interval; i++)										   // Loop for each logging interval (based on duration)
   {
       if (flash_address + 8 > FLASH_MAX_ADDRESS)							   // Check if writing 8 more bytes would exceed flash memory limit
       {
           uart_buf_len = sprintf(uart_buf, "FLASH FULL at i = %d\r\n", i);    // Print "FLASH FULL" message with current iteration
           HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
           break;															   // Stop logging to prevent writing out of bounds
       }

       time = logging_interval * i;	 // Calculate the current timestamp in seconds (used later when writing to flash)
       Read_DHT11();				 // Read temperature and humidity values from the DHT11 sensor
       HAL_Delay(5);				 // Short delay to allow the DHT sensor to settle before starting the ultrasonic

       measured_distance = -1.0f;	 // Reset the distance value before triggering the ultrasonic sensor
       Trigger_Ultrasonic();		 // Start ultrasonic measurement by sending a 10us pulse and enabling input capture
       SPI_DisableAAIMode();		 // Ensure AAI mode is disabled before starting a new flash write sequence
       SPI_WriteEnable();			 // Send the Write Enable (WREN) command so we can write to SPI flash

       /*
       uart_buf_len = sprintf(uart_buf, "SPI Test Start\r\n");
       HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);

       uart_buf_len = sprintf(uart_buf, "Wrote to addr: 0x%06lX\r\n", flash_address);
       HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);

       uart_buf_len = sprintf(uart_buf, "Sent WREN (0x06)\r\n");
       HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
	   */

       if ((flash_address % 4096) == 0)					// Check if flash_address is at the start of a 4KB sector (4096 bytes)
       {
    	   // Split the 24-bit flash address into three separate bytes for SPI transmission
           uint8_t ea1 = (flash_address >> 16) & 0xFF;  // Extract most significant byte (MSB)
           uint8_t ea2 = (flash_address >> 8) & 0xFF;	// Extract middle byte
           uint8_t ea3 = flash_address & 0xFF;			// Extract least significant byte (LSB)
           SPI_SectorEraseAt(ea1, ea2, ea3);			// Erase the 4KB sector starting at this address before writing new data
       }

       SPI_WriteEnable();								// Send WREN (0x06) command to enable write/erase operations on flash
       SPI_UnlockBlockProtection();						// Clear block protection bits in flash status register in order to allow writing to all sectors

       uint8_t a1 = (flash_address >> 16) & 0xFF;		// Extract the highest byte
       uint8_t a2 = (flash_address >> 8) & 0xFF;		// Extract the middle byte
       uint8_t a3 = flash_address & 0xFF;				// Extract the lowest byte

       // Split 16-bit time into two 8-bit bytes (MSB and LSB) (0x100 = 256, 256 is the max value for one byte)
       SPI_InitialAAIWrite(a1, a2, a3, time / 0x100, time % 0x100);     // Start the AAI write process by writing the first 2 bytes: the timestamp (split into high and low bytes)
       SPI_WriteAAIPair(temp_int, temp_dec);						    // Write the next 2 bytes: temperature integer and decimal values
       SPI_WriteAAIPair(hum_int, hum_dec);							    // Write the next 2 bytes: humidity integer and decimal values
       SPI_WriteAAIPair(echo_duration / 0x100, echo_duration % 0x100);  // Write the final 2 bytes: ultrasonic echo duration (split into high and low bytes)

       SPI_DisableAAIMode();	// Sends the WRDI (Write Disable) command to safely exit AAI mode
       HAL_Delay(5);			// Short delay to give the flash chip time to settle

       flash_address += 8;      // Simply add 8 each time now

       /*
       int processorTime = HAL_GetTick();
       uart_buf_len = sprintf(uart_buf, "%d ms\r\n", processorTime);
       HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
	   */

       // UART takes 8ms to send, For loop takes 55ms to process
       HAL_Delay((logging_interval * 1000) - 47); // Offset is 55ms - 8ms = 47ms
   }
   SPI_ReadBackAndPrint(i);  // Read and print all logged sensor data from flash (i = total entries written)
}

/* ------------------------------------------------------------------------- */

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* Configure the main internal regulator output voltage   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Initializes the CPU, AHB and APB buses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_SPI2_Init(void)
{
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM1_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM10_Init(void)
{
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 84-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 0xffff-1;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, trig_output_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_5, GPIO_PIN_RESET);

  /* Set CS HIGH by default */
  HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : trig_output_Pin LD2_Pin */
  GPIO_InitStruct.Pin = trig_output_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = FLASH_CS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FLASH_CS_PORT, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
