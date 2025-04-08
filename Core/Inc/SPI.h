/* Includes */
#include "main.h"
#include <stdio.h>
#include <string.h>

/* Private variables */
SPI_HandleTypeDef hspi2;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
const uint8_t Read = 0x03;				 // SPI Flash command to read data starting from a specified address
const uint8_t KbyteSectorErase = 0x20;	 // SPI Flash command to erase a 4KB sector starting from a specified address
const uint8_t AAIWordProgram = 0xAD;	 // SPI Flash command for AAI mode (2 bytes at a time)
const uint8_t RDSR = 0x05;				 // SPI Flash command to read the status register
const uint8_t EWSR = 0x50;			 	 // SPI Flash command to enable writing to the status register
const uint8_t WRSR = 0x01;				 // SPI Flash command to write to the status register
const uint8_t WREN = 0x06;				 // SPI Flash command to enable writing/erasing to flash
const uint8_t WRDI = 0x04;				 // SPI Flash command to disable writing
uint8_t status_unlock = 0x00;			 // Data byte used to clear block protection bits in the status register

/* Flash Functions */
void SPI_WriteEnable(void);																// Sends Write Enable (WREN) command to enable flash write/erase operations
void SPI_DisableAAIMode(void);															// Sends Write Disable (WRDI) command to safely exit AAI programming mode
void SPI_SectorEraseAt(uint8_t a1, uint8_t a2, uint8_t a3);								// Sends Sector Erase command (0x20) with a 3-byte address to erase a 4KB flash memory block
void SPI_UnlockBlockProtection(void);													// Clears block protection bits in the flash status register (WRSR 0x00)
void SPI_WriteAAIPair(uint8_t d1, uint8_t d2);											// Writes a 2-byte pair using AAI mode
void SPI_InitialAAIWrite(uint8_t a1, uint8_t a2, uint8_t a3, uint8_t d1, uint8_t d2);	// Performs the first AAI write: address + two bytes
void SPI_ReadStatusAndPrint(void);														// Reads the flash status register and sends it via UART for debugging
void SPI_ReadBackAndPrint(int max_interval);											// Reads max_interval entries (8 bytes each) from SPI flash and sends to UART
void ProcessSensorData(uint8_t *read_buf, int max_interval);							// Converts flash data into readable sensor output over UART

/* Waits until the SPI flash memory has completed its internal write or erase operation */
void WaitForWriteComplete() {
  uint8_t status = 0xFF; 								   // Initialize status byte to a non-zero value
  do {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // Begin SPI transaction by pulling Chip Select (CS) low
    HAL_SPI_Transmit(&hspi2, (uint8_t *)&RDSR, 1, 100);    // Send the Read Status Register (RDSR) command (0x05) to the flash
    HAL_SPI_Receive(&hspi2, &status, 1, 100);			   // Receive 1 byte status response from the flash
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);   // End SPI transaction by pulling CS high
  } while (status & 0x01);  							   // Loop while the BUSY bit (bit 0) is set (1 = busy, 0 = ready)
}

/* Sends the Write Enable (WREN) command to the SPI flash */
void SPI_WriteEnable() {
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // Pull Chip Select (CS) low to begin SPI transaction
  HAL_SPI_Transmit(&hspi2, (uint8_t *)&WREN, 1, 100);	 // Transmit the WREN (0x06) command over SPI
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);	 // Pull CS high to end the SPI transaction
}

/* Sends the Write Disable (WRDI) command to exit AAI mode */
void SPI_DisableAAIMode() {
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // Pull Chip Select (CS) low to start SPI communication
  HAL_SPI_Transmit(&hspi2, (uint8_t *)&WRDI, 1, 100);	 // Send WRDI (0x04) command to disable write operations
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);	 // Pull CS high to end SPI communication
}

/* Erases a 4KB sector at the specified 3-byte address in SPI flash memory */
void SPI_SectorEraseAt(uint8_t a1, uint8_t a2, uint8_t a3) {
  uint8_t cmd[4] = {KbyteSectorErase, a1, a2, a3};		    // Prepare command: 0x20 + 3-byte address
  SPI_WriteEnable();										// Enable writing to flash
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);	// Pull CS low to begin SPI transaction
  HAL_SPI_Transmit(&hspi2, cmd, 4, 100);					// Send the erase command with the address
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);		// Pull CS high to end SPI transaction
  WaitForWriteComplete();									// Wait until erase operation is complete (BUSY = 0)
}

/* Unlocks the status register and clears block protection bits so the flash can be written or erased */
void SPI_UnlockBlockProtection() {
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);	    // Pull CS low to begin SPI transaction
  HAL_SPI_Transmit(&hspi2, (uint8_t *)&EWSR, 1, 100);			// Send Enable Write Status Register command
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);			// Pull CS high to end transaction

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);		// Pull CS low to start second SPI transaction
  HAL_SPI_Transmit(&hspi2, (uint8_t *)&WRSR, 1, 100);			// Send Write Status Register command
  HAL_SPI_Transmit(&hspi2, (uint8_t *)&status_unlock, 1, 100);	// Send data 0x00 to clear all block protection bits
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);			// Pull CS high to end transaction
  /*
  char uart_buf[50];
  int uart_buf_len = sprintf(uart_buf, "Cleared block protection (WRSR 0x00)\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
  */
}

/* Performs the first AAI (Auto Address Increment) write to the SPI flash */
void SPI_InitialAAIWrite(uint8_t a1, uint8_t a2, uint8_t a3, uint8_t d1, uint8_t d2) {
  SPI_WriteEnable();											// Enable write operations by sending the WREN command (0x06)
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);		// Pull CS low to begin SPI transaction
  HAL_SPI_Transmit(&hspi2, (uint8_t *)&AAIWordProgram, 1, 100);	// Send AAI command (0xAD) to enter AAI mode
  HAL_SPI_Transmit(&hspi2, &a1, 1, 100);						// Send address byte 1 (MSB)
  HAL_SPI_Transmit(&hspi2, &a2, 1, 100);						// Send address byte 2
  HAL_SPI_Transmit(&hspi2, &a3, 1, 100);						// Send address byte 3 (LSB)
  HAL_SPI_Transmit(&hspi2, &d1, 1, 100);						// Send first data byte to be written
  HAL_SPI_Transmit(&hspi2, &d2, 1, 100);						// Send second data byte to be written
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);			// Pull CS high to end SPI transaction
  WaitForWriteComplete();										// Wait for BUSY bit to clear (write process must finish before continuing)
}

/* Writes a 2-byte data pair using Auto Address Increment (AAI) mode */
void SPI_WriteAAIPair(uint8_t d1, uint8_t d2) {
  SPI_WriteEnable();											// Enable write operation (WREN must be sent before each AAI pair)
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);		// Pull CS low to begin SPI transaction
  HAL_SPI_Transmit(&hspi2, (uint8_t *)&AAIWordProgram, 1, 100); // Send AAI command (0xAD) to continue AAI mode
  HAL_SPI_Transmit(&hspi2, &d1, 1, 100);						// Send first byte of the data pair
  HAL_SPI_Transmit(&hspi2, &d2, 1, 100);						// Send second byte of the data pair
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);			// Pull CS high to end SPI transaction
  WaitForWriteComplete();										// Wait until flash completes write operation (BUSY = 0)
}

/* Reads the status register from SPI flash and sends the result over UART for debugging */
void SPI_ReadStatusAndPrint() {
  uint8_t status;														 // Variable to store the status byte received from the SPI flash
  char uart_buf[50];													 // Buffer to hold the UART message
  int uart_buf_len;														 // Variable to store the length of the UART message

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);				 // Pull CS low to begin SPI transaction
  HAL_SPI_Transmit(&hspi2, (uint8_t *)&RDSR, 1, 100);					 // Send the Read Status Register (RDSR) command (0x05) to the flash
  HAL_SPI_Receive(&hspi2, &status, 1, 100);								 // Receive 1 byte status response from the flash
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);					 // Pull CS high to end the SPI transaction

  uart_buf_len = sprintf(uart_buf, "Final Status: 0x%02X\r\n", status);  // Format the status byte into the UART buffer
  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);    // Transmit the status via UART for debugging/monitoring
}

/* Reads back sensor data from SPI flash memory and sends it to UART */
void SPI_ReadBackAndPrint(int max_interval)
{
    uint8_t read_buf[max_interval * 8];						    // Buffer to hold all read data (8 bytes per entry)
    uint32_t flash_address = 0x000000;  						// Starting address in flash memory

    for (int i = 0; i < max_interval; i++) {
        uint8_t addr[3];
        addr[0] = (flash_address >> 16) & 0xFF;					// Top byte
        addr[1] = (flash_address >> 8) & 0xFF;					// Middle byte
        addr[2] = flash_address & 0xFF;							// Lowest byte

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);	// Begin SPI transaction by pulling CS low
        HAL_SPI_Transmit(&hspi2, (uint8_t *)&Read, 1, 100);		// Send Read (0x03) command
        HAL_SPI_Transmit(&hspi2, addr, 3, 100);					// Send 3-byte flash memory address
        HAL_SPI_Receive(&hspi2, &read_buf[i * 8], 8, 100);		// Read 8 bytes from that address into the buffer
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);	// End SPI transaction by pulling CS high

        flash_address += 8;										// Move to the next 8-byte entry in flash
    }

    ProcessSensorData(read_buf, max_interval);					// Format and display all sensor data entries over UART
}
