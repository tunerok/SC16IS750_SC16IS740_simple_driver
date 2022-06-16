#ifndef SC16IS750IBS
#define SC16IS750IBS

#include <stdint.h>

//used for memcpy
#include <string.h>


#define X_TALL_FREQ 1843200

#define I2C_EXT_ADDR	0x90

#define SC16IS7XX_FCR   0x02	//FIFO Control Register
#define SC16IS7XX_SPR   0x07	//Scratchpad Register
#define SC16IS7XX_IIR   0x02	//Interrupt Identification Register
#define SC16IS7XX_IER   0x01	//Interrupt Enable Register
#define SC16IS7XX_LCR   0x03	//Line Control Register
#define SC16IS7XX_DLL   0x00	//divisor latch LSB
#define SC16IS7XX_DLH   0x01	//divisor latch MSB
#define SC16IS7XX_EFR   0x02	//Extra Features Register
#define SC16IS7XX_LSR   0x05	//Line Status Register
#define SC16IS7XX_RXLVL 0x09	//Receive FIFO Level Register
#define SC16IS7XX_TXLVL 0x08	//Transmit FIFO Level Register

#define SC16IS7XX_RHR   0x00	//Receive Holding Register
#define SC16IS7XX_THR   0x00	//Transmit Holding Register

#define SC16IS7XX_IOSTATE 0x0B	//I/O pin States Register
#define SC16IS7XX_IODIR	  0x0A	//I/O pin Direction Register


/* Misc definitions */
#define SC16IS7XX_FIFO_SIZE		(64)
#define SC16IS7XX_REG_SHIFT		2

#define SC16_CUSTOM_ID          0x32
#define SC16_2STOPBITS          (1 << 2)


#define SC16IS7XX_LCR_IRQ 		0x06
#define SC16IS7XX_TIMEOUT_IRQ	0x0C //time-out interrupt == some data at fifo
#define SC16IS7XX_RHR_IRQ 		0x04 //RHR == some data at fifo

/**
  * @brief Initialization
  * @param baud Transmission speed
  */
void SC16_init(const uint32_t baud);

/**
  * @brief Setting the baud rate
  * @param baud Transmission speed
  */
void SC16_set_baudrate(const uint32_t baud);

/**
  * @brief Callback function to call from an interrupt
  */
void SC16_interrupt_callback(void);

/**
  * @brief Sending an array of data
  * @param *data Pointer to array
  * @param len Array length
  */
void SC16_TX_frame(const uint8_t *data, uint8_t len);

/**
  * @brief Sending one byte
  * @param data Data to send via UART
  */
void SC16_TX(const uint8_t data);

/**
  * @brief Receive one byte
  * @retval RHR register data
  */
uint8_t SC16_RX(void);

/**
  * @brief Checking the availability of a device on the line
  * @retval Returns 1 if there is a connection
  */
uint8_t SC16_ping(void);

/**
  * @brief Checking the presence of data at the reception
  * @retval Returns 1 if there is data
  */
uint8_t SC16_is_data_aval(void);

/**
  * @brief Set chip output mode
  * @param gpio_num Port number [0..7]
  * @param mode Mode (0 = input, 1 - output)
  */
void SC16_set_gpio_mode(uint8_t gpio_num, uint8_t mode);

/**
  * @brief Chip pin setting
  * @param gpio_num Port number [0..7]
  * @param state Output state (0 = gnd, 1 - vcc)
  */
void SC16_set_gpio(uint8_t gpio_num, uint8_t state);

/**
  * @brief Fifo cleaning
  */
void SC16_flush_fifo(void);

/**
  * @brief Get interrupt type
  * @retval Interrupt number
  */
uint8_t SC16_get_irq(void);

/**
  * @brief Write to register
  * @param address Baud rate
  * @param data Data to write
  * @retval Returns 1 if error
  */
uint8_t SC16_write_register(uint8_t address, uint8_t data);

/**
  * @brief Reading from case
  * @param address Register address
  * @retval Register data
  */
uint8_t SC16_read_register(uint8_t address);

#endif
