# SC16IS740_simple_driver

The most simple driver for the SC16IS7XX chip from NXP.

Usage:

The SC16_interrupt_callback(void) function must be called in a loop or in an SC16IS7XX pin interrupt. 

The functions i2c_write(uint8_t addr, const uint8_t *data, int len), i2c_read(uint8_t addr, uint8_t *data, int len) and get_data(uint8_t data) **must** be defined for your target platform.
Example for STM32 and HAL:
```
#define ARR_SIZE 8
static uint8_t my_best_array[ARR_SIZE] = {0};
static uint8_t array_pointer = 0;

HAL_StatusTypeDef i2c_write_ext(uint8_t addr, const uint8_t *data, int len)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_Master_Transmit(EXT_I2C, addr, (uint8_t*)data, len, I2CTimeout);
    return status;
}

void get_data(uint8_t data){
    my_best_array[array_pointer] = data;
    array_pointer++;
}
```

For your device, you **must** change the oscillator frequency of the SC16IS7XX **X_TALL_FREQ** chip and the chip address **I2C_EXT_ADDR**.

When using I2C speed at 100kHz and long bursts via UART at 115200, the internal FIFO of the microcircuit will quickly overflow. Short parcels may well be transmitted.
