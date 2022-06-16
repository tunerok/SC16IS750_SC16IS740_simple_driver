#include "SC16IS750IBS.h"

void SC16_init(const uint32_t baud){
    SC16_set_baudrate(baud);
    SC16_flush_fifo();
    SC16_write_register(SC16IS7XX_FCR,  0x01);   //enable fifo
    SC16_write_register(SC16IS7XX_IER,  0x05);  //enable Rx data ready, lsi interrupt
}

void SC16_set_baudrate(const uint32_t baud){
    uint16_t divisor = 0;
    SC16_write_register(SC16IS7XX_LCR, 0x80);  // magic number to program baud rate 

    if (baud > 0)
    {
    	divisor = (uint16_t)(X_TALL_FREQ/(baud*16));
    }
    else{
    	divisor = 0x000C; //9600 as Default
    }

    if (divisor < 1 || divisor > 2304){
    	divisor = 0x000C; //9600 as Default
    }

    SC16_write_register(SC16IS7XX_DLL, (uint8_t)(divisor & 0xFFu));
    SC16_write_register(SC16IS7XX_DLH, (uint8_t)((divisor >> 8) & 0xFFu));
    SC16_write_register(SC16IS7XX_LCR, 0xBF);   //magic number to unlock ench reg access
    SC16_write_register(SC16IS7XX_EFR, 0X10);
    SC16_write_register(SC16IS7XX_LCR, 0x03); // 8 data bit, no parity (add "| SC16_2STOPBITS" to 2-stopbits support)
}

void SC16_interrupt_callback(void){
    uint8_t data = 0, irq_number = 0, fifo_len = 0;
    irq_number = SC16_get_irq();
    switch (irq_number)
    {
    case SC16IS7XX_LCR_IRQ:
        if (SC16_is_data_aval()){   // data in receiver
            data = SC16_RX();
            get_data(data);
        }
        break;
    case SC16IS7XX_TIMEOUT_IRQ:
    case SC16IS7XX_RHR_IRQ:
    	fifo_len = SC16_read_register(SC16IS7XX_RXLVL);
    	while (fifo_len--){
    		data = SC16_RX();
    		get_data(data);
    	}
        break;
    default:
        break;
    }
}

//check is chip alive
//to check connection we used Scratchpad Register (SPR)
uint8_t SC16_ping(void){
	uint8_t ping_req = 0;
	SC16_write_register(SC16IS7XX_SPR, SC16_CUSTOM_ID); //write custom id to device
    ping_req = SC16_read_register(SC16IS7XX_SPR);
    if (ping_req == SC16_CUSTOM_ID){
        return 1;
    }
    return 0;
}

uint8_t SC16_is_data_aval(void){
    if (SC16_read_register(SC16IS7XX_LSR) && 0x01){
        return 1;
    }
    return 0;
}

uint8_t SC16_get_irq(void){
	uint8_t data = 0;
	data = SC16_read_register(SC16IS7XX_IIR);
	data &= 0x0f;
    return data;
}

void SC16_TX_frame(const uint8_t *data, uint8_t len){
	uint8_t t_tx_space = 0;
	uint8_t t_data[SC16IS7XX_FIFO_SIZE];

	if (len >= SC16IS7XX_FIFO_SIZE)
		return;

	t_tx_space = SC16_read_register(SC16IS7XX_TXLVL);
	if (t_tx_space < len)
		return;

	t_data[0] = SC16IS7XX_THR << 3;
	memcpy(&t_data[1], data, len);              //use something else if you don't want to use libraries
	i2c_write(I2C_EXT_ADDR, t_data, len + 1);

}

void SC16_TX(const uint8_t data){
	uint8_t t_tx_space = 0;
	t_tx_space = SC16_read_register(SC16IS7XX_TXLVL);
	if (t_tx_space > 0)
		SC16_write_register(SC16IS7XX_THR, data);
}

uint8_t SC16_RX(void){
    return SC16_read_register(SC16IS7XX_RHR);
}


uint8_t SC16_write_register(uint8_t address, uint8_t data){
	uint8_t t_data[2] = {address << 3, data};
	if (i2c_write(I2C_EXT_ADDR, t_data, 2)){
		return 1;
	}
	return 0;
}

uint8_t SC16_read_register(uint8_t address){
	uint8_t data[2] = {address << 3, 0};
	if (i2c_write(I2C_EXT_ADDR, &data[0], 1)){
		return 0;
	}
	if (i2c_read(I2C_EXT_ADDR,  &data[1], 1)){
		return 0;
	}
    return data[1];
}

//--------------------------FIFO--------------------------
void SC16_flush_fifo(void){
    SC16_write_register(SC16IS7XX_FCR, 0x02); //flush rx
    SC16_write_register(SC16IS7XX_FCR, 0x04); //flush tx
}
//--------------------------FIFO--------------------------

//--------------------------GPIO--------------------------
//set GPIO [7:0]
void SC16_set_gpio_mode(uint8_t gpio_num, uint8_t mode){
	uint8_t t_data = 0;
	if (gpio_num > 7 || mode > 1){
	        return;
	}
	t_data = SC16_read_register(SC16IS7XX_IODIR);
	t_data ^= mode << gpio_num;
    SC16_write_register(SC16IS7XX_IODIR, t_data);
}

void SC16_set_gpio(uint8_t gpio_num, uint8_t state){
	uint8_t t_data = 0;
	if (gpio_num > 7 || state > 1){
	        return;
	}
	t_data = SC16_read_register(SC16IS7XX_IOSTATE);
	t_data ^= state << gpio_num;
    SC16_write_register(SC16IS7XX_IOSTATE, t_data);

}

uint8_t SC16_get_gpio(uint8_t gpio_num){
    uint8_t gpio_state = 0;
    gpio_state = SC16_read_register(SC16IS7XX_IOSTATE) & (1 << gpio_num);
    return gpio_state;
}
//--------------------------GPIO--------------------------
