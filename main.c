#include "stm32g030xx.h"
#include "serial.h"
#include "adc.h"
#include "i2c.h"
#include "soft_i2c.h"


// Master

void emitter_high(void){
	// rcc enable
	// configure as output 
	// put output high
	RCC->IOPENR |= RCC_IOPENR_GPIOCEN;    //GPIOC clock En
	GPIOC->MODER  &=~ GPIO_MODER_MODE15_1; //make PC15 as GPIO output
	GPIOC->MODER  |= GPIO_MODER_MODE15_0;
	GPIOC-> ODR |= (1<15);
	GPIOC->BSRR |= (1<<15);
	
	//GPIOC-> ODR &=~ (1<15);
	//GPIOC->BSRR &=~ (1<<15);
}





int main(void) {
	  
    uart_Init(38400);    // Initialize UART with desired baud rate
    //adc_init();
	  //emitter_high();
		/*i2c_init();
	  i2c_get_slave_addr();
	  for(uint32_t i=0;i<10000;i++){
					__NOP();
				}
		i2c_read(0x2D);
		i2c_write(0x2D, (1<<3));*/
	
	
	
	  /*Software I2C*/
	  
	  uart_send_char("\r\nDebug Started\r\n");
	  
	  SoftI2C_Init();
	  SoftI2C_Write_Register(0x2D, (1<<3) );
	  
    while (1) {
			
				uint8_t sts;
			  SoftI2C_Start();
		    sts = SoftI2C_Send_Byte(0xA6);
        SoftI2C_Stop();			
			
			  uart_send_num(sts);
			  uart_send_char("\r\n");
			
			  
				for(uint32_t i=0;i<1000000;i++){
					__NOP();
				}
			
    }
}

