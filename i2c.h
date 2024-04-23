#include "stm32g030xx.h"


typedef struct i2c_t{
  uint8_t Addr;
	uint8_t Error;
}	i2c_t;

i2c_t I2C;

void     i2c_init(void);
uint8_t  i2c_get_error_code(void);
void     i2c_set_slave_addr(uint8_t addr);
void     i2c_set_nbytes(uint8_t val);
uint8_t  i2c_nack_flag(void);
void     i2c_cmd_write(void);
void     i2c_cmd_read(void);
void     i2c_send_nack(void);
void     i2c_send_ack(void);
void     i2c_start(void);
void     i2c_stop(void);
void     i2c_clear_nackf(void);
void     i2c_clear_stopf(void);
void     i2c_get_slave_addr(void);
uint8_t  i2c_read(uint8_t reg_addr);
void     i2c_write(uint8_t reg_addr, uint8_t val);


//gpio configuraiton
//clock conf
//i2c clock
//i2c enable
void i2c_init(void){
	
	
	I2C1->CR1     &=~ I2C_CR1_PE;
	for(uint32_t i=0;i<10000;i++){
					__NOP();
				}
	
	I2C.Addr=0;
	I2C.Error=0;
				
				
	RCC->IOPENR   |= RCC_IOPENR_GPIOBEN;       // Enable GPIOA clock
	RCC->APBENR1  |= RCC_APBENR1_I2C1EN;       // Enable i2c1 clock
	
	//pb6 ->scl
	GPIOB->MODER  |= GPIO_MODER_MODE6_1;
	GPIOB->MODER  &=~ GPIO_MODER_MODE6_0;      //GPIOB_6 as alt. fnc (scl)
	
	//pb9 ->sda
	GPIOB->MODER  |= GPIO_MODER_MODE9_1;
	GPIOB->MODER  &=~ GPIO_MODER_MODE9_0;      //GPIOB_9 as alt. fnc (sda)
	
	GPIOB->OTYPER |= GPIO_OTYPER_OT6;          //GPIOB_6 Open-drain
	GPIOB->OTYPER |= GPIO_OTYPER_OT9;          //GPIOB_9 Open-drain
				
	//select alternative fnc  for PB6
	GPIOB->AFR[0] &=~ GPIO_AFRL_AFSEL6_0;
	GPIOB->AFR[0] |= GPIO_AFRL_AFSEL6_1;
	GPIOB->AFR[0] |= GPIO_AFRL_AFSEL6_2;
	GPIOB->AFR[0] &=~ GPIO_AFRL_AFSEL6_3;
	
	//select alternative funciton for PB9
	GPIOB->AFR[1] &=~ GPIO_AFRH_AFSEL9_0;
	GPIOB->AFR[1] |= GPIO_AFRH_AFSEL9_1;
	GPIOB->AFR[1] |= GPIO_AFRH_AFSEL9_2;
	GPIOB->AFR[1] &=~ GPIO_AFRH_AFSEL9_3;
	
	// configure clock for i2c1
	RCC->CCIPR    |= RCC_CCIPR_I2C1SEL_1;
	RCC->CCIPR    &=~ RCC_CCIPR_I2C1SEL_0;
	
	// i2c1 configuration
	I2C1->CR1     &=~ I2C_CR1_PE;              // i2c disabled
	I2C1->CR1     &=~ I2C_CR1_ANFOFF;          // noise filter enabled
	I2C1->CR1     &=~ I2C_CR1_DNF_Msk;         // digital noise filter disabled
	
	I2C1->TIMINGR &= (uint32_t)(~I2C_TIMINGR_PRESC_Msk);   
	I2C1->TIMINGR |= (uint32_t)(1<<28);
	I2C1->TIMINGR |= (uint32_t)(1<<29);
	I2C1->TIMINGR |= (uint32_t)(1<<30);
	I2C1->TIMINGR |= (uint32_t)(1<<31);                // prescaler is set as 15 (PRESC=15)
	
	
	I2C1->TIMINGR &= (uint32_t)(~I2C_TIMINGR_SCLDEL_Msk);  
	I2C1->TIMINGR &= (uint32_t)(~(1<<20));
	I2C1->TIMINGR |= (uint32_t)(1<<21);
	I2C1->TIMINGR &= (uint32_t)(~(1<<22));
	I2C1->TIMINGR &= (uint32_t)(~(1<<23));                 // SCLDEL=2
	
	I2C1->TIMINGR &= (uint32_t)(~I2C_TIMINGR_SDADEL_Msk);  
	I2C1->TIMINGR &= (uint32_t)(~(1<<16));
	I2C1->TIMINGR |= (uint32_t)(1<<17);
	I2C1->TIMINGR &= (uint32_t)(~(1<<18));
	I2C1->TIMINGR &= (uint32_t)(~(1<<19));                 // SDADEL =2

	I2C1->TIMINGR &=~ I2C_TIMINGR_SCLH_Msk;    
	I2C1->TIMINGR |= (uint32_t)(1<<8);
	I2C1->TIMINGR |= (uint32_t)(1<<9);
	I2C1->TIMINGR |= (uint32_t)(1<<10);
	I2C1->TIMINGR &= (uint32_t)(~(1<<11));
	I2C1->TIMINGR &=~ (1<<12);
	I2C1->TIMINGR &=~ (1<<13);
	I2C1->TIMINGR &=~ (1<<14);
	I2C1->TIMINGR &=~ (1<<15);                  // SCLH is set as 7
	
	I2C1->TIMINGR &=~ I2C_TIMINGR_SCLL_Msk;    
	I2C1->TIMINGR |= (uint32_t)(1<<0);
	I2C1->TIMINGR |= (uint32_t)(1<<1);
	I2C1->TIMINGR |= (uint32_t)(1<<2);
	I2C1->TIMINGR &=~ (1<<3);
	I2C1->TIMINGR &=~ (1<<4);
	I2C1->TIMINGR &=~ (1<<5);
	I2C1->TIMINGR &=~ (1<<6);
	I2C1->TIMINGR &=~ (1<<7);                   // SCLL is set as 7
	
	
	I2C1->CR1     &=~ I2C_CR1_NOSTRETCH;       // kept cleared in master mode
	
	I2C1->CR1     |= I2C_CR1_PE;               // i2c peripheral enable
	//I2C1->CR1     |= I2C_CR1_TXIE;             // i2c transmission int. enable
	//I2C1->CR1     |= I2C_CR1_RXIE;             // i2c receiving int. enable
	I2C1->TXDR    &=  0x00;
}


uint8_t i2c_get_error_code(void){
	return I2C.Error;
}
	
	
void i2c_set_slave_addr(uint8_t addr){
	I2C.Addr=addr;
}

void i2c_set_nbytes(uint8_t val){
	I2C1->CR2 &=~ I2C_CR2_NBYTES_Msk;
	I2C1->CR2 |=  (val<<I2C_CR2_NBYTES_Pos);
}

uint8_t i2c_nack_flag(void){
	if(I2C1->ISR & I2C_ISR_NACKF){
		return 1;
	}else{
		return 0;
	}
}

void i2c_cmd_write(void){
	I2C1->CR2 &=~ I2C_CR2_RD_WRN;
}

void i2c_cmd_read(void){
	I2C1->CR2 |= I2C_CR2_RD_WRN;
}

void i2c_send_nack(void){
	I2C1->CR2 |= I2C_CR2_NACK;
}

void i2c_send_ack(void){
	I2C1->CR2 &=~I2C_CR2_NACK;
}

void i2c_start(void){
	I2C1->CR2 &=~ I2C_CR2_SADD_Msk;
	I2C1->CR2 |= (I2C.Addr<<1);
	I2C1->CR2 |= I2C_CR2_START;
	while((I2C1->CR2 & I2C_CR2_START)==I2C_CR2_START);
}

void i2c_stop(void){
	I2C1->CR2 |= I2C_CR2_STOP;                         //Stop request
	while((I2C1->CR2 & I2C_CR2_STOP)==I2C_CR2_STOP);   //Wait until Stop is sent
	while((I2C1->ISR & I2C_ISR_STOPF)==0);             //Wait for Stop flag is set
	I2C1->ICR |= I2C_ICR_STOPCF;                       //Request for Stop flag clear
	while((I2C1->ISR & I2C_ISR_STOPF)==I2C_ISR_STOPF); //Wait until stop flag is cleared
}

void i2c_clear_nackf(void){
	if((I2C1->ISR & I2C_ISR_NACKF)==I2C_ISR_NACKF){
		I2C1->ICR |= I2C_ICR_NACKCF;
	  while((I2C1->ISR & I2C_ISR_NACKF)==I2C_ISR_NACKF);
	}
}

void i2c_clear_stopf(void){
	if((I2C1->ISR & I2C_ISR_STOPF)==I2C_ISR_STOPF){
		I2C1->ICR |= I2C_ICR_STOPCF;
	  while((I2C1->ISR & I2C_ISR_STOPF)==I2C_ISR_STOPF);
	}
}

void i2c_get_slave_addr(void){
	for(uint8_t i=1;i<255;i++){
		i2c_set_slave_addr(i);
		i2c_start();
		if(i2c_nack_flag()){  //NACKed
			i2c_clear_nackf();
		}
		else{                 //ACKed
			I2C.Addr = i;
			i2c_stop();
			break;
		}
	}
}


uint8_t i2c_read(uint8_t reg_addr){
	uint8_t rx_data=0;
	i2c_set_nbytes(1);                            //1 byte to be transmitted
	i2c_cmd_write();                              //Write cmd
	I2C1->TXDR=reg_addr;
	i2c_start();                                  //SlaveAddr+Write
	while((I2C1->ISR & I2C_ISR_TXE)==0);
	
	if(i2c_nack_flag()==1){                       //NACK received
		I2C.Error=0x01;                             //Generate Error Code
	}else{                                        //ACK received
		i2c_stop();                                 //Stop
	  i2c_set_nbytes(1);                          //1 byte to be transmitted
	  i2c_cmd_read();                             //Write cmd
	  i2c_start();                                //Start+SLAVE_ADD
	  while((I2C1->ISR & I2C_ISR_RXNE)==1);       //Wait until RXDR is full
	  rx_data=(uint8_t)I2C1->RXDR;
	  i2c_stop();
		I2C.Error=0x00;
	}
	
	return rx_data;
}

void i2c_write(uint8_t reg_addr, uint8_t val){
	uint8_t rx_data=0;
	i2c_set_nbytes(2);                            //1 byte to be transmitted
	i2c_cmd_write();                              //Write cmd
	I2C1->TXDR=reg_addr;
	i2c_start();                                  //SlaveAddr+Write
	while((I2C1->ISR & I2C_ISR_TXE)==0);
	I2C1->TXDR=val;
	while((I2C1->ISR & I2C_ISR_TXE)==0);
	i2c_stop(); 
}

