

#include "stm32g030xx.h"
#include "soft_i2c.h"
#include "serial.h"


typedef struct soft_i2c_t{
	uint8_t SDAState;
	uint8_t SlaveAddress;
	uint8_t ReadAddress;
	uint8_t WriteAddress;
	uint8_t Error;
}soft_i2c_t;

soft_i2c_t SoftI2C;



/*SoftI2c_Basic Functions*/


void SoftI2C_Struct_Init(void){
	SoftI2C.SDAState = 0;
	SoftI2C.SlaveAddress = 0;
	SoftI2C.ReadAddress = 0;
	SoftI2C.WriteAddress = 0;
	SoftI2C.Error = 0;
}

void SoftI2C_GPIO_Init(void){
  RCC->IOPENR   |= RCC_IOPENR_GPIOBEN;       // Enable GPIOA clock
	
	GPIOB->MODER  &=~GPIO_MODER_MODE6_Msk;     // Clear MODER 
	GPIOB->MODER  |= GPIO_MODER_MODE6_0;       // GP Output -> SCL
	GPIOB->OTYPER |= GPIO_OTYPER_OT6;          // Open drain
	GPIOB->ODR    |= GPIO_ODR_OD6;
	
	GPIOB->MODER  &=~GPIO_MODER_MODE9_Msk;     // Clear MODER
	GPIOB->MODER  |= GPIO_MODER_MODE9_0;       // GP Output -> SDA
	GPIOB->OTYPER |= GPIO_OTYPER_OT9;          // Open drain
	GPIOB->ODR    |= GPIO_ODR_OD9;
	SoftI2C_SDA_Set_Mode(1);                   // SDA Output
}


uint8_t SoftI2C_SDA_Get_Mode(void){
	return SoftI2C.SDAState;
}

void SoftI2C_SDA_Set_Mode(uint8_t state){
	if(state){
	  GPIOB->MODER  |= GPIO_MODER_MODE9_0;       // GP Output -> SDA
		SoftI2C.SDAState = 1;
	}else{
	  GPIOB->MODER  &=~GPIO_MODER_MODE9_0;       // GP Output -> SDA
		SoftI2C.SDAState = 0;
	}
}

void SoftI2C_SCL_Set_Output_State(uint8_t state){
	if(state){
		GPIOB->ODR |= GPIO_ODR_OD6;
	}else{
		GPIOB->ODR &=~GPIO_ODR_OD6;
	}
}

void SoftI2C_SDA_Set_Output_State(uint8_t state){
	if(state){
		GPIOB->ODR |= GPIO_ODR_OD9;
	}else{
		GPIOB->ODR &=~GPIO_ODR_OD9;
	}
}

uint8_t SoftI2C_SDA_Get_State(void){
	if(GPIOB->IDR & (1<<9)){
	  return 1;
	}else{
	  return 0;
	}
}


void SoftI2C_Bit_Delay(void){
	for(uint32_t i=0;i<20;i++){
		 __NOP();
	}
}


void SoftI2C_Start(void){
	
	if(SoftI2C_SDA_Get_Mode()==0){
		SoftI2C_SDA_Set_Mode(1);
	}
	
	SoftI2C_SCL_Set_Output_State(1);
	SoftI2C_Bit_Delay();
	SoftI2C_SDA_Set_Output_State(1);
	SoftI2C_Bit_Delay();
	SoftI2C_SDA_Set_Output_State(0);
	SoftI2C_Bit_Delay();
	SoftI2C_SCL_Set_Output_State(0);
	SoftI2C_Bit_Delay();
}

void SoftI2C_Stop(void){
	
	if(SoftI2C_SDA_Get_Mode()==0){
		SoftI2C_SDA_Set_Mode(1);
	}
	
	SoftI2C_SDA_Set_Output_State(0);
	SoftI2C_Bit_Delay();
	SoftI2C_SCL_Set_Output_State(0);
	SoftI2C_Bit_Delay();
	SoftI2C_SCL_Set_Output_State(1);
	SoftI2C_Bit_Delay();
	SoftI2C_SDA_Set_Output_State(1);
	SoftI2C_Bit_Delay();
}

uint8_t SoftI2C_Get_Ack(void){
	
	if(SoftI2C_SDA_Get_Mode()==1){
		SoftI2C_SDA_Set_Mode(0);
	}
	
  uint8_t sts=0;
	SoftI2C_Bit_Delay();
	SoftI2C_SCL_Set_Output_State(1);
	sts = SoftI2C_SDA_Get_State();
	SoftI2C_Bit_Delay();
	SoftI2C_SCL_Set_Output_State(0);
	SoftI2C_Bit_Delay();
	return sts;
}


void SoftI2C_Send_Ack(void){
	
	if(SoftI2C_SDA_Get_Mode()==0){
		SoftI2C_SDA_Set_Mode(1);
	}
	
	SoftI2C_Bit_Delay();
	SoftI2C_SDA_Set_Output_State(0);
	SoftI2C_Bit_Delay();
	SoftI2C_SCL_Set_Output_State(1);
	SoftI2C_Bit_Delay();
	SoftI2C_SCL_Set_Output_State(0);
	SoftI2C_Bit_Delay();
	SoftI2C_SDA_Set_Output_State(0);
	SoftI2C_Bit_Delay();
}

void SoftI2C_Send_Nack(void){
	
	if(SoftI2C_SDA_Get_Mode()==0){
		SoftI2C_SDA_Set_Mode(1);
	}
	
	SoftI2C_Bit_Delay();
	SoftI2C_SDA_Set_Output_State(1);
	SoftI2C_Bit_Delay();
	SoftI2C_SCL_Set_Output_State(1);
	SoftI2C_Bit_Delay();
	SoftI2C_SCL_Set_Output_State(0);
	SoftI2C_Bit_Delay();
	SoftI2C_SDA_Set_Output_State(0);
	SoftI2C_Bit_Delay();
}



void SoftI2C_Send_Raw_Bit(uint8_t data){
	
	if(SoftI2C_SDA_Get_Mode()==0){
		SoftI2C_SDA_Set_Mode(1);
	}
	
  if(data){
		SoftI2C_SDA_Set_Output_State(1);
		SoftI2C_Bit_Delay();
	  SoftI2C_SCL_Set_Output_State(1);
		SoftI2C_Bit_Delay();
		SoftI2C_SCL_Set_Output_State(0);
		SoftI2C_Bit_Delay();
	  SoftI2C_SDA_Set_Output_State(0);
		SoftI2C_Bit_Delay();
	}else{
		SoftI2C_SDA_Set_Output_State(0);
		SoftI2C_Bit_Delay();
	  SoftI2C_SCL_Set_Output_State(1);
		SoftI2C_Bit_Delay();
		SoftI2C_SCL_Set_Output_State(0);
		SoftI2C_Bit_Delay();
	  SoftI2C_SDA_Set_Output_State(0);
		SoftI2C_Bit_Delay();
	}
}
		
void SoftI2C_Send_Raw_Byte(uint8_t data){
	
	if(SoftI2C_SDA_Get_Mode()==0){
		SoftI2C_SDA_Set_Mode(1);
	}
	
	for(int i=7;i>=0;i--){
		if(data & (1<<i)){
			SoftI2C_Send_Raw_Bit(1);
		}else{
			SoftI2C_Send_Raw_Bit(0);
		}
	} 
}


uint8_t SoftI2C_Send_Byte(uint8_t data){
	uint8_t sts=0;
	SoftI2C_Send_Raw_Byte(data);
	sts = SoftI2C_Get_Ack();
	return sts;
}



uint8_t SoftI2C_Receive_Raw_Byte(void){
	
	if(SoftI2C_SDA_Get_Mode()==1){
		SoftI2C_SDA_Set_Mode(0);
	}
	
	uint8_t sts=0;
	for(int i=0;i<8;i++){   //read bits
		SoftI2C_Bit_Delay();
	  SoftI2C_SCL_Set_Output_State(1);
		sts<<=1;
		sts |= SoftI2C_SDA_Get_State();
		SoftI2C_Bit_Delay();
		SoftI2C_SCL_Set_Output_State(0);
		SoftI2C_Bit_Delay();
		SoftI2C_Bit_Delay();
	}
	return sts;
}

uint8_t SoftI2C_Read_Register(uint8_t reg_addr){
	SoftI2C_Start();
	SoftI2C_Send_Byte(SoftI2C.WriteAddress);
	SoftI2C_Send_Byte(reg_addr);
	SoftI2C_Stop();
	SoftI2C_Start();
	SoftI2C_Send_Byte(SoftI2C.ReadAddress);
	uint8_t sts = SoftI2C_Receive_Raw_Byte();
	SoftI2C_Send_Nack();
	SoftI2C_Stop();
	return sts;
}
	

uint8_t SoftI2C_Write_Register(uint8_t reg_addr, uint8_t data){
	SoftI2C_Start();
	SoftI2C_Send_Byte(SoftI2C.WriteAddress);
	SoftI2C_Send_Byte(reg_addr);
	SoftI2C_Send_Byte(data);
	SoftI2C_Stop();
	return 0;         //optimize
}








uint8_t SoftI2C_Get_Slave_Address(void){
	uint8_t sts;
	for(int i=1;i<255;i++){   //read bits
		SoftI2C_Start();
		sts = SoftI2C_Send_Byte(i);
		if(sts==0){
			SoftI2C.SlaveAddress = (i>>1);
			SoftI2C.WriteAddress = (SoftI2C.SlaveAddress<<1) | 0;
			SoftI2C.ReadAddress  = (SoftI2C.SlaveAddress<<1) | 1;
			break;
		}
	}
	return SoftI2C.SlaveAddress;
}



void SoftI2C_Init(void){
  SoftI2C_Struct_Init();
	SoftI2C_GPIO_Init();
	SoftI2C_Get_Slave_Address();
}
	
	