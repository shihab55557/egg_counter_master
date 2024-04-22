
#include "stm32g030xx.h"


void     SoftI2C_Struct_Init(void);

void     SoftI2C_GPIO_Init(void);
uint8_t  SoftI2C_SDA_Get_Mode(void);
void     SoftI2C_SDA_Set_Mode(uint8_t state);
void     SoftI2C_SCL_Set_Output_State(uint8_t state);
void     SoftI2C_SDA_Set_Output_State(uint8_t state);
uint8_t  SoftI2C_SDA_Get_State(void);


void     SoftI2C_Bit_Delay(void);
void     SoftI2C_Start(void);
void     SoftI2C_Stop(void);
uint8_t  SoftI2C_Get_Ack(void);
void     SoftI2C_Send_Ack(void);
void     SoftI2C_Send_Nack(void);


void     SoftI2C_Send_Raw_Bit(uint8_t data);
void     SoftI2C_Send_Raw_Byte(uint8_t data);
uint8_t  SoftI2C_Send_Byte(uint8_t data);

uint8_t  SoftI2C_Receive_Raw_Byte(void);
uint8_t  SoftI2C_Read_Register(uint8_t reg_addr);

uint8_t  SoftI2C_Write_Register(uint8_t reg_addr, uint8_t data);





uint8_t  SoftI2C_Get_Slave_Address(void);
void     SoftI2C_Init(void);