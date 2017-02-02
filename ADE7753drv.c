#include        "stm32f0xx_hal.h"
#include        "cmsis_os.h"
#include        "ADE7753drv.h"
#include        "Delay.h"

extern SPI_HandleTypeDef hspi2;


uint16_t ADE7753_Read(uint8_t ADE_reg, uint8_t bytes_to_read)
	{
		uint16_t ADE_ansver = 0;
		uint8_t tmp = 0;
		ADE7753_SELECT;
		
		uint16_t count = 0xff;
		while(count--);

		HAL_SPI_Transmit(&hspi2, (uint8_t*) &ADE_reg, 1, 100);
		for(uint8_t i = 0; i<bytes_to_read; i++)
		{
			tmp = HAL_SPI_Receive(&hspi2, (uint8_t*) &ADE_ansver, 1, 500);
			ADE_ansver = ADE_ansver | tmp;
		}
		ADE7753_deSELECT;
		return ADE_ansver;
}

void ADE7753_Write(uint8_t ADE_reg, uint8_t* buf_to_write, uint8_t bytes_to_write)
	{
		ADE7753_SELECT;
		Delay_ms(1);
		uint8_t ADE_reg_addr = ADE_reg | 0x80;
		HAL_SPI_Transmit(&hspi2, (uint8_t*) &ADE_reg_addr, 1, 100);
		for(uint8_t i = 0; i < bytes_to_write; i++) 
		{
			
		}
	
}