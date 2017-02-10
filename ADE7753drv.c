#include        "stm32f0xx_hal.h"
#include        "cmsis_os.h"
#include        "ADE7753drv.h"
#include        "Delay.h"

extern SPI_HandleTypeDef hspi2;
uint16_t adeMode;
void ADE7753Reset(void)
{
	adeMode = 0;
	uint8_t cmd[2];
	uint8_t count = 10;
	while(count-- && adeMode == 0)
	{
		adeMode = ADE7753Read(adeMODE, 2);
	}
	adeMode |= (1 << modeTEMPSEL);
	cmd[0] = (uint8_t) adeMode;
	cmd[1] = (uint8_t) adeMode>>8;
	ADE7753Write(adeMODE, cmd, 2);
	DelayMs(1);
}

uint32_t ADE7753Read(uint8_t ADE_reg, uint8_t bytes_to_read)
	{
		uint32_t ADE_ansver = 0;
		uint8_t tmp = 0;
		ADE7753_SELECT;
		
		uint16_t count = 0xff;
		while(count--);

		HAL_SPI_Transmit(&hspi2, (uint8_t*) &ADE_reg, 1, 100);
		for(uint8_t i = 1; i <= bytes_to_read; i++)
		{
			HAL_SPI_Receive(&hspi2, (uint8_t*) &tmp, 1, 500);
			ADE_ansver = ADE_ansver | tmp;
			if(i < bytes_to_read)
			{
				ADE_ansver = ADE_ansver<<8;
			}
			//tmp =0;
		}
		ADE7753_deSELECT;
		return ADE_ansver;
}

void ADE7753Write(uint8_t ADE_reg, uint8_t* buf, uint8_t bufLength)
	{
		ADE7753_SELECT;
		uint16_t count = 0xff;
		while(count--);
		uint8_t ADE_reg_addr = ADE_reg | 0x80;
		HAL_SPI_Transmit(&hspi2, (uint8_t*) &ADE_reg_addr, 1, 100);
		HAL_SPI_Transmit(&hspi2, buf, 2, 100);
		ADE7753_deSELECT;
		//for(uint8_t i = 0; i < bytes_to_write; i++) 
		//{
			
		//}
	
}
	
void ADE7753Init(void)
{
	ADE7753Reset();
	//ADE7753_Write(MODE, 
}

uint16_t Get_VPEAK(void) 
{
	uint32_t res = 0;
	res = ADE7753Read(adeRSTVPEAK , 3);
	res *= kofCH2DIV;
	res /= kofVPEAK;
	return (uint16_t) res>>1;
}