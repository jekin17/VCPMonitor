#include        "stm32f0xx_hal.h"
#include        "cmsis_os.h"
#include        "LCD_Driver.h"
#include        "LCD_Fonts.h"
#include        "Delay.h"


extern SPI_HandleTypeDef hspi1;

unsigned int Count;
unsigned char Bcd_data[5];

//=========================================================================================================//
//=========================================================================================================//
//
void LCD_Reset(void) {
unsigned int Count;     
	LCD_DeSELECT;						//	Deselect LCD
	
    Count = 5000;
	while(Count) {
    Count--;
        }							                //	�������� � 10��
	LCD_SELECT;						//	Select LCD
	LCD_RESET;						//	HW RESET LCD
	Count = 15000;
	while(Count)
		{
			Count--;
        }							                //	Hold RESET in low 10ms
	LCD_RESET_OFF;						// 	Relased RESET
	Count = 5000;
	while(Count)
		{     
			Count--;
        }
}
//=========================================================================================================//
//=========================================================================================================//
//
void LCD_Write_Cmd(unsigned char cmd) 
	{
        LCD_SELECT;                                                             // �������� �������
		LCD_CMD;																// ��� ��������  - ������
		HAL_SPI_Transmit(&hspi1, (uint8_t*) &cmd, 1, 100);
		LCD_DeSELECT;															// ��������� ���������� - LCD
}
//=========================================================================================================//
//=========================================================================================================//
//
void LCD_Write_Data_8(unsigned char data)
	{
        LCD_SELECT;                                                             // �������� �������
		LCD_DATA;																// ��� ��������  - ������
        HAL_SPI_Transmit(&hspi1, (uint8_t*) &data, 1, 100);
		LCD_DeSELECT;															// ��������� ���������� - LCD
}
//******************************************************************************//
void LCD_Write_Data_16(unsigned char lByte, unsigned char hByte)
	{
        LCD_SELECT;                                                             // �������� �������
		LCD_DATA;																// ��� ��������  - ������
        HAL_SPI_Transmit(&hspi1, (uint8_t*) &lByte, 1, 100);
		HAL_SPI_Transmit(&hspi1, (uint8_t*) &hByte, 1, 100);
		LCD_DeSELECT;															// ��������� ���������� - LCD
}
//=========================================================================================================//
//=========================================================================================================//
//
void Set_lcd_window(unsigned char X_beg, unsigned char X_end, unsigned char Y_str, unsigned char Y_end) {
	LCD_Write_Cmd(CASET);
	LCD_Write_Data_16(0, X_beg);
	LCD_Write_Data_16(0, X_end);
	LCD_Write_Cmd(RASET);
	LCD_Write_Data_16(0, Y_str);
	LCD_Write_Data_16(0, Y_end);
}
//******************************************************************************//
void Clr_Window_Set_Fon(unsigned char X_beg, unsigned char X_end, unsigned char Y_beg, unsigned char Y_end, unsigned int Fon_color)
	{
		unsigned char W = X_end - X_beg + 1;
		unsigned char H = Y_end - Y_beg + 1;
		unsigned char Back_color_l = Fon_color;
		unsigned char Back_color_h = Fon_color>>8;

		Set_lcd_window(X_beg, X_end, Y_beg, Y_end);					// �������� �������� ������ �� �����
		LCD_Write_Cmd(RAMWR);										// ������ � ������ �������
		for(H=0; H < Y_RESOLUTION; H++) {
			for(W=0; W < X_RESOLUTION; W++) {
				LCD_Write_Data_16(Back_color_h, Back_color_l);
			}
		}
}
//=========================================================================================================//
//=========================================================================================================//
//******************************************************************************//
void LCD_Initial(void)
	{
		LCD_Reset();								
		LCD_Write_Cmd(SWRESET);					                //	Software reset
		Delay_ms(120);                                          //Delay_ms(120);							        //	�������� 120ms
		LCD_Write_Cmd(SLPOUT);					                //	Sleep out
		Delay_ms(120);                                          //Delay_ms(10);							        //	�������� 5ms
		LCD_Write_Cmd(INVOFF);					                //	INVOFF
		LCD_Write_Cmd(IDMOFF);					                //	
		LCD_Write_Cmd(NORON);					                //	
		LCD_Write_Cmd(DISPON);
		LCD_Write_Cmd(COLMOD);
		LCD_Write_Data_8(0x05);					                // 	MCU interface color format - 16bit / RGB - 16bit
		LCD_Write_Cmd(MADCTRL);
		LCD_Write_Data_8(0x10);
		Clr_Window_Set_Fon(0, X_PIX_MAX, 0, Y_PIX_MAX, WHITE);
}
//=========================================================================================================//
//=========================================================================================================//
//
void Bin_To_Bcd(unsigned int Bin_data)
	{
		Bcd_data[0] = ' ';
		Bcd_data[1] = ' ';
		Bcd_data[2] = ' ';
		Bcd_data[3] = ' ';
		Bcd_data[4] = '0';
		if(Bin_data >= 10000) {
			Bcd_data[0]='0';
			Bcd_data[1]='0';
			Bcd_data[2]='0';
			Bcd_data[3]='0';
			Bcd_data[4]='0';
		}	
		while (Bin_data >= 10000) {
			Bin_data -= 10000;
			Bcd_data[0]++;
		}
		if(Bin_data >= 1000) {
			Bcd_data[1]='0';
			Bcd_data[2]='0';
			Bcd_data[3]='0';
			Bcd_data[4]='0';
		}
		while(Bin_data >= 1000) {
			Bin_data -= 1000;
			Bcd_data[1]++;
		}
		if(Bin_data >= 100) {
			Bcd_data[2]='0';
			Bcd_data[3]='0';
			Bcd_data[4]='0';
		}
		while(Bin_data >= 100) {
			Bin_data -= 100;
			Bcd_data[2]++;
		}
		if(Bin_data >= 10) {
			Bcd_data[3]='0';
			Bcd_data[4]='0';
		}
		while(Bin_data >= 10) {
			Bin_data -= 10;
			Bcd_data[3]++;
		}
		Bcd_data[4] += (unsigned char)Bin_data;
}
//=========================================================================================================//
//=========================================================================================================//
//
void Put_Str_to_Lcd(unsigned char Y_pos, unsigned char X_pos, unsigned char Font_index, const char *Str, unsigned int Txt_color, unsigned int Back_color)
	{	
		
		unsigned int Char_index = 0;															// ���������� - ��� �������
		unsigned char Gliph = 0;																// ���������� - ���� ����������� ������� 
		unsigned char Gliph_ptr = 0;															// ������� �������� ��� �����������
		unsigned char X_pixel = 0;																// �������� ������ �������
		unsigned char Y_pixel = 0;																// �������� ������ �������								
		unsigned char Bit_mask = 0x80;															// ����� ��� ������ ���� �� ����� ����������� �������
		unsigned char Txt_color_l = Txt_color;									// ������� ���� ����� ������ 
		unsigned char Txt_color_h = Txt_color >> 8;								// ������� ���� ����� ������
		unsigned char Back_color_l = Back_color;									// ������� ��� ���� ����
		unsigned char Back_color_h = Back_color >> 8;								// ������� ���� ����� ����
		unsigned char Y_Y;
		unsigned char X_X = X_PIX_MAX - X_pos;
		unsigned char X = 0;
		unsigned char Y = Y_pos;
		unsigned char Last_Y_pos = Y_pos;

				
		while(*Str!=0x00)
		{
			Char_index = (unsigned char) *Str;
			Char_index -= Font[Font_index].Font_offset;											/////////////////////
			X_pixel = Font[Font_index].Char[Char_index].Image->Gliph_height;					// ��������� �������� ������ �������
			Y_pixel = Font[Font_index].Char[Char_index].Image->Gliph_wight;						// ��������� �������� ������ �������
			Y = Last_Y_pos;
			X = (X_X - X_pixel)+1;
			Y_Y = (Y + Y_pixel)-1;
			Last_Y_pos = Y_Y;
			Set_lcd_window(X, X_X, Y, Y_Y);
			LCD_Write_Cmd(RAMWR);																// ������ � ������ �������
			while(Y_pixel != 0x00) 
			{
				while(X_pixel != 0x00)
				{																				// ���� �� ���������� ������� ������ ������� ������� ��������� ���
					Gliph = Font[Font_index].Char[Char_index].Image->Gliph[Gliph_ptr];			// �������� ��������� ���� ����������� �������
					if(Bit_mask != 0x00)
					{																			// ���� ����������� �������� ����� ����������							
						if(Gliph & Bit_mask)
						{	
							LCD_Write_Data_16(Txt_color_h, Txt_color_l);
						}	
						else
						{
							LCD_Write_Data_16(Back_color_h, Back_color_l);
						}
						X_pixel--;																// ��������� ������� ������ �������
						Bit_mask >>= 1;															// ����� ��� ���������� ���� �����������
					}
					else
					{
						Bit_mask = 0x80;														// ������������� ����� ��� ���������� ����� ����������� �������
						Gliph_ptr++;															// ����������� ��������� - �� ��������� ���� ����������� �������
					}	
				}
				Y_pixel--;
				X_pixel = Font[Font_index].Char[Char_index].Image->Gliph_height;				// ��������� �������� ������ �������
			}																	
			Bit_mask = 0x80;
			Gliph_ptr = 0;
			Str++; 																			// ����������� ��������� ��� ������� ���������� ������� �� ������
		}	
}
//=========================================================================================================//
//=========================================================================================================//
//
void Put_Val_to_Lcd(unsigned char Y_pos, unsigned char X_pos, unsigned char Font_index, unsigned int Bin_data, unsigned int Txt_color, unsigned int Back_color) 
	{
		unsigned int Char_index = 0;															// ���������� - ��� �������
		unsigned char Gliph = 0;																// ���������� - ���� ����������� ������� 
		unsigned char Gliph_ptr = 0;															// ������� �������� ��� �����������
		unsigned char X_pixel = 0;																// �������� ������ �������
		unsigned char Y_pixel = 0;																// �������� ������ �������
		unsigned int Pixel_ptr = 0;									
		unsigned char Bit_mask = 0x80;															// ����� ��� ������ ���� �� ����� ����������� �������
		unsigned char Txt_color_l = Txt_color;									// ������� ���� ����� ������ 
		unsigned char Txt_color_h = Txt_color >> 8;								// ������� ���� ����� ������
		unsigned char Back_color_l = Back_color;									// ������� ��� ���� ����
		unsigned char Back_color_h = Back_color >> 8;								// ������� ���� ����� ����
		unsigned char Y_Y;
		unsigned char X_X = X_PIX_MAX - X_pos;
		unsigned char X = 0;
		unsigned char Y = Y_pos;
		unsigned char Last_Y_pos = Y_pos;
		unsigned char 	Data_index;
				
		Bin_To_Bcd(Bin_data);
		Data_index = 0;
		while(Data_index <= 4) 
		{
			Char_index = (unsigned char) Bcd_data[Data_index];
			Char_index -= Font[Font_index].Font_offset;											/////////////////////
			X_pixel = Font[Font_index].Char[Char_index].Image->Gliph_height;					// ��������� �������� ������ �������
			Y_pixel = Font[Font_index].Char[Char_index].Image->Gliph_wight;						// ��������� �������� ������ �������
			Y = Last_Y_pos;
			X = (X_X - X_pixel)+1;
			Y_Y = (Y + Y_pixel)-1;
			Last_Y_pos = Y_Y;
			Set_lcd_window(X, X_X, Y, Y_Y);
			LCD_Write_Cmd(RAMWR);			
			while(Y_pixel != 0x00) 
			{
				while(X_pixel != 0x00)
				{																				// ���� �� ���������� ������� ������ ������� ������� ��������� ���
					Gliph = Font[Font_index].Char[Char_index].Image->Gliph[Gliph_ptr];			// �������� ��������� ���� ����������� �������
					if(Bit_mask != 0x00)
					{																			// ���� ����������� �������� ����� ����������							
						if(Gliph & Bit_mask)
						{	
							LCD_Write_Data_16(Txt_color_h, Txt_color_l);
						}	
						else
						{
							LCD_Write_Data_16(Back_color_h, Back_color_l);
						}
						X_pixel--;																// ��������� ������� ������ �������
						Bit_mask >>= 1;															// ����� ��� ���������� ���� �����������
					}
					else
					{
						Bit_mask = 0x80;														// ������������� ����� ��� ���������� ����� ����������� �������
						Gliph_ptr++;															// ����������� ��������� - �� ��������� ���� ����������� �������
					}	
				}
				Y_pixel--;
				X_pixel = Font[Font_index].Char[Char_index].Image->Gliph_height;				// ��������� �������� ������ �������
			}
			Bit_mask = 0x80;
			Gliph_ptr = 0;
			Data_index++;																			// ����������� ��������� ��� ������� ���������� ������� �� ������
		}	
}	
//=========================================================================================================//
//=========================================================================================================//
//
