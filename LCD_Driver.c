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
        }							                //	задержка в 10мс
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
        LCD_SELECT;                                                             // выбираем дисплей
		LCD_CMD;																// тип передачи  - данные
		HAL_SPI_Transmit(&hspi1, (uint8_t*) &cmd, 1, 100);
		LCD_DeSELECT;															// отпускаем устройство - LCD
}
//=========================================================================================================//
//=========================================================================================================//
//
void LCD_Write_Data_8(unsigned char data)
	{
        LCD_SELECT;                                                             // выбираем дисплей
		LCD_DATA;																// тип передачи  - данные
        HAL_SPI_Transmit(&hspi1, (uint8_t*) &data, 1, 100);
		LCD_DeSELECT;															// отпускаем устройство - LCD
}
//******************************************************************************//
void LCD_Write_Data_16(unsigned char lByte, unsigned char hByte)
	{
        LCD_SELECT;                                                             // выбираем дисплей
		LCD_DATA;																// тип передачи  - данные
        HAL_SPI_Transmit(&hspi1, (uint8_t*) &lByte, 1, 100);
		HAL_SPI_Transmit(&hspi1, (uint8_t*) &hByte, 1, 100);
		LCD_DeSELECT;															// отпускаем устройство - LCD
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

		Set_lcd_window(X_beg, X_end, Y_beg, Y_end);					// выбираем диапазон вывода на экран
		LCD_Write_Cmd(RAMWR);										// запись в память дисплея
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
		Delay_ms(120);                                          //Delay_ms(120);							        //	задержка 120ms
		LCD_Write_Cmd(SLPOUT);					                //	Sleep out
		Delay_ms(120);                                          //Delay_ms(10);							        //	Задержка 5ms
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
		
		unsigned int Char_index = 0;															// переменная - код символа
		unsigned char Gliph = 0;																// переменная - байт изображения символа 
		unsigned char Gliph_ptr = 0;															// счётчик пикселей для видеобуфера
		unsigned char X_pixel = 0;																// значения ширины символа
		unsigned char Y_pixel = 0;																// значения высоты символа								
		unsigned char Bit_mask = 0x80;															// маска для выбора бита из байта изображения символа
		unsigned char Txt_color_l = Txt_color;									// младший байт цвета текста 
		unsigned char Txt_color_h = Txt_color >> 8;								// старший байт цвета текста
		unsigned char Back_color_l = Back_color;									// младший бат цвта фона
		unsigned char Back_color_h = Back_color >> 8;								// старший байт цвета фона
		unsigned char Y_Y;
		unsigned char X_X = X_PIX_MAX - X_pos;
		unsigned char X = 0;
		unsigned char Y = Y_pos;
		unsigned char Last_Y_pos = Y_pos;

				
		while(*Str!=0x00)
		{
			Char_index = (unsigned char) *Str;
			Char_index -= Font[Font_index].Font_offset;											/////////////////////
			X_pixel = Font[Font_index].Char[Char_index].Image->Gliph_height;					// получение значения высота символа
			Y_pixel = Font[Font_index].Char[Char_index].Image->Gliph_wight;						// получение значения ширина символа
			Y = Last_Y_pos;
			X = (X_X - X_pixel)+1;
			Y_Y = (Y + Y_pixel)-1;
			Last_Y_pos = Y_Y;
			Set_lcd_window(X, X_X, Y, Y_Y);
			LCD_Write_Cmd(RAMWR);																// запись в память дисплея
			while(Y_pixel != 0x00) 
			{
				while(X_pixel != 0x00)
				{																				// если не превыслили границу ширины символа выводим очередной бит
					Gliph = Font[Font_index].Char[Char_index].Image->Gliph[Gliph_ptr];			// выбираем очередной байт изображения символа
					if(Bit_mask != 0x00)
					{																			// если непревысили диапазон байта продолжаем							
						if(Gliph & Bit_mask)
						{	
							LCD_Write_Data_16(Txt_color_h, Txt_color_l);
						}	
						else
						{
							LCD_Write_Data_16(Back_color_h, Back_color_l);
						}
						X_pixel--;																// уменьшаем счётчик ширины символа
						Bit_mask >>= 1;															// маска для следующего бита изображения
					}
					else
					{
						Bit_mask = 0x80;														// устанавливаем маску для очередного байта изображения символа
						Gliph_ptr++;															// увеличиваем указатель - на следующий байт изображения символа
					}	
				}
				Y_pixel--;
				X_pixel = Font[Font_index].Char[Char_index].Image->Gliph_height;				// получение значения высоты символа
			}																	
			Bit_mask = 0x80;
			Gliph_ptr = 0;
			Str++; 																			// увеличиваем указатель для выборки следующего символа из строки
		}	
}
//=========================================================================================================//
//=========================================================================================================//
//
void Put_Val_to_Lcd(unsigned char Y_pos, unsigned char X_pos, unsigned char Font_index, unsigned int Bin_data, unsigned int Txt_color, unsigned int Back_color) 
	{
		unsigned int Char_index = 0;															// переменная - код символа
		unsigned char Gliph = 0;																// переменная - байт изображения символа 
		unsigned char Gliph_ptr = 0;															// счётчик пикселей для видеобуфера
		unsigned char X_pixel = 0;																// значения ширины символа
		unsigned char Y_pixel = 0;																// значения высоты символа
		unsigned int Pixel_ptr = 0;									
		unsigned char Bit_mask = 0x80;															// маска для выбора бита из байта изображения символа
		unsigned char Txt_color_l = Txt_color;									// младший байт цвета текста 
		unsigned char Txt_color_h = Txt_color >> 8;								// старший байт цвета текста
		unsigned char Back_color_l = Back_color;									// младший бат цвта фона
		unsigned char Back_color_h = Back_color >> 8;								// старший байт цвета фона
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
			X_pixel = Font[Font_index].Char[Char_index].Image->Gliph_height;					// получение значения высота символа
			Y_pixel = Font[Font_index].Char[Char_index].Image->Gliph_wight;						// получение значения ширина символа
			Y = Last_Y_pos;
			X = (X_X - X_pixel)+1;
			Y_Y = (Y + Y_pixel)-1;
			Last_Y_pos = Y_Y;
			Set_lcd_window(X, X_X, Y, Y_Y);
			LCD_Write_Cmd(RAMWR);			
			while(Y_pixel != 0x00) 
			{
				while(X_pixel != 0x00)
				{																				// если не превыслили границу ширины символа выводим очередной бит
					Gliph = Font[Font_index].Char[Char_index].Image->Gliph[Gliph_ptr];			// выбираем очередной байт изображения символа
					if(Bit_mask != 0x00)
					{																			// если непревысили диапазон байта продолжаем							
						if(Gliph & Bit_mask)
						{	
							LCD_Write_Data_16(Txt_color_h, Txt_color_l);
						}	
						else
						{
							LCD_Write_Data_16(Back_color_h, Back_color_l);
						}
						X_pixel--;																// уменьшаем счётчик ширины символа
						Bit_mask >>= 1;															// маска для следующего бита изображения
					}
					else
					{
						Bit_mask = 0x80;														// устанавливаем маску для очередного байта изображения символа
						Gliph_ptr++;															// увеличиваем указатель - на следующий байт изображения символа
					}	
				}
				Y_pixel--;
				X_pixel = Font[Font_index].Char[Char_index].Image->Gliph_height;				// получение значения высоты символа
			}
			Bit_mask = 0x80;
			Gliph_ptr = 0;
			Data_index++;																			// увеличиваем указатель для выборки следующего символа из строки
		}	
}	
//=========================================================================================================//
//=========================================================================================================//
//
