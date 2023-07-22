#include "stm32f407xx.h"
/*	 Application configurable items	*/

#define LCD_I2C 			 	I2C1
#define LCD_I2C_GPIO_PORT	    GPIOB
#define LCD_I2C_SCL_PIN		    GPIO_PIN_NO_6
#define LCD_I2C_SDA_PIN		    GPIO_PIN_NO_7
#define LCD_I2C_SPEED		    I2C_SCL_SPEED_SM
#define LCD_I2C_PUPD			GPIO_PIN_PU

void lcd_init (void);   // initialize lcd

void lcd_send_cmd (char cmd);  // send command to the lcd

void lcd_send_data (char data);  // send data to the lcd

void lcd_send_string (char *str);  // send string to the lcd

void lcd_put_cur(int row, int col);  // put cursor at the entered position row (0 or 1), col (0-15);

void lcd_clear (void);
