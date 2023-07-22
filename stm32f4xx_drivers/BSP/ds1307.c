/*
 * ds1307.c
 *
 *  Created on: Jun 4, 2023
 *      Author: Mahdi Kaffel
 */
#include "ds1307.h"

static void ds1307_i2c_pin_config(void);
static void ds1307_i2c_config(void);
static void ds1307_write(uint8_t value,uint8_t reg_addr);
static uint8_t ds1307_read(uint8_t reg_addr);
static uint8_t binary_to_bcd(uint8_t value);
static uint8_t bcd_to_binary(uint8_t value);


I2C_Handle_t DS1307_I2CHandle;

uint8_t ds1307_init(void)
{
	// Init the I2C Pins
	ds1307_i2c_pin_config();
	// Initialize the I2C peripheral
	ds1307_i2c_config();
	// Enable I2C
	I2C_PeripheralControl(DS1307_I2C, ENABLE);
	// Make clock halt
	ds1307_write(0x00,DS1307_ADDR_SEC);
	// Read back clock halt bit
	uint8_t clock_state = ds1307_read(DS1307_ADDR_SEC);
	// return 1 : CH = 1 ; init failed
	// return 0 : CH = 0 ; init success
	return ((clock_state >> 7) & 0x1);

}

void ds1307_set_current_time(RTC_time_t *rtc_time)
{
	uint8_t seconds , hrs ;
	// Sec
	seconds = binary_to_bcd(rtc_time->seconds);
	seconds &= ~( 1 << 7 );
	ds1307_write(seconds, DS1307_ADDR_SEC);
	// Min
	ds1307_write(binary_to_bcd(rtc_time->minutes), DS1307_ADDR_MIN);
	// HRS
	hrs = binary_to_bcd(rtc_time->hours);
	ds1307_write(binary_to_bcd(rtc_time->hours), DS1307_ADDR_HRS);
	if(rtc_time->time_format == TIME_FORMAT_24HRS)
	{
		hrs &= ~( 1 << 6 );
	}else{
		hrs |= ( 1 << 6 );
		hrs = (rtc_time->time_format == TIME_FORMAT_12HRS_PM) ? hrs | (1 << 5) : hrs & ~(1 << 5);
	}
	ds1307_write(hrs, DS1307_ADDR_HRS);

}

void ds1307_get_current_time(RTC_time_t *rtc_time)
{
	uint8_t seconds , hrs;
	seconds = ds1307_read(DS1307_ADDR_SEC);
	seconds &= ~(1 << 7);
	rtc_time->seconds = bcd_to_binary(seconds);
	rtc_time->minutes = bcd_to_binary(ds1307_read(DS1307_ADDR_MIN));
	hrs = ds1307_read(DS1307_ADDR_HRS);
	if(hrs & (1 << 6 ))
	{
		//12 hr format
		rtc_time->time_format = !((hrs & (1 << 5)) == 0);
		hrs &= ~(0x3 << 5);
	}else
	{
		//24 hr format
		rtc_time->time_format = TIME_FORMAT_24HRS;
	}
	rtc_time->hours = bcd_to_binary(hrs);
}

void ds1307_set_current_date(RTC_date_t *rtc_date)
{
	ds1307_write(binary_to_bcd(rtc_date->date), DS1307_ADDR_DATE);
	ds1307_write(binary_to_bcd(rtc_date->day), DS1307_ADDR_DAY);
	ds1307_write(binary_to_bcd(rtc_date->month), DS1307_ADDR_MONTH);
	ds1307_write(binary_to_bcd(rtc_date->year), DS1307_ADDR_YEAR);
}

void ds1307_get_current_date(RTC_date_t *rtc_date)
{
	rtc_date->date = bcd_to_binary(ds1307_read(DS1307_ADDR_DATE));
	rtc_date->day = bcd_to_binary(ds1307_read(DS1307_ADDR_DAY));
	rtc_date->month = bcd_to_binary(ds1307_read(DS1307_ADDR_MONTH));
	rtc_date->year = bcd_to_binary(ds1307_read(DS1307_ADDR_YEAR));
}

/*
 *  Helper Functions
 */

static void ds1307_i2c_pin_config(void)
{
	GPIO_Handle_t i2c_sda, i2c_scl;

	memset(&i2c_scl,0,sizeof(i2c_scl));
	memset(&i2c_sda,0,sizeof(i2c_sda));
	// SDA
	i2c_sda.pGPIOx = DS1307_I2C_GPIO_PORT;
	i2c_sda.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	i2c_sda.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	i2c_sda.GPIO_PinConfig.GPIO_PinNumber = DS1307_I2C_SDA_PIN;
	i2c_sda.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	i2c_sda.GPIO_PinConfig.GPIO_PinPuPdControl = DS1307_I2C_PUPD;
	i2c_sda.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Init(&i2c_sda);
	// SCL
	i2c_scl.pGPIOx = DS1307_I2C_GPIO_PORT;
	i2c_scl.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	i2c_scl.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	i2c_scl.GPIO_PinConfig.GPIO_PinNumber = DS1307_I2C_SCL_PIN;
	i2c_scl.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	i2c_scl.GPIO_PinConfig.GPIO_PinPuPdControl = DS1307_I2C_PUPD;
	i2c_scl.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Init(&i2c_scl);

}

static void ds1307_i2c_config(void)
{
	DS1307_I2CHandle.pI2Cx = DS1307_I2C;
	DS1307_I2CHandle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	DS1307_I2CHandle.I2C_Config.I2C_SclkSpeed = I2C_SCL_SPEED_SM;
	I2C_Init(&DS1307_I2CHandle);

}

static void ds1307_write(uint8_t value,uint8_t reg_addr)
{
	uint8_t tx[2];
	tx[0] = reg_addr;
	tx[1] = value;
	I2C_MasterSendData(&DS1307_I2CHandle, tx, 2, DS1037_I2C_ADDR, 0);
}

static uint8_t ds1307_read(uint8_t reg_addr)
{
	uint8_t rx;
	I2C_MasterSendData(&DS1307_I2CHandle, &reg_addr, 1, DS1037_I2C_ADDR, 0);
	I2C_MasterReceiveData(&DS1307_I2CHandle, &rx , 1 , DS1037_I2C_ADDR, 0);

	return rx;
}

static uint8_t binary_to_bcd(uint8_t value)
{
	uint8_t m , n ;
	uint8_t bcd ;
	if(value >= 10)
	{
		m = value / 10 ;
		n = value % 10 ;
		bcd = (uint8_t)((m << 4) | n );

	}else
	{
		bcd = value;
	}
	return bcd;
}

static uint8_t bcd_to_binary(uint8_t value)
{
	uint8_t binary , m , n;

	m = (uint8_t)((value >> 4) * 10);
	n = value & (uint8_t)0x0F ;
	binary = m+n;

	return binary;
}