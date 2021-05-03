
#include "hdr.h"

#ifdef SET_OLED_I2C

#include "main.h"
#include "ssd1306.h"


//------------------------------------------------------------------------

uint8_t invert = OLED_CMD_DISPLAY_NORMAL;
uint8_t oled_withDMA = 0;

//******************************************************************************************

//-----------------------------------------------------------------------------------------
uint8_t i2c_ssd1306_calcx(int len)
{
uint8_t ret = 0;

    if ( (len > 0) && (len < 16) ) ret = ((16 - len) >> 1) + 1;

    return ret;
}
//-----------------------------------------------------------------------------------------
void i2c_ssd1306_on(bool flag)
{
uint8_t dat[] = {OLED_CONTROL_BYTE_CMD_SINGLE, 0};
HAL_StatusTypeDef rt = HAL_OK;

    if (flag) dat[1] = OLED_CMD_DISPLAY_ON;
    	 else dat[1] = OLED_CMD_DISPLAY_OFF;

    if (oled_withDMA) {
    	rt = HAL_I2C_Master_Transmit_DMA(portOLED, OLED_I2C_ADDRESS, dat, sizeof(dat));
    	while (HAL_I2C_GetState(portOLED) != HAL_I2C_STATE_READY) {}
    } else {
    	rt = HAL_I2C_Master_Transmit(portOLED, OLED_I2C_ADDRESS, dat, sizeof(dat), min_wait_ms);
    }

    if (rt != HAL_OK) devError |= devI2C;
}
//-----------------------------------------------------------------------------------------
void i2c_ssd1306_init()
{
uint8_t dat[] = {
	OLED_CONTROL_BYTE_CMD_STREAM,//0x00
	OLED_CMD_SET_CHARGE_PUMP,    //0x8D
	0x14,
	OLED_CMD_SET_SEGMENT_REMAP,  //0xA1
	OLED_CMD_SET_COM_SCAN_MODE,  //0xC8
	OLED_CMD_SET_COLUMN_RANGE,   //0x21
	0x00,
	0x7F,
	OLED_CMD_SET_PAGE_RANGE,     //0x22
	0x00,
	0x07,
	OLED_CMD_DISPLAY_ON,         //0xAF
	invert
};
HAL_StatusTypeDef rt = HAL_OK;

	if (oled_withDMA) {
		rt = HAL_I2C_Master_Transmit_DMA(portOLED, OLED_I2C_ADDRESS, dat, sizeof(dat));
		while (HAL_I2C_GetState(portOLED) != HAL_I2C_STATE_READY) {}
	} else {
		rt = HAL_I2C_Master_Transmit(portOLED, OLED_I2C_ADDRESS, dat, sizeof(dat), min_wait_ms);
	}


	if (rt != HAL_OK) devError |= devI2C;
}
//-----------------------------------------------------------------------------------------
#ifdef SET_SSD1306_INVERT
void i2c_ssd1306_invert()
{
uint8_t dat[] = {OLED_CONTROL_BYTE_CMD_SINGLE, 0};
HAL_StatusTypeDef rt = HAL_OK;


    if (invert == OLED_CMD_DISPLAY_INVERTED) invert = OLED_CMD_DISPLAY_NORMAL;
										else invert = OLED_CMD_DISPLAY_INVERTED;
    dat[1] = invert;

    if (oled_withDMA) {
    	rt = HAL_I2C_Master_Transmit_DMA(portOLED, OLED_I2C_ADDRESS, dat, sizeof(dat));
    	while (HAL_I2C_GetState(portOLED) != HAL_I2C_STATE_READY) {/*HAL_Delay(1);*/}
    } else {
    	rt = HAL_I2C_Master_Transmit(portOLED, OLED_I2C_ADDRESS, dat, sizeof(dat), min_wait_ms);
    }

    if (rt != HAL_OK) devError |= devI2C;
}
#endif
//-----------------------------------------------------------------------------------------
void i2c_ssd1306_clear()
{
uint8_t i, dat[] = {OLED_CONTROL_BYTE_CMD_SINGLE, 0};
HAL_StatusTypeDef rt = HAL_OK;
uint8_t zero[129] = {0};
uint8_t dma = oled_withDMA;


	zero[0] = OLED_CONTROL_BYTE_DATA_STREAM;
	for (i = 0; i < 8; i++) {
		dat[1] = 0xB0 | i;
		if (dma) {
			rt  = HAL_I2C_Master_Transmit(portOLED, OLED_I2C_ADDRESS, dat,    2, min_wait_ms);
			rt |= HAL_I2C_Master_Transmit_DMA(portOLED, OLED_I2C_ADDRESS, zero, 129);
			while (HAL_I2C_GetState(portOLED) != HAL_I2C_STATE_READY) {}
		} else {
			rt  = HAL_I2C_Master_Transmit(portOLED, OLED_I2C_ADDRESS, dat,    2, min_wait_ms);
			rt |= HAL_I2C_Master_Transmit(portOLED, OLED_I2C_ADDRESS, zero, 129, max_wait_ms);
		}
	}

	if (rt != HAL_OK) devError |= devI2C;
}
//-----------------------------------------------------------------------------------------
void i2c_ssd1306_clear_line(uint8_t cy)
{
cy--;
HAL_StatusTypeDef rt = HAL_OK;
uint8_t cif_zero[] = {OLED_CONTROL_BYTE_DATA_STREAM, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t first[] = {
	OLED_CONTROL_BYTE_CMD_STREAM,
	OLED_CMD_SET_COLUMN_RANGE,
	0,
	0x7f,
	OLED_CMD_SET_PAGE_RANGE,
	cy,
	7
};
uint8_t next[] = {
	OLED_CONTROL_BYTE_CMD_SINGLE,
	0xB0 | cy
};
	uint8_t dma = oled_withDMA;

	if (HAL_I2C_Master_Transmit(portOLED, OLED_I2C_ADDRESS, first, sizeof(first), max_wait_ms) == HAL_OK) {
		if (HAL_I2C_Master_Transmit(portOLED, OLED_I2C_ADDRESS, next, sizeof(next), min_wait_ms) == HAL_OK) {
			for (uint8_t i = 0; i < 16; i++) {
				if (dma) {
					rt |= HAL_I2C_Master_Transmit_DMA(portOLED, OLED_I2C_ADDRESS, cif_zero, sizeof(cif_zero));
					while (HAL_I2C_GetState(portOLED) != HAL_I2C_STATE_READY) {}
				} else {
					rt |= HAL_I2C_Master_Transmit(portOLED, OLED_I2C_ADDRESS, cif_zero, sizeof(cif_zero), max_wait_ms);
				}
			}
		}
	}

	if (rt != HAL_OK) devError |= devI2C;
}
//-----------------------------------------------------------------------------------------
void i2c_ssd1306_pattern()
{
uint8_t i, dat[] = {OLED_CONTROL_BYTE_CMD_SINGLE, 0};
uint8_t buf[129] = {0};
HAL_StatusTypeDef rt = HAL_OK;
uint8_t dma = oled_withDMA;

	buf[0] = OLED_CONTROL_BYTE_DATA_STREAM;
	for (i = 1; i < 129; i++) buf[i] = 0xFF >> (i % 8);
	for (i = 0; i < 8; i++) {
		dat[1] = 0xB0 | i;
		if (dma) {
			rt |= HAL_I2C_Master_Transmit(portOLED, OLED_I2C_ADDRESS, dat,   2, min_wait_ms);
			rt |= HAL_I2C_Master_Transmit_DMA(portOLED, OLED_I2C_ADDRESS, buf, 129);
			while (HAL_I2C_GetState(portOLED) != HAL_I2C_STATE_READY) {}
		} else {
			rt |= HAL_I2C_Master_Transmit(portOLED, OLED_I2C_ADDRESS, dat,   2, min_wait_ms);
			rt |= HAL_I2C_Master_Transmit(portOLED, OLED_I2C_ADDRESS, buf, 129, max_wait_ms);
		}
	}

	if (rt != HAL_OK) devError |= devI2C;
}
//-----------------------------------------------------------------------------------------
void i2c_ssd1306_contrast(uint8_t value)//0xff or 0x00
{
uint8_t dat[] = {OLED_CONTROL_BYTE_CMD_STREAM, OLED_CMD_SET_CONTRAST, value};
HAL_StatusTypeDef rt = HAL_OK;

	if (oled_withDMA) {
		rt = HAL_I2C_Master_Transmit_DMA(portOLED, OLED_I2C_ADDRESS, dat, sizeof(dat));
		while (HAL_I2C_GetState(portOLED) != HAL_I2C_STATE_READY) {}
	} else {
		rt = HAL_I2C_Master_Transmit(portOLED, OLED_I2C_ADDRESS, dat, sizeof(dat), min_wait_ms);
	}

	if (rt != HAL_OK) devError |= devI2C;
}
//-----------------------------------------------------------------------------------------
void i2c_ssd1306_text_xy(const char *stroka, uint8_t cx, uint8_t cy, bool inv)
{
HAL_StatusTypeDef rt = HAL_OK;
uint8_t i, lin = cy - 1, col = cx - 1;
int len = strlen(stroka);
uint8_t dat[] = {OLED_CONTROL_BYTE_CMD_STREAM, 0, 0x10, 0};
uint8_t cif[] = {OLED_CONTROL_BYTE_DATA_STREAM, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t first[] = {
	OLED_CONTROL_BYTE_CMD_STREAM,
	OLED_CMD_SET_COLUMN_RANGE,
	col << 3,
	0x7f,
	OLED_CMD_SET_PAGE_RANGE,
	lin,
	7
};
	uint8_t dma = 0;//oled_withDMA;

	rt = HAL_I2C_Master_Transmit(portOLED, OLED_I2C_ADDRESS, first, sizeof(first), min_wait_ms);
	if (rt == HAL_OK) {
		for (i = 0; i < len; i++) {
			if (stroka[i] == '\n') {
				dat[3] = 0xB0 | ++lin;
				if (dma) {
					rt = HAL_I2C_Master_Transmit_DMA(portOLED, OLED_I2C_ADDRESS, dat, sizeof(dat));
					while (HAL_I2C_GetState(portOLED) != HAL_I2C_STATE_READY) {}
				} else {
					rt = HAL_I2C_Master_Transmit(portOLED, OLED_I2C_ADDRESS, dat, sizeof(dat), min_wait_ms);
				}
			} else {
				memcpy(&cif[1], &font8x8[(uint8_t)stroka[i]][0], 8);
				if (inv) for (uint8_t j = 1; j < sizeof(cif); j++) cif[j] = ~cif[j];
				if (dma) {
					rt = HAL_I2C_Master_Transmit_DMA(portOLED, OLED_I2C_ADDRESS, cif, sizeof(cif));
					while (HAL_I2C_GetState(portOLED) != HAL_I2C_STATE_READY) {}
				} else {
					rt = HAL_I2C_Master_Transmit(portOLED, OLED_I2C_ADDRESS, cif, sizeof(cif), max_wait_ms);
				}
			}
		}
	}

	if (rt != HAL_OK) devError |= devI2C;
}
//-----------------------------------------------------------------------------------------
void i2c_ssd1306_text(const char *stroka)
{
	if (stroka) i2c_ssd1306_text_xy(stroka, 1, 1, false);
}
//-----------------------------------------------------------------------------------------
char *mkLineCenter(char *str, uint16_t width)
{
char st[32] = {0};

	memset(st, 0x20, 32);
	uint8_t slen = OLED_WIDTH / width;
	uint8_t k = strlen(str);
	if (k < slen) {
		uint8_t n = (slen - k) >> 1;
		memcpy((char *)&st[n], (char *)str, k);
		st[slen] = '\0';
		strcpy(str, st);
	}

	return str;
}
//-----------------------------------------------------------------------------------------
char *mkLineWidth(char *str1, char *str2, uint16_t width)
{
char st[64] = {0};

	uint8_t slen = OLED_WIDTH / width;
	uint8_t k1 = strlen(str1);
	uint8_t k2 = strlen(str2);
	if ((k1 + k2) <= slen) {
		uint8_t k = slen - (k1 + k2);
		strcpy(st, str1);
		for (int8_t i = 0; i < k; i++) st[k1 + i] = 0x20;
		strcat(st, str2);
		strcpy(str1, st);
	}

	return str1;
}
//******************************************************************************************

#endif

