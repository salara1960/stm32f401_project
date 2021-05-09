/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <malloc.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <stdarg.h>
#include "hdr.h"

#ifdef SET_W25FLASH
	#include "w25.h"
#endif
#ifdef SET_OLED_I2C
	#include "ssd1306.h"
#endif
#ifdef SET_KBD
	#include "mpr121.h"
#endif
#ifdef SET_DFPLAYER
	#include "dfplayer.h"
#endif
#ifdef SET_IRED
	#include "IRremote.h"
#endif


/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
enum {
	devOK = 0,
	devSPI = 1,
	devUART = 2,
	devI2C = 4,
	devKBD = 8,
	devRTC = 0x10,
	devMem = 0x20,
	devDFP = 0x40,
	devFifo = 0x80
};

typedef enum {
	msg_empty = 0,
	//msg_rst,
	msg_250ms,
	msg_sec,
	msg_kbd,
	msg_showFolder,
	msg_getAllTracks,
	msg_getAllFolders,
	msg_folders,
	msg_newDir,
	msg_newDirSel,
	msg_ukDirSel,
	msg_tail,
	msg_chkMode,
	msg_play,
	msg_rplay,
	msg_stop,
	msg_back,
	msg_fwd,
	msg_volUp,
	msg_volDown,
	msg_volGet,
	msg_track,
	msg_eqSet,
	msg_eqGet,
	msg_dfpRX,
	msg_bleRx,
	msg_none
} evt_t;

#define _10ms 1
#define _20ms (_10ms * 2)
#define _30ms (_10ms * 3)
#define _40ms (_10ms * 4)
#define _50ms (_10ms * 5)
#define _60ms (_10ms * 6)
#define _70ms (_10ms * 7)
#define _80ms (_10ms * 8)
#define _90ms (_10ms * 9)
#define _100ms (_10ms * 10)
#define _150ms (_10ms * 15)
#define _200ms (_10ms * 20)
#define _250ms (_10ms * 25)
#define _300ms (_10ms * 30)
#define _350ms (_10ms * 35)
#define _400ms (_10ms * 40)
#define _450ms (_10ms * 45)
#define _500ms (_10ms * 50)
#define _600ms (_10ms * 60)
#define _700ms (_10ms * 70)
#define _800ms (_10ms * 80)
#define _900ms (_10ms * 90)
#define _1s (_100ms * 10)
#define _1s5 (_100ms * 15)
#define _2s (_1s * 2)//2000
#define _3s (_1s * 3)//3000
#define _4s (_1s * 4)//4000
#define _5s (_1s * 5)//5000
#define _10s (_1s * 10)//10000
//#define _15s (_1s * 15)
//#define _20s (_1s * 20)
//#define _25s (_1s * 25)
//#define _30s (_1s * 30)

#define MAX_UART_BUF  768
#define MAX_FIFO_SIZE 50

//#define TIME_kbdKeyPressed 75

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

uint8_t Report(const char *tag, bool addTime, const char *fmt, ...);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define KBD_INT_Pin GPIO_PIN_1
#define KBD_INT_GPIO_Port GPIOA
#define KBD_INT_EXTI_IRQn EXTI1_IRQn
#define SPI1_NSS_Pin GPIO_PIN_4
#define SPI1_NSS_GPIO_Port GPIOA
#define IRED_Pin GPIO_PIN_2
#define IRED_GPIO_Port GPIOB
#define LED_ERROR_Pin GPIO_PIN_12
#define LED_ERROR_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_4
#define LED_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define min_wait_ms 350
#define max_wait_ms 1000

#define W25_CS_GPIO_Port SPI1_NSS_GPIO_Port
#define W25_CS_Pin SPI1_NSS_Pin


#ifdef SET_KBD
	I2C_HandleTypeDef *portKBD;
	int16_t kbdAddr;
#endif
#ifdef SET_OLED_I2C
	I2C_HandleTypeDef *portOLED;
#endif
#ifdef SET_W25FLASH
	SPI_HandleTypeDef *portFLASH;
#endif

#ifdef SET_DFPLAYER

#pragma pack(push,1)
typedef struct {
	int number;//number of folder
	int tracks;//total tracks in folder
} folder_t;
#pragma pack(pop)

	folder_t *all_folder;
	UART_HandleTypeDef *portDFP;
	uint8_t dfpCmd;
	uint8_t dfpRdy;
#endif

uint8_t devError;

#ifdef SET_IRED

	#define MAX_IRED_KEY 21

	enum {
		key_ch_minus = 0,
		key_ch,
		key_ch_plus,
		key_left,
		key_right,
		key_sp,
		key_minus,
		key_plus,
		key_eq,
		key_100,
		key_200,
		key_0,
		key_1,
		key_2,
		key_3,
		key_4,
		key_5,
		key_6,
		key_7,
		key_8,
		key_9
	};

	#pragma pack(push,1)
	typedef struct {
		char name[8];
		uint32_t code;
	} one_key_t;
	#pragma pack(pop)

	TIM_HandleTypeDef *portIRED;//htim3; // таймер для приёма

#endif


#ifdef SET_BLE
	#define MAX_BLE_BUF 255

	#pragma pack(push,1)
	typedef struct {
		uint8_t con;
		char name[16];
	} ble_client_t;
	#pragma pack(pop)

	UART_HandleTypeDef *portBLE;
//	static char ble_RxBuf[255];
//	volatile uint8_t ble_rx_uk;
//	volatile uint8_t ble_uRxByte = 0;
//	static char BleBuf[255];
#endif


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
