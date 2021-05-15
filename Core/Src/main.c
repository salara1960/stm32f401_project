/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
  ******************************************************************************
  ******************************************************************************
  */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

//arm-none-eabi-objcopy -O ihex "${BuildArtifactFileBaseName}.elf" "${BuildArtifactFileBaseName}.hex" && arm-none-eabi-objcopy -O binary "${BuildArtifactFileBaseName}.elf" "${BuildArtifactFileBaseName}.bin" && ls -la | grep "${BuildArtifactFileBaseName}.*"

#include "stm32f4xx_hal_rtc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c2_tx;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi1_rx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart6_tx;

/* USER CODE BEGIN PV */

//const char *version = "0.1 (25.04.2021)";
//const char *version = "0.2 (26.04.2021)";
//const char *version = "0.3 (27.04.2021)";
//const char *version = "0.4 (28.04.2021)";
//const char *version = "0.5 (28.04.2021)";//support KBD done !
//const char *version = "0.6 (01.05.2021)";// add DFPlayer and connect to ARM via USART6(9600 8N1)
//const char *version = "0.7 (02.05.2021)";// add library for DFPlayer
//const char *version = "0.8 (03.05.2021)";// major changes for DFPlayer support (add recv. in interrupt mode)
//const char *version = "0.9 (05.05.2021)";//add support folders on storage
//const char *version = "1.0 (06.05.2021)";
//const char *version = "1.1 (07.05.2021)";// add infrared control module (TL1838)
//const char *version = "1.2 (08.05.2021)";// add BLE audio module (uart) - first step (host version - listen client)
//const char *version = "1.3 (09.05.2021)";// minor changes for BLE audio module support - second step
//const char *version = "1.4 (10.05.2021)";// major changes for BLE audio module : support 'income_opid:' message from ble_client's
//const char *version = "1.5 (11.05.2021)";// add ble_connect_enable_pin
//const char *version = "1.6 (12.05.2021)";// add ble_clients list
//const char *version = "1.7 (13.05.2021)";
//const char *version = "1.8 (14.05.2021)";// add support ble clients enable list : init, add, print
const char *version = "1.9 (15.05.2021)";// add support ble clients enable list : write/read list to/from spi flash



static evt_t evt_fifo[MAX_FIFO_SIZE] = {msg_empty};
uint8_t rd_evt_adr = 0;
uint8_t wr_evt_adr = 0;
uint8_t wr_evt_err = 0;
uint8_t cnt_evt = 0;
uint8_t max_evt = 0;
UART_HandleTypeDef *portLOG = &huart1;

//1620036392;//1619997553;//1619963555;//1619617520;//1619599870;//1619513155;//1619473366;//1619375396;//1619335999;
//1620725850;//1620673738;//1620557655;//1620467295;//1620406195;//1620329368;//1620246336;//1620214632;//1620063356;
volatile time_t epoch = 1621081250;//1621015568;//1620893900;//1620850510;//1620827210;
uint8_t tZone = 2;
volatile uint32_t cnt_err = 0;
volatile uint8_t restart_flag = 0;
volatile static uint32_t secCounter = 0;
volatile static uint32_t HalfSecCounter = 0;
volatile static bool setDate = false;
static const char *_extDate = "epoch=";
static const char *_restart = "rst";
volatile uint32_t extDate = 0;
static char RxBuf[MAX_UART_BUF];
volatile uint8_t rx_uk;
volatile uint8_t uRxByte = 0;
uint8_t uartRdy = 1;
uint8_t devError = 0;
char stx[128] = {0};

I2C_HandleTypeDef *portOLED = &hi2c2;
char buf[128] = {0};
uint8_t screenON = 0;

SPI_HandleTypeDef *portFLASH = &hspi1;
uint32_t spi_cnt = 0;
uint8_t spiRdy = 1;

bool kbdPresent = false;
bool kbdInitOk = false;
int16_t kbdAddr = 0;
bool kbdEnable = false;
#ifdef SET_KBD
	I2C_HandleTypeDef *portKBD = &hi2c1;
	uint8_t cntKBD = 0;
	volatile uint16_t kbdCode = 0;
	volatile uint32_t kbdCnt = 0;
	uint8_t kbdRdy = 1;
#endif

#ifdef SET_DFPLAYER
	UART_HandleTypeDef *portDFP = &huart6;
	int eqClass = EQ_Normal;
	int dfp_volume = DFPLAYER_MAX_VOLUME;
	uint8_t dfpCmd = 0;
	//
	static char dfp_RxBuf[MAX_UART_BUF];
	volatile uint8_t dfp_rx_uk;
	volatile uint8_t dfp_uRxByte = 0;
	uint8_t dfpRdy = 1;
	//
	uint8_t dfp_ACK[64] = {0};
	uint8_t dfp_ACK_len = 0;
	bool dfp_Begin = false;
	bool dfp_End = false;
	uint32_t dfp_tmr_next = 0;
	int8_t dfp_folder = 0;//folder number
	int8_t dfp_folders = 0;//total folders on storage
	uint16_t dfp_track = 0;//track number on storage
	uint16_t dfp_folder_tracks = 0;//total tracks in folder
	uint16_t dfp_folder_trk = 0;//track number in folder
	uint16_t dfp_all_tracks = 0;
	bool dfp_pause = false;
	//
	folder_t *all_folder = NULL;
	//
	bool newDir = false;
	uint32_t tmr_newDir = 0;
	int8_t uk_newDir = 0;
	bool inv_newDir = false;

#endif

#ifdef SET_IRED

	const one_key_t keyAll[MAX_IRED_KEY] = {
			{"irCH-",   0xe318261b},
			{"irCH",    0x00511dbb},
			{"irCH+",   0xee886d7f},
			{"irLEFT",  0x52a3d41f},
			{"irRIGHT", 0xd7e84b1b},
			{"irSP",    0x20fe4dbb},
			{"ir-",     0xf076c13b},
			{"ir+",     0xa3c8eddb},
			{"irEQ",    0xe5cfbd7f},
			{"ir100+",  0x97483bfb},
			{"ir200+",  0xf0c41643},
			{"ir0",     0xc101e57b},
			{"ir1",     0x9716be3f},
			{"ir2",     0x3d9ae3f7},
			{"ir3",     0x6182021b},
			{"ir4",     0x8c22657b},
			{"ir5",     0x488f3cbb},
			{"ir6",     0x0449e79f},
			{"ir7",     0x32c6fdf7},
			{"ir8",     0x1bc0157b},
			{"ir9",     0x3ec3fc1b}
	};

	TIM_HandleTypeDef *portIRED = &htim3; // таймер для приёма

#endif

#ifdef SET_BLE
	const char *TAG_BLE = "BLE";
	UART_HandleTypeDef *portBLE = &huart2;
	static char ble_RxBuf[MAX_BLE_BUF] = {0};
	volatile uint8_t ble_rx_uk;
	volatile uint8_t ble_uRxByte = 0;
	static char BleBuf[MAX_BLE_BUF] = {0};
	ble_client_t ble_client;
	char ble_str[32] = {0};
	uint8_t ble_withDMA = 1;
	char bleTmp[MAX_BLE_BUF] = {0};
	char ble_TxBuf[MAX_BLE_BUF] = {0};
	int ble_status = bleERR;
	uint32_t tmr_ble_con = 0;
	uint16_t ble_CtlCode = 0;
	uint8_t ble_CtlInd = 0;
	//
	ble_cli_hdr ble_hdr;
	int8_t ble_index = -1;

	const char *ble_allName_def[ble_maxName_def] =
	{
		"Sabbat",
		"S650"
	};

	str_name_t cli_name[MAX_BLE];
	uint8_t total_cli = 0;

	uint8_t pageBufTx[PAGE_BUF_SIZE] = {0};
	uint8_t pageBufRx[PAGE_BUF_SIZE] = {0};

#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C2_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

uint32_t get_tmr10(uint32_t ms);
bool check_tmr10(uint32_t ms);
uint32_t get_tmr(uint32_t sec);
bool check_tmr(uint32_t sec);
//void floatPart(float val, s_float_t *part);
void errLedOn(const char *from);
void set_Date(time_t epoch);
int sec_to_str_time(uint32_t sec, char *stx);
uint8_t Report(const char *tag, bool addTime, const char *fmt, ...);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//-------------------------------------------------------------------------------------------
void putMsg(evt_t evt)
{

	if (cnt_evt > (MAX_FIFO_SIZE - 5)) return;

	HAL_NVIC_DisableIRQ(EXTI1_IRQn);
	HAL_NVIC_DisableIRQ(TIM3_IRQn);
#ifdef SET_DFPLAYER
	HAL_NVIC_DisableIRQ(USART6_IRQn);
#endif
#ifdef SET_BLE
	HAL_NVIC_DisableIRQ(USART2_IRQn);
#endif


	if (cnt_evt >= MAX_FIFO_SIZE) {
		wr_evt_err++;
	} else {
		evt_fifo[wr_evt_adr] = evt;
		cnt_evt++;
		if (wr_evt_adr < (MAX_FIFO_SIZE - 1) ) {
			wr_evt_adr++;
		} else  {
			wr_evt_adr = 0;
		}
		wr_evt_err = 0;
		if (cnt_evt > max_evt) max_evt = cnt_evt;
	}

	if (wr_evt_err) devError |= devFifo;
		       else devError &= ~devFifo;

#ifdef SET_BLE
	HAL_NVIC_EnableIRQ(USART2_IRQn);
#endif
#ifdef SET_DFPLAYER
	HAL_NVIC_EnableIRQ(USART6_IRQn);
#endif
	HAL_NVIC_EnableIRQ(TIM3_IRQn);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}
//-------------------------------------------------------------------------------------------
evt_t getMsg()
{
evt_t ret = msg_empty;

	HAL_NVIC_DisableIRQ(EXTI1_IRQn);
	HAL_NVIC_DisableIRQ(TIM3_IRQn);
#ifdef SET_DFPLAYER
	HAL_NVIC_DisableIRQ(USART6_IRQn);
#endif
#ifdef SET_BLE
	HAL_NVIC_DisableIRQ(USART2_IRQn);
#endif


	if (cnt_evt) {
		ret = evt_fifo[rd_evt_adr];
		if (cnt_evt) cnt_evt--;
		if (rd_evt_adr < (MAX_FIFO_SIZE - 1) ) {
			rd_evt_adr++;
		} else {
			rd_evt_adr = 0;
		}
	}

#ifdef SET_BLE
	HAL_NVIC_EnableIRQ(USART2_IRQn);
#endif
#ifdef SET_DFPLAYER
	HAL_NVIC_EnableIRQ(USART6_IRQn);
#endif
	HAL_NVIC_EnableIRQ(TIM3_IRQn);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

	return ret;
}
//-------------------------------------------------------------------------------------------

//*******************************************************************************************
#ifdef SET_BLE
//-------------------------------------------------------------------------------------------
	#ifdef SET_W25FLASH
	void pageUpdate(uint32_t pnum)
	{
		W25qxx_EraseSector(W25qxx_PageToSector(pnum));

		memset(pageBufTx, 0xff, PAGE_BUF_SIZE);
		memcpy(pageBufTx, (uint8_t *)&cli_name[0].name[0], total_cli * sizeof(str_name_t));
		W25qxx_WritePage(pageBufTx, pnum, 0, PAGE_BUF_SIZE);
	}
	#endif

//-------------------------------------------------------------------------------------------
void ble_init_names()
{
	memset((char *)&cli_name[0], 0, sizeof(str_name_t) * MAX_BLE);

	for (uint8_t i = 0; i < ble_maxName_def; i++)
		strcpy(cli_name[i].name, ble_allName_def[i]);

	total_cli = ble_maxName_def;
}
//-------------------------------------------------------------------------------------------
uint8_t ble_add_names(char *name)
{
uint8_t ok = 0;

	uint8_t i = 255;
	while (++i < total_cli) {
		if (!strncmp(cli_name[i].name, name, strlen(name))) ok |= 1;
	}

	if (!ok) strcpy(cli_name[total_cli++].name, name);

	return ok;
}
//-------------------------------------------------------------------------------------------
void ble_prnEnableList()
{
	if (!total_cli) return;

	Report(TAG_BLE, true, "Ble enable list [%u]:\n", total_cli);

	uint8_t i = 255;
	while (++i < total_cli)
		Report(NULL, false, "\t%s\n", cli_name[i].name);

}
//-------------------------------------------------------------------------------------------
void bleWrite(char *str, bool prn)
{
	if (ble_withDMA) {
		if (HAL_UART_Transmit_DMA(portBLE, (uint8_t *)str, strlen(str)) != HAL_OK) devError |= devBLE;
		/*
		while (HAL_UART_GetState(portBLE) != HAL_UART_STATE_READY) {
			if (HAL_UART_GetState(portBLE) == HAL_UART_STATE_BUSY_RX) break;
			//HAL_Delay(1);
		}
		*/
	} else {
		if (HAL_UART_Transmit(portBLE, (uint8_t *)str, strlen(str), 1000) != HAL_OK) devError |= devBLE;
	}

	if (prn) Report(TAG_BLE, true, "%s\n", str);

}
//-------------------------------------------------------------------------------------------
void parseCtl(uint16_t ccode)
{
	evt_t ev = msg_none;
	switch (ccode) {
		case bleNextTrk://0x4bcb:// next track
			ev = msg_fwd;
		break;
		case bleBackTrk://0x4ccc:// back track
			ev = msg_back;
		break;
		case blePlay://0x46c6: //play
		case blePause://0x44c4: //pause
			ev = msg_rplay;
		break;
	}

	if (ev != msg_none) putMsg(ev);
}
//-------------------------------------------------------------------------------------------
bool ble_checkName(char *name)
{
bool ret = false;

	if (!name || !strlen(name)) return ret;

	for (uint8_t i = 0; i < total_cli; i++) {
		if (strstr(cli_name[i].name, name)) {
			ret = true;
			break;
		}
	}

	return ret;
}
//-------------------------------------------------------------------------------------------
void blePin()
{
	BLE_CONN_DOWN();
	uint32_t start = HAL_GetTick();
	HAL_Delay(9);
	BLE_CONN_UP();
	Report(TAG_BLE, true, "\t*** BLE_CONN_PIN UP (%lu) ***\n", HAL_GetTick() - start);
}
//-------------------------------------------------------------------------------------------
void ble_hdrClear()
{
	ble_hdr.lock = ble_hdr.total = 0;
	ble_hdr.begin = ble_hdr.end = NULL;
}
//-------------------------------------------------------------------------------------------
uint8_t ble_getTotalRec()
{
	while (ble_hdr.lock) { HAL_Delay(1); };
	//
	ble_hdr.lock = 1;
	uint8_t ret = ble_hdr.total;
	ble_hdr.lock = 0;

	return ret;
}
//-------------------------------------------------------------------------------------------
int8_t ble_addRec(ble_client_t *rec)
{
int8_t ret = -1;

	while (ble_hdr.lock) { HAL_Delay(1); };
	//
	ble_hdr.lock = 1;
	uint8_t err = devOK;
	bool add = true;
	//
	if (ble_hdr.total) {
		ble_cli_rec *tp = NULL, *tmp = ble_hdr.begin;
		while (tmp) {
			ble_client_t *rc = (ble_client_t *)tmp->rec;
			if (!strncmp(rc->name, rec->name, strlen(rc->name))) {
				if (!strncmp(rc->mac, rec->mac, strlen(rc->mac))) {
					add = false;
					break;
				}
			} else {
				tp = (ble_cli_rec *)tmp->next;
				tmp = tp;
			}
		}
	}
	//
	if (add) {
		//
		ble_cli_rec *cur = (ble_cli_rec *)calloc(1, sizeof(ble_cli_rec));
		if (cur) {
			ble_client_t *rc = (ble_client_t *)calloc(1, sizeof(ble_client_t));//(ble_client_t *)cur->rec;
			if (rc) {
				memcpy((uint8_t *)rc , (uint8_t *)rec, sizeof(ble_client_t));
				if (!ble_hdr.total) {
					ble_hdr.begin = ble_hdr.end = cur;
				} else {
					ble_cli_rec *back = ble_hdr.end;
					ble_hdr.end = cur;
					back->next = cur;
				}
				cur->rec = (void *)rc;
				cur->next = NULL;
				cur->ind = ble_hdr.total;
				ret = (int8_t)cur->ind;
				ble_hdr.total++;
			} else err |= devMem;
		} else err |= devMem;
		//
	}
	//
	ble_hdr.lock = 0;

	devError |= err;

	return ret;
}
//-------------------------------------------------------------------------------------------
ble_cli_rec *ble_getRec(uint8_t ind)
{
	while (ble_hdr.lock) { HAL_Delay(1); };
	//
	ble_hdr.lock = 1;
	//
	ble_cli_rec *tmp = NULL, *cur = NULL;
	if (ind < ble_hdr.total) {
		cur = ble_hdr.begin;
		while (cur) {
			if (cur->ind == ind) break;
			tmp = (ble_cli_rec *)cur->next;
			cur = tmp;
		}
	}
	//
	ble_hdr.lock = 0;

	return cur;
}
//-------------------------------------------------------------------------------------------
void ble_prnRec()
{
	char stz[128];
	ble_cli_hdr hdr;
	while (ble_hdr.lock) { HAL_Delay(1); }
	ble_hdr.lock = 1;
	memcpy((uint8_t *)&hdr, (uint8_t *)&ble_hdr, sizeof(ble_cli_hdr));
	ble_hdr.lock = 0;

	sprintf(stz, "\n\tHDR: total=%u begin=%p end=%p\n", hdr.total, hdr.begin, hdr.end);
	Report(TAG_BLE, true, "%s", stz);

	uint8_t i = 255;
	while (++i < hdr.total) {
		ble_cli_rec *adr = ble_getRec(i);
		if (adr) {
			ble_client_t *cli = (ble_client_t *)adr->rec;
			sprintf(stz, "\tREC: ind=%u next=%p rec=%p: con=0x%X name='%.*s' mac=0x%.*s\n",
						adr->ind, adr->next, adr->rec, cli->con, sizeof(cli->name), cli->name, sizeof(cli->mac), cli->mac);
			Report(NULL, false, "%s", stz);
		}
	};
}
//-------------------------------------------------------------------------------------------
#endif
//*******************************************************************************************

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_I2C2_Init();
  MX_RTC_Init();
  MX_TIM3_Init();
  MX_USART6_UART_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  	HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_SET);
    HAL_Delay(250);
    HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);



    //"start" rx_interrupt
    HAL_UART_Receive_IT(portLOG, (uint8_t *)&uRxByte, 1);
#ifdef SET_DFPLAYER
    HAL_UART_Receive_IT(portDFP, (uint8_t *)&dfp_uRxByte, 1);
#endif
#ifdef SET_BLE
    HAL_UART_Receive_IT(portBLE, (uint8_t *)&ble_uRxByte, 1);

    BLE_CONN_DOWN();
#endif

    // start timer3 in interrupt mode
    HAL_TIM_Base_Start_IT(&htim3);
    // start timer2 for IRED control
    HAL_TIM_Base_Start_IT(&htim2);


    oled_withDMA = 1;

    screenON = 1;

    i2c_ssd1306_init();//screen INIT
    i2c_ssd1306_pattern();//set any params for screen
    //i2c_ssd1306_invert();
    i2c_ssd1306_clear();//clear screen

    set_Date((time_t)(++epoch));
    uint32_t last_sec = (uint32_t)epoch - 2;

#ifdef SET_KBD
    kbdPresent = KBD_getAddr(&kbdAddr);
    if (kbdPresent) {
    	kbdInitOk = kbdInit();
    	if (kbdInitOk) kbdEnable = true;
    }
#endif

    Report(NULL, true, "Version '%s' KBD: Present=%d Init=%d Addr=0x%02X, ERROR: 0x%X\n", version, kbdPresent, kbdInitOk, kbdAddr, devError);

#ifdef SET_W25FLASH
    //W25_UNSELECT();

    int8_t cn = 4;
    while (--cn) {
    	HAL_Delay(250);
    	if (W25qxx_Init()) break;
    }
#endif

#ifdef SET_DFPLAYER
    uint8_t idDev = 0;
    uint8_t lnum = 4;
    DFP_reset();
    HAL_Delay(50);
    DFP_set_eq(eqClass);
    HAL_Delay(50);
    if (!(devError & devDFP)) {
    	dfp_volume = DFPLAYER_MAX_VOLUME >> 1;
    	DFP_set_volume(dfp_volume);
    	// for folder
    	dfp_folder = 1;
    	dfp_folder_tracks = 1;
    	dfp_folder_trk = 1;
    	//for storage
    	dfp_track = 1;
    }
    int8_t curFolder = dfp_folder;
    uint32_t tmr_get_trk = 0;
    uk_newDir = dfp_folder;
    char line_folder[32] = {0};
#endif


#ifdef SET_IRED
    memset((uint8_t *)&ble_client, 0, sizeof(ble_client_t));
    char stline[32] = {0};
	uint32_t tmr_ired = 0;
	enIntIRED();
#endif

#ifdef SET_BLE
	ble_init_names();
	ble_prnEnableList();

	strcpy(ble_TxBuf, "AT+REST");// MacAddr=0x0000100014a,Name=Sabbat X12 Pro
	bleWrite(ble_TxBuf, true);
	char bleTmp[MAX_BLE_BUF] = {0};
	//BLE_CONN_DOWN();
	blePin();
	//
	ble_hdrClear();
	//
	#ifdef SET_W25FLASH
		uint32_t page_num = 0;//uint32_t Page_Address
		//W25qxx_EraseSector(0);
		if (W25qxx_IsEmptyPage(page_num, 0, 0)) {//if page #0 empty -> first write
			memset(pageBufTx, 0xff, PAGE_BUF_SIZE);
			memcpy(pageBufTx, (uint8_t *)&cli_name[0].name[0], total_cli * sizeof(str_name_t));
			W25qxx_WritePage(pageBufTx, page_num, 0, PAGE_BUF_SIZE);
		}
		W25qxx_ReadPage(pageBufRx, page_num, 0, 0);
		if (!w25_withDMA) {
			str_name_t *ones = (str_name_t *)pageBufRx;
			uint8_t tcli = 255;
			while (++tcli < (PAGE_BUF_SIZE / sizeof(str_name_t))) {
				if (ones->name[0] == 0xff) break;
				ones++;
			}
			total_cli = tcli;
			memcpy(&cli_name[0].name[0], (char *)pageBufRx, total_cli * sizeof(str_name_t));
			ble_prnEnableList();
		}
	#endif
#endif

    bool startOne = true;
    evt_t evt = msg_none;
    //uint32_t last_sec = (uint32_t)epoch;




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    while (!restart_flag) {

#ifdef SET_IRED
  		if (!tmr_ired) {
			if (decodeIRED(&results)) {

				tmr_ired = get_tmr10(_300ms);
				//HAL_GPIO_TogglePin(ENC_LED_GPIO_Port, ENC_LED_Pin);
				int8_t kid = -1;
				for (int8_t i = 0; i < MAX_IRED_KEY; i++) {
					if (results.value == keyAll[i].code) {
						kid = i;
						break;
					}
				}
				//
				if (kid == -1) sprintf(stline, "CODE:%08lX", results.value);
						  else sprintf(stline, "irKEY: %s", keyAll[kid].name);
				mkLineCenter(stline, FONT_WIDTH);
				i2c_ssd1306_text_xy(stline, 1, 3, false);
				//
				//uint8_t ys = 0;
				evt_t eve = msg_none;
				if (kid != -1) {
					switch (kid) {
						case key_ch:
							restart_flag = 1;
						break;
						case key_ch_plus:
							eve = msg_ukDirSel;//msg_eqGet;
						break;
						case key_ch_minus:
							eve = msg_newDir;//msg_volGet;
						break;
						case key_minus:
							eve = msg_volDown;
						break;
						case key_plus:
							eve = msg_volUp;
						break;
						case key_left:
							eve = msg_back;
						break;
						case key_right:
							eve = msg_fwd;
						break;
						case key_eq:
							eve = msg_rplay;
						break;
						case key_sp:
							eve = msg_play;
						break;
						case key_100:
							screenON++;
							screenON &= 1;
							i2c_ssd1306_on(screenON);
						break;
						case key_200:
							eve = msg_eqSet;
						break;
						case key_0:
						case key_1:
						case key_2:
						case key_3:
						case key_4:
						case key_5:
						case key_6:
						case key_7:
						case key_8:
						case key_9:
						break;
					}//switch (kid)
					if (eve != msg_none) putMsg(eve);
				}//if (kid != -1)
			}//if (decodeIRED(&results))
		}
		if (tmr_ired) {
			if (check_tmr10(tmr_ired)) {
				tmr_ired = 0;
				resumeIRED();
			}
		}
#endif

		if (restart_flag) break;

    	evt = getMsg();
    	switch ((int)evt) {
    		case msg_kbd:
#if defined(SET_KBD) && defined(SET_OLED_I2C)
    			//kbdCnt++;
    			//kbdCode = kbd_get_touch();
    			sprintf(buf, "KBD: <%c>", (char)kbdCode);
    			mkLineCenter(buf, FONT_WIDTH);
    			i2c_ssd1306_text_xy(buf, 1, 3, false);
#endif
#ifdef SET_DFPLAYER
    			evt_t new_evt = msg_none;
    			switch (kbdCode) {
    				case '*':
    					new_evt = msg_play;//msg_chkMode;//msg_play;
    					break;
    				case '0':
    					new_evt = msg_rplay;
    					break;
    				case '#':
    					new_evt = msg_stop;
    					break;
    				//
    				case '9':
    					new_evt = msg_back;
    					break;
    				case '8':
    					new_evt = msg_eqSet;
    					break;
    				case '7':
    					new_evt = msg_fwd;
    					break;
    				//
    				case '6':
    					new_evt = msg_volDown;
    					break;
    				case '5':
    					new_evt = msg_newDir;//msg_volGet;
    					break;
    				case '4':
    					new_evt = msg_volUp;
    					break;
    				//
    				case '3':
    					//new_evt = msg_bleList;
    					break;
    				case '2':
    					new_evt = msg_ukDirSel;//msg_eqGet;
    					break;
    				case '1':
    					new_evt = msg_bleCliSave;
    					break;
    			}
    			if (new_evt != msg_none) putMsg(new_evt);
    		break;
    		case msg_newDirSel:
    			mkLineCenter(line_folder, FONT_WIDTH);
    			i2c_ssd1306_text_xy(line_folder, 1, 5, inv_newDir);
    		break;
    		case msg_newDir:
    		{
    			evt_t ev;
    			if (!newDir) {
    				uk_newDir = dfp_folder;
    				if (uk_newDir > 0) {
    					sprintf(line_folder, "Folder: %u", uk_newDir);
    				} else {
    					sprintf(line_folder, "Folder: root");
    				}
    				inv_newDir = true;
    				newDir = true;
    				tmr_newDir = get_tmr(10);
    				ev = msg_newDirSel;
    			} else {
    				DFP_stop();
    				dfp_pause = true;
    				HAL_Delay(50);
    				//
    				dfp_folder = uk_newDir;
    				if (!dfp_folder) {
    					dfp_track = 1;
    				} else {
    					folder_t *fold = all_folder + dfp_folder - 1;
    					dfp_folder_trk = 1;
    					dfp_folder_tracks = fold->tracks;
    				}
    				newDir = false;
    				tmr_newDir = 0;
    				ev = msg_play;
    			}
    			putMsg(ev);
    		}
    		break;
    		case msg_ukDirSel:
    			if (newDir) {
    				tmr_newDir = get_tmr(10);
    				uk_newDir++;
    				inv_newDir = true;
    				if (uk_newDir > dfp_folders) uk_newDir = 0;
    				if (uk_newDir > 0) {
    					sprintf(line_folder, "Folder: %u", uk_newDir);
    				} else {
    					sprintf(line_folder, "Folder: root");
    				}
    				putMsg(msg_newDirSel);
    			}
    		break;
    		case msg_chkMode:
    			if (!(devError & devDFP)) {
    				if (dfp_folder > 0) {
    					DFP_get_tracks(dfp_folder);
    				} else {
    					putMsg(msg_play);
    				}
    			}
    		break;
    		case msg_play:
    			if (!(devError & devDFP)) {
    				putMsg(msg_showFolder);
    				if (dfp_folder > 0) {
    					DFP_play_folder(dfp_folder, dfp_folder_trk, false);
    				} else {
    					DFP_play_root(dfp_track, false);
    				}
    				putMsg(msg_track);
    			}
    		break;
    		case msg_rplay:
    			if (!dfp_pause) {
    				dfp_pause = true;
    				DFP_pause();
    				//
    				sprintf(buf, " Pause ");
    				mkLineCenter(buf, FONT_WIDTH);
    				i2c_ssd1306_text_xy(buf, 1, 8, true);
    				//i2c_ssd1306_shift(8, OLED_CMD_SHIFT_START);
    				//
    			} else {
    				dfp_pause = false;
    				DFP_unpause();
    				//
    				//i2c_ssd1306_shift(8, OLED_CMD_SHIFT_STOP);
    				tmr_get_trk = get_tmr10(_10ms);
    			}
    		break;
    		case msg_stop:
    			DFP_stop();
    			dfp_pause = true;
    			//
    			strcpy(buf, " Stop ");
    			mkLineCenter(buf, FONT_WIDTH);
    			i2c_ssd1306_text_xy(buf, 1, 8, true);
    		break;
    		case msg_back:
    			if (dfp_folder > 0) {
    				if (dfp_folder_trk <= 1) {
    					dfp_folder--;
    					if (dfp_folder > 0) {
    						if (!dfp_folder) dfp_folder = dfp_folders;
    						dfp_folder_trk = 1;
    						if (all_folder) {
    							folder_t *fold = all_folder + dfp_folder - 1;
    							dfp_folder_trk = 1;//fold->tracks;
    							dfp_folder_tracks = fold->tracks;
    						}
    					} else  {
    						dfp_folder = 0;
    						dfp_track = 1;
    					}
    					putMsg(msg_play);
    					break;
    				} else dfp_folder_trk--;
    			}/* else if (!dfp_folder) {
    				if (dfp_track <= 1) {
    					if (dfp_folders) {
    						dfp_folder = dfp_folders;
    						dfp_folder_trk = 1;
    						if (all_folder) {
    							folder_t *fold = all_folder + dfp_folder - 1;
    							dfp_folder_trk = 1;
    							dfp_folder_tracks = fold->tracks;
    						}
    					}
    				}
    			}*/
    			DFP_play_previous();
    			putMsg(msg_track);
    		break;
    		case msg_fwd:
    			if (dfp_folder > 0) {
    				if (dfp_folder_trk >= dfp_folder_tracks) {
    					dfp_folder++;
    					if (dfp_folder > dfp_folders) dfp_folder = 0;//1;
    					if (dfp_folder > 0) {
    						dfp_folder_trk = 1;
    						if (all_folder) {
    							folder_t *fold = all_folder + dfp_folder - 1;
    							dfp_folder_tracks = fold->tracks;
    						}
    					} else {
    						dfp_folder = 0;
    						dfp_track = 1;
    					}
    					putMsg(msg_play);
    					break;
    				} else dfp_folder_trk++;
    			}/* else if (!dfp_folder) {
    				//
    				if (dfp_track >= dfp_all_tracks) {
    					if (dfp_folders > 0) {
    						dfp_folder = 1;
    						dfp_folder_trk = 1;
    						if (all_folder) {
    							folder_t *fold = all_folder + dfp_folder - 1;
    							dfp_folder_tracks = fold->tracks;
    						}
    					}
    				}
    				//
    			}*/
    			DFP_play_next();
    			putMsg(msg_track);
    		break;
    		case msg_eqSet:
    			eqClass++;
    			if (eqClass > EQ_Bass) eqClass = EQ_Normal;
    			DFP_set_eq(eqClass);
    			//
    			sprintf(buf, "Eq: %s ", eqName[eqClass]);
    			mkLineCenter(buf, FONT_WIDTH);
    			i2c_ssd1306_text_xy(buf, 1, 7, false);
    		break;
    		case msg_eqGet:
    		    DFP_get_eq();
    		break;
    		case msg_volUp:
    			DFP_volumeUP();
    			if (dfp_volume < DFPLAYER_MAX_VOLUME) dfp_volume++;
    			//
    			sprintf(buf, "Volume: %d ", dfp_volume);
    			mkLineCenter(buf, FONT_WIDTH);
    			i2c_ssd1306_text_xy(buf, 1, 6, false);
    		break;
    		case msg_volDown:
    			DFP_volumeDOWN();
    			if (dfp_volume > 0) dfp_volume--;
    			//
    			sprintf(buf, "Volume: %d ", dfp_volume);
    			mkLineCenter(buf, FONT_WIDTH);
    			i2c_ssd1306_text_xy(buf, 1, 6, false);
    		break;
    		case msg_volGet:
    			DFP_get_volume();
    		break;
    		case msg_track:
    			if (!tmr_get_trk) tmr_get_trk = get_tmr10(_500ms);
    		break;
    		case msg_folders:
    			DFP_get_folders();
    		break;
    		case msg_getAllFolders:
    			if ((curFolder > 0) && (curFolder <= dfp_folders)) {
    				DFP_get_tracks(curFolder);
    			} else {
    				dfp_folder = all_folder->number;
					dfp_folder_tracks = all_folder->tracks;
					dfp_folder_trk = 1;

    				putMsg(msg_tail);
    			}
    		break;
    		case msg_tail:
    			if (!(devError & devDFP)) {
    				DFP_set_storage(idDev);//if (idDev) putMsg(msg_playDev);
    				if (startOne) putMsg(msg_volGet);
    			}
    		break;
    		case msg_showFolder:
    			if (dfp_folder > 0) {
    				sprintf(line_folder, "Folder: %u.%u", dfp_folder, dfp_folder_tracks);
    			} else {
    				sprintf(line_folder, "Folder: root");
    			}
    			mkLineCenter(line_folder, FONT_WIDTH);
    			i2c_ssd1306_text_xy(line_folder, 1, 5, false);
    		break;
    		case msg_getAllTracks:
    			DFP_get_track();
    		break;
    		case msg_dfpRX:
    		{
#ifdef DFP_DUBUG
    			stx[0] = '\0';//memset(stx, 0, sizeof(stx));
    			for (int i = 0; i < dfp_ACK_len; i++) sprintf(stx+strlen(stx), " %02X", *(dfp_ACK + i));
    			Report("msg_dfpRX", true, "%s\n", stx);
#endif
    			cmd_t *ack = (cmd_t *)&dfp_ACK[0];
    			dfpCmd = ack->ccode;
    			bool ys = false;
    			bool inv = false;
    			switch (dfpCmd) {
    				case DFPLAYER_QUERY_SD_FILES:
    					dfp_all_tracks = ack->par2;
    					lnum = 4;
    					//sprintf(buf, "Total trk: %u ", dfp_all_tracks);
    					sprintf(buf, "Storage: %s %u", storageName[idDev], dfp_all_tracks);
    					ys = true;
    					//
    				break;
    				case DFPLAYER_QUERY_STORAGE_DEV:
    					lnum = 4;
    					idDev = ack->par2;
    					if (idDev >= DFPLAYER_MAX_STORAGES) idDev = 0;
    					sprintf(buf, "Storage: %s ", storageName[idDev]);//ack->par2);
    					ys = true;
    					//
    					putMsg(msg_folders);
    					//
    				break;
    				case DFPLAYER_QUERY_FOLDER_FILES:
    					dfp_folder_tracks = ack->par2;
    					sprintf(line_folder, "Folder: %u.%u", curFolder, dfp_folder_tracks);
    					lnum = 5;
    					ys = true;
    					if (all_folder && (dfp_folders > 0)) {
    						folder_t *fold = all_folder + curFolder - 1;
    						fold->number = curFolder;
    						fold->tracks = dfp_folder_tracks;
    						//
#ifdef DFP_DUBUG
    						Report(NULL, true, "Folder %d with %d tracks\n", fold->number, fold->tracks);
#endif
    					}
    					//
    					curFolder++;
    					putMsg(msg_getAllFolders);//msg_play);//msg_track);
					break;
    				case DFPLAYER_QUERY_FOLDERS:
    					dfp_folders = ack->par2;
    					sprintf(line_folder, "Folders: %u ", dfp_folders);
    					lnum = 5;
    					ys = true;
    					//
#ifdef DFP_DUBUG
    					Report(NULL, true, "Total Folders : %d + root\n", dfp_folders);
#endif
    					if (dfp_folders > 0) {
    						if (!all_folder) {
    							all_folder = (folder_t *)calloc(dfp_folders, sizeof(folder_t));
    							if (all_folder) {
    								devError &= ~devMem;
    								putMsg(msg_getAllFolders);
    							} else {
    								devError |= devMem;
    							}
    						}
    					} else {
    						putMsg(msg_tail);
    					}
    				break;
    				case DFPLAYER_QUERY_TRACK_END:
    				case DFPLAYER_QUERY_UTRACK_END:
    					if (!dfp_tmr_next) dfp_tmr_next = get_tmr10(_2s); //putMsg(msg_fwd);//goto play next track
    				break;
    				case DFPLAYER_QUERY_VOLUME:
    					lnum = 6;
    					dfp_volume = ack->par2;
    					sprintf(buf, "Volume: %d ", dfp_volume);
    					ys = true;
    					if (startOne) {
    						putMsg(msg_eqGet);
    						startOne = false;
    					}
    				break;
    				case DFPLAYER_QUERY_EQ:
    					lnum = 7;
    					eqClass = ack->par2;
    					sprintf(buf, "Eq: %s ", eqName[eqClass]);
    					ys = true;
    					putMsg(msg_getAllTracks);
    				break;
    				case DFPLAYER_QUERY_SD_TRACK:
    					lnum = 8;
    					inv = true;
    					dfp_track = ((ack->par1 << 8) | ack->par2);// & 0xfff;
    					if (dfp_folder > 0)
    						sprintf(buf, "Trk: %d.%d.%d ", dfp_folder, dfp_track, dfp_folder_trk);
    					else
    						sprintf(buf, "Trk:  .%d ", dfp_track);
    					ys = true;
    				break;
    				case DFPLAYER_QUERY_ERROR:// Returned data of errors
    				{
    					uint8_t byte = ack->par2;
    					if (byte) devError |= devDFP;// ack->par2 - error code
    					lnum = 8;
    					if (byte >= DFPLAYER_MAX_ERROR) byte = DFPLAYER_MAX_ERROR - 2;
    					sprintf(buf, "ERR: %s ", errName[byte]);
    					ys = true;
    				}
    				break;
    			}
    			if (ys) {
    				char *uki = buf;
    				if (lnum == 5) uki = line_folder;
    				mkLineCenter(uki, FONT_WIDTH);
    				i2c_ssd1306_text_xy(uki, 1, lnum, inv);
    			}
    		}
    		break;
#endif
#ifdef SET_BLE
    		case msg_bleRx:// MacAddr=0x0000100014a,Name=Sabbat X12 Pro //mac=0000100014a name=Sabbat X12 Pro
    		{
    			char *uks = NULL, *uke = NULL;
    			int dl = 0;
    			bool cli_add = false, inv = false;
    			strcpy(bleTmp, BleBuf);
    			evt_t e = msg_none;
    			//
    			if ((uks = strstr(bleTmp, "income_opid:"))) {//income_opid:4b // income_opid:cb
    				uks += 12;
    				uint8_t val = (uint8_t)strtol(uks, NULL, 16);
    				if (!ble_CtlInd) {
    					ble_CtlCode = val;
    					ble_CtlCode <<= 8;
    				} else {
    					ble_CtlCode |= val;
    					parseCtl(ble_CtlCode);//goto action : play, pause, next/back track
    				}
    				ble_CtlInd++;
    				ble_CtlInd &= 1;
    			} else if (strstr(bleTmp, "OK+")) {
    				if (ble_status == bleCONADDR) tmr_ble_con = get_tmr(10);
    			} else if (strstr(bleTmp, "ERR")) {
    				if (ble_status == bleCONADDR) {
    					tmr_ble_con = 0;
    					e = msg_bleCmd;//send rst
    					ble_status = bleDISCADDR;
    				}
    			} else if (strstr(bleTmp, "ALL Devices=0")) {
    				memset((uint8_t *)&ble_client, 0, sizeof(ble_client_t));
    			} else if (strstr(bleTmp, "OK+REST")) {
    				ble_status = bleCON;
    			} else if (strstr(bleTmp, "POWER ON")) {
    				e = msg_bleCmd;//send rst
    				ble_status = bleSCAN;
    				//blePin();
    			} else if (strstr(bleTmp, "OK+DISCON")) {
    				blePin();
    				ble_status = bleSCAN;//send scan
    				e = msg_bleCmd;
    			} else if (strstr(bleTmp, "New Devices")) {
    				//
    				//i2c_ssd1306_shift(2, OLED_CMD_SHIFT_STOP);
    				//
    				if ((uks = strstr(bleTmp, "Name"))) {
    					uks += 4;
    					if ((*uks == '=') || (*uks == ':')) uks++;
    					dl = strlen(uks);
    					uke = strchr(uks, '\n');
    					if (uke && dl) dl--;//*uke = '\0';
    					if (dl > sizeof(ble_client.name) - 1) dl = sizeof(ble_client.name) - 1;
    					memset(ble_client.name, 0, sizeof(ble_client.name));
    					strncpy(ble_client.name, uks, dl);
    					cli_add = true;
    				}
    				//
    				if ((uks = strstr(bleTmp, "MacAdd"))) {
    					uks += 7;
    					uke = strstr(bleTmp, "0x");
    					if (uke) {
    						uks = uke;
    						uks += 2;
    						uke = strchr(uks, ',');
    						if (uke) {
    							dl = uke - uks;
    							if (dl > (sizeof(ble_client.mac) - 1)) dl = sizeof(ble_client.mac) - 1;
    							memset(ble_client.mac, 0, sizeof(ble_client.mac));
    							ble_client.mac[0] = '0';
    							if (dl == 11) strncpy(&ble_client.mac[1], uks, dl);
    							         else strncpy(&ble_client.mac[0], uks, dl);
    				    	}
    					}
    				}
    				//BLE_CONN_UP();
    				//
    				if (ble_checkName(ble_client.name)) {
    					if (!tmr_ble_con && (ble_status != bleCON)) {
    						ble_status = bleRDY;//send connect
    						e = msg_bleCmd;
    					}
    				}
    				//
    			} else if (strstr(bleTmp, "CONNECTED")) {
    				tmr_ble_con = 0;
    				ble_client.con = 0x81;
    				strcpy(ble_str, ble_client.name);
    				ble_status = bleCON;
    				inv = true;
    			} else if (strstr(bleTmp, "DISCONNECT")) {
    				BLE_CONN_UP();
    				//
    				memset((uint8_t *)&ble_client, 0, sizeof(ble_client_t));
    				ble_client.con = 0x80;
    				strcpy(ble_str, "BLE cli discon.");
    				ble_status = bleSCAN;//send scan
    				e = msg_bleCmd;
    				inv = false;
    				//
    				BLE_CONN_UP();
    				//
    			} else ble_client.con = 0;
    			//
    			if (e != msg_none) putMsg(e);
    			//
    			if (ble_client.con & 0x80) {// con/discon event !
    				ble_client.con = 0;
    				mkLineCenter(ble_str, FONT_WIDTH);
    				i2c_ssd1306_text_xy(ble_str, 1, 2, inv);
    				//sprintf(bleTmp+strlen(bleTmp), "\n\tmac=%s name=%s ctlCode=0x%04X", ble_client.mac, ble_client.name, ble_CtlCode);

    			}
    			//

    			sprintf(bleTmp+strlen(bleTmp), "\n\tmac=%s name=%s ctlCode=0x%04X", ble_client.mac, ble_client.name, ble_CtlCode);
    			Report(TAG_BLE, true, "%s\n", bleTmp);

    			//
    			if (cli_add) {
    				ble_index = ble_addRec(&ble_client);
    				if (ble_index >= 0) {
    					strcpy(ble_str, ble_client.name);
    					mkLineCenter(ble_str, FONT_WIDTH);
    					i2c_ssd1306_text_xy(ble_str, 1, 2, false);
    					//i2c_ssd1306_shift(2, OLED_CMD_SHIFT_STOP);

    					Report(TAG_BLE, true, "Cli #%u add OK\n", ble_index);
    					//
    					//i2c_ssd1306_shift(2, OLED_CMD_SHIFT_START);

    					putMsg(msg_bleList);
    				} else {
    					Report(TAG_BLE, true, "Cli already present\n");
    				}
    			}
    			//
    			//memset((uint8_t *)&ble_client, 0, sizeof(ble_client_t));
    			//
    		}
    		break;
    		case msg_bleCmd:
    			ble_TxBuf[0] = '\0';
    			switch (ble_status) {
    				case bleON:
    				case bleRST:
    					strcpy(ble_TxBuf, "AT+REST");
    				break;
    				case bleSCAN:
    					//
    					BLE_CONN_DOWN();
    					//
    					strcpy(ble_TxBuf, "AT+SCAN");
    				break;
    				case bleRDY:
    					sprintf(ble_TxBuf, "AT+CONADD=0x%s", ble_client.mac);// MacAddr=0x0000100014a,Name=Sabbat X12 Pro
    					ble_status = bleCONADDR;
    				break;
    				case bleCON:
    				break;
    				case bleDISCADDR:
    					strcpy(ble_TxBuf, "AT+DISCON");
    				break;
    			}
    			if (strlen(ble_TxBuf)) bleWrite(ble_TxBuf, true);
    		break;
    		case msg_bleList:
    			blePin();
    			//
    			ble_prnRec();
    		break;
    		case msg_bleCliSave:
    			//i2c_ssd1306_shift(2, OLED_CMD_SHIFT_STOP);
    			if (strlen(ble_client.name)) {
    				strcpy(ble_str, ble_client.name);
    				mkLineCenter(ble_str, FONT_WIDTH);
    				i2c_ssd1306_text_xy(ble_str, 1, 2, false);
    				if (!ble_add_names(ble_client.name)) {
    					sprintf(buf, "Client '%s' add to enable list\n", ble_client.name);
    					putMsg(msg_bleEnableList);
    				} else {
    					sprintf(buf, "Client '%s' already present in enable list\n", ble_client.name);
    				}
    				Report(TAG_BLE, true, "%s\n", buf);
    				ble_prnEnableList();
    			} else {
    				i2c_ssd1306_clear_line(2);
    			}
    		break;
    		case msg_bleEnableList:
    			//
    			//	update ble_enable_list : write cli_name[0]..cli_name[total_cli - 1] to flash (sector 0 page 0)
	#ifdef SET_W25FLASH
    			pageUpdate(page_num);
	#endif
    			//
    		break;
    		case msg_spiRxDone:
    		{
	#ifdef SET_W25FLASH
    			Report(NULL, true, "Read page # %u in DMA mode:\n", page_num);
    			memcpy(pageBufRx, pageTmp + PAGE_HDR_BYTES, w25qxx.PageSize);
    			for (uint32_t i = 0; i < w25qxx.PageSize; i++) {
    				if ((i % 16 == 0) && (i > 2)) Report(NULL, false, "\r\n");
    				Report(NULL, false, "0x%02X,", pageBufRx[i]);
    			}
    			Report(NULL, false, "\r\n");
    			str_name_t *ones = (str_name_t *)pageBufRx;
    			uint8_t tcli = 255;
    			while (++tcli < (w25qxx.PageSize / sizeof(str_name_t))) {
    				if (ones->name[0] == 0xff) break;
    				ones++;
    			}
    			total_cli = tcli;
    			memcpy(&cli_name[0].name[0], (char *)pageBufRx, total_cli * sizeof(str_name_t));
    			ble_prnEnableList();
	#endif
    		}
    		case msg_spiTxDone:
    		{
	#ifdef SET_W25FLASH
    			Report(NULL, true, "Write page # %u in DMA mode:\n", page_num);
    			for (uint32_t i = 0; i < w25qxx.PageSize; i++) {
    				if ((i % 16 == 0) && (i > 2)) Report(NULL, false, "\r\n");
    				Report(NULL, false, "0x%02X,", pageBufTx[i]);
    			}
    			Report(NULL, false, "\r\n");
	#endif
    		}
#endif
    		case msg_sec:
    		{
    			uint32_t cur_sec = get_tmr(0);

#ifdef SET_OLED_I2C
    			sec_to_str_time(cur_sec, buf);
    			i2c_ssd1306_text_xy(mkLineCenter(buf, FONT_WIDTH), 1, 1, true);
    			//
#ifdef SET_BLE
    			if (devError) {
#endif
    				sprintf(buf, "Fifo:%u %u Err:%X", cnt_evt, max_evt, devError);
    				i2c_ssd1306_text_xy(buf, 1, 2, false);
#ifdef SET_BLE
    			}
#endif

#endif

    			if (devError) errLedOn(NULL);

    			if (cur_sec >= (uint32_t)epoch) {
    				if (cur_sec == last_sec)
    					set_Date((time_t)(cur_sec));
    				else
    					last_sec = cur_sec;
    			}
    		}
    		break;
    	}

#ifdef SET_BLE
    	if (tmr_ble_con) {
    		if (check_tmr(tmr_ble_con)) {
    			tmr_ble_con = 0;
    			if (ble_status != bleCON) {
    				ble_status = bleON;//bleDISCADDR;
    				putMsg(msg_bleCmd);
    			}
    		}
    	}
#endif

#ifdef SET_DFPLAYER
    	if (dfp_tmr_next) {
    		if (check_tmr10(dfp_tmr_next)) {
    			dfp_tmr_next = 0;
    			putMsg(msg_fwd);//goto play next track
    		}
    	}

    	if (tmr_get_trk) {
    		if (check_tmr10(tmr_get_trk)) {
    			tmr_get_trk = 0;
    			if (!(devError & devDFP)) {
    				if (dfp_folder <= 0) DFP_get_playing();
    								else DFP_get_folder_playing(dfp_folder);
    			}
    		}
    	}

    	if (tmr_newDir) {
    		if (check_tmr(tmr_newDir)) {
    			if (dfp_folder > 0) {
    				sprintf(line_folder, "Folder: %u", dfp_folder);
    			} else {
    				sprintf(line_folder, "Folder: root");
    			}
    			inv_newDir = false;
    			newDir = false;
    			tmr_newDir = 0;
    			putMsg(msg_newDirSel);
    		}
    	}
#endif


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
#ifdef DFP_DUBUG
    HAL_Delay(200);
    Report(NULL, true, "Restart...\n");
#endif
    HAL_Delay(800);
    NVIC_SystemReset();

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 49;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 41999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLE_CONN_GPIO_Port, BLE_CONN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_ERROR_Pin|LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : KBD_INT_Pin */
  GPIO_InitStruct.Pin = KBD_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KBD_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_NSS_Pin */
  GPIO_InitStruct.Pin = SPI1_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPI1_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BLE_CONN_Pin LED_ERROR_Pin */
  GPIO_InitStruct.Pin = BLE_CONN_Pin|LED_ERROR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : IRED_Pin */
  GPIO_InitStruct.Pin = IRED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IRED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */

//-----------------------------------------------------------------------------------------
// set LED_ERROR when error on and send message to UART1 (in from != NULL)
//     from - name of function where error location
void errLedOn(const char *from)
{
	HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_SET);//LED OFF
	HAL_Delay(25);
	HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_RESET);//LED ON

	if (from) Report(NULL, true, "Error in function '%s'\r\n", from);
}
//------------------------------------------------------------------------------------------
void set_Date(time_t epoch)
{
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;
struct tm ts;

	gmtime_r(&epoch, &ts);

	sDate.WeekDay = ts.tm_wday;
	sDate.Month   = ts.tm_mon + 1;
	sDate.Date    = ts.tm_mday;
	sDate.Year    = ts.tm_year;
	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) devError |= devRTC;//errLedOn(__func__);
	else {
		sTime.Hours   = ts.tm_hour + tZone;
		sTime.Minutes = ts.tm_min;
		sTime.Seconds = ts.tm_sec;
		if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) devError |= devRTC;//errLedOn(__func__);
		else {
			setDate = true;
		}
	}
}
//------------------------------------------------------------------------------------------

uint32_t get_Date()
{
	if (!setDate) return get_tmr(0);

	struct tm ts;

	RTC_TimeTypeDef sTime = {0};
	if (HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) return get_tmr(0);
	ts.tm_hour = sTime.Hours;
	ts.tm_min  = sTime.Minutes;
	ts.tm_sec  = sTime.Seconds;

	RTC_DateTypeDef sDate = {0};
	if (HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) return get_tmr(0);
	ts.tm_wday = sDate.WeekDay;
	ts.tm_mon  = sDate.Month - 1;
	ts.tm_mday = sDate.Date;
	ts.tm_year = sDate.Year;

	return ((uint32_t)mktime(&ts));
}
//-----------------------------------------------------------------------------
int sec_to_str_time(uint32_t sec, char *stx)
{
int ret = 0;

	if (!setDate) {//no valid date in RTC
		uint32_t day = sec / (60 * 60 * 24);
		sec %= (60 * 60 * 24);
		uint32_t hour = sec / (60 * 60);
		sec %= (60 * 60);
		uint32_t min = sec / (60);
		sec %= 60;
		ret = sprintf(stx, "%lu.%02lu:%02lu:%02lu", day, hour, min, sec);
	} else {//in RTC valid date (epoch time)
		RTC_TimeTypeDef sTime;
		RTC_DateTypeDef sDate;
		if (HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) devError |= devRTC;//errLedOn(__func__);
		else {
			if (HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) devError |= devRTC;////errLedOn(__func__);
			else {
				ret = sprintf(stx, "%02u.%02u %02u:%02u:%02u",
								   sDate.Date, sDate.Month,
								   sTime.Hours, sTime.Minutes, sTime.Seconds);
			}
		}
	}

	return ret;
}
//-----------------------------------------------------------------------------
uint32_t get_secCounter()
{
	return secCounter;
}
//-----------------------------------------------------------------------------
void inc_secCounter()
{
	secCounter++;
}
//-----------------------------------------------------------------------------
uint32_t get_hsCounter()
{
	return HalfSecCounter;
}
//-----------------------------------------------------------------------------
void inc_hsCounter()
{
	HalfSecCounter++;
}
//------------------------------------------------------------------------------------------
uint32_t get_tmr10(uint32_t ms)
{
	return (get_hsCounter() + ms);
}
//------------------------------------------------------------------------------------------
bool check_tmr10(uint32_t ms)
{
	return (get_hsCounter() >= ms ? true : false);
}
//------------------------------------------------------------------------------------------
uint32_t get_tmr(uint32_t sec)
{
	return (get_secCounter() + sec);
}
//------------------------------------------------------------------------------------------
bool check_tmr(uint32_t sec)
{
	return (get_secCounter() >= sec ? true : false);
}
//------------------------------------------------------------------------------------------
uint64_t get_hstmr(uint64_t hs)
{
	return (get_hsCounter() + hs);
}
//------------------------------------------------------------------------------------------
bool check_hstmr(uint64_t hs)
{
	return (get_hsCounter() >= hs ? true : false);
}
//----------------------------------------------------------------------------------------
int sec_to_string(uint32_t sec, char *stx)
{
int ret = 0;

	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
	if (HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) devError |= devRTC;//errLedOn(__func__);
	else {
		if (HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) devError |= devRTC;//errLedOn(__func__);
		else {
			ret = sprintf(stx, "%02u.%02u %02u:%02u:%02u ",
							sDate.Date, sDate.Month,
							sTime.Hours, sTime.Minutes, sTime.Seconds);
		}
	}

    return ret;
}
//------------------------------------------------------------------------------------------
uint8_t Report(const char *tag, bool addTime, const char *fmt, ...)
{
va_list args;
size_t len = MAX_UART_BUF;
int dl = 0;


	/*if (!uartRdy) {
		HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_SET);//ON err_led
		return 1;
	} else {
		HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_RESET);//OFF err_led
	}*/

#ifdef SET_STATIC_MEM
	char *buff = &PrnBuf[0];
	buff[0] = 0;
#else
	char *buff = (char *)calloc(1, len);
	if (buff) {
#endif
		if (addTime) {
			uint32_t ep;
			if (!setDate) ep = get_secCounter();
					 else ep = extDate;
			dl = sec_to_string(ep, buff);
		}
		if (tag) dl += sprintf(buff+strlen(buff), "[%s] ", tag);
		va_start(args, fmt);
		vsnprintf(buff + dl, len - dl, fmt, args);
		uartRdy = 0;
		//er = HAL_UART_Transmit(&huart1, (uint8_t *)buff, strlen(buff), 1000);
		if (HAL_UART_Transmit_DMA(&huart1, (uint8_t *)buff, strlen(buff)) != HAL_OK) devError |= devUART;
		/**/
		while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY) {
			if (HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_RX) break;
			HAL_Delay(1);
		}
		/**/
		va_end(args);
#ifndef SET_STATIC_MEM
		free(buff);
	}
#endif

	return 0;
}
//------------------------------------------------------------------------------------------
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1) uartRdy = 1;
	else
	if (huart->Instance == USART6) dfpRdy = 1;
}
//------------------------------------------------------------------------------------------
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1) {
		RxBuf[rx_uk & 0xff] = (char)uRxByte;
		if (uRxByte == 0x0a) {//end of line
			char *uk = strstr(RxBuf, _extDate);//const char *_extDate = "epoch=";
			if (uk) {
				uk += strlen(_extDate);
				if (*uk != '?') {
					if (strlen(uk) < 10) setDate = false;
					else {
						char *uke = strchr(uk, ':');
						if (uke) {
							tZone = atoi(uke + 1);
							*uke = '\0';
						} else tZone = 0;
						extDate = atol(uk);
						set_Date((time_t)extDate);
					}
				} else setDate = true;
			} else if (strstr(RxBuf, _restart)) {//const char *_restart = "restart";
				if (!restart_flag) restart_flag = 1;
			}
			rx_uk = 0;
			memset(RxBuf, 0, sizeof(RxBuf));
		} else rx_uk++;

		HAL_UART_Receive_IT(huart, (uint8_t *)&uRxByte, 1);
	}
#ifdef SET_DFPLAYER
	else if (huart->Instance == USART6) {
		if (dfp_uRxByte == DFPLAYER_START_BYTE) dfp_Begin = true;
		if (dfp_Begin) {
			dfp_RxBuf[dfp_rx_uk & 0x3f] = dfp_uRxByte;
			dfp_rx_uk++;
		}
		if (dfp_uRxByte == DFPLAYER_END_BYTE) {//end of pack
			memcpy(dfp_ACK, dfp_RxBuf, dfp_rx_uk);
			dfp_ACK_len = dfp_rx_uk;
			dfp_rx_uk = 0;
			dfp_Begin = false;
			memset(dfp_RxBuf, 0, sizeof(dfp_RxBuf));
			putMsg(msg_dfpRX);
		}

		HAL_UART_Receive_IT(huart, (uint8_t *)&dfp_uRxByte, 1);
	}
#endif
#ifdef SET_BLE
	else if (huart->Instance == USART2) {
		if ((ble_uRxByte >= 0x0a) && (ble_uRxByte <= 0x7f)) {
			ble_RxBuf[ble_rx_uk++] = (char)ble_uRxByte;
			ble_rx_uk &= 0xff;
			char *uki = NULL;
			if ((uki = strstr(ble_RxBuf, "\r\r\n"))) {
				*uki = '\0';
				memset(BleBuf, 0, sizeof(BleBuf));
				strncpy(BleBuf, ble_RxBuf, strlen(ble_RxBuf));

				putMsg(msg_bleRx);

				ble_rx_uk = 0;
				memset(ble_RxBuf, 0, sizeof(ble_RxBuf));
			}
		}

		HAL_UART_Receive_IT(huart, (uint8_t *)&ble_uRxByte, 1);
	}
#endif

}
//-------------------------------------------------------------------------------------------
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if (htim->Instance == TIM3) {//one call in _10ms

		HalfSecCounter++;//+10ms

		//if (!(HalfSecCounter % _500ms)) putMsg(msg_500ms);

		if (!(HalfSecCounter % _1s)) {//seconda
			secCounter++;
			//HalfSecCounter = 0;
			//HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);//set ON/OFF LED

			if (screenON) {
				HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);//set ON/OFF LED1
			} else {
				if (HAL_GPIO_ReadPin(LED_GPIO_Port, LED_Pin) == GPIO_PIN_SET)
					HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
			}

			putMsg(msg_sec);
		}
	}
#ifdef SET_IRED
	else if (htim->Instance == TIM2) {
		uint8_t irdata = RECIV_PIN; // пин для приёма
		irparams.timer++;  // One more 50uS tick
		if (irparams.rawlen >= RAWBUF) irparams.rcvstate = STATE_OVERFLOW;  // Buffer overflow

		switch (irparams.rcvstate) {
			case STATE_IDLE: // In the middle of a gap
				if (irdata == MARK) {
					if (irparams.timer < GAP_TICKS) { // Not big enough to be a gap.
						irparams.timer = 0;
					} else {
						// Gap just ended; Record duration; Start recording transmission
						irparams.overflow = 0;
						irparams.rawlen  = 0;
						irparams.rawbuf[irparams.rawlen++] = irparams.timer;
						irparams.timer = 0;
						irparams.rcvstate = STATE_MARK;
					}
				}
			break;
			case STATE_MARK:  // Timing Mark
				if (irdata == SPACE) {// Mark ended; Record time
					irparams.rawbuf[irparams.rawlen++] = irparams.timer;
					irparams.timer = 0;
					irparams.rcvstate = STATE_SPACE;
				}
			break;
			case STATE_SPACE:  // Timing Space
				if (irdata == MARK) {// Space just ended; Record time
					irparams.rawbuf[irparams.rawlen++] = irparams.timer;
					irparams.timer = 0;
					irparams.rcvstate = STATE_MARK;
				} else if (irparams.timer > GAP_TICKS) {// Space
					irparams.rcvstate = STATE_STOP;
				}
			break;
			case STATE_STOP:  // Waiting; Measuring Gap
			 	if (irdata == MARK) irparams.timer = 0;  // Reset gap timer
			break;
			case STATE_OVERFLOW:  // Flag up a read overflow; Stop the State Machine
				irparams.overflow = 1;
				irparams.rcvstate = STATE_STOP;
			break;
		}
		//
	}
#endif

}
//-------------------------------------------------------------------------------------------
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == SPI1) {//FLASH
		spi_cnt++;
		spiRdy = 1;
#ifdef SET_W25FLASH
		if (w25_withDMA) {
			W25_UNSELECT();
			putMsg(msg_spiTxDone);
		}
#endif
	}
}
//-------------------------------------------------------------------------------------------
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == SPI1) {//FLASH
		spi_cnt++;
		spiRdy = 1;
#ifdef SET_W25FLASH
		if (w25_withDMA) {
			W25_UNSELECT();
			putMsg(msg_spiRxDone);
		}
#endif
	}
}
//-------------------------------------------------------------------------------------------
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

#ifdef SET_KBD
	if (GPIO_Pin == KBD_INT_Pin) {
		if (kbdEnable) {
			kbdCode = kbd_get_touch();
			if ((kbdCode >= 0x23) && (kbdCode <= 0x39)) {// '#'...'9'
				kbdCnt++;
				putMsg(msg_kbd);
			}
		}
	}
#endif

}
//-------------------------------------------------------------------------------------------
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	if (hi2c->Instance == I2C1) devError |= devI2C;
	else
	if (hi2c->Instance == I2C2) devError |= devKBD;
}
/*
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if (hi2c->Instance == I2C1) {//from KBD
		kbdRdy = 1;
	} else if (hi2c->Instance == I2C2) {//from OLED
		//
	}
}
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
}
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
}
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
}
*/
//-------------------------------------------------------------------------------------------


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
