/*
 * dfplayer.c
 *
 *  Created on: May 2, 2021
 *      Author: alarm
 */

#include "hdr.h"

#ifdef SET_DFPLAYER

#include "main.h"
#include "dfplayer.h"


//******************************************************************************************

uint8_t dfp_withDMA = 1;
uint32_t dfp_wait = 1000;
const char *eqName[DFPLAYER_MAX_EQ] = {"Normal", "Pop", "Rock", "Jazz", "Classic", "Bass"};
const char *storageName[DFPLAYER_MAX_STORAGES] = {"???", "USB", "SD", "USB+SD", "PC"};
const char *errName[DFPLAYER_MAX_ERROR] = {"OK", "BUSY", "SLEEP", "SERIAL", "CRC", "TRACK", "NOTRACK", "INSERT", "SD", "UNKNOWN", "SLMODE"};

#ifdef DFP_DUBUG
	char stz[128] = {0};
#endif

//-----------------------------------------------------------------------------------------
static char *cmdName(uint8_t command)
{
	switch (command) {
		case DFPLAYER_PLAY_NEXT://          0x01
			return "DFPLAYER_PLAY_NEXT";
		case DFPLAYER_PLAY_PREVIOUS://      0x02
			return "DFPLAYER_PLAY_PREVIOUS";
		case DFPLAYER_PLAY_ROOT://          0x03  // 1-2999
			return "DFPLAYER_PLAY_ROOT";
		case DFPLAYER_INCREASE_VOLUME://    0x04
			return "DFPLAYER_INCREASE_VOLUME";
		case DFPLAYER_DECREASE_VOLUME://    0x05
			return "DFPLAYER_DECREASE_VOLUME";
		case DFPLAYER_SPECIFY_VOLUME://     0x06
			return "DFPLAYER_SPECIFY_VOLUME";
		case DFPLAYER_SPECIFY_EQ://         0x07  // 0:Normal / 1:Pop / 2:Rock / 3:Jazz / 4:Classic / 5:Bass
			return "DFPLAYER_SPECIFY_EQ";
		case DFPLAYER_SINGLE_REPEAT://      0x08  // 1-2999
			return "DFPLAYER_SINGLE_REPEAT";
		case DFPLAYER_PLAYBACK_DEVICE://    0x09  // 0:USB / 1:SD
			return "DFPLAYER_PLAYBACK_DEVICE";
		case DFPLAYER_STANDBY://            0x0A
			return "DFPLAYER_STANDBY";
		case DFPLAYER_RESET://              0x0C
			return "DFPLAYER_RESET";
		case DFPLAYER_UNPAUSE://            0x0D
			return "DFPLAYER_UNPAUSE";
		case DFPLAYER_PAUSE://              0x0E
			return "DFPLAYER_PAUSE";
		case DFPLAYER_PLAY_FOLDER://        0x0F  // 01-99 folder and 1-255 track
			return "DFPLAYER_PLAY_FOLDER";
		case DFPLAYER_ALL_REPEAT://         0x11  // 0:Stop Repeat / 1:All Repeat
			return "DFPLAYER_ALL_REPEAT";
		case DFPLAYER_PLAY_MP3://           0x12  // Play "/mp3" folder files
			return "DFPLAYER_PLAY_MP3";
		case DFPLAYER_PLAY_AD://            0x13  // Play "/advert" folder ads while saving current playback position
			return "DFPLAYER_PLAY_AD";
		case DFPLAYER_STOP_AD://            0x15  // Stop playing ad and resume playback
			return "DFPLAYER_STOP_AD";
		case DFPLAYER_STOP://               0x16  // Stop Playback
			return "DFPLAYER_STOP";
		case DFPLAYER_FOLDER_REPEAT://      0x17
			return "DFPLAYER_FOLDER_REPEAT";
		case DFPLAYER_PLAY_RANDOM://        0x18
			return "DFPLAYER_PLAY_RANDOM";
		case DFPLAYER_REPEAT_CURRENT://     0x19
			return "DFPLAYER_REPEAT_CURRENT";
		case DFPLAYER_QUERY_STORAGE_DEV://  0x3F //Query current online storage device
			return "DFPLAYER_QUERY_STORAGE_DEV";
		case DFPLAYER_QUERY_TRACK_END://    0x3D //end of play track (SD)
			return "DFPLAYER_QUERY_TRACK_END";
		case DFPLAYER_QUERY_UTRACK_END://   0x3C //end of play track (USB)
			return "DFPLAYER_QUERY_UTRACK_END";
		case DFPLAYER_QUERY_ERROR://        0x40 //Returned data of errors
			return "DFPLAYER_QUERY_ERROR";
		case DFPLAYER_QUERY_ACK://          0x41 //Module reports a feedback with this command
			return "DFPLAYER_QUERY_ACK";
		case DFPLAYER_QUERY_STATUS://       0x42
			return "DFPLAYER_QUERY_STATUS";
		case DFPLAYER_QUERY_VOLUME://       0x43
			return "DFPLAYER_QUERY_VOLUME";
		case DFPLAYER_QUERY_EQ://           0x44
			return "DFPLAYER_QUERY_EQ";
		case DFPLAYER_QUERY_USB_FILES://    0x47
			return "DFPLAYER_QUERY_USB_FILES";
		case DFPLAYER_QUERY_SD_FILES://     0x48
			return "DFPLAYER_QUERY_SD_FILES";
		case DFPLAYER_QUERY_USB_TRACK://    0x4B
			return "DFPLAYER_QUERY_USB_TRACK";
		case DFPLAYER_QUERY_SD_TRACK://     0x4C
			return "DFPLAYER_QUERY_SD_TRACK";
		case DFPLAYER_QUERY_FOLDER_FILES:// 0x4E
			return "DFPLAYER_QUERY_FOLDER_FILES";
		case DFPLAYER_QUERY_FOLDERS://      0x4F
			return "DFPLAYER_QUERY_FOLDERS";
	}

	return "???";
}
//-----------------------------------------------------------------------------------------
uint16_t DFP_CRC_cmd(uint8_t *data, uint16_t len)
{
uint16_t crc = 0;
uint16_t i = 0xffff;

	while (++i < len) crc += *(data + i);

	return -crc;
}
//-----------------------------------------------------------------------------------------
void send_cmd(uint8_t command, uint8_t param1, uint8_t param2)
{
	cmd_t cmd = {
		.start = DFPLAYER_START_BYTE,
		.ver = DFPLAYER_VERSION_BYTE,
		.len = DFPLAYER_CMD_LEN_BYTE,
		.ccode = command,
		.ack = DFPLAYER_NO_ACK_BYTE,
		.par1 = param1,
		.par2 = param2,
		.crc = 0,// (st + ml) bytes		0,//(uint8_t)(checksum >> 8), 0,//(uint8_t)(checksum & 0xFF),
		.end = DFPLAYER_END_BYTE
	};
	cmd.crc = htons(DFP_CRC_cmd((uint8_t *)&cmd.ver, DFPLAYER_CMD_LEN_BYTE));
	uint8_t *uk = (uint8_t *)&cmd.start;

	uint16_t len = (uint16_t)sizeof(cmd_t);

	//dfpCmd = command;

	dfpRdy = 0;
	if (dfp_withDMA) {
		if (HAL_UART_Transmit_DMA(portDFP, uk, len) != HAL_OK) devError |= devDFP;
		while (HAL_UART_GetState(portDFP) != HAL_UART_STATE_READY) {
			if (HAL_UART_GetState(portDFP) == HAL_UART_STATE_BUSY_RX) break;
			HAL_Delay(1);
		}
	} else {
		if (HAL_UART_Transmit(portDFP, uk, len, dfp_wait) != HAL_OK) devError |= devDFP;
		dfpRdy = 1;
	}
#ifdef DFP_DUBUG
	stz[0] = '\0';//memset(stz, 0, sizeof(stz));
	for (int i = 0; i < len; i++) sprintf(stz+strlen(stz), " %02X", *(uk + i));
	Report(cmdName(cmd.ccode), true, "%s\n", stz);
#endif

}
//-----------------------------------------------------------------------------------------
int read_cmd(uint8_t command, uint8_t param1, uint8_t param2)
{
	send_cmd(command, param1, param2);

	// read data from device in interrup mode

	return 0;
}
//-----------------------------------------------------------------------------------------
void DFP_reset()
{
	send_cmd(DFPLAYER_RESET, 0 ,0);
}
//-----------------------------------------------------------------------------------------
void DFP_standby()
{
	send_cmd(DFPLAYER_STANDBY, 0, 0);
}
//-----------------------------------------------------------------------------------------
void DFP_volumeUP()
{
	send_cmd(DFPLAYER_INCREASE_VOLUME, 0, 0);
}
//-----------------------------------------------------------------------------------------
void DFP_volumeDOWN()
{
	send_cmd(DFPLAYER_DECREASE_VOLUME, 0, 0);
}
//-----------------------------------------------------------------------------------------
void DFP_set_volume(int volume)
{
	send_cmd(DFPLAYER_SPECIFY_VOLUME, 0, volume);
}
//-----------------------------------------------------------------------------------------
int DFP_get_volume()
{
	return read_cmd(DFPLAYER_QUERY_VOLUME, 0, 0);
}
//-----------------------------------------------------------------------------------------
void DFP_set_eq(int eq)
{
	send_cmd(DFPLAYER_SPECIFY_EQ, 0, eq);
}
//-----------------------------------------------------------------------------------------
int DFP_get_eq()
{
	return read_cmd(DFPLAYER_QUERY_EQ, 0, 0);
}
//-----------------------------------------------------------------------------------------
void DFP_stop()
{
	send_cmd(DFPLAYER_STOP, 0, 0);
}
//-----------------------------------------------------------------------------------------
void DFP_pause()
{
	send_cmd(DFPLAYER_PAUSE, 0, 0);
}
//-----------------------------------------------------------------------------------------
void DFP_unpause()
{
	send_cmd(DFPLAYER_UNPAUSE, 0, 0);
}
//-----------------------------------------------------------------------------------------
void DFP_play()
{
	send_cmd(DFPLAYER_UNPAUSE, 0, 0);
}
//-----------------------------------------------------------------------------------------
void DFP_play_next()
{
	send_cmd(DFPLAYER_PLAY_NEXT, 0, 0);
}
//-----------------------------------------------------------------------------------------
void DFP_play_previous()
{
	send_cmd(DFPLAYER_PLAY_PREVIOUS, 0, 0);
}
//-----------------------------------------------------------------------------------------
void DFP_repeat(bool bRepeat)
{
	send_cmd(DFPLAYER_REPEAT_CURRENT, 0, !bRepeat);
}
//-----------------------------------------------------------------------------------------
void DFP_play_root(int track, bool bRepeat)
{
	send_cmd(DFPLAYER_PLAY_ROOT, 0, track);

	if	(bRepeat) {
		DFP_delay(DFPLAYER_CMD_DELAY);
		DFP_repeat(true);
	}
}
//-----------------------------------------------------------------------------------------
void DFP_play_folder(int folder, int track, bool bRepeat)
{
	send_cmd(DFPLAYER_PLAY_FOLDER, folder, track);

	if (bRepeat) {
		DFP_delay(DFPLAYER_CMD_DELAY);
		DFP_repeat(true);
	}
}
//-----------------------------------------------------------------------------------------
void DFP_play_mp3(int track, bool bRepeat)
{
	send_cmd(DFPLAYER_PLAY_MP3, 0, track);

	if (bRepeat) {
		DFP_delay(DFPLAYER_CMD_DELAY);
		DFP_repeat(true);
	}
}
//-----------------------------------------------------------------------------------------
void DFP_play_ad(int track)
{
	send_cmd(DFPLAYER_PLAY_AD, 0, track);
}
//-----------------------------------------------------------------------------------------
void DFP_stop_ad()
{
	send_cmd(DFPLAYER_STOP_AD, 0, 0);
}
//-----------------------------------------------------------------------------------------
int DFP_get_folders()
{
	return read_cmd(DFPLAYER_QUERY_FOLDERS, 0, 0);
}
//-----------------------------------------------------------------------------------------
int DFP_get_track()
{
	return read_cmd(DFPLAYER_QUERY_SD_FILES, 0, 0);
}
//-----------------------------------------------------------------------------------------
int DFP_get_tracks(int folder)
{
	return read_cmd(DFPLAYER_QUERY_FOLDER_FILES, 0, folder);
}
//-----------------------------------------------------------------------------------------
int DFP_get_playing()
{
	return read_cmd(DFPLAYER_QUERY_SD_TRACK, 0, 0);
}
//-----------------------------------------------------------------------------------------
int DFP_get_folder_playing(int folder)
{
	return read_cmd(DFPLAYER_QUERY_SD_TRACK, folder, 0);
}
//-----------------------------------------------------------------------------------------
void DFP_set_storage(uint8_t storage)
{
	send_cmd(DFPLAYER_PLAYBACK_DEVICE, 0, storage);
}
//******************************************************************************************


#endif


