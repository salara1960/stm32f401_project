/*
 * dfplayer.h
 *
 *  Created on: May 2, 2021
 *      Author: alarm
 */

#ifndef INC_DFPLAYER_H_
#define INC_DFPLAYER_H_

#include "hdr.h"

#ifdef SET_DFPLAYER

//------------------------------------------------------------------------------------------

// DFPlayer Module Commands
#define DFPLAYER_PLAY_NEXT          0x01
#define DFPLAYER_PLAY_PREVIOUS      0x02
#define DFPLAYER_PLAY_ROOT          0x03  // 1-2999
#define DFPLAYER_INCREASE_VOLUME    0x04
#define DFPLAYER_DECREASE_VOLUME    0x05
#define DFPLAYER_SPECIFY_VOLUME     0x06
#define DFPLAYER_SPECIFY_EQ         0x07  // 0:Normal / 1:Pop / 2:Rock / 3:Jazz / 4:Classic / 5:Bass
#define DFPLAYER_SINGLE_REPEAT      0x08  // 1-2999
#define DFPLAYER_PLAYBACK_DEVICE    0x09  // 0:USB / 1:SD
#define DFPLAYER_STANDBY            0x0A
#define DFPLAYER_RESET              0x0C
#define DFPLAYER_UNPAUSE            0x0D
#define DFPLAYER_PAUSE              0x0E
#define DFPLAYER_PLAY_FOLDER        0x0F  // 01-99 folder and 1-255 track
#define DFPLAYER_ALL_REPEAT         0x11  // 0:Stop Repeat / 1:All Repeat
#define DFPLAYER_PLAY_MP3           0x12  // Play "/mp3" folder files
#define DFPLAYER_PLAY_AD            0x13  // Play "/advert" folder ads while saving current playback position
#define DFPLAYER_STOP_AD            0x15  // Stop playing ad and resume playback
#define DFPLAYER_STOP               0x16  // Stop Playback
#define DFPLAYER_FOLDER_REPEAT      0x17
#define DFPLAYER_PLAY_RANDOM        0x18
#define DFPLAYER_REPEAT_CURRENT     0x19

#define DFPLAYER_QUERY_STORAGE_DEV  0x3F //Query current online storage device
#define DFPLAYER_QUERY_TRACK_END    0x3D //end of play track (SD)
#define DFPLAYER_QUERY_UTRACK_END   0x3C //end of play track (USB)
#define DFPLAYER_QUERY_ERROR        0x40 //Returned data of errors
#define DFPLAYER_QUERY_ACK          0x41 //Module reports a feedback with this command
#define DFPLAYER_QUERY_STATUS       0x42
#define DFPLAYER_QUERY_VOLUME       0x43
#define DFPLAYER_QUERY_EQ           0x44
#define DFPLAYER_QUERY_USB_FILES    0x47
#define DFPLAYER_QUERY_SD_FILES     0x48
#define DFPLAYER_QUERY_USB_TRACK    0x4B
#define DFPLAYER_QUERY_SD_TRACK     0x4C
#define DFPLAYER_QUERY_FOLDER_FILES 0x4E
#define DFPLAYER_QUERY_FOLDERS      0x4F

#define DFPLAYER_START_BYTE   0x7E
#define DFPLAYER_VERSION_BYTE 0xFF
#define DFPLAYER_CMD_LEN_BYTE 0x06
#define DFPLAYER_NO_ACK_BYTE  0x00
#define DFPLAYER_ACK_BYTE     0x01
#define DFPLAYER_END_BYTE     0xEF
#define DFPLAYER_INIT_DELAY   250
#define DFPLAYER_CMD_DELAY    10
#define DFPLAYER_READ_TIMEOUT 250


#define DFPLAYER_MAX_VOLUME   30
#define DFPLAYER_MAX_EQ       6
#define DFPLAYER_MAX_STORAGES 5
#define DFPLAYER_MAX_ERROR    11




#define DFP_delay(x) HAL_Delay(x)

#define htons(x) \
    ((uint16_t)((x >> 8) | ((x << 8) & 0xff00)))
#define htonl(x) \
    ((uint32_t)((x >> 24) | ((x >> 8) & 0xff00) | ((x << 8) & 0xff0000) | ((x << 24) & 0xff000000)))


//------------------------------------------------------------------------------------------

enum {
  SM_Hardware = 0,
  SM_Software
};

enum {
  EQ_Normal = 0,
  EQ_Pop,
  EQ_Rock,
  EQ_Jazz,
  EQ_Classic,
  EQ_Bass
};



#pragma pack(push,1)
typedef struct {
	uint8_t start; //DFPLAYER_START_BYTE,
	uint8_t ver;//DFPLAYER_VERSION_BYTE,
	uint8_t len;//DFPLAYER_CMD_LEN_BYTE,
	uint8_t ccode;//command,
	uint8_t ack;//DFPLAYER_NO_ACK_BYTE,
	uint8_t par1;//param1,
	uint8_t par2;//param2,
	uint16_t crc;//			0,//(uint8_t)(checksum >> 8), 0,//(uint8_t)(checksum & 0xFF),
	uint8_t end;//DFPLAYER_END_BYTE
} cmd_t;
#pragma pack(pop)

//------------------------------------------------------------------------------------------

uint8_t dfp_withACK;
uint8_t dfp_withDMA;
const char *eqName[DFPLAYER_MAX_EQ];
const char *storageName[DFPLAYER_MAX_STORAGES];
const char *errName[DFPLAYER_MAX_ERROR];

//------------------------------------------------------------------------------------------

    void DFP_reset();
    void DFP_standby();
    void DFP_play();
    void DFP_stop();
    void DFP_play_next();
    void DFP_play_previous();
    void DFP_volumeUP();
    void DFP_volumeDOWN();
    void DFP_set_volume(int volume);
    int DFP_get_volume();
    void DFP_repeat(bool bRepeat);
    void DFP_play_root(int track, bool bRepeat);
    void DFP_play_folder(int folder, int track, bool bRepeat);
    int DFP_get_track();
    int DFP_get_tracks();
    int DFP_get_folders();
    void DFP_set_eq(int eq);
    int DFP_get_eq();
    int DFP_get_playing();
    int DFP_get_folder_playing(int folder);
    void DFP_set_storage(uint8_t storage);
    void DFP_pause();
    void DFP_unpause();

    /* Playback
    void DFP_play_mp30(int track);
    void DFP_play_mp3(int track, bool repeat);
    void DFP_play_ad(int track);
    void DFP_stop_ad();

    // Folders/Tracks
    int DFP_get_tracks();
    int DFP_get_tracks(int folder);
    */

//------------------------------------------------------------------------------------------

#endif


#endif /* INC_DFPLAYER_H_ */
