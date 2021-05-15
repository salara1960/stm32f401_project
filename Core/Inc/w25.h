#ifndef __W25_H__
#define __W25_H__

#include "hdr.h"

#ifdef SET_W25FLASH

#define W25QXX_DEBUG

#define W25QXX_DUMMY_BYTE 0xA5

#define W25qxx_Delay(delay) HAL_Delay(delay)
#define W25_SEL()   HAL_GPIO_WritePin(W25_CS_GPIO_Port, W25_CS_Pin, GPIO_PIN_RESET);//set to 0
#define W25_UNSEL() HAL_GPIO_WritePin(W25_CS_GPIO_Port, W25_CS_Pin, GPIO_PIN_SET);  //set to 1

#define PAGE_BUF_SIZE 256
#define PAGE_HDR_BYTES 4

typedef enum {
    W25Q10 = 1,
    W25Q20,
    W25Q40,
    W25Q80,
    W25Q16,
    W25Q32,
    W25Q64,
    W25Q128,
    W25Q256,
    W25Q512,
} W25QXX_ID_t;

#pragma pack(push,1)
typedef struct {
    W25QXX_ID_t ID;
    uint8_t UniqID[8];

    uint16_t PageSize;
    uint32_t PageCount;
    uint32_t SectorSize;
    uint32_t SectorCount;
    uint32_t BlockSize;
    uint32_t BlockCount;

    uint32_t CapacityInKiloByte;

    uint8_t StatusRegister1;
    uint8_t StatusRegister2;
    uint8_t StatusRegister3;

    uint8_t Lock;
} w25qxx_t;
#pragma pack(pop)


//------------------------------------------------------------------------------------------

w25qxx_t w25qxx;
//const char *all_chipID[];
//const uint32_t all_chipBLK[];
uint8_t w25_withDMA;
uint8_t pageTmp[PAGE_BUF_SIZE + PAGE_HDR_BYTES];

//------------------------------------------------------------------------------------------

void W25_SELECT();
void W25_UNSELECT();

bool W25qxx_Init(void);

void W25qxx_EraseChip(void);
void W25qxx_EraseSector(uint32_t SectorAddr);
void W25qxx_EraseBlock(uint32_t BlockAddr);

uint32_t W25qxx_PageToSector(uint32_t PageAddress);
uint32_t W25qxx_PageToBlock(uint32_t PageAddress);
uint32_t W25qxx_SectorToBlock(uint32_t SectorAddress);
uint32_t W25qxx_SectorToPage(uint32_t SectorAddress);
uint32_t W25qxx_BlockToPage(uint32_t BlockAddress);

bool W25qxx_IsEmptyPage(uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToCheck_up_to_PageSize);
bool W25qxx_IsEmptySector(uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToCheck_up_to_SectorSize);
bool W25qxx_IsEmptyBlock(uint32_t Block_Address, uint32_t OffsetInByte, uint32_t NumByteToCheck_up_to_BlockSize);

void W25qxx_WriteByte(uint8_t pBuffer, uint32_t Bytes_Address);
void W25qxx_ReadByte(uint8_t *pBuffer, uint32_t Bytes_Address);
void W25qxx_ReadBytes(uint8_t *pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead);

void W25qxx_WritePage(uint8_t *pBuffer, uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_PageSize);
void W25qxx_ReadPage(uint8_t *pBuffer, uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_PageSize);

void W25qxx_WriteSector(uint8_t *pBuffer, uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_SectorSize);
void W25qxx_ReadSector(uint8_t *pBuffer, uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_SectorSize);

void W25qxx_WriteBlock(uint8_t *pBuffer, uint32_t Block_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_BlockSize);
void W25qxx_ReadBlock(uint8_t *pBuffer, uint32_t Block_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_BlockSize);

//------------------------------------------------------------------------------------------


#endif

#endif /* __W25_H__ */
