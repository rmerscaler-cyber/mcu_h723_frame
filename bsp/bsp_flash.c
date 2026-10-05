#include "bsp_flash.h"
#include "main.h"
#include "string.h"

/**
  * @brief          get the sector number of flash
  * @param[in]      address: flash address
  * @retval         sector number
  */
/**
  * @brief          获取flash的sector号
  * @param[in]      address: flash 地址
  * @retval         sector号
  */
uint32_t ger_sector(uint32_t address);

#define FLASHWORD_SIZE_BYTES 32u
#define FLASHWORD_SIZE_WORDS (FLASHWORD_SIZE_BYTES / sizeof(uint32_t))

/* STM32H723 programs Flash in 256-bit words.  The HAL takes the address of
 * this buffer, so it must be 32-byte aligned and reside in AXI SRAM. */
static uint32_t flashword_buffer[FLASHWORD_SIZE_WORDS]
    __attribute__((section(".ram_d1"), aligned(FLASHWORD_SIZE_BYTES)));

static int8_t flash_program_words(uint32_t start_address, uint32_t limit_address,
                                  const void *buf, uint32_t len);

/**
  * @brief          erase flash
  * @param[in]      address: flash address
  * @param[in]      len: page num
  * @retval         none
  */
/**
  * @brief          擦除flash
  * @param[in]      address: flash 地址
  * @param[in]      len: 页数量
  * @retval         none
  */
void flash_erase_address(uint32_t address, uint16_t len)
{
    FLASH_EraseInitTypeDef flash_erase;
    uint32_t error;

    flash_erase.Sector = ger_sector(address);
    flash_erase.TypeErase = FLASH_TYPEERASE_SECTORS;
	  flash_erase.Banks = FLASH_BANK_1; 
    flash_erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    flash_erase.NbSectors = len;

    HAL_FLASH_Unlock();
    HAL_FLASHEx_Erase(&flash_erase, &error);
    HAL_FLASH_Lock();
}

/**
  * @brief          write data to one page of flash
  * @param[in]      start_address: flash address
  * @param[in]      buf: data point
  * @param[in]      len: data num
  * @retval         success 0, fail -1
  */
/**
  * @brief          往一页flash写数据
  * @param[in]      start_address: flash 地址
  * @param[in]      buf: 数据指针
  * @param[in]      len: 数据长度
  * @retval         success 0, fail -1
  */
int8_t flash_write_single_address(uint32_t start_address, uint32_t *buf, uint32_t len)
{
    uint32_t limit_address;

    if (start_address >= FLASH_END_ADDR)
        return -1;

    limit_address = get_next_flash_address(start_address);
    if (limit_address == 0u)
        return -1;

    return flash_program_words(start_address, limit_address - 1u, buf, len);
}

/**
  * @brief          write data to some pages of flash
  * @param[in]      start_address: flash start address
  * @param[in]      end_address: flash end address
  * @param[in]      buf: data point
  * @param[in]      len: data num
  * @retval         success 0, fail -1
  */
/**
  * @brief          往几页flash写数据
  * @param[in]      start_address: flash 开始地址
  * @param[in]      end_address: flash 结束地址
  * @param[in]      buf: 数据指针
  * @param[in]      len: 数据长度
  * @retval         success 0, fail -1
  */
int8_t flash_write_muli_address(uint32_t start_address, uint32_t end_address, uint32_t *buf, uint32_t len)
{
    return flash_program_words(start_address, end_address, buf, len);
}

static int8_t flash_program_words(uint32_t start_address, uint32_t limit_address,
                                  const void *buf, uint32_t len)
{
    const uint8_t *source = (const uint8_t *)buf;
    uint32_t address = start_address;
    uint32_t remaining_bytes = len * sizeof(uint32_t);

    if (buf == NULL || len == 0u || (start_address % FLASHWORD_SIZE_BYTES) != 0u)
        return -1;

    HAL_FLASH_Unlock();

    while (remaining_bytes > 0u) {
        uint32_t copy_bytes = remaining_bytes;

        if ((address + FLASHWORD_SIZE_BYTES - 1u) > limit_address) {
            HAL_FLASH_Lock();
            return -1;
        }

        if (copy_bytes > FLASHWORD_SIZE_BYTES)
            copy_bytes = FLASHWORD_SIZE_BYTES;

        memset(flashword_buffer, 0xFF, sizeof(flashword_buffer));
        memcpy(flashword_buffer, source, copy_bytes);

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, address,
                              (uint32_t)flashword_buffer) != HAL_OK) {
            HAL_FLASH_Lock();
            return -1;
        }

        source += copy_bytes;
        remaining_bytes -= copy_bytes;
        address += FLASHWORD_SIZE_BYTES;
    }

    HAL_FLASH_Lock();
    return 0;
}

/**
  * @brief          read data for flash
  * @param[in]      address: flash address
  * @param[out]     buf: data point
  * @param[in]      len: data num
  * @retval         none
  */
/**
  * @brief          从flash读数据
  * @param[in]      start_address: flash 地址
  * @param[out]     buf: 数据指针
  * @param[in]      len: 数据长度
  * @retval         none
  */
void flash_read(uint32_t address, uint32_t *buf, uint32_t len)
{
    memcpy(buf, (void*)address, len *4);
}


/**
  * @brief          get the sector number of flash
  * @param[in]      address: flash address
  * @retval         sector number
  */
/**
  * @brief          获取flash的sector号
  * @param[in]      address: flash 地址
  * @retval         sector号
  */
uint32_t ger_sector(uint32_t address)
{
    uint32_t sector = 0;
    if ((address < ADDR_FLASH_SECTOR_1) && (address >= ADDR_FLASH_SECTOR_0))
    {
        sector = FLASH_SECTOR_0;
    }
    else if ((address < ADDR_FLASH_SECTOR_2) && (address >= ADDR_FLASH_SECTOR_1))
    {
        sector = FLASH_SECTOR_1;
    }
    else if ((address < ADDR_FLASH_SECTOR_3) && (address >= ADDR_FLASH_SECTOR_2))
    {
        sector = FLASH_SECTOR_2;
    }
    else if ((address < ADDR_FLASH_SECTOR_4) && (address >= ADDR_FLASH_SECTOR_3))
    {
        sector = FLASH_SECTOR_3;
    }
    else if ((address < ADDR_FLASH_SECTOR_5) && (address >= ADDR_FLASH_SECTOR_4))
    {
        sector = FLASH_SECTOR_4;
    }
    else if ((address < ADDR_FLASH_SECTOR_6) && (address >= ADDR_FLASH_SECTOR_5))
    {
        sector = FLASH_SECTOR_5;
    }
    else if ((address < ADDR_FLASH_SECTOR_7) && (address >= ADDR_FLASH_SECTOR_6))
    {
        sector = FLASH_SECTOR_6;
    }
    else if ((address < FLASH_END_ADDR) && (address >= ADDR_FLASH_SECTOR_7))
    {
        sector = FLASH_SECTOR_7;
    }
//    else if ((address < ADDR_FLASH_SECTOR_9) && (address >= ADDR_FLASH_SECTOR_8))
//    {
//        sector = FLASH_SECTOR_8;
//    }
//    else if ((address < ADDR_FLASH_SECTOR_10) && (address >= ADDR_FLASH_SECTOR_9))
//    {
//        sector = FLASH_SECTOR_9;
//    }
//    else if ((address < ADDR_FLASH_SECTOR_11) && (address >= ADDR_FLASH_SECTOR_10))
//    {
//        sector = FLASH_SECTOR_10;
//    }
//    else if ((address < ADDR_FLASH_SECTOR_12) && (address >= ADDR_FLASH_SECTOR_11))
//    {
//        sector = FLASH_SECTOR_11;
//    }
    else
    {
        sector = FLASH_SECTOR_7;
    }

    return sector;
}

/**
  * @brief          get the next page flash address
  * @param[in]      address: flash address
  * @retval         next page flash address
  */
/**
  * @brief          获取下一页flash地址
  * @param[in]      address: flash 地址
  * @retval         下一页flash地址
  */
uint32_t get_next_flash_address(uint32_t address)
{
    uint32_t sector = 0;

    if ((address < ADDR_FLASH_SECTOR_1) && (address >= ADDR_FLASH_SECTOR_0))
    {
        sector = ADDR_FLASH_SECTOR_1;
    }
    else if ((address < ADDR_FLASH_SECTOR_2) && (address >= ADDR_FLASH_SECTOR_1))
    {
        sector = ADDR_FLASH_SECTOR_2;
    }
    else if ((address < ADDR_FLASH_SECTOR_3) && (address >= ADDR_FLASH_SECTOR_2))
    {
        sector = ADDR_FLASH_SECTOR_3;
    }
    else if ((address < ADDR_FLASH_SECTOR_4) && (address >= ADDR_FLASH_SECTOR_3))
    {
        sector = ADDR_FLASH_SECTOR_4;
    }
    else if ((address < ADDR_FLASH_SECTOR_5) && (address >= ADDR_FLASH_SECTOR_4))
    {
        sector = ADDR_FLASH_SECTOR_5;
    }
    else if ((address < ADDR_FLASH_SECTOR_6) && (address >= ADDR_FLASH_SECTOR_5))
    {
        sector = ADDR_FLASH_SECTOR_6;
    }
    else if ((address < ADDR_FLASH_SECTOR_7) && (address >= ADDR_FLASH_SECTOR_6))
    {
        sector = ADDR_FLASH_SECTOR_7;
    }
//    else if ((address < ADDR_FLASH_SECTOR_8) && (address >= ADDR_FLASH_SECTOR_7))
//    {
//        sector = ADDR_FLASH_SECTOR_8;
//    }
//    else if ((address < ADDR_FLASH_SECTOR_9) && (address >= ADDR_FLASH_SECTOR_8))
//    {
//        sector = ADDR_FLASH_SECTOR_9;
//    }
//    else if ((address < ADDR_FLASH_SECTOR_10) && (address >= ADDR_FLASH_SECTOR_9))
//    {
//        sector = ADDR_FLASH_SECTOR_10;
//    }
//    else if ((address < ADDR_FLASH_SECTOR_11) && (address >= ADDR_FLASH_SECTOR_10))
//    {
//        sector = ADDR_FLASH_SECTOR_11;
//    }
    else /*(address < FLASH_END_ADDR) && (address >= ADDR_FLASH_SECTOR_23))*/
    {
        sector = FLASH_END_ADDR;
    }
    return sector;
}
