#ifndef BSP_FLASH_H
#define BSP_FLASH_H
#include "struct_typedef.h"

/* Base address of the Flash sectors */
#define ADDR_FLASH_SECTOR_0 ((uint32_t)0x08000000)  /* Base address of Sector 0, 16 Kbytes   */
#define ADDR_FLASH_SECTOR_1 ((uint32_t)0x08020000)  /* Base address of Sector 1, 16 Kbytes   */
#define ADDR_FLASH_SECTOR_2 ((uint32_t)0x08040000)  /* Base address of Sector 2, 16 Kbytes   */
#define ADDR_FLASH_SECTOR_3 ((uint32_t)0x08060000)  /* Base address of Sector 3, 16 Kbytes   */
#define ADDR_FLASH_SECTOR_4 ((uint32_t)0x08080000)  /* Base address of Sector 4, 64 Kbytes   */
#define ADDR_FLASH_SECTOR_5 ((uint32_t)0x080A0000)  /* Base address of Sector 5, 128 Kbytes  */
#define ADDR_FLASH_SECTOR_6 ((uint32_t)0x080C0000)  /* Base address of Sector 6, 128 Kbytes  */
#define ADDR_FLASH_SECTOR_7 ((uint32_t)0x080E0000)  /* Base address of Sector 7, 128 Kbytes  */


#define FLASH_END_ADDR ((uint32_t)0x08100000)       /* Base address of Sector 23, 128 Kbytes */




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
extern void flash_erase_address(uint32_t address, uint16_t len);

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
extern int8_t flash_write_single_address(uint32_t start_address, uint32_t *buf, uint32_t len);


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
extern int8_t flash_write_muli_address(uint32_t start_address, uint32_t end_address, uint32_t *buf, uint32_t len);

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
extern void flash_read(uint32_t address, uint32_t *buf, uint32_t len);

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
extern uint32_t ger_sector(uint32_t address);
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
extern uint32_t get_next_flash_address(uint32_t address);

#endif
