/**
  ******************************************************************************
  * @file    flash_f4.c
  * @author  MCD Application Team
  * @brief   Management of the F4 internal flash memory.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "flash.h"
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#define ROUND_DOWN(a,b) (((a) / (b)) * (b))
#define MIN(a,b)        (((a) < (b)) ? (a) : (b))
#define MAX(a,b)        (((a) > (b)) ? (a) : (b))


/* Private variables ----------------------------------------------------------*/

  /* F413 1.5 Mbyte single-bank organization. */
  const uint32_t l_sector_map[] = {
    0x08000000, 0x08004000, 0x08008000, 0x0800C000, // 16 kbytes sectors
    0x08010000,                                     // 64 kbytes sector
    0x08020000, 0x08040000, 0x08060000, 0x08080000, // 128 kbytes sectors...
    0x080A0000, 0x080C0000, 0x080E0000, 0x08100000, 
    0x08120000, 0x08140000, 0x08160000, 0x08180000
  };

  
/* Private function prototypes -----------------------------------------------*/
static int32_t GetSector(uint32_t Address);
static int FLASH_unlock_erase(uint32_t address, uint32_t len_bytes);

/* Functions Definition ------------------------------------------------------*/

/**
  * @brief  Erase FLASH memory sector(s) at address.
  * @param  In: address     Start address to erase from.
  * @param  In: len_bytes   Length to be erased.
  * @retval  0:  Success.
            -1:  Failure.
  */
int FLASH_unlock_erase(uint32_t address, uint32_t len_bytes)
{
  int rc = -1;
  uint32_t SectorError = 0;
  FLASH_EraseInitTypeDef EraseInit;

  /* WARN: ABW. If the passed address and size are not aligned on the sectors geometry,
   * the start of the first sector and the end of the last sector are erased anyway.
   * After erase, the flash is left in unlocked state.
   */
  EraseInit.TypeErase     = FLASH_TYPEERASE_SECTORS;
  EraseInit.VoltageRange  = FLASH_VOLTAGE_RANGE_3;  // Does not support more than single-word programming. See §3.5.2 in RM0430.
  EraseInit.Sector        = GetSector(address);
  EraseInit.NbSectors     = GetSector(address + len_bytes - 1) - EraseInit.Sector + 1;

  if (HAL_FLASH_Unlock() == HAL_OK)
  {
   // printf("Flash unlocked successfully!\n");
  }
  else
  {
   // printf("Flash was already unlocked!\n");
  }
  
  if (HAL_FLASHEx_Erase(&EraseInit, &SectorError) == HAL_OK)
  {  
    rc = 0;
  }
  else
  {
    printf("Error %lu erasing at 0x%08lx\n", SectorError, address);
  }
  
  return rc;
}

/**
  * @brief  Write to FLASH memory.
  * @param  In: address     Destination address.
  * @param  In: pData       Data to be programmed: Must be 8 byte aligned.
  * @param  In: len_bytes   Number of bytes to be programmed.
  * @retval  0: Success.
            -1: Failure.
  */
int FLASH_write_at(uint32_t address, uint32_t *pData, uint32_t len_bytes)
{
  int i;
  int ret = -1;

  __disable_irq();
  
  for (i = 0; i < len_bytes; i += 4)
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
        address + i,
        *(pData + (i/4) )) != HAL_OK)
    {
      break;
    }
  }
  
  /* Memory check */
  for (i = 0; i < len_bytes; i += 4)
  {
    uint32_t *dst = (uint32_t *)(address + i);
    uint32_t *src = ((uint32_t *) pData) + (i/4);
      
    if ( *dst != *src )
    {
      printf("Write failed @0x%08lx, read value=0x%08lx, expected=0x%08lx\n", (uint32_t) dst, *dst, *src);
      break;
    }
    ret = 0;
  }
  __enable_irq();

  return ret;
}


/**
  * @brief  Update a chunk of the FLASH memory.
  * @note   The FLASH chunk must no cross a FLASH bank boundary.
  * @note   The source and destination buffers have no specific alignment constraints.
  * @param  In: dst_addr    Destination address in the FLASH memory.
  * @param  In: data        Source address. 
  * @param  In: size        Number of bytes to update.
  * @retval  0:  Success.
  *         <0:  Failure (at malloc, erase, or write).
  */
int FLASH_update(uint32_t dst_addr, const void *data, uint32_t size)
{
  int rc = -1;
  int ret = 0;
  int sector_cache_size = 0;
  int remaining = size;
  uint8_t * src_addr = (uint8_t *) data;
  uint32_t * sector_cache = NULL;
  uint8_t * sector_cache_buffer = NULL;
  
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP    | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |\
                         FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR| FLASH_FLAG_PGSERR);  
 
  /* Compute the size of the largest sector to be updated. */
  for (int i = GetSector(dst_addr); i < (GetSector(dst_addr + size - 1) + 1); i++)
  {
    sector_cache_size = MAX(sector_cache_size, l_sector_map[i+1] - l_sector_map[i]);
  }
  
  /* Allocate and align the sector cache on double-word boundaries, in order to allow double-word page programming. */
  sector_cache_buffer = malloc (sector_cache_size + sizeof(uint32_t));
  
  if (sector_cache_buffer != NULL)
  {
    sector_cache = (uint32_t *) ( (uint32_t)sector_cache_buffer & ~(sizeof(uint32_t) - 1) ) + 1; 
        
    do {
      uint32_t sector = GetSector(dst_addr); 
      uint32_t sector_size = l_sector_map[sector + 1] - l_sector_map[sector];
      uint32_t fl_addr = l_sector_map[sector];
      int fl_offset = dst_addr - fl_addr;
      int len = MIN(sector_size - fl_offset, remaining);
      
      /* Load from the flash into the cache */
      memcpy(sector_cache, (void *) fl_addr, sector_size);  
      /* Update the cache from the source */
      memcpy((uint8_t *)sector_cache + fl_offset, src_addr, len);
      /* Erase the page, and write the cache */
      ret = FLASH_unlock_erase(fl_addr, sector_size);
      if (ret != 0)
      {
        printf("Error erasing at 0x%08lx\n", fl_addr);
      }
      else
      {
        ret = FLASH_write_at(fl_addr, sector_cache, sector_size);
        if((ret != 0) && (memcmp((void*)fl_addr, sector_cache, sector_size)))
        {
          printf("Error %d writing %lu bytes at 0x%08lx\n", ret, sector_size, fl_addr);
        }
        else
        {
          dst_addr += len;
          src_addr += len;
          remaining -= len;
        }
      }
    } while ((ret == 0) && (remaining > 0));
    if (ret == 0)
    {
      rc = 0;
    }
    
    free(sector_cache_buffer);
  }
  return rc;
}


/**
  * @brief  Gets the sector number of a given address.
  * @param  In: address
  * @retval >=0 Sector number.
  *         -1  Error: Invalid address.
  */
static int32_t GetSector(uint32_t address)
{
  int32_t sector = -1;
    
  
  if ( (l_sector_map[0] <= address) && (address < l_sector_map[sizeof(l_sector_map)/sizeof(uint32_t) - 1]) )
  { /* The address is within the range of the internal flash. */
    for (int i = 0; i < (sizeof(l_sector_map) / sizeof(uint32_t) - 1); i++)
    {
      if (address < l_sector_map[i+1])
      { /* Matching sector found. */ 
        sector = i;
        break;
      }
    }
  }
  
  return sector;
}



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

