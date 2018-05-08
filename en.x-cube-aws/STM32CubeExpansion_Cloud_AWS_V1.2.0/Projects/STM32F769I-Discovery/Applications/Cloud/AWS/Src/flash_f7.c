/**
  ******************************************************************************
  * @file    flash_f7.c
  * @author  MCD Application Team
  * @brief   Management of the F7 internal flash memory.
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
#include "stm32f7xx_hal.h"
#include "flash.h"
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "msg.h"

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#define ROUND_DOWN(a,b) (((a) / (b)) * (b))
#define MIN(a,b)        (((a) < (b)) ? (a) : (b))
#define MAX(a,b)        (((a) > (b)) ? (a) : (b))


/* Private variables ----------------------------------------------------------*/
#define SINGLE_BANK
#ifdef SINGLE_BANK
/* F7 2 Mbyte single-bank organization. */
  const uint32_t l_sector_map[] = {
    0x08000000, 0x08008000, 0x08010000, 0x08018000,                                       // 32 kbytes sectors
    0x08020000,                                                                           // 128 kbytes sector
    0x08040000, 0x08080000, 0x080C0000, 0x08100000, 0x08140000, 0x08180000, 0x081C0000,   // 256 kbytes sectors
    0x08200000                                                                            // End of range
  };
#else /* SINGLE_BANK */
  /* 2 Mbyte dual-bank organization. */
  const uint32_t l_sector_map[] = {
    0x08000000, 0x08004000, 0x08008000, 0x0800C000,                                       // 16 kbytes sectors, bank1
    0x08010000,                                                                           // 64 kbytes sector, bank1 
    0x08020000, 0x08040000, 0x08060000, 0x08080000, 0x080A0000, 0x080C0000, 0x080E0000,   // 128 kbytes sectors, bank1
    0x08100000, 0x08104000, 0x08108000, 0x0810C000,                                       // 16 kbytes sectors, bank2
    0x08110000,                                                                           // 64 kbytes sector, bank2 
    0x08120000, 0x08140000, 0x08160000, 0x08180000, 0x081A0000, 0x081C0000, 0x081E0000,   // 128 kbytes sectors, bank2
    0x08200000                                                                            // End of range
  };
#endif  /* SINGLE_BANK */
  
/* Private function prototypes -----------------------------------------------*/
static int32_t GetSector(uint32_t Address);
int FLASH_unlock_erase(uint32_t address, uint32_t len_bytes);
uint32_t FLASH_get_bank(uint32_t addr);

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
  
#if defined (FLASH_OPTCR_nDBANK)  
  EraseInit.Banks         = 0;  /* This field is not used in FLASH_TYPEERASE_SECTORS mode */
#endif /* FLASH_OPTCR_nDBANK */  

  if (HAL_FLASH_Unlock() != HAL_OK)
  {
   // msg_info("Flash was already unlocked!\n");
  }
  
  if (HAL_FLASHEx_Erase(&EraseInit, &SectorError) == HAL_OK)
  {
    rc = 0;
  }
  else
  {
    msg_info("Error %lu erasing at 0x%08lx\n", SectorError, address);
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
      msg_info("Write failed @0x%08lx, read value=0x%08lu, expected=0x%08lu\n", (uint32_t) dst, *dst, *src);
      break;
    }
    ret = 0;
  }
  __enable_irq();

  return ret;
}

/**
  * @brief  Get the bank of a given address.
  * @param  In: addr      Address in the FLASH Memory.
  * @retval Bank identifier.
  *           FLASH_BANK_1  // Also when running in single-bank mode.
  *           FLASH_BANK_2
  */
uint32_t FLASH_get_bank(uint32_t addr)
{
  uint32_t bank = 0;
  FLASH_OBProgramInitTypeDef OBInit; 
      
  HAL_FLASHEx_OBGetConfig(&OBInit);

  /* Detect the current boot bank. The BootAddrx value "0x2040" corresponds to bank2 address: "0x08100000" */
  if (((__HAL_SYSCFG_GET_BOOT_MODE() == SYSCFG_MEM_BOOT_ADD0) && (OBInit.BootAddr0 < 0x2040)) || \
      ((__HAL_SYSCFG_GET_BOOT_MODE() == SYSCFG_MEM_BOOT_ADD1) && (OBInit.BootAddr1 < 0x2040))) 
  { /* Bank1 - No bank swap. */
    // if ( GetSector(uint32_t address) > FLASH_BANK_11) 
    // bank = (addr < (FLASH_BASE + FLASH_BANK_SIZE)) ? FLASH_BANK_1 : FLASH_BANK_2;
  }
  else
  { /* Bank2 - Bank swap. */
    // bank = (addr < (FLASH_BASE + FLASH_BANK_SIZE)) ? FLASH_BANK_2 : FLASH_BANK_1;
  }
  
  return bank;
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
      
      /* Clear potential residual errors from the flash loader. */
      __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
      
      /* Load from the flash into the cache */
      memcpy(sector_cache, (void *) fl_addr, sector_size);  
      /* Update the cache from the source */
      memcpy((uint8_t *)sector_cache + fl_offset, src_addr, len);
      /* Erase the page, and write the cache */
      ret = FLASH_unlock_erase(fl_addr, sector_size);
      if (ret != 0)
      {
        msg_error("Could not erasing at 0x%08lx\n", fl_addr);
      }
      else
      {
        ret = FLASH_write_at(fl_addr, sector_cache, sector_size);
        if((ret != 0) && (memcmp((void*)fl_addr, sector_cache, sector_size)))
        {
          msg_error("Could not write %lu bytes at 0x%08lx (err %d)\n", sector_size, fl_addr, ret);
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
  else
  {
    msg_error("Could not allocate %d bytes for Flash update.\n", sector_cache_size + sizeof(uint32_t));
  }
  return rc;
}


#if 0
/**
 * @brief   Set the FLASH bank from which the bootloader which start after the next reset.
 * @note    See also the FLASH_BualBoot L4 example project.
 * @param   bank: Boot target.
 *            FLASH_BANK_1
 *            FLASH_BANK_2
 *            FLASH_BANK_BOTH   Switch to the other bank than the one which is currently used.
 * @retval  0: Success.
 *         <0: Failure.
 */
int FLASH_set_boot_bank(uint32_t bank)
{
  int rc = 0;

  FLASH_OBProgramInitTypeDef    OBInit;          
  /* Set BFB2 bit to enable boot from Flash Bank2 */
  HAL_FLASH_Unlock();                         // Allow Access to the Flash control registers and user Flash.
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR); // Clear OPTVERR bit set on virgin samples.
  HAL_FLASH_OB_Unlock();                      // Allow Access to the option bytes sector.
  HAL_FLASHEx_OBGetConfig(&OBInit);           // Get the Dual boot configuration status.
  
  /* Enable/Disable dual boot feature */
  OBInit.OptionType = OPTIONBYTE_USER;
  OBInit.USERType   = OB_USER_BFB2;
  switch (bank)
  {
    case FLASH_BANK_1:
      OBInit.USERConfig = OB_BFB2_DISABLE;
      break;
    case FLASH_BANK_2:
      OBInit.USERConfig = OB_BFB2_ENABLE;
      break;
    case FLASH_BANK_BOTH:
      OBInit.USERConfig = ( (OBInit.USERConfig & OB_BFB2_ENABLE) == OB_BFB2_ENABLE ) ? OB_BFB2_DISABLE : OB_BFB2_ENABLE;
      break;
    default:
      rc = -1;
  }
  
  if(HAL_FLASHEx_OBProgram (&OBInit) != HAL_OK)
  { /* Failed setting the option bytes configuration.
     * Call 'HAL_FLASH_GetError()' for details. */
    rc = -1;
    msg_info("Error: Could not init the option bytes programming.\n");
  }
  else
  { /* Start the Option Bytes programming process */  
    if (HAL_FLASH_OB_Launch() != HAL_OK)  // /!\ WARN: Triggers an unexpected reset.
    { /* Failed reloading the option bytes configuration.
       * Call 'HAL_FLASH_GetError()' for details. */
      rc = -1;
      msg_info("Error: Could not program the 2nd bank boot option byte.\n");
    }
  }
  
  return rc;
}
#endif // 0

uint32_t  GetSectorMap(void)
{
  FLASH_OBProgramInitTypeDef OBInit; 
  HAL_FLASHEx_OBGetConfig(&OBInit);
  if ((OBInit.USERConfig & OB_NDBANK_SINGLE_BANK) == OB_NDBANK_SINGLE_BANK)
  {
    return 1; //&l_single_bank_sector_map;
  }
  else
  {
    return 2; //&l_dual_bank_sector_map;
  }
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

