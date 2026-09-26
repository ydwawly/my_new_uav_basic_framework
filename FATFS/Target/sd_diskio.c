/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sd_diskio.c
  * @brief   SD Disk I/O driver
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Note: code generation based on sd_diskio_dma_rtos_template_bspv1.c v2.1.4
   as FreeRTOS is enabled. */

/* USER CODE BEGIN firstSection */
/* can be used to modify / undefine following code or add new definitions */
/* USER CODE END firstSection*/

/* Includes ------------------------------------------------------------------*/
#include "ff_gen_drv.h"
#include "sd_diskio.h"
#include "bsp_memory_section.h"

#include <string.h>
#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define QUEUE_SIZE         (uint32_t) 10
#define READ_CPLT_MSG      (uint32_t) 1
#define WRITE_CPLT_MSG     (uint32_t) 2
#define RW_ERROR_MSG       (uint32_t) 3
#define RW_ABORT_MSG       (uint32_t) 4
#define DEFERRED_READ_MSG  (uint32_t) 5
#define DEFERRED_WRITE_MSG (uint32_t) 6
/*
==================================================================
enable the defines below to send custom rtos messages
when an error or an abort occurs.
Notice: depending on the HAL/SD driver the HAL_SD_ErrorCallback()
may not be available.
See BSP_SD_ErrorCallback() and BSP_SD_AbortCallback() below
==================================================================

#define RW_ERROR_MSG       (uint32_t) 3
#define RW_ABORT_MSG       (uint32_t) 4
*/
/*
 * the following Timeout is useful to give the control back to the applications
 * in case of errors in either BSP_SD_ReadCpltCallback() or BSP_SD_WriteCpltCallback()
 * the value by default is as defined in the BSP platform driver otherwise 30 secs
 */
#define SD_TIMEOUT 30 * 1000

#define SD_DEFAULT_BLOCK_SIZE 512

/*
 * Depending on the use case, the SD card initialization could be done at the
 * application level: if it is the case define the flag below to disable
 * the BSP_SD_Init() call in the SD_Initialize() and add a call to
 * BSP_SD_Init() elsewhere in the application.
 */
/* USER CODE BEGIN disableSDInit */
/* #define DISABLE_SD_INIT */
/* USER CODE END disableSDInit */

/*
 * when using cacheable memory region, it may be needed to maintain the cache
 * validity. Enable the define below to activate a cache maintenance at each
 * read and write operation.
 * Notice: This is applicable only for cortex M7 based platform.
 */
/* USER CODE BEGIN enableSDDmaCacheMaintenance */
#define ENABLE_SD_DMA_CACHE_MAINTENANCE  1
/* USER CODE END enableSDDmaCacheMaintenance */

/*
* Some DMA requires 4-Byte aligned address buffer to correctly read/write data,
* in FatFs some accesses aren't thus we need a 4-byte aligned scratch buffer to correctly
* transfer data
*/
/* USER CODE BEGIN enableScratchBuffer */
#define ENABLE_SCRATCH_BUFFER
/* USER CODE END enableScratchBuffer */

/* Private variables ---------------------------------------------------------*/
#if defined(ENABLE_SCRATCH_BUFFER)
#if defined (ENABLE_SD_DMA_CACHE_MAINTENANCE)
SDMMC_DMA_BUFFER static uint8_t scratch[BLOCKSIZE];
#else
__ALIGN_BEGIN static uint8_t scratch[BLOCKSIZE] __ALIGN_END;
#endif
#endif
/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;

#if (osCMSIS <= 0x20000U)
static osMessageQId SDQueueID = NULL;
#else
static osMessageQueueId_t SDQueueID = NULL;
#endif
/* Private function prototypes -----------------------------------------------*/
static DSTATUS SD_CheckStatus(BYTE lun);
static void SD_ClearQueue(void);
static uint16_t SD_WaitTransferEvent(uint32_t timeout);
DSTATUS SD_initialize (BYTE);
DSTATUS SD_status (BYTE);
DRESULT SD_read (BYTE, BYTE*, DWORD, UINT);
#if _USE_WRITE == 1
DRESULT SD_write (BYTE, const BYTE*, DWORD, UINT);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
DRESULT SD_ioctl (BYTE, BYTE, void*);
#endif  /* _USE_IOCTL == 1 */

const Diskio_drvTypeDef  SD_Driver =
{
  SD_initialize,
  SD_status,
  SD_read,
#if  _USE_WRITE == 1
  SD_write,
#endif /* _USE_WRITE == 1 */

#if  _USE_IOCTL == 1
  SD_ioctl,
#endif /* _USE_IOCTL == 1 */
};

/* USER CODE BEGIN beforeFunctionSection */
/* can be used to modify / undefine following code or add new code */
/* USER CODE END beforeFunctionSection */

/* Private functions ---------------------------------------------------------*/

static int SD_CheckStatusWithTimeout(uint32_t timeout)
{
  uint32_t timer;
  /* block until SDIO peripheral is ready again or a timeout occur */
#if (osCMSIS <= 0x20000U)
  timer = osKernelSysTick();
  while( osKernelSysTick() - timer < timeout)
#else
  timer = osKernelGetTickCount();
  while( osKernelGetTickCount() - timer < timeout)
#endif
  {
    if (BSP_SD_GetCardState() == SD_TRANSFER_OK)
    {
      return 0;
    }
  }

  return -1;
}

static DSTATUS SD_CheckStatus(BYTE lun)
{
  Stat = STA_NOINIT;

  if(BSP_SD_GetCardState() == SD_TRANSFER_OK)
  {
    Stat &= ~STA_NOINIT;
  }

  return Stat;
}

/**
 * @brief 清除上一次超时或异常遗留的 DMA 完成事件。
 *
 * SDMMC 同一时刻只允许一个 FatFS 请求。每次启动新 DMA 前清空队列，可避免旧
 * 事件被下一次读写误认为本次完成事件。
 */
static void SD_ClearQueue(void)
{
  if (SDQueueID != NULL)
  {
#if (osCMSIS < 0x20000U)
    while (osMessageGet(SDQueueID, 0U).status == osEventMessage)
    {
    }
#else
    (void)osMessageQueueReset(SDQueueID);
#endif
  }
}

/**
 * @brief 等待一次 SD DMA 事件，并在任务上下文完成多块传输的 CMD12 收尾。
 * @param timeout CMSIS-RTOS 消息等待超时，单位为内核 Tick。
 * @return READ_CPLT_MSG、WRITE_CPLT_MSG、错误事件；0 表示等待超时。
 *
 * 多块 DATAEND 或 DMA 数据错误 ISR 发送 DEFERRED_*，不直接轮询 CMD12。
 * 本函数运行在发起 FatFs I/O 的任务中，因此卡响应时间可被姿态、控制任务抢占。
 */
static uint16_t SD_WaitTransferEvent(uint32_t timeout)
{
  uint16_t event;

#if (osCMSIS < 0x20000U)
  const osEvent message = osMessageGet(SDQueueID, timeout);
  if (message.status != osEventMessage)
  {
    return 0U;
  }
  event = (uint16_t)message.value.v;
#else
  if (osMessageQueueGet(SDQueueID, (void *)&event, NULL, timeout) != osOK)
  {
    return 0U;
  }
#endif

  if ((event == DEFERRED_READ_MSG) || (event == DEFERRED_WRITE_MSG))
  {
    const uint16_t completed_event = (event == DEFERRED_READ_MSG) ? READ_CPLT_MSG : WRITE_CPLT_MSG;
    return (BSP_SD_FinalizeDeferredTransfer() == MSD_OK) ? completed_event : RW_ERROR_MSG;
  }

  return event;
}

/**
  * @brief  Initializes a Drive
  * @param  lun : not used
  * @retval DSTATUS: Operation status
  */
DSTATUS SD_initialize(BYTE lun)
{
Stat = STA_NOINIT;

  /*
   * check that the kernel has been started before continuing
   * as the osMessage API will fail otherwise
   */
#if (osCMSIS <= 0x20000U)
  if(osKernelRunning())
#else
  if(osKernelGetState() == osKernelRunning)
#endif
  {
#if !defined(DISABLE_SD_INIT)

    if(BSP_SD_Init() == MSD_OK)
    {
      Stat = SD_CheckStatus(lun);
    }

#else
    Stat = SD_CheckStatus(lun);
#endif

    /*
    * if the SD is correctly initialized, create the operation queue
    * if not already created
    */

    if (Stat != STA_NOINIT)
    {
      if (SDQueueID == NULL)
      {
 #if (osCMSIS <= 0x20000U)
      osMessageQDef(SD_Queue, QUEUE_SIZE, uint16_t);
      SDQueueID = osMessageCreate (osMessageQ(SD_Queue), NULL);
#else
      SDQueueID = osMessageQueueNew(QUEUE_SIZE, 2, NULL);
#endif
      }

      if (SDQueueID == NULL)
      {
        Stat |= STA_NOINIT;
      }
    }
  }

  return Stat;
}

/**
  * @brief  Gets Disk Status
  * @param  lun : not used
  * @retval DSTATUS: Operation status
  */
DSTATUS SD_status(BYTE lun)
{
  return SD_CheckStatus(lun);
}

/* USER CODE BEGIN beforeReadSection */
/* can be used to modify previous code / undefine following code / add new code */
/* USER CODE END beforeReadSection */
/**
  * @brief  Reads Sector(s)
  * @param  lun : not used
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */

DRESULT SD_read(BYTE lun, BYTE *buff, DWORD sector, UINT count)
{
  uint8_t ret;
  DRESULT res = RES_ERROR;
  uint32_t timer;
#if (ENABLE_SD_DMA_CACHE_MAINTENANCE == 1)
  uint32_t alignedAddr;
#endif
  /*
  * ensure the SDCard is ready for a new operation
  */

  if (SD_CheckStatusWithTimeout(SD_TIMEOUT) < 0)
  {
    return res;
  }

#if defined(ENABLE_SCRATCH_BUFFER)
  if (!((uint32_t)buff & 0x1F))
  {
#endif
    /* Fast path cause destination buffer is correctly aligned */
    SD_ClearQueue();
    ret = BSP_SD_ReadBlocks_DMA((uint32_t*)buff, (uint32_t)(sector), count);

    if ((ret == MSD_OK) && (SD_WaitTransferEvent(SD_TIMEOUT) == READ_CPLT_MSG))
    {
#if (osCMSIS < 0x20000U)
      timer = osKernelSysTick();
      while ((osKernelSysTick() - timer) < SD_TIMEOUT)
#else
      timer = osKernelGetTickCount();
      while ((osKernelGetTickCount() - timer) < SD_TIMEOUT)
#endif
      {
        if (BSP_SD_GetCardState() == SD_TRANSFER_OK)
        {
          res = RES_OK;
#if (ENABLE_SD_DMA_CACHE_MAINTENANCE == 1)
          /* DMA 已完全结束后再失效缓存，CPU 才能取得卡片写入内存的新数据。 */
          alignedAddr = (uint32_t)buff & ~0x1F;
          SCB_InvalidateDCache_by_Addr((uint32_t*)alignedAddr,
                                      count*BLOCKSIZE + ((uint32_t)buff - alignedAddr));
#endif
          break;
        }
      }
    }

#if defined(ENABLE_SCRATCH_BUFFER)
    }
    else
    {
      /* Slow path, fetch each sector a part and memcpy to destination buffer */
      int i;

      for (i = 0; i < count; i++)
      {
        SD_ClearQueue();
        ret = BSP_SD_ReadBlocks_DMA((uint32_t*)scratch, (uint32_t)sector++, 1);
        if (ret == MSD_OK )
        {
          if (SD_WaitTransferEvent(SD_TIMEOUT) == READ_CPLT_MSG)
          {
#if (osCMSIS < 0x20000U)
            timer = osKernelSysTick();
            while ((osKernelSysTick() - timer) < SD_TIMEOUT)
#else
            timer = osKernelGetTickCount();
            while ((osKernelGetTickCount() - timer) < SD_TIMEOUT)
#endif
            {
              ret = BSP_SD_GetCardState();
              if (ret == MSD_OK)
              {
                break;
              }
            }

            if (ret != MSD_OK)
            {
              break;
            }
          }
          else
          {
            ret = MSD_ERROR;
            break;
          }
#if (ENABLE_SD_DMA_CACHE_MAINTENANCE == 1)
          /*
          *
          * invalidate the scratch buffer before the next read to get the actual data instead of the cached one
          */
          SCB_InvalidateDCache_by_Addr((uint32_t*)scratch, BLOCKSIZE);
#endif
          memcpy(buff, scratch, BLOCKSIZE);
          buff += BLOCKSIZE;
        }
        else
        {
          break;
        }
      }

      if ((i == count) && (ret == MSD_OK ))
        res = RES_OK;
    }
#endif
    return res;
}

/* USER CODE BEGIN beforeWriteSection */
/* can be used to modify previous code / undefine following code / add new code */
/* USER CODE END beforeWriteSection */
/**
 * @brief 通过 SDMMC DMA 写入一个或多个连续扇区
 * @param lun     逻辑驱动器号，当前单 SD 卡配置中未使用
 * @param buff    待写数据首地址，允许不是 32 字节对齐
 * @param sector  起始逻辑扇区地址 LBA
 * @param count   连续写入的扇区数量，范围 1～128
 * @return RES_OK 写入完成，其他值表示 DMA、队列等待或卡状态异常
 *
 * 非对齐缓冲区逐扇区复制到 AXI SRAM scratch；对齐缓冲区直接启动 DMA。
 * 两条路径都会在 DMA 前执行 D-Cache clean，并等待 WRITE_CPLT_MSG。
 */
#if _USE_WRITE == 1

DRESULT SD_write(BYTE lun, const BYTE *buff, DWORD sector, UINT count)
{
    DRESULT  res = RES_ERROR;
    uint32_t timer;

#if defined(ENABLE_SCRATCH_BUFFER)
    int32_t ret;
#endif

    /* 前一笔传输必须结束且卡状态恢复为可传输，才能启动新的写操作。 */

    if (SD_CheckStatusWithTimeout(SD_TIMEOUT) < 0)
    {
        return res;
    }

#if defined(ENABLE_SCRATCH_BUFFER)
    /*
     * FatFS 的窗口缓冲区不保证按 32 字节对齐。STM32H7 的 D-Cache 维护和
     * SDMMC IDMA 都要求 DMA 缓冲区满足对齐及 AXI SRAM 可访问条件，因此非对齐
     * 数据必须先复制到专用 scratch，再逐扇区写入。
     *
     * 这里采用提前返回，避免 CubeMX 原模板中的 else 误绑定到内层
     * BSP_SD_WriteBlocks_DMA() 判断，导致非对齐写入未启动 DMA 就直接失败。
     */
    if (((uint32_t)buff & 0x1FU) != 0U)
    {
        int i;

        for (i = 0; i < (int)count; i++)
        {
            memcpy((void *)scratch, buff, BLOCKSIZE);
            buff += BLOCKSIZE;

#if (ENABLE_SD_DMA_CACHE_MAINTENANCE == 1)
            SCB_CleanDCache_by_Addr((uint32_t *)scratch, BLOCKSIZE);
#endif

            SD_ClearQueue();
            ret = BSP_SD_WriteBlocks_DMA((uint32_t *)scratch, (uint32_t)sector++, 1U);
            if (ret != MSD_OK)
            {
                break;
            }

            if (SD_WaitTransferEvent(SD_TIMEOUT) != WRITE_CPLT_MSG)
            {
                ret = MSD_ERROR;
                break;
            }

#if (osCMSIS < 0x20000U)
            timer = osKernelSysTick();
            while ((osKernelSysTick() - timer) < SD_TIMEOUT)
#else
            timer = osKernelGetTickCount();
            while ((osKernelGetTickCount() - timer) < SD_TIMEOUT)
#endif
            {
                ret = BSP_SD_GetCardState();
                if (ret == MSD_OK)
                {
                    break;
                }
            }

            if (ret != MSD_OK)
            {
                break;
            }
        }

        if ((i == (int)count) && (ret == MSD_OK))
        {
            res = RES_OK;
        }

        return res;
    }
#endif

#if (ENABLE_SD_DMA_CACHE_MAINTENANCE == 1)
    uint32_t alignedAddr;
    /*
     * 对齐路径可直接使用调用者缓冲区；启动 DMA 前必须将待写数据清到内存。
     */
    alignedAddr = (uint32_t)buff & ~0x1FU;
    SCB_CleanDCache_by_Addr((uint32_t *)alignedAddr, count * BLOCKSIZE + ((uint32_t)buff - alignedAddr));
#endif

    SD_ClearQueue();
    if (BSP_SD_WriteBlocks_DMA((uint32_t *)buff, (uint32_t)sector, count) == MSD_OK)
    {
        if (SD_WaitTransferEvent(SD_TIMEOUT) == WRITE_CPLT_MSG)
        {
#if (osCMSIS < 0x20000U)
            timer = osKernelSysTick();
            while ((osKernelSysTick() - timer) < SD_TIMEOUT)
#else
            timer = osKernelGetTickCount();
            while ((osKernelGetTickCount() - timer) < SD_TIMEOUT)
#endif
            {
                if (BSP_SD_GetCardState() == SD_TRANSFER_OK)
                {
                    res = RES_OK;
                    break;
                }
            }
        }
    }

    return res;
}
#endif /* _USE_WRITE == 1 */

/* USER CODE BEGIN beforeIoctlSection */
/* can be used to modify previous code / undefine following code / add new code */
/* USER CODE END beforeIoctlSection */
/**
  * @brief  I/O control operation
  * @param  lun : not used
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
DRESULT SD_ioctl(BYTE lun, BYTE cmd, void *buff)
{
  DRESULT res = RES_ERROR;
  BSP_SD_CardInfo CardInfo;

  if (Stat & STA_NOINIT) return RES_NOTRDY;

  switch (cmd)
  {
  /* Make sure that no pending write process */
  case CTRL_SYNC :
    res = RES_OK;
    break;

  /* Get number of sectors on the disk (DWORD) */
  case GET_SECTOR_COUNT :
    BSP_SD_GetCardInfo(&CardInfo);
    *(DWORD*)buff = CardInfo.LogBlockNbr;
    res = RES_OK;
    break;

  /* Get R/W sector size (WORD) */
  case GET_SECTOR_SIZE :
    BSP_SD_GetCardInfo(&CardInfo);
    *(WORD*)buff = CardInfo.LogBlockSize;
    res = RES_OK;
    break;

  /* Get erase block size in unit of sector (DWORD) */
  case GET_BLOCK_SIZE :
    BSP_SD_GetCardInfo(&CardInfo);
    *(DWORD*)buff = CardInfo.LogBlockSize / SD_DEFAULT_BLOCK_SIZE;
    res = RES_OK;
    break;

  default:
    res = RES_PARERR;
  }

  return res;
}
#endif /* _USE_IOCTL == 1 */

/* USER CODE BEGIN afterIoctlSection */
/* can be used to modify previous code / undefine following code / add new code */
/* USER CODE END afterIoctlSection */

/* USER CODE BEGIN callbackSection */
/**
 * @brief 从 SDMMC ISR 通知等待任务：多块数据阶段已完成，仍需在任务中发送 CMD12。
 * @param context HAL SD_CONTEXT 位掩码。
 * @return MSD_OK 表示事件已进入队列，否则返回 MSD_ERROR。
 */
uint8_t BSP_SD_DeferredTransferCallback(uint32_t context)
{
  uint16_t message;

  if (SDQueueID == NULL)
  {
    return MSD_ERROR;
  }

  if ((context & (SD_CONTEXT_READ_SINGLE_BLOCK | SD_CONTEXT_READ_MULTIPLE_BLOCK)) != 0U)
  {
    message = DEFERRED_READ_MSG;
  }
  else if ((context & (SD_CONTEXT_WRITE_SINGLE_BLOCK | SD_CONTEXT_WRITE_MULTIPLE_BLOCK)) != 0U)
  {
    message = DEFERRED_WRITE_MSG;
  }
  else
  {
    return MSD_ERROR;
  }

#if (osCMSIS < 0x20000U)
  return (osMessagePut(SDQueueID, message, 0U) == osOK) ? MSD_OK : MSD_ERROR;
#else
  return (osMessageQueuePut(SDQueueID, (const void *)&message, 0U, 0U) == osOK) ? MSD_OK : MSD_ERROR;
#endif
}
/* USER CODE END callbackSection */
/**
  * @brief Tx Transfer completed callbacks
  * @param hsd: SD handle
  * @retval None
  */
void BSP_SD_WriteCpltCallback(void)
{

  /*
   * No need to add an "osKernelRunning()" check here, as the SD_initialize()
   * is always called before any SD_Read()/SD_Write() call
   */
#if (osCMSIS < 0x20000U)
   osMessagePut(SDQueueID, WRITE_CPLT_MSG, 0);
#else
   const uint16_t msg = WRITE_CPLT_MSG;
   osMessageQueuePut(SDQueueID, (const void *)&msg, 0, 0);
#endif
}

/**
  * @brief Rx Transfer completed callbacks
  * @param hsd: SD handle
  * @retval None
  */
void BSP_SD_ReadCpltCallback(void)
{
  /*
   * No need to add an "osKernelRunning()" check here, as the SD_initialize()
   * is always called before any SD_Read()/SD_Write() call
   */
#if (osCMSIS < 0x20000U)
   osMessagePut(SDQueueID, READ_CPLT_MSG, 0);
#else
   const uint16_t msg = READ_CPLT_MSG;
   osMessageQueuePut(SDQueueID, (const void *)&msg, 0, 0);
#endif
}

/* USER CODE BEGIN ErrorAbortCallbacks */
void BSP_SD_ErrorCallback(void)
{
#if (osCMSIS < 0x20000U)
   osMessagePut(SDQueueID, RW_ERROR_MSG, 0);
#else
   const uint16_t msg = RW_ERROR_MSG;
   osMessageQueuePut(SDQueueID, (const void *)&msg, 0, 0);
#endif
}

void BSP_SD_AbortCallback(void)
{
#if (osCMSIS < 0x20000U)
   osMessagePut(SDQueueID, RW_ABORT_MSG, 0);
#else
   const uint16_t msg = RW_ABORT_MSG;
   osMessageQueuePut(SDQueueID, (const void *)&msg, 0, 0);
#endif
}
/* USER CODE END ErrorAbortCallbacks */

/* USER CODE BEGIN lastSection */
/* can be used to modify / undefine previous code or add new code */
/* USER CODE END lastSection */
