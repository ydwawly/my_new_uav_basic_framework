/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * @file    bsp_driver_sd.h (based on stm32h743i_eval_sd.h)
  * @brief   This file contains the common defines and functions prototypes for
  *          the bsp_driver_sd.c driver.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32H7_SD_H
#define __STM32H7_SD_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Exported types --------------------------------------------------------*/
/**
  * @brief SD Card information structure
  */
#define BSP_SD_CardInfo HAL_SD_CardInfoTypeDef

/* Exported constants --------------------------------------------------------*/
/**
  * @brief  SD status structure definition
  */
#define MSD_OK                   ((uint8_t)0x00)
#define MSD_ERROR                ((uint8_t)0x01)
#define MSD_ERROR_SD_NOT_PRESENT ((uint8_t)0x02)

/**
  * @brief  SD transfer state definition
  */
#define SD_TRANSFER_OK   ((uint8_t)0x00)
#define SD_TRANSFER_BUSY ((uint8_t)0x01)

#define SD_PRESENT     ((uint8_t)0x01)
#define SD_NOT_PRESENT ((uint8_t)0x00)
#define SD_DATATIMEOUT ((uint32_t)100000000)

/* USER CODE BEGIN BSP_H_CODE */
#define SD_DetectIRQHandler() HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8)

    /* Exported functions --------------------------------------------------------*/
    uint8_t BSP_SD_Init(void);
    uint8_t BSP_SD_ITConfig(void);
    uint8_t BSP_SD_ReadBlocks(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks, uint32_t Timeout);
    uint8_t BSP_SD_WriteBlocks(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks, uint32_t Timeout);
    uint8_t BSP_SD_ReadBlocks_DMA(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks);
    uint8_t BSP_SD_WriteBlocks_DMA(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks);
    uint8_t BSP_SD_Erase(uint32_t StartAddr, uint32_t EndAddr);
    uint8_t BSP_SD_GetCardState(void);
    void    BSP_SD_GetCardInfo(BSP_SD_CardInfo *CardInfo);
    uint8_t BSP_SD_IsDetected(void);

    /* These functions can be modified in case the current settings (e.g. DMA stream)
   need to be changed for specific application needs */
    void BSP_SD_AbortCallback(void);
    void BSP_SD_ErrorCallback(void);
    void BSP_SD_WriteCpltCallback(void);
    void BSP_SD_ReadCpltCallback(void);

    /**
     * @brief 截获多块 DMA DATAEND 或 DMA 数据错误，把阻塞式 CMD12 收尾延后到任务。
     * @param hsd SDMMC HAL 句柄。
     * @return 1 表示本次中断已处理，调用者不得再进入 HAL_SD_IRQHandler；0 表示交给 HAL。
     */
    uint8_t BSP_SD_DeferTransferIRQ(SD_HandleTypeDef *hsd);

    /**
     * @brief 在 FatFs 调用任务中完成被延后的 CMD12、HAL 状态和错误收尾。
     * @return MSD_OK 表示无需收尾或收尾成功；MSD_ERROR 表示数据阶段或 CMD12 出错。
     */
    uint8_t BSP_SD_FinalizeDeferredTransfer(void);

    /**
     * @brief ISR 到 FatFs diskio 层的延后完成通知钩子。
     * @param context HAL SD_CONTEXT 位掩码，用于区分读写方向。
     * @return MSD_OK 表示通知成功，否则表示等待任务无法被唤醒。
     */
    uint8_t BSP_SD_DeferredTransferCallback(uint32_t context);
    /* USER CODE END BSP_H_CODE */

#ifdef __cplusplus
}
#endif

#endif /* __STM32H7_SD_H */
