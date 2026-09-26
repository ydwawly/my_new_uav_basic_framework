/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    bsp_driver_sd.c for H7 (based on stm32h743i_eval_sd.c)
 * @brief   This file includes a generic uSD card driver.
 *          To be completed by the user according to the board used for the project.
 * @note    Some functions generated as weak: they can be overridden by
 *          - code in user files
 *          - or BSP code from the FW pack files
 *          if such files are added to the generated project (by the user).
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

/* USER CODE BEGIN FirstSection */
/* can be used to modify / undefine following code or add new definitions */
/* USER CODE END FirstSection */
/* Includes ------------------------------------------------------------------*/
#include "bsp_driver_sd.h"

/* Extern variables ---------------------------------------------------------*/

extern SD_HandleTypeDef hsd1;

/* USER CODE BEGIN BeforeInitSection */
/*
 * 多块 DMA 完成及 DMA 错误后的 CMD12 需要等待 SD 卡响应，不能在
 * SDMMC ISR 中轮询；发起本次 FatFs I/O 的任务随后完成收尾。
 */
static volatile uint8_t sd_deferred_pending;
/* USER CODE END BeforeInitSection */
/**
  * @brief  Initializes the SD card device.
  * @retval SD status
  */
__weak uint8_t BSP_SD_Init(void)
{
    uint8_t sd_state = MSD_OK;
    /* Check if the SD card is plugged in the slot */
    if (BSP_SD_IsDetected() != SD_PRESENT)
    {
        return MSD_ERROR_SD_NOT_PRESENT;
    }
    /* HAL SD initialization */
    sd_state = HAL_SD_Init(&hsd1);
    /* Configure SD Bus width (4 bits mode selected) */
    if (sd_state == MSD_OK)
    {
        /* Enable wide operation */
        if (HAL_SD_ConfigWideBusOperation(&hsd1, SDMMC_BUS_WIDE_4B) != HAL_OK)
        {
            sd_state = MSD_ERROR;
        }
    }

    return sd_state;
}
/* USER CODE BEGIN AfterInitSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END AfterInitSection */

/* USER CODE BEGIN InterruptMode */
/**
  * @brief  Configures Interrupt mode for SD detection pin.
  * @retval Returns 0
  */
__weak uint8_t BSP_SD_ITConfig(void)
{
    /* Code to be updated by the user or replaced by one from the FW pack (in a stmxxxx_sd.c file) */

    return (uint8_t)0;
}

/* USER CODE END InterruptMode */

/* USER CODE BEGIN BeforeReadBlocksSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END BeforeReadBlocksSection */
/**
  * @brief  Reads block(s) from a specified address in an SD card, in polling mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  ReadAddr: Address from where data is to be read
  * @param  NumOfBlocks: Number of SD blocks to read
  * @param  Timeout: Timeout for read operation
  * @retval SD status
  */
__weak uint8_t BSP_SD_ReadBlocks(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks, uint32_t Timeout)
{
    uint8_t sd_state = MSD_OK;

    if (HAL_SD_ReadBlocks(&hsd1, (uint8_t *)pData, ReadAddr, NumOfBlocks, Timeout) != HAL_OK)
    {
        sd_state = MSD_ERROR;
    }

    return sd_state;
}

/* USER CODE BEGIN BeforeWriteBlocksSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END BeforeWriteBlocksSection */
/**
  * @brief  Writes block(s) to a specified address in an SD card, in polling mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  WriteAddr: Address from where data is to be written
  * @param  NumOfBlocks: Number of SD blocks to write
  * @param  Timeout: Timeout for write operation
  * @retval SD status
  */
__weak uint8_t BSP_SD_WriteBlocks(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks, uint32_t Timeout)
{
    uint8_t sd_state = MSD_OK;

    if (HAL_SD_WriteBlocks(&hsd1, (uint8_t *)pData, WriteAddr, NumOfBlocks, Timeout) != HAL_OK)
    {
        sd_state = MSD_ERROR;
    }

    return sd_state;
}

/* USER CODE BEGIN BeforeReadDMABlocksSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END BeforeReadDMABlocksSection */
/**
  * @brief  Reads block(s) from a specified address in an SD card, in DMA mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  ReadAddr: Address from where data is to be read
  * @param  NumOfBlocks: Number of SD blocks to read
  * @retval SD status
  */
__weak uint8_t BSP_SD_ReadBlocks_DMA(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks)
{
    uint8_t sd_state = MSD_OK;

    /* Read block(s) in DMA transfer mode */
    if (HAL_SD_ReadBlocks_DMA(&hsd1, (uint8_t *)pData, ReadAddr, NumOfBlocks) != HAL_OK)
    {
        sd_state = MSD_ERROR;
    }

    return sd_state;
}

/* USER CODE BEGIN BeforeWriteDMABlocksSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END BeforeWriteDMABlocksSection */
/**
  * @brief  Writes block(s) to a specified address in an SD card, in DMA mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  WriteAddr: Address from where data is to be written
  * @param  NumOfBlocks: Number of SD blocks to write
  * @retval SD status
  */
__weak uint8_t BSP_SD_WriteBlocks_DMA(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks)
{
    uint8_t sd_state = MSD_OK;

    /* Write block(s) in DMA transfer mode */
    if (HAL_SD_WriteBlocks_DMA(&hsd1, (uint8_t *)pData, WriteAddr, NumOfBlocks) != HAL_OK)
    {
        sd_state = MSD_ERROR;
    }

    return sd_state;
}

/* USER CODE BEGIN BeforeEraseSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END BeforeEraseSection */
/**
  * @brief  Erases the specified memory area of the given SD card.
  * @param  StartAddr: Start byte address
  * @param  EndAddr: End byte address
  * @retval SD status
  */
__weak uint8_t BSP_SD_Erase(uint32_t StartAddr, uint32_t EndAddr)
{
    uint8_t sd_state = MSD_OK;

    if (HAL_SD_Erase(&hsd1, StartAddr, EndAddr) != HAL_OK)
    {
        sd_state = MSD_ERROR;
    }

    return sd_state;
}

/* USER CODE BEGIN BeforeGetCardStateSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END BeforeGetCardStateSection */

/**
  * @brief  Gets the current SD card data status.
  * @param  None
  * @retval Data transfer state.
  *          This value can be one of the following values:
  *            @arg  SD_TRANSFER_OK: No data transfer is acting
  *            @arg  SD_TRANSFER_BUSY: Data transfer is acting
  */
__weak uint8_t BSP_SD_GetCardState(void)
{
    return ((HAL_SD_GetCardState(&hsd1) == HAL_SD_CARD_TRANSFER) ? SD_TRANSFER_OK : SD_TRANSFER_BUSY);
}

/**
  * @brief  Get SD information about specific SD card.
  * @param  CardInfo: Pointer to HAL_SD_CardInfoTypedef structure
  * @retval None
  */
__weak void BSP_SD_GetCardInfo(HAL_SD_CardInfoTypeDef *CardInfo)
{
    /* Get SD card Information */
    HAL_SD_GetCardInfo(&hsd1, CardInfo);
}

/* USER CODE BEGIN BeforeCallBacksSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END BeforeCallBacksSection */
/**
  * @brief SD Abort callbacks
  * @param hsd: SD handle
  * @retval None
  */
void HAL_SD_AbortCallback(SD_HandleTypeDef *hsd)
{
    (void)hsd;
    BSP_SD_AbortCallback();
}

/**
 * @brief SDMMC DMA 错误回调。
 * @param hsd SD 外设句柄。
 */
void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd)
{
    (void)hsd;
    BSP_SD_ErrorCallback();
}

/**
  * @brief Tx Transfer completed callback
  * @param hsd: SD handle
  * @retval None
  */
void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd)
{
    (void)hsd;
    BSP_SD_WriteCpltCallback();
}

/**
  * @brief Rx Transfer completed callback
  * @param hsd: SD handle
  * @retval None
  */
void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd)
{
    (void)hsd;
    BSP_SD_ReadCpltCallback();
}

/* USER CODE BEGIN CallBacksSection_C */
/**
  * @brief BSP SD Abort callback
  * @retval None
  * @note empty (up to the user to fill it in or to remove it if useless)
  */
__weak void BSP_SD_AbortCallback(void) {}

/**
 * @brief BSP 错误回调，具体实现由 FatFS diskio 层提供。
 */
__weak void BSP_SD_ErrorCallback(void) {}

/**
  * @brief BSP Tx Transfer completed callback
  * @retval None
  * @note empty (up to the user to fill it in or to remove it if useless)
  */
__weak void BSP_SD_WriteCpltCallback(void) {}

/**
  * @brief BSP Rx Transfer completed callback
  * @retval None
  * @note empty (up to the user to fill it in or to remove it if useless)
  */
__weak void BSP_SD_ReadCpltCallback(void) {}

/**
 * @brief 多块 DMA DATAEND 的延后通知默认实现。
 * @note FatFs 的 sd_diskio.c 会提供强符号实现；默认失败可避免静默丢失完成事件。
 */
__weak uint8_t BSP_SD_DeferredTransferCallback(uint32_t context)
{
    (void)context;
    return MSD_ERROR;
}
/* USER CODE END CallBacksSection_C */

/**
 * @brief  Detects if SD card is correctly plugged in the memory slot or not.
 * @param  None
 * @retval Returns if SD is detected or not
 */
__weak uint8_t BSP_SD_IsDetected(void)
{
    __IO uint8_t status = SD_PRESENT;

    /* USER CODE BEGIN IsDetectedSection */
    /* user code can be inserted here */
    /* USER CODE END IsDetectedSection */

    return status;
}

/* USER CODE BEGIN AdditionalCode */

uint8_t BSP_SD_DeferTransferIRQ(SD_HandleTypeDef *hsd)
{
    if ((hsd == NULL) || (hsd != &hsd1) || (sd_deferred_pending != 0U))
    {
        return 0U;
    }

    const uint32_t context          = hsd->Context;
    const uint32_t multi_block_mask = SD_CONTEXT_READ_MULTIPLE_BLOCK | SD_CONTEXT_WRITE_MULTIPLE_BLOCK;
    const uint32_t data_error_flags = SDMMC_FLAG_DCRCFAIL | SDMMC_FLAG_DTIMEOUT | SDMMC_FLAG_RXOVERR |
                                      SDMMC_FLAG_TXUNDERR;
    const uint8_t is_multi_data_end = ((__HAL_SD_GET_FLAG(hsd, SDMMC_FLAG_DATAEND) != RESET) &&
                                       ((context & SD_CONTEXT_DMA) != 0U) &&
                                       ((context & multi_block_mask) != 0U))
                                          ? 1U
                                          : 0U;
    const uint8_t is_dma_error = ((__HAL_SD_GET_FLAG(hsd, data_error_flags) != RESET) &&
                                  ((context & SD_CONTEXT_DMA) != 0U))
                                     ? 1U
                                     : 0U;

    /* 正常单块完成及其他无阻塞路径继续使用 ST HAL 的原处理流程。 */
    if ((is_multi_data_end == 0U) && (is_dma_error == 0U))
    {
        return 0U;
    }

    if (is_dma_error != 0U)
    {
        if (__HAL_SD_GET_FLAG(hsd, SDMMC_FLAG_DCRCFAIL) != RESET)
        {
            hsd->ErrorCode |= HAL_SD_ERROR_DATA_CRC_FAIL;
        }
        if (__HAL_SD_GET_FLAG(hsd, SDMMC_FLAG_DTIMEOUT) != RESET)
        {
            hsd->ErrorCode |= HAL_SD_ERROR_DATA_TIMEOUT;
        }
        if (__HAL_SD_GET_FLAG(hsd, SDMMC_FLAG_RXOVERR) != RESET)
        {
            hsd->ErrorCode |= HAL_SD_ERROR_RX_OVERRUN;
        }
        if (__HAL_SD_GET_FLAG(hsd, SDMMC_FLAG_TXUNDERR) != RESET)
        {
            hsd->ErrorCode |= HAL_SD_ERROR_TX_UNDERRUN;
        }

        __HAL_SD_CLEAR_FLAG(hsd, SDMMC_STATIC_DATA_FLAGS);
        hsd->Instance->DCTRL |= SDMMC_DCTRL_FIFORST;
    }
    else
    {
        __HAL_SD_CLEAR_FLAG(hsd, SDMMC_FLAG_DATAEND);
    }

    /* 关闭数据事件和 IDMA，确保退出 ISR 后不会由相同状态重复进入。 */
    __HAL_SD_DISABLE_IT(hsd, SDMMC_IT_DATAEND | SDMMC_IT_DCRCFAIL | SDMMC_IT_DTIMEOUT |
                                SDMMC_IT_TXUNDERR | SDMMC_IT_RXOVERR | SDMMC_IT_TXFIFOHE |
                                SDMMC_IT_RXFIFOHF);
    __HAL_SD_DISABLE_IT(hsd, SDMMC_IT_IDMABTC);
    __SDMMC_CMDTRANS_DISABLE(hsd->Instance);

    if (is_dma_error == 0U)
    {
        /* 与 HAL 的正常 DMA DATAEND 路径一致，终止本次数据路径。 */
        hsd->Instance->DLEN  = 0U;
        hsd->Instance->DCTRL = 0U;
    }
    hsd->Instance->IDMACTRL = SDMMC_DISABLE_IDMA;
    __DSB();

    sd_deferred_pending = 1U;
    __DMB();

    if (BSP_SD_DeferredTransferCallback(context) != MSD_OK)
    {
        /*
         * 正常情况下队列在每次传输前均为空，此分支只用于通知设施损坏时避免
         * SD 卡永久停在多块传输状态。退化为 ISR 内收尾并上报本次 I/O 错误。
         */
        hsd->ErrorCode |= HAL_SD_ERROR_TIMEOUT;
        (void)BSP_SD_FinalizeDeferredTransfer();
        HAL_SD_ErrorCallback(hsd);
    }

    return 1U;
}

uint8_t BSP_SD_FinalizeDeferredTransfer(void)
{
    if (sd_deferred_pending == 0U)
    {
        return MSD_OK;
    }

    const uint32_t previous_error = hsd1.ErrorCode;
    const uint32_t stop_error     = SDMMC_CmdStopTransfer(hsd1.Instance);

    if (stop_error != HAL_SD_ERROR_NONE)
    {
        hsd1.ErrorCode |= stop_error;
    }

    /* 完成 HAL 原 DATAEND 分支剩余的状态转换，之后才允许下一笔 DMA。 */
    hsd1.Instance->CMD &= ~SDMMC_CMD_CMDSTOP;
    __HAL_SD_CLEAR_FLAG(&hsd1, SDMMC_FLAG_DABORT);
    __HAL_SD_CLEAR_FLAG(&hsd1, SDMMC_STATIC_DATA_FLAGS);
    hsd1.State   = HAL_SD_STATE_READY;
    hsd1.Context = SD_CONTEXT_NONE;

    sd_deferred_pending = 0U;
    __DMB();

    return ((previous_error == HAL_SD_ERROR_NONE) && (stop_error == HAL_SD_ERROR_NONE)) ? MSD_OK : MSD_ERROR;
}
/* USER CODE END AdditionalCode */
