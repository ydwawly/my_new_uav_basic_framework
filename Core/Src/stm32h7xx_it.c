/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "modules_BMI270.h"
#include "modules_BMI088.h"
#include "modules_SPL06.h"
#include "bsp_driver_sd.h"
#include "SEGGER_SYSVIEW.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

volatile FaultDiagnostic_t fault_diagnostic;

/*
 * HAL_Init() 启动 TIM17 Tick 时，RTT/SystemView 尚未完成初始化。所有 ISR
 * 追踪钩子必须在 SEGGER_SYSVIEW_Conf() 返回后才允许访问 RTT 的 NOLOAD 控制块。
 */
static volatile uint8_t systemview_isr_trace_ready;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

static void Fault_RecordAndBreak(FaultType_e fault_type);
static void SystemView_RecordEnterISRIfReady(void);
static void SystemView_RecordExitISRIfReady(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void Fault_RecordAndBreak(FaultType_e fault_type)
{
    __disable_irq();

    fault_diagnostic.fault_magic = 0x4641554CUL;
    fault_diagnostic.fault_type  = fault_type;
    fault_diagnostic.ipsr        = __get_IPSR();
    fault_diagnostic.cfsr        = SCB->CFSR;
    fault_diagnostic.hfsr        = SCB->HFSR;
    fault_diagnostic.dfsr        = SCB->DFSR;
    fault_diagnostic.afsr        = SCB->AFSR;
    fault_diagnostic.mmfar       = SCB->MMFAR;
    fault_diagnostic.bfar        = SCB->BFAR;
    fault_diagnostic.shcsr       = SCB->SHCSR;

    __DSB();
    __ISB();

    /* 仅在调试器已经连接时触发断点，脱机运行不会因 BKPT 再次嵌套异常。 */
    if ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0U)
    {
        __BKPT(0);
    }
}

static void SystemView_RecordEnterISRIfReady(void)
{
    if (systemview_isr_trace_ready != 0U)
    {
        SEGGER_SYSVIEW_RecordEnterISR();
    }
}

static void SystemView_RecordExitISRIfReady(void)
{
    if (systemview_isr_trace_ready != 0U)
    {
        SEGGER_SYSVIEW_RecordExitISR();
    }
}

void SystemView_ISRTraceSetReady(void)
{
    __DMB();
    systemview_isr_trace_ready = 1U;
}

/* 后续 CubeMX ISR 用户区中的原调用统一经过初始化状态保护。 */
#define SEGGER_SYSVIEW_RecordEnterISR() SystemView_RecordEnterISRIfReady()
#define SEGGER_SYSVIEW_RecordExitISR()  SystemView_RecordExitISRIfReady()

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef   hpcd_USB_OTG_FS;
extern FDCAN_HandleTypeDef hfdcan1;
extern DMA_HandleTypeDef   hdma_i2c2_rx;
extern I2C_HandleTypeDef   hi2c2;
extern SD_HandleTypeDef    hsd1;
extern DMA_HandleTypeDef   hdma_spi2_rx;
extern DMA_HandleTypeDef   hdma_spi2_tx;
extern DMA_HandleTypeDef   hdma_spi3_rx;
extern DMA_HandleTypeDef   hdma_spi3_tx;
extern SPI_HandleTypeDef   hspi2;
extern SPI_HandleTypeDef   hspi3;
extern TIM_HandleTypeDef   htim2;
extern DMA_HandleTypeDef   hdma_uart4_rx;
extern DMA_HandleTypeDef   hdma_uart5_rx;
extern DMA_HandleTypeDef   hdma_uart7_rx;
extern DMA_HandleTypeDef   hdma_uart8_rx;
extern DMA_HandleTypeDef   hdma_uart8_tx;
extern DMA_HandleTypeDef   hdma_usart1_rx;
extern DMA_HandleTypeDef   hdma_usart2_rx;
extern DMA_HandleTypeDef   hdma_usart3_rx;
extern DMA_HandleTypeDef   hdma_usart6_rx;
extern UART_HandleTypeDef  huart4;
extern UART_HandleTypeDef  huart5;
extern UART_HandleTypeDef  huart7;
extern UART_HandleTypeDef  huart8;
extern UART_HandleTypeDef  huart1;
extern UART_HandleTypeDef  huart2;
extern UART_HandleTypeDef  huart3;
extern UART_HandleTypeDef  huart6;
extern TIM_HandleTypeDef   htim17;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
    /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

    /* USER CODE END NonMaskableInt_IRQn 0 */
    /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
    while (1)
    {
    }
    /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
    /* USER CODE BEGIN HardFault_IRQn 0 */
    Fault_RecordAndBreak(FAULT_TYPE_HARD);
    /* USER CODE END HardFault_IRQn 0 */
    while (1)
    {
        /* USER CODE BEGIN W1_HardFault_IRQn 0 */
        /* USER CODE END W1_HardFault_IRQn 0 */
    }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
    /* USER CODE BEGIN MemoryManagement_IRQn 0 */
    Fault_RecordAndBreak(FAULT_TYPE_MEMORY);
    /* USER CODE END MemoryManagement_IRQn 0 */
    while (1)
    {
        /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
        /* USER CODE END W1_MemoryManagement_IRQn 0 */
    }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
    /* USER CODE BEGIN BusFault_IRQn 0 */
    Fault_RecordAndBreak(FAULT_TYPE_BUS);
    /* USER CODE END BusFault_IRQn 0 */
    while (1)
    {
        /* USER CODE BEGIN W1_BusFault_IRQn 0 */
        /* USER CODE END W1_BusFault_IRQn 0 */
    }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
    /* USER CODE BEGIN UsageFault_IRQn 0 */
    Fault_RecordAndBreak(FAULT_TYPE_USAGE);
    /* USER CODE END UsageFault_IRQn 0 */
    while (1)
    {
        /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
        /* USER CODE END W1_UsageFault_IRQn 0 */
    }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
    /* USER CODE BEGIN DebugMonitor_IRQn 0 */

    /* USER CODE END DebugMonitor_IRQn 0 */
    /* USER CODE BEGIN DebugMonitor_IRQn 1 */

    /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
    /* USER CODE BEGIN EXTI0_IRQn 0 */
    SEGGER_SYSVIEW_RecordEnterISR();
    /* USER CODE END EXTI0_IRQn 0 */
    HAL_GPIO_EXTI_IRQHandler(SPL06_INT_Pin);
    /* USER CODE BEGIN EXTI0_IRQn 1 */
    SEGGER_SYSVIEW_RecordExitISR();
    /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
    /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */
    SEGGER_SYSVIEW_RecordEnterISR();
    /* USER CODE END DMA1_Stream0_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_spi2_rx);
    /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */
    SEGGER_SYSVIEW_RecordExitISR();
    /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
    /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */
    SEGGER_SYSVIEW_RecordEnterISR();

    /* USER CODE END DMA1_Stream1_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_spi2_tx);
    /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

    SEGGER_SYSVIEW_RecordExitISR();
    /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream2 global interrupt.
  */
void DMA1_Stream2_IRQHandler(void)
{
    /* USER CODE BEGIN DMA1_Stream2_IRQn 0 */
    SEGGER_SYSVIEW_RecordEnterISR();
    /* USER CODE END DMA1_Stream2_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_spi3_rx);
    /* USER CODE BEGIN DMA1_Stream2_IRQn 1 */
    SEGGER_SYSVIEW_RecordExitISR();
    /* USER CODE END DMA1_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream3 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
    /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */
    SEGGER_SYSVIEW_RecordEnterISR();

    /* USER CODE END DMA1_Stream3_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_spi3_tx);
    /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

    SEGGER_SYSVIEW_RecordExitISR();
    /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream4 global interrupt.
  */
void DMA1_Stream4_IRQHandler(void)
{
    /* USER CODE BEGIN DMA1_Stream4_IRQn 0 */
    SEGGER_SYSVIEW_RecordEnterISR();

    /* USER CODE END DMA1_Stream4_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_usart6_rx);
    /* USER CODE BEGIN DMA1_Stream4_IRQn 1 */

    SEGGER_SYSVIEW_RecordExitISR();
    /* USER CODE END DMA1_Stream4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
    /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */
    SEGGER_SYSVIEW_RecordEnterISR();

    /* USER CODE END DMA1_Stream5_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_usart1_rx);
    /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

    SEGGER_SYSVIEW_RecordExitISR();
    /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream6 global interrupt.
  */
void DMA1_Stream6_IRQHandler(void)
{
    /* USER CODE BEGIN DMA1_Stream6_IRQn 0 */
    SEGGER_SYSVIEW_RecordEnterISR();

    /* USER CODE END DMA1_Stream6_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_uart4_rx);
    /* USER CODE BEGIN DMA1_Stream6_IRQn 1 */

    SEGGER_SYSVIEW_RecordExitISR();
    /* USER CODE END DMA1_Stream6_IRQn 1 */
}

/**
  * @brief This function handles FDCAN1 interrupt 0.
  */
void FDCAN1_IT0_IRQHandler(void)
{
    /* USER CODE BEGIN FDCAN1_IT0_IRQn 0 */
    SEGGER_SYSVIEW_RecordEnterISR();

    /* USER CODE END FDCAN1_IT0_IRQn 0 */
    HAL_FDCAN_IRQHandler(&hfdcan1);
    /* USER CODE BEGIN FDCAN1_IT0_IRQn 1 */

    SEGGER_SYSVIEW_RecordExitISR();
    /* USER CODE END FDCAN1_IT0_IRQn 1 */
}

/**
  * @brief This function handles FDCAN1 interrupt 1.
  */
void FDCAN1_IT1_IRQHandler(void)
{
    /* USER CODE BEGIN FDCAN1_IT1_IRQn 0 */
    SEGGER_SYSVIEW_RecordEnterISR();

    /* USER CODE END FDCAN1_IT1_IRQn 0 */
    HAL_FDCAN_IRQHandler(&hfdcan1);
    /* USER CODE BEGIN FDCAN1_IT1_IRQn 1 */

    SEGGER_SYSVIEW_RecordExitISR();
    /* USER CODE END FDCAN1_IT1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
    /* USER CODE BEGIN EXTI9_5_IRQn 0 */
    SEGGER_SYSVIEW_RecordEnterISR();
    /* USER CODE END EXTI9_5_IRQn 0 */
    HAL_GPIO_EXTI_IRQHandler(BMI270_INT_Pin);
    /* USER CODE BEGIN EXTI9_5_IRQn 1 */
    SEGGER_SYSVIEW_RecordExitISR();
    /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
    /* USER CODE BEGIN TIM2_IRQn 0 */
    SEGGER_SYSVIEW_RecordEnterISR();

    /* USER CODE END TIM2_IRQn 0 */
    HAL_TIM_IRQHandler(&htim2);
    /* USER CODE BEGIN TIM2_IRQn 1 */

    SEGGER_SYSVIEW_RecordExitISR();
    /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles I2C2 event interrupt.
  */
void I2C2_EV_IRQHandler(void)
{
    /* USER CODE BEGIN I2C2_EV_IRQn 0 */
    SEGGER_SYSVIEW_RecordEnterISR();

    /* USER CODE END I2C2_EV_IRQn 0 */
    HAL_I2C_EV_IRQHandler(&hi2c2);
    /* USER CODE BEGIN I2C2_EV_IRQn 1 */

    SEGGER_SYSVIEW_RecordExitISR();
    /* USER CODE END I2C2_EV_IRQn 1 */
}

/**
  * @brief This function handles I2C2 error interrupt.
  */
void I2C2_ER_IRQHandler(void)
{
    /* USER CODE BEGIN I2C2_ER_IRQn 0 */
    SEGGER_SYSVIEW_RecordEnterISR();

    /* USER CODE END I2C2_ER_IRQn 0 */
    HAL_I2C_ER_IRQHandler(&hi2c2);
    /* USER CODE BEGIN I2C2_ER_IRQn 1 */

    SEGGER_SYSVIEW_RecordExitISR();
    /* USER CODE END I2C2_ER_IRQn 1 */
}

/**
  * @brief This function handles SPI2 global interrupt.
  */
void SPI2_IRQHandler(void)
{
    /* USER CODE BEGIN SPI2_IRQn 0 */
    SEGGER_SYSVIEW_RecordEnterISR();

    /* USER CODE END SPI2_IRQn 0 */
    HAL_SPI_IRQHandler(&hspi2);
    /* USER CODE BEGIN SPI2_IRQn 1 */

    SEGGER_SYSVIEW_RecordExitISR();
    /* USER CODE END SPI2_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
    /* USER CODE BEGIN USART1_IRQn 0 */
    SEGGER_SYSVIEW_RecordEnterISR();

    /* USER CODE END USART1_IRQn 0 */
    HAL_UART_IRQHandler(&huart1);
    /* USER CODE BEGIN USART1_IRQn 1 */

    SEGGER_SYSVIEW_RecordExitISR();
    /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
    /* USER CODE BEGIN USART2_IRQn 0 */
    SEGGER_SYSVIEW_RecordEnterISR();

    /* USER CODE END USART2_IRQn 0 */
    HAL_UART_IRQHandler(&huart2);
    /* USER CODE BEGIN USART2_IRQn 1 */

    SEGGER_SYSVIEW_RecordExitISR();
    /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
    /* USER CODE BEGIN USART3_IRQn 0 */
    SEGGER_SYSVIEW_RecordEnterISR();

    /* USER CODE END USART3_IRQn 0 */
    HAL_UART_IRQHandler(&huart3);
    /* USER CODE BEGIN USART3_IRQn 1 */

    SEGGER_SYSVIEW_RecordExitISR();
    /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
    /* USER CODE BEGIN EXTI15_10_IRQn 0 */
    SEGGER_SYSVIEW_RecordEnterISR();
    /* USER CODE END EXTI15_10_IRQn 0 */
    HAL_GPIO_EXTI_IRQHandler(ACC_INT_Pin);
    HAL_GPIO_EXTI_IRQHandler(GYRO_INT_Pin);
    /* USER CODE BEGIN EXTI15_10_IRQn 1 */
    SEGGER_SYSVIEW_RecordExitISR();
    /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream7 global interrupt.
  */
void DMA1_Stream7_IRQHandler(void)
{
    /* USER CODE BEGIN DMA1_Stream7_IRQn 0 */
    SEGGER_SYSVIEW_RecordEnterISR();

    /* USER CODE END DMA1_Stream7_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_usart2_rx);
    /* USER CODE BEGIN DMA1_Stream7_IRQn 1 */

    SEGGER_SYSVIEW_RecordExitISR();
    /* USER CODE END DMA1_Stream7_IRQn 1 */
}

/**
  * @brief This function handles SDMMC1 global interrupt.
  */
void SDMMC1_IRQHandler(void)
{
    /* USER CODE BEGIN SDMMC1_IRQn 0 */
    SEGGER_SYSVIEW_RecordEnterISR();

    /*
     * 多块 DMA 的 DATAEND 和 DMA 数据错误不能进入 HAL 默认的 CMD12 忙等
     * 路径。这里仅完成有界的寄存器收尾并通知任务，CMD12 在任务中执行。
     */
    if (BSP_SD_DeferTransferIRQ(&hsd1) != 0U)
    {
        SEGGER_SYSVIEW_RecordExitISR();
        return;
    }
    /* USER CODE END SDMMC1_IRQn 0 */
    HAL_SD_IRQHandler(&hsd1);
    /* USER CODE BEGIN SDMMC1_IRQn 1 */
    SEGGER_SYSVIEW_RecordExitISR();
    /* USER CODE END SDMMC1_IRQn 1 */
}

/**
  * @brief This function handles SPI3 global interrupt.
  */
void SPI3_IRQHandler(void)
{
    /* USER CODE BEGIN SPI3_IRQn 0 */
    SEGGER_SYSVIEW_RecordEnterISR();

    /* USER CODE END SPI3_IRQn 0 */
    HAL_SPI_IRQHandler(&hspi3);
    /* USER CODE BEGIN SPI3_IRQn 1 */

    SEGGER_SYSVIEW_RecordExitISR();
    /* USER CODE END SPI3_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
    /* USER CODE BEGIN UART4_IRQn 0 */
    SEGGER_SYSVIEW_RecordEnterISR();

    /* USER CODE END UART4_IRQn 0 */
    HAL_UART_IRQHandler(&huart4);
    /* USER CODE BEGIN UART4_IRQn 1 */

    SEGGER_SYSVIEW_RecordExitISR();
    /* USER CODE END UART4_IRQn 1 */
}

/**
  * @brief This function handles UART5 global interrupt.
  */
void UART5_IRQHandler(void)
{
    /* USER CODE BEGIN UART5_IRQn 0 */
    SEGGER_SYSVIEW_RecordEnterISR();

    /* USER CODE END UART5_IRQn 0 */
    HAL_UART_IRQHandler(&huart5);
    /* USER CODE BEGIN UART5_IRQn 1 */

    SEGGER_SYSVIEW_RecordExitISR();
    /* USER CODE END UART5_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
    /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */
    SEGGER_SYSVIEW_RecordEnterISR();

    /* USER CODE END DMA2_Stream0_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_uart8_rx);
    /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

    SEGGER_SYSVIEW_RecordExitISR();
    /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream1 global interrupt.
  */
void DMA2_Stream1_IRQHandler(void)
{
    /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */
    SEGGER_SYSVIEW_RecordEnterISR();

    /* USER CODE END DMA2_Stream1_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_uart8_tx);
    /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */

    SEGGER_SYSVIEW_RecordExitISR();
    /* USER CODE END DMA2_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream3 global interrupt.
  */
void DMA2_Stream3_IRQHandler(void)
{
    /* USER CODE BEGIN DMA2_Stream3_IRQn 0 */
    SEGGER_SYSVIEW_RecordEnterISR();

    /* USER CODE END DMA2_Stream3_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_uart5_rx);
    /* USER CODE BEGIN DMA2_Stream3_IRQn 1 */

    SEGGER_SYSVIEW_RecordExitISR();
    /* USER CODE END DMA2_Stream3_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream4 global interrupt.
  */
void DMA2_Stream4_IRQHandler(void)
{
    /* USER CODE BEGIN DMA2_Stream4_IRQn 0 */
    SEGGER_SYSVIEW_RecordEnterISR();

    /* USER CODE END DMA2_Stream4_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_usart3_rx);
    /* USER CODE BEGIN DMA2_Stream4_IRQn 1 */

    SEGGER_SYSVIEW_RecordExitISR();
    /* USER CODE END DMA2_Stream4_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream5 global interrupt.
  */
void DMA2_Stream5_IRQHandler(void)
{
    /* USER CODE BEGIN DMA2_Stream5_IRQn 0 */
    SEGGER_SYSVIEW_RecordEnterISR();

    /* USER CODE END DMA2_Stream5_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_uart7_rx);
    /* USER CODE BEGIN DMA2_Stream5_IRQn 1 */

    SEGGER_SYSVIEW_RecordExitISR();
    /* USER CODE END DMA2_Stream5_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream7 global interrupt.
  */
void DMA2_Stream7_IRQHandler(void)
{
    /* USER CODE BEGIN DMA2_Stream7_IRQn 0 */
    SEGGER_SYSVIEW_RecordEnterISR();
    /* USER CODE END DMA2_Stream7_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_i2c2_rx);
    /* USER CODE BEGIN DMA2_Stream7_IRQn 1 */
    SEGGER_SYSVIEW_RecordExitISR();
    /* USER CODE END DMA2_Stream7_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
    /* USER CODE BEGIN USART6_IRQn 0 */
    SEGGER_SYSVIEW_RecordEnterISR();

    /* USER CODE END USART6_IRQn 0 */
    HAL_UART_IRQHandler(&huart6);
    /* USER CODE BEGIN USART6_IRQn 1 */

    SEGGER_SYSVIEW_RecordExitISR();
    /* USER CODE END USART6_IRQn 1 */
}

/**
  * @brief This function handles UART7 global interrupt.
  */
void UART7_IRQHandler(void)
{
    /* USER CODE BEGIN UART7_IRQn 0 */
    SEGGER_SYSVIEW_RecordEnterISR();

    /* USER CODE END UART7_IRQn 0 */
    HAL_UART_IRQHandler(&huart7);
    /* USER CODE BEGIN UART7_IRQn 1 */

    SEGGER_SYSVIEW_RecordExitISR();
    /* USER CODE END UART7_IRQn 1 */
}

/**
  * @brief This function handles UART8 global interrupt.
  */
void UART8_IRQHandler(void)
{
    /* USER CODE BEGIN UART8_IRQn 0 */
    SEGGER_SYSVIEW_RecordEnterISR();

    /* USER CODE END UART8_IRQn 0 */
    HAL_UART_IRQHandler(&huart8);
    /* USER CODE BEGIN UART8_IRQn 1 */

    SEGGER_SYSVIEW_RecordExitISR();
    /* USER CODE END UART8_IRQn 1 */
}

/**
  * @brief This function handles USB On The Go FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
    /* USER CODE BEGIN OTG_FS_IRQn 0 */
    SEGGER_SYSVIEW_RecordEnterISR();
    /* USER CODE END OTG_FS_IRQn 0 */
    HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
    /* USER CODE BEGIN OTG_FS_IRQn 1 */
    SEGGER_SYSVIEW_RecordExitISR();
    /* USER CODE END OTG_FS_IRQn 1 */
}

/**
  * @brief This function handles TIM17 global interrupt.
  */
void TIM17_IRQHandler(void)
{
    /* USER CODE BEGIN TIM17_IRQn 0 */
    SEGGER_SYSVIEW_RecordEnterISR();

    /* USER CODE END TIM17_IRQn 0 */
    HAL_TIM_IRQHandler(&htim17);
    /* USER CODE BEGIN TIM17_IRQn 1 */

    SEGGER_SYSVIEW_RecordExitISR();
    /* USER CODE END TIM17_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin)
    {
    case GPIO_PIN_0: // PD0 = SPL06 压力数据就绪
        SPL06_DRDY_Handler();
        break;

    case GPIO_PIN_7: // PB7 = BMI270 DRDY
        BMI270_DRDY_Handler();
        break;

    case GPIO_PIN_15: // PC15 = BMI088 GYRO DRDY (如果同时使用)
        BMI088_GYRO_DRDY_Handler();
        break;

    default:
        break;
    }
}
/* USER CODE END 1 */
