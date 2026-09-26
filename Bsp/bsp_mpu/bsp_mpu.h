#ifndef STM32H743_UAV_FLIGHT_CONTROLLER_BSP_MPU_H
#define STM32H743_UAV_FLIGHT_CONTROLLER_BSP_MPU_H

/**
 * @file bsp_mpu.h
 * @brief STM32H7 内存保护和缓存属性配置接口。
 */

/** @brief 配置 FLASH、D1/D2/D3 SRAM 的 MPU 区域属性并启用 MPU。 */
void Bsp_MPU_Config(void);

#endif /* STM32H743_UAV_FLIGHT_CONTROLLER_BSP_MPU_H */
