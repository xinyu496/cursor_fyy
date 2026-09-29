/**
 * @file    drv_i2c_eeprom.c
 * @brief   I2C EEPROM（AT24C02）读写驱动实现
 *
 * 基于 HAL I2C Mem_Read / Mem_Write；写操作通过 IsDeviceReady
 * 等待 EEPROM 内部写周期结束（典型 5ms 内，视型号而定）。
 *
 * @note    GPIO/时钟 MSP 初始化由 CubeMX 生成；本文件内 HAL_I2C_MspInit 已移除。
 */

#include "Driver/drv_i2c_eeprom.h"
#include "Bsp/SEGGER_RTT.h"

/** 全局 I2C 句柄，由应用层绑定 CubeMX 已初始化的 hi2c2（见 USER_Ctrl_OptCalibEepromInit） */
I2C_HandleTypeDef I2C_Handle;

/* ========================================================================
 *                          内部：I2C 模式配置
 * ====================================================================== */

/**
 * @brief （已弃用）原 I2C 主机模式配置：7bit 地址、400kHz、占空比 2:1
 * @note  OwnAddress1 为本机地址；Master 模式下通常无需被寻址
 */
//static void I2C_Mode_Config(void)
//{
//    I2C_Handle.Instance = I2Cx;

//    I2C_Handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
//    I2C_Handle.Init.ClockSpeed = 400000; /* Fast-mode 400kHz */
//    I2C_Handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
//    I2C_Handle.Init.DutyCycle = I2C_DUTYCYCLE_2;           /* 标准 Fast 占空比 */
//    I2C_Handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
//    I2C_Handle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
//    I2C_Handle.Init.OwnAddress1 = I2C_OWN_ADDRESS7;
//    I2C_Handle.Init.OwnAddress2 = 0;

//    HAL_I2C_Init(&I2C_Handle); /* MSP（GPIO/时钟）由 CubeMX 提供 */
//}

///** @brief EEPROM 驱动初始化入口 */
//void I2C_EE_Init(void)
//{
//    I2C_Mode_Config();
//}

/* ========================================================================
 *                          写入：页对齐拆分 + 底层页/字节写
 * ====================================================================== */

/**
 * @brief 缓冲区批量写入 EEPROM，自动处理页边界
 *
 * AT24C 系列页写不能跨页：先算 WriteAddr 在页内偏移 Addr，
 * 再按「当前页剩余 → 整页 → 尾页」顺序调用 PageWrite。
 *
 * @param pBuffer         源数据指针
 * @param WriteAddr       起始 EEPROM 地址
 * @param NumByteToWrite  总字节数
 */
void I2C_EE_BufferWrite(uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
    uint8_t NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0;

    /* 当前地址在页内偏移；count = 本页还可连续写的字节数 */
    Addr = WriteAddr % EEPROM_PAGESIZE;
    count = EEPROM_PAGESIZE - Addr;
    NumOfPage = NumByteToWrite / EEPROM_PAGESIZE;   /* 完整页数 */
    NumOfSingle = NumByteToWrite % EEPROM_PAGESIZE;   /* 不足一页的尾字节 */

    if (Addr == 0) {
        /* 起始地址页对齐：整页循环 + 可选尾页 */
        if (NumOfPage == 0) {
            I2C_EE_PageWrite(pBuffer, WriteAddr, NumOfSingle);
        } else {
            while (NumOfPage--) {
                I2C_EE_PageWrite(pBuffer, WriteAddr, EEPROM_PAGESIZE);
                WriteAddr += EEPROM_PAGESIZE;
                pBuffer += EEPROM_PAGESIZE;
            }
            if (NumOfSingle != 0) {
                I2C_EE_PageWrite(pBuffer, WriteAddr, NumOfSingle);
            }
        }
    } else {
        /* 起始地址未页对齐：先写满当前页剩余 count 字节 */
        if (NumOfPage == 0) {
            I2C_EE_PageWrite(pBuffer, WriteAddr, NumOfSingle);
        } else {
            /* 跨页：先扣掉首段 count，再按整页 + 尾页写剩余数据 */
            NumByteToWrite -= count;
            NumOfPage = NumByteToWrite / EEPROM_PAGESIZE;
            NumOfSingle = NumByteToWrite % EEPROM_PAGESIZE;

            if (count != 0) {
                I2C_EE_PageWrite(pBuffer, WriteAddr, count);
                WriteAddr += count;
                pBuffer += count;
            }

            while (NumOfPage--) {
                I2C_EE_PageWrite(pBuffer, WriteAddr, EEPROM_PAGESIZE);
                WriteAddr += EEPROM_PAGESIZE;
                pBuffer += EEPROM_PAGESIZE;
            }
            if (NumOfSingle != 0) {
                I2C_EE_PageWrite(pBuffer, WriteAddr, NumOfSingle);
            }
        }
    }
}

/**
 * @brief 写 1 字节到指定地址
 * @return HAL_StatusTypeDef 转 uint32_t（HAL_OK=0）
 *
 * 写后流程：Mem_Write → 等 I2C READY → IsDeviceReady 等 EEPROM ACK → 再等 READY。
 */
uint32_t I2C_EE_ByteWrite(uint8_t *pBuffer, uint8_t WriteAddr)
{
    HAL_StatusTypeDef status = HAL_OK;

    status = HAL_I2C_Mem_Write(&I2C_Handle, EEPROM_ADDRESS, (uint16_t)WriteAddr,
                               I2C_MEMADD_SIZE_8BIT, pBuffer, 1, 100);

    /* 等待 HAL 状态回到 READY */
    while (HAL_I2C_GetState(&I2C_Handle) != HAL_I2C_STATE_READY) {
    }

    /* EEPROM 内部写周期：轮询 ACK 直到器件 Ready */
    while (HAL_I2C_IsDeviceReady(&I2C_Handle, EEPROM_ADDRESS, EEPROM_MAX_TRIALS,
                                 I2Cx_TIMEOUT_MAX) == HAL_TIMEOUT) {
    }

    while (HAL_I2C_GetState(&I2C_Handle) != HAL_I2C_STATE_READY) {
    }

    return status;
}

/**
 * @brief 页内连续写（≤ EEPROM_PAGESIZE，且不跨页）
 * @return HAL 状态码
 */
uint32_t I2C_EE_PageWrite(uint8_t *pBuffer, uint8_t WriteAddr, uint8_t NumByteToWrite)
{
    HAL_StatusTypeDef status = HAL_OK;

    status = HAL_I2C_Mem_Write(&I2C_Handle, EEPROM_ADDRESS, WriteAddr,
                               I2C_MEMADD_SIZE_8BIT, (uint8_t *)(pBuffer),
                               NumByteToWrite, 100);

    if (status != HAL_OK) {
        SEGGER_RTT_printf(0, "EW%X\n", (unsigned)status);
        return status;
    }

    /* 页写后同样需等 HAL 空闲与 EEPROM 内部编程完成 */
    while (HAL_I2C_GetState(&I2C_Handle) != HAL_I2C_STATE_READY) {
    }

    while (HAL_I2C_IsDeviceReady(&I2C_Handle, EEPROM_ADDRESS, EEPROM_MAX_TRIALS,
                                 I2Cx_TIMEOUT_MAX) == HAL_TIMEOUT) {
    }

    while (HAL_I2C_GetState(&I2C_Handle) != HAL_I2C_STATE_READY) {
    }

    return status;
}

/* ========================================================================
 *                          批量读
 * ====================================================================== */

/**
 * @brief 从 ReadAddr 起连续读取 NumByteToRead 字节
 * @param pBuffer        接收缓冲
 * @param ReadAddr       EEPROM 起始地址
 * @param NumByteToRead  读取长度
 * @return HAL 状态码（读操作无需写周期等待）
 */
uint32_t I2C_EE_BufferRead(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
    HAL_StatusTypeDef status = HAL_OK;

    status = HAL_I2C_Mem_Read(&I2C_Handle, EEPROM_ADDRESS, ReadAddr,
                              I2C_MEMADD_SIZE_8BIT, (uint8_t *)pBuffer,
                              NumByteToRead, 1000);

    if (status != HAL_OK) {
        SEGGER_RTT_printf(0, "ER%X\n", (unsigned)status);
    }

    return status;
}
