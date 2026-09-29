/**
 * @file    drv_i2c_eeprom.h
 * @brief   I2C 外挂 EEPROM（AT24C02 等）驱动接口
 *
 * 硬件：I2C1，PB8=SCL / PB9=SDA；7bit 从机地址 0x50，8bit 帧地址 0xA0（写）/ 0xA1（读）。
 * 页写约束：AT24C01/02 每页 8 字节，跨页写由 I2C_EE_BufferWrite 自动分页。
 */

#ifndef __I2C_EE_H
#define __I2C_EE_H

#include "stm32f4xx.h"

/* -------------------------------------------------------------------------
 * EEPROM 容量与页大小（按实际芯片型号二选一）
 * ------------------------------------------------------------------------- */
#define EEPROM_PAGESIZE 8   /**< AT24C01/02：8 字节/页，256B 总容量 */
//#define EEPROM_PAGESIZE 16  /**< AT24C04/08A/16A：16 字节/页 */

/* I2C 本机 7bit 地址（Master 模式一般不被从机寻址，仅占位） */
#define I2C_OWN_ADDRESS7 0X0A

/* -------------------------------------------------------------------------
 * I2C1 外设与 GPIO 映射（STM32F4 AF4）
 * ------------------------------------------------------------------------- */
#define I2Cx                    I2C1
#define I2Cx_CLK_ENABLE()       __HAL_RCC_I2C1_CLK_ENABLE()
#define I2Cx_SDA_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define I2Cx_SCL_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define I2Cx_FORCE_RESET()      __HAL_RCC_I2C1_FORCE_RESET()
#define I2Cx_RELEASE_RESET()    __HAL_RCC_I2C1_RELEASE_RESET()

#define I2Cx_SCL_PIN            GPIO_PIN_8
#define I2Cx_SCL_GPIO_PORT      GPIOB
#define I2Cx_SCL_AF             GPIO_AF4_I2C1
#define I2Cx_SDA_PIN            GPIO_PIN_9
#define I2Cx_SDA_GPIO_PORT      GPIOB
#define I2Cx_SDA_AF             GPIO_AF4_I2C1

/* -------------------------------------------------------------------------
 * 超时与重试（HAL 轮询 / IsDeviceReady 用）
 * ------------------------------------------------------------------------- */
#define I2CT_FLAG_TIMEOUT       ((uint32_t)0x1000)              /**< 单次标志等待上限 */
#define I2CT_LONG_TIMEOUT       ((uint32_t)(10 * I2CT_FLAG_TIMEOUT)) /**< 长超时 */
#define I2Cx_TIMEOUT_MAX        300                             /**< Mem 读写单次超时(ms) */
#define EEPROM_MAX_TRIALS         300                             /**< 写周期完成后 Ready 探测次数 */

/* 调试打印开关：EEPROM_DEBUG_ON=1 时启用 DEBUG 宏 */
#define EEPROM_DEBUG_ON 0

#define EEPROM_INFO(fmt, arg...)  printf("<<-EEPROM-INFO->> " fmt "\n", ##arg)
#define EEPROM_ERROR(fmt, arg...) printf("<<-EEPROM-ERROR->> " fmt "\n", ##arg)
#define EEPROM_DEBUG(fmt, arg...) \
    do { \
        if (EEPROM_DEBUG_ON) \
            printf("<<-EEPROM-DEBUG->> [%d]" fmt "\n", __LINE__, ##arg); \
    } while (0)

/*
 * AT24C02：2kb = 256B，32 页 × 8 字节/页
 * 器件地址（7bit 左移后 8bit 帧格式）：
 *   1010 A2 A1 A0 R/W；A2=A1=A0=0 → 写 0xA0，读 0xA1
 */
#define EEPROM_Block0_ADDRESS 0xA0 /**< E2=0，主块 */
#define EEPROM_Block1_ADDRESS 0xA2 /**< 多片级联时块 1 */
#define EEPROM_Block2_ADDRESS 0xA4 /**< 多片级联时块 2 */
#define EEPROM_Block3_ADDRESS 0xA6 /**< 多片级联时块 3 */

#define EEPROM_ADDRESS        0xA0 /**< 当前使用的 8bit 写地址（R/W=0） */

/* -------------------------------------------------------------------------
 * 对外 API
 * ------------------------------------------------------------------------- */

/** @brief 初始化 I2C1 为 400kHz Fast-mode，供 EEPROM 访问 */
void I2C_EE_Init(void);

/**
 * @brief 按页边界自动分块写，避免跨页写失败
 * @param pBuffer         源数据
 * @param WriteAddr       EEPROM 内部字节地址（AT24C02：0~255）
 * @param NumByteToWrite  写入字节数
 */
void I2C_EE_BufferWrite(uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite);

/** @brief 写单字节，写后等待器件内部编程完成 */
uint32_t I2C_EE_ByteWrite(uint8_t *pBuffer, uint8_t WriteAddr);

/**
 * @brief 页内连续写（NumByteToWrite ≤ EEPROM_PAGESIZE，且不跨页）
 * @note  不可跨页；跨页场景由 I2C_EE_BufferWrite 处理
 */
uint32_t I2C_EE_PageWrite(uint8_t *pBuffer, uint8_t WriteAddr, uint8_t NumByteToWrite);

/** @brief 从指定地址连续读 NumByteToRead 字节 */
uint32_t I2C_EE_BufferRead(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);

/** @brief 本模块 HAL I2C 句柄（I2C_EE_Init / I2C_Mode_Config 初始化） */
extern I2C_HandleTypeDef I2C_Handle;

#endif /* __I2C_EE_H */
