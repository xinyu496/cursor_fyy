/**
 * @file    app_ctrl.c
 * @brief   俯仰板（GD）应用层控制实现
 *
 * 主要职责：
 *   - 系统初始化与主循环调度
 *   - 图像板 / 方位板 / 陀螺 串口通信接收
 *   - 编码器 SPI 读取、心跳灯等外设任务（ADC 由 bsp_adc 可选接入，当前工程未编入）
 *
 * 陀螺接收（USART6）：
 *   - 定长 13 字节，帧头 0xA2，byte[1~11] 异或校验
 *   - 驱动接口 COM_REC_DataAnalysis_1head_xor()
 *   - 角速度按大端 24bit 解析，结果写入 GyroRxData
 *
 * 源文件结构：
 *   - 通信辅助 / 新帧标志 / 运控给定缓存
 *   - 端序转换（LeUnpack / LePack / GyroBeUnpack）
 *   - 俯仰→图像/方位/驱动板 发送组包
 *   - 图像板伺服指令分发表（0x01~0x0A）
 *   - 光电校正持久化（Flash 或 EEPROM）
 *   - 串口接收 Init/Task
 *   - 系统 Init/Handle、编码器 SPI
 */
#include "Driver/drv_uart.h"
#if SF_INCLUDE
#include "../sf/sf_ctrl.h"
#endif
#if VIDEO_TRACK_INCLUDE
#include "../cam_track/Track.h"
#endif
#if LASER_INCLUDE
#include "../laser/laser_lrd_0301/laser_lrd_0301.h"
#include "../laser/laser_dyc_15a/laser_dyc_15a.h"

#endif

#if CLI_INCLUDE
#include "../cli/cli_cmd_line.h"
#endif

#if IR_CTRL_INCLUDE
#include "../ir/ir_ctrl_api.h"
#endif

#if ETH_INCLUDE
#include "Driver/drv_udp.h"
#endif
#include "BootApp/BootApp.h"
#include "Bsp/bsp_timer.h"

#include "../master_ctrl/master_ctrl.h"
#include "APP/app/app_ctrl.h"
#include "Bsp/bsp_timer.h"
#include "Bsp/bsp_uart.h"

#include "spi.h"
#include "Bsp/bsp_adc.h"
#include "Bsp/SEGGER_RTT.h"
#include <stdlib.h>
#include <string.h>
#include "Common/utl_check.h"
#include "main.h"
#if (USER_CTRL_OPT_CALIB_STORE == USER_CTRL_OPT_CALIB_STORE_FLASH)
#include "Bsp/bsp_flash.h"
#elif (USER_CTRL_OPT_CALIB_STORE == USER_CTRL_OPT_CALIB_STORE_EEPROM)
#include "Driver/drv_i2c_eeprom.h"
#include "i2c.h"
#endif
#include "tim.h"
#include "servo_module.h"
#include "APP/GEO_Track_C.h"
#include "SF.h"

/* -------------------------------------------------------------------------- */
/*                              模块私有变量                                     */
/* -------------------------------------------------------------------------- */
IMG_TO_GD_U ImgToGdCmd;                                /**< 图像→俯仰接收缓冲 */
FW_TO_GD_U FwToGdCmd;                                  /**< 方位→俯仰接收缓冲 */
GD_TO_IMG_U GdToImgCmd;                                /**< 俯仰→图像发送缓冲 */
GD_TO_FW_U GdToFwCmd;                                  /**< 俯仰→方位发送缓冲 */
GD_TO_MDRV_U GdToMdrvCmd;                              /**< 俯仰→驱动板发送缓冲 */
MDRV_TO_GD_U MdrvToGdCmd;                              /**< 驱动板→俯仰接收缓冲 */
GYRO_RX_U GyroRxFrame;                                 /**< 陀螺原始帧缓冲（最新一帧，13 字节） */
GYRO_RX_DATA_T GyroRxData;                             /**< 陀螺解析结果（X/Z 轴 raw 与 °/s） */

#define led_run HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12) /**< 运行指示灯 PB12 翻转 */

/* -------------------------------------------------------------------------- */
/*                              静态函数声明                                     */
/* 接收：Img/Fw/Gyro/Mdrv Init+Task；发送：Pack/Fill/Send；伺服：ImgSfCmd*     */
/* 持久化：OptCalib*；编码器：BmqSpi*                                            */
/* -------------------------------------------------------------------------- */
static void USER_Ctrl_ImgCmdRecvInit(void);
static void USER_Ctrl_FwCmdRecvInit(void);
static void USER_Ctrl_GyroCmdRecvInit(void);
static void USER_Ctrl_MdrvCmdRecvInit(void);
static void USER_Ctrl_ImgCmdRecvTask(void);
static void USER_Ctrl_FwCmdRecvTask(void);
static void USER_Ctrl_GyroCmdRecvTask(void);
static void USER_Ctrl_MdrvCmdRecvTask(void);
static void USER_Ctrl_ImgCmdSendInit(void);
static void USER_Ctrl_FwCmdSendInit(void);
static void USER_Ctrl_MdrvCmdSendInit(void);
static void USER_Ctrl_ImgCmdPack(GD_TO_IMG_U *frame);
static void USER_Ctrl_FwCmdPack(GD_TO_FW_U *frame);
static void USER_Ctrl_MdrvCmdPack(GD_TO_MDRV_U *frame);
static void USER_Ctrl_MdrvCmdFillPayload(void);
static void USER_Ctrl_FwCmdFillPayload(void);
static void USER_Ctrl_ImgCmdFillRandom(GD_TO_IMG_U *frame);
static void USER_Ctrl_ImgToGdLeUnpack(IMG_TO_GD_U *frame);
static void USER_Ctrl_FwToGdLeUnpack(FW_TO_GD_U *frame);
static void USER_Ctrl_GyroBeUnpack(const GYRO_RX_U *frame, GYRO_RX_DATA_T *out);
static void USER_Ctrl_GdToImgLePack(GD_TO_IMG_U *frame);
static void USER_Ctrl_GdToFwLePack(GD_TO_FW_U *frame);
static void USER_Ctrl_GdToMdrvLePack(GD_TO_MDRV_U *frame);
static uint8_t USER_Ctrl_MdrvToGdVerifyChecksum(const MDRV_TO_GD_U *frame);
static void USER_Ctrl_BmqSpiRecover(void);
static void USER_Ctrl_BmqSpiReadInit(void);
static void USER_Ctrl_ImgSfCmdDispatch(const IMG_TO_GD_U *cmd);

static void USER_Ctrl_ImgSfCmdBrake(const IMG_TO_GD_U *cmd);
static void USER_Ctrl_ImgSfCmdSpeedMove(const IMG_TO_GD_U *cmd);
static void USER_Ctrl_ImgSfCmdPosMove(const IMG_TO_GD_U *cmd);
static void USER_Ctrl_ImgSfCmdGuide(const IMG_TO_GD_U *cmd);
static void USER_Ctrl_ImgSfCmdPointTrack(const IMG_TO_GD_U *cmd);
static void USER_Ctrl_ImgSfCmdStow(const IMG_TO_GD_U *cmd);
static void USER_Ctrl_ImgSfCmdLock(const IMG_TO_GD_U *cmd);
static void USER_Ctrl_ImgSfCmdUnlock(const IMG_TO_GD_U *cmd);
static void USER_Ctrl_ImgSfCmdOptCalib(const IMG_TO_GD_U *cmd);
static void USER_Ctrl_ImgSfCmdStepMove(const IMG_TO_GD_U *cmd);
static void USER_Ctrl_OptCalibApply(void);
static void USER_Ctrl_OptCalibStoreLoad(void);
static void USER_Ctrl_OptCalibStoreSave(void);
#if (USER_CTRL_OPT_CALIB_STORE == USER_CTRL_OPT_CALIB_STORE_EEPROM)
static void USER_Ctrl_OptCalibEepromInit(void);
#endif

extern __IO uint16_t ADC1_CovertedValue[20];   /**< ADC1 DMA 缓冲（bsp_adc：通道数×采集次数，当前未编入工程） */
extern SPI_HandleTypeDef hspi1;                /**< SPI1 句柄（俯仰编码器） */

__IO uint8_t SPI1_rxbuf[SPI1_BMQ_RX_LEN];      /**< SPI1 DMA 编码器原始接收缓冲 */

volatile uint8_t dma_get_flag;                 /**< 保留标志位，当前工程未使用 */

static volatile uint8_t s_img_cmd_recv_fresh;  /**< 图像指令新帧标志：1=ImgToGdCmd 已更新 */
static volatile uint8_t s_fw_cmd_recv_fresh;   /**< 方位指令新帧标志：1=FwToGdCmd 已更新 */
static volatile uint8_t s_gyro_recv_fresh;     /**< 陀螺新帧标志：1=GyroRxFrame/GyroRxData 已更新 */
static volatile uint8_t s_mdrv_cmd_recv_fresh; /**< 驱动板应答新帧标志：1=MdrvToGdCmd 已更新 */
static int16_t s_mdrv_ctrl_current_mA;         /**< 运控电流给定缓存（mA，发送前写入 GdToMdrvCmd） */
static int16_t s_fw_ctrl_current_mA;           /**< 方位电流给定缓存（mA，发送前写入 GdToFwCmd） */
static uint8_t s_fw_motor_enable;              /**< 方位电机使能：0 关，1 开 */
static uint8_t s_fw_lock_enable;               /**< 电磁锁使能（协议 byte5） */
static int16_t s_fw_dev_angle_cdeg;            /**< 方位偏差角，0.01° */
static int16_t s_gd_dev_angle_cdeg;            /**< 俯仰偏差角，0.01° */
static uint8_t s_fw_servo_work_sta;            /**< 伺服工作状态（协议 byte10） */
static uint8_t s_fw_fault_code;                /**< 故障码（协议 byte12） */
static uint16_t s_gyro_rx_stale_ms;            /**< 陀螺有效帧超时计数（ms） */
static uint8_t s_spi1_bmq_busy_ms;             /**< SPI1 DMA 忙等待计数（ms） */

#define SPI1_BMQ_BUSY_TIMEOUT_MS     3U        /**< SPI 超时未就绪则复位总线 */
#define SPI1_BMQ_ABORT_TIMEOUT_MS    10U       /**< DMA Abort 等待超时 */
#define GYRO_RX_FAULT_TIMEOUT_MS     100U      /**< 陀螺无有效帧判故障阈值 */
#define FW_RECV_DRAIN_MAX_PER_LOOP   8U        /**< 方位接收：主循环每圈最多解析帧数（防环形缓冲堆积） */
#define GYRO_RECV_DRAIN_MAX_PER_LOOP 16U       /**< 陀螺接收：主循环每圈最多解析帧数（高频上报时取最新帧） */
#define MDRV_RECV_DRAIN_MAX_PER_LOOP 10U       /**< 驱动板接收：主循环每圈最多解析帧数（1ms 周期） */
#define MDRV_RX_STAGING_LEN          32U       /**< nocheck 临时缓冲，避免环形缓冲堆积时溢出 8 字节帧 */

#define USER_CTRL_OPT_CALIB_RECORD_MAGIC   0x434F4C42U  /**< 'COLB'，光电校正记录魔数 */
#if (USER_CTRL_OPT_CALIB_STORE == USER_CTRL_OPT_CALIB_STORE_FLASH)
#define USER_CTRL_OPT_CALIB_STORE_ADDR   FLASH_F4ADDR_SECTOR_7  /**< Flash 存储扇区 */
#endif

/* -------------------------------------------------------------------------- */
/*           光电校正持久化记录（26B，Flash/EEPROM 共用，见 USER_CTRL_OPT_CALIB_STORE） */
/* -------------------------------------------------------------------------- */
#pragma pack(push, 1)
typedef struct {
    uint32_t magic;    /**< 有效标志，固定 USER_CTRL_OPT_CALIB_RECORD_MAGIC */
    int32_t lon;       /**< 经度，LSB=0.00001° */
    int32_t lat;       /**< 纬度，LSB=0.00001° */
    int32_t alt;       /**< 高度，m */
    int32_t fw_comp;   /**< 方位补偿，0.01° */
    int32_t gd_comp;   /**< 俯仰补偿，0.01° */
    uint16_t crc;      /**< CRC16，覆盖 magic~gd_comp 共 24 字节 */
} USER_Ctrl_OptCalibRecord_t;
#pragma pack(pop)

#define USER_CTRL_OPT_CALIB_RECORD_CRC_LEN  24U  /**< magic + 5×int32，不含 crc 本身 */
#define USER_CTRL_OPT_CALIB_RECORD_SIZE     26U  /**< 紧凑布局总字节数 */

/** 光电校正 RAM 缓存，FillRandom/0x09 命令与持久化读写均使用此结构 */
static USER_Ctrl_OptCalibRecord_t s_opt_calib_record;

volatile uint32_t FY_BMP_Val;                 /**< 编码器解析后的原始码值（25 位） */
volatile uint32_t WHGBMQ_Data1;                /**< 编码器码值备份 1 */
volatile uint32_t WHGBMQ_Data2;                /**< 编码器码值备份 2 */
volatile float angle_real;                     /**< 编码器码值换算后的角度（°） */




uint8_t image_connect_cnt , image_connect_error;

/* -------------------------------------------------------------------------- */
/*                              通信辅助接口                                     */
/* -------------------------------------------------------------------------- */

/**
 * @brief  计算累加和校验（低 8 位）
 * @param  buf  待校验数据首地址
 * @param  len  参与累加的字节数（不含 checksum 本身）
 * @return 校验字节，可直接赋给 frame.checksum
 * @note   发送组帧时使用；接收校验由 COM_REC_DataAnalysis_1head_accu 内部完成
 */
uint8_t USER_Ctrl_CalcChecksum(const uint8_t *buf, uint16_t len)
{
    return (uint8_t)UTL_ADD_CHECK((uint8_t *)buf, len);
}

/**
 * @brief  计算陀螺帧异或校验
 * @param  buf  待校验数据首地址，通常为 &GyroRxFrame.buf[1]
 * @param  len  参与异或的字节数（GYRO_CHECKSUM_LEN = 11）
 * @return 校验字节
 * @note   正常接收时 COM_REC_DataAnalysis_1head_xor 已校验，本函数供组帧或调试使用
 */
uint8_t USER_Ctrl_GyroCalcXorChecksum(const uint8_t *buf, uint16_t len)
{
    uint8_t xor_val = 0U;
    uint16_t i;

    if (buf == NULL) {
        return 0U;
    }

    for (i = 0U; i < len; i++) {
        xor_val ^= buf[i];
    }
    return xor_val;
}

/**
 * @brief  查询图像指令是否收到新帧
 * @return 1=有新数据，0=无新数据
 * @note   业务处理完毕后应调用 USER_Ctrl_ImgCmdRecvClearFresh()
 */
uint8_t USER_Ctrl_ImgCmdRecvIsFresh(void)
{
    return (uint8_t)s_img_cmd_recv_fresh;
}

/**
 * @brief  查询方位指令是否收到新帧
 * @return 1=有新数据，0=无新数据
 * @note   业务处理完毕后应调用 USER_Ctrl_FwCmdRecvClearFresh()
 */
uint8_t USER_Ctrl_FwCmdRecvIsFresh(void)
{
    return (uint8_t)s_fw_cmd_recv_fresh;
}

/**
 * @brief  清除图像指令新帧标志
 */
void USER_Ctrl_ImgCmdRecvClearFresh(void)
{
    s_img_cmd_recv_fresh = 0;
}

/**
 * @brief  清除方位指令新帧标志
 */
void USER_Ctrl_FwCmdRecvClearFresh(void)
{
    s_fw_cmd_recv_fresh = 0;
}

/**
 * @brief  查询陀螺是否收到新帧
 * @return 1=有新数据，0=无新数据
 * @note   读取 GyroRxFrame / GyroRxData 后应调用 USER_Ctrl_GyroCmdRecvClearFresh()
 */
uint8_t USER_Ctrl_GyroCmdRecvIsFresh(void)
{
    return (uint8_t)s_gyro_recv_fresh;
}

/** @brief  清除陀螺新帧标志 */
void USER_Ctrl_GyroCmdRecvClearFresh(void)
{
    s_gyro_recv_fresh = 0;
}

/**
 * @brief  查询驱动板应答是否收到新帧
 */
uint8_t USER_Ctrl_MdrvCmdRecvIsFresh(void)
{
    return (uint8_t)s_mdrv_cmd_recv_fresh;
}

/** @brief  清除驱动板应答新帧标志 */
void USER_Ctrl_MdrvCmdRecvClearFresh(void)
{
    s_mdrv_cmd_recv_fresh = 0;
}

/**
 * @brief  更新驱动板运控给定（1kHz 发送前由伺服/电流环调用）
 */
void USER_Ctrl_MdrvSetRunCmd(int16_t current_mA, uint8_t enable)
{
    if (enable != 0U) {
        s_mdrv_ctrl_current_mA = current_mA;
        GdToMdrvCmd.field.motor_cmd = MDRV_CMD_MOTOR_ON;
    } else {
        s_mdrv_ctrl_current_mA = 0;
        GdToMdrvCmd.field.motor_cmd = MDRV_CMD_MOTOR_OFF;
    }
}

/**
 * @brief  将偏差角钳位到协议范围（±1000，0.01°）
 */
static int16_t USER_Ctrl_ClampDevAngleCdeg(int32_t cdeg)
{
    if (cdeg < GD_TO_FW_DEV_ANGLE_MIN) {
        return (int16_t)GD_TO_FW_DEV_ANGLE_MIN;
    }
    if (cdeg > GD_TO_FW_DEV_ANGLE_MAX) {
        return (int16_t)GD_TO_FW_DEV_ANGLE_MAX;
    }
    return (int16_t)cdeg;
}

/**
 * @brief  更新方位轴运控给定（1kHz 发送前由伺服/电流环调用）
 */
void USER_Ctrl_FwSetRunCmd(int16_t current_mA, uint8_t enable)
{
    s_fw_motor_enable = (enable != 0U) ? 1U : 0U;
    s_fw_ctrl_current_mA = (s_fw_motor_enable != 0U) ? current_mA : 0;
}

/** @brief  设置电磁锁使能（协议 byte5：0 关，1 开） */
void USER_Ctrl_FwSetLockEnable(uint8_t enable)
{
    s_fw_lock_enable = (enable != 0U) ? 1U : 0U;
}

/** @brief  设置方位/俯仰偏差角缓存，1kHz 经 GdToFwCmd 下发 */
void USER_Ctrl_FwSetDevAngle(int16_t fw_cdeg, int16_t gd_cdeg)
{
    s_fw_dev_angle_cdeg = USER_Ctrl_ClampDevAngleCdeg(fw_cdeg);
    s_gd_dev_angle_cdeg = USER_Ctrl_ClampDevAngleCdeg(gd_cdeg);
}

/** @brief  设置伺服工作状态（停止/引导/锁定），写入 GdToFwCmd.byte10 */
void USER_Ctrl_FwSetServoWorkSta(uint8_t sta)
{
    if (sta <= GD_TO_FW_SERVO_LOCK) {
        s_fw_servo_work_sta = sta;
    }
}

/** @brief  设置俯仰→方位帧 byte12 故障码 */
void USER_Ctrl_FwSetFaultCode(uint8_t code)
{
    s_fw_fault_code = code;
}

/**
 * @brief  俯仰编码器 SPI 是否异常（全 0xFF 或 DMA 长时间未就绪）
 */
static uint8_t USER_Ctrl_GdEncoderIsFault(void)
{
    if (s_spi1_bmq_busy_ms >= SPI1_BMQ_BUSY_TIMEOUT_MS) {
        return 1U;
    }

    if (SPI1_rxbuf[0] == 0xFF && SPI1_rxbuf[1] == 0xFF && SPI1_rxbuf[2] == 0xFF && SPI1_rxbuf[3] == 0xFF) {
        return 1U;
    }

    return 0U;
}

/**
 * @brief  陀螺接收是否异常
 */
static uint8_t USER_Ctrl_GyroIsFault(void)
{
    return (s_gyro_rx_stale_ms >= GYRO_RX_FAULT_TIMEOUT_MS) ? 1U : 0U;
}

/**
 * @brief  1ms 更新陀螺接收超时计数
 */
static void USER_Ctrl_GyroFaultTick1ms(void)
{
    if (s_gyro_rx_stale_ms < 0xFFFFU) {
        s_gyro_rx_stale_ms++;
    }
}

/* -------------------------------------------------------------------------- */
/*              端序转换：图像/方位为小端；陀螺角速度为大端 24bit               */
/* -------------------------------------------------------------------------- */

/**
 * @brief  图像→俯仰：按小端从 buf 解析多字节字段到 field
 */
static void USER_Ctrl_ImgToGdLeUnpack(IMG_TO_GD_U *frame)
{
    if (frame == NULL) {
        return;
    }

    frame->field.fw_miss = USER_CTRL_LE_GET_S16(&frame->buf[24]);
    frame->field.gd_miss = USER_CTRL_LE_GET_S16(&frame->buf[26]);
}

/**
 * @brief  方位→俯仰：按小端从 buf 解析多字节字段到 field
 */
static void USER_Ctrl_FwToGdLeUnpack(FW_TO_GD_U *frame)
{
    uint8_t *b;

    if (frame == NULL) {
        return;
    }

    b = frame->buf;
    frame->field.encoder_val = USER_CTRL_LE_GET_U32(&b[2]);
    frame->field.fw_current = USER_CTRL_LE_GET_S16(&b[6]);
    frame->field.vehicle_heading = USER_CTRL_LE_GET_U16(&b[11]);
    frame->field.vehicle_pitch = USER_CTRL_LE_GET_S16(&b[13]);
    frame->field.vehicle_roll = USER_CTRL_LE_GET_U16(&b[15]);
    frame->field.longitude = USER_CTRL_LE_GET_S32(&b[17]);
    frame->field.latitude = USER_CTRL_LE_GET_S32(&b[21]);
    frame->field.altitude = USER_CTRL_LE_GET_S32(&b[25]);
    frame->field.turntable_fw = USER_CTRL_LE_GET_U16(&b[29]);
    frame->field.turntable_gd = USER_CTRL_LE_GET_S16(&b[31]);
    frame->field.axis_x = USER_CTRL_LE_GET_S16(&b[34]);
    frame->field.axis_y = USER_CTRL_LE_GET_S16(&b[36]);
    frame->field.axis_z = USER_CTRL_LE_GET_S16(&b[38]);
    frame->field.time_year = USER_CTRL_LE_GET_U16(&b[40]);
    frame->field.time_ms = USER_CTRL_LE_GET_U16(&b[47]);
    frame->field.track_id = USER_CTRL_LE_GET_U32(&b[49]);

    /* 方位编码器异常时，置位俯仰→图像故障状态 bit2 */
    GdToImgCmd.field.fault_sta.fw_encoder_fault =
        (frame->field.fw_encoder_sta != 0U) ? 1U : 0U;
}

/**
 * @brief  陀螺帧大端解析：wire 字节 → 24bit 原始值 → °/s
 * @param  frame  原始帧，只读；来自 GyroRxFrame
 * @param  out    解析输出；通常传 &GyroRxData
 * @note   X/Z 角速度按 byte[4~6]、byte[7~9] 高→中→低组装为 int32，
 *         再乘以 GYRO_DPS_FACTOR（1/16384）得到 °/s；与图像/方位 LeUnpack 无关
 */
static void USER_Ctrl_GyroBeUnpack(const GYRO_RX_U *frame, GYRO_RX_DATA_T *out)
{
    if (frame == NULL || out == NULL) {
        return;
    }

    out->gyro_x_raw =
        USER_CTRL_GYRO_GET_S24(frame->field.gyro_x_h, frame->field.gyro_x_m, frame->field.gyro_x_l);
    out->gyro_z_raw =
        USER_CTRL_GYRO_GET_S24(frame->field.gyro_z_h, frame->field.gyro_z_m, frame->field.gyro_z_l);
    out->gyro_x_dps = (float)out->gyro_x_raw * GYRO_DPS_FACTOR;
    out->gyro_z_dps = (float)out->gyro_z_raw * GYRO_DPS_FACTOR;
}

/**
 * @brief  俯仰→图像：按小端把 field 多字节字段写入 buf（发送前调用）
 */
static void USER_Ctrl_GdToImgLePack(GD_TO_IMG_U *frame)
{
    uint8_t *b;

    if (frame == NULL) {
        return;
    }

    b = frame->buf;
    USER_CTRL_LE_PUT_U16(&b[2], frame->field.fw_angle);
    USER_CTRL_LE_PUT_S16(&b[4], frame->field.gd_angle);
    USER_CTRL_LE_PUT_S16(&b[6], frame->field.fw_speed);
    USER_CTRL_LE_PUT_S16(&b[8], frame->field.gd_speed);
    USER_CTRL_LE_PUT_S32(&b[10], frame->field.target_lon);
    USER_CTRL_LE_PUT_S32(&b[14], frame->field.target_lat);
    USER_CTRL_LE_PUT_S32(&b[18], frame->field.target_alt);
    USER_CTRL_LE_PUT_U16(&b[22], frame->field.time_year);
    USER_CTRL_LE_PUT_U16(&b[29], frame->field.time_ms);
    USER_CTRL_LE_PUT_U32(&b[31], frame->field.track_id);
    USER_CTRL_LE_PUT_U16(&b[37], frame->field.vis_focal);
    USER_CTRL_LE_PUT_U16(&b[44], frame->field.ir_focal);
    USER_CTRL_LE_PUT_S16(&b[51], frame->field.north_fw);
    USER_CTRL_LE_PUT_S16(&b[53], frame->field.center_gd);
}

/**
 * @brief  俯仰→方位：按小端把 field 多字节字段写入 buf（发送前调用）
 */
static void USER_Ctrl_GdToFwLePack(GD_TO_FW_U *frame)
{
    uint8_t *b;

    if (frame == NULL) {
        return;
    }

    b = frame->buf;
    USER_CTRL_LE_PUT_S16(&b[3], frame->field.ctrl_current);
    USER_CTRL_LE_PUT_S16(&b[6], frame->field.fw_dev_angle);
    USER_CTRL_LE_PUT_S16(&b[8], frame->field.gd_dev_angle);
}

/**
 * @brief  俯仰→驱动板：按小端把 field 多字节字段写入 buf（发送前调用）
 */
static void USER_Ctrl_GdToMdrvLePack(GD_TO_MDRV_U *frame)
{
    uint8_t *b;

    if (frame == NULL) {
        return;
    }

    b = frame->buf;
    USER_CTRL_LE_PUT_S16(&b[1], frame->field.ctrl_current);
    b[3] = frame->field.encoder_l;
    b[4] = frame->field.encoder_m;
    b[5] = frame->field.encoder_h;
    b[6] = frame->field.motor_cmd;
    USER_CTRL_LE_PUT_U16(&b[7], frame->field.param);
}

/**
 * @brief  刷新俯仰→驱动板发送缓冲中的运控字段（编码器 + 电流 + 使能）
 * @note   配置类命令（0xF0~0xFD）期间不覆盖 motor_cmd/param；非 0x01 时电流强制为 0
 */
uint32_t encoder_val_dbg = 0;
static void USER_Ctrl_MdrvCmdFillPayload(void)
{
    uint32_t enc24 = SPI_BMQ_TO_MDRV_ENC24(FY_BMP_Val);

    GdToMdrvCmd.field.encoder_l = (uint8_t)(enc24 & 0xFFU);
    GdToMdrvCmd.field.encoder_m = (uint8_t)((enc24 >> 8) & 0xFFU);
    GdToMdrvCmd.field.encoder_h = (uint8_t)((enc24 >> 16) & 0xFFU);

    encoder_val_dbg =
        GdToMdrvCmd.field.encoder_h << 16 | GdToMdrvCmd.field.encoder_m << 8 | GdToMdrvCmd.field.encoder_l;

    if (GdToMdrvCmd.field.motor_cmd == MDRV_CMD_MOTOR_ON) {
        GdToMdrvCmd.field.ctrl_current = s_mdrv_ctrl_current_mA;
    } else if (GdToMdrvCmd.field.motor_cmd == MDRV_CMD_MOTOR_OFF) {
        GdToMdrvCmd.field.ctrl_current = 0;
    } else {
        /* 0xF0~0xFD 配置命令：电流环输入保持 0 */
        GdToMdrvCmd.field.ctrl_current = 0;
    }
}

/**
 * @brief  刷新俯仰→方位发送缓冲（协议：内部协议 俯仰至方位，1kHz）
 * @note   byte2 电机使能；byte3-4 控制电流(0.001A)；byte5 电磁锁；
 *         byte6-9 方位/俯仰偏差角(0.01°)；byte10 伺服状态；byte11 故障状态；byte12 故障码
 */
static void USER_Ctrl_FwCmdFillPayload(void)
{
    GD_TO_FW_FAULT_T fault_sta = {0};

    if (s_mdrv_cmd_recv_fresh != 0U) {
        s_mdrv_cmd_recv_fresh = 0U;
    }

    *(uint8_t *)&fault_sta = *(uint8_t *)&GdToImgCmd.field.fault_sta;

//    GdToFwCmd.field.motor_enable = s_fw_motor_enable;
//    GdToFwCmd.field.ctrl_current = s_fw_ctrl_current_mA;
    GdToFwCmd.field.lock_enable = s_fw_lock_enable;
    GdToFwCmd.field.fw_dev_angle = s_fw_dev_angle_cdeg;
    GdToFwCmd.field.gd_dev_angle = s_gd_dev_angle_cdeg;
    GdToFwCmd.field.servo_work_sta = s_fw_servo_work_sta;

    fault_sta.fw_encoder_fault = (FwToGdCmd.field.fw_encoder_sta != 0U) ? 1U : 0U;
    fault_sta.gd_encoder_fault = USER_Ctrl_GdEncoderIsFault();
    fault_sta.gyro_fault = USER_Ctrl_GyroIsFault();
    fault_sta.servo_comm_fault = (FwToGdCmd.field.ext_servo_comm_sta != 0U) ? 1U : 0U;
    GdToFwCmd.field.fault_sta = fault_sta;
    GdToFwCmd.field.fault_code = s_fw_fault_code | (*(uint8_t *)&fault_sta);
	GdToFwCmd.field.fault_code |= image_connect_error;
}

/**
 * @brief  刷新俯仰→图像发送缓冲中的故障状态等字段
 * @note   编码器故障与 USER_Ctrl_FwCmdFillPayload 同源判定（USER_Ctrl_GdEncoderIsFault）
 */
static void USER_Ctrl_ImgCmdFillPayload(void)
{
    GdToImgCmd.field.fault_sta.gd_encoder_fault = USER_Ctrl_GdEncoderIsFault();
    GdToImgCmd.field.fault_sta.fw_encoder_fault =
        (FwToGdCmd.field.fw_encoder_sta != 0U) ? 1U : 0U;
    GdToImgCmd.field.fault_sta.gyro_fault = USER_Ctrl_GyroIsFault();
}

/* -------------------------------------------------------------------------- */
/*                         俯仰 → 图像 / 方位 发送                               */
/* -------------------------------------------------------------------------- */

/**
 * @brief  俯仰→图像发送缓冲初始化
 * @note   清零并预置帧头、长度字段；在 APP_Ctrl_System_Init 中调用
 */
static void USER_Ctrl_ImgCmdSendInit(void)
{
    memset(&GdToImgCmd, 0, sizeof(GdToImgCmd));
    GdToImgCmd.field.head = GD_IMG_FRAME_HEAD;
    GdToImgCmd.field.len = (uint8_t)GD_TO_IMG_LEN_VAL;
}

/**
 * @brief  俯仰→方位发送缓冲初始化
 */
static void USER_Ctrl_FwCmdSendInit(void)
{
    memset(&GdToFwCmd, 0, sizeof(GdToFwCmd));
    GdToFwCmd.field.head = GD_FW_FRAME_HEAD;
    GdToFwCmd.field.len = (uint8_t)GD_TO_FW_LEN_VAL;
}

/**
 * @brief  俯仰→驱动板发送缓冲初始化
 */
static void USER_Ctrl_MdrvCmdSendInit(void)
{
    memset(&GdToMdrvCmd, 0, sizeof(GdToMdrvCmd));
    GdToMdrvCmd.field.head = MDRV_FRAME_HEAD;
    memset(&MdrvToGdCmd, 0, sizeof(MdrvToGdCmd));
}

/**
 * @brief  填充俯仰→图像 payload（角/速来自伺服，时间等字段为随机数；head/len/checksum 由组包处理）
 * @param  frame  待填充帧，不可为 NULL
 */
static void USER_Ctrl_ImgCmdFillRandom(GD_TO_IMG_U *frame)
{
    if (frame == NULL) {
        return;
    }

    frame->field.fw_angle = g_servo.fw_axis.sensor.angle_360 * 100;  /* 方位角，0.01° */
	if(g_servo.gd_axis.sensor.angle_360>90)
	{
		frame->field.gd_angle = (g_servo.gd_axis.sensor.angle_360-360) * 100;  /* 俯仰>90° 折到负角 */
	}
	else
	{
		frame->field.gd_angle = g_servo.gd_axis.sensor.angle_360 * 100;
	}
    
    frame->field.fw_speed = g_servo.fw_axis.sensor.ev * 100;         /* 方位角速度，0.01°/s */
    frame->field.gd_speed = g_servo.gd_axis.sensor.ev * 100;         /* 俯仰角速度，0.01°/s */
    /* 经纬高来自持久化缓存，非随机数 */
    frame->field.target_lon = s_opt_calib_record.lon;
    frame->field.target_lat = s_opt_calib_record.lat;
    frame->field.target_alt = s_opt_calib_record.alt;
    frame->field.time_year = (uint16_t)rand();
    frame->field.time_month = (uint8_t)((rand() % 12U) + 1U);
    frame->field.time_day = (uint8_t)((rand() % 28U) + 1U);
    frame->field.time_hour = (uint8_t)(rand() % 24U);
    frame->field.time_min = (uint8_t)(rand() % 60U);
    frame->field.time_sec = (uint8_t)(rand() % 60U);
    frame->field.time_ms = (uint16_t)(rand() % 1000U);
    frame->field.track_id = (uint32_t)(((uint32_t)rand() << 16) | (uint32_t)rand());
//    *(uint8_t *)&frame->field.fault_sta = (uint8_t)rand();
	uint8_t warning_state = (g_servo.fw_axis.status.in_place << 1) | g_servo.gd_axis.status.in_place;
    *(uint8_t *)&frame->field.alarm_sta = warning_state; /* bit0 俯仰到位，bit1 方位到位 */
//    frame->field.vis_focal = (uint16_t)rand();
//    *(uint8_t *)&frame->field.vis_sta = (uint8_t)rand();
//    frame->field.brightness = (uint8_t)rand();
//    frame->field.contrast = (uint8_t)rand();
//    frame->field.saturation = (uint8_t)rand();
//    frame->field.ele_zoom = (uint8_t)rand();
//    frame->field.ir_focal = (uint16_t)rand();
//    *(uint8_t *)&frame->field.ir_sta = (uint8_t)rand();
//    frame->field.ir_contrast = (uint8_t)rand();
//    frame->field.ir_brightness = (uint8_t)rand();
//    frame->field.integration_time = (uint8_t)rand();
//    frame->field.noise_filter = (uint8_t)(rand() & 0x01U);
//    frame->field.north_fw = (int16_t)rand();
//    frame->field.center_gd = (int16_t)rand();
}

/**
 * @brief  俯仰→图像帧组包：补帧头、长度、累加和
 * @param  frame  待组包帧，不可为 NULL
 * @note   校验范围 byte[0]~byte[72]，结果写入 byte[73]（整帧 74 字节）
 */
static void USER_Ctrl_ImgCmdPack(GD_TO_IMG_U *frame)
{
    if (frame == NULL) {
        return;
    }

    frame->field.head = GD_IMG_FRAME_HEAD;
    frame->field.len = (uint8_t)GD_TO_IMG_LEN_VAL;
    USER_Ctrl_GdToImgLePack(frame);
    frame->field.checksum = USER_Ctrl_CalcChecksum(frame->buf, GD_TO_IMG_CHECKSUM_LEN - 1);
}


/**
 * @brief  俯仰→方位帧组包：补帧头、长度、累加和
 * @param  frame  待组包帧，不可为 NULL
 * @note   校验范围 byte[0]~byte[12]，结果写入 byte[13]
 */
static void USER_Ctrl_FwCmdPack(GD_TO_FW_U *frame)
{
    if (frame == NULL) {
        return;
    }

    frame->field.head = GD_FW_FRAME_HEAD;
    frame->field.len = (uint8_t)GD_TO_FW_LEN_VAL;
    USER_Ctrl_GdToFwLePack(frame);
    frame->field.checksum = USER_Ctrl_CalcChecksum(frame->buf, GD_TO_FW_CHECKSUM_LEN);
}

/**
 * @brief  俯仰→驱动板帧组包：补帧头、累加和
 * @param  frame  待组包帧，不可为 NULL
 * @note   校验范围 byte[1]~byte[8]，结果写入 byte[9]
 */
static void USER_Ctrl_MdrvCmdPack(GD_TO_MDRV_U *frame)
{
    if (frame == NULL) {
        return;
    }

    frame->field.head = MDRV_FRAME_HEAD;
    USER_Ctrl_GdToMdrvLePack(frame);
    frame->field.checksum = USER_Ctrl_CalcChecksum(&frame->buf[1], GD_TO_MDRV_CHECKSUM_PAYLOAD_LEN);
}

/**
 * @brief  校验驱动板→俯仰应答帧累加和
 */
static uint8_t USER_Ctrl_MdrvToGdVerifyChecksum(const MDRV_TO_GD_U *frame)
{
    uint8_t calc;

    if (frame == NULL || frame->field.head != MDRV_FRAME_HEAD) {
        return 0U;
    }

    calc = USER_Ctrl_CalcChecksum(&frame->buf[1], MDRV_TO_GD_CHECKSUM_PAYLOAD_LEN);
    return (calc == frame->field.checksum) ? 1U : 0U;
}

/**
 * @brief  俯仰→图像板发送
 * @param  frame  待发送帧；NULL 时使用 GdToImgCmd
 * @note   通信口 COM_GD_IMG（UART5），整帧 74 字节
 */
void USER_Ctrl_ImgCmdSend(GD_TO_IMG_U *frame)
{
    GD_TO_IMG_U *tx = (frame != NULL) ? frame : &GdToImgCmd;

    if (frame == NULL) {
        USER_Ctrl_ImgCmdFillPayload();
    }

    USER_Ctrl_ImgCmdPack(tx);
    COM_API_Send_Data(COM_GD_IMG, tx->buf, (uint16_t)GD_TO_IMG_FRAME_LEN);
}
/**
 * @brief  俯仰→方位板发送
 * @param  frame  待发送帧；NULL 时使用 GdToFwCmd
 * @note   通信口 COM_GD_FW（USART2），整帧 14 字节，建议 1kHz 周期调用
 */
void USER_Ctrl_FwCmdSend(GD_TO_FW_U *frame)
{
    GD_TO_FW_U *tx = (frame != NULL) ? frame : &GdToFwCmd;

    if (frame == NULL) {
        USER_Ctrl_FwCmdFillPayload();
    }

    USER_Ctrl_FwCmdPack(tx);
    COM_API_Send_Data(COM_GD_FW, tx->buf, (uint16_t)GD_TO_FW_FRAME_LEN);
}

uint8_t driver_board_debug_cmd, enable_dbug;   /**< 驱动板调试：命令码 / 使能 */
int16_t i_give_dbg;                            /**< 驱动板调试：电流给定 mA */
uint16_t driver_board_debug_cnt, driver_board_debug_enable, driver_board_debug_para;
/**
 * @brief  俯仰→驱动板发送
 * @param  frame  待发送帧；NULL 时使用 GdToMdrvCmd
 * @note   通信口 COM_GD_MDRV，整帧 10 字节，建议 1ms 周期调用
 */
void USER_Ctrl_MdrvCmdSend(GD_TO_MDRV_U *frame)
{
    GD_TO_MDRV_U *tx = (frame != NULL) ? frame : &GdToMdrvCmd;

    if (frame == NULL) {
        USER_Ctrl_MdrvCmdFillPayload();
    }
	
	tx->field.ctrl_current =  GDControl.I_give*1000;
//	tx->field.motor_cmd = 

    if (driver_board_debug_cnt) {  //电机驱动配置
        driver_board_debug_cnt--;
        tx->field.ctrl_current = i_give_dbg;
        tx->field.motor_cmd = driver_board_debug_cmd;
        tx->field.param = driver_board_debug_para;
    } else if (driver_board_debug_enable) {  //电机驱动使用：调试可用
        tx->field.motor_cmd = enable_dbug;   /* 调试：电机命令/使能 */
        tx->field.ctrl_current = i_give_dbg; /* 调试：电流给定 mA */
        tx->field.param = 0;
    } else {                                 // 正常控制走这条分支
        tx->field.param = 0;
    }
    USER_Ctrl_MdrvCmdPack(tx);
    COM_API_Send_Data(COM_GD_MDRV, tx->buf, (uint16_t)GD_TO_MDRV_FRAME_LEN);
}

/* -------------------------------------------------------------------------- */
/*              图像板 → 俯仰：伺服控制指令处理（函数指针分发表）                  */
/* -------------------------------------------------------------------------- */

/**
 * @brief  图像板伺服控制指令分发表
 * @note   recv_addr == REC_ADDR_SF 且帧校验通过后，由 USER_Ctrl_ImgSfCmdDispatch 查表调用
 */
static const USER_Ctrl_ImgSfCmdEntry_t s_img_sf_cmd_table[] = {
    {IMG_SF_CMD_BRAKE, USER_Ctrl_ImgSfCmdBrake},
    {IMG_SF_CMD_SPEED_MOVE, USER_Ctrl_ImgSfCmdSpeedMove},
    {IMG_SF_CMD_POS_MOVE, USER_Ctrl_ImgSfCmdPosMove},
    {IMG_SF_CMD_GUIDE, USER_Ctrl_ImgSfCmdGuide},
    {IMG_SF_CMD_POINT_TRACK, USER_Ctrl_ImgSfCmdPointTrack},
    {IMG_SF_CMD_STOW, USER_Ctrl_ImgSfCmdStow},
    {IMG_SF_CMD_LOCK, USER_Ctrl_ImgSfCmdLock},
    {IMG_SF_CMD_UNLOCK, USER_Ctrl_ImgSfCmdUnlock},
    {IMG_SF_CMD_OPT_CALIB, USER_Ctrl_ImgSfCmdOptCalib},
    {IMG_SF_CMD_STEP_MOVE, USER_Ctrl_ImgSfCmdStepMove},
};

#define IMG_SF_CMD_TABLE_SIZE (sizeof(s_img_sf_cmd_table) / sizeof(s_img_sf_cmd_table[0]))

/**
 * @brief  按 ctrl_cmd 查表分发伺服控制指令
 * @param  cmd  已校验通过的图像→俯仰帧
 * @note   未在表中找到的指令码将被忽略
 */
static void USER_Ctrl_ImgSfCmdDispatch(const IMG_TO_GD_U *cmd)
{
    uint16_t i;

    if (cmd == NULL) {
        return;
    }

    for (i = 0; i < IMG_SF_CMD_TABLE_SIZE; i++) {
        if (s_img_sf_cmd_table[i].cmd != cmd->field.ctrl_cmd) {
            continue;
        }
        if (s_img_sf_cmd_table[i].handler != NULL) {
            s_img_sf_cmd_table[i].handler(cmd);
        }
        return;
    }
}

/**
 * @brief  0x01 刹车
 * @note   参数区无额外有效载荷
 */
static void USER_Ctrl_ImgSfCmdBrake(const IMG_TO_GD_U *cmd)
{
    (void)cmd;
    servo_ext_cmd_t excmd = {0};
    excmd.cmd = SERVO_EXT_CMD_BRAKE;
//    servo_module_on_command(&excmd);

    // USER_Ctrl_FwSetServoWorkSta(GD_TO_FW_SERVO_STOP);
    // USER_Ctrl_FwSetRunCmd(0, 0U);
}

/**
 * @brief  0x02 速度运动
 * @note   param1/2：方位速度（Int16）；param3/4：俯仰速度（Int16）
 */
static void USER_Ctrl_ImgSfCmdSpeedMove(const IMG_TO_GD_U *cmd)
{
    const uint8_t *p;

    if (cmd == NULL) {
        return;
    }

    // USER_Ctrl_FwSetServoWorkSta(GD_TO_FW_SERVO_STOP);

    p = &cmd->field.param1;
    int16_t fw_speed = USER_CTRL_LE_GET_S16(&p[0]); /* 方位速度 Int16，param1=低字节 param2=高字节 */
    int16_t fy_speed = USER_CTRL_LE_GET_S16(&p[2]); /* 俯仰速度 Int16，param3=低字节 param4=高字节 */

    // int16_t fw_speed = (int16_t)((data[1] << 8) | data[0]);
    // int16_t fy_speed = (int16_t)((data[3] << 8) | data[2]);

    servo_ext_cmd_t excmd = {0};
    excmd.cmd = SERVO_EXT_CMD_VELOCITY;
    excmd.p.velocity.fw = fw_speed;
    excmd.p.velocity.gd = fy_speed;
//    servo_module_on_command(&excmd);
}

/**
 * @brief  0x03 位置运动
 * @note   param1/2：方位角度（Uint16）；param3/4：俯仰角度（Int16）
 */
static void USER_Ctrl_ImgSfCmdPosMove(const IMG_TO_GD_U *cmd)
{
    const uint8_t *p;

    if (cmd == NULL) {
        return;
    }

    // USER_Ctrl_FwSetServoWorkSta(GD_TO_FW_SERVO_STOP);

    p = &cmd->field.param1;
    int16_t fw_angle = USER_CTRL_LE_GET_U16(&p[0]); /* 方位角度 Uint16 */
    int16_t fy_angle = USER_CTRL_LE_GET_S16(&p[2]); /* 俯仰角度 Int16 */

    // int16_t fw_angle = (int16_t)((data[1] << 8) | data[0]);
    // int16_t fy_angle = (int16_t)((data[3] << 8) | data[2]);

    servo_ext_cmd_t excmd = {0};
    excmd.cmd = SERVO_EXT_CMD_POSITION;
    excmd.p.position.fw = fw_angle;
    excmd.p.position.gd = fy_angle;
//    servo_module_on_command(&excmd);
}
/**********************地理引导全局变量**********************/
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
/** 角度制 → 弧度 */
double deg__(double deg)
{
    return deg * M_PI / 180.0;
}
GeoPoint_t middle_self, middle_target , gps_self , gps_target;
/***************************************************************/
/**
 * @brief  0x04 引导
 * @note   param1~12：经度/纬度/高度（Int32）；param13：0 停止跟踪，1 开始跟踪
 */
static void USER_Ctrl_ImgSfCmdGuide(const IMG_TO_GD_U *cmd)
{
    const uint8_t *p;

    if (cmd == NULL) {
        return;
    }
	p = &cmd->field.param1;
	int lon =    USER_CTRL_LE_GET_S32(&p[0]); /* 经度 Int32 */
	int lat =    USER_CTRL_LE_GET_S32(&p[4]); /* 纬度 Int32 */
	float high = (float)USER_CTRL_LE_GET_S32(&p[8]); /* 高度 Int32 */
	
	 /* 从 param 区解析目标经纬高（小端 Int32） */
    double longtitude = (double)(lon * 0.00001f);
    double latitude   = (double)(lat * 0.00001f);

    /* 写入 GEO 解算目标点 */
    middle_target.alt_m   = high;
    middle_target.lon_deg = (float)longtitude;
    middle_target.lat_deg = (float)latitude;
	
	Target.lattitude = deg__(middle_target.lat_deg);	//目标值
	Target.longitude = deg__(middle_target.lon_deg);	//目标值
	Target.height = middle_target.alt_m;            	//目标值
	INS.lattitude = deg__(middle_self.lat_deg);     	//自身值
	INS.longitude = deg__(middle_self.lon_deg);     	//自身值
	INS.height = middle_self.alt_m;                 	//自身值
//	PT_Angle.Fw = FWControl.P_fb;						//实测方位俯仰值 林辉程序*需修改飞总程序变量
//	PT_Angle.Fy = GDControl.P_fb;						//实测方位俯仰值 林辉程序*需修改飞总程序变量
	PT_Angle.Hg = 0;									//横滚填0

	TGTV = TGT(Target,INS);//目标与载机连线在大地系下的角度
	Triangle_Fun_Cal_Result = Triangle_Fun_Cal(INS,PT_Angle);
	OrientLoad = OrientLoadCal(TGTV.Pitch,TGTV.Yaw,Triangle_Fun_Cal_Result);

#if DEBUG_UART_DRIVER
    SEGGER_RTT_SetTerminal(2);
    SEGGER_RTT_printf(0, "receive guide cmd\n");
    SEGGER_RTT_SetTerminal(0);
#endif
}

/**
 * @brief  0x05 点选跟踪
 */
static void USER_Ctrl_ImgSfCmdPointTrack(const IMG_TO_GD_U *cmd)
{
    int16_t fw_miss;
    int16_t gd_miss;

    if (cmd == NULL) {
        return;
    }

    if (cmd->field.miss_valid != 0U) {
        fw_miss = USER_CTRL_LE_GET_S16(&cmd->buf[24]);
        gd_miss = USER_CTRL_LE_GET_S16(&cmd->buf[26]);

        // USER_Ctrl_FwSetDevAngle(fw_miss, gd_miss);
    }

    uint8_t is_track = cmd->field.miss_valid;

    servo_ext_cmd_t excmd = {0};
    excmd.cmd = SERVO_EXT_CMD_TRACK;
    excmd.p.track.fw_miss = fw_miss * 0.01f;
    excmd.p.track.gd_miss = gd_miss * 0.01f;
    excmd.p.track.is_track = is_track;
//    servo_module_on_command(&excmd);
}

/**
 * @brief  0x06 收藏
 */
static void USER_Ctrl_ImgSfCmdStow(const IMG_TO_GD_U *cmd)
{
    (void)cmd;
    // USER_Ctrl_FwSetServoWorkSta(GD_TO_FW_SERVO_STOP);

    servo_ext_cmd_t excmd = {0};
    excmd.cmd = SERVO_EXT_CMD_STOW;
//    servo_module_on_command(&excmd);
}

/**
 * @brief  0x07 上锁
 */
static void USER_Ctrl_ImgSfCmdLock(const IMG_TO_GD_U *cmd)
{
    (void)cmd;

	if((GdToImgCmd.field.gd_angle <= 5) || (GdToImgCmd.field.gd_angle >= 95))
	{
		servo_lock_on_command(LOCK_CMD_LOCK, 5000);
	}
    
	if((GdToImgCmd.field.fw_angle <= 30) || (GdToImgCmd.field.fw_angle >= 35800))
	{
		USER_Ctrl_FwSetLockEnable(1U);
	}
    
}

/**
 * @brief  0x08 解锁
 */
static void USER_Ctrl_ImgSfCmdUnlock(const IMG_TO_GD_U *cmd)
{
    (void)cmd;

    servo_lock_on_command(LOCK_CMD_UNLOCK, 5000);
    USER_Ctrl_FwSetLockEnable(0U);
}

/**
 * @brief  将持久化缓存的经纬高/补偿应用到运行时变量
 */
static void USER_Ctrl_OptCalibApply(void)
{
    GdToImgCmd.field.target_lon = s_opt_calib_record.lon;
    GdToImgCmd.field.target_lat = s_opt_calib_record.lat;
    GdToImgCmd.field.target_alt = s_opt_calib_record.alt;
    s_fw_dev_angle_cdeg = USER_Ctrl_ClampDevAngleCdeg(s_opt_calib_record.fw_comp);
    s_gd_dev_angle_cdeg = USER_Ctrl_ClampDevAngleCdeg(s_opt_calib_record.gd_comp);
}

/**
 * @brief  校验光电校正记录 magic 与 CRC16
 * @return 1=有效，0=无效
 */
static uint8_t USER_Ctrl_OptCalibRecordIsValid(const USER_Ctrl_OptCalibRecord_t *rec)
{
    uint16_t crc;

    if (rec == NULL) {
        return 0U;
    }
    if (rec->magic != USER_CTRL_OPT_CALIB_RECORD_MAGIC) {
        return 0U;
    }

    crc = UTL_CRC16_CCITT((uint8_t *)rec, USER_CTRL_OPT_CALIB_RECORD_CRC_LEN);
    return (crc == rec->crc) ? 1U : 0U;
}

/** @brief  写入前刷新 magic 并计算 CRC16 */
static void USER_Ctrl_OptCalibRecordUpdateCrc(void)
{
    uint8_t *buf = (uint8_t *)&s_opt_calib_record;

    s_opt_calib_record.magic = USER_CTRL_OPT_CALIB_RECORD_MAGIC;
    s_opt_calib_record.crc = UTL_CRC16_CCITT(buf, USER_CTRL_OPT_CALIB_RECORD_CRC_LEN);
}

#if (USER_CTRL_OPT_CALIB_STORE == USER_CTRL_OPT_CALIB_STORE_FLASH)

/** @brief  从 Flash Sector 7 加载光电校正记录 */
static void USER_Ctrl_OptCalibFlashLoad(void)
{
    USER_Ctrl_OptCalibRecord_t tmp;

    Flash_Read(USER_CTRL_OPT_CALIB_STORE_ADDR, (uint8_t *)&tmp, USER_CTRL_OPT_CALIB_RECORD_SIZE);
    if (USER_Ctrl_OptCalibRecordIsValid(&tmp) == 0U) {
        memset(&s_opt_calib_record, 0, sizeof(s_opt_calib_record));
        return;
    }

    s_opt_calib_record = tmp;
    USER_Ctrl_OptCalibApply();
}

/** @brief  擦除扇区并写入光电校正记录到 Flash */
static void USER_Ctrl_OptCalibFlashSave(void)
{
    uint8_t *buf = (uint8_t *)&s_opt_calib_record;

    USER_Ctrl_OptCalibRecordUpdateCrc();
    Flash_Erase_Sector(USER_CTRL_OPT_CALIB_STORE_ADDR);
    Flash_WriteNoErase(USER_CTRL_OPT_CALIB_STORE_ADDR, buf, USER_CTRL_OPT_CALIB_RECORD_SIZE);
}

#elif (USER_CTRL_OPT_CALIB_STORE == USER_CTRL_OPT_CALIB_STORE_EEPROM)

/** @brief  EEPROM 模式：绑定 CubeMX 已初始化的 I2C2 句柄 */
static void USER_Ctrl_OptCalibEepromInit(void)
{
    /* CubeMX 已初始化 hi2c2，绑定到 EEPROM 驱动句柄 */
    I2C_Handle = hi2c2;
}

/** @brief  从 EEPROM 指定地址加载光电校正记录 */
static void USER_Ctrl_OptCalibEepromLoad(void)
{
    USER_Ctrl_OptCalibRecord_t tmp;

    I2C_EE_BufferRead((uint8_t *)&tmp, USER_CTRL_OPT_CALIB_EEPROM_ADDR,
                      USER_CTRL_OPT_CALIB_RECORD_SIZE);
    if (USER_Ctrl_OptCalibRecordIsValid(&tmp) == 0U) {
        memset(&s_opt_calib_record, 0, sizeof(s_opt_calib_record));
        return;
    }

    s_opt_calib_record = tmp;
    USER_Ctrl_OptCalibApply();
}

/** @brief  将光电校正记录写入 EEPROM（自动分页） */
static void USER_Ctrl_OptCalibEepromSave(void)
{
    uint8_t *buf = (uint8_t *)&s_opt_calib_record;

    USER_Ctrl_OptCalibRecordUpdateCrc();
    I2C_EE_BufferWrite(buf, USER_CTRL_OPT_CALIB_EEPROM_ADDR, USER_CTRL_OPT_CALIB_RECORD_SIZE);
}

#endif

/** @brief  上电加载：按 USER_CTRL_OPT_CALIB_STORE 选择 Flash 或 EEPROM */
static void USER_Ctrl_OptCalibStoreLoad(void)
{
#if (USER_CTRL_OPT_CALIB_STORE == USER_CTRL_OPT_CALIB_STORE_FLASH)
    USER_Ctrl_OptCalibFlashLoad();
#elif (USER_CTRL_OPT_CALIB_STORE == USER_CTRL_OPT_CALIB_STORE_EEPROM)
    USER_Ctrl_OptCalibEepromLoad();
#endif
}

/** @brief  0x09 命令保存：按 USER_CTRL_OPT_CALIB_STORE 选择 Flash 或 EEPROM */
static void USER_Ctrl_OptCalibStoreSave(void)
{
#if (USER_CTRL_OPT_CALIB_STORE == USER_CTRL_OPT_CALIB_STORE_FLASH)
    USER_Ctrl_OptCalibFlashSave();
#elif (USER_CTRL_OPT_CALIB_STORE == USER_CTRL_OPT_CALIB_STORE_EEPROM)
    USER_Ctrl_OptCalibEepromSave();
#endif
}

/**
 * @brief  0x09 光电校正
 * @note   param1~12：经纬高（Int32，小端）；param13~20：方位/俯仰补偿（Int32，0.01°）
 */
static void USER_Ctrl_ImgSfCmdOptCalib(const IMG_TO_GD_U *cmd)
{
    const uint8_t *p;
    int32_t lon;
    int32_t lat;
    int32_t alt;
    int32_t fw_comp;
    int32_t gd_comp;
    uint8_t updated = 0U;
	float   roll;
    int16_t roll16;

    if (cmd == NULL) {
        return;
    }
	
    p = &cmd->field.param1;
    lon = USER_CTRL_LE_GET_S32(&p[0]);  	/* param1~4   经度，LSB=0.00001° */
    lat = USER_CTRL_LE_GET_S32(&p[4]);  	/* param5~8   纬度，LSB=0.00001° */
    roll16 = (int16_t)(p[11] << 8 | p[10]);
	roll = (float)roll16;					//横滚补偿
	alt = (int32_t)(p[9] << 8 | p[8]);
    fw_comp = USER_CTRL_LE_GET_S32(&p[12]); /* param13~16 方位补偿，0.01° */
    gd_comp = USER_CTRL_LE_GET_S32(&p[16]); /* param17~20 俯仰补偿，0.01° */

    /* —— 经纬高：无效值或超范围则不更新 —— */
    if (lon != 40000000 && lon >= -18000000 && lon <= 18000000) {
        GdToImgCmd.field.target_lon = lon;
        s_opt_calib_record.lon = lon;
        updated = 1U;
    }
    if (lat != 40000000 && lat >= -9000000 && lat <= 9000000) {
        GdToImgCmd.field.target_lat = lat;
        s_opt_calib_record.lat = lat;
        updated = 1U;
    }
    if (alt >= -1000 && alt <= 30000) {
        GdToImgCmd.field.target_alt = alt;
        s_opt_calib_record.alt = alt;
        updated = 1U;
    }

    /* —— 方位/俯仰补偿 → 俯仰→方位帧偏差角（缓存后由 1kHz 下发） —— */
    if (fw_comp >= -35999 && fw_comp <= 35999) {
        s_fw_dev_angle_cdeg = USER_Ctrl_ClampDevAngleCdeg(fw_comp);
        s_opt_calib_record.fw_comp = fw_comp;
        updated = 1U;
    }
	
	/* 横滚/俯仰补偿范围检查（原逻辑保留） */
    if ((roll > 180) && (roll < -180)) {
        return;
    }
    if ((gd_comp > 180) && (gd_comp < -180)) {
		gd_comp = 0;
        return;
    }

//    if (updated != 0U) {						//暂时不掉电保存
//        USER_Ctrl_OptCalibStoreSave();        //暂时不掉电保存
//    }                                         //暂时不掉电保存
	INS.Roll      = (float)roll * 0.01;
    INS.Pitch     = (float)gd_comp * 0.01;
	INS.Yaw       = (float)fw_comp * 0.01;
	
	middle_self.alt_m = alt;
	middle_self.lat_deg = (float)lat / 100000;
	middle_self.lon_deg = (float)lon / 100000;

    /* 方位补偿角归一化到 [0, 360) */
    if (INS.Yaw >= 360) {
        INS.Yaw -= 360;
    }
}
uint16_t debug_gd_step_len_u;
uint16_t debug_gd_step_len_f;
/**
 * @brief  0x0A 步进运动
 * @note   param1：方位方向（0x01 左 / 0x02 右）；
 *         param2/3：方位步长（Uint16，0~50°，0.01°）；
 *         param4：俯仰方向（0x01 上 / 0x02 下）；
 *         param5/6：俯仰步长（Uint16，0~20°，0.01°）
 */
static void USER_Ctrl_ImgSfCmdStepMove(const IMG_TO_GD_U *cmd)
{
    const uint8_t *p;

    if (cmd == NULL) {
        return;
    }

    // USER_Ctrl_FwSetServoWorkSta(GD_TO_FW_SERVO_STOP);

    p = &cmd->field.param1;
    uint8_t fw_dir = p[0];                              /* 方位方向 */
    uint16_t fw_step_len = USER_CTRL_LE_GET_U16(&p[1]); /* 方位步长，param2=低字节 param3=高字节 */
    uint8_t fy_dir = p[3];                              /* 俯仰方向 */
    uint16_t fy_step_len = USER_CTRL_LE_GET_U16(&p[4]); /* 俯仰步长，param5=低字节 param6=高字节 */

    // uint8_t fw_dir = data[0];          //1: 左 2：右
    // uint16_t fw_step_len = (uint16_t)((data[2] << 8) | data[1]);
    // uint8_t fy_dir = data[3];          //1: 上 2：下
    // uint16_t fy_step_len = (uint16_t)((data[4] << 8) | data[3]);

    servo_ext_cmd_t excmd = {0};
    excmd.cmd = SERVO_EXT_CMD_STEP;
    excmd.p.step.fw_dir = fw_dir;               /* 0x01 左 / 0x02 右 */
    excmd.p.step.fw_step = fw_step_len * 0.01f; /* 步长(°) */
    excmd.p.step.gd_dir = fy_dir;               /* 0x01 上 / 0x02 下 */
    excmd.p.step.gd_step = fy_step_len * 0.01f;
	debug_gd_step_len_f = excmd.p.step.gd_step;
	debug_gd_step_len_u = fy_step_len;
//    servo_module_on_command(&excmd);
}

/* -------------------------------------------------------------------------- */
/*                    图像 / 方位 / 陀螺 串口接收                               */
/* -------------------------------------------------------------------------- */

/**
 * @brief  图像链路串口接收初始化
 * @note   UART5（COM_GD_IMG）；帧头 0xEB，变长帧，累加和校验；
 *         解析由 COM_REC_DataAnalysis_1head_accu 完成，按 byte[1] 取帧长
 */
static void USER_Ctrl_ImgCmdRecvInit(void)
{
    COM_Rcv_SerialPort_Init(COM_GD_IMG, GD_IMG_FRAME_HEAD, 0, IMG_TO_GD_FRAME_LEN);
}

/**
 * @brief  方位链路串口接收初始化
 * @note   USART2（COM_GD_FW）；帧头 0xEB，变长帧，累加和校验
 */
static void USER_Ctrl_FwCmdRecvInit(void)
{
    COM_Rcv_SerialPort_Init(COM_GD_FW, GD_FW_FRAME_HEAD, 0, FW_TO_GD_FRAME_LEN);
}

/**
 * @brief  陀螺串口接收初始化
 * @note   串口：USART6（COM_GD_GYRO）
 *         帧头：0xA2（GYRO_FRAME_HEAD）
 *         帧长：13 字节定长（GYRO_FRAME_LEN）
 *         包尾：0（无固定包尾，最后一字节为异或校验）
 *         解析：COM_REC_DataAnalysis_1head_xor()，校验 byte[1]~byte[11]
 * @warning COM_GD_IN 也映射 USART6，勿与其他模块重复初始化同一串口
 */
static void USER_Ctrl_GyroCmdRecvInit(void)
{
    COM_Rcv_SerialPort_Init(COM_GD_GYRO, GYRO_FRAME_HEAD, 0, GYRO_FRAME_LEN);
}

/**
 * @brief  驱动板串口接收初始化
 * @note   COM_GD_MDRV（RS422 460800）；帧头 0x55，定长 8 字节；
 *         接收解析由 COM_REC_DataAnalysis_DriverBoard 完成（定长 8 字节 0x55 帧）；
 *         累加和由 USER_Ctrl_MdrvToGdVerifyChecksum 在应用层校验
 */
static void USER_Ctrl_MdrvCmdRecvInit(void)
{
    COM_Rcv_SerialPort_Init(COM_GD_MDRV, MDRV_FRAME_HEAD, 0, MDRV_NOCHECK_FRAME_LEN);
}

/**
 * @brief  图像指令接收任务
 * @note   从 COM_GD_IMG 环形缓冲取帧，校验通过后写入 ImgToGdCmd；
 *         若 recv_addr 为 REC_ADDR_SF，则查表分发伺服控制指令；
 *         图像板下行周期约 1Hz，建议在主循环中周期调用
 */
static void USER_Ctrl_ImgCmdRecvTask(void)
{
	if(image_connect_cnt > 100)//超时报故
	{
		image_connect_cnt = 0;
		image_connect_error = 1;
	}
    if (COM_REC_DataAnalysis_1head_accu(COM_GD_IMG, ImgToGdCmd.buf) != 0) {
        USER_Ctrl_ImgToGdLeUnpack(&ImgToGdCmd);
        s_img_cmd_recv_fresh = 1;

        if (ImgToGdCmd.field.recv_addr == REC_ADDR_SF) {
			image_connect_cnt = 0;
			image_connect_error = 0;
            USER_Ctrl_FwSetFaultCode(ImgToGdCmd.field.fault_code);
            USER_Ctrl_ImgSfCmdDispatch(&ImgToGdCmd);
			commandprocess();
        }
    }
}

/**
 * @brief  方位指令接收任务
 * @note   从 COM_GD_FW 环形缓冲取帧，校验通过后写入 FwToGdCmd；
 *         方位板上送周期约 1kHz，建议在主循环中周期调用
 */
static void USER_Ctrl_FwCmdRecvTask(void)
{
    uint8_t drained = 0;

    while (drained < FW_RECV_DRAIN_MAX_PER_LOOP
           && COM_REC_DataAnalysis_1head_accu(COM_GD_FW, FwToGdCmd.buf) != 0) {
        USER_Ctrl_FwToGdLeUnpack(&FwToGdCmd);
        s_fw_cmd_recv_fresh = 1;
        drained++;
    }
}

/**
 * @brief  陀螺串口接收任务
 * @note   调用链：
 *           COM_REC_DataAnalysis_1head_xor(COM_GD_GYRO, GyroRxFrame.buf)
 *             → 从环形缓冲取定长帧，帧头/异或校验由驱动完成
 *           USER_Ctrl_GyroBeUnpack(&GyroRxFrame, &GyroRxData)
 *             → 大端 24bit 角速度解析为 °/s
 *         每圈最多处理 GYRO_RECV_DRAIN_MAX_PER_LOOP 帧，保留最新数据；
 *         建议在 APP_Ctrl_System_Handle() 主循环中周期调用
 */
static void USER_Ctrl_GyroCmdRecvTask(void)
{
    uint8_t drained = 0;

    while (drained < GYRO_RECV_DRAIN_MAX_PER_LOOP
           && COM_REC_DataAnalysis_1head_xor(COM_GD_GYRO, GyroRxFrame.buf) != 0) {
        USER_Ctrl_GyroBeUnpack(&GyroRxFrame, &GyroRxData);
        s_gyro_recv_fresh = 1;
        drained++;
    }

    if (drained != 0U) {
        s_gyro_rx_stale_ms = 0U;
    }
}

/**
 * @brief  驱动板应答接收任务
 * @note   COM_REC_DataAnalysis_DriverBoard(COM_GD_MDRV) 取帧后写入 MdrvToGdCmd，
 *         再校验 byte[1]~byte[6] 累加和；1ms 周期建议在主循环调用
 */
static void USER_Ctrl_MdrvCmdRecvTask(void)
{
    static uint8_t s_mdrv_rx_staging[MDRV_RX_STAGING_LEN];
    uint8_t drained = 0;

    while (drained < MDRV_RECV_DRAIN_MAX_PER_LOOP) {
        uint16_t rx_len = COM_REC_DataAnalysis_DriverBoard(COM_GD_MDRV, s_mdrv_rx_staging);

        if (rx_len == 0U) {
            break;
        }

        if (rx_len == 1) {
            memcpy(MdrvToGdCmd.buf, s_mdrv_rx_staging, MDRV_TO_GD_FRAME_LEN);
            if (USER_Ctrl_MdrvToGdVerifyChecksum(&MdrvToGdCmd) == 0U) {
                drained++;
                continue;
            }
        }

        s_mdrv_cmd_recv_fresh = 1;
        drained++;
    }
}

/* -------------------------------------------------------------------------- */
/*                              系统任务                                         */
/* -------------------------------------------------------------------------- */

/** @brief  心跳灯处理（PB12），上电 5s 内 100ms 闪，之后 500ms 闪 */
void USER_Ctrl_LedFun(void)
{
    static uint32_t led_tick;

    if (HAL_GetTick() >= 5000) {
        if (HAL_GetTick() >= led_tick + 500) {
            led_tick = HAL_GetTick();
            led_run;
        }
    } else {
        if (HAL_GetTick() >= led_tick + 100) {
            led_tick = HAL_GetTick();
            led_run;
        }
    }
}

/**
 * @brief  模块功能测试占位函数（Flash / 红外校准等）
 * @note   当前为空实现，保留供调试时扩展
 */
void _test_model_flash_demo(void)
{
    //	uint8_t send_type;
    //	uint8_t send_data;
    ////	if (User_Tick.EthSend >= 1000)
    ////	{
    ////		send_type = IR_CALIBRATION;
    ////		send_data = 1;
    //////		Infrared_API_Ctrl_SendHandle(IR_TWIN612RG2,send_type,&send_data);
    //////		MASTER_Version_Rsp_Send();
    ////		User_Tick.EthSend = 0;
    ////	}
}
/** @brief Flash 读写调试缓冲（手动测试用，正常流程不使用） */
uint8_t debug_w_buff[100]={1,1,1,1,1,1,1,1,1};
uint8_t debug_r_buff[100];
/**
 * @brief  系统初始化
 * @note   依次完成：串口 DMA、定时器、图像/方位/陀螺/驱动板接收、PWM、伺服初始化；
 *         USER_CTRL_OPT_CALIB_STORE 可切换 Flash 或 EEPROM 持久化
 */
void APP_Ctrl_System_Init(void)
{
    srand((unsigned int)HAL_GetTick() ^ 0x5A5A5A5AU);

    COM_DRV_SerialPort_Init();   /* 串口 DMA 初始化 */
    TIMER_Interupt_Init();       /* 定时器中断初始化 */

    USER_Ctrl_ImgCmdRecvInit();  /* 图像板：USART5帧头 0xEB，累加和，小端 */
    USER_Ctrl_FwCmdRecvInit();   /* 方位板：USART2,帧头 0xEB，累加和，小端 */
    USER_Ctrl_GyroCmdRecvInit(); /* 陀螺  ：USART6，帧头 0xA2，异或，大端 24bit */
    USER_Ctrl_MdrvCmdRecvInit(); /* 驱动板：USART4,COM_GD_MDRV，帧头 0x55，nocheck 定长 8 字节 */
    USER_Ctrl_ImgCmdSendInit();  /* 俯仰→图像发送缓冲初始化 */
#if (USER_CTRL_OPT_CALIB_STORE == USER_CTRL_OPT_CALIB_STORE_EEPROM)
    USER_Ctrl_OptCalibEepromInit(); /* 绑定 I2C2 句柄 */
#endif
    USER_Ctrl_OptCalibStoreLoad(); /* 恢复经纬高/补偿 */
    USER_Ctrl_FwCmdSendInit();   /* 俯仰→方位发送缓冲初始化 */
    USER_Ctrl_MdrvCmdSendInit(); /* 俯仰→驱动板发送缓冲初始化 */
    USER_Ctrl_BmqSpiReadInit();  /* 编码器 SPI 接收缓冲初始化 */

    // TIMER_Pwm_Init(3, 1);        /* PWM 使能 */

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); /* PWM 使能 */  
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); /* PWM 使能 */
//    servo_module_init();                      /* 伺服模块初始化 */
    initdatapar();
	initcontrolpar();
	
//	Flash_Erase_Sector(FLASH_F4ADDR_SECTOR_7);
//	Flash_WriteNoErase(FLASH_F4ADDR_SECTOR_7 , debug_w_buff , 100);
//	Flash_Read(FLASH_F4ADDR_SECTOR_7 , debug_r_buff , 100);
}
uint8_t debug_lxy = 0;
/**
 * @brief  主循环周期处理
 * @note   由 main 循环调用；包含心跳灯、图像/方位/陀螺/驱动板接收及 1ms 时基任务
 */
void APP_Ctrl_System_Handle(void)
{
    USER_Ctrl_LedFun();
    USER_Ctrl_ImgCmdRecvTask();  /* 图像→俯仰，约 1Hz 下行，小端 + 累加和 */
    USER_Ctrl_FwCmdRecvTask();   /* 方位→俯仰，约 1kHz，小端 + 累加和 */
    USER_Ctrl_GyroCmdRecvTask(); /* 陀螺→俯仰，USART6，大端 + 异或校验 */
    USER_Ctrl_MdrvCmdRecvTask(); /* 驱动板→俯仰，nocheck + 累加和校验 */

    uint32_t pending_1ms = TAKE_TIM_FLAG_1ms();

    while (pending_1ms > 0U) {

//        servo_module_handler();
		alldeal();
        USER_Ctrl_GyroFaultTick1ms();          /* 陀螺有效帧超时判故障 */
        USER_Ctrl_BmqSpiReadTask();              /* SPI1 DMA 读编码器 → SPI1_rxbuf */
        USER_Ctrl_MdrvCmdSend(NULL);             /* 驱动板：1kHz 发送电流/编码器/使能 */
        USER_Ctrl_FwCmdSend(NULL);               /* 方位板：1kHz 发送电流/使能/偏差角等 */
        USER_Ctrl_ImgCmdFillRandom(&GdToImgCmd); /* 发送前填充 payload（角/速来自伺服） */
        USER_Ctrl_ImgCmdSend(NULL);              /* 图像板：1kHz 周期上行 */
        pending_1ms--;
#if DEBUG_MODE
		if(debug_lxy)
		{
			debug_lxy = 0;
			ImgToGdCmd.field.param1 = 0xe8;
			ImgToGdCmd.field.param2 = 0x03;
			ImgToGdCmd.field.param3 = 0x00;
			ImgToGdCmd.field.param4 = 0x00;
			ImgToGdCmd.field.param5 = 0xe8;
			ImgToGdCmd.field.param6 = 0x03;
			ImgToGdCmd.field.param7 = 0x00;
			ImgToGdCmd.field.param8 = 0x00;
			ImgToGdCmd.field.param9 =  0xe8;
			ImgToGdCmd.field.param10 = 0x03;
			ImgToGdCmd.field.param11 = 0x00;
			ImgToGdCmd.field.param12 = 0x00;
			ImgToGdCmd.field.param13 = 0xe8;
			ImgToGdCmd.field.param14 = 0x03;
			ImgToGdCmd.field.param15 = 0x00;
			ImgToGdCmd.field.param16 = 0x00;
			ImgToGdCmd.field.param17 = 0xe8;
			ImgToGdCmd.field.param18 = 0x03;
			ImgToGdCmd.field.param19 = 0x00;
			ImgToGdCmd.field.param20 = 0x00;
			USER_Ctrl_ImgSfCmdOptCalib(&ImgToGdCmd);
		}
#endif
		}
		
}

/* -------------------------------------------------------------------------- */
/*                              编码器 SPI                                       */
/* -------------------------------------------------------------------------- */

/**
 * @brief  编码器 SPI 接收初始化
 * @note   清零 DMA 缓冲与忙等待计数
 */
static void USER_Ctrl_BmqSpiReadInit(void)
{
    memset((void *)SPI1_rxbuf, 0, SPI1_BMQ_RX_LEN);
    s_spi1_bmq_busy_ms = 0;
}

/**
 * @brief  SPI1 总线/DMA 异常恢复
 * @note   当 DMA 长时间未释放时强制 Abort 并复位 hspi1 状态
 */
static void USER_Ctrl_BmqSpiRecover(void)
{
    uint32_t tickstart;

    (void)HAL_SPI_Abort(&hspi1);

    if (hspi1.hdmarx != NULL) {
        tickstart = HAL_GetTick();
        while (READ_BIT(hspi1.hdmarx->Instance->CR, DMA_SxCR_EN) != 0U) {
            if ((HAL_GetTick() - tickstart) > SPI1_BMQ_ABORT_TIMEOUT_MS) {
                break;
            }
        }

        if (hspi1.hdmarx->Lock == HAL_LOCKED) {
            __HAL_UNLOCK(hspi1.hdmarx);
        }

        if (hspi1.hdmarx->State != HAL_DMA_STATE_READY) {
            hspi1.hdmarx->State = HAL_DMA_STATE_READY;
        }
    }

    if (hspi1.hdmatx != NULL) {
        if (hspi1.hdmatx->Lock == HAL_LOCKED) {
            __HAL_UNLOCK(hspi1.hdmatx);
        }

        if (hspi1.hdmatx->State != HAL_DMA_STATE_READY) {
            hspi1.hdmatx->State = HAL_DMA_STATE_READY;
        }
    }

    CLEAR_BIT(hspi1.Instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);

    if (hspi1.Lock == HAL_LOCKED) {
        __HAL_UNLOCK(&hspi1);
    }

    hspi1.State = HAL_SPI_STATE_READY;
    s_spi1_bmq_busy_ms = 0;
}

/**
 * @brief  1ms 任务：SPI1 DMA 读取俯仰编码器
 * @note   就绪时启动 HAL_SPI_Receive_DMA，数据写入 SPI1_rxbuf；
 *         完成后 HAL_SPI_RxCpltCallback → BmqData_Dispose 解析角度
 */
void USER_Ctrl_BmqSpiReadTask(void)
{
    if (hspi1.State == HAL_SPI_STATE_READY && hspi1.Lock == HAL_UNLOCKED) {
        if (HAL_SPI_Receive_DMA(&hspi1, (uint8_t *)SPI1_rxbuf, SPI1_BMQ_RX_LEN) == HAL_OK) {
            s_spi1_bmq_busy_ms = 0;
			GdToImgCmd.field.fault_sta.gd_encoder_fault = 0U;
        }
        return;
    }

    s_spi1_bmq_busy_ms++;
    if (s_spi1_bmq_busy_ms >= SPI1_BMQ_BUSY_TIMEOUT_MS) {
        USER_Ctrl_BmqSpiRecover();
		GdToImgCmd.field.fault_sta.gd_encoder_fault = 1U;
    }
}

/**
 * @brief  编码器 SPI 数据解析
 * @note   从 SPI1_rxbuf 前 4 字节提取 25 位绝对码值（32bit 组装后 >>6 去状态位），换算为 0~360°；
 *         前 4 字节全 0xFF 表示通信异常（待处理）
 */
void BmqData_Dispose(void)
{
    FY_BMP_Val =
        (uint32_t)(((SPI1_rxbuf[0] << 24) | (SPI1_rxbuf[1] << 16) | (SPI1_rxbuf[2] << 8) | SPI1_rxbuf[3])
                   >> 6)
        & SPI_BMQ_RAW_MASK;

    // FY_BMP_Val = (raw32 >> SPI_BMQ_STATUS_SHIFT) & SPI_BMQ_RAW_MASK;
    WHGBMQ_Data1 = FY_BMP_Val;
    WHGBMQ_Data2 = WHGBMQ_Data1;
    angle_real = (float)FY_BMP_Val * SPI_BMQ_ANGLE_PER_CODE;

    if (SPI1_rxbuf[0] == 0xFF && SPI1_rxbuf[1] == 0xFF && SPI1_rxbuf[2] == 0xFF && SPI1_rxbuf[3] == 0xFF) {
        /* 编码器无效帧：置位俯仰→图像故障状态 bit3 */
        GdToImgCmd.field.fault_sta.gd_encoder_fault = 1U;
    } else {
        /* 编码器有效帧：清除俯仰编码器故障位 */
        GdToImgCmd.field.fault_sta.gd_encoder_fault = 0U;
    }
}


/**
 * @brief  SPI 接收完成回调（HAL 弱函数重载）
 * @param  hspi  触发回调的 SPI 句柄
 * @note   仅处理 SPI1（俯仰编码器）；收到一帧后调用 BmqData_Dispose 解析
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi1) {
        BmqData_Dispose();
    }
}
