/**
 * @file    servo_tune.h
 * @brief   伺服在线整定（调参）接口
 *
 *          提供“整定表 → 轴对象”的统一下发路径，供仿真 Watch 窗口与上位机调参帧共用。
 *          典型流程：
 *            ① 上电 servo_tune_sync() 用轴现值回填表，避免 enable 后误写 0
 *            ② 仿真/上位机改表：servo_tune_write() 或 servo_tune_on_frame()
 *            ③ 1ms 主循环 servo_tune_apply() 把 enable=1 的表批量写入轴 PI 参数
 *
 * @author  LinHui
 * @version 1.00
 * @date    2026-06-01
 */

#ifndef _SERVO_TUNE_H_
#define _SERVO_TUNE_H_

#include "servo_controller.h" /* dev_servo_t / servo_axis_id_e / SERVO_AXIS_MAX */
#include "servo_axis.h"       /* servo_tune_param_e / set_param / get_param */

/* ========================================================================
 *                         整定表与通信帧定义
 * ====================================================================== */

/**
 * @brief 单轴在线整定表
 *
 *        每轴一张表（g_tune[SERVO_AXIS_FW/GD]），仿真与上位机都只写这张表；
 *        主循环统一 servo_tune_apply() 下发，避免多处直接改 axis->param。
 */
typedef struct {
    uint8_t enable;                  /**< 0=不干预(保留轴内现值)  1=本周期按 val[] 批量下发 */
    uint8_t idx;                     /**< 增益调度档位：FW 轴 0~5，GD 轴恒 0 */
    uint8_t reserved[2];             /**< 保留，保证 val[] 4 字节对齐，sizeof=60 */
    float val[SERVO_TUNE_PARAM_MAX]; /**< 以 servo_tune_param_e 为下标的目标 PI/限幅值 */
} servo_tune_t;

/**
 * @brief 上位机调参帧（预留协议）
 *
 *        一帧只改一个参数；解析通过后写入对应轴整定表，下一拍 apply 生效。
 *        帧格式：head + axis + param_id + idx + val + sum（累加和校验）
 */
#pragma pack(1)
typedef struct {
    uint8_t head;     /**< 帧头固定 0xB5 */
    uint8_t axis;     /**< 轴 ID：servo_axis_id_e（0=FW，1=GD） */
    uint8_t param_id; /**< 参数 ID：servo_tune_param_e */
    uint8_t idx;      /**< 增益档位（Kp_A/Ki_A/Kp_V/Ki_V 有效） */
    float val;        /**< 新参数值 */
    uint8_t sum;      /**< head..val 字节累加校验 */
} servo_tune_frame_t;
#pragma pack()        /* 恢复默认对齐，避免影响后续结构体 */

/* ========================================================================
 *                              公开 API
 * ====================================================================== */

/**
 * @brief 用轴当前 PI 参数回填整定表（开机调用一次）
 * @param t  目标整定表
 * @param ax 已 dev_servo_axis_init() 的轴对象
 * @note  回填后 enable=0，不会立即覆盖轴内参数
 */
void servo_tune_sync(servo_tune_t *t, dev_servo_axis_t *ax);

/**
 * @brief 向整定表写入一条参数（仿真/上位机共用入口）
 * @param t    目标整定表
 * @param id   参数 ID
 * @param idx  增益档位
 * @param val  新值
 * @note  写入后自动 enable=1，下一拍 apply 生效
 */
void servo_tune_write(servo_tune_t *t, servo_tune_param_e id, uint8_t idx, float val);

/**
 * @brief 把整定表批量下发到轴（主循环每 1ms 调用）
 * @param ax 目标轴
 * @param t  整定表；仅 t->enable==1 时生效
 * @note  本函数是 PI 在线修改的唯一落地点，避免多处直接写 param
 */
void servo_tune_apply(dev_servo_axis_t *ax, const servo_tune_t *t);

/**
 * @brief 解析上位机调参帧并写入整定表（预留）
 * @param tbl  双轴整定表数组 g_tune[SERVO_AXIS_MAX]
 * @param buf  原始帧缓冲
 * @param len  缓冲长度
 * @return true=校验通过并已 write；false=帧非法或长度不足
 */
bool servo_tune_on_frame(servo_tune_t tbl[SERVO_AXIS_MAX], const uint8_t *buf, uint16_t len);

#endif /* _SERVO_TUNE_H_ */
