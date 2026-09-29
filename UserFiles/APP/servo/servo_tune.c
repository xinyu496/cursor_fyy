/**
 * @file    servo_tune.c
 * @brief   伺服在线整定（调参）实现
 *
 *          整定表 g_tune[] 由 servo_module.c 持有；本文件只负责
 *          sync / write / apply / on_frame 四条路径，保证 PI 修改入口唯一。
 *
 * @author  LinHui
 * @version 1.00
 * @date    2026-06-01
 */

#include "servo_tune.h"
#include <string.h>

/* ========================================================================
 *                          整定表 ↔ 轴参数同步
 * ====================================================================== */

/**
 * @brief 用轴当前参数回填整定表，避免 enable 后用 0 覆盖现值
 *
 *        遍历 SERVO_TUNE_PARAM_MAX 个槽位，通过 get_param 读出轴内现值；
 *        回填完成后 enable=0，表示“表已就绪但尚未请求下发”。
 */
void servo_tune_sync(servo_tune_t *t, dev_servo_axis_t *ax)
{
    /* 空指针保护：任一为空则直接返回，不写表 */
    if (!t || !ax) {
        return;
    }

    /* 逐参数 ID 回填：idx 传 0 读默认档位，FW 多档参数在 write/apply 时再带 idx */
    for (uint8_t id = 0; id < SERVO_TUNE_PARAM_MAX; id++) {
        t->val[id] = dev_servo_axis_get_param(ax, (servo_tune_param_e)id, 0);
    }

    t->idx = 0u;    /* 默认增益档位 */
    t->enable = 0u; /* 同步后不自动下发，需显式 write 或 on_frame 才 enable */
}

/* ========================================================================
 *                          写表 / 下发
 * ====================================================================== */

/**
 * @brief 写一条到整定表（仿真 Watch / 上位机共用）
 *
 *        只改 val[id] 与 idx，并置 enable=1；
 *        实际写入轴 param 由 servo_tune_apply() 在下一拍完成。
 */
void servo_tune_write(servo_tune_t *t, servo_tune_param_e id, uint8_t idx, float val)
{
    if (!t || id >= SERVO_TUNE_PARAM_MAX) {
        return;
    }

    t->val[id] = val; /* 目标值写入对应槽位 */
    t->idx = idx;     /* 记录本次请求的增益档位 */
    t->enable = 1u;   /* 标记待下发：apply 会消费此标志对应的整表 */
}

/**
 * @brief 整表下发到轴（enable 时生效）—— PI 在线修改唯一落地点
 *
 *        当 t->enable==1 时，把 val[] 中全部参数按 t->idx 档位写入 ax->param；
 *        本函数不自动清 enable，便于连续多拍保持同一套表（若需单次下发可在上层清 0）。
 */
void servo_tune_apply(dev_servo_axis_t *ax, const servo_tune_t *t)
{
    /* 未 enable 或指针无效：不干预轴内现值 */
    if (!ax || !t || !t->enable) {
        return;
    }

    /* 批量 set_param：与 sync 时的 get 一一对应 */
    for (uint8_t id = 0; id < SERVO_TUNE_PARAM_MAX; id++) {
        dev_servo_axis_set_param(ax, (servo_tune_param_e)id, t->idx, t->val[id]);
    }
}

/* ========================================================================
 *                          上位机帧解析（预留）
 * ====================================================================== */

/**
 * @brief 上位机调参帧解析：校验 → 写表
 *
 *        校验项：长度、帧头 0xB5、axis/param_id 范围、累加和；
 *        通过后调用 servo_tune_write 写入 tbl[axis]。
 */
bool servo_tune_on_frame(servo_tune_t tbl[SERVO_AXIS_MAX], const uint8_t *buf, uint16_t len)
{
    if (!tbl || !buf || len < sizeof(servo_tune_frame_t)) {
        return false;
    }

    /* 拷贝到本地结构体，避免未对齐访问 */
    servo_tune_frame_t f;
    memcpy(&f, buf, sizeof(f));

    /* 帧头与轴/参数 ID 合法性检查 */
    if (f.head != 0xB5u || f.axis >= SERVO_AXIS_MAX || f.param_id >= SERVO_TUNE_PARAM_MAX) {
        return false;
    }

    /* 累加和：head 至 val 最后一字节，不含 sum 自身 */
    uint8_t sum = 0;
    for (uint16_t i = 0; i < sizeof(f) - 1; i++) {
        sum += buf[i];
    }
    if (sum != f.sum) {
        return false;
    }

    /* 写入对应轴整定表，下一拍 apply 生效 */
    servo_tune_write(&tbl[f.axis], (servo_tune_param_e)f.param_id, f.idx, f.val);
    return true;
}
