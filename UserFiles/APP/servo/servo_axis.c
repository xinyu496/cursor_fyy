/**
 * @file    servo_axis.c
 * @brief   单轴对象实现
 *
 *          本文件只做控制算法（PI/陷波/滤波）的编排管理
 *
 *          【控制链概览 — 1ms 周期】
 *          update_sensor()  → 编码器/陀螺/电流采样与滤波
 *          sync_feedback()  → 观测值写入 rt.*_fb
 *          run_loop(level)  → 由外到内级联 PI，输出 I_give 或 U_give
 *          enable()         → 保护判定 + PWM/电流下发
 *
 *          【GD 俯仰轴】有限角，级联：GEO/TV → V → A → I（有陀螺）
 *                                    或 GEO/TV → Ev → I（无陀螺）
 *          【FW 方位轴】连续旋转，V/A 环 6 档增益随 |GD角| 调度，含 sec 解耦
 *
 * @author  LinHui
 * @version 1.00
 * @date    2026-05-29
 */

#include "servo_axis.h"
#include "SFlibhead.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

/* ========================================================================
 *                    数据结构 / 运行时量字段说明（只读注释）
 * ====================================================================== */
/*
 * dev_servo_axis_t 核心成员：
 *   id          — SERVO_AXIS_GD(俯仰) / SERVO_AXIS_FW(方位)
 *   io          — 硬件钩子：编码器/陀螺/电流/PWM/硬使能
 *   config      — 硬件配置：码盘、限位、驱动类型、保护阈值
 *   param       — PI 系数：Kp/Ki/Imax/Amax/bound 等
 *   sensor      — 传感器观测：angle/ev/gyro/acc/current
 *   rt          — 控制运行时：*_give 给定、*_fb 反馈、U_give/I_give 输出
 *   status      — 故障/使能/到位/软限位标志
 *
 * rt 给定/反馈链（由外到内）：
 *   GEO_give → V_give → A_give → I_give → U_give(PWM)
 *   P_give   → Ev_give ────────────────┘
 *   T_fb     → V_give（跟踪）
 *
 * sensor 主要量纲：
 *   angle, P_fb     — °（机械角，相对 bmq_zero）
 *   ev, Ev_fb, gyro — °/s（角速度）
 *   acc, A_fb       — °/s² 量级（陀螺微分加速度）
 *   current, I_fb   — A（电机电流）
 *
 * status.fault 位（示意）：
 *   overcurrent — 1s 窗口内过流次数 ≥800
 *   overspeed   — 编码器 ev 连续 ≥100°/s 达 1000ms
 *   limit_hit   — 软限位触发（仅 soft_limit_enable 且非 continuous）
 *
 * config.drive_type：
 *   SERVO_DRIVE_CURRENT_CMD — 无刷，I_give→驱动器，驱动器内闭电流环
 *   SERVO_DRIVE_BRUSHED_PWM — 有刷，run_loop 后 gdiloop/fwiloop→U_give→PWM
 *
 * param 整定参数含义（servo_axis_ctrl_param_t）：
 *   SamT / SamT_I     — 主环 / 电流环离散周期（s），通常 0.001
 *   Kp_I / Ki_I       — 有刷内层电流环 PI
 *   Kp_A[] / Ki_A[]   — 陀螺加速度环；FW 6 元数组按 |GD角| 档
 *   Kg_A / BW_A[]     — 加速度环模型增益与补偿带宽（Hz）
 *   Kp_V[] / Ki_V[]   — 陀螺速度环；FW 6 档
 *   bound_A[] / bound_V[] — FW 增益切换边界（°，相对 |GD俯仰|）
 *   Kp_Ev / Ki_Ev     — 编码器惯性速度环（gdevloop/fwevloop）
 *   Kp_P / Ki_P       — 轴位置环；Ki_P 常为 0
 *   bound_P           — |位置误差| 大小区间阈值（°）
 *   Ev_Set / Evmax    — 大误差固定逼近速度 / 位置环输出限速（°/s）
 *   Kp_1_T1/Ki_1_T1   — TV1 小误差段 PI
 *   Kp_2_T1/Ki_2_T1   — TV1 大误差段 PI
 *   bound_T1          — TV1 分段阈值（°）
 *   Kp_G / Ki_G       — GEO 稳定环（若启用）
 *   Imax              — 电流环输出限幅（A）
 *   Amax              — 陀螺速度环输出限幅（加速度量级）
 *   Vmax              — 跟踪/GEO 输出角速度限幅（°/s）
 *   Umax              — 有刷 PWM 电压限幅
 */

/* ========================================================================
 *                          内部辅助宏 / 常量
 * ====================================================================== */
/* 下列常量与原 SF_20260303 / gd_en/fw_en 保持一致，修改时需同步保护逻辑与 PI 整定。 */

#define SF_DEG2RAD          0.017453293f /**< 度→弧度，π/180，用于 cos(sec) 解耦 */
#define SF_OVERSPEED_THRESH 100.0f       /**< 超速阈值（°/s），编码器 ev 连续超限判据 */
#define SF_OVERCUR_THRESH   5.0f         /**< 过流阈值（A），|I_fb| 单次采样判据 */
#define SF_FAULT_WINDOW_MS  1000         /**< 过流统计窗口长度（ms），满窗后清零计数 */
#define SF_FAULT_DANGER_TH  800          /**< 窗口内允许的超限采样次数，≥则过流故障 */
#define SF_OVERSPEED_TH     1000         /**< 连续超速采样次数上限，≥则超速故障（≈1s@1ms） */

#define SF_INPLACE_ERR_DEG   0.1f        /**< 到位判定：|P_give-P_fb| 误差阈值（°） */
#define SF_INPLACE_DWELL_TH  500         /**< 到位驻留拍数，>500 且误差合格才 in_place=true */
#define SF_CUR_CALIB_SAMPLES 3000        /**< 电流零偏标定样本数，与 CurrentInit 一致 */

/* 编码器分辨率换算：每增加 1 个 LSB 对应的角度（°/码） */
#define SF_BMQ_COF_FW 0.0000214576721f /**< FW 24bit：360/16777216 */
#define SF_BMQ_COF_GD 0.0000214576721f /**< GD 24bit：与 FW 同分辨率时可共用 */
/* 其它分辨率参考：21bit:0.0001716613769f; 20bit:0.0003433227539f; 24bit:0.0000214576721f */

/* ========================================================================
 *                          前置静态方法声明
 * ====================================================================== */
/* 下列静态函数构成单轴对象的“方法表”实现，由 dev_servo_axis_init 挂到函数指针上。
 * 调用链（1ms 周期）：update_sensor → sync_feedback → run_loop → enable/brake。
 * 单位约定：角度 °、角速度 °/s、电流 A、时间 s、PWM/电流给定依驱动类型而定。 */

/** @brief 1ms 传感器刷新：编码器 + 陀螺/电流 + 软限位标志 */
static void axis_update_sensor(dev_servo_axis_t *pobj);
/** @brief 观测同步到 rt.*_fb，供 run_loop 反馈（fbdispose 等价） */
static void axis_sync_feedback(dev_servo_axis_t *pobj);
/** @brief 级联 PI 编排，level 指定起始环，返回 I_give */
static float axis_run_loop(dev_servo_axis_t *pobj, servo_loop_level_e level, uint8_t dz_flag);
/** @brief 从 from_level 起清除对应 PI 积分状态 */
static void axis_clear_loops(dev_servo_axis_t *pobj, servo_loop_level_e from_level);
/** @brief 清除本轴全部 PI 状态（clr_xx_all） */
static void axis_clear_all(dev_servo_axis_t *pobj);
/** @brief 保护检查后使能输出（PWM 或电流指令） */
static void axis_enable(dev_servo_axis_t *pobj);
/** @brief 主动刹车：关使能并清零输出 */
static void axis_brake(dev_servo_axis_t *pobj);
/** @brief 设置位置目标 P_give（°），有限角轴裁剪 */
static void axis_set_pos_tgt(dev_servo_axis_t *pobj, float deg);
/** @brief 设置角速度目标 V_give（°/s） */
static void axis_set_vel_tgt(dev_servo_axis_t *pobj, float dps);
/** @brief 设置显示零偏 offset_deg（未量产使用） */
static bool axis_set_zero(dev_servo_axis_t *pobj, float angle_deg);
/** @brief 读机械角 sensor.angle（°） */
static float axis_get_angle(dev_servo_axis_t *pobj);
/** @brief 读陀螺滤波速度 sensor.gyro（°/s） */
static float axis_get_vel(dev_servo_axis_t *pobj);
/** @brief 读故障字 status.fault.all */
static uint16_t axis_get_fault(dev_servo_axis_t *pobj);
/** @brief 清除故障并解锁 brake_locked */
static void axis_clear_fault(dev_servo_axis_t *pobj);
/** @brief 设置 FW sec 耦合系数 sec_couple */
static void axis_set_couple(dev_servo_axis_t *pobj, float sec);
/** @brief 设置编码器正方向 dir */
static void axis_set_dir(dev_servo_axis_t *pobj, servo_axis_dir_e dir);
/** @brief 读取编码器正方向 dir */
static servo_axis_dir_e axis_get_dir(dev_servo_axis_t *pobj);
/** @brief 电流零偏标定（累加 N 样本求均值） */
static void axis_calibrate_current_bias(dev_servo_axis_t *pobj);
/** @brief 软限位标志 xw_up/xw_dn（不自动刹车） */
static void axis_check_soft_limit(dev_servo_axis_t *pobj);
/** @brief 到位判定 in_place（误差+驻留防抖） */
static void axis_update_in_place(dev_servo_axis_t *pobj);

/* 内部工具函数 — 传感器链、保护、默认配置 */
static void axis_load_default_config(dev_servo_axis_t *pobj);   /* 硬件/驱动默认 config */
static void axis_process_encoder(dev_servo_axis_t *pobj);       /* 编码器→角度/速度/ev */
static void axis_process_gyro(dev_servo_axis_t *pobj);          /* 陀螺+电流反馈链 */
static void axis_check_protect(dev_servo_axis_t *pobj);         /* 过流/超速故障判定 */
static void axis_close_current_loop(dev_servo_axis_t *pobj);    /* 有刷内层 iloop */

/* ========================================================================
 *                          对外初始化 API
 * ====================================================================== */

/**
 * @brief 初始化单轴对象：装配函数指针 + 加载默认配置 + 清状态
 */
void dev_servo_axis_init(dev_servo_axis_t *pobj, servo_axis_id_e id, const servo_axis_io_t *io)
{
    if (pobj == NULL || io == NULL) {
        return; /* 防御：空指针不初始化 */
    }

    /* 1) 全清，避免野值污染 PI 状态与传感器历史 */
    memset(pobj, 0, sizeof(dev_servo_axis_t));

    /* 2) 身份 / IO 钩子：id 决定 GD/FW 默认参数与级联分支 */
    pobj->id = id;
    pobj->io = *io;                      /* 拷贝钩子表（编码器/陀螺/电流/PWM） */
    pobj->dir = SERVO_AXIS_DIR_CW;       /* 默认顺时针为正；可按实测反向 */
    if (pobj->id == SERVO_AXIS_GD) {
        pobj->dir = SERVO_AXIS_DIR_CW;   /* GD 俯仰固定 CW，与旧代码一致 */
    }
    pobj->sec_couple = 1.0f;             /* FW 跟踪 sec 耦合初值，运行时可 set_couple */

    /* 3) 装配方法表（函数指针），对外只暴露 dev_servo_axis_t 接口 */
    pobj->update_sensor = axis_update_sensor;
    pobj->sync_feedback = axis_sync_feedback;
    pobj->run_loop = axis_run_loop;
    pobj->clear_loops = axis_clear_loops;
    pobj->clear_all = axis_clear_all;
    pobj->enable = axis_enable;
    pobj->brake = axis_brake;
    pobj->set_position_target = axis_set_pos_tgt;
    pobj->set_velocity_target = axis_set_vel_tgt;
    pobj->set_zero_angle = axis_set_zero;
    pobj->get_angle = axis_get_angle;
    pobj->get_velocity = axis_get_vel;
    pobj->get_fault = axis_get_fault;
    pobj->clear_fault = axis_clear_fault;
    pobj->set_couple = axis_set_couple;
    pobj->set_dir = axis_set_dir;
    pobj->get_dir = axis_get_dir;
    pobj->calibrate_current_bias = axis_calibrate_current_bias;
    pobj->check_soft_limit = axis_check_soft_limit;
    pobj->update_in_place = axis_update_in_place;

    /* 4) 默认 config（硬件）/ param（PI 系数） */
    axis_load_default_config(pobj);
    dev_servo_axis_load_default_param(pobj);

    /* 5) 陀螺有无：未提供 get_gyro_raw 钩子即判为"仅编码器"。
     *    如需强制指定，可在 init 之后写 pobj->config.has_gyro = true/false; 覆盖。 */
    pobj->config.has_gyro = (io->get_gyro_raw != NULL);
}

/* ========================================================================
 *              GD / FW 默认 PI 缺省值对照（便于整定与版本比对）
 * ====================================================================== */
/*
 * GD 缺省（俯仰，单档 V/A）：
 *   Kp_I=750 Ki_I=300000 Imax=4 Umax=4000
 *   Kp_V[0]=0 Ki_V[0]=0 Amax=500 Kg_A=321.69 BW_A[0]=10
 *   Kp_Ev=0.30 Ki_Ev=14.5
 *   Kp_P=12.5 Ki_P=0 bound_P=6 Ev_Set=35 Evmax=40
 *   TV1: Kp1=2.0 Ki1=3.5 bound=2 Kp2=2.5 Ki2=4.5 Vmax=35
 *   限位: up=82° dn=-12° bmq_zero=1024350 coef=24bit
 *
 * FW 缺省（方位，6档 V/A，|GD| 边界 75/70/60/40/30°）：
 *   Kp_I=700 Ki_I=200000 Imax=4.5 Umax=4000
 *   Kp_Ev=0.55 Ki_Ev=15
 *   Kp_P=15 Ki_P=0 bound_P=6 Ev_Set=35 Evmax=40
 *   TV1: Kp1=2.5 Ki1=1.0 bound=2 Kp2=3.0 Ki2=3.0 Vmax=35
 *   Amax=1500 Kg_A=254.37
 *   连续轴 bmq_zero=7297419 sec_couple 初值=1.0
 *
 * 有刷编译覆盖（GD/FW 各自 BRUSHED 开关）：
 *   仅重写 Kp_I/Ki_I/SamT_I/Umax，run_loop 末尾强制 iloop。
 *
 * 采样：SamT=SamT_I=0.001s（1kHz），与旧 SF_20260303 一致。
 */

/**
 * @brief 加载默认 PI 控制参数（来源：SF_20260303.c::initcontrolpar）
 *        FW / GD 系数不同，按 id 分流。
 */
void dev_servo_axis_load_default_param(dev_servo_axis_t *pobj)
{
    if (pobj == NULL)
        return;
    servo_axis_ctrl_param_t *p = &pobj->param;

    /* 采样周期：位置/速度/跟踪环 1ms；电流环 SamT_I 可与主环不同 */
    p->SamT_I = 0.001f;  /* 电流环离散周期（s） */
    p->SamT = 0.001f;     /* 主控制环离散周期（s） */

    if (pobj->id == SERVO_AXIS_GD) {
        /* ---------- 俯仰（GD）-----------
         * 级联：GEO/TV → 陀螺V → 陀螺A → I（有陀螺）
         *       或 GEO/TV → 编码器Ev → I（无陀螺，跳过A环）
         * 有限角轴：angle_up/dn_limit 参与各环抗饱和。 */

        /* 电流环参数（有刷 gdiloop 使用；无刷时 I 直接送驱动器） */
        p->Kp_I = 750.0f;       /* 电流环比例增益 */
        p->Ki_I = 300000.0f;    /* 电流环积分增益，SamT_I=1ms */
        p->Umax = 4000.0f;      /* 有刷 PWM 占空比上限（±） */
        p->Imax = 4.0f;         /* 电流环输出限幅（A），作为内环给定上限 */

        /* 陀螺加速度环参数（gdaloop，输入 A_give，输出 I_give） */
        p->Kp_A[0] = 0.0f;      /* GD 仅单档，无俯仰角增益调度 */
        p->Ki_A[0] = 0.0f;
        p->Kg_A = 321.69f;      /* 加速度传递函数增益，补偿陀螺-电机链 */
        p->BW_A[0] = 10.0f;     /* 加速度环补偿带宽（Hz） */

        /* 陀螺速度环参数（gdvloop，输入 V_give，输出 A_give） */
        p->Kp_V[0] = 0.0f;
        p->Ki_V[0] = 0.0f;
        p->Amax = 500.0f;       /* 陀螺速度环输出加速度上限（°/s² 量级） */

        /* 轴角速度环参数（gdevloop，编码器 ev 反馈，输出 I_give） */
        p->Kp_Ev = 0.30f;       /* 轴惯性速度环 Kp */
        p->Ki_Ev = 14.5f;       /* 轴惯性速度环 Ki */

        /* 轴位置环参数（gdploop，输入 P_give，输出 Ev_give °/s） */
        p->Kp_P = 12.5f;        /* GD 位置环 Kp */
        p->Ki_P = 0.0f;         /* GD 位置环 Ki */
        p->bound_P = 6.0f;      /* |位置误差|>bound 时用 Ev_Set 固定逼近，否则 PI 闭环 */
        p->Ev_Set = 35.0f;      /* 大误差区固定逼近角速度（°/s） */
        p->Evmax = 40.0f;       /* 位置环输出速度上限（°/s） */

        /* 脱靶量跟踪环 TV1（gdtloop_TV1，输入 T_fb，输出 V_give °/s） */
        p->Kp_1_T1 = 2.0f;      /* 小误差段 PI 比例 */
        p->Ki_1_T1 = 3.5f;      /* 小误差段 PI 积分 */
        p->bound_T1 = 2.0f;     /* 脱靶量分段阈值（°），|T|>bound 切大误差段增益 */
        p->Kp_2_T1 = 2.5f;      /* 大误差段 PI 比例 */
        p->Ki_2_T1 = 4.5f;      /* 大误差段 PI 积分 */
        p->Vmax = 35.0f;        /* 跟踪环输出角速度上限（°/s） */

        /* TV2（本机型未启用，系数置零，clr_gd_t_2/3 仍保留） */
        p->Kp_1_T2 = 0.0f;      /* TV2 小误差 Kp */
        p->Ki_1_T2 = 0.0f;      /* TV2 小误差 Ki */
        p->bound_T2 = 0.0f;     /* TV2 分段阈值 */
        p->Kp_2_T2 = 0.0f;      /* TV2 大误差 Kp */
        p->Ki_2_T2 = 0.0f;      /* TV2 大误差 Ki */

#if SERVO_GD_DRIVE_BRUSHED
        /* —— 编译开关 SERVO_GD_DRIVE_BRUSHED=1：GD 有刷 H 桥 ——
         * config.drive_type → BRUSHED_PWM；run_loop 后走 gdiloop；
         * 电流采样需 Butterworth + current_sign；Umax 为 PWM 满量程。 */
        p->Kp_I = 750.0f;       /* 有刷 GD 电流环 Kp */
        p->Ki_I = 300000.0f;    /* 有刷 GD 电流环 Ki */
        p->SamT_I = 0.001f;     /* iloop 离散周期 1ms */
        p->Umax = 4000.0f;      /* H 桥 PWM 占空比上限 ±4000 */
#endif

    } else {
        /* ---------- 方位（FW）-----------
         * 级联：GEO/TV → 陀螺V → 陀螺A → I（有陀螺，6档增益随 |GD角| 调度）
         *       或 GEO/TV → 编码器Ev → I（无陀螺）
         * 连续旋转轴：无机械上下限裁剪，但 fwvloop/fwaloop 仍用 limit 参数抗饱和。 */

        /* 电流环（有刷 fwiloop / 无刷送驱动器） */
        p->Kp_I = 700.0f;       /* FW 电流环 Kp（无刷时驱动器内环，此处备用） */
        p->Ki_I = 200000.0f;    /* FW 电流环 Ki */
        p->Umax = 4000.0f;      /* PWM/电压限幅 */
        p->Imax = 4.5f;         /* FW 电流给定上限略高于 GD */

        /* 陀螺加速度环：6 档 Kp_A/Ki_A/BW_A，按 |GD 俯仰角| 切换（bound_A[]） */
        const float kA_base = 0.00042f * 1.60f * 0.8f * 0.38f;
        const float scl_A[6] = {0.30f, 0.50f, 0.50f, 0.70f, 1.0f, 1.0f};
        for (int i = 0; i < 6; i++) {
            p->Kp_A[i] = kA_base * scl_A[i];              /* 第 i 档加速度环 Kp */
            p->Ki_A[i] = 1.60f * scl_A[i] * 0.8f * 0.38f; /* 第 i 档加速度环 Ki */
            p->BW_A[i] = 10.0f;                           /* 各档补偿带宽（Hz） */
        }
        p->Kg_A = 254.37f;       /* FW 加速度传递函数增益（含 sec 耦合前） */
        p->bound_A[0] = 75.0f;   /* |GD角| 分段边界（°），决定选用哪档 A 环增益 */
        p->bound_A[1] = 70.0f;
        p->bound_A[2] = 60.0f;
        p->bound_A[3] = 40.0f;
        p->bound_A[4] = 30.0f;

        /* 陀螺速度环：6 档 Kp_V/Ki_V，同样随 |GD角| 在 bound_V[] 间切换 */
        const float scl_V[6] = {1.0f, 1.0f, 1.2f, 1.4f, 1.6f, 1.6f};
        const float scl_KV[6] = {0.30f, 0.50f, 0.70f, 0.70f, 1.0f, 1.0f};
        for (int i = 0; i < 6; i++) {
            p->Kp_V[i] = 50.0f * 0.80f * scl_V[i];       /* 第 i 档 V 环 Kp */
            p->Ki_V[i] = 1100.0f * 0.80f * scl_KV[i];     /* 第 i 档 V 环 Ki */
        }
        p->Amax = 1500.0f;       /* FW 陀螺速度环输出加速度上限 */
        p->bound_V[0] = 75.0f;   /* |GD角| 分段边界（°），决定 V 环增益档 */
        p->bound_V[1] = 70.0f;
        p->bound_V[2] = 60.0f;
        p->bound_V[3] = 40.0f;
        p->bound_V[4] = 30.0f;

        /* 轴角速度环（fwevloop，编码器 ev 反馈） */
        p->Kp_Ev = 0.55f;       /* FW 编码器速度环 Kp */
        p->Ki_Ev = 15.0f;       /* FW 编码器速度环 Ki */

        /* 轴位置环（fwploop） */
        p->Kp_P = 15.0f;        /* FW 位置环 Kp */
        p->Ki_P = 0.0f;         /* FW 位置环 Ki（通常为 0） */
        p->bound_P = 6.0f;       /* 大误差固定逼近阈值（°） */
        p->Ev_Set = 35.0f;       /* 大误差固定逼近速度（°/s） */
        p->Evmax = 40.0f;        /* 位置环输出速度上限（°/s） */

        /* 脱靶量跟踪 TV1（fwtloop_TV1，含 sec_couple 解耦） */
        p->Kp_1_T1 = 2.5f;      /* TV1 小误差 Kp */
        p->Ki_1_T1 = 1.0f;      /* TV1 小误差 Ki */
        p->bound_T1 = 2.0f;      /* 脱靶量分段 PI 阈值（°） */
        p->Kp_2_T1 = 3.0f;      /* TV1 大误差 Kp */
        p->Ki_2_T1 = 3.0f;      /* TV1 大误差 Ki */
        p->Vmax = 35.0f;         /* 跟踪速度上限（°/s） */

        /* TV2（未启用，保留 clr_fw_t_2/3 接口） */
        p->Kp_1_T2 = 0.0f;
        p->Ki_1_T2 = 0.0f;
        p->bound_T2 = 0.0f;
        p->Kp_2_T2 = 0.0f;
        p->Ki_2_T2 = 0.0f;

#if SERVO_FW_DRIVE_BRUSHED
        /* —— 编译开关 SERVO_FW_DRIVE_BRUSHED=1：FW 有刷 ——
         * current_sign=-1 与旧 FwCurrentButter 符号一致；闭 fwiloop 输出 FW_PWM。 */
        p->Kp_I = 700.0f;       /* 有刷 FW 电流环 Kp */
        p->Ki_I = 200000.0f;    /* 有刷 FW 电流环 Ki */
        p->SamT_I = 0.001f;
        p->Umax = 4000.0f;      /* H 桥 PWM 占空比上限 */
#endif
    }
}

/* ========================================================================
 *                       在线整定（调参）接口实现
 * ====================================================================== */
/* dev_servo_axis_set_param / get_param：上位机或调试器在线改写 PI 系数。
 * idx 仅对 FW 的 Kp_A/Ki_A/Kp_V/Ki_V 数组有效（0~5 档）；GD 固定用 idx=0。 */

/**
 * @brief 在线写入单个整定参数
 * @note  id 为参数枚举；idx 为增益档索引（FW 0~5）；val 为新系数值
 */
bool dev_servo_axis_set_param(dev_servo_axis_t *pobj, servo_tune_param_e id, uint8_t idx, float val)
{
    if (pobj == NULL || id >= SERVO_TUNE_PARAM_MAX) {
        return false; /* 空对象或非法参数 ID */
    }
    servo_axis_ctrl_param_t *p = &pobj->param;

    /* 带增益调度的环路用 idx，越界保护：GD 只有 [0]，FW 0~5 */
    if (idx >= 6u) {
        idx = 5u; /* 钳到最大档，防止数组越界 */
    }

    switch (id) {
        case SERVO_TUNE_KP_I:       /* 电流环比例 Kp_I（A/V 或占空比/A） */
            p->Kp_I = val;
            break;
        case SERVO_TUNE_KI_I:       /* 电流环积分 Ki_I */
            p->Ki_I = val;
            break;
        case SERVO_TUNE_KP_A:       /* 陀螺加速度环 Kp_A[idx] */
            p->Kp_A[idx] = val;
            break;
        case SERVO_TUNE_KI_A:       /* 陀螺加速度环 Ki_A[idx] */
            p->Ki_A[idx] = val;
            break;
        case SERVO_TUNE_KP_V:       /* 陀螺速度环 Kp_V[idx] */
            p->Kp_V[idx] = val;
            break;
        case SERVO_TUNE_KI_V:       /* 陀螺速度环 Ki_V[idx] */
            p->Ki_V[idx] = val;
            break;
        case SERVO_TUNE_KP_EV:      /* 编码器轴速度环 Kp_Ev */
            p->Kp_Ev = val;
            break;
        case SERVO_TUNE_KI_EV:      /* 编码器轴速度环 Ki_Ev */
            p->Ki_Ev = val;
            break;
        case SERVO_TUNE_KP_P:       /* 轴位置环 Kp_P */
            p->Kp_P = val;
            break;
        case SERVO_TUNE_KI_P:       /* 轴位置环 Ki_P */
            p->Ki_P = val;
            break;
        case SERVO_TUNE_KP_G:       /* 地理/稳定环 Kp_G */
            p->Kp_G = val;
            break;
        case SERVO_TUNE_KI_G:       /* 地理/稳定环 Ki_G */
            p->Ki_G = val;
            break;
        case SERVO_TUNE_IMAX:       /* 电流环输出限幅 Imax（A） */
            p->Imax = val;
            break;
        case SERVO_TUNE_AMAX:       /* 陀螺速度环输出限幅 Amax */
            p->Amax = val;
            break;
        default:
            return false; /* 未实现的整定项 */
    }
    return true;
}

/**
 * @brief 在线读取单个整定参数
 * @note  参数含义与 set_param 对称；非法 id 返回 0
 */
float dev_servo_axis_get_param(dev_servo_axis_t *pobj, servo_tune_param_e id, uint8_t idx)
{
    if (pobj == NULL || id >= SERVO_TUNE_PARAM_MAX) {
        return 0.0f;
    }
    servo_axis_ctrl_param_t *p = &pobj->param;
    if (idx >= 6u) {
        idx = 5u;
    }

    switch (id) {
        case SERVO_TUNE_KP_I:
            return p->Kp_I;
        case SERVO_TUNE_KI_I:
            return p->Ki_I;
        case SERVO_TUNE_KP_A:
            return p->Kp_A[idx];   /* 读第 idx 档加速度环 Kp */
        case SERVO_TUNE_KI_A:
            return p->Ki_A[idx];
        case SERVO_TUNE_KP_V:
            return p->Kp_V[idx];   /* 读第 idx 档速度环 Kp */
        case SERVO_TUNE_KI_V:
            return p->Ki_V[idx];
        case SERVO_TUNE_KP_EV:
            return p->Kp_Ev;
        case SERVO_TUNE_KI_EV:
            return p->Ki_Ev;
        case SERVO_TUNE_KP_P:
            return p->Kp_P;
        case SERVO_TUNE_KI_P:
            return p->Ki_P;
        case SERVO_TUNE_KP_G:
            return p->Kp_G;
        case SERVO_TUNE_KI_G:
            return p->Ki_G;
        case SERVO_TUNE_IMAX:
            return p->Imax;
        case SERVO_TUNE_AMAX:
            return p->Amax;
        default:
            return 0.0f;
    }
}

/* ========================================================================
 *                          内部工具：加载默认硬件配置
 * ====================================================================== */

/**
 * @brief 加载轴默认硬件/驱动配置（编码器零点、限位、驱动类型等）
 * @note  GD 为有限角轴；FW 为连续轴。有刷编译开关会覆盖 drive_type。
 */
static void axis_load_default_config(dev_servo_axis_t *pobj)
{
    servo_axis_config_t *c = &pobj->config;

    /* —— 编码器与保护阈值（全轴共用默认值）—— */
    /* 手持：20位 1048576 ；GA220：21位 2097152；GA270：24位 16777216（实际25位，只使用了24bit） */
    c->bmq_max_code = 16777216u; /* 单圈码盘总量，用于取模与翻圈判定 */
    c->fault_window_ms = SF_FAULT_WINDOW_MS;   /* 过流统计滑动窗口（ms） */
    c->fault_count_th = SF_FAULT_DANGER_TH;    /* 窗口内过流次数上限 */
    c->overcur_thresh_a = SF_OVERCUR_THRESH;  /* 单次采样过流判定阈值（A） */
    c->overspd_thresh_dps = SF_OVERSPEED_THRESH; /* 超速判定阈值（°/s，编码器 ev） */

    c->has_gyro = true; /* 默认有陀螺；dev_servo_axis_init 会按 get_gyro_raw 钩子改写 */

    /* —— 驱动方式：无刷电流指令 / 有刷 PWM+内环 iloop —— */
    c->drive_type = SERVO_DRIVE_CURRENT_CMD;
    c->current_sign = 1.0f;           /* 电流反馈符号（有刷时可能为 -1） */
    c->current_use_butter = false;    /* 无刷驱动器电流已滤波，不需 Butterworth */
    c->cur_calib_samples = SF_CUR_CALIB_SAMPLES; /* 零偏标定累加样本数 */

    if (pobj->id == SERVO_AXIS_GD) {
        /* —— GD（俯仰）轴专用 —— */
        c->bmq_zero = 1064350;//1024350;                               /* 机械零位对应编码器计数值 */
        c->bmq_coef_deg = SF_BMQ_COF_GD;               /* 1 码 → 角度（°） */
        c->gyro_factor = 0.0000292035869364061814765f; /* 陀螺原始码 → °/s */
        c->angle_up_limit = 82.0f;                     /* 俯仰软/控限位上限（°） */
        c->angle_dn_limit = -12.0f;                    /* 俯仰下限（°） */
        c->is_continuous = false; /* 有限角轴，set_pos_tgt 会裁剪目标角 */

                                  /* —— GD 有刷电机配置 ——
         * 仅当 SERVO_GD_DRIVE_BRUSHED=1 时生效；默认 0 → 维持无刷伺服驱动器。
         *   → 电流符号 -1、过 250Hz 二阶 Butterworth。
         * 闭电流环走 gdiloop，U_give 经 FY_PWM(TIMx->CCR1)输出。 */
#if SERVO_GD_DRIVE_BRUSHED
        c->drive_type = SERVO_DRIVE_BRUSHED_PWM;
        c->current_sign = 1.0f;
        c->current_use_butter = true;
#endif
    } else {
        /* —— FW（方位）轴专用 —— */
        c->bmq_zero = 7297419;                        /* 方位零位编码器计数值 */
        c->bmq_coef_deg = SF_BMQ_COF_FW;              /* 1 码 → 角度（°） */
        c->gyro_factor = 0.000029119908707921404318f; /* 方位陀螺换算系数 */
        c->angle_up_limit = 0.0f;   /* 连续轴：limit 仅用于环内抗饱和，非机械限位 */
        c->angle_dn_limit = 0.0f;
        c->is_continuous = true; /* 可跨 0 点累加多圈，目标角不裁剪 */

                                 /* —— 有刷电机配置 ——
         * 仅当编译开关 SERVO_FW_DRIVE_BRUSHED=1 时生效；
         * 对应原代码：
         *   FWControl.I_fb = -1 * FwCurrentButter(2,250,0.001,FW_Current);   → 符号 -1 + 滤波
         *   FWControl.U_give = fwiloop(...);  FW_PWM(FWControl.U_give);      → 闭电流环 + PWM */
#if SERVO_FW_DRIVE_BRUSHED
        c->drive_type = SERVO_DRIVE_BRUSHED_PWM;
        c->current_sign = -1.0f;
        c->current_use_butter = true;
#endif
    }
}

/* ========================================================================
 *                          传感器更新
 * ====================================================================== */
/*
 * 【Servolib 环函数与本文件映射（GD / FW）】
 * 编码器链：
 *   GdAxisEvDis / FwAxisEvDis — 连续角微分 → ev（°/s）
 *   GdeAccDis                 — GD 编码器 ev 再微分 → Ev_acc
 * 陀螺链：
 *   GdGyroButter / FwGyroButter — 陀螺低通 → gyro（FW 含 sec_couple）
 *   GdAccDis / FwAccDis         — 陀螺微分 → acc
 * 电流链：
 *   GdCurrentButter / FwCurrentButter — 有刷 ADC 电流滤波
 * GEO / 跟踪：
 *   gdgeo / fwgeo           — 稳定/地理环 → V_give
 *   gdtloop_TV1 / fwtloop_TV1 — 脱靶 TV1 → V_give
 * 陀螺级联（有陀螺）：
 *   gdvloop / fwvloop       — V_give → A_give（FW 6档 bound_V）
 *   gdaloop / fwaloop       — A_give → I_give（FW 6档 bound_A）
 * 编码器速度级联（无陀螺或 POSITION 后）：
 *   gdploop / fwploop       — P_give → Ev_give
 *   gdevloop / fwevloop     — Ev_give → I_give
 * 有刷最内层：
 *   gdiloop / fwiloop       — I_give → U_give（PWM 占空比）
 * 清除（clr_*）：
 *   clr_gd_p / clr_fw_p … — 按环清 PI 积分；clr_gd_all / clr_fw_all 全清
 *
 * 【IO 钩子约定】
 *   get_encoder_raw()      → 原始码值（uint32/int，未去零偏）
 *   get_gyro_raw()         → 陀螺原始码；NULL 表示无陀螺
 *   get_motor_current_mA() → 电流 mA；有刷 ADC 或无刷驱动器反馈
 *   set_motor_current_mA(mA, en) — 无刷电流指令
 *   set_motor_pwm(u, en)   — 有刷 U_give 与方向
 *   motor_hard_enable(en)  — 功率级使能
 *
 * 【dz_flag】
 *   传入 ploop/evloop，通常作死区/积分抑制开关，与旧代码 gdploop(..., dz) 一致。
 */

/**
 * @brief 编码器处理（替代 gdbmqdispose / fwbmqdispose）
 *
 *        流程：读原始码 → 去零偏+方向 → 单圈位置 → 机械角 → 翻圈计数 →
 *              sec/cos 解耦量 → 连续角微分得 ev（°/s）。
 */
static void axis_process_encoder(dev_servo_axis_t *pobj)
{
    servo_axis_sensor_t *s = &pobj->sensor;
    servo_axis_config_t *c = &pobj->config;

    /* 单圈码盘总量与半圈，用于 [-half, half) 折叠及翻圈判定 */
    const int32_t full = (int32_t)c->bmq_max_code;
    const int32_t half = full / 2;

    /* 方向符号：CW(默认)=+1 顺时针角度增大；CCW=-1 反向 */
    const int32_t sgn = (pobj->dir == SERVO_AXIS_DIR_CCW) ? -1 : +1;

    /* Step1：读编码器原始计数值，保留上一拍用于算增量 */
    s->bmq_raw_old = s->bmq_raw;
    if (pobj->io.get_encoder_raw)
        s->bmq_raw = pobj->io.get_encoder_raw();

    /* Step2：去机械零点并乘方向 → 本拍单圈相对位置 pos ∈ [-half, half) */
    int32_t pos = sgn * ((int32_t)s->bmq_raw - c->bmq_zero);
    pos %= full;
    if (pos < 0)
        pos += full;
    if (pos >= half)
        pos -= full;

    /* Step2b：上一拍单圈位置（同样带方向），供 Step4 算 delta */
    int32_t pos_old = sgn * ((int32_t)s->bmq_raw_old - c->bmq_zero);
    pos_old %= full;
    if (pos_old < 0)
        pos_old += full;
    if (pos_old >= half)
        pos_old -= full;

    s->bmq_data = pos; /* 单圈有符号码值，单位：码 */

    /* Step3：码值转机械角（°），angle_360 映射到 [0,360) */
    s->angle = c->bmq_coef_deg * (float)s->bmq_data;
    s->angle_360 = (s->angle < 0.0f) ? (s->angle + 360.0f) : s->angle;

    /* Step4：本拍增量 delta_raw，越过 ±half 时调整 lap 圈计数
     * 前提：单拍位移 < 半圈（1ms 周期下恒成立） */
    int32_t delta_raw = pos - pos_old; /* 原始差 ∈ (-full, full) */
    if (delta_raw >= half) {
        delta_raw -= full;
        s->lap--;                       /* 反向越过 -180° 边界 → 减一圈 */
    } else if (delta_raw < -half) {
        delta_raw += full;
        s->lap++;                       /* 正向越过 +180° 边界 → 加一圈 */
    }

    /* Step5：俯仰角 cos/sec，FW 跟踪时用于 1/cos(GD) 解耦 */
    s->Cos = cosf(s->angle * SF_DEG2RAD);
    if (fabsf(s->Cos) > 1e-6f)
        s->Sec = 1.0f / s->Cos;

    /* Step6：连续累计角（°）→ 微分滤波得 ev（编码器角速度，°/s） */
    s->angle_vel_raw = ((double)s->lap * (double)full + (double)s->bmq_data) * (double)c->bmq_coef_deg;
    if (pobj->id == SERVO_AXIS_GD) {
        /* GD：2 阶 30Hz 微分 + 70Hz 加速度微分 Ev_acc */
        s->ev = GdAxisEvDis(2, 30, 0.001f, (float)s->angle_vel_raw);
        s->Ev_acc = GdeAccDis(70.0f, 0.001f, s->ev);
    } else {
        /* FW：独立滤波器 FwAxisEvDis */
        s->ev = (float)FwAxisEvDis(2, 30, 0.001f, s->angle_vel_raw);
    }
}

/**
 * @brief 速度/加速度/电流反馈处理（替代 gyrodispose 的轴对应部分）
 *
 *        有陀螺：速度=陀螺滤波、加速度=陀螺微分；
 *        无陀螺：速度=编码器微分 ev、加速度=编码器微分 Ev_acc（GD）。
 */
static void axis_process_gyro(dev_servo_axis_t *pobj)
{
    servo_axis_sensor_t *s = &pobj->sensor;
    servo_axis_config_t *c = &pobj->config;

    /* —— Step A：电流反馈（A）—— 与陀螺有无无关，每拍先处理 */
    /* 无刷 CURRENT_CMD：驱动器回传真实电流，sign=+1、不滤波、bias=0
     * 有刷 BRUSHED_PWM：ADC→Butterworth→sign→减 current_bias */
    if (pobj->io.get_motor_current_mA) {
        float raw = (float)pobj->io.get_motor_current_mA() * 0.001f; /* mA → A */
        if (c->drive_type == SERVO_DRIVE_BRUSHED_PWM && c->current_use_butter) {

            /* 2 阶 ~400Hz Butterworth；FW/GD 各用独立滤波器状态 */
            raw = (pobj->id == SERVO_AXIS_GD) ? GdCurrentButter(2, 400, pobj->param.SamT_I, raw)
                                              : FwCurrentButter(2, 400, pobj->param.SamT_I, raw);
        }
        s->current = c->current_sign * raw - s->current_bias; /* 符号+零偏校正 */
    }

    /* —— Step B：无陀螺分支 — 速度/加速度反馈取自编码器微分 —— */
    if (!c->has_gyro) {
        s->gyro_raw = 0.0f;
        s->gyro = s->ev; /* V_fb 用编码器 ev（°/s） */

        /* FW 无 Ev_acc，无陀螺时不参与加速度环 */
        s->acc = (pobj->id == SERVO_AXIS_GD) ? s->Ev_acc : 0.0f;
        return;
    }

    /* —— Step C：有陀螺 — 读原始陀螺并换算 —— */
    int32_t raw = 0;
    if (pobj->io.get_gyro_raw) {
        raw = pobj->io.get_gyro_raw();
    }

    /* 与原代码符号约定一致：取负 + 去 gyro_bias，再乘 gyro_factor → °/s */
    s->gyro_raw = -1.0f * ((float)raw * c->gyro_factor - s->gyro_bias);

    if (pobj->id == SERVO_AXIS_GD) {
        /* GD：70Hz 低通 + ±300°/s 限幅；加速度由原始陀螺微分 GdAccDis */
        s->gyro = GdGyroButter(1, 70, 0.001f, s->gyro_raw);
        if (s->gyro > 300.0f)
            s->gyro = 300.0f;
        if (s->gyro < -300.0f)
            s->gyro = -300.0f;
        s->acc = GdAccDis(100, 0.001f, s->gyro_raw);
    } else {
        /* FW：滤波带 sec_couple；±500°/s 限幅；FwAccDis 含解耦 */
        s->gyro = FwGyroButter(1, 70, 0.001f, s->gyro_raw, pobj->sec_couple);
        if (s->gyro > 500.0f)
            s->gyro = 500.0f;
        if (s->gyro < -500.0f)
            s->gyro = -500.0f;
        s->acc = FwAccDis(100, 0.001f, s->gyro_raw, pobj->sec_couple);
    }
    /* 加速度反馈限幅 ±3000（单位与 Servolib 一致） */
    if (s->acc > 3000.0f)
        s->acc = 3000.0f;
    if (s->acc < -3000.0f)
        s->acc = -3000.0f;
}

/**
 * @brief 周期性传感器刷新（外部 1ms 调用）
 */
static void axis_update_sensor(dev_servo_axis_t *pobj)
{
    if (pobj == NULL)
        return;
    axis_process_encoder(pobj);  /* Step1：编码器 → angle/ev/lap/sec */
    axis_process_gyro(pobj);     /* Step2：电流 + 陀螺（或无陀螺 ev 分支） */
    axis_check_soft_limit(pobj); /* Step3：刷新 xw_up/xw_dn（ReadXW 等价） */
}

/**
 * @brief 把传感器观测同步到控制环反馈字段（fbdispose 等价物）
 *        无陀螺时速度/加速度反馈取编码器微分量。
 */
static void axis_sync_feedback(dev_servo_axis_t *pobj)
{
    if (pobj == NULL)
        return;

    /* 电流/位置/编码器速度：始终来自 sensor，供各级 PI 反馈 */
    pobj->rt.I_fb = pobj->sensor.current;  /* A */
    pobj->rt.P_fb = pobj->sensor.angle;    /* ° 机械角 */
    pobj->rt.Ev_fb = pobj->sensor.ev;      /* °/s 编码器微分速度 */

    if (pobj->config.has_gyro) {
        /* 有陀螺：V/A 反馈用陀螺滤波链 */
        pobj->rt.V_fb = pobj->sensor.gyro;
        pobj->rt.A_fb = pobj->sensor.acc;
    } else {
        /* 无陀螺：V 用 ev；A 仅 GD 有 Ev_acc，FW 置 0 跳过 A 环 */
        pobj->rt.V_fb = pobj->sensor.ev;

        pobj->rt.A_fb = (pobj->id == SERVO_AXIS_GD) ? pobj->sensor.Ev_acc : 0.0f;
    }
}
/* ========================================================================
 *                          开环调试变量
 * ====================================================================== */
/* 开环 PWM 测试给定：仅在 run_loop(level=SERVO_LOOP_VOLTAGE 电压开环) 下生效。
 * 符号=方向，幅值自动限到 ±Umax(默认4000)。调试器 Watch 里直接改 g_olp_u_give[轴]。
 *   [0]=方位FW   [1]=俯仰GD */
volatile float g_olp_u_give[SERVO_AXIS_MAX] = {0};

/**
 * @brief 开环 PWM：把调试给定限幅后作为 U_give，旁路全部控制环。
 *        下游 axis_enable() 的 set_motor_pwm(U_give,true) 据此输出占空比+方向。
 */
static inline void axis_voltage_open_loop(dev_servo_axis_t *pobj)
{
    /* 从全局调试变量取开环电压给定（Watch 修改 g_olp_u_give[id]） */
    float u = g_olp_u_give[pobj->id];
    const float umax = pobj->param.Umax; /* 限幅到 ±Umax，防止超出 PWM 周期 */
    if (u > umax)
        u = umax;
    if (u < -umax)
        u = -umax;
    pobj->rt.U_give = u;      /* 开环电压直接作为 PWM 依据 */
    pobj->rt.I_give = 0.0f;   /* 电流给定清零，避免观测混淆 */
}

/* ========================================================================
 *                          级联控制环编排
 * ====================================================================== */
/*
 * 【环路层级 servo_loop_level_e 说明（由外到内）】
 * SERVO_LOOP_GEO      — 地理/稳定环，GEO_give → V_give
 * SERVO_LOOP_TRACK    — 脱靶 TV1 跟踪，T_fb → V_give
 * SERVO_LOOP_V_SPEED  — 陀螺速度环（或 Ev 直通），V_give → A_give 或 Ev_give
 * SERVO_LOOP_ACC      — 陀螺加速度环，A_give → I_give
 * SERVO_LOOP_POSITION — 轴位置环，P_give → Ev_give
 * SERVO_LOOP_EV_SPEED — 编码器惯性速度环，Ev_give → I_give
 * SERVO_LOOP_CURRENT  — 电流环透传，I_give 由外部给定
 * SERVO_LOOP_VOLTAGE  — 开环 PWM，U_give 来自 g_olp_u_give
 *
 * GD/FW 的 switch 使用 intentional fall-through：
 * 调用 run_loop(POSITION) 会执行 P 环 + Ev 环；调用 run_loop(V_SPEED) 从 V 环算到 I。
 * 有刷驱动在函数末尾统一 axis_close_current_loop：I_give → U_give → PWM。
 */
// -------------------------------------------------------------------------
// GD run_loop 分支速查（level → 执行的环）
// -------------------------------------------------------------------------
// GEO        → gdgeo → V_give；可 fall-through 到 TRACK/V_SPEED
// TRACK      → gdtloop_TV1（仅 level==TRACK 时）→ V_give
// V_SPEED    → 有陀螺: gdvloop→A_give + gdaloop→I_give
//              无陀螺: Ev_give=V_give + gdevloop→I_give
// ACC        → gdaloop(A_give→I_give)
// POSITION   → gdploop→Ev_give，fall-through EV_SPEED→gdevloop→I_give
// EV_SPEED   → gdevloop(Ev_give→I_give)
// CURRENT    → 透传 I_give
// VOLTAGE    → axis_voltage_open_loop，跳过 iloop
// -------------------------------------------------------------------------
// FW run_loop 分支速查（另含 sec_couple 与 gd_abs_angle 增益档）
// -------------------------------------------------------------------------
// GEO        → fwgeo(..., sec_couple)
// TRACK      → fwtloop_TV1(..., sec_couple)
// V_SPEED    → 有陀螺: fwvloop(bound_V)+fwaloop(bound_A)
//              无陀螺: fwevloop
// ACC        → fwaloop(bound_A)
// POSITION   → fwploop → fall-through fwevloop
// EV_SPEED   → fwevloop
// CURRENT    → 透传
// VOLTAGE    → 开环 PWM
// -------------------------------------------------------------------------
// gdvloop / fwvloop 入参：V_give,V_fb, angle, limit（GD 上下限参与抗饱和）
// gdaloop / fwaloop 入参：A_give,A_fb, Kg_A, BW, Imax；FW 另传 gd_abs_angle,bound_A
// gdevloop / fwevloop 入参：Ev_give,Ev_fb, angle, limit, dz_flag
// gdploop / fwploop 入参：P_give,P_fb, bound_P, Ev_Set, Evmax, dz_flag
// gdtloop_TV1 / fwtloop_TV1：T_fb, 分段 PI, Vmax；FW 加 sec_couple
// -------------------------------------------------------------------------
// 返回值：始终 r->I_give（A）；有刷时在函数末尾转 U_give 送 PWM
// p_error  ：每轴在 switch 后更新 P_give-P_fb（°），供到位判定
// dz_flag  ：位置/速度环 integral 抑制，与 legacy 一致
// use_gyro ：config.has_gyro，false 时跳过 V/A 陀螺环
// gd_abs_angle：FW 专用，fabs(GD.angle)，选 bound_V/A 档位
// -------------------------------------------------------------------------
// FW bound_V / bound_A 默认边界（°，相对 |GD 俯仰|）：
//   档区 0: |GD|≥75  档区1: ≥70  档区2: ≥60  档区3: ≥40  档区4: ≥30  档区5: <30
//   仰角越大选用越高档，Kp_V/Ki_V/Kp_A/Ki_A 越大，补偿重力/耦合
// -------------------------------------------------------------------------
// GD 有限角 limit：up=82 dn=-12 传入 ploop/vloop/evloop 抗积分饱和
// FW limit 参数在连续轴上主要用于环内 clamp，非机械限位
// -------------------------------------------------------------------------
// 开环调试：level=VOLTAGE 时不调用 axis_close_current_loop
// 无刷 CURRENT_CMD：enable 发送 I_give(mA)；有刷 BRUSHED_PWM：enable 发送 U_give
// -------------------------------------------------------------------------
// clr 顺序建议：模式切换时 brake → clear_loops(TRACK或POSITION) → 改给定 → enable
// -------------------------------------------------------------------------
// TV2 系数全零，但 clr_gd_t_2/3、clr_fw_t_2/3 仍保留，便于扩展第二跟踪环
// -------------------------------------------------------------------------
// sec_couple 由 axis_set_couple 设置，FW 陀螺/加速度/GEO/TV 路径共用
// -------------------------------------------------------------------------
// Imax：电流环输出上限(A)；无刷时为送驱动器的最大电流指令
// Amax：陀螺速度环输出的加速度给定上限，喂给 A 环
// Evmax：位置环输出的最大轴速度(°/s)，喂给 Ev 环
// Vmax ：TV/GEO 输出的最大跟踪角速度(°/s)
// Umax ：有刷 PWM 电压/占空比限幅，iloop 与开环共用
// -------------------------------------------------------------------------
// SamT 与 SamT_I 均为 0.001s；滤波器函数（Butter/Dis）也使用 0.001
// -------------------------------------------------------------------------
// 编码器 ev 限速：GdAxisEvDis/FwAxisEvDis 2阶30Hz，抑制量化噪声
// 陀螺 Butter：1阶70Hz；FW 路径乘以 sec_couple 补偿俯仰
// 电流 Butter：有刷 2阶400Hz（原注释250Hz，现参数400）
// -------------------------------------------------------------------------
// axis_process_encoder 输出 ev 供 Ev_fb；有陀螺时 V_fb 仍用 gyro
// 无陀螺时 V_fb=ev，FW 的 A_fb=0，run_loop 跳过 fwaloop 需由 V_SPEED 分支走 fwevloop
// -------------------------------------------------------------------------
// brake_locked 与 fault 独立：limit_hit 不锁死，仅 overcurrent/overspeed 锁死
// clear_fault 不清 enabled，需外部再 enable
// -------------------------------------------------------------------------
// inplace：update_in_place 在 enable 之外调用，500ms 驻留+0.1° 误差
// soft_limit：check_soft_limit 在 update_sensor 末执行，默认 soft_limit_enable 可能 false
// -------------------------------------------------------------------------
// offset_deg：axis_set_zero 设置，当前产品未使用；零偏以 config.bmq_zero 为准
// dir：axis_set_dir 影响编码器 pos 符号，进而影响 angle/ev 符号
// -------------------------------------------------------------------------
// calibrate_current_bias：电机静止时调用 N 次，写 sensor.current_bias
// get_param/set_param：idx 对 FW 数组环有效；GD 仅 idx=0
// -------------------------------------------------------------------------
// g_olp_u_give[0]=FW g_olp_u_give[1]=GD，开环幅值单位与 Umax 一致
// -------------------------------------------------------------------------
// 旧代码映射：gdbmqdispose→axis_process_encoder；fbdispose→sync_feedback+process_gyro
// gd_en/fw_en→axis_check_protect+axis_enable；CurrentInit→axis_calibrate_current_bias
// -------------------------------------------------------------------------

/**
 * @brief 有刷电机的内层电流环：I_give → U_give
 *        无刷伺服驱动器跳过本函数，I_give 直接送驱动器。
 */
static void axis_close_current_loop(dev_servo_axis_t *pobj)
{
    servo_axis_ctrl_param_t *p = &pobj->param;
    servo_axis_rt_t *r = &pobj->rt;
    if (pobj->id == SERVO_AXIS_GD) {
        r->U_give = gdiloop(p->Kp_I, p->Ki_I, p->SamT_I, p->Umax, r->I_give, r->I_fb); /* GD 有刷 iloop */
    } else {
        r->U_give = fwiloop(p->Kp_I, p->Ki_I, p->SamT_I, p->Umax, r->I_give, r->I_fb); /* FW 有刷 iloop */
    }
}

/**
 * @brief 按指定环路层级执行级联 PI（核心控制编排）
 *
 *        level 指定“从哪一层开始算到最内层”。GD/FW 的 switch 故意 fall-through：
 *        例如 SERVO_LOOP_POSITION 会依次执行 P 环 → Ev 环 →（有刷再 iloop）。
 *        FW 的 gd_abs_angle 取 |GD角| 供 6 档增益调度。
 *
 * @return 最内层电流给定 I_give（A），供外层观测或电流环透传
 */
static float axis_run_loop(dev_servo_axis_t *pobj, servo_loop_level_e level, uint8_t dz_flag)
{
    if (pobj == NULL)
        return 0.0f;

    servo_axis_ctrl_param_t *p = &pobj->param;
    servo_axis_rt_t *r = &pobj->rt;
    const bool use_gyro = pobj->config.has_gyro;

    /* FW 用 |GD 俯仰角| 选增益档；GD 自身用俯仰角（可负） */
    const float gd_abs_angle = (pobj->id == SERVO_AXIS_FW) ? fabsf(pobj->sensor.angle) : pobj->sensor.angle;

    if (pobj->id == SERVO_AXIS_GD) {
        /* ===================== GD 轴级联（故意 fall-through）===================== */
        switch (level) {
            case SERVO_LOOP_GEO:
                /* GEO 稳定环：GEO_give/GEO_fb → V_give（°/s），为最外环之一 */
                r->V_give =
                    gdgeo(p->Kp_G, p->Ki_G, p->SamT, p->bound_G, p->V_Set, p->Vmax, r->GEO_give, r->GEO_fb);

            case SERVO_LOOP_TRACK:
                if (level == SERVO_LOOP_TRACK) {
                    /* TV1 脱靶跟踪：T_fb → V_give，仅 level==TRACK 时单独调用 */
                    r->V_give = gdtloop_TV1(p->Kp_1_T1,
                                            p->Ki_1_T1,
                                            p->bound_T1,
                                            p->Kp_2_T1,
                                            p->Ki_2_T1,
                                            p->SamT,
                                            p->Vmax,
                                            r->T_fb);
                }
            case SERVO_LOOP_V_SPEED:
                /* 陀螺速度级：V_give →（V环→A环→I）或（Ev环→I） */
                if (use_gyro) {
                    /* 有陀螺：gdvloop 输出 A_give，gdaloop 输出 I_give */
                    r->A_give = gdvloop(p->Kp_V[0],
                                        p->Ki_V[0],
                                        p->SamT,
                                        p->Amax,
                                        pobj->config.angle_up_limit,
                                        pobj->config.angle_dn_limit,
                                        r->V_give,
                                        r->V_fb,
                                        pobj->sensor.angle);
                    r->I_give = gdaloop(
                        p->Kp_A[0], p->Ki_A[0], p->SamT, p->Kg_A, p->BW_A[0], p->Imax, r->A_give, r->A_fb);
                } else {
                    /* 无陀螺：V_give 直通 Ev_give，gdevloop 用编码器 ev 闭环到 I_give */
                    r->Ev_give = r->V_give;
                    r->I_give = gdevloop(p->Kp_Ev,
                                         p->Ki_Ev,
                                         p->SamT,
                                         p->Imax,
                                         pobj->config.angle_up_limit,
                                         pobj->config.angle_dn_limit,
                                         r->Ev_give,
                                         r->Ev_fb,
                                         pobj->sensor.angle,
                                         dz_flag);
                }
                break;

            case SERVO_LOOP_ACC:
                /* 仅加速度环：A_give → I_give（上层已给 A_give） */
                r->I_give = gdaloop(
                    p->Kp_A[0], p->Ki_A[0], p->SamT, p->Kg_A, p->BW_A[0], p->Imax, r->A_give, r->A_fb);
                break;

            case SERVO_LOOP_POSITION:
                /* 位置环：P_give/P_fb → Ev_give（°/s） */
                r->Ev_give = gdploop(p->Kp_P,
                                     p->Ki_P,
                                     p->SamT,
                                     pobj->config.angle_up_limit,
                                     pobj->config.angle_dn_limit,
                                     p->bound_P,
                                     p->Ev_Set,
                                     p->Evmax,
                                     r->P_give,
                                     r->P_fb,
                                     dz_flag);
            case SERVO_LOOP_EV_SPEED:
                /* fall-through：轴惯性速度环 Ev_give → I_give */
                r->I_give = gdevloop(p->Kp_Ev,
                                     p->Ki_Ev,
                                     p->SamT,
                                     p->Imax,
                                     pobj->config.angle_up_limit,
                                     pobj->config.angle_dn_limit,
                                     r->Ev_give,
                                     r->Ev_fb,
                                     pobj->sensor.angle,
                                     dz_flag);
                break;

            case SERVO_LOOP_CURRENT:
                /* 电流环透传：I_give 由外部写入，本层不计算 */
                break;
            case SERVO_LOOP_VOLTAGE:
                /* 开环 PWM：旁路全部 PI，U_give 来自 g_olp_u_give */
                axis_voltage_open_loop(pobj);
                break;
        }
        r->p_error = r->P_give - r->P_fb; /* 位置误差（°），供到位/显示 */
    } else {
        /* ===================== FW 轴级联（含 sec_couple 与 6 档增益）===================== */
        switch (level) {
            case SERVO_LOOP_GEO:
                /* GEO 环：含 sec 耦合项，输出 V_give */
                r->V_give = fwgeo(p->Kp_G,
                                  p->Ki_G,
                                  p->SamT,
                                  p->bound_G,
                                  p->V_Set,
                                  p->Vmax,
                                  r->GEO_give,
                                  r->GEO_fb,
                                  pobj->sec_couple);
            case SERVO_LOOP_TRACK:
                if (level == SERVO_LOOP_TRACK) {
                    /* TV1 跟踪：T_fb + sec_couple → V_give */
                    r->V_give = fwtloop_TV1(p->Kp_1_T1,
                                            p->Ki_1_T1,
                                            p->bound_T1,
                                            p->Kp_2_T1,
                                            p->Ki_2_T1,
                                            p->SamT,
                                            p->Vmax,
                                            r->T_fb,
                                            pobj->sec_couple);
                }
            case SERVO_LOOP_V_SPEED:
                if (use_gyro) {
                    /* 有陀螺：fwvloop→A_give，fwaloop→I_give，增益随 gd_abs_angle 调度 */
                    r->A_give = fwvloop(p->Kp_V,
                                        p->Ki_V,
                                        p->SamT,
                                        p->Amax,
                                        pobj->config.angle_dn_limit,
                                        pobj->config.angle_up_limit,
                                        r->V_give,
                                        r->V_fb,
                                        pobj->sensor.angle,
                                        gd_abs_angle,
                                        p->bound_V);
                    r->I_give = fwaloop(p->Kp_A,
                                        p->Ki_A,
                                        p->SamT,
                                        p->Kg_A,
                                        p->BW_A,
                                        p->Imax,
                                        r->A_give,
                                        r->A_fb,
                                        gd_abs_angle,
                                        p->bound_A);
                } else {
                    /* 无陀螺：V_give→Ev_give，fwevloop 闭环到 I_give */
                    r->Ev_give = r->V_give;
                    r->I_give = fwevloop(p->Kp_Ev,
                                         p->Ki_Ev,
                                         p->SamT,
                                         p->Imax,
                                         pobj->config.angle_dn_limit,
                                         pobj->config.angle_up_limit,
                                         r->Ev_give,
                                         r->Ev_fb,
                                         pobj->sensor.angle,
                                         dz_flag);
                }
                break;

            case SERVO_LOOP_ACC:
                /* 仅加速度环：fwaloop，带 gd_abs_angle 增益档 */
                r->I_give = fwaloop(p->Kp_A,
                                    p->Ki_A,
                                    p->SamT,
                                    p->Kg_A,
                                    p->BW_A,
                                    p->Imax,
                                    r->A_give,
                                    r->A_fb,
                                    gd_abs_angle,
                                    p->bound_A);
                break;

            case SERVO_LOOP_POSITION:
                /* 位置环：fwploop → Ev_give */
                r->Ev_give = fwploop(p->Kp_P,
                                     p->Ki_P,
                                     p->SamT,
                                     pobj->config.angle_dn_limit,
                                     pobj->config.angle_up_limit,
                                     p->bound_P,
                                     p->Ev_Set,
                                     p->Evmax,
                                     r->P_give,
                                     r->P_fb,
                                     dz_flag);
            case SERVO_LOOP_EV_SPEED:
                /* fall-through：fwevloop 轴速度环 → I_give */
                r->I_give = fwevloop(p->Kp_Ev,
                                     p->Ki_Ev,
                                     p->SamT,
                                     p->Imax,
                                     pobj->config.angle_dn_limit,
                                     pobj->config.angle_up_limit,
                                     r->Ev_give,
                                     r->Ev_fb,
                                     pobj->sensor.angle,
                                     dz_flag);
                break;

            case SERVO_LOOP_CURRENT:
                /* 电流给定透传 */
                break;
            case SERVO_LOOP_VOLTAGE:
                /* 开环 PWM 测试 */
                axis_voltage_open_loop(pobj);
                break;
        }
        r->p_error = r->P_give - r->P_fb;
    }

    /* 有刷电机：I_give 再经 gdiloop/fwiloop 得到 U_give，供 set_motor_pwm */
    if (pobj->config.drive_type == SERVO_DRIVE_BRUSHED_PWM && level != SERVO_LOOP_VOLTAGE) {
        axis_close_current_loop(pobj);
    }

    return r->I_give;
}

/* ========================================================================
 *                          PI 寄存器清除
 * ====================================================================== */

/**
 * @brief 从指定环路层级起，清除对应 PI 积分/状态寄存器
 * @note  from_level 及以上层级全部 clr；调用 Servolib 预编译 clr_xxx 函数
 */
static void axis_clear_loops(dev_servo_axis_t *pobj, servo_loop_level_e from_level)
{
    if (pobj == NULL)
        return;

    /* 按轴 ID 分流；clr_xxx_all 粒度较粗，此处按 level 分段清除 */
    if (pobj->id == SERVO_AXIS_GD) {
        if (from_level >= SERVO_LOOP_GEO)
            clr_gd_g();           /* 清 GEO 环状态 */
        if (from_level >= SERVO_LOOP_TRACK) {
            clr_gd_t_1();         /* 清 TV1 跟踪环 */
            clr_gd_t_2();         /* 清 TV2（保留接口） */
            clr_gd_t_3();
        }
        if (from_level >= SERVO_LOOP_POSITION)
            clr_gd_p();           /* 清位置环 */
        if (from_level >= SERVO_LOOP_EV_SPEED)
            clr_gd_ev();          /* 清编码器速度环 */
        if (from_level >= SERVO_LOOP_V_SPEED)
            clr_gd_v();           /* 清陀螺速度环 */
        if (from_level >= SERVO_LOOP_ACC)
            clr_gd_a();           /* 清加速度环 */
        if (from_level >= SERVO_LOOP_CURRENT)
            clr_gd_i();           /* 清电流环 */
        clr_gddg();               /* 清 GD 微分/滤波辅助状态 */
    } else {
        if (from_level >= SERVO_LOOP_GEO)
            clr_fw_g();
        if (from_level >= SERVO_LOOP_TRACK) {
            clr_fw_t_1();
            clr_fw_t_2();
            clr_fw_t_3();
        }
        if (from_level >= SERVO_LOOP_POSITION)
            clr_fw_p();
        if (from_level >= SERVO_LOOP_EV_SPEED)
            clr_fw_ev();
        if (from_level >= SERVO_LOOP_V_SPEED)
            clr_fw_v();
        if (from_level >= SERVO_LOOP_ACC)
            clr_fw_a();
        if (from_level >= SERVO_LOOP_CURRENT)
            clr_fw_i();
        clr_fwdg();               /* 清 FW 微分/滤波辅助状态 */
    }
}

/**
 * @brief 清除本轴全部控制环 PI 状态（模式切换/急停后调用）
 */
static void axis_clear_all(dev_servo_axis_t *pobj)
{
    if (pobj == NULL)
        return;
    if (pobj->id == SERVO_AXIS_GD)
        clr_gd_all();
    else
        clr_fw_all();
}

/* ========================================================================
 *                          使能 / 刹车 / 保护
 * ====================================================================== */
/*
 * 【保护 → 使能 状态机】
 *   正常：check_protect 未触发 → enable 输出 I_give 或 U_give
 *   过流：1s 内 |I_fb|≥5A 计次 ≥800 → fault.overcurrent + brake_locked
 *   超速：|ev|≥100°/s 连续 1000 拍 → fault.overspeed + brake_locked
 *   锁死：brake_locked 时 enable 强制关输出，status.enabled=false
 *   恢复：clear_fault() 清 fault 与 lock → 再次 enable
 *
 * 【brake vs enable 锁死】
 *   brake()：主动停机，清给定与窗口计数，不 necessarily 置 fault 位
 *   enable()+lock：故障保护触发的强制关断，需 clear_fault
 */

/**
 * @brief 过流/超速保护检查（替代 gd_en/fw_en 中的故障判定段）
 *
 *        过流：1s 窗口内 |I_fb|≥阈值 的采样次数 ≥800 → 过流故障 + brake_locked。
 *        超速：编码器 ev 连续 ≥100°/s 达 1000 拍 → 超速故障 + brake_locked。
 */
static void axis_check_protect(dev_servo_axis_t *pobj)
{
    servo_axis_status_t *st = &pobj->status;
    servo_axis_config_t *cf = &pobj->config;

    /* Step1：过流滑动窗口统计（fault_window_ms=1000ms，每拍 +1） */
    if (st->fault_timer <= cf->fault_window_ms) {
        st->fault_timer++;
        if (fabsf(pobj->rt.I_fb) >= cf->overcur_thresh_a) {
            st->danger_count++;   /* 本拍电流超 5A，窗口计数 +1 */
        }
        if (st->fault_timer == cf->fault_window_ms) {
            /* 窗口期满：判定是否达危险次数阈值 */
            if (st->danger_count >= cf->fault_count_th) {
                st->fault.bit.overcurrent = 1; /* 置过流故障位 */
                st->brake_locked = true;       /* 锁死输出，需 clear_fault 解锁 */
            }
            st->fault_timer = 0;   /* 重置窗口 */
            st->danger_count = 0;
        }
    }

    /* Step2：超速连续计数（基于编码器 ev，阈值 100°/s） */
    if (fabsf(pobj->sensor.ev) >= cf->overspd_thresh_dps
        /*|| fabsf(pobj->sensor.gyro) >= cf->overspd_thresh_dps*/) {
        st->overspeed_count++;     /* 本拍超速，连续计数累加 */
    } else {
        st->overspeed_count = 0;   /* 速度正常则清零连续计数 */
    }
    if (st->overspeed_count >= SF_OVERSPEED_TH) {
        st->fault.bit.overspeed = 1;
        st->brake_locked = true;
        st->overspeed_count = 0;
    }
}

/**
 * @brief 使能电机输出：先保护检查，再按驱动类型下发 PWM 或电流指令
 *
 *        brake_locked 时强制关断；有刷输出 U_give→PWM，无刷输出 I_give→mA。
 */
static void axis_enable(dev_servo_axis_t *pobj)
{
    if (pobj == NULL)
        return;

    axis_check_protect(pobj); /* Step1：每使能周期执行过流/超速判定 */

    const bool brushed = (pobj->config.drive_type == SERVO_DRIVE_BRUSHED_PWM);

    if (pobj->status.brake_locked) {
        /* Step2a：故障锁死 — 硬使能关、输出清零、标记 disabled */
        if (pobj->io.motor_hard_enable)
            pobj->io.motor_hard_enable(false);
        if (brushed) {
            if (pobj->io.set_motor_pwm)
                pobj->io.set_motor_pwm(0.0f, false); /* 有刷：TIM CCR=0 */
        } else {
            if (pobj->io.set_motor_current_mA)
                pobj->io.set_motor_current_mA(0, false); /* 无刷：0mA */
        }
        pobj->status.enabled = false;
        return;
    }

    /* Step2b：正常使能 — 打开功率级 */
    if (pobj->io.motor_hard_enable)
        pobj->io.motor_hard_enable(true);

    if (brushed) {
        /* Step3a：有刷 — U_give 经 iloop 已算好，set_motor_pwm 负责方向与占空比 */
        if (pobj->io.set_motor_pwm)
            pobj->io.set_motor_pwm(pobj->rt.U_give, true);
    } else {
        /* Step3b：无刷 — I_give(A)→mA 送伺服驱动器，驱动器内部闭电流环 */
        if (pobj->io.set_motor_current_mA) {
            int32_t mA = (int32_t)(pobj->rt.I_give * 1000.0f);
            pobj->io.set_motor_current_mA(mA, true);
        }
    }
    pobj->status.enabled = true;
}

/**
 * @brief 主动刹车：关硬使能、清零输出与给定，复位保护计数器
 */
static void axis_brake(dev_servo_axis_t *pobj)
{
    if (pobj == NULL)
        return;

    /* Step1：关闭功率级硬使能 */
    if (pobj->io.motor_hard_enable)
        pobj->io.motor_hard_enable(false);

    /* Step2：按驱动类型清零物理输出 */
    if (pobj->config.drive_type == SERVO_DRIVE_BRUSHED_PWM) {
        if (pobj->io.set_motor_pwm)
            pobj->io.set_motor_pwm(0.0f, false);
        pobj->rt.U_give = 0.0f;   /* 同步清运行时电压给定 */
    } else {
        if (pobj->io.set_motor_current_mA)
            pobj->io.set_motor_current_mA(0, false);
    }

    /* Step3：清控制给定与保护滑动窗口（不自动 clear_fault 位） */
    pobj->rt.I_give = 0.0f;
    pobj->status.fault_timer = 0;
    pobj->status.danger_count = 0;
    pobj->status.enabled = false;
}

/* ========================================================================
 *                          给定 / 查询 / 配置
 * ====================================================================== */

/**
 * @brief 设置位置目标（°），有限角轴会裁剪到 angle_up/dn_limit
 */
static void axis_set_pos_tgt(dev_servo_axis_t *pobj, float deg)
{
    if (pobj == NULL)
        return;
    /* 限位裁剪（continuous 轴跳过，无限位），目标角度会被限制在机械范围内 */
    if (!pobj->config.is_continuous) {
        if (deg > pobj->config.angle_up_limit)
            deg = pobj->config.angle_up_limit;
        if (deg < pobj->config.angle_dn_limit)
            deg = pobj->config.angle_dn_limit;
    }
    pobj->rt.P_give = deg; /* 写入位置环给定 P_give（°） */
}

/**
 * @brief 设置角速度目标（°/s），供 TV/GEO/V 环使用
 */
static void axis_set_vel_tgt(dev_servo_axis_t *pobj, float dps)
{
    if (pobj == NULL)
        return;
    pobj->rt.V_give = dps;
}

/**
 * @brief 设置显示零偏：使当前机械角读数等于 angle_deg
 * @note  实际产品未使用；通过 offset_deg 实现，不改变 bmq_zero
 */
static bool axis_set_zero(dev_servo_axis_t *pobj, float angle_deg)
{
    if (pobj == NULL)
        return false;
    if (angle_deg < -360.0f || angle_deg > 360.0f)
        return false;

    /* 思路：当前编码角 - 期望显示角 = 偏置 */
    axis_process_encoder(pobj);
    pobj->offset_deg = fmodf(pobj->sensor.angle - angle_deg, 360.0f);
    if (pobj->offset_deg < 0.0f)
        pobj->offset_deg += 360.0f;
    return true;
}

/** @brief 读取当前机械角（°，已去 bmq_zero） */
static float axis_get_angle(dev_servo_axis_t *pobj)
{
    return (pobj == NULL) ? 0.0f : pobj->sensor.angle;
}

/** @brief 读取陀螺滤波角速度（°/s）；无陀螺时等同 ev */
static float axis_get_vel(dev_servo_axis_t *pobj)
{
    return (pobj == NULL) ? 0.0f : pobj->sensor.gyro;
}

/** @brief 读取故障字（16bit 位域，含过流/超速/限位等） */
static uint16_t axis_get_fault(dev_servo_axis_t *pobj)
{
    return (pobj == NULL) ? 0u : pobj->status.fault.all;
}

/**
 * @brief 清除故障位并解锁 brake_locked，允许再次 enable
 */
static void axis_clear_fault(dev_servo_axis_t *pobj)
{
    if (pobj == NULL)
        return;
    pobj->status.fault.all = 0u;
    pobj->status.brake_locked = false;
    pobj->status.fault_timer = 0;
    pobj->status.danger_count = 0;
    pobj->status.overspeed_count = 0;
}

/** @brief 设置 FW 方位跟踪 sec(θ) 耦合系数（解耦 GD 俯仰） */
static void axis_set_couple(dev_servo_axis_t *pobj, float sec)
{
    if (pobj == NULL)
        return;
    pobj->sec_couple = sec;
}

/** @brief 设置编码器/角度正方向（CW/CCW） */
static void axis_set_dir(dev_servo_axis_t *pobj, servo_axis_dir_e dir)
{
    if (pobj == NULL)
        return;
    pobj->dir = dir;
}

/** @brief 读取当前角度正方向枚举 */
static servo_axis_dir_e axis_get_dir(dev_servo_axis_t *pobj)
{
    return (pobj == NULL) ? SERVO_AXIS_DIR_CW : pobj->dir;
}

/**
 * @brief 电流零偏标定（CurrentInit 等价物）。
 */
static void axis_calibrate_current_bias(dev_servo_axis_t *pobj)
{
    /* 静态累加器：按轴 ID 索引，标定期间禁止电机转动 */
    static float sum[SERVO_AXIS_MAX] = {0.0f};
    static uint32_t cnt[SERVO_AXIS_MAX] = {0u};

    if (pobj == NULL)
        return;

    uint8_t i = (pobj->id < SERVO_AXIS_MAX) ? (uint8_t)pobj->id : 0u;
    uint16_t N = pobj->config.cur_calib_samples ? pobj->config.cur_calib_samples : SF_CUR_CALIB_SAMPLES;

    if (cnt[i] < N) {
        /* 还原未减 bias 的测量原值再累加，避免重复减偏 */
        sum[i] += pobj->sensor.current + pobj->sensor.current_bias;
        cnt[i]++;
        if (cnt[i] == N) {
            pobj->sensor.current_bias = sum[i] / (float)N; /* N 点均值 = 直流零偏（A） */
        }
    }
}

/**
 * @brief 软限位检查。
 *        仅作上报标志，不自动刹车。
 *        注意：阈值来自 config 的控制上下限。默认 soft_limit_enable=false，
 */
static void axis_check_soft_limit(dev_servo_axis_t *pobj)
{
    if (pobj == NULL)
        return;

    servo_axis_status_t *st = &pobj->status;
    servo_axis_config_t *c = &pobj->config;

    /* Step1：连续轴或未使能 soft_limit → 清标志并返回 */
    if (c->is_continuous || !c->soft_limit_enable) {
        st->xw_up = false;           /* 未触上限 */
        st->xw_dn = false;           /* 未触下限 */
        st->fault.bit.limit_hit = 0; /* 限位故障位清 */
        return;
    }

    /* Step2：与 config 控制上下限比较（°），置 xw_up/xw_dn */
    const float a = pobj->sensor.angle; /* 已去 bmq_zero 的机械角 */
    if (a >= c->angle_up_limit) {
        st->xw_up = true;   /* 角度 ≥ 上限 */
        st->xw_dn = false;
    } else if (a <= c->angle_dn_limit) {
        st->xw_up = false;
        st->xw_dn = true;   /* 角度 ≤ 下限 */
    } else {
        st->xw_up = false;
        st->xw_dn = false;  /* 在限位带内 */
    }

    /* Step3：限位标志映射到 fault.limit_hit（仅上报，不 brake） */
    if (st->xw_up || st->xw_dn)
        st->fault.bit.limit_hit = 1;
    else
        st->fault.bit.limit_hit = 0;
}

/**
 * @brief 判断某个伺服轴是否已经“稳定到位
 *        要求位置误差连续一段时间都小于阈值，才把 in_place 置为 true，相当于一个“到位防抖 / 驻留判定”。
 */
static void axis_update_in_place(dev_servo_axis_t *pobj)
{
    if (pobj == NULL)
        return;

    servo_axis_status_t *st = &pobj->status;

    /* Step1：|P_give - P_fb| < 0.1° 则驻留计数 +1，否则清零并 in_place=false */
    if (fabsf(pobj->rt.P_give - pobj->rt.P_fb) < SF_INPLACE_ERR_DEG) {
        if (st->inplace_dwell <= SF_INPLACE_DWELL_TH)
            st->inplace_dwell++;   /* 误差合格，累计驻留拍数 */
    } else {
        st->inplace_dwell = 0;     /* 误差超差，重新计时 */
        st->in_place = false;
    }

    /* Step2：驻留 >500 拍（约 500ms@1ms）才置 in_place=true，并钳住计数防溢出 */
    if (st->inplace_dwell > SF_INPLACE_DWELL_TH) {
        st->in_place = true;
        st->inplace_dwell = SF_INPLACE_DWELL_TH + 1;
    }
}

