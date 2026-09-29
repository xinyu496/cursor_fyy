/**
 * @file    SFlibhead.h
 * @brief   Servolib 预编译库公共头文件（原 SF.h 抽象版）/ Precompiled servo library public header
 *
 *          包含 GD/FW 高低轨伺服控制环 PI 状态结构、滤波器状态结构，
 *          以及 gd*loop / fw*loop 等控制算法、clr_* 清零复位与 Butterworth 滤波接口。
 *          Defines PI state structs for GD/FW orbit servo loops, filter state,
 *          gd/fw loop algorithms, clr_* reset helpers, and Butterworth filter APIs.
 *
 *          servo_axis.c 通过本头文件调用库内闭环算法，直接访问 GDservo/FWservo 全局。
 *          servo_axis.c invokes closed-loop routines and global GDservo/FWservo state.
 *
 * @note    结构体字段名保留历史命名（igive/ierror/icon 等），修改须与链接 .lib 布局一致。
 *          Field names are legacy ABI; must match linked .lib object layout.
 *
 * @par     控制环层级 / Control loop hierarchy (inner -> outer)
 *          电流 i -> 角加速度 a -> 速度 v -> 欧拉角加速度 ea -> 欧拉速度 ev -> 位置 p
 *          -> 寻北 t_1/2/3 -> GEO g
 */
#ifndef _SFLIBHEAD_H_
#define _SFLIBHEAD_H_
/** @brief 角度单位换算：度 -> 弧度 / degrees to radians */
#define degree_to_rad 0.01745329
/** @brief 角度单位换算：弧度 -> 度 / radians to degrees */
#define rad_to_degree 57.2957795
/* ========================================================================
 *  GDservoTypeDef — 高低轨（GD）伺服 PI 状态 / GD orbit servo PI state
 *  公共字段：give(给定) / error(误差) / err[](积分历史) / con[](输出历史)
 *  Each loop: setpoint, error, integral history[2], controller output history[2]
 * ====================================================================== */
typedef struct {
    /* ------------------------------------------------------------------
     *  电流环 / Current loop (innermost torque/current PI)
     *  最内环，直接产生电压驱动 ugive 供功率级
     * ---------------------------------------------------------------- */
    float igive;  /**< 电流给定(A) / current command */
    float ierror; /**< 电流误差 = igive - I_fb / current tracking error */
    float ierr[2];/**< 电流环积分器历史(2拍) / integral state history */
    float icon[2];/**< 电流环 PI 输出历史 / PI output history */
    float ugive;  /**< 电压驱动输出(V) / voltage command to power stage */
    /* ------------------------------------------------------------------
     *  角加速度环 / Angular acceleration loop
     *  含模型补偿 amodel_* 与比较器 acmp_* 支路
     * ---------------------------------------------------------------- */
    float agive;       /**< 角加速度给定(deg/s2) / angular accel setpoint */
    float aerror;      /**< 角加速度误差 / angular accel error */
    float aerr[2];     /**< 角加速度环积分历史 / integral history */
    float acon[2];     /**< 角加速度环 PI 输出历史 / PI output history */
    float amodel_in[2];  /**< 模型支路输入延迟线 / model branch delay line in */
    float amodel_out[2]; /**< 模型支路输出延迟线 / model branch delay line out */
    float acmp_in[2];    /**< 比较器支路输入延迟线 / comparator branch in */
    float acmp_out[2];   /**< 比较器支路输出延迟线 / comparator branch out */
    /* ------------------------------------------------------------------
     *  速度环 / Velocity loop
     *  scanvgive 为扫描速度给定叠加量
     * ---------------------------------------------------------------- */
    float vgive;     /**< 速度给定(deg/s) / velocity setpoint */
    float scanvgive; /**< 扫描模式速度给定叠加 / scan-mode velocity overlay */
    float verror;    /**< 速度误差 / velocity error */
    float verr[2];   /**< 速度环积分历史 / integral history */
    float vcon[2];   /**< 速度环 PI 输出历史 / PI output history */
    /* ------------------------------------------------------------------
     *  欧拉角加速度环 / Euler angular acceleration loop
     *  处理欧拉角加速度给定
     * ---------------------------------------------------------------- */
    float eagive;  /**< 欧拉角加速度给定 / euler angular accel setpoint */
    float eaerror; /**< 欧拉角加速度误差 / euler angular accel error */
    float eaerr[2];/**< 欧拉角加速度环积分历史 / integral history */
    float eacon[2];/**< 欧拉角加速度环 PI 输出历史 / PI output history */
    /* ------------------------------------------------------------------
     *  欧拉速度环 / Euler velocity loop
     *  欧拉轴速度外环，支持 deadzone_flag
     * ---------------------------------------------------------------- */
    float evgive;  /**< 欧拉速度给定(deg/s) / euler velocity setpoint */
    float everror; /**< 欧拉速度误差 / euler velocity error */
    float everr[2];/**< 欧拉速度环积分历史 / integral history */
    float evcon[2];/**< 欧拉速度环 PI 输出历史 / PI output history */
    /* ------------------------------------------------------------------
     *  位置环 / Position loop
     *  姿态角位置外环，输出限幅由 ev_limit 约束
     * ---------------------------------------------------------------- */
    float pgive;  /**< 位置(角度)给定(deg) / position setpoint */
    float perror; /**< 位置误差 / position error */
    float perr[2];/**< 位置环积分历史 / integral history */
    float pcon[2];/**< 位置环 PI 输出历史 / PI output history */
    /* ------------------------------------------------------------------
     *  寻北环 TV1 / Seek-north loop channel 1 (turntable rate loop)
     *  第一路寻北/转台速率环，含比较器 tcmp_* 支路
     * ---------------------------------------------------------------- */
    float terror_1;    /**< 寻北环1 速率误差 / seek-north ch1 rate error */
    float terr_1[2];   /**< 寻北环1 积分历史 / integral history */
    float tcon_1[2];   /**< 寻北环1 PI 输出历史 / PI output history */
    float tcmp_in_1[2];  /**< 寻北环1 比较器输入延迟 / comparator in */
    float tcmp_out_1[2]; /**< 寻北环1 比较器输出延迟 / comparator out */
    /* ------------------------------------------------------------------
     *  寻北环 TV2 / Seek-north loop channel 2
     * ---------------------------------------------------------------- */
    float terror_2;    /**< 寻北环2 速率误差 / seek-north ch2 rate error */
    float terr_2[2];   /**< 寻北环2 积分历史 / integral history */
    float tcon_2[2];   /**< 寻北环2 PI 输出历史 / PI output history */
    float tcmp_in_2[2];  /**< 寻北环2 比较器输入延迟 / comparator in */
    float tcmp_out_2[2]; /**< 寻北环2 比较器输出延迟 / comparator out */
    /* ------------------------------------------------------------------
     *  寻北环 TV3 / Seek-north loop channel 3
     * ---------------------------------------------------------------- */
    float terror_3;    /**< 寻北环3 速率误差 / seek-north ch3 rate error */
    float terr_3[2];   /**< 寻北环3 积分历史 / integral history */
    float tcon_3[2];   /**< 寻北环3 PI 输出历史 / PI output history */
    float tcmp_in_3[2];  /**< 寻北环3 比较器输入延迟 / comparator in */
    float tcmp_out_3[2]; /**< 寻北环3 比较器输出延迟 / comparator out */
    /* ------------------------------------------------------------------
     *  GEO 环 / GEO pointing loop
     *  地理指向角速率外环，含 gcmp_* 比较器支路
     * ---------------------------------------------------------------- */
    float ggive;      /**< GEO 角给定(deg) / GEO angle setpoint */
    float gerror;     /**< GEO 角误差 / GEO angle error */
    float gerr[2];    /**< GEO 环积分历史 / integral history */
    float gcon[2];    /**< GEO 环 PI 输出历史 / PI output history */
    float gcmp_in[2]; /**< GEO 比较器输入延迟 / comparator in */
    float gcmp_out[2];/**< GEO 比较器输出延迟 / comparator out */
    /* ------------------------------------------------------------------
     *  PI 离散化系数与限幅 / Discrete PI coefficients and saturations
     *  *_e_cof[2]=误差支路系数, *_u_cof[2]=输出支路系数 (双线性/梯形欧拉)
     * ---------------------------------------------------------------- */
    float icon_e_cof[2];  /**< 电流环误差支路离散系数 / current loop error coeffs */
    float icon_u_cof[2];  /**< 电流环输出支路离散系数 / current loop output coeffs */
    float U_limit;        /**< 电压输出限幅(V) / voltage saturation limit */
    float acon_e_cof[2];    /**< 角加速度环误差系数 / ang-accel error coeffs */
    float acon_u_cof[2];    /**< 角加速度环输出系数 / ang-accel output coeffs */
    float amodel_e_cof[2];  /**< 角加速度模型支路误差系数 / model branch error coeffs */
    float amodel_u_cof[2];  /**< 角加速度模型支路输出系数 / model branch output coeffs */
    float acmp_e_cof[2];    /**< 角加速度比较器误差系数 / comparator error coeffs */
    float acmp_u_cof[2];    /**< 角加速度比较器输出系数 / comparator output coeffs */
    float i_limit;          /**< 电流给定限幅(A) / current command limit */
    float vcon_e_cof[2]; /**< 速度环误差系数 / velocity error coeffs */
    float vcon_u_cof[2]; /**< 速度环输出系数 / velocity output coeffs */
    float a_limit;       /**< 角加速度输出限幅(deg/s2) / ang-accel limit */
    float eacon_e_cof[2]; /**< 欧拉角加速度环误差系数 / euler-ang-accel error coeffs */
    float eacon_u_cof[2]; /**< 欧拉角加速度环输出系数 / euler-ang-accel output coeffs */
    float evcon_e_cof[2]; /**< 欧拉速度环误差系数 / euler-velocity error coeffs */
    float evcon_u_cof[2]; /**< 欧拉速度环输出系数 / euler-velocity output coeffs */
    float ea_limit;       /**< 欧拉角加速度限幅 / euler ang-accel limit */
    float pcon_e_cof[2]; /**< 位置环误差系数 / position error coeffs */
    float pcon_u_cof[2]; /**< 位置环输出系数 / position output coeffs */
    float ev_limit;      /**< 欧拉速度输出限幅(deg/s) / euler velocity limit */
    float tcon_e_cof_1[2]; /**< 寻北环1 误差系数 / seek-north ch1 error coeffs */
    float tcon_u_cof_1[2]; /**< 寻北环1 输出系数 / seek-north ch1 output coeffs */
    float tcon_e_cof_2[2]; /**< 寻北环2 误差系数 / seek-north ch2 error coeffs */
    float tcon_u_cof_2[2]; /**< 寻北环2 输出系数 / seek-north ch2 output coeffs */
    float tbl_linear_set;  /**< 寻北转台线性区设定 / turntable linear region setpoint */
    float v_limit;         /**< 速度输出限幅(deg/s) / velocity limit */
    float gcon_e_cof[2];    /**< GEO 环误差系数 / GEO error coeffs */
    float gcon_u_cof[2];    /**< GEO 环输出系数 / GEO output coeffs */
    float gerr_linear_set;  /**< GEO 误差线性化设定 / GEO error linearization setpoint */
    float gcmp_e_cof[2];    /**< GEO 比较器误差系数 / GEO comparator error coeffs */
    float gcmp_u_cof[2];    /**< GEO 比较器输出系数 / GEO comparator output coeffs */
} GDservoTypeDef;
/** @brief 高低轨伺服全局 PI 状态实例 / GD orbit servo global state */
extern GDservoTypeDef GDservo;
/* ========================================================================
 *  FWservoTypeDef — 方位（FW）伺服 PI 状态 / Azimuth (FW) servo PI state
 *  结构与 GD 类似，额外含 T_disturb 扰动观测与 v_max 速度上限
 *  Same loop layout as GD; adds disturbance torque and v_max cap
 * ====================================================================== */
typedef struct {
    /* ------------------------------------------------------------------
     *  电流环 / Current loop
     * ---------------------------------------------------------------- */
    float igive;  /**< 电流给定(A) / current command */
    float ierror; /**< 电流误差 / current error */
    float ierr[2];/**< 电流环积分历史 / integral history */
    float icon[2];/**< 电流环 PI 输出历史 / PI output history */
    float ugive;  /**< 电压驱动输出(V) / voltage command */
    /* ------------------------------------------------------------------
     *  角加速度环 / Angular acceleration loop
     * ---------------------------------------------------------------- */
    float agive;       /**< 角加速度给定(deg/s2) / angular accel setpoint */
    float aerror;      /**< 角加速度误差 / angular accel error */
    float aerr[2];     /**< 角加速度环积分历史 / integral history */
    float acon[2];     /**< 角加速度环 PI 输出历史 / PI output history */
    float amodel_in[2];  /**< 模型支路输入延迟线 / model branch in */
    float amodel_out[2]; /**< 模型支路输出延迟线 / model branch out */
    float acmp_in[2];    /**< 比较器支路输入延迟线 / comparator branch in */
    float acmp_out[2];   /**< 比较器支路输出延迟线 / comparator branch out */
    /* ------------------------------------------------------------------
     *  速度环 / Velocity loop
     * ---------------------------------------------------------------- */
    float vgive;     /**< 速度给定(deg/s) / velocity setpoint */
    float scanvgive; /**< 扫描速度给定叠加 / scan velocity overlay */
    float verror;    /**< 速度误差 / velocity error */
    float verr[2];   /**< 速度环积分历史 / integral history */
    float vcon[2];   /**< 速度环 PI 输出历史 / PI output history */
    /* ------------------------------------------------------------------
     *  欧拉角加速度环 / Euler angular acceleration loop
     * ---------------------------------------------------------------- */
    float eagive;  /**< 欧拉角加速度给定 / euler angular accel setpoint */
    float eaerror; /**< 欧拉角加速度误差 / euler angular accel error */
    float eaerr[2];/**< 欧拉角加速度环积分历史 / integral history */
    float eacon[2];/**< 欧拉角加速度环 PI 输出历史 / PI output history */
    /* ------------------------------------------------------------------
     *  欧拉速度环 / Euler velocity loop
     * ---------------------------------------------------------------- */
    float evgive;  /**< 欧拉速度给定(deg/s) / euler velocity setpoint */
    float everror; /**< 欧拉速度误差 / euler velocity error */
    float everr[2];/**< 欧拉速度环积分历史 / integral history */
    float evcon[2];/**< 欧拉速度环 PI 输出历史 / PI output history */
    /* ------------------------------------------------------------------
     *  位置环 / Position loop
     * ---------------------------------------------------------------- */
    float pgive;  /**< 位置(角度)给定(deg) / position setpoint */
    float perror; /**< 位置误差 / position error */
    float perr[2];/**< 位置环积分历史 / integral history */
    float pcon[2];/**< 位置环 PI 输出历史 / PI output history */
    /* ------------------------------------------------------------------
     *  寻北环 TV1 / Seek-north loop channel 1
     * ---------------------------------------------------------------- */
    float terror_1;    /**< 寻北环1 速率误差 / seek-north ch1 rate error */
    float terr_1[2];   /**< 寻北环1 积分历史 / integral history */
    float tcon_1[2];   /**< 寻北环1 PI 输出历史 / PI output history */
    float tcmp_in_1[2];  /**< 寻北环1 比较器输入延迟 / comparator in */
    float tcmp_out_1[2]; /**< 寻北环1 比较器输出延迟 / comparator out */
    /* ------------------------------------------------------------------
     *  寻北环 TV2 / Seek-north loop channel 2
     * ---------------------------------------------------------------- */
    float terror_2;    /**< 寻北环2 速率误差 / seek-north ch2 rate error */
    float terr_2[2];   /**< 寻北环2 积分历史 / integral history */
    float tcon_2[2];   /**< 寻北环2 PI 输出历史 / PI output history */
    float tcmp_in_2[2];  /**< 寻北环2 比较器输入延迟 / comparator in */
    float tcmp_out_2[2]; /**< 寻北环2 比较器输出延迟 / comparator out */
    /* ------------------------------------------------------------------
     *  寻北环 TV3 / Seek-north loop channel 3
     * ---------------------------------------------------------------- */
    float terror_3;    /**< 寻北环3 速率误差 / seek-north ch3 rate error */
    float terr_3[2];   /**< 寻北环3 积分历史 / integral history */
    float tcon_3[2];   /**< 寻北环3 PI 输出历史 / PI output history */
    float tcmp_in_3[2];  /**< 寻北环3 比较器输入延迟 / comparator in */
    float tcmp_out_3[2]; /**< 寻北环3 比较器输出延迟 / comparator out */
    /* ------------------------------------------------------------------
     *  GEO 环 / GEO pointing loop
     * ---------------------------------------------------------------- */
    float ggive;      /**< GEO 角给定(deg) / GEO angle setpoint */
    float gerror;     /**< GEO 角误差 / GEO angle error */
    float gerr[2];    /**< GEO 环积分历史 / integral history */
    float gcon[2];    /**< GEO 环 PI 输出历史 / PI output history */
    float gcmp_in[2]; /**< GEO 比较器输入延迟 / comparator in */
    float gcmp_out[2];/**< GEO 比较器输出延迟 / comparator out */
    float T_disturb;  /**< 扰动力矩观测/补偿量(N*m) / disturbance torque estimate */
    /* ------------------------------------------------------------------
     *  PI 离散化系数与限幅 / Discrete PI coefficients and saturations
     * ---------------------------------------------------------------- */
    float icon_e_cof[2];  /**< 电流环误差系数 / current loop error coeffs */
    float icon_u_cof[2];  /**< 电流环输出系数 / current loop output coeffs */
    float U_limit;        /**< 电压输出限幅(V) / voltage limit */
    float acon_e_cof[2];    /**< 角加速度环误差系数 / ang-accel error coeffs */
    float acon_u_cof[2];    /**< 角加速度环输出系数 / ang-accel output coeffs */
    float amodel_e_cof[2];  /**< 角加速度模型支路误差系数 / model error coeffs */
    float amodel_u_cof[2];  /**< 角加速度模型支路输出系数 / model output coeffs */
    float acmp_e_cof[2];    /**< 角加速度比较器误差系数 / comparator error coeffs */
    float acmp_u_cof[2];    /**< 角加速度比较器输出系数 / comparator output coeffs */
    float i_limit;          /**< 电流给定限幅(A) / current limit */
    float vcon_e_cof[2]; /**< 速度环误差系数 / velocity error coeffs */
    float vcon_u_cof[2]; /**< 速度环输出系数 / velocity output coeffs */
    float a_limit;       /**< 角加速度限幅(deg/s2) / ang-accel limit */
    float eacon_e_cof[2]; /**< 欧拉角加速度环误差系数 / euler-ang-accel error coeffs */
    float eacon_u_cof[2]; /**< 欧拉角加速度环输出系数 / euler-ang-accel output coeffs */
    float evcon_e_cof[2]; /**< 欧拉速度环误差系数 / euler-velocity error coeffs */
    float evcon_u_cof[2]; /**< 欧拉速度环输出系数 / euler-velocity output coeffs */
    float ea_limit;       /**< 欧拉角加速度限幅 / euler ang-accel limit */
    float pcon_e_cof[2]; /**< 位置环误差系数 / position error coeffs */
    float pcon_u_cof[2]; /**< 位置环输出系数 / position output coeffs */
    float ev_limit;      /**< 欧拉速度限幅(deg/s) / euler velocity limit */
    float tcon_e_cof_1[2]; /**< 寻北环1 误差系数 / seek-north ch1 error coeffs */
    float tcon_u_cof_1[2]; /**< 寻北环1 输出系数 / seek-north ch1 output coeffs */
    float tcon_e_cof_2[2]; /**< 寻北环2 误差系数 / seek-north ch2 error coeffs */
    float tcon_u_cof_2[2]; /**< 寻北环2 输出系数 / seek-north ch2 output coeffs */
    float tbl_linear_set;  /**< 寻北转台线性区设定 / turntable linear setpoint */
    float v_limit;         /**< 速度输出限幅(deg/s) / velocity limit */
    float v_max;           /**< 方位轴最大允许速度(deg/s) / FW axis max velocity cap */
    float gcon_e_cof[2];    /**< GEO 环误差系数 / GEO error coeffs */
    float gcon_u_cof[2];    /**< GEO 环输出系数 / GEO output coeffs */
    float gerr_linear_set;  /**< GEO 误差线性化设定 / GEO linearization setpoint */
    float gcmp_e_cof[2];    /**< GEO 比较器误差系数 / GEO comparator error coeffs */
    float gcmp_u_cof[2];    /**< GEO 比较器输出系数 / GEO comparator output coeffs */
} FWservoTypeDef;
/** @brief 方位伺服全局 PI 状态实例 / Azimuth servo global state */
extern FWservoTypeDef FWservo;
/* ========================================================================
 *  GDDataTypeDef — 高低轨滤波器状态 / GD orbit filter delay-line state
 *  存储 IIR 滤波器输入/输出延迟线与离散系数，供 Gd* 系列函数使用
 *  Holds IIR delay lines and coeffs for Gd* filter routines
 * ====================================================================== */
typedef struct {
    /* ------------------------------------------------------------------
     *  电流 Butterworth 低通 / Current Butterworth low-pass
     *  抑制高频噪声，提高电流反馈稳定性
     * ---------------------------------------------------------------- */
    float current_in[3];  /**< 电流滤波器输入延迟线(3阶) / current filter input delay */
    float current_out[3]; /**< 电流滤波器输出延迟线 / current filter output delay */
    /* ------------------------------------------------------------------
     *  陀螺 Butterworth 低通 / Gyro Butterworth low-pass
     *  平滑角速率反馈，减少机械共振对速度环的影响
     * ---------------------------------------------------------------- */
    float gyrobutter_in[3];  /**< 陀螺 Butterworth 输入延迟 / gyro LP input delay */
    float gyrobutter_out[3]; /**< 陀螺 Butterworth 输出延迟 / gyro LP output delay */
    /* ------------------------------------------------------------------
     *  陀螺陷波 1/2/3 / Gyro notch filters (3 channels)
     *  在结构共振频率处陷波，抑制特定频率分量
     * ---------------------------------------------------------------- */
    float gyronotch_1_in[3];  /**< 陀螺陷波1 输入延迟 / notch-1 input delay */
    float gyronotch_1_out[3]; /**< 陀螺陷波1 输出延迟 / notch-1 output delay */
    float gyronotch_2_in[3];  /**< 陀螺陷波2 输入延迟 / notch-2 input delay */
    float gyronotch_2_out[3]; /**< 陀螺陷波2 输出延迟 / notch-2 output delay */
    float gyronotch_3_in[3];  /**< 陀螺陷波3 输入延迟 / notch-3 input delay */
    float gyronotch_3_out[3]; /**< 陀螺陷波3 输出延迟 / notch-3 output delay */
    /* ------------------------------------------------------------------
     *  加速度解耦 / Acceleration disturbance decoupling
     *  消除载体线加速度对陀螺角速率的耦合
     * ---------------------------------------------------------------- */
    float acc_dis_in[2];  /**< 加速度解耦输入延迟 / acc-decouple input delay */
    float acc_dis_out[2]; /**< 加速度解耦输出延迟 / acc-decouple output delay */
    /* ------------------------------------------------------------------
     *  欧拉角加速度解耦 / Euler angular acceleration decoupling
     * ---------------------------------------------------------------- */
    float eacc_dis_in[2];  /**< 欧拉角加速度解耦输入延迟 / eacc-decouple in */
    float eacc_dis_out[2]; /**< 欧拉角加速度解耦输出延迟 / eacc-decouple out */
    /* ------------------------------------------------------------------
     *  欧拉速度滤波 / Euler velocity filter
     *  对角度微分得到的欧拉速度进行低通平滑
     * ---------------------------------------------------------------- */
    float ev_in[3];  /**< 欧拉速度滤波器输入延迟 / euler-velocity filter in */
    float ev_out[3]; /**< 欧拉速度滤波器输出延迟 / euler-velocity filter out */
    /* ------------------------------------------------------------------
     *  陀螺漂移(dg)滤波 / Gyro drift (dg) filter
     *  估计并滤除陀螺零偏漂移分量
     * ---------------------------------------------------------------- */
    float dg_in[2];  /**< 漂移滤波器输入延迟 / drift filter input delay */
    float dg_out[2]; /**< 漂移滤波器输出延迟 / drift filter output delay */
    /* ------------------------------------------------------------------
     *  滤波器离散系数 / Filter discrete coefficients
     *  *_e_cof=分子, *_u_cof=分母 (IIR 直接 II 型)
     * ---------------------------------------------------------------- */
    float current_e_cof[3];     /**< 电流 Butterworth 分子系数 / current LP numerator */
    float current_u_cof[3];     /**< 电流 Butterworth 分母系数 / current LP denominator */
    float gyrobutter_e_cof[3];  /**< 陀螺 Butterworth 分子系数 / gyro LP numerator */
    float gyrobutter_u_cof[3];  /**< 陀螺 Butterworth 分母系数 / gyro LP denominator */
    float gyronotch_1_e_cof[3]; /**< 陀螺陷波1 分子系数 / notch-1 numerator */
    float gyronotch_1_u_cof[3]; /**< 陀螺陷波1 分母系数 / notch-1 denominator */
    float gyronotch_2_e_cof[3]; /**< 陀螺陷波2 分子系数 / notch-2 numerator */
    float gyronotch_2_u_cof[3]; /**< 陀螺陷波2 分母系数 / notch-2 denominator */
    float gyronotch_3_e_cof[3]; /**< 陀螺陷波3 分子系数 / notch-3 numerator */
    float gyronotch_3_u_cof[3]; /**< 陀螺陷波3 分母系数 / notch-3 denominator */
    float acc_dis_e_cof[2];  /**< 加速度解耦分子系数 / acc-decouple numerator */
    float acc_dis_u_cof[2];  /**< 加速度解耦分母系数 / acc-decouple denominator */
    float eacc_dis_e_cof[2]; /**< 欧拉角加速度解耦分子系数 / eacc-decouple numerator */
    float eacc_dis_u_cof[2]; /**< 欧拉角加速度解耦分母系数 / eacc-decouple denominator */
    float ev_e_cof[3];  /**< 欧拉速度滤波分子系数 / euler-velocity filter numerator */
    float ev_u_cof[3];  /**< 欧拉速度滤波分母系数 / euler-velocity filter denominator */
    float dg_e_cof[2];  /**< 漂移滤波分子系数 / drift filter numerator */
    float dg_u_cof[2];  /**< 漂移滤波分母系数 / drift filter denominator */
} GDDataTypeDef;
/** @brief 高低轨滤波器全局状态 / GD filter global state */
extern GDDataTypeDef GDData;
/* ========================================================================
 *  FWDataTypeDef — 方位滤波器状态 / Azimuth filter delay-line state
 *  含 FW(方位)/HG(横滚) 双通道陀螺滤波与 double 精度 ev 支路
 *  Dual FW/HG gyro paths; euler-velocity uses double precision
 * ====================================================================== */
typedef struct {
    /* ------------------------------------------------------------------
     *  电流 Butterworth 低通 / Current Butterworth low-pass
     * ---------------------------------------------------------------- */
    float current_in[3];  /**< 电流滤波器输入延迟 / current filter input delay */
    float current_out[3]; /**< 电流滤波器输出延迟 / current filter output delay */
    /* ------------------------------------------------------------------
     *  方位(FW)陀螺 Butterworth / Azimuth gyro Butterworth
     * ---------------------------------------------------------------- */
    float gyrobutter_fw_in[3];  /**< 方位陀螺 Butterworth 输入 / FW gyro LP in */
    float gyrobutter_fw_out[3]; /**< 方位陀螺 Butterworth 输出 / FW gyro LP out */
    /* ------------------------------------------------------------------
     *  方位(FW)陀螺陷波 1/2/3 / Azimuth gyro notch filters
     * ---------------------------------------------------------------- */
    float gyronotch_fw_1_in[3];  /**< 方位陷波1 输入 / FW notch-1 in */
    float gyronotch_fw_1_out[3]; /**< 方位陷波1 输出 / FW notch-1 out */
    float gyronotch_fw_2_in[3];  /**< 方位陷波2 输入 / FW notch-2 in */
    float gyronotch_fw_2_out[3]; /**< 方位陷波2 输出 / FW notch-2 out */
    float gyronotch_fw_3_in[3];  /**< 方位陷波3 输入 / FW notch-3 in */
    float gyronotch_fw_3_out[3]; /**< 方位陷波3 输出 / FW notch-3 out */
    /* ------------------------------------------------------------------
     *  横滚(HG)陀螺 Butterworth / Roll (HG) gyro Butterworth
     * ---------------------------------------------------------------- */
    float gyrobutter_hg_in[3];  /**< 横滚陀螺 Butterworth 输入 / HG gyro LP in */
    float gyrobutter_hg_out[3]; /**< 横滚陀螺 Butterworth 输出 / HG gyro LP out */
    /* ------------------------------------------------------------------
     *  横滚(HG)陀螺陷波 1/2/3 / Roll gyro notch filters
     * ---------------------------------------------------------------- */
    float gyronotch_hg_1_in[3];  /**< 横滚陷波1 输入 / HG notch-1 in */
    float gyronotch_hg_1_out[3]; /**< 横滚陷波1 输出 / HG notch-1 out */
    float gyronotch_hg_2_in[3];  /**< 横滚陷波2 输入 / HG notch-2 in */
    float gyronotch_hg_2_out[3]; /**< 横滚陷波2 输出 / HG notch-2 out */
    float gyronotch_hg_3_in[3];  /**< 横滚陷波3 输入 / HG notch-3 in */
    float gyronotch_hg_3_out[3]; /**< 横滚陷波3 输出 / HG notch-3 out */
    /* ------------------------------------------------------------------
     *  加速度解耦 FW/HG / Acceleration decoupling (FW and HG axes)
     * ---------------------------------------------------------------- */
    float acc_dis_fw_in[2];  /**< 方位加速度解耦输入 / FW acc-decouple in */
    float acc_dis_fw_out[2]; /**< 方位加速度解耦输出 / FW acc-decouple out */
    float acc_dis_hg_in[2];  /**< 横滚加速度解耦输入 / HG acc-decouple in */
    float acc_dis_hg_out[2]; /**< 横滚加速度解耦输出 / HG acc-decouple out */
    /* ------------------------------------------------------------------
     *  欧拉角加速度解耦(FW) / Euler angular acceleration decoupling
     * ---------------------------------------------------------------- */
    float eacc_dis_fw_in[2];  /**< 方位欧拉角加速度解耦输入 / FW eacc-decouple in */
    float eacc_dis_fw_out[2]; /**< 方位欧拉角加速度解耦输出 / FW eacc-decouple out */
    /* ------------------------------------------------------------------
     *  欧拉速度滤波(double) / Euler velocity filter (double precision)
     *  高精度角度微分低通，减少量化噪声
     * ---------------------------------------------------------------- */
    double ev_in[3];  /**< 欧拉速度滤波输入(double) / euler-velocity filter in */
    double ev_out[3]; /**< 欧拉速度滤波输出(double) / euler-velocity filter out */
    /* ------------------------------------------------------------------
     *  陀螺漂移(dg)滤波 / Gyro drift filter
     * ---------------------------------------------------------------- */
    float dg_in[2];  /**< 漂移滤波器输入延迟 / drift filter input delay */
    float dg_out[2]; /**< 漂移滤波器输出延迟 / drift filter output delay */
    /* ------------------------------------------------------------------
     *  滤波器离散系数 / Filter discrete coefficients
     * ---------------------------------------------------------------- */
    float current_e_cof[3];     /**< 电流 Butterworth 分子系数 / current LP numerator */
    float current_u_cof[3];     /**< 电流 Butterworth 分母系数 / current LP denominator */
    float gyrobutter_e_cof[3];  /**< 陀螺 Butterworth 分子系数 / gyro LP numerator */
    float gyrobutter_u_cof[3];  /**< 陀螺 Butterworth 分母系数 / gyro LP denominator */
    float gyronotch_1_e_cof[3]; /**< 陷波1 分子系数 / notch-1 numerator */
    float gyronotch_1_u_cof[3]; /**< 陷波1 分母系数 / notch-1 denominator */
    float gyronotch_2_e_cof[3]; /**< 陷波2 分子系数 / notch-2 numerator */
    float gyronotch_2_u_cof[3]; /**< 陷波2 分母系数 / notch-2 denominator */
    float gyronotch_3_e_cof[3]; /**< 陷波3 分子系数 / notch-3 numerator */
    float gyronotch_3_u_cof[3]; /**< 陷波3 分母系数 / notch-3 denominator */
    float acc_dis_e_cof[2];  /**< 加速度解耦分子系数 / acc-decouple numerator */
    float acc_dis_u_cof[2];  /**< 加速度解耦分母系数 / acc-decouple denominator */
    float eacc_dis_e_cof[2]; /**< 欧拉角加速度解耦分子系数 / eacc-decouple numerator */
    float eacc_dis_u_cof[2]; /**< 欧拉角加速度解耦分母系数 / eacc-decouple denominator */
    float ev_e_cof[3];  /**< 欧拉速度滤波分子系数 / euler-velocity numerator */
    float ev_u_cof[3];  /**< 欧拉速度滤波分母系数 / euler-velocity denominator */
    float dg_e_cof[2];  /**< 漂移滤波分子系数 / drift filter numerator */
    float dg_u_cof[2];  /**< 漂移滤波分母系数 / drift filter denominator */
} FWDataTypeDef;
/** @brief 方位滤波器全局状态 / Azimuth filter global state */
extern FWDataTypeDef FWData;
/* ========================================================================
 *  高低轨（GD）控制算法 / GD orbit control loop algorithms
 *  各函数执行单步 PI 运算并更新 GDservo 对应环状态
 *  Each routine runs one PI step and updates GDservo loop state
 * ====================================================================== */
/**
 * @brief  高低轨电流环 PI / GD current-loop PI
 * @param  Kp,Ki   比例/积分增益 / proportional & integral gains
 * @param  SamT    采样周期(s) / sample period
 * @param  Umax    电压限幅(V) / voltage saturation
 * @param  I_give  电流给定(A) / current setpoint
 * @param  I_fb    电流反馈(A) / current feedback
 * @return 电压驱动输出(V) / voltage command
 */
float gdiloop(float Kp, float Ki, float SamT, float Umax, float I_give, float I_fb);
/**
 * @brief  高低轨角加速度环 PI / GD angular-acceleration loop PI
 * @note   Kg 为电流-角加速度增益(deg/s2/A)，BW 为模型带宽
 *         Kg maps motor current to angular accel; BW sets model bandwidth
 * @param  Imax    电流输出限幅(A) / current limit
 * @param  A_give  角加速度给定 / ang-accel setpoint
 * @param  A_fb    角加速度反馈 / ang-accel feedback
 */
float gdaloop(float Kp, float Ki, float SamT, float Kg, float BW, float Imax, float A_give, float A_fb);
/**
 * @brief  高低轨速度环 PI / GD velocity loop PI
 * @param  Amax       角加速度限幅 / ang-accel limit
 * @param  Up_Limit   速度上限限幅 / velocity upper bound
 * @param  Dwn_Limit  速度下限限幅 / velocity lower bound
 * @param  V_give     速度给定(deg/s) / velocity setpoint
 * @param  V_fb       速度反馈(deg/s) / velocity feedback
 * @param  gd_angle   当前高低轨角(deg) / current GD gimbal angle
 */
float gdvloop(float Kp, float Ki, float SamT, float Amax, float Up_Limit, float Dwn_Limit, float V_give,
              float V_fb, float gd_angle);
/**
 * @brief  高低轨欧拉角加速度环 PI / GD euler angular-acceleration loop PI
 * @param  Imax           电流限幅 / current limit
 * @param  eA_give        欧拉角加速度给定 / setpoint
 * @param  eA_fb          欧拉角加速度反馈 / feedback
 * @param  deadzone_flag  死区使能标志 / dead-zone enable flag
 */
float gdealoop(float Kp, float Ki, float SamT, float Imax, float eA_give, float eA_fb, char deadzone_flag);
/**
 * @brief  高低轨欧拉速度环 PI / GD euler-velocity loop PI
 * @param  Eamax          欧拉角加速度限幅 / euler ang-accel limit
 * @param  Ev_give        欧拉速度给定 / euler velocity setpoint
 * @param  Ev_fb          欧拉速度反馈 / euler velocity feedback
 * @param  gd_angle       高低轨角(deg) / GD angle for cross-coupling
 * @param  deadzone_flag  死区使能 / dead-zone flag
 */
float gdevloop(float Kp, float Ki, float SamT, float Eamax, float Up_Limit, float Dwn_Limit, float Ev_give,
               float Ev_fb, float gd_angle, char deadzone_flag);
/**
 * @brief  高低轨位置环 PI / GD position loop PI
 * @param  bound          高低轨切换边界(deg) / gimbal switch boundary
 * @param  Ev_Set         固定平移速度(deg/s) / fixed slew rate
 * @param  Evmax          最大欧拉速度 / max euler velocity
 * @param  P_give         位置给定(deg) / position setpoint
 * @param  P_fb           位置反馈(deg) / position feedback
 * @param  deadzone_flag  死区使能 / dead-zone flag
 */
float gdploop(float Kp, float Ki, float SamT, float Up_Limit, float Dwn_Limit, float bound, float Ev_Set,
              float Evmax, float P_give, float P_fb, char deadzone_flag);
/**
 * @brief  高低轨寻北环 TV1 PI / GD seek-north loop channel 1
 * @note   bound 判定大寻北/小角/线性区切换，不同 PI 增益与寻北策略
 *         bound selects seek-north region and switches PI gains accordingly
 * @param  Kp_1,Ki_1      大角区 PI 增益 / large-angle PI gains
 * @param  Kp_2,Ki_2      小角/粗寻北区 PI 增益 / fine/coarse PI gains
 * @param  Vmax           最大寻北速率(deg/s) / max seek rate
 * @param  Tbl_angle_fb   寻北转台角反馈(deg) / turntable angle feedback
 */
float gdtloop_TV1(float Kp_1, float Ki_1, float bound, float Kp_2, float Ki_2, float SamT, float Vmax,
                  float Tbl_angle_fb);
/** @brief  高低轨寻北环 TV2 PI / GD seek-north loop channel 2 (same as TV1) */
float gdtloop_TV2(float Kp_1, float Ki_1, float bound, float Kp_2, float Ki_2, float SamT, float Vmax,
                  float Tbl_angle_fb);
/** @brief  高低轨寻北环 TV3 PI / GD seek-north loop channel 3 (same as TV1) */
float gdtloop_TV3(float Kp_1, float Ki_1, float bound, float Kp_2, float Ki_2, float SamT, float Vmax,
                  float Tbl_angle_fb);
/**
 * @brief  高低轨 GEO 指向 PI / GD GEO pointing loop PI
 * @param  V_Set    固定平移速度(deg/s) / fixed slew rate
 * @param  Vmax     最大速率 / max rate
 * @param  GEO_give GEO 角给定(deg) / GEO angle setpoint
 * @param  GEO_fb   GEO 角反馈(deg) / GEO angle feedback
 */
float gdgeo(float Kp, float Ki, float SamT, float bound, float V_Set, float Vmax, float GEO_give,
            float GEO_fb);
/* ========================================================================
 *  方位（FW）控制算法 / Azimuth (FW) control loop algorithms
 *  部分环给定数组 Kp/Ki 支持分段增益；Sec/CSC 为扇区/余割补偿角
 *  Pointer-array Kp/Ki for piecewise gains; Sec/CSC are sector angles
 * ====================================================================== */
/**
 * @brief  方位电流环 PI / FW current-loop PI
 * @param  Umax   电压限幅(V) / voltage limit
 * @param  I_give 电流给定(A) / current setpoint
 * @param  I_fb   电流反馈(A) / current feedback
 */
float fwiloop(float Kp, float Ki, float SamT, float Umax, float I_give, float I_fb);
/**
 * @brief  方位角加速度环 PI / FW angular-acceleration loop PI
 * @param  Kp,Ki    PI 增益指针数组(分段) / piecewise gain arrays
 * @param  bound    高低轨角边界指针 / gimbal boundary array
 * @param  gd_angle 当前高低轨角(deg) / GD angle for gain scheduling
 */
float fwaloop(float *Kp, float *Ki, float SamT, float Kg, float *BW, float Imax, float A_give, float A_fb,
              float gd_angle, float *bound);
/**
 * @brief  方位速度环 PI / FW velocity loop PI
 * @param  L_Limit,R_limit  左右速度限幅 / left & right velocity limits
 * @param  fw_angle         方位角(deg) / azimuth angle
 * @param  gd_angle         高低轨角(deg) / GD angle for cross-coupling
 */
float fwvloop(float *Kp, float *Ki, float SamT, float Amax, float L_Limit, float R_limit, float V_give,
              float V_fb, float fw_angle, float gd_angle, float *bound);
/**
 * @brief  方位欧拉角加速度环 PI / FW euler angular-acceleration loop PI
 * @param  Eamax  欧拉角加速度限幅 / euler ang-accel limit
 * @param  eA_give,eA_fb  给定与反馈 / setpoint & feedback
 */
float fwealoop(float Kp, float Ki, float SamT, float Eamax, float eA_give, float eA_fb, char deadzone_flag);
/**
 * @brief  方位欧拉速度环 PI / FW euler-velocity loop PI
 * @param  Imax     电流限幅 / current limit
 * @param  fw_angle 方位角(deg) / azimuth angle
 */
float fwevloop(float Kp, float Ki, float SamT, float Imax, float L_Limit, float R_limit, float Ev_give,
               float Ev_fb, float fw_angle, char deadzone_flag);
/**
 * @brief  方位位置环 PI / FW position loop PI
 * @param  L_Limit,R_Limit  左右位置限幅 / position limits
 * @param  bound            切换边界(deg) / switch boundary
 * @param  Ev_Set,Evmax     平移速度与上限 / slew rate and max
 */
float fwploop(float Kp, float Ki, float SamT, float L_Limit, float R_Limit, float bound, float Ev_Set,
              float Evmax, float P_give, float P_fb, char deadzone_flag);
/**
 * @brief  方位寻北环 TV1 PI / FW seek-north loop channel 1
 * @param  Sec  扇区角补偿(deg) / sector angle compensation
 */
float fwtloop_TV1(float Kp_1, float Ki_1, float bound, float Kp_2, float Ki_2, float SamT, float Vmax,
                  float Tbl_angle_fb, float Sec);
/** @brief  方位寻北环 TV2 PI / FW seek-north loop channel 2 */
float fwtloop_TV2(float Kp_1, float Ki_1, float bound, float Kp_2, float Ki_2, float SamT, float Vmax,
                  float Tbl_angle_fb, float Sec);
/** @brief  方位寻北环 TV3 PI / FW seek-north loop channel 3 */
float fwtloop_TV3(float Kp_1, float Ki_1, float bound, float Kp_2, float Ki_2, float SamT, float Vmax,
                  float Tbl_angle_fb, float Sec);
/**
 * @brief  方位 GEO 指向 PI / FW GEO pointing loop PI
 * @param  Sec  扇区角补偿(deg) / sector compensation angle
 */
float fwgeo(float Kp, float Ki, float SamT, float bound, float V_Set, float Vmax, float GEO_give,
            float GEO_fb, float Sec);
/* ========================================================================
 *  环状态清零 clr_* / Loop state reset (anti-windup on mode switch)
 *  切换工作模式或重新使能伺服时调用，防止积分饱和
 *  Call on mode change or re-enable to prevent integrator windup
 * ====================================================================== */
/** @defgroup clr_gd GD 高低轨环清零 / GD loop clear functions
 *  @{ */
void clr_gd_i(void);    /**< 清零电流环状态 / clear current loop */
void clr_gd_a(void);    /**< 清零角加速度环状态 / clear ang-accel loop */
void clr_gd_v(void);    /**< 清零速度环状态 / clear velocity loop */
void clr_gd_ea(void);   /**< 清零欧拉角加速度环状态 / clear euler-ang-accel loop */
void clr_gd_ev(void);   /**< 清零欧拉速度环状态 / clear euler-velocity loop */
void clr_gd_p(void);    /**< 清零位置环状态 / clear position loop */
void clr_gd_t_1(void);  /**< 清零寻北环1 状态 / clear seek-north ch1 */
void clr_gd_t_2(void);  /**< 清零寻北环2 状态 / clear seek-north ch2 */
void clr_gd_t_3(void);  /**< 清零寻北环3 状态 / clear seek-north ch3 */
void clr_gd_g(void);    /**< 清零 GEO 环状态 / clear GEO loop */
void clr_gd_all(void);  /**< 清零 GD 全部环状态 / clear all GD loops */
/** @} */
/** @defgroup clr_fw FW 方位环清零 / FW loop clear functions
 *  @{ */
void clr_fw_i(void);    /**< 清零电流环状态 / clear current loop */
void clr_fw_a(void);    /**< 清零角加速度环状态 / clear ang-accel loop */
void clr_fw_v(void);    /**< 清零速度环状态 / clear velocity loop */
void clr_fw_ea(void);   /**< 清零欧拉角加速度环状态 / clear euler-ang-accel loop */
void clr_fw_ev(void);   /**< 清零欧拉速度环状态 / clear euler-velocity loop */
void clr_fw_p(void);    /**< 清零位置环状态 / clear position loop */
void clr_fw_t_1(void);  /**< 清零寻北环1 状态 / clear seek-north ch1 */
void clr_fw_t_2(void);  /**< 清零寻北环2 状态 / clear seek-north ch2 */
void clr_fw_t_3(void);  /**< 清零寻北环3 状态 / clear seek-north ch3 */
void clr_fw_g(void);    /**< 清零 GEO 环状态 / clear GEO loop */
void clr_fw_all(void);  /**< 清零 FW 全部环状态 / clear all FW loops */
/** @} */
/** @brief  清零 GD 与 FW 全部伺服环状态 / Clear all GD and FW servo loops */
void clr_all(void);
/* ========================================================================
 *  滤波算法 / Filter processing routines
 *  IIR Butterworth 低通、陷波、解耦与漂移估计，更新 GDData/FWData 延迟线
 *  IIR Butterworth LP, notch, decoupling, drift; updates delay-line state
 * ====================================================================== */
/** @defgroup filter_gd GD 高低轨滤波 / GD orbit filters
 *  @{ */
/**
 * @brief  高低轨电流 Butterworth 低通滤波
 *         GD current Butterworth low-pass — anti-alias & noise rejection
 * @param  order   滤波器阶数 / filter order
 * @param  BW      截止带宽(Hz) / cutoff bandwidth
 * @param  SamT    采样周期(s) / sample period
 * @param  Current 原始电流采样(A) / raw current sample
 * @return 滤波后电流(A) / filtered current
 */
float GdCurrentButter(int order, int BW, float SamT, float Current);
/**
 * @brief  高低轨陀螺 Butterworth 低通滤波
 *         GD gyro Butterworth LP — smooth rate feedback for velocity loop
 * @param  Gyro  原始陀螺角速率(deg/s) / raw gyro rate
 * @return 滤波后角速率 / filtered rate
 */
float GdGyroButter(int order, float BW, float SamT, float Gyro);
/**
 * @brief  高低轨陀螺陷波滤波器 1/2/3
 *         GD gyro notch — suppress structural resonance at freq
 * @param  freq  陷波中心频率(Hz) / notch center frequency
 * @param  Gyro  原始陀螺输入 / raw gyro input
 */
float GdGyroNotch_1(float freq, float SamT, float Gyro);
float GdGyroNotch_2(float freq, float SamT, float Gyro);
float GdGyroNotch_3(float freq, float SamT, float Gyro);
/**
 * @brief  高低轨加速度解耦滤波
 *         GD acceleration decoupling — remove linear accel coupling from gyro
 * @param  Gyro  陀螺角速率输入 / gyro rate input
 */
float GdAccDis(float BW, float SamT, float Gyro);
/**
 * @brief  高低轨欧拉速度解耦滤波
 *         GD euler-velocity axis decoupling filter
 * @param  angle  角度输入(deg) / angle input
 */
float GdAxisEvDis(int order, float BW, float SamT, float angle);
/**
 * @brief  高低轨欧拉角加速度解耦滤波
 *         GD euler angular-acceleration decoupling
 * @param  Gd_ev  欧拉速度输入 / euler velocity input
 */
float GdeAccDis(float BW, float SamT, float Gd_ev);
/**
 * @brief  高低轨陀螺漂移(dg)滤波
 *         GD gyro drift estimation — low-pass bias tracking
 * @param  dg  漂移原始输入 / raw drift input
 */
float GddgDis(float SamT, float dg);
/** @} */
/** @defgroup filter_fw FW 方位滤波 / Azimuth (FW) filters
 *  @{ */
/** @brief  方位电流 Butterworth 低通 / FW current Butterworth LP */
float FwCurrentButter(int order, int BW, float SamT, float Current);
/** @brief  方位陀螺陷波 1/2/3 / FW gyro notch filters at structural freq */
float FwGyroNotch_1(float freq, float SamT, float Gyro);
float FwGyroNotch_2(float freq, float SamT, float Gyro);
float FwGyroNotch_3(float freq, float SamT, float Gyro);
/**
 * @brief  方位陀螺 Butterworth 低通（含扇区角 Sec）
 *         FW gyro Butterworth LP with sector angle Sec compensation
 */
float FwGyroButter(int order, float BW, float SamT, float Gyro, float Sec);
/** @defgroup filter_hg HG 横滚滤波 / Roll (HG) filters
 *  @{ */
/** @brief  横滚陀螺陷波 1/2/3 / HG gyro notch filters */
float HgGyroNotch_1(float freq, float SamT, float Gyro);
float HgGyroNotch_2(float freq, float SamT, float Gyro);
float HgGyroNotch_3(float freq, float SamT, float Gyro);
/**
 * @brief  横滚陀螺 Butterworth 低通（含 CSC 补偿角）
 *         HG gyro Butterworth LP with CSC compensation angle
 */
float HgGyroButter(int order, float BW, float SamT, float Gyro, float CSC);
/** @} */
/**
 * @brief  方位/横滚加速度解耦滤波（含扇区/余割补偿）
 *         FW/HG acceleration decoupling with Sec/CSC compensation
 */
float FwAccDis(float BW, float SamT, float Gyro, float Sec);
float HgAccDis(float BW, float SamT, float Gyro, float CSC);
/**
 * @brief  方位欧拉速度解耦滤波(double 精度)
 *         FW euler-velocity decoupling filter (double precision)
 */
double FwAxisEvDis(int order, float BW, float SamT, double angle);
/** @brief  方位欧拉角加速度解耦 / FW euler angular-acceleration decoupling */
float FweAccDis(float BW, float SamT, float Fw_ev);
/** @brief  方位陀螺漂移滤波 / FW gyro drift filter */
float FwdgDis(float SamT, float dg);
/** @} */
/** @defgroup clr_dg 漂移滤波状态清零 / Drift filter state clear
 *  @{ */
void clr_gddg(void);  /**< 清零 GD 漂移滤波延迟线 / clear GD drift filter state */
void clr_fwdg(void);  /**< 清零 FW 漂移滤波延迟线 / clear FW drift filter state */
void clr_dg(void);    /**< 清零全部漂移滤波状态 / clear all drift filter state */
/** @} */
#endif /* _SFLIBHEAD_H_ */
