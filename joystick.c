#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include "hal_uart.h"
#include "hal_timer.h"
#include "hal_motor.h"
#include "pid.h"
#include "protocol.h"
#include "motorctrl.h"
#include "motor_curve.h"
#include "joystick.h"
#include "cmd_shell.h"
// ===================================================================
// 【按键/移动信号枚举】
// ===================================================================
typedef enum {
    BUTTON_CENTER = 0x01,
    BUTTON_RIGHT  = 0x02,
    BUTTON_UP     = 0x04,
    BUTTON_LEFT   = 0x08,
    BUTTON_DOWN   = 0x10,
    MOVE_DOWN     = 0x20,
    MOVE_UP       = 0x40,
    MOVE_LEFT     = 0x80,
    MOVE_RIGHT    = 0x100,
} joystick_signal_e;

// ===================================================================
// 【全局控制权令牌】
// ===================================================================
#define MOTOR_OWNER_IDLE     0  // 空闲态
#define MOTOR_OWNER_JOYSTICK 1  // 摇杆/按键控制中
#define MOTOR_OWNER_HOST     2  // 上位机控制中

volatile uint8_t g_motor_owner = MOTOR_OWNER_IDLE;
// 摇杆私有日志开关: 1 为开启打印，0 为静默 (默认关闭防止刷屏)
static uint8_t s_log_joy_enabled = 0;

// 前置槽函数声明
int  joystick_shell_cmd(int argc, char *argv[]);
void joystick_shell_help(void);
void Host_Claim_Motor_Control(void) {
    g_motor_owner = MOTOR_OWNER_HOST;
}

void Host_Release_Motor_Control(void) {
    if (g_motor_owner == MOTOR_OWNER_HOST) {
        g_motor_owner = MOTOR_OWNER_IDLE;
    }
}

extern void System_Abort_All_Tasks_And_Stop(void);
extern struct tcp_pcb *g_current_tpcb;
extern uint8_t Is_Center_Back_Running(void);
// ===================================================================
// 【核心参数配置区】
// ===================================================================
#define MOTOR_STEPS_PER_MM        800.0f
#define TOTAL_STROKE_MM_X         22.0f
#define TOTAL_STROKE_MM_Y         15.0f

#define JOY_RANGE_SCALE_FACTOR    0.5f
#define JOY_START_TRIGGER_THRESH  10
#define SERVO_DEADBAND_STEPS      5.0

#define MAX_RUN_SPEED_XY          24000
#define MAX_SPEED_Z               30000
#define SLOP_Z                    1000

#define BTN_START_SPEED           500.0f
#define BTN_ACCEL_STEP            150.0f
#define M2_SAFE_MAX_FREQ          24000

#define XY_ACCEL_STEP             1500
// 减速步进：控制刹车平滑度。若想更柔和可调为1500，若想更急可调为3000
#define XY_DECEL_STEP             2000
#define XY_MIN_RUN_SPEED          500

#define CLAMP(x, min_val, max_val) ((x) < (min_val) ? (min_val) : ((x) > (max_val) ? (max_val) : (x)))
#define BUFFER_SIZE               20

#ifdef MOTOR_COMPATIBLE
    #define JOY_MOTOR_M1 TMC2660_M2  // 电机 2 (X 轴)
    #define JOY_MOTOR_M2 TMC2660_M1  // 电机 1 (Y 轴)
#else
    #define JOY_MOTOR_M1 TMC2660_M2  // 电机 2 (X 轴)
    #define JOY_MOTOR_M2 TMC2660_M1  // 电机 1 (Y 轴)
#endif

typedef struct {
    int buffer[BUFFER_SIZE];
    int index;
    int count;
    int ready;
} JoystickFilter;

typedef struct {
    int16_t z_value;
    uint64_t z_time;
} z_move_info_t;

// ===================================================================
// 【文件作用域控制寄存器与状态变量】
// ===================================================================
uint8_t g_joystick_active_flag = 0;
uint8_t m_joystick_mode = 20;
uint8_t m_joystick_ctrl = 0;

joystick_state_t g_current_joystick_data = {0};
uint8_t g_joystick_data_updated = 0;
uint64_t g_last_joystick_pack_time = 0;
void joystick_process_packet(uint8_t *data, uint32_t len);
static joystick_state_t last_state = {0};
static int first_sample = 1;

static JoystickFilter jf_x;
static JoystickFilter jf_y;

static double s_integrated_tgt_x = 0.0;
static double s_integrated_tgt_y = 0.0;
static double s_prev_enc_x = 0.0;
static double s_prev_enc_y = 0.0;
static uint8_t s_inited = 0;

static uint8_t s_is_moving_active = 0;
static uint8_t s_stop_hold_cnt = 0;

static uint32_t s_curr_speed_x = 0;
static uint32_t s_curr_speed_y = 0;
static uint32_t current_z_speed = 0;

static float s_btn_ramp_speed_x = 0.0f;
static float s_btn_ramp_speed_y = 0.0f;
static uint64_t s_last_log_time = 0;

static motor_dir_e last_x_dir = MOTOR_DIR_NONE;
static motor_dir_e last_y_dir = MOTOR_DIR_NONE;
static motor_dir_e last_z_dir = MOTOR_DIR_NONE;
static z_move_info_t last_move_z = {0, 0};

extern uint8_t g_motor2_speed_param;

// ===================================================================
// 滤波器与辅助控制函数
// ===================================================================
void joystick_filter_init(JoystickFilter *jf) {
    jf->index = 0; jf->count = 0; jf->ready = 0;
}

void joystick_filter_add(JoystickFilter *jf, int y_value) {
    jf->buffer[jf->index] = y_value;
    jf->index = (jf->index + 1) % BUFFER_SIZE;
    if (jf->count < BUFFER_SIZE) jf->count++;
    if (jf->count == BUFFER_SIZE) jf->ready = 1;
}

void joystick_init(void) {
    uart_init();
    register_uart_packet_handler(joystick_process_packet);
    joystick_filter_init(&jf_x);
    joystick_filter_init(&jf_y);
    cmd_shell_register_ext("JOYSTICK", joystick_shell_cmd, joystick_shell_help);
}

void joystick_process(void) {
    protocol_unpack();
}

void move_step_check(int* step) {
#define MOVE_STEP_MAX 4200
#define MOVE_STEP_MIN 4000
    (*step)++;
    if (*step > MOVE_STEP_MAX) *step = MOVE_STEP_MIN;
}

void joystick_motor_ctrl(MOTOR_ADDR addr, motor_dir_e dir, uint32_t speed, uint32_t steps) {
    if ((dir != MOTOR_DIR_NONE) && (steps > 0)) {
        motor_record_move_time(addr);
    }
    switch (dir) {
        case MOTOR_DIR_NONE:
            motor_control(addr, MOTOR_DIR_NONE, 0, 0);
            break;
        case MOTOR_DIR_FORWARD:
            if ((int32_t)motor_get_cur_step(addr) >= (int32_t)m_motorctrl[addr-1].forward_limit - 10)
                motor_control(addr, MOTOR_DIR_NONE, 0, 0);
            else
                motor_control(addr, MOTOR_DIR_FORWARD, speed, steps);
            break;
        case MOTOR_DIR_BACKWARD:
            if ((int32_t)motor_get_cur_step(addr) <= (int32_t)m_motorctrl[addr-1].backward_limit + 10)
                motor_control(addr, MOTOR_DIR_NONE, 0, 0);
            else
                motor_control(addr, MOTOR_DIR_BACKWARD, speed, steps);
            break;
        default:
            motor_control(addr, MOTOR_DIR_NONE, 0, 0);
            break;
    }
}

void joystick_smooth_speed(uint32_t *curr_speed, uint32_t target_speed, uint16_t slop) {
#define MOTOR_MAX_SPEED 50000
    if (*curr_speed >= (target_speed + slop))
        *curr_speed = *curr_speed - slop;
    else if (*curr_speed < target_speed) {
        *curr_speed = *curr_speed + slop;
        if (*curr_speed > MOTOR_MAX_SPEED) *curr_speed = MOTOR_MAX_SPEED;
    } else {
        *curr_speed = target_speed;
        if (*curr_speed > MOTOR_MAX_SPEED) *curr_speed = MOTOR_MAX_SPEED;
    }
}

uint32_t calculate_xy_speed(int32_t abs_error, uint32_t user_max_speed, uint8_t mode_val) {
    if (abs_error <= SERVO_DEADBAND_STEPS) return 0;

    double p_speed = (abs_error - SERVO_DEADBAND_STEPS) * (15.0 + mode_val * 1.2);
    uint32_t min_spd = 120 + mode_val * 8;

    if (p_speed < min_spd) p_speed = min_spd;
    if (p_speed > user_max_speed) p_speed = user_max_speed;
    return (uint32_t)p_speed;
}

// ===================================================================
// 核心主控制逻辑
// ===================================================================
extern uint8_t Is_Center_Back_Running(void);
extern uint8_t m_joystick_mode_x;
extern uint8_t m_joystick_mode_y;
extern uint8_t m_joystick_mode_z;

void ProcessMotorControl(const joystick_state_t* last, const joystick_state_t* curr)
{
    // 1. 摇杆总开关检查
    if (m_joystick_ctrl == 0) {
        s_integrated_tgt_x = (double)motor_get_cur_step(JOY_MOTOR_M1);
        s_integrated_tgt_y = (double)motor_get_cur_step(JOY_MOTOR_M2);
        s_prev_enc_x       = s_integrated_tgt_x;
        s_prev_enc_y       = s_integrated_tgt_y;
        s_curr_speed_x     = 0;
        s_curr_speed_y     = 0;
        s_btn_ramp_speed_x = 0.0f;
        s_btn_ramp_speed_y = 0.0f;
        last_x_dir         = MOTOR_DIR_NONE;
        last_y_dir         = MOTOR_DIR_NONE;
        return;
    }

//    uint16_t curr_phys_btn = curr->buttons & 0x01FF;
//    uint16_t last_phys_btn = last->buttons & 0x01FF;
    uint16_t curr_phys_btn = curr->buttons & 0x001F;
    uint16_t last_phys_btn = last->buttons & 0x001F;
    if (curr_phys_btn != last_phys_btn) {
        //protocol_report_button((uint8_t)(curr_phys_btn & 0x00FF));
    	protocol_report_button((uint8_t)curr_phys_btn);
    }

    static int button_move_step = 100;
    uint16_t buttons = curr->buttons;

    int64_t dt = curr->timestamp_us - last->timestamp_us;
    if (dt < 100) dt = 1000;

    double cur_enc_x = (double)motor_get_cur_step(JOY_MOTOR_M1);
    double cur_enc_y = (double)motor_get_cur_step(JOY_MOTOR_M2);

    if (!s_inited) {
        s_integrated_tgt_x = cur_enc_x;
        s_integrated_tgt_y = cur_enc_y;
        s_prev_enc_x       = cur_enc_x;
        s_prev_enc_y       = cur_enc_y;
        s_inited           = 1;
        printf("[JOY_INIT] Delta-Integration Model Inited at Enc(%.0f, %.0f)\r\n", s_integrated_tgt_x, s_integrated_tgt_y);
    }

    // 光电纠偏阶跃吸收
    double delta_enc_x = cur_enc_x - s_prev_enc_x;
    double delta_enc_y = cur_enc_y - s_prev_enc_y;

    if (fabs(delta_enc_x) > 600.0) s_integrated_tgt_x += delta_enc_x;
    if (fabs(delta_enc_y) > 600.0) s_integrated_tgt_y += delta_enc_y;
    s_prev_enc_x = cur_enc_x;
    s_prev_enc_y = cur_enc_y;

    // 原始帧间物理位移与按键动作判定
    int16_t dx = curr->x - last->x;
    int16_t dy = curr->y - last->y;
    int16_t dz = curr->z - last->z;

    // ===================================================================
    // 【150 毛刺单帧拦截器】
    // ===================================================================
    static uint8_t s_glitch_pending_x = 0;
    static uint8_t s_glitch_pending_y = 0;

    if (!s_is_moving_active) {
        if (s_glitch_pending_x == 0) {
            if (abs(dx) > 100) { s_glitch_pending_x = 1; dx = 0; }
        } else {
            if (abs(curr->x - last->x) > 100) { s_glitch_pending_x = 0; }
            else { s_glitch_pending_x = 0; dx = 0; }
        }
    } else {
        s_glitch_pending_x = 0;
    }

    if (!s_is_moving_active) {
        if (s_glitch_pending_y == 0) {
            if (abs(dy) > 100) { s_glitch_pending_y = 1; dy = 0; }
        } else {
            if (abs(curr->y - last->y) > 100) { s_glitch_pending_y = 0; }
            else { s_glitch_pending_y = 0; dy = 0; }
        }
    } else {
        s_glitch_pending_y = 0;
    }

    int16_t abs_dx = abs(dx);
    int16_t abs_dy = abs(dy);
    uint16_t joy_btn_motion = buttons & (MOVE_LEFT | MOVE_RIGHT | MOVE_DOWN | MOVE_UP);

    // ===================================================================
    // 【上位机强力打断仲裁】
    // 注意：严禁在此处直接 return 截断，否则会导致 [JOY_TRACK] 丢失心跳打印
    // ===================================================================
    if (g_motor_owner == MOTOR_OWNER_HOST) {
        if (Is_Center_Back_Running()) {
            s_curr_speed_x = 0;
            s_curr_speed_y = 0;
            s_btn_ramp_speed_x = 0.0f;
            s_btn_ramp_speed_y = 0.0f;
            last_x_dir = MOTOR_DIR_NONE;
            last_y_dir = MOTOR_DIR_NONE;
            s_is_moving_active = 0;
            s_stop_hold_cnt = 0;
            s_integrated_tgt_x = cur_enc_x;
            s_integrated_tgt_y = cur_enc_y;
        } else {
            uint16_t btn_rising_edge = (curr->buttons & ~last->buttons) & 0x01FF;

            // 只有位移达到明确意图（>20）或按下实体按键，才抢夺控制权
            if (abs_dx > 20 || abs_dy > 20 || btn_rising_edge != 0) {
                g_motor_owner = MOTOR_OWNER_JOYSTICK;
                s_is_moving_active = 1;
                s_stop_hold_cnt = 0;
                g_joystick_active_flag = 1;

                System_Abort_All_Tasks_And_Stop();

                s_integrated_tgt_x = cur_enc_x;
                s_integrated_tgt_y = cur_enc_y;
                s_curr_speed_x = 0;
                s_curr_speed_y = 0;
                s_btn_ramp_speed_x = 0.0f;
                s_btn_ramp_speed_y = 0.0f;
                last_x_dir = MOTOR_DIR_NONE;
                last_y_dir = MOTOR_DIR_NONE;
                printf("[PREEMPTION] Host Interrupted by Joystick! Token seized.\r\n");
            }
        }
    }

    // ===================================================================
    // 【核心状态转移：时间相关性消抖（解决 3~8 底噪而不伤慢速微动）】
    // ===================================================================
    static uint8_t s_motion_detect_cnt = 0;

    if (!s_is_moving_active) {
        // 静止态：单帧位移大于 4 视为有效微动候选
        uint8_t has_motion = (abs_dx > 4 || abs_dy > 4);

        if (joy_btn_motion != 0) {
            s_motion_detect_cnt = 3;
        } else if (has_motion) {
            if (s_motion_detect_cnt < 3) s_motion_detect_cnt++;
        } else {
            s_motion_detect_cnt = 0;
        }

        // 必须连续 3 帧确认有微动（约 20~30ms），彻底杜绝单次/偶发底噪跳变
        if (s_motion_detect_cnt >= 3) {
            s_motion_detect_cnt = 0;
            s_is_moving_active = 1;
            s_stop_hold_cnt = 0;

            if (g_motor_owner == MOTOR_OWNER_HOST) {
                if (!Is_Center_Back_Running()) {
                    g_motor_owner = MOTOR_OWNER_JOYSTICK;
                    System_Abort_All_Tasks_And_Stop();
                    s_integrated_tgt_x = cur_enc_x;
                    s_integrated_tgt_y = cur_enc_y;
                }
            } else {
                g_motor_owner = MOTOR_OWNER_JOYSTICK;
            }
        }
    } else {
        // 运动态：连续 6 帧完全停止（<=1）才平滑退出
        if (abs_dx <= 1 && abs_dy <= 1 && joy_btn_motion == 0) {
            s_stop_hold_cnt++;
            if (s_stop_hold_cnt >= 6) {
                s_is_moving_active = 0;
                s_stop_hold_cnt = 0;
                s_motion_detect_cnt = 0;
            }
        } else {
            s_stop_hold_cnt = 0;
        }
    }

    // ===================================================================
    // 【速度与距离多档映射（X/Y/Z 三轴解耦独立）】
    // ===================================================================
    uint8_t mode_val_x = (m_joystick_mode_x < 1) ? 1 : m_joystick_mode_x;
    uint8_t mode_val_y = (m_joystick_mode_y < 1) ? 1 : m_joystick_mode_y;

    float scale_x = (float)mode_val_x / 20.0f;
    float scale_y = (float)mode_val_y / 20.0f;

    float step_ratio_x = (((TOTAL_STROKE_MM_X * MOTOR_STEPS_PER_MM * scale_x) / 65536.0f) * JOY_RANGE_SCALE_FACTOR);
    float step_ratio_y = (((TOTAL_STROKE_MM_Y * MOTOR_STEPS_PER_MM * scale_y) / 65536.0f) * JOY_RANGE_SCALE_FACTOR);

    uint32_t max_speed_x = (mode_val_x == 1) ? 400 :
                          ((mode_val_x == 20) ? MAX_RUN_SPEED_XY : CLAMP((uint32_t)(mode_val_x * 1500), 500, MAX_RUN_SPEED_XY));
    uint32_t max_speed_y = (mode_val_y == 1) ? 400 :
                          ((mode_val_y == 20) ? MAX_RUN_SPEED_XY : CLAMP((uint32_t)(mode_val_y * 1000), 500, MAX_RUN_SPEED_XY));

    // ===================================================================
    // 【推杆位移累加：运动期 100% 原始直通，手感细腻不断续】
    // ===================================================================
    char active_axis = '0';
    if (s_is_moving_active && joy_btn_motion == 0) {
        if (abs_dx >= abs_dy) {
            if (dx != 0) {
                s_integrated_tgt_x += (double)dx * step_ratio_x;
                active_axis = 'X';
            }
        } else {
            if (dy != 0) {
                s_integrated_tgt_y -= (double)dy * step_ratio_y;
                active_axis = 'Y';
            }
        }
    }
    else if (!s_is_moving_active) {
        // 静止态：锁紧当前位置，平滑归还控制权
        s_integrated_tgt_x = cur_enc_x;
        s_integrated_tgt_y = cur_enc_y;

        if (g_motor_owner == MOTOR_OWNER_JOYSTICK &&
            s_curr_speed_x == 0 && s_curr_speed_y == 0 &&
            s_btn_ramp_speed_x == 0.0f && s_btn_ramp_speed_y == 0.0f &&
            current_z_speed == 0) {
            g_motor_owner = MOTOR_OWNER_IDLE;
        }
    }

    // 软限位保护截断
    int32_t fwd_limit_x = (int32_t)m_motorctrl[JOY_MOTOR_M1 - 1].forward_limit - 10;
    int32_t bwd_limit_x = (int32_t)m_motorctrl[JOY_MOTOR_M1 - 1].backward_limit + 10;
    if (s_integrated_tgt_x > fwd_limit_x) s_integrated_tgt_x = fwd_limit_x;
    if (s_integrated_tgt_x < bwd_limit_x) s_integrated_tgt_x = bwd_limit_x;

    int32_t fwd_limit_y = (int32_t)m_motorctrl[JOY_MOTOR_M2 - 1].forward_limit - 10;
    int32_t bwd_limit_y = (int32_t)m_motorctrl[JOY_MOTOR_M2 - 1].backward_limit + 10;
    if (s_integrated_tgt_y > fwd_limit_y) s_integrated_tgt_y = fwd_limit_y;
    if (s_integrated_tgt_y < bwd_limit_y) s_integrated_tgt_y = bwd_limit_y;

    double error_x = s_integrated_tgt_x - cur_enc_x;
    double abs_error_x = fabs(error_x);

    double error_y = s_integrated_tgt_y - cur_enc_y;
    double abs_error_y = fabs(error_y);

    // ===================================================================
    // X 轴控制
    // ===================================================================
    if (buttons & (MOVE_LEFT | MOVE_RIGHT)) {
        if (g_motor_owner == MOTOR_OWNER_JOYSTICK) {
            active_axis = 'X';
            if (motor_move_flag[JOY_MOTOR_M1 - 1] == 1) { PID_Stop(JOY_MOTOR_M1); last_x_dir = MOTOR_DIR_NONE; }
            move_step_check(&button_move_step);

            motor_dir_e dir_btn_x = (buttons & MOVE_LEFT) ? MOTOR_DIR_BACKWARD : MOTOR_DIR_FORWARD;
            uint32_t target_btn_speed_x = CLAMP(max_speed_x, 500, 50000);

            if (dt > 45000 || (last_x_dir != MOTOR_DIR_NONE && dir_btn_x != last_x_dir)) {
                            s_btn_ramp_speed_x = BTN_START_SPEED;
                        }
            // 【新增 2】失步自恢复：若速度 >3000Hz 但连续 5 帧 (~50ms) 编码器未动，打回 500Hz 恢复转矩
			static double  s_btn_last_enc_x = 0.0;
			static uint8_t s_btn_slip_cnt_x = 0;
			if (s_btn_ramp_speed_x > 3000.0f && fabs(cur_enc_x - s_btn_last_enc_x) < 2.0) {
				if (++s_btn_slip_cnt_x >= 5) {
					s_btn_ramp_speed_x = BTN_START_SPEED;
					s_btn_slip_cnt_x   = 0;
					xil_printf("[JOY_RECOVER] X-Btn slip detected, reset speed to 500Hz!\r\n");
				}
			} else {
				s_btn_slip_cnt_x = 0;
			}
			s_btn_last_enc_x = cur_enc_x;

            if (s_btn_ramp_speed_x < BTN_START_SPEED) {
                s_btn_ramp_speed_x = BTN_START_SPEED;
            } else if (s_btn_ramp_speed_x + BTN_ACCEL_STEP < target_btn_speed_x) {
                s_btn_ramp_speed_x += BTN_ACCEL_STEP;
            } else {
                s_btn_ramp_speed_x = (float)target_btn_speed_x;
            }

            uint32_t cur_btn_speed_x = (uint32_t)s_btn_ramp_speed_x;
            if (cur_btn_speed_x > M2_SAFE_MAX_FREQ) cur_btn_speed_x = M2_SAFE_MAX_FREQ;

            joystick_motor_ctrl(JOY_MOTOR_M1, dir_btn_x, cur_btn_speed_x, button_move_step);
            last_x_dir = dir_btn_x;
            s_integrated_tgt_x = cur_enc_x;
            s_curr_speed_x = cur_btn_speed_x;

            s_integrated_tgt_y = cur_enc_y;
            s_curr_speed_y = 0;
            last_y_dir = MOTOR_DIR_NONE;
            s_btn_ramp_speed_y = 0.0f;
        }
    }
    else if (g_motor_owner == MOTOR_OWNER_JOYSTICK) {
        s_btn_ramp_speed_x = 0.0f;

        if (abs_error_x > SERVO_DEADBAND_STEPS) {
            motor_dir_e dir_x = (error_x > 0) ? MOTOR_DIR_FORWARD : MOTOR_DIR_BACKWARD;

            if ((dir_x == MOTOR_DIR_FORWARD && cur_enc_x >= fwd_limit_x) ||
                (dir_x == MOTOR_DIR_BACKWARD && cur_enc_x <= bwd_limit_x))
            {
                s_curr_speed_x = 0;
                if (last_x_dir != MOTOR_DIR_NONE) {
                    joystick_motor_ctrl(JOY_MOTOR_M1, MOTOR_DIR_NONE, 0, 0);
                    last_x_dir = MOTOR_DIR_NONE;
                }
            }
            else
            {
                uint32_t target_spd_x = calculate_xy_speed((int32_t)abs_error_x, max_speed_x, mode_val_x);
                if (target_spd_x > M2_SAFE_MAX_FREQ) target_spd_x = M2_SAFE_MAX_FREQ;

                if (last_x_dir == MOTOR_DIR_NONE || s_curr_speed_x == 0) {
                    s_curr_speed_x = (max_speed_x < XY_MIN_RUN_SPEED) ? max_speed_x : XY_MIN_RUN_SPEED;
                    last_x_dir = dir_x;
                    uint32_t steps_init = (uint32_t)abs_error_x;
                    if (steps_init < 5) steps_init = 5;
                    joystick_motor_ctrl(JOY_MOTOR_M1, dir_x, s_curr_speed_x, steps_init);
                }
                else if (dir_x != last_x_dir) {
                    if (s_curr_speed_x > XY_DECEL_STEP * 2) s_curr_speed_x -= (XY_DECEL_STEP * 2);
                    else s_curr_speed_x = 0;

                    uint32_t steps_brk = (uint32_t)(s_curr_speed_x / 100 + 5);
                    joystick_motor_ctrl(JOY_MOTOR_M1, last_x_dir, s_curr_speed_x, steps_brk);

                    if (s_curr_speed_x <= XY_MIN_RUN_SPEED) {
                        last_x_dir = dir_x;
                        s_curr_speed_x = (max_speed_x < XY_MIN_RUN_SPEED) ? max_speed_x : XY_MIN_RUN_SPEED;
                    }
                }
                else {
                    uint32_t min_floor = (max_speed_x < XY_MIN_RUN_SPEED) ? max_speed_x : XY_MIN_RUN_SPEED;
                    if (s_curr_speed_x < min_floor) s_curr_speed_x = min_floor;
                    if (s_curr_speed_x < target_spd_x) {
                        s_curr_speed_x += XY_ACCEL_STEP;
                        if (s_curr_speed_x > target_spd_x) s_curr_speed_x = target_spd_x;
                    } else if (s_curr_speed_x > target_spd_x) {
                        if (s_curr_speed_x > XY_DECEL_STEP) s_curr_speed_x -= XY_DECEL_STEP;
                        else s_curr_speed_x = target_spd_x;
                    }

                    uint32_t steps_to_send_x = (uint32_t)abs_error_x;
                    if (steps_to_send_x < 5) steps_to_send_x = 5;

                    joystick_motor_ctrl(JOY_MOTOR_M1, dir_x, s_curr_speed_x, steps_to_send_x);
                    last_x_dir = dir_x;
                }
            }
        }
        else {
            if (s_curr_speed_x > XY_DECEL_STEP) {
                s_curr_speed_x -= XY_DECEL_STEP;
                uint32_t steps_brk = (uint32_t)(s_curr_speed_x / 100 + 5);
                joystick_motor_ctrl(JOY_MOTOR_M1, last_x_dir, s_curr_speed_x, steps_brk);
            } else {
                s_curr_speed_x = 0;
                if (last_x_dir != MOTOR_DIR_NONE) {
                    joystick_motor_ctrl(JOY_MOTOR_M1, MOTOR_DIR_NONE, 0, 0);
                    last_x_dir = MOTOR_DIR_NONE;
                }
            }
        }
    } else {
        s_btn_ramp_speed_x = 0.0f;
        if (s_curr_speed_x > XY_DECEL_STEP) {
            s_curr_speed_x -= XY_DECEL_STEP;
            uint32_t steps_brk = (uint32_t)(s_curr_speed_x / 100 + 5);
            joystick_motor_ctrl(JOY_MOTOR_M1, last_x_dir, s_curr_speed_x, steps_brk);
        } else {
            s_curr_speed_x = 0;
            if (last_x_dir != MOTOR_DIR_NONE) {
                joystick_motor_ctrl(JOY_MOTOR_M1, MOTOR_DIR_NONE, 0, 0);
                last_x_dir = MOTOR_DIR_NONE;
            }
        }
    }

    // ===================================================================
    // Y 轴控制
    // ===================================================================
    if (buttons & (MOVE_DOWN | MOVE_UP)) {
        if (g_motor_owner == MOTOR_OWNER_JOYSTICK) {
            active_axis = 'Y';
            if (motor_move_flag[JOY_MOTOR_M2 - 1] == 1) { PID_Stop(JOY_MOTOR_M2); last_y_dir = MOTOR_DIR_NONE; }
            move_step_check(&button_move_step);

            motor_dir_e dir_btn_y = (buttons & MOVE_DOWN) ? MOTOR_DIR_FORWARD : MOTOR_DIR_BACKWARD;
            uint32_t target_btn_speed_y = CLAMP(max_speed_y, 500, 50000);


            if (dt > 45000 || (last_y_dir != MOTOR_DIR_NONE && dir_btn_y != last_y_dir)) {
                            s_btn_ramp_speed_y = BTN_START_SPEED;
                        }

			// 【新增 2】失步自恢复
			static double  s_btn_last_enc_y = 0.0;
			static uint8_t s_btn_slip_cnt_y = 0;
			if (s_btn_ramp_speed_y > 3000.0f && fabs(cur_enc_y - s_btn_last_enc_y) < 2.0) {
				if (++s_btn_slip_cnt_y >= 5) {
					s_btn_ramp_speed_y = BTN_START_SPEED;
					s_btn_slip_cnt_y   = 0;
					xil_printf("[JOY_RECOVER] Y-Btn slip detected, reset speed to 500Hz!\r\n");
				}
			} else {
				s_btn_slip_cnt_y = 0;
			}
			s_btn_last_enc_y = cur_enc_y;




            if (s_btn_ramp_speed_y < BTN_START_SPEED) {
                s_btn_ramp_speed_y = BTN_START_SPEED;
            } else if (s_btn_ramp_speed_y + BTN_ACCEL_STEP < target_btn_speed_y) {
                s_btn_ramp_speed_y += BTN_ACCEL_STEP;
            } else {
                s_btn_ramp_speed_y = (float)target_btn_speed_y;
            }

            uint32_t cur_btn_speed_y = (uint32_t)s_btn_ramp_speed_y;
            joystick_motor_ctrl(JOY_MOTOR_M2, dir_btn_y, cur_btn_speed_y, button_move_step);
            last_y_dir = dir_btn_y;
            s_integrated_tgt_y = cur_enc_y;
            s_curr_speed_y = cur_btn_speed_y;

            s_integrated_tgt_x = cur_enc_x;
            s_curr_speed_x = 0;
            last_x_dir = MOTOR_DIR_NONE;
            s_btn_ramp_speed_x = 0.0f;
        }
    }
    else if (g_motor_owner == MOTOR_OWNER_JOYSTICK) {
        s_btn_ramp_speed_y = 0.0f;

        if (abs_error_y > SERVO_DEADBAND_STEPS) {
            motor_dir_e dir_y = (error_y > 0) ? MOTOR_DIR_FORWARD : MOTOR_DIR_BACKWARD;

            if ((dir_y == MOTOR_DIR_FORWARD && cur_enc_y >= fwd_limit_y) ||
                (dir_y == MOTOR_DIR_BACKWARD && cur_enc_y <= bwd_limit_y))
            {
                s_curr_speed_y = 0;
                if (last_y_dir != MOTOR_DIR_NONE) {
                    joystick_motor_ctrl(JOY_MOTOR_M2, MOTOR_DIR_NONE, 0, 0);
                    last_y_dir = MOTOR_DIR_NONE;
                }
            }
            else
            {
                uint32_t target_spd_y = calculate_xy_speed((int32_t)abs_error_y, max_speed_y, mode_val_y);

                if (last_y_dir == MOTOR_DIR_NONE || s_curr_speed_y == 0) {
                    s_curr_speed_y = (max_speed_y < XY_MIN_RUN_SPEED) ? max_speed_y : XY_MIN_RUN_SPEED;
                    last_y_dir = dir_y;
                    uint32_t steps_init = (uint32_t)abs_error_y;
                    if (steps_init < 5) steps_init = 5;
                    joystick_motor_ctrl(JOY_MOTOR_M2, dir_y, s_curr_speed_y, steps_init);
                }
                else if (dir_y != last_y_dir) {
                    if (s_curr_speed_y > XY_DECEL_STEP * 2) s_curr_speed_y -= (XY_DECEL_STEP * 2);
                    else s_curr_speed_y = 0;

                    uint32_t steps_brk = (uint32_t)(s_curr_speed_y / 100 + 5);
                    joystick_motor_ctrl(JOY_MOTOR_M2, last_y_dir, s_curr_speed_y, steps_brk);

                    if (s_curr_speed_y <= XY_MIN_RUN_SPEED) {
                        last_y_dir = dir_y;
                        s_curr_speed_y = (max_speed_y < XY_MIN_RUN_SPEED) ? max_speed_y : XY_MIN_RUN_SPEED;
                    }
                }
                else {
                    uint32_t min_floor_y = (max_speed_y < XY_MIN_RUN_SPEED) ? max_speed_y : XY_MIN_RUN_SPEED;
                    if (s_curr_speed_y < min_floor_y) s_curr_speed_y = min_floor_y;
                    if (s_curr_speed_y < target_spd_y) {
                        s_curr_speed_y += XY_ACCEL_STEP;
                        if (s_curr_speed_y > target_spd_y) s_curr_speed_y = target_spd_y;
                    } else if (s_curr_speed_y > target_spd_y) {
                        if (s_curr_speed_y > XY_DECEL_STEP) s_curr_speed_y -= XY_DECEL_STEP;
                        else s_curr_speed_y = target_spd_y;
                    }

                    uint32_t steps_to_send_y = (uint32_t)abs_error_y;
                    if (steps_to_send_y < 5) steps_to_send_y = 5;

                    joystick_motor_ctrl(JOY_MOTOR_M2, dir_y, s_curr_speed_y, steps_to_send_y);
                    last_y_dir = dir_y;
                }
            }
        }
        else {
            if (s_curr_speed_y > XY_DECEL_STEP) {
                s_curr_speed_y -= XY_DECEL_STEP;
                uint32_t steps_brk = (uint32_t)(s_curr_speed_y / 100 + 5);
                joystick_motor_ctrl(JOY_MOTOR_M2, last_y_dir, s_curr_speed_y, steps_brk);
            } else {
                s_curr_speed_y = 0;
                if (last_y_dir != MOTOR_DIR_NONE) {
                    joystick_motor_ctrl(JOY_MOTOR_M2, MOTOR_DIR_NONE, 0, 0);
                    last_y_dir = MOTOR_DIR_NONE;
                }
            }
        }
    } else {
        s_btn_ramp_speed_y = 0.0f;
        if (s_curr_speed_y > XY_DECEL_STEP) {
            s_curr_speed_y -= XY_DECEL_STEP;
            uint32_t steps_brk = (uint32_t)(s_curr_speed_y / 100 + 5);
            joystick_motor_ctrl(JOY_MOTOR_M2, last_y_dir, s_curr_speed_y, steps_brk);
        } else {
            s_curr_speed_y = 0;
            if (last_y_dir != MOTOR_DIR_NONE) {
                joystick_motor_ctrl(JOY_MOTOR_M2, MOTOR_DIR_NONE, 0, 0);
                last_y_dir = MOTOR_DIR_NONE;
            }
        }
    }

    // ===================================================================
    // 【Z 轴控制（解耦独立配置、平滑加减速斜坡）】
    // ===================================================================
    uint8_t mode_val_z = (m_joystick_mode_z < 1) ? 1 : m_joystick_mode_z;
    uint32_t max_speed_z = (mode_val_z == 1) ? 800 :
                          ((mode_val_z == 20) ? MAX_SPEED_Z : CLAMP((uint32_t)(mode_val_z * 1000), 500, MAX_SPEED_Z));
    uint32_t min_speed_z = (mode_val_z == 1) ? 200 : 1000;

    int16_t valid_dz = (abs(dz) > 20 && curr->z != 0) ? dz : 0;

    if (valid_dz != 0) {
        active_axis = 'Z';
        motor_dir_e dir_z = (valid_dz > 0) ? MOTOR_DIR_FORWARD : MOTOR_DIR_BACKWARD;

        float z_ratio = (float)abs(valid_dz) / 120.0f;
        if (z_ratio > 1.0f) z_ratio = 1.0f;

        uint32_t target_spd_z = (uint32_t)(z_ratio * max_speed_z);
        if (target_spd_z < min_speed_z) target_spd_z = min_speed_z;
        if (target_spd_z > max_speed_z) target_spd_z = max_speed_z;

        uint32_t steps_z = abs(valid_dz) * mode_val_z;
        if (steps_z < 10) steps_z = 10;

        if ((dir_z != last_z_dir) && (last_z_dir != MOTOR_DIR_NONE)) {
            joystick_smooth_speed(&current_z_speed, 0, SLOP_Z * 4);
            joystick_motor_ctrl(TMC2660_M3, last_z_dir, current_z_speed, steps_z);
            if (current_z_speed <= 500) {
                last_z_dir = dir_z;
            }
        } else {
            joystick_smooth_speed(&current_z_speed, target_spd_z, SLOP_Z);
            joystick_motor_ctrl(TMC2660_M3, dir_z, current_z_speed, steps_z);
            last_z_dir = dir_z;
        }
    } else {
        if (last_z_dir != MOTOR_DIR_NONE) {
            joystick_smooth_speed(&current_z_speed, 0, SLOP_Z * 2);
            uint32_t steps_brk = (uint32_t)(current_z_speed / 100 + 5);
            joystick_motor_ctrl(TMC2660_M3, last_z_dir, current_z_speed, steps_brk);
            if (current_z_speed == 0) {
                last_z_dir = MOTOR_DIR_NONE;
            }
        }
    }

    // ===================================================================
    // 【调试日志输出（心跳机制，绝对不会断流）】
    // ===================================================================
    uint64_t cur_us = curr->timestamp_us;
    uint8_t is_moving = (abs_error_x > SERVO_DEADBAND_STEPS || abs_error_y > SERVO_DEADBAND_STEPS ||
                         last_x_dir != MOTOR_DIR_NONE || last_y_dir != MOTOR_DIR_NONE ||
                         last_z_dir != MOTOR_DIR_NONE || current_z_speed > 0);

   // if ((is_moving && (cur_us - s_last_log_time >= 100000)) || (cur_us - s_last_log_time >= 1000000)) {
//        printf("[JOY_TRACK] Mode:[X:%d Y:%d Z:%d] Axis:%c Owner:%d | X:[Tgt:%5d Enc:%5d Err:%4d Spd:%5d] | Y:[Tgt:%5d Enc:%5d Err:%4d Spd:%5d] | Z:[Raw:%5d dZ:%4d Dir:%d Spd:%5d]\r\n",
//               (int)mode_val_x, (int)mode_val_y, (int)mode_val_z, active_axis, (int)g_motor_owner,
//               (int)s_integrated_tgt_x, (int)cur_enc_x, (int)error_x, (int)s_curr_speed_x,
//               (int)s_integrated_tgt_y, (int)cur_enc_y, (int)error_y, (int)s_curr_speed_y,
//               (int)curr->z, (int)dz, (int)last_z_dir, (int)current_z_speed);
    	if (s_log_joy_enabled) {
    	        if ((is_moving && (cur_us - s_last_log_time >= 100000)) || (cur_us - s_last_log_time >= 1000000)) {
    	            xil_printf("[JOY_TRACK] Mode:[X:%d Y:%d Z:%d] Axis:%c Owner:%d | X:[Tgt:%5d Enc:%5d] | Y:[Tgt:%5d Enc:%5d]\r\n",
    	                       (int)mode_val_x, (int)mode_val_y, (int)mode_val_z, active_axis, (int)g_motor_owner,
    	                       (int)s_integrated_tgt_x, (int)cur_enc_x, (int)s_integrated_tgt_y, (int)cur_enc_y);
    	            s_last_log_time = cur_us;
    	        }
    	    }

    joystick_state_t* writable_curr = (joystick_state_t*)curr;
    writable_curr->x_dir = (uint8_t)last_x_dir;
    writable_curr->y_dir = (uint8_t)last_y_dir;

    g_joystick_active_flag = (last_x_dir != MOTOR_DIR_NONE || last_y_dir != MOTOR_DIR_NONE ||
                              last_z_dir != MOTOR_DIR_NONE || (curr->buttons & 0x01FF) != 0) ? 1 : 0;
    g_current_joystick_data = *writable_curr;
}
typedef struct {
    int16_t x; int16_t y; int16_t z;
} filter_xyz_t;

filter_xyz_t joystick_data_filter(filter_xyz_t joy_data) {
#define FILTER_COUNT 5
    static int16_t data_x[FILTER_COUNT] = {0}, data_y[FILTER_COUNT] = {0}, data_z[FILTER_COUNT] = {0};
    filter_xyz_t ret_data;
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;

    for (int i = 0; i < FILTER_COUNT - 1; i++) {
        data_x[i] = data_x[i+1]; data_y[i] = data_y[i+1]; data_z[i] = data_z[i+1];
    }
    data_x[FILTER_COUNT-1] = joy_data.x; data_y[FILTER_COUNT-1] = joy_data.y; data_z[FILTER_COUNT-1] = joy_data.z;

    for (int j = 0; j < FILTER_COUNT; j++) {
        sum_x += data_x[j]; sum_y += data_y[j]; sum_z += data_z[j];
    }
    ret_data.x = sum_x / FILTER_COUNT; ret_data.y = sum_y / FILTER_COUNT; ret_data.z = sum_z / FILTER_COUNT;

    return ret_data;
}

void joystick_process_packet(uint8_t *data, uint32_t len) {
    joystick_state_t current;
    filter_xyz_t xyz_filter;

    xyz_filter.x = (int16_t)((data[0] << 8) | data[1]);
    xyz_filter.y = (int16_t)((data[2] << 8) | data[3]);
    xyz_filter.z = (int16_t)((data[4] << 8) | data[5]);

    xyz_filter = joystick_data_filter(xyz_filter);

    current.x = xyz_filter.x;
    current.y = xyz_filter.y;
    current.z = ((int16_t)((data[4] << 8) | data[5]) == 0) ? 0 : xyz_filter.z;

    uint16_t raw_buttons = (uint16_t)((data[6] << 8) | data[7]);

    if (raw_buttons & 0xF000) {
        raw_buttons = 0;
    }

    current.buttons = raw_buttons;
    current.timestamp_us = timer_get_1us();

    current.x_dir = 0;
    current.y_dir = 0;

    g_joystick_data_updated = 1;

    int16_t diff_x = current.x - last_state.x;
    int16_t diff_y = current.y - last_state.y;
    int16_t diff_z = current.z - last_state.z;
    if (abs(diff_x) > 2 || abs(diff_y) > 2 || abs(diff_z) > 2) {
        g_last_joystick_pack_time = current.timestamp_us;
    }

    if ((!first_sample) && (m_joystick_ctrl)) {
        ProcessMotorControl(&last_state, &current);
    }
    last_state = current;
    first_sample = 0;
}
// 默认值均初始化为 20
uint8_t m_joystick_mode_x = 20;
uint8_t m_joystick_mode_y = 20;
uint8_t m_joystick_mode_z = 20;
/**
 * @brief 摇杆模式与三轴独立档位配置
 * @param mode  模式 (0: 默认高速20, 1: 低速1, 2: 自定义各轴档位)
 * @param val_x X轴自定义档位 (mode=2 时生效，上限50)
 * @param val_y Y轴自定义档位 (mode=2 时生效，上限50)
 * @param val_z Z轴自定义档位 (mode=2 时生效，上限50)
 */
void joystick_mode_config(uint8_t mode, uint8_t val_x, uint8_t val_y, uint8_t val_z) {
    switch (mode) {
        case 0:
            m_joystick_mode_x = 20;
            m_joystick_mode_y = 20;
            m_joystick_mode_z = 20;
            break;

        case 1:
            m_joystick_mode_x = 1;
            m_joystick_mode_y = 1;
            m_joystick_mode_z = 1;
            break;

        case 2:
            if (val_x > 0 && val_x < 50) m_joystick_mode_x = val_x;
            if (val_y > 0 && val_y < 50) m_joystick_mode_y = val_y;
            if (val_z > 0 && val_z < 50) m_joystick_mode_z = val_z;
            break;

        default:
            break;
    }

    printf("[JOY_CONFIG] Mode:%d -> Applied: X=%d, Y=%d, Z=%d\r\n",
           mode, m_joystick_mode_x, m_joystick_mode_y, m_joystick_mode_z);
}
void joystick_control_config(uint8_t mode) {
    m_joystick_ctrl = mode;
}

// ===================================================================
// 【FPGA 接管保护实现逻辑】
// ===================================================================
void Joystick_FPGA_Takeover_Enter(void)
{
    m_joystick_ctrl = 0;

    motor_control(TMC2660_M1, MOTOR_DIR_NONE, 0, 0);
    motor_control(TMC2660_M2, MOTOR_DIR_NONE, 0, 0);
    motor_control(TMC2660_M3, MOTOR_DIR_NONE, 0, 0);

    g_joystick_active_flag = 0;

    xil_printf("[FPGA_TAKEOVER] Enter: Joystick disabled, PS motor drive halted.\r\n");
}

void Joystick_FPGA_Takeover_Exit(void)
{
    ReadAllSteps();
    double cur_enc_x = (double)motor_get_cur_step(JOY_MOTOR_M1);
    double cur_enc_y = (double)motor_get_cur_step(JOY_MOTOR_M2);

    s_integrated_tgt_x = cur_enc_x;
    s_integrated_tgt_y = cur_enc_y;
    s_prev_enc_x       = cur_enc_x;
    s_prev_enc_y       = cur_enc_y;

    s_curr_speed_x     = 0;
    s_curr_speed_y     = 0;
    current_z_speed    = 0;
    last_x_dir         = MOTOR_DIR_NONE;
    last_y_dir         = MOTOR_DIR_NONE;
    last_z_dir         = MOTOR_DIR_NONE;
    s_btn_ramp_speed_x = 0.0f;
    s_btn_ramp_speed_y = 0.0f;
    s_is_moving_active = 0;
    s_stop_hold_cnt    = 0;

    first_sample = 1;

    Joystick_Filter_Flush();
    g_last_joystick_pack_time = timer_get_1us();

    g_motor_owner   = MOTOR_OWNER_IDLE;
    m_joystick_ctrl = 1;

    xil_printf("[FPGA_TAKEOVER] Exit: Resynced Target to Enc(X:%.0f, Y:%.0f). Safe startup guaranteed!\r\n",
               cur_enc_x, cur_enc_y);
}

// ===================================================================
// 状态与异常上报
// ===================================================================
void check_and_report_motor_status(void) {
    #define REPORT_INTERVAL_US 1000000
    #define MOTOR_MOVE_THRESHOLD 50

    static uint64_t s_last_report_time = 0;
    static uint8_t  s_last_reported_status = 0xFF;
    static uint64_t s_last_sample_time = 0;
    static int32_t  s_last_steps[4] = {0};
    static uint8_t  s_is_moving_saved = 0;

    uint8_t joy_private_buf[2] = {0};
    uint64_t current_time_us = timer_get_1us();

    extern int32_t Get_Current_Coord(uint8_t addr);

    if (current_time_us - s_last_sample_time >= 100000) {
        uint8_t moving_detected = 0;
        for (uint8_t i = 0; i < 4; i++) {
            int32_t current_step = Get_Current_Coord(i + 1);
            int32_t step_diff = abs(current_step - s_last_steps[i]);
            if (step_diff > MOTOR_MOVE_THRESHOLD) moving_detected = 1;
            s_last_steps[i] = current_step;
        }
        s_is_moving_saved = moving_detected;
        s_last_sample_time = current_time_us;
    }

    if (s_is_moving_saved == 0x01) {
        if (s_last_reported_status != 0x01 || (current_time_us - s_last_report_time >= REPORT_INTERVAL_US)) {
            joy_private_buf[0] = 0x01;
            if (g_current_tpcb != NULL) { protocol_send(0x0A, g_current_tpcb, 0x65, joy_private_buf, 1); tcp_output(g_current_tpcb); }
            s_last_report_time = current_time_us; s_last_reported_status = 0x01;
        }
    } else {
        if (s_last_reported_status != 0x00) {
            joy_private_buf[0] = 0x00;
            if (g_current_tpcb != NULL) { protocol_send(0x0A, g_current_tpcb, 0x65, joy_private_buf, 1); tcp_output(g_current_tpcb); }
            s_last_report_time = current_time_us; s_last_reported_status = 0x00;
        }
    }
}

void check_joystick_disconnect_watchdog(void) {
    if (g_last_joystick_pack_time == 0) return;
    uint64_t current_time = timer_get_1us();
    if (current_time - g_last_joystick_pack_time > 200000) {
        g_joystick_active_flag = 0;
        motor_control(TMC2660_M2, 0, 0, 0);
        motor_control(TMC2660_M1, 0, 0, 0);
        motor_control(TMC2660_M3, 0, 0, 0);
        static uint64_t last_warn_time = 0;
        if (current_time - last_warn_time >= 1000000) {
            xil_printf("[CRITICAL_ALERT] JOYSTICK SIGNAL LOST! HARDWARE BRAKE APPLIED!\r\n");
            last_warn_time = current_time;
        }
    }
}

void Joystick_Filter_Flush(void) {
    g_joystick_active_flag = 0;
    joystick_filter_init(&jf_x);
    joystick_filter_init(&jf_y);
    for (int i = 0; i < 20; i++) {
        joystick_filter_add(&jf_x, -16184);
        joystick_filter_add(&jf_y, 113);
    }
}








// ===================================================================
// 【堵转黑匣子快照 & 自动化复现压测引擎】
// ===================================================================
extern uint8_t motor_get_motor_fault(uint8_t motor_id);
extern void Motor_ConfigRegister(MOTOR_ADDR motor_id, uint32_t reg_addr, uint32_t value);
extern volatile uint64_t g_motor_expected_end_us[];
extern volatile uint8_t motor_move_flag[];

static uint8_t s_joy_stress_mode = 0;   // 0:关闭, 1:全行程长移压测, 2:纯PL寄存器刷写压测
static uint8_t s_is_synthetic_pkt = 0;  // 内部注入数据包标记

// 1. 黑匣子抓拍：基于理论时间窗监控，零误报且不干扰电机运行
void Joy_Stall_Blackbox_Poll(void)
{
    static uint64_t last_chk_us = 0;
    static int32_t  last_enc[3] = {0, 0, 0};
    static uint8_t  stall_ticks[3] = {0, 0, 0};
    static uint8_t  reported[3] = {0, 0, 0};
    static uint8_t  inited = 0;

    uint64_t now = timer_get_1us();
    if (now - last_chk_us < 10000) return; // 10ms 采样周期
    last_chk_us = now;

    if (!inited) {
        for (int i = 0; i < 3; i++) {
            last_enc[i] = motor_get_cur_step((MOTOR_ADDR)(i + 1));
        }
        inited = 1;
        return;
    }

    // 同时监控 M1(Y轴)、M2(X轴)、M3(Z轴)
    for (int i = 0; i < 3; i++) {
        MOTOR_ADDR addr = (MOTOR_ADDR)(i + 1);
        int32_t cur_enc = motor_get_cur_step(addr);
        int32_t delta   = abs(cur_enc - last_enc[i]);
        last_enc[i]     = cur_enc;

        uint8_t in_time_window = (now < g_motor_expected_end_us[i]);

        uint32_t pl_reg = Xil_In32(WRITE_ADDR_MOTOR1 + i * 4);
        uint32_t pl_spd = (pl_reg >> 16) & 0xFFFF;
        uint32_t pl_dir = (pl_reg >> 15) & 0x01;
        uint32_t pl_stp = pl_reg & 0x7FFF;

        // 连续 30ms (3 ticks) 无位移即抓拍，确保 40ms 的 50kHz 短脉冲堵转也能必抓
        if (in_time_window && pl_spd > 1200 && pl_stp > 20 && delta < 2) {
            stall_ticks[i]++;
            if (stall_ticks[i] >= 3 && !reported[i]) {
                reported[i] = 1;
                uint8_t sg_fault = motor_get_motor_fault(addr);
                int8_t  holl     = motor_get_limit_center(addr);

                xil_printf("\r\n================ [STALL BLACKBOX CAPTURED] ================\r\n");
                xil_printf("  Stalled Motor : M%d\r\n", addr);
                xil_printf("  PL Reg Raw    : 0x%08X (Spd=%u Hz, Dir=%u, RemStep=%u)\r\n",
                           pl_reg, pl_spd, pl_dir, pl_stp);
                xil_printf("  Encoder Pos   : %ld (Delta in 10ms = %ld)\r\n", (long)cur_enc, (long)delta);
                xil_printf("  TMC2660 SG Pin: %u (1=Driver Stall/Fault, 0=Normal)\r\n", sg_fault);
                xil_printf("  Hall Sensor   : %d | Owner: %u | JoyCtrl: %u | PID_Flag: %u\r\n",
                           holl, g_motor_owner, m_joystick_ctrl, motor_move_flag[i]);
                xil_printf("===========================================================\r\n");
            }
        } else {
            stall_ticks[i] = 0;
            reported[i]    = 0;
        }
    }
}

// 2. 自动压测发包器：模拟极端摇杆动作或底层寄存器高频刷写
// 2. 自动压测发包器：全行程长距离连续移动压测（覆盖 34,400 步全程）
void Joy_Stress_Test_Poll(void)
{
    if (s_joy_stress_mode == 0) return;

    static uint64_t last_step_us = 0;
    static uint32_t phase_cnt = 0;
    static uint8_t  stage = 0; // 0:X正向长移到+17200, 1:X反向长移到-17200, 2:Y正向长移到+6000, 3:Y反向长移到-9000
    static int16_t  sim_x = 0, sim_y = 0;

    uint64_t now = timer_get_1us();

    // ---------------- 模式 1：摇杆全链路【全行程长移动】压测（10ms 周期） ----------------
    if (s_joy_stress_mode == 1) {
        if (now - last_step_us < 10000) return;
        last_step_us = now;
        phase_cnt++;

        int32_t cur_x = motor_get_cur_step(TMC2660_M2);
        int32_t cur_y = motor_get_cur_step(TMC2660_M1);

        // 防止 int16_t 溢出反转：当模拟推杆接近 ±30000 时，无缝重置基准继续长推
        if (sim_x > 28000 || sim_x < -28000) {
            sim_x = 0;
            last_state.x = 0;
        }
        if (sim_y > 28000 || sim_y < -28000) {
            sim_y = 0;
            last_state.y = 0;
        }

        switch (stage) {
            case 0: // X 轴全行程长移动 -> 正向冲向 +17200（接近正限位但不触发限位）
                sim_x += 350;
                if (cur_x >= 17200) {
                    stage = 1;
                    sim_x = 0; last_state.x = 0;
                    xil_printf("[STRESS_1] X reached +17200, reversing long move to -17200...\r\n");
                }
                break;

            case 1: // X 轴全行程长移动 -> 反向冲向 -17200（跨过 -1500 霍尔沿共 34,400 步）
                sim_x -= 350;
                if (cur_x <= -17200) {
                    stage = 2;
                    sim_x = 0; last_state.x = 0;
                    xil_printf("[STRESS_1] X reached -17200, switching to Y long move...\r\n");
                }
                break;

            case 2: // Y 轴全行程长移动 -> 正向冲向 +6000
                sim_y += 300;
                if (cur_y >= 6000) {
                    stage = 3;
                    sim_y = 0; last_state.y = 0;
                }
                break;

            case 3: // Y 轴全行程长移动 -> 反向冲向 -9000（共 15,000 步）
                sim_y -= 300;
                if (cur_y <= -9000) {
                    stage = 0;
                    sim_y = 0; last_state.y = 0;
                    xil_printf("[STRESS_1] Full XY cycle done, repeating...\r\n");
                }
                break;
        }

        uint8_t pkt[8] = {
            (uint8_t)(sim_x >> 8), (uint8_t)(sim_x & 0xFF),
            (uint8_t)(sim_y >> 8), (uint8_t)(sim_y & 0xFF),
            0, 0, 0, 0
        };
        s_is_synthetic_pkt = 1;
        joystick_process_packet(pkt, 8);
        s_is_synthetic_pkt = 0;
    }
    // ---------------- 模式 2：纯 PL 寄存器【全行程长移动】刷写压测（5ms 周期） ----------------
    else if (s_joy_stress_mode == 2) {
        if (now - last_step_us < 5000) return;
        last_step_us = now;
        phase_cnt++;

        int32_t cur_x = motor_get_cur_step(TMC2660_M2);
        static uint8_t dir = MOTOR_DIR_FORWARD;
        if (cur_x >= 17200)  dir = MOTOR_DIR_BACKWARD;
        if (cur_x <= -17200) dir = MOTOR_DIR_FORWARD;

        // 全行程高速长移 + 5ms 高频动态更新速度与剩余步数
        uint32_t spd = 16000 + (phase_cnt % 9) * 1000; // 16kHz ~ 24kHz
        uint32_t stp = 2000 + (phase_cnt % 53);
        motor_control(TMC2660_M2, dir, spd, stp);
    }
}









// ===================================================================
// 【Shell 扩展槽位实现】
// ===================================================================

/**
 * @brief 槽位 1：命令处理函数
 * @return 1 表示命中本模块命令；0 表示不是本模块命令，交给下一个槽位
 */
int joystick_shell_cmd(int argc, char *argv[])
{
    if (argc == 0 || argv[0] == NULL) return 0;

    // 1. 匹配 "log" 命令
    if (strcmp(argv[0], "log") == 0) {
        if (argc >= 2 && strcmp(argv[1], "status") == 0) {
            xil_printf("[LOG] Joystick log is %s\r\n", s_log_joy_enabled ? "ON" : "OFF");
            return 1;
        }

        if (argc >= 2 && strcmp(argv[1], "joy") == 0) {
            if (argc >= 3) {
                if (strcmp(argv[2], "on") == 0)       s_log_joy_enabled = 1;
                else if (strcmp(argv[2], "off") == 0) s_log_joy_enabled = 0;
            } else {
                s_log_joy_enabled = !s_log_joy_enabled;
            }
            xil_printf("[LOG] Joystick log is now %s\r\n", s_log_joy_enabled ? "ENABLED" : "DISABLED");
            return 1;
        }

        xil_printf("Usage: log joy <on|off> | log status\r\n");
        return 1;
    }

    // 2. 匹配 "joy" 命令
    if (strcmp(argv[0], "joy") == 0) {
        if (argc >= 2 && strcmp(argv[1], "status") == 0) {
            xil_printf("[JOY] Mode: X=%d, Y=%d, Z=%d, Ctrl=%d, Owner=%d, Stress=%d\r\n",
                       m_joystick_mode_x, m_joystick_mode_y, m_joystick_mode_z,
                       m_joystick_ctrl, g_motor_owner, s_joy_stress_mode);
            return 1;
        }
        if (argc >= 3 && strcmp(argv[1], "test") == 0) {
            uint8_t mode = (uint8_t)atoi(argv[2]);
            if (mode == 0) {
                s_joy_stress_mode = 0;
                motor_stop(TMC2660_M1);
                motor_stop(TMC2660_M2);
                xil_printf("[TEST] Stress test STOPPED.\r\n");
            } else if (mode == 1 || mode == 2) {
                m_joystick_ctrl = 1;
                g_motor_owner = MOTOR_OWNER_IDLE;
                s_joy_stress_mode = mode;
                xil_printf("[TEST] Stress Mode %d STARTED! Watching for stall...\r\n", mode);
            } else if (mode == 3) {
                Motor_ConfigRegister(TMC2660_M1, 0x000A0000, 0x000A0000);
                Motor_ConfigRegister(TMC2660_M2, 0x000A0000, 0x000A0000);
                xil_printf("[TEST] M1 & M2 CoolStep DISABLED (Constant full current).\r\n");
            }
            return 1;
        }
        xil_printf("Usage: joy status | joy test <0:stop | 1:full_joy | 2:raw_pl | 3:no_coolstep>\r\n");
        return 1;
    }

    return 0; // 非摇杆模块指令，交给其他槽位
}


/**
 * @brief 槽位 2：帮助提示函数 (在终端敲 help 时自动调用拼接)
 */
void joystick_shell_help(void)
{
    xil_printf("  [Joystick & Delta-Integration]\r\n");
    xil_printf("    log joy <on|off>            : Enable/Disable joystick debug trace\r\n");
    xil_printf("    log status                  : Show joystick log status\r\n");
    xil_printf("    joy status                  : Show joystick mode & axes config\r\n");
}
