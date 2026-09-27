/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2026-09-27 14:56:18
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2026-09-27 15:21:05
 * @FilePath: \rmproject1\User\ControlTask.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "ControlTask.h"
#include "can.h"
#include "tim.h"
#include "Gm6020.h"
#include "Pid.h"
#include "main.h"   // 包含 hcan1, htim6 等句柄声明
#include <cmath>

// ================== 自定义正弦波参数 ==================
#define PI 3.1415926535f

// 控制模式：0 = 速度跟随，1 = 位置跟随
// 建议你先跑速度模式，调好PID后再切位置模式
uint8_t control_mode = 0; 

// 正弦波频率 (Hz)，不要设太大，0.2Hz 起步
constexpr float kFreq = 0.2f; 

// 速度环正弦波参数 (单位: rad/s)
constexpr float kSpeedAmplitude = 5.0f; 

// 位置环正弦波参数 (单位: rad，注意GM6020角度为连续弧度，多圈累加)
constexpr float kPosAmplitude = 1.5f; 
constexpr float kPosOffset    = 0.0f; 

// ================== 实例化对象 ==================
// 1. 实例化电机对象，假设电机ID为 1（请根据拨码开关实际ID修改）
Gm6020 motor(1);

// 2. 实例化PID对象
// 构造函数参数: Kp, Ki, Kd, 输出上限, 输出下限
// GM6020 电压控制量范围为 -25000 ~ +25000
Pid speed_pid(150.0f, 5.0f, 0.0f, 25000.0f, -25000.0f); // 速度环初始参数
Pid pos_pid(100.0f, 0.0f, 0.0f, 25000.0f, -25000.0f);   // 位置环初始参数

// ================== 任务函数实现 ==================

// 上电初始化: 在 main.c 中 MX_XXX_Init() 全部执行完后调用一次
void ControlTaskInit(void) {
    // 1. 启动 CAN 外设
    HAL_CAN_Start(&hcan1); // 根据你的实际情况，可能是 hcan1 或 hcan2

    // 2. 开启 CAN 接收中断 (FIFO0)
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    // 3. 启动 1kHz 定时器中断 (TIM6)
    HAL_TIM_Base_Start_IT(&htim6); // 确保你配的是 TIM6

    // 4. 可选：重置 PID 状态
    speed_pid.reset();
    pos_pid.reset();
}

// 1kHz 控制入口: 在 TIM6 周期中断(HAL_TIM_PeriodElapsedCallback)中调用
void MainTask(void) {
    static float time_sec = 0.0f; // 静态变量记录时间
    time_sec += 0.001f;           // 1kHz 中断，每次累加 1ms
    if (time_sec > 10000.0f) time_sec = 0.0f; // 防止时间溢出

    float pid_output = 0.0f;

    // 1. 获取电机反馈值
    float actual_vel = motor.vel();      // 获取角速度 (rad/s)
    float actual_pos = motor.angle();    // 获取累计角度 (rad)

    // 2. 根据控制模式计算正弦波目标值并运行 PID
    if (control_mode == 0) {
        // 速度跟随模式
        float target_speed = kSpeedAmplitude * std::sin(2.0f * PI * kFreq * time_sec);
        pid_output = speed_pid.calc(target_speed, actual_vel);
    } else {
        // 位置跟随模式
        float target_pos = kPosOffset + kPosAmplitude * std::sin(2.0f * PI * kFreq * time_sec);
        pid_output = pos_pid.calc(target_pos, actual_pos);
    }

    // 3. 将 PID 输出转换为电压给定，发送给电机
    motor.setVoltage(static_cast<int16_t>(pid_output));

    // 4. 通过 CAN 发送控制帧
    uint8_t tx_data[8] = {0};
    motor.encode(tx_data); // 将本电机的电压值填入数组

    CAN_TxHeaderTypeDef tx_header;
    tx_header.StdId = motor.txId(); // 0x1FF 或 0x2FF
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    uint32_t tx_mailbox;
    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox);
}

// CAN 接收回调: 在 HAL_CAN_RxFifo0MsgPendingCallback 中调用
void CanFeedbackCallback(uint32_t std_id, const uint8_t *data) {
    // 判断收到的报文是不是本电机的反馈
    if (std_id == motor.rxId()) {
        motor.decode(data); //111 解析反馈数据，更新角度、速度等
    }
}