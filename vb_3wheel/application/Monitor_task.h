#ifndef MONITOR_TASK_H_
#define MONITOR_TASK_H_


typedef struct
{

    volatile enum roboState {
        DEAD = 0,            // 机器人死亡
        INITIALIZING = 1,    // 初始化
        ON_PROCESSING = 2,   // 运行中
        WELL_PROCESSING = 3, // 处理完毕
        DBUS_ERROR = 4,      // 遥控器出错
        MOTOR_ERROR = 5,     // 电机出错
        JUDGE_ERROR = 6,     // 裁判系统出错
        CAP_ERROR = 7,       // 电容控制板出错
        PC_ERROR = 8,        // PC出错
        NOMAL = 9,           // 一切正常
    } roboState;             // 机器人状态

    volatile enum roboMode {
        MODE_STOP      = 0,    // 停止模式
        MODE_ARTIFICAL = 1,    // 人工模式
        MODE_AUTO      = 2,    // 自动模式

        ARTIFICAL_CHASSIS = 3, // 人工模式下，机器人底盘
        ARTIFICAL_BAT     = 4, // 人工模式下，机器人球拍
        ARTIFICAL_STRIKER = 5, // 人工模式下，机器人击球杆

    } roboMode; // 机器控制模式

} s_robo_Mode_Setting;


extern const s_robo_Mode_Setting *get_robot_mode_pint(void);

#endif /* MONITOR_TASK_H_ */
