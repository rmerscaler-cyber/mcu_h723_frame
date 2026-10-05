#ifndef ANGLECALC_H
#define ANGLECALC_H


#define OPT_HORIZON      6
#define OPT_ACTION_NUM   3

typedef struct
{
    float dt;                  // 优化器周期 s

    float max_speed;           // 最大角速度 rad/s
    float max_accel;           // 最大角加速度 rad/s^2
    float speed_response;      // 每周期实际速度跟随速度参考的比例 (0, 1]

    float q_angle;             // 角度误差权重
    float q_speed;             // 速度权重
    float r_accel;             // 加速度权重

    float q_terminal_angle;    // 终端角度误差权重
    float q_terminal_speed;    // 终端速度权重

} AngleOptimalParam_t;

float AngleOptimal_GetSpeedRef(float angle_error,
                               float speed,
                               float speed_ref,
                               const AngleOptimalParam_t *p);

#endif /* ANGLECALC_H */
