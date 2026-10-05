#include "anglecalc.h"

#include <math.h>
#include <float.h>
#include <stddef.h>
#include <stdint.h>

/*
 * 三种离散控制动作：
 *
 * -amax  减速/反向加速
 *  0     保持速度
 * +amax  正向加速
 */
static const float action_ratio[OPT_ACTION_NUM] =
{
    -1.0f,
     0.0f,
    +1.0f
};

/* 求出总控制序列数量。用于离散化目标 */
static uint32_t IntPow(uint32_t base, uint32_t exp)
{
    uint32_t result = 1;

    while (exp--)
        result *= base;

    return result;
}


/*
 * 输入：
 * angle_error
 *      target_angle - current_angle
 * speed:当前真实角速度 rad/s
 * speed_ref:当前送给速度 PID 的角速度参考值 rad/s
 * 输出：
 * 最优序列第一步对应的下一拍角速度参考值。
 */
float AngleOptimal_GetSpeedRef(
    float angle_error,
    float speed,
    float speed_ref,
    const AngleOptimalParam_t *p)
{
    if (p == NULL || p->dt <= 0.0f || p->max_speed <= 0.0f ||
        p->max_accel <= 0.0f || p->speed_response <= 0.0f ||
        p->speed_response > 1.0f)
        return 0.0f;

    /*
     * 总控制序列数量：3^N
     * N = 6 时有729种
     */
    const uint32_t sequence_num = IntPow(OPT_ACTION_NUM, OPT_HORIZON);

    float best_cost = FLT_MAX;
    float best_first_speed_ref = speed_ref;

    for (uint32_t seq = 0; seq < sequence_num; seq++)
    {
        uint32_t code = seq;

        /*
         * 从当前真实状态开始预测
         */
        float e = angle_error;
        float w = speed;
        float w_ref = speed_ref;
        float cost = 0.0f;
        float first_speed_ref = speed_ref;
        uint8_t feasible = 1;

        for (uint32_t k = 0; k < OPT_HORIZON; k++)
        {
            /*
             * 从三进制序列中取出这一拍的动作
             */
            uint32_t action = code % OPT_ACTION_NUM;
            code /= OPT_ACTION_NUM;

            float a = action_ratio[action] * p->max_accel;
            float next_w_ref = w_ref + a * p->dt;
            float next_w;
            float next_e;

            /*
             * 动作先改变速度 PID 参考值；实际速度按已调速度环的一阶响应跟随。
             * speed_response=1 表示一拍完全跟随，越小表示速度环/机械响应越慢。
             */
            if (fabsf(next_w_ref) > p->max_speed)
            {
                feasible = 0;
                break;
            }
            next_w = w + p->speed_response * (next_w_ref - w);
            next_e = e - 0.5f * (w + next_w) * p->dt;

            if (k == 0)
                first_speed_ref = next_w_ref;

            /* J_k = Qe * e² + Qw * w² + Ra * a² */
            cost += p->q_angle * next_e * next_e
                  + p->q_speed * next_w * next_w
                  + p->r_accel * a * a;
            /* 更新状态 */
            e = next_e;
            w = next_w;
            w_ref = next_w_ref;
        }


        if (!feasible)
            continue;


        /*
         * 终端性能指标
         * e -> 0
         * w -> 0
         */
        cost += p->q_terminal_angle * e * e
              + p->q_terminal_speed * w * w;
        /*
         * 保存当前最优控制序列
         */
        if (cost < best_cost)
        {
            best_cost = cost;
            best_first_speed_ref = first_speed_ref;
        }
    }

    if (best_first_speed_ref > p->max_speed)
        best_first_speed_ref = p->max_speed;

    else if (best_first_speed_ref < -p->max_speed)
        best_first_speed_ref = -p->max_speed;


    return best_first_speed_ref;
}
