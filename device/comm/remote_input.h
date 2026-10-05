#ifndef REMOTE_INPUT_H
#define REMOTE_INPUT_H

#include "struct_typedef.h"

/*
 * 与输入来源无关的遥控快照。
 * 具体工程可由板间链路或本地 DBUS 模块产生；
 * 使用者不需要知道这份输入来自哪条物理链路。快照是值语义：调用者复制后
 * 可以跨越一个任务周期使用，不得把它解释为底层可写句柄。
 */
typedef struct
{
    int16_t ch[4];
    int8_t  s[4];

    uint8_t dbus_lost;
    uint8_t frame_lost;
    uint8_t failsafe;
    uint16_t sequence;
    uint32_t timestamp_ms;
    uint8_t  online;
} remote_input_snapshot_t;

#endif /* REMOTE_INPUT_H */
