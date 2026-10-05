# Motor 模块设计

本文描述当前 H7 工程的 motor 数据边界。代码行为以 `motor_typedef.h`、
`motor.c` 和 `motor_reg.c` 为准。

## 核心约束

1. `motor_feedback_t` 只承载协议无关的最终物理量；协议原始值由各驱动私有结构体保存，中间层和应用不可见。
2. CAN 驱动只写驱动私有原始值；controller 和 application 状态不再保存反馈字段。
3. 驱动只负责保存原始字段、提供未滤波物理量和命令组帧，不决定任务层滤波策略与 PID 反馈源。
4. Application 的 `feedback_update` 是原始反馈变成最终反馈的唯一任务层入口，并把完整结果覆盖发布到 `motor->fb`。
5. `motor_control_t` 只保存模式、PID 状态、目标值和输出，不保存 `fb_angle/fb_speed` 反馈副本。
6. 同一 application 任务在 `feedback_update` 后直接使用 `motor->fb` 最终区；局部变量只用于复用或中间计算。

## 文件职责

| 文件 | 职责 |
|---|---|
| `motor_typedef.h` | 公共类型、原始/最终反馈、控制状态和注册配置 |
| `motor_reg.c` (device/driver/motor) | 一行一电机的拓扑、标定、PID、反馈源和输出配置 |
| `motor.c` | 注册、协议私有值的帧一致性读取、最终区发布、PID、在线检测和统一组帧入口 |
| `motor_driver.h` | Motor 与具体协议驱动之间的最小接口 |
| `device/driver/motor/dji_motor.c` | DJI 原始值保存、物理量换算和电流命令组帧 |
| `device/driver/motor/lk_motor.c` | LK 原始值保存、物理量换算，以及闭环输出或 RELAY 命令组帧 |
| `motor_utils.h` | 角度包裹与输出限幅工具 |

## 注册表模板

框架不预设底盘、云台、发射或拨盘。`device/driver/motor/motor_reg.c`
当前是空表，具体机器人只需在自己的工程中填写 `motor_reg_t` 数组，并为每个
实例选择 `MOTOR_ID_n`、`MOTOR_BUS_n`、协议驱动、反馈/命令 ID 和控制参数。
上板和下板分别维护自己的注册表，不共享另一块板的电机数量。

同一命令帧内，`cmd_slot` 表示该电机占用的两个字节位置。它只在相同物理总线和相同命令 ID 的组内比较；不同总线可以复用相同反馈 ID、命令 ID 和 slot。

## 反馈数据流

```text
CAN endpoint 回调
    - store_protocol_feedback：只写驱动私有原始值
    - 不修改 motor->fb 的最终区
    ↓
application feedback_update
    - motor_feedback_unfiltered_read：读取一致的未滤波物理量
    - 在局部 motor_feedback_t 上完成滤波/任务层修正
    - 上一周期 motor->fb 可直接作为滤波历史
    - motor_feedback_publish：覆盖发布到 motor->fb
    ↓
motor_calc / application 直接消费 motor->fb 最终区
```

`raw_feedback_version` 标记驱动私有原始值的写入窗口；`feedback_version` 保护最终区。
版本为奇数时表示写入中，为偶数时表示完整快照。
跨任务读取需要使用 `motor_feedback_read()`；同一 application 控制流程直接读取最终字段。

## 最终反馈

协议原始值 (DJI 编码器/rpm/电流, LK 14bit 编码器/deg/s/电流) 只保存在
`dji_motor_t` / `lk_motor_t` 的私有结构体中，由 CAN 中断写入、
`read_unfiltered_feedback` 换算；`motor_feedback_t` 只包含协议无关的物理量：

| 字段 | 单位/语义 |
|---|---|
| `angle` | rad，单圈角度 `[-pi, pi]`；DJI 为输出轴单圈角度 |
| `total_angle` | rad，累计角度；DJI 为输出轴连续累计角度 |
| `speed_angle` | rad/s，输出端角速度；协议层已除以注册减速比 |
| `speed_ms` | m/s，轮缘线速度；无轮机构为 0 |
| `current` | mA |
| `torque` | N·m；无估算能力时为 0 |
| `temperature` | 摄氏度 |
| `timestamp_ms` | 当前快照接收时间 |
| `sequence` | 有效反馈帧序号 |
| `valid` | application 是否已经发布一份结构完整的最终反馈；不代表电机当前在线 |

DJI 协议层在接收每帧时累计转子编码器增量，在读取物理反馈时统一执行
`累计计数 × 2π / 8192 × direction / reduction`。`angle` 是同一坐标的输出轴
单圈角度，`speed_angle` 是该坐标的角速度；直驱实例的减速比必须填 `1.0f`。
`offset_ecd` 仍是转子原始编码器计数。首帧以 `ecd - offset_ecd` 建立坐标，
上电前的转子圈数和减速器输出轴绝对相位无法由单圈转子编码器恢复；单发应以
触发时的 `total_angle` 加相对拨弹角度作为目标。
跨圈解算要求相邻有效反馈帧之间转子位移小于半圈，长时间断帧后的累计位置
不能保证正确。对外累计角度仍为 `fp32`，长期连续转动后细小角度差的精度会降低。

框架不规定滤波周期、滤波器或反馈源。具体机构的 application 可以对
`motor_feedback_unfiltered_read()` 的结果做任务层处理，再通过
`motor_feedback_publish()` 发布；无特殊需求时直接使用 `speed_angle`。

`valid` 只说明最终区已经完成一次发布。当前底盘在电机尚未收到反馈或已经离线时，也会发布一份 `valid = 1` 的零反馈快照。因此：

- 判断电机是否在线必须调用 `motor_is_online()`；
- `sequence` 和 `timestamp_ms` 用于观察真实反馈进度；
- 不能把 `valid` 单独作为电机在线或 CAN 正常的依据。

## 控制数据流

```text
application: motor_set_ref(handle, ref)
    ↓
motor_calc(handle)
    - 检查 enabled / online
    - 从最终 motor_feedback_t 选择反馈量
    - SPEED：速度单环
    - POSITION：角度→速度串级
    - RELAY：ref 透传到协议输出
    - 输出限幅后写入 ctrl.output
    ↓
application 可执行功率限制等后处理：motor_set_output
    ↓
motor_cmd_pack(handles, count)
```

控制层字段的所有权：

| 字段 | 写入者 | 说明 |
|---|---|---|
| `ctrl.ref` | application | 当前目标值 |
| `ctrl.angle_pid/speed_pid` | motor.c | PID 历史和诊断状态 |
| `ctrl.output` | motor.c，随后可由安全/功率逻辑覆盖 | 最终协议输出输入 |
| 驱动私有原始值 | CAN 驱动 | 协议原始反馈，仅驱动内部换算使用 |
| `fb` 最终区 | 对应 application 的 `feedback_update` | 后续计算与 PID 的唯一反馈 |

## Application 使用规则

CAN 中断只写驱动私有原始值，因此同一 application 在 `feedback_update` 返回后可以直接
读取最终字段。不要为了单次读取复制整个反馈结构；只有需要反复变换的中间计算
才建立局部变量。跨任务或异步调试读取才使用 `motor_feedback_read()`。

```c
fp32 angle_error = rad_format(target - motor->fb.angle);
/* angle_error 是中间计算；最终目标仍写回 ctrl.ref。 */
motor_set_ref(motor, target);
```

不要在机构控制结构中增加 `feedback`、`fb_angle`、
`fb_speed`、`steer_angle[]` 等长期镜像字段。真正属于机构层的新派生状态可以保存，
但名称和含义必须与电机反馈不同，并明确唯一写入者。

## 新增电机

首先判断新增电机是否使用工程已经支持的协议。新增电机实例和新增电机协议是两类不同修改，不应混在一起。

### 使用已有协议新增实例

1. 在 `motor_id_e` 中选择尚未使用的 `MOTOR_ID_n` 编号。
2. 在 `device/driver/motor/motor_reg.c` 填写已有 driver、总线、CAN ID、控制模式、标定、PID 和反馈策略。
3. 检查同一物理总线、同一命令 ID 内的 `cmd_slot` 是否唯一。
4. Application 初始化时调用 `motor_register()` 并保存 handle；运行期只使用 handle。
5. 对应 application 的 `feedback_update` 调用 `motor_feedback_unfiltered_read()`，完成任务层处理后调用 `motor_feedback_publish()`。
6. 在同一 application 任务中读取 `motor->fb` 最终区，并使用 `motor_set_ref()`、`motor_calc()` 和 `motor_cmd_pack()` 完成控制。
7. 检查具体驱动的静态实例容量。当前 DJI 和 LK 驱动分别受 `DJI_MOTOR_STATE_MAX`、`LK_MOTOR_STATE_MAX` 限制；只有数量超过容量时才调整容量，不需要重新实现协议函数。

### 引入新的电机协议

1. 新增对应的驱动类型和驱动文件，不在 application 中直接解析协议。
2. 实现 `motor_driver_ops_t` 要求的 `register_instance()`、`store_protocol_feedback()`、`read_unfiltered_feedback()`、`pack_commands()`、`enable()` 和 `disable()`。
3. `store_protocol_feedback()` 只保存协议原始值和驱动私有的跨帧状态，不写最终反馈区。
4. `read_unfiltered_feedback()` 将私有原始值换算为统一 `motor_feedback_t` 物理量，不决定 application 的滤波和机构策略。
5. 在 `device/driver/motor/motor_reg.c` 中把新电机实例绑定到新的 driver ops，再按已有实例流程注册和使用。

新增字段前先判断：它是单帧中间量、跨帧算法状态、最终反馈、目标值，还是输出。
只有跨周期确有意义的数据才进入长期结构体。
