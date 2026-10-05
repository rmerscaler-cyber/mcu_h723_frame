# H7 Robot Framework

这是 H7 下板的通用任务框架，不预设具体机器人类型和机构玩法。框架只保留
所有项目都可能需要的启动、FreeRTOS 任务入口、IMU、遥控输入、裁判通信、LED、
检测任务、通用电机协议和 HAL/BSP 适配。

## 已留出的扩展点

- `app/robot_application.c`：应用层初始化和周期入口；在这里接入本机器人的任务。
- `model/cmd/`：统一的中间层命令结构；当前默认安全空实现，命令来源由具体项目接入。
- `device/driver/motor/motor_reg.c`：本板电机注册表；当前为空表，上板和下板各自注册。
- `device/driver/motor/motor_port.c`：把逻辑 `MOTOR_BUS_n` 映射到本板实际 CAN。
- `device/tasks/`：保留板级公共任务；机器人专属任务直接在具体项目新增并加入 CMake。

## 有意移除的内容

底盘、云台跟随、拨弹/发射、超级电容、视觉角度控制、板间游戏协议、底盘功率
监控和校准任务都不属于所有机器人共有能力，已从此框架的 CMake 和目录中移除。
需要这些功能的工程应在自己的应用层、设备层中实现，不要把产品级编号和策略写回
框架公共接口。

## 构建

```text
cmake --preset Debug
cmake --build --preset Debug
```

具体工程开始实现后，再填写 `motor_reg.c`、命令采集端口和 `robot_application_step()`。
