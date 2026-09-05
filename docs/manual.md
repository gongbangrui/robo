# 智能车调试手册



# 一、命令行速查

串口连接开发板（115200 8N1），输入 `help` 查看所有命令。

## 系统状态
| 命令 | 说明 |
|------|------|
| `state` | FSM状态、传感器位置、导航进度 |
| `battery` | 电池电压 + 电量百分比 |

## 传感器
| 命令                      | 说明             |
| ----------------------- | -------------- |
| `gray` / `grayonce`     | 灰度传感器（连续/单次）   |
| `imu` / `imuonce`       | IMU姿态角、陀螺仪、加速度 |
| `motor` / `motoronce`   | 四轮电机RPM        |
| `ir` / `ironce`         | 前/左/右/备红外传感器   |
| `vision tl|qr|digit`    | 视觉模块测试            |

## 参数调节
| 命令 | 说明 |
|------|------|
| `pid [Kp Ki Kd]` | 查询/设置循线PID |
| `speed [值]` | 查询/设置巡线速度（0.1~1.5 m/s） |
| `slow [值]` | 查询/设置路口慢速（0.1~1.5 m/s） |

## 路线
| 命令 | 说明 |
|------|------|
| `path` | 列出所有可用路线 |
| `path <名称>` | 切换路线 |

## 控制
| 命令 | 说明 |
|------|------|
| `stop` | 紧急停车 |
| `start` | 恢复运行 |
| `reset` | 系统复位 |

## 测试
| 命令 | 说明 |
|------|------|
| `buzzer` | 蜂鸣器测试 |
| `servo <ch> <us>` | 舵机测试（ch:0-2, us:500-2500） |
| `voice <track>` | 语音模块播放音轨（0-15） |

## 调试开关
| 命令 | 说明 |
|------|------|
| `trace on/off` | FSM状态转换日志 |
| `log on/off` | 传感器/电机连续输出 |
| `clear` | 清屏 |



# 二、速度与 PID 调参

## 速度调节

两个速度，运行时用命令改，无需编译：

| 速度 | 命令 | 默认值 | 说明 |
|------|------|--------|------|
| 巡线速度 | `speed` | 1.0 m/s | 正常巡线时的前进速度 |
| 路口慢速 | `slow` | 0.2 m/s | 进路口和转弯时的速度 |

```
speed 1.0     # 设置巡线速度
slow 0.3      # 设置路口慢速
```

直行过路口时不降速，用巡线速度直接冲过去。只有转弯才降为路口慢速。

---

## 循线 PID

| 参数 | 默认值 | 作用 |
|------|--------|------|
| **Kp** | 1.5 | 比例：纠偏力度。大了猛，小了慢 |
| **Ki** | 0.001 | 积分：消除长期偏置。大了反应迟钝 |
| **Kd** | 0.8 | 微分：抑制震荡。大了方向抖 |

运行时调参：

```
pid               # 查看当前值
pid 1.5 0.001 0.8 # 设置
```

### 调参流程

1. `speed 0.3` 降速，`trace on` 开日志
2. 观察车的巡线行为：

| 现象 | 调法 |
|------|------|
| 左右来回摇摆 | 减小 Kp，加大 Kd |
| 反应慢，线跑远了才纠正 | 加大 Kp |
| 一直偏在某一侧 | 加大 Ki |
| 方向高频抖动 | 减小 Kd |

3. 反复试到满意，最终值写入 `line_track_task.h` 的 `LT_KP/LT_KI/LT_KD`

---

## 电机速度 PID

文件：`chassis_task.h`

| 参数 | 默认值 | 说明 |
|------|--------|------|
| M3505\_MOTOR\_SPEED\_PID\_KP | 15000 | 速度响应 |
| M3505\_MOTOR\_SPEED\_PID\_KI | 10 | 稳态误差消除 |
| M3505\_MOTOR\_SPEED\_PID\_KD | 0 | 一般不需要 |
| M3505\_MOTOR\_SPEED\_PID\_MAX\_OUT | 10000 | 最大电流输出 |

一般不需要调电机PID，除非换电机或换轮子。

---

## 底盘常数

文件：`chassis_task.h`

| 参数 | 默认值 | 说明 |
|------|--------|------|
| WHEEL\_BASE | 0.35 m | 左右轮距 |
| M2006\_MOTOR\_RPM\_TO\_VECTOR | 1.57e-4 | RPM→m/s 转换（M2006 36:1 + 108mm轮） |

换了轮径或电机需要改这两个。



# 三、死推算参数（转弯调参）

## 概念

车穿越路口时靠"死推算"——不看线，纯靠陀螺仪和速度把自己推出路口。

文件：`application/line_track_task.c` → `DR_TABLE[]`

每个动作三个参数：

| 参数 | 含义 |
|------|------|
| `vx` | 前向速度（m/s）。VX\_SLOW=用slow速度，VX\_BASE=用speed速度 |
| `wz` | 角速度（rad/s）。正=左转，负=右转 |
| `yaw_th` | 陀螺仪转到多少度算完成。0=不用陀螺仪，靠传感器线重获判出 |

---

## 默认参数

| 动作 | vx | wz | yaw\_th |
|------|-----|-----|--------|
| 直行 FORWARD | VX\_BASE | 0 | 0° |
| 左转 TURN\_LEFT | VX\_SLOW | +1.0 | 80° |
| 右转 TURN\_RIGHT | VX\_SLOW | -1.5 | 80° |
| 掉头 U\_TURN | 0.15 | +3.0 | 160° |
| 左绕 AROUND\_LEFT | 0.2 | +1.0 | 120° |
| 右绕 AROUND\_RIGHT | 0.2 | -1.0 | 120° |
| 锐角左 SHARP\_LEFT | 0.15 | +2.5 | 30° |
| 锐角右 SHARP\_RIGHT | 0.15 | -2.5 | 30° |
| 钝角左 OBTUSE\_LEFT | 0.25 | +0.8 | 120° |
| 钝角右 OBTUSE\_RIGHT | 0.25 | -0.8 | 120° |
| 岔路左 FORK\_LEFT | 0.15 | +2.0 | 80° |
| 岔路右 FORK\_RIGHT | 0.15 | -2.0 | 80° |
| 上平台 CLIMB\_PLATFORM | VX\_SLOW | 0 | 0° |
| 过桥 CROSS\_BRIDGE | VX\_SLOW | 0 | 0° |

---

## 直行（yaw\_th = 0）

直行不用陀螺仪。车进入路口中心（传感器全黑），然后向前走，等到黑线重获（能看到线了），就算穿过路口了。

`VX_BASE` 哨兵值表示用 `speed` 命令的速度，所以直行不降速。

---

## 转弯（yaw\_th > 0）

转弯用陀螺仪。车进路口后开始旋转，陀螺仪累积转到设定角度就算完成。

**触发条件**：转弯动作看到目标方向的分支线，立刻触发（不等消抖）。

比如左转：传感器检测到路口 + 左侧有分支线 → 立刻进 CROSS 开始转。

---

## 调转弯

| 问题 | 改什么 | 示例 |
|------|--------|------|
| 转弯转不够 | 加大 yaw\_th | 80° → 90° |
| 转弯转过头 | 减小 yaw\_th | 80° → 70° |
| 转弯太慢 | 加大 vx | 0.2 → 0.3 |
| 转弯太猛冲出线 | 减小 wz 绝对值 | 1.5 → 1.0 |
| 锐角转不过去 | 加大 wz | 2.5 → 3.0 |
| 钝角转回来太快 | 减小 wz | 0.8 → 0.5 |

改完重新编译烧录。



# 四、路线编写

## 概念

路线是一串动作，车每过一个路口执行下一个动作，直到 STOP 停车。

---

## 运行时切换（无需编译）

```
path              # 列出所有路线
path basic        # 切换到基础路线
path complex      # 切换到复杂路线
```

预设路线：

| 名称 | 步骤数 | 内容 |
|------|--------|------|
| basic | 4 | FORWARD → TURN\_LEFT → FORWARD → STOP |
| sharp | 4 | FORWARD → SHARP\_LEFT → FORWARD → STOP |
| obtuse | 4 | FORWARD → OBTUSE\_RIGHT → FORWARD → STOP |
| uturn | 4 | FORWARD → U\_TURN → FORWARD → STOP |
| fork | 4 | FORWARD → FORK\_LEFT → FORWARD → STOP |
| complex | 10 | 含多种路口 |
| platform | 4 | FORWARD → CLIMB\_PLATFORM → FORWARD → STOP |
| bridge | 4 | FORWARD → CROSS\_BRIDGE → FORWARD → STOP |

---

## 添加新路线（运行时）

编辑 `application/debug_console.c`，找到 `test_paths[]`，按格式添加：

```c
// 1. 定义路径数组
static const navigate_action_t path_mine[] = {
    NAV_ACTION_FORWARD,
    NAV_ACTION_TURN_LEFT,
    NAV_ACTION_CLIMB_PLATFORM,
    NAV_ACTION_FORWARD,
    NAV_ACTION_STOP,
};

// 2. 在 test_paths[] 中注册
{"mine", path_mine, sizeof(path_mine) / sizeof(path_mine[0])},
```

然后重新编译。之后就可以用 `path mine` 切换了。

---

## 编译时写路线

编辑 `Src/main.c`，修改 `demo_path`：

```c
static const navigate_action_t demo_path[] = {
    NAV_ACTION_FORWARD,         // 第1步: 直行
    NAV_ACTION_TURN_LEFT,       // 第2步: 左转
    NAV_ACTION_CROSS_BRIDGE,    // 第3步: 过桥
    NAV_ACTION_FORWARD,         // 第4步: 直行
    NAV_ACTION_STOP,            // 终点 (必须)
};
```

---

## 完整动作表

### 基本动作
| 指令 | 值 | 含义 |
|------|-----|------|
| NAV\_ACTION\_FORWARD | 0 | 直行通过路口 |
| NAV\_ACTION\_TURN\_LEFT | 1 | 左转90° |
| NAV\_ACTION\_TURN\_RIGHT | 2 | 右转90° |
| NAV\_ACTION\_STOP | 3 | 停车（路线终点） |

### 特殊转向 (10-31)
| 指令 | 值 | 含义 |
|------|-----|------|
| NAV\_ACTION\_U\_TURN | 10 | 180°掉头 |
| NAV\_ACTION\_AROUND\_LEFT | 11 | 大弯左转 |
| NAV\_ACTION\_AROUND\_RIGHT | 12 | 大弯右转 |
| NAV\_ACTION\_SHARP\_LEFT | 20 | 锐角左转 |
| NAV\_ACTION\_SHARP\_RIGHT | 21 | 锐角右转 |
| NAV\_ACTION\_OBTUSE\_LEFT | 30 | 钝角左转 |
| NAV\_ACTION\_OBTUSE\_RIGHT | 31 | 钝角右转 |

### 岔路 (40-44)
| 指令 | 值 | 含义 |
|------|-----|------|
| NAV\_ACTION\_FORK\_LEFT | 40 | 岔路走左 |
| NAV\_ACTION\_FORK\_RIGHT | 41 | 岔路走右 |
| NAV\_ACTION\_FORK\_CENTER | 42 | 岔路走中 |

### 速度控制 (50-51)
| 指令 | 值 | 含义 |
|------|-----|------|
| NAV\_ACTION\_SLOW\_DOWN | 50 | 减速 |
| NAV\_ACTION\_SPEED\_UP | 51 | 加速 |

### 场地任务 (80-81)
| 指令 | 值 | 含义 |
|------|-----|------|
| NAV\_ACTION\_CLIMB\_PLATFORM | 80 | 上平台（自动执行全套动作） |
| NAV\_ACTION\_CROSS\_BRIDGE | 81 | 过桥（红外纠偏+IMU监控） |

> 最后一步**必须是** `NAV_ACTION_STOP`。



# 五、平台与桥梁

## 上平台 (CLIMB\_PLATFORM)

车子自动执行四个阶段：

```
直行 → 前红外触发 → 停车+舵机推杆 → 等视觉识别数字 → 语音报数 → 继续
```

### 可调参数
文件：`application/platform_exec.c`

| 参数 | 默认值 | 说明 |
|------|--------|------|
| PLAT\_DRIVE\_SPEED | 0.15 m/s | 接近平台的速度 |
| PLAT\_DRIVE\_TIMEOUT\_MS | 5000 | 前红外超时（5秒内必须碰到） |
| PLAT\_SERVO\_HOLD\_MS | 800 | 舵机推出后保持时间 |
| PLAT\_VISION\_TIMEOUT\_MS | 3000 | 等视觉模块返回数字的超时 |

### 硬件对应

| 硬件 | 引脚 | 用途 |
|------|------|------|
| 前红外传感器 | PE9 | 检测碰到平台边缘 |
| 舵机 | PC6 (TIM8 CH1) | 推杆动作 |
| 语音模块 | PB12-15 (4路GPIO) | 播报数字 |
| 视觉模块 | USART6 | 红绿灯/二维码/数字识别 |

---

## 过桥 (CROSS\_BRIDGE)

车子在桥上慢速前进，左右红外检测红色桥边自动纠偏，IMU 监控倾斜判断是否过完桥。

### 可调参数
文件：`application/bridge_exec.c`

| 参数 | 默认值 | 说明 |
|------|--------|------|
| BRIDGE\_BASE\_SPEED | 0.12 m/s | 过桥速度 |
| BRIDGE\_STEER\_GAIN | 0.8 | 红外纠偏力度（大了晃，小了偏） |
| BRIDGE\_PITCH\_THRESH\_DEG | 3.0° | 俯仰角多小算"水平" |
| BRIDGE\_LEVEL\_HOLD\_MS | 500 | 水平状态持续多久算过完 |
| BRIDGE\_LINE\_LOST\_TIMEOUT | 3000 | 桥上丢线超时（超时放弃） |

### 纠偏逻辑

- 左红外检测到红色 → 向右微调（wz > 0）
- 右红外检测到红色 → 向左微调（wz < 0）
- 两个都没看到或都看到了 → 直行

### 完成判定

陀螺仪俯仰角（pitch）回到与刚上桥时相同的水平状态，且持续 500ms，同时传感器重新看到黑线 → 判定过桥完成。

### 硬件对应

| 硬件 | 引脚 | 用途 |
|------|------|------|
| 左红外传感器 | PE11 | 检测桥左侧红色边缘 |
| 右红外传感器 | PE13 | 检测桥右侧红色边缘 |
| IMU | 已内置 | 检测俯仰角变化 |

---

## 调参要点

**上平台常见问题**：

| 问题 | 改什么 |
|------|--------|
| 红外触发太晚或太早 | 调整传感器安装位置 |
| 舵机推出力度不够 | 调 servo 脉宽 `application/servo_ctrl.h` |
| 视觉识别超时 | 加大 PLAT\_VISION\_TIMEOUT\_MS |
| 没到平台线就丢了 | 加大 PLAT\_DRIVE\_TIMEOUT\_MS |

**过桥常见问题**：

| 问题 | 改什么 |
|------|--------|
| 桥面红外误触发 | 调整传感器阈值或安装高度 |
| 纠偏太猛来回晃 | 减小 BRIDGE\_STEER\_GAIN |
| 纠偏不够偏出去 | 加大 BRIDGE\_STEER\_GAIN |
| 桥没走完就判完成的 | 加大 BRIDGE\_PITCH\_THRESH\_DEG 或 LEVEL\_HOLD\_MS |



# 六、日志与故障排查

## 蜂鸣器

| 音效 | 含义 |
|------|------|
| 升调旋律 C→E→G→C | 系统启动完成 / 校准完成 |
| 急促四短鸣 G-G-G-G | 电池电压过低（6S < 21V），每5秒重复 |

电池电压用 `battery` 命令查看。

---

## 日志格式（trace on）

```
[TRACE]  12345 ms  TRACKING -> APPROACH
         reason=intersection pos=+0.15 bm=0x01C0 lc=4
[TRACE]  itype=T_LEFT exp_b=1 act=1 dr_vx=0.20 dr_wz=1.00 yaw_th=80
```

### 第一行：状态转换

| 字段 | 含义 |
|------|------|
| `12345 ms` | 系统运行时间 |
| `TRACKING -> APPROACH` | 从巡线→接近路口 |
| `reason=intersection` | 原因：检测到路口 |
| `pos=+0.15` | 黑线位置（0=中心，-1=最左，+1=最右） |
| `bm=0x01C0` | 16通道位掩码（调试用） |
| `lc=4` | 看到4个通道有黑线 |

### 第二行：路口信息

| 字段 | 含义 |
|------|------|
| `itype` | 路口类型：BLOB/T\_LEFT/T\_RIGHT/CROSS/MULTI |
| `exp_b` | 期望方向：0=直行, 1=左, 2=右 |
| `act` | 当前动作编号 |
| `dr_vx/dr_wz` | 死推算速度和角速度 |
| `yaw_th` | 陀螺仪角度阈值 |

---

## 五种状态

| 状态 | 说明 | 进入条件 | 退出条件 |
|------|------|---------|---------|
| `LOST_LINE` | 丢线 | 传感器看不到线 | 重新看到线 |
| `TRACKING` | 正常巡线 | 看到线 | 检测到路口 |
| `APPROACH` | 接近路口 | 传感器检测到分支 | 确认路口或误报 |
| `CROSS` | 穿越路口 | 确认路口 | 转弯完成/线重获/超时 |
| `STOP` | 停车 | NAV\_ACTION\_STOP | 新动作触发重启 |

---

## 常见故障

### 车不走直线、原地转圈

1. 检查电机接线方向 → 发 `motoronce` 看四轮 RPM
2. PID 方向有问题 → `pid 1.0 0 0` 纯比例测试
3. 右边电机 RPM 应该和左边同符号（一起正或一起负）

### 路口不识别

1. 灰度传感器脏污 → 清洁传感器表面
2. 传感器阈值不对 → 检查 `gray_sensor_init()` threshold（默认80）
3. 开启 `trace on` 看是否检测到路口

### 转弯过头或不够

1. 转弯过头 → 减小对应动作的 `yaw_th`
2. 转弯不够 → 加大对应动作的 `yaw_th`
3. 看日志中 `dr_wz` 是否符合预期

### 直行路口秒退

正常现象。横线路口传感器没断线，forward executor 直接判完成。如果没过完就退，检查线宽。

### 上电卡死

1. 检查 TICK\_INT\_PRIORITY 是否为 0
2. 检查电池电压是否 > 21V
3. 重新上电

### 串口无输出

1. 检查 USB CDC 是否正常枚举
2. 尝试用 USART6 (PG14) 直连
3. 重新上电

### 蜂鸣器持续响

1. 电池低电压报警 → `battery` 确认，充电
2. 如果电压正常还响 → 检查校准宏是否清理干净

### PID 命令不生效

检查 Makefile 链接标志是否包含 `-u _scanf_float`。
