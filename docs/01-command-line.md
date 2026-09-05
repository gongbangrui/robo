# 智能车调试手册

## 命令行

串口连接开发板（USART6, 115200 8N1），输入 `help` 查看所有命令。

### 系统状态
| 命令 | 说明 |
|------|------|
| `state` | FSM状态、传感器位置、导航进度 |
| `battery` | 电池电压 + 电量百分比 |

### 传感器
| 命令 | 说明 |
|------|------|
| `gray` / `grayonce` | 灰度传感器（连续/单次） |
| `imu` / `imuonce` | IMU姿态角、陀螺仪、加速度 |
| `motor` / `motoronce` | 四轮电机RPM |
| `ir` / `ironce` | 前/左/右/备红外传感器 |

### 参数调节
| 命令 | 说明 |
|------|------|
| `pid [Kp Ki Kd]` | 查询/设置循线PID |
| `speed [值]` | 查询/设置巡线速度 |
| `slow [值]` | 查询/设置路口慢速 |

### 路线
| 命令 | 说明 |
|------|------|
| `path` | 列出所有可用路线 |
| `path <名称>` | 加载并运行指定路线 |
| `act` | 列出所有可用动作 |
| `act <名称>` | 执行单个动作（见动作表） |

### 视觉系统
| 命令 | 说明 |
|------|------|
| `vision tl` | 手动发红绿灯检测请求 (3s 超时) |
| `vision qr` | 手动发二维码扫描请求 |
| `vision digit` | 手动发数字识别请求 |
| `vision results` | 列出之前所有存储的视觉结果 |
| `vt tl [期望值]` | 视觉测试: 发请求, 回显; 带期望值则比对, 匹配蜂鸣 |
| `vt qr [a b c]` | 二维码测试: 期望 qr[0..2]; 可给 1~3 个值, 全匹配蜂鸣 |
| `vt digit [期望值]` | 数字识别测试: 期望 0~9, 匹配蜂鸣 |
| `vto` | 查看视觉应答超时 (默认 3000ms) |
| `vto <ms>` | 设置视觉应答超时 (100~30000ms) |
| `act tl` | 路径模式: 加载 {红绿灯, STOP} |
| `act qr` | 路径模式: 加载 {二维码, STOP} |
| `act digit` | 路径模式: 加载 {数字识别, STOP} |

> **`vt` 用法示例**: `vt qr 1 2 3`（期望 qr[0]=1, qr[1]=2, qr[2]=3, 全匹配蜂鸣）; `vt digit 5`; `vt tl 1`（期望绿灯）。
> 匹配成功蜂鸣一次 (150ms), 串口回显 `[OK]` / `[FAIL]`。
> 协议细节与模拟测试见 `vision.md`。

### 动作表 (`act` 可用)

| 名称 | 动作 | 说明 |
|------|------|------|
| `fwd` | FORWARD | 直行 |
| `left` / `right` | TURN_LEFT / RIGHT | 标准 90° 转弯 |
| `uturn` | U_TURN | 180° 掉头 |
| `around_l` / `around_r` | AROUND | 大回环绕行 |
| `sharp_l` / `sharp_r` | SHARP | 锐角转弯 |
| `obtuse_l` / `obtuse_r` | OBTUSE | 钝角转弯 |
| `fork_l` / `fork_r` / `fork_c` | FORK | 分叉路口 |
| `fork_2l` / `fork_2r` | FORK_2ND | 多叉第二分支 |
| `slow` / `fast` | SLOW_DOWN / SPEED_UP | 减速/加速 |
| `avoid_l` / `avoid_r` | AVOID | 避障绕行 |
| `merge_l` / `merge_r` | MERGE | 并线汇入 |
| `plat` | CLIMB_PLATFORM | 上平台 |
| `bridge` | CROSS_BRIDGE | 过桥 |
| `auto` | AUTO_SELECT | 自动选方向 |
| `tl` | TRAFFIC_LIGHT | 红绿灯检测 |
| `qr` | QR_SCAN | 二维码扫描 |
| `digit` | DIGIT_RECOGNIZE | 数字识别 |
| `stop` | STOP | 终止 |

### 控制
| 命令 | 说明 |
|------|------|
| `stop` | 紧急停车 |
| `start` | 恢复运行 |
| `reset` | 系统复位 |

### 测试
| 命令 | 说明 |
|------|------|
| `buzzer` | 蜂鸣器测试 |
| `servo <ch> <us>` | 舵机测试 (ch:0-2, us:500-2500) |
| `voice <track>` | 语音模块播放 (0-15) |

### 调试开关
| 命令 | 说明 |
|------|------|
| `trace on/off` | FSM状态转换日志 |
| `log on/off` | 传感器/电机连续输出 |
| `clear` | 清屏 |
