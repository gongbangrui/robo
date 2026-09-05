# 路线编写

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
