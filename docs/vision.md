# 视觉系统

视觉模块通过 **USART6** 与主控通信，与 debug console 分时复用同一串口。

## 协议

```
MCU → 视觉:  0xA5 <cmd> 0x5A   （3 字节命令帧）
视觉 → MCU:  <cmd> <data...>     （可变长度应答帧, 首字节为命令回显）
```

> **注意**: 应答帧第一个字节是 `<cmd>` 回显, **不计入数据**。`vision_module_on_usart6_byte` 会把首字节存入 `vis_rsp_cmd`, 其余字节才进入 `data[]`。

视觉模块只在收到命令后应答，不主动发送。

## 命令

| 命令 | 代码 | 应答数据 | 说明 |
|------|------|------------|------|
| 红绿灯检测 | `0x01` | 1 字节 | `0` = 红灯 / `1` = 绿灯 |
| 二维码扫描 | `0x02` | 3 字节 | `qr[0] qr[1] qr[2]`, **ASCII 字符** (`'0'`=0x30 ... `'9'`=0x39) |
| 数字识别 | `0x03` | 1 字节 | 返回 0~9 |

> **二维码返回 3 个 ASCII 字节**（用户确认三个字段都有用，分别记为 `qr[0]`~`qr[2]`）。
> 例如内容 "123" 时数据为 `0x31 0x32 0x33`（收到时会带尾随 `\r`=0x0D, 视模块而定）。

### 模拟测试 (串口助手)

用串口助手模拟视觉模块应答时, **必须带 cmd 前缀**, 且建议用 HEX 发送:

| 场景 | HEX 发送 |
|------|---------|
| 红绿灯=绿 | `01 01` |
| 红绿灯=红 | `01 00` |
| 二维码 "123" | `02 31 32 33` |
| 数字=5 | `03 05` |

不要直接发文本 `"123"` —— 那样 `'1'` 会被当成 cmd 吞掉, 只剩 `"23\r"`。

## 结果存储

所有结果存入 `g_vision_results[]`（`vision_module.h`），掉电丢失：

```c
extern vision_stored_t g_vision_results[8];   // 最多 8 条
extern uint8_t g_vision_result_count;          // 当前条数
```

每条记录: `cmd`(命令码) + `data[]`(应答) + `len`(字节数)

### 查询 API（`vision_module.h`）

```c
/* 通用 */
uint8_t vision_result_get_cmd(uint8_t idx);        // 第 idx 条的命令码
uint8_t vision_result_get_byte(uint8_t idx, uint8_t n); // 第 idx 条的第 n 字节
void     vision_result_clear(void);                 // 清空, 准备下一轮扫描

/* 红绿灯专用 — 找到第一个绿灯的索引 */
uint8_t vision_find_first_green(void);  // 返回索引, 全红返回 0xFF

/* 二维码专用 — 第 qr_idx 个 QR 结果的第 n 个目标 */
uint8_t vision_qr_get_target(uint8_t qr_idx, uint8_t n);  // 返回 0~7, 无效返回 0xFF
```

## 红绿灯检测

路口遇到 `NAV_ACTION_TRAFFIC_LIGHT` 时，`vision_exec` 自动发请求。绿灯放行，红灯每 200ms 重试，最多 3 秒。

`act tl` 可手动测试单次红绿灯行为。

## 二维码扫描

平台操作完成后，`NAV_ACTION_QR_SCAN` 触发扫描。结果自动存入 `g_vision_results[]`。

`act qr` 可手动测试。

## 数字识别

`NAV_ACTION_DIGIT_RECOGNIZE` 触发，结果自动存入 `g_vision_results[]`。

`act digit` 可手动测试。

## 路由决策 — if/else 选路

例行流程：第一遍探索 → 查结果 → 手动或函数选回程路线。

### 步骤

**1. 写探索路径** (`paths.c`):
```c
static const navigate_step_t s_explore_l1[] = {
    FWD, {.action=NAV_ACTION_TRAFFIC_LIGHT}, FWD, PLAT_V(1), STOP };
static const navigate_step_t s_explore_l2[] = {
    FWD, {.action=NAV_ACTION_TRAFFIC_LIGHT}, FWD, PLAT_V(2), STOP };
```

**2. 写平台间路径**:
```c
static const navigate_step_t s_1_2[] = {
    PLAT_V(1), FWD, LEFT, FWD, PLAT_V(2), STOP };
```

**3. 写回程路径** (每种可能的结果一条):
```c
static const navigate_step_t s_return_l1[] = {
    {.action=NAV_ACTION_DIGIT_RECOGNIZE}, FWD, {"1_2"}, STOP };
static const navigate_step_t s_return_l2[] = {
    {.action=NAV_ACTION_DIGIT_RECOGNIZE}, FWD, {"2_3"}, STOP };
```

**4. 选路函数** (`main.c`):
```c
void build_return_path(void) {
    uint8_t g = vision_find_first_green();
    if      (g == 0) navigate_load_path(s_return_l1, 4);
    else if (g == 1) navigate_load_path(s_return_l2, 4);
    else             navigate_load_path(s_demo, 2);  // fallback
}
```

**5. QR 路由示例**:
```c
void route_from_qr(void) {
    uint8_t a = vision_qr_get_target(0, 0);  // 第一个 QR 目标的 A
    uint8_t b = vision_qr_get_target(0, 1);  // 第一个 QR 目标的 B
    if      (a == 3 && b == 5) navigate_load_path(s_3_5, 4);
    else if (a == 4 && b == 6) navigate_load_path(s_4_6, 4);
}
```

### 运行时操作

```
# 探索阶段
path explore_l1    → 走 TL1 路线, 结果自动存储
path explore_l2    → 走 TL2 路线, 结果自动存储
path explore_l3    → 走 TL3 路线, 结果自动存储

# 查看结果
vision results     → 列出所有存储的视觉结果
vision tl          → 手动测一次红绿灯
vision qr          → 手动测一次 QR

# 选路
vision results     → 看哪个 TL 是绿灯
path return_l2     → 手动输入对应的回程路线
```

## 路径用法

```c
static const navigate_step_t my_route[] = {
    FWD, LEFT, FWD,
    PLAT_V(2),                                 /* 上平台 2, 播语音 2 */
    { .action = NAV_ACTION_QR_SCAN },          /* 扫二维码 → 存结果 */
    FWD,
    { .action = NAV_ACTION_TRAFFIC_LIGHT },    /* 红绿灯检测 */
    { .action = NAV_ACTION_DIGIT_RECOGNIZE },  /* 数字识别 → 存结果 */
    STOP,
};
```

## 分时复用

USART6 同一时间只有一种模式：

- **debug 模式**（默认）：RX 字节写入 console 命令缓冲区
- **vision 模式**：RX 字节路由到 `vision_module_on_usart6_byte()`

视觉执行器（`vision_exec.c`）进入时自动切到 vision 模式，退出时切回 debug 模式。
