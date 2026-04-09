# STM32_LED 项目记忆（调试与启动）

## 1. 当前调试方案（已验证）
- 调试器：ST-Link（无 SWO 引脚）
- 日志通道：Semihosting（通过 SWD，不占用 UART）
- 日志函数：`DEBUG_SendString()` 使用 `SYS_WRITE0`（整串输出，速度比逐字符快）

## 2. 关键前提
- 如果没有 `SWO` 引脚，不使用 `st-trace`。
- 使用 `openocd + gdb-multiarch`。
- CMake 必须开启：`LORA_DEBUG_VERBOSE_BUILD=ON`，否则只有极少日志。

## 3. 构建命令（本机可用）
本机 ARM 工具链路径：
`/home/ubuntu22/.local/share/stm32cube/bundles/gnu-tools-for-stm32/14.3.1+st.2/bin`

构建命令：
```bash
PATH=/home/ubuntu22/.local/share/stm32cube/bundles/gnu-tools-for-stm32/14.3.1+st.2/bin:$PATH \
cmake -S . -B build \
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
  -DCMAKE_BUILD_TYPE=Debug \
  -DLORA_DEBUG_VERBOSE_BUILD=ON

PATH=/home/ubuntu22/.local/share/stm32cube/bundles/gnu-tools-for-stm32/14.3.1+st.2/bin:$PATH \
cmake --build build -j4
```

输出固件：`build/LED.elf`

## 4. 调试启动流程
### 4.1 启动 openocd（终端 A）
```bash
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg \
  -c "transport select hla_swd" \
  -c "init" \
  -c "reset halt" \
  -c "arm semihosting enable"
```

### 4.2 启动 gdb（终端 B）
```bash
gdb-multiarch build/LED.elf
```

gdb 内执行：
```gdb
target extended-remote :3333
monitor reset halt
load
continue
```

## 5. 运行中重启代码（不重连）
在 gdb 内：
```gdb
monitor reset halt
continue
```

如果要重刷：
```gdb
monitor reset halt
load
continue
```

如果要停在 `main`：
```gdb
monitor reset halt
tbreak main
continue
```

## 6. 常见问题与结论
### 6.1 只有一条启动日志，后续没有
- 原因：`LORA_DEBUG_VERBOSE_BUILD` 没开。
- 处理：重新 CMake，带 `-DLORA_DEBUG_VERBOSE_BUILD=ON`。

### 6.2 编译报 `wfi/wfe` 非法指令
- 原因：用了宿主机 gcc 而非 `arm-none-eabi-gcc`。
- 处理：按“第 3 节”命令构建，保证 PATH 指向 ARM 工具链。

### 6.3 日志输出很慢
- Semihosting 先天慢；已从 `SYS_WRITEC` 升级到 `SYS_WRITE0`。
- 仍慢时，应减少日志频率/长度。

### 6.4 为什么看到 `reset halt` 后红灯不呼吸
- `halt` 状态下 CPU 停止执行，LED 任务也停止，这是正常现象。
- `continue` 后程序继续跑，呼吸灯才恢复。

## 7. 本次已修复行为（便于回溯）
- `AT+RESET` 命令允许“无响应成功”，避免误判配置失败。
- `SETTING...` 命令处理后直接退出该分支，不再误报 Unknown payload。
- 已接入 IR 命令链路与中断转发：
  - `IRSTUDY`/`IRLEARN`
  - `IRSEND`/`IRPLAY`
  - `IRSTOP`/`IREXIT`
  - `IRCLEAR`

