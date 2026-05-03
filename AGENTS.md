# AGENTS.md

本文件用于约束智能代理在本仓库中的工作方式，适用范围为仓库根目录及其所有子目录。每次启动后应优先读取并遵守本文件内容。

## 仓库介绍

- 这是一套运行在 STM32F407IGH6 芯片上的舵轮底盘、双轴自稳云台、带摩擦轮发射机构的竞赛机器人整车代码仓库。
- 使用 GitHub 进行远程仓库托管。

## 业务实现

- 使用 FreeRTOS 进行任务调度，各任务运行频率均为 1 kHz。
- 使用 `message_center` 进行类 ROS 话题的消息订阅与发布。
- 整车搭载两块开发板（创建 `Steering_Wheel` 分支之后）。
- 在 `application/robot_def.h` 中使用宏定义区分为底盘板和云台板。
- 各开发板运行的任务数量与类型相同，但同一任务在不同板上的内容不同。
- 板间通信使用串口 `UART1` 进行通信，发包频率为 1 kHz。
- 上板负责接收遥控器（`UART3`）及上位机（`USB2.0_FS`）控制命令，并运行整车状态机、滑环以上电机控制。
- 下板负责接收裁判系统态势信息（`UART6`），并负责滑环以下电机控制。
- 与电机的通信使用 1 MHz CAN 总线，所有电机控制与反馈频率均为 1 kHz。

## 行为要求

- 除非我明确要求，否则不得删除远程仓库中的任何内容。
- 所有文件都应使用 UTF-8 编码。
- 仓库中存在许多行末中文注释；读取与写入时，注意不要误以为行末中文注释屏蔽了下一行代码。
- 如果你认为读到了“行末中文注释屏蔽了其后紧邻的业务代码”的情况，不要进行修改。
- 每次修改内容后，检查所修改文件中的中文注释是否产生乱码。
- 需要修改代码时，优先仅修改我明确提到的函数，不进行整文件重构，不要修改我的业务代码实现逻辑。

## 构建与验证

本仓库当前优先使用 EIDE 构建。不要优先花时间寻找 CMake/Makefile 构建入口。

### 已验证环境

- `cmake` 当前不在 PATH。
- `ninja` 当前不在 PATH。
- `make` 当前不在 PATH。
- `mingw32-make` 存在，但当前仓库没有可直接使用的 Makefile。
- `git` 可能不在 PATH，优先使用：

```powershell
D:\Git\cmd\git.exe
```

- EIDE ARM GCC：

```powershell
$env:USERPROFILE\.eide\tools\gcc_arm\bin\arm-none-eabi-gcc.exe
```

- EIDE 构建器：

```powershell
$env:USERPROFILE\.vscode\extensions\cl.eide-3.26.9\res\tools\win32\unify_builder\unify_builder.exe
```

### 完整 Debug 构建

在仓库根目录执行：

```powershell
& "$env:USERPROFILE\.vscode\extensions\cl.eide-3.26.9\res\tools\win32\unify_builder\unify_builder.exe" --no-color -p build\Debug\builder.params
```

该命令会写入 `build\Debug\.lock`、`.obj`、`.elf`、`.hex`、`.bin` 等产物。
如果当前工具处于只读沙箱，必须申请提升权限后再运行；即使 `--dry-run` 也会尝试写 `.lock`，只读下会失败。

2026-05-03 实测结果：

```text
[ DONE ] build successfully !
```

输出文件：

```text
build/Debug/basic_framework.hex
build/Debug/basic_framework.bin
```

构建中存在若干既有 warning，例如 unused function/variable、指针类型不匹配，以及 `AHRS.lib` 的 wchar_t 链接 warning；这些不一定是本次修改引入的问题。

### 轻量检查

检查补丁空白问题：

```powershell
D:\Git\cmd\git.exe diff --check --
```

只检查单个文件：

```powershell
D:\Git\cmd\git.exe diff --check -- application/chassis/chassis.c
```

### 终端显示注意

PowerShell 输出里可能反复出现：

```text
无法设置属性。此语言模式仅支持核心类型的属性设置。
```

这是受限语言模式下设置 `[Console]::OutputEncoding` 失败。若命令 exit code 为 0，可忽略。

中文注释在终端中可能显示乱码，这是控制台 codepage/解码问题，不等于文件内容一定损坏。修改中文注释后仍需谨慎检查实际文件内容。
