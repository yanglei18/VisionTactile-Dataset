# STM32 手套固件

此目录保存 STM32F446RCT6 主机和 STM32C011F6 从机工程，以及 LSM6DSR 驱动接入代码。
原始用户固件以 Git 提交 `ac22963` 固定；当前修改版报告 `firmware_version: 2`。
旧固件对应的电脑端 v1 硬件配置保持不变，新固件应选择配套 v2 配置。

**当前状态：两种 MCU 均已完成交叉编译与主机模拟测试，尚未刷写或完成真机验收。**
不能用编译通过代替 I2C 时序、USB 重连、传感器精度和长时间稳定性的实测。

## 构建

安装 [Arm GNU Toolchain](https://developer.arm.com/Tools%20and%20Software/GNU%20Toolchain)，使
`arm-none-eabi-gcc` 可执行；本次验证使用 14.3.rel1。需要 Python 3 和编译器自带的 newlib。
在仓库根目录执行：

```bash
python3 data_imu_stm32/build.py
# 或指定工具链，不修改系统 PATH：
python3 data_imu_stm32/build.py --prefix /path/to/toolchain/bin/arm-none-eabi-
```

默认生成 `data_imu_stm32/build/slave/slave.elf/.bin` 与
`data_imu_stm32/build/master/master.elf/.bin`。可使用 `--target slave`、`--target master`、
`--output`、`--jobs`。脚本只编译，不接触设备或执行烧录。CubeIDE 工程和 `.ioc` 同时保留，
从机缺失的链接脚本已补齐；从机限制 32KB Flash / 6KB RAM，主机 256KB Flash / 128KB RAM。

应用逻辑位于 `Core/Src/imu_node.c`、`Core/Src/imu_transport.c`，生成的 `main.c` 仅负责
外设初始化与调用。不要用 CubeMX 重新生成的 USB 回调覆盖 `imu_transport_*` 接入。
USB 中间件 `USBD_CDC_TransmitPacket` 有一处明确修改：向上返回底层发送失败，并释放未启动
传输的 `TxState`；保留原文件许可证，升级中间件时需要检查此行为。

## 数据与可靠性约定

- 主机二进制格式保持不变：每个节点 14 字节：节点号、8 位采集计数、6 个有符号小端 int16。
  六轴顺序为 Gx/Gy/Gz/Ax/Ay/Az；每个 bank 为 6 节点、84 字节，节点 1–6 与 7–12 分别成组。
- 从机配置并读回 WHO_AM_I=`0x6B`、CTRL1_XL=`0x4C`、CTRL2_G=`0x48`、CTRL3_C=`0x44`。
  后者明确开启 BDU 和地址自增。对应名义 104Hz、±8g、±1000°/s，换算保持
  `0.000244 g/LSB`、`0.035 °/s/LSB`。查询时重新读回，并每秒检查一次配置。
- SPI scratch、已发布样本、I2C 发送快照分离。短临界区只保护样本提交，异步 I2C 不引用
  正在被 SPI 写入的内存。SPI 失败、配置验证失败或样本年龄超过 30ms 时拒绝数据读取。
- 8 位计数仅在成功读取并发布样本后递增，回绕正常；它不是硬件时间戳或独立硬件采样序列。
  节点时钟与主机轮询独立，最近有效样本允许重复读取。相同计数不能直接等同丢帧。
- DRDY 和主机定时器使用原子取走的事件计数。没有传感器 FIFO，积压事件无法通过连续重读
  恢复：只读取当前样本，遗漏的处理机会记入 `drdy_coalesced` / `timer_coalesced`。
- 主机先核验 v2 从机状态。未核验、混装旧固件或某次 I2C 读取失败时，拒绝整个对应 bank；
  不把未更新缓存拼成完整健康帧。暂未核验的从机每秒重试，采集失败后清除身份缓存，必须重新核验才能恢复；已经核验的从机自行持续检查配置。
- USB 队列固定为 8 个 bank；入队复制、传输完成回调后才释放头部，BUSY 保留重试，队满丢弃
  新 bank 并累计 `queue_overflows`。USB deinit/init 后清空失效队列并统计丢弃；在途内存不会
  在端点仍持有它时被复用。若端点永远不完成，队列保持有界，需断开重连，不能假报成功。
- 该协议仍没有各节点硬件采样时间、帧 CRC 或全手同步触发；本次改动不能证明严格零丢样或
  所有传感器同时采样。需要严格时间重建时，应另立带长度、CRC 和硬件时间戳的传输协议版本。

## 状态查询协议

仅在独占串口、没有采集任务时查询。主机默认发送二进制流，**不会主动插入文本**。
发送 ASCII `VTIMU STATUS 1 <request_id>\n` 后，主机暂停采样转发，排空已入队 USB 数据，再回复一行：

```text
VTIMU STATUS 1 {"schema_version":1,"request_id":"0011223344556677","firmware_version":2,"imu_model":"LSM6DSR","nominal_sample_rate_hz":104,"nodes":[...],"transport":{...}}
```

`request_id` 必须是调用方为本次查询新生成的 16 个小写十六进制字符。STATUS 命令正文恰为
31 字节，加 LF 共 32 字节；无 request_id、长度错误或含大写字符均拒绝。回复 JSON 原样
回显 `request_id`，调用方只接受与本次请求匹配的回复，忽略其他请求的迟到回复；不能把
旧查询的读回当作当前设备状态。中断接收区、待处理请求和已构建/在途回复分别保存，下一次
请求不会改变上一回复的关联标识或发送内存。

整行小于 8192 字节。调用方应有超时和缓冲区上限，忽略前缀前已在途的二进制数据。
无论查询成功与否，结束时发送 `VTIMU STREAM 1\n` 恢复二进制流；此命令不回复文本 ACK。
命令允许跨 USB 包分片，但不接受 CRLF、额外空白或超过 31 字节的命令行。
旧固件不提供此协议，应报告“未知/未验证”，不能根据普通样本猜测版本。

`nodes` 恰为节点 1–12。可读取的 v2 节点提供：

- `node_id`、`firmware_version`：整数。
- `reachable`、`identity_verified`、`configuration_verified`、`registers_read`、`sample_valid`：布尔值。
- `registers`：只有本次四个寄存器全部读取成功才出现，含整数 `who_am_i`、`ctrl1_xl`、`ctrl2_g`、`ctrl3_c`。
- `sample_count`、`spi_errors`、`drdy_coalesced`、`i2c_errors`：从机 uint32 累计计数，重启归零、溢出回绕。
- `master_i2c_failures`：主机读取该节点数据失败的累计次数。

不可读取或不支持状态协议的节点只提供 `node_id`、`reachable:false` 和
`master_i2c_failures`，不伪造寄存器或从机统计。
`transport` 提供 `queue_capacity_banks`、`queue_overflows`、`usb_busy_attempts`、
`usb_failures`、`usb_discarded_banks`、`usb_completed_banks`、`timer_coalesced`、
`bank_failures`（两个 bank 分别累计）、`command_errors`、`status_overflows`。
这些计数表示相应软件层的事件，不应相加当作传感器物理丢样数。

### I2C 从机状态扩展

主机向节点地址写入一个字节 `0xF0`；从机在前台重新读取寄存器并准备快照，之后一次读返回
32 字节。尚未就绪时拒绝读取，主机最多重试三次，每次间隔 1ms。状态读取后恢复普通
13 字节样本读取；发送非 `0xF0` 的单字节命令也会清除状态选择。

| 字节偏移 | 内容 |
|---|---|
| 0–1 | ASCII `VT` |
| 2 / 3 / 4 | 状态协议 1 / 固件版本 2 / 节点号 |
| 5 | 位 0 身份通过、位 1 配置通过、位 2 样本有效、位 3 四个寄存器读取成功 |
| 6–9 | WHO_AM_I、CTRL1_XL、CTRL2_G、CTRL3_C 实际读值 |
| 10–11 | 保留，0 |
| 12 / 16 / 20 / 24 | 小端 uint32：发布样本数、SPI 失败、DRDY 合并、I2C 失败 |
| 28 | 小端 uint32：生成快照时尚待处理的 DRDY 事件 |

**主机和全部 12 个从机需要配套升级到 v2。** 新主机不会把不能核验的旧从机作为健康节点。
保留旧固件与旧配置可用于历史数据复现；不要覆盖已有标定结果。

## 验证与刷写后的验收

主机模拟测试只依赖 Python 3 与支持 ASan/UBSan 的 C 编译器：

```bash
python3 data_imu_stm32/tests/run_tests.py
```

测试直接执行应用模块，覆盖 SPI/I2C 交错、发送快照不可变、DRDY 处理期间再次触发、失败与
过期样本拒绝、寄存器异常、USB 队满/忙/完成/断开、失败 bank、分片命令、请求关联、旧响应在途时的新查询、查询取消和有界 JSON。
两种 ARM ELF 链接完成也只是静态与模拟验证；供应商 HAL 的实际 I2C NACK/STOP 中断顺序、
USB 长包完成以及全部节点同时工作，必须在硬件上验收。

刷写应由持有原固件备份、确认接线与 SWD 目标的维护者执行。本仓库不自动刷写。
刷写后先查询 12 节点身份/配置，再测试静止约 1g、各轴方向、持续活动、单节点拔除、USB
反复断开和长时间录制；最后使用配套硬件配置重新进行手势标定，记录统计与设备固件版本。

## 许可证与依据

新增应用模块沿用仓库 Apache-2.0；ST HAL、CMSIS、USB 库及生成文件保留各自许可声明。
参考 [LSM6DSR 数据手册](https://www.st.com/resource/en/datasheet/lsm6dsr.pdf)、
[STM32C011F6 数据手册](https://www.st.com/resource/en/datasheet/stm32c011f6.pdf)。
