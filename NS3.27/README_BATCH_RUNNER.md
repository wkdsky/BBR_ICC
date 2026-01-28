# BBRv2 批量测试运行脚本

这是一套用于批量运行和管理 BBRv2 拥塞控制算法测试的脚本工具集。

## 📋 目录结构

```
.
├── run_bbrv2_batch.sh              # Bash 脚本（推荐初学者使用）
├── run_bbrv2_batch.py              # Python 脚本（功能更强大）
├── run_examples.sh                 # 使用示例
├── bbrv2_config_examples.json      # 配置文件示例
├── README.md                       # 本文件
└── scratch/
    ├── 4_bbrv2.cc                  # 4 条流脚本
    ├── 8_bbrv2.cc                  # 8 条流脚本
    ├── 16_bbrv2.cc                 # 16 条流脚本
    └── 32_bbrv2.cc                 # 32 条流脚本
```

## 🚀 快速开始

### 1. 使用 Bash 脚本（简单模式）

```bash
# 运行所有测试（4、8、16、32 条流）
./run_bbrv2_batch.sh

# 只运行 8 条流
./run_bbrv2_batch.sh --flows 8

# 自定义参数
./run_bbrv2_batch.sh --flows 4,8 --sender-bw 10 --bottle-bw 16 --sim-time 60
```

### 2. 使用 Python 脚本（高级模式）

```bash
# 运行所有测试
python3 run_bbrv2_batch.py

# 自定义参数
python3 run_bbrv2_batch.py --flows 8 --sender-bw 10 --bottle-bw 16 --sim-time 60

# 从配置文件加载
python3 run_bbrv2_batch.py --config my_config.json
```

## 📝 参数说明

### 通用参数

| 参数 | 说明 | 默认值 | 示例 |
|------|------|--------|------|
| `--flows` | 流数（逗号分隔） | `4,8,16,32` | `--flows 4,8,16` |
| `--sender-bw` | 边缘链路带宽 (Mbps) | `10` | `--sender-bw 20` |
| `--bottle-bw` | 瓶颈链路带宽 (Mbps) | 自动 | `--bottle-bw 32` |
| `--bottle-delay` | 瓶颈链路延迟 (ms) | `28` | `--bottle-delay 50` |
| `--sim-time` | 仿真时长 (秒) | `30` | `--sim-time 120` |

### 自动 Bottleneck 带宽

若不指定 `--bottle-bw`，将按以下规则自动设置：

| 流数 | 自动 Bottleneck 带宽 |
|------|---------------------|
| 4 | 8 Mbps |
| 8 | 16 Mbps |
| 16 | 32 Mbps |
| 32 | 64 Mbps |

这确保每条流平均分享相同的带宽 (2 Mbps/flow)。

## 💻 使用示例

### 示例 1：基准测试（所有流，默认参数）

```bash
./run_bbrv2_batch.sh
```

**结果：**
- 运行 4、8、16、32 条流
- 边缘链路：10 Mbps
- 瓶颈延迟：28 ms
- 仿真时长：30 秒

### 示例 2：高延迟测试

```bash
./run_bbrv2_batch.sh --flows 4,8,16 --bottle-delay 100 --sim-time 60
```

**测试场景：**
- 瓶颈延迟增加到 100 ms（高延迟网络）
- RTT 增加到约 206 ms
- 仿真时长 60 秒

### 示例 3：低带宽测试

```bash
python3 run_bbrv2_batch.py --flows 4,8 --sender-bw 5 --bottle-bw 8 --sim-time 45
```

**测试场景：**
- 边缘链路：5 Mbps
- 瓶颈链路：8 Mbps
- 仿真时长：45 秒

### 示例 4：高带宽多流测试

```bash
./run_bbrv2_batch.sh --flows 16,32 --sender-bw 20 --bottle-delay 10 --sim-time 120
```

**测试场景：**
- 高带宽边缘链路：20 Mbps
- 多条流竞争
- 低延迟网络：10 ms
- 长仿真时间：120 秒

## 📊 输出文件

### 日志文件
```
test_4flows_10mbps_8mbps_28ms.log
test_8flows_10mbps_16mbps_28ms.log
...
```

### Trace 文件（traces/ 目录）

对于每条流生成以下 trace 文件：

```
1_bbrv2_1_inflight.txt    # Inflight 字节数
1_bbrv2_1_bbrmode.txt     # BBR 模式
1_bbrv2_1_sendrate.txt    # 发送速率
1_bbrv2_1_recvrate.txt    # 接收速率
1_bbrv2_1_rtt.txt         # RTT
1_bbrv2_1_bw.txt          # 带宽估计
1_bbrv2_1_owd.txt         # 单向延迟
...
```

## 🔧 高级用法

### 创建自定义配置文件

创建 `my_test.json`：

```json
{
  "flows": "8,16",
  "sender_bw": 15,
  "bottle_bw": 24,
  "bottle_delay": 50,
  "sim_time": 90
}
```

运行测试：

```bash
python3 run_bbrv2_batch.py --config my_test.json
```

### 组合多个参数

```bash
# 测试不同延迟下的性能
for delay in 10 28 50 100; do
  ./run_bbrv2_batch.sh --flows 8 --bottle-delay $delay --sim-time 60
done

# 测试不同流数
./run_bbrv2_batch.sh --flows 4 --bottle-delay 28 --sim-time 30
./run_bbrv2_batch.sh --flows 8 --bottle-delay 28 --sim-time 30
./run_bbrv2_batch.sh --flows 16 --bottle-delay 28 --sim-time 30
```

## 📈 网络配置参数详解

### RTT 计算

```
RTT = 2 × (边缘链路延迟) + 2 × (瓶颈链路延迟)
    = 2 × 1 + 2 × 28
    = 58 ms (默认配置)
```

### BDP (带宽延迟乘积) 计算

```
BDP = 瓶颈带宽 × RTT
    = 16 Mbps × 58 ms
    = 116 KB (8流配置)
```

### 缓冲区大小

```
Buffer = 2 × BDP  (默认配置)
```

## ⚠️ 注意事项

1. **首次运行**：脚本会重新编译相关文件，时间较长
2. **备份源文件**：脚本自动创建 `.orig` 备份
3. **清理构建**：如需完全清理，运行 `./waf clean`
4. **权限问题**：确保脚本有执行权限 `chmod +x run_bbrv2_batch.sh`

## 🔍 故障排除

### 编译失败

```bash
# 清理构建缓存
./waf clean

# 重新配置
./waf configure

# 重新编译
./waf build
```

### 脚本执行权限

```bash
chmod +x run_bbrv2_batch.sh
chmod +x run_bbrv2_batch.py
chmod +x run_examples.sh
```

### Python 依赖

Python 脚本仅使用标准库，无需额外安装。

## 📚 相关文件

- **源脚本**: `scratch/4_bbrv2.cc`, `scratch/8_bbrv2.cc`, 等
- **配置示例**: `bbrv2_config_examples.json`
- **文档**: 本文件

## 🎯 典型工作流程

```bash
# 1. 基准测试
./run_bbrv2_batch.sh

# 2. 高延迟测试
./run_bbrv2_batch.sh --flows 8,16 --bottle-delay 100 --sim-time 60

# 3. 分析结果
ls -lh traces/*inflight*.txt | head -20

# 4. 查看 trace 数据
head -5 traces/1_bbrv2_1_inflight.txt
```

## 💡 最佳实践

# Example 1: Run all flows with default parameters
echo "=== Example 1: Default test ==="
./run_bbrv2_batch.sh

# Example 2: Run only 4 and 8 flows with custom bottleneck delay
echo "=== Example 2: Custom delay ==="
./run_bbrv2_batch.sh --flows 4,8 --bottle-delay 50 --sim-time 60

# Example 3: Custom bandwidth and delay
echo "=== Example 3: Custom bandwidth ==="
./run_bbrv2_batch.sh --flows 8 --sender-bw 10 --bottle-bw 20 --bottle-delay 100 --sim-time 120

# Example 4: Quick test (only 4 flows, 10 seconds)
echo "=== Example 4: Quick test ==="
./run_bbrv2_batch.sh --flows 4 --sim-time 10


