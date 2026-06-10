# SpinPublisherModule 技术笔记

> 参考文件：`onboard/lidar/spin_publisher_module.cc` / `.h` / `spin_structs.h` / `spin_util.cc`
> 学习日期：2026-06-09

---

## 目录

1. [模块定位与整体架构](#1-模块定位与整体架构)
2. [初始化流程](#2-初始化流程)
3. [主线一：UDP Socket 接收](#3-主线一udp-socket-接收)
4. [主线二：实时线程管理与调度](#4-主线二实时线程管理与调度)
5. [主线三：无锁 SPSC 队列](#5-主线三无锁-spsc-队列)
6. [主线四：拼帧处理流水线](#6-主线四拼帧处理流水线)
7. [主线五：坐标变换（位姿插值 + 内参校正）](#7-主线五坐标变换位姿插值--内参校正)
8. [主线六：故障码监听（epoll）](#8-主线六故障码监听epoll)
9. [发布层：三类输出](#9-发布层三类输出)
10. [线程同步总结](#10-线程同步总结)
11. [内存管理总结](#11-内存管理总结)
12. [面试知识点速查](#12-面试知识点速查)
13. [开发陷阱速查](#13-开发陷阱速查)

---

## 1. 模块定位与整体架构

### 1.1 在自动驾驶栈中的位置

```
雷达硬件（禾赛 AT128 / ATX / 速腾 M1 / EMX-Mini 等）
    │  以太网 UDP 包，AT128 约 12,000 包/秒，10.5 MB/s
    ↓
┌──────────────────────────────────────────────────────┐
│                 SpinPublisherModule                  │
│  （LiteModule，管理多个 LidarFrameBuilder）            │
│                                                      │
│  ┌──────────────────────────────────────────────┐   │
│  │            LidarFrameBuilder（每个雷达一个）   │   │
│  │                                              │   │
│  │  receiver_thread_ ──SPSC Ring── thread_pool_ │   │
│  │  （SCHED_RR 实时）    无锁队列    （4线程）     │   │
│  │       ↓                    ↓                 │   │
│  │  recvfrom()           拼帧 + 质检             │   │
│  │  写环形缓冲区          位姿插值 + 内参校正      │   │
│  │                       PublishShmMsg          │   │
│  │                                              │   │
│  │  FaultThread（epoll，故障码监听）              │   │
│  └──────────────────────────────────────────────┘   │
└──────────────────────────────────────────────────────┘
    │  SHM（零拷贝）    │  SHM（压缩）     │  Lite
    ↓                  ↓                 ↓
PerceptionModule   LoggingModule   HealthManager
（障碍物检测）      （录制上传）     （故障告警）
```

### 1.2 两层类结构

| 类 | 职责 |
|---|---|
| `SpinPublisherModule` | LiteModule，管理生命周期；按整车参数创建 `LidarFrameBuilder`；在 Lite 线程处理位姿/自动驾驶状态订阅 |
| `LidarFrameBuilder` | 单个雷达的完整处理流水线：socket、收包、拼帧、变换、发布 |

每辆车每个启用的雷达创建一个 `LidarFrameBuilder`，多个雷达并行运行，互不干扰。

---

## 2. 初始化流程

### 2.1 构造函数做了什么

```cpp
// spin_publisher_module.cc:585-641
LidarFrameBuilder::LidarFrameBuilder(LidarParametersProto lidar_params,
                                     LiteModule* lite_module)
    : spin_processor_(lidar_params_),    // ① 预计算 sin/cos 查找表
      command_client_(InitCommandClient(...))  // ② 建立 TCP 命令通道
{
    // ③ 打开 UDP 接收 socket
    lidar_socket_ = OpenLidarSocket(lidar_params_);

    // ④ 计算每批最少处理包数（按雷达型号和帧率推算）
    min_num_packets_to_process_ = ComputeMinPacketsToProcess(...);

    // ⑤ 预分配 packet_buffer_（FetchPackets 旧路径用，避免堆分配）
    packet_buffer_.resize(min_num_packets_to_process_);
    for (auto& pkt : packet_buffer_) pkt.data.reserve(kEthernetMtu);

    // ⑥ 创建 SPSC 环形缓冲区（容量 = 32 × batch_size，2的幂）
    const size_t ring_capacity = min_num_packets_to_process_ * 32;
    packet_ring_buffer_ = std::make_unique<PacketRingBuffer>(ring_capacity, kEthernetMtu);

    // ⑦ 可选：通过命令通道从雷达硬件同步最新内参（SN 不一致时触发）
    SyncIntrinsicsFromLidarIfNeeded();
}
```

**重点：⑦ 内参在线同步**

雷达出厂内参存在 `lidar_params.inherent().intrinsics()` 里。但如果换过雷达（序列号变了），配置文件里的内参是旧雷达的，必须通过 TCP 命令通道重新从硬件读取：

```cpp
void LidarFrameBuilder::SyncIntrinsicsFromLidarIfNeeded() {
    need_update_intrinsics_ = HesaiCommandSettings::SyncIntrinsicsAndSnToProto(
        command_client_.get(), &lidar_params_proto);
    if (need_update_intrinsics_) {
        lidar_params_ = lidar_params_proto;  // 用从硬件读到的最新内参
    }
}
```

### 2.2 启动顺序

```cpp
// spin_publisher_module.cc:723-776
void LidarFrameBuilder::Start() {
    OnSetUpTimers();                   // 注册定时器（故障码发布等）
    StartReceiverThread();             // ① 先启动接收线程（开始填充环形缓冲区）
    StartSpinPublisherThread();        // ② 再启动处理线程（从缓冲区消费）
    MaybeStartFaultMessageListener();  // ③ 启动故障码监听线程
    ScheduleFuture(&thread_pool_, [=] {// ④ 异步：ID 认证、ATX 日志获取、定期检查
        // 每 3 秒循环检查雷达配置和内参一致性
        while (!stop_notification_.HasBeenNotified()) {
            get_command_settings_log_();
            std::this_thread::sleep_for(3s);
        }
    });
}
```

先启动接收线程确保缓冲区开始积累数据，再启动处理线程，避免处理线程启动瞬间空转。

### 2.3 停止与析构

```cpp
// spin_publisher_module.cc:2481-2499
void LidarFrameBuilder::Stop() {
    stop_notification_.Notify();      // 通知所有线程退出（幂等）
    if (receiver_thread_.joinable())
        receiver_thread_.join();      // 等接收线程退出
    thread_pool_.Drain();             // 等线程池所有任务完成
}

LidarFrameBuilder::~LidarFrameBuilder() {
    Stop();  // 防御性：即使外部没调用 Stop()，析构时也保证 join
    // 如果不 join，joinable 的 std::thread 析构时会 std::terminate 进程
}
```

---

## 3. 主线一：UDP Socket 接收

### 3.1 业务需求

激光雷达通过以太网把原始扫描数据以 UDP 包发出来（AT128：约 1.5KB/包，12000 包/秒）。软件必须以有界延迟从内核取走这些包，否则内核缓冲区溢出丢包。

### 3.2 Socket 创建与绑定

```cpp
// spin_publisher_module.cc: InitSocket()
int InitSocket(int port) {
    const int socket_id = socket(PF_INET, SOCK_DGRAM, 0); // UDP

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(port);      // 主机字节序 → 网络字节序
    addr.sin_addr.s_addr = INADDR_ANY;       // 接受所有网卡来的包
    bind(socket_id, (sockaddr*)&addr, sizeof(sockaddr));

    // 非阻塞：recvfrom 无数据时立刻返回 -1/EAGAIN
    fcntl(socket_id, F_SETFL, O_NONBLOCK | FASYNC);
    return socket_id;
}
```

**为什么 UDP**：雷达数据是实时流，丢一两个包可接受，TCP 重传机制引入不确定延迟。

**为什么非阻塞**：接收线程需要检查 `stop_notification_`，阻塞 `recvfrom` 会让线程卡住无法退出。

### 3.3 接收缓冲区：setsockopt 静默截断陷阱

```cpp
// spin_publisher_module.cc: ConfigureSocketReceiveBuffer()
void ConfigureSocketReceiveBuffer(int socket_id, int port) {
    const int requested = kSocketReceiveBufferSize;  // 4MB

    // 设置（可能被截断！内核不报错）
    setsockopt(socket_id, SOL_SOCKET, SO_RCVBUF, &requested, sizeof(requested));

    // 必须 getsockopt 反查！
    int actual = 0; socklen_t optlen = sizeof(actual);
    getsockopt(socket_id, SOL_SOCKET, SO_RCVBUF, &actual, &optlen);

    // Linux: getsockopt 返回实际值 × 2（另一半是 skb metadata 开销）
    // actual < requested → 被 /proc/sys/net/core/rmem_max 截断
    if (actual < requested) {
        QLOG(ERROR) << "SO_RCVBUF TRUNCATED: requested=" << requested
                    << " got=" << actual
                    << ". Run: sysctl net.core.rmem_max>=" << requested;
    }
}
```

**为什么截断不报错**：Linux `setsockopt(SO_RCVBUF)` 永远返回 0，即使实际分配量远小于请求量。必须 `getsockopt` 反查，这是几乎所有网络程序的漏洞点。

### 3.4 接收循环（receiver_thread_ 内）

```cpp
// spin_publisher_module.cc:1035-1174
while (!stop_notification_.HasBeenNotified()) {
    // 1. 预申请写槽（批量）
    const int reserved_slots = packet_ring_buffer_->AcquireWriteBatch(
        write_batch_ptrs.data(), min_num_packets_to_process_);

    if (reserved_slots == 0) {
        // 环形缓冲区满：先取走一个内核包（防止内核层也溢出）
        char discard[kEthernetMtu];
        const ssize_t n = recvfrom(lidar_socket_, discard, sizeof(discard), 0, ...);
        if (n > 0) {
            ++total_ring_buffer_drops_;  // 统计丢包
        } else {
            // 内核也没包：处理线程是瓶颈，必须 sleep 让出 CPU！
            // CRITICAL: SCHED_RR 不主动 sleep 会饿死 SCHED_OTHER 处理线程
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
        continue;
    }

    // 2. 批量 recvfrom（直接写进预分配槽，零额外内存分配）
    int committed_packets = 0;
    for (; committed_packets < reserved_slots; ++committed_packets) {
        LidarPacket* slot = write_batch_ptrs[committed_packets];
        slot->data.resize(kEthernetMtu);
        const ssize_t nbytes = recvfrom(lidar_socket_, slot->data.data(), ...);
        if (nbytes > 0) {
            slot->data.resize(nbytes);
            slot->timestamp = ToUnixDoubleSeconds(Clock::Now());
        } else if (nbytes < 0 && errno == EAGAIN) {
            break;  // 内核队列已空，提前结束本批
        }
    }

    // 3. 提交（推进 write_idx_，消费者才能看到）
    if (committed_packets > 0) {
        packet_ring_buffer_->CommitWrite(committed_packets);
        total_packets_received_.fetch_add(committed_packets, memory_order_relaxed);
    } else {
        // 无数据：用 poll 等待，避免 CPU 空转
        pollfd fds{.fd = lidar_socket_, .events = POLLIN};
        poll(&fds, 1, poll_timeout_ms);  // 超时触发 QIssue 告警
    }
}
```

---

## 4. 主线二：实时线程管理与调度

### 4.1 线程类型选择

```
receiver_thread_（std::thread，独立）
    → 设 SCHED_RR，优先级 60%
    → 不进线程池：线程池无法单独设置某一个线程的调度策略

thread_pool_{4}（ThreadPool）
    → 普通 SCHED_OTHER
    → 处理线程 + 故障监听线程 + 命令检查线程
```

### 4.2 SCHED_RR 配置

```cpp
// spin_publisher_module.cc:491-516
void SetCurrentThreadRtSchedRR(const char* thread_name_for_log) {
    const int min_pri = sched_get_priority_min(SCHED_RR);  // Linux: 1
    const int max_pri = sched_get_priority_max(SCHED_RR);  // Linux: 99

    // 60% 位置（约优先级 60）：
    //   高于普通用户 RT 任务（通常 1~20）
    //   低于内核 UDP softirq（负责把网卡数据写进 socket 缓冲区）
    //   如果高于 softirq → softirq 跑不了 → 缓冲区不更新 → 死锁
    sched_param sp{};
    sp.sched_priority = min_pri + (int)((max_pri - min_pri) * 0.6);

    const int rc = pthread_setschedparam(pthread_self(), SCHED_RR, &sp);
    if (rc != 0) {
        QLOG(WARNING) << "Failed (CAP_SYS_NICE?), staying on SCHED_OTHER";
        // 降级：程序不崩，但高负载下可能丢包
    }
}
```

**调度优先级层次**：
```
优先级 99  ：内核 watchdog
优先级 ~80 ：内核 softirq（UDP 收包、网卡中断处理）
优先级 60  ：我们的接收线程（SCHED_RR）← 设这里
优先级  0  ：处理线程、Lite 线程（SCHED_OTHER/CFS）
```

### 4.3 实时线程饿死普通线程（最容易犯的错）

`SCHED_RR` 优先级 60，只要不主动让出 CPU，普通线程（优先级 0）永远排不上队。环形缓冲区满时：

```cpp
// 错误写法：自旋等待（会死锁）
while (packet_ring_buffer_->IsFull()) {}  // 占满 CPU，处理线程永远跑不到

// 正确写法：主动 sleep（让出 CPU，处理线程得以消费）
std::this_thread::sleep_for(std::chrono::microseconds(100));
```

---

## 5. 主线三：无锁 SPSC 队列

### 5.1 结构

```cpp
// spin_publisher_module.h:106-192
class PacketRingBuffer {
    const size_t capacity_;  // 2 的幂（用于位掩码取模，比 % 快）
    const size_t mask_;      // = capacity_ - 1

    std::vector<LidarPacket> buffer_;  // 构造时一次性预分配，不再变动

    alignas(64) std::atomic<size_t> write_idx_{0};  // 生产者独占
    alignas(64) std::atomic<size_t> read_idx_{0};   // 消费者独占
    // alignas(64)：各自独占一个 CPU 缓存行，防止伪共享（False Sharing）
};
```

### 5.2 生产者（接收线程）

```cpp
// 1. 预申请槽（不修改 write_idx_）
int AcquireWriteBatch(LidarPacket** ptrs, int max_count) {
    const size_t write_pos = write_idx_.load(memory_order_relaxed); // 读自己，relaxed 够
    const size_t read_pos  = read_idx_.load(memory_order_acquire);  // 读消费者，需要 acquire
    const size_t free = capacity_ - (write_pos - read_pos);
    // ... 填充 ptrs
}

// 2. recvfrom 直接写入预申请的槽（零拷贝）

// 3. 提交（推进 write_idx_，消费者可见）
void CommitWrite(int count) {
    // release：保证步骤 2 的写在此之前完成且对其他核可见
    write_idx_.fetch_add(count, memory_order_release);
}
```

### 5.3 消费者（处理线程）

```cpp
// 1. 窥视（不修改 read_idx_）
int PeekBatch(const LidarPacket** ptrs, int max_count) {
    const size_t read_pos  = read_idx_.load(memory_order_relaxed);
    const size_t write_pos = write_idx_.load(memory_order_acquire); // acquire 读生产者
    // ... 填充 ptrs
}

// 2. 处理数据

// 3. 释放槽位
void AdvanceRead(int count) {
    read_idx_.fetch_add(count, memory_order_release); // 通知生产者槽已空闲
}
```

### 5.4 memory_order 三个必要性

| 问题 | 没有 atomic 的后果 | atomic 的解决 |
|---|---|---|
| 编译器重排 | 先推 write_idx，再写数据，消费者读到旧数据 | `release` 禁止 store 之前的写移到 store 之后 |
| CPU 乱序执行 | 机器码顺序≠实际执行顺序 | `fetch_add(release)` 生成 fence 指令（x86: mfence，ARM: dmb） |
| 缓存不可见 | 写操作停在本核 L1，其他核看不到 | `release-store` 广播，`acquire-load` 强制从全局读 |

**happens-before 保证**：消费者 `acquire-load` 读到生产者 `release-store` 写入的值时，生产者在 release 之前的所有写，对消费者 acquire 之后全部可见。

### 5.5 为什么不用 mutex

```
mutex = 内存屏障 + 互斥阻塞（futex 系统调用 + 线程挂起/唤醒）
SPSC atomic = 只有内存屏障，无互斥（两线程访问不同槽，天然无竞争）
```

用 mutex 的额外代价：
- 系统调用开销（`futex`）
- **优先级反转**：SCHED_RR 接收线程等 SCHED_OTHER 处理线程释放锁 → 实时线程被卡住 → 丢包

---

## 6. 主线四：拼帧处理流水线

### 6.1 处理循环框架

```cpp
// spin_publisher_module.cc:1394-1462
void LidarFrameBuilder::RunSpinPublisherIteration(const SpinPublisherLoopCtx& ctx) {
    // 步骤 1：从环形缓冲区取包，更新帧
    if (DrainAndProcessPackets(ctx)) return;  // 无数据，sleep 后返回

    // 步骤 2：发布已就绪的局部帧（每 128 条扫描线发一次）
    const int start_scan = PublishReadyPartialSpins(...);

    // 步骤 3：AT128 超时切帧检测
    MaybeMarkAt128Finished(ctx, spin, num_scans);

    // 步骤 4：帧完整性判断
    if (!last_frame_finished_ && lidar_frames_in_build_.size() <= 1) return;

    // 步骤 5：一系列质量检查（丢帧条件）
    if (MaybeDropAt128TooFewScans(...))    return;  // AT128 扫描线太少
    if (ReportClockSyncIssueAndMaybeDrop(...)) return;  // 时钟不同步
    if (CheckSyncTimeTailAndMaybeDrop(...))    return;  // tail 时间异常
    if (CheckScanIntervalAndMaybeDrop(...))    return;  // 方位角间隔异常

    // 步骤 6：无位姿特殊处理
    if (latest_pose_timestamp == max()) { HandleNoPoseSpin(...); return; }

    // 步骤 7：发布完整帧
    PublishOrDropFinalSpin(...);
}
```

### 6.2 包分发：数据包 vs 故障包

同一个 UDP 端口可能收到两种包（以 ATL/ATX 为例）：

```cpp
// spin_publisher_module.cc:1504-1531
void ProcessSpinPacketBatch(...) {
    for (const auto& packet : packets) {
        if (packet.data.size() < packet_size_) {
            // 尺寸小于正常数据包 → 判断是否是故障包
            if (packet.data.size() == kHeSaiFaultMessageSize) {  // 99 字节
                hesai_fault_messages::UpdateFaultMessageproto(...);
            }
            continue;  // 跳过，不加入拼帧
        }
        // 正常数据包 → 解析并加入当前帧
        latest_packet_time_ = packet.timestamp;
        UpdateSpin(packet);
    }
}
```

### 6.3 切帧逻辑

**普通旋转雷达**：azimuth 越过起始角（通常 0°）时认为一帧结束，开始新帧。

**AT128 特殊切帧**：
- 不按 0° 切，使用多个 `start_frames` 角度（从内参读取）
- 角度数组顺序敏感，写错会导致点云"撕裂"
- 还有超时兜底：等待超过 `kMaxWaitingTimeNoDataReceivedToFinishCurrentFrame=10ms` 且扫描线数达标，强制切帧

### 6.4 局部帧（Partial Spin）

```cpp
// spin_publisher_module.cc:1534-1561
int PublishReadyPartialSpins(...) {
    // 每 kNumScansPerPartialSpin=128 条扫描线发一次局部帧
    for (; end_scan <= num_scans; start_scan += 128, end_scan += 128) {
        // 前提：位姿历史必须覆盖本段时间（不能外推太多）
        if (latest_pose_timestamp < spin.scan(end_scan-1).timestamp - max_extrapolate)
            break;
        UpdateSpinAndPublishShm(..., /*partial_spin=*/true, ...);
    }
}
```

**业务价值**：128 条扫描线约 3~4ms（1/28 圈）。感知模块不等 100ms 完整帧，提前 70ms 开始处理，端到端感知延迟从 ~120ms 降到 ~50ms。

### 6.5 E2E 完整性校验（奇瑞 ATX 专用）

```cpp
// spin_publisher_module.cc:1486-1501
void LidarFrameBuilder::MaybeRunE2eCheckOnBatch(...) {
    if (!(FLAGS_enable_e2e_check && ctx.lidar_type == LIDAR_PANDAR_ATX)) return;
    for (const auto& packet : packets) {
        // 从包的固定偏移（kE2eLidarDataOffset=858）读 e2e 计数器
        // 检查序号是否连续，发现跳变说明中间丢包
        E2eProvider::GetInstance().Check(packet, kE2eLidarDataOffset, ...);
    }
}
```

奇瑞项目要求在数据链路层做端到端完整性校验，序号不连续时上报 QIssue。

### 6.6 质量检查详细

| 检查项 | 触发条件 | 处理 |
|---|---|---|
| 时钟同步 | `now - last_scan_ts > 80ms` 或 `latest_pose_ts - last_scan_ts > 80ms` | 上报 `QIT_CLOCK_SYNC` Warning，丢帧 |
| 扫描线数 | AT128 扫描线 < 期望值 × 25% | 丢帧（太少的帧对感知无意义） |
| 方位角间隔 | 相邻扫描线角度差 > 5 × 期望分辨率 | 上报 `kErrorTooLargeIntervalBetweenScans`，丢帧 |
| 有效点数 | 有效点 < 总点数 × 10% | 上报 `kErrorTooLessValidShotsOrPoints` |
| 无位姿 | `pose_history_` 没有任何历史 | 发布不带位姿的帧（感知端处理零位姿情况） |

---

## 7. 主线五：坐标变换（位姿插值 + 内参校正）

### 7.1 为什么两步都需要

```
100ms 一帧，车以 60km/h 跑，位移 1.67m
→ 帧内各扫描线的车辆位置不同
→ 用同一位姿处理整帧：帧头的点误差 1.67m，障碍物"拖影"

雷达出厂内参：每条线束真实仰角、方位偏置 ≠ 标称值
→ 不校正：点云整体畸变，目标位置不准
```

### 7.2 调用顺序（不可颠倒）

```cpp
// spin_publisher_module.cc:2224-2231
{
    absl::MutexLock lock(&pose_history_mutex_);
    spin_util::SetScanPoses(pose_history_, new_spin, start, end);  // ① 先插值位姿
}
spin_processor_.ComputeCalibratedReturns(new_spin, start, end);    // ② 再用位姿做变换
// ② 的变换矩阵里用到 scan.pose，① 没完成则 scan.pose 是零值 → 变换结果全错
```

### 7.3 位姿插值（spin_util.cc）

```cpp
// spin_util.cc:27-90
VehiclePose ComputeEstimatedPoseGivenTimestamp(poses, timestamp) {
    // 二分查找：找到夹住 timestamp 的两个位姿
    int next = upper_bound(poses, timestamp);
    int prev = next - 1;  // 边界处理：超出范围则外推

    // 防止外推基准时间差太小导致误差放大
    // pose_1.time - pose_prev.time < 10ms → 继续往前找
    while (prev > 0 && pose_1.time - poses[prev].time < 0.01) prev--;

    const double ratio = (timestamp - poses[prev].time)
                       / (poses[next].time - poses[prev].time);

    // 位置：直接线性插值
    pose.x = x0 + ratio * (x1 - x0);

    // 角度：必须 NormalizeAngle 处理 ±180° 跨界
    // 例：179° → -179° 实际只转 2°，直接相减得 -358° → ratio=0.5 → 0°（错！）
    // NormalizeAngle(-358°) = 2° → ratio=0.5 → 180°（正确）
    pose.yaw = yaw0 + ratio * NormalizeAngle(yaw1 - yaw0);
}
```

**并发保护**：`pose_history_` 被 `pose_history_mutex_` 保护，Lite 线程写、处理线程读。用 `ABSL_GUARDED_BY` 编译期注解，忘加锁直接编译报错。

**乱序位姿过滤**：

```cpp
// spin_publisher_module.cc:648-657
void UpdatePose(shared_ptr<const PoseProto> pose) {
    absl::MutexLock lock(&pose_history_mutex_);
    if (!pose_history_.empty() &&
        pose_history_.back().timestamp >= pose->timestamp()) {
        QLOG(ERROR) << "Pose arrives out of order!";
        return;  // 丢弃乱序位姿，避免 circular_buffer 时间戳不单调
    }
    pose_history_.push_back({pose->timestamp(), VehiclePose(*pose)});
}
```

### 7.4 内参校正（spin_structs.cc）

按雷达型号分派：

```cpp
// spin_structs.cc:129-148
void SpinProcessor::ComputeCalibratedReturns(Spin* spin, int start, int end) {
    if      (model == AT128)    ComputeCalibratedReturnsInternalAT128(...);
    else if (model == ATL)      ComputeCalibratedReturnsInternalATL(...);
    else if (model == ATX)      ComputeCalibratedReturnsInternalATX(...);
    else if (model == EMX_MINI) ComputeCalibratedReturnsInternalEMXMini(...);
    else                        ComputeCalibratedReturnsInternal(...);  // 通用版
}
```

**通用版核心计算**：

```cpp
// spin_structs.cc:151-209
for each scan (azimuth angle):
    // 合并变换矩阵（每条扫描线算一次，因 scan.pose 不同）
    point_transform = scan.pose.ToTransform() * lidar_transform_;
    //                ↑ 车身 → 世界                ↑ 雷达 → 车身（固定，构造时算好）

    for each beam:
        // 从预计算表查 sin/cos（避免实时三角函数计算）
        [sin_elev, cos_elev] = beam_angle_trig_table_[beam_index];
        [sin_off, cos_off]   = azimuth_offset_trig_table_[beam_index];

        // 角度加法展开（cos(A+B) = cosA cosB - sinA sinB）
        sin_azimuth = -sin_scan * cos_off - cos_scan * sin_off;
        cos_azimuth =  cos_scan * cos_off - sin_scan * sin_off;

        // 球坐标 → 单位向量（雷达坐标系）
        unit = Vec3d(cos_elev * cos_azimuth, cos_elev * sin_azimuth, sin_elev);

        // 距离换算 + 一次矩阵乘法完成两个坐标系变换
        range = raw_range * meters_per_tick;
        coord = point_transform.TransformPoint(unit * range);
        // coord 已是世界坐标系下的三维点
```

**预计算表的意义**：每帧 3600 × 128 = 460,800 个点，每个点查表（~1ns）vs 实时 sin/cos（~20ns），整帧节省约 9ms。

**坐标变换链**：
```
极坐标 (azimuth, beam_id, range)
    ↓ 内参校正 → 雷达坐标系 (x_lidar, y_lidar, z_lidar)
    ↓ × lidar_transform_ → 车身坐标系
    ↓ × scan.pose.ToTransform() → 世界坐标系
CalibratedReturn.x/y/z（感知直接使用）
```

---

## 8. 主线六：故障码监听（epoll）

### 8.1 为什么单独一个线程

某些雷达型号（ATL/ATX/EMX-Mini）故障包和数据包在**不同端口**：

```cpp
// spin_publisher_module.cc:884-889
constexpr int kCheryFaultMessagePort    = 51006;
constexpr int kDongfengFaultMessagePort = 50001;
constexpr int kChanganFaultMessagePort  = 55819;
constexpr int kGeelyFaultMessagePort    = 33038;
```

故障包频率极低（每帧 1 个，10Hz），需要一个高效的长时间等待机制。

### 8.2 epoll vs poll

| | poll | epoll |
|---|---|---|
| 调用次数 | 每次都要传所有 fd | 只调用一次注册，之后 O(1) 等待 |
| 适用场景 | 高频、短时等待 | 低频、长时间阻塞等待 |
| 本模块用在 | 接收线程（`timeout=0`，立刻返回） | 故障线程（`timeout=100ms`，长时等待） |

```cpp
// spin_publisher_module.cc:故障线程
int epoll_fd = epoll_create1(0);
epoll_event event;
event.events = EPOLLIN | EPOLLPRI;
event.data.fd = lidar_fault_message_socket_;
epoll_ctl(epoll_fd, EPOLL_CTL_ADD, lidar_fault_message_socket_, &event);

while (!stop_notification_.HasBeenNotified()) {
    int n = epoll_wait(epoll_fd, events, 1, 100);  // 最多等 100ms
    if (n > 0) {
        recvfrom(lidar_fault_message_socket_, buf, ...);
        // 按包大小区分厂商型号
        if (buf.size() == kHeSaiFaultMessageSize)   // 99B → 禾赛
            hesai_fault_messages::UpdateFaultMessageproto(...);
        else if (buf.size() == kRsEmxMiniFaultMessageSize)  // 748B → 速腾
            rs_fault_messages::UpdateEmxMiniFaultMessage(...);
    }
}
close(epoll_fd);
```

### 8.3 故障信息发布（5秒定时器）

```cpp
// spin_publisher_module.cc:667-690
lite_module_->AddTimerOrDie("publish lidar fault message proto",
    [&]() { lite_module_->Publish(lidar_fault_message_proto_, "lidar_fault_message_proto"); },
    absl::Milliseconds(5000), /*one_shot=*/false);
```

HealthManager 订阅此 topic，根据故障码决定是否触发安全降级。

---

## 9. 发布层：三类输出

### 9.1 主数据：SHM 点云帧

```cpp
// spin_publisher_module.cc:2298-2312
// 局部帧
QLOG_IF_NOT_OK(WARNING, lite_module_->PublishShmMsg(new_spin_shm));
// 完整帧（带 mid_scan 时间戳用于端到端延迟统计）
QLOG_IF_NOT_OK(WARNING,
    lite_module_->PublishShmMsg(new_spin_shm,
        static_cast<int64_t>(new_spin->MidScan().timestamp * 1e6),
        lidar_params_.installation().lidar_id()));
```

Channel：`lidar_spin`（SHM 类型，零拷贝）

### 9.2 压缩帧：encoded_lidar_frame（录制用）

```cpp
// spin_publisher_module.cc:2502-2550
void UpdateEncodedSpin(...) {
    // 1. 序列化 Spin → 字节流（只含 raw_returns，不含 calibrated_returns）
    spin.Serialize(&shm_value_, spin_meta.compat_version());

    // 2. Snappy 压缩（压缩比约 3~4x）
    spin_util::EncodeWithSnappy(shm_value_, &lidar_frame_);

    // 3. 发布（SHM_ENCODED_LIDAR_FRAME 类型）
    auto shm_msg = ShmMessage::CreateToWrite(lidar_frame_.size(),
        SHM_ENCODED_LIDAR_FRAME, "encoded_lidar_frame", "");
    lite_module_->PublishShmMsg(shm_msg, ...);
}
```

用途：LoggingModule 录制原始帧到磁盘/上传云端。注意序列化只保存原始测量值，不保存已校正坐标（可在离线重新计算）。

### 9.3 内参：MultiLidarIntrinsicsProto（启动时一次性）

```cpp
// spin_publisher_module.cc:2732-2740
this->AddTimerOrDie("publish lidar intrinsics",
    [this]() { QLOG_IF_NOT_OK(WARNING, this->Publish(*lidar_multi_intrinsics_)); },
    absl::Milliseconds(1000), /*one_shot=*/true);
```

下游模块（如感知、标定）需要知道每条线束的仰角等几何参数时订阅此话题。

### 9.4 仿真模式（无 UDP）

```cpp
// spin_publisher_module.cc:2643-2698
void SpinPublisherModule::OnSubscribeChannels() {
    if (IsOnboardMode()) {
        Subscribe(&SpinPublisherModule::UpdatePose, this, "pose_proto", 50);
        // 数据来自 UDP，无需订阅
    } else {
        Subscribe(&SpinPublisherModule::UpdatePose, this, "sensor_pose", 50);
        SubscribeLidarFrame([this](const LidarFrame& lidar_frame) {
            // PCAP 回放来的帧，重新走一遍 UpdateSpinAndPublishShm
            spin_builders_[lidar_id]->UpdateSpinAndPublishShm(...);
        }, "log_lidar_spin");
    }
}
```

下游感知接口完全一致，不感知数据来源差异。

---

## 10. 线程同步总结

| 共享资源 | 写线程 | 读线程 | 同步机制 |
|---|---|---|---|
| `PacketRingBuffer` | receiver_thread_（生产者） | thread_pool_ 处理线程（消费者） | atomic release/acquire（无锁 SPSC） |
| `pose_history_` | Lite 线程（`UpdatePose`） | thread_pool_ 处理线程（`SetScanPoses`） | `absl::Mutex` + `ABSL_GUARDED_BY` |
| `stop_notification_` | 主线程（`Stop()`） | 所有工作线程 | `absl::Notification`（一次性，线程安全） |
| `lidar_fault_message_proto_` | thread_pool_ 故障线程（写） | Lite 线程（定时器发布） | 由 Lite 调度保证，故障线程更新后等下一个定时器周期 |
| `is_apa_searching_` | Lite 线程（`UpdateAutonomyState`） | thread_pool_ 处理线程 | 读写均为单 bool，在此架构下无竞争风险 |

**同步难点（面试重点）**：
1. **SCHED_RR 线程不主动 sleep → 饿死普通线程**（环形缓冲区满时必须 sleep 100us）
2. **pose_history_ 迭代器失效**（必须持锁读，ABSL_GUARDED_BY 编译期防呆）
3. **atomic 的 memory_order 选错**（用 relaxed 代替 acquire 会导致偶发读到旧值）

---

## 11. 内存管理总结

所有高频路径上的内存都在启动时一次性预分配：

| 数据 | 分配时机 | 方式 | 大小 |
|---|---|---|---|
| `PacketRingBuffer::buffer_` | 构造函数 | `vector::resize()` | 32 × batch × 25KB ≈ 几十MB |
| 每个 `LidarPacket::data` | 构造时 `reserve(kEthernetMtu=25000)` | `vector::reserve()` | 25KB/包 |
| `Spin` 对象 | `Spin::MakeSpinOnShm()` | SHM placement new | ~11MB（AT128） |
| `packet_buffer_` | 构造函数 | `vector::resize()` | min_batch × 25KB |
| `pose_history_` | 构造时 `circular_buffer{20}` | boost 固定容量 | 20 × sizeof(VehiclePoseWithTimestamp) |

**recvfrom 的零拷贝路径**：

```
内核缓冲区 → recvfrom → slot->data（预分配内存，resize(nbytes) 只改 size，无分配）
```

**SHM placement new**：

```cpp
static std::unique_ptr<ShmMessage> MakeSpinOnShm(LidarModel lidar_type) {
    // shm_alloc(SizeInBytes) → 在共享内存区域 placement-new 一个 Spin
    // 感知进程 mmap 同一块内存，读到的就是已校正的点云，零拷贝
}
```

**CalibratedReturn 布局保护**：

```cpp
struct CalibratedReturn { float x, y, z, range; uint8_t intensity; uint8_t reserved[3]; };
static_assert(sizeof(CalibratedReturn) == 20);  // 跨进程版本布局必须一致
```

---

## 12. 面试知识点速查

### UDP/Socket

**Q：为什么用 UDP？**
> 实时流，丢包可接受，TCP 重传引入不确定延迟。

**Q：setsockopt 的陷阱？**
> 超过 `rmem_max` 时静默截断，`setsockopt` 返回 0，必须 `getsockopt` 反查实际值。Linux 下 `getsockopt` 返回值是实际分配量 × 2（skb 开销），截断时 `actual < requested`。

**Q：为什么非阻塞？**
> 需要检查停止信号，阻塞 `recvfrom` 无法响应 `stop_notification_`。

### 线程调度

**Q：为什么 SCHED_RR？**
> AT128 12000包/秒，间隔 83us，CFS 调度延迟可达 30ms，必须实时调度保证及时取包。

**Q：优先级为什么是 60%？**
> 高于普通 RT 任务，但低于 UDP softirq（负责写 socket 缓冲区）。超过 softirq → 缓冲区不更新 → 死锁。

**Q：SCHED_RR 最危险的错误？**
> 缓冲区满时不主动 sleep，会把所有普通线程饿死，形成死锁。

### 无锁队列

**Q：为什么 SPSC 不用 mutex？**
> 天然无竞争，只需内存屏障；mutex 引入 futex 开销和优先级反转风险。

**Q：memory_order_release/acquire 的作用？**
> 解决编译器重排、CPU 乱序、缓存不可见三个问题，建立 happens-before 关系。

**Q：alignas(64) 的作用？**
> 防止两个 atomic 共享缓存行（伪共享），避免两核同时操作时频繁缓存行失效。

### 业务逻辑

**Q：位姿插值为什么需要 NormalizeAngle？**
> 角度跨 ±180° 时直接线性插值方向反转，NormalizeAngle 把差值归一化到 (-π, π]。

**Q：为什么位姿校正在内参校正之前？**
> 内参校正的变换矩阵用到 `scan.pose`，位姿不填就是零矩阵，结果全错。

**Q：局部帧的业务价值？**
> 每 128 条扫描线（3~4ms）发一次，感知提前 70ms 开始处理，端到端延迟从 120ms 降到 50ms。

---

## 13. 开发陷阱速查

| 陷阱 | 表现 | 根因 | 解决 |
|---|---|---|---|
| `setsockopt` 静默截断 | 偶发丢包，无报错 | `rmem_max` 限制，setsockopt 不报错 | `getsockopt` 反查 + `sysctl rmem_max` |
| SCHED_RR 饿死普通线程 | 管道完全卡死 | 缓冲区满时没有 `sleep(100us)` | RT 线程满载必须主动让出 CPU |
| AT128 切帧角顺序错 | 点云撕裂、鬼影 | `start_frames` 数组顺序填错 | 严格按厂商文档顺序 |
| CalibratedReturn 布局改变 | 读到乱数据不崩溃 | 跨进程 SHM，OTA 升级中间态版本不一致 | `static_assert(sizeof==20)` 保护 |
| 位姿乱序 | 插值结果错误 | circular_buffer 时间戳不单调 | `UpdatePose` 检测并丢弃乱序 |
| 角度插值跨 ±180° | 偏航角跳变 | 直接线性插值方向反转 | `NormalizeAngle` 归一化差值 |
| 位姿外推分母近零 | 误差放大 | 相邻位姿时间差 < 10ms | 往前找时间差 > 10ms 的位姿 |
| PCAP 回放时钟不同步 | 位姿全零 | `use_sim_time` 未设置 | 回放必须 `--use_sim_time=true` |
| 强度跨型号比较 | 阈值失效 | 各厂商强度范围不同 | 通过 `intensity_util` 归一化 |
| 不用 atomic 的 SPSC | 偶发读旧值/垃圾值 | 编译器重排 + CPU 乱序 + 缓存不可见 | `memory_order_release/acquire` |

---

## 一句话总结

SpinPublisherModule 是一个**实时传感器数据采集 + 多步几何变换流水线**：实时线程从 UDP socket 收包，通过无锁 SPSC 队列传给处理线程，处理线程完成拼帧、运动补偿、内参校正，把"以传感器为中心的极坐标原始测量"变换成"世界坐标系下精确的三维点云"，通过共享内存零拷贝交给感知。每个设计决策——实时调度、无锁队列、预计算查找表、局部帧——都是为了在嵌入式 SoC 的资源约束下，保证 10Hz 帧率和亚毫秒级传输延迟。

---

## 14. 简历条目支撑：关键代码 + 可量化指标

> 对应简历：Lite 中间件与点云数据发布链路开发

---

### 条目一：基于 PublishShmMsg 共享内存机制完成点云数据零拷贝发布

#### 关键代码

```cpp
// spin_publisher_module.cc:2301-2309
// 局部帧：直接发布 SHM 对象引用，不做任何拷贝
QLOG_IF_NOT_OK(WARNING, lite_module_->PublishShmMsg(new_spin_shm));

// 完整帧：附带 mid_scan 时间戳用于端到端延迟统计
QLOG_IF_NOT_OK(WARNING,
    lite_module_->PublishShmMsg(
        new_spin_shm,
        static_cast<int64_t>(new_spin->MidScan().timestamp * 1e6),
        lidar_params_.installation().lidar_id()));
```

```cpp
// spin_structs.h:425-431（Spin 对象大小计算）
static int SizeInBytes(LidarModel lidar_type) {
    const int num_beams = GetLidarConfig(lidar_type).spinning_lidar_params().num_beams();
    constexpr auto kShotSize =
        LaserShot::kMaxNumReturns * sizeof(LaserShot) + sizeof(uint8_t);
    //   = 2 × 24B + 1B = 49B
    return sizeof(Spin) + kMaxNumScans * num_beams * kShotSize;
    //                    3620       ×  128(AT128) × 49B ≈ 22.7 MB
}
```

```cpp
// Spin 存在共享内存，感知进程 mmap 同一块地址
static std::unique_ptr<ShmMessage> MakeSpinOnShm(LidarModel lidar_type) {
    // placement new in SHM region：不经过堆分配，不需要序列化
}
REGISTER_SHM_MSG_WITHOUT_SIZE(Spin);  // 声明 Spin 为可直接 SHM 存储类型
```

#### 可量化指标

| 指标 | 数值 | 来源 |
|---|---|---|
| 单帧点云大小（AT128，双回波） | `3620 × 128 × 49B ≈ 22.7 MB` | `SizeInBytes()` 公式 |
| 帧率 | `10 Hz` | `kLidarSpinningRate = 10` |
| 跨进程数据吞吐量 | `22.7 MB × 10 = 227 MB/s` | 帧大小 × 帧率 |
| SHM 传输拷贝次数 | **0 次**（placement new + mmap） | `PublishShmMsg` 实现 |
| 若用 Protobuf 序列化传输 | 至少 2 次拷贝（序列化 + 反序列化）+ CPU 编解码耗时 | 对比方案 |
| 序列化开销（录制用途） | 约 5~8 MB/帧（只含 raw_returns，Snappy 压缩后） | `Serialize` 只存原始测量值 |

**一句话**：单帧 22.7MB、10Hz 的点云数据，通过共享内存零拷贝避免了 227 MB/s 的内存带宽消耗和 Protobuf 序列化 CPU 开销。

---

### 条目二：Partial Spin 局部帧发布机制——降低感知链路处理延迟

#### 关键代码

```cpp
// spin_publisher_module.cc:112
constexpr int kNumScansPerPartialSpin = 128;  // 每 128 条扫描线发一次

// spin_publisher_module.cc:1534-1561
int LidarFrameBuilder::PublishReadyPartialSpins(...) {
    int start_scan = spin_in_build->num_published_scans;
    int end_scan = start_scan + kNumScansPerPartialSpin;   // 滑动窗口
    for (; end_scan <= num_scans; ...) {
        // 确保位姿覆盖（不外推过多）
        if (latest_pose_timestamp < spin.scan(end_scan-1).timestamp - max_extrapolate)
            break;
        // 不等整帧，立刻发布
        UpdateSpinAndPublishShm(..., /*partial_spin=*/true, start_scan, end_scan);
        spin_in_build->num_published_scans = end_scan;
    }
}
```

```cpp
// RunSpinPublisherIteration 中：局部帧在整帧发布之前先发
const int start_scan = PublishReadyPartialSpins(ctx, ...); // ← 优先
// ...
PublishOrDropFinalSpin(ctx, ...);                          // ← 之后
```

#### 延迟分析与可量化指标

AT128 参数：约 3600 条扫描线/帧，10Hz（100ms/帧），128 线束

```
全帧模式（无 Partial Spin）：
  雷达旋转开始 ──────────────────── 100ms ──→ 感知收到完整帧，开始处理
  感知能看到第一个点的时刻：100ms 后

Partial Spin 模式：
  雷达旋转开始 → 3.6ms → 感知收到第 1 批(128 条扫描线) → 开始处理前 1/28 帧
              → 7.1ms → 感知收到第 2 批
              → ...（共 ~28 批，每批 3~4ms）
  感知能看到第一个点的时刻：3.6ms 后
```

| 指标 | 数值 | 计算依据 |
|---|---|---|
| 每帧局部帧发布次数 | `3600 / 128 ≈ 28 次` | `kNumScansPerPartialSpin = 128` |
| 第一批局部帧到达时刻 | `100ms × (128/3600) ≈ 3.6ms` | 扫描比例 |
| 感知可见第一个点的时刻（优化前） | `~100ms` | 等整帧 |
| 感知可见第一个点的时刻（优化后） | `~3.6ms` | 第一批局部帧 |
| 感知流水线启动时间提前量 | **~96ms** | 100ms - 3.6ms |
| 端到端感知延迟降低（估算） | **50~70ms** | 前半帧重叠处理 |

**直觉解释**：雷达旋转时，感知不需要等待"画完整幅画"，3.6ms 后就能开始处理左侧 1/28 的点云，到 100ms 整帧到齐时感知已经处理完前 ~70% 的帧，整体端到端延迟大幅降低。

---

### 条目三：高频数据链路性能优化——实时调度 + 无锁 SPSC + 共享内存

#### 关键代码

**实时线程调度**：
```cpp
// spin_publisher_module.cc:491-516
void SetCurrentThreadRtSchedRR(const char* name) {
    const int min_pri = sched_get_priority_min(SCHED_RR);  // 1
    const int max_pri = sched_get_priority_max(SCHED_RR);  // 99
    sp.sched_priority = min_pri + (int)((max_pri - min_pri) * 0.6); // ≈ 60
    pthread_setschedparam(pthread_self(), SCHED_RR, &sp);
}
```

**无锁 SPSC 队列**：
```cpp
// spin_publisher_module.h:190-191
alignas(64) std::atomic<size_t> write_idx_{0};  // 生产者独占，独占缓存行
alignas(64) std::atomic<size_t> read_idx_{0};   // 消费者独占，独占缓存行

// 提交（release 语义，确保数据对消费者可见）
write_idx_.fetch_add(count, std::memory_order_release);
// 消费（acquire 语义，读到生产者的最新状态）
const size_t write_pos = write_idx_.load(std::memory_order_acquire);
```

**环形缓冲区预分配**：
```cpp
// spin_publisher_module.cc:616-624
// 容量 = 32 × min_batch（AT128 约 32×12 = 384 槽）
const size_t ring_capacity = min_num_packets_to_process_ * 32;
packet_ring_buffer_ = std::make_unique<PacketRingBuffer>(ring_capacity, kEthernetMtu);
// 每个槽预分配 kEthernetMtu=25000 字节，recvfrom 写入时零额外分配
```

**Socket 缓冲区**：
```cpp
// spin_publisher_module.cc:107,169
constexpr int kSocketReceiveBufferSize = 4096 * 1024;  // 4MB
setsockopt(socket_id, SOL_SOCKET, SO_RCVBUF, &requested, sizeof(requested));
// 必须 getsockopt 反查，防止 rmem_max 静默截断
```

#### 可量化指标

| 优化点 | 指标 | 数值 | 来源 |
|---|---|---|---|
| SCHED_RR 接收线程 | 调度延迟上界 | **< 1ms**（实时调度） vs CFS 30~40ms | `sched_get_priority_*` |
| AT128 收包间隔 | 1/12000 包/秒 | **83 微秒** | 厂商规格 |
| 不用实时调度时丢包风险 | CFS 单次调度延迟 | 30~40ms >> 83us | Linux CFS 特性 |
| SPSC 无锁 vs mutex | 省去 futex 系统调用 | 约 1~3 微秒/次 | futex 开销 |
| SPSC 无锁操作次数 | 每帧 ~12,000 次 | 节省 **12~36ms** futex 开销/帧 | 推算 |
| 伪共享消除（alignas(64)） | 两核并发写 atomic 的缓存行失效 | 0 次（各独占 64B） vs 每次写都失效 | CPU 架构 |
| 环形缓冲区预分配 | recvfrom 路径上堆分配次数 | **0 次**（全部 reserve）| 代码直接体现 |
| socket 缓冲区 | 内核层最大可缓存量 | **4 MB**（约 4000 个包的余量） | `kSocketReceiveBufferSize` |
| 默认 rmem_max（未配置时） | ~200KB → 仅 200 个包余量 | 高负载下极易丢包 | Linux 默认值 |

**整体链路延迟拆解（AT128，一帧 100ms）**：

```
雷达发出 UDP 包
  → 内核 DMA 写 socket 缓冲区（纳秒级，硬件）
  → SCHED_RR 接收线程调度到（< 1ms，实时保证）
  → recvfrom → SPSC 写槽（无分配，< 1us）
  → CommitWrite release 可见（1 原子操作，< 100ns）
  → 处理线程 PeekBatch acquire（1 原子操作，< 100ns）
  → 拼帧 + 位姿插值 + 内参校正（~3ms/批次，CPU 运算）
  → PublishShmMsg（SHM，0 拷贝，< 1us）
  → 感知进程 mmap 读取（直接访问，0 拷贝，< 1us）

第一批局部帧到感知：≈ 3.6ms（收包时间）+ < 5ms（处理）≈ 8~10ms
```

---

### 简历条目对照总结

| 简历描述 | 关键代码位置 | 核心量化数据 |
|---|---|---|
| `PublishShmMsg` 零拷贝发布 | `spin_publisher_module.cc:2301`；`spin_structs.h:425` | 单帧 22.7MB，10Hz，**0 次拷贝**，节省 227 MB/s 内存带宽 |
| 支撑高频点云数据跨进程传输 | `REGISTER_SHM_MSG_WITHOUT_SIZE(Spin)`；`MakeSpinOnShm` | 22.7MB × 10Hz = 227 MB/s 吞吐，传输开销 < 1μs |
| Partial Spin 降低感知延迟 | `kNumScansPerPartialSpin=128`；`PublishReadyPartialSpins` | 感知启动提前 **~96ms**，端到端延迟降低 **50~70ms** |
| 实时线程调度 | `SetCurrentThreadRtSchedRR`；优先级 60% | 调度延迟从 30~40ms 降到 **< 1ms**，满足 83μs 收包时间要求 |
| 无锁 SPSC 队列 | `PacketRingBuffer`；`alignas(64)`；`memory_order_release/acquire` | **0 次**锁竞争，消除优先级反转，每帧节省 12~36ms futex 开销 |
| 共享内存通信机制 | `MakeSpinOnShm`（placement new）；`PublishShmMsg` | 跨进程传输 **0 序列化，0 拷贝** |
