# SpinPublisherModule 技术笔记

> 以模块的 5 条业务主线为骨架，将每条主线用到的 Linux 应用层技术栈拆开讲透。
> 参考文件：`onboard/lidar/spin_publisher_module.cc` / `.h` / `spin_structs.h`

---

## 目录

1. [业务全景](#1-业务全景)
2. [主线一：UDP Socket 接收](#2-主线一udp-socket-接收)
3. [主线二：实时线程管理](#3-主线二实时线程管理)
4. [主线三：无锁 SPSC 队列](#4-主线三无锁-spsc-队列)
5. [主线四：拼帧与内参校正](#5-主线四拼帧与内参校正)
6. [主线五：故障码监听（epoll）](#6-主线五故障码监听epoll)
7. [内存管理总结](#7-内存管理总结)
8. [开发陷阱速查](#8-开发陷阱速查)

---

## 1. 业务全景

```
雷达硬件（禾赛 AT128 / 速腾 M1 等）
    │  以太网 UDP 包，每秒约 12,000 包，10.5 MB/s
    ↓
┌─────────────────────────────────────────────┐
│              LidarFrameBuilder              │
│                                             │
│  receiver_thread_  ──SPSC Ring──  SpinPublisherThread  │
│  （实时 SCHED_RR）   无锁队列    （线程池）              │
│       ↓                              ↓                 │
│  recvfrom()                    拼帧 + 内参校正           │
│  写环形缓冲区                   位姿插值 + 发布 SHM       │
│                                             │
│  FaultMessageThread（epoll，故障码监听）       │
└─────────────────────────────────────────────┘
    │  PublishShmMsg（零拷贝共享内存）
    ↓
PerceptionModule（感知）→ 障碍物检测 → 规划
```

每辆车每个雷达对应一个 `LidarFrameBuilder`，`SpinPublisherModule` 只负责按整车参数创建和管理所有 Builder。

---

## 2. 主线一：UDP Socket 接收

### 2.1 业务需求

激光雷达通过以太网把原始扫描数据以 UDP 包发出来，每个包约 1500 字节，包含若干条扫描线。软件必须从操作系统网络栈里把这些包及时取走，才不会丢包。

### 2.2 Socket 创建与绑定

```cpp
// spin_publisher_module.cc: InitSocket()
int InitSocket(int port) {
    // 1. 创建 UDP socket（SOCK_DGRAM = 数据报，无连接）
    const int socket_id = socket(PF_INET, SOCK_DGRAM, 0);

    // 2. 绑定到本机指定端口（雷达往这个端口发包）
    sockaddr_in lidar_address{};
    lidar_address.sin_family      = AF_INET;
    lidar_address.sin_port        = htons(port);      // 主机序→网络序
    lidar_address.sin_addr.s_addr = INADDR_ANY;       // 接受所有网卡
    bind(socket_id, (sockaddr*)&lidar_address, sizeof(sockaddr));

    // 3. 设为非阻塞（O_NONBLOCK）
    //    recvfrom 没数据时立刻返回 -1/EAGAIN，不阻塞线程
    fcntl(socket_id, F_SETFL, O_NONBLOCK | FASYNC);

    return socket_id;
}
```

**为什么用 UDP 不用 TCP**：雷达数据是实时流，丢一两个包比卡顿等待更可接受。TCP 的重传机制会引入不确定延迟，不适合传感器数据。

**为什么设非阻塞**：接收线程需要主动控制何时睡眠、何时读取，阻塞模式下线程会卡在 `recvfrom`，无法在没有数据时执行其他逻辑（如检查停止信号）。

### 2.3 接收缓冲区配置

内核为每个 socket 维护一个接收缓冲区，包先进缓冲区，再由应用层 `recvfrom` 取走。如果取得慢，缓冲区满了，后续包被内核直接丢弃（没有任何错误提示）。

```cpp
// spin_publisher_module.cc: ConfigureSocketReceiveBuffer()
void ConfigureSocketReceiveBuffer(int socket_id, int port) {
    const int requested = 4096 * 1024;  // 请求 4MB

    // 扩大内核 socket 接收缓冲区
    setsockopt(socket_id, SOL_SOCKET, SO_RCVBUF, &requested, sizeof(requested));

    // 关键：setsockopt 失败不报错，必须 getsockopt 反查！
    int actual = 0;
    getsockopt(socket_id, SOL_SOCKET, SO_RCVBUF, &actual, &optlen);

    if (actual < requested) {
        // Linux 返回值是实际值的 2 倍（skb 开销）
        // 如果 actual < requested，说明被内核截断
        // 原因：/proc/sys/net/core/rmem_max 限制
        QLOG(ERROR) << "SO_RCVBUF TRUNCATED, expect packet drops";
    }
}
```

**Linux 的 rmem_max 机制**：
```
用户请求 4MB → 内核检查 /proc/sys/net/core/rmem_max
如果 rmem_max = 2MB → 实际只给 2MB，但 setsockopt 不报错
解决方法：sysctl -w net.core.rmem_max=4194304
```

### 2.4 recvfrom 接收数据

```cpp
// 接收线程内部
slot->data.resize(kEthernetMtu);  // 25000 字节，MTU 上限
sockaddr_in sender_addr;
socklen_t sender_addr_len = sizeof(sender_addr);

// recvfrom 系统调用：从内核缓冲区复制数据到用户空间
const ssize_t nbytes = recvfrom(
    lidar_socket_,           // socket fd
    slot->data.data(),       // 目标内存（预分配，零额外分配）
    kEthernetMtu,            // 最大读取字节数
    0,                       // flags
    (sockaddr*)&sender_addr, // 发送方地址（可选）
    &sender_addr_len
);

if (nbytes > 0) {
    slot->data.resize(nbytes);                          // 裁剪到实际大小
    slot->timestamp = ToUnixDoubleSeconds(Clock::Now()); // 记录接收时间戳
}
// nbytes == -1 且 errno == EAGAIN：没有数据，非阻塞立刻返回
```

**数据流**：
```
雷达网卡 → 内核 DMA → 内核 socket 缓冲区 → recvfrom 复制 → 用户空间 slot
```

---

## 3. 主线二：实时线程管理

### 3.1 业务需求

`recvfrom` 必须在 **83 微秒**内执行完（1/12000 秒），否则内核缓冲区积压，最终丢包。普通线程（`SCHED_OTHER`/CFS 调度）的调度延迟可达 30~40ms，不满足要求。

### 3.2 线程创建：为什么不用线程池

```cpp
// spin_publisher_module.h
std::thread receiver_thread_;         // 专用 std::thread
ThreadPool thread_pool_{4};          // 处理线程用线程池

void LidarFrameBuilder::StartReceiverThread() {
    // 用专用 thread，不用 thread_pool_
    // 原因：需要单独设置这一个线程的调度策略
    //       线程池里的线程无法独立控制
    receiver_thread_ = std::thread([this] {
        SetCurrentThreadRtSchedRR("LidarReceiver");  // 必须在线程内部调用
        // ... 接收循环
    });
}
```

### 3.3 实时调度：SCHED_RR

Linux 有两套调度策略：

| 策略 | 说明 | 优先级 |
|---|---|---|
| `SCHED_OTHER`（CFS） | 默认，公平分时调度 | nice 值，无实时保证 |
| `SCHED_RR` | 实时轮转，同优先级轮流，高优先级必然抢占低优先级 | 1~99，越大越高 |
| `SCHED_FIFO` | 实时先进先出，不主动让出就一直跑 | 1~99 |

```cpp
// spin_publisher_module.cc: SetCurrentThreadRtSchedRR()
void SetCurrentThreadRtSchedRR(const char* thread_name_for_log) {
    // 运行时查询优先级范围（跨 Linux/QNX 可移植）
    const int min_pri = sched_get_priority_min(SCHED_RR);  // Linux: 1
    const int max_pri = sched_get_priority_max(SCHED_RR);  // Linux: 99

    // 设在 60% 位置（约 60）：
    // - 高于普通用户实时任务（通常 1~20）
    // - 低于内核 softirq/watchdog（通常 99）
    // 原因：UDP softirq 负责把网卡数据写进 socket 缓冲区，
    //       如果我们的优先级 > softirq，softirq 跑不了，缓冲区不更新，
    //       recvfrom 永远取不到新数据 —— 死锁
    sched_param sp{};
    sp.sched_priority = min_pri + (int)((max_pri - min_pri) * 0.6);

    // pthread_setschedparam 修改当前线程的调度策略
    // 需要 CAP_SYS_NICE 权限（或 RLIMIT_RTPRIO 配置）
    const int rc = pthread_setschedparam(pthread_self(), SCHED_RR, &sp);
    if (rc != 0) {
        QLOG(WARNING) << "Failed, staying on SCHED_OTHER";
        // 降级处理：程序不崩，但实时性降低，高负载下可能丢包
    }
}
```

### 3.4 优先级反转问题

如果实时线程去抢一个被普通线程持有的 mutex：

```
接收线程 SCHED_RR（优先级 60）
    → lock(mutex)
    → mutex 被处理线程 SCHED_OTHER（优先级 0）持有
    → 接收线程被阻塞
    → 普通线程得不到 CPU（被其他普通线程抢占）
    → 接收线程一直等 → 丢包
```

这叫**优先级反转**。`PacketRingBuffer` 用无锁设计彻底规避这个问题——接收线程的操作不依赖任何其他线程持有的锁。

### 3.5 线程停止与生命周期

```cpp
// spin_publisher_module.h
absl::Notification stop_notification_;  // 线程间停止信号

// 停止时
void Stop() {
    stop_notification_.Notify();  // 通知所有线程退出
    // receiver_thread_ 在析构函数里 join
}

// 接收线程内
while (!stop_notification_.HasBeenNotified()) {
    // ... 接收循环
}
```

`absl::Notification` 是一次性的线程安全通知机制，底层用 mutex + condition_variable 实现，比手写 `std::atomic<bool>` 更清晰。

---

## 4. 主线三：无锁 SPSC 队列

### 4.1 业务需求

接收线程（实时）产生数据，处理线程消费数据，两者之间需要一个缓冲区。要求：

- 接收线程写入操作不能阻塞（不能等锁）
- 单生产者单消费者（SPSC）
- 零额外内存分配

### 4.2 整体结构

```cpp
class PacketRingBuffer {
    const size_t capacity_;           // 容量，必须是 2 的幂（用于位掩码取模）
    const size_t mask_;               // = capacity_ - 1，用 & 代替 % 取模
    std::vector<LidarPacket> buffer_; // 固定大小数组，构造时一次性分配完

    alignas(64) std::atomic<size_t> write_idx_{0};  // 生产者独占写
    alignas(64) std::atomic<size_t> read_idx_{0};   // 消费者独占写
};
```

**容量为 2 的幂的原因**：
```cpp
// 取模（慢）：index % capacity_
// 位掩码（快）：index & mask_
// 等价条件：capacity_ 是 2 的幂时，mask_ = capacity_ - 1
// 例：capacity_=8, mask_=7(0b111)
//     10 & 7 = 2 （等同于 10 % 8 = 2）
```

**`alignas(64)` 的作用**：CPU 缓存行是 64 字节。如果两个 atomic 挨在一起（16字节内），两个核同时改它们时会互相触发缓存行失效（伪共享），严重影响性能。`alignas(64)` 强制每个 atomic 独占一个缓存行。

### 4.3 生产者流程（接收线程）

```cpp
// 步骤 1：预申请写槽（不修改 write_idx_）
int AcquireWriteBatch(LidarPacket** ptrs, int max_count) {
    const size_t write_pos = write_idx_.load(memory_order_relaxed);
    //                                         ↑ relaxed：只读自己的，不需要同步
    const size_t read_pos  = read_idx_.load(memory_order_acquire);
    //                                        ↑ acquire：读消费者的最新值

    const size_t free_slots = capacity_ - (write_pos - read_pos);
    const int count = min(free_slots, max_count);

    // 只是把指针填进数组，write_idx_ 完全没变！
    for (int i = 0; i < count; ++i) {
        ptrs[i] = &buffer_[(write_pos + i) & mask_];
    }
    return count;
}

// 步骤 2：recvfrom 直接写进预申请的槽（零拷贝）
recvfrom(lidar_socket_, ptrs[i]->data.data(), kEthernetMtu, 0, ...);

// 步骤 3：提交（推进 write_idx_，消费者才能看到）
void CommitWrite(int count) {
    // release：保证步骤 2 的写操作在这条指令之前完成并对外可见
    write_idx_.fetch_add(count, memory_order_release);
}
```

### 4.4 消费者流程（处理线程）

```cpp
// 步骤 1：窥视可读包（不修改 read_idx_）
int PeekBatch(const LidarPacket** ptrs, int max_count) {
    const size_t read_pos  = read_idx_.load(memory_order_relaxed);
    const size_t write_pos = write_idx_.load(memory_order_acquire);
    //                                        ↑ acquire：读生产者提交后的最新值

    const size_t available = write_pos - read_pos;
    const int count = min(available, max_count);

    for (int i = 0; i < count; ++i) {
        ptrs[i] = &buffer_[(read_pos + i) & mask_];
    }
    return count;
}

// 步骤 2：处理数据
UpdateSpin(*ptrs[i]);

// 步骤 3：推进读索引，释放槽位
void AdvanceRead(int count) {
    // release：通知生产者这些槽已释放可复用
    read_idx_.fetch_add(count, memory_order_release);
}
```

### 4.5 memory_order 为什么必须用

不用 atomic（用普通 `size_t`）的三个问题：

**① 编译器重排**
```cpp
// 你写的：
slot->data = recv_data;   // 先写数据
write_idx_++;             // 再推进索引

// 编译器优化后可能变成：
write_idx_++;             // 先推进（消费者看到有包）
slot->data = recv_data;   // 后写数据（消费者读到旧数据）
```
`memory_order_release` 告诉编译器：release 之前的写不能移到 release 之后。

**② CPU 乱序执行**
现代 CPU 内部有乱序执行引擎，机器码顺序不等于实际执行顺序。
`fetch_add(release)` 生成硬件 fence 指令（x86: `mfence`，ARM: `dmb`），强制 CPU 按序执行。

**③ 缓存可见性**
每个 CPU 核有独立的 L1/L2 缓存，写操作先写缓存，不立刻同步到其他核。
`release-store` 强制把写操作广播给其他核；`acquire-load` 强制从全局内存读。

**release/acquire 配对保证**：
> 当消费者的 `acquire-load` 读到了生产者 `release-store` 写入的值时，
> 生产者在 release 之前的所有写操作，对消费者在 acquire 之后都保证可见。

这叫 **happens-before 关系**，是 C++11 内存模型的核心。

### 4.6 为什么不用 mutex

```
mutex = 内存屏障 + 互斥阻塞（争抢锁 + 挂起/唤醒线程）
atomic = 只有内存屏障，没有互斥阻塞
```

SPSC 天然没有竞争（两个线程访问不同的槽），不需要互斥，只需要内存屏障。用 mutex 会引入：
- 系统调用开销（futex）
- 优先级反转风险（实时线程被普通线程卡住）
- 不可预测的阻塞时间

---

## 5. 主线四：拼帧与内参校正

### 5.1 业务需求

每个 UDP 包只有一小段扫描数据，必须把多个包拼成一个完整的旋转帧（360°），再做坐标变换，才能发给感知模块使用。

### 5.2 数据结构层次

```
Spin（一帧，约 100ms，360°）
  ├─ lidar_type_          雷达型号（AT128/M1等）
  ├─ num_scans_           实际扫描线数（~3600）
  ├─ num_beams_           线束数（AT128=128）
  └─ scans_[kMaxNumScans=3620]
       └─ Scan（一个水平角度的一列点）
            ├─ timestamp          这次扫描的时间戳（秒）
            ├─ azimuth_in_degree  水平角度（0~360°，顺时针）
            ├─ pose               该时刻的车辆位姿（插值得到）
            └─ shots[num_beams]
                 └─ LaserShot
                      └─ CalibratedReturn（内参校正后，20字节）
                           ├─ x, y, z   车体坐标系三维坐标（米）
                           ├─ range     径向距离（米）
                           └─ intensity 反射强度 [0, 255]
```

### 5.3 切帧逻辑

雷达把每圈数据分成很多 UDP 包发出来，驱动层判断"一帧结束"：

```cpp
// 普通旋转雷达：azimuth 越过起始角（通常 0°）时切帧
const bool new_frame_created = spinning_lidar_driver::UpdateCurrentOrNextSpin(
    lidar_type, lidar_params_, last_frame_finished_, packet,
    lidar_frames_in_build_.back().lidar_frame->mutable_value<Spin>(),
    next_new_lidar_frame_->mutable_value<Spin>()
);

// AT128 特殊：用多个 start_frames 角度切帧（顺序敏感！）
ctx.start_frames.assign(start_frames_protobuf.begin(), start_frames_protobuf.end());
```

### 5.4 局部帧（Partial Spin）

不等整帧完成，每积累 128 条扫描线（约 1/28 圈，3~4ms）就发一次：

```cpp
// kNumScansPerPartialSpin = 128
int PublishReadyPartialSpins(...) {
    for (; end_scan <= num_scans;
         start_scan += 128, end_scan += 128) {
        // 前提：位姿历史覆盖了这段时间（必须有位姿才能发）
        if (latest_pose_timestamp < spin.scan(end_scan-1).timestamp) {
            break;  // 位姿还没到，等下次
        }
        UpdateSpinAndPublishShm(..., /*partial_spin=*/true, ...);
    }
}
```

**局部帧的价值**：感知模块不需要等 100ms 的完整帧，可以提前 70ms 开始处理第一批点，降低端到端延迟。

### 5.5 位姿插值

```cpp
// 位姿历史（循环缓冲区，存最近 20 个位姿）
boost::circular_buffer<VehiclePoseWithTimestamp> pose_history_{20};

// Lite 线程更新位姿（有锁）
void UpdatePose(shared_ptr<const PoseProto> pose) {
    absl::MutexLock lock(&pose_history_mutex_);
    pose_history_.push_back({pose->timestamp(), VehiclePose(*pose)});
}

// 处理线程读位姿，给每条扫描线插值（有锁）
spin_util::SetScanPoses(pose_history_, new_spin, start_scan, end_scan);
// 内部：对每条扫描线的 timestamp，在历史中找前后两个位姿做线性插值
```

**VehiclePose 是什么**：
```cpp
struct VehiclePose {
    float x, y, z;         // 车辆在世界坐标系的位置（米，东/北/高）
    float yaw, pitch, roll; // 偏航角/俯仰角/横滚角（弧度）
};
```

### 5.6 内参校正

```cpp
// 把 (beam_id, azimuth, range) → (x, y, z) 车体坐标系
spin_processor_.ComputeCalibratedReturns(new_spin, start_scan, end_scan);

// 内部步骤：
// 1. 查内参表：每条线束的实际仰角（出厂测量值，非标称值）
// 2. 查正余弦查找表（预计算 65536 个值，避免实时 sin/cos）
//    x = range * cos(elevation) * cos(azimuth + azimuth_offset)
//    y = range * cos(elevation) * sin(azimuth + azimuth_offset)
//    z = range * sin(elevation)
// 3. 乘以安装变换矩阵（雷达安装位置 → 车体坐标系）
```

### 5.7 共享内存发布

```cpp
// 通过 Lite 共享内存发布（零拷贝，不走序列化）
lite_module_->PublishShmMsg(new_spin_shm, ...);

// CalibratedReturn 结构固定 20 字节，布局不能变动！
struct CalibratedReturn {
    float x, y, z, range;   // 16 bytes
    uint8_t intensity;       // 1 byte
    uint8_t reserved[3];     // 3 bytes（对齐填充）
};  // = 20 bytes
static_assert(sizeof(CalibratedReturn) == 20);
// 感知模块直接读共享内存里的这个结构，如果布局改变多进程版本不同步
```

---

## 6. 主线五：故障码监听（epoll）

### 6.1 业务需求

雷达硬件会通过另一个 UDP 端口周期性发送自身的故障状态（温度过高、激光器异常等）。这些包到达频率很低（每帧一个），需要一个高效的等待机制。

### 6.2 poll vs epoll

| | poll | epoll |
|---|---|---|
| 适用场景 | 少量 fd，短时等待 | 大量 fd，长时间等待 |
| 实现 | 每次调用遍历所有 fd | 内核维护事件表，O(1) 通知 |
| 本模块选择 | 接收线程用 poll | 故障线程用 epoll |

```cpp
// 接收线程：用 poll 做非阻塞探测（短暂等待，高频调用）
pollfd fds;
fds.fd = lidar_socket_;
fds.events = POLLIN;
int ret = poll(&fds, 1, 0);  // timeout=0，立刻返回

// 故障线程：用 epoll 长时间等待低频事件
int epoll_fd = epoll_create1(0);
epoll_event event;
event.events = EPOLLIN | EPOLLPRI;
event.data.fd = lidar_fault_message_socket_;
epoll_ctl(epoll_fd, EPOLL_CTL_ADD, lidar_fault_message_socket_, &event);

while (!stop_notification_.HasBeenNotified()) {
    // 等待最多 100ms，有事件则立刻返回
    int num_events = epoll_wait(epoll_fd, events, 1, 100);

    if (num_events > 0) {
        // 收到故障包
        recvfrom(lidar_fault_message_socket_, packet.data, ...);
        // 按包大小区分雷达型号
        if (packet.data.size() == kHeSaiFaultMessageSize) {     // 99 字节
            hesai_fault_messages::UpdateFaultMessageproto(...);
        } else if (packet.data.size() == kRsEmxMiniFaultMessageSize) { // 748 字节
            rs_fault_messages::UpdateEmxMiniFaultMessage(...);
        }
    }
}
close(epoll_fd);
```

### 6.3 故障信息发布

```cpp
// 每 5 秒通过 Lite 定时器发布一次故障状态
AddTimerOrDie("publish lidar fault message proto",
    [&]() { lite_module_->Publish(lidar_fault_message_proto_, "lidar_fault_message_proto"); },
    absl::Milliseconds(5000), /*one_shot=*/false);
```

健康管理模块订阅这个 topic，判断是否需要触发告警或切换雷达。

---

## 7. 内存管理总结

模块对内存做了精心的预分配设计，避免在高频路径上触发堆分配：

| 数据 | 分配时机 | 方式 |
|---|---|---|
| `PacketRingBuffer::buffer_` | 构造函数一次性 | `vector::resize()` |
| 每个 `LidarPacket::data` | 构造函数 `reserve(kEthernetMtu)` | `vector::reserve()` |
| `Spin` 对象（共享内存） | `Spin::MakeSpinOnShm()` | 共享内存 placement new |
| 位姿历史 | 构造时 `circular_buffer{20}` | boost 循环缓冲区 |

**`recvfrom` 的零拷贝路径**：

```
内核缓冲区 → recvfrom → slot->data（预分配内存，resize 是 no-op）

对比（有额外拷贝的方式）：
内核缓冲区 → recvfrom → 临时 vector → 拷贝到 slot
```

**共享内存 placement new**：

```cpp
// Spin 对象存在共享内存里，不是普通堆内存
// MakeSpinOnShm 在 SHM 区域里 placement-new 一个 Spin
static std::unique_ptr<ShmMessage> MakeSpinOnShm(LidarModel lidar_type) {
    // 内部：shm_alloc(SizeInBytes(lidar_type))
    //       new(ptr) Spin(lidar_type)
}
// 感知进程直接映射同一块共享内存，不需要任何拷贝
```

---

## 8. 开发陷阱速查

| 陷阱 | 表现 | 根因 | 解决 |
|---|---|---|---|
| UDP 缓冲区截断 | 偶发丢包，无报错 | `rmem_max` 未配置，`setsockopt` 静默截断 | 必须 `getsockopt` 反查；配置 `sysctl net.core.rmem_max` |
| 实时线程饿死处理线程 | 数据完全卡死 | 删掉了缓冲区满时的 `sleep(100us)` | SCHED_RR 线程必须主动让出 CPU |
| AT128 切帧角顺序 | 点云"撕裂"、障碍物鬼影 | `start_frames` 数组顺序写错 | 严格按厂商文档顺序填写 |
| 共享内存结构改变 | 读到乱数据，不崩溃 | `CalibratedReturn` 布局改变，多进程二进制版本不同步 | 加 `static_assert(sizeof==20)` 保护 |
| PCAP 回放时钟 | 位姿全零，感知结果错误 | 未开 `use_sim_time`，PCAP 时间戳与系统时钟无关 | 回放前必须设 `--use_sim_time=true` |
| 强度值跨型号比较 | 换车型后阈值失效 | 各厂商原始强度范围不同（0~255 / 0~511 等） | 先通过 `intensity_util` 归一化 |
| 局部帧 `num_scans()` | 数组越界 | 成员函数对局部帧返回的是当前积累数，不是完整帧总数 | 用 `spin_metadata.num_scans()` 而非成员函数 |
| 不用 atomic 的 SPSC | 偶发读到旧值/垃圾值 | 编译器重排 + CPU 乱序 + 缓存不可见 | 必须用 `memory_order_release/acquire` |

---

*学习日期：2026-06-08*  
*参考文件：`onboard/lidar/spin_publisher_module.cc` / `spin_publisher_module.h` / `spin_structs.h`*
