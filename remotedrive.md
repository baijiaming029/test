# RemoteDrive 模块学习笔记

> 本文记录对 `onboard/remote_drive/remote_drive_control/` 三个核心文件的学习理解。  
> 覆盖：架构设计、各组件职责、关键代码片段、数据流向。

---

## 目录

1. [整体架构](#1-整体架构)
2. [RemoteControlModule —— 中枢调度器](#2-remotecontrolmodule--中枢调度器)
3. [LocalVehicleImpl —— 数据上行员](#3-localvehicleimpl--数据上行员)
4. [RemoteCockpitImpl —— 指令接收员](#4-remotecockpitimpl--指令接收员)
5. [完整数据流](#5-完整数据流)
6. [关键设计决策](#6-关键设计决策)

---

## 1. 整体架构

### 1.1 为什么需要这个模块

远程驾驶面临三个核心工程问题：

| 问题 | 后果 | 解决方 |
|------|------|--------|
| 驾驶员可发任意速度的指令 | 超速 | LocalVehicleImpl 的限速控制器 |
| 网络随时可能中断 | 车辆失控 | RemoteControlModule 的看门狗 + 故障上报 |
| 底盘协议 ≠ 驾驶舱协议 | 无法直接对接 | LocalVehicleImpl 的 DataAdapter |

底盘不能直连云端，驾驶员指令不能直接发给底盘，中间必须有一个"安全代理"层。`RemoteControlModule` 就是这个安全代理。

### 1.2 三层结构

```
┌──────────────────────────────────────────────────────┐
│              RemoteControlModule                      │  ← LiteModule + Mediator
│  ┌─────────────────────┐  ┌──────────────────────┐   │
│  │  LocalVehicleImpl   │  │  RemoteCockpitImpl   │   │  ← 组件（Component）
│  │  （车辆侧 / 上行）   │  │  （驾驶舱侧 / 下行） │   │
│  └─────────────────────┘  └──────────────────────┘   │
│  ┌──────────────────────────────────────────────────┐ │
│  │           CommunicationManager                   │ │  ← RTC 抽象层
│  └──────────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────────┘
        │                               │
   车辆内部总线                      实时网络（RTC）
  （Lite SHM）                    （火山引擎 / 自研）
        │                               │
   底盘/感知/规划                  远端驾驶舱/小程序
```

### 1.3 组件与模块的关系

组件不是 `LiteModule`，它们**借用模块的能力**工作：

```cpp
// RemoteControlModule 继承两个基类
class RemoteControlModule : public LiteModule,          // Lite 框架能力
                            public RemoteControlMediator // 中间人接口

// 组件通过 Mediator 接口使用模块能力
class LocalVehicleImpl {
  void OnSubscribeChannels() {
    SafeCallMediator([this](auto mediator) {
      // mediator 就是 RemoteControlModule 本身
      mediator->Subscribe(&LocalVehicleImpl::HandleChassisData, this, 25);
    });
  }
};
```

模块包含组件（`unique_ptr`），组件通过 Mediator 指针反向调用模块：

```
Module ──owns──> LocalVehicleImpl
Module ──owns──> RemoteCockpitImpl
LocalVehicleImpl ──calls via Mediator──> Module（订阅、发布、定时器）
RemoteCockpitImpl ──calls via Mediator──> Module（发送 RTC 消息等）
```

---

## 2. RemoteControlModule —— 中枢调度器

**文件**：`remote_drive_control/remote_control_module.cc`

### 2.1 职责概述

模块本身不处理任何具体业务，只做三件事：
1. **生命周期管理**：创建/初始化/停止三个组件
2. **消息路由**：组件间互发消息必须经过模块中转
3. **健康监控**：周期性检查网络连接状态和指令接收超时

### 2.2 启动流程

```cpp
void RemoteControlModule::OnInit() {
  communication_manager_->Initialize();   // 1. 初始化 RTC 通信层
  InitializeComponents();                  // 2. 创建三个组件
  cockpit_component_->Initialize();        // 3. 组件各自初始化
  vehicle_component_->Initialize();
  RegisterRtcHandlers();                   // 4. 注册 RTC 回调
}

void RemoteControlModule::OnSubscribeChannels() {
  vehicle_component_->OnSubscribeChannels(); // 委托给组件订阅
  cockpit_component_->OnSubscribeChannels();
}

void RemoteControlModule::OnSetUpTimers() {
  vehicle_component_->OnSetUpTimers();
  cockpit_component_->OnSetUpTimers();
  // 模块自己的定时器
  AddTimerOrDie("flush_rtc_batch_messages", ...);   // 批量消息刷新
  AddTimerOrDie("check_rtc_ack_messages", ...);     // 消息确认检查
  AddTimerOrDie("check_rtc_health", ...);           // 健康检查 100ms
}
```

### 2.3 消息路由（Mediator 模式）

组件之间不能直接通信，必须通过模块转发：

```cpp
// 车辆侧想发消息给驾驶舱侧
void RemoteControlModule::OnSendMessageToCockpit(
    const google::protobuf::Message& message,
    const RemoteMessageInfo& message_info) {
  if (cockpit_component_) {
    cockpit_component_->ReceiveMessageFromComponent(message, message_info);
  }
}

// 驾驶舱侧想发消息给车辆侧
void RemoteControlModule::OnSendMessageToVehicle(...) {
  if (vehicle_component_) {
    vehicle_component_->ReceiveMessageFromComponent(message, message_info);
  }
}
```

### 2.4 健康检查（安全核心）

每 100ms 执行，包含两项检查：

```cpp
void RemoteControlModule::CheckRtcConnectionHealth() {
  // 仅在远程驾驶激活、且是 RTC 接入（非蓝牙）时检查
  if (takeover_type == TAKEOVER_APPLET_BLUETOOTH ||
      vehicle_state != VS_REMOTE_DRIVE) {
    return;
  }

  if (communication_manager_->HasConnectionError()) {
    // 上报致命级别故障 → 触发整车安全停车
    MODULE_QISSUEX(QIssueSeverity::QIS_FATAL, QIssueType::QIT_NETWORK,
                   QIssueSubType::QIST_REMOTE_CONTROL_RTC_CONNECT_ERROR, ...);
    // 后台线程断开 RTC
    DoOnDemandSideCarJob([this](LiteModule*) {
      communication_manager_->DisconnectRtc();
    });
  }
}

void RemoteControlModule::CheckDataReceiveTimeout() {
  // 默认超时阈值 1000ms，从配置文件读取
  auto result = communication_manager_->CheckDataReceiveTimeout(params);

  if (result == kEnterTimeout) {
    // 进入远程驾驶后从未收到指令
    MODULE_QISSUEX(..., QIST_REMOTE_CONTROL_DATA_RECEIVE_TIMEOUT, ...);
  } else if (result == kDataReceiveTimeout) {
    // 曾正常收到但随后中断
    MODULE_QISSUEX(..., QIST_REMOTE_CONTROL_DATA_RECEIVE_TIMEOUT, ...);
  }
}
```

### 2.5 车辆状态变化响应

```cpp
void RemoteControlModule::NotifyVehicleStateChanged(
    VehicleState previous_state, VehicleState new_state) {
  if (new_state == VS_REMOTE_DRIVE && previous_state != VS_REMOTE_DRIVE) {
    HandleEnterRemoteDrive();  // 发起 RTC 连接
  }
  if (previous_state == VS_REMOTE_DRIVE && new_state != VS_REMOTE_DRIVE) {
    HandleExitRemoteDrive();   // 重置计时
  }
  cockpit_component_->OnVehicleStateChanged(...);  // 通知各组件
}
```

---

## 3. LocalVehicleImpl —— 数据上行员

**文件**：`remote_drive_control/core_impl/local_vehicle_impl.cc`

### 3.1 职责概述

- 订阅车内约 25 种数据（底盘、感知、规划、定位等）
- 经过**格式转换**和**流量控制**后，通过 RTC 推送给远端驾驶舱
- 监听自动驾驶状态机，是**车辆状态的权威判断方**
- 处理接管请求通知，管理网络连接上下文

### 3.2 订阅的数据（基础组）

```cpp
void LocalVehicleImpl::SetupBasicSubscriptions(RemoteControlMediator* mediator) {
  mediator->Subscribe(&LocalVehicleImpl::HandleChassisData, this, 25);       // 底盘
  mediator->Subscribe(&LocalVehicleImpl::HandleAutonomyStateData, this, 20); // 自动驾驶状态
  mediator->Subscribe(&LocalVehicleImpl::HandleTrajectoryData, this, 20);    // 规划轨迹
  mediator->Subscribe(&LocalVehicleImpl::HandlePoseData, this, 50);          // 位姿
  mediator->Subscribe(&LocalVehicleImpl::HandleEmergencyBrakeProto, this, 20); // 紧急制动
  mediator->Subscribe(&LocalVehicleImpl::HandleRoutingResultData, this, 20); // 路由结果
  // ... 共约 13 种基础订阅
}
```

扩展组（远程辅助可视化，可配置关闭）还包括交通灯识别、感知目标、鸟瞰图、规划器状态等约 12 种。

### 3.3 上行前的两道处理

**① 流量控制（节流）**

车内数据高频（25~100Hz），RTC 带宽有限，需要降频：

```cpp
void LocalVehicleImpl::SendVehicleMessageToCockpitLimitFreq(
    const google::protobuf::Message& message, ..., int32_t interval_ms) {
  if (interval_ms < 0) {
    interval_ms = config_->rtc_message_send_interval_ms();  // 从配置读默认间隔
  }

  const absl::Time now = steady_clock_->UptimeNow();
  const auto last_send_it = last_send_timestamps_.find(message_type);

  // 距上次发送未到最小间隔，丢弃
  if (last_send_it != last_send_timestamps_.end() &&
      (now - last_send_it->second) < min_interval) {
    return;
  }
  last_send_timestamps_[message_type] = now;
  SendMessageToCockpitProxy(message, ...);  // 转发给 CockpitImpl → RTC
}
```

**② 空间裁剪（障碍物/目标物）**

感知数据量大，只发驾驶员关注的附近区域：

```cpp
void LocalVehicleImpl::HandleObjectsData(...) {
  const double filter_x_range = config_->object_filter_x_range();
  const double filter_y_range = config_->object_filter_y_range();

  for (const auto& object : objects_msg->objects()) {
    // 把目标物世界坐标转换到以车辆为中心的坐标系
    const auto vcs_point = perception_util::InverseTransformPoint2d(
        vehicle_position, pose_snapshot->yaw(), object_pos);

    // 超出范围的过滤掉
    if (std::abs(vcs_point.x()) > filter_x_range ||
        std::abs(vcs_point.y()) > filter_y_range) {
      continue;
    }
    filtered_msg.add_objects()->CopyFrom(object);
  }
  SendVehicleMessageToCockpitLimitFreq(filtered_msg);
}
```

### 3.4 状态机监听（最核心）

```cpp
void LocalVehicleImpl::HandleAutonomyStateData(...) {
  const auto previous_l4_state = l4_state_.load();
  l4_state_.store(autonomy_state->l4_state().state());

  if (l4_state_.load() == L4_STATE_REMOTE_NORMAL) {
    if (vehicle_state_.load() != VS_REMOTE_DRIVE) {
      vehicle_state_.store(VS_REMOTE_DRIVE);        // 进入远程驾驶
      SetRemoteDriveStartTime(steady_clock_->UptimeNow());
      // 通知模块层 → 模块层发起 RTC 连接
    }
  } else {
    if (previous_l4_state == L4_STATE_REMOTE_NORMAL) {
      ResetTimeout();
      vehicle_state_.store(VS_IDLE);                // 退出远程驾驶
      SetRemoteDriverUidWithType("", TAKEOVER_NONE); // 清除驾驶员信息
      ClearPendingRtcConnectContexts();
    }
  }

  if (previous_vehicle_state != new_vehicle_state) {
    NotifyVehicleStateChanged(previous_vehicle_state, new_vehicle_state);
  }
}
```

### 3.5 限速逻辑

```cpp
void LocalVehicleImpl::ApplySpeedLimit(GuardianCmdProto* guardian_cmd) {
  // 坡道下滑警告时暂停限速，不干扰防溜坡
  if (chassis_snapshot->slide_slope_warning()) {
    return;
  }

  // 倒档用单独的限速值
  const double speed_limit_mps =
      (chassis_snapshot->gear_location() == Chassis::GEAR_REVERSE)
          ? config_reverser_speed_limit * kKmphToMps
          : config_speed_limit * kKmphToMps;

  // 计算限速后的执行量
  const double speed_limited_actuation =
      speed_limit_controller_->CalculateSpeedLimitResult(
          pose_snapshot.get(), *chassis_snapshot, speed_limit_mps,
          current_throttle, current_brake);

  // 油门和刹车互斥输出
  if (speed_limited_actuation >= 0.0) {
    control_cmd->set_throttle(speed_limited_actuation);
    control_cmd->set_brake(0.0);
  } else {
    control_cmd->set_throttle(0.0);
    control_cmd->set_brake(-speed_limited_actuation);
  }
}
```

---

## 4. RemoteCockpitImpl —— 指令接收员

**文件**：`remote_drive_control/core_impl/remote_cockpit_impl.cc`

### 4.1 职责概述

- 从云端获取实时网络连接凭证（令牌、应用标识）
- 接收并分类处理来自远端的所有下行指令
- 把驾驶操控指令经过协议转换、限速约束后，以**固定 50Hz** 发布给底盘
- 驾驶员连接时推送初始车辆信息
- 处理紧急停车、限速调整、日志控制、重启等运维命令

### 4.2 定时任务

```cpp
void RemoteCockpitImpl::OnSetUpTimers() {
  // 拉取凭证，成功即停止（100ms 轮询直到成功）
  mediator->AddTimerOrDie("fetch_rtc_config",
      [this]() { FetchRTCConfigFromCloud(); }, absl::Milliseconds(100), false);

  // 定期刷新令牌（令牌有有效期）
  mediator->AddTimerOrDie("update_rtc_token",
      [this]() { UpdateRTCConfigFromCloud(); },
      absl::Minutes(config_->update_rtc_token_interval_minutes()), false);

  // 查询服务商类型，成功即停止
  mediator->AddTimerOrDie("fetch_rtc_type",
      [this]() { FetchRtcTypeFromCloud(); }, absl::Milliseconds(100), false);

  // 50Hz 定时发布控制指令（核心）
  mediator->AddTimerOrDie("send_control_cmd",
      [this]() { SendCachedControlCommand(); }, absl::Milliseconds(20), false);
}
```

### 4.3 下行指令分发

```cpp
absl::Status RemoteCockpitImpl::HandleRemoteDriveMessage(
    const uint8_t* message, size_t size, ...) {
  RemoteDriveProto remote_msg;
  remote_msg.ParseFromArray(message, size);  // 反序列化 RTC 字节流

  const std::string& message_type = remote_msg.message_type();

  if (message_type == kCockpitPingProto) {
    OnPingCommand(remote_msg, source_uid);          // 心跳探测 → 立即回复
  } else if (message_type == kCockpitCommonCmdProto) {
    HandleCommonCommand(common_cmd, source_uid);    // 运维指令
  } else if (message_type == kCockpitControlCommandProto) {
    OnCockpitControlCommand(remote_msg, source_uid); // 驾驶操控
  } else if (message_type == kRemoteAssistCmdProto) {
    HandleRemoteAssistCommand(...);                 // 远程协助平台指令
  }
}
```

### 4.4 驾驶操控指令处理链（最关键）

```cpp
void RemoteCockpitImpl::HandleControlCommand(
    const CockpitControlCommandProto& command, ...) {

  // 1. 刷新看门狗（仅在激活状态下）
  if (GetVehicleState() == VS_REMOTE_DRIVE) {
    last_data_receive_time_.store(steady_clock_->UptimeNow());
  }

  // 2. 发布原始指令供日志/监控使用
  PublishLiteMessage<CockpitControlCommandProto>(command);

  // 3. 紧急停车检查
  if (command.emergency_stop_state() == EMERGENCY_STOP_STATE_ON) {
    PublishCockpitCommonCmd(source_uid, CCT_EMERGENCY_STOP, true);
    if (!emergency_stop_data_collection_triggered_.load()) {
      DDS_TRIGGER("Remote_Estop_Trigger");  // 触发数据采集（只触发一次）
      emergency_stop_data_collection_triggered_.store(true);
    }
  }

  // 4. 协议转换：驾驶舱格式 → 车辆格式
  GuardianCmdProto guardian_cmd;
  AdaptDownstreamCommand(command, &guardian_cmd, /*is_mini_program=*/false);

  // 5. 施加限速约束
  ApplySpeedLimit(&guardian_cmd);

  // 6. 写入缓存（不直接发布！）
  {
    std::lock_guard<std::mutex> lock(control_cmd_cache_mutex_);
    cached_guardian_cmd_ = guardian_cmd;
  }
  has_cached_cmd_.store(true);
}
```

### 4.5 为什么写缓存而不直接发布

```cpp
// 50Hz 定时器从缓存取最新值发布
void RemoteCockpitImpl::SendCachedControlCommand() {
  if (!is_remote_drive || !has_cached_cmd) return;

  GuardianCmdProto guardian_cmd;
  { /* 加锁读缓存 */ guardian_cmd = cached_guardian_cmd_; }

  // AEB（自动紧急制动）激活时，清空所有控制值只保留刹车
  if (is_aeb_enabled && guardian_cmd.has_control_cmd()) {
    const double brake_value = guardian_cmd.control_cmd().brake();
    guardian_cmd.mutable_control_cmd()->Clear();
    guardian_cmd.mutable_control_cmd()->set_brake(brake_value);
  }

  PublishLiteMessage<GuardianCmdProto>(guardian_cmd, "remote_guardian_cmd_proto");
  has_cached_cmd_.store(false);
}
```

**设计原因**：RTC 网络发包是突发不均匀的（可能一帧内收到多包，也可能某帧没有）。如果每收到一包就立刻发给底盘，底盘收到的频率会忽高忽低。改为定时器 50Hz 取最新缓存值，底盘收到的频率稳定均匀，符合底盘控制器设计预期。

### 4.6 驾驶员建立连接时推送初始信息

```cpp
void RemoteCockpitImpl::OnRtcUserJoined(const std::string& uid) {
  SendInitializationDataToCockpit();
}

void RemoteCockpitImpl::SendInitializationDataToCockpit() {
  SendMessageToCockpit(release_info_, ...);      // 软件版本信息
  SendMessageToCockpit(vehicle_params_proto_, ...); // 整车参数
  if (cached_routing_result_) {
    SendMessageToCockpit(*cached_routing_result_, ...); // 当前路线
  }
  SendMapInfoToCockpit();                        // 当前地图名称
}
```

---

## 5. 完整数据流

### 5.1 上行（车辆 → 驾驶舱）

```
底盘/感知/规划模块
    │ Lite SHM 发布
    ↓
LocalVehicleImpl::Handle*Data()
    ├─ 流量控制（节流）
    ├─ 空间裁剪（障碍物/目标物）
    └─ 协议转换（Chassis → CockpitVehicleInfoProto）
    │ ForwardMessageToCockpit()
    ↓
RemoteControlModule::OnSendMessageToCockpit()  ← Mediator 路由
    │
    ↓
RemoteCockpitImpl::ReceiveMessageFromComponent()
    │ SendRtcMessage()
    ↓
CommunicationManager → RTC SDK → 实时网络 → 远端驾驶舱
```

### 5.2 下行（驾驶舱 → 车辆）

```
远端驾驶员操控
    │ RTC SDK 接收字节
    ↓
CommunicationManager::RouteMessage()
    │ 回调注册的 Handler
    ↓
RemoteCockpitImpl::HandleRemoteDriveMessage()
    ├─ 反序列化 RemoteDriveProto
    └─ 按 message_type 分发
           │ OnCockpitControlCommand()
           ↓
    HandleControlCommand()
    ├─ 刷新看门狗时间戳
    ├─ 紧急停车检查
    ├─ 协议转换（CockpitControlCommand → GuardianCmdProto）
    ├─ 限速约束（ApplySpeedLimit）
    └─ 写入缓存

    50Hz 定时器
    ↓
SendCachedControlCommand()
    ├─ AEB 覆盖检查
    └─ PublishLiteMessage<GuardianCmdProto>
           │ Lite SHM
           ↓
    VehicleControlModule / Guardian → 底盘执行
```

---

## 6. 关键设计决策

### 6.1 Mediator 模式的必要性

如果 `LocalVehicleImpl` 和 `RemoteCockpitImpl` 直接互相引用，会形成循环依赖，也会让每个组件都需要知道另一方的存在。Mediator 模式解耦了这种依赖：

```
                    ┌────────────┐
LocalVehicleImpl ──→│  Mediator  │←── RemoteCockpitImpl
                    │  (Module)  │
                    └────────────┘
  组件只认识 Mediator，不认识对方
```

### 6.2 写缓存 + 定时发布

```
RTC 收包（不均匀）:  ─────●───●●───────●──●─────
写缓存:             ─────W───WW───────W──W─────
50Hz 定时发布:      ──T──T──T──T──T──T──T──T──T──
                    （取最新缓存，空则跳过）
底盘收到（均匀）:   ──●──●──●──●──●──●──●──●──●──
```

### 6.3 多线程安全

这个模块涉及多个线程：

| 线程 | 操作 |
|------|------|
| Lite 主调度线程 | 所有 Subscribe 回调、定时器回调 |
| RTC SDK 内部线程 | 收到网络包 → 调用注册的 Handler |
| 旁路线程（SideCarJob） | RTC 连接/断开（阻塞操作，不能在 Lite 线程做） |

共享数据的保护方式：

```cpp
// 使用 std::atomic 保护标量
std::atomic<VehicleState> vehicle_state_{VS_IDLE};
std::atomic<L4SystemState> l4_state_{L4_STATE_OFF};
std::atomic<bool> has_cached_cmd_{false};
std::atomic<bool> aeb_enabled_{false};

// 使用 std::mutex 保护复合对象
std::mutex control_cmd_cache_mutex_;
GuardianCmdProto cached_guardian_cmd_;  // 由 mutex 保护

std::mutex remote_driver_uid_mutex_;
std::string remote_driver_uid_;         // 由 mutex 保护
```

### 6.4 看门狗设计

超时检测在**模块层**（不在组件层），原因是模块拥有 `CommunicationManager`，可以直接查询最后收包时间，并上报系统级故障：

```
组件层：记录最后收到有效控制指令的时间戳
模块层：每 100ms 查询 CommunicationManager → 计算超时 → 上报 QIS_FATAL 故障
系统层：接收到 QIS_FATAL 故障 → 触发安全停车
```

---

*学习日期：2026-06-03*  
*参考文件：`remote_drive/remote_drive_control/remote_control_module.cc` / `local_vehicle_impl.cc` / `remote_cockpit_impl.cc`*
