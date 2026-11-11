# 架构与代码导读

## 三条链路

**任务链路**：`office-agent-console.py` 创建 HomeBody 会话，真机模式交给 `office_real_workflow.RealWorkflow`。常见单步指令由 `direct_steps()` 解析；复杂语句交给模型，但必须经过 `validate_steps()` 校验。模型只能给出已注册技能和参数。`prepare()` 从当前定位生成路线、转向和描述，不发送速度；操作员点击执行后，`Executor` 才接管。

**定位链路**：机器人 ROSbridge 输出点云与 IMU，`rosbridge_to_ros2.py` 处理源时间、队列和连续性，`g1_fastlio_bridge` 校正倒装雷达方向与加速度单位。FAST-LIO 生成当前会话的里程计和配准点云；`live_ros_observer.py` 原子写入文件。AMCL 的 `adapter.py` 将这些文件转成二维激光、地图和 TF，再将 AMCL 候选与近期三维点云独立核对，写出定位状态。

**执行链路**：`g1_real_control.Executor` 结合当前位姿与路线持续发送速度，通过 `ControlHTTP` 到已有机器人遥操作服务。执行器检查反馈、实时障碍、停止标志和控制状态。障碍触发重规划时，目标仍是原任务目标；定位验证的短暂停顿先发零速度，再在受限条件下等待并重规划。定时移动与显式转身不会因通信错误自动重放。

## 模块索引

| 文件 | 主要责任 |
|---|---|
| `office/office_real_workflow.py` | 技能注册、中文解析、预览、待执行状态与按钮状态 |
| `office/g1_real_control.py` | 认证控制客户端、导航和转向反馈、停止与有限恢复 |
| `office/g1_control_http.py` | 每线程 HTTP 连接、超时、失效连接检查、诊断记录 |
| `office/g1_dynamic_navigation.py` | 动态障碍时效、碰撞检查、路线重规划 |
| `office/g1_readonly_geometry.py` | 占据图 A*、净空、停车点、底座运动学与只读反馈 |
| `office/g1_pointcloud_slice.py` | 高度切片、机身过滤、二维扫描投影 |
| `office/office_live_geometry.py` | PCD 读取、结构点过滤、历史点云匹配与质量评估 |
| `office/office_landmarks.py` | 物体锚点、可接近位置、隐藏方位目标和房间中央 |
| `office/g1-readonly-viewer.py` | 独立 3D 地图、点云、底座与路线查看 |
| `office/sensors/readonly_supervisor.py` | 隔离的传感器子进程生命周期、会话更新与恢复 |
| `office/sensors/amcl/adapter.py` | AMCL 消息适配、候选核验和状态输出 |
| `office/sensors/amcl/geometry.py` | 纯几何质量判据和变换计算，可离线测试 |
| `office/sensors/amcl/seed.py` | 三维全局候选比较，提供经核验的自动初始位姿 |
| `office/official_gestures.py` | 认证、可取消、独占的官方手势任务 |
| `office/official_gesture_runner.py` | 官方动作调用和结束释放 |
| `src/homebody/` | 上游会话、模型、技能契约、MuJoCo 和 UI |

## 坐标系与消息

办公室坐标单位为米，东为 `+x`、北为 `+y`。地图角度零点朝东，逆时针为正：北为 `+90°`，西为 `180°`。栅格图片行号向南增大，通过 `T_map_px` 与办公室坐标转换，不应把图片行号直接当成 `y`。

```text
MID360 / IMU
  ├─ /utlidar/cloud_livox_mid360
  └─ /utlidar/imu_livox_mid360
       ↓ 倒装校正与单位转换
FAST-LIO：camera_init → body
       ↓ 本次会话里程计、配准点云
AMCL：office_map → hb_amcl_odom → hb_amcl_base
       ↓ 历史三维核验
办公室中的传感器位置 → 在线腰部运动学 → 底座位置
```

FAST-LIO 运行在本地 ROS domain **77**；AMCL 在隔离 domain **78**，仅使用本地 UDP。二者通过文件交换受检数据，不依赖跨 Windows / Wi-Fi 的 DDS 多播发现。

## 文件契约

`office/sensors/runtime/live-readonly/` 是运行时输出目录，不属于版本控制。

| 输出 | 消费者与含义 |
|---|---|
| `session-id` | 当前传感器代次；会话变化时旧变换失效 |
| `raw-state.json` | 原始里程计、传感器接收时间、源时间、连续性状态 |
| `clouds.npz` | 当前及有限时间窗口内的点云，带会话和时间 |
| `live-state.json` | 通过定位质量检查的全局变换，或明确的 waiting/matching 状态 |
| `real-view-state.json` | 查看器核对后的底座位置，供真机流程读取 |

传感器坐标候选与底座导航就绪状态是不同层次：AMCL 输出不直接证明底座外参已标定，也不等于可无条件开始行走。

## 规划、净空和恢复

静态栅格负责已有墙体和家具，实时切片负责临时障碍。命名地点使用物体旁的可接近位置，而不是障碍物内部锚点；停车搜索还考虑后续转向空间。空间有限时可在条件满足后先沿当前朝向离开窄处，再转向，不能以删除障碍代替可行性判断。

导航沿路线正面行走，在折点调整朝向；相对机身的左右平移仍可单独使用。实际转向、位移和定位会持续反馈，不能只按命令持续时间宣称到达。

恢复是有边界的：短暂 AMCL 验证、同一会话、当前点云及里程计新鲜，且恢复后的位姿与停止参考相符时，可以重新计算到原目标的路径。传感器真正断流、控制接口不可达、大幅重定位或碰撞风险不会被隐藏成“定位在线”。
