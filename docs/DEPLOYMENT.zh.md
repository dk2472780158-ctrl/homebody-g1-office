# 安装与运行

## 运行环境

| 部分 | 要求 |
|---|---|
| Linux 工作站 | 建议 Ubuntu 22.04；主程序 Python 3.11+、HomeBody、MuJoCo / EGL；已有部署使用 RTX 3090 |
| 隔离传感器环境 | ROS 2 Humble、FAST-LIO、`nav2_amcl`；提供 Docker 构建定义，容器内使用 ROS 自带 Python |
| G1 | MID360 和 IMU、ROSbridge、兼容的已认证遥操作服务；固件和步态状态影响实际响应 |
| Windows 本机（可选） | OpenSSH、PowerShell、浏览器；作为独立的控制/传感器通道中转 |
| 模型服务 | OpenAI 格式 Chat Completions 和结构化响应；常见单步指令可以直接解析 |

本仓库不自动安装机器人固件、启动腿控制或替换已有机器人服务。`office/robot/` 描述当前接口适配的方式与参考源码。

## 1. 安装 Python 环境

```bash
python3.11 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e '.[office,dev]'
cp config/local.env.example config/local.env
```

在 `config/local.env` 中设置主机名及模型服务。密钥从已有的外部文件读取，或通过进程环境提供；不要把实际密钥写进可提交的配置模板。所有脚本从 `config/env.sh` 获得工作目录；Python 扩展从环境变量及 `hb_config.py` 读取相同默认值。

`HB_MODEL` 是 `src/homebody/session/config.py` 中的模型别名，默认 `astra-api`。它在本项目原部署中使用兼容中转；复现者应选择自己服务支持的已注册别名，并设置 `OPENAI_BASE_URL`。这里没有把某个中转地址写死在启动脚本里。

## 2. 建立传输通道

采用 Windows 中转时，先在正常 SSH 配置/密钥管理中建立工作站认证和可信主机记录，再执行：

```powershell
.\office\windows\Start-HomeBodyTunnels.ps1 `
  -Workstation 'robotics@workstation.local' `
  -RobotHost 'robot.local'
```

主机名只是模板，换成自己的解析名称或本地配置值。脚本仅建立 SSH 通道，不订阅传感器、不发送动作。

| 工作站回环端口 | 目标 | 作用 |
|---|---|---|
| 18098 | G1:8099 | 执行器专用认证 HTTP 控制 |
| 18099 | G1:8099 | 查看器的状态读取 |
| 19090 | G1:9090 | 点云与 IMU 转发 |
| 19091 | G1:9090 | 独立关节反馈连接 |
| 本机 8092 / 8093 | 工作站同名端口 | 任务页与只读查看器 |

停止这些通道使用 `Stop-HomeBodyTunnels.ps1`。停止隧道脚本只处理自己记录的 SSH 进程，**不代替任务页中的停止按钮**。

若 Linux 工作站直接访问机器人，也可在自己的隧道工具中提供相同回环端口。控制客户端只接受回环地址，避免把认证控制接口直接暴露到外网。

## 3. 准备地图并构建隔离传感器服务

```bash
source config/env.sh
python -B scripts/prepare-office.py --publish-localization
docker compose -f deploy/compose.yaml build
docker compose -f deploy/compose.yaml up -d
docker compose -f deploy/compose.yaml logs --tail 80
```

`prepare-office.py` 将地图与几何依赖复制到 AMCL 目录，不启动进程。容器以 host 网络访问工作站的回环转发端口，使用独立共享内存；构建包安装在 `/opt/homebody-sensor-install`，不会被源码挂载覆盖。

容器工作目录 `/workspace` 对应 `office/sensors/`，仓库只读挂载在 `/repo`。它在本地启动 FAST-LIO、传感器监督和 AMCL 文件适配器；不发布机器人运动命令。

Docker 构建定义是根据随仓库交付的 ROS 源码整理的可复现入口，首次发布未重新完成整镜像构建验收，详见验证文档。排查构建时先保留原错误信息，不要以跳过包或降低护栏作为默认处理。

## 4. 接入现有认证遥操作服务

`HB_TELEOP_TOKEN_FILE` 指向仓库外的认证文件，执行器要求其 Linux 权限为 `600`。该文件内容应与已有 G1 服务配置一致，不在仓库中生成或分发。

主要接口：

| 方法 | 路径 | 用途 |
|---|---|---|
| GET | `/health` | 控制状态、使能状态、限速与控制权 |
| POST | `/teleop/cmd_vel` | 速度与零速度停止 |
| POST | `/teleop/arm` | 人工点击使能后请求腿控制 |
| GET | `/teleop/gestures/status` | 官方手势能力与任务状态 |
| POST | `/teleop/gestures/start` | 开始指定手势 |
| POST | `/teleop/gestures/heartbeat` | 手势任务心跳 |
| POST | `/teleop/gestures/stop` | 取消手势 |

请求包含 `X-Teleop-Token`。速度接口需返回 `ok`，并可返回实际接受的 `cmd`。当前桥接层对横向与偏航的符号有约定，客户端在发送前转换符号；接入其他后端时必须核对，不能通过忽略“转向不符”检查来适配。

参考服务还依赖现有运动控制权、ROS 2 运动桥接和设备状态组件；它不是一个复制文件就能替代所有厂商服务的独立安装包。不要覆盖机器人上既有舞蹈或其他应用。

## 5. 启动查看器与任务页

```bash
source config/env.sh
bash office/start-g1-readonly-viewer.sh
bash office/start-office-agent.sh
```

先查看 `8093` 的黄色底座位置与朝向，再在 `8092` 提交任务生成预览。有效预览会显示目标、路线、速度和可能的停车调整；每份预览只执行一次，过期或位姿变化后需要重新生成。

“使能腿控制”可能触发站立；它与普通任务提交分开。完成现场准备后由操作员点击，不由启动脚本自动触发。

## 6. 停止服务

先在任务页停止当前任务并确认反馈，再按需要关闭页面或传感器服务：

```bash
bash office/stop-office-agent.sh
docker compose -f deploy/compose.yaml down
```

页面、传感器和网络通道是独立部分。传感器停止后，任务页会失去有效定位；重新启动会产生新会话，旧任务预览不会跨会话自动执行。

## 常见问题

| 现象 | 首先查看 |
|---|---|
| 页面无法访问 | 8092/8093 服务是否启动、SSH 页面转发是否成功 |
| 等待传感器 | 原始话题发布者、ROSbridge、19090、源时间与积压状态 |
| AMCL matching | 当前切片、历史地图适配、几何核验；AMCL 粒子不等于已通过定位 |
| 位置在线但无法执行 | 预览是否过期、控制状态、令牌文件权限、腿控制状态、现场空间 |
| 有动作趋势但无位移 | 先确认反馈记录及现有步态是否接受该速度；不能只根据 HTTP 成功判定完成 |
| 到目标附近后不能再出发 | 目标停车净空、转向空间、实时障碍层；检查调整后的停车点 |

完整参数和状态条件见源码与 [架构文档](ARCHITECTURE.zh.md)。
