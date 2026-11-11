# 来源、依赖与许可

## 源码组成

| 内容 | 来源 | 说明 |
|---|---|---|
| HomeBody | Stanford-TML/homebody，提交 `2388aa44c875bb0c2ce4603dd557bf3918b7a1a1` | 保留 Apache-2.0 LICENSE、NOTICE、第三方声明；加入办公室适配 |
| FAST-LIO ROS 2 适配 | databall-official/git_slam，基线记录前缀 `adcff016`，包含当前部署调整 | 保留包内源码、README 和 LICENSE |
| AMCL | 官方 ROS 2 Humble `nav2_amcl` | 通过系统包安装，不复制其二进制 |
| 地图、任务与控制扩展 | 本项目 | 随当前有效版本导出，整理可配置路径与发布入口 |
| 机器人模型、步态与第三方资源 | 上游 HomeBody 的 assets/vendor | 许可及来源按原目录中的声明保留 |
| Unitree SDK2 | 官方 SDK | 手势接口运行依赖，未将账号凭据或设备配置打包 |

详细来源与发布修改范围见 `SOURCE_PROVENANCE.json`。

## 许可边界

根目录 `LICENSE` 是上游 HomeBody 的 Apache-2.0 许可；它不覆盖所有独立第三方组件的许可。特别是 `office/sensors/src/fast_lio/LICENSE` 为 **GPL-2.0** 文本，而该包 `package.xml` 中写有 BSD；发布保留原始两者，并明确此差异，不将其改写成单一许可。其余 ROS 包在 `package.xml` 声明各自许可，已有版权信息保持原样。

FAST-LIO 作为独立传感器进程，通过 ROS 消息与文件向办公室扩展提供观测。发布同时交付其对应源码。重新分发或修改某个组件时，按该组件随附的实际许可与上游说明处理。

## 依赖安装分层

- **离线回归**：NumPy、SciPy、Pillow、websocket-client、FastAPI、HTTPX；地图渲染另需 Matplotlib。
- **HomeBody UI / 仿真**：根 `pyproject.toml` 的基础依赖及 `office` extra；原框架使用 MuJoCo、PyTorch、Viser 等。
- **ROS 传感器容器**：ROS 2 Humble、PCL、Eigen、rclpy、nav2_amcl、cbor2 与随仓库交付的 ROS 包。
- **真机服务**：已有 ROSbridge、运动控制桥接、认证遥操作、官方手势 SDK；设备侧依赖以现有服务为准。

GitHub / Gitea 仓库不含虚拟环境、构建目录、运行容器镜像、密钥、令牌或私人诊断录包。
