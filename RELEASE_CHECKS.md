# 首次发布校验

校验日期：2026-11-11。以下均为发布副本的离线检查，不发送机器人运动命令。

| 检查 | 结果 |
|---|---|
| 办公室回归 `python -B scripts/test-office.py` | 161 项办公室测试、9 项 AMCL 几何测试通过 |
| 模型接口回归 `pytest tests/test_api_providers.py` | 18 项通过，使用模拟传输 |
| 发布检查 `python -B scripts/check-release.py` | Python 源码可编译；敏感模式、文本 CR 字节和目标点检查通过 |
| Bash 入口 | 23 个 shell 文件通过 `bash -n` |
| Windows 入口 | 两个 PowerShell 文件通过语法解析，未实际建立网络通道 |
| 地图 | 14 个地点/方位目标通过静态可用性检查；中央目标净空约 1.43 m |
| README 插图 | 使用交付的真实占据图生成，已检查标签、路线和图片边界 |

本次轻量测试环境：Windows、Python 3.13.9、NumPy 2.3.5、SciPy 1.16.3、FastAPI 0.115.12、Starlette 0.46.2、HTTPX 0.28.1。完整工作站环境另见部署文档。FastAPI 发出已有 `on_event` 生命周期接口的弃用提示，测试仍通过。

文本统一为 LF，PCD、NPZ、模型和图像以二进制保存。`ARTIFACTS.sha256` 与 `ARTIFACTS.md5` 记录交付文件的摘要，不包含这两份摘要文件本身。它们用于检验重新克隆后的文件字节。

本次未重新构建完整 ROS 传感器镜像、未运行所有上游重依赖测试、未完成新的物理机器人端到端验收。机器人端服务仅提供集成参考；既有现场验证与当前限制见 [验证说明](docs/VALIDATION.zh.md)。
