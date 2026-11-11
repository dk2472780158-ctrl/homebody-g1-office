# 办公室地图与地点

## 随仓库交付的数据

| 文件 | 含义 |
|---|---|
| `office/maps/source-office.pcd` | 用户提供的原始办公室 PCD 副本；保存输入，不用于替代已修正的运行地图 |
| `office/g1_fastlio.historical-office.pcd` | 当前历史匹配点云，包含现场已移除西侧凸起后的地图修正 |
| `office/historical-office-registration.json` | 历史 PCD 到办公室坐标的刚体变换与来源校验记录 |
| `office/real-office-navigation.npz` | 当前真机静态规划占据图，包含 `occupancy` 和 `T_map_px` |
| `office/office-cloud-slice/` | 静态障碍切片、二维投影和地图元数据 |
| `office/office-landmarks.json` | 物体锚点与可接近位置 |
| `office/office-waypoints.json` | 八方位与门口目标及其栅格坐标约定 |
| `office/office-map-proxy-scene/` | MuJoCo 仿真代理场景；保留其生成时的栅格和几何 |

原始 PCD、历史匹配点云、规划图和仿真代理是不同产物。仿真代理不是毫米级数字孪生；不能把仿真穿越或成功到达当成真实现场净空证明。真机规划以 `real-office-navigation.npz` 和实时障碍层为准。

## 地点定义

地图北侧左边为桌子1、北侧右边为桌子2；右侧障碍为架子；左下角为窗口；右下开口为门口。西侧其他障碍不额外命名。八方位目标保留中文识别，但不显示为八个可见物体标记。

房间中央由 `office_landmarks.py` 的 `ROOM_CENTER_XY_M` 定义，当前为办公室坐标 **(-0.65, -0.85) m**，位于主长方形房间内部。它依据墙体范围近似确定，不是所有空闲格的统计重心，也不是门口的别名。

物体的 `anchor_xy_m` 是绘制标签的位置；`approach_xy_m` 是导航请求位置。执行前规划器可能在目标附近搜索可停车并再次出发的位置，预览会披露调整距离。

## 坐标与切片

栅格分辨率 **0.05 m**。图片列向东、行向南；物理坐标 `+x` 东、`+y` 北。使用文件内变换矩阵转换，不应根据图片像素自行猜坐标。

实时避障选取地面以上 **0.15–1.80 m** 的点，再过滤机器人近身点。静态地图来自历史观测，不能替代当前感知；现场物体移走后，旧静态地图若仍占据，该位置仍会被规划器考虑。

## 更新自己的地图

1. 导出自己的点云与栅格，保存原始文件及校验值。
2. 明确地图原点、分辨率、坐标朝向和地面高度。
3. 使用 `build-real-navigation-map.py`、`export-office-cloud-slice.py` 等工具的参数入口生成规划图与切片；具体输入以工具 `--help` 与函数定义为准。
4. 在自己的办公室设置锚点和可接近位置，更新中央坐标。
5. 离线检查命名点可用性、路线和停车空间，停止定位服务后重新准备 AMCL 地图，再由现场观察核对。

现有工具包含本项目地图处理约定，不是对任意点云零配置自动生成可导航地图的承诺。先备份自己的输入，避免直接在唯一一份地图上覆盖处理。

重新生成 README 插图：

```bash
python -B scripts/render-office-map.py
```

校验所有交付文件的字节：`ARTIFACTS.sha256` 和 `ARTIFACTS.md5`。清单不包含自身、Git 元数据或运行时文件。
