# 参数、坐标和输出

入口 `scripts/workflow.py`；`--scene`、`--min-height`、`--max-height` 必填。

| 参数 | 默认与作用 |
|---|---|
| --samples | 每臂 1048576，建议 2 的幂；非 2 的幂取下一个 Sobol 幂次的前 N 组 |
| --seed | 42 |
| --robot-root | 自动识别共同装配根；优先 franka_vision_and_manipulation_kit |
| --table-prim | 优先 /World/airbag_switch_baked/node_1_桌面/Cube_002，否则唯一桌面名称 Mesh |
| --left-base / --right-base | 各臂基座 prim，默认对应 left_/right_ 开头的 *_link0 |
| --left-tip / --right-tip | 各臂末端 prim，默认对应 *_link8 |
| --height-only | 只按高度，不按桌面 XY 凸包 |
| --voxel-m | 0.02，统计位置网格；不会对导出 CSV 做降采样 |
| --output-root | 当前目录/fk_workspace_runs；实际结果在 <时间>/results |
| --no-view | 不启动 GUI，仍导出 HTML 和预览 USD |
| --viewer-test-frames | 默认 0，窗口保持；测试时如 180 帧后退出 |

CSV：每行 sample_id、joint_1_rad … joint_7_rad，然后分别是 world、arm_base、robot_root 的 x/y/z（m）和 qx/qy/qz/qw（四元数 xyzw）。四元数将末端坐标转到所命名的参考坐标系；不是欧拉角。关节顺序从 summary.json 的 joint_names 读取。filtered CSV 保留原 sample_id，不重新编号。无点时输出仅有表头，不放宽范围或制造点。

NPZ 对应 `arm_q`、`pose_world`、`pose_arm_base`、`pose_robot_root`（N×4×4），以及 `world_from_arm_base`、`world_from_robot_root`。使用列向量：

```
T_world_tip = T_world_arm_base @ T_arm_base_tip
T_arm_base_tip = inverse(T_world_arm_base) @ T_world_tip
```

左右臂各用自己的 arm_base。robot_root 是整个双臂装配根。位置已按 USD metersPerUnit 换算成 m，旋转不变。

主要输出：

- left_fk_filtered.csv / right_fk_filtered.csv：桌面范围内的完整位姿样本。
- left_fk.csv / right_fk.csv：未按桌面筛选的全部样本；对应 NPZ 同时保留。
- shared_position_voxels.csv：两臂分别命中的共同空间格子，带各自 sample_id；体素中心/显示均值不是经过 FK 验证的目标。
- summary.json：输入 hash、层 hash、坐标变换、关节限位、采样与筛选数量、高度约定。
- validation.json：数学与文件一致性校验。
- input_scene.usdc / workspace_preview.usda：输入快照与点云覆盖层。
- workspace.html：自带 Plotly 的离线交互三维图；每臂最多显示 20000 个点，CSV 不受显示上限影响。
- workspace.png / viewer_status.json / viewer.log：Isaac Sim 截图、已渲染状态和日志。

Isaac Sim 中蓝色为左臂，橙色为右臂，青绿色边框为高度上下界。未筛选点云默认隐藏。显示的机器人配置是样本静态示意，并未经过碰撞校验；不要启动物理把它当作抓取动画。

本机环境已有 numpy/scipy/pxr/plotly。外部环境需要这些包，并通过自己的 Isaac Sim Python 启动 viewer。不要在普通系统 Python 中导入 Isaac Sim API。脚本默认不连接或控制真实机器人。
