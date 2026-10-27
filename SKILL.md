---
name: dual-franka-fk-workspace
description: 输入双臂 Franka/FR3 的 USD 和桌面上方高度范围，重读机器人与桌面变换，数值采样正向工作空间，导出左右臂位姿 CSV/NPZ 并在 Isaac Sim 可视化。用于正向设计、桌上空间筛选和修改 USD 后重新计算；不执行 IK、碰撞验证或物理抓取。
---

# 双臂正向工作空间

用户提供 USD 和桌面上方高度范围后，使用本技能自带脚本完成计算、校验和可视化。默认每臂 Sobol 采样 1,048,576 组关节角，可按用户要求修改。无需依赖旧工程目录或旧计算结果。

## 输入与定义

- 必需：已保存 USD 路径，最低和最高高度。参数单位 m；将用户给出的 cm/mm 换算。如果只说“上方 10 cm”，先澄清是单一高度还是 0～10 cm 高度带，不能默默选零厚度范围。
- 默认输出左右臂 link8 法兰原点；不是连接件位置或指尖。指定 TCP 时用 `--left-tip`、`--right-tip`，输出仍按选定末端原点筛选，并在交付中写明。
- 高度取桌面 Mesh 世界最高 Z 加范围；默认 XY 在该 Mesh 的投影凸包内。适用 Z-up；倾斜/凹形桌面的实际表面距离或孔洞约束不由此过滤器表达。
- 自动识别当前项目的桌面 Mesh 和双臂。若有多个候选或命名改变，检查 USD 后提供 `--table-prim`、`--robot-root`、左右 `--*-base/--*-tip`。不能静默换错对象。
- 支持每臂 7 个有限限位的转动关节及固定关节链；机器人坐标系不支持缩放。链校验失败先分析资产，不把旧模型套到新 USD。

## 执行

脚本路径相对本技能。先确认可用的 Isaac Sim Python 环境；本机已验证环境为 `C:/Users/59778/.conda/envs/env_isaaclab/python.exe`，启动可视化优先通过 `C:/ProgramData/miniforge3/Scripts/conda.exe run --no-capture-output -n env_isaaclab python`。其他机器使用对应环境，不安装或替换已有 Isaac Sim。

```powershell
& C:/ProgramData/miniforge3/Scripts/conda.exe run --no-capture-output -n env_isaaclab python -u C:/Users/59778/.codex/skills/dual-franka-fk-workspace/scripts/workflow.py --scene "E:/path/input.usdc" --min-height 0.03 --max-height 0.05 --output-root "E:/isacsim_codex/fk_workspace_runs"
```

`--samples` 修改每臂采样数；`--height-only` 取消桌面 XY 限制；`--no-view` 仅在用户不需要实时窗口或无图形会话时使用，仍生成 USD 和离线 HTML。参数和文件格式见 [references/output.md](references/output.md)。

流程重读 USD 关节轴、局部坐标、限位、安装位置和桌面几何；在新的时间戳目录写输入参数、结果和独立校验。原 USD 不修改。相同输入与 seed 可复现采样；移动机器人或改变桌面后重新执行即可。

## 检查与交付

- 必须检查 `validation.json` 的 passed 和左右 selected_count。校验包括独立空间螺旋 FK、坐标变换往返、限位、筛选、CSV 与点云，以及临时移动机器人后位姿跟随。
- 默认启动 Isaac Sim，并保持时间线停止。进程启动不等于窗口渲染成功；检查 `viewer.log`、`viewer_status.json` 的 ready 和 `workspace.png`。用图像工具查看截图，确认左右点云与高度框可见。启动较慢时持续检查并报告具体进度。GUI 失败时保留 CSV/HTML，解释原因，不称可视化已成功。
- 交付左右 `*_fk_filtered.csv`、`summary.json`、`workspace.html`、预览 USD/截图和结果目录链接，报告采样数、筛选数、对应末端、坐标与高度约定。CSV 包含位姿和生成它的关节角。
- FK 未包含自碰撞、桌面/障碍碰撞、接近路径、抓取或动力学验证。两臂独立采样；共享体素不证明两臂同时可达同一位姿，也不是可直接执行的目标。采样稀疏不证明其他位置不可达。用户后续明确要求 IK/碰撞时再进入对应流程。

本技能不打包整个 Isaac Sim 安装或场景资产库。结果快照会合并 USD 层，但贴图仍可能引用原文件；跨机器交付资产时另做依赖收集和相对路径校验。
