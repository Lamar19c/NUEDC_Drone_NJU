# TREK1000 迁移 — 文件索引

## 入口文档

| 文件 | 内容 | 适用场景 |
|------|------|---------|
| **MIGRATION_COMPLETE.md** | 🌟 完整迁移指南 | 从头开始迁移，包含所有步骤 |
| README.md | 快速集成指南 | 仅做软件补丁 |
| HARDWARE.md | 硬件连接方案 (A/B/C) | 选择硬件方案 |
| NETLIST.md | 方案B 结构化网表 | PCB 设计参考 |
| SCHEMATIC_GUIDE.md | EasyEDA 手工绘制指南 | 画原理图 |
| FILTER_TUNING.md | 滤波参数调优建议 | 飞行调参 |
| ioc.patch | CubeMX .ioc 修改指引 | 同步 baud rate |

## 软件补丁

| 文件 | 说明 |
|------|------|
| `uwb_solver.c` | 已修改的求解器 (直接替换 stm32_uwb/Core/Src/) |
| `uwb_solver_original.c` | 原始求解器 (备份, 用于回滚) |
| `uwb_solver.patch` | Git diff 格式补丁 |

## EasyEDA 自动化

| 文件 | 说明 |
|------|------|
| `trek1000_sch_dwm1000.json` | 方案B 原理图数据 |
| `generate_schematic.py` | API 自动生成脚本 |
| `place_components.js` | 元件布局脚本 |
| `easyeda-api-skill/` | EasyEDA API 工具集 |

## 外部参考

| 文件 | 说明 |
|------|------|
| `../docs/superpowers/specs/2026-07-13-trek1000-migration-design.md` | 迁移设计规格书 |
| `../../product/UWB/docs/Schematic2-Netlist-解析.md` | 方案A 网表完整解析 |
